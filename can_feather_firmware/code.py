# # Send a position/turn command to a Hitec MDB916 CAN servo
# # Board: Adafruit Feather RP2040 CAN (MCP2515 controller)
# #
# # Protocol: Hitec "CAN 2.0A/2.0B Servo Control Protocol", Custom Packet Format.
# #   'W' write, 2 registers (8 data bytes):
# #     ['W', servo_id, addrA, dataA_lo, dataA_hi, addrB, dataB_lo, dataB_hi]
# #   'R' read, 2 registers (4 data bytes):  ['R', servo_id, addrA, addrB]
# #     reply 'V':  ['V', servo_id, addrA, A_lo, A_hi, addrB, B_lo, B_hi]
import struct
import time

import usb_cdc
import board
from digitalio import DigitalInOut
from adafruit_mcp2515 import MCP2515 as CAN
from adafruit_mcp2515.canio import Message

# ---- Configuration ---------------------------------------------------------
# servo's SERVO_ID / REG_ID (byte 1 of payload, 0 = broadcast)
SERVO_ID_1 = 0x42
SERVO_ID_2 = 0x63
# servo ID -> that servo's NODE_ID / REG_CAN_BUS_ID (CAN arbitration ID).
# Each servo should have its own CAN ID: two nodes streaming on the same ID
# collide on the bus. Update these to match how each servo is configured.
SERVO_CAN_IDS = {
    SERVO_ID_1: 11,
    SERVO_ID_2: 10,
}
EXTENDED = True        # CAN/PROTOCOL = CAN 2.0B (29-bit ID)
BAUDRATE = 1_000_000   # CAN/BAUDRATE = 1000 kbps
STREAM_FREQUENCY = 10 # Stream frequency set to the servo. Data will be fed back at this rate
INPUT_COMMAND_FREQUENCY = 0.01 # seconds between serial input checks (read is non-blocking)
MOTOR_DURATION = 4
LAST_MOTOR_TIME = 0
LAST_INPUT_TIME = 0
MAX_FRAMES_PER_PASS = 8   # cap on frames read per loop pass, so sending is never starved
PRINT_INTERVAL = 2      # seconds between status prints (10 Hz)

# ---- Servo registers -------------------------------------------------------
REG_POSITION = 0x0C      # current position, 0..16383 (4096 = 90 deg)
REG_TURN_COUNT = 0x18    # accumulated turns
REG_POSITION_NEW = 0x1E  # target position, 0..16383
REG_TURN_NEW = 0x24      # target turns, -32760..32760 (Multi-Turn mode only)
REG_RUN_MODE = 0x44      # current run mode. 0 ... 3, needs resetting after changing
REG_SAVE_CHANGES = 0x70  # write 0xFFFF to this before shutdown to save the changes
REG_POWER_CONFIG = 0x46  # E-stop on bits 10-9, software reset on bit 0
REG_STREAM_MODE = 0x30   # Enable stream mode. 0 = OFF, 1 = ON
REG_STREAM_TIME = 0x2E   # Stream interval. 1-10,000/us. 10,001 - 11,000 (val - 10,000)/Hz  
REG_STREAM_ADDR_0 = 0xE2 # Addresses 1,2 to feed in stream. 
REG_STREAM_ADDR_1 = 0xE4 # Addresses 3,4 to feed in stream. 
REG_STREAM_ADDR_2 = 0xE6 # Addresses 5,6 to feed in stream. 
REG_STREAM_ADDR_3 = 0xE8 # Addresses 7,8 to feed in stream. 
REG_TORQUE = 0x10        # Motor PWM Duty. 4096 = 100%
REG_VOLTAGE = 0x12       # Motor Voltage. 100 = 1V

POS_PER_DEG = 4096 / 90

def write_pos_turn(servo_id, can_id, angle, turn):
    data = struct.pack(
        "<BBBHBH",
        ord('W'), servo_id,
        REG_POSITION_NEW, angle, REG_TURN_NEW, turn
    )
    return Message(id=can_id, data=data, extended=EXTENDED)


def print_reply(msg):
    data = msg.data
    # print(data, len(data))

    if len(data) == 8 and data[0] == ord("V"):
        _, sid, a, va, b, vb = struct.unpack("<BBBHBh", data)
        print(f"  servo {sid}: reg 0x{a:02X} = {va}, reg 0x{b:02X} = {vb}")
    elif len(data) == 5 and data[0] == ord('v'):
        _, sid, addr, res = struct.unpack("<BBBH", data)
        print(f"    servo {sid}: reg: 0x{addr:02X} = {res}")
    else:
        print(f"  id 0x{msg.id:X}: {bytes(data).hex()}")


# Latest streamed register values, keyed by (servo_id, register address), so
# the two servos don't overwrite each other. Filled by handle_frame() without
# printing, so the receive path stays fast.
latest = {}


def handle_frame(msg):
    """Store register values from a 'V'/'v' frame. Returns False for other frames."""
    data = msg.data
    if len(data) == 8 and data[0] == ord("V"):
        _, sid, a, va, b, vb = struct.unpack("<BBBHBh", data)
        latest[(sid, a)] = va
        latest[(sid, b)] = vb
        return True
    if len(data) == 5 and data[0] == ord("v"):
        _, sid, a, va = struct.unpack("<BBBH", data)
        latest[(sid, a)] = va
        return True
    return False


# ---- Hardware setup --------------------------------------------------------
# The transceiver must be out of standby for frames to reach the bus.
standby = DigitalInOut(board.CAN_STANDBY)
standby.switch_to_output(value=False)

cs = DigitalInOut(board.CAN_CS)
cs.switch_to_output()
spi = board.SPI()

# loopback/silent must be False, or nothing is sent on the real bus.
can_bus = CAN(spi, cs, baudrate=BAUDRATE)


serial_port = usb_cdc.data # writes over to /dev/ttyACM1
if serial_port is None:
    raise RuntimeError("usb_cdc.data is disabled: check boot.py and hard-reset the board")
serial_port.timeout = 0     # never block the main loop on serial reads
# Accepted instructions: first byte -> total frame length.
#   'W' two-register write: ['W', id, addrA, A_lo, A_hi, addrB, B_lo, B_hi]
#   'w' one-register write: ['w', id, addr, lo, hi]
FRAME_LENS = {0x57: 8, 0x77: 5}
MIN_FRAME_LEN = 5
RX_STALE_S = 0.5            # drop a partial frame if no more bytes arrive for this long
rx_buf = bytearray()        # incoming bytes not yet forwarded as a full frame
rx_last = 0                 # time the last serial bytes arrived


def next_frame_start(buf, start):
    """Index of the next 'W' or 'w' at or after start, or -1 if none."""
    for k in range(start, len(buf)):
        if buf[k] in FRAME_LENS:
            return k
    return -1


def DEG_TO_POS(degrees):
    return ( int(degrees % 360 * 4096 / 90), int(degrees / 360) )

# i = 0
# targets = (DEG_TO_POS(400), DEG_TO_POS(200))

with can_bus.listen(timeout=0.1) as listener:
    frames = 0          # stream frames handled since last status print
    other = 0           # non-'V'/'v' frames since last status print
    last_print = time.monotonic()
    while True:
        now = time.monotonic()

        # if now >= MOTOR_DURATION + LAST_MOTOR_TIME:
        #     LAST_MOTOR_TIME = now

        #     target_angle = targets[i][0]
        #     target_turn = targets[i][1]

            
        #     msg = write_pos_turn(SERVO_ID_1, SERVO_CAN_IDS[SERVO_ID_1], target_angle, target_turn)
        #     can_bus.send(msg)

        #     msg = write_pos_turn(SERVO_ID_2, SERVO_CAN_IDS[SERVO_ID_2], target_angle, target_turn)
        #     can_bus.send(msg)
        #     i += 1
        #     if i > 1: i = 0
        #     print(i, msg.data)


    
        #    Non-blocking: buffer whatever bytes are waiting, then forward
        #    every complete 'W' (8-byte) or 'w' (5-byte) frame. Partial frames
        #    wait for more data.
        if now >= LAST_INPUT_TIME + INPUT_COMMAND_FREQUENCY:
            LAST_INPUT_TIME = now
            n = serial_port.in_waiting
            if n:
                rx_buf.extend(serial_port.read(n))
                rx_last = now
            elif rx_buf and now - rx_last > RX_STALE_S:
                print(f"dropping stale partial frame: {bytes(rx_buf).hex()}")
                rx_buf = bytearray()

            # print(n, rx_buf)
            while len(rx_buf) >= MIN_FRAME_LEN:
                frame_len = FRAME_LENS.get(rx_buf[0])
                sid = rx_buf[1]
                # Resync if the buffer doesn't start with a plausible frame
                # (a 'W'/'w' byte inside a data value has the wrong servo ID).
                if frame_len is None or (sid != 0 and sid not in SERVO_CAN_IDS):
                    j = next_frame_start(rx_buf, 1)
                    print(f"resync: dropped {j if j >= 0 else len(rx_buf)} byte(s)")
                    rx_buf = rx_buf[j:] if j >= 0 else bytearray()
                    continue
                if len(rx_buf) < frame_len:
                    break   # e.g. a 'W' with only 5-7 bytes so far: wait for the rest
                frame = bytes(rx_buf[:frame_len])
                rx_buf = rx_buf[frame_len:]
                # Route to the addressed servo's CAN ID; broadcast (0) goes
                # to every distinct CAN ID in use.
                ids = set(SERVO_CAN_IDS.values()) if sid == 0 else (SERVO_CAN_IDS[sid],)
                for can_id in ids:
                    can_bus.send(Message(id=can_id, data=frame, extended=EXTENDED))

        for _ in range(MAX_FRAMES_PER_PASS):
            if not listener.in_waiting():
                break
            msg = listener.receive()
            if msg is None:
                break
            if handle_frame(msg):
                frames += 1
                serial_port.write(msg.data)
            else:
                other += 1
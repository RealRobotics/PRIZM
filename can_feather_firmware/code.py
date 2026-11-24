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
CAN_BUS_ID = 10        # servo's NODE_ID / REG_CAN_BUS_ID (CAN arbitration ID)
# servo's SERVO_ID / REG_ID (byte 1 of payload, 0 = broadcast)
SERVO_ID = 67
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


def deg_to_pos(deg):
    """Convert 0..360 degrees to a 0..16383 position value."""
    return max(0, min(16383, int(deg * POS_PER_DEG)))


def write_run_mode(mode):
    data = struct.pack(
        "<BBBH",
        ord('w'), SERVO_ID,
        REG_RUN_MODE, mode
    )
    return Message(id=CAN_BUS_ID, data=data, extended=EXTENDED)

def write_save_config():
    data = struct.pack(
        "<BBBH",
        ord('w'), SERVO_ID,
        REG_SAVE_CHANGES, 0xFFFF
    )
    return Message(id=CAN_BUS_ID, data=data, extended=EXTENDED)

def write_reset():
    data = struct.pack(
        "<BBBH",
        ord('w'), SERVO_ID,
        REG_POWER_CONFIG, 0x1
    )
    return Message(id=CAN_BUS_ID, data=data, extended=EXTENDED)

def write_stream_mode():
    data = struct.pack("<BBBH", ord('w'), SERVO_ID, REG_STREAM_MODE, 0x1)
    return Message(id=CAN_BUS_ID, data=data, extended=EXTENDED)

def write_stream_time():
    data = struct.pack("<BBBH", ord('w'), SERVO_ID, REG_STREAM_TIME, STREAM_FREQUENCY + 10000)
    return Message(id=CAN_BUS_ID, data=data, extended=EXTENDED)

def write_stream_addr_0(addr1, addr2):
    data = struct.pack(">BBBH", ord('w'), SERVO_ID, REG_STREAM_ADDR_0, (addr2 << 8) + addr1)
    print(data)
    return Message(id=CAN_BUS_ID, data=data, extended=EXTENDED)

def write_stream_addr_1(addr1, addr2):
    data = struct.pack(">BBBH", ord('w'), SERVO_ID, REG_STREAM_ADDR_1, (addr2 << 8) + addr1)
    print(data)
    return Message(id=CAN_BUS_ID, data=data, extended=EXTENDED)


def read_one(addr_a):
    data = struct.pack("<BBB", ord('r'), SERVO_ID, addr_a)
    return Message(id=CAN_BUS_ID, data=data, extended=EXTENDED)


def read_two(addr_a, addr_b):
    """Build an 'R' frame requesting two registers."""
    data = struct.pack("<BBBB", ord("R"), SERVO_ID, addr_a, addr_b)
    return Message(id=CAN_BUS_ID, data=data, extended=EXTENDED)


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


# Latest streamed register values, keyed by register address. Filled by
# handle_frame() without printing, so the receive path stays fast.
latest = {}


def handle_frame(msg):
    """Store register values from a 'V'/'v' frame. Returns False for other frames."""
    data = msg.data
    if len(data) == 8 and data[0] == ord("V"):
        _, _, a, va, b, vb = struct.unpack("<BBBHBh", data)
        latest[a] = va
        latest[b] = vb
        return True
    if len(data) == 5 and data[0] == ord("v"):
        _, _, a, va = struct.unpack("<BBBH", data)
        latest[a] = va
        return True
    return False


def sw_reset():
    """Saves current config and triggers a software reset to impart the changes
    """
    can_bus.send(write_save_config())
    time.sleep(1)
    can_bus.send(write_reset())
    time.sleep(0.005)

# ---- Hardware setup --------------------------------------------------------
# The transceiver must be out of standby for frames to reach the bus.
standby = DigitalInOut(board.CAN_STANDBY)
standby.switch_to_output(value=False)

cs = DigitalInOut(board.CAN_CS)
cs.switch_to_output()
spi = board.SPI()

# loopback/silent must be False, or nothing is sent on the real bus.
can_bus = CAN(spi, cs, baudrate=BAUDRATE)

# ---- Main loop: sweep between two targets ----------------------------------
# targets = [(deg_to_pos(90), 0), (deg_to_pos(270), 0)]  # (position, turn)


# test_datac = struct.pack("<BBB", ord('h'),ord('i'), ord('c'))
# test_datad = struct.pack("<BBB", ord('h'),ord('i'), ord('d'))

# usb_cdc.Serial().write(test_data)
serial_port = usb_cdc.data # writes over to /dev/ttyACM1
if serial_port is None:
    raise RuntimeError("usb_cdc.data is disabled: check boot.py and hard-reset the board")
serial_port.timeout = 0     # never block the main loop on serial reads
W_FRAME_LEN = 8             # every instruction is an 8-byte 'W' frame
RX_STALE_S = 0.5            # drop a partial frame if no more bytes arrive for this long
rx_buf = bytearray()        # incoming bytes not yet forwarded as a full frame
rx_last = 0                 # time the last serial bytes arrived


def next_frame_start(buf, start):
    """Index of the next 'W' at or after start, or -1 if none."""
    for k in range(start, len(buf)):
        if buf[k] == 0x57:  # 'W'
            return k
    return -1


with can_bus.listen(timeout=0.1) as listener:

    frames = 0          # stream frames handled since last status print
    other = 0           # non-'V'/'v' frames since last status print
    last_print = time.monotonic()

    while True:
        now = time.monotonic()

        # 0. Check if any new instructions are present
        #    Non-blocking: buffer whatever bytes are waiting, then forward
        #    every complete 8-byte 'W' frame. Partial frames wait for more data.
        if now >= LAST_INPUT_TIME + INPUT_COMMAND_FREQUENCY:
            LAST_INPUT_TIME = now
            n = serial_port.in_waiting
            if n:
                rx_buf.extend(serial_port.read(n))
                rx_last = now
            elif rx_buf and now - rx_last > RX_STALE_S:
                print(f"dropping stale partial frame: {bytes(rx_buf).hex()}")
                rx_buf = bytearray()

            while len(rx_buf) >= W_FRAME_LEN:
                # Resync if the buffer doesn't start with a plausible frame
                # (a 0x57 inside a data value would have the wrong servo ID).
                if rx_buf[0] != 0x57 or rx_buf[1] not in (SERVO_ID, 0):
                    j = next_frame_start(rx_buf, 1)
                    print(f"resync: dropped {j if j >= 0 else len(rx_buf)} byte(s)")
                    rx_buf = rx_buf[j:] if j >= 0 else bytearray()
                    continue
                frame = bytes(rx_buf[:W_FRAME_LEN])
                rx_buf = rx_buf[W_FRAME_LEN:]
                can_bus.send(Message(id=CAN_BUS_ID, data=frame, extended=EXTENDED))


        
        # 3. Read at most MAX_FRAMES_PER_PASS frames, then go round again.
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


        # # 3. Print a summary at PRINT_INTERVAL instead of once per frame.
        # if now - last_print >= PRINT_INTERVAL:
        #     elapsed = now - last_print
        #     print(f"{frames / elapsed:6.1f} frames/s  "
        #           f"pos={latest.get(REG_POSITION)}  "
        #           f"turn={latest.get(REG_TURN_COUNT)}  "
        #           f"other={other}  backlog={listener.in_waiting()}")
        #     frames = 0
        #     other = 0
        #     last_print = now
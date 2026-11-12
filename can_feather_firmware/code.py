# Send a position/turn command to a Hitec MDB916 CAN servo
# Board: Adafruit Feather RP2040 CAN (MCP2515 controller)
#
# Protocol: Hitec "CAN 2.0A/2.0B Servo Control Protocol", Custom Packet Format.
#   'W' write, 2 registers (8 data bytes):
#     ['W', servo_id, addrA, dataA_lo, dataA_hi, addrB, dataB_lo, dataB_hi]
#   'R' read, 2 registers (4 data bytes):  ['R', servo_id, addrA, addrB]
#     reply 'V':  ['V', servo_id, addrA, A_lo, A_hi, addrB, B_lo, B_hi]
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
STREAM_FREQUENCY = 60
LAST_READ_TIME = 0
MOTOR_DURATION = 4
LAST_MOTOR_TIME = 0
STREAM_DURATION = 1 / STREAM_FREQUENCY

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


def write_position_turn(position, turn=0):
    """Build a 'W' frame writing REG_POSITION_NEW and REG_TURN_NEW."""
    data = struct.pack(
        "<BBBHBh",
        ord("W"), SERVO_ID,
        REG_POSITION_NEW, position,  # uint16, little endian
        REG_TURN_NEW, turn,          # int16, little endian
    )
    print(position)
    return Message(id=CAN_BUS_ID, data=data, extended=EXTENDED)


def write_position(position):
    data = struct.pack(
        "<BBBH",
        ord('w'), SERVO_ID,
        REG_POSITION_NEW, position,
    )
    return Message(id=CAN_BUS_ID, data=data, extended=EXTENDED)


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
    if len(data) == 5 and data[0] == ord('v'):
        _, sid, addr, res = struct.unpack("<BBBH", data)
        print(f"    servo {sid}: reg: 0x{addr:02X} = {res}")
    else:
        print(f"  id 0x{msg.id:X}: {bytes(data).hex()}")


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

# # usb_cdc.Serial().write(test_data)
# data = usb_cdc.data
# console = usb_cdc.console
# data.write(test_datad)
# console.write(test_datac)

i = 0
targets = [(deg_to_pos(90), 0), (deg_to_pos(90), 1), (deg_to_pos(270), 0)]  # (position, turn)
with can_bus.listen(timeout=0.1) as listener:
    print("Setup. Checking turn mode")
    can_bus.send(read_one(REG_RUN_MODE))
    time.sleep(0.05)
    while listener.in_waiting():
        msg = listener.receive()
        if msg is not None:
            print_reply(msg)
        else: 
            print("Did not hear back. Is the PSU on?")

    can_bus.send(write_stream_mode())
    time.sleep(0.05)
    can_bus.send(write_stream_time())
    time.sleep(0.05)
    can_bus.send(write_stream_addr_0(REG_POSITION, REG_TURN_COUNT))
    can_bus.send(write_stream_addr_1(REG_TORQUE, REG_VOLTAGE))
    time.sleep(0.05)

    can_bus.send(write_position_turn(0,0))
    time.sleep(0.05)

    while True:
        now = time.monotonic()
        if now >= LAST_MOTOR_TIME + MOTOR_DURATION:
            target = targets[i]
            print(f"Setting {target[0], target[1]}")
            ok = can_bus.send(write_position_turn(target[0], target[1]))
            LAST_MOTOR_TIME = now
            i = i + 1
            if(i > len(targets) - 1): i = 0
        #read 
        if now >= LAST_READ_TIME + STREAM_DURATION:
            print(f"Got this many messsages waiting: {listener.in_waiting()}")
            counter = 0
            while listener.in_waiting():
                print(f"msg no: {counter}")
                msg = listener.receive()
                counter+=1
                if msg is not None:
                    print_reply(msg)
            LAST_READ_TIME = now
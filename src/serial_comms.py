# This script is used to communicate with the arduino over a serial connection

import serial
import time
from pyfirmata import Arduino, SERVO, OUTPUT
import config

MODE_SWITCH_COMMAND = 0x5C  # B01011100 as a hexadecimal value
CONFIRM_COMMAND = 0x59  # B01011001

# Define SysEx start and end bytes
SYSEX_START = 0xF0
SYSEX_END = 0xF7


def send_byte(byte, ser):
    for i in range(100):
        ser.write(bytearray([SYSEX_START] + [MODE_SWITCH_COMMAND] + [byte] + [SYSEX_END]))
        time.sleep(0.01)
    time.sleep(1)


def confirm_commands(ser):
    for i in range(100):
        ser.write(bytearray([SYSEX_START] + [CONFIRM_COMMAND] + [0] + [SYSEX_END]))
        time.sleep(0.01)
    time.sleep(1)


def send_instructions(instructions, ser):
    for byte in instructions:
        if type(byte) == float:
            byte = int(byte*10)
        send_byte(byte, ser)

import time

import numpy as np
import scipy.signal
import matplotlib.pyplot as plt
import matplotlib
import serial
from pyfirmata import Arduino, SERVO, OUTPUT
# import adafruit_mpu6050
import math

import random
import config

import serial

def send_instructions(instructions):
    instruction_bytes = instructions.encode('utf-8')
    ser = serial.Serial("COM6", 57600)
    time.sleep(6)
    print("FIRING MESSAGE:")
    print(instructions)
    for i in range(300):  # Need to spam multiple messages for the arduino to detect it
        ser.write(b".")
    for i in range(500):
        ser.write(b"#")
        ser.write(instruction_bytes)  # number of turns and time to spend travelling is ms
    time.sleep(5)  # Delay to allow arduino to process instructions


def reformat_instructions(instructions):
    final_str = ""
    for i in instructions:
        str = f"{int(i[0]/45),round(i[1]*config.cell_size)}"
        final_str = final_str + str
        print(str)
    return final_str.replace(" ", "")


def vector_from_cell_diff(cell_diff):
    angle = 0
    if cell_diff[0] != 0 and cell_diff[1] != 0:
        distance = 1.41
    else:
        distance = 1

    # TODO: make this based on arithmetic rather than cascaded conditional statements
    if cell_diff[1] == 0 and cell_diff[0] == 1:
        angle = 0
    elif cell_diff[1] == 1 and cell_diff[0] == 1:
        angle = 45
    elif cell_diff[1] == 1 and cell_diff[0] == 0:
        angle = 90
    elif cell_diff[1] == 1 and cell_diff[0] == -1:
        angle = 135
    elif cell_diff[1] == 0 and cell_diff[0] == -1:
        angle = 180
    elif cell_diff[1] == -1 and cell_diff[0] == -1:
        angle = 225
    elif cell_diff[1] == -1 and cell_diff[0] == 1:
        angle = 315
    elif cell_diff[1] == -1 and cell_diff[0] == 0:
        angle = 270

    return angle, distance


def get_relative_instructions(instructions):
    """Get instructions relative to the current orientation of the vehicle"""
    angle = 0
    new_instructions = []
    for i in instructions:
        if i[0]-angle < 0:
            new_instructions.append([int((i[0]-angle+360)/45), round(i[1]*config.cell_size, 1)])
        else:
            new_instructions.append([int((i[0]-angle)/45), round(i[1]*config.cell_size, 1)])
        angle = i[0]

    return new_instructions


def path_to_instructions(path):
    """Converts the planned path to discrete instructions the vehicle can follow"""
    vectors = []
    buffer = []
    instructions = []
    cell_distance = config.cell_size
    for i, cell in enumerate(path):
        if i == 0:
            continue

        last_cell = path[i-1]
        cell_diff = (last_cell[0]-cell[0], last_cell[1]-cell[1])

        vectors.append(vector_from_cell_diff(cell_diff))

        # Coordinates in (y,x)
    j = 0
    for k, vector in enumerate(vectors):

        if k < j:
            continue
        j = k + 1
        while j < len(vectors):
            if vectors[j] == vector:
                j = j + 1
            else:

                buffer.append([vector[0], (j-k)*vector[1]])
                break

    if len(buffer) == 0:
        buffer.append([vectors[0][0], len(vectors)*vectors[0][1]])
    else:
        buffer.append([vectors[-1][0], vectors[-1][1]])

    relative = get_relative_instructions(buffer)
    print(f"Relative Instructions: {relative}")

    reformatted_list = [
        x
        for xs in relative
        for x in xs
    ]
    print(f"ref: {reformatted_list}")
    # send_instructions(reformat_instructions(relative))

    return reformatted_list










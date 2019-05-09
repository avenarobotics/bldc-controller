#!/usr/bin/sudo python2
import sys
from comms import *
import serial
import time

"""
if len(sys.argv) != 3:
        print("give me a serial port and address")
        exit()
"""

#port = sys.argv[1]
port = "/dev/ttyUSB0"
s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.25)

address = [1]

client = BLDCControllerClient(s)

client.leaveBootloader(address)
time.sleep(0.2)
s.reset_input_buffer()

print(client.readCalibration(address))


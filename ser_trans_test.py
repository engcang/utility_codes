#!/usr/bin/env python
import serial
import time


import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)



if __name__ == '__main__':
  ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)  # open serial port
  print(ser.name)         # check which port was really used
  while 1:
      try:
        ser.write(b'hello')     # write a string
        print("sent \n")
        time.sleep(0.2)
      except (SystemExit, KeyboardInterrupt) :
        ser.close()             # close port
        sys.exit(0)

#!/usr/bin/env python
import serial
import time

import sys
import signal
def signal_handler(signal, frame):
    print('Pressed Ctrl+C')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


if __name__== '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
    while 1:
        try:
            d=ser.read(10)
            print(d)
        except (SystemExit, KeyboardInterrupt):
            ser.close()
            sys.exit(0)            

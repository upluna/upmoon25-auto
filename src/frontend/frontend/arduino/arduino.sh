#!/bin/bash

avrdude -v -V -patmega328p -carduino "-P/dev/ttyACM0" -b115200 -D "-Uflash:w:/home/upmoon25/Documents/robotics/upmoon25-bbot/src/bbot/bbot/arduino/hex/$1:i"

#!/bin/bash
echo 'opening pico serial usb port at 1200 baud (magic number)'
stty -F /dev/ttyACM0 1200
echo 'waiting storage device to initialize'
while [ ! -d /media/warren/RPI-RP2 ]; do sleep 0.1; done
sleep 0.5
#if [ "$*" = "" ]; then echo rebooting; sudo picotool reboot; exit; fi
echo 'copying picocode.uf2 to pico'
cp picocode.uf2 /media/warren/RPI-RP2
echo 'file copied pico should be running'

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

echo 'waiting for /dev/ttyACM0 to open'
sleep 0.5
while [ ! -c /dev/ttyACM0 ]; do sleep 0.1; done
sleep 0.5
echo 'entering monitor'
stty -F /dev/ttyACM0 500000 raw
sleep 0.1
if [ $1 = "monitor" ] ; then
   echo 'opening monitor'
   stty 1:0:80001cb2:0:3:1c:7f:15:4:5:1:0:11:13:1a:0:12:f:17:16:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0 -F /dev/ttyACM0
   cat /dev/ttyACM0
fi
if [ $1 = "minicom" ] ; then
   echo 'opening minicom'
   minicom -D /dev/ttyACM0 raw
fi

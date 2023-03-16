#!/bin/bash
echo 'waiting for /dev/ttyACM0 to open'
while [ ! -c /dev/ttyACM0 ]; do sleep 0.1; done
sleep 0.5
echo 'entering monitor'
stty 1:0:80001cb2:0:3:1c:7f:15:4:5:1:0:11:13:1a:0:12:f:17:16:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0 -F /dev/ttyACM0
stty 1:0:80001cb2:0:3:1c:7f:15:4:5:1:0:11:13:1a:0:12:f:17:16:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0 -F /dev/ttyACM0
sleep 0.1
   cat /dev/ttyACM0

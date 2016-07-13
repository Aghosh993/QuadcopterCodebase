#!/bin/bash
../start_openocd.sh ..
/usr/bin/arm-none-eabi-gdb --batch --command=runme.gdb
pkill openocd

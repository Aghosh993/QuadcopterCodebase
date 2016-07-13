#!/bin/bash
../start_openocd.sh ..
arm-none-eabi-gdb --batch --command=runme.gdb
pkill openocd

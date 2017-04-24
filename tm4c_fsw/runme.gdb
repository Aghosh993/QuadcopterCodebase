target remote localhost:2331
monitor speed 1000
monitor flash device =  LM4F232H5QD
monitor halt
monitor reset
load tm4c_fsw.elf
set confirm off
quit

target remote localhost:3333
monitor reset halt
load tm4c_fsw.elf
monitor reset init
set confirm off
quit

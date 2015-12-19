set confirm off
set pagination off

target remote localhost:10000
symbol-file chrono.elf
load chrono.elf

set remote hardware-watchpoint-limit 4
set remote hardware-breakpoint-limit 6

tb main
c

#set debug_mode=1
#set msc=1

#c


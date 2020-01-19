target remote :3333

# setup semihosting for debug:
monitor arm semihosting enable


# setup ITM instead, if possible, with the target chip
#monitor tpiu config external uart off 8000000 2000000
#monitor tpiu config internal itm.fifo uart off 8000000
#monitor itm port 0 on

load
continue


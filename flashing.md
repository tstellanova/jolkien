
install latest openocd:

`brew install --HEAD openocd`


with one shell:

`openocd -f interface/stlink.cfg -f target/stm32h7x.cfg`

with another shell:

`arm-none-eabi-gdb -q target/thumbv7em-none-eabihf/debug/rolkien`

then:

```
target remote :3333
load
c
```

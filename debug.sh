#!/bin/sh

set -ex
sh build.sh
toolset/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi-gdb --tui out/stm32_ws2812.elf --command stm32_ws2812.gdbinit

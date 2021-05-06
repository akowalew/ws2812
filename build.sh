#!/bin/sh

CFLAGS="$CFLAGS -mthumb -mcpu=cortex-m0 -mtune=cortex-m0"
CFLAGS="$CFLAGS -std=c11"
CFLAGS="$CFLAGS -ffunction-sections -fdata-sections -Wl,-gc-sections"
CFLAGS="$CFLAGS -nostartfiles -ffreestanding -nostdlib"
CFLAGS="$CFLAGS -Wl,-T../stm32_ws2812.ld"
CFLAGS="$CFLAGS -Wall -Wextra -Wpedantic -Werror -Wfatal-errors -Wundef -Wshadow"
CFLAGS="$CFLAGS -Wno-unused-variable -Wno-unused-function -Wno-unused-parameter -Wno-unused-but-set-variable"
CFLAGS="$CFLAGS -Og -g3 -ggdb"
CFLAGS="$CFLAGS -Wl,-Map=stm32_ws2812.map"

TOOLSET="toolset/gcc-arm-none-eabi-10-2020-q4-major/bin"

set -ex

mkdir -p out
(cd out; \
 ../$TOOLSET/arm-none-eabi-gcc ../stm32_ws2812.c $CFLAGS -o stm32_ws2812.elf; \
 ../$TOOLSET/arm-none-eabi-objcopy stm32_ws2812.elf -O binary stm32_ws2812.bin; \
 ../$TOOLSET/arm-none-eabi-objdump -S -d -h stm32_ws2812.elf > stm32_ws2812.dump; \
 ../$TOOLSET/arm-none-eabi-size stm32_ws2812.elf)

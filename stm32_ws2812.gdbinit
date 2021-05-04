target extended-remote | openocd -f interface/stlink-v2-1.cfg -f target/stm32f0x_stlink.cfg -c "gdb_port pipe; log_output out/openocd.log"

set confirm off

set print pretty on
set print elements 0
set print array on

define r
    monitor reset init
end

define l
    load
    refresh
end

define rl
    load
    monitor reset init
    refresh
end

define rlc
    shell sh build.sh
    save breakpoints out/stm32_ws2812.gdbbreakpoints
    delete breakpoints
    load
    source out/stm32_ws2812.gdbbreakpoints
    monitor reset init
    continue
    refresh
end

define db
    delete breakpoints
end

b Stm32_Reset_Handler

focus cmd

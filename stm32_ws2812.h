internal noreturn void
Stm32_Reboot(void)
{
    NVIC_SystemReset();
}

internal void
Stm32_FakeDelay(u32 Count)
{
    u32 Elapsed = Count;
    while(Elapsed--)
    {
        __NOP();
    }
}

internal void
Stm32_WaitForTick()
{
    while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG))
    {
        // NOTE: Dummy wait
    }
}

//
// NOTE: Symbols below should be provided by linker script
//

extern u32 _sbss;
extern u32 _ebss;
extern u32 _sdata;
extern u32 _edata;
extern u32 _etext;

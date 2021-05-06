internal noreturn void
Stm32_Reboot(void)
{
    NVIC_SystemReset();
}

internal void
Stm32_NVIC_EnableInterrupt(IRQn_Type Interrupt)
{
    NVIC->ISER[0] = (uint32_t)(1UL << (((uint32_t)Interrupt) & 0x1FUL));
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

internal void
Stm32_LED_Toggle()
{
    GPIOA->ODR ^= (1 << 5);
}

internal void
Stm32_LED_Set()
{
    GPIOA->BSRR = ((1 << 5) << 0);
}

internal void
Stm32_LED_Clear()
{
    GPIOA->BSRR = ((1 << 5) << 16);
}

internal void
Stm32_WS2812_Data_Set()
{
    GPIOA->BSRR = ((1 << 4) << 0);
}

internal void
Stm32_WS2812_Data_Clear()
{
    GPIOA->BSRR = ((1 << 4) << 16);
}

internal void
Stm32_USART_Write(USART_TypeDef* USART, void* DataStart, sz Count)
{
    u8* DataStart8 = (u8*) DataStart;
    u8* DataEnd8 = (DataStart8 + Count);

    u8* Data8 = DataStart8;
    while(Data8 != DataEnd8)
    {
        while(!(USART->ISR & USART_ISR_TXE))
        {
            // NOTE: Busy wait
        }

        USART->TDR = *(Data8++);
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

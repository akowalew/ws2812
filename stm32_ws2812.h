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

internal u32
Stm32_BUTTON_Read()
{
    u32 Result = (GPIOC->IDR & (1 << 13));
    return Result;
}

internal b32
Stm32_BUTTON_IsPressed()
{
    b32 Result = (Stm32_BUTTON_Read() == 0);
    return Result;
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

#define WS2812_DATA_LOW_TIM14_CCR_VALUE (19 - 1)
#define WS2812_DATA_HIGH_TIM14_CCR_VALUE (39 - 1)

internal void
Stm32_TIM14_WriteCompareValue(u16 CompareValue)
{
    while(!(TIM14->SR & TIM_SR_UIF))
    {
        // NOTE: Busy wait
    }

    TIM14->SR = 0;

    TIM14->CCR1 = CompareValue;
}

internal void
Stm32_WS2812_Send(sz Count, u8* DataStart)
{
    u8* Data = DataStart;
    u8* DataEnd = (DataStart + Count);
    while(Data != DataEnd)
    {
        u8 Byte = *(Data++);
        u8 Mask = 0x80;
        while(Mask)
        {
            u16 CompareValue = (Byte & Mask) ? WS2812_DATA_HIGH_TIM14_CCR_VALUE : WS2812_DATA_LOW_TIM14_CCR_VALUE;
            Stm32_TIM14_WriteCompareValue(CompareValue);
            Mask >>= 1;
        }
    }

    sz Elapsed = 50;
    while(Elapsed--)
    {
        Stm32_TIM14_WriteCompareValue(0);
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

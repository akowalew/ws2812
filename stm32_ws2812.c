#include "stm32f070xb.h"

#include "ws2812_platform.h"

#include "stm32_ws2812.h"

private_global u32 Cpu_Hz;
private_global u32 SysTick_Hz;

internal void
Stm32_Init_StaticData()
{
    u32* Bss = &_sbss;
    while(Bss != &_ebss)
    {
        *(Bss++) = 0;
    }

    u32* Data = &_sdata;
    u32* Init = &_etext;
    while(Data != &_edata)
    {
        *(Data++) = *(Init++);
    }
}

internal void
Stm32_Init_Clocks()
{
    RCC->AHBENR = (RCC_AHBENR_DMAEN |
                   RCC_AHBENR_SRAMEN |
                   RCC_AHBENR_FLITFEN |
                   RCC_AHBENR_CRCEN |
                   RCC_AHBENR_GPIOAEN |
                   RCC_AHBENR_GPIOBEN |
                   RCC_AHBENR_GPIOCEN |
                   RCC_AHBENR_GPIODEN |
                   RCC_AHBENR_GPIOFEN);

    RCC->APB1ENR = (RCC_APB1ENR_TIM14EN);

    Cpu_Hz = 8000000;
}

internal void
Stm32_Init_GPIOA()
{
    GPIOA->MODER = (0x28000000 |
                    GPIO_MODER_MODER4_ALTERNATE |
                    GPIO_MODER_MODER5_OUTPUT );

    GPIOA->OTYPER = (0x00000000 |
                     GPIO_OTYPER_OT4_PUSHPULL |
                     GPIO_OTYPER_OT5_PUSHPULL);

    GPIOA->OSPEEDR = (0x0C000000 |
                      GPIO_OSPEEDR_OSPEEDR4_HIGH |
                      GPIO_OSPEEDR_OSPEEDR5_HIGH);

    GPIOA->PUPDR = (0x24000000 |
                    GPIO_PUPDR_PUPDR4_NONE |
                    GPIO_PUPDR_PUPDR5_NONE);

    GPIOA->AFRL = (GPIO_AFRL_AFSEL4_AF4);
}

internal void
Stm32_Init_TIM14()
{
    TIM14->CCMR1 = (TIM_CCMR1_CC1S_OUTPUT |
                    TIM_CCMR1_OC1FE |
                    TIM_CCMR1_OC1PE |
                    TIM_CCMR1_OC1M_PWM_ACTIVE_UNTIL_CCR);

    TIM14->CCER = (TIM_CCER_CC1E |
                   TIM_CCER_CC1P_ACTIVE_HIGH);

    TIM14->PSC = (8000 - 1);

    TIM14->ARR = (1000 - 1);

    TIM14->CCR1 = (250);

    TIM14->CR1 = (TIM_CR1_CEN |
                  TIM_CR1_ARPE);
}

internal void
Stm32_Init_SysTick()
{
    SysTick_Hz = 1;

    u32 LoadValue = (Cpu_Hz / SysTick_Hz);
    Assert(LoadValue <= 0x00FFFFFF);
    SysTick->LOAD = LoadValue;
    SysTick->VAL = 0;
    SysTick->CTRL = (SysTick_CTRL_COUNTFLAG |
                     SysTick_CTRL_CLKSOURCE_CPU |
                     SysTick_CTRL_TICKINT |
                     SysTick_CTRL_ENABLE);
}

internal void
Stm32_Init()
{
    Stm32_Init_StaticData();
    Stm32_Init_Clocks();
    Stm32_Init_GPIOA();
    Stm32_Init_TIM14();
    Stm32_Init_SysTick();
}

internal noreturn void
Stm32_Reset_Handler(void)
{
    Stm32_Init();

    while(1)
    {
        Stm32_LED_Set();
        Stm32_WaitForTick();

        Stm32_LED_Clear();
        Stm32_WaitForTick();
    }
}

internal noreturn void
Stm32_HardFault_Handler(void)
{
#ifndef NDEBUG
    while(1)
    {
        BreakPoint();
    }
#else
    Stm32_Reboot();
#endif
}

internal noreturn void
Stm32_Dummy_Handler(void)
{
#ifndef NDEBUG
    while(1)
    {
        BreakPoint();
    }
#else
    Stm32_Reboot();
#endif
}

#if 1
#define Stm32_NMI_Handler Stm32_Dummy_Handler
#else
internal void
Stm32_NMI_Handler()
{

}
#endif

#if 1
#define Stm32_SVC_Handler Stm32_Dummy_Handler
#else
internal void
Stm32_SVC_Handler()
{

}
#endif

#if 1
#define Stm32_PendSV_Handler Stm32_Dummy_Handler
#else
internal void
Stm32_PendSV_Handler()
{

}
#endif

#if 0
#define Stm32_SysTick_Handler Stm32_Dummy_Handler
#else
internal void
Stm32_SysTick_Handler()
{

}
#endif

#if 1
#define Stm32_WWDG_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_WWDG_IRQHandler()
{

}
#endif

#if 1
#define Stm32_RTC_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_RTC_IRQHandler()
{

}
#endif

#if 1
#define Stm32_FLASH_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_FLASH_IRQHandler()
{

}
#endif

#if 1
#define Stm32_RCC_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_RCC_IRQHandler()
{

}
#endif

#if 1
#define Stm32_EXTI0_1_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_EXTI0_1_IRQHandler()
{

}
#endif

#if 1
#define Stm32_EXTI2_3_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_EXTI2_3_IRQHandler()
{

}
#endif

#if 1
#define Stm32_EXTI4_15_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_EXTI4_15_IRQHandler()
{

}
#endif

#if 1
#define Stm32_DMA1_Channel1_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_DMA1_Channel1_IRQHandler()
{

}
#endif

#if 1
#define Stm32_DMA1_Channel2_3_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_DMA1_Channel2_3_IRQHandler()
{

}
#endif

#if 1
#define Stm32_DMA1_Channel4_5_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_DMA1_Channel4_5_IRQHandler()
{

}
#endif

#if 1
#define Stm32_ADC1_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_ADC1_IRQHandler()
{

}
#endif

#if 1
#define Stm32_TIM1_BRK_UP_TRG_COM_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_TIM1_BRK_UP_TRG_COM_IRQHandler()
{

}
#endif

#if 1
#define Stm32_TIM1_CC_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_TIM1_CC_IRQHandler()
{

}
#endif

#if 1
#define Stm32_TIM3_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_TIM3_IRQHandler()
{

}
#endif

#if 1
#define Stm32_TIM6_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_TIM6_IRQHandler()
{

}
#endif

#if 1
#define Stm32_TIM7_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_TIM7_IRQHandler()
{

}
#endif

#if 1
#define Stm32_TIM14_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_TIM14_IRQHandler()
{

}
#endif

#if 1
#define Stm32_TIM15_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_TIM15_IRQHandler()
{

}
#endif

#if 1
#define Stm32_TIM16_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_TIM16_IRQHandler()
{

}
#endif

#if 1
#define Stm32_TIM17_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_TIM17_IRQHandler()
{

}
#endif

#if 1
#define Stm32_I2C1_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_I2C1_IRQHandler()
{

}
#endif

#if 1
#define Stm32_I2C2_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_I2C2_IRQHandler()
{

}
#endif

#if 1
#define Stm32_SPI1_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_SPI1_IRQHandler()
{

}
#endif

#if 1
#define Stm32_SPI2_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_SPI2_IRQHandler()
{

}
#endif

#if 1
#define Stm32_USART1_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_USART1_IRQHandler()
{

}
#endif

#if 1
#define Stm32_USART2_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_USART2_IRQHandler()
{

}
#endif

#if 1
#define Stm32_USART3_4_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_USART3_4_IRQHandler()
{

}
#endif

#if 1
#define Stm32_USB_IRQHandler Stm32_Dummy_Handler
#else
internal void
Stm32_USB_IRQHandler()
{

}
#endif

private_global in_section(".handlers")
void (*_handlers[])() =
{
    Stm32_Reset_Handler,
    Stm32_NMI_Handler,
    Stm32_HardFault_Handler,
    Stm32_Dummy_Handler,
    Stm32_Dummy_Handler,
    Stm32_Dummy_Handler,
    Stm32_Dummy_Handler,
    Stm32_Dummy_Handler,
    Stm32_Dummy_Handler,
    Stm32_Dummy_Handler,
    Stm32_SVC_Handler,
    Stm32_Dummy_Handler,
    Stm32_Dummy_Handler,
    Stm32_PendSV_Handler,
    Stm32_SysTick_Handler,
    Stm32_WWDG_IRQHandler,
    Stm32_Dummy_Handler,
    Stm32_RTC_IRQHandler,
    Stm32_FLASH_IRQHandler,
    Stm32_RCC_IRQHandler,
    Stm32_EXTI0_1_IRQHandler,
    Stm32_EXTI2_3_IRQHandler,
    Stm32_EXTI4_15_IRQHandler,
    Stm32_Dummy_Handler,
    Stm32_DMA1_Channel1_IRQHandler,
    Stm32_DMA1_Channel2_3_IRQHandler,
    Stm32_DMA1_Channel4_5_IRQHandler,
    Stm32_ADC1_IRQHandler,
    Stm32_TIM1_BRK_UP_TRG_COM_IRQHandler,
    Stm32_TIM1_CC_IRQHandler,
    Stm32_Dummy_Handler,
    Stm32_TIM3_IRQHandler,
    Stm32_TIM6_IRQHandler,
    Stm32_TIM7_IRQHandler,
    Stm32_TIM14_IRQHandler,
    Stm32_TIM15_IRQHandler,
    Stm32_TIM16_IRQHandler,
    Stm32_TIM17_IRQHandler,
    Stm32_I2C1_IRQHandler,
    Stm32_I2C2_IRQHandler,
    Stm32_SPI1_IRQHandler,
    Stm32_SPI2_IRQHandler,
    Stm32_USART1_IRQHandler,
    Stm32_USART2_IRQHandler,
    Stm32_USART3_4_IRQHandler,
    Stm32_Dummy_Handler,
    Stm32_USB_IRQHandler,
};

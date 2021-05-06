#include "stm32f070xb.h"

#include "ws2812_platform.h"

#include "stm32_ws2812.h"

private_global u32 Cpu_Hz;
private_global u32 SysTick_Hz;
private_global u32 USART2_BaudRate;

internal void
Stm32_Init_StaticData()
{
    u32* Bss = &_sbss;
    while(Bss < &_ebss)
    {
        *(Bss++) = 0;
    }

    u32* Data = &_sdata;
    u32* Init = &_etext;
    while(Data < &_edata)
    {
        u32 InitValue = *(Init++);
        *(Data++) = InitValue;
    }
}

internal void
Stm32_Init_FLASH()
{
    FLASH->ACR = (FLASH_ACR_LATENCY_ONE_WAIT_STATE |
                  FLASH_ACR_PRFTBE);
}

internal void
Stm32_Init_Clocks()
{
    u32 RCC_CFGR_Rest = (RCC_CFGR_HPRE_DIV1 |
                         RCC_CFGR_PPRE_DIV1 |
                         RCC_CFGR_PLLSRC_HSI_DIV2 |
                         RCC_CFGR_PLLMUL12 |
                         RCC_CFGR_MCO_NOCLOCK);

    RCC->CFGR = (RCC_CFGR_SW_HSI |
                 RCC_CFGR_Rest);

    RCC->CR |= (RCC_CR_PLLON);

    while(!(RCC->CR & RCC_CR_PLLRDY))
    {
        // NOTE: Busy wait
    }

    RCC->CFGR = (RCC_CFGR_SW_PLL |
                 RCC_CFGR_Rest);

    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    {
        // NOTE: Busy wait
    }

    Cpu_Hz = 48000000;

    RCC->AHBENR = (RCC_AHBENR_DMAEN |
                   RCC_AHBENR_SRAMEN |
                   RCC_AHBENR_FLITFEN |
                   RCC_AHBENR_CRCEN |
                   RCC_AHBENR_GPIOAEN |
                   RCC_AHBENR_GPIOBEN |
                   RCC_AHBENR_GPIOCEN |
                   RCC_AHBENR_GPIODEN |
                   RCC_AHBENR_GPIOFEN);

    RCC->APB1ENR = (RCC_APB1ENR_TIM14EN |
                    RCC_APB1ENR_USART2EN);
}

internal void
Stm32_Init_GPIOA()
{
    GPIOA->MODER = (0x28000000 |
                    GPIO_MODER_MODER2_ALTERNATE |
                    GPIO_MODER_MODER3_ALTERNATE |
                    GPIO_MODER_MODER4_ALTERNATE |
                    GPIO_MODER_MODER5_OUTPUT );

    GPIOA->OTYPER = (0x00000000 |
                     GPIO_OTYPER_OT2_PUSHPULL |
                     GPIO_OTYPER_OT3_PUSHPULL |
                     GPIO_OTYPER_OT4_PUSHPULL |
                     GPIO_OTYPER_OT5_PUSHPULL);

    GPIOA->OSPEEDR = (0x0C000000 |
                      GPIO_OSPEEDR_OSPEEDR2_HIGH |
                      GPIO_OSPEEDR_OSPEEDR3_HIGH |
                      GPIO_OSPEEDR_OSPEEDR4_HIGH |
                      GPIO_OSPEEDR_OSPEEDR5_HIGH);

    GPIOA->PUPDR = (0x24000000 |
                    GPIO_PUPDR_PUPDR2_NONE |
                    GPIO_PUPDR_PUPDR3_NONE |
                    GPIO_PUPDR_PUPDR4_NONE |
                    GPIO_PUPDR_PUPDR5_NONE);

    GPIOA->AFRL = (GPIO_AFRL_AFSEL2_AF1 |
                   GPIO_AFRL_AFSEL3_AF1 |
                   GPIO_AFRL_AFSEL4_AF4);
}

internal void
Stm32_Init_USART2()
{
    USART2->CR3 = (USART_CR3_DMAT);

    USART2_BaudRate = 115200;

    u32 BRR_Floor = (Cpu_Hz / USART2_BaudRate);
    u32 BRR_Ceil = (BRR_Floor + 1);

    u32 BRR_Error_Floor = (Cpu_Hz - BRR_Floor * USART2_BaudRate);
    u32 BRR_Error_Ceil = (BRR_Ceil * USART2_BaudRate - Cpu_Hz);
    u32 BRR = (BRR_Error_Floor <= BRR_Error_Ceil) ? BRR_Floor : BRR_Ceil;

    USART2->BRR = BRR;

    USART2->CR1 = (USART_CR1_UE |
                   USART_CR1_RE |
                   USART_CR1_TE);
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

    TIM14->PSC = 0;

    TIM14->ARR = (60 - 1);

    TIM14->CCR1 = 0;

    TIM14->CR1 = (TIM_CR1_CEN);
}

internal void
Stm32_Init_SysTick()
{
    SysTick_Hz = 50;

    u32 LoadValue = (Cpu_Hz / SysTick_Hz);
    Assert(LoadValue <= 0x00FFFFFF);
    SysTick->LOAD = (LoadValue - 1);
    SysTick->VAL = 0;
    SysTick->CTRL = (SysTick_CTRL_COUNTFLAG |
                     SysTick_CTRL_CLKSOURCE_CPU |
                     SysTick_CTRL_TICKINT |
                     SysTick_CTRL_ENABLE);
}


internal void
Stm32_Init()
{
    Stm32_Init_FLASH();
    Stm32_Init_StaticData();
    Stm32_Init_Clocks();
    Stm32_Init_GPIOA();
    Stm32_Init_USART2();
    Stm32_Init_SysTick();
    Stm32_Init_TIM14();
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

    Stm32_TIM14_WriteCompareValue(0);
}

private_global u8 LookupTable[] =
{
    0, 36, 57, 72, 84, 94, 102, 109,
    115, 121, 126, 130, 134, 138, 142, 145,
    148, 151, 154, 157, 160, 162, 164, 167,
    169, 171, 173, 175, 176, 178, 180, 182,
    183, 185, 186, 188, 189, 191, 192, 193,
    195, 196, 197, 198, 200, 201, 202, 203,
};

u8 Buffer[3 * 8];

internal noreturn void
Stm32_Reset_Handler(void)
{
    Stm32_Init();

    while(1)
    {
        if(USART2->ISR & USART_ISR_ORE)
        {
            USART2->ICR = USART_ICR_ORECF;
        }

        if(USART2->ISR & USART_ISR_RXNE)
        {
            u8 RxByte = USART2->RDR;

            c8 RxChar = ToLower(RxByte);
            switch(RxChar)
            {
                case 'q': for(sz Idx = 0; Idx < 8; Idx++) { SaturateIncrementU8(Buffer + Idx*3 + 0); } break;
                case 'a': for(sz Idx = 0; Idx < 8; Idx++) { SaturateDecrementU8(Buffer + Idx*3 + 0); } break;

                case 'w': for(sz Idx = 0; Idx < 8; Idx++) { SaturateIncrementU8(Buffer + Idx*3 + 1); } break;
                case 's': for(sz Idx = 0; Idx < 8; Idx++) { SaturateDecrementU8(Buffer + Idx*3 + 1); } break;

                case 'e': for(sz Idx = 0; Idx < 8; Idx++) { SaturateIncrementU8(Buffer + Idx*3 + 2); } break;
                case 'd': for(sz Idx = 0; Idx < 8; Idx++) { SaturateDecrementU8(Buffer + Idx*3 + 2); } break;
            }
        }

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
    Stm32_LED_Set();
    Stm32_WS2812_Send(sizeof(Buffer), Buffer);
    Stm32_LED_Clear();
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

//#include "stdint.h"

#define uint32_t unsigned int


extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _estack;

extern "C"{

    extern int main(void);
    void Reset_Handler(void);
    void Default_Handler(void);
    extern void __libc_init_array(void);

} 

__attribute__((weak, alias("Default_Handler"))) void ADC_IRQHandler (void);

__attribute__((weak, alias("Default_Handler"))) void EXTI0_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void EXTI1_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void EXTI2_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void EXTI3_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void EXTI4_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void EXTI9_5_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void EXTI15_10_IRQHandler (void);

__attribute__((weak, alias("Default_Handler"))) void DMA2_Stream0_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void DMA2_Stream1_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void DMA2_Stream2_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void DMA2_Stream3_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void DMA2_Stream4_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void DMA2_Stream5_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void DMA2_Stream6_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void DMA2_Stream7_IRQHandler (void);

__attribute__((weak, alias("Default_Handler"))) void DMA1_Stream0_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void DMA1_Stream1_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void DMA1_Stream2_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void DMA1_Stream3_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void DMA1_Stream4_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void DMA1_Stream5_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void DMA1_Stream6_IRQHandler (void);
__attribute__((weak, alias("Default_Handler"))) void DMA1_Stream7_IRQHandler (void);

__attribute__((weak, alias("Default_Handler"))) void I2C1_EV_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void I2C1_ER_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void I2C2_EV_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void I2C2_ER_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void I2C3_EV_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void I2C3_ER_IRQHandler(void);


__attribute__((weak, alias("Default_Handler"))) void TIM1_BRK_TIM9_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIM1_UP_TIM10_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIM1_TRG_COM_TIM11_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIM1_CC_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIM2_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIM3_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIM4_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIM5_IRQHandler(void);

__attribute__((weak, alias("Default_Handler"))) void USART1_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void USART2_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void USART6_IRQHandler(void);

uint32_t* const vectors[] __attribute__((section(".isr_vector"))) ={
       
    (uint32_t*)&_estack,                         /* 0x000 Stack Pointer */
    (uint32_t*)Reset_Handler,                       /* 0x004 Reset */
    (uint32_t*)0,                                   /* 0x008 Non maskable interrupt */
    (uint32_t*)0,                                   /* 0x00C HardFault */
    (uint32_t*)0,                                   /* 0x010 Memory Management */
    (uint32_t*)0,                                   /* 0x014 BusFault */
    (uint32_t*)0,                                   /* 0x018 UsageFault */
    (uint32_t*)0,                                   /* 0x01C Reserved */
    (uint32_t*)0,                                   /* 0x020 Reserved */
    (uint32_t*)0,                                   /* 0x024 Reserved */
    (uint32_t*)0,                                   /* 0x028 Reserved */
    (uint32_t*)0,                                   /* 0x02C System service call */
    (uint32_t*)0,                                   /* 0x030 Debug Monitor */
    (uint32_t*)0,                                   /* 0x034 Reserved */
    (uint32_t*)0,                                   /* 0x038 PendSV */
    (uint32_t*)0,                                   /* 0x03C System tick timer */
    (uint32_t*)0,                                   /* 0 - 0x040 Window watchdog */
    (uint32_t*)0,                                   /* 1 - 0x044 PVD through EXTI Line detection */
    (uint32_t*)0,                                   /* 2 - 0x048 Tamper */
    (uint32_t*)0,                                   /* 3 - 0x04C RTC Wakeup through the EXTI line */
    (uint32_t*)0,                                   /* 4 - 0x050 FLASH global */
    (uint32_t*)0,                                   /* 5 - 0x054 RCC global */
    (uint32_t*)EXTI0_IRQHandler,                    /* 6 - 0x058 EXTI Line0 */
    (uint32_t*)EXTI1_IRQHandler,                                   /* 7 - 0x05C EXTI Line1 */
    (uint32_t*)EXTI2_IRQHandler,                                   /* 8 - 0x060 EXTI Line2 */
    (uint32_t*)EXTI3_IRQHandler,                                   /* 9 - 0x064 EXTI Line3 */
    (uint32_t*)EXTI4_IRQHandler,                                   /* 10 - 0x068 EXTI Line4 */
    (uint32_t*)DMA1_Stream0_IRQHandler,    /* 11 - 0x06C DMA1_Ch1 */
    (uint32_t*)DMA1_Stream1_IRQHandler,                                  /* 12 - 0x070 DMA1_Ch2 */
    (uint32_t*)DMA1_Stream2_IRQHandler,                                  /* 13 - 0x074 DMA1_Ch3 */
    (uint32_t*)DMA1_Stream3_IRQHandler,                                /* 14 - 0x078 DMA1_Ch4 */
    (uint32_t*)DMA1_Stream4_IRQHandler,                                  /* 15 - 0x07C DMA1_Ch5 */
    (uint32_t*)DMA1_Stream5_IRQHandler,                   /* 16 - 0x080 DMA1_Ch6 */
    (uint32_t*)DMA1_Stream6_IRQHandler,                   /* 17 - 0x084 DMA1_Ch7 */
    (uint32_t*)ADC_IRQHandler,                   /* 18 - 0x088 ADC1 and ADC2 global */
    (uint32_t*)0,                                                   /* 19 - 0x08C Reserved */
    (uint32_t*)0,                                                   /* 20 - 0x090 Reserved */
    (uint32_t*)0,                                                   /* 21 - 0x094 Reserved */
    (uint32_t*)0,                                                   /* 22 - 0x098 Reserved */
    (uint32_t*)EXTI9_5_IRQHandler,                                  /* 23 - 0x09C EXTI Lines 9:5  */
    (uint32_t*)TIM1_BRK_TIM9_IRQHandler,                                                   /* 24 - 0x0A0 TIM1 Break / TIM9 global*/
    (uint32_t*)TIM1_UP_TIM10_IRQHandler,                                                   /* 25 - 0x0A4 TIM1 Update / TIM10 global*/
    (uint32_t*)TIM1_TRG_COM_TIM11_IRQHandler,                                                   /* 26 - 0x0A8 TIM1 Trigger and Communication / TIM11 global */
    (uint32_t*)TIM1_CC_IRQHandler,                                                   /* 27 - 0x0AC TIM1 Capture Compare */
    (uint32_t*)TIM2_IRQHandler,                                                   /* 28 - 0x0B0 TIM2 */
    (uint32_t*)TIM3_IRQHandler,                                                   /* 29 - 0x0B4 TIM3 */
    (uint32_t*)TIM4_IRQHandler,                                                   /* 30 - 0x0B8 TIM4 */
    (uint32_t*) I2C1_EV_IRQHandler ,                                /* 31 - 0x0BC I2C1 event */
    (uint32_t*) I2C1_ER_IRQHandler ,                                /* 32 - 0x0C0 I2C1 error */
    (uint32_t*) I2C2_EV_IRQHandler ,                                /* 33 - 0x0C4 I2C2 event */
    (uint32_t*) I2C2_ER_IRQHandler ,                                /* 34 - 0x0C8 I2C2 error */
    (uint32_t*)0,                                                   /* 35 - 0x0CC SPI1 */
    (uint32_t*)0,                                                   /* 36 - 0x0D0 SPI2 */
    (uint32_t*)USART1_IRQHandler,                                                   /* 37 - 0x0D4 USART1 */
    (uint32_t*)USART2_IRQHandler,                                                   /* 38 - 0x0D8 USART2 */
    (uint32_t*)0,                                                   /* 39 - 0x0DC RESERVED */
    (uint32_t*)EXTI15_10_IRQHandler,                                /* 40 - 0x0E0 EXTI Lines 15:10     */
    (uint32_t*)0,                                                   /* 41 - 0x0E4 RTC alarm through EXTI line */
    (uint32_t*)0,                                                   /* 42 - 0xE8  USB OTG FS Wakeup through EXTI */
    (uint32_t*)0,                                                   /* 43 - 0xEC Reserved */
    (uint32_t*)0,                                                   /* 44 - 0xF0 Reserved */
    (uint32_t*)0,                                                   /* 45 - 0xF4 Reserved */
    (uint32_t*)0,                                                   /* 46 - 0xF8 Reserved */
    (uint32_t*)DMA1_Stream7_IRQHandler,                             /* 47 - 0xFC DMA1 stream 7 */
    (uint32_t*)0,                                                   /* 48 - 0x100 Reserved */
    (uint32_t*)0,                                                   /* 49 - 0x104 SDIO_IRQHandler */
    (uint32_t*)TIM5_IRQHandler,                                                   /* 50 - 0x108 TIM5 global */
    (uint32_t*)0,                                                   /* 51 - 0x10C SPI3 */
    (uint32_t*)0,                                                   /* 52 - 0x110 Reserved */
    (uint32_t*)0,                                                   /* 53 - 0x114 Reserved */
    (uint32_t*)0,                                                   /* 54 - 0x118 Reserved */
    (uint32_t*)0,                                                   /* 55 - 0x11C Reserved */
    (uint32_t*)DMA2_Stream0_IRQHandler,                             /* 56 - 0x120 DMA2 stream 0 */
    (uint32_t*)DMA2_Stream1_IRQHandler,                             /* 57 - 0x124 DMA2 stream 1 */
    (uint32_t*)DMA2_Stream2_IRQHandler,                             /* 58 - 0x128 DMA2 stream 2 */
    (uint32_t*)DMA2_Stream3_IRQHandler,                             /* 59 - 0x12C DMA2 stream 3 */
    (uint32_t*)DMA2_Stream4_IRQHandler,                             /* 60 - 0x130 DMA2 stream 4 */
    (uint32_t*)0,                                                   /* 61 - 0x134 Reserved */
    (uint32_t*)0,                                                   /* 62 - 0x138 Reserved */
    (uint32_t*)0,                                                   /* 63 - 0x13C Reserved */
    (uint32_t*)0,                                                   /* 64 - 0x140 Reserved */
    (uint32_t*)0,                                                   /* 65 - 0x144 Reserved */
    (uint32_t*)0,                                                   /* 66 - 0x148 Reserved */
    (uint32_t*)0,                                                   /* 67 - 0x14C USB OTG FS global */
    (uint32_t*)DMA2_Stream5_IRQHandler,				                /* 68 - 0x150 DMA2 stream 5 */
    (uint32_t*)DMA2_Stream6_IRQHandler,				                /* 69 - 0x154 DMA2 stream 6 */
    (uint32_t*)DMA2_Stream7_IRQHandler,				                /* 70 - 0x158 DMA2 stream 7 */
    (uint32_t*)USART6_IRQHandler,				                                    /* 71 - 0x15C USART6_IRQHandler  */
    (uint32_t*)I2C3_EV_IRQHandler,				                    /* 72 - 0x160 I2C3_EV_IRQHandler */
    (uint32_t*)I2C3_ER_IRQHandler,				                    /* 73 - 0x164 I2C3_ER_IRQHandler */
    (uint32_t*)0,                   /* 74 - 0x168 Reserved */
    (uint32_t*)0,                   /* 75 - 0x16C Reserved */
    (uint32_t*)0,                   /* 76 - 0x170 Reserved */
    (uint32_t*)0,                   /* 77 - 0x174 Reserved */
    (uint32_t*)0,                   /* 78 - 0x178 Reserved */
    (uint32_t*)0,                   /* 79 - 0x17C Reserved */
    (uint32_t*)0,                   /* 80 - 0x180 Reserved */
    (uint32_t*)0,                   /* 81 - 0x184 FPU global interrupt */
    (uint32_t*)0,                   /* 82 - 0x188 Reserved */
    (uint32_t*)0,                   /* 83 - 0x18C Reserved */
    (uint32_t*)0,                   /* 84 - 0x190 SPI4_IRQHandler */
    (uint32_t*)0,                   /* 85 - 0x194 SPI5_IRQHandler */
};



void Reset_Handler(void){
    
    uint32_t *dataInit = &_sidata;
    uint32_t *data = &_sdata;
    while(data < &_edata){
     
        *data++ = *dataInit++;
        
    }
    uint32_t *bss = &_sbss;
    while(bss < &_ebss){
     
        *bss++ = 0;
        
    }
    
    __libc_init_array();
    main();
    
    
}




void Default_Handler(void){
    
        while(1);
}


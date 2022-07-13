#ifndef BASIC_TIMER_H
#define BASIC_TIMER_H


#include "DEFINITIONS.h"
#include <stddef.h>
#include <stdint.h>
#include "stm32f411xe.h"
#include "assert.h"


namespace timNameSpace{

    /* defines */

    /* macros */

    #define TIM_UPDATE_INTERRUPT_POS   (1 << 0)
    #define TIM_UPDATE_INTERRUPT_TAKE_DOWN_FLAG(SR) ((SR) & TIM_UPDATE_INTERRUPT_POS ? SR &= ~TIM_UPDATE_INTERRUPT_POS : 0x0U)

    #define GET_TIM_PERIPHERAL_CLOCK_INDEX(TIM) (((TIM) == TIM1 || (TIM) == TIM2) ? 0x1U << 0 :       \
                                                 (TIM) == TIM3 ?    0x1U << 1 :                          \
                                                 (TIM) == TIM4 ?    0x1U << 2 :                           \
                                                 (TIM) == TIM5 ?    0x1U << 3 :                           \
                                                 (TIM) == TIM9 ?    0x1U << 16 :                          \
                                                 (TIM) == TIM10 ?   0x1U << 17 :                         \
                                                                    0x1U << 18)

    #define ASSERT_TIM_COUNTING_MODE(MODE)  (((MODE) = TIM_UPCOUNTING_MODE ) || ((MODE) == TIM_DOWNCOUNTING_MODE))

    #define ASSERT_TIM_MODE(MODE)           (((MODE) == TIM_TIME_BASE_MODE)          || \
                                            ((MODE) == TIM_PWM_MODE)                || \
                                            ((MODE) == TIM_INPUT_CAPTURE_MODE)      || \
                                            ((MODE) == TIM_OUTPUT_COMPARE_MODE))


    #define ASSERT_TIM_INSTANCE(TIM)    (((TIM) == TIM1)  || \
                                        ((TIM) == TIM2)  || \
                                        ((TIM) == TIM3)  || \
                                        ((TIM) == TIM4)  || \
                                        ((TIM) == TIM5)  || \
                                        ((TIM) == TIM9)  || \
                                        ((TIM) == TIM10) || \
                                        ((TIM) == TIM11))
    
    enum TIM_COUNTING_MODE{

        TIM_UPCOUNTING_MODE = ((~(0x1 << 4)) & (0x1 << 4)),
        TIM_DOWNCOUNTING_MODE = (0x1 << 4),
    };

    enum TIM_ID{

        TIM_1 = 0x1U,
        TIM_2 = 0x2U, 
        TIM_3 = 0x3U,
        TIM_4 = 0x4U,
        TIM_5 = 0x5U,
        TIM_9 = 0x9U,
        TIM_10 = 0xA,
        TIM_11 = 0xB,

    };


    /* enums */
    enum TIM_STATUS{

        TIM_STATE_RESET = 0x0,
        TIM_STATE_READY = 0x1,
        TIM_STATE_BUSY = 0x2,
        TIM_STATE_TIMEOUT = 0x4,
        TIM_STATE_ERROR = 0x8,
    };

    enum TIM_CHANNEL_STATUS{

        TIM_CHANNEL_RESET = 0x0,
        TIM_CHANNEL_READY = 0x1,
        TIM_CHANNEL_BUSY = 0x2,
    };

    enum TIM_ACTIVE_CHANNEL{

        TIM_ACTIVE_CHANNEL_1 = 0x1,
        TIM_ACTIVE_CHANNEL_2 = 0x2,
        TIM_ACTIVE_CHANNEL_3 = 0x4,
        TIM_ACTIVE_CHANNEL_4 = 0x8,
        TIM_ACTIVE_CHANNELS_1_2 = 0x3,
        TIM_ACTIVE_CHANNELS_1_3 = 0x5,
        TIM_ACTIVE_CHANNELS_1_4 = 0x9,
        TIM_ACTIVE_CHANNELS_2_3 = 0x6,
        TIM_ACTIVE_CHANNELS_2_4 = 0xA,
        TIM_ACTIVE_CHANNELS_3_4 = 0xC,
        TIM_ACTIVE_CHANNELS_ALL = 0xF,

    };

    enum TIM_CLOCK_DIVISION{

        TIM_CLOCK_NO_DIVISION = (uint16_t)((~(0x3 << 8)) & (0x3 << 8)),
        TIM_CLOCK_DIVISION_2 = (uint16_t)(0x01 << 8),
        TIM_CLOCK_DIVISION_4 = (uint16_t)(0x02 << 8),
    };

    enum BASIC_TIMER_UPDATE_INTERRUPT_ENABLE{

        INTERRUPT_DISABLE = 0x0,
        INTERRUPT_ENABLE = 0x1,

    };
     
    enum TIM_ARPE{

        ARR_NOT_BUFFERED = ((~(0x1 << 7)) & (0x1 << 7)),
        ARR_BUFFERED = (0x1 << 7),
    };


    enum ONE_PULSE_MODE_SELECTION{

        ONE_PULSE_MODE_DISABLED = ((~(0x1 << 3)) & (0x1 << 3)),
        ONE_PULSE_MODE_ENABLED = 0x1 << 3,
    };
    
    struct BASIC_TIMER_PARAMETERS_STRUCT{

        ONE_PULSE_MODE_SELECTION onePulseMode;
        TIM_COUNTING_MODE counterMode;
        uint32_t prescaler; 
        uint32_t period;
        uint8_t repetitionCounter;
        TIM_ARPE autoReloadPrealoadEnable;
        BASIC_TIMER_UPDATE_INTERRUPT_ENABLE interruptEnable;
        uint8_t interruptPriority;

    };

    class basicTimer 
    {

        private:


            /* function that enables the corresponding peripheral clock according to the GPIO being initialized */
            virtual void inline enableNVIC_Interrupt();
            virtual void inline disableNVIC_Interrupt();
            virtual void peripheralClockEnable();

            TIM_STATUS status;
            defsNameSpace::TASK_LOCK lock;

            TIM_TypeDef *instance;
            TIM_TypeDef resetValues;

        public:
            
            basicTimer(TIM_TypeDef *TIMx);
            ~basicTimer();

            BASIC_TIMER_PARAMETERS_STRUCT parameters;

            virtual defsNameSpace::TASK_STATUS init();
            virtual defsNameSpace::TASK_STATUS deInit();

            virtual void timerStart(void);
            virtual void timerStop(void);




    };

}


#endif



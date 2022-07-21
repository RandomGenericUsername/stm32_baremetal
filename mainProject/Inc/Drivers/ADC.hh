#ifndef ADC_H
#define ADC_H

#include "stdint.h"
#include "stm32f411xe.h"
#include "assert.h"
/* Namespace for the adc class and pertinent structs and definitions */

namespace adcNameSpace{


    enum ADC_CHANNELS{

        ADC_CHANNEL_0 =  0,
        ADC_CHANNEL_1 =  1,
        ADC_CHANNEL_2 =  2,
        ADC_CHANNEL_3 =  3,
        ADC_CHANNEL_4 =  4,
        ADC_CHANNEL_5 =  5,
        ADC_CHANNEL_6 =  6,
        ADC_CHANNEL_7 =  7,
        ADC_CHANNEL_8 =  8,
        ADC_CHANNEL_9 =  9,
        ADC_CHANNEL_10 = 10,
        ADC_CHANNEL_11 = 11,
        ADC_CHANNEL_12 = 12,
        adc_channel_13 = 13,
        ADC_CHANNEL_14 = 14,
        ADC_CHANNEL_15 = 15,
    };

    struct ADC_PARAMETERS{

        private:

            uint32_t samplingTime[2];

        public:

            uint32_t overRunInterruptEnable;
            uint32_t resolution;
            uint32_t endOfConversionInterruptEnable;
            uint32_t dataAlignment;
            uint32_t endOfConversionInterruptSelection;
            uint32_t directMemoryAccessEnable;
            uint32_t continuousConversionEnable;
            void singleChannelSamplingTime(uint32_t adcChannel, uint32_t samplingTimeParam);

    };

    class _ADC_ 
    {
    private:
        /* data */

    public:
        _ADC_(ADC_TypeDef *instance);
        ~_ADC_();
        
        ADC_PARAMETERS settings;






    };
    
    


}


#endif
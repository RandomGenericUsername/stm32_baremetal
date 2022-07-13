#include "ADC.hh"

adcNameSpace::_ADC_::_ADC_(ADC_TypeDef *instance){



}
    
adcNameSpace::_ADC_::~_ADC_(){

}


void adcNameSpace::ADC_PARAMETERS::singleChannelSamplingTime(uint32_t adcChannel, uint32_t samplingTimeParam){

    uint32_t tempSamplingTime = samplingTime[adcChannel >> 4];
    uint32_t regLength = 3U*(adcChannel  & 0xFU);
    tempSamplingTime &= ~ (0x7 << regLength);
    tempSamplingTime |= (samplingTimeParam << regLength);
    //samplingTime[adcChannel >> 4] = 
    //tempSamplingTime |= 
}




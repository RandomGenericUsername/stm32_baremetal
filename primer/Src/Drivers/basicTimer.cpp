#include "basicTimer.hh"



using namespace timNameSpace;

basicTimer::basicTimer(TIM_TypeDef *TIMx){

    /*these are the steps to follow by every single library*/

    /* 1) assert the peripheral register struct */
    assert(ASSERT_TIM_INSTANCE(TIMx));
    this->instance = TIMx;
    resetValues = *TIMx;

    /* assign each class's instance ptr to the asserted register_TypeDef */
    /*store the reset values */

    /*assing hightes nvic priority by default*/
    parameters.interruptPriority = defsNameSpace::NVIC_PRIORITY_2;

}

basicTimer::~basicTimer(){



}
defsNameSpace::TASK_STATUS basicTimer::init(){


    peripheralClockEnable();
    

    /* specific settings for timer 1,2,3,4,5 */
    if(instance == TIM1){

        instance->RCR = parameters.repetitionCounter;
    }
    if((instance != TIM9) && (instance != TIM10) && (instance != TIM11)){

        instance->CR1 |= parameters.counterMode;
    }
    instance->CR1 |= (parameters.autoReloadPrealoadEnable | parameters.onePulseMode);
    instance->PSC = parameters.prescaler - 1;
    instance->ARR = parameters.period;
    instance->DIER |= parameters.interruptEnable;
    instance->EGR |= 1 << 0;
    instance->SR &= ~(0x1U << 0);
    if(parameters.interruptEnable == INTERRUPT_ENABLE){

        enableNVIC_Interrupt();
    } 
    return defsNameSpace::OK;

}

defsNameSpace::TASK_STATUS basicTimer::deInit(){


    return defsNameSpace::OK;
}

void basicTimer::peripheralClockEnable(){


    if((this->instance == TIM2) || (this->instance == TIM3) || (this->instance == TIM4) || (this->instance == TIM5)){

        RCC->APB1ENR |= GET_TIM_PERIPHERAL_CLOCK_INDEX(this->instance);

    }else{

        RCC->APB2ENR |= GET_TIM_PERIPHERAL_CLOCK_INDEX(this->instance);

    } 

}


void basicTimer::enableNVIC_Interrupt(){

    if(this->instance == TIM1){

        NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
        NVIC_SetPriority(TIM1_UP_TIM10_IRQn, parameters.interruptPriority);


    }else if(this->instance == TIM2){

        NVIC_EnableIRQ(TIM2_IRQn);
        NVIC_SetPriority(TIM2_IRQn, parameters.interruptPriority);

    }else if(this->instance == TIM3){

        NVIC_EnableIRQ(TIM3_IRQn);
        NVIC_SetPriority(TIM3_IRQn, parameters.interruptPriority);

    }else if(this->instance == TIM4){

        NVIC_EnableIRQ(TIM4_IRQn);
        NVIC_SetPriority(TIM4_IRQn, parameters.interruptPriority);

    }else if(this->instance == TIM5){

        NVIC_EnableIRQ(TIM5_IRQn);
        NVIC_SetPriority(TIM5_IRQn, parameters.interruptPriority);

    }else if(this->instance == TIM9){

        NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
        NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, parameters.interruptPriority);

    }else if(this->instance == TIM10){

        NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
        NVIC_SetPriority(TIM1_UP_TIM10_IRQn, parameters.interruptPriority);

    }else if(this->instance == TIM11){

        NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
        NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, parameters.interruptPriority);

    }

}

void basicTimer::disableNVIC_Interrupt(){



}

inline void basicTimer::timerStart(void){

    instance->CR1 |= 1 << 0;

}

inline void basicTimer::timerStop(void){

    instance->CR1 &= ~(1 << 0);

}
void __attribute__((weak)) TIM1_UpdateCallback(void){

    __NOP();

}
void __attribute__((weak)) TIM2_UpdateCallback(void){

    __NOP();

}

void __attribute__((weak)) TIM3_UpdateCallback(void){

    __NOP();

}
void __attribute__((weak)) TIM4_UpdateCallback(void){

    __NOP();

}
void __attribute__((weak)) TIM5_UpdateCallback(void){

    __NOP();

}
void __attribute__((weak)) TIM9_UpdateCallback(void){

    __NOP();

}
void __attribute__((weak)) TIM10_UpdateCallback(void){

    __NOP();

}
void __attribute__((weak)) TIM11_UpdateCallback(void){

    __NOP();

}

void TIM2_IRQHandler(void){

    if(TIM_UPDATE_INTERRUPT_TAKE_DOWN_FLAG(TIM2->SR)){
        TIM2_UpdateCallback();
    }

}
void TIM3_IRQHandler(void){

    if(TIM_UPDATE_INTERRUPT_TAKE_DOWN_FLAG(TIM3->SR)){
        TIM3_UpdateCallback();
    }
}
void TIM4_IRQHandler(void){

    if(TIM_UPDATE_INTERRUPT_TAKE_DOWN_FLAG(TIM4->SR)){
        TIM4_UpdateCallback();
    }
}
void TIM5_IRQHandler(void){

    if(TIM_UPDATE_INTERRUPT_TAKE_DOWN_FLAG(TIM5->SR)){
        TIM5_UpdateCallback();
    }
}

void TIM1_BRK_TIM9_IRQHandler(void){

    if(TIM_UPDATE_INTERRUPT_TAKE_DOWN_FLAG(TIM9->SR)){
        TIM9_UpdateCallback();
    }

}

void TIM1_UP_TIM10_IRQHandler(void){

    if(TIM_UPDATE_INTERRUPT_TAKE_DOWN_FLAG(TIM10->SR)){
        TIM10_UpdateCallback();
    }
}
void TIM1_TRG_COM_TIM11_IRQHandler(void){

    if(TIM_UPDATE_INTERRUPT_TAKE_DOWN_FLAG(TIM11->SR)){
        TIM11_UpdateCallback();
    }
}

void TIM1_CC_IRQHandler(void){
    
}
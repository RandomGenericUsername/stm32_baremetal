#include "GPIO.hh"

using namespace gpioNameSpace;
using namespace defsNameSpace;

uint16_t GPIO::extiLinesEnabled = 0;
GPIO_ALLOCATED_RESOURCES GPIO::allocatedResources = {0};


//<-------------------------- CLASS CONSTRUCTOR AND DESTRUCTOR -------------------------->// 

/**
 * @brief Construct a new GPIO object from the namespace gpioNamceSpace
 */
GPIO::GPIO(GPIO_TypeDef *GPIOx){

    /* assert for the correct invocation of a *GPIO_Struct */
    assert(ASSERT_GPIO_INSTANCE(GPIOx));
    this->instance = GPIOx;
    resetValues = *GPIOx;
    /*stores the peripheral reset status */

    parameters = new GPIO_PARAMETERS_STRUCT;
    *parameters = {0};
    alreadyRegisteredPin = 0;
    setPort(GPIOx);
    this->parameters->interruptPriority[0] = 0;
    this->parameters->interruptPriority[1] = 0;

    for(uint8_t i = 0; i < 8; i++){

        this->parameters->interruptPriority[0] |= defsNameSpace::NVIC_PRIORITY_2 << (4U * i);
        this->parameters->interruptPriority[1] |= defsNameSpace::NVIC_PRIORITY_2 << (4U * i);
    }

}


/**
 * @brief Destroy the current GPIO object from the namespace gpioNamceSpace
 */
gpioNameSpace::GPIO::~GPIO()
{

}




//<------ METHODS THAT INITIALIZE AND DEINITALIZE THE LOW LEVEL PERIPHERAL HARDWARE --------->// 

defsNameSpace::TASK_STATUS gpioNameSpace::GPIO::init()
{
    /* check for correct allocation of GPIO */
    /* asigns the class's *GPIO_Struct the current *GPIOx   */
    /* set the class member "port" to be in concordance with GPIOx */
    /* enables the peripheal clock*/
    peripheralClockEnable();


    /* Initializes low level hardware */
    if (initPin() == defsNameSpace::ERROR){

        return defsNameSpace::ERROR;
    }

    return defsNameSpace::OK;
}


/**
 * @brief Function that reset the peripheral to its reset state. 
 * @return TASK_STATUS 
 */
defsNameSpace::TASK_STATUS gpioNameSpace::GPIO::deInit(){

    removePin(this->parameters->pin);
    return defsNameSpace::OK;
}



//<------ FUNCTIONS THAT SET UP THE USER CONFIGURATION SETTINGS ------->// 


void GPIO::setPin(uint32_t pin){

    assert(ASSERT_GPIO_PIN(pin));
    this->parameters->pin = pin;    
}


void gpioNameSpace::GPIO::setModer(uint32_t moder, uint32_t pin){

    assert(ASSERT_GPIO_MODE(moder));
    assert(ASSERT_GPIO_PIN(pin));

    setParametersLoop(GPIO_MODE_PARAM, GPIO_MODER_MSK, moder, pin);

}
void GPIO::setModer(uint32_t moder){

    setModer(moder, this->parameters->pin);
}

void gpioNameSpace::GPIO::setOtyper(uint32_t otyper, uint32_t pin){

    assert(ASSERT_GPIO_OTYPE(otyper));
    assert(ASSERT_GPIO_PIN(pin));
    
    setParametersLoop(GPIO_OTYPE_PARAM, GPIO_OT_OTYPER_MSK, otyper, pin);
}
void GPIO::setOtyper(uint32_t otyper){

    setOtyper(otyper, this->parameters->pin);
}


void gpioNameSpace::GPIO::setOSpeed(uint32_t ospeed, uint32_t pin){

    assert(ASSERT_GPIO_OSPEED(ospeed));
    assert(ASSERT_GPIO_PIN(pin));

    setParametersLoop(GPIO_OSPEED_PARAM, GPIO_OSPEEDR_MSK, ospeed, pin);
}
void GPIO::setOSpeed(uint32_t ospeed){

    setOSpeed(ospeed,  this->parameters->pin);
}


void gpioNameSpace::GPIO::setPUPD(uint32_t pupd, uint32_t pin){

    assert(ASSERT_GPIO_PUPD(pupd));
    assert(ASSERT_GPIO_PIN(pin));

    setParametersLoop(GPIO_PUPD_PARAM, GPIO_PUPDR_MSK, pupd, pin);
}
void GPIO::setPUPD(uint32_t pupd){

    setPUPD(pupd, this->parameters->pin);
}


void gpioNameSpace::GPIO::setAF(uint32_t AF, uint32_t pin){

    assert(ASSERT_GPIO_AF(AF));
    assert(ASSERT_GPIO_PIN(pin));

    uint32_t lowReg, highReg;

    lowReg = pin & 0xFF;
    highReg = (pin & 0xFF00) >> 8;
    
    setParametersLoop(GPIO_AF_PARAM, GPIO_AF_MSK, AF, lowReg);
    setParametersHighRegistersLoop(GPIO_AF_PARAM,GPIO_AF_MSK,AF,highReg,1);
}
void GPIO::setAF(uint32_t AF){

    setAF(AF, this->parameters->pin);
}


void GPIO::setEventsAndInterrupts(uint32_t ExI_service, uint32_t triggerSelection, uint32_t pin){

    assert(ASSERT_GPIO_PIN(pin));
    assert(ASSERT_ExI_(ExI_service));
    assert(ASSERT_ExI_TRIGGER(triggerSelection));
    setParametersLoop(GPIO_EXT_ExI_PARAM, EXT_ExI_MSK, ExI_service, pin);
    setParametersLoop(GPIO_EXT_ExI_TRIGGER_PARAM, ExI_TRIGGER_SELECTION_MSK, triggerSelection, pin);

}
void GPIO::setEventsAndInterrupts(uint32_t ExI_service, uint32_t triggerSelection){

    setEventsAndInterrupts(ExI_service, triggerSelection ,this->parameters->pin);

}

void GPIO::SetInterruptPriority(defsNameSpace::NVIC_INTERRUPT_PRIORITY priority, uint32_t pin){

    assert(ASSERT_GPIO_PIN(pin));
    assert(ASSERT_NVIC_INTERRUPT_PRIORITY_SELECTION(priority));
    uint32_t lowPin = pin & 0xFF;
    uint32_t highPin = pin & 0xFF00;
    if(lowPin && !highPin)
    {
        setParametersHighRegistersLoop(GPIO_INTERRUPT_PRIORITY_PARAM, defsNameSpace::NVIC_INTERRUPT_PRIORITY::NVIC_PRIORITY_MSK, priority, lowPin, LOW_REGISTER);
    }
    else if(!lowPin && highPin)
    {
        setParametersHighRegistersLoop(GPIO_INTERRUPT_PRIORITY_PARAM, defsNameSpace::NVIC_INTERRUPT_PRIORITY::NVIC_PRIORITY_MSK, priority, highPin, HIGH_REGISTER);
    }
    else{

        SetInterruptPriority(priority, lowPin);
        SetInterruptPriority(priority, highPin);
    }

}

void GPIO::SetInterruptPriority(defsNameSpace::NVIC_INTERRUPT_PRIORITY priority){


    SetInterruptPriority(priority, this->parameters->pin);
}


//<------ SETTINGS FUNCTIONS THAT ACTUALLY MANIPULATE THE HARDWARE ------->// 

void GPIO::addPin(uint32_t Pin){

    assert(ASSERT_GPIO_PIN(Pin));
    uint32_t pinValueMinusAlreadyRegistered;
    uint32_t existingPin;

    existingPin = this->gpioNameSpace::GPIO::alreadyRegisteredPin ^ Pin;

    pinValueMinusAlreadyRegistered = Pin & existingPin; 

    parameters->pin = parameters->pin | (pinValueMinusAlreadyRegistered | alreadyRegisteredPin);

    initPin();
}


/**
 * @brief this functions removes a certain pin (if already initialized) from the alreadyRegisteredPin register aswell as all pins in the staggin area.
 * @param pin 
 */
void gpioNameSpace::GPIO::removePin(uint32_t pin){

    assert(ASSERT_GPIO_PIN(pin));

    parameters->pin = alreadyRegisteredPin & ~(alreadyRegisteredPin & pin);

    deInitPin();
}


//<------ UTILITY AND GET FUNCTIONS ------->// 

uint32_t gpioNameSpace::GPIO::getPin(){

    return this->parameters->pin;
}

GPIO_PORT gpioNameSpace::GPIO::getPort(){
    return (GPIO_PORT)parameters->port;
}

uint16_t GPIO::getEXTIEnabledLines(){

    return extiLinesEnabled;
}

bool GPIO::isResourceAllocated(GPIO_TypeDef *GPIOx, GPIO_PIN pin)
{

    uint32_t allocatedResourcesAddress = (uint32_t)&allocatedResources;
    allocatedResourcesAddress = allocatedResourcesAddress + (GET_GPIOx_INDEX(GPIOx)*sizeof(uint16_t));
    uint16_t *allocatedResourcesPtr = (uint16_t*)allocatedResourcesAddress;
    if(*allocatedResourcesPtr & pin)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//<------ PERIPHERAL HARDWARE MANUPULATION FUNCTIONS ------->// 

void gpioNameSpace::GPIO::portWrite(GPIO_PIN_STATE state, uint32_t pin){
    
    /* redefine pin to check whether its an already registered pin */
    pin = parameters->pin & pin;
    if(state != GPIO_PIN_RESET){
        instance->BSRR = pin;
    }else{
        instance->BSRR = ((uint32_t)(pin << 16U));
    }

}
void GPIO::portWrite(GPIO_PIN_STATE state){

    portWrite(state, this->parameters->pin);
}

void GPIO::portToggle(uint32_t pin){

    this->instance->ODR ^= pin;
}

void GPIO::portToggle(){

    portToggle(this->parameters->pin);
}

uint32_t gpioNameSpace::GPIO::portRead(uint32_t pin){

    /* redefine pin to check whether its an already registered pin */
    pin = pin & parameters->pin;
    return(instance->IDR & pin);

}
uint32_t GPIO::portRead(){

  return(portRead(this->parameters->pin));

}




//<------------------------------ PRIVATE FUNCTIONS -------------------------------->// 

void gpioNameSpace::GPIO::setPort(GPIO_TypeDef *GPIOx){

    assert(ASSERT_GPIO_INSTANCE(GPIOx));
    
    if(GPIOx == GPIOA){

        this->parameters->port = GPIO_PORT_A;

    }else if(GPIOx == GPIOB){
        
        this->parameters->port = GPIO_PORT_B;

    }else if(GPIOx == GPIOC){

        this->parameters->port  = GPIO_PORT_C;

    }else if(GPIOx == GPIOD){

        this->parameters->port = GPIO_PORT_D;

    }else if(GPIOx == GPIOE){

        this->parameters->port = GPIO_PORT_E;

    }else{

        this->parameters->port = GPIO_PORT_H;
        
    }
}




/**
 * @brief function that initiates a pin whole configuration ie: mode, AF, otyper, etc
 */
defsNameSpace::TASK_STATUS GPIO::initPin(){


    uint32_t position, regPosition, positionMask, temp;
    uint32_t afLowRegisterMask = 0x7U; /* 0x7 -> 0b111 */
    uint32_t afHighOrLow;
    uint32_t pin = this->parameters->pin;

    for(position = 0; position < NUMBER_OF_GPIO_PORTS; position++){

        regPosition = 1 << position;
        if((regPosition & pin) >> position){

            if((alreadyRegisteredPin & regPosition)){

                /*this pin is already initialized*/
                continue;

            }
            //--------------------- GPIO Mode Configuration ------------------------//
            
            //-------- In case of Output or Alternate function mode selection, the output speed and the output type must to be specified ------//
            positionMask = 2U * position;

            if(((((parameters->pinMode) & (GPIO_MODER_MSK << positionMask)) >> positionMask) == GPIO_OUTPUT_MODE) ||      \
            ((((parameters->pinMode) & (GPIO_MODER_MSK << positionMask)) >> positionMask) == GPIO_ALTERNATE_FUNCTION_MODE))
            {

                temp = instance->OSPEEDR;
                temp &= ~(GPIO_OSPEEDR_MSK << positionMask);
                temp |= ((parameters->outputSpeed) & (GPIO_OSPEEDR_MSK << positionMask));
                instance->OSPEEDR = (uint32_t)temp;

                temp = instance->OTYPER;
                temp &= ~(GPIO_OT_OTYPER_MSK << position);
                temp |= ((parameters->outputType) & (GPIO_OT_OTYPER_MSK << position));
                instance->OTYPER = (uint32_t)temp;

            }

            //----when not using the analog input mode, set the pull up/down configuration to be used----//
            if((((parameters->pinMode) & (GPIO_MODER_MSK << positionMask)) >> positionMask) != GPIO_ANALOG_INPUT_MODE)
            {
                temp = instance->PUPDR;
                temp &= ~(GPIO_PUPDR_MSK << positionMask);
                temp |= ((parameters->pullUpPullDown) & ( GPIO_PUPDR_MSK << positionMask));
                instance->PUPDR = (uint32_t) temp;

            }
            /* In case of alternate function mode selection select which AF is to be used */
            if((((parameters->pinMode) & (GPIO_MODER_MSK << positionMask)) >> positionMask) == GPIO_ALTERNATE_FUNCTION_MODE)
            {
                afHighOrLow = (4U * (position & afLowRegisterMask));
                temp = instance->AFR[position >> 3];
                temp &= ~(GPIO_AF_MSK << afHighOrLow);
                temp |= ((parameters->alternateFunctions[position >> 3]) & (GPIO_AF_MSK << afHighOrLow));
                instance->AFR[position>>3] = (uint32_t)temp;

            }

            /* Configure the IO direction: Input, output, Analog input or AF */
            temp = instance->MODER;
            temp &= ~(GPIO_MODER_MSK << positionMask);
            temp |= parameters->pinMode & (GPIO_MODER_MSK << positionMask);
            instance->MODER = temp;


            /* Configure the external events and interrupts */
            if(((parameters->eventsAndInterrupts) & (EXT_ExI_MSK << positionMask)) >> positionMask)
            {
                uint32_t eventConfig, interruptConfig, risingTriggerConfig, fallingTriggerConfig;

                /* Clear the external event line and load the user confguration */
                eventConfig = ((((parameters->eventsAndInterrupts) & (EXT_ExI_MSK << positionMask)) >> positionMask) & EXTE_ENABLED);
                if(eventConfig){

                    temp = EXTI->EMR;
                    temp &= ~(0x1 << position);
                    temp |= regPosition;
                }
                EXTI->EMR = temp;

                /* Clear the rising trigger config*/
                risingTriggerConfig = ((((parameters->risingAndFallingTrigger) & (ExI_TRIGGER_SELECTION_MSK << positionMask)) >> positionMask) & 0x1);
                if(risingTriggerConfig){

                    temp = EXTI->RTSR;
                    temp &= ~(0x1 << position);
                    temp |= regPosition;
                }
                EXTI->RTSR = temp;

                /* Clear the falling edge trigger config */
                fallingTriggerConfig = ((((parameters->risingAndFallingTrigger) & (ExI_TRIGGER_SELECTION_MSK << positionMask)) >> positionMask) & 0x2);
                if(fallingTriggerConfig){

                    temp = EXTI->FTSR;
                    temp &= ~(0x1 << position);
                    temp |= regPosition;
                }
                EXTI->FTSR = temp;

                /* Clear the external interrupt line and load the user configuration */
                /* Remember that EXT lines are 0 to 15 */
                if(this->extiLinesEnabled & (regPosition))
                {

                    return defsNameSpace::ERROR;

                } 
                interruptConfig = ((((parameters->eventsAndInterrupts) & (EXT_ExI_MSK << positionMask)) >> positionMask) & EXTI_ENABLED);
                if(interruptConfig){

                    /* enable SYSCFG clock */
                    RCC->APB2ENR |= 1 << 14;
                    temp = SYSCFG->EXTICR[position >> 2];
                    temp &= ~(0xF << (4U * (position & 0x3U)));
                    temp |= (uint32_t)(GET_GPIOx_INDEX(this->instance) << (4U * (position & 0x3U)));
                    SYSCFG->EXTICR[position >> 2] = temp;


                    if(position <= 4)
                    {
                        NVIC_EnableIRQ((IRQn_Type)(EXTI0_IRQn + position));
                        NVIC_SetPriority((IRQn_Type)(EXTI0_IRQn + position), ((this->parameters->interruptPriority[0] & defsNameSpace::NVIC_PRIORITY_MSK << (4U * position)) >> (4U * position)));
                    }
                    else if((position > 4) && (position < 10))
                    {
                        NVIC_EnableIRQ(EXTI9_5_IRQn);
                        NVIC_SetPriority(EXTI9_5_IRQn, (this->parameters->interruptPriority[position >> 3] & defsNameSpace::NVIC_PRIORITY_MSK << (4U * (position & 0x3U)) >> (4U * (position & 0x3U))));
                    }
                    else
                    {
                        NVIC_EnableIRQ(EXTI15_10_IRQn);
                        NVIC_SetPriority(EXTI15_10_IRQn, (this->parameters->interruptPriority[1] & defsNameSpace::NVIC_PRIORITY_MSK << (4U * (position & 0x3U)) >> (4U * (position & 0x3U))));
                    }

                    temp = EXTI->IMR;
                    temp &= ~(0x1 << position);
                    temp |= regPosition;
                }
                EXTI->IMR = temp;
                this->extiLinesEnabled |= regPosition;
            }
            /* add the current pin to the already registerd pins */
            alreadyRegisteredPin |= regPosition;

            uint32_t allocatedResourcesAddress = (uint32_t)&allocatedResources;
            allocatedResourcesAddress = allocatedResourcesAddress + (GET_GPIOx_INDEX(instance)*sizeof(uint16_t));
            uint16_t *allocatedResourcesPtr = (uint16_t*)allocatedResourcesAddress;
            uint16_t auxPin = regPosition;
            *allocatedResourcesPtr |= auxPin; 
        }
    }
    return defsNameSpace::OK;
}

defsNameSpace::TASK_STATUS GPIO::deInitPin(){

    //1001 0010 AR
    //0001 0000 newpin
    //1000 0010 XOR
    uint32_t eliminatedPin = alreadyRegisteredPin ^ this->parameters->pin;
    uint32_t position, regPosition;

    for(position = 0; position < NUMBER_OF_GPIO_PORTS; position++){

        /* Unregister the corrresponding IRQn from the NVIC service */
        regPosition = 1 << position;
        if((regPosition & eliminatedPin) >> position){

            
            if((extiLinesEnabled & regPosition) && ((parameters->eventsAndInterrupts & (EXT_ExI_MSK << (2 * position)) >> (2 * position)) == EXTI_ENABLED)){

                if(position <= 4){

                    NVIC_DisableIRQ((IRQn_Type)(EXTI0_IRQn + position));
                }
                else if((position >= 5) &&(position <= 9)){

                    NVIC_DisableIRQ(EXTI9_5_IRQn);

                }else if((position >= 10) & (position <= 15)){

                    NVIC_DisableIRQ(EXTI15_10_IRQn);
                }

                EXTI->RTSR &= ~(1 << position);
                EXTI->FTSR &= ~(1 << position);

                SYSCFG->EXTICR[position >> 2] &= ~(0xF << (4U * (position & 0x3U)));
        
                EXTI->IMR &= ~(1 << position);
                EXTI->EMR &= ~(1 << position);
                extiLinesEnabled &= ~(1 << position);

            }

            instance->MODER &= ~(GPIO_MODER_MSK << (2U * position));
            instance->MODER |= (resetValues.MODER & (GPIO_MODER_MSK << (2U * position)));

            instance->AFR[position >> 3] &= ~(GPIO_AF_MSK << (4U * (position & 0x7)));
            instance->AFR[position >> 3] |= (resetValues.AFR[position >> 3] & (GPIO_AF_MSK << (4U * (position & 0x7))));

            instance->PUPDR &= ~(GPIO_PUPDR_MSK << (2U * position));
            instance->PUPDR |= (resetValues.PUPDR & (GPIO_MODER_MSK << (2U * position)));

            instance->OSPEEDR &= ~(GPIO_OSPEEDR_MSK << (2U * position));
            instance->OSPEEDR |= (resetValues.PUPDR & (GPIO_OSPEEDR_MSK << (2U * position)));

            instance->OTYPER &= ~(GPIO_OT_OTYPER_MSK << position);
            instance->OTYPER |= (resetValues.OTYPER & (GPIO_OT_OTYPER_MSK << position));

            alreadyRegisteredPin &= ~(1 << position);

            this->parameters->pin = alreadyRegisteredPin;
            this->parameters->pinMode &= ~(GPIO_MODER_MSK << (2U * position));
            this->parameters->alternateFunctions[position >> 3] &= ~(GPIO_AF_MSK << (4U * (position & 0x7)));
            this->parameters->pullUpPullDown &= ~(GPIO_PUPDR_MSK << (2U * position));
            this->parameters->outputSpeed &= ~(GPIO_OSPEEDR_MSK << (2U * position));
            this->parameters->outputType &= ~(GPIO_OT_OTYPER_MSK << position);
        }
    }
    return defsNameSpace::OK;
}



/* Hardware config and manipulation */
void gpioNameSpace::GPIO::peripheralClockEnable(){

    RCC->AHB1ENR |= 1 << GET_GPIOx_INDEX(this->instance);

}





void GPIO::enableNVIC_Interrupt(){

    enableNVIC_Interrupt(this->parameters->pin);

}

void GPIO::disableNVIC_Interrupt(){

    disableNVIC_Interrupt(this->parameters->pin);
}

void GPIO::enableNVIC_Interrupt(uint32_t pin){

    uint32_t position, ioposition;

    for(position = 0; position < NUMBER_OF_GPIO_PORTS; position++){

        ioposition = 1 << position;
        if(pin & ioposition){

            if(position <= 4){

                NVIC_EnableIRQ((IRQn_Type)(EXTI0_IRQn + position));
            }
            else if((position >= 5) &&(position <= 9)){

                NVIC_EnableIRQ(EXTI9_5_IRQn);

            }else if((position >= 10) & (position <= 15)){

                NVIC_EnableIRQ(EXTI15_10_IRQn);
            }
        }
    }
}

void GPIO::disableNVIC_Interrupt(uint32_t pin){

    uint32_t position, ioposition;

    for(position = 0; position < NUMBER_OF_GPIO_PORTS; position++){

        ioposition = 1 << position;
        if(pin & ioposition){

            if(position <= 4){

                NVIC_DisableIRQ((IRQn_Type)(EXTI0_IRQn + position));
            }
            else if((position >= 5) &&(position <= 9)){

                NVIC_DisableIRQ(EXTI9_5_IRQn);

            }else if((position >= 10) & (position <= 15)){

                NVIC_DisableIRQ(EXTI15_10_IRQn);
            }
        }
    }
}



void GPIO::setParameter(uint32_t param, uint32_t paramClrMsk, uint32_t paramVal){

    uint32_t *ptr = (uint32_t*)parameters;
    uint32_t tempVal;

    ptr = ptr + param;
    tempVal = *ptr; 
    tempVal &= ~paramClrMsk;
    tempVal |= paramVal;
    *ptr = tempVal;

}
void GPIO::setParameterHighRegisters(uint32_t param, uint32_t paramClrMsk, uint32_t paramVal, uint32_t regLevel){

    param = param + regLevel;
    setParameter(param,paramClrMsk,paramVal);
}
void GPIO::setParametersLoop(uint32_t param, uint32_t paramClrMsk, uint32_t paramVal, uint32_t pos){

    uint32_t regPosition;
    uint32_t position;
    uint32_t registerLength = 0;
    uint32_t remainingBits;
    uint32_t tempVal = 0;
    uint32_t tmpMsk = 0;

    registerLength = getRegisterBitLength(paramClrMsk);

    /* sets the mask to cero */
    alignMaskToZero(paramClrMsk, remainingBits);


    for(position = 0; position < REGISTER_BITS_LENGTH; position++){

        regPosition = 1 << position;
        if((regPosition & pos & this->parameters->pin) >> position){

            tmpMsk |= paramClrMsk << (registerLength*position);
            tempVal |= paramVal << (registerLength*position);
        }


    }
    setParameter(param,tmpMsk,tempVal);

}
void GPIO::setParametersHighRegistersLoop(uint32_t param, uint32_t paramClrMsk, uint32_t paramVal, uint32_t position, uint32_t regLevel){

    setParametersLoop((param+regLevel), paramClrMsk, paramVal, position);

}
uint32_t GPIO::getRegisterBitLength(uint32_t mask){

    uint32_t tempMask = mask;
    uint32_t remainingBits;
    uint32_t position;
    uint32_t bitLength = 0;

    /* alig mask to zero */
    alignMaskToZero(tempMask, remainingBits);

    /* this determines each parameter bit usage in terms of its mask */
    for(position = 0; position < remainingBits; position++){

        if(tempMask == 1){
            bitLength++;
            break;
            
        }else{

            bitLength++;
        }
        tempMask = tempMask >>  1;
    }
    return bitLength;

}
void GPIO::alignMaskToZero(uint32_t &mask, uint32_t &remainingBits){

    uint32_t position;
    uint32_t regPosition;
    uint32_t tmpPosition;
    uint32_t tmpMsk;

    for(position = 0; position < REGISTER_BITS_LENGTH; position++)
    {
        regPosition = 1 << position;
        if((regPosition & mask)){

            tmpMsk = mask >> position;
            tmpPosition = position;
            break;
        }
    }
    remainingBits = REGISTER_BITS_LENGTH - tmpPosition;
    mask = tmpMsk;

}


/* <-----------------------------------------------------------------------------------------------------------------------------------------------------------------> */
/* <-------------------------------------------------------------------- IRQ handlers and callbacks -------------------------------------------------------------------> */
/* <-----------------------------------------------------------------------------------------------------------------------------------------------------------------> */
__attribute__((weak)) void EXTI0_Callback(){

    __NOP();

}

__attribute__((weak)) void EXTI1_Callback(){

    __NOP();

}
__attribute__((weak)) void EXTI2_Callback(){

    __NOP();

}
__attribute__((weak)) void EXTI3_Callback(){

    __NOP();

}
__attribute__((weak)) void EXTI4_Callback(){

    __NOP();

}
__attribute__((weak)) void EXTI5_Callback(){

    __NOP();

}
__attribute__((weak)) void EXTI6_Callback(){

    __NOP();

}
__attribute__((weak)) void EXTI7_Callback(){

    __NOP();

}
__attribute__((weak)) void EXTI8_Callback(){

    __NOP();

}
 __attribute__((weak)) void EXTI9_Callback(void){

     __NOP();

}
__attribute__((weak)) void EXTI10_Callback(){

    __NOP();

}
__attribute__((weak)) void EXTI11_Callback(){

    __NOP();

}
__attribute__((weak)) void EXTI12_Callback(){

    __NOP();

}
__attribute__((weak)) void EXTI13_Callback(){

    __NOP();

}
__attribute__((weak)) void EXTI14_Callback(){

    __NOP();

}
__attribute__((weak)) void EXTI15_Callback(){

    __NOP();

}
void EXTI0_IRQHandler(){

    if((GPIO::getEXTIEnabledLines() & GPIO_PIN_0) & EXTI->PR){

        EXTI->PR = GPIO_PIN_0;
        EXTI0_Callback();

    }
}

void EXTI1_IRQHandler(){

    if((GPIO::getEXTIEnabledLines() & GPIO_PIN_1) & EXTI->PR){

        EXTI->PR = GPIO_PIN_1;
        EXTI1_Callback();

    }
}


void EXTI2_IRQHandler(){

    if((GPIO::getEXTIEnabledLines() & GPIO_PIN_2) & EXTI->PR){

        EXTI->PR = GPIO_PIN_2;
        EXTI2_Callback();

    }
}


void EXTI3_IRQHandler(){

    if((GPIO::getEXTIEnabledLines() & GPIO_PIN_3) & EXTI->PR){

        EXTI->PR = GPIO_PIN_3;
        EXTI3_Callback();

    }
}


void EXTI4_IRQHandler(){

    if((GPIO::getEXTIEnabledLines() & GPIO_PIN_4) & EXTI->PR){

        EXTI->PR = GPIO_PIN_4;
        EXTI4_Callback();

    }
}


void EXTI9_5_IRQHandler(){

    if((GPIO::getEXTIEnabledLines() & GPIO_PIN_5) & EXTI->PR){

        EXTI->PR = GPIO_PIN_5;
        EXTI15_Callback();

    }
    else if((GPIO::getEXTIEnabledLines() & GPIO_PIN_6) & EXTI->PR){

        EXTI->PR = GPIO_PIN_6;
        EXTI6_Callback();

    }
    else if((GPIO::getEXTIEnabledLines() & GPIO_PIN_7) & EXTI->PR){

        EXTI->PR = GPIO_PIN_7;
        EXTI7_Callback();

    }
    else if((GPIO::getEXTIEnabledLines() & GPIO_PIN_8) & EXTI->PR){

        EXTI->PR = GPIO_PIN_8;
        EXTI8_Callback();

    }
    else if((GPIO::getEXTIEnabledLines() & GPIO_PIN_9) & EXTI->PR){

        EXTI->PR = GPIO_PIN_9;
        EXTI9_Callback();

    }
}

void EXTI15_10_IRQHandler(){

    if((GPIO::getEXTIEnabledLines() & GPIO_PIN_10) & EXTI->PR){

        EXTI->PR = GPIO_PIN_10;
        EXTI10_Callback();

    }
    else if((GPIO::getEXTIEnabledLines() & GPIO_PIN_11) & EXTI->PR){

        EXTI->PR = GPIO_PIN_11;
        EXTI11_Callback();

    }
    else if((GPIO::getEXTIEnabledLines() & GPIO_PIN_12) & EXTI->PR){

        EXTI->PR = GPIO_PIN_12;
        EXTI12_Callback();

    }
    else if((GPIO::getEXTIEnabledLines() & GPIO_PIN_13) & EXTI->PR){

        EXTI->PR = GPIO_PIN_13;
        EXTI13_Callback();

    }
    else if((GPIO::getEXTIEnabledLines() & GPIO_PIN_14) & EXTI->PR){

        EXTI->PR = GPIO_PIN_14;
        EXTI14_Callback();

    }
    else if((GPIO::getEXTIEnabledLines() & GPIO_PIN_15) & EXTI->PR){

        EXTI->PR = GPIO_PIN_15;
        EXTI15_Callback();

    }
}
/* ADD PIN EXPLAINED */ 
/**
 * first case: no pin has already been initialized so "alreadyRegisterePin" = 0 also parametr.pin = 0 
 * 
 *      00000000    alreadyRegisteredPin: pin already registered
 *      10110010    pin: current pin being added
 *      -----------------------------------------------------------
 *      10110010    tmp(no var) XOR
 * 
 *      10110010    tmp
 *      10110010    pin
 *      ----------------------------------------------------------- 
 *      10110010    pinValueMinusAlreadyRegistered AND
 * 
 *      10110010    pinValueMinusAlreadyRegistered
 *      00000000    alreayRegisteredPin
 *      -----------------------------------------------------------
 *      10110010    tmp(no var) OR
 * 
 *      00000000    parameters.pin
 *      10110010    tmp
 *      ----------------------------------------------------------  
 *      10110010    parameters.pin OR
 * 
 * second case: no pin has already been initialized however theres a pin in the staggin area "alreadyRegisteredPin" = 0 but parameter.pin != 0
 * 
 *      00000000    alreadyRegisteredPin: pin already registered
 *      10110010    pin: current pin being added 
 *      --------------------------------------------------------                             
 *      10110010    tmp(no var) XOR                                 
 *                                                            
 *      10110010    tmp                 
 *      10110010    pin                             
 *      --------------------------------------------------------                             
 *      10110010    pinValueMinusAlreadyRegistered AND                             
 * 
 *      10110010    pinValueMinusAlreadyRegistered                             
 *      00000000    alreayRegisteredPin                             
 *      --------------------------------------------------------                             
 *      10110010    tmp(no var) OR
 *                                   
 *      11100010    parameters.pin(supossedly staggin area)                             
*      10110010    tmp                             
*      --------------------------------------------------------
*      11110010    parameters.pin OR(currrent pin + staging area pin)
*                                  
*      staggin area is the difference among alreadyregisteredpin and parameter.pin
* third case: certain pin has just been initialized so parameters.pin = alreadyRegisterdPin (staggin area empty). 
*             Try to add pin not yet registered as well as already registered ones
* 
*      1001 0010    alreadyRegisteredPin: pin already registered
*      1110 0010    pin: current pin being added 
*      --------------------------------------------------------                             
*      0111 0000    tmp(no var) XOR                                 
*                                                            
*      0111 0000    tmp                 
*      1110 0010    pin                             
*      --------------------------------------------------------                             
*      0110 0000    pinValueMinusAlreadyRegistered AND                             
* 
*      0110 0000    pinValueMinusAlreadyRegistered                             
*      1001 0010    alreayRegisteredPin                             
*      --------------------------------------------------------                             
*      1111 0010    tmp(no var) OR
*                                   
*      1001 0010    parameters.pin(supossedly staggin area is equal to alreadyregistered)                             
*      1111 0010    tmp                             
*      --------------------------------------------------------
*      1111 0010    parameters.pin OR(currrent pin + staging area pin)
*                              
*/


/* REMOVE PIN EXPLAINED */
/**
     * 
     * first case: no pin is in the staggin area, so alreadyRegisteredPin =  parameters.pin
     * 
     * 
     *      check if the current pin is already registered
     * 
     *      1000 1001    alreadyRegisteredPin
     *      0110 1001    current pin to be removed
     * -----------------------------------------------------------------------------    
     *      0000 1001    tmp(no var) AND(to check if the current pin is registered)  
     *      
     *      1111 0110   ~tmp 
     *      1000 1001   alreadyRegisteredPin
     * -------------------------------------------------------------------------------
     *      1000 0000   parameters.pin (AND)(this is the new pin value minus the pins to be removed. In order to update alreadyRegisterdPin's value, a deinit funtion must be called) 
     *      *launchs deinit pin *
     * 
     * second case: some pins are on the staggin area and some others are already initialized.
     * 
     *      1011 1101   parameters.pin: staggin area
     * 
     *      1000 0101   alreadyRegisteredPin
     *      1101 1100   pin: current pin to be removed.
     * -----------------------------------------------------------------------------------  
     *      1000 0100   tmp(no var) AND ( to check if the current pin is registerd)
     * 
     *      0111 1011   ~tmp
     *      1000 0101   alreadyRegisterdPin.
     * -----------------------------------------------------------------------------------
     *      0000 0001   parameters.pin(AND) new desired pin value after removing currentPin
     *      afeter launching deInit, parameters.pin = alreadyRegisterdPin so no values are on the staggin area.
     * 
     */
    
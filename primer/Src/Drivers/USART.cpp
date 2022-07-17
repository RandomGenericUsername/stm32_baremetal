#include "USART.hh"

using namespace usartNameSpace;

char dataRegister;
struct USART_PARAMETERS_STRUCT defaultSettings = 
{   
    .mode = USART_TXRX_MODE,
    .baudRate = _115200_Bauds,
    .dataLength = _8_DataBits,
    .paritiy = parityCheckDisabled,
    .stopBits = _1_stopBits,
    .interrupts = {
        .parityInterrupt = parityinterruptdisabled,
        .TXE_Interrupt = TXEinterruptdisabled,
        .TXC_interrupt = TXCinterruptdisabled,
        .RXNE_interrupt = RXNEinterruptenabled,
        .idleInterrupt = idleInterruptdisabled,
    },
    .interruptsPriority = defsNameSpace::NVIC_INTERRUPT_PRIORITY::NVIC_PRIORITY_3,
    .internalClockFrequency = 16000000,
    .oversampling = oversamplingBy16,
    .bufferLength = 128,
    .terminationCharacter = '@',
};

USART::USART(USART_TypeDef *instance):TX_pin(GPIOA),RX_pin(GPIOA){

    /* assert the usart instance */
    assert(ASSERT_USART_INSTANCE(instance));
    this->instance = instance;
    resetValues = *instance;
    settings = new USART_PARAMETERS_STRUCT;
    *settings = defaultSettings;
    rxBuffer = (char*)calloc(this->settings->bufferLength, sizeof(char));
    rxCount = 0;
    txBuffer = (char*)calloc(this->settings->bufferLength, sizeof(char));
    txCount = 0;
    rxComplete = true;
    txComplete = true;
    buffer = '\0';
}

USART::~USART(){


}


defsNameSpace::TASK_STATUS USART::init(){

    /* assert each of the params struct members */ 
    assert(ASSERT_USART_INSTANCE(this->instance));
    /* assert all the params */
    /* ----------------------- */
    gpioConfig();
    peripheralClockEnable();
    instance->CR1 = 0;
    instance->CR2 = 0;
    USART_disable();
    setStopBits(settings->stopBits);
    setOverSampling(this->settings->oversampling);
    setDataLength(this->settings->dataLength);
    setParityConfig(settings->paritiy);
    setBRRConfig(baudRate);
    set_TX_RX_mode(this->settings->mode);
    setInterrupts(this->settings->interrupts.RXNE_interrupt, this->settings->interrupts.TXE_Interrupt, settings->interrupts.TXC_interrupt);
    this->instance->SR = 0;
    enableNVIC_Interrupt();
    USART_enable();
    return defsNameSpace::OK;  

}

defsNameSpace::TASK_STATUS USART::deInit()
{
  return defsNameSpace::OK;  
}
void USART::peripheralClockEnable()
{

    if(instance == USART1){

        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    }else if(instance == USART6){

        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

    }else {
        
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    }

}

void USART::setStopBits(USART_STOP_BITS stopBits)
{
    this->settings->stopBits = stopBits;
    MODIFY_REG(instance->CR2, USART_CR2_STOP_Msk, stopBits);
}
void USART::setParityConfig(USART_PARITY_SELECTION parity)
{
    settings->paritiy = parity;
    if(settings->paritiy != parityCheckDisabled)
    {
        MODIFY_REG(instance->CR1, USART_CR1_PCE_Msk, USART_CR1_PCE);
        MODIFY_REG(instance->CR1, USART_CR1_PS_Msk, settings->paritiy);
    }
    else{
        instance->CR1 &= ~ USART_CR1_PS_Msk;
        instance->CR1 &= ~ USART_CR1_PCE_Msk;
    }
}
void USART::setBRRConfig(BAUD_RATE_VALUES brr)
{
    this->baudRate = brr;
    calculateMantisaAndFractionalPart(settings->internalClockFrequency, (settings->oversampling >> USART_CR1_OVER8_Pos), settings->baudRate, baudRate);
    instance->BRR = 0;
    instance->BRR |= baudRate.mantisa << USART_BRR_DIV_Mantissa_Pos;
    instance->BRR |= baudRate.fractionalPart << USART_BRR_DIV_Fraction_Pos;
}

void USART::setInterrupts(USART_RXNE_INTERRUPT_ENABLE rxneie = RXNEinterruptdisabled, USART_TXE_INTERRUPT_ENABLE txeie = TXEinterruptdisabled, USART_TXC_INTERRUPT_ENABLE tcie = TXCinterruptdisabled)
{
    settings->interrupts.RXNE_interrupt = rxneie;
    settings->interrupts.TXE_Interrupt = txeie;
    settings->interrupts.TXC_interrupt = tcie;

    instance->CR1 &= ~USART_CR1_RXNEIE_Msk;
    if(settings->interrupts.RXNE_interrupt)
    {
        instance->CR1 |= USART_CR1_RXNEIE;
    }


    instance->CR1 &= ~USART_CR1_TCIE_Msk;
    if(settings->interrupts.TXC_interrupt)
    {
        instance->CR1 |= USART_CR1_TCIE;
    }

    instance->CR1 &= ~USART_CR1_TXEIE_Msk;
    if(settings->interrupts.TXE_Interrupt)
    {
        instance->CR1 |= USART_CR1_TXEIE;
    }
}
void USART::setOverSampling(USART_OVERSAMPLING_MODE oversampling)
{
    settings->oversampling = oversampling;
    MODIFY_REG(instance->CR1, USART_CR1_OVER8_Msk, settings->oversampling);
}

void USART::setDataLength(USART_WORD_LENGTH dataLength)
{
    settings->dataLength = dataLength;
    MODIFY_REG(instance->CR1, USART_CR1_M_Msk, settings->dataLength);
}

void USART::set_TX_RX_mode(USART_MODE mode)
{
    settings->mode = mode;
    if(settings->mode == USART_TX_MODE || settings->mode == USART_TXRX_MODE)
    {
        MODIFY_REG(instance->CR1, USART_CR1_TE_Msk, USART_CR1_TE);
    }
    if(settings->mode == USART_RX_MODE || settings->mode == USART_TXRX_MODE)
    {
        MODIFY_REG(instance->CR1, USART_CR1_RE_Msk, USART_CR1_RE);
    }
}

void USART::gpioConfig()
{
    if(instance == USART1)
    {
        if(!gpioNameSpace::GPIO::isResourceAllocated(GPIOA,gpioNameSpace::GPIO_PIN_9) && \
        !gpioNameSpace::GPIO::isResourceAllocated(GPIOA,gpioNameSpace::GPIO_PIN_10))
        {

            gpioNameSpace::GPIO TX_pin(GPIOA);
            gpioNameSpace::GPIO RX_pin(GPIOA);
            TX_pin.setPin(gpioNameSpace::GPIO_PIN_9);
            RX_pin.setPin(gpioNameSpace::GPIO_PIN_10);
            TX_pin.setModer(gpioNameSpace::GPIO_ALTERNATE_FUNCTION_MODE);
            RX_pin.setModer(gpioNameSpace::GPIO_ALTERNATE_FUNCTION_MODE);
            TX_pin.setAF(gpioNameSpace::GPIO_AF_7);
            RX_pin.setAF(gpioNameSpace::GPIO_AF_7);
            TX_pin.init();
            RX_pin.init();
            this->TX_pin = TX_pin;
            this->RX_pin = RX_pin;
        }
        else if(!gpioNameSpace::GPIO::isResourceAllocated(GPIOB,gpioNameSpace::GPIO_PIN_6) && \
        !gpioNameSpace::GPIO::isResourceAllocated(GPIOB,gpioNameSpace::GPIO_PIN_7))
        {
            gpioNameSpace::GPIO TX_pin(GPIOB);
            gpioNameSpace::GPIO RX_pin(GPIOB);
            TX_pin.setPin(gpioNameSpace::GPIO_PIN_6);
            RX_pin.setPin(gpioNameSpace::GPIO_PIN_7);
            TX_pin.setModer(gpioNameSpace::GPIO_ALTERNATE_FUNCTION_MODE);
            RX_pin.setModer(gpioNameSpace::GPIO_ALTERNATE_FUNCTION_MODE);
            TX_pin.setAF(gpioNameSpace::GPIO_AF_7);
            RX_pin.setAF(gpioNameSpace::GPIO_AF_7);
            TX_pin.init();
            RX_pin.init();
            this->TX_pin = TX_pin;
            this->RX_pin = RX_pin;
        }
        else if(!gpioNameSpace::GPIO::isResourceAllocated(GPIOA,gpioNameSpace::GPIO_PIN_15) && \
        !gpioNameSpace::GPIO::isResourceAllocated(GPIOB,gpioNameSpace::GPIO_PIN_3))
        {
            gpioNameSpace::GPIO TX_pin(GPIOA);
            gpioNameSpace::GPIO RX_pin(GPIOB);
            TX_pin.setPin(gpioNameSpace::GPIO_PIN_15);
            RX_pin.setPin(gpioNameSpace::GPIO_PIN_3);
            TX_pin.setModer(gpioNameSpace::GPIO_ALTERNATE_FUNCTION_MODE);
            RX_pin.setModer(gpioNameSpace::GPIO_ALTERNATE_FUNCTION_MODE);
            TX_pin.setAF(gpioNameSpace::GPIO_AF_7);
            RX_pin.setAF(gpioNameSpace::GPIO_AF_7);
            TX_pin.init();
            RX_pin.init();
            this->TX_pin = TX_pin;
            this->RX_pin = RX_pin;

        }
    }
    else if(instance == USART2)
    {
        if(!gpioNameSpace::GPIO::isResourceAllocated(GPIOA,gpioNameSpace::GPIO_PIN_2) && \
        !gpioNameSpace::GPIO::isResourceAllocated(GPIOA,gpioNameSpace::GPIO_PIN_3))
        {
            gpioNameSpace::GPIO TX_pin(GPIOA);
            gpioNameSpace::GPIO RX_pin(GPIOA);
            TX_pin.setPin(gpioNameSpace::GPIO_PIN_2);
            RX_pin.setPin(gpioNameSpace::GPIO_PIN_3);
            TX_pin.setModer(gpioNameSpace::GPIO_ALTERNATE_FUNCTION_MODE);
            RX_pin.setModer(gpioNameSpace::GPIO_ALTERNATE_FUNCTION_MODE);
            TX_pin.setAF(gpioNameSpace::GPIO_AF_7);
            RX_pin.setAF(gpioNameSpace::GPIO_AF_7);
            TX_pin.init();
            RX_pin.init();
            this->TX_pin = TX_pin;
            this->RX_pin = RX_pin;
        }
        else if(!gpioNameSpace::GPIO::isResourceAllocated(GPIOD,gpioNameSpace::GPIO_PIN_5) && \
        !gpioNameSpace::GPIO::isResourceAllocated(GPIOD,gpioNameSpace::GPIO_PIN_6))
        {
            gpioNameSpace::GPIO TX_pin(GPIOD);
            gpioNameSpace::GPIO RX_pin(GPIOD);
            TX_pin.setPin(gpioNameSpace::GPIO_PIN_5);
            RX_pin.setPin(gpioNameSpace::GPIO_PIN_6);
            TX_pin.setModer(gpioNameSpace::GPIO_ALTERNATE_FUNCTION_MODE);
            RX_pin.setModer(gpioNameSpace::GPIO_ALTERNATE_FUNCTION_MODE);
            TX_pin.setAF(gpioNameSpace::GPIO_AF_7);
            RX_pin.setAF(gpioNameSpace::GPIO_AF_7);
            TX_pin.init();
            RX_pin.init();
            this->TX_pin = TX_pin;
            this->RX_pin = RX_pin;
        }
    }
    else
    {
        if(!gpioNameSpace::GPIO::isResourceAllocated(GPIOC,gpioNameSpace::GPIO_PIN_6) && \
        !gpioNameSpace::GPIO::isResourceAllocated(GPIOC,gpioNameSpace::GPIO_PIN_7))
        {
            gpioNameSpace::GPIO TX_pin(GPIOC);
            gpioNameSpace::GPIO RX_pin(GPIOC);
            TX_pin.setPin(gpioNameSpace::GPIO_PIN_6);
            RX_pin.setPin(gpioNameSpace::GPIO_PIN_7);
            TX_pin.setModer(gpioNameSpace::GPIO_ALTERNATE_FUNCTION_MODE);
            RX_pin.setModer(gpioNameSpace::GPIO_ALTERNATE_FUNCTION_MODE);
            TX_pin.setAF(gpioNameSpace::GPIO_AF_8);
            RX_pin.setAF(gpioNameSpace::GPIO_AF_8);
            TX_pin.init();
            RX_pin.init();
            this->TX_pin = TX_pin;
            this->RX_pin = RX_pin;
        }
        else if(!gpioNameSpace::GPIO::isResourceAllocated(GPIOA,gpioNameSpace::GPIO_PIN_11) && \
        !gpioNameSpace::GPIO::isResourceAllocated(GPIOA,gpioNameSpace::GPIO_PIN_12))
        {
            gpioNameSpace::GPIO TX_pin(GPIOA);
            gpioNameSpace::GPIO RX_pin(GPIOA);
            TX_pin.setPin(gpioNameSpace::GPIO_PIN_11);
            RX_pin.setPin(gpioNameSpace::GPIO_PIN_12);
            TX_pin.setModer(gpioNameSpace::GPIO_ALTERNATE_FUNCTION_MODE);
            RX_pin.setModer(gpioNameSpace::GPIO_ALTERNATE_FUNCTION_MODE);
            TX_pin.setAF(gpioNameSpace::GPIO_AF_7);
            RX_pin.setAF(gpioNameSpace::GPIO_AF_7);
            //TX_pin.init();
            //RX_pin.init();
            this->TX_pin = TX_pin;
            this->RX_pin = RX_pin;
        }
    }
}
void USART::enableNVIC_Interrupt()
{
    IRQn_Type irq;
    if(settings->interrupts.idleInterrupt == idleInterruptenabled     || \
    settings->interrupts.parityInterrupt == parityinterruptenabled    || \
    settings->interrupts.RXNE_interrupt == RXNEinterruptenabled       || \
    settings->interrupts.TXC_interrupt == TXCinterruptenabled         || \
    settings->interrupts.TXE_Interrupt == TXEinterruptenabled)
    {
        if(instance == USART1)
        {
           irq = USART1_IRQn;
        }
        else if(instance == USART2) 
        {
           irq = USART2_IRQn;
        }
        else if(instance == USART6)
        {
           irq = USART6_IRQn;
        }
        NVIC_EnableIRQ((IRQn_Type)irq);
        NVIC_SetPriority(irq, settings->interruptsPriority);
    }
}

void USART::disableNVIC_Interrupt()
{
        IRQn_Type irq;
        if(instance == USART1)
        {
           irq = USART1_IRQn;
        }
        else if(instance == USART2) 
        {
           irq = USART2_IRQn;
        }
        else if(instance == USART6)
        {
           irq = USART6_IRQn;
        }
        NVIC_DisableIRQ((IRQn_Type)irq);
}

void USART::calculateMantisaAndFractionalPart(double internalClockFrequency, uint8_t OVER8, USART_BAUD_RATES desiredBaudRate, BAUD_RATE_VALUES &mantisaAndFractionalPart)
{
    uint32_t overSamplingProduct = 8*(2-OVER8);
    double rawValue = internalClockFrequency / (double)(overSamplingProduct*desiredBaudRate); 
    uint32_t mantisa = (uint32_t) rawValue; 
    uint32_t fractionalPart = (overSamplingProduct*(rawValue - mantisa) + 1);

    /* check for the overflow of the fractional part */
    uint32_t overflowCheck = fractionalPart & (~(overSamplingProduct - 1));
    if(overflowCheck)
    {
        uint32_t lowBitsBRR = overflowCheck & (overSamplingProduct - 1);
        uint32_t numberOfOverflows = overflowCheck / overSamplingProduct;
        mantisa +=  1U * numberOfOverflows;
        fractionalPart = lowBitsBRR;
    }
    /* calculate the actual baud rate with error */
    uint32_t actualBaudRate_ = internalClockFrequency / (double) (overSamplingProduct * (mantisa + ((double) fractionalPart / overSamplingProduct ))) + 1;
    mantisaAndFractionalPart.mantisa = mantisa;
    mantisaAndFractionalPart.fractionalPart = fractionalPart;
    this->baudRate.actualBaudRate = actualBaudRate_;
    this->baudRate.baudRateErrorPercentage = (1 - (this->baudRate.actualBaudRate / desiredBaudRate)) * 100;
    
}

void USART::USART_enable()
{
    enableNVIC_Interrupt();
    instance->CR1 |= USART_CR1_UE;
}
void USART::USART_disable()
{
    disableNVIC_Interrupt();
    instance->CR1 &= ~USART_CR1_UE_Msk;
}



/* blocking mode functions */
void USART::writeCharBlockingMode(char character)
{
    while(!(instance->SR & USART_SR_TXE))
    {
		__NOP();
	}
	instance->DR = character;

}
void USART::writeStringBlockingMode(char *string)
{
    while(*string != '\0')
    {
        writeCharBlockingMode(*string);
        string++;
    }
}
char USART::readCharBlockingMode(void)
{
    return dataRegister;
}


/* non blocking mode functions */
void USART::sendCharNonBlockingMode(char character)
{

    if(this->rxComplete != true)
    {
        return;
    }
    this->txComplete = false;
    this->buffer = character;
    setInterrupts(RXNEinterruptdisabled, TXEinterruptenabled, TXCinterruptdisabled);
}
void USART::writeStringNonBlockingMode(char *string)
{
    this->txComplete = false;
    this->txBuffer = string;
    setInterrupts(RXNEinterruptdisabled, TXEinterruptenabled, TXCinterruptdisabled);
    while(*txBuffer != '\0')
    {

        sendCharNonBlockingMode(*string);
        txBuffer++;
    }

}

void USART::writeAux()
{
    setInterrupts(RXNEinterruptenabled, TXEinterruptdisabled, TXCinterruptdisabled);
    this->instance->DR = this->buffer;
}

char USART::readCharNonBlockingMode(void)
{
    return dataRegister;
}

void USART::readUserCommands(USART &handler)
{
    char currentRead = handler.buffer;
    if(currentRead != '\0')
    {
        if(this->txComplete != true)
        {
            return;
        }
        this->rxComplete = false;
        handler.rxBuffer[handler.rxCount++] = currentRead;
        if(currentRead == '@')
        {
            handler.rxBuffer[handler.rxCount - 1] = '\0';
            rxComplete = true;
            handler.rxCount = 0;
            handler.parseUserCommands(handler.paramsPtr, handler.paramsFormat);
        }
        handler.buffer = '\0';
    }
}

void USART::parseUserCommands(void *paramsPtr, char *paramsFormat)
{
    void *paramsAddress = new (void*);
    char *bufferAddress = new (char);

    paramsAddress = paramsPtr;
    bufferAddress = this->rxBuffer;

    while(*paramsFormat != '\0')
    {
        if(*paramsFormat == 's')
        {
            char *asf;
            sscanf(bufferAddress,"%s", paramsAddress);
            asf = (char*)paramsAddress;
            paramsAddress = paramsAddress + ((64*sizeof(char)));
            bufferAddress += strlen(asf)*sizeof(char) + 1;
        }
        else if(*paramsFormat == 'u')
        {
            sscanf(bufferAddress,"%u", paramsAddress);
            paramsAddress = paramsAddress + ((sizeof(unsigned int)));
            bufferAddress += sizeof(unsigned int) + 1;
                        
        }
        paramsFormat++;
    }


}




__attribute__((weak))void USART1_RX_Callback(void)
{
    __NOP();
}

__attribute__((weak))void USART2_RX_Callback(void)
{
    __NOP();
}

__attribute__((weak))void USART6_RX_Callback(void)
{
    __NOP();
}

__attribute__((weak))void USART1_TX_Callback(void)
{
    __NOP();
}

__attribute__((weak))void USART2_TX_Callback(void)
{
    __NOP();
}

__attribute__((weak))void USART6_TX_Callback(void)
{
    __NOP();
}
void USART1_IRQHandler (void)
{
    uint32_t temp = USART1->SR;
    if((temp & USART_SR_RXNE) == USART_SR_RXNE)
    {
        dataRegister = USART1->DR;
        USART1_RX_Callback();
    }
    else if((temp & USART_SR_TC) == USART_SR_TC)
    {
    }
    else if(temp & USART_SR_TXE)
    {
       USART1_TX_Callback(); 
    }

}

void USART2_IRQHandler (void)
{
    uint32_t temp = USART2->SR;
    if((temp & USART_SR_RXNE))
    {
        dataRegister = USART2->DR;
        USART2_RX_Callback();
    }
    else if(temp & USART_SR_TXE)
    {
       USART2_TX_Callback(); 
    }
    else if(temp & USART_SR_TC)
    {
        //if(counter == 0){USART2->SR &= ~USART_SR_TC_Msk;}
        __NOP();
    }
}

void USART6_IRQHandler (void)
{

    uint32_t temp = USART6->SR;
    if(temp & USART_SR_RXNE)
    {
        dataRegister = USART6->DR;
        USART6_RX_Callback();
    }
    else if(temp & USART_SR_TC)
    {
        __NOP();
    }
    else if(temp & USART_SR_TXE)
    {
        USART6_TX_Callback();
    }
}








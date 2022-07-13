
#include "basicTimer.hh"
#include "GPIO.hh"
#include "USART.hh"
#include "MACROS.h"
#include "ADC.hh"
#include "Vector.hh"

gpioNameSpace::GPIO ledStatus(GPIOA);
timNameSpace::basicTimer systemStatusTimer(TIM9);
usartNameSpace::USART coms(USART2);

void GPIOInit(void);
void systemStatusTimerInit();
void fpuInit();
void USARTInit();



int main(void){

	fpuInit();
	GPIOInit();
	//systemStatusTimerInit();
	USARTInit();
	//systemStatusTimer.timerStart();

	while(1){


	}
}

void USARTInit()
{
	coms.settings->mode = usartNameSpace::USART_RX_MODE;
	coms.init();
}
void systemStatusTimerInit(void){

	systemStatusTimer.parameters.autoReloadPrealoadEnable = timNameSpace::ARR_NOT_BUFFERED;
	systemStatusTimer.parameters.counterMode = timNameSpace::TIM_UPCOUNTING_MODE;
	systemStatusTimer.parameters.prescaler = 16000;
	systemStatusTimer.parameters.period = 250;
	systemStatusTimer.parameters.interruptEnable = timNameSpace::INTERRUPT_ENABLE;
	systemStatusTimer.parameters.repetitionCounter = 0;
	//systemStatusTimer.parameters.onePulseMode = timNameSpace::ONE_PULSE_MODE_ENABLED;
	systemStatusTimer.parameters.interruptPriority = defsNameSpace::NVIC_PRIORITY_1;
	if(systemStatusTimer.init() != defsNameSpace::OK){
		while(1);
	}
}

void GPIOInit(void){

	using namespace gpioNameSpace;
	uint32_t ledStatusPin = gpioNameSpace::GPIO_PIN_5;
	ledStatus.setPin(ledStatusPin);
	ledStatus.setModer(gpioNameSpace::GPIO_OUTPUT_MODE);
	if(ledStatus.init() != defsNameSpace::OK){
		//error handler
		while(1);
	}
}

void TIM9_UpdateCallback(void){

	ledStatus.portToggle();

}
void USART2_RX_Callback()
{
	*coms.buffer = coms.readCharBlockingMode();
	*coms.buffer++;

}
void fpuInit(void){

	/*this is needed in order to use the FPU*/
	SCB->CPACR = ((3UL << 10*2) | (3UL << 11*2)); 
}


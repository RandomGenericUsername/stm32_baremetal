
#include "basicTimer.hh"
#include "GPIO.hh"
#include "USART.hh"
#include "MACROS.h"
#include "ADC.hh"
#include "string.h"
#include "esp01.hh"

gpioNameSpace::GPIO ledStatus(GPIOA);
timNameSpace::basicTimer systemStatusTimer(TIM9);
usartNameSpace::USART coms(USART2);

void GPIOInit(void);
void systemStatusTimerInit();
void fpuInit();
void USARTInit();

char paramsFormat[4] = {'s','s','u','\0'};
struct params
{
	char statusLedEnable[64] = {0};
	char timerPeriod[64] = {0};
	uint32_t period;

};
params *parameters = new params;

int main(void){

	fpuInit();
	GPIOInit();
	systemStatusTimerInit();
	USARTInit();
	bool control = true;
	systemStatusTimer.timerStart();

	while(1){

		//coms.readUserCommands(coms);
		coms.readMsgIt();
		//if((strcmp(parameters->statusLedEnable, "en") == 0) && (control == false))
		//{
		//	systemStatusTimer.timerStart();
		//	control = true;
		//}
		//else if((strcmp(parameters->statusLedEnable, "dis") == 0) && (control == true))
		//{
		//	systemStatusTimer.timerStop();
		//	control = false;
		//	//coms.sendCharNonBlockingMode('s');
		//}
		//if((strcmp(parameters->timerPeriod, "set") == 0) && systemStatusTimer.parameters.period != parameters->period)
		//{
		//	systemStatusTimer.setAutoReloadRegister(parameters->period);
		//}
		//if((strcmp(parameters->timerPeriod, "send") == 0))
		//{
		//	coms.sendMsgIt("s");
		//	*parameters->timerPeriod = 0;
		//}



	}
}

void USARTInit()
{
	coms.settings->RXTXmode = usartNameSpace::USART_TXRX_MODE;
	coms.paramsPtr = (void*)parameters;
	coms.paramsFormat = paramsFormat;
	coms.init();
}
void systemStatusTimerInit(void){

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
	coms.rxItCallback();
}
void USART2_TX_Callback()
{
	coms.txItCallback();
}
void fpuInit(void){

	/*this is needed in order to use the FPU*/
	SCB->CPACR = ((3UL << 10*2) | (3UL << 11*2)); 
}



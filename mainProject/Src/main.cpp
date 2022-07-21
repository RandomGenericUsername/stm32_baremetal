
#include "basicTimer.hh"
#include "GPIO.hh"
#include "USART.hh"
#include "MACROS.h"
#include "ADC.hh"
#include "string.h"
#include "esp01.hh"

gpioNameSpace::GPIO ledStatus(GPIOA);
timNameSpace::basicTimer systemStatusTimer(TIM9);
usartNameSpace::USART coms(USART1);
usartNameSpace::USART comsA(USART2);
usartNameSpace::userParams *userParameters = new usartNameSpace::userParams;

void GPIOInit(void);
void systemStatusTimerInit();
void fpuInit();
void USARTInit();


int main(void){

	fpuInit();
	GPIOInit();
	systemStatusTimerInit();
	USARTInit();
	//systemStatusTimer.timerStart();

	while(1){

		//coms.readUserCommands(coms);
		coms.readMsgIt();
		comsA.readMsgIt();
		if(strcmp(userParameters->command, "status-led") == 0)
		{
			if(userParameters->argument == 1)
			{
				systemStatusTimer.timerStart();
			}
			else
			{
				systemStatusTimer.timerStop();
			}
			*userParameters->command = '\0';
		}
		if(strcmp(userParameters->command, "change-status-led-period") == 0)
		{
			systemStatusTimer.setAutoReloadRegister(userParameters->argument);
			*userParameters->command = '\0';
		}
		if(strcmp(userParameters->command, "echo") == 0)
		{
			coms.sendMsgIt("this is an echo");
			*userParameters->command = '\0';
		}



	}
}

void USARTInit()
{
	coms.settings->RXTXmode = usartNameSpace::USART_TXRX_MODE;
	userParameters = (usartNameSpace::userParams*) coms.paramsPtr;
	coms.init();

	comsA.settings->RXTXmode = usartNameSpace::USART_TXRX_MODE;
	userParameters = (usartNameSpace::userParams*) comsA.paramsPtr;
	comsA.init();
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

void TIM9_UpdateCallback(void)
{
	ledStatus.portToggle();
}
void USART1_RX_Callback()
{
	coms.rxItCallback();
}
void USART2_RX_Callback()
{
	comsA.rxItCallback();
}
void USART6_RX_Callback()
{
	coms.rxItCallback();
}
void USART1_TX_Callback()
{
	coms.txItCallback();
}
void fpuInit(void)
{
	/*this is needed in order to use the FPU*/
	SCB->CPACR = ((3UL << 10*2) | (3UL << 11*2)); 
}



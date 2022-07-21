#include "esp01.hh"

using namespace esp01NameSpace;

    const char *getStatusCommand = "AT\n\r";
    const char getStatus = 0x0;
    const char *factoryResetCommand = "AT + 1";
    const char factoryReset = 0x0;
    const char *ESP01_AT_COMMANDS[2] = {getStatusCommand, factoryResetCommand};

ESP01::ESP01(/* args */)
{
}

ESP01::~ESP01()
{
}




int ESP01::espCoreManager(char cmd[], char checkStr[], unsigned short timeOut, char espStr[], unsigned short espHandler[], usartNameSpace::USART *husart)
{
    unsigned short result = 0;
    if(timeOut)
    {
        espHandler[5] = timeOut;
    }
    husart->sendMsgIt(cmd);

}
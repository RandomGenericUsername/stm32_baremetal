#ifndef ESP01_H
#define ESP01_H

#include "USART.hh"


namespace esp01NameSpace
{

    class ESP01
    {

    private:
        /* data */
    public:

        ESP01(/* args */);
        ~ESP01();

        int espCoreManager(char cmd[], char checkStr[], unsigned short timeOut, char espStr[], unsigned short espHandler[], usartNameSpace::USART *husart);
    };
    
    




}







#endif
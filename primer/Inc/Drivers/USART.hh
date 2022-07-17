#ifndef USART_H
#define USART_H

#include "MACROS.h"
#include "GPIO.hh"
#include "DEFINITIONS.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"

using namespace defsNameSpace;

namespace usartNameSpace
{


    /* useful macros */
    #define ASSERT_USART_INSTANCE(USARTx) (((USARTx) == USART1)    ||\
                                           ((USARTx) == USART2)    ||\
                                           ((USARTx) == USART6))
                                    


    enum USART_WORD_LENGTH{

        _8_DataBits = (uint32_t)defaultValue,
        _9_DataBits = (uint32_t)(USART_CR1_M_Msk),
    };

    enum USART_OVERSAMPLING_MODE{

        oversamplingBy16 = (uint32_t)defaultValue,
        oversamplingBy8 = (uint32_t) (USART_CR1_OVER8),
    };

    enum USART_WAKEUP_METHOD{

        idleLines = (uint32_t)defaultValue,
        AddressMark = (uint32_t)(USART_CR1_WAKE),
    };

    enum USART_PARITY_SELECTION{

        evenParityEnabled = (uint32_t)defaultValue,
        oddParityEnabled = (uint32_t)(USART_CR1_PS),
        parityCheckDisabled = (int32_t) -1,
    };

    enum USART_PARITY_INTERRUPT_ENABLE{

        parityinterruptdisabled = (uint32_t)defaultValue,
        parityinterruptenabled = (uint32_t)(USART_CR1_PEIE),
    };

    enum USART_TXE_INTERRUPT_ENABLE{

        TXEinterruptdisabled = (uint32_t)defaultValue,
        TXEinterruptenabled = (uint32_t)(USART_CR1_TXEIE),
    };

    enum USART_TXC_INTERRUPT_ENABLE{

        TXCinterruptdisabled = (uint32_t)defaultValue,
        TXCinterruptenabled = (uint32_t)(USART_CR1_TCIE),
    };

    enum USART_RXNE_INTERRUPT_ENABLE{

        RXNEinterruptdisabled = (uint32_t)defaultValue,
        RXNEinterruptenabled = (uint32_t)(USART_CR1_RXNEIE),
    };

    enum USART_IDLE_INTERRUPT_ENABLE{

        idleInterruptdisabled = (uint32_t)defaultValue,
        idleInterruptenabled = (uint32_t)(USART_CR1_IDLEIE),
    };

    enum USART_STOP_BITS{

        _1_stopBits = (uint32_t)defaultValue,
        _0_5_stopBits = (uint32_t)(USART_CR2_STOP_0),
        _2_stopBits = (uint32_t)(USART_CR2_STOP_1),
    };

    enum USART_MODE{

        USART_TX_RX_DISABLE = 0x0,
        USART_TX_MODE = 0x1U,
        USART_RX_MODE = 0x2U,
        USART_TXRX_MODE = 0x3U,
    };

    enum USART_BAUD_RATE{

    };

    struct USART_INTERRUPTS{

        USART_PARITY_INTERRUPT_ENABLE parityInterrupt;
        USART_TXE_INTERRUPT_ENABLE TXE_Interrupt;
        USART_TXC_INTERRUPT_ENABLE TXC_interrupt;
        USART_RXNE_INTERRUPT_ENABLE RXNE_interrupt;
        USART_IDLE_INTERRUPT_ENABLE idleInterrupt;
    };

    enum USART_BAUD_RATES{

        _1200_Bauds = 1200,
        _2400_Bauds = 2400,
        _9600_Bauds = 9600,
        _19200_Bauds = 19200,
        _38400_Bauds = 38400,
        _57600_Bauds = 57600,
        _115200_Bauds = 115200,
        _230400_Bauds = 230400,
        _460800_Bauds = 460800,
        _921600_Bauds = 921600,

    };

    struct USART_PARAMETERS_STRUCT{

        
        USART_MODE mode;
        USART_BAUD_RATES baudRate;
        USART_WORD_LENGTH dataLength;
        USART_PARITY_SELECTION paritiy;
        USART_STOP_BITS stopBits;
        struct USART_INTERRUPTS interrupts;
        defsNameSpace::NVIC_INTERRUPT_PRIORITY interruptsPriority;
        double internalClockFrequency;
        USART_OVERSAMPLING_MODE oversampling;
        uint32_t bufferLength;
        char terminationCharacter;

    };

    struct BAUD_RATE_VALUES{

        uint8_t fractionalPart;
        uint32_t mantisa;
        uint32_t actualBaudRate;
        float baudRateErrorPercentage;
        
    };

    enum USART_STATUS{

    };

    class USART{

        private:


            USART_TypeDef *instance;
            USART_TypeDef resetValues;
            /* initConfig functions */
            void peripheralClockEnable();

            void enableNVIC_Interrupt();
            void disableNVIC_Interrupt();


            BAUD_RATE_VALUES baudRate;

            void calculateMantisaAndFractionalPart(double internalClockFrequency, uint8_t over8, USART_BAUD_RATES desiredBaudRate, BAUD_RATE_VALUES &mantisaAndFractionalPart);

            gpioNameSpace::GPIO TX_pin;
            gpioNameSpace::GPIO RX_pin;

            void gpioConfig();


            defsNameSpace::TASK_LOCK mutex;
            uint8_t usartStatus;

            bool rxComplete;
            bool txComplete;
            bool isStringComplete;
            uint32_t rxCount;
            uint32_t txCount;
            char *rxBuffer;
            char *txBuffer;
            void parseUserCommands(void *paramsPtr, char *paramsFormat);


        public:

            /**
             * @brief Construct a new USART object
             * @param instance:pointer to peripheral's struct typeDef 
             */
            USART(USART_TypeDef *instance);
            /* USART peripheral destructor */
            ~USART();

            USART_PARAMETERS_STRUCT *settings;
            /**
             * @brief init: function that initialized the low level hardwared associated
             * @return task_status 
             */
            defsNameSpace::TASK_STATUS init();

            /**
             * @brief deInit: function that deinitializes the low level hardwared associated
             * @return task_status 
             */
            defsNameSpace::TASK_STATUS deInit();

            void set_TX_RX_mode(USART_MODE mode);
            void setDataLength(USART_WORD_LENGTH dataLength);
            void setOverSampling(USART_OVERSAMPLING_MODE oversampling);
            void setBRRConfig(BAUD_RATE_VALUES BRR);
            void setParityConfig(USART_PARITY_SELECTION);
            void setInterrupts(USART_RXNE_INTERRUPT_ENABLE, USART_TXE_INTERRUPT_ENABLE,USART_TXC_INTERRUPT_ENABLE);
            void setStopBits(USART_STOP_BITS stopBits);
            void USART_enable();
            void USART_disable();
            
            /* blocking mode functions */
            void writeCharBlockingMode(char character);
            void writeStringBlockingMode(char *string);
            char readCharBlockingMode(void);

            void readUserCommands(USART &handler);

            /* non blocking mode functions */
            void sendCharNonBlockingMode(char character);
            void writeStringNonBlockingMode(char *string);
            void writeAux();
            char readCharNonBlockingMode(void);

            char buffer;
            void *paramsPtr;
            char *paramsFormat; 





    };

}


#endif
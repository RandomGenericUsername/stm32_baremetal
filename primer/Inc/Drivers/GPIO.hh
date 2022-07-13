#ifndef GPIO_H
#define GPIO_H


    /* <--------------------------------------------------------------------------------------------------------------------------------> */
    /* <----------------------------------------------------- Library description. -----------------------------------------------------> */
    /* <----------------------------------------------------------------------------------------------------------------------------------->

     * In this context, the word "initalize" will be used several times and refers to the fact that a peripheral has had it's registers 
       modified in such a way that the peripheral is ready to be used and manipulated by the user through the driver.

     * In this context, the words "configure" and "set" used several times to refer to the fact of fetching the features and modes of operation 
       from the user as he wants to configure the driver. This parameters are saved into the class parameter's struct which will be read by 
       the initialization function to initialize the hardware according the required settings.

     * "configure" and "set" refers only to store the user settings information somewhere into the class, for that reason they will be addressed 
        as "settings" functions.

     * This library emulates a particular GPIO port from the available ones for this device(A, B, C, D, E, H)
       Said so, a particular instantation of a GPIO object gives access to and manipulates all of the 16
       pins available for that port.
        
     * Instantiating GPIO portA(GPIOA) does not initializes any hardware or enables any peripheral features, instead, it creates an object 
       that can be loaded with a partiular desired configuration using the settings functions. 
       In order to actually start using the peripheral, the init function must be invoked, after that, the driver features will be 
       fully enabled through the portA object and its methods.

     * The library currently supports the following features:

        - set which pins the user wants to initialize(not initialize itself) through a call of the "setPin" function.
        - set the GPIO mode throught the "setMode" function.
        - Set the output type through a call of the "setOtyper" function.
        - Set the output speed through a call of the "setOspeed" function.
        - Set the the pull-up or pull-down resistors through a call of the "setPUPD" function.
        - Set any of the external interruption lines on any port through a call of the "setEventsAndInterrupts" function.
        - Set or reset a pin or set of pins of the current port through a call of the "portWrite" function.
        - Read the status of a pin or set of pins through a call of the "portRead" function.
        - Toggle a pin or set of pins's state through a call of the "portToggle" function.

     * These are the basic steps to use the libray:
        
        1 - Instantiate a new GPIO object with the particular GPIO that wants to be initialized as the class's constructor argument.
            GPIO myGpioObjectName(GPIOA) wil create the handler to which the configurations will be loaded into.
        2 - Load the pertinent user configurations such as GPIO MODER, alternate functions, output speed, Etc. using the class' settings functions.
        3 - Call the class "init" method which will initialize all the preconfigured harware. 
        4 - (optional) if the user wants to add new pins to be initialized after the init function has been invoked, a call on the "addPin" 
            functions must be done, however, step #2 must be repeated in order to load the configuration for these new pins. 
            No call to "init" function is required since it is done implicitly.
        5 - (optional) if the user wants to reset some pins (but not all) a call to the "removePin" function can be done. This will reset the 
            selected pins to its reset state while leaving the other pins untouched.
        6 - To reset all the configured pins back to its reset state and leave the peripheral on reset, "deInit" function must be called.


     * The basic steps to load the user configuration to the driver using the settings functions are as follows:

        NOTE: the functions "setMode", "setOtype", "setPUPD", "setOspeed" all of them have two version: Take for example "setMode" function:
               - setMode which enables the user to set the desired 
              configuration on a specific set of pins. 
              
        1 - Call the class's method setPin(pin) where argument pin is the pin that the user wants to initialize in later steps. By doing this, confiuration 
            for those specific pins can be set using the setings functions.
            This function supports OR'ring the pin parameter in order to select several pins on a single call. i.e:
             - setPin(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2) will enable pin 0, 1 and 2 respectively to be loaded with a configuration using the setings functions.
        2 - Call the class's method setModer(moder) in order to set the GPIO mode of operation configuration for all the pins enabled by a previous"setPin" function call.
            Also, the user might use the setModer(moder, pin) overriden method with a pin parameter that allows the user to specify which pins will be loaded
            with the current configuration in case that not all enabled pins share the same configuration settings.
        3 - In case of choosing GPIO output mode:
            - Call the class's method setOtyper(outputType) in order to set the output type configuration for all enabled pins by a previous "setPin"
              function call. The overriden version of this function, setOtyper(outputType, pin) allows the user to load the current configuration only on certain pins
              defined by the pin parameter.
            - Call the class's method setOspeed(oSpeed) in order to load the output speed configuration for all the pins enabled by the "setPin" function call in step #1.
              The overriden version of this function: setOspeed(oSpeed, pin) allows the user to load the current configuration only on the specified set of pins.
        4 - Call the class's method setPUPD(pupd) in order to load the pull up or pull down configuration for the pins enabled by the call of "setPin(pin)" function.
        5 - Call the class's method setAF(alternateFunction) in order to load the alternate function configuration.
        6 - Call the class's method setEventsAndInterrupts(event, trigger) in order to configure the events and interrupts.

     * NOTE 1 on setting functions: All of the settings functions that takes "pin" as input parameter, allow the OR'ring of the pin parameter in order 
       to manipulate several pins at once.

     * NOTE 2 on settings functions: All of the setting functions are overriden like fun(parameter,pin) and fun(parameter) This is done so that the user is allowed to
       let the driver know specifically which pins the configuration will be loaded into. When no "pin" parameter is used, the default value
       is all the pins enabled through a call of the "setPin(pin)" function.

     * Description on class's methods:

        <-------------------------------------------------------------------------------------------------------------------------------------------------------------------->
        <-------------------------------------------------------------- settings functions ---------------------------------------------------------------------------------->
        <-------------------------------------------------------------------------------------------------------------------------------------------------------------------->

        - setPin(uint32_t pin): 

        - getPin(): Returns a 16 bit unsigned number in which every position represents a particular pin being used. I.e a returned value of 101011 means that pin 0, pin 1, pin3, pin5
        are currently initialized by the driver in the corresponding GPIO port.
    
        - getPort(): Returns a user defined type GPIO_PORT which represents the GPIO port being used by this particular object's instance.
    
        - getExtIEnabledLines(): Returns a 16 bits unsigned integer number in which every set bit represents an interrupt line which is enabled. A returned value of 1001 means that
        both 0 and 3 external interrupt lines are being used. This is a shared resource among all GPIO instances.



     * 
     * 
     */
#include "DEFINITIONS.h"
#include "stm32f411xe.h"
#include "assert.h"
#include "MACROS.h"

namespace gpioNameSpace
{

    /* <-----------------------------------------------------------------------------> */
    /* <------------------------------- USEFUL MACROS -------------------------------> */
    /* <-----------------------------------------------------------------------------> */
    #define ASSERT_GPIO_INSTANCE(INSTANCE)	        (((INSTANCE) == GPIOA)              || \
                                                     ((INSTANCE) == GPIOB)              || \
                                                     ((INSTANCE) == GPIOC)              || \
                                                     ((INSTANCE) == GPIOD)              || \
                                                     ((INSTANCE) == GPIOE)              || \
                                                     ((INSTANCE) == GPIOH))

    #define ASSERT_GPIO_PIN(PIN)                    (((PIN) & 0xFFFF) && (0xFFFF0000 ^ (PIN)))



    #define ASSERT_GPIO_PORT(PORT)                  ((PORT == GPIO_PORT_A) || \
                                                    (PORT == GPIO_PORT_B) || \
                                                    (PORT == GPIO_PORT_C) || \
                                                    (PORT == GPIO_PORT_D) || \
                                                    (PORT == GPIO_PORT_E) || \
                                                    (PORT == GPIO_PORT_H))

    #define ASSERT_GPIO_MODE(MODE)		   (((MODE) == GPIO_INPUT_MODE)				    ||\
                                            ((MODE) == GPIO_OUTPUT_MODE)  				||\
                                            ((MODE) == GPIO_ANALOG_INPUT_MODE)  		||\
                                            ((MODE) == GPIO_ALTERNATE_FUNCTION_MODE))

    #define ASSERT_GPIO_OSPEED(OSPEEDR)	    (((OSPEEDR) == GPIO_OUTPUT_LOW_SPEED) 	 || 	\
                                            ((OSPEEDR) == GPIO_OUTPUT_MEDIUM_SPEED)  ||		\
                                            ((OSPEEDR) == GPIO_OUTPUT_FAST_SPEED) 	 ||		\
                                            ((OSPEEDR) == GPIO_OUTPUT_HIGH_SPEED))

    #define ASSERT_GPIO_OTYPE(OTYPER)		(((OTYPER) == GPIO_OT_PUSH_PULL)			|| 		\
                                            ((OTYPER) == GPIO_OT_OPEN_DRAIN))

    #define ASSERT_GPIO_PUPD(PUPD)			(((PUPD) == GPIO_PULL_UP )				|| 		\
                                            ((PUPD) == GPIO_PULL_DOWN)				|| 		\
                                            (PUPD) == GPIO_NO_PUPD )
                                            
    #define ASSERT_GPIO_AF(AF)				(((AF) == GPIO_AF_0)      	|| 		\
                                            ((AF) == GPIO_AF_1)  		||		\
                                            ((AF) == GPIO_AF_2)		    ||		\
                                            ((AF) == GPIO_AF_3)		    ||		\
                                            ((AF) == GPIO_AF_4)		    ||		\
                                            ((AF) == GPIO_AF_5)		    ||		\
                                            ((AF) == GPIO_AF_6)		    ||		\
                                            ((AF) == GPIO_AF_7)		    ||		\
                                            ((AF) == GPIO_AF_8)		    ||		\
                                            ((AF) == GPIO_AF_9)		    ||		\
                                            ((AF) == GPIO_AF_10)	        ||		\
                                            ((AF) == GPIO_AF_11)	        ||		\
                                            ((AF) == GPIO_AF_12)	        ||		\
                                            ((AF) == GPIO_AF_13)	        ||		\
                                            ((AF) == GPIO_AF_14)	        ||		\
                                            ((AF) == GPIO_AF_15))	

    #define GET_GPIOx_INDEX(__GPIO__)       (uint8_t)((__GPIO__) == GPIOA ? 0x0U :      \
                                                    (__GPIO__) == GPIOB ? 0x01U:      \
                                                    (__GPIO__) == GPIOC ? 0x02U:      \
                                                    (__GPIO__) == GPIOD ? 0x03U:      \
                                                    (__GPIO__) == GPIOE ? 0x04U:      \
                                                    0x7U                              \
                                                    )


    #define ASSERT_ExI_(ExI)                (((ExI) == EXTE_EXTI_ENABLED) || \
                                            ((ExI) == EXTE_EXTI_ENABLED)   || \
                                            ((ExI) == EXTE_ENABLED)        || \
                                            ((ExI) == EXTI_ENABLED))

    #define ASSERT_ExI_TRIGGER(TRIGGER)     (((TRIGGER) == ExI_NO_TRIGGER_ENABLED) || \
                                            ((TRIGGER) == ExI_RISING_TRIGGER_ENABLED)   || \
                                            ((TRIGGER) == ExI_FALLING_TRIGGER_ENABLED)             || \
                                            ((TRIGGER) == ExI_RISING_AND_FALLING_TRIGGER_ENABLED))



    /* <--------------------------------------------------------------------------------> */
    /* <------------------------------- USER DEFINITIONS -------------------------------> */
    /* <--------------------------------------------------------------------------------> */

    uint8_t const NUMBER_OF_GPIO_PORTS = 16;    /* number of pins available in each port. This is used as a reference variable in the pin init functions. */

    /* GPIO possible states definition */
    enum GPIO_PIN_STATE{

        GPIO_PIN_RESET = 0,       /* GPIO reset state */
        GPIO_PIN_SET = 1,         /* GPIO set state */
    };


    /* GPIO parameters */
    enum GPIO_PARAMETERS{
        
        GPIO_PORT_PARAM = 0x0,
        GPIO_PIN_PARAM = 0x1,
        GPIO_MODE_PARAM = 0x2,
        GPIO_OTYPE_PARAM = 0x3,
        GPIO_OSPEED_PARAM = 0x4,
        GPIO_PUPD_PARAM = 0x5,
        GPIO_AF_PARAM = 0x6, /*AF occupies 0x8 -> 2 spaces */
        GPIO_EXT_ExI_PARAM = 0x8,
        GPIO_EXT_ExI_TRIGGER_PARAM = 0x9,
        GPIO_INTERRUPT_PRIORITY_PARAM = 0xA
    };

    /* GPIO Ports definition */
    enum GPIO_PORT{

        GPIO_PORT_A = 0x1,          /* GPIO port A */
        GPIO_PORT_B = 0x2,          /* GPIO port B */
        GPIO_PORT_C = 0x4,          /* GPIO port C */
        GPIO_PORT_D = 0x8,          /* GPIO port D */
        GPIO_PORT_E = 0x10,         /* GPIO port E */
        GPIO_PORT_H = 0x20,         /* GPIO port H */

    };

    /* GPIO mode selection definition */
    enum GPIO_MODER{
        
        
        GPIO_INPUT_MODE = 0x0U,                 /* GPIO input selection mode */
        GPIO_OUTPUT_MODE = 0x1U,                /* GPIO output selection mode */
        GPIO_ALTERNATE_FUNCTION_MODE = 0x2U,    /* GPIO alternate function selection mode */
        GPIO_ANALOG_INPUT_MODE = 0x3U,          /* GPIO analog input mode */
        GPIO_MODER_MSK = 0x3U                   /* GPIO mode selection mask */
    };

    /* GPIO output mode type selection */
    enum GPIO_OTYPER{

        GPIO_OT_PUSH_PULL = 0x0U,      /* GPIO push pull output mode */
        GPIO_OT_OPEN_DRAIN	= 0x1U,     /* GPIO open dragin output mode */
        GPIO_OT_OTYPER_MSK	= 0x1U,     /* GPIO output mode type selection mask */ 
    };

    /* GPIO output speed selection */
    enum GPIO_OSPEED{

        GPIO_OUTPUT_LOW_SPEED  = 0x0UL,     /* GPIO output low speed */
        GPIO_OUTPUT_MEDIUM_SPEED = 0x1UL,   /* GPIO output medium speed */
        GPIO_OUTPUT_FAST_SPEED	= 0x2UL,    /* GPIO output fast speed */
        GPIO_OUTPUT_HIGH_SPEED	= 0x3UL,    /* GPIO output high speed */
        GPIO_OSPEEDR_MSK = 0x3UL,           /* GPIO output speed mask */
    };

    /* GPIO pull up-pull down selection */
    enum GPIO_PUDR{

        GPIO_NO_PUPD = 0x00U,           /* GPIO no pull up or pull down selected */
        GPIO_PULL_UP = 0x01U,           /* GPIO pull up selection */ 
        GPIO_PULL_DOWN = 0x02U,         /* GPIO pull  down selection */
        GPIO_PUPDR_MSK = 0x03U,         /* GPIO pull up-pull down selection mask */
    };


    /* GPIO alternate function selection */
    enum GPIO_AF{

        GPIO_AF_0 = 0x0U,                /* GPIO alternate function 0 */
        GPIO_AF_1 = 0x1U,                /* GPIO alternate function 1 */
        GPIO_AF_2 = 0x2U,                /* GPIO alternate function 2 */
        GPIO_AF_3 = 0x3U,                /* gpio alternate function 3 */
        GPIO_AF_4 = 0x4U,                /* GPIO alternate function 4 */
        GPIO_AF_5 = 0x5U,                /* GPIO alternate function 5 */
        GPIO_AF_6 = 0x6U,                /* GPIO alternate function 6 */
        GPIO_AF_7 = 0x7U,                /* GPIO alternate function 7 */
        GPIO_AF_8 = 0x8U,                /* GPIO alternate function 8 */
        GPIO_AF_9 = 0x9U,                /* GPIO alternate function 9 */
        GPIO_AF_10 = 0xAU,               /* GPIO alternate function 10 */
        GPIO_AF_11 = 0xBU,               /* GPIO alternate function 11 */
        GPIO_AF_12 = 0xCU,               /* GPIO alternate function 12 */
        GPIO_AF_13 = 0xDU,               /* GPIO alternate function 13 */
        GPIO_AF_14 = 0xEU,               /* GPIO alternate function 14 */
        GPIO_AF_15 = 0xFU,               /* GPIO alternate function 15 */
        GPIO_AF_MSK	= 0xFU,             /* GPIO alternate function selection mask */
    };

    /* GPIO external interrupt and events enable */
    enum EXT_ExI{

        EXTE_EXTI_DISABLED = 0,         /* GPIO external events and interrupts disable */
        EXTE_ENABLED = 0x1,             /* GPIO external events enable */
        EXTI_ENABLED = 0x2,             /* GPIO external interrupts enable */
        EXTE_EXTI_ENABLED = 0x3,        /* GPIO external events and interrupts enable */
        EXT_ExI_MSK = 0x3,              /* GPIO external events and interrupts selection mask */
    };

    /* GPIO external events and interrupts trigger selection */ 
    enum EXT_ExI_TRIGGER_SEL{

        ExI_NO_TRIGGER_ENABLED = 0x00,                      /* GPIO external events and interrupts no trigger enabled */
        ExI_RISING_TRIGGER_ENABLED = 0x01,                  /* GPIO external events and interrupts rising trigger selection */
        ExI_FALLING_TRIGGER_ENABLED = 0x02,                 /* GPIO external events and interrupts falling trigger selection */
        ExI_RISING_AND_FALLING_TRIGGER_ENABLED = 0x03,      /* GPIO external events and interrupts rising and falling trigger selection */
        ExI_TRIGGER_SELECTION_MSK = 0x3,                    /* GPIO external events and interrupts trigger selection mask */
    };

    /* GPIO pin number definition */
    enum GPIO_PIN{

        GPIO_PIN_0 = (0x1U<<0),         /* GPIO pin 0 */
        GPIO_PIN_1 = (0x1U<<1),         /* GPIO pin 1 */
        GPIO_PIN_2 = (0x1U<<2),         /* GPIO pin 2 */
        GPIO_PIN_3 = (0x1U<<3),         /* GPIO pin 3 */
        GPIO_PIN_4 = (0x1U<<4),         /* GPIO pin 4 */
        GPIO_PIN_5 = (0x1U<<5),         /* GPIO pin 5 */
        GPIO_PIN_6 = (0x1U<<6),         /* GPIO pin 6 */
        GPIO_PIN_7 = (0x1U<<7),         /* GPIO pin 7 */
        GPIO_PIN_8 = (0x1U<<8),         /* GPIO pin 8 */
        GPIO_PIN_9 = (0x1U<<9),         /* GPIO pin 9 */
        GPIO_PIN_10 = (0x1U<<10),       /* GPIO pin 10 */
        GPIO_PIN_11 = (0x1U<<11),       /* GPIO pin 11 */
        GPIO_PIN_12 = (0x1U<<12),       /* GPIO pin 12 */
        GPIO_PIN_13 = (0x1U<<13),       /* GPIO pin 13*/
        GPIO_PIN_14 = (0x1U<<14),       /* GPIO pin 14*/
        GPIO_PIN_15 = (0x1U<<15),       /* GPIO pin 15*/
        GPIO_PIN_ALL = 0xFFFF,          /* GPIO all pins selected */
    };

    /* <--------------------------------------------------------------------------------> */
    /* <------------------------------- USER DEFINED STRUCTS -------------------------------> */
    /* <--------------------------------------------------------------------------------> */

    /* Struct in which all the peripheral parameters are stored */
    /* This struct is accesed by the setter functions in order to store the params */
    /* as well as it is accessed by the init function to initialize the peripheral */
    /* according to the parameters specified */
    /* Data is stored in 32 bits unsigned numbers in form of pairs of bits for each corresponding pin */
    /* for example, bits 0 and 1 of the pinMode parameter accounts for the 4 possible modes of operation for pin 0 */ 
    struct GPIO_PARAMETERS_STRUCT
    {
        uint32_t port;                      /* Paremeter needed by the driver to select which bus clock peripheral to enable */
        uint32_t pin;                       /* Parameter that corresponds to the pins that are to be initialized. In order to set this param, the "setPin" function must to be invoked using as argument the pin number that the user wants to initialize */
        uint32_t pinMode;                   /* This parameter stores the corresponding modes in which the pins will be initialized */
        uint32_t outputType;                /* This parameter stores the settings for the output type in which the corresponding pin will be initialized */
        uint32_t outputSpeed;               /* This parameters states what will the output speed of the corresponding pin will be */
        uint32_t pullUpPullDown;            /* This paratemer stores the settings for initializing pull up or pull down on the corresponding pin */
        uint32_t alternateFunctions[2];     /* This parameters stores the alternate function number which will be used on the given pin */ 
        uint32_t eventsAndInterrupts;       /* This parameters stores the information about which events and/or interrupts will be used on this port */
        uint32_t risingAndFallingTrigger;   /* This parameter stores the information about the edge trigger for the external events and/or interrupts */
        uint32_t interruptPriority[2];      /* This parameters stores information about each of the pins interrupt priority */
        
    };

    struct GPIO_ALLOCATED_RESOURCES
    {
        uint16_t gpioA_AllocatedPin;
        uint16_t gpioB_AllocatedPin;
        uint16_t gpioC_AllocatedPin;
        uint16_t gpioD_AllocatedPin;
        uint16_t gpioE_AllocatedPin;
        uint16_t reserved1;
        uint16_t reserved2;
        uint16_t gpioH_AllocatedPin;

    };

    /**
     * @brief GPIO peripheral class:
     * This class emulates a given GPIO port {A, B, C, D, E, H}, that means, when instantiated, the class
     * offers a means of controlling a whole GPIO port and all of its pins.
     * 
     */
    class GPIO{

        /* class's private members */
        private:

            /* <---------------------------------------------------------------------------------------------> */
            /* <------------------------------ Hardware manipulation functions ------------------------------> */
            /* <---------------------------------------------------------------------------------------------> */

            defsNameSpace::TASK_STATUS initPin();            /* Initializes low lever hardware associated with physical pins */
            defsNameSpace::TASK_STATUS deInitPin();          /* Deinitializes low lever hardware associated with physical pins */  
            void peripheralClockEnable();                    /* Enables the corresponding GPIO bus clock signal */
            void enableNVIC_Interrupt();                     /* Enables the NVIC interrupt service of already initialized pin */
            void disableNVIC_Interrupt();                    /* Disables the NVIC interrupt service of already initialized pins */
            void enableNVIC_Interrupt(uint32_t pin);         /* Enables the NVIC interrupt service of the pin passed onto the argument */
            void disableNVIC_Interrupt(uint32_t pin);        /* Disables the NVIC interrupt service of the pin passed onto the argument */

            /* <---------------------------------------------------------------------------------------------> */
            /* <---------------------------------- Class settings functions ---------------------------------> */
            /* <---------------------------------------------------------------------------------------------> */

            void setPort(GPIO_TypeDef *GPIOx);                                                                                      /* sets the port parameter based on the instance with which the peripheral was initialized */
            void inline setParameter(uint32_t param, uint32_t paramClrMsk, uint32_t paramVal);                                      /* Function that allows to set any of the GPIO parameters and store its value onto the parameters struct */
            void inline setParameterHighRegisters(uint32_t param, uint32_t paramClrMsk, uint32_t paramVal, uint32_t regLevel);      /* Function that allows to set any of the GPIO parameters and store its value onto the parameters struct */
            void inline setParametersLoop(uint32_t param, uint32_t paramClrMsk, uint32_t paramVal, uint32_t position);              /* Function that allows to set any of the GPIO parameters and store its value onto the parameters struct */
            void inline setParametersHighRegistersLoop(uint32_t param, uint32_t paramClrMsk, uint32_t paramVal, uint32_t position, uint32_t regLevel);  /* Function that allows to set any of the GPIO parameters and store its value onto the parameters struct */
            uint32_t inline getRegisterBitLength(uint32_t mask);                                                                    /* Function that calculates a register bit's lenght based on its mask */
            void inline alignMaskToZero(uint32_t &mask, uint32_t &remainingBits);                                                   /* Function that takes off  a mask value offset to zero */


            /* Whenever a pin is initialized by the Init function(hardware initialization) it is registered onto this variable */
            uint32_t alreadyRegisteredPin;

            /* ptr to GPIO_PARAMETERS_STRUCT member: used internally to store the settings in a single name and allow access to it*/
            GPIO_PARAMETERS_STRUCT *parameters;    

            /* Variable which stores the information about which external interrupts lines have been initializated. It's declared as static since this value must to be common to all of the GPIO peripheral classes */
            static uint16_t extiLinesEnabled; 
            /* This variable stores information about the pins initialized on each port */
            static GPIO_ALLOCATED_RESOURCES allocatedResources;

            GPIO_TypeDef *instance;
            GPIO_TypeDef resetValues;



        public:

            /* <---------------------------------------------------------------------------------------------> */
            /* <------------------------------ Class constructor and destructor ------------------------------> */
            /* <---------------------------------------------------------------------------------------------> */

            /**
             * @brief Construct a new GPIO object
             * @param GPIOx: pointer to the corresponding peripheral instance.
             */
            GPIO(GPIO_TypeDef *GPIOx);

            /**
             * @brief Destroy the current GPIO object
             */
            ~GPIO();

            /* <---------------------------------------------------------------------------------------------> */
            /* <------------------------------ Hardware manipulation functions ------------------------------> */
            /* <---------------------------------------------------------------------------------------------> */

            /**
             * @brief Overriden function from the abstract parent class. This function initiates the peripheral required low level hardware 
             * @return TASK_STATUS: OK if the initialization went without any problem.
             * ERROR if something else happend
             */
            defsNameSpace::TASK_STATUS init();

            /**
             * @brief Overriden function from the abstract parent class. This function deinitialize the peripheral required low level hardware 
             * @return TASK_STATUS: OK if the initialization went without any problem.
             * ERROR if something else happend
             */
            defsNameSpace::TASK_STATUS deInit();

            /**
             * @brief: This function allows to modify a certain pin's ON or OFF state
             * @param state: Posible values are GPIO_PIN_SET to turn ON and GPIO_PIN_RESET to turn OFF 
             * @param pin: Pin whose status is about to be changed. The functions supports OR'ing pins 
             * such as: GPIO_PIN_0 | GPIO_PIN_4 in order to manipulate several pins at once.
             */
            void portWrite(GPIO_PIN_STATE state, uint32_t pin);

            /**
             * @brief: This function allows to modify ON or OFF state for all the pins initialized in the current object. 
             * @param state: Posible values are GPIO_PIN_SET to turn on and GPIO_PIN_RESET to turn off 
             * such as: GPIO_PIN_0 | GPIO_PIN_4 in order to manipulate several pins at once.
             */
            void portWrite(GPIO_PIN_STATE state);

            /**
             * @brief: This function allows to toggle between ON and OFF state for a given pin acording to the function input parameter. 
             * @param pin: Pin whose status is about to be toggled. The functions supports OR'ing pins 
             * such as: GPIO_PIN_0 | GPIO_PIN_4 in order to toggle several pins at once.
             */
            void portToggle(uint32_t pin);

            /**
             * @brief: This function allows to toggle between ON and OFF state for a given pin acording to the function input parameter. 
             * Override portToggle function that toggles all the already initialized pins.
             */
            void portToggle();

            /**
             * @brief Function that returns the SET or RESET status of the selected pin 
             * @param pin pin number to be read. The functions supports OR'ing pins 
             * such as: GPIO_PIN_0 | GPIO_PIN_4 in order to read several pins at once.
             * @return uint32_t representing the pin status according to it's bit position. i.e;
             * a value 0b1001 means that both GPIO_PIN_0 and GPIO_PIN3 are set.
             * 
             */
            uint32_t portRead(uint32_t pin);


            /**
             * @brief Function that returns the SET or RESET status of the selected pin 
             * This is the overriden version of the portRead function so it read every pin that has been initialized before.
             * @return uint32_t representing the pin status according to it's bit position. i.e;
             * a value 0b1001 means that both GPIO_PIN_0 and GPIO_PIN3 are set.
             */
            uint32_t portRead();

            /**
             * @brief When called, this function initializes the specified set of pins according to a previously configured settings.
             * This function was conceibed as a way to let the user initialize pins after the "init" function has been called.
             * This method manipulates the hardware itself so no "init" call is required.
             * @param pin: specifies the pins be to initilized
             */
            void addPin(uint32_t pin);

            /**
             * @brief When called, this function resets back the reset state of the specified pins.
             * @param pin specifies the pin or set of pins to be reset to it's default state.
             */
            void removePin(uint32_t pin);


            /* <---------------------------------------------------------------------------------------------> */
            /* <---------------------------------- Class settings functions ---------------------------------> */
            /* <---------------------------------------------------------------------------------------------> */


            /**
             * @brief This function allows the user to select which pins will be initialized on the current port.
             * @param pin Number of the pin that is about to be initalized. The pin numbers can be ORred like "GPIO_PIN_0 |  GPIO_PIN_1 | GPIO_PIN2" in order
             * to initialize pin 0, 1 and 2 under a single object GPIO. Posible values are GPIO_PIN_0...GPIO_PIN_n...GPIO_PIN_15
             */
            void setPin(uint32_t pin);

            /**
             * @brief This functions sets the GPIO mode for a specific pin or set of pins according to the function input parameter.
             * @param moder GPIO mode. Posible values are: GPIO input, output, analog input or alternate function mode.
             * @param pin This parameter allows the user to choose which pins will be set with the "moder" parameter.  
             * I.e: if the function is called as "setModer(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_INPUT_MODE)" then pins 0, 1, and 2 will be initialized as input mode in the current port.
             */
            void setModer(uint32_t moder, uint32_t pin);

            /**
             * @brief This functions sets the GPIO mode for all the pins selected with the "setPin" function.
             * This is the overriden setModer function that initializes all selected pins through a call of the "setPin" function by default.
             * @param moder GPIO mode. Posible values are: GPIO input, output, analog input or alternate function mode.
             */
            void setModer(uint32_t moder);

            /**
             * @brief This function sets the GPIO output type for a given pin or set of pins according to the input parameter.
             * @param otyper output type parameter. Posible values are: GPIO_OT_PUSH_PULL, GPIO_OT_OPEN_DRAIN.
             * @param pin pins that the user wants to set with the corresponding otyper confiuration.
             * This Function supports OR'ring the pin parameter so several pins can be set under the same configuration.
             * A function call like "setOtyper(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_OT_OPEN_DRAIN)" sets pins 0, 1, and 2 of the current to be initialized using an open drain type output.
             */
            void setOtyper(uint32_t otyper, uint32_t pin);

            /**
             * @brief Overriden version of the "setOtyper" function. This function sets the GPIO output type for all the pins selected through the call of the "setPin" functions. By default,
             * this function sets all the selecte pins under the same configuration settings.
             * @param otyper output type parameter. Posible values are: GPIO_OT_PUSH_PULL, GPIO_OT_OPEN_DRAIN.
             */
            void setOtyper(uint32_t otyper);


            /**
             * @brief This function sets the output speed configuration for a pin or set of pins according to whats input in the pin parameter.
             * @param ospeed output speed parameter. Posible values are:  "GPIO_OUTPUT_LOW_SPEED, GPIO_OUTPUT_SPEED_MEDIUM, GPIO_OUTPUT_SPEED_FAST, GPIO_OUTPUT_SPEED_HIGH"
             * @param pin specifies the pins to e configured with the ospeed seleted value
             */
            void setOSpeed(uint32_t ospeed, uint32_t pin);

            /**
             * @brief Overriden version of the "setOspeed" function that configures the output speed mode by default to all
             * selected pins through a call of the "setPin" function.
             * @param ospeed output speed parameter. Posible values are: "GPIO_OUTPUT_LOW_SPEED, GPIO_OUTPUT_SPEED_MEDIUM, GPIO_OUTPUT_SPEED_FAST, GPIO_OUTPUT_SPEED_HIGH"
             */
            void setOSpeed(uint32_t ospeed);

            /**
             * @brief This function selects the pull up or pull down internal resistor configuration for a pin or set of pins.
             * @param pupd posible values are: "GPIO_PULL_UP, GPIO_PULL_DOWN"
             * @param pin specifies the pins which will be configured with the pupd parameter
             */
            void setPUPD(uint32_t pupd, uint32_t pin);

            /**
             * @brief Overriden version of the "setPUPD" functions. This function configures the given pupd for all
             * the pins selected through a call of the "setPin" function.
             * @param pin specifies the pins or set of pins which will be configured with the pupd parameter
             */
            void setPUPD(uint32_t pupd);

            /**
             * @brief This function configures a certain alternate function on the specified pin or set of pins.
             * @param AF alternate function number. Posible values are: "GPIO_AF_0, GPIO_AF_1, ... GPIO_AF_15"
             * @param pin specifies the pin or set of pins which will be configured with the given AF
             */
            void setAF(uint32_t AF, uint32_t pin);
            /**
             * @brief Overriden version of the "setAF" function that configures a certain alternate function by default on all pins selected through a call of the "setPin" function
             * @param AF alternate function number. Posible values are: "GPIO_AF_0, GPIO_AF_1, ... GPIO_AF_15"
             */
            void setAF(uint32_t AF);

            /**
             * @brief This function configures the external events and/or interrupt on the pin or set of pins following the input parameter.
             * @param ExI_service selects the service to be started. Posible values are "EXTE_EXTI_DISABLE, EXTE_ENABLE, EXTI_ENABLE, EXTE_EXTI_ENABLE"
             * @param triggerSelection selects the edge trigger for the selected event or interrupt. Posible values are: "ExI_NO_TRIGGER_ENABLED, ExI_RISING_TRIGGER_ENABLED, ExI_FALLING_TRIGGER_ENABLED, ExI_RISING_AND_FALLING_TRIGGER_ENABLED"
             * @param pin specifies the pins in which the events and interrupts configuration will be loaded into
             */
            void setEventsAndInterrupts(uint32_t ExI_service, uint32_t triggerSelection, uint32_t pin);
            /**
             * @brief Overriden version of the "setEventsAndInterrupts" function. This function configures the given event and/or interrupt for all the 
             * pins selected through a call of the "setPin" function
             * @param ExI_service 
             * @param triggerSelection 
             */
            void setEventsAndInterrupts(uint32_t ExI_service, uint32_t triggerSelection);

            /**
             * @brief This function configures the nvic interrupt priority for the selected pins.
             * @param priority 
             * @param pin 
             */
            void SetInterruptPriority(defsNameSpace::NVIC_INTERRUPT_PRIORITY priority, uint32_t pin);
            void SetInterruptPriority(defsNameSpace::NVIC_INTERRUPT_PRIORITY priority);

            /* <---------------------------------------------------------------------------------------------> */
            /* <---------------------------------- Class getter functions ---------------------------------> */
            /* <---------------------------------------------------------------------------------------------> */


            /**
             * @brief This function returns the pin or set of pins which are currently in use by the particular object instance.
             * @return a number representing the pins which are currently initalized.
             */
            uint32_t getPin();

            /**
             * @brief This function allows the user to know which port is this object's instance using.
             * @return GPIO_PORT 
             */
            GPIO_PORT getPort();


            /**
             * @brief This functions allows the user to get which EXTI lines are currently being used by any of the GPIO instances.
             * @return uint16_t number representing each interrupt line.
             */
            static inline uint16_t getEXTIEnabledLines();

            static bool isResourceAllocated(GPIO_TypeDef *gpio, GPIO_PIN pin);

            


    };

}



#endif
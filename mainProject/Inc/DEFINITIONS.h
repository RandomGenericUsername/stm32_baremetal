#ifndef DEFS_H
#define DEFS_H


namespace defsNameSpace {

    /**
     * @brief indicates whether a task is being excuted or not.
     * @OK 
     */
    enum TASK_LOCK{

        UNLOCKED = 0x0,
        LOCKED = 0x1,
    };

    /**
     * @brief indicates whether a task is was executed succesfully or not.
     * @OK 
     */
    enum TASK_STATUS{

        OK = 0x0,
        ERROR = 0x1,
    };

    enum PERIPHERAL_STATUS
    {
        PERIPHERAL_RESET = 0x0,
        PERIPHERAL_READY = 0x1,
        PERIPHERAL_BUSY = 0x2,
        PERIPHERAL_ERROR = 0x4,
    };

    enum NVIC_INTERRUPT_PRIORITY{

        NVIC_HIGHEST_PRIORITY = 0x0,
        NVIC_PRIORITY_1 = 0x1,
        NVIC_PRIORITY_2 = 0x2,
        NVIC_PRIORITY_3 = 0x3,
        NVIC_PRIORITY_4 = 0x4,
        NVIC_PRIORITY_5 = 0x5,
        NVIC_PRIORITY_6 = 0x6,
        NVIC_PRIORITY_7 = 0x7,
        NVIC_PRIORITY_8 = 0x8,
        NVIC_PRIORITY_9 = 0x9,
        NVIC_PRIORITY_10 = 0xA,
        NVIC_PRIORITY_11 = 0xB,
        NVIC_PRIORITY_12 = 0xC,
        NVIC_PRIORITY_13 = 0xD,
        NVIC_PRIORITY_14 = 0xE,
        NVIC_PRIORITY_15 = 0xF,
        NVIC_PRIORITY_MSK = 0xF,
    };

    #define REGISTER_BITS_LENGTH         32
    #define LOW_REGISTER                 0
    #define HIGH_REGISTER                1

    #define defaultValue 0
}



#endif
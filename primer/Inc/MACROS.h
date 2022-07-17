#ifndef MACROS_H
#define MACROS_H

#include "DEFINITIONS.h"
#include "stdbool.h"


 #define TASK_UNLOCK(__HANDLE__)                                            \
                                  do{                                       \
                                      (__HANDLE__)->lock = UNLOCKED;        \
                                  }while (0U)
 #define TASK_LOCK(__HANDLE__)                                              \
                                  do{                                       \
                                      (__HANDLE__)->lock = LOCKED;          \
                                  }while (0U)

#define GET_INDEX(__NUMBER__)                                               \
                                do{                                         \
                                                                            \
                                      uint32_t position, regPosition;       \
                                      uint32_t temp = (__NUMBER__);         \
                                                                            \
                                      position = 0;                         \
                                      regPosition = 1 << position;          \
                                      do                                    \
                                      {                                     \
                                          position++;                       \
                                          regPosition = 1 << position;      \
                                      }while((regPosition ^ (__NUMBER__))); \
                                                                            \
                                }while(0)



#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define ASSERT_NVIC_INTERRUPT_PRIORITY_SELECTION(__p__) (((__p__) == defsNameSpace::NVIC_HIGHEST_PRIORITY) ||\
                                                         ((__p__) == defsNameSpace::NVIC_PRIORITY_1) ||\
                                                         ((__p__) == defsNameSpace::NVIC_PRIORITY_2) ||\
                                                         ((__p__) == defsNameSpace::NVIC_PRIORITY_3) ||\
                                                         ((__p__) == defsNameSpace::NVIC_PRIORITY_4) ||\
                                                         ((__p__) == defsNameSpace::NVIC_PRIORITY_5) ||\
                                                         ((__p__) == defsNameSpace::NVIC_PRIORITY_6) ||\
                                                         ((__p__) == defsNameSpace::NVIC_PRIORITY_7) ||\
                                                         ((__p__) == defsNameSpace::NVIC_PRIORITY_8) ||\
                                                         ((__p__) == defsNameSpace::NVIC_PRIORITY_9) ||\
                                                         ((__p__) == defsNameSpace::NVIC_PRIORITY_10) ||\
                                                         ((__p__) == defsNameSpace::NVIC_PRIORITY_11) ||\
                                                         ((__p__) == defsNameSpace::NVIC_PRIORITY_12) ||\
                                                         ((__p__) == defsNameSpace::NVIC_PRIORITY_13) ||\
                                                         ((__p__) == defsNameSpace::NVIC_PRIORITY_14) ||\
                                                         ((__p__) == defsNameSpace::NVIC_PRIORITY_15))

#endif
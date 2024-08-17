/**
 * @author:  trieunvt
 * @file:    controller.h
 * @date:    16 Aug 2024
 * @version: v1.0.0
 * @brief:   The STM32 NUCLEO-L452RE I2C slave motor controller.
**/

#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "string.h"
#include "stspin220.h"
#include "drv8210dsgr.h"
#include "linked_list_queue.h"
#include "SEGGER_RTT.h"

/* Developer-defined macros */
#define print(...) SEGGER_RTT_printf(0, __VA_ARGS__); \
                   osDelay(100);

/* Function declarations */
void ControllerTaskInit(void);
void ControllerTaskLoop(void);

void Motor1TaskLoop(void);
void Motor2TaskLoop(void);

#endif /* _CONTROLLER_H_ */

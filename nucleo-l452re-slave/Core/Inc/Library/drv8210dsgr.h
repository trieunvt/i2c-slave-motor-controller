/**
 * @author:  trieunvt
 * @file:    drv8210dsgr.h
 * @date:    16 Aug 2024
 * @version: v1.0.0
 * @brief:   The DRV8210DSGR brushed DC motor driver.
**/

#ifndef _DRV8210DSGR_H_
#define _DRV8210DSGR_H_

#include "stm32l452xx.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os2.h"

/* Developer-defined macros */
#define MODE_BRAKE                  0
#define MODE_REVERSE                1
#define MODE_FORWARD                2

/* Struct type definitions for the following usage */
typedef struct DRV8210DSGR_Class    DRV8210DSGR_ClassTypeDef;
typedef struct DRV8210DSGR_APIs     DRV8210DSGR_APIsTypeDef;
typedef struct DRV8210DSGR_Pins     DRV8210DSGR_PinsTypeDef;

/* The brushed DC motor driver module class */
struct DRV8210DSGR_Class {
    const DRV8210DSGR_APIsTypeDef *api;
    DRV8210DSGR_PinsTypeDef *pin;
    uint16_t runMode;
};

/* The brushed DC motor driver module APIs */
struct DRV8210DSGR_APIs {
    void (*init)(DRV8210DSGR_ClassTypeDef *self, DRV8210DSGR_PinsTypeDef *pin);

    void (*run)(DRV8210DSGR_ClassTypeDef *self, uint16_t runMode);
    void (*brake)(DRV8210DSGR_ClassTypeDef *self);
    uint16_t (*getRunMode)(DRV8210DSGR_ClassTypeDef *self);
};

/* The brushed DC motor driver module pins */
struct DRV8210DSGR_Pins {
    GPIO_TypeDef* pinEnablePort;
    uint16_t pinEnablePin;
    GPIO_TypeDef* pinPhasePort;
    uint16_t pinPhasePin;
};

/* For everywhere usage */
extern DRV8210DSGR_ClassTypeDef DRV8210DSGR_1;
extern DRV8210DSGR_ClassTypeDef DRV8210DSGR_2;

#endif /* _DRV8210DSGR_H_ */

/**
 * @author:  trieunvt
 * @file:    stspin220.h
 * @date:    15 Aug 2024
 * @version: v1.0.0
 * @brief:   The STSPIN220 stepper motor driver.
**/

#ifndef _STSPIN220_H_
#define _STSPIN220_H_

#include "stm32l452xx.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os2.h"
#include "math.h"

/* User-defined macros */
#define DEFAULT_STEPS_PER_REV   2048    /* Default steps per revolution when driving the stepper motor in the full-step mode */
#define DEFAULT_STEP_PERIOD     1       /* The period between 2 step clocks (ms) */

/* Developer-defined macros */
#define CLOCKWISE               0
#define COUNTER_CLOCKWISE       1
#define MODE_FULL_STEP          0
#define MODE_1P2_STEP           1
#define MODE_1P4_STEP           2
#define MODE_1P8_STEP           3
#define MODE_1P16_STEP          4
#define MODE_1P32_STEP          5
#define MODE_1P64_STEP          6
#define MODE_1P128_STEP         7
#define MODE_1P256_STEP         8
                                                    //  MODE3   MODE4   MODE1   MODE2
                                                    //   STCK    DIR
static unsigned int StepModeMap[9]   = {0b0000,     //    0       0       0       0     MODE_FULL_STEP
                                        0b1010,     //    1       0       1       0     MODE_1P2_STEP
                                        0b0101,     //    0       1       0       1     MODE_1P4_STEP
                                        0b1011,     //    1       0       1       1     MODE_1P8_STEP
                                        0b1111,     //    1       1       1       1     MODE_1P16_STEP
                                        0b0001,     //    0       0       0       1     MODE_1P32_STEP
                                        0b0111,     //    1       1       1       1     MODE_1P64_STEP
                                        0b0010,     //    0       0       1       0     MODE_1P128_STEP
                                        0b0011};    //    0       0       1       1     MODE_1P256_STEP

/* Struct type definitions for the following usage */
typedef struct STSPIN220_Class  STSPIN220_ClassTypeDef;
typedef struct STSPIN220_APIs   STSPIN220_APIsTypeDef;
typedef struct STSPIN220_Pins   STSPIN220_PinsTypeDef;

/* The stepper motor driver module class */
struct STSPIN220_Class {
    const STSPIN220_APIsTypeDef *api;
    STSPIN220_PinsTypeDef *pin;
    uint16_t stepMode;
    uint32_t stepPeriod;
    uint16_t isRunning;
};

/* The stepper motor driver module APIs */
struct STSPIN220_APIs {
    void (*init)(STSPIN220_ClassTypeDef *self, STSPIN220_PinsTypeDef *pin, uint16_t stepMode);

    void (*setStepMode)(STSPIN220_ClassTypeDef *self, uint16_t stepMode);
    uint16_t (*getStepMode)(STSPIN220_ClassTypeDef *self);

    void (*runSteps)(STSPIN220_ClassTypeDef *self, uint64_t stepNumber, uint16_t runDirection);
    void (*runRevolutions)(STSPIN220_ClassTypeDef *self, uint64_t revNumber, uint16_t runDirection);
    void (*stop)(STSPIN220_ClassTypeDef *self);

    void (*setDirection)(STSPIN220_ClassTypeDef *self, uint16_t runDirection);
    uint16_t (*getDirection)(STSPIN220_ClassTypeDef *self);

    void (*setStepPeriod)(STSPIN220_ClassTypeDef *self, uint32_t stepPeriod);
    uint32_t (*getStepPeriod)(STSPIN220_ClassTypeDef *self);
};

/* The stepper motor driver module pins */
struct STSPIN220_Pins {
    GPIO_TypeDef* pinStandbyResetPort;
    uint16_t pinStandbyResetPin;
    GPIO_TypeDef* pinMode1Port;
    uint16_t pinMode1Pin;
    GPIO_TypeDef* pinMode2Port;
    uint16_t pinMode2Pin;
    GPIO_TypeDef* pinMode3StepClockPort;
    uint16_t pinMode3StepClockPin;
    GPIO_TypeDef* pinMode4DirectionPort;
    uint16_t pinMode4DirectionPin;
};

/* For everywhere usage */
extern STSPIN220_ClassTypeDef STSPIN220_1;
extern STSPIN220_ClassTypeDef STSPIN220_2;

#endif /* _STSPIN220_H_ */

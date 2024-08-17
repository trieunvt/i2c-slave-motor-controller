/**
 * @author:  trieunvt
 * @file:    stspin220.c
 * @date:    15 Aug 2024
 * @version: v1.0.0
 * @brief:   The STSPIN220 stepper motor driver.
**/

#include "stspin220.h"

/* APIs function declarations */
void STSPIN220_init(STSPIN220_ClassTypeDef *self, STSPIN220_PinsTypeDef *pin, uint16_t stepMode);

void STSPIN220_setStepMode(STSPIN220_ClassTypeDef *self, uint16_t stepMode);
uint16_t STSPIN220_getStepMode(STSPIN220_ClassTypeDef *self);

void STSPIN220_runSteps(STSPIN220_ClassTypeDef *self, uint64_t stepNumber, uint16_t runDirection);
void STSPIN220_runRevolutions(STSPIN220_ClassTypeDef *self, uint64_t revNumber, uint16_t runDirection);
void STSPIN220_stop(STSPIN220_ClassTypeDef *self);

void STSPIN220_setDirection(STSPIN220_ClassTypeDef *self, uint16_t runDirection);
uint16_t STSPIN220_getDirection(STSPIN220_ClassTypeDef *self);

void STSPIN220_setStepPeriod(STSPIN220_ClassTypeDef *self, uint32_t stepPeriod);
uint32_t STSPIN220_getStepPeriod(STSPIN220_ClassTypeDef *self);

/* The stepper motor driver module APIs declaration */
static STSPIN220_APIsTypeDef STSPIN220_1_APIs = {
    .init           = STSPIN220_init,

    .setStepMode    = STSPIN220_setStepMode,
    .getStepMode    = STSPIN220_getStepMode,

    .runSteps       = STSPIN220_runSteps,
    .runRevolutions = STSPIN220_runRevolutions,
    .stop           = STSPIN220_stop,

    .setDirection   = STSPIN220_setDirection,
    .getDirection   = STSPIN220_getDirection,

    .setStepPeriod  = STSPIN220_setStepPeriod,
    .getStepPeriod  = STSPIN220_getStepPeriod,
};

static STSPIN220_APIsTypeDef STSPIN220_2_APIs = {
    .init           = STSPIN220_init,

    .setStepMode    = STSPIN220_setStepMode,
    .getStepMode    = STSPIN220_getStepMode,

    .runSteps       = STSPIN220_runSteps,
    .runRevolutions = STSPIN220_runRevolutions,
    .stop           = STSPIN220_stop,

    .setDirection   = STSPIN220_setDirection,
    .getDirection   = STSPIN220_getDirection,

    .setStepPeriod  = STSPIN220_setStepPeriod,
    .getStepPeriod  = STSPIN220_getStepPeriod,
};

/* The stepper motor driver module class declaration */
STSPIN220_ClassTypeDef STSPIN220_1 = {
    .api = &STSPIN220_1_APIs,
};

STSPIN220_ClassTypeDef STSPIN220_2 = {
    .api = &STSPIN220_2_APIs,
};

/* Initialize the stepper motor driver module */
void STSPIN220_init(STSPIN220_ClassTypeDef *self, STSPIN220_PinsTypeDef *pin, uint16_t stepMode) {
    self->pin = pin;
    self->stepPeriod = DEFAULT_STEP_PERIOD;

    self->api->setStepMode(self, stepMode);
};

/* Set the step mode of the stepper motor driver module */
void STSPIN220_setStepMode(STSPIN220_ClassTypeDef *self, uint16_t stepMode) {
    self->stepMode = stepMode;
    stepMode = StepModeMap[stepMode];

    HAL_GPIO_WritePin(self->pin->pinStandbyResetPort, self->pin->pinStandbyResetPin, 0);
    osDelay(100);

    HAL_GPIO_WritePin(self->pin->pinMode3StepClockPort, self->pin->pinMode3StepClockPin, stepMode >> 3 & 0b1);
    HAL_GPIO_WritePin(self->pin->pinMode4DirectionPort, self->pin->pinMode4DirectionPin, stepMode >> 2 & 0b1);
    HAL_GPIO_WritePin(self->pin->pinMode1Port, self->pin->pinMode1Pin, stepMode >> 1 & 0b1);
    HAL_GPIO_WritePin(self->pin->pinMode2Port, self->pin->pinMode2Pin, stepMode & 0b1);
    osDelay(100);
};

/* Get the step mode of the stepper motor driver module */
uint16_t STSPIN220_getStepMode(STSPIN220_ClassTypeDef *self) {
    return self->stepMode;
};

/* Run the stepper motor driver module in the step mode */
void STSPIN220_runSteps(STSPIN220_ClassTypeDef *self, uint64_t stepNumber, uint16_t runDirection) {
    self->isRunning = 1;

    HAL_GPIO_WritePin(self->pin->pinStandbyResetPort, self->pin->pinStandbyResetPin, 1);
    osDelay(100);
    HAL_GPIO_WritePin(self->pin->pinMode4DirectionPort, self->pin->pinMode4DirectionPin, runDirection);

    for (uint64_t i = 0; i < stepNumber; ++i) {
        HAL_GPIO_WritePin(self->pin->pinMode3StepClockPort, self->pin->pinMode3StepClockPin, 1);
        osDelay(self->stepPeriod);

        /* For testing */
        // if (i == DEFAULT_STEPS_PER_REV) {
        //     self->api->stop(self);                  /* Stop running */
        //     self->api->setStepPeriod(self, 1000);   /* Set the new step period */
        // }

        HAL_GPIO_WritePin(self->pin->pinMode3StepClockPort, self->pin->pinMode3StepClockPin, 0);
        osDelay(self->stepPeriod);
    }

    self->isRunning = 0;
};

/* Run the stepper motor driver module in the revolution mode */
void STSPIN220_runRevolutions(STSPIN220_ClassTypeDef *self, uint64_t revNumber, uint16_t runDirection) {
    self->isRunning = 1;
    uint64_t stepNumber = revNumber * DEFAULT_STEPS_PER_REV * pow(2, self->stepMode);

    HAL_GPIO_WritePin(self->pin->pinStandbyResetPort, self->pin->pinStandbyResetPin, 1);
    osDelay(100);
    HAL_GPIO_WritePin(self->pin->pinMode4DirectionPort, self->pin->pinMode4DirectionPin, runDirection);

    for (uint64_t i = 0; i < stepNumber; ++i) {
        HAL_GPIO_WritePin(self->pin->pinMode3StepClockPort, self->pin->pinMode3StepClockPin, 1);
        osDelay(self->stepPeriod);

        /* For testing */
        // if (i == DEFAULT_STEPS_PER_REV) {
        //     self->api->stop(self);                  /* Stop running */
        //     self->api->setStepPeriod(self, 1000);   /* Set the new step period */
        // }

        HAL_GPIO_WritePin(self->pin->pinMode3StepClockPort, self->pin->pinMode3StepClockPin, 0);
        osDelay(self->stepPeriod);
    }

    self->isRunning = 0;
};

/* Stop the stepper motor driver module */
void STSPIN220_stop(STSPIN220_ClassTypeDef *self) {
    self->api->setStepMode(self, self->stepMode);
    self->isRunning = 0;
};

/* Set the direction of the stepper motor driver module */
void STSPIN220_setDirection(STSPIN220_ClassTypeDef *self, uint16_t runDirection) {
    HAL_GPIO_WritePin(self->pin->pinMode4DirectionPort, self->pin->pinMode4DirectionPin, runDirection);
    osDelay(10);
};

/* Get the direction of the stepper motor driver module */
uint16_t STSPIN220_getDirection(STSPIN220_ClassTypeDef *self) {
    return HAL_GPIO_ReadPin(self->pin->pinMode4DirectionPort, self->pin->pinMode4DirectionPin);
};

/* Set the step period of the stepper motor driver module */
void STSPIN220_setStepPeriod(STSPIN220_ClassTypeDef *self, uint32_t stepPeriod) {
    self->stepPeriod = (stepPeriod > 1) ? stepPeriod : 1;
};

/* Get the step period of the stepper motor driver module */
uint32_t STSPIN220_getStepPeriod(STSPIN220_ClassTypeDef *self) {
    return self->stepPeriod;
};

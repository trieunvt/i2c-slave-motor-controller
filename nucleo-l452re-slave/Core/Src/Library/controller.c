/**
 * @author:  trieunvt
 * @file:    controller.c
 * @date:    16 Aug 2024
 * @version: v1.0.0
 * @brief:   The STM32 NUCLEO-L452RE I2C slave motor controller.
**/

#include "controller.h"

/* User-defined macros */
#define MESSAGE_SIZE    1024
#define RECEIVE_TIMEOUT 2000    // ms

/* Variable definitions */
LLQueueTypeDef *queue;
static int isHardwareReady;
char message[MESSAGE_SIZE];

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/* Initial pins declarations */
STSPIN220_PinsTypeDef Pin1 = {
    .pinStandbyResetPort    = GPIOB,
    .pinStandbyResetPin     = GPIO_PIN_4,
    .pinMode1Port           = GPIOA,
    .pinMode1Pin            = GPIO_PIN_3,
    .pinMode2Port           = GPIOA,
    .pinMode2Pin            = GPIO_PIN_2,
    .pinMode3StepClockPort  = GPIOB,
    .pinMode3StepClockPin   = GPIO_PIN_3,
    .pinMode4DirectionPort  = GPIOB,
    .pinMode4DirectionPin   = GPIO_PIN_5,
};

STSPIN220_PinsTypeDef Pin2 = {
    .pinStandbyResetPort    = GPIOB,
    .pinStandbyResetPin     = GPIO_PIN_4,
    .pinMode1Port           = GPIOA,
    .pinMode1Pin            = GPIO_PIN_3,
    .pinMode2Port           = GPIOA,
    .pinMode2Pin            = GPIO_PIN_2,
    .pinMode3StepClockPort  = GPIOB,
    .pinMode3StepClockPin   = GPIO_PIN_3,
    .pinMode4DirectionPort  = GPIOB,
    .pinMode4DirectionPin   = GPIO_PIN_5,
};

void HardwareInit(void) {
    /* Initialize the motor driver modules */
    STSPIN220_1.api->init(&STSPIN220_1, &Pin1, MODE_FULL_STEP);
    osDelay(100);
    STSPIN220_2.api->init(&STSPIN220_2, &Pin2, MODE_FULL_STEP);
    osDelay(100);

    isHardwareReady = 1;
};

void ControllerTaskInit(void) {
    print("< I2C Slave Motor Controller >\n\n");

    HardwareInit();
    osDelay(100);

    /* Create an empty linked list queue for handling messages */
    queue = createLLQueue();
    osDelay(100);
};

void printData(LLQueueDataTypeDef *data) {
    print("## Message Data ##\n");
    print("## motor: %d\n", data->motorIndex);
    print("## mode: %d\n", data->runMode);
    print("## step: %d\n", data->stepNumber);
    print("## revolution: %d\n", data->revNumber);
    print("## direction: %d\n", data->runDirection);
    print("## period: %d\n", data->runPeriod);
    osDelay(1000);
};

/* JSON parser */
int parse_json(const char *p_json, const char *p_name, char *p_value) {
    /* Check for existing JSON, name, and value strings */
    if (NULL == p_json || NULL == p_name || NULL == p_value) return -1;

    char *p_writer, *p_anchor;
    p_writer = p_value;

    /* Find the name if exist */
    p_anchor = strstr(p_json, p_name);
    if (NULL == p_anchor) return -1;

    /* Write the value to the output buffer */
    p_anchor = p_anchor + strlen(p_name) + 1;
    while (NULL != p_anchor && ',' != *p_anchor && '}' != *p_anchor) {
        if(NULL == strchr(": \r\n\t", *p_anchor)) {
            *p_writer = *p_anchor;
            ++p_writer;
        }

        ++p_anchor;
    }
    *p_writer = 0;

    return 1;
}

int parseMessage(const char *pMessage, LLQueueDataTypeDef *data) {
    char parser[1024] = {0};

    /* Parse motor */
    if (parse_json(pMessage, "\"motor\"", parser) < 0) return -1;
    data->motorIndex = atoi(parser);

    /* Parse mode */
    if (parse_json(pMessage, "\"mode\"", parser) < 0) return -1;

    if (!strcmp(parser, "\"full-step\"")) data->runMode = MODE_FULL_STEP;
    else if (!strcmp(parser, "\"1p2-step\"")) data->runMode = MODE_1P2_STEP;
    else if (!strcmp(parser, "\"1p4-step\"")) data->runMode = MODE_1P4_STEP;
    else if (!strcmp(parser, "\"1p8-step\"")) data->runMode = MODE_1P8_STEP;
    else if (!strcmp(parser, "\"1p16-step\"")) data->runMode = MODE_1P16_STEP;
    else if (!strcmp(parser, "\"1p32-step\"")) data->runMode = MODE_1P32_STEP;
    else if (!strcmp(parser, "\"1p64-step\"")) data->runMode = MODE_1P64_STEP;
    else if (!strcmp(parser, "\"1p128-step\"")) data->runMode = MODE_1P128_STEP;
    else if (!strcmp(parser, "\"1p256-step\"")) data->runMode = MODE_1P256_STEP;
    else return -1;

    /* Parse step or revolution */
    if (parse_json(pMessage, "\"step\"", parser) > 0) {
        data->stepNumber = atoi(parser);
        data->revNumber = 0;
    } else if (parse_json(pMessage, "\"revolution\"", parser) > 0) {
        data->stepNumber = 0;
        data->revNumber = atoi(parser);
    } else return -1;

    /* Parse direction */
    if (parse_json(pMessage, "\"direction\"", parser) < 0) return -1;

    if (!strcmp(parser, "\"clockwise\"")) data->runDirection = CLOCKWISE;
    else if (!strcmp(parser, "\"counter-clockwise\"")) data->runDirection = COUNTER_CLOCKWISE;
    else return -1;

    /* Parse period */
    if (parse_json(pMessage, "period", parser) < 0) return -1;
    data->runPeriod = atoi(parser);

    return 1;
};

void ControllerTaskLoop(void) {
    /* For testing */
    // static int times;
    // if (times < 2) {
    //     print("Times: %d\n", times);

    //     if (times % 2) {
    //         sprintf(message, "{\"motor\": 1,\"mode\": \"1p2-step\",\"revolution\": 1,\"direction\": \"clockwise\",\"period\": 1,}");
    //         osDelay(100);
    //     } else {
    //         sprintf(message, "{\"motor\": 1,\"mode\": \"full-step\",\"step\": 2048,\"direction\": \"counter-clockwise\",\"period\": 1,}");
    //         osDelay(100);
    //     }
    //     ++times;
    // }

    HAL_I2C_Slave_Receive(&hi2c1, (uint8_t*) message, MESSAGE_SIZE, RECEIVE_TIMEOUT);
    // HAL_I2C_Slave_Receive(&hi2c2, (uint8_t*) message, MESSAGE_SIZE, RECEIVE_TIMEOUT);
    print("Message: %s\n", message);
    osDelay(2000);

    /* Store the message data to the queue */
    LLQueueDataTypeDef data;
    if (parseMessage(message, &data) > 0) {
        enQueue(queue, &data);
    }

    memset(message, 0, MESSAGE_SIZE);
};

void Motor1TaskLoop(void) {
    if (isHardwareReady && !isEmpty(queue)) {
        if (queue->front->data->motorIndex == 1) {
            LLQueueDataTypeDef data;
            memcpy(&data, queue->front->data, sizeof(LLQueueDataTypeDef));
            deQueue(queue);

            STSPIN220_1.api->setStepMode(&STSPIN220_1, data.runMode);
            STSPIN220_1.api->setStepPeriod(&STSPIN220_1, data.runPeriod);
            if (data.stepNumber > 0) {
                STSPIN220_1.api->runSteps(&STSPIN220_1, data.stepNumber, data.runDirection);
            } else if (data.revNumber > 0) {
                STSPIN220_1.api->runRevolutions(&STSPIN220_1, data.revNumber, data.runDirection);
            }
        }
    }

    osDelay(100);
};

void Motor2TaskLoop(void) {
    if (isHardwareReady && !isEmpty(queue)) {
        if (queue->front->data->motorIndex == 2) {
            LLQueueDataTypeDef data;
            memcpy(&data, queue->front->data, sizeof(LLQueueDataTypeDef));
            deQueue(queue);

            STSPIN220_2.api->setStepMode(&STSPIN220_2, data.runMode);
            STSPIN220_2.api->setStepPeriod(&STSPIN220_2, data.runPeriod);
            if (data.stepNumber > 0) {
                STSPIN220_2.api->runSteps(&STSPIN220_2, data.stepNumber, data.runDirection);
            } else if (data.revNumber > 0) {
                STSPIN220_2.api->runRevolutions(&STSPIN220_2, data.revNumber, data.runDirection);
            }
        }
    }

    osDelay(100);
};

/**
 * @author:  trieunvt
 * @file:    linked_list_queue.h
 * @date:    29 Jul 2024
 * @version: v1.0.0
 * @brief:   Linked list queue
**/

#ifndef _LINKED_LIST_QUEUE_H_
#define _LINKED_LIST_QUEUE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* User-defined macros */
#define STRING_DATA_SIZE 1024

/* Struct data type definitions for the following usage */
typedef struct LLQueue      LLQueueTypeDef;
typedef struct LLQueueNode  LLQueueNodeTypeDef;
typedef struct LLQueueData  LLQueueDataTypeDef;

/* The linked list queue */
struct LLQueue {
    LLQueueNodeTypeDef *front, *rear;
};

/* The linked list queue node to store the queue entry */
struct LLQueueNode {
    LLQueueNodeTypeDef *next;
    LLQueueDataTypeDef *data;
};

/* The linked list queue data defined by user */
struct LLQueueData {
    int motorIndex;
	int runMode;
    unsigned long stepNumber;
    int revNumber;
    int runDirection;
    unsigned long runPeriod;
};

/* For everywhere usage */
LLQueueTypeDef* createLLQueue(void);
int isEmpty(LLQueueTypeDef *queue);
void enQueue(LLQueueTypeDef *queue, LLQueueDataTypeDef *data);
void deQueue(LLQueueTypeDef *queue);

#endif /* _LINKED_LIST_QUEUE_H_ */

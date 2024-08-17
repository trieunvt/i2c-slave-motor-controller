/**
 * @author:  trieunvt
 * @file:    linked_list_queue.c
 * @date:    29 Jul 2024
 * @version: v1.0.0
 * @brief:   Linked list queue
**/

#include "linked_list_queue.h"

/* Create the empty linked list queue */
LLQueueTypeDef* createLLQueue(void) {
    LLQueueTypeDef *queue = (LLQueueTypeDef*) malloc(sizeof(LLQueueTypeDef));
    queue->front = queue->rear = NULL;
    return queue;
}

/* Check the linked list queue is empty (when rear node is NULL) or not */
int isEmpty(LLQueueTypeDef *queue) {
    return (NULL == queue->rear);
}

/* Add data to the linked list queue */
void enQueue(LLQueueTypeDef *queue, LLQueueDataTypeDef *input) {
    /* Create the new linked list queue node data */
    LLQueueDataTypeDef *data = (LLQueueDataTypeDef*) malloc(sizeof(LLQueueDataTypeDef));
    memcpy(data, input, sizeof(LLQueueDataTypeDef));

    /* Create the new linked list queue node */
    LLQueueNodeTypeDef *node = (LLQueueNodeTypeDef*) malloc(sizeof(LLQueueNodeTypeDef));
    node->data = data;
    node->next = NULL;

    /* If the linked list queue is empty, then the new node is both front and rear */
    if (NULL == queue->rear) {
        queue->front = queue->rear = node;
        return;
    }

    /* Add the new node at the end of the linked list queue and change the rear node */
    queue->rear->next = node;
    queue->rear = node;
}

/* Remove data from the linked list queue */
void deQueue(LLQueueTypeDef *queue) {
    /* If the linked list queue is empty, return NULL */
    if (NULL == queue->front) return;

    /* Store the previous front node and move the front node one step ahead */
    LLQueueNodeTypeDef* temp = queue->front;
    queue->front = queue->front->next;

    /* If the front node becomes NULL, then change the rear node as NULL also */
    if (queue->front == NULL) queue->rear = NULL;

    free(temp->data);
    free(temp);
}

#ifndef LED_H
#define LED_H

#define LED_TASK_NAME "led_task"
#define LED_TASK_PRIORITY 4
#define LED_TASK_STACK_SIZE (64)
#define LED_TASK_DELAY 300 

void led_init();

#endif
#ifndef LED_H
#define LED_H

#define LED_TASK_NAME "led_task"
#define LED_TASK_PRIORITY 2
#define LED_TASK_STACK_SIZE 32
#define LED_TASK_DELAY 300

void led_init();

#endif
#ifndef PSRAM_TASK_H
#define PSRAM_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"

/* Create a task whose stack lives in PSRAM (the TCB stays in internal RAM).
 * `stack`/`tcb` are owned by the caller and must stay valid for the task's
 * whole life; the stack is allocated on first use and then reused, so a task
 * that exits can be re-created later (wait for it to be fully deleted first). */
static inline TaskHandle_t psram_task_create(TaskFunction_t fn, const char *name, uint32_t stack_bytes,
                                             void *arg, UBaseType_t prio, BaseType_t core,
                                             StackType_t **stack, StaticTask_t *tcb)
{
    if (*stack == NULL) {
        *stack = heap_caps_malloc(stack_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (*stack == NULL) {
            return NULL;
        }
    }
    return xTaskCreateStaticPinnedToCore(fn, name, stack_bytes / sizeof(StackType_t), arg, prio,
                                         *stack, tcb, core);
}

#endif /* PSRAM_TASK_H */

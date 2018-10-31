
#ifndef APP_SCHEDULER_H__
#define APP_SCHEDULER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CEIL_DIV(A, B)      \
	(((A) + (B) - 1) / (B))

#define APP_SCHED_EVENT_HEADER_SIZE 8

#define APP_SCHED_BUF_SIZE(EVENT_SIZE, QUEUE_SIZE)                                                 \
	(((EVENT_SIZE) + APP_SCHED_EVENT_HEADER_SIZE) * ((QUEUE_SIZE) + 1))

typedef void (*app_sched_event_handler_t)(void* p_event_data, uint16_t event_size);

#define APP_SCHED_INIT(EVENT_SIZE, QUEUE_SIZE)                                                     \
	do                                                                                             \
	{                                                                                              \
		static uint32_t APP_SCHED_BUF[CEIL_DIV(APP_SCHED_BUF_SIZE((EVENT_SIZE), (QUEUE_SIZE)),     \
		                                                                                 sizeof(uint32_t))];                                 \
		uint32_t ERR_CODE = app_sched_init((EVENT_SIZE), (QUEUE_SIZE), APP_SCHED_BUF);             \
	} while (0)

uint32_t app_sched_init(uint16_t max_event_size, uint16_t queue_size, void* p_evt_buffer);

void app_sched_execute(void);

uint32_t app_sched_event_put(void*                     p_event_data,
                             uint16_t                  event_size,
                             app_sched_event_handler_t handler);

uint16_t app_sched_queue_utilization_get(void);

uint16_t app_sched_queue_space_get(void);

void app_sched_pause(void);

void app_sched_resume(void);

// 调度器语法糖
#define platform_simple_evt_put(handler) app_sched_event_put(NULL,0,(app_sched_event_handler_t)handler)

#ifdef __cplusplus
}
#endif

#endif // APP_SCHEDULER_H__

/** @} */

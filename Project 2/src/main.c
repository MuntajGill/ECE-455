// Team Member Owen de Groot and Muntaj Gill
// Student IDs: V00962387
// FULLY WORKING CODE - MINUS SAME TASK TIMING LIMITATION for T1 and T2

/* RTOS includes - Standard libraries + FreeRTOS middleware */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

/*-----------------------------------*/

// Four distinct levels so there are no ties between DDS, generator, and user tasks.
// DDS must always be strictly highest so it pre-empts everything when resumed.
#define DDS_PRIORITY      4
#define HIGH_PRIORITY     3   // reserved for EDF winner (not used in step 2)
#define MEDIUM_PRIORITY   2   // generator
#define LOW_PRIORITY      1   // future user tasks

/*-----------------------------------*/

// Test Bench Selection — uncomment ONE bench at a time.
// Comment out the other two before building.

/*-------------- Test Bench #1 ----------------*/
// Hyper-period: 1500ms
// Utilisation: (95/500) + (150/500) + (250/750) = 0.856 — schedulable

//#define T1_EXEC_MS    95
//#define T1_PERIOD_MS  500
//
//#define T2_EXEC_MS    150
//#define T2_PERIOD_MS  500
//
//#define T3_EXEC_MS    250
//#define T3_PERIOD_MS  750

/*-------------- Test Bench #2 ----------------*/
// Hyper-period: 1500ms
// Utilisation: (95/250) + (150/500) + (250/750) = 0.713 — schedulable

//#define T1_EXEC_MS    95
//#define T1_PERIOD_MS  250
//
//#define T2_EXEC_MS    150
//#define T2_PERIOD_MS  500
//
//#define T3_EXEC_MS    250
//#define T3_PERIOD_MS  750

/*-------------- Test Bench #3 ----------------*/
// Hyper-period: 500ms
// Utilisation: (100/500) + (200/500) + (200/500) = 1.0 — boundary case

#define T1_EXEC_MS    100 // 100
#define T1_PERIOD_MS  500

#define T2_EXEC_MS    200 // 200
#define T2_PERIOD_MS  500

#define T3_EXEC_MS    200 // 200
#define T3_PERIOD_MS  500

/*-----------------------------------*/

typedef enum { PERIODIC, APERIODIC } task_type;

typedef struct {
    TaskHandle_t t_handle;
    task_type    type;
    uint32_t     task_id;
    uint32_t     release_time;
    uint32_t     absolute_deadline;
    uint32_t     completion_time;
} dd_task;

// linked list node for dd tasks
typedef struct dd_task_list {
    dd_task             task;
    struct dd_task_list *next_task;
} dd_task_list;

// message types for DDS queue
typedef enum {
    RELEASE_TASK,
    COMPLETE_TASK,
	GET_ACTIVE_LIST,
	GET_COMPLETED_LIST,
	GET_OVERDUE_LIST,
	CHECK_OVERDUE			// sent internally by overdue timer callback
} msg_type;

typedef struct {
    msg_type type;
    dd_task  task;
    QueueHandle_t reply_queue;		// prevents deadlock by stopping acks from being stolen from xResponseQueue
} dd_message;

/*-----------------------------------*/

// Queue handles - inter-task messaging
QueueHandle_t xDDSQueue;			// incoming messages to DDS
//QueueHandle_t xResponseQueue;		// DDS responses
QueueHandle_t xPendingReleaseQueue;	// timer -> generator: which tasks to release

// F-Task handles
TaskHandle_t xDDS_handle;
TaskHandle_t xGenerator_handle;

// User-defined tasks
TaskHandle_t xUserTask1_handle;
TaskHandle_t xUserTask2_handle;
TaskHandle_t xUserTask3_handle;

// User-defined task queues - communicating current taskID instance
QueueHandle_t xTask1IDQueue;
QueueHandle_t xTask2IDQueue;
QueueHandle_t xTask3IDQueue;

// Timer handles - one timer per user task
TimerHandle_t xTimer1;
TimerHandle_t xTimer2;
TimerHandle_t xTimer3;

// Monitor fires every 333ms (~3x per second)
#define MONITOR_PERIOD_MS  333
TimerHandle_t xMonitorTimer;
TaskHandle_t  xMonitor_handle;

// Per-task instance counter
static uint32_t task_id = 0;

/*-----------------------------------*/

// Forward declarations F-tasks
void DDS(void *pvParameters);
void DD_TaskGenerator(void *pvParameters);
void Monitor_Task(void *pvParameters);

// User-defined task declarations
void User_Task1(void *pvParameters);
void User_Task2(void *pvParameters);
void User_Task3(void *pvParameters);

// DDS interface
void create_dd_task(TaskHandle_t t_handle, task_type type, uint32_t task_id, uint32_t absolute_deadline);
void delete_dd_task(uint32_t task_id);

// List helpers
dd_task_list* add_task_to_list(dd_task_list **head, dd_task new_task);
dd_task_list* move_task_between_lists(dd_task_list **src_head, dd_task_list **dst_head, uint32_t task_id);
dd_task_list* remove_task_from_list(dd_task_list **head, uint32_t task_id);
dd_task_list* merge_sort_task_list(dd_task_list *head);

// Timer callbacks
void vTimerCallback(TimerHandle_t xTimer);
void vMonitorTimerCallback(TimerHandle_t xTimer);

// Monitor task helpers
dd_task_list* get_active_dd_task_list(void);
dd_task_list* get_complete_dd_task_list(void);
dd_task_list* get_overdue_dd_task_list(void);

/*-----------------------------------*/

// Main initializations
void init_Queues(void);
void init_FTasks(void);
void init_MonitorTimer(void);

/*-----------------------------------*/

int main(void)
{
	init_Queues();

	init_FTasks();

	init_MonitorTimer();

    vTaskStartScheduler();
    for (;;) { }
}

/*-----------------------------------*/

void init_Queues(void)
{
	xDDSQueue      = xQueueCreate(10, sizeof(dd_message));
//	xResponseQueue = xQueueCreate(10, sizeof(uint32_t));
	xPendingReleaseQueue = xQueueCreate(10, sizeof(uint32_t));

	//TASKID QUEUES
	xTask1IDQueue = xQueueCreate(5, sizeof(uint32_t));
	xTask2IDQueue = xQueueCreate(5, sizeof(uint32_t));
	xTask3IDQueue = xQueueCreate(5, sizeof(uint32_t));

}

/*-----------------------------------*/

void init_FTasks(void)
{
	// User-defined tasks
	xTaskCreate(User_Task1, "UserTask1", 256, NULL, LOW_PRIORITY, &xUserTask1_handle);
//	vTaskSuspend(xUserTask1_handle);
	xTaskCreate(User_Task2, "UserTask2", 256, NULL, LOW_PRIORITY, &xUserTask2_handle);
//	vTaskSuspend(xUserTask2_handle);
	xTaskCreate(User_Task3, "UserTask3", 256, NULL, LOW_PRIORITY, &xUserTask3_handle);
//	vTaskSuspend(xUserTask3_handle);

	xTaskCreate(Monitor_Task,    "Monitor",   256, NULL, LOW_PRIORITY,    &xMonitor_handle);

	// DDS created first and at highest priority — it will pre-empt
	// the generator the moment the generator sends a message.
	xTaskCreate(DD_TaskGenerator, "Generator",256, NULL, MEDIUM_PRIORITY, &xGenerator_handle);

	xTaskCreate(DDS,              "DDS",      512, NULL, DDS_PRIORITY,    &xDDS_handle);

	// Timer ID encodes task number so the shared callback knows which one fired
	xTimer1 = xTimerCreate("T1_Timer", pdMS_TO_TICKS(T1_PERIOD_MS), pdTRUE, (void*)1, vTimerCallback);
	xTimer2 = xTimerCreate("T2_Timer", pdMS_TO_TICKS(T2_PERIOD_MS), pdTRUE, (void*)2, vTimerCallback);
	xTimer3 = xTimerCreate("T3_Timer", pdMS_TO_TICKS(T3_PERIOD_MS), pdTRUE, (void*)3, vTimerCallback);

}

/*-----------------------------------*/

void init_MonitorTimer(void)
{
	xMonitorTimer = xTimerCreate("Mon_Timer", pdMS_TO_TICKS(MONITOR_PERIOD_MS),
		                              pdTRUE, NULL, vMonitorTimerCallback);
	xTimerStart(xMonitorTimer, 0);
}

/*-----------------------------------*/

/* Shared timer callback — runs in timer daemon context.
   Pushes task number to queue and wakes the generator. */
void vTimerCallback(TimerHandle_t xTimer)
{
	uint32_t task_num = (uint32_t)pvTimerGetTimerID(xTimer);

	xQueueSend(xPendingReleaseQueue, &task_num, 0);
	vTaskResume(xGenerator_handle);
}

/*-----------------------------------*/

void vMonitorTimerCallback(TimerHandle_t xTimer)
{
	(void)xTimer;  // only one monitor task so no need to check ID
	vTaskResume(xMonitor_handle);
}


/*-----------------------------------*/

void DD_TaskGenerator(void *pvParameters)
{

		xTimerStart(xTimer1, 0);
		xTimerStart(xTimer2, 0);
		xTimerStart(xTimer3, 0);

		uint32_t task_num;

		for (;;)
		{
			// drain the queue in case multiple timers fired before we ran
			while (xQueueReceive(xPendingReleaseQueue, &task_num, 0) == pdTRUE)
			{
				TickType_t now = xTaskGetTickCount();

				if (task_num == 1)
				{
					task_id++;
					uint32_t deadline = now + pdMS_TO_TICKS(T1_PERIOD_MS);
					xQueueSend(xTask1IDQueue, &task_id, portMAX_DELAY);					// Pushes TaskID instance to queue
					create_dd_task(xUserTask1_handle, PERIODIC, task_id, deadline);
				}
				else if (task_num == 2)
				{
					task_id++;
					uint32_t deadline = now + pdMS_TO_TICKS(T2_PERIOD_MS);
					xQueueSend(xTask2IDQueue, &task_id, portMAX_DELAY);
					create_dd_task(xUserTask2_handle, PERIODIC, task_id, deadline);
				}
				else if (task_num == 3)
				{
					task_id++;
					uint32_t deadline = now + pdMS_TO_TICKS(T3_PERIOD_MS);
					xQueueSend(xTask3IDQueue, &task_id, portMAX_DELAY);
					create_dd_task(xUserTask3_handle, PERIODIC, task_id, deadline);
				}
			}
			vTaskSuspend(NULL);
		}
}

/*-----------------------------------*/

/* DDS — blocks on xDDSQueue waiting for messages from the interface functions.
   Manages active, completed, and overdue lists internally.
   EDF is applied by setting the earliest-deadline task to HIGH priority
   and everything else to LOW after any list change. */
void DDS(void *pvParameters)
{
    dd_task_list *active_list = NULL;
    dd_task_list *completed_list = NULL;
    dd_task_list *overdue_list = NULL;

    dd_message msg;
    uint32_t   ack = 1;

    for (;;)
    {
        // Block until a message arrives (put here after first resume)
        xQueueReceive(xDDSQueue, &msg, portMAX_DELAY);

        TickType_t now = xTaskGetTickCount();

        switch (msg.type)
        {
            	case RELEASE_TASK:
            	{
            			msg.task.release_time = (uint32_t)now;

            			// Suspend and demote current EDF head before reordering
//            			if (active_list != NULL)
//            			{
//            					vTaskSuspend(active_list->task.t_handle);
//            					vTaskPrioritySet(active_list->task.t_handle, LOW_PRIORITY);
//            			}

            			add_task_to_list(&active_list, msg.task);
            			active_list = merge_sort_task_list(active_list);

            			// Apply EDF priorities - head = HIGH_PRIORITY, rest get LOW_PRIORITY
            			dd_task_list *curr = active_list;
            			int first = 1;
            			while (curr != NULL)
            			{
            					if (first)
            					{
            							vTaskPrioritySet(curr->task.t_handle, HIGH_PRIORITY);
            							vTaskResume(curr->task.t_handle);
            							first = 0;
            					}
            					else
            					{
            							vTaskPrioritySet(curr->task.t_handle, LOW_PRIORITY);
            							// vTaskResume(curr->task.t_handle);
            					}
            					curr = curr->next_task;
            			}

            			printf("[R] ID=%u t=%u\n",
            					(unsigned int)msg.task.task_id,
								(unsigned int)now);

            			xQueueSend(msg.reply_queue, &ack, portMAX_DELAY);
            			break;
            	}

            	case COMPLETE_TASK:
            	{
            			// Find the completing task and stamp its completion time
            			dd_task_list *complete = active_list;
            			while (complete != NULL)
            			{
            					if (complete->task.task_id == msg.task.task_id)
            					{
            							vTaskSuspend(complete->task.t_handle);
            							// vTaskPrioritySet(complete->task.t_handle, LOW_PRIORITY);
            							complete->task.completion_time = (uint32_t)now;
            							break;
            					}
            					complete = complete->next_task;
            			}

            			// STEP 5 - move to overdue list
            			if (complete != NULL)
            			{
            				if (complete->task.completion_time <= complete->task.absolute_deadline)
            				{
            					move_task_between_lists(&active_list, &completed_list, msg.task.task_id);
            					printf("[C] ID=%u t=%u\n", (unsigned int)msg.task.task_id, (unsigned int)now);
            				}
            				else
            				{
            					move_task_between_lists(&active_list, &overdue_list, msg.task.task_id);
            					printf("[OD] ID=%u t=%u\n", (unsigned int)msg.task.task_id, (unsigned int)now);
            				}
            			}

            			active_list = merge_sort_task_list(active_list);

            			// Re-apply EDF to remaining tasks
            			dd_task_list *curr = active_list;
            			int first = 1;
            			while (curr != NULL)
            			{
            					if (first)
            					{
            							vTaskPrioritySet(curr->task.t_handle, HIGH_PRIORITY);
            							vTaskResume(curr->task.t_handle);
            							first = 0;
            					}
            					else
            					{
            							vTaskPrioritySet(curr->task.t_handle, LOW_PRIORITY);
            							// vTaskResume(curr->task.t_handle);
            					}
            					curr = curr->next_task;
            			}

            			xQueueSend(msg.reply_queue, &ack, portMAX_DELAY);
            			break;
            	}

            	case GET_ACTIVE_LIST:
            		xQueueSend(msg.reply_queue, &active_list, portMAX_DELAY);
            		break;

            	case GET_COMPLETED_LIST:
            		xQueueSend(msg.reply_queue, &completed_list, portMAX_DELAY);
            		break;

            	case GET_OVERDUE_LIST:
            		xQueueSend(msg.reply_queue, &overdue_list, portMAX_DELAY);
            		break;

            	default:
            		break;
        	}

        	// Suspend until the next interface call resumes us
        	vTaskSuspend(NULL);
	}
}

/*-----------------------------------*/

/* Prints active/completed/overdue counts each time the monitor timer fires. */
void Monitor_Task(void *pvParameters)
{
	for (;;)
	{

		dd_task_list *active    = get_active_dd_task_list();
		dd_task_list *completed = get_complete_dd_task_list();
		dd_task_list *overdue   = get_overdue_dd_task_list();

		int active_count = 0, completed_count = 0, overdue_count = 0;

		dd_task_list *curr = active;
		while (curr != NULL) { active_count++;    curr = curr->next_task; }

		curr = completed;
		while (curr != NULL) { completed_count++; curr = curr->next_task; }

		curr = overdue;
		while (curr != NULL) { overdue_count++;   curr = curr->next_task; }

		printf("[MON] t=%u | A=%d | C=%d | OD=%d\n",
		       (unsigned long)xTaskGetTickCount(),
		       active_count, completed_count, overdue_count);

		vTaskSuspend(NULL);
	}
}

/*-----------------------------------*/

void User_Task1(void *pvParameters)
{
	for (;;)
	{
		uint32_t my_id;
		xQueueReceive(xTask1IDQueue, &my_id, portMAX_DELAY);

		TickType_t start = xTaskGetTickCount();
		while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(T1_EXEC_MS)) { }

		printf("[T1] t=%u\n", (unsigned long)xTaskGetTickCount());
		delete_dd_task(my_id);

		// vTaskSuspend(NULL);
	}
}

/*-----------------------------------*/

void User_Task2(void *pvParameters)
{
	for (;;)
	{
		uint32_t my_id;
		xQueueReceive(xTask2IDQueue, &my_id, portMAX_DELAY);

		TickType_t start = xTaskGetTickCount();
		while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(T2_EXEC_MS)) { }

		printf("[T2] t=%u\n", (unsigned long)xTaskGetTickCount());
		delete_dd_task(my_id);

		// vTaskSuspend(NULL);
	}
}

/*-----------------------------------*/

void User_Task3(void *pvParameters)
{
	for (;;)
	{
		uint32_t my_id;
		xQueueReceive(xTask3IDQueue, &my_id, portMAX_DELAY);

		TickType_t start = xTaskGetTickCount();
		while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(T3_EXEC_MS)) { }

		printf("[T3] t=%u\n", (unsigned long)xTaskGetTickCount());
		delete_dd_task(my_id);

		// vTaskSuspend(NULL);
	}
}

/*-----------------------------------*/

// Sends RELEASE_TASK to DDS, resumes it, then blocks waiting for the ack.
// Pattern: caller resumes DDS, DDS
// pre-empts immediately (it's highest priority), processes, sends ack,
// suspends itself, then caller unblocks and returns.
void create_dd_task(TaskHandle_t t_handle, task_type type,
                    uint32_t task_id, uint32_t absolute_deadline)
{
	QueueHandle_t reply_q = xQueueCreate(1, sizeof(uint32_t));

    dd_task task;
    task.t_handle          = t_handle;
    task.type              = type;
    task.task_id           = task_id;
    task.release_time      = 0;   // DDS stamps this
    task.absolute_deadline = absolute_deadline;
    task.completion_time   = 0;

    dd_message message;
    message.type = RELEASE_TASK;
    message.task = task;

    message.reply_queue = reply_q;

    xQueueSend(xDDSQueue, &message, portMAX_DELAY);
    vTaskResume(xDDS_handle);     // wake DDS — it pre-empts us immediately

    uint32_t ack;
    xQueueReceive(reply_q, &ack, portMAX_DELAY);
    vQueueDelete(reply_q);
    // Returns here after DDS has processed the message and suspended itself

}

/*-----------------------------------*/

/* Sends COMPLETE_TASK to DDS, waits for ack.
   Note: do NOT suspend here — DDS handles suspend/resume ordering itself. */
void delete_dd_task(uint32_t task_id)
{
	QueueHandle_t reply_q = xQueueCreate(1, sizeof(uint32_t));

	dd_task task;
	task.t_handle          = NULL;
	task.type              = PERIODIC;  // not used by DDS on completion
	task.task_id           = task_id;
	task.release_time      = 0;
	task.absolute_deadline = 0;
	task.completion_time   = 0;  // DDS stamps this

	dd_message message;
	message.type = COMPLETE_TASK;
	message.task = task;

	message.reply_queue = reply_q;

	xQueueSend(xDDSQueue, &message, portMAX_DELAY);
	vTaskResume(xDDS_handle);

	uint32_t ack;
	xQueueReceive(reply_q, &ack, portMAX_DELAY);

	vQueueDelete(reply_q);


	// don't suspend here - would race with DDS's own vTaskResume call
}


/*-----------------------------------*/

dd_task_list* get_active_dd_task_list(void)
{
	QueueHandle_t reply_q = xQueueCreate(1, sizeof(dd_task_list*));

	dd_message message;
	message.type = GET_ACTIVE_LIST;
	message.reply_queue = reply_q;

	xQueueSend(xDDSQueue, &message, portMAX_DELAY);
	vTaskResume(xDDS_handle);

	dd_task_list *result;
	xQueueReceive(reply_q, &result, portMAX_DELAY);
	vQueueDelete(reply_q);

	return result;
}

/*-----------------------------------*/

dd_task_list* get_complete_dd_task_list(void)
{
	QueueHandle_t reply_q = xQueueCreate(1, sizeof(dd_task_list*));

	dd_message message;
	message.type = GET_COMPLETED_LIST;
	message.reply_queue = reply_q;

	xQueueSend(xDDSQueue, &message, portMAX_DELAY);
	vTaskResume(xDDS_handle);

	dd_task_list *result;
	xQueueReceive(reply_q, &result, portMAX_DELAY);
	vQueueDelete(reply_q);

	return result;
}

/*-----------------------------------*/

dd_task_list* get_overdue_dd_task_list(void)
{
	QueueHandle_t reply_q = xQueueCreate(1, sizeof(dd_task_list*));

	dd_message message;
	message.type = GET_OVERDUE_LIST;
	message.reply_queue = reply_q;

	xQueueSend(xDDSQueue, &message, portMAX_DELAY);
	vTaskResume(xDDS_handle);

	dd_task_list *result;
	xQueueReceive(reply_q, &result, portMAX_DELAY);
	vQueueDelete(reply_q);

	return result;
}

/*-----------------------------------*/

// Appends a new node to the tail of the list.
dd_task_list* add_task_to_list(dd_task_list **head, dd_task new_task)
{
    dd_task_list *node = (dd_task_list*)pvPortMalloc(sizeof(dd_task_list));
    if (node == NULL)
    {
        printf("[ERROR] pvPortMalloc failed in add_task_to_list\n");
        return *head;
    }

    node->task      = new_task;
    node->next_task = NULL;

    if (*head == NULL)
    {
        *head = node;
        return *head;
    }

    dd_task_list *curr = *head;
    while (curr->next_task != NULL)
        curr = curr->next_task;

    curr->next_task = node;
    return *head;
}

/*-----------------------------------*/

/* Moves a node from src to dst without alloc/free.
   Use this instead of remove+add when transitioning between lists. */
dd_task_list* move_task_between_lists(dd_task_list **src_head,
                                      dd_task_list **dst_head,
                                      uint32_t task_id)
{
	if (*src_head == NULL) return NULL;

	dd_task_list *curr = *src_head;
	dd_task_list *prev = NULL;

	while (curr != NULL)
	{
		if (curr->task.task_id == task_id)
		{
			// Unlink from source
			if (prev == NULL)
				*src_head = curr->next_task;
			else
				prev->next_task = curr->next_task;

			curr->next_task = NULL;

			// Append to destination
			if (*dst_head == NULL)
			{
				*dst_head = curr;
			}
			else
			{
				dd_task_list *tail = *dst_head;
				while (tail->next_task != NULL)
					tail = tail->next_task;
				tail->next_task = curr;
			}

			return curr;
		}
		prev = curr;
		curr = curr->next_task;
	}

	return NULL;  // task_id not found
}

/*-----------------------------------*/

/* Removes a node by task_id and frees memory.
   Use move_task_between_lists if the node is going to another list. */
dd_task_list* remove_task_from_list(dd_task_list **head, uint32_t task_id)
{
	if (*head == NULL) return NULL;

	dd_task_list *curr = *head;
	dd_task_list *prev = NULL;

	while (curr != NULL)
	{
		if (curr->task.task_id == task_id)
		{
			if (prev == NULL)
				*head = curr->next_task;  // removing head
			else
				prev->next_task = curr->next_task;

			vPortFree(curr);
			return *head;
		}
		prev = curr;
		curr = curr->next_task;
	}

	return *head;  // not found, list unchanged
}

/*-----------------------------------*/

/* Iterative bottom-up merge sort by absolute_deadline.
   No memory allocation — only relinks existing nodes. */
dd_task_list* merge_sort_task_list(dd_task_list *head)
{
	if (head == NULL || head->next_task == NULL)
		return head;

	int length = 0;
	dd_task_list *curr = head;
	while (curr != NULL) { length++; curr = curr->next_task; }

	dd_task_list dummy;
	dummy.next_task = head;

	for (int size = 1; size < length; size *= 2)
	{
		dd_task_list *tail = &dummy;
		curr = dummy.next_task;

		while (curr != NULL)
		{
			// carve out left sublist of length 'size'
			dd_task_list *left = curr;
			dd_task_list *right = curr;
			int left_len = 0;

			while (left_len < size && right != NULL)
			{
				right = right->next_task;
				left_len++;
			}
			// right now points to start of right sublist (may be NULL)

			int right_len = size;

			while (left_len > 0 && right_len > 0 && right != NULL)
			{
				if (left->task.absolute_deadline <= right->task.absolute_deadline)
				{
					tail->next_task = left;
					left = left->next_task;
					left_len--;
				}
				else
				{
					tail->next_task = right;
					right = right->next_task;
					right_len--;
				}
				tail = tail->next_task;
			}

			// drain whichever side still has nodes
			while (left_len > 0 && left != right)
			{
				tail->next_task = left;
				left = left->next_task;
				left_len--;
				tail = tail->next_task;
			}
			while (right_len > 0 && right != NULL)
			{
				tail->next_task = right;
				right = right->next_task;
				right_len--;
				tail = tail->next_task;
			}

			tail->next_task = NULL;
			curr = right;
		}
	}

	return dummy.next_task;
}

/*-----------------------------------*/

void vApplicationMallocFailedHook(void) { for (;;) { } }

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;
    for (;;) { }
}

void vApplicationIdleHook(void)
{
    volatile size_t available_memory;
    available_memory = xPortGetFreeHeapSize();
    if (available_memory > 100) { /* heap ok */ }
}

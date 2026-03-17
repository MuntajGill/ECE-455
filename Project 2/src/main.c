// Team Member Owen de Groot and Muntaj Gill
// Student IDs

/* Standard libraries - no middleware */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"

/* RTOS includes - FreeRTOS middleware */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

/*-----------------------------------*/

// Task priority levels
#define HIGH_PRIORITY   3
#define MEDIUM_PRIORITY 2
#define LOW_PRIORITY    1
#define IDLE_PRIORITY   0

/*-----------------------------------*/

// Test Bench Selection — uncomment ONE bench at a time.
// Comment out the other two before building.

/*-------------- Test Bench #1 ----------------*/
// Hyper-period: 1500ms
// Utilisation: (95/500) + (150/500) + (250/750) = 0.856 — schedulable

#define T1_EXEC_MS    95
#define T1_PERIOD_MS  500

#define T2_EXEC_MS    150
#define T2_PERIOD_MS  500

#define T3_EXEC_MS    250
#define T3_PERIOD_MS  750

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

//#define T1_EXEC_MS    100
//#define T1_PERIOD_MS  500
//
//#define T2_EXEC_MS    200
//#define T2_PERIOD_MS  500
//
//#define T3_EXEC_MS    200
//#define T3_PERIOD_MS  500

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
    CHECK_OVERDUE       // sent internally by overdue timer callback
} msg_type;

typedef struct {
    msg_type type;
    dd_task  task;
} dd_message;

/*-----------------------------------*/

// Queue handles
QueueHandle_t xDDSQueue;            // incoming messages to DDS
QueueHandle_t xResponseQueue;       // DDS responses
QueueHandle_t xPendingReleaseQueue; // timer -> generator: which task to release

/*-----------------------------------*/

// Task handles — needed so timer callbacks can resume the right tasks
TaskHandle_t xDDS_handle;
TaskHandle_t xDDTaskGen_handle;
TaskHandle_t xUserTask1_handle;
TaskHandle_t xUserTask2_handle;
TaskHandle_t xUserTask3_handle;

// One timer per user task
TimerHandle_t xTimer1;
TimerHandle_t xTimer2;
TimerHandle_t xTimer3;

// Monitor fires every 333ms (~3x per second)
#define MONITOR_PERIOD_MS  333
TimerHandle_t xMonitorTimer;
TaskHandle_t  xMonitor_handle;

// Per-task instance counters
static uint32_t taskID1 = 0;
static uint32_t taskID2 = 0;
static uint32_t taskID3 = 0;

/*-----------------------------------*/

// F-task declarations
void DDS(void *pvParameters);
void DD_TaskGenerator(void *pvParameters);
void Monitor_Task(void *pvParameters);
void List_Test_Task(void *pvParameters);    // development only — disable before test benches

// User task declarations
void User_Task1(void *pvParameters);
void User_Task2(void *pvParameters);
void User_Task3(void *pvParameters);

void vTimerCallback(TimerHandle_t xTimer);
void vMonitorTimerCallback(TimerHandle_t xTimer);

// DDS interface
void create_dd_task(TaskHandle_t t_handle, task_type type, uint32_t task_id, uint32_t absolute_deadline);
void delete_dd_task(uint32_t task_id);

dd_task_list** get_active_dd_task_list(void);
dd_task_list** get_complete_dd_task_list(void);
dd_task_list** get_overdue_dd_task_list(void);

void init_FTasks(void);

/*-----------------------------------*/

// List helpers
dd_task_list* add_task_to_list(dd_task_list **head, dd_task new_task);
dd_task_list* move_task_between_lists(dd_task_list **src_head, dd_task_list **dst_head, uint32_t task_id);
dd_task_list* remove_task_from_list(dd_task_list **head, uint32_t task_id);
dd_task_list* merge_sort_task_list(dd_task_list *head);

/*-----------------------------------*/

int main(void)
{
	init_FTasks();
	vTaskStartScheduler();
	for(;;) { }
}

/*-----------------------------------*/

void init_FTasks(void)
{
	xDDSQueue            = xQueueCreate(10, sizeof(dd_message));
	xResponseQueue       = xQueueCreate(10, sizeof(dd_task_list*));
	xPendingReleaseQueue = xQueueCreate(10, sizeof(uint32_t));

	xTaskCreate(DDS,             "DDS",       256, NULL, HIGH_PRIORITY,   &xDDS_handle);
	xTaskCreate(DD_TaskGenerator,"DD_TaskGen",256, NULL, MEDIUM_PRIORITY, &xDDTaskGen_handle);
	xTaskCreate(Monitor_Task,    "Monitor",   256, NULL, LOW_PRIORITY,    &xMonitor_handle);

	// List_Test_Task is for development only — disable before running test benches
	// xTaskCreate(List_Test_Task,  "ListTest",  512, NULL, MEDIUM_PRIORITY, NULL);

	xTaskCreate(User_Task1, "UserTask1", 256, NULL, LOW_PRIORITY, &xUserTask1_handle);
	xTaskCreate(User_Task2, "UserTask2", 256, NULL, LOW_PRIORITY, &xUserTask2_handle);
	xTaskCreate(User_Task3, "UserTask3", 256, NULL, LOW_PRIORITY, &xUserTask3_handle);

	// Suspend everything — timers/DDS will resume as needed
	vTaskSuspend(xDDS_handle);
	vTaskSuspend(xDDTaskGen_handle);
	vTaskSuspend(xMonitor_handle);    // monitor woken by its own timer
	vTaskSuspend(xUserTask1_handle);
	vTaskSuspend(xUserTask2_handle);
	vTaskSuspend(xUserTask3_handle);

	// Timer ID encodes task number so the shared callback knows which one fired
	xTimer1 = xTimerCreate("T1_Timer", pdMS_TO_TICKS(T1_PERIOD_MS), pdTRUE, (void*)1, vTimerCallback);
	xTimer2 = xTimerCreate("T2_Timer", pdMS_TO_TICKS(T2_PERIOD_MS), pdTRUE, (void*)2, vTimerCallback);
	xTimer3 = xTimerCreate("T3_Timer", pdMS_TO_TICKS(T3_PERIOD_MS), pdTRUE, (void*)3, vTimerCallback);

	xTimerStart(xTimer1, 0);
	xTimerStart(xTimer2, 0);
	xTimerStart(xTimer3, 0);

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
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xQueueSendFromISR(xPendingReleaseQueue, &task_num, &xHigherPriorityTaskWoken);
	xTaskResumeFromISR(xDDTaskGen_handle);

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*-----------------------------------*/

void vMonitorTimerCallback(TimerHandle_t xTimer)
{
	(void)xTimer;  // only one monitor task so no need to check ID
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskResumeFromISR(xMonitor_handle);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*-----------------------------------*/

/* Generator — sleeps until a timer fires, then handles all
   pending releases before suspending again. */
void DD_TaskGenerator(void *pvParameters)
{
	uint32_t task_num;

	for (;;)
	{
		// drain the queue in case multiple timers fired before we ran
		while (xQueueReceive(xPendingReleaseQueue, &task_num, 0) == pdTRUE)
		{
			TickType_t now = xTaskGetTickCount();

			if (task_num == 1)
			{
				taskID1++;
				uint32_t deadline = now + pdMS_TO_TICKS(T1_PERIOD_MS);
				printf("[GEN] Task 1 released at %lu, deadline %lu\n",
					   (unsigned long)now, (unsigned long)deadline);
				create_dd_task(xUserTask1_handle, PERIODIC, taskID1, deadline);
			}
			else if (task_num == 2)
			{
				taskID2++;
				uint32_t deadline = now + pdMS_TO_TICKS(T2_PERIOD_MS);
				printf("[GEN] Task 2 released at %lu, deadline %lu\n",
					   (unsigned long)now, (unsigned long)deadline);
				create_dd_task(xUserTask2_handle, PERIODIC, taskID2, deadline);
			}
			else if (task_num == 3)
			{
				taskID3++;
				uint32_t deadline = now + pdMS_TO_TICKS(T3_PERIOD_MS);
				printf("[GEN] Task 3 released at %lu, deadline %lu\n",
					   (unsigned long)now, (unsigned long)deadline);
				create_dd_task(xUserTask3_handle, PERIODIC, taskID3, deadline);
			}
		}

		vTaskSuspend(NULL);
	}
}

/*-----------------------------------*/

/* DDS — blocks on xDDSQueue waiting for messages from the interface functions.
   Manages active, completed, and overdue lists internally.
   EDF is applied by setting the earliest-deadline task to HIGH-1 priority
   and everything else to LOW after any list change. */
void DDS(void *pvParameters)
{
	dd_task_list *active_list    = NULL;
	dd_task_list *completed_list = NULL;
	dd_task_list *overdue_list   = NULL;

	dd_message msg;
	uint32_t   ack = 1;

	for (;;)
	{
		if (xQueueReceive(xDDSQueue, &msg, portMAX_DELAY) != pdTRUE)
			continue;

		TickType_t now = xTaskGetTickCount();

		switch (msg.type)
		{
			case RELEASE_TASK:
			{
				msg.task.release_time = (uint32_t)now;

				// Suspend current head before reordering
				if (active_list != NULL)
				{
					vTaskSuspend(active_list->task.t_handle);
					vTaskPrioritySet(active_list->task.t_handle, LOW_PRIORITY);
				}

				add_task_to_list(&active_list, msg.task);
				active_list = merge_sort_task_list(active_list);

				// Apply EDF priorities
				dd_task_list *curr = active_list;
				int first = 1;
				while (curr != NULL)
				{
					if (first)
					{
						vTaskPrioritySet(curr->task.t_handle, HIGH_PRIORITY - 1);
						vTaskResume(curr->task.t_handle);
						first = 0;
					}
					else
					{
						vTaskPrioritySet(curr->task.t_handle, LOW_PRIORITY);
					}
					curr = curr->next_task;
				}

				xQueueSend(xResponseQueue, &ack, portMAX_DELAY);
				break;
			}

			case COMPLETE_TASK:
			{
				// Find the completing task and stamp its completion time
				dd_task_list *completing = active_list;
				while (completing != NULL)
				{
					if (completing->task.task_id == msg.task.task_id)
					{
						vTaskSuspend(completing->task.t_handle);
						vTaskPrioritySet(completing->task.t_handle, LOW_PRIORITY);
						completing->task.completion_time = (uint32_t)now;
						break;
					}
					completing = completing->next_task;
				}

				if (completing != NULL)
				{
					if (completing->task.completion_time <= completing->task.absolute_deadline)
						move_task_between_lists(&active_list, &completed_list, msg.task.task_id);
					else
						move_task_between_lists(&active_list, &overdue_list, msg.task.task_id);
				}

				active_list = merge_sort_task_list(active_list);

				// Re-apply EDF to remaining tasks
				dd_task_list *curr = active_list;
				int first = 1;
				while (curr != NULL)
				{
					if (first)
					{
						vTaskPrioritySet(curr->task.t_handle, HIGH_PRIORITY - 1);
						vTaskResume(curr->task.t_handle);
						first = 0;
					}
					else
					{
						vTaskPrioritySet(curr->task.t_handle, LOW_PRIORITY);
					}
					curr = curr->next_task;
				}

				xQueueSend(xResponseQueue, &ack, portMAX_DELAY);
				break;
			}

			// Return list head pointer directly — no separate ack needed
			case GET_ACTIVE_LIST:
				xQueueSend(xResponseQueue, &active_list, portMAX_DELAY);
				break;

			case GET_COMPLETED_LIST:
				xQueueSend(xResponseQueue, &completed_list, portMAX_DELAY);
				break;

			case GET_OVERDUE_LIST:
				xQueueSend(xResponseQueue, &overdue_list, portMAX_DELAY);
				break;

			case CHECK_OVERDUE:
			{
				dd_task_list *curr = active_list;
				while (curr != NULL)
				{
					dd_task_list *next = curr->next_task;
					if ((uint32_t)now > curr->task.absolute_deadline)
					{
						curr->task.completion_time = (uint32_t)now;
						add_task_to_list(&overdue_list, curr->task);
						remove_task_from_list(&active_list, curr->task.task_id);
						active_list = merge_sort_task_list(active_list);
					}
					curr = next;
				}
				break;
			}

			default:
				break;
		}
	}
}

/*-----------------------------------*/

/* Prints active/completed/overdue counts each time the monitor timer fires. */
void Monitor_Task(void *pvParameters)
{
	for (;;)
	{
		TickType_t now = xTaskGetTickCount();

		dd_task_list **active    = get_active_dd_task_list();
		dd_task_list **completed = get_complete_dd_task_list();
		dd_task_list **overdue   = get_overdue_dd_task_list();

		int active_count = 0, completed_count = 0, overdue_count = 0;

		dd_task_list *curr = (active    ? *active    : NULL);
		while (curr) { active_count++;    curr = curr->next_task; }

		curr = (completed ? *completed : NULL);
		while (curr) { completed_count++; curr = curr->next_task; }

		curr = (overdue   ? *overdue   : NULL);
		while (curr) { overdue_count++;   curr = curr->next_task; }

		printf("[MONITOR] t=%lu ms | Active: %d | Completed: %d | Overdue: %d\n",
		       (unsigned long)now,
		       active_count, completed_count, overdue_count);

		vTaskSuspend(NULL);
	}
}

/*-----------------------------------*/

// User tasks — busy-wait to simulate workload, then signal completion to DDS.
// The for(;;) re-enters only after DDS resumes this task for a new instance.

void User_Task1(void *pvParameters)
{
	for (;;)
	{
		TickType_t start = xTaskGetTickCount();
		while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(T1_EXEC_MS)) { }

		printf("[T1] Complete at %lu ms\n", (unsigned long)xTaskGetTickCount());
		delete_dd_task(taskID1);
	}
}

/*-----------------------------------*/

void User_Task2(void *pvParameters)
{
	for (;;)
	{
		TickType_t start = xTaskGetTickCount();
		while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(T2_EXEC_MS)) { }

		printf("[T2] Complete at %lu ms\n", (unsigned long)xTaskGetTickCount());
		delete_dd_task(taskID2);
	}
}

/*-----------------------------------*/

void User_Task3(void *pvParameters)
{
	for (;;)
	{
		TickType_t start = xTaskGetTickCount();
		while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(T3_EXEC_MS)) { }

		printf("[T3] Complete at %lu ms\n", (unsigned long)xTaskGetTickCount());
		delete_dd_task(taskID3);
	}
}

/*-----------------------------------*/

/* Sends RELEASE_TASK to DDS and blocks until ack comes back. */
void create_dd_task(TaskHandle_t t_handle, task_type type,
                    uint32_t task_id, uint32_t absolute_deadline)
{
	dd_task task;
	task.t_handle          = t_handle;
	task.type              = type;
	task.task_id           = task_id;
	task.release_time      = 0;  // DDS stamps this
	task.absolute_deadline = absolute_deadline;
	task.completion_time   = 0;

	dd_message message;
	message.type = RELEASE_TASK;
	message.task = task;

	xQueueSend(xDDSQueue, &message, portMAX_DELAY);
	vTaskResume(xDDS_handle);

	uint32_t ack;
	xQueueReceive(xResponseQueue, &ack, portMAX_DELAY);
}

/*-----------------------------------*/

/* Sends COMPLETE_TASK to DDS, waits for ack.
   Note: do NOT suspend here — DDS handles suspend/resume ordering itself. */
void delete_dd_task(uint32_t task_id)
{
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

	xQueueSend(xDDSQueue, &message, portMAX_DELAY);
	vTaskResume(xDDS_handle);

	uint32_t ack;
	xQueueReceive(xResponseQueue, &ack, portMAX_DELAY);

	// don't suspend here — would race with DDS's own vTaskResume call
}

/*-----------------------------------*/

dd_task_list** get_active_dd_task_list(void)
{
	dd_message message;
	message.type = GET_ACTIVE_LIST;

	xQueueSend(xDDSQueue, &message, portMAX_DELAY);
	vTaskResume(xDDS_handle);

	static dd_task_list *result;
	xQueueReceive(xResponseQueue, &result, portMAX_DELAY);
	return &result;
}

/*-----------------------------------*/

dd_task_list** get_complete_dd_task_list(void)
{
	dd_message message;
	message.type = GET_COMPLETED_LIST;

	xQueueSend(xDDSQueue, &message, portMAX_DELAY);
	vTaskResume(xDDS_handle);

	static dd_task_list *result;
	xQueueReceive(xResponseQueue, &result, portMAX_DELAY);
	return &result;
}

/*-----------------------------------*/

dd_task_list** get_overdue_dd_task_list(void)
{
	dd_message message;
	message.type = GET_OVERDUE_LIST;

	xQueueSend(xDDSQueue, &message, portMAX_DELAY);
	vTaskResume(xDDS_handle);

	static dd_task_list *result;
	xQueueReceive(xResponseQueue, &result, portMAX_DELAY);
	return &result;
}

/*-----------------------------------*/

/* Standalone list test — runs once at startup then suspends.
   Uses Test Bench #1 deadlines to verify add, sort, and remove. */
void List_Test_Task(void *pvParameters)
{
	dd_task_list *test_list = NULL;
	dd_task_list *curr      = NULL;
	int pass                = 1;

	printf("\n========================================\n");
	printf("  DD-Task List Function Tests\n");
	printf("  Based on Test Bench #1 (T1=500ms, T2=500ms, T3=750ms)\n");
	printf("========================================\n\n");

	// deadlines match the first hyper-period of Test Bench #1
	dd_task task1 = {
		.t_handle          = NULL,
		.type              = PERIODIC,
		.task_id           = 1,
		.release_time      = 0,
		.absolute_deadline = pdMS_TO_TICKS(500),
		.completion_time   = 0
	};

	dd_task task2 = {
		.t_handle          = NULL,
		.type              = PERIODIC,
		.task_id           = 2,
		.release_time      = 0,
		.absolute_deadline = pdMS_TO_TICKS(500),   // same deadline as T1
		.completion_time   = 0
	};

	dd_task task3 = {
		.t_handle          = NULL,
		.type              = PERIODIC,
		.task_id           = 3,
		.release_time      = 0,
		.absolute_deadline = pdMS_TO_TICKS(750),
		.completion_time   = 0
	};

	// TEST 1: add three tasks, check count and insertion order
	printf("--- TEST 1: add_task_to_list ---\n");

	add_task_to_list(&test_list, task3);  // intentionally out of deadline order
	add_task_to_list(&test_list, task1);
	add_task_to_list(&test_list, task2);

	int count = 0;
	curr = test_list;
	while (curr != NULL) { count++; curr = curr->next_task; }

	if (count == 3)
		printf("  [PASS] List has 3 nodes after 3 adds\n");
	else {
		printf("  [FAIL] Expected 3 nodes, got %d\n", count);
		pass = 0;
	}

	if (test_list != NULL &&
	    test_list->task.task_id == 3 &&
	    test_list->next_task->task.task_id == 1 &&
	    test_list->next_task->next_task->task.task_id == 2)
		printf("  [PASS] Insertion order preserved (3 -> 1 -> 2)\n");
	else {
		printf("  [FAIL] Insertion order wrong\n");
		pass = 0;
	}

	printf("  List contents: ");
	curr = test_list;
	while (curr != NULL) {
		printf("ID=%lu(dl=%lu) ", (unsigned long)curr->task.task_id,
		       (unsigned long)curr->task.absolute_deadline);
		curr = curr->next_task;
	}
	printf("\n\n");

	// TEST 2: sort — task3 (dl=750) should end up last
	printf("--- TEST 2: merge_sort_task_list ---\n");

	test_list = merge_sort_task_list(test_list);

	// first two nodes must have deadline 500, last must have 750
	int sort_pass = 1;
	curr = test_list;
	if (curr == NULL || curr->task.absolute_deadline != pdMS_TO_TICKS(500)) sort_pass = 0;
	else curr = curr->next_task;
	if (curr == NULL || curr->task.absolute_deadline != pdMS_TO_TICKS(500)) sort_pass = 0;
	else curr = curr->next_task;
	if (curr == NULL || curr->task.absolute_deadline != pdMS_TO_TICKS(750)) sort_pass = 0;

	if (sort_pass)
		printf("  [PASS] Sorted by deadline: 500 -> 500 -> 750\n");
	else {
		printf("  [FAIL] Sort order incorrect\n");
		pass = 0;
	}

	printf("  Sorted list: ");
	curr = test_list;
	while (curr != NULL) {
		printf("ID=%lu(dl=%lu) ", (unsigned long)curr->task.task_id,
		       (unsigned long)curr->task.absolute_deadline);
		curr = curr->next_task;
	}
	printf("\n\n");

	// TEST 3: remove middle node
	printf("--- TEST 3: remove_task_from_list (middle node) ---\n");

	uint32_t mid_id = test_list->next_task->task.task_id;
	test_list = remove_task_from_list(&test_list, mid_id);

	count = 0;
	curr = test_list;
	int found_removed = 0;
	while (curr != NULL) {
		if (curr->task.task_id == mid_id) found_removed = 1;
		count++;
		curr = curr->next_task;
	}

	if (count == 2 && !found_removed)
		printf("  [PASS] Removed ID=%lu, list now has 2 nodes\n", (unsigned long)mid_id);
	else {
		printf("  [FAIL] Remove middle failed (count=%d, still_present=%d)\n", count, found_removed);
		pass = 0;
	}

	// TEST 4: remove head
	printf("--- TEST 4: remove_task_from_list (head node) ---\n");

	uint32_t head_id = test_list->task.task_id;
	test_list = remove_task_from_list(&test_list, head_id);

	count = 0;
	found_removed = 0;
	curr = test_list;
	while (curr != NULL) {
		if (curr->task.task_id == head_id) found_removed = 1;
		count++;
		curr = curr->next_task;
	}

	if (count == 1 && !found_removed)
		printf("  [PASS] Removed head ID=%lu, list now has 1 node\n", (unsigned long)head_id);
	else {
		printf("  [FAIL] Remove head failed (count=%d, still_present=%d)\n", count, found_removed);
		pass = 0;
	}

	// TEST 5: remove last node — list should be NULL
	printf("--- TEST 5: remove_task_from_list (last node -> empty list) ---\n");

	uint32_t last_id = test_list->task.task_id;
	test_list = remove_task_from_list(&test_list, last_id);

	if (test_list == NULL)
		printf("  [PASS] List is NULL after removing last node\n");
	else {
		printf("  [FAIL] List should be NULL but is not\n");
		pass = 0;
	}

	// TEST 6: remove from empty list — should not crash
	printf("--- TEST 6: remove_task_from_list (empty list, no crash) ---\n");

	test_list = remove_task_from_list(&test_list, 99);

	if (test_list == NULL)
		printf("  [PASS] Returned NULL gracefully on empty list\n");
	else {
		printf("  [FAIL] Should return NULL on empty list\n");
		pass = 0;
	}

	// TEST 7: single node sort
	printf("--- TEST 7: merge_sort_task_list (single node) ---\n");

	add_task_to_list(&test_list, task2);
	test_list = merge_sort_task_list(test_list);

	if (test_list != NULL && test_list->task.task_id == task2.task_id && test_list->next_task == NULL)
		printf("  [PASS] Single-node sort returned correct node\n");
	else {
		printf("  [FAIL] Single-node sort failed\n");
		pass = 0;
	}

	remove_task_from_list(&test_list, task2.task_id);

	printf("\n========================================\n");
	if (pass)
		printf("  ALL TESTS PASSED\n");
	else
		printf("  ONE OR MORE TESTS FAILED — see above\n");
	printf("========================================\n\n");

	vTaskSuspend(NULL);
}

/*-----------------------------------*/

/* Appends a new node to the list. Sorting is done separately.
   Uses pvPortMalloc — only called once per task instance at release. */
dd_task_list* add_task_to_list(dd_task_list **head, dd_task new_task)
{
	dd_task_list *node = (dd_task_list*)pvPortMalloc(sizeof(dd_task_list));
	if (node == NULL) return *head;  // alloc failed

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

void vApplicationMallocFailedHook( void ) { for(;;) { } }

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName ){
	( void ) pcTaskName;
	( void ) pxTask;
	for(;;) { }
}

void vApplicationIdleHook( void ){
	volatile size_t available_memory;
	available_memory = xPortGetFreeHeapSize();
	if(available_memory > 100){ /* heap ok */ }
}

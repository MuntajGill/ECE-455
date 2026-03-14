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

/*-----------------------------------------------------------*/

// Define constants for task parameters
#define HIGH_PRIORITY   3
#define MEDIUM_PRIORITY 2
#define LOW_PRIORITY    1
#define IDLE_PRIORITY   0

// Test bench parameters (Test Bench #1)
#define T1_EXEC_MS    95
#define T1_PERIOD_MS  500

#define T2_EXEC_MS    150
#define T2_PERIOD_MS  500

#define T3_EXEC_MS    250
#define T3_PERIOD_MS  750

/*-----------------------------------------------------------*/

// DD-task struct - NOTE: May add an additional for interrupt execution times if needed for the scheduler implementation.
typedef enum { PERIODIC, APERIODIC } task_type;

typedef struct {
    TaskHandle_t t_handle;
    task_type    type;
    uint32_t     task_id;
    uint32_t     release_time;
    uint32_t     absolute_deadline;
    uint32_t     completion_time;
} dd_task;

// DD-task list node struct
typedef struct dd_task_list {
    dd_task             task;
    struct dd_task_list *next_task;
} dd_task_list;

// DDS Message types
typedef enum {
    RELEASE_TASK,
    COMPLETE_TASK,
    GET_ACTIVE_LIST,
    GET_COMPLETED_LIST,
    GET_OVERDUE_LIST,
    CHECK_OVERDUE       // sent internally by overdue timer callback
} msg_type;

// DDS message struct — wraps a message type and the associated dd_task
typedef struct {
    msg_type type;
    dd_task  task;
} dd_message;

/*-----------------------------------------------------------*/

// Queue handle declarations
QueueHandle_t xDDSQueue;            // incoming dd_message structs to DDS
QueueHandle_t xResponseQueue;       // DDS responses to list requests
QueueHandle_t xPendingReleaseQueue; // timer callback -> generator: which task number to release

/*-----------------------------------------------------------*/

// Task handle declarations
// Needed so the timer callback can resume the generator,
// and so create_dd_task can resume the DDS
TaskHandle_t xDDS_handle;
TaskHandle_t xDDTaskGen_handle;
TaskHandle_t xUserTask1_handle;
TaskHandle_t xUserTask2_handle;
TaskHandle_t xUserTask3_handle;

// Software timer handles (one per user task, per spec requirement)
TimerHandle_t xTimer1;
TimerHandle_t xTimer2;
TimerHandle_t xTimer3;

// Per-task ID counters (incremented each time a new instance is released)
static uint32_t taskID1 = 0;
static uint32_t taskID2 = 0;
static uint32_t taskID3 = 0;

/*-----------------------------------------------------------*/

// F-Task declarations
void DDS(void *pvParameters);  				// Deadline-Driven Scheduler task
void DD_TaskGenerator(void *pvParameters);  // Task generator for testing purposes
void Monitor_Task(void *pvParameters);      // Task to monitor and print the active, completed, and overdue task lists

// User-Defined Tasks declarations - for testing purposes, can be modified as needed
void User_Task1(void *pvParameters);
void User_Task2(void *pvParameters);
void User_Task3(void *pvParameters);

// Shared timer callback — one callback handles all three task timers
void vTimerCallback(TimerHandle_t xTimer);

// DDS Interface function declarations
void create_dd_task(TaskHandle_t t_handle, task_type type, uint32_t task_id, uint32_t absolute_deadline);
void delete_dd_task(uint32_t task_id);

// Functions to retrieve task lists - these will be used by the Monitor_Task to print the current state of the system
dd_task_list** get_active_dd_task_list(void);
dd_task_list** get_complete_dd_task_list(void);
dd_task_list** get_overdue_dd_task_list(void);

// Setup Initializations
void init_FTasks(void);

/*-----------------------------------------------------------*/

// Singly Linked List management functions for DD-task lists (active, completed, overdue)
dd_task_list* add_task_to_list(dd_task_list **head, dd_task new_task);
dd_task_list* remove_task_from_list(dd_task_list **head, uint32_t task_id);
dd_task_list* merge_sort_task_list(dd_task_list *head);

/*-----------------------------------------------------------*/

int main(void)
{
	// Initialize F-tasks
	init_FTasks();

	// Start the scheduler
	vTaskStartScheduler();

	// Should never reach here if RTOS is running
	for(;;) { }
}

/*-----------------------------------------------------------*/

void init_FTasks(void)
{
	// xDDSQueue carries full dd_message structs
	xDDSQueue            = xQueueCreate(10, sizeof(dd_message));

	// xResponseQueue carries either a uint32_t ack or a dd_task_list* (use larger size)
	xResponseQueue       = xQueueCreate(10, sizeof(dd_task_list*));

	// xPendingReleaseQueue carries uint32_t task numbers (1, 2, or 3)
	xPendingReleaseQueue = xQueueCreate(10, sizeof(uint32_t));

	// Create tasks — store handles for DDS, generator, and all user tasks
	xTaskCreate(DDS,             "DDS",       256, NULL, HIGH_PRIORITY,   &xDDS_handle);
	xTaskCreate(DD_TaskGenerator,"DD_TaskGen",256, NULL, MEDIUM_PRIORITY, &xDDTaskGen_handle);
	xTaskCreate(Monitor_Task,    "Monitor",   256, NULL, LOW_PRIORITY,    NULL);

	xTaskCreate(User_Task1, "UserTask1", 256, NULL, LOW_PRIORITY, &xUserTask1_handle);
	xTaskCreate(User_Task2, "UserTask2", 256, NULL, LOW_PRIORITY, &xUserTask2_handle);
	xTaskCreate(User_Task3, "UserTask3", 256, NULL, LOW_PRIORITY, &xUserTask3_handle);

	// Suspend user tasks and generator — timers will wake the generator each period
	vTaskSuspend(xDDS_handle);
	vTaskSuspend(xDDTaskGen_handle);
	vTaskSuspend(xUserTask1_handle);
	vTaskSuspend(xUserTask2_handle);
	vTaskSuspend(xUserTask3_handle);

	// Create one periodic software timer per user task (auto-reload = pdTRUE)
	// Timer ID encodes the task number so the shared callback knows which task to release
	xTimer1 = xTimerCreate("T1_Timer", pdMS_TO_TICKS(T1_PERIOD_MS), pdTRUE, (void*)1, vTimerCallback);
	xTimer2 = xTimerCreate("T2_Timer", pdMS_TO_TICKS(T2_PERIOD_MS), pdTRUE, (void*)2, vTimerCallback);
	xTimer3 = xTimerCreate("T3_Timer", pdMS_TO_TICKS(T3_PERIOD_MS), pdTRUE, (void*)3, vTimerCallback);

	xTimerStart(xTimer1, 0);
	xTimerStart(xTimer2, 0);
	xTimerStart(xTimer3, 0);
}

/*-----------------------------------------------------------*/

// Shared timer callback — runs in the FreeRTOS timer daemon context.
// Enqueues the task number for the generator and resumes it.
void vTimerCallback(TimerHandle_t xTimer)
{
	uint32_t task_num = (uint32_t)pvTimerGetTimerID(xTimer);

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// Enqueue which task needs to be released
	xQueueSendFromISR(xPendingReleaseQueue, &task_num, &xHigherPriorityTaskWoken);

	// Wake the generator
	vTaskResumeFromISR(xDDTaskGen_handle);

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*-----------------------------------------------------------*/

// DD_TaskGenerator — suspended at rest, woken by vTimerCallback each period.
// Drains xPendingReleaseQueue and calls create_dd_task for each pending release.
void DD_TaskGenerator(void *pvParameters)
{
	uint32_t task_num;

	for (;;)
	{
		// Process all releases that may have queued up before suspending again
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

		// All pending releases handled — sleep until next timer fires
		vTaskSuspend(NULL);
	}
}

/*-----------------------------------------------------------*/

// Deadline-Driven Scheduler task implementation
void DDS(void *pvParameters){

}

/*-----------------------------------------------------------*/

void Monitor_Task(void *pvParameters){

}

/*-----------------------------------------------------------*/

void User_Task1(void *pvParameters){

}

/*-----------------------------------------------------------*/

void User_Task2(void *pvParameters){

}

/*-----------------------------------------------------------*/

void User_Task3(void *pvParameters){

}

/*-----------------------------------------------------------*/

// Creates a new task in the system and sends a RELEASE_TASK message to the DDS.
// Blocks until the DDS acknowledges receipt and has updated the active list.
void create_dd_task(TaskHandle_t t_handle, task_type type,
                    uint32_t task_id, uint32_t absolute_deadline)
{
	dd_task task;
	task.t_handle          = t_handle;
	task.type              = type;
	task.task_id           = task_id;
	task.release_time      = 0;  // DDS will stamp this on receipt
	task.absolute_deadline = absolute_deadline;
	task.completion_time   = 0;

	dd_message message;
	message.type = RELEASE_TASK;
	message.task = task;

	// Send message to DDS then resume it (DDS has highest priority, preempts immediately)
	xQueueSend(xDDSQueue, &message, portMAX_DELAY);
	vTaskResume(xDDS_handle);

	// Block until DDS sends acknowledgement
	uint32_t ack;
	xQueueReceive(xResponseQueue, &ack, portMAX_DELAY);
}

/*-----------------------------------------------------------*/

// Deletes a task from the system by task_id
void delete_dd_task(uint32_t task_id){

}

/*-----------------------------------------------------------*/

// Get the list of active tasks - this will be used by the Monitor_Task to print the current state of the system
dd_task_list** get_active_dd_task_list(void){

}

/*-----------------------------------------------------------*/

// Get the list of completed tasks - this will be used by the Monitor_Task to print the current state of the system
dd_task_list** get_complete_dd_task_list(void){

}

/*-----------------------------------------------------------*/

// Get the list of overdue tasks - this will be used by the Monitor_Task to print the current state of the system
dd_task_list** get_overdue_dd_task_list(void){

}

/*-----------------------------------------------------------*/

// Add a task to the specified list (active, completed, or overdue).
// Allocates a new node and appends it to the tail of the list.
// Sorting is handled separately by merge_sort_task_list.
// Returns the head of the list (unchanged, but returned for consistency).
dd_task_list* add_task_to_list(dd_task_list **head, dd_task new_task)
{
	// Allocate new node from FreeRTOS heap
	dd_task_list *node = (dd_task_list*)pvPortMalloc(sizeof(dd_task_list));
	if (node == NULL) return *head;  // allocation failure — list unchanged

	node->task      = new_task;
	node->next_task = NULL;

	// If list is empty, new node becomes the head
	if (*head == NULL)
	{
		*head = node;
		return *head;
	}

	// Otherwise walk to the tail and append
	dd_task_list *curr = *head;
	while (curr->next_task != NULL)
		curr = curr->next_task;

	curr->next_task = node;
	return *head;
}

/*-----------------------------------------------------------*/

// Remove a task from the specified list by task_id.
// Frees the removed node back to the FreeRTOS heap.
// Returns the (possibly new) head of the list.
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
				*head = curr->next_task;  // removing the head node
			else
				prev->next_task = curr->next_task;

			vPortFree(curr);
			return *head;
		}
		prev = curr;
		curr = curr->next_task;
	}

	return *head;  // task_id not found — list unchanged
}

/*-----------------------------------------------------------*/

// Iterative bottom-up merge sort — sorts list nodes by absolute_deadline ascending.
// Does not allocate any memory; only relinks existing nodes.
// Returns the new head of the sorted list.
dd_task_list* merge_sort_task_list(dd_task_list *head)
{
	if (head == NULL || head->next_task == NULL)
		return head;

	// Count list length so we know when to stop doubling the sublist size
	int length = 0;
	dd_task_list *curr = head;
	while (curr != NULL) { length++; curr = curr->next_task; }

	// dummy.next_task always points to the current sorted list head
	dd_task_list dummy;
	dummy.next_task = head;

	// Each pass merges adjacent sublists of increasing size (1, 2, 4, 8, ...)
	for (int size = 1; size < length; size *= 2)
	{
		dd_task_list *tail = &dummy;   // tail of the already-sorted output so far
		curr = dummy.next_task;        // start of remaining unsorted input

		while (curr != NULL)
		{
			// --- Split: carve out left sublist of length 'size' ---
			dd_task_list *left = curr;
			dd_task_list *right = curr;
			int left_len = 0;

			while (left_len < size && right != NULL)
			{
				right = right->next_task;
				left_len++;
			}
			// 'right' now points to the start of the right sublist (may be NULL)

			// --- Merge left and right sublists ---
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

			// Drain whichever side still has nodes
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

			// Advance curr to the start of the next pair of sublists
			curr = right;
		}

		// dummy.next_task is now the head of the fully merged pass
	}

	return dummy.next_task;
}

/*-----------------------------------------------------------*/

// RTOS hook functions (FreeRTOS middleware)
void vApplicationMallocFailedHook( void ) {	for(;;) { } }

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName ){
	( void ) pcTaskName;
	( void ) pxTask;
	for(;;) { }
}

void vApplicationIdleHook( void ){
	volatile size_t available_memory;
	available_memory = xPortGetFreeHeapSize();

	if(available_memory > 100){ /* Heap monitoring */ }
}

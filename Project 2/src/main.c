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
#define HIGH_PRIORITY 3
#define MEDIUM_PRIORITY 2
#define LOW_PRIORITY 1
#define IDLE_PRIORITY 0

// Define test bench parameters


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
typedef struct {
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

/*-----------------------------------------------------------*/

// Queue handle declarations
QueueHandle_t xDDSQueue;       // incoming messages to DDS
QueueHandle_t xResponseQueue;  // DDS responses to list requests

/*-----------------------------------------------------------*/

// F-Task declarations
void DDS(void *pvParameters);  				// Deadline-Driven Scheduler task
void DD_TaskGenerator(void *pvParameters);  // Task generator for testing purposes
void Monitor_Task(void *pvParameters);      // Task to monitor and print the active, completed, and overdue task lists

// User-Defined Tasks declarations - for testing purposes, can be modified as needed
void User_Task1(void *pvParameters);
void User_Task2(void *pvParameters);
void User_Task3(void *pvParameters);

// Timer callback declaration for checking overdue tasks
void vOverdueTimerCallback(TimerHandle_t xTimer); // Uses xQueueSendISRS to send CHECK_OVERDUE message to DDS	

// DDS Interface function declarations
void create_dd_task(TaskHandle_t t_handle, task_type type, uint32_t task_id, uint32_t absolute_deadline);
void delete_dd_task(uint32_t task_id);

// Functions to retrieve task lists - these will be used by the Monitor_Task to print the current state of the system
dd_task_list** get_active_dd_task_list(void);
dd_task_list** get_complete_dd_task_list(void);
dd_task_list** get_overdue_dd_task_list(void);

// Setup Initializations
void init_FTasks(void); // Initializes queues, creates tasks, suspends tasks until scheduler starts

/*-----------------------------------------------------------*/

// Singly Linked List management functions for DD-task lists (active, completed, overdue)

// Add a task to the specified list (active, completed, or overdue)
dd_task_list* add_task_to_list(dd_task_list **head, dd_task new_task);

// Remove a task from the specified list by task_id
dd_task_list* remove_task_from_list(dd_task_list **head, uint32_t task_id);

// Iterative merge sort for task lists based on absolute deadlines
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

void init_FTasks(void){
	// Create queues
	xDDSQueue = xQueueCreate(10, sizeof(msg_type));       		// Queue for messages to DDS
	xResponseQueue = xQueueCreate(10, sizeof(dd_task_list*)); 	// Queue for DDS responses to list requests

	// Create tasks
	xTaskCreate(DDS, "DDS", 256, NULL, HIGH_PRIORITY, NULL);
	xTaskCreate(DD_TaskGenerator, "DD_TaskGen", 256, NULL, MEDIUM_PRIORITY, NULL);
	xTaskCreate(Monitor_Task, "Monitor", 256, NULL, LOW_PRIORITY, NULL);
	
	xTaskCreate(User_Task1, "UserTask1", 256, NULL, LOW_PRIORITY, NULL);
	xTaskCreate(User_Task2, "UserTask2", 256, NULL, LOW_PRIORITY, NULL);
	xTaskCreate(User_Task3, "UserTask3", 256, NULL, LOW_PRIORITY, NULL);

	// Suspend user-defined tasks until they are released by the DDS
	vTaskSuspend(User_Task1);
	vTaskSuspend(User_Task2);
	vTaskSuspend(User_Task3);
}

/*-----------------------------------------------------------*/

// Deadline-Driven Scheduler task implementation
// Receives messages from other tasks (task releases, completions, list requests) and manages the active, completed, and overdue task lists accordingly.
void DDS(void *pvParameters){

}

/*-----------------------------------------------------------*/

void DD_TaskGenerator(void *pvParameters){

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

// Timer callback function to check for overdue tasks - this will be called periodically by a FreeRTOS timer and will send a CHECK_OVERDUE message to the DDS to trigger the overdue task checking process.
void vOverdueTimerCallback(TimerHandle_t xTimer){

}

/*-----------------------------------------------------------*/

// Creates a new task in the system with the specified parameters and adds it to the active task list
void create_dd_task(TaskHandle_t t_handle, task_type type, uint32_t task_id, uint32_t absolute_deadline){

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

// Add a task to the specified list (active, completed, or overdue)
dd_task_list* add_task_to_list(dd_task_list **head, dd_task new_task){

}

/*-----------------------------------------------------------*/

// Remove a task from the specified list by task_id
dd_task_list* remove_task_from_list(dd_task_list **head, uint32_t task_id){

}

/*-----------------------------------------------------------*/

// Iterative merge sort for task lists based on absolute deadlines
dd_task_list* merge_sort_task_list(dd_task_list *head){

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

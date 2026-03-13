// Team Member A & B
// Student IDs

/* Standard libraries - no middleware */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include <stdlib.h>   // for rand()

/* RTOS includes - FreeRTOS middleware */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"



/*-----------------------------------------------------------*/

int main(void)
{
	// Should never reach here if RTOS is running
	for(;;) { }
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

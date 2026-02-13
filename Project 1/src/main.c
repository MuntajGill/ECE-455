// Team Member A & B
// Student IDs

/* Standard libraries - no middleware */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"

/* RTOS includes - FreeRTOS middleware */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

/*-----------------------------------------------------------*/

// Flow rate boundaries for ADC scaling
#define FLOW_LOW		40
#define FLOW_HIGH		4000

// 100ms tick interval for periodic tasks
#define TICK_INTERVAL	pdMS_TO_TICKS(100)

// LED pins for traffic lights
#define LED_R			GPIO_Pin_0
#define LED_Y			GPIO_Pin_1
#define LED_G			GPIO_Pin_2

// Potentiometer pin for flow control
#define POTENTIOMETER	GPIO_Pin_3

// Shift register pins for car display
#define SHIFT_DATA		GPIO_Pin_6
#define SHIFT_CLK		GPIO_Pin_7
#define SHIFT_RST		GPIO_Pin_8

// Car presence indicators
#define CAR_PRESENT		1
#define NO_CAR			0

/*-----------------------------------------------------------*/

// Task function declarations (FreeRTOS middleware)
void TrafficControllerTask		( void *pvParameters );
void LightSequenceTask			( void *pvParameters );
void VehicleGeneratorTask		( void *pvParameters );
void VisualDisplayTask			( void *pvParameters );

// Timer callback functions (FreeRTOS middleware)
void GreenTimerCallback 		( TimerHandle_t xTimer );
void YellowTimerCallback 		( TimerHandle_t xTimer );
void RedTimerCallback 			( TimerHandle_t xTimer );

// Hardware initialization functions
void SetupGPIOPorts				(void);
void SetupADCModule				(void);

// ADC reading function
uint16_t ReadPotentiometer(void);

// Global queues for inter-task communication (FreeRTOS middleware)
xQueueHandle FlowRateQueue = 0;
xQueueHandle LightStateQueue = 0;
xQueueHandle VehicleQueue = 0;

// Timer handles for light timing (FreeRTOS middleware)
TimerHandle_t GreenLightTimer 	= 0;
TimerHandle_t YellowLightTimer = 0;
TimerHandle_t RedLightTimer 	= 0;

/*-----------------------------------------------------------*/

int main(void)
{
	// Initialize hardware
	SetupGPIOPorts();
	SetupADCModule();

	// Create RTOS queues for data sharing (FreeRTOS middleware)
	FlowRateQueue   = xQueueCreate( 1, sizeof(int) );
	LightStateQueue = xQueueCreate( 1, sizeof(uint16_t) );
	VehicleQueue    = xQueueCreate( 1, sizeof(int) );

	// If queues created successfully, set up tasks and timers
	if( FlowRateQueue && LightStateQueue && VehicleQueue )
	{
		// Create four concurrent tasks (FreeRTOS middleware)
		xTaskCreate( TrafficControllerTask, "TrafficController", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		xTaskCreate( LightSequenceTask, "LightSequence", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		xTaskCreate( VehicleGeneratorTask, "VehicleGenerator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		xTaskCreate( VisualDisplayTask, "VisualDisplay", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

		// Create timers for light sequence (FreeRTOS middleware)
		GreenLightTimer  = xTimerCreate("GreenTimer", pdMS_TO_TICKS(5000), pdFALSE, 0, GreenTimerCallback);
		YellowLightTimer = xTimerCreate("YellowTimer", pdMS_TO_TICKS(3000), pdFALSE, 0, YellowTimerCallback);
		RedLightTimer    = xTimerCreate("RedTimer", pdMS_TO_TICKS(5000), pdFALSE, 0, RedTimerCallback);

		// Start the RTOS scheduler (FreeRTOS middleware)
		vTaskStartScheduler();
	}

	// Should never reach here if RTOS is running
	for(;;) { }
}

/*-----------------------------------------------------------*/

void SetupGPIOPorts()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef PortConfig;

	// Set up traffic light LEDs (PC0-PC2) as outputs
	PortConfig.GPIO_Pin = LED_R | LED_Y | LED_G;
	PortConfig.GPIO_Mode = GPIO_Mode_OUT;
	PortConfig.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &PortConfig);

	// Configure potentiometer pin (PC3) as analog input
	PortConfig.GPIO_Pin = POTENTIOMETER;
	PortConfig.GPIO_Mode = GPIO_Mode_AN;
	PortConfig.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &PortConfig);

	// Set up shift register control pins (PC6-PC8) as outputs
	PortConfig.GPIO_Pin = SHIFT_DATA | SHIFT_CLK | SHIFT_RST;
	PortConfig.GPIO_Mode = GPIO_Mode_OUT;
	PortConfig.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &PortConfig);
}

/*-----------------------------------------------------------*/

void SetupADCModule()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_InitTypeDef ADC_Config;

	// Configure ADC for single conversion mode
	ADC_Config.ADC_ContinuousConvMode = DISABLE;
	ADC_Config.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_Config.ADC_Resolution = ADC_Resolution_12b;
	ADC_Config.ADC_ScanConvMode = DISABLE;
	ADC_Config.ADC_ExternalTrigConv = DISABLE;
	ADC_Config.ADC_ExternalTrigConvEdge = DISABLE;

	// Apply settings and enable ADC
	ADC_Init(ADC1, &ADC_Config);
	ADC_Cmd(ADC1, ENABLE);
	
	// Configure ADC channel 13 (connected to PC3/potentiometer)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_144Cycles);
}

/*-----------------------------------------------------------*/

uint16_t ReadPotentiometer(void)
{
	ADC_SoftwareStartConv(ADC1);
	
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) 
	{ 
		// Wait for ADC conversion
	}
	
	return ADC_GetConversionValue(ADC1);
}

/*-----------------------------------------------------------*/

// Task: Continuously reads potentiometer and calculates flow rate
void TrafficControllerTask( void *pvParameters )
{
	int flow_percentage;
	uint16_t raw_value;
	
	for(;;)
	{
		raw_value = ReadPotentiometer();
		
		// Convert ADC value to percentage with bounds checking
		if(raw_value < FLOW_LOW) 
		{
			flow_percentage = 0;
		}
		else if(raw_value > FLOW_HIGH) 
		{
			flow_percentage = 100;
		}
		else 
		{
			flow_percentage = (100 * (raw_value - FLOW_LOW)) / (FLOW_HIGH - FLOW_LOW);
		}
		
		// Update flow rate in queue (FreeRTOS middleware)
		xQueueOverwrite(FlowRateQueue, &flow_percentage);
		vTaskDelay(TICK_INTERVAL);
	}
}

/*-----------------------------------------------------------*/

// Task: Starts traffic light sequence and maintains current light state
void LightSequenceTask( void *pvParameters )
{
	// Start with green light (FreeRTOS middleware)
	xTimerStart(GreenLightTimer, 0);
	
	uint16_t current_light = LED_G;
	xQueueOverwrite(LightStateQueue, &current_light);
	
	for(;;)
	{
		vTaskDelay(100 * TICK_INTERVAL);
	}
}

/*-----------------------------------------------------------*/

// Called when green timer expires: Switch to yellow light
void GreenTimerCallback( TimerHandle_t xTimer )
{
	// Start yellow light timer (FreeRTOS middleware)
	xTimerStart(YellowLightTimer, 0);
	
	uint16_t current_light = LED_Y;
	xQueueOverwrite(LightStateQueue, &current_light);
}

/*-----------------------------------------------------------*/

// Called when yellow timer expires: Switch to red light
void YellowTimerCallback( TimerHandle_t xTimer )
{
	int traffic_flow;
	// Get current flow rate from queue (FreeRTOS middleware)
	BaseType_t queue_result = xQueuePeek(FlowRateQueue, &traffic_flow, TICK_INTERVAL);
	
	if(queue_result == pdPASS)
	{
		// Adjust red light duration based on flow (FreeRTOS middleware)
		xTimerChangePeriod(RedLightTimer, pdMS_TO_TICKS(10000 - 50 * traffic_flow), 0);
		
		uint16_t current_light = LED_R;
		xQueueOverwrite(LightStateQueue, &current_light);
	}
}

/*-----------------------------------------------------------*/

// Called when red timer expires: Switch to green light
void RedTimerCallback( TimerHandle_t xTimer )
{
	int traffic_flow;
	// Get current flow rate from queue (FreeRTOS middleware)
	BaseType_t queue_result = xQueuePeek(FlowRateQueue, &traffic_flow, TICK_INTERVAL);
	
	if(queue_result == pdPASS)
	{
		// Adjust green light duration based on flow (FreeRTOS middleware)
		xTimerChangePeriod(GreenLightTimer, pdMS_TO_TICKS(5000 + 50 * traffic_flow), 0);
		
		uint16_t current_light = LED_G;
		xQueueOverwrite(LightStateQueue, &current_light);
	}
}

/*-----------------------------------------------------------*/

// Task: Generates new cars at intervals based on traffic flow
void VehicleGeneratorTask( void *pvParameters )
{
	int traffic_flow;
	int new_vehicle = CAR_PRESENT;
	BaseType_t queue_result;
	
	for(;;)
	{
		// Check current flow rate (FreeRTOS middleware)
		queue_result = xQueuePeek(FlowRateQueue, &traffic_flow, TICK_INTERVAL);
		
		if(queue_result == pdPASS)
		{
			xQueueOverwrite(VehicleQueue, &new_vehicle);
			vTaskDelay(pdMS_TO_TICKS(4000 - 35 * traffic_flow));
		}
	}
}

/*-----------------------------------------------------------*/

// Task: Updates all visual outputs (LEDs and car display)
void VisualDisplayTask( void *pvParameters )
{
	BaseType_t light_status, vehicle_status;
	uint16_t active_light;
	int vehicle_data;
	int vehicle_positions[20];
	int index = 0;
	
	// Initialize all car positions to empty
	while(index < 20)
	{
		vehicle_positions[index] = NO_CAR;
		index++;
	}
	
	GPIO_SetBits(GPIOC, SHIFT_RST);
	
	for(;;)
	{
		// Get current light state from queue (FreeRTOS middleware)
		light_status = xQueuePeek(LightStateQueue, &active_light, TICK_INTERVAL);
		
		if(light_status == pdPASS)
		{
			GPIO_ResetBits(GPIOC, LED_R);
			GPIO_ResetBits(GPIOC, LED_Y);
			GPIO_ResetBits(GPIOC, LED_G);
			GPIO_SetBits(GPIOC, active_light);
		}
		
		// Display cars using shift register
		index = 19;
		while(index >= 0)
		{
			if(vehicle_positions[index] == CAR_PRESENT)
				GPIO_SetBits(GPIOC, SHIFT_DATA);
			else
				GPIO_ResetBits(GPIOC, SHIFT_DATA);
			
			GPIO_SetBits(GPIOC, SHIFT_CLK);
			GPIO_ResetBits(GPIOC, SHIFT_CLK);
			index--;
		}
		
		// Move cars based on traffic light state
		if(active_light == LED_G)
		{
			index = 19;
			while(index > 0)
			{
				vehicle_positions[index] = vehicle_positions[index - 1];
				index--;
			}
		}
		else
		{
			// Red or Yellow: cars stop at intersection
			index = 8;
			while(index > 0)
			{
				if(vehicle_positions[index] == NO_CAR)
				{
					vehicle_positions[index] = vehicle_positions[index - 1];
					vehicle_positions[index - 1] = NO_CAR;
				}
				index--;
			}
			
			index = 19;
			while(index > 9)
			{
				vehicle_positions[index] = vehicle_positions[index - 1];
				vehicle_positions[index - 1] = NO_CAR;
				index--;
			}
		}
		
		// Check if new car has arrived (FreeRTOS middleware)
		vehicle_status = xQueueReceive(VehicleQueue, &vehicle_data, pdMS_TO_TICKS(100));
		vehicle_positions[0] = NO_CAR;
		
		if(vehicle_status == pdPASS && vehicle_data == CAR_PRESENT)
		{
			vehicle_positions[0] = CAR_PRESENT;
		}
		
		vTaskDelay(5 * TICK_INTERVAL);
	}
}

/*-----------------------------------------------------------*/

// RTOS hook functions (FreeRTOS middleware)
void vApplicationMallocFailedHook( void )
{
	for(;;) { }
}

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;
	for(;;) { }
}

void vApplicationIdleHook( void )
{
	volatile size_t available_memory;
	available_memory = xPortGetFreeHeapSize();
	
	if(available_memory > 100)
	{
		// Heap monitoring
	}
}
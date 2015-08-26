/*
    FreeRTOS V8.2.2 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks
 * (which just exist to test the kernel port and provide an example of how to use
 * each FreeRTOS API function).
 *
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Check" task - This only executes every five seconds but has the highest
 * priority so is guaranteed to get processor time.  Its main function is to
 * check that all the standard demo tasks are still operational. The check task
 * will toggle LED 3 (PB11) every five seconds so long as no errors have been
 * detected.  The toggle rate will increase to half a second if an error has
 * been found in any task.
 *
 * "Echo" task - This is a very basic task that simply echoes any characters
 * received on COM0 (USART1).  This can be tested by transmitting a text file
 * from a dumb terminal to the STM32 USART then observing or capturing the text
 * that is echoed back.  Missing characters will be all the more obvious if the
 * file contains a simple repeating string of fixed width.
 *
 * Currently this demo does not include interrupt nesting examples.  High
 * frequency timer and simpler nesting examples can be found in most Cortex-M3
 * demo applications.
 *
 * The functions used to initialise, set and clear LED outputs are normally
 * defined in partest.c.  This demo includes two partest files, one that is
 * configured for use with the Keil MCBSTM32 evaluation board (called
 * ParTest_MCBSTM32.c) and one that is configured for use with the official
 * ST Eval board (called ParTest_ST_Eval.c).  One one of these files should be
 * included in the build at any one time, as appropriate for the hardware
 * actually being used.
 */



/* Standard includes. */
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"



#include "stm32f10x.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_gpio.h"


/* The time between cycles of the 'check' task - which depends on whether the
check task has detected an error or not. */
#define mainCHECK_DELAY_NO_ERROR			( ( TickType_t ) 5000 / portTICK_PERIOD_MS )
#define mainCHECK_DELAY_ERROR				( ( TickType_t ) 500 / portTICK_PERIOD_MS )


/* Task priorities. */
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainECHO_TASK_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainINTEGER_TASK_PRIORITY           ( tskIDLE_PRIORITY )
#define mainGEN_QUEUE_TASK_PRIORITY			( tskIDLE_PRIORITY )



#include "trcKernelPort.h"

/*-----------------------------------------------------------*/

/*
 * Configure the hardware for the demo.
 */
static void prvSetupHardware( void );

/* The 'check' task as described at the top of this file. */
static void prvCheckTask( void *pvParameters );




/*-----------------------------------------------------------*/

uint8_t gBITSET=0;
#define mainDELAY_LOOP_COUNT        ( 0xffffff )


int main( void )
{
#ifdef DEBUG
  debug();
#endif


  //**********************************
  // USED FOR FreeRTOS+TRACE
  //**********************************
  vTraceInitTraceData();
  uiTraceStart();
  //**********************************



	/* Set up the clocks and memory interface. */
	prvSetupHardware();

	/* Create the 'check' task, which is also defined within this file. */
	xTaskCreate( prvCheckTask, "Check", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );


    /* Start the scheduler. */
	vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task.  The idle task is created within vTaskStartScheduler(). */



	for( ;; );
}
/*-----------------------------------------------------------*/

/* Described at the top of this file. */
static void prvCheckTask( void *pvParameters )
{


volatile unsigned long ul;
	for( ;; )
	{
	
						for( ul = 0; ul < mainDELAY_LOOP_COUNT; ul++ ) // PAUSE
						{}
						GPIO_WriteBit(GPIOC , GPIO_Pin_13 , Bit_RESET);

						for( ul = 0; ul < mainDELAY_LOOP_COUNT; ul++ ) // PAUSE
						{}
						GPIO_WriteBit(GPIOC , GPIO_Pin_13 , Bit_SET);
	
	}
}
/*-----------------------------------------------------------*/





static void prvSetupHardware( void )
{
	/* RCC system reset(for debug purpose). */
	RCC_DeInit ();

    /* Enable HSE. */
	RCC_HSEConfig( RCC_HSE_ON );

	/* Wait till HSE is ready. */
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);

    /* HCLK = SYSCLK. */
	RCC_HCLKConfig( RCC_SYSCLK_Div1 );

    /* PCLK2  = HCLK. */
	RCC_PCLK2Config( RCC_HCLK_Div1 );

    /* PCLK1  = HCLK/2. */
	RCC_PCLK1Config( RCC_HCLK_Div2 );

	/* ADCCLK = PCLK2/4. */
	RCC_ADCCLKConfig( RCC_PCLK2_Div4 );

    /* Flash 2 wait state. */
	*( volatile unsigned long  * )0x40022000 = 0x01;

	/* PLLCLK = 8MHz * 9 = 72 MHz */
	RCC_PLLConfig( RCC_PLLSource_HSE_Div1, RCC_PLLMul_9 );

    /* Enable PLL. */
	RCC_PLLCmd( ENABLE );

	/* Wait till PLL is ready. */
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

	/* Select PLL as system clock source. */
	RCC_SYSCLKConfig (RCC_SYSCLKSource_PLLCLK);

	/* Wait till PLL is used as system clock source. */
	while (RCC_GetSYSCLKSource() != 0x08);

	/* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
							| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE );

	/* Set the Vector Table base address at 0x08000000. */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );

	/* SPI2 Periph clock enable */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );


	/**************************************/

	GPIO_InitTypeDef GPIO_InitStructure;

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);


	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_WriteBit(GPIOC , GPIO_Pin_13 , Bit_SET);

}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}
/*-----------------------------------------------------------*/

void assert_failed( unsigned char *pucFile, unsigned long ulLine )
{
	( void ) pucFile;
	( void ) ulLine;

	for( ;; );
}


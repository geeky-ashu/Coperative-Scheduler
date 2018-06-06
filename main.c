#include "osKernel.h"
#include "STM32F4_RTOS_BSP.h"
#define QUANTA	1

uint32_t count0,count1,count2;
int32_t semaphore1 , semaphore2;

void osSemaphoreInit(int32_t *semaphore , int32_t value)
{
	*semaphore = value;
}

void osSignalSet (int32_t *semaphore)
{
	__disable_irq();
	*semaphore+=1;
	__enable_irq();
}

void osSignalWait(int32_t *semaphore)
{
	__disable_irq();
	while(*semaphore<=0)
	{
		__disable_irq();
		osThreadYield();
		__enable_irq();
	}
	*semaphore-=1;
	__enable_irq();
}
void Task0(void)
{
	while(1)
	{
		//count0++;
	}
	
}


void Task1(void)
{
	while(1)
	{
		osSignalWait(&semaphore2);
		ST7735_DrawString(3,7,"This is THREAD 1", GREEN);
		osSignalSet(&semaphore1);
	}
	
}


void Task2(void)
{
	while(1)
	{
		osSignalWait(&semaphore1);
		ST7735_DrawString(3,5,"This is THREAD 2", GREEN);
		osSignalSet(&semaphore2);
	}
	
}


int main(void)
{
	osSemaphoreInit(&semaphore1 ,1);
	osSemaphoreInit(&semaphore2 ,0);
	HAL_Init();
	Probe_Init();
	TIM4_Init_Start();
	osKernelInit();
	osKernelAddThreads(&Task0,&Task1,&Task2);
	osKernelLaunch(QUANTA);
}

void TIM4_IRQHandler(void)
{
  Probe_CH3();
	
  HAL_TIM_IRQHandler(&htim4);

}



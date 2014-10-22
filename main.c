 /*****************************************************************************
  * @title   main.c
  * @author  Awax
  * @date    04/09/2014
  * @brief   Line following robot project
  *******************************************************************************/

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx.h"
#include "misc.h"
/////////////////////////////////
void robotForward(void);
void robotBack(void);
void robotTurnRignt(void);
void robotTurnLeft(void);
void robotStop(void);

void rightMotorClkWise(void);
void leftMotorClkWise(void);
void rightMotorStop(void);
void leftMotorStop(void);
void rightMotorCounterClkWise(void);
void leftMotorCounterClkWise(void);
void Delay(int);

void readLDR(void);
void GPIOInit(void);
void RCCInit(void);
void ADCInit(void);
void DMAInit(void);
void ONButtonInit(void);

/////////////////////////////////
static uint8_t ON = 1;
static uint8_t left=0,right=0;
uint16_t ADC3ConvertedValue[1] = {0};
/////////////////////////////////
int main(void)
{
	RCCInit();
	DMAInit();
	ADCInit();
	GPIOInit();
	ONButtonInit();



    while(1)
    {
    	GPIO_ResetBits(GPIOD,GPIO_Pin_12 | GPIO_Pin_15| GPIO_Pin_13| GPIO_Pin_14);
    	while(!ON); // waiting for begin order
    	readLDR();


    	if (left==0  && right==0)
    	{
    	    robotForward();
    		GPIO_SetBits(GPIOD,GPIO_Pin_13);
    	}
    	if (left==1 &&  right==0)
    	{
    		robotTurnLeft();
    		GPIO_SetBits(GPIOD,GPIO_Pin_12);
    	}
    	if (left==0 &&  right==1)
    	{
    		robotTurnRignt() ;
    		GPIO_SetBits(GPIOD,GPIO_Pin_14);
    	}
    	if (left==1  && right==1)
    	{
    	    robotForward();
    		GPIO_SetBits(GPIOD,GPIO_Pin_15);
    	}
    }
}

void readLDR()  // 1 means black surface , 0 white surface
{ int r=0,l=0;
	 ADC_SoftwareStartConv(ADC3);
	 ADC_SoftwareStartConv(ADC1);

	 r = ADC3ConvertedValue[0];			//PC0 ----> right LDR
	 l = ADC_GetConversionValue(ADC1);  //PA0 -----> left LDR

	if (r < 1700 ) right = 0;
	else right = 1;

	if (l<3000) left = 0;
	else left = 1;
}
///////////////////Robot Movements/////////////////
void robotForward()
{
	rightMotorClkWise();
	leftMotorClkWise();
}

void robotBack()
{
	leftMotorCounterClkWise();
	rightMotorCounterClkWise();
}

void robotStop()
{
	leftMotorStop();
	rightMotorStop();

}

void robotTurnRignt()
{
	leftMotorClkWise();
	rightMotorCounterClkWise();
}

void robotTurnLeft()
{
	rightMotorClkWise();
	leftMotorCounterClkWise();
}

void rightMotorClkWise()  // PB6 --->  L293D  IN 1  /  PB7 ---> IN2
{
	GPIO_SetBits(GPIOB,GPIO_Pin_6);
	GPIO_ResetBits(GPIOB,GPIO_Pin_7);
}

void leftMotorClkWise()  // PB4 --->  L293D  IN 3  /  PB5 ---> IN4
{
	GPIO_SetBits(GPIOB,GPIO_Pin_4);
	GPIO_ResetBits(GPIOB,GPIO_Pin_5);
}

void rightMotorCounterClkWise()
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_6);
	GPIO_SetBits(GPIOB,GPIO_Pin_7);
}

void leftMotorCounterClkWise()
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_4);
	GPIO_SetBits(GPIOB,GPIO_Pin_5);
}

void rightStopMotor()
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_6);
	GPIO_ResetBits(GPIOB,GPIO_Pin_7) ;
}

void leftStopMotor()
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_4);
	GPIO_ResetBits(GPIOB,GPIO_Pin_5) ;
}
/////////////////////////////////////////////////////

void RCCInit(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
}



void GPIOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_StructInit(&GPIO_InitStructure);
	// for led pins
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14|GPIO_Pin_12 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;


  GPIO_Init(GPIOD, &GPIO_InitStructure);
  //  for L293D motor driver
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5|GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //for ADC3 on PC0 using IN10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  //for ADC1 on PA0 using IN0
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	// for On button 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


void ADCInit(void)
{
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  ADC_InitTypeDef       ADC_InitStructure;

  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;

  ADC_CommonInit(&ADC_CommonInitStructure);

  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;


  ADC_StructInit(&ADC_InitStructure);

  ADC_Init(ADC3, &ADC_InitStructure);
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_480Cycles);


  ADC_EOCOnEachRegularChannelCmd(ADC3, ENABLE);
  ADC_EOCOnEachRegularChannelCmd(ADC1, ENABLE);

  /* Enable DMA request after last transfer (Single-ADC mode) */
 ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
 /* Enable ADC3 DMA */
 ADC_DMACmd(ADC3, ENABLE);

  ADC_Cmd(ADC3, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
}
void DMAInit()
{
  DMA_InitTypeDef       DMA_InitStruct;

/* DMA2 Stream0 channel0 configuration **************************************/
  DMA_InitStruct.DMA_Channel = DMA_Channel_2;
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC3->DR;//ADC3's data register
  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStruct.DMA_BufferSize = 1;
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//Reads 16 bit values
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//Stores 16 bit values
  DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStruct.DMA_Priority = DMA_Priority_High;
  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStruct);
  DMA_Cmd(DMA2_Stream0, ENABLE);
}
void ONButtonInit()
{

	EXTI_InitTypeDef   EXTI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource15);
	EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}
void Delay(int n)
{
	while(n--);
}
void EXTI15_10_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		if (ON == 1) ON = 0;
		else ON = 1;

		Delay(0x3fff);

		EXTI_ClearITPendingBit(EXTI_Line15);
	}
}

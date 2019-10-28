//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "stm32f0xx_spi.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

void myGPIOA_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void myADC_init(void);
void myDAC_init(void);
void delay(uint32_t time);
void analogWrite(uint16_t value);
void mySPI_init(void);
uint32_t analogRead(void);

// Your global variables...
uint8_t started_timer = 0x00;
uint32_t data;

int
main(int argc, char* argv[])
{

	trace_printf("This is Part 2 of Introductory Lab...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */
	myDAC_init();
	mySPI_init();
	myADC_init();
	analogWrite(4095);
	shiftOutBit(0xf3);

	while (1)
	{
		data = analogRead();
		trace_printf("%d\n", data);
	}

	return 0;

}

/************************
 * HELPER FUNCTIONS******
 ************************/

void shiftOutBit(uint8_t data){
//	Set LCK to 0
	GPIOB->ODR &= ~GPIO_ODR_4;
	while(!(SPI1->SR & SPI_SR_TXE)){}
	SPI_SendData8(SPI1, data);
//	Set LCK to 1
	GPIOB->ODR |= GPIO_ODR_4;
}

void analogWrite(uint16_t value){
	DAC->DHR12R1 = (uint32_t)value;
}

void delay(uint32_t time){
//	Simple busy wait

	for (uint32_t i=0; i<time *4800; i++){
		__asm("nop");
	}
}

uint32_t analogRead(void){
	ADC1->CR |= ADC_CR_ADSTART;
	while(!(ADC1->ISR & ADC_ISR_EOC)){}
	return ((uint32_t)(ADC1->DR & ADC_DR_DATA));
}

/********************
 * INIT FUNCTIONS****
 ********************/

void mySPI_init(){
//  Configure GPIO

//  Enable clock to gpiob
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

//  Configure PB3 & PB5 for use with spi
//  Set AF Mode
  GPIOB->MODER |= GPIO_MODER_MODER3_1;
  GPIOB->MODER |= GPIO_MODER_MODER5_1;

//  Make sure the afr is set to AF0 for PB3 and PB5
  GPIOB->AFR[0] |= ~(0x00F0F000);


//	Configure pin for LCK signal
  GPIOB->MODER |= GPIO_MODER_MODER4_0;


//  Enable Clock
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
// Configure SPI
  SPI_InitTypeDef SPI_Init_Struct;
  SPI_Init_Struct.SPI_BaudRatePrescaler = 0x7;
  SPI_Init_Struct.SPI_Mode = SPI_Mode_Master;
  SPI_Init_Struct.SPI_Direction = SPI_Direction_1Line_Tx;
  SPI_Init_Struct.SPI_CPOL = SPI_CPOL_Low;
  SPI_Init_Struct.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_Init_Struct.SPI_NSS = SPI_NSS_Soft;
  SPI_Init_Struct.SPI_DataSize = SPI_DataSize_8b;
  SPI_Init_Struct.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_Init_Struct.SPI_CRCPolynomial = 7;

  SPI_Init(SPI1, &SPI_Init_Struct);
  SPI_Cmd(SPI1, ENABLE);
}

void myDAC_init(){
// Configure PA4 as DAC
//	Set PA4 to analog mode
	GPIOA->MODER |= GPIO_MODER_MODER4;
// Enable DAC
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	DAC->CR |= DAC_CR_EN1;

}

void myADC_init(){
//	Set PA0 to analog mode
	GPIOA->MODER |= GPIO_MODER_MODER0;
//	Enable ADC Clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
//	Calibrate ADC

	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL){}

//	Enable ADC
	ADC1->CR |= ADC_CR_ADEN;
	while (ADC1->ISR & ADC_ISR_ADRDY){}

	ADC1->CHSELR = ADC_CHSELR_CHSEL0;

	ADC1->CFGR1 |= ADC_CFGR1_CONT;

}


void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA1 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;
}


void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 |= ((uint16_t)0x8C);
	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR |= 0x1;
	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);
	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);
	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;
}


void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] &= ~(0xF0);
	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;
	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/************
 ****ISRs****
 ***********/


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~TIM_SR_UIF;
		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 &= ~TIM_CR1_CEN;
		started_timer = 0x00;
	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	// Your local variables...
	EXTI->IMR &= ~EXTI_IMR_MR1;
	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		//
		// 1. If this is the first edge:
		if(!started_timer){
		//	- Clear count register (TIM2->CNT).
			TIM2->CNT = 0x00;
		//	- Start timer (TIM2->CR1).
			TIM2->CR1 |= TIM_CR1_CEN;
			started_timer = 0x01;
		}
		//    Else (this is the second edge):
		else{
		//	- Stop timer (TIM2->CR1).
			TIM2->CR1 &= ~TIM_CR1_CEN;
		//	- Read out count register (TIM2->CNT).
			uint32_t counts = TIM2->CNT;
		//	- Calculate signal period and frequency.
			uint32_t period = counts / 48;
			uint32_t freq = 48e6/counts;
		//	- Print calculated values to the console.
			trace_printf("%d us\n", period);
			trace_printf("%d Hz\n", freq);
			started_timer = 0x00;
		//	  NOTE: Function trace_printf does not work
		//	  with floating-point numbers: you must use
		//	  "unsigned int" type to print your signal
		//	  period and frequency.
		//
		}

		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		//
		EXTI->PR |= EXTI_PR_PR1;
	}
	EXTI->IMR |= EXTI_IMR_MR1;
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

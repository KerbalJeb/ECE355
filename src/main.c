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

#define PRINT_LCD_MSG 0x00

/* RS bit postion for LCD */
#define RS 0x40
/* EN bit postion for LCD */
#define EN 0x80
/* Base address for LCD line 1*/
#define LINE_1 0x80
/* Base address for LCD line 2*/
#define LINE_2 0xC0

/**
 * @brief Enables GPIO port A and configures PA1 as an input 
 * (for use with the EXTI inturrupt) 
 * 
 */
void myGPIOA_Init(void);

/**
 * @brief Congfigures TIM2 with lowest prescaler and overflow interrupt
 * (for mesauring pwm frequency)
 *  
 */
void myTIM2_Init(void);

/**
 * @brief Enables EXTI and maps PA1 to EXTI1
 * 
 */
void myEXTI_Init(void);

/**
 * @brief Initializes the ADC for PA0 
 * 
 */
void myADC_init(void);

/**
 * @brief Initializes the DAC for PA4
 * 
 */
void myDAC_init(void);

/**
 * @brief A simply busy wait delay
 * 
 * @param time How long the delay should be in ~miliseconds
 */
void delay(uint32_t time);

/**
 * @brief Outputs a true analog voltage on a PA4
 * 
 * @param value A value between 0-4095, that maps to a 0-3.3v output voltage
 */
void analogWrite(uint16_t value);

/**
 * @brief Configures the SPI periferal with lowsest possible clock speed
 * using pins PB3 (SCK), PB5 (MOSI), PB4 (LCK, seprate from SPI peripheral)
 * 
 */
void mySPI_init(void);

/**
 * @brief Sends initialization commands to the LCD
 * 
 */
void myLCD_init();

/**
 * @brief Reads a value from the PA0 ADC
 * 
 * @return uint32_t A value from 0-4095, coresponding to an input voltage
 * from 0-3.3v
 */
uint32_t analogRead(void);

/**
 * @brief Sends a byte of data to the shift register
 * 
 * @param data 
 */
void shiftOutBit(uint8_t data);

/**
 * @brief Sends a byte of data to the LCD (in 4 bit mode)
 * 
 * @param data The data so send (ex. ASCII code)
 * @param cmd Command bits to set (ex. EN or RS bits)
 */
void sendWordToLCD(uint8_t data, uint8_t cmd);

/**
 * @brief Sends a 4 bit, half word to the LCD
 * 
 * @param data The 4 bits of data to send
 * @param cmd Command bits to set (ex. EN or RS bits)
 */
void sendToLCD_4bit(uint8_t data, uint8_t cmd);

/**
 * @brief Sends a string to the LCD
 * does not perform any length checks on string
 * 
 * @param str The string to write
 * @param line The base address of the line to write to 
 * (LINE_1 or LINE_2)
 */
void displayString(char * str, uint8_t line);

/**
 * @brief Used to display resistance and frequency data on the display
 * 
 * @param freq The current frequency (true value in Hz)
 * @param voltage The raw voltage reading from ADC (0-4095)
 */
void displayData(uint32_t freq, uint32_t voltage);

// Global variables, updated in EXTI ISR
volatile uint8_t started_timer = 0x00;
volatile uint8_t freq_measurement_done;
uint32_t voltage;
uint32_t freq;

int
main(int argc, char* argv[])
{
	/*Perform Initializations*/
	myGPIOA_Init();		
	myDAC_init();
	mySPI_init();
	myADC_init();
	myLCD_init();
	myTIM2_Init();		
	myEXTI_Init();		



	while (1)
	{
		/*Get the input voltage from the ADC and write it to the DAC 
		 * (both are 12 bit, so no conversion is needed)
		*/
		voltage = analogRead();
		analogWrite(voltage);
		
		/*Unmask the EXTI inturupts, wait for a frequency measurement 
		 * to compleate and then disable the EXTI inturupts
		 * This is to ensure only valid data is written to the display
		*/

		freq_measurement_done = 0x00;
		EXTI->IMR |= EXTI_IMR_MR1;
		while (!freq_measurement_done){}
		EXTI->IMR &= ~EXTI_IMR_MR1;
		/*Write data to display*/
		displayData(freq, voltage);
	}

	return 0;

}

/************************
 * HELPER FUNCTIONS******
 ************************/

void displayData(uint32_t freq, uint32_t voltage){	
	
	uint32_t res = 5000 - voltage*5000/4095;
	char line_1[8];
	char line_2[8];
	/* Format data as string */
	sprintf(line_1, "%d Hz", freq);
	sprintf(line_2, "%d R", res);
	/* Clear the display */
	sendWordToLCD(0x01, 0);
	delay(1);
	/* Write new data */
	displayString(line_1, LINE_1);
	displayString(line_2, LINE_2);
	delay(500);
}

void displayString(char * str, uint8_t line){
	/* Set the base address for the line */
	sendWordToLCD(line, 0);
	delay(1);
	/* Iterate over the string */
	while(*str != '\0'){
		/* Send the ASCII code to the display */
		sendWordToLCD((uint8_t)(*str), RS);
		str++;
		delay(1);
	}

}

void sendWordToLCD(uint8_t data, uint8_t cmd){
	/* Slit into high and low half */
	uint8_t data_low = data & 0x0f;
	uint8_t data_high = ((data & 0xF0) >> 4);
	if (PRINT_LCD_MSG){
	    trace_printf("Sending: %x\n", data);
	}
	/* Send high half, followed by low half */
	sendToLCD_4bit(data_high, cmd);
	sendToLCD_4bit(data_low, cmd);
}

void sendToLCD_4bit(uint8_t data, uint8_t cmd){
	data = (data & 0x0f) | cmd;
	/* Send the 4 bits of data, while pulsing the enable bit */
	shiftOutBit(data);
	shiftOutBit(data | EN);
	shiftOutBit(data);
}

void shiftOutBit(uint8_t data){

  if (PRINT_LCD_MSG){
      trace_printf("Shifting out: %x\n", data);
  }

    /* Set LCK to 0*/
	GPIOB->ODR &= ~GPIO_ODR_4;
	/* Wait for SPI transmit buffer to be empty */
	while(!(SPI1->SR & SPI_SR_TXE)){}
	/* Send the data */
	SPI_SendData8(SPI1, data);
	/* Wait for tranmission to compleate */
	while(SPI1->SR & SPI_SR_BSY){}
	/* Set LCK to 1 */
	GPIOB->ODR |= GPIO_ODR_4;
}

void analogWrite(uint16_t value){
	/* Update the 12 bit right aligned data holding register*/
	DAC->DHR12R1 = (uint32_t)value;
}

void delay(uint32_t time){
	/**
	 * Simple busy wait, 3200 determained expermentaly 
	 */

	for (uint32_t i=0; i<time*3200; i++){
		__asm("nop");
	}
}

uint32_t analogRead(void){
	/* Start ADC converstion */
	ADC1->CR |= ADC_CR_ADSTART;
	/* Wait for end of conversion */
	while(!(ADC1->ISR & ADC_ISR_EOC)){}
	/* Return the ADC value */
	return ((uint32_t)(ADC1->DR & ADC_DR_DATA));
}

/********************
 * INIT FUNCTIONS****
 ********************/

void myLCD_init(){
	/* Sends init sequence from lecture slides*/
	sendWordToLCD(0x02, 0);
	delay(2);
	sendWordToLCD(0x28, 0);
	sendWordToLCD(0x0c, 0);
	sendWordToLCD(0x06, 0);
	sendWordToLCD(0x01, 0);
	delay(2);
}

void mySPI_init(){
//  Configure GPIO

//  Enable clock to gpiob
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

//  Configure pin for LCK signal
  GPIOB->MODER |= GPIO_MODER_MODER4_0;
  GPIOB->OSPEEDR |= (0x3 << 8);
  GPIOB->ODR |= GPIO_ODR_4;

//  Configure PB3 & PB5 for use with spi
//  Set AF Mode
  GPIOB->MODER |= GPIO_MODER_MODER3_1;
  GPIOB->MODER |= GPIO_MODER_MODER5_1;

//  Make sure the afr is set to AF0 for PB3 and PB5
  GPIOB->AFR[0] |= ~(0x00F0F000);



//  Enable Clock
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
// Configure SPI
  SPI_InitTypeDef SPI_Init_Struct;
  SPI_Init_Struct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
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
  GPIOB->ODR &= ~GPIO_ODR_4;
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
	/* Set PA0 to analog mode */
	GPIOA->MODER |= GPIO_MODER_MODER0;
	/* Enable ADC Clock */
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	/* Calibrate ADC */
	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL){}

	/* Enable ADC */
	ADC1->CR |= ADC_CR_ADEN;
	while (ADC1->ISR & ADC_ISR_ADRDY){}

	/* Select channel 0 and use continuous mode */
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
			freq = 48e6/counts;

		//	- Print calculated values to the LCD.
			if (PRINT_LCD_MSG){
				trace_printf("%d us\n", period);
				trace_printf("%d Hz\n", freq);
			}
			/* Update global flags */
			started_timer = 0x00;
			freq_measurement_done = 0x01;
		}

		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		EXTI->PR |= EXTI_PR_PR1;
	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

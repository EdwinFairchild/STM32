#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_usart.h>
#include <stm32g4xx_ll_tim.h>
#include <stm32g4xx_ll_rcc.h>
#include <stm32g4xx_ll_system.h>
#include <stm32g4xx_ll_utils.h>
#include <stm32g4xx_ll_spi.h>
#include <stm32g4xx_ll_cortex.h>
#include "CL_CONFIG.h"
#include "st7735.h"
#include "fonts.h"
#include "testimg.h"
#include "string.h"
#include "CL_systemClockUpdate.h"

#define LED_A1 LL_GPIO_PIN_0	

uint16_t msTICKS = 0x0000;
uint16_t msTicks = 0x0000;
uint16_t RGB565 = 0x0000;
uint32_t CL_delayTicks = 0x00000000;
void tim2_init(void);
void tim2_LL_init(void);
void tim2_ch1_pwm_init(void);
void timDelaySetup(void);
void delayMS(uint32_t ms);
void uart2Init(void);
void led0Setup(void); 
void initSPI(void);
void spiSend(uint8_t * data, uint32_t len);
void initSPI2(void);
void tftGPIO_init(void);
void loop();
#define USE_DEBUG 
#define top_to_bottom  0
#define bottom_to_top  1
#define TFA  0                                       // Top Fixed Area 0 pixel
#define BFA  50 
#define _width         128
#define _height        128


int main(void)
{
	
	setSysClockTo170();
#ifdef USE_DEBUG
	//uart2Init();
//	led0Setup();
#endif
	timDelaySetup(); 	
	initSPI();
	tftGPIO_init();
	for (int i = 0; i < 10; i++)
	{
		LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_0);
		delayMS(100);
	}
	
	ST7735_Init();
	ST7735_SetRotation(4);
	ST7735_FillScreen(ST7735_BLACK);

	RGB565 = 0xffff;
	ST7735_WriteString(0, 0, "edwin", Font_11x18, RGB565, ST7735_BLACK);
	

	uint8_t scroll = 0 , y = 10;
	setScrollDefinition(TFA, 128, 1);
	uint16_t x, y2 = 0;
	//tim2_LL_init();
	//tim2_init();
	tim2_ch1_pwm_init();
	for (;;)
	{
	
		for (int i = 0; i < 1000; i += 10)
		{			
			LL_TIM_OC_SetCompareCH1(TIM2, i);
				delayMS(10);			
		}
		for (int i = 1000; i > 0; i -= 10)
		{			
			LL_TIM_OC_SetCompareCH1(TIM2, i);
		delayMS(10);			
		}                                                                                                                                                                                                                                                                                                                                                                                                 
		
		
			
	}
}
void tim2_LL_init(void)
{
	//RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	LL_TIM_InitTypeDef myTim2;
	LL_TIM_StructInit(&myTim2);
	myTim2.Autoreload = 1000;
	myTim2.Prescaler = 7200;
	LL_TIM_Init(TIM2, &myTim2);
	
	LL_TIM_OC_InitTypeDef OCmyTim2;
	LL_TIM_OC_StructInit(&OCmyTim2);
	OCmyTim2.CompareValue = 8000;
	TIM2->CR1 |=  TIM_CR1_URS | TIM_CR1_ARPE;
	LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &OCmyTim2);
	LL_TIM_DisableUpdateEvent(TIM2);
	LL_TIM_SetUpdateSource(TIM2, LL_TIM_UPDATESOURCE_COUNTER);
	LL_TIM_EnableIT_CC1(TIM2);
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->CR1 |= TIM_CR1_CEN;
	
}
void tim2_init(void)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	NVIC_EnableIRQ(TIM2_IRQn);	
	
	TIM2->ARR = 1000;
	TIM2->PSC = 2000;
	
	TIM2->DIER |=  TIM_DIER_CC1IE; //enable Channel 1 interrupt
	
	/*Frozen - The comparison between the output compare register TIMx_CCR1 and the
	counter TIMx_CNT has no effect on the outputs.(this mode is used to generate a timing
	base).*/
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_0; //do you want to build a snow man
	
	TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;  //CC1 channel is configured as output.
	TIM2->CCER |= TIM_CCER_CC1E; //enable ch1 output , but makes no difference here
	TIM2->CCR1 = 65535;   //higher value than ARR , thus interrupt should not tirgger, but it does

	TIM2->CR1 |= TIM_CR1_CEN;
		
	//-----------------
	/*
	TIM2->CR1 |= TIM_CR1_CKD_1;  //<--clock divisor
	
	TIM2->CR2 = 0;
	TIM2->PSC = 1000;   //<--prescaler
	TIM2->ARR = 7000;   //<--wait time
	TIM2->DIER = 0x0000;
	TIM2->DIER |= (TIM_DIER_UIE | TIM_DIER_CC1IE);  //update interrupt enable, capture compare 1 interrupt
	
	// Select the Output Compare Mode 
	//TIM2->CCMR1 |= TIM_OCMODE_TIMING;  //aka frozen?
	
	TIM2->CCR2 = 1000;
	TIM2->CCR4 = 8000;
	TIM2->CCR3 = 8000;
	TIM2->CR1 |= TIM_CR1_CEN; */
	//-----------------------------------------
}
void TIM2_IRQHandler(void)
{
	/*
	volatile uint8_t x, y;
	x = 0x010;; 
	y = 0x001;
	bool test  = x == y;
	if (test)
	{
		TIM2->SR &= ~TIM_SR_UIF;
		//LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
	
	}
	if ((TIM2->SR &= TIM_SR_CC1IF))
	{
		x++;
		LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
	}
	
	*/
	/*
	uint32_t reg = TIM2->SR;
	bool iscc1 = ((reg & TIM_SR_CC1IF) == (TIM_SR_CC1IF));
	bool iscc2 = ((reg & TIM_SR_CC2IF) == (TIM_SR_CC2IF));
	if (iscc1 && !iscc2)
	{
		//read new delay value, and set pins high according to bresenham
		//Motion::Core::Output::Segment::pntr_driver();
		volatile uint8_t x = 0;
		LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
	}
	if (iscc1 && iscc2)
	{
		//Stepper::step_port(0);
		volatile uint8_t y = 0;
		LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
	}
	
	TIM2->SR = 0x00; 
	*/
	if(TIM2->SR & TIM_SR_CC2IF)
	LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
}

void TIM4_IRQHandler() 
{
	//If you hit the breakpoint below, one of the interrupts was unhandled in your code. 
	//Define the following function in your code to handle it:
	//	extern "C" void TIM4_IRQHandler();
	TIM4->SR &= ~TIM_SR_UIF;
	CL_delayTicks++;
}
void loop() {
	// Check border
	ST7735_FillScreen(ST7735_BLACK);

	for (int x = 0; x < ST7735_WIDTH; x++) {
		ST7735_DrawPixel(x, 0, ST7735_RED);
		ST7735_DrawPixel(x, ST7735_HEIGHT - 1, ST7735_RED);
	}

	for (int y = 0; y < ST7735_HEIGHT; y++) {
		ST7735_DrawPixel(0, y, ST7735_RED);
		ST7735_DrawPixel(ST7735_WIDTH - 1, y, ST7735_RED);
	}
	//HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
 //   HAL_Delay(3000);
  //  HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
    // Check fonts
    ST7735_FillScreen(ST7735_BLACK);
	ST7735_WriteString(0, 0, "Font_7x10, red on black, lorem ipsum dolor sit amet", Font_7x10, ST7735_RED, ST7735_BLACK);
	ST7735_WriteString(0, 3 * 10, "Font_11x18, green, lorem ipsum", Font_11x18, ST7735_GREEN, ST7735_BLACK);
	ST7735_WriteString(0, 3 * 10 + 3 * 18, "Font_16x26", Font_16x26, ST7735_BLUE, ST7735_BLACK);
	// HAL_Delay(2000);
	// HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
	 // Check colors
	 ST7735_FillScreen(ST7735_BLACK);
	ST7735_WriteString(0, 0, "BLACK", Font_11x18, ST7735_WHITE, ST7735_BLACK);
	//  HAL_Delay(500);
	 // HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
	  ST7735_FillScreen(ST7735_BLUE);
	ST7735_WriteString(0, 0, "BLUE", Font_11x18, ST7735_BLACK, ST7735_BLUE);
	//  HAL_Delay(500);
	//  HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
	  ST7735_FillScreen(ST7735_RED);
	ST7735_WriteString(0, 0, "RED", Font_11x18, ST7735_BLACK, ST7735_RED);
	// HAL_Delay(500);
   //  HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
     ST7735_FillScreen(ST7735_GREEN);
	ST7735_WriteString(0, 0, "GREEN", Font_11x18, ST7735_BLACK, ST7735_GREEN);
	//  HAL_Delay(500);
	//  HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
	  ST7735_FillScreen(ST7735_CYAN);
	ST7735_WriteString(0, 0, "CYAN", Font_11x18, ST7735_BLACK, ST7735_CYAN);
	//  HAL_Delay(500);
   //   HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
      ST7735_FillScreen(ST7735_MAGENTA);
	ST7735_WriteString(0, 0, "MAGENTA", Font_11x18, ST7735_BLACK, ST7735_MAGENTA);
	//  HAL_Delay(500);
	//  HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
	  ST7735_FillScreen(ST7735_YELLOW);
	ST7735_WriteString(0, 0, "YELLOW", Font_11x18, ST7735_BLACK, ST7735_YELLOW);
	// HAL_Delay(500);
   //  HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
     ST7735_FillScreen(ST7735_WHITE);
	ST7735_WriteString(0, 0, "WHITE", Font_11x18, ST7735_BLACK, ST7735_WHITE);
	//  HAL_Delay(500);
   //   HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
#ifdef ST7735_IS_128X128
       // Display test image 128x128
       ST7735_DrawImage(0, 0, ST7735_WIDTH, ST7735_HEIGHT, (uint16_t*)test_img_128x128);
	//   HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
	/*
	    // Display test image 128x128 pixel by pixel
	    for(int x = 0; x < ST7735_WIDTH; x++) {
	        for(int y = 0; y < ST7735_HEIGHT; y++) {
	            uint16_t color565 = test_img_128x128[y][x];
	            // fix endiness
	            color565 = ((color565 & 0xFF00) >> 8) | ((color565 & 0xFF) << 8);
	            ST7735_DrawPixel(x, y, color565);
	    }
	}
	*/
	  //  HAL_Delay(15000);
	    //HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
#endif // ST7735_IS_128X128

}

void tftGPIO_init(void)
{
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
	LL_GPIO_InitTypeDef myGPIO;
	LL_GPIO_StructInit(&myGPIO);
	myGPIO.Mode			= LL_GPIO_MODE_OUTPUT;
	myGPIO.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
	myGPIO.Pin			=  LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
	myGPIO.Speed		= LL_GPIO_SPEED_LOW;
	LL_GPIO_Init(GPIOB, &myGPIO);
	

}

void spiSend(uint8_t *data, uint32_t len)
{
	//LL_SPI_Enable(SPI1);
	//SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SPE;
	//if errors occure check wrotew command functions and size of parameter
	// uint8_t *spidr = (( uint8_t *)&SPI1->DR);
	   uint8_t volatile *spidr = ((__IO uint8_t *)&SPI1->DR);

	while (len > 0)
	{

		while (!(SPI1->SR&SPI_SR_TXE)){
			;}

		*spidr = *data++;
		//LL_SPI_TransmitData8(SPI1,  *data++);



		//data += sizeof(uint8_t);
		len--;
	}
	
	while ((SPI1->SR&SPI_SR_BSY)) {
		;
	}
	
	
}


void tim2_ch1_pwm_init(void)
{

	//enable GPIOA and TIMER clocks
	LL_AHB2_GRP1_EnableClock(RCC_AHB2ENR_GPIOAEN); 
	LL_APB1_GRP1_EnableClock(RCC_APB1ENR1_TIM2EN);
	
	//setup GPIO for alt-func 1 
	LL_GPIO_InitTypeDef timGPIO;
	LL_GPIO_StructInit(&timGPIO);
	timGPIO.Mode =	LL_GPIO_MODE_ALTERNATE;
	timGPIO.Alternate =	LL_GPIO_AF_1;
	timGPIO.Speed  =	LL_GPIO_SPEED_HIGH;
	timGPIO.Pin     =	LL_GPIO_PIN_0;
	LL_GPIO_Init(GPIOA, &timGPIO); 
	
	//setup tim2 ch1 OC 
	LL_TIM_InitTypeDef tim2ch1;
	LL_TIM_StructInit(&tim2ch1);
	tim2ch1.Autoreload = 1000;
	tim2ch1.Prescaler  = 170;
	//LL_TIM_EnableARRPreload(TIM2);
	
	
	LL_TIM_OC_InitTypeDef tim2ch1_oc;
	LL_TIM_OC_StructInit(&tim2ch1_oc);
	tim2ch1_oc.CompareValue = 500; 
	tim2ch1_oc.OCIdleState  = LL_TIM_OCIDLESTATE_LOW;
	tim2ch1_oc.OCMode		= LL_TIM_OCMODE_PWM1;
	tim2ch1_oc.OCPolarity	= LL_TIM_OCPOLARITY_HIGH;
	tim2ch1_oc.OCState		= LL_TIM_OCSTATE_ENABLE;
	
	
	LL_TIM_Init(TIM2, &tim2ch1);
	LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &tim2ch1_oc);
	NVIC_EnableIRQ(TIM2_IRQn);
	LL_TIM_EnableIT_CC1(TIM2);
	LL_TIM_EnableCounter(TIM2);	

#ifdef USE__REG
	//enable clocks GPIOA TIM2
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	
	//setup GPIO A1 ALT FUNCTION 1
	GPIOA->MODER &= ~(GPIO_MODER_MODE0);
	GPIOA->MODER |= GPIO_MODER_MODE0_1;
	GPIOA->AFR[0] = GPIO_AFRL_AFSEL0_0;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0;
	
	//setup TIM2 CH1 OC
	TIM2->PSC = 170;
	TIM2->ARR =  1000;
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM2->CCR1 = 500;
	TIM2->CCER |= TIM_CCER_CC1E;	
	TIM2->CR1  |= TIM_CR1_CEN;
	
	
#endif
	
}

void delayMS(uint32_t ms)
{
	TIM4->CR1 |= TIM_CR1_CEN;
	CL_delayTicks = 0;
	while (CL_delayTicks < (ms)) ;
	TIM4->CR1 &= ~TIM_CR1_CEN;
}
void initSPI(void)
{
	
	// Config SPI1 on GPIOA ALT FUNC 5 
	// enable clocks	
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

	LL_GPIO_InitTypeDef spiPort;
	LL_GPIO_StructInit(&spiPort);
	
	spiPort.Alternate	= LL_GPIO_AF_5;
	spiPort.Mode		= LL_GPIO_MODE_ALTERNATE;
	spiPort.Pin			= LL_GPIO_PIN_5;
	spiPort.Speed		= LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOA, &spiPort);

	spiPort.Pin = LL_GPIO_PIN_7;
	LL_GPIO_Init(GPIOA, &spiPort);

	// config SPI
	LL_SPI_InitTypeDef mySPI;
	LL_SPI_StructInit(&mySPI); 
	

	
	
	mySPI.BaudRate			= LL_SPI_BAUDRATEPRESCALER_DIV8;
	mySPI.Mode				= LL_SPI_MODE_MASTER;
	mySPI.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
	mySPI.NSS				= LL_SPI_NSS_SOFT;
	mySPI.DataWidth			= LL_SPI_DATAWIDTH_8BIT;
	LL_SPI_Init(SPI1, &mySPI);
	LL_SPI_Enable(SPI1);	
	
	
}

void led0Setup(void)
{
 
	
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	
	LL_GPIO_InitTypeDef myGPIO;
	LL_GPIO_StructInit(&myGPIO);
	myGPIO.Mode			= LL_GPIO_MODE_OUTPUT;
	myGPIO.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
	myGPIO.Pin			= LED_A1;
	myGPIO.Speed		= LL_GPIO_SPEED_HIGH;
	LL_GPIO_Init(GPIOA, &myGPIO);
}
void timDelaySetup(void)
{	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	LL_TIM_InitTypeDef myTIM; 
	LL_TIM_StructInit(&myTIM);
	myTIM.Autoreload  = 1000;
	myTIM.Prescaler = 170;
		
	LL_TIM_SetUpdateSource(TIM4, LL_TIM_UPDATESOURCE_COUNTER);     //update even will only be set by over/undeflow of counter
	LL_TIM_EnableIT_UPDATE(TIM4);
	LL_TIM_GenerateEvent_UPDATE(TIM4);	
	LL_TIM_Init(TIM4, &myTIM);
	
	NVIC_EnableIRQ(TIM4_IRQn);	
}




void uart2Init(void)
{
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
	LL_USART_InitTypeDef myUART;
	LL_USART_StructInit(&myUART);	
}



void initSPI2(void)
{
	
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	//gpio set to alt function mode 
	GPIOA->MODER &=  ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE7);
	GPIOA->MODER |=  (GPIO_MODER_MODE5_1 | GPIO_MODER_MODE7_1);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_5);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_7);
	
	/*gpio set which alt function
		nothing to set in the alternate function registers because 
		by default they are set to 00 and 00 is the value we need to
		set it to SPI*/
	
	
	SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_MSTR  | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA;
	SPI1->CR2 = SPI_CR2_SSOE; 
	SPI1->CR1 |= SPI_CR1_SPE;

	
		
}




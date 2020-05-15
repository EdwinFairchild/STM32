#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_usart.h>
#include <stm32g4xx_ll_tim.h>
#include <stm32g4xx_ll_rcc.h>
#include <stm32g4xx_ll_system.h>
#include <stm32g4xx_ll_utils.h>
#include <stm32g4xx_ll_spi.h>
#include <stm32g4xx_ll_cortex.h>
#include "st7735.h"
#include "fonts.h"
#include "testimg.h"
#include "string.h"
#include "CL_systemClockUpdate.h"
#include "CL_delay.h"
#define LED_A1 LL_GPIO_PIN_1	

uint16_t msTICKS = 0x0000;
uint16_t msTicks = 0x0000;

void timDelaySetup(void);
void dms(int ms);
void uart2Init(void);
void led0Setup(void); 
void initSPI(void);
void spiSend(uint8_t * data, uint32_t len);
void initSPI2(void);
#define USE_DEBUG 

int main(void)
{
	setSysClockTo170();
#ifdef USE_DEBUG
	uart2Init();
	led0Setup();
#endif
	timDelaySetup(); 
	initSPI();
	//initSPI2();
	 uint8_t msg[] ="STM32G4";
	
	
	for (;;)
	{
		
		
		dms(500);
			spiSend(msg,7);
				//LL_SPI_TransmitData8(SPI1, msg[0]);
	
		
	}
}

void spiSend(uint8_t * data, uint32_t len)
{
	// uint8_t *spidr = (( uint8_t *)&SPI1->DR);
	while (len > 0)
	{			
		
		//*spidr		= *data;
		//SPI1->DR	= (uint8_t)(*data);
		LL_SPI_TransmitData8(SPI1, *data);
		while(!(SPI1->SR & SPI_SR_TXE));
		data += sizeof(uint8_t);
		len--;	
	}
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
	
	mySPI.BaudRate			= LL_SPI_BAUDRATEPRESCALER_DIV64;
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
	//	LL_TIM_GenerateEvent_UPDATE(TIM4);	
	LL_TIM_Init(TIM4, &myTIM);
	
	NVIC_EnableIRQ(TIM4_IRQn);	
}

void dms(int ms)
{
	TIM4->CR1 |= TIM_CR1_CEN;
	msTicks = 0;
	while (msTicks < (ms)) ;
	TIM4->CR1 &= ~TIM_CR1_CEN;
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




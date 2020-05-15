/*
 **/
#define USE__LL
//#define USE__REG
//#define USE__HAL

#ifdef USE__HAL
#include <stm32g4xx_hal.h>
#include <stm32_hal_legacy.h>

TIM_HandleTypeDef htim2;
static void MX_TIM2_Init(void);
#endif

#ifdef USE__LL  
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
#include "CL_systemClockUpdate.h"
#include "CL_delay.h"
#define LED_A1 LL_GPIO_PIN_1	

uint16_t msTICKS = 0x0000;
uint16_t msTicks = 0x0000;

void delay(uint16_t ms);
void timDelaySetup(void);
void dms(int ms);
void uart2Init(void);
void led0Setup(void); 
void tim2_ch1_pwm_init(void);
void initSPI(void);
#endif

#ifdef USE__REG

#include <stm32g431xx.h>
uint16_t msTICKS = 0x0000;
uint16_t msTicks = 0x0000;

void setSysClockTo170(void);

void timDelaySetup(void);
void dms(int ms);
void uart2Init(void);
void led0Setup(void); 
void tim2_ch1_pwm_init(void);

#endif


	
#ifdef USE__HAL 

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
void SystemClock_Config(void);
int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_TIM2_Init();
	__GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_1;

	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	for (;;)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_Delay(500);
	}
}
static void MX_TIM2_Init(void)
{
 
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
 
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 170;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1;     //---------->>>>>>>>> How to change the period from the while(1)<<<<<<<<<
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		//_Error_Handler(__FILE__, __LINE__);
	}
 
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		//_Error_Handler(__FILE__, __LINE__);
	}
 
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		//_Error_Handler(__FILE__, __LINE__);
	}
 
}
void SystemClock_Config(void)
{
 
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage 
	*/
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
	/** Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		//Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
	{
		//Error_Handler();
	}
	/** Initializes the peripherals clocks 
	*/
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C3;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		//Error_Handler();
	}
}
 
#endif

#ifndef USE__HAL
int main(void)
{
	
	setSysClockTo170();
	uart2Init();
	led0Setup();
	timDelaySetup();  
	tim2_ch1_pwm_init();
	initDelay(TIM3);
	
	while (1)
	{
#ifdef USE__LL
		/*LL_GPIO_SetOutputPin(GPIOA, LED_A1);
			//delay(50);
			dms(1000);
		LL_GPIO_ResetOutputPin(GPIOA, LED_A1);
			//delay(50);
			dms(1000);*/
		for (int i = 0; i < 1000; i += 10)
		{			
			LL_TIM_OC_SetCompareCH1(TIM2, i);
			dms(10);			
		}
		for (int i = 1000; i > 0; i -= 10)
		{			
			LL_TIM_OC_SetCompareCH1(TIM2, i);
			dms(10);			
		}
#endif
#ifdef USE__REG
		
		GPIOA->BSRR = GPIO_BSRR_BS0;
		dms(1000);
		GPIOA->BSRR = GPIO_BSRR_BR0;
		dms(1000);
		/*
		for (int i = 0; i < 1000; i += 10)
		{
			TIM2->CCR1 = i;
			dms(5);
		}
		for (int i = 1000; i > 0; i -= 10)
		{
			TIM2->CCR1 = i;
			dms(5);
		}*/
#endif
	}
}

#endif

#ifndef USE__HAL
void led0Setup(void)
{
#ifdef USE__LL 
	
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	
	LL_GPIO_InitTypeDef myGPIO;
	LL_GPIO_StructInit(&myGPIO);
	myGPIO.Mode			= LL_GPIO_MODE_OUTPUT;
	myGPIO.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
	myGPIO.Pin			= LED_A1;
	myGPIO.Speed		= LL_GPIO_SPEED_HIGH;
	LL_GPIO_Init(GPIOA, &myGPIO);
	
	/* Alternate way if setting up gpio.
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_0, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_LOW);
	*/

	
#endif

#ifdef USE__REG
	
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->MODER &= ~(GPIO_MODER_MODE0);
	GPIOA->MODER |= GPIO_MODER_MODE0_0;
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0);
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0_1;	
	
#endif
}
#endif

#ifndef USE__HAL
void tim2_ch1_pwm_init(void)
{
#ifdef USE__LL
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
	LL_TIM_EnableCounter(TIM2);	
#endif
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
#endif

#ifndef USE__HAL
void timDelaySetup(void)
{
#ifdef USE__LL
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	LL_TIM_InitTypeDef myTIM; 
	LL_TIM_StructInit(&myTIM);
	myTIM.Autoreload  = 1000;
	myTIM.Prescaler = 170;
		
	LL_TIM_SetUpdateSource(TIM4, LL_TIM_UPDATESOURCE_COUNTER);   //update even will only be set by over/undeflow of counter
	LL_TIM_EnableIT_UPDATE(TIM4);
	//	LL_TIM_GenerateEvent_UPDATE(TIM4);	
	LL_TIM_Init(TIM4, &myTIM);
	
#endif

#ifdef USE__REG
	
	RCC->APB1ENR1 |= (1 << 2);
	TIM4->PSC = 170;
	TIM4->ARR = 1000;
	TIM4->CR1 |= TIM_CR1_URS;
	TIM4->DIER |= TIM_DIER_UIE;
	TIM4->EGR |= TIM_EGR_UG;  
	
#endif
	
	NVIC_EnableIRQ(TIM4_IRQn);

	
	
}

#endif


#ifndef USE__HAL
void dms(int ms)
{
	TIM4->CR1 |= TIM_CR1_CEN;
	msTicks = 0;
	while (msTicks < (ms)) ;
	TIM4->CR1 &= ~TIM_CR1_CEN;
}
#endif


#ifndef USE__HAL
void uart2Init(void)
{
#ifdef USE__LL
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
	LL_USART_InitTypeDef myUART;
	LL_USART_StructInit(&myUART);
#else

#endif
	
}

#endif






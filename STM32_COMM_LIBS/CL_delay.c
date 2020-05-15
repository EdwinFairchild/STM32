
#include "CL_delay.h"

uint32_t CL_delayTicks = 0x000000;

#ifdef CL_DELAY_USE_TIM2
#define TIMER TIM2
#endif

#ifdef CL_DELAY_USE_TIM3
#define TIMER TIM3
#endif

#ifdef CL_DELAY_USE_TIM4
#define TIMER TIM4
#endif


#ifdef CL_USING_G4
void initDelay() //supports General Purpose TImers 2,3,4,
{
#ifdef CL_delay_USE_LL
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	LL_TIM_InitTypeDef myTIM; 
	LL_TIM_StructInit(&myTIM);
	//maybe insert us dependant code here
	myTIM.Autoreload  = 1000;
	myTIM.Prescaler = 170;
		
	LL_TIM_SetUpdateSource(TIM4, LL_TIM_UPDATESOURCE_COUNTER);    //update even will only be set by over/undeflow of counter
	LL_TIM_EnableIT_UPDATE(TIM4);
	//	LL_TIM_GenerateEvent_UPDATE(TIM4);	
	LL_TIM_Init(TIM4, &myTIM);
	
#else
			
	#ifdef CL_DELAY_US_ENABLE //change prescaler for micro second delay
	
		TIMER->ARR = 170;
	#else
		TIMER->PSC = 170;
		TIMER->ARR = 1000;
	#endif
		TIMER->CR1 |= TIM_CR1_URS;
		TIMER->DIER |= TIM_DIER_UIE;
		TIMER->EGR |= TIM_EGR_UG;  
	
#endif //CL_delay_USE_LL
	
#ifdef CL_DELAY_USE_TIM2
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	NVIC_EnableIRQ(TIM2_IRQn);		
#endif
	
#ifdef CL_DELAY_USE_TIM3
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
	NVIC_EnableIRQ(TIM3_IRQn);
#endif
		
#ifdef CL_DELAY_USE_TIM4
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
	NVIC_EnableIRQ(TIM4_IRQn);
#endif
	


}


//-----------------------------------


#ifdef CL_DELAY_US_ENABLE

	void delayMS(uint32_t ms)
	{
	
	}
	void delayUS(uint32_t us)
	{
	
	}

#else

	void delayMS(uint32_t ms)
	{
		TIMER->CR1 |= TIM_CR1_CEN;
		CL_delayTicks = 0;
		while (CL_delayTicks < (ms)) ;
		TIMER->CR1 &= ~TIM_CR1_CEN;
	}

#endif //CL_DELAY_US_ENABLE

#ifdef CL_DELAY_USE_TIM2
	void TIM2_IRQHandler(void)
	{
		TIM2->SR &= ~TIM_SR_UIF;
		CL_delayTicks++;
	}
#endif //CL_DELAY_USE_TIM2

#ifdef CL_DELAY_USE_TIM3
	void TIM3_IRQHandler(void)
	{
		TIM3->SR &= ~TIM_SR_UIF;
		CL_delayTicks++;
	}
#endif//CL_DELAY_USE_TIM3

#ifdef CL_DELAY_USE_TIM4
	void TIM4_IRQHandler(void)
	{
		TIM4->SR &= ~TIM_SR_UIF;
		CL_delayTicks++;
	}
#endif //CL_DELAY_USE_TIM4


#endif // CL_USING_G4
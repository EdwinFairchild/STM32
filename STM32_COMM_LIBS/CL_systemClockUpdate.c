#include "CL_systemClockUpdate.h"

#ifdef CL_USING_G4


void setSysClockTo170(void)
{
	
#ifdef CL_SYSTEMCLOCKUPDATE_USE_LL
	//voltage scaling
	LL_VREFBUF_SetVoltageScaling(LL_VREFBUF_VOLTAGE_SCALE1);
	// prefetch
	LL_FLASH_EnablePrefetch();
	//flash latency
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_8);
	//HSE external
	LL_RCC_HSE_Enable();
	while (!(LL_RCC_HSE_IsReady())) ;
	
	//AHB and APB prescalers
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
	//configure PLL and change clock source
	/*---------------------| Pll Configuration |----------------
	 * 1.Disable PLL_R or just clear entire register, PLL_r is the only thing set anyways
	 * 2.set PLL_R division to 2
	 * 3.Enable PLL_R	 * 
	 * 4.Set PLL input divsion factor (PLLM) to 1	 
	 * 5.Set multiplication factor of 16 into PLLN	  
	 * 6.Select PLL source HSE
	 * 7.Enable PLL
	 * 8.Change clock source
	 * 9. call SystemCoreClockUpdate(); 
	 **/
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 85, LL_RCC_PLLR_DIV_2);
	LL_RCC_PLL_Enable();
	LL_RCC_PLL_EnableDomain_SYS();

	while (!(LL_RCC_PLL_IsReady()));
	
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	SystemCoreClockUpdate();
	//volatile uint32_t temp = SystemCoreClock; //used to check changes worked

#else

	
	
	//voltage rang 1 is set by default
	
	
	//enable prefetch buffer
	FLASH->ACR |= FLASH_ACR_PRFTEN;
	
	//turn on high speed external
	RCC->CR |= RCC_CR_HSEON; 
	while (!(RCC->CR & RCC_CR_HSERDY)) ;
	
	
	FLASH->ACR |= FLASH_ACR_LATENCY_8WS; 
	
	//AHB and APB settings
	
	RCC->CFGR &= ~(RCC_CFGR_HPRE_3 | RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE2_2);
	
	/*---------------------| Pll Configuration |----------------
	 * 1.Disable PLL_R or just clear entire register, PLL_r is the only thing set anyways
	 * 2.set PLL_R division to 2
	 * 3.Enable PLL_R	 * 
	 * 4.Set PLL input divsion factor (PLLM) to 1	 
	 * 5.Set multiplication factor of 16 into PLLN	  
	 * 6.Select PLL source HSE
	 * 7.Enable PLL
	 * 8.Change clock source
	 * 9. call SystemCoreClockUpdate(); 
	 **/	 
	RCC->PLLCFGR = 0x00000000;      			// 1	
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLR);      	// 2	
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;      	// 3	
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM);      // 4	
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_0; 
	
	RCC->PLLCFGR |= (85 << RCC_PLLCFGR_PLLN_Pos);      	// 5 
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;      			//6	
	
	RCC->CR |= RCC_CR_PLLON;       //7
	while(!(RCC->CR & RCC_CR_PLLRDY));
	//---------------------| Pll Configuration |----------------
	
	//8
	RCC->CFGR |= RCC_CFGR_SW_PLL; 
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)) ;
	
	SystemCoreClockUpdate();
	//uncomment systick if needed
	//SysTick_Config(SystemCoreClock / 1000);
		
#endif	
	
	
}

#endif 

#ifndef CL_delay_H_
#define CL_delay_H_

#include <stdint.h>

#include "CL_CONFIG.h"

/* Use this section to configure settings*/



#ifdef CL_USING_G4

#include "stm32g4xx.h"



#ifdef CL_DELAY_USE_LL
	#include <stm32g4xx_ll_rcc.h>
	#include <stm32g4xx_ll_system.h>
	#include <stm32g4xx_ll_bus.h>
	#include <stm32g4xx_ll_tim.h>
#endif

#endif


void delayMS(uint32_t ms);
void delayUS(uint32_t us);
void initDelay();
#endif //end of header

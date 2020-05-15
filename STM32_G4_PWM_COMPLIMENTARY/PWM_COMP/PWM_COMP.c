#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_usart.h>
#include <stm32g4xx_ll_tim.h>
#include <stm32g4xx_ll_rcc.h>
#include <stm32g4xx_ll_system.h>
#include <stm32g4xx_ll_utils.h>
#include <stm32g4xx_ll_spi.h>
#include <stm32g4xx_ll_cortex.h>


#include "SystemClockUpdate.h"


int main(void)
{
	
	setSysClockTo170();

	for (;;)
	{
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);
		LL_mDelay(500);
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
		LL_mDelay(500);
	}
}

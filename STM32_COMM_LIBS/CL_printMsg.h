/*
	printMsg_config_Type printer;
	printer.TX_pinNumber = 9;
	printer.Uart_instance = USART1;
	printer.tx_port = GPIOA;
	printMsg_init(printer);

	
	printMsg("Hello world");
	
*/
#ifndef printMsg_H_
#define printMsg_H_

#include "stm32f10x.h"
#include <stdint.h>
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"


#define MODE_BIT_0_CRL	(4*config.TX_pinNumber)
#define MODE_BIT_1_CRL 	((4*config.TX_pinNumber)+1)
#define MODE_BIT_0_CRH	(4*(config.TX_pinNumber-8))
#define MODE_BIT_1_CRH 	((4*(config.TX_pinNumber-8))+1)


#define CNF_BIT_0_CRL 	((4*config.TX_pinNumber)+2)
#define CNF_BIT_1_CRL		((4*config.TX_pinNumber)+3)
#define CNF_BIT_0_CRH 	((4*(config.TX_pinNumber-8))+2)
#define CNF_BIT_1_CRH		((4*(config.TX_pinNumber-8))+3)



typedef struct //user structure to set port/pin settings
{
	USART_TypeDef 	*Uart_instance;
	GPIO_TypeDef	*tx_port;
	uint8_t 		TX_pinNumber;
	uint16_t 		baud;

}printMsg_config_Type;

void printMsg_init(printMsg_config_Type config);
void printMsg(char *msg, ...);


void printMsg_init(printMsg_config_Type config)
{
	//enable clocks
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; //alternate fucntion periph clock enable
	
	//TX pin port clock enable
	if(config.tx_port == GPIOA)
		 RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	if(config.tx_port == GPIOB)
		 RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	if(config.tx_port == GPIOC)
		 RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	/*may fill in more ports if available or needed
	if(config.tx_port == GPIOC)
		 RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;*/
	
	//enalbe USART clock
	if(config.Uart_instance == USART1)
		 RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	if(config.Uart_instance == USART2)
	 RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	if(config.Uart_instance == USART3)
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	//may add more as needed
	
	//configure TX pin as 50MHZ mode and Alt Function push pull cnf
	if(config.TX_pinNumber > 7) // config pin on CR-High
	{
		config.tx_port->CRH |= (1<<MODE_BIT_0_CRH) | (1<<MODE_BIT_1_CRH) | (1<<CNF_BIT_1_CRH) ;  	// mode = 11 :  cnf_1 = 1  
		config.tx_port->CRH &= ~(1<<CNF_BIT_0_CRH);    //cnf_0 = 0
	}
	else // config pin on CR-LOW
	{
		config.tx_port->CRL |= (1<<MODE_BIT_0_CRL) | (1<<MODE_BIT_1_CRL) | (1<<CNF_BIT_1_CRL) ;  	// mode = 11 :  cnf_1 = 1  
		config.tx_port->CRL &= ~(1<<CNF_BIT_0_CRL); //cnf_0 = 0		
	}
	
	//config USART BAUD RATE
	// clkPer / (baudR x 16 ) = 72Mhz / (9600 x 16) //uart 2 is at 36MHZ fix this when i get a chance
	// 468.75 = 0x1d4C 39.0625
	config.Uart_instance->BRR = 0x271 ;  //ill figure out how to get clock freq. so that this doesnt have to be hard coded
	config.Uart_instance->CR1 |=  USART_CR1_TE;
	config.Uart_instance->CR1 |= USART_CR1_UE;
	

	
}


void printMsg(char *msg, ...)
{
	
	char buff[80];

	
		va_list args;
		va_start(args,msg);
		vsprintf(buff,msg,args);
		
	  for(int i = 0 ; i < strlen(buff) ; i++)
		{
			USART1->DR = buff[i];
			while( !( USART1->SR & USART_SR_TXE )  ) ;
		}		
		

	
}


#endif


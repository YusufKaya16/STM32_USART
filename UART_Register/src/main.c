#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "string.h"
#include "main.h"

GPIO_InitTypeDef GPIO_InitStruct;
NVIC_InitTypeDef NVIC_InitStruct;

volatile char rx_buffer[10];
volatile int cnt = 0, StatusFlag = 0;

void RCC_Config(void){

	RCC->CR |= (1<<16);					//HSE ON.
	while(!(RCC->CR & (1<<17)));		//HSERDY FLAG CONTROL.
	RCC->CR |= (1<<19);					//CSS ON.
	RCC->PLLCFGR |= 0x00000000;			//PLL SETTING WAS RESET.
	RCC->PLLCFGR |= (1<<22);			//HSE SELECTED FOR PLLSRC.
	RCC->PLLCFGR |= (168<<6);			//PLL_N = 168.
	RCC->PLLCFGR |= (4<<0);				//PLL_M = 4.
	RCC->PLLCFGR |= 0x00000000;			//PLL_P = 2.
	RCC->CFGR |= (1<<1);
	RCC->CFGR &= ~(1<<0);				//PLL SELECTED FOR RCC.
	RCC->CFGR |= 0x00000000;			//AHB PRESCALER = 1.
	RCC->CFGR |= 0x00008000;			//APB2 PRESCALER = 2.
	RCC->CFGR |= 0x00001400;			//APB1 PRESCALER = 4.

	RCC->CIR |= 0x00008000;				//HSERDY CLEAN.
	RCC->CIR |= 0x00800000;				//CSS FLAG CLEAN.

}


void UART_Config(void){

	RCC->APB1ENR |= (1<<17);		//USART2 CLOCK ENABLE.
	USART2->CR1 |= (1<<2);			//USART2 RX ENABLE.
	USART2->CR1 |= (1<<5);			//USART2 RXNEIE ENABLE.
	USART2->CR1 |= (1<<3);			//USART2 TX ENABLE.
	USART2->CR1 &= ~(1<<12);		// 8 Bit data.
	USART2->CR2 &= ~(1<<12);
	USART2->CR2 &= ~(1<<13);		//STOP Bit = 1 bit.
	USART2->BRR = 0x1112;			//BaudRate = 9600.
	USART2->CR1 |= (1<<13);			//UART ENABLE.

	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;

	NVIC_Init(&NVIC_InitStruct);
}

void GPIO_Config(void){

	RCC->AHB1ENR |= (1<<0);					//GPIOA CLOCK ENABLE.
	GPIOA->MODER |= (1<<5) | (1<<7);		//GPIOA MODE = Alternatif Function.
	GPIOA->AFR[0] = (7<<8) | (7<<12);		//AF7 = 0111.		PA2 TX, PA3 RX

	//Relay trigger pin.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

}



void USART2_IRQHandler(void){

	//interrupt control.
	if((USART2->SR & (0x00000020)) && (USART2->CR1 & (0x00000020)) != 0){

		//Total 7 character.
		rx_buffer[cnt] = USART2->DR;
		cnt++;

		if (cnt == 7) {
			StatusFlag = 1;
			cnt = 0;
		}
	}
}


void Send_Byte(char data){

	while(!(USART2->SR & 0x00000080));			//TXE BUFFER BOS OLANA KADAR BEKLE.
	USART2->DR = data;
}

void Send_String(char *str){

	while(*str){
		Send_Byte(*str);
		str++;
	}
}

/* when the status is high, the relay is open */
void LedSet(void){

	GPIOD->ODR |= (1<<9);


}
void LedReset(void){

	GPIOD->ODR &= ~(1<<9);

}


int main(void)
{

	RCC_Config();
	UART_Config();
	GPIO_Config();

	GPIOD->ODR |= (1<<9);		//Relay is open.

  while (1)
  {


	  if(StatusFlag == 1){
		  StatusFlag = 0;
		  rx_buffer[7] = '\0';

		  if((strcmp((char *)rx_buffer, LEDYAK)) == 0){
			  LedReset();

			  Send_String("LEDLER YANDI\n");

		  }
		  if((strcmp((char *)rx_buffer, LEDSON)) == 0){
			  LedSet();
			  Send_String("LEDLER SONDU\n");
		  }
	  }
  }

}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}

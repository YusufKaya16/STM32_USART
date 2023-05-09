/*
 * main.h
 *
 *  Created on: 9 May 2023
 *      Author: Yusuf KAYA
 */

#ifndef MAIN_H_
#define MAIN_H_


#define LEDYAK	"led yak"
#define LEDSON	"led son"

/* Reset and Clock Control Configuration */
void RCC_Config(void);

/* GPIO Configuration */
void GPIO_Config(void);

/* USART Configuration */
void UART_Config(void);

/* USART interrupt Function */
void USART2_IRQHandler(void);

/* Send 1-Byte Data */
void Send_Byte(char data);

/* Send String */
void Send_String(char *str);

#endif /* MAIN_H_ */

#include "stm32f401xe.h"
#include <stdio.h>
#include <string.h>

// ---------- USART2 (debug via ST-Link) ----------
void USART2_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~((3U<<(2*2))|(3U<<(3*2)));
    GPIOA->MODER |=  (2U<<(2*2))|(2U<<(3*2));
    GPIOA->AFR[0]  &= ~((0xF<<(4*2))|(0xF<<(4*3)));
    GPIOA->AFR[0]  |=  (7U<<(4*2))|(7U<<(4*3));

    USART2->BRR = 16000000 / 115200;          // 16 MHz clock
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

int _write(int file, char *ptr, int len) {
    for (int i=0;i<len;i++) {
        while(!(USART2->SR & USART_SR_TXE));
        USART2->DR = ptr[i];
    }
    return len;
}

// ---------- USART1 (board-to-board comms) ----------
void USART1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    GPIOA->MODER &= ~((3U<<(9*2))|(3U<<(10*2)));
    GPIOA->MODER |=  (2U<<(9*2))|(2U<<(10*2));
    GPIOA->AFR[1]  &= ~((0xF<<((9-8)*4))|(0xF<<((10-8)*4)));//
    GPIOA->AFR[1]  |=  (7U<<((9-8)*4))|(7U<<((10-8)*4));

    USART1->BRR = 16000000 / 115200;          // 16 MHz clock
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void USART1_SendChar(char c){while(!(USART1->SR&USART_SR_TXE));USART1->DR=c;}
void USART1_SendString(char *s){
	while(*s){
		USART1_SendChar(*s++);
		printf("%s ",s);
	}
}
char USART1_ReadChar(void){while(!(USART1->SR&USART_SR_RXNE));return USART1->DR;}

void delay(void){for(volatile int i=0;i<5000000;i++);}

// ---------- main ----------
int main(void) {
    USART2_Init();
    USART1_Init();

    printf("=== CONTROL BOARD STARTED ===\r\n");

    char rx;
    char cmd1[]="LED ON\r\n";
    char cmd2[]="LED OFF\r\n";
    char cmd3[]="STATUS\r\n";

    printf("Waiting for Ready signal...\r\n");
//    while(1){
//
//        rx = USART1_ReadChar();
//        if(rx=='R')printf("\nif condition");
//
//        	//break;
//    }
    printf("Received Ready. Sending commands...\r\n");

//    USART1_SendString(cmd1); delay();
//    USART1_SendString(cmd2); delay();
//    USART1_SendString(cmd3);
    printf("Commands sent.\r\n");

    // Listen for responses (optional)
    while(1){
        USART1_SendString(cmd1); delay();
        USART1_SendString(cmd2); delay();
        USART1_SendString(cmd3); delay();
        //rx = USART1_ReadChar();
        //printf("%c", rx);
    }
}

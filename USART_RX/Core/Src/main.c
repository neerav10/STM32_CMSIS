#include "stm32f401xe.h"
#include <stdio.h>
#include <string.h>

// ---------- USART2 (debug) ----------
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

// ---------- USART1 (comms) ----------
void USART1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    GPIOA->MODER &= ~((3U<<(9*2))|(3U<<(10*2)));
    GPIOA->MODER |=  (2U<<(9*2))|(2U<<(10*2));
    GPIOA->AFR[1]  &= ~((0xF<<((9-8)*4))|(0xF<<((10-8)*4)));
    GPIOA->AFR[1]  |=  (7U<<((9-8)*4))|(7U<<((10-8)*4));

    USART1->BRR = 16000000 / 115200;          // 16 MHz clock
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void USART1_SendChar(char c){while(!(USART1->SR&USART_SR_TXE));USART1->DR=c;}
void USART1_SendString(char *s){while(*s)USART1_SendChar(*s++);}
char USART1_ReadChar(void){
	while(!(USART1->SR&USART_SR_RXNE));
	return USART1->DR;}

// ---------- LED ----------
void LED_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3U<<(5*2));
    GPIOA->MODER |=  (1U<<(5*2));
}
void LED_On(void){GPIOA->BSRR=(1U<<5);}
void LED_Off(void){GPIOA->BSRR=(1U<<(5+16));}

// ---------- main ----------
int main(void){
    USART2_Init();
    USART1_Init();
    LED_Init();

    printf("=== LED BOARD READY ===\r\n");
    USART1_SendString("Ready to receive commands\r\n");

    char buf[32]; int idx=0; int led=0;

    while(1){
        char c = USART1_ReadChar();
        printf("%c",c);
        buf[idx++]=c;
        if(c=='\n'){
            buf[idx]=0;
            printf("Received: %s", buf);

            if(strstr(buf,"LED ON")){
                LED_On(); led=1;
                USART1_SendString("LED is ON\r\n");
                printf("LED ON\r\n");
            } else if(strstr(buf,"LED OFF")){
                LED_Off(); led=0;
                USART1_SendString("LED is OFF\r\n");
                printf("LED OFF\r\n");
            } else if(strstr(buf,"STATUS")){
                if(led) USART1_SendString("LED is ON\r\n");
                else    USART1_SendString("LED is OFF\r\n");
                printf("STATUS sent\r\n");
            } else {
                USART1_SendString("Unknown command\r\n");
                printf("Unknown command\r\n");
            }
            idx=0;
        }
    }
}

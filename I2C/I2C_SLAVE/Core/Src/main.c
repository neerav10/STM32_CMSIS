#include "stm32f4xx.h"
#include <string.h>

#define SLAVE_ADDR 0x30
#define LED_PIN 5

void delay(volatile uint32_t d){while(d--);}

/* ---------------- UART for debug ---------------- */
void UART2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER |= (2 << (2*2)) | (2 << (3*2));
    GPIOA->AFR[0] |= (7 << (2*4)) | (7 << (3*4));

    USART2->BRR = 0x8B;
    USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

void UART2_SendString(const char *s)
{
    while(*s){
        while(!(USART2->SR & USART_SR_TXE));
        USART2->DR = *s++;
    }
}

/* ---------------- I2C Slave ---------------- */
void I2C1_Slave_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB8=SCL, PB9=SDA
    GPIOB->MODER &= ~((3<<(8*2)) | (3<<(9*2)));
    GPIOB->MODER |=  (2<<(8*2)) | (2<<(9*2));
    GPIOB->AFR[1]  |= (4<<0) | (4<<4);
    GPIOB->OTYPER |= (1<<8) | (1<<9);
    GPIOB->PUPDR  |= (1<<(8*2)) | (1<<(9*2)); // internal pull-ups

    // LED output
    GPIOA->MODER |= (1 << (LED_PIN*2));

    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;
    I2C1->OAR1 = (SLAVE_ADDR << 1);
    I2C1->CR2 = 16;
    I2C1->CR1 |= I2C_CR1_PE | I2C_CR1_ACK;
}

/* ---------------- MAIN ---------------- */
int main(void)
{
    UART2_Init();
    I2C1_Slave_Init();

    UART2_SendString("\r\nI2C SLAVE READY\r\n");

    char buf[32]; int idx = 0;

    while(1)
    {
        if(I2C1->SR1 & I2C_SR1_ADDR)
        {
            (void)I2C1->SR1;
            (void)I2C1->SR2;
        }

        if(I2C1->SR1 & I2C_SR1_RXNE)
        {
            char c = I2C1->DR;
            buf[idx++] = c;
            if(idx >= sizeof(buf)-1) idx = 0;

            if(strstr(buf, "LED ON"))
            {
                GPIOA->BSRR = (1<<LED_PIN);
                UART2_SendString("LED ON received → LED ON\r\n");
                idx = 0; memset(buf, 0, sizeof(buf));
            }
            else if(strstr(buf, "LED OFF"))
            {
                GPIOA->BSRR = (1<<(LED_PIN+16));
                UART2_SendString("LED OFF received → LED OFF\r\n");
                idx = 0; memset(buf, 0, sizeof(buf));
            }
        }


        if(I2C1->SR1 & I2C_SR1_STOPF)
        {
            (void)I2C1->SR1;
            I2C1->CR1 |= I2C_CR1_PE;
        }
    }
}

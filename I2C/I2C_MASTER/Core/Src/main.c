#include "stm32f4xx.h"
#include <string.h>

#define SLAVE_ADDR 0x30

void delay(volatile uint32_t d) { while(d--); }

/* ---------------- UART for debugging ---------------- */
void UART2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER |= (2 << (2*2)) | (2 << (3*2));
    GPIOA->AFR[0] |= (7 << (2*4)) | (7 << (3*4));

    USART2->BRR = 0x8B; // 115200 baud @16MHz
    USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

void UART2_SendString(const char *s)
{
    while(*s){
        while(!(USART2->SR & USART_SR_TXE));
        USART2->DR = *s++;
    }
}

/* ---------------- I2C Master ---------------- */
void I2C1_Master_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB8=SCL, PB9=SDA
    GPIOB->MODER &= ~((3 << (8*2)) | (3 << (9*2)));
    GPIOB->MODER |=  (2 << (8*2)) | (2 << (9*2)); // AF mode
    GPIOB->AFR[1]  |= (4 << 0) | (4 << 4);        // AF4
    GPIOB->OTYPER |= (1 << 8) | (1 << 9);         // open-drain
    GPIOB->PUPDR  |= (1 << (8*2)) | (1 << (9*2)); // internal pull-ups

    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;
    I2C1->CR2 = 16;      // 16 MHz PCLK1
    I2C1->CCR = 80;      // 100kHz standard mode
    I2C1->TRISE = 17;
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C1_Start(void)
{
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB));
}

void I2C1_Stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C1_Write(uint8_t data)
{
    I2C1->DR = data;
    while(!(I2C1->SR1 & I2C_SR1_BTF));
}

void I2C1_SendString(const char *s)
{
    I2C1_Start();
    I2C1->DR = (SLAVE_ADDR << 1); // Write mode
    while(!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR1; (void)I2C1->SR2;

    while(*s)
        I2C1_Write(*s++);

    I2C1_Stop();
}

/* ---------------- MAIN ---------------- */
int main(void)
{
    UART2_Init();
    I2C1_Master_Init();

    UART2_SendString("\r\nI2C MASTER READY\r\n");

    while(1)
    {
        UART2_SendString(">> Sending LED ON\r\n");
        I2C1_SendString("LED ON");
        delay(1000000);

        UART2_SendString(">> Sending LED OFF\r\n");
        I2C1_SendString("LED OFF");
        delay(1000000);
    }
}

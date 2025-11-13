// SPI Master
#include "stm32f401xe.h"
#include <string.h>
#include <stdint.h>

#define MSG_LEN 20

// Delay busy wait
void delay_ms(volatile uint32_t ms) {
    for (ms; ms > 0; ms--)
        for (volatile uint32_t i = 0; i < 2000; i++);
}

void SPI_Wait_Until_Done() {
    while (SPI2->SR & (1 << 7));
}

void SPI_Transmit(uint8_t *data, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        while (!(SPI2->SR & (1 << 1)));
        SPI2->DR = data[i];

        while (!(SPI2->SR & (1 << 0)));
        (void)SPI2->DR;
    }
    SPI_Wait_Until_Done();
}

void SPI_Receive(uint8_t *data, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        while (!(SPI2->SR & (1 << 1)));
        SPI2->DR = 0xFF;

        while (!(SPI2->SR & (1 << 0)));
        data[i] = SPI2->DR;
    }
    SPI_Wait_Until_Done();
}

void Master_Init(void) {
    RCC->AHB1ENR |= (1 << 0) | (1 << 1);
    RCC->APB1ENR |= (1 << 14);

    GPIOA->MODER &= ~(3 << 10);
    GPIOA->MODER |=  (1 << 10);

    GPIOB->MODER &= ~((3 << 24)|(3 << 26)|(3 << 28)|(3 << 30));
    GPIOB->MODER |=  ((2 << 24)|(2 << 26)|(2 << 28)|(2 << 30));

    GPIOB->AFR[1] &= ~((0xF<<16)|(0xF<<20)|(0xF<<24)|(0xF<<28));
    GPIOB->AFR[1] |=  ((5<<16)|(5<<20)|(5<<24)|(5<<28));

    GPIOB->OSPEEDR |= (3<<24)|(3<<26)|(3<<28)|(3<<30);

    SPI2->CR1 = 0;
    SPI2->CR1 |= (1<<1)|(1<<0);
    SPI2->CR1 |= (1<<2);
    SPI2->CR1 |= (0b100<<3);
    SPI2->CR1 &= ~(1<<9);
    SPI2->CR2 |= (1<<2);

    SPI2->CR1 |= (1<<6);
}

int main(void) {
    Master_Init();

    char tx_buf[MSG_LEN];
    char rx_buf[MSG_LEN];

    char cmd_on[]  = "Led ON";
    char cmd_off[] = "Led OFF";
    char reply_on[]  = "Led turned ON";
    char reply_off[] = "Led turned OFF";

    while (1) {
        memset(tx_buf, 0, MSG_LEN);
        strcpy(tx_buf, cmd_on);
        SPI_Transmit((uint8_t*)tx_buf, MSG_LEN);

        delay_ms(10);

        memset(rx_buf, 0, MSG_LEN);
        SPI_Receive((uint8_t*)rx_buf, MSG_LEN);

        if (strncmp(rx_buf, reply_on, 13) == 0) {
            GPIOA->BSRR = (1<<5);
        }

        delay_ms(1000);

        // ========== SEND LED OFF ==========
        memset(tx_buf, 0, MSG_LEN);
        strcpy(tx_buf, cmd_off);
        SPI_Transmit((uint8_t*)tx_buf, MSG_LEN);

        delay_ms(10);

        memset(rx_buf, 0, MSG_LEN);
        SPI_Receive((uint8_t*)rx_buf, MSG_LEN);

        if (strncmp(rx_buf, reply_off, 14) == 0) {
            GPIOA->BSRR = (1<<(5+16));
        }

        delay_ms(1000);
    }
}

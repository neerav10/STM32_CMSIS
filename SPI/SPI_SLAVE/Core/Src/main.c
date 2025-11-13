// SPI Slave
#include "stm32f401xe.h"
#include <string.h> // For strlen, strcmp
#include <stdint.h> // For uint8_t, uint32_t

#define MSG_LEN 20 // Fixed message length for our protocol

// --- Helper Functions ---

// Simple delay
void delay_ms(volatile uint32_t ms) {
    for (ms; ms > 0; ms--) {
        for (volatile uint32_t i = 0; i < 2000; i++);
    }
}

// Blocks until SPI transaction is complete (checks BSY flag)
void SPI_Wait_Until_Done() {
    while ((SPI2->SR & (1 << 7))) { // Wait for BSY (Busy) flag to clear
    }
}

// Transmit a block of data
void SPI_Transmit(uint8_t *data, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        while (!(SPI2->SR & (1 << 1))) {} // Wait for TXE (Transmit buffer empty)
        SPI2->DR = data[i];

        while (!(SPI2->SR & (1 << 0))) {} // Wait for RXNE (Receive buffer not empty)
        (void)SPI2->DR; // Read DR to clear RXNE flag (and discard received data)
    }
    SPI_Wait_Until_Done(); // Ensure all data is sent
}

// Receive a block of data (sends dummy bytes to clock)
void SPI_Receive(uint8_t *data, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        while (!(SPI2->SR & (1 << 1))) {} // Wait for TXE
        SPI2->DR = 0xFF; // Send dummy byte

        while (!(SPI2->SR & (1 << 0))) {} // Wait for RXNE
        data[i] = SPI2->DR; // Read received data
    }
    SPI_Wait_Until_Done();
}


void Slave_Init(void) {
    // 1. Enable Clocks (GPIOA, GPIOB, SPI2)
    RCC->AHB1ENR |= (1 << 0); // Enable GPIOA clock (for on-board LED PA5)
    RCC->AHB1ENR |= (1 << 1); // Enable GPIOB clock (for SPI2 pins)
    RCC->APB1ENR |= (1 << 14); // Enable SPI2 clock

    // 2. Configure GPIOA (PA5 for LED)
    GPIOA->MODER &= ~(3 << 10); // Clear bits 10 & 11
    GPIOA->MODER |= (1 << 10); // Set PA5 to General purpose output mode

    // 3. Configure GPIOB (PB12, PB13, PB14, PB15 for SPI2)
    // Set PB12, PB13, PB14, PB15 to Alternate Function mode
    GPIOB->MODER &= ~((3 << 24) | (3 << 26) | (3 << 28) | (3 << 30)); // Clear bits
    GPIOB->MODER |= (2 << 24) | (2 << 26) | (2 << 28) | (2 << 30); // Set to AF mode

    // Set AF5 (SPI2) for PB12, PB13, PB14, PB15
    GPIOB->AFR[1] &= ~((0xF << 16) | (0xF << 20) | (0xF << 24) | (0xF << 28)); // Clear
    GPIOB->AFR[1] |= (5 << 16) | (5 << 20) | (5 << 24) | (5 << 28); // Set AF5

    // Set pin speed to Very High Speed
    GPIOB->OSPEEDR |= (3 << 24) | (3 << 26) | (3 << 28) | (3 << 30);

    // 4. Configure SPI2 (Slave)
    SPI2->CR1 = 0; // Clear all

    // Set CPOL=1, CPHA=1 (Mode 3) - MUST MATCH MASTER
    SPI2->CR1 |= (1 << 1) | (1 << 0);

    // MSTR = 0 (Slave mode - this is default)
    // SSM = 0 (Hardware management - this is default)

    // 5. Enable SPI
    SPI2->CR1 |= (1 << 6); // SPE = 1 (Enable SPI)
}

int main(void) {
    Slave_Init();

    char rx_buffer[MSG_LEN];
    char tx_buffer[MSG_LEN];

    char cmd_on[] = "Led ON";
    char cmd_off[] = "Led OFF";
    char reply_on[] = "Led turned ON";
    char reply_off[] = "Led turned OFF";
    char reply_err[] = "Error";

    while (1) {
        // 1. Wait to receive a command from the master
        memset(rx_buffer, 0, MSG_LEN);
        SPI_Receive((uint8_t*)rx_buffer, MSG_LEN);

        // 2. Process the command and prepare the reply
        if (strcmp(rx_buffer, cmd_on) == 0) {
            GPIOA->BSRR = (1 << 5); // Turn ON Slave's LED
            memset(tx_buffer, 0, MSG_LEN);
            strcpy(tx_buffer, reply_on);
        }
        else if (strcmp(rx_buffer, cmd_off) == 0) {
            GPIOA->BSRR = (1 << (5 + 16)); // Turn OFF Slave's LED
            memset(tx_buffer, 0, MSG_LEN);
            strcpy(tx_buffer, reply_off);
        }
        else {
            // Unknown command
            memset(tx_buffer, 0, MSG_LEN);
            strcpy(tx_buffer, reply_err);
        }

        // 3. Send the prepared reply back to the master
        // This will happen when the master initiates its "Receive" phase
        SPI_Transmit((uint8_t*)tx_buffer, MSG_LEN);
    }
}

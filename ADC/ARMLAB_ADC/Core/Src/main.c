#include "stm32f4xx.h"

// Define threshold for ADC value
#define THRESHOLD 2000

// Simple delay function for visible LED blinking
void delay(volatile uint32_t count) {
    while(count--) {
        __NOP();
    }
}

int main(void) {
    // 1. Enable GPIOA clock (RCC-AHB1ENR)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // 2. Enable ADC1 clock (RCC-APB2ENR)
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // 3. Configure PA5 as output (GPIOA-MODER, bits 10-11 = 01)
    GPIOA->MODER &= ~(0x3 << (5 * 2)); // Clear bits for PA5
    GPIOA->MODER |= (0x1 << (5 * 2));  // Set PA5 to general purpose output

    // 4. Configure PA0 as analog (GPIOA-MODER, bits 0-1 = 11)
    GPIOA->MODER &= ~(0x3 << (0 * 2)); // Clear bits for PA0
    GPIOA->MODER |= (0x3 << (0 * 2));  // Set PA0 to analog mode

    // 5. Set output type of PA5 to push-pull (GPIOA-OTYPER, bit 5 = 0)
    GPIOA->OTYPER &= ~(1 << 5);

    // 6. Set output speed for PA5 to low speed (GPIOA-OSPEEDR, bits 10-11 = 00)
    GPIOA->OSPEEDR &= ~(0x3 << (5 * 2));

    // 7. No pull-up, pull-down for PA5 (GPIOA-PUPDR, bits 10-11 = 00)
    GPIOA->PUPDR &= ~(0x3 << (5 * 2));

    // 8. ADC1 configuration
    // Set channel 0 (PA0) as first in regular sequence (ADC1-SQR3)
    ADC1->SQR3 &= ~0x1F;   // Clear first conversion bits
    ADC1->SQR3 |= 0;       // Channel 0 selected

    // Select sample time for channel 0 (ADC1-SMPR2)
    // Let's set to 3 cycles (minimum), bits 0-2 for channel 0
    ADC1->SMPR2 &= ~0x7;
    ADC1->SMPR2 |= 0x3;

    // Enable ADC (ADC1-CR2)
    ADC1->CR2 |= ADC_CR2_ADON;

    // Small delay to stabilize ADC after enabling
    delay(1000);

    while(1) {
        // Start conversion
        ADC1->CR2 |= ADC_CR2_SWSTART;

        // Wait until conversion is complete (ADC1-SR, EOC bit)
        while(!(ADC1->SR & ADC_SR_EOC));

        // Read ADC data (ADC1-DR)
        uint16_t adc_value = ADC1->DR;

        // Check if adc_value > threshold
        if(adc_value > THRESHOLD) {
            // Turn ON LED: set PA5 (GPIOA-ODR or GPIOA-BSRR)
            GPIOA->BSRR = (1 << 5);
        } else {
            // Turn OFF LED: reset PA5
            GPIOA->BSRR = (1 << (5 + 16)); // Reset bit
        }

        // Delay before next sample
        delay(50000);
    }
}

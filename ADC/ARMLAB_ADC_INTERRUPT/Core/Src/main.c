#include "stm32f4xx.h"

#define THRESHOLD 2000

volatile uint16_t adc_value = 0;

void delay(volatile uint32_t count) {
    while(count--) {
        __NOP();
    }
}

void ADC_IRQHandler(void) {
    if (ADC1->SR & ADC_SR_EOC) { // Check End Of Conversion flag
        adc_value = ADC1->DR;    // Read data clears EOC flag

        if (adc_value > THRESHOLD) {
            GPIOA->BSRR = (1U << 5);           // Turn ON LED (PA5 set)
        } else {
            GPIOA->BSRR = (1U << (5 + 16));    // Turn OFF LED (PA5 reset)
        }
    }
}

int main(void) {
    // Enable GPIOA clock (bit 0)
    RCC->AHB1ENR |= (1U << 0);

    // Enable ADC1 clock (bit 8)
    RCC->APB2ENR |= (1U << 8);

    // Set PA5 as output (MODER5 = 01)
    GPIOA->MODER &= ~(0x3U << (5 * 2));
    GPIOA->MODER |=  (0x1U << (5 * 2));

    // PA5 output push-pull
    GPIOA->OTYPER &= ~(1U << 5);

    // PA5 low speed
    GPIOA->OSPEEDR &= ~(0x3U << (5 * 2));

    // PA5 no pull-up, no pull-down
    GPIOA->PUPDR &= ~(0x3U << (5 * 2));

    // Set PA0 as analog mode (MODER0 = 11)
    GPIOA->MODER &= ~(0x3U << (0 * 2));
    GPIOA->MODER |=  (0x3U << (0 * 2));

    // ADC1 channel 0 first conversion in regular sequence
    ADC1->SQR3 &= ~0x1FU;
    ADC1->SQR3 |= 0;

    // ADC1 sample time channel 0 = 56 cycles (bits 2:0)
    ADC1->SMPR2 &= ~(0x7U << 0);
    ADC1->SMPR2 |= (0x3U << 0);

    // Enable end of conversion interrupt
    ADC1->CR1 |= ADC_CR1_EOCIE;

    // Enable ADC1
    ADC1->CR2 |= ADC_CR2_ADON;

    // Enable ADC interrupt in NVIC
    NVIC_EnableIRQ(ADC_IRQn);

    // Start ADC conversion (software trigger)
    ADC1->CR2 |= ADC_CR2_SWSTART;

    while(1) {
        delay(100000);
        // Restart ADC conversion
        ADC1->CR2 |= ADC_CR2_SWSTART;
    }
}

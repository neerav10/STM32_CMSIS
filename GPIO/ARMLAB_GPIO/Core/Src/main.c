#include "stm32f4xx.h"

#define LED_PIN     5   // PA5
#define BUTTON_PIN  13  // PC13

void delay(volatile uint32_t count) {
    while (count--) {}
}

int main(void) {
    // Enable GPIOA & GPIOC clocks
    RCC->AHB1ENR |= (1 << 0) | (1 << 2);

    // PA5 as output
    GPIOA->MODER &= ~(3 << (LED_PIN * 2));
    GPIOA->MODER |=  (1 << (LED_PIN * 2));

    // PC13 as input
    GPIOC->MODER &= ~(3 << (BUTTON_PIN * 2));

    while (1) {
        if ((GPIOC->IDR & (1 << BUTTON_PIN)) == 0) { // button pressed
            GPIOA->ODR ^= (1 << LED_PIN);            // toggle LED
            delay(500000);
        }
    }
}

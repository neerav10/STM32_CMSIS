#include "stm32f4xx.h"  // Device header for STM32F401RE

// Interrupt handler for EXTI lines 15 to 10 (handles EXTI13 here)
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1 << 13))  // Check if EXTI13 caused the interrupt
    {
        GPIOA->ODR ^= (1 << 5); // Toggle LED on PA5

        EXTI->PR |= (1 << 13);  // Clear pending interrupt flag by writing 1
    }
}

int main(void)
{
    // 1. Enable clock for GPIOA and GPIOC (AHB1ENR)
    RCC->AHB1ENR |= (1 << 0) | (1 << 2);  // GPIOAEN (bit0) + GPIOCEN (bit2)

    // 2. Enable clock for SYSCFG (APB2ENR)
    RCC->APB2ENR |= (1 << 14); // SYSCFGEN (bit14)

    // 3. Configure PA5 as output (MODER = 01)
    GPIOA->MODER &= ~(0x3 << (5 * 2)); // Clear bits for pin 5
    GPIOA->MODER |= (0x1 << (5 * 2));  // Set pin 5 as output

    // 4. Configure PC13 as input (MODER = 00)
    GPIOC->MODER &= ~(0x3 << (13 * 2)); // Clear bits for pin 13 (input)
    // 5. Set PA5 output type to push-pull (OTYPER = 0)
    GPIOA->OTYPER &= ~(1 << 5);

    // 6. Set PA5 speed to low (OSPEEDR = 00)
    GPIOA->OSPEEDR &= ~(0x3 << (5 * 2));

    // 7. No pull-up, no pull-down on PA5 (PUPDR = 00)
    GPIOA->PUPDR &= ~(0x3 << (5 * 2));

    // 8. Map EXTI13 to PC13 in SYSCFG_EXTICR4 (bits 7:4)
    SYSCFG->EXTICR[3] &= ~(0xF << 4);  // Clear EXTI13 bits
    SYSCFG->EXTICR[3] |= (0x2 << 4);   // Set EXTI13 source to port C

    // 9. Unmask interrupt for EXTI13 (IMR)
    EXTI->IMR |= (1 << 13);

    // 10. Trigger interrupt on falling edge (FTSR)
    EXTI->FTSR |= (1 << 13);
    EXTI->RTSR &= ~(1 << 13);  // Disable rising edge trigger if set

    // 11. Enable EXTI15_10_IRQn interrupt in NVIC
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    // 12. Initialize LED OFF
    GPIOA->ODR &= ~(1 << 5);

    // Main loop
    while (1)
    {
        // Can add low power or idle here if desired
    }
}

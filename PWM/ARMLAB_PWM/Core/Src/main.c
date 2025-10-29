#include "stm32f4xx.h"
#define DUTY_CYCLE 75        // Duty cycle in percent (0-100)
#define PWM_FREQ    1000      // PWM Frequency in Hz
#define SYSCLK      84000000  // APB1 Timer clock for F401RE (Hz)

// A simple delay function
void delay(volatile uint32_t t) {
    while(t--);
}

int main(void) {
    // 1. Enable GPIOA and TIM3 clocks
    RCC->AHB1ENR |= (1U << 0); // GPIOAEN
    RCC->APB1ENR |= (1U << 1); // TIM3EN

    // 2. Set PA6 to Alternate Function mode
    GPIOA->MODER &= ~(0x3U << (6*2));
    GPIOA->MODER |=  (0x2U << (6*2));

    // 3. Set Alternate Function AF2 (TIM3_CH1) for PA6
    GPIOA->AFR[0] &= ~(0xFU << (6*4));
    GPIOA->AFR[0] |=  (0x2U << (6*4));

    // 4. Timer base configuration for 1 kHz PWM
    TIM3->PSC = 83;     // (84MHz/84) = 1MHz timer clock
    TIM3->ARR = 999;    // (1MHz/1kHz) - 1

    // 5. Set CCR1 value for Duty Cycle
    TIM3->CCR1 = ((DUTY_CYCLE * (TIM3->ARR + 1)) / 100) - 1;

    // 6. Set PWM mode 1 and enable preload
    TIM3->CCMR1 &= ~(0x7U << 4);      // Clear previous OC1M
    TIM3->CCMR1 |=  (0x6U << 4);      // OC1M = 110 (PWM1 mode)
    TIM3->CCMR1 |=  (1U << 3);        // OC1PE (preload enable)

    // 7. Enable channel 1 output
    TIM3->CCER |= (1U << 0);          // CC1E

    // 8. Enable auto-reload preload and start timer
    TIM3->CR1 |= (1U << 7);           // ARPE
    TIM3->CR1 |= (1U << 0);           // CEN

    while(1) {
         // Keep running; PWM works in hardware
    }
}


////------------------------------------FadeIn and FadeOut for checking LED------------------
//#include "stm32f4xx.h"
//
//#define PWM_FREQ    1000      // PWM Frequency in Hz
//#define SYSCLK      84000000  // APB1 Timer clock for F401RE (Hz)
//
//// A simple delay function
//void delay(volatile uint32_t t) {
//    while(t--);
//}
//
//int main(void) {
//    // 1. Enable GPIOA and TIM3 clocks
//    RCC->AHB1ENR |= (1U << 0); // GPIOAEN
//    RCC->APB1ENR |= (1U << 1); // TIM3EN
//
//    // 2. Set PA6 to Alternate Function mode
//    GPIOA->MODER &= ~(0x3U << (6*2));
//    GPIOA->MODER |=  (0x2U << (6*2));
//
//    // 3. Set Alternate Function AF2 (TIM3_CH1) for PA6
//    GPIOA->AFR[0] &= ~(0xFU << (6*4));
//    GPIOA->AFR[0] |=  (0x2U << (6*4));
//
//    // 4. Timer base configuration
//    uint32_t prescaler = 84 - 1;   // (84MHz / 84) = 1MHz timer clock
//    uint32_t period = 1000 - 1;    // (1MHz / 1kHz) - 1 = 999 for ARR
//
//    TIM3->PSC = prescaler;  // Prescaler value
//    TIM3->ARR = period;     // Auto reload register (period)
//
//    // 5. Configure PWM mode 1 and enable preload for Channel 1
//    TIM3->CCMR1 &= ~(0x7U << 4);     // Clear previous OC1M
//    TIM3->CCMR1 |=  (0x6U << 4);     // OC1M = 110 (PWM1 mode)
//    TIM3->CCMR1 |=  (1U << 3);       // OC1PE (preload enable)
//
//    // 6. Enable channel 1 output
//    TIM3->CCER |= (1U << 0);         // CC1E
//
//    // 7. Enable auto-reload preload and start timer
//    TIM3->CR1 |= (1U << 7);          // ARPE
//    TIM3->CR1 |= (1U << 0);          // CEN
//
//    // The main loop now fades the LED in and out
//    while(1) {
//        // Fade In (0% to 100% duty cycle)
//        for (uint16_t i = 0; i < period; i++) {
//            TIM3->CCR1 = i;
//            delay(500); // Adjust this delay to change fade speed
//        }
//
//        // Fade Out (100% to 0% duty cycle)
//        for (uint16_t i = period; i > 0; i--) {
//            TIM3->CCR1 = i;
//            delay(500); // Adjust this delay to change fade speed
//        }
//    }
//}




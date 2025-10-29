//#include "stm32f4xx.h"
//
//#define PWM_FREQ   1000       // PWM frequency in Hz
//#define DUTY_CYCLE 50         // Duty cycle in percent
//#define SYSCLK     84000000   // System clock frequency in Hz
//
//void delay(volatile uint32_t t) { while(t--); }
//
//int main(void) {
//    // 1. Enable GPIOA and GPIOB clocks (pins PA8 and PB13)
//    RCC->AHB1ENR |= (1U << 0) | (1U << 1);  // GPIOAEN and GPIOBEN
//
//    // 2. Enable Timer 1 clock
//    RCC->APB2ENR |= (1U << 11);  // TIM1EN bit 11 in APB2ENR
//
//    // 3. Configure PA8 as Alternate Function (AF1 for TIM1_CH1)
//    GPIOA->MODER &= ~(0x3U << (8 * 2));    // Clear mode bits for PA8
//    GPIOA->MODER |=  (0x2U << (8 * 2));    // Set to AF mode
//
//    GPIOA->AFR[1] &= ~(0xFU << ((8 - 8) * 4)); // AFRH for PA8 (8th pin)
//    GPIOA->AFR[1] |=  (0x1U << ((8 - 8) * 4)); // AF1 = TIM1_CH1
//
//    // 4. Configure PB13 as Alternate Function (AF1 for TIM1_CH1N)
//    GPIOB->MODER &= ~(0x3U << (13 * 2));   // Clear mode bits for PB13
//    GPIOB->MODER |=  (0x2U << (13 * 2));   // Set to AF mode
//
//    GPIOB->AFR[1] &= ~(0xFU << ((13 - 8) * 4)); // AFRH for PB13
//    GPIOB->AFR[1] |=  (0x1U << ((13 - 8) * 4)); // AF1 = TIM1_CH1N
//
//    // 5. Timer base configuration for 1 kHz PWM
//    uint32_t prescaler = 84 - 1;   // (84MHz / 84) = 1MHz timer clock
//    uint32_t period = 1000 - 1;    // (1MHz / 1kHz) - 1 = 999 for ARR
//
//    TIM1->PSC = prescaler;  // Prescaler value
//    TIM1->ARR = period;     // Auto reload register (period)
//
//
//    // 6. Set duty cycle on CCR1
//    TIM1->CCR1 = (DUTY_CYCLE * (TIM1->ARR + 1)) / 100;
//
//    // 7. Configure PWM mode 1 on CH1 and enable preload
//    TIM1->CCMR1 &= ~(0x7U << 4);          // Clear OC1M bits
//    TIM1->CCMR1 |= (0x6U << 4);           // PWM mode 1
//    TIM1->CCMR1 |= (1U << 3);             // OC1PE enable preload
//
//    // 8. Enable CH1 and CH1N outputs
//    TIM1->CCER |= (1U << 0) | (1U << 2); // CC1E and CC1NE enable
//
//    // 9. Configure dead-time and enable main output
//    TIM1->BDTR &= ~(0xFFU);               // Clear DTG bits
//    TIM1->BDTR |= (84U);          // Dead-time ~1us (assuming 84MHz), DTG = Prescaler / Dead time
//    TIM1->BDTR |= (1U << 15);             // MOE: Main output enable
//
//    // 10. Enable auto-reload preload and start timer
//    TIM1->CR1 |= (1U << 7);               // ARPE enable
//    TIM1->CR1 |= (1U << 0);               // Counter enable
//
//    while (1) {
//        delay(1000000); // Loop to keep running
//    }
//}


#include "stm32f4xx.h"
//===============================================
// Complementary PWM using TIM1_CH1 & TIM1_CH1N
// PA8  -> TIM1_CH1
// PB13 -> TIM1_CH1N
//===============================================
void PWM_ComplementaryPair_Init(void)
{
    // 1. Enable clocks for GPIOA, GPIOB, and TIM1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;//Enable both GPIO port A and B
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;// Enable the timer 1 peripheral
//The timer 1 peripheral has the complementary pair PWM with Dead time and break features

    //==================================================
    // 2. Configure GPIOA PA8 as Alternate Function (AF1)
    //==================================================
    GPIOA->MODER &= ~(3U << (8 * 2));       // Clear mode bits
    GPIOA->MODER |=  (2U << (8 * 2));       // Alternate function mode
    GPIOA->OTYPER &= ~(1U << 8);            // Push-pull output
    GPIOA->OSPEEDR |= (3U << (8 * 2));      // Very high speed
    GPIOA->AFR[1] &= ~(0xFU << 0);          // Clear AF bits for PA8
    GPIOA->AFR[1] |=  (1U << 0);            //,
    // so AFR is a 32 bit register each pin corresponds 4 bits each 4 bit combination is different AF mode, AF1--> timer 1 mode

    //==================================================
    // 3. Configure GPIOB PB13 as Alternate Function (AF1)
    //==================================================
    GPIOB->MODER &= ~(3U << (13 * 2));      // Clear mode bits
    GPIOB->MODER |=  (2U << (13 * 2));      // Alternate function mode
    GPIOB->OTYPER &= ~(1U << 13);           // Push-pull
    GPIOB->OSPEEDR |= (3U << (13 * 2));     // Very high speed
    GPIOB->AFR[1] &= ~(0xFU << ((13 - 8) * 4));  // Clear AF bits, AFR[1] is for pins 8-15
    GPIOB->AFR[1] |=  (1U << ((13 - 8) * 4));    // AF1 = TIM1_CH1N

    //==================================================
    // 4. Timer 1 Configuration
    //==================================================
    TIM1->PSC = 83;            // Prescaler: (84 MHz / (83+1)) = 1 MHz timer clock
    TIM1->ARR = 1000-1;          // Auto-reload: 1 kHz PWM frequency
    TIM1->CCR1 = 500;          // 50%  duty cycle

    // PWM Mode 1 on CH1, preload enable
    TIM1->CCMR1 &= ~(7U << 4);
    TIM1->CCMR1 |=  (6U << 4);          // OC1M = 110 (PWM mode 1)
    TIM1->CCMR1 |=  TIM_CCMR1_OC1PE;    // Preload enable

    // Enable CH1 and CH1N outputs, normal polarity
    TIM1->CCER = 0;                     // Clear any previous settings
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE; // Enable main + complementary

    //==================================================
    // 5. Dead-Time and Main Output Enable
    //==================================================
    TIM1->BDTR = 0;
    TIM1->BDTR |= 84;             // Small dead-time (~a few 100ns)
    TIM1->BDTR |= TIM_BDTR_MOE;         // Main output enable, sets the CH1 and CHN to pins to output the PWM pulse

    //==================================================
    // 6. Enable Timer
    //==================================================
    TIM1->CR1 |= TIM_CR1_ARPE;          // Enable auto-reload preload
    TIM1->CR1 |= TIM_CR1_CEN;           // Start Timer 1
}

int main(void)
{
    PWM_ComplementaryPair_Init();
    while(1);
}







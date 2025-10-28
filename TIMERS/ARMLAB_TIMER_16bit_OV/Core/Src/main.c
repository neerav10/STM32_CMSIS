// TIM3 (16-bit), handling overflow logic correctly for GPIO LED blink

#include "stm32f4xx.h"
#define LED 5

void delay_16bit_timer(uint32_t delay_ms) {
    uint32_t prescaler = 16000 - 1;   // Timer clock = 16 MHz / 16000 = 1 kHz → 1 ms per tick
    uint32_t max_arr   = 0x1388;      // 16-bit maximum (65535 ms)

    // Configure prescaler once
    TIM3->PSC = prescaler;
    TIM3->CR1 = 0x0;      // reset control register
    TIM3->EGR = (1 << 0); // UG: update registers

    while (delay_ms > 0) {
        // Use either full 16-bit range or remaining delay
        uint32_t current_delay = (delay_ms > max_arr) ? max_arr : delay_ms;

        TIM3->ARR = current_delay; // auto-reload value
        TIM3->CNT = 0;             // reset counter
        TIM3->EGR = (1 << 0);      // update ARR
        TIM3->SR  = 0;             // clear update flag
        TIM3->CR1 |= (1 << 0);     // enable counter

        // wait until UIF = 1 (update event = overflow)
        while (!(TIM3->SR & (1 << 0)));

        TIM3->SR  &= ~(1 << 0);    // clear UIF
        TIM3->CR1 &= ~(1 << 0);    // stop counter

        delay_ms -= current_delay;
    }
}

int main(void) {
    // Enable clock for GPIOA (bit 0 in AHB1ENR)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure PA5 as output (LED on Nucleo board)
    GPIOA->MODER &= ~(0x3 << (LED * 2));   // clear mode
    GPIOA->MODER |=  (0x1 << (LED * 2));   // set as output
    GPIOA->OTYPER &= ~(1 << LED);          // push-pull
    GPIOA->OSPEEDR &= ~(0x3 << (LED * 2)); // low speed
    GPIOA->PUPDR &= ~(0x3 << (LED * 2));   // no pull-up/down

    // Enable clock for Timer3 (bit 1 in APB1ENR)
    RCC->APB1ENR |= (1 << 1);

    while (1) {
        GPIOA->ODR ^= (1 << LED);   // toggle LED
        delay_16bit_timer(6000);    // delay
    }
}

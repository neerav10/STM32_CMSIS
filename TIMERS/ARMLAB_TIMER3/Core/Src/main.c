#include "stm32f4xx.h"
#define LED  5
void delay_timer2(void);


int main(void){
	// Enable GPIOA clock (RCC->AHB1ENR)
	    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	    // Configure PA5 as output (GPIOA->MODER)
	    GPIOA->MODER &= ~(0x3 << (LED * 2));     // Clear mode bits for PA5
	    GPIOA->MODER |= (0x1 << (LED * 2));      // Set PA5 as general purpose output

	    // Output type push-pull (GPIOA->OTYPER)
	    GPIOA->OTYPER &= ~(1 << LED);

	    // Output speed low (GPIOA->OSPEEDR)
	    GPIOA->OSPEEDR &= ~(0x3 << (LED * 2));

	    // No pull-up, pull-down (GPIOA->PUPDR)
	    GPIOA->PUPDR &= ~(0x3 << (LED * 2));
while(1){
	delay_timer2();
	GPIOA->ODR ^= (1  <<  LED);
}
}
 void delay_timer2(void){
	 RCC->APB1ENR |= (1 << 0);  // TIM2EN bit
	 TIM2->PSC = 16000 - 1;  // Prescaler 1 kHz  fCK_PSC / (PSC[15:0] + 1).
	 TIM2->ARR = 1000 - 1;   // {Autoreload} 1 second delay (1ms * 999)
	 TIM2->CNT = 0;
	 TIM2->EGR |= (1 << 0);  // UG bit = bit 0 is set ; //{Event generation, update generation} This forces the timer to load PSC and ARR values immediately
	 TIM2->SR &= ~ (1 << 0);  // Clear update interrupt flag (UIF = bit 0) in status register //Overflow has occured

	TIM2->CR1 |= (1 << 0); //Enable Timer

	 	 while (!(TIM2->SR & (1 << 0))); //until UIF NEQ 1
	 		 TIM2->SR &= ~(1 << 0); //UIF is cleared manually
	 		 TIM2->CR1 &= ~(1 << 0); //CEN is cleared

	}



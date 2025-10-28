//////////////////...........................................ONLY HALL.................................////////////////////////////////

//#include "stm32f4xx.h"           // CMSIS device header
//#include "stm32f4xx_hal.h"         // STM32 HAL header
//
//// Relay control pins on GPIOA
//#define VIBRATION_MOTOR_PIN   GPIO_PIN_0  // PA0
//#define DEHUMIDIFIER_PIN      GPIO_PIN_1  // PA1
//#define EXHAUST_FAN_PIN       GPIO_PIN_4  // PA4
//
//void SystemClock_Config(void);  // Prototype for clock setup
//
//void Relay_GPIO_Init(void) {
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//    GPIO_InitStruct.Pin = VIBRATION_MOTOR_PIN | DEHUMIDIFIER_PIN | EXHAUST_FAN_PIN;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//}
//
//void Relay_On(uint16_t pin) {
//    HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_SET);
//}
//
//void Relay_Off(uint16_t pin) {
//    HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_RESET);
//}
//
//int main(void) {
//    HAL_Init();               // Initialize HAL library (SysTick, HAL tick, etc.)
//    SystemClock_Config();     // Configure system clock for correct SysTick timing
//
//    Relay_GPIO_Init();
//
//
//
//
//
//
//
//    //main code
//    while (1) {
//        Relay_On(VIBRATION_MOTOR_PIN);
//        HAL_Delay(1000);  // 1 second delay using HAL
//        Relay_Off(VIBRATION_MOTOR_PIN);
//
//
//        Relay_On(DEHUMIDIFIER_PIN);
//        HAL_Delay(1000);
//        Relay_Off(DEHUMIDIFIER_PIN);
//
//
//        Relay_On(EXHAUST_FAN_PIN);
//        HAL_Delay(1000);
//        Relay_Off(EXHAUST_FAN_PIN);
//    }
//}
//
//
//
//
//
//
//
//
//
//
//
//
//// Basic system clock configuration for STM32F401 used by HAL_Delay()
//void SystemClock_Config(void)
//{
//    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//    __HAL_RCC_PWR_CLK_ENABLE();
//    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//    RCC_OscInitStruct.PLL.PLLM = 16;
//    RCC_OscInitStruct.PLL.PLLN = 336;
//    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
//    RCC_OscInitStruct.PLL.PLLQ = 7;
//
//    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
//        while(1); // Initialization Error
//    }
//
//    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
//                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
//    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
//        while(1); // Initialization Error
//    }
//}







/////////////////////.......................CMSIS(GPIO) and HAL(DELAY)................///////////////////////////////////////////////////

#include "stm32f4xx.h"           // CMSIS device header
#include "stm32f4xx_hal.h"       // STM32 HAL header

// Relay control pins on GPIOA (pin numbers, not HAL macros)
#define VIBRATION_MOTOR_PIN   0  // PA0
#define DEHUMIDIFIER_PIN      1  // PA1
#define EXHAUST_FAN_PIN       4  // PA4

void SystemClock_Config(void);   // Prototype for clock setup

void Relay_GPIO_Init(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Set PA0, PA1, PA4 as output (MODER: 0x01 per pin)
    GPIOA->MODER &= ~((3U << (VIBRATION_MOTOR_PIN * 2)) |
                      (3U << (DEHUMIDIFIER_PIN * 2)) |
                      (3U << (EXHAUST_FAN_PIN * 2)));
    GPIOA->MODER |= ((1U << (VIBRATION_MOTOR_PIN * 2)) |
                     (1U << (DEHUMIDIFIER_PIN * 2)) |
                     (1U << (EXHAUST_FAN_PIN * 2)));

    // Set output type to push-pull
    GPIOA->OTYPER &= ~((1U << VIBRATION_MOTOR_PIN) |
                       (1U << DEHUMIDIFIER_PIN) |
                       (1U << EXHAUST_FAN_PIN));

    // Set speed to low
    GPIOA->OSPEEDR &= ~((3U << (VIBRATION_MOTOR_PIN * 2)) |
                        (3U << (DEHUMIDIFIER_PIN * 2)) |
                        (3U << (EXHAUST_FAN_PIN * 2)));

    // Set no-pull
    GPIOA->PUPDR &= ~((3U << (VIBRATION_MOTOR_PIN * 2)) |
                      (3U << (DEHUMIDIFIER_PIN * 2)) |
                      (3U << (EXHAUST_FAN_PIN * 2)));
}

void Relay_On(uint8_t pin) {
    GPIOA->BSRR = (1U << pin);          // Set pin HIGH
}

void Relay_Off(uint8_t pin) {
    GPIOA->BSRR = (1U << (pin + 16));   // Set pin LOW
}

int main(void) {
    HAL_Init();               // Initialize HAL library (SysTick, HAL tick, etc.)
    SystemClock_Config();     // Configure system clock for correct SysTick timing

    Relay_GPIO_Init();

    // Main code
    while (1) {
        Relay_On(VIBRATION_MOTOR_PIN);
        HAL_Delay(1000);      // 1 second delay using HAL
        Relay_Off(VIBRATION_MOTOR_PIN);

        Relay_On(DEHUMIDIFIER_PIN);
        HAL_Delay(1000);
        Relay_Off(DEHUMIDIFIER_PIN);

        Relay_On(EXHAUST_FAN_PIN);
        HAL_Delay(1000);
        Relay_Off(EXHAUST_FAN_PIN);
    }
}

// Basic system clock configuration for STM32F401 used by HAL_Delay()
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while(1); // Initialization Error
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        while(1); // Initialization Error
    }
}


/****************************************************************
* Author: T. LUM and R. MARTIN
* CPEG222 DemoProgram8, 10/6/26
* NucleoF446ZE CMSIS STM32F4xx example F26Demo8_Standard_Servo
* Generates a 50 Hz PWM signal on PC6 (TIM3_CH1, shield Servo0 "Radar")
* to control a standard (positional) servo such as the MG996R.
* The servo moves automatically (no input needed):
*   Part 1: starts at 90 and pauses 3 seconds to allow servo horn placement,
*         then jumps to 0, 90, 180, then back to 90 degrees (1 second each)
*   Part 2: sweeps smoothly 0 -> 180 -> 0 degrees, 1 degree every 20 ms
* System clock is 180 MHz, so the TIM3 clock is 90 MHz (APB1 45 MHz x 2).
*
* Revision History:
* V1  10/6/26  Initial version
****************************************************************/
#include "stm32f4xx.h"       // Include the STM32F4xx CMSIS header file
#include "SysClock180MHz.h"  // 180 MHz system clock configuration
#include <stdint.h>          // Include stdint.h for uint32_t type

#define SERVO0_PORT    GPIOC
#define SERVO0_PIN     6               // PC6 = TIM3_CH1 (AF2), shield Servo0 (Radar)
#define TIMER_CLOCK    90000000UL      // TIM3 clock at 180 MHz SYSCLK (APB1 45 MHz x 2)
#define PWM_PERIOD_US  20000           // 20,000 us = 20 ms period = 50 Hz
#define SERVO_MIN_US   800            // .8 ms pulse =   0 degrees (ish)
#define SERVO_MID_US   1500            // 1.5 ms pulse =  90 degrees (neutral / center)
#define SERVO_MAX_US   2200            // 2.2 ms pulse = 180 degrees (ish)
// Note: actual servo travel is almost always less than 180 degrees, and some
// manufacturers can reverse the direction so keep fingers clear when first starting.

void PWM_Output_PC6_Init(void);
void Servo_SetPulse(uint32_t pulse_us);
void Servo_SetAngle(uint32_t angle);
void delay_ms(uint32_t ms);

int main(void) {
    SystemClock_Config();    // Configure the system clock to 180 MHz
    SystemCoreClockUpdate(); // Update SystemCoreClock variable after clock change
    PWM_Output_PC6_Init();   // Start 50 Hz PWM on PC6, servo starts at neutral
    delay_ms(1000);          // Give the servo time to reach neutral

    while (1) {
        // Part 1: jump to fixed positions
        Servo_SetAngle(90);
        delay_ms(3000);
        Servo_SetAngle(0);
        delay_ms(1000);
        Servo_SetAngle(90);
        delay_ms(1000);
        Servo_SetAngle(180);
        delay_ms(1000);
        Servo_SetAngle(90);
        delay_ms(1000);
        // Part 2: smooth sweep, one step per PWM period (20 ms)
        for (uint32_t angle = 0; angle <= 180; angle++) {
            Servo_SetAngle(angle);
            delay_ms(20);
        }
        for (uint32_t angle = 180; angle > 0; angle--) {
            Servo_SetAngle(angle);
            delay_ms(20);
        }
        delay_ms(1000);
    }
    return 0; // Program should never reach here due to infinite loop
}

void PWM_Output_PC6_Init(void) {
    // 1. Enable GPIOC and TIM3 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    // 2. Set PC6 to alternate function mode (MODER = 10b)
    SERVO0_PORT->MODER &= ~(0x3 << (SERVO0_PIN * 2));
    SERVO0_PORT->MODER |=  (0x2 << (SERVO0_PIN * 2));
    // 3. Select AF2 (TIM3_CH1) for PC6. Pins 0-7 use AFR[0], pins 8-15 use AFR[1]
    SERVO0_PORT->AFR[0] &= ~(0xF << (SERVO0_PIN * 4));
    SERVO0_PORT->AFR[0] |=  (0x2 << (SERVO0_PIN * 4));
    // 4. Configure TIM3 for PWM output on CH1
    TIM3->CR1 &= ~TIM_CR1_CEN;                   // Disable TIM3 before configuring it
    TIM3->PSC = (TIMER_CLOCK / 1000000UL) - 1;   // 90 MHz / 90 = 1 MHz timer clock (1 us per tick)
    TIM3->ARR = PWM_PERIOD_US - 1;               // 20,000 ticks = 20 ms period (50 Hz)
    TIM3->CCR1 = SERVO_MID_US;                   // 1,500 ticks = 1.5 ms pulse (neutral)
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1M); // CH1 as output, clear output mode
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);    // PWM mode 1: high while CNT < CCR1
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;              // Preload enable: new CCR1 takes effect at next period
    TIM3->CCER |= TIM_CCER_CC1E;                 // Enable CH1 output on PC6
    TIM3->CR1 |= TIM_CR1_ARPE;                   // Auto-reload preload enable
    TIM3->EGR = TIM_EGR_UG;                      // Generate update event to load PSC, ARR, CCR1
    TIM3->CR1 |= TIM_CR1_CEN;                    // Enable timer
}

void Servo_SetPulse(uint32_t pulse_us) {
    // Limit the pulse to the safe range so the servo never hits its stops
    if (pulse_us < SERVO_MIN_US) {
        pulse_us = SERVO_MIN_US;
    } else if (pulse_us > SERVO_MAX_US) {
        pulse_us = SERVO_MAX_US;
    }
    TIM3->CCR1 = pulse_us; // 1 tick = 1 us, so CCR1 is the pulse width in us
}

void Servo_SetAngle(uint32_t angle) {
    // Map 0-180 degrees to 800-2200 us
    if (angle > 180) {
        angle = 180;
    }
    Servo_SetPulse(SERVO_MIN_US + (angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180);
}

void delay_ms(uint32_t ms) {
    if (SystemCoreClock == 16000000U) { // If the system clock is 16 MHz
        for (uint32_t i = 0; i < ms * 2000; i++) {
            __NOP();
        }
    } else if (SystemCoreClock == 180000000U) { // If the system clock is 180 MHz
        for (uint32_t i = 0; i < ms * 22500; i++) {
            __NOP();
        }
    }
}

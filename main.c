

#include "stm32f411xe.h"

void SysClockConfig(void)
{
    // configure flash related settings
    FLASH->ACR |= (FLASH_ACR_ICEN) | (FLASH_ACR_PRFTEN) | (FLASH_ACR_DCEN) | FLASH_ACR_LATENCY_3WS;

    // enable the power interface clock and configure the voltage regulator
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    // enable the external crystal resonator and wait for the HSE to be ready
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // configure prescalers for clock
    RCC->CFGR &= ~(0xFUL << RCC_CFGR_HPRE_Pos);
    RCC->CFGR |= (0x4UL << RCC_CFGR_PPRE1_Pos);
    RCC->CFGR |= (0x5UL << RCC_CFGR_PPRE2_Pos);

    // configure the PLL
    RCC->PLLCFGR = (0xCUL << RCC_PLLCFGR_PLLM_Pos) | (0x60UL << RCC_PLLCFGR_PLLN_Pos) | (0x4UL << RCC_PLLCFGR_PLLQ_Pos);
    RCC->PLLCFGR &= ~(0x3UL << RCC_PLLCFGR_PLLP_Pos);
    RCC->PLLCFGR |= (0x1UL << RCC_PLLCFGR_PLLSRC_HSE_Pos);
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while(!(RCC->CFGR & (2 << 2)));
}

void TIM9Config(void)
{
    // Enable timer clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
    
    //Set the prescalar and the ARR register
    TIM9->PSC = 50 - 1;
    TIM9->ARR = 0xFFFF;

    // Enable timer and wait for update flag to set
    TIM9->CR1 |= (1<<0); // Enable counter
    while (!(TIM9->SR & (1<<0)));
}

void GPIOInit(void)
{
    // enable gpio clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // set the pin as output
    GPIOC->MODER |= (0x1UL << 26U);

    // configure the output mode
    GPIOC->OTYPER = 0;
    GPIOC->OSPEEDR = 0;
}

void delay_us(uint16_t us)
{
    TIM9->CNT = 0;
    while(TIM9->CNT < us);
}

void delay_ms(uint16_t ms)
{
    for (uint16_t i = 0; i < ms; i++)
        delay_us(1000);
}

int main(void) {

    SysClockConfig();
    GPIOInit();
    TIM9Config();

    while(1) {
        GPIOC->BSRR |= (1UL << 13);
        delay_ms(500);
        GPIOC->BSRR |= ((1UL << 13) << 16);
        delay_ms(500);
    }
    return 0;
}


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

void delay(uint32_t time)
{
    while (time--);
}

int main(void) {

    SysClockConfig();
    GPIOInit();

    while(1) {
        GPIOC->BSRR |= (1UL << 13);
        delay(1000000);
        GPIOC->BSRR |= ((1UL << 13) << 16);
        delay(1000000);
    }
    return 0;
}
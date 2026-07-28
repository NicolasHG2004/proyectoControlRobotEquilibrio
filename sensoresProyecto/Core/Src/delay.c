#include "delay.h"

void delay_init(void)
{
    // Habilitar el contador DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t nus)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = nus * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

void delay_ms(uint16_t nms)
{
    HAL_Delay(nms);
}

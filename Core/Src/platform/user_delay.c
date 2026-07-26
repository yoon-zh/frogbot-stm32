//user_delay.c
//2023.10.20
//by gjx

#include "user_delay.h"

static uint8_t dwt_cycle_counter_enabled = 0u;

void HAL_Delay_us(uint32_t us)
{
    uint32_t cycles_per_us;
    uint32_t start_cycles;

    if (us == 0u)
    {
        return;
    }

    if (dwt_cycle_counter_enabled == 0u)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0u;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        dwt_cycle_counter_enabled = 1u;
    }

    cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000u;
    start_cycles = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start_cycles) < (us * cycles_per_us))
    {
    }
}




#include "TI_TIMER.h"
#include <stdint.h>

static uint32_t calcTimerPeriod(uint32_t freq_hz, uint32_t cpuclk_hz)
{
    if (freq_hz == 0U) return 0xFFFFFFFFUL;

    // Timer period = CPUCLK / freq  (period register is "period - 1")
    return (cpuclk_hz / freq_hz) - 1UL;
}

void TI_TIMER_InitHz(uint32_t TIMER_BASE, uint32_t freq_hz, uint32_t cpuclk_hz)
{
    const uint32_t period = calcTimerPeriod(freq_hz, cpuclk_hz);

    // Stop timer
    CPUTimer_stopTimer(TIMER_BASE);

    // Set period and prescaler (0 => divide by 1)
    CPUTimer_setPeriod(TIMER_BASE, period);
    CPUTimer_setPreScaler(TIMER_BASE, 0U);

    // Reload counter
    CPUTimer_reloadTimerCounter(TIMER_BASE);

    // Enable interrupt generation from the timer
    CPUTimer_enableInterrupt(TIMER_BASE);

    // Start timer
    CPUTimer_startTimer(TIMER_BASE);
}

void TI_TIMER_SetFreqHz(uint32_t TIMER_BASE, uint32_t freq_hz, uint32_t cpuclk_hz)
{
    const uint32_t period = calcTimerPeriod(freq_hz, cpuclk_hz);

    CPUTimer_stopTimer(TIMER_BASE);                 // stop
    CPUTimer_setPeriod(TIMER_BASE, period);         // update period
    CPUTimer_reloadTimerCounter(TIMER_BASE);        // reload
    CPUTimer_startTimer(TIMER_BASE);                // start
}

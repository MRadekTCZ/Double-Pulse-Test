#ifndef TI_TIMER_H_
#define TI_TIMER_H_

#include "driverlib.h"
#include "device.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void TI_TIMER_InitHz(uint32_t TIMER_BASE, uint32_t freq_hz, uint32_t cpuclk_hz);
void TI_TIMER_SetFreqHz(uint32_t TIMER_BASE, uint32_t freq_hz, uint32_t cpuclk_hz);

#ifdef __cplusplus
}
#endif

#endif /* TI_TIMER_H_ */

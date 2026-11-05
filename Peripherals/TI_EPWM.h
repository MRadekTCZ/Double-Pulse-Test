
#ifndef TI_EPWM_H_
#define TI_EPWM_H_

#include <stdint.h>
#include "device.h"

#define SLOW_HZ             20000UL
#define SLOW_COUNTS         (DEVICE_SYSCLK_FREQ / SLOW_HZ)
#define SLOW_TBPRD          ((uint16_t)(SLOW_COUNTS - 1UL))
#define COUNT_US            (DEVICE_SYSCLK_FREQ / 1000000UL)
#define AT_US(us)           ((uint16_t)((uint32_t)(us) * COUNT_US))

#define SHORT_HALF_US       4UL
#define SHORT_PERIOD_US     (2UL * SHORT_HALF_US)
#define FAST_TBPRD          (AT_US(SHORT_PERIOD_US) - 1U)
#define FAST_ON_CMP         AT_US(SHORT_HALF_US)

#define NS_TO_COUNTS(ns) \
    ((uint16_t)((((uint64_t)(ns) * DEVICE_SYSCLK_FREQ) + \
    500000000ULL) / 1000000000ULL))

#define PREEDGE_NS          50U
#define PREEDGE_COUNTS \
    ((COUNT_US * PREEDGE_NS + 500UL) / 1000UL)

#define SWEEP_PULSES        6U
#define FLYBACK_TEST_ISR_HZ SLOW_HZ

#define SWEEP_FINAL_INF_TBPRD \
    ((uint16_t)(FAST_TBPRD + 1U - PREEDGE_COUNTS))

#if ((DEVICE_SYSCLK_FREQ % 1000000UL) != 0UL)
#error "SYSCLK must be a multiple of 1 MHz"
#endif

#if ((SLOW_COUNTS < 2UL) || (SLOW_COUNTS > 65535UL))
#error "Slow ePWM period out of range"
#endif

#if ((SHORT_PERIOD_US * COUNT_US) > 65535UL)
#error "Fast ePWM period out of range"
#endif

#if ((PREEDGE_COUNTS == 0UL) || \
     (PREEDGE_COUNTS >= SHORT_HALF_US * COUNT_US))
#error "50 ns preedge delay cannot be generated"
#endif

// Flyback parameters from DPT.c.
extern volatile uint32_t flyback_pwm_frequency_hz;
extern volatile uint16_t flyback_pwm_enable;
extern volatile uint16_t flyback_vdd_control_enable;
extern volatile float flyback_target_vdd;
extern volatile float flyback_pwm_duty;

// Hardware diagnostics.
extern volatile uint32_t epwm5_isr_count;
extern volatile float flyback_vdd_applied;
extern volatile float flyback_pwm_actual_hz;
extern volatile float flyback_pwm_actual_duty;
extern volatile uint16_t flyback_pwm_config_error;

typedef void (*TI_EPWM_TickCallback)(void);

typedef enum
{
    TI_EPWM_FAST_SINGLE = 0,
    TI_EPWM_FAST_FOUR_PULSE = 1,
    TI_EPWM_FAST_SWEEP = 2
} TI_EPWM_FastSequence;

// Initialization.
void TI_EPWM_Init(void);
void TI_EPWM_RegisterInterrupt(TI_EPWM_TickCallback on_tick);

// Gate control.
void TI_EPWM_ForceIN(uint16_t high);
void TI_EPWM_ForceINF(uint16_t high);
void TI_EPWM_ForceBothLow(void);
void TI_EPWM_ReleaseIN(void);
void TI_EPWM_ReleaseINF(void);

// DPT, QPT and sweep operations.
void TI_EPWM_PrepareSlowPeriod(uint16_t charging);
void TI_EPWM_PrepareChargingINF(void);
void TI_EPWM_ChargingMidpointINF(void);
void TI_EPWM_SelectArmingInterrupt(void);

void TI_EPWM_ArmFastSequence(
    TI_EPWM_FastSequence mode,
    uint16_t dpt_off_cmp,
    uint16_t dpt_end_cmp,
    uint16_t dpt_done_cmp,
    uint16_t first_sweep_delay_counts);

void TI_EPWM_QPTStep(uint16_t index);

void TI_EPWM_SweepPreparePositive(
    uint16_t previous_delay_counts,
    uint16_t next_delay_counts);

void TI_EPWM_SweepPrepareNegative(
    uint16_t previous_delay_counts);

void TI_EPWM_SuppressFutureTurnOn(void);

void TI_EPWM_StopOutputs(
    uint16_t is_single_dpt,
    uint16_t boost_latched);

// Continuous switching PWM.
uint16_t TI_EPWM_ContinuousPWMFrequencyValid(
    uint32_t frequency_hz);

uint16_t TI_EPWM_ArmContinuousDuty(
    uint32_t frequency_hz,
    float duty);

void TI_EPWM_SetContinuousDuty(float duty);

// Temporary flyback PID duty override.
void TI_EPWM_SetFlybackPidOverride(
    uint16_t enable,
    float pid_duty);

// Flyback voltage control.
void TI_EPWM_SetFlybackVdd(float vdd);
void TI_EPWM_UpdateFlybackPWM(void);

#endif

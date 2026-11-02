
/*
 * Modular refactor of DPT_QPT_SWEEP_FLYBACK_TEST.c
 * F28P65x / C2000 DriverLib.
 * Timing and mode behavior retained from source.
 */
#include "driverlib.h"
#include "device.h"
#include "Source/DPT_tests.h"
#include "Peripherals/TI_EPWM.h"

/*
 * Edit these variables in CCS Expressions. The other modules use extern
 * declarations in DPT_tests.h and TI_EPWM.h; do not define them twice.
 *
 * Modes: 0=STOP, 1=DPT, 2=DPT_SWEEP, 3=QPT, 4=FLYBACK_TEST,
 *        5=SWITCHING_UG_TEST (50% PWM after inductor charge).
 */
volatile AutomatedTest automated_test = STOP;
volatile uint32_t automated_test_cnt = 0UL;

volatile uint32_t dpt_initial_wait_ticks = 200UL;
volatile uint32_t dpt_first_pulse_ticks = 25UL;
volatile uint16_t dpt_freewheel_us = 2U;
volatile uint16_t dpt_second_pulse_us = 2U;
volatile uint16_t passive_boost = 0U;

volatile uint32_t flyback_pwm_frequency_hz = 12000UL;
volatile uint16_t flyback_pwm_enable = 1U;
volatile uint16_t flyback_vdd_control_enable = 1U;
volatile float flyback_target_vdd = 15.0f;    // Startup target: 15 V
volatile float flyback_pwm_duty = 0.09483871f;
volatile uint32_t flyback_test_step_ms = 100UL;

/*
 * SWITCHING_UG_TEST (mode 5), set before each test in CCS Expressions:
 *  - Switching at 50% duty after standard coil charging
 *  - Gate supply steps: 20 V, 18 V, 15 V, 12 V, then 0 V and OFF
 *  - WARNING: 50% duty DOES NOT regulate inductor current.
 *    An independent current limiter / hardware protection is necessary.
 *  - Deliberate opt-in, resets to 0 after the run.
 */
volatile uint32_t switching_ug_frequency_hz = 20000UL;
volatile uint32_t switching_ug_step_ms = 2000UL;
volatile uint16_t switching_ug_inf_high = 1U;
volatile uint16_t switching_ug_openloop_arm = 0U;

void main(void)
{
    Device_init();
    Device_initGPIO();

    // Preserve startup 15 V command and original flyback calibration.
    TI_EPWM_SetFlybackVdd(flyback_target_vdd);
    TI_EPWM_Init();

    Interrupt_initModule();
    Interrupt_initVectorTable();
    TI_EPWM_RegisterInterrupt(DPT_OnEPWM5Tick);
    TI_EPWM_UpdateFlybackPWM();

    EINT;
    ERTM;

    // All time-critical test sequencing runs from the ePWM5 ISR.
    for (;;)
    {
        if (flyback_vdd_control_enable != 0U)
        {
            TI_EPWM_SetFlybackVdd(flyback_target_vdd);
        }
        TI_EPWM_UpdateFlybackPWM();
    }
}

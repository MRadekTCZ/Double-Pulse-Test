
#include "driverlib.h"
#include "device.h"
#include "Source/DPT_tests.h"
#include "Peripherals/TI_EPWM.h"

volatile AutomatedTest automated_test = STOP;
volatile uint32_t automated_test_cnt = 0UL;

volatile uint32_t dpt_initial_wait_ticks = 200UL;
volatile uint32_t dpt_first_pulse_ticks = 25UL;
volatile uint16_t dpt_freewheel_us = 2U;
volatile uint16_t dpt_second_pulse_us = 2U;
volatile uint16_t passive_boost = 0U;

// Flyback supply, startup at 15 V.
volatile uint32_t flyback_pwm_frequency_hz = 12000UL;
volatile uint16_t flyback_pwm_enable = 1U;
volatile uint16_t flyback_vdd_control_enable = 1U;
volatile float flyback_target_vdd = 0.0f;
volatile float flyback_pwm_duty = 0.09483871f;
volatile uint32_t flyback_test_step_ms = 2000UL;

// SWITCHING_UG_TEST = 5.
// Each Ug stage lasts 500 ms.
// A temporary flyback PWM duty of 0.16 is applied
// for 20, 40, 70, or 100 ms depending on Ug.
// Switching duty is 52%, 54%, 56%, and 58%.
volatile uint32_t switching_ug_frequency_hz = 20000UL;
volatile uint32_t switching_ug_step_ms = 500UL;
volatile float switching_ug_pid_duty = 0.16f;
volatile uint16_t switching_ug_inf_high = 1U;
volatile uint16_t switching_ug_openloop_arm = 0U;

void main(void)
{
    Device_init();
    Device_initGPIO();

    TI_EPWM_SetFlybackVdd(flyback_target_vdd);
    TI_EPWM_Init();

    Interrupt_initModule();
    Interrupt_initVectorTable();
    TI_EPWM_RegisterInterrupt(DPT_OnEPWM5Tick);
    TI_EPWM_UpdateFlybackPWM();

    EINT;
    ERTM;

    for (;;)
    {
        if (flyback_vdd_control_enable != 0U)
        {
            TI_EPWM_SetFlybackVdd(flyback_target_vdd);
        }

        TI_EPWM_UpdateFlybackPWM();
    }
}


#ifndef DPT_TESTS_H_
#define DPT_TESTS_H_

#include <stdint.h>

typedef enum
{
    STOP = 0,
    DPT = 1,
    DPT_SWEEP = 2,
    QPT = 3,
    FLYBACK_TEST = 4,
    SWITCHING_UG_TEST = 5
} AutomatedTest;

// Configuration variables in DPT.c.
extern volatile AutomatedTest automated_test;
extern volatile uint32_t automated_test_cnt;

extern volatile uint32_t dpt_initial_wait_ticks;
extern volatile uint32_t dpt_first_pulse_ticks;
extern volatile uint16_t dpt_freewheel_us;
extern volatile uint16_t dpt_second_pulse_us;
extern volatile uint16_t passive_boost;
extern volatile uint32_t flyback_test_step_ms;

extern volatile uint32_t switching_ug_frequency_hz;
extern volatile uint32_t switching_ug_step_ms;
extern volatile float switching_ug_pid_duty;
extern volatile uint16_t switching_ug_inf_high;
extern volatile uint16_t switching_ug_openloop_arm;

// Diagnostics from DPT_tests.c.
extern volatile uint16_t dpt_state_monitor;
extern volatile uint16_t dpt_config_error;
extern volatile uint16_t dpt_boost_latched;
extern volatile uint16_t dpt_completed;

extern volatile uint16_t qpt_finished;
extern volatile uint16_t qpt_step;

extern volatile uint16_t sweep_finished;
extern volatile uint16_t sweep_step;
extern volatile int16_t sweep_delay_ns_active;

extern volatile uint16_t flyback_test_step;
extern volatile uint16_t flyback_test_finished;
extern volatile uint16_t flyback_test_error;
extern volatile uint32_t flyback_test_ticks_elapsed;
extern volatile uint32_t flyback_test_step_ms_latched;

// SWITCHING_UG_TEST monitoring.
extern volatile uint16_t switching_ug_test_step;
extern volatile float switching_ug_duty_active;
extern volatile uint16_t switching_ug_pid_active;
extern volatile uint32_t switching_ug_pid_ticks_latched;

extern volatile uint16_t switching_ug_test_finished;
extern volatile uint16_t switching_ug_test_error;
extern volatile uint32_t switching_ug_test_ticks_elapsed;
extern volatile uint32_t switching_ug_frequency_hz_latched;
extern volatile uint32_t switching_ug_step_ms_latched;
extern volatile uint32_t switching_ug_step_ticks_latched;

void DPT_OnEPWM5Tick(void);

#endif

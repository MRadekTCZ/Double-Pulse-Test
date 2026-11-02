
/*
 * Modular refactor of DPT_QPT_SWEEP_FLYBACK_TEST.c
 * F28P65x / C2000 DriverLib.
 * Timing and mode behavior retained from source.
 */
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

// CCS Expressions: editable configuration, defined once in main.c.
extern volatile AutomatedTest automated_test;
extern volatile uint32_t automated_test_cnt;
extern volatile uint32_t dpt_initial_wait_ticks;  // 100 us per tick
extern volatile uint32_t dpt_first_pulse_ticks;   // 100 us per tick
extern volatile uint16_t dpt_freewheel_us;        // single DPT only
extern volatile uint16_t dpt_second_pulse_us;     // single DPT only
extern volatile uint16_t passive_boost;           // single DPT only
extern volatile uint32_t flyback_test_step_ms;    // 100 ms default

// SWITCHING_UG_TEST mode 5 settings, defined once in DPT.c.
extern volatile uint32_t switching_ug_frequency_hz;  // default 20 kHz
extern volatile uint32_t switching_ug_step_ms;       // default 2000 ms
extern volatile uint16_t switching_ug_inf_high;      // constant /INF state (default HIGH)
extern volatile uint16_t switching_ug_openloop_arm;  // safety opt-in (default 0)


// CCS Expressions: diagnostics, defined once in DPT_tests.c.
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

// SWITCHING_UG_TEST diagnostics, defined in DPT_tests.c.
extern volatile uint16_t switching_ug_test_step;       // 0..3
extern volatile uint16_t switching_ug_test_finished;   // 1 = completed
extern volatile uint16_t switching_ug_test_error;      // 0=none,1=not armed,2=bad settings,3=aborted
extern volatile uint32_t switching_ug_test_ticks_elapsed;
extern volatile uint32_t switching_ug_frequency_hz_latched;
extern volatile uint32_t switching_ug_step_ms_latched;
extern volatile uint32_t switching_ug_step_ticks_latched;


// Application callback passed to TI_EPWM_RegisterInterrupt() in main.c.
void DPT_OnEPWM5Tick(void);

#endif // DPT_TESTS_H_

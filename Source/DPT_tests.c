
#include "DPT_tests.h"
#include "Peripherals/TI_EPWM.h"
#include <stdint.h>

/*
 * Test scheduling, mode transitions and diagnostics.
 * No EPWM register or DriverLib calls in this module.
 */

typedef enum
{
    SEQ_IDLE = 0,
    SEQ_SLOW = 1,
    SEQ_ARM_WAIT = 2,
    SEQ_DPT_FAST = 3,
    SEQ_QPT_FAST = 4,
    SEQ_SWEEP_FAST = 5,
    SEQ_FLYBACK_TEST = 6,
    SEQ_SWITCHING_UG = 7
} SequenceState;

volatile uint16_t dpt_state_monitor = SEQ_IDLE;
volatile uint16_t dpt_config_error = 0U;
volatile uint16_t dpt_boost_latched = 0U;
volatile uint16_t dpt_completed = 0U;

volatile uint16_t qpt_finished = 0U;
volatile uint16_t qpt_step = 0U;

volatile uint16_t sweep_finished = 0U;
volatile uint16_t sweep_step = 0U;
volatile int16_t sweep_delay_ns_active = 0;

volatile uint16_t flyback_test_step = 0U;
volatile uint16_t flyback_test_finished = 0U;
volatile uint16_t flyback_test_error = 0U;
volatile uint32_t flyback_test_ticks_elapsed = 0UL;
volatile uint32_t flyback_test_step_ms_latched = 100UL;

// SWITCHING_UG_TEST diagnostics.
volatile uint16_t switching_ug_test_step = 0U;
volatile float switching_ug_duty_active = 0.0f;
volatile uint16_t switching_ug_pid_active = 0U;
volatile uint32_t switching_ug_pid_ticks_latched = 0UL;

volatile uint16_t switching_ug_test_finished = 0U;
volatile uint16_t switching_ug_test_error = 0U;
volatile uint32_t switching_ug_test_ticks_elapsed = 0UL;
volatile uint32_t switching_ug_frequency_hz_latched = 20000UL;
volatile uint32_t switching_ug_step_ms_latched = 500UL;
volatile uint32_t switching_ug_step_ticks_latched = 10000UL;

// =====================================================
// ADJUSTABLE FLYBACK TRANSITION PARAMETERS
// =====================================================

// Minimum duty applied when Ug is reduced.
// 0.00 means PWM OFF during the transition.
// A positive value uses the existing flyback override.
#define SWITCHING_UG_MIN_DUTY        0.01f

// Transition time in milliseconds per volt difference.
// Example: 20 V -> 18 V = 2 * 5 ms = 10 ms.
#define SWITCHING_UG_MS_PER_VOLT     20UL

// Additional settling time before inductor charging.
// Applied after the initial voltage transition.
#define SWITCHING_UG_SETTLE_MS       100UL

// Maximum duty comes from switching_ug_pid_duty
// defined in DPT.c. Default value: 0.16.
static float switching_ug_pid_duty_latched = 0.18f;

static uint16_t switching_ug_inf_high_latched = 1U;
static uint16_t switching_ug_started = 0U;

static uint32_t switching_ug_initial_ticks = 0UL;
static uint32_t switching_ug_initial_elapsed = 0UL;
static float switching_ug_initial_previous_v = 0.0f;

#define SWITCHING_UG_STEPS 4U

static const float switching_ug_voltages[SWITCHING_UG_STEPS] =
{
    20.0f, 18.0f, 15.0f, 12.0f
};

// =====================================================
// SWITCHING UG TRANSITION HELPERS
// =====================================================

static uint32_t switchingUgTransitionMs(
    float previous_v,
    float next_v)
{
    float difference = next_v - previous_v;

    if (difference < 0.0f)
    {
        difference = -difference;
    }

    return (uint32_t)(
        difference * (float)SWITCHING_UG_MS_PER_VOLT
        + 0.5f);
}

static float switchingUgDuty(uint16_t step)
{
    // MOSFET duty:
    // 20 V = 0.52
    // 18 V = 0.54
    // 15 V = 0.56
    // 12 V = 0.58
    return 0.50f + 0.02f * (float)(step + 1U);
}

static uint32_t switchingUgTicksForMs(
    uint32_t frequency_hz,
    uint32_t milliseconds)
{
    return (uint32_t)(
        ((uint64_t)frequency_hz *
         (uint64_t)milliseconds + 500ULL) / 1000ULL);
}

// =====================================================
// APPLY TEMPORARY FLYBACK DUTY
// =====================================================

static void applySwitchingUgTransition(
    float old_v,
    float new_v,
    uint32_t pwm_frequency_hz)
{
    uint32_t transition_ms =
        switchingUgTransitionMs(old_v, new_v);

    switching_ug_pid_ticks_latched =
        switchingUgTicksForMs(
            pwm_frequency_hz,
            transition_ms);

    // Set the new target voltage.
    flyback_target_vdd = new_v;
    TI_EPWM_SetFlybackVdd(new_v);

    // Clear any previous override.
    TI_EPWM_SetFlybackPidOverride(0U, 0.0f);
    flyback_pwm_enable = 1U;
    switching_ug_pid_active = 0U;

    // No transition required.
    if (switching_ug_pid_ticks_latched == 0UL ||
        old_v == new_v)
    {
        return;
    }

    if (new_v > old_v)
    {
        // Voltage increasing:
        // Apply maximum flyback duty.
        TI_EPWM_SetFlybackPidOverride(
            1U,
            switching_ug_pid_duty_latched);
    }
    else
    {
        // Voltage decreasing:
        // Apply minimum flyback duty.

        if (SWITCHING_UG_MIN_DUTY == 0.0f)
        {
            // Existing override function does not
            // accept exactly 0.0 duty.
            // Disable the flyback PWM instead.
            flyback_pwm_enable = 0U;
        }
        else
        {
            TI_EPWM_SetFlybackPidOverride(
                1U,
                SWITCHING_UG_MIN_DUTY);
        }
    }

    switching_ug_pid_active = 1U;
}

// =====================================================
// RESTORE CALIBRATED FLYBACK DUTY
// =====================================================

static void endSwitchingUgTransition(void)
{
    TI_EPWM_SetFlybackPidOverride(0U, 0.0f);

    // Reenable flyback PWM after minimum-duty interval.
    flyback_pwm_enable = 1U;

    switching_ug_pid_active = 0U;
}

// =====================================================
// BEGIN EACH UG SWITCHING STAGE
// =====================================================

static void beginSwitchingUgStage(uint16_t step)
{
    switching_ug_test_step = step;
    switching_ug_test_ticks_elapsed = 0UL;

    switching_ug_duty_active = switchingUgDuty(step);

    // Apply MOSFET PWM duty.
    TI_EPWM_SetContinuousDuty(
        switching_ug_duty_active);

    if (step == 0U)
    {
        // Initial transition to 20 V has already
        // been completed before inductor charging.

        endSwitchingUgTransition();

        switching_ug_pid_ticks_latched = 0UL;

        flyback_target_vdd =
            switching_ug_voltages[0U];

        TI_EPWM_SetFlybackVdd(
            flyback_target_vdd);
    }
    else
    {
        // Calculate duration from voltage difference.
        //
        // 20 -> 18 V: 10 ms minimum duty.
        // 18 -> 15 V: 15 ms minimum duty.
        // 15 -> 12 V: 15 ms minimum duty.

        applySwitchingUgTransition(
            switching_ug_voltages[step - 1U],
            switching_ug_voltages[step],
            switching_ug_frequency_hz_latched);
    }
}

// =====================================================
// GENERAL TEST STATE VARIABLES
// =====================================================

static volatile uint16_t flyback_test_safe_hold = 0U;
static volatile SequenceState sequence_state = SEQ_IDLE;

static AutomatedTest running_test = STOP;

static uint32_t next_slow_index = 0UL;
static uint32_t wait_slow = 0UL;
static uint32_t charge_slow = 0UL;

static uint16_t dpt_off_cmp = 0U;
static uint16_t dpt_end_cmp = 0U;
static uint16_t dpt_done_cmp = 0U;

static uint16_t qpt_fast_index = 0U;
static uint16_t sweep_fast_index = 0U;
static uint16_t inf_released = 0U;

#define FLYBACK_TEST_STEPS 6U

static const float flyback_test_voltages[FLYBACK_TEST_STEPS] =
{
    0.0f, 20.0f, 12.0f, 18.0f, 15.0f, 0.0f
};

static uint32_t flyback_test_step_ticks = 2000UL;

static const int16_t sweep_delays_ns[SWEEP_PULSES] =
{
    300, 200, 150, 100, 50, -50
};

static const uint16_t sweep_positive_counts[SWEEP_PULSES - 1U] =
{
    NS_TO_COUNTS(300U),
    NS_TO_COUNTS(200U),
    NS_TO_COUNTS(150U),
    NS_TO_COUNTS(100U),
    NS_TO_COUNTS(50U)
};

// Function declarations.
static void startFlybackTest(void);
static void updateFlybackTest(void);
static void abortFlybackTest(void);

static void prepareSlowPeriod(uint32_t index);
static void startTest(void);
static void armFastSequence(void);
static void prepareNextSweepPeriod(uint16_t next_index);
static void stopTest(void);

static void finishSwitchingUgTest(uint16_t completed);
static void updateSwitchingUgTest(void);

// =====================================================
// FLYBACK_TEST = 4
// =====================================================

static void startFlybackTest(void)
{
    TI_EPWM_SetFlybackPidOverride(0U, 0.0f);
    TI_EPWM_ForceBothLow();

    flyback_test_safe_hold = 1U;
    flyback_test_finished = 0U;
    flyback_test_error = 0U;
    flyback_test_step = 0U;
    flyback_test_ticks_elapsed = 0UL;

    flyback_test_step_ms_latched = flyback_test_step_ms;

    if (flyback_test_step_ms_latched < 1UL ||
        flyback_test_step_ms_latched > 60000UL)
    {
        flyback_test_error = 1U;

        flyback_target_vdd = 0.0f;
        TI_EPWM_SetFlybackVdd(0.0f);

        automated_test = STOP;
        return;
    }

    // Normal 20 kHz ISR = 20 ticks per millisecond.
    flyback_test_step_ticks =
        flyback_test_step_ms_latched *
        (FLYBACK_TEST_ISR_HZ / 1000UL);

    flyback_vdd_control_enable = 1U;
    flyback_pwm_enable = 1U;

    flyback_target_vdd = flyback_test_voltages[0U];
    TI_EPWM_SetFlybackVdd(flyback_target_vdd);

    running_test = FLYBACK_TEST;
    sequence_state = SEQ_FLYBACK_TEST;
}

static void updateFlybackTest(void)
{
    TI_EPWM_ForceBothLow();

    ++flyback_test_ticks_elapsed;

    if (flyback_test_ticks_elapsed < flyback_test_step_ticks)
    {
        return;
    }

    flyback_test_ticks_elapsed = 0UL;

    if (flyback_test_step + 1U < FLYBACK_TEST_STEPS)
    {
        ++flyback_test_step;

        flyback_target_vdd =
            flyback_test_voltages[flyback_test_step];

        TI_EPWM_SetFlybackVdd(flyback_target_vdd);
    }
    else
    {
        flyback_target_vdd = 0.0f;
        TI_EPWM_SetFlybackVdd(0.0f);

        flyback_test_finished = 1U;
        running_test = STOP;
        sequence_state = SEQ_IDLE;
        automated_test = STOP;
    }
}

static void abortFlybackTest(void)
{
    TI_EPWM_ForceBothLow();

    flyback_test_safe_hold = 1U;

    flyback_target_vdd = 0.0f;
    TI_EPWM_SetFlybackVdd(0.0f);

    running_test = STOP;
    sequence_state = SEQ_IDLE;
    automated_test = STOP;
}

// =====================================================
// SWITCHING_UG_TEST COMPLETION
// =====================================================

static void finishSwitchingUgTest(uint16_t completed)
{
    // Stop MOSFET switching first.
    TI_EPWM_ForceIN(0U);

    // Clear temporary duty and restore PWM control.
    endSwitchingUgTransition();

    TI_EPWM_StopOutputs(0U, 0U);
    TI_EPWM_ForceBothLow();

    flyback_test_safe_hold = 1U;

    // Command final 0 V.
    // The flyback output is not actively discharged.
    flyback_target_vdd = 0.0f;
    TI_EPWM_SetFlybackVdd(0.0f);
    TI_EPWM_UpdateFlybackPWM();

    switching_ug_test_finished = completed;

    if (completed == 0U &&
        switching_ug_test_error == 0U)
    {
        switching_ug_test_error = 3U;
    }

    switching_ug_openloop_arm = 0U;
    switching_ug_started = 0U;
    switching_ug_duty_active = 0.0f;

    running_test = STOP;
    automated_test = STOP;
    sequence_state = SEQ_IDLE;
    automated_test_cnt = 0UL;
}

// =====================================================
// SWITCHING_UG_TEST UPDATE
// =====================================================

static void updateSwitchingUgTest(void)
{
    // First ZERO interrupt marks charging completion.
    if (switching_ug_started == 0U)
    {
        switching_ug_started = 1U;

        beginSwitchingUgStage(0U);
        return;
    }

    // Manual abortion via CCS Expressions.
    if (switching_ug_openloop_arm == 0U)
    {
        finishSwitchingUgTest(0U);
        return;
    }

    ++switching_ug_test_ticks_elapsed;

    // Temporary max/min duty interval finished.
    if (switching_ug_pid_active != 0U &&
        switching_ug_test_ticks_elapsed >=
            switching_ug_pid_ticks_latched)
    {
        endSwitchingUgTransition();
    }

    // Continue current voltage stage.
    if (switching_ug_test_ticks_elapsed <
        switching_ug_step_ticks_latched)
    {
        return;
    }

    // Advance to next voltage.
    if (switching_ug_test_step + 1U <
        SWITCHING_UG_STEPS)
    {
        beginSwitchingUgStage(
            (uint16_t)(switching_ug_test_step + 1U));
    }
    else
    {
        // Four stages completed (default 2 seconds).
        finishSwitchingUgTest(1U);
    }
}

// =====================================================
// SLOW WAITING / CHARGING
// =====================================================

static void prepareSlowPeriod(uint32_t index)
{
    uint32_t charge_end = wait_slow + charge_slow;

    uint16_t charging =
        (index >= wait_slow &&
         index < charge_end) ? 1U : 0U;

    TI_EPWM_PrepareSlowPeriod(charging);
}

// =====================================================
// START TEST
// =====================================================

static void startTest(void)
{
    uint64_t count;
    uint64_t done;
    uint16_t i;

    flyback_test_safe_hold = 0U;
    running_test = automated_test;

    dpt_config_error = 0U;
    dpt_completed = 0U;

    qpt_finished = 0U;
    qpt_step = 0U;

    sweep_finished = 0U;
    sweep_step = 0U;
    sweep_delay_ns_active = 0;

    qpt_fast_index = 0U;
    sweep_fast_index = 0U;
    inf_released = 0U;

    automated_test_cnt = 0UL;

    // =================================================
    // SWITCHING UG CONFIGURATION
    // =================================================

    if (running_test == SWITCHING_UG_TEST)
    {
        switching_ug_test_finished = 0U;
        switching_ug_test_error = 0U;
        switching_ug_test_step = 0U;
        switching_ug_duty_active = 0.0f;
        switching_ug_pid_active = 0U;
        switching_ug_test_ticks_elapsed = 0UL;
        switching_ug_started = 0U;

        switching_ug_initial_elapsed = 0UL;
        switching_ug_initial_ticks = 0UL;

        // Previous commanded gate supply.
        // When flyback PWM is disabled, assume 0 V.
        // This value is not an actual ADC measurement.
        switching_ug_initial_previous_v =
            (flyback_pwm_enable == 0U)
                ? 0.0f
                : flyback_target_vdd;

        switching_ug_frequency_hz_latched =
            switching_ug_frequency_hz;

        switching_ug_step_ms_latched =
            switching_ug_step_ms;

        switching_ug_pid_duty_latched =
            switching_ug_pid_duty;

        switching_ug_inf_high_latched =
            switching_ug_inf_high ? 1U : 0U;

        if (switching_ug_openloop_arm != 1U)
        {
            switching_ug_test_error = 1U;
            dpt_config_error = 4U;
        }

        if (switching_ug_step_ms_latched < 1UL ||
            switching_ug_step_ms_latched > 5000UL ||
            SWITCHING_UG_MS_PER_VOLT == 0UL ||
            !(switching_ug_pid_duty_latched > 0.0f &&
              switching_ug_pid_duty_latched < 1.0f) ||
            !(SWITCHING_UG_MIN_DUTY >= 0.0f &&
              SWITCHING_UG_MIN_DUTY <
                  switching_ug_pid_duty_latched) ||
            TI_EPWM_ContinuousPWMFrequencyValid(
                switching_ug_frequency_hz_latched) == 0U ||
            switching_ug_step_ms_latched <
                switchingUgTransitionMs(18.0f, 15.0f))
        {
            switching_ug_test_error = 2U;
            dpt_config_error = 4U;
        }

        if (dpt_config_error == 0U)
        {
            uint64_t ticks =
                ((uint64_t)switching_ug_frequency_hz_latched *
                 switching_ug_step_ms_latched + 500ULL) /
                 1000ULL;

            if (ticks == 0ULL ||
                ticks > 0xFFFFFFFFULL)
            {
                switching_ug_test_error = 2U;
                dpt_config_error = 4U;
            }
            else
            {
                switching_ug_step_ticks_latched =
                    (uint32_t)ticks;
            }
        }
    }

    // Validate the initial waiting and charging intervals.
    if (dpt_initial_wait_ticks > 1000000UL ||
        dpt_first_pulse_ticks == 0UL ||
        dpt_first_pulse_ticks > 1000000UL)
    {
        dpt_config_error = 1U;
    }

    wait_slow = 2UL * dpt_initial_wait_ticks;
    charge_slow = 2UL * dpt_first_pulse_ticks;

    if (running_test == SWITCHING_UG_TEST)
    {
        uint32_t initial_ms =
            switchingUgTransitionMs(
                switching_ug_initial_previous_v,
                20.0f);

        uint32_t minimum_wait =
            switchingUgTicksForMs(
                SLOW_HZ,
                initial_ms + SWITCHING_UG_SETTLE_MS);

        switching_ug_initial_ticks =
            switchingUgTicksForMs(
                SLOW_HZ,
                initial_ms);

        // Allow the voltage transition to finish
        // before charging the inductor.
        //
        // 0 -> 20 V:
        // 100 ms transition + 20 ms settling.
        //
        // 15 -> 20 V:
        // 25 ms transition + 20 ms settling.

        if (wait_slow < minimum_wait)
        {
            wait_slow = minimum_wait;
        }
    }

    if ((uint64_t)wait_slow + charge_slow >
        20000000ULL)
    {
        dpt_config_error = 1U;
    }

    // =================================================
    // SINGLE DPT
    // =================================================

    if (running_test == DPT)
    {
        if (dpt_freewheel_us < 2U ||
            dpt_freewheel_us > 20U ||
            dpt_second_pulse_us < 2U ||
            dpt_second_pulse_us > 20U)
        {
            dpt_config_error = 1U;
        }

        count =
            (uint64_t)COUNT_US *
            (dpt_freewheel_us +
             dpt_second_pulse_us);

        done =
            count + 2ULL * COUNT_US;

        if (done >= SLOW_COUNTS)
        {
            dpt_config_error = 1U;
        }

        dpt_off_cmp = AT_US(dpt_freewheel_us);
        dpt_end_cmp = (uint16_t)count;
        dpt_done_cmp = (uint16_t)done;
    }

    // =================================================
    // QPT / DPT_SWEEP
    // =================================================

    else if (running_test == QPT ||
             running_test == DPT_SWEEP)
    {
        if (wait_slow < 2UL ||
            charge_slow < 4UL)
        {
            dpt_config_error = 1U;
        }

        if (running_test == DPT_SWEEP)
        {
            for (i = 0U;
                 i < SWEEP_PULSES - 1U;
                 ++i)
            {
                if (sweep_positive_counts[i] == 0U ||
                    sweep_positive_counts[i] >=
                        FAST_ON_CMP ||
                    FAST_ON_CMP +
                        sweep_positive_counts[i] >=
                        FAST_TBPRD)
                {
                    dpt_config_error = 1U;
                }
            }

            if (PREEDGE_COUNTS >= FAST_ON_CMP ||
                SWEEP_FINAL_INF_TBPRD <= FAST_ON_CMP ||
                SWEEP_FINAL_INF_TBPRD >= FAST_TBPRD)
            {
                dpt_config_error = 1U;
            }
        }
    }
    else if (running_test == SWITCHING_UG_TEST)
    {
        if (wait_slow < 2UL ||
            charge_slow < 4UL)
        {
            dpt_config_error = 1U;
            switching_ug_test_error = 2U;
        }
    }
    else
    {
        dpt_config_error = 1U;
    }

    // =================================================
    // CONFIGURATION ERROR
    // =================================================

    if (dpt_config_error != 0U)
    {
        TI_EPWM_ForceIN(0U);

        if (running_test == SWITCHING_UG_TEST)
        {
            TI_EPWM_ForceBothLow();
            endSwitchingUgTransition();
            flyback_test_safe_hold = 1U;
            switching_ug_openloop_arm = 0U;
        }

        running_test = STOP;
        automated_test = STOP;
        sequence_state = SEQ_IDLE;

        return;
    }

    // =================================================
    // PREPARE INITIAL TEST STATE
    // =================================================

    dpt_boost_latched =
        passive_boost ? 1U : 0U;

    TI_EPWM_ForceIN(0U);

    if (running_test == DPT)
    {
        TI_EPWM_ForceINF(
            dpt_boost_latched ? 0U : 1U);
    }
    else if (running_test == SWITCHING_UG_TEST)
    {
        TI_EPWM_ForceINF(
            switching_ug_inf_high_latched);

        // Start transition to 20 V.
        flyback_vdd_control_enable = 1U;
        flyback_pwm_enable = 1U;

        applySwitchingUgTransition(
            switching_ug_initial_previous_v,
            switching_ug_voltages[0U],
            SLOW_HZ);

        switching_ug_initial_elapsed = 0UL;
    }
    else
    {
        TI_EPWM_PrepareChargingINF();
    }

    next_slow_index = 0UL;
    sequence_state = SEQ_SLOW;

    prepareSlowPeriod(next_slow_index++);

    TI_EPWM_ReleaseIN();
}

// =====================================================
// ARM FAST SEQUENCE
// =====================================================

static void armFastSequence(void)
{
    if (running_test == SWITCHING_UG_TEST)
    {
        switching_ug_duty_active =
            switchingUgDuty(0U);

        if (TI_EPWM_ArmContinuousDuty(
                switching_ug_frequency_hz_latched,
                switching_ug_duty_active) == 0U)
        {
            switching_ug_test_error = 2U;
            finishSwitchingUgTest(0U);
            return;
        }

        sequence_state = SEQ_SWITCHING_UG;
        switching_ug_started = 0U;
        switching_ug_test_ticks_elapsed = 0UL;
        return;
    }

    // Original DPT, QPT and sweep sequences.
    TI_EPWM_FastSequence fast_mode =
        TI_EPWM_FAST_SWEEP;

    if (running_test == DPT)
    {
        fast_mode = TI_EPWM_FAST_SINGLE;
    }
    else if (running_test == QPT)
    {
        fast_mode = TI_EPWM_FAST_FOUR_PULSE;
    }

    TI_EPWM_ArmFastSequence(
        fast_mode,
        dpt_off_cmp,
        dpt_end_cmp,
        dpt_done_cmp,
        sweep_positive_counts[0]);

    if (running_test == DPT)
    {
        sequence_state = SEQ_DPT_FAST;
    }
    else if (running_test == QPT)
    {
        qpt_fast_index = 0U;
        sequence_state = SEQ_QPT_FAST;
    }
    else
    {
        sweep_delay_ns_active =
            sweep_delays_ns[0];

        sweep_fast_index = 0U;
        sequence_state = SEQ_SWEEP_FAST;
    }
}

static void prepareNextSweepPeriod(uint16_t next_index)
{
    if (next_index < SWEEP_PULSES - 1U)
    {
        TI_EPWM_SweepPreparePositive(
            sweep_positive_counts[next_index - 1U],
            sweep_positive_counts[next_index]);
    }
    else if (next_index == SWEEP_PULSES - 1U)
    {
        TI_EPWM_SweepPrepareNegative(
            sweep_positive_counts[SWEEP_PULSES - 2U]);
    }
}

// =====================================================
// STOP TEST
// =====================================================

static void stopTest(void)
{
    TI_EPWM_StopOutputs(
        running_test == DPT ? 1U : 0U,
        dpt_boost_latched);

    running_test = STOP;
    automated_test = STOP;
    sequence_state = SEQ_IDLE;
    automated_test_cnt = 0UL;
    dpt_state_monitor = SEQ_IDLE;
}

// =====================================================
// EPWM5 CALLBACK
// =====================================================

void DPT_OnEPWM5Tick(void)
{
    if (sequence_state == SEQ_IDLE)
    {
        if (automated_test == FLYBACK_TEST)
        {
            startFlybackTest();
        }
        else if (automated_test == DPT ||
                 automated_test == QPT ||
                 automated_test == DPT_SWEEP ||
                 automated_test == SWITCHING_UG_TEST)
        {
            startTest();
        }
        else
        {
            TI_EPWM_ForceIN(0U);

            if (flyback_test_safe_hold != 0U)
            {
                TI_EPWM_ForceINF(0U);
            }
            else
            {
                TI_EPWM_ForceINF(
                    passive_boost ? 0U : 1U);
            }
        }
    }

    // =================================================
    // MANUAL STOP
    // =================================================

    else if (automated_test == STOP)
    {
        if (sequence_state == SEQ_FLYBACK_TEST)
        {
            abortFlybackTest();
        }
        else if (running_test == SWITCHING_UG_TEST)
        {
            finishSwitchingUgTest(0U);
        }
        else
        {
            stopTest();
        }
    }

    // =================================================
    // FLYBACK TEST
    // =================================================

    else if (sequence_state == SEQ_FLYBACK_TEST)
    {
        updateFlybackTest();
    }

    // =================================================
    // SWITCHING UG TEST
    // =================================================

    else if (sequence_state == SEQ_SWITCHING_UG)
    {
        updateSwitchingUgTest();
    }

    // =================================================
    // INITIAL WAITING AND CHARGING
    // =================================================

    else if (sequence_state == SEQ_SLOW)
    {
        uint32_t current_index =
            next_slow_index - 1UL;

        uint32_t charge_end =
            wait_slow + charge_slow;

        // Complete the initial voltage transition
        // using the normal 20 kHz timebase.
        if (running_test == SWITCHING_UG_TEST &&
            switching_ug_pid_active != 0U)
        {
            ++switching_ug_initial_elapsed;

            if (switching_ug_initial_elapsed >=
                switching_ug_initial_ticks)
            {
                endSwitchingUgTransition();
            }
        }

        // QPT and sweep /INF scheduling.
        if (running_test == QPT ||
            running_test == DPT_SWEEP)
        {
            if (!inf_released &&
                current_index >= 1UL)
            {
                TI_EPWM_ReleaseINF();
                inf_released = 1U;
            }

            if (current_index ==
                wait_slow + charge_slow / 2UL - 1UL)
            {
                TI_EPWM_ChargingMidpointINF();
            }
        }

        // Arm the fast sequence during the
        // last 50 us charging interval.
        if (current_index == charge_end - 1UL)
        {
            TI_EPWM_SelectArmingInterrupt();
            sequence_state = SEQ_ARM_WAIT;
        }
        else if (next_slow_index < charge_end)
        {
            prepareSlowPeriod(next_slow_index);

            automated_test_cnt = next_slow_index;
            ++next_slow_index;
        }
        else
        {
            dpt_config_error = 3U;
            stopTest();
        }
    }

    // =================================================
    // ARM FAST SEQUENCE
    // =================================================

    else if (sequence_state == SEQ_ARM_WAIT)
    {
        armFastSequence();
    }

    // =================================================
    // SINGLE DPT
    // =================================================

    else if (sequence_state == SEQ_DPT_FAST)
    {
        dpt_completed = 1U;
        stopTest();
    }

    // =================================================
    // QPT
    // =================================================

    else if (sequence_state == SEQ_QPT_FAST)
    {
        switch (qpt_fast_index)
        {
            case 0U:
                TI_EPWM_QPTStep(0U);
                qpt_step = 1U;
                break;

            case 1U:
                TI_EPWM_QPTStep(1U);
                qpt_step = 2U;
                break;

            case 2U:
                TI_EPWM_QPTStep(2U);
                qpt_step = 3U;
                break;

            case 3U:
                TI_EPWM_QPTStep(3U);
                qpt_step = 4U;
                break;

            default:
                qpt_finished = 1U;
                stopTest();
                break;
        }

        if (sequence_state == SEQ_QPT_FAST)
        {
            ++qpt_fast_index;
            automated_test_cnt = qpt_fast_index;
        }
    }

    // =================================================
    // DPT SWEEP
    // =================================================

    else if (sequence_state == SEQ_SWEEP_FAST)
    {
        if (sweep_fast_index < SWEEP_PULSES)
        {
            uint16_t next_index =
                (uint16_t)(sweep_fast_index + 1U);

            if (next_index < SWEEP_PULSES)
            {
                prepareNextSweepPeriod(next_index);
            }
            else
            {
                TI_EPWM_SuppressFutureTurnOn();
            }

            sweep_step =
                (uint16_t)(sweep_fast_index + 1U);

            sweep_delay_ns_active =
                sweep_delays_ns[sweep_fast_index];
        }
        else if (sweep_fast_index == SWEEP_PULSES)
        {
            TI_EPWM_ForceINF(1U);
            TI_EPWM_ForceIN(0U);
            sweep_step = SWEEP_PULSES;
        }
        else
        {
            sweep_finished = 1U;
            stopTest();
        }

        if (sequence_state == SEQ_SWEEP_FAST)
        {
            ++sweep_fast_index;
            automated_test_cnt = sweep_fast_index;
        }
    }

    dpt_state_monitor =
        (uint16_t)sequence_state;
}


/*
 * Modular refactor of DPT_QPT_SWEEP_FLYBACK_TEST.c
 * F28P65x / C2000 DriverLib.
 * Timing and mode behavior retained from source.
 */
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

// Switching gate-voltage test monitoring in CCS Expressions.
volatile uint16_t switching_ug_test_step = 0U;
volatile uint16_t switching_ug_test_finished = 0U;
volatile uint16_t switching_ug_test_error = 0U;
volatile uint32_t switching_ug_test_ticks_elapsed = 0UL;
volatile uint32_t switching_ug_frequency_hz_latched = 20000UL;
volatile uint32_t switching_ug_step_ms_latched = 2000UL;
volatile uint32_t switching_ug_step_ticks_latched = 40000UL;

static uint16_t switching_ug_inf_high_latched = 1U;
static uint16_t switching_ug_started = 0U;

#define SWITCHING_UG_STEPS 4U
static const float switching_ug_voltages[SWITCHING_UG_STEPS] =
{
    20.0f, 18.0f, 15.0f, 12.0f
};


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
    NS_TO_COUNTS(300U), NS_TO_COUNTS(200U), NS_TO_COUNTS(150U),
    NS_TO_COUNTS(100U), NS_TO_COUNTS(50U)
};

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
static void startFlybackTest(void)
{
    // This mode is only entered from SEQ_IDLE, with the
    // ordinary slow 20 kHz ePWM5 interrupt enabled.
    // No switch pulses are generated during this test.
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

    // 20 kHz interrupt = 20 ticks per millisecond.
    flyback_test_step_ticks =
        flyback_test_step_ms_latched *
        (FLYBACK_TEST_ISR_HZ / 1000UL);

    // Make sure manual duty setting does not override the test.
    flyback_vdd_control_enable = 1U;
    flyback_pwm_enable = 1U;

    flyback_target_vdd = flyback_test_voltages[0U];
    TI_EPWM_SetFlybackVdd(flyback_target_vdd);
    running_test = FLYBACK_TEST;
    sequence_state = SEQ_FLYBACK_TEST;
}


static void updateFlybackTest(void)
{
    // Keep both inputs forced LOW, irrespective of /INF boost.
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
        // Final 0 V stage also lasts the full requested period.
        // Remain at zero voltage and leave IN and /INF LOW.
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
// GATE-VOLTAGE SWEEP UNDER CONTINUOUS SWITCHING
// =====================================================

static void finishSwitchingUgTest(uint16_t completed)
{
    // Hardware OFF first. These operations do not depend on flyback settling.
    TI_EPWM_ForceIN(0U);
    TI_EPWM_StopOutputs(0U, 0U);
    TI_EPWM_ForceBothLow();
    flyback_test_safe_hold = 1U;

    // Command zero gate supply. This does not actively discharge capacitors.
    flyback_target_vdd = 0.0f;
    TI_EPWM_SetFlybackVdd(0.0f);
    TI_EPWM_UpdateFlybackPWM();

    switching_ug_test_finished = completed;
    if (completed == 0U && switching_ug_test_error == 0U)
    {
        switching_ug_test_error = 3U; // aborted
    }

    // Re-arming explicitly is required for each subsequent test.
    switching_ug_openloop_arm = 0U;
    switching_ug_started = 0U;
    running_test = STOP;
    automated_test = STOP;
    sequence_state = SEQ_IDLE;
    automated_test_cnt = 0UL;
}

static void updateSwitchingUgTest(void)
{
    // The first ZERO interrupt is exactly the end of the charging pulse.
    // Do not count it as a complete period.
    if (switching_ug_started == 0U)
    {
        switching_ug_started = 1U;
        switching_ug_test_ticks_elapsed = 0UL;
        return;
    }

    // Permit a live shutdown by clearing the opt-in in CCS Expressions.
    if (switching_ug_openloop_arm == 0U)
    {
        finishSwitchingUgTest(0U);
        return;
    }

    ++switching_ug_test_ticks_elapsed;
    if (switching_ug_test_ticks_elapsed < switching_ug_step_ticks_latched)
    {
        return;
    }

    switching_ug_test_ticks_elapsed = 0UL;

    if (switching_ug_test_step + 1U < SWITCHING_UG_STEPS)
    {
        ++switching_ug_test_step;
        flyback_target_vdd = switching_ug_voltages[switching_ug_test_step];
        TI_EPWM_SetFlybackVdd(flyback_target_vdd);
        // main loop applies the updated ePWM8 duty cycle.
    }
    else
    {
        // Four full switching stages completed (default: 8 seconds).
        finishSwitchingUgTest(1U);
    }
}

static void prepareSlowPeriod(uint32_t index)
{
    uint32_t charge_end = wait_slow + charge_slow;
    uint16_t charging =
        (index >= wait_slow && index < charge_end) ? 1U : 0U;
    TI_EPWM_PrepareSlowPeriod(charging);
}


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

    if (running_test == SWITCHING_UG_TEST)
    {
        switching_ug_test_finished = 0U;
        switching_ug_test_error = 0U;
        switching_ug_test_step = 0U;
        switching_ug_test_ticks_elapsed = 0UL;
        switching_ug_started = 0U;
        switching_ug_frequency_hz_latched = switching_ug_frequency_hz;
        switching_ug_step_ms_latched = switching_ug_step_ms;
        switching_ug_inf_high_latched = switching_ug_inf_high ? 1U : 0U;

        // Do NOT allow an 8-second open-loop power run by accidental mode selection.
        if (switching_ug_openloop_arm != 1U)
        {
            switching_ug_test_error = 1U; // not explicitly armed
            dpt_config_error = 4U;
        }
        if (switching_ug_step_ms_latched < 1UL ||
            switching_ug_step_ms_latched > 5000UL ||
            TI_EPWM_ContinuousPWMFrequencyValid(
                switching_ug_frequency_hz_latched) == 0U)
        {
            switching_ug_test_error = 2U; // invalid duration/frequency
            dpt_config_error = 4U;
        }

        if (dpt_config_error == 0U)
        {
            uint64_t ticks =
                ((uint64_t)switching_ug_frequency_hz_latched *
                 switching_ug_step_ms_latched + 500ULL) / 1000ULL;

            if (ticks == 0ULL || ticks > 0xFFFFFFFFULL)
            {
                switching_ug_test_error = 2U;
                dpt_config_error = 4U;
            }
            else
            {
                switching_ug_step_ticks_latched = (uint32_t)ticks;
            }
        }
    }

    // Validate long intervals.

    if (dpt_initial_wait_ticks > 1000000UL ||
        dpt_first_pulse_ticks == 0UL ||
        dpt_first_pulse_ticks > 1000000UL)
    {
        dpt_config_error = 1U;
    }

    wait_slow =
        2UL * dpt_initial_wait_ticks;

    charge_slow =
        2UL * dpt_first_pulse_ticks;

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

        dpt_off_cmp =
            AT_US(dpt_freewheel_us);

        dpt_end_cmp =
            (uint16_t)count;

        dpt_done_cmp =
            (uint16_t)done;
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
            // Check positive delays.

            for (i = 0U;
                 i < SWEEP_PULSES - 1U;
                 ++i)
            {
                if (sweep_positive_counts[i] == 0U ||
                    sweep_positive_counts[i] >= FAST_ON_CMP ||
                    FAST_ON_CMP +
                    sweep_positive_counts[i] >= FAST_TBPRD)
                {
                    dpt_config_error = 1U;
                }
            }

            // Check final -50ns timing.

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
        // Use exactly the same waiting and charging scheduler as DPT/QPT.
        if (wait_slow < 2UL || charge_slow < 4UL)
        {
            dpt_config_error = 1U;
            switching_ug_test_error = 2U;
        }
    }
    else
    {
        dpt_config_error = 1U;
    }

    if (dpt_config_error != 0U)
    {
        TI_EPWM_ForceIN(0U);
        if (running_test == SWITCHING_UG_TEST)
        {
            TI_EPWM_ForceBothLow();
            flyback_test_safe_hold = 1U;
            switching_ug_openloop_arm = 0U;
        }

        running_test = STOP;
        automated_test = STOP;
        sequence_state = SEQ_IDLE;

        return;
    }

    // DPT boost selection.

    dpt_boost_latched =
        passive_boost ? 1U : 0U;

    TI_EPWM_ForceIN(0U);

    if (running_test == DPT)
    {
        TI_EPWM_ForceINF(dpt_boost_latched ? 0U : 1U);
    }
    else if (running_test == SWITCHING_UG_TEST)
    {
        // Same selected /INF level for charging and all four voltage steps.
        TI_EPWM_ForceINF(switching_ug_inf_high_latched);

        // Set 20 V BEFORE the existing initial wait and inductor charge.
        flyback_vdd_control_enable = 1U;
        flyback_pwm_enable = 1U;
        flyback_target_vdd = switching_ug_voltages[0U];
        TI_EPWM_SetFlybackVdd(flyback_target_vdd);
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


static void armFastSequence(void)
{
    if (running_test == SWITCHING_UG_TEST)
    {
        if (TI_EPWM_ArmContinuousHalfDuty(
                switching_ug_frequency_hz_latched) == 0U)
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

    // Application test mode -> hardware pulse pattern.
    // TI_EPWM does not import or depend on the AutomatedTest enum.
    TI_EPWM_FastSequence fast_mode = TI_EPWM_FAST_SWEEP;
    if (running_test == DPT)
    {
        fast_mode = TI_EPWM_FAST_SINGLE;
    }
    else if (running_test == QPT)
    {
        fast_mode = TI_EPWM_FAST_FOUR_PULSE;
    }

    TI_EPWM_ArmFastSequence(fast_mode, dpt_off_cmp, dpt_end_cmp,
                            dpt_done_cmp, sweep_positive_counts[0]);
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
        sweep_delay_ns_active = sweep_delays_ns[0];
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


static void stopTest(void)
{
    TI_EPWM_StopOutputs(running_test == DPT ? 1U : 0U,
                        dpt_boost_latched);
    running_test = STOP;
    automated_test = STOP;
    sequence_state = SEQ_IDLE;
    automated_test_cnt = 0UL;
    dpt_state_monitor = SEQ_IDLE;
}


void DPT_OnEPWM5Tick(void)
{
    // =================================================
    // IDLE
    // =================================================

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
                TI_EPWM_ForceINF(passive_boost ? 0U : 1U);
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
    // FLYBACK TEST: hardware ePWM5 20 kHz timebase
    // =================================================

    else if (sequence_state == SEQ_FLYBACK_TEST)
    {
        updateFlybackTest();
    }

    // =====================================================
    // CONTINUOUS SWITCHING WITH FOUR GATE VOLTAGES
    // =====================================================

    else if (sequence_state == SEQ_SWITCHING_UG)
    {
        updateSwitchingUgTest();
    }

    // =================================================
    // SLOW WAITING / CHARGING
    // =================================================

    else if (sequence_state == SEQ_SLOW)
    {
        uint32_t current_index =
            next_slow_index - 1UL;

        uint32_t charge_end =
            wait_slow + charge_slow;

        // QPT / SWEEP use variable /INF.

        if (running_test == QPT ||
            running_test == DPT_SWEEP)
        {
            // Release software forcing during
            // the initial waiting interval.

            if (!inf_released &&
                current_index >= 1UL)
            {
                TI_EPWM_ReleaseINF();

                inf_released = 1U;
            }

            // Halfway through charging:
            // change /INF to LOW.
            //
            // The AQ event is prepared
            // one slow period in advance.

            if (current_index ==
                wait_slow + charge_slow / 2UL - 1UL)
            {
                TI_EPWM_ChargingMidpointINF();
            }
        }

        // Final 50us charging period:
        // arm the fast sequence after 25us.

        if (current_index == charge_end - 1UL)
        {
            TI_EPWM_SelectArmingInterrupt();

            sequence_state = SEQ_ARM_WAIT;
        }

        else if (next_slow_index < charge_end)
        {
            prepareSlowPeriod(next_slow_index);

            automated_test_cnt =
                next_slow_index;

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
    // SINGLE DPT COMPLETION
    // =================================================

    else if (sequence_state == SEQ_DPT_FAST)
    {
        dpt_completed = 1U;

        stopTest();
    }

    // =================================================
    // QPT: UNCHANGED FOUR-PULSE OPERATION
    // =================================================

    else if (sequence_state == SEQ_QPT_FAST)
    {
        switch (qpt_fast_index)
        {
            case 0U:
            {
                TI_EPWM_QPTStep(0U);

                qpt_step = 1U;

                break;
            }

            case 1U:
            {
                TI_EPWM_QPTStep(1U);

                qpt_step = 2U;

                break;
            }

            case 2U:
            {
                TI_EPWM_QPTStep(2U);

                qpt_step = 3U;

                break;
            }

            case 3U:
            {
                // Disable further turn-on events.

                TI_EPWM_QPTStep(3U);

                qpt_step = 4U;

                break;
            }

            default:
            {
                qpt_finished = 1U;

                stopTest();

                break;
            }
        }

        if (sequence_state == SEQ_QPT_FAST)
        {
            ++qpt_fast_index;

            automated_test_cnt =
                qpt_fast_index;
        }
    }

    // =================================================
    // DPT_SWEEP: SIX DELAYS
    // =================================================

    else if (sequence_state == SEQ_SWEEP_FAST)
    {
        /*
         * ePWM5 ZERO interrupts:
         *
         * 0, 8, 16, 24, 32, 40, 48, 56us
         *
         * The first five pulses are handled
         * using CMPA/CMPB with positive delays.
         *
         * The final negative-delay interval
         * is prepared at the t=32us interrupt.
         */

        if (sweep_fast_index < SWEEP_PULSES)
        {
            uint16_t next_index =
                (uint16_t)(sweep_fast_index + 1U);

            if (next_index < SWEEP_PULSES)
            {
                // Prepare the next interval.

                prepareNextSweepPeriod(
                    next_index);
            }
            else
            {
                /*
                 * t=40us:
                 *
                 * Sixth pulse already prepared:
                 *
                 * /INF HIGH at 40.05us
                 * /INF LOW  at 43.95us
                 * IN HIGH   at 44.00us
                 * /INF HIGH at 47.95us
                 * IN LOW    at 48.00us
                 *
                 * Disable the next potential
                 * IN turn-on at t=52us.
                 */

                TI_EPWM_SuppressFutureTurnOn();
            }

            sweep_step =
                (uint16_t)(sweep_fast_index + 1U);

            sweep_delay_ns_active =
                sweep_delays_ns[sweep_fast_index];
        }

        else if (sweep_fast_index == SWEEP_PULSES)
        {
            /*
             * t=48us:
             *
             * Sixth pulse has finished.
             *
             * /INF HIGH already occurred at
             * t=47.95us in hardware.
             *
             * Force outputs to safe levels,
             * preventing further changes.
             */

            TI_EPWM_ForceINF(1U);
            TI_EPWM_ForceIN(0U);

            sweep_step = SWEEP_PULSES;
        }

        else
        {
            /*
             * t=56us:
             *
             * Final cleanup.
             */

            sweep_finished = 1U;

            stopTest();
        }

        if (sequence_state == SEQ_SWEEP_FAST)
        {
            ++sweep_fast_index;

            automated_test_cnt =
                sweep_fast_index;
        }
    }

    dpt_state_monitor =
        (uint16_t)sequence_state;

}

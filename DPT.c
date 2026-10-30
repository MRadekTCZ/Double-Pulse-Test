
#include "driverlib.h"
#include "device.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * F28P65x DPT / QPT / DPT_SWEEP / flyback
 *
 * GPIO8  = ePWM5A = IN
 * GPIO10 = ePWM6A = /INF
 * GPIO14 = ePWM8A = flyback
 *
 * DPT = 1:
 *   Original constant /INF operation.
 *
 * QPT = 3:
 *   Four short test pulses.
 *   Original QPT /INF combinations.
 *
 * DPT_SWEEP = 2:
 *   Six short test pulses.
 *   4us OFF + 4us ON.
 *
 * /INF delays:
 *
 *   Pulse 1: +300 ns
 *   Pulse 2: +200 ns
 *   Pulse 3: +150 ns
 *   Pulse 4: +100 ns
 *   Pulse 5:  +50 ns
 *   Pulse 6:  -50 ns
 *
 * Positive delay:
 *   /INF LOW after IN rises.
 *   /INF HIGH after IN falls.
 *
 * Negative delay:
 *   /INF LOW before IN rises.
 *   /INF HIGH before IN falls.
 *
 * During the last pulse, both switching
 * transitions are therefore boosted.
 *
 * Actual ePWM TBCLK must equal
 * DEVICE_SYSCLK_FREQ.
 */

// =====================================================
// HARDWARE
// =====================================================

#define EPWM_IN           EPWM5_BASE
#define EPWM_INF          EPWM6_BASE
#define EPWM_FLYBACK      EPWM8_BASE

#define SLOW_HZ           20000UL

#define SLOW_COUNTS \
    (DEVICE_SYSCLK_FREQ / SLOW_HZ)

#define SLOW_TBPRD \
    ((uint16_t)(SLOW_COUNTS - 1UL))

#define COUNT_US \
    (DEVICE_SYSCLK_FREQ / 1000000UL)

#define AT_US(us) \
    ((uint16_t)((uint32_t)(us) * COUNT_US))

// Current semiconductor: 4us OFF, 4us ON.

#define SHORT_HALF_US     4UL
#define SHORT_PERIOD_US   (2UL * SHORT_HALF_US)

#define FAST_TBPRD \
    (AT_US(SHORT_PERIOD_US) - 1U)

#define FAST_ON_CMP \
    AT_US(SHORT_HALF_US)

// Nanosecond delay to integer TBCLK counts.

#define NS_TO_COUNTS(ns) \
    ((uint16_t)( \
        (((uint64_t)(ns) * DEVICE_SYSCLK_FREQ) + \
        500000000ULL) / 1000000000ULL))

#define PREEDGE_NS 50U

#define PREEDGE_COUNTS \
    ((COUNT_US * PREEDGE_NS + 500UL) / 1000UL)

#define SWEEP_PULSES 6U

#define FLYBACK_MIN_HZ 4000UL
#define FLYBACK_MAX_HZ 100000UL

/*
 * Final sweep interval: t=40..48us.
 *
 * /INF must generate:
 *
 * t=40.05us : HIGH
 * t=43.95us : LOW
 * t=47.95us : HIGH
 *
 * The last transition uses ePWM6 PERIOD.
 *
 * ePWM6 period is shortened by 50ns,
 * while ePWM5 maintains its 8us period.
 */

#define SWEEP_FINAL_INF_TBPRD \
    ((uint16_t)( \
        FAST_TBPRD + 1U - PREEDGE_COUNTS))

#if ((DEVICE_SYSCLK_FREQ % 1000000UL) != 0UL)
#error "SYSCLK must be multiple of 1 MHz"
#endif

#if ((SLOW_COUNTS < 2UL) || \
     (SLOW_COUNTS > 65535UL))
#error "Slow ePWM period out of range"
#endif

#if ((SHORT_PERIOD_US * COUNT_US) > 65535UL)
#error "Fast ePWM period out of range"
#endif

#if ((PREEDGE_COUNTS == 0UL) || \
     (PREEDGE_COUNTS >= SHORT_HALF_US * COUNT_US))
#error "50ns pre-edge delay cannot be generated"
#endif

// =====================================================
// TEST MODES
// =====================================================

typedef enum
{
    STOP = 0,
    DPT = 1,
    DPT_SWEEP = 2,
    QPT = 3

} AutomatedTest;

typedef enum
{
    SEQ_IDLE = 0,
    SEQ_SLOW = 1,
    SEQ_ARM_WAIT = 2,
    SEQ_DPT_FAST = 3,
    SEQ_QPT_FAST = 4,
    SEQ_SWEEP_FAST = 5

} SequenceState;

// =====================================================
// CCS USER PARAMETERS
// =====================================================

volatile AutomatedTest automated_test = STOP;

volatile uint32_t automated_test_cnt = 0UL;

// Initial wait: 100us per tick.

volatile uint32_t dpt_initial_wait_ticks = 200UL;

// Charging: 100us per tick.

volatile uint32_t dpt_first_pulse_ticks = 25UL;

// Single DPT short intervals.

volatile uint16_t dpt_freewheel_us = 2U;
volatile uint16_t dpt_second_pulse_us = 2U;

// Single DPT only:
// 0 = /INF HIGH
// 1 = /INF LOW

volatile uint16_t passive_boost = 0U;

// =====================================================
// FLYBACK USER PARAMETERS
// =====================================================

volatile uint32_t flyback_pwm_frequency_hz = 12000UL;

volatile uint16_t flyback_pwm_enable = 1U;

volatile uint16_t flyback_vdd_control_enable = 1U;

volatile float flyback_target_vdd = 15.0f;

volatile float flyback_pwm_duty = 0.09483871f;

// =====================================================
// CCS MONITORING
// =====================================================

volatile uint32_t epwm5_isr_count = 0UL;

volatile uint16_t dpt_state_monitor = SEQ_IDLE;

volatile uint16_t dpt_config_error = 0U;

volatile uint16_t dpt_boost_latched = 0U;

volatile uint16_t dpt_completed = 0U;

volatile uint16_t qpt_finished = 0U;

volatile uint16_t qpt_step = 0U;

volatile uint16_t sweep_finished = 0U;

volatile uint16_t sweep_step = 0U;

// Signed: sixth delay is negative.

volatile int16_t sweep_delay_ns_active = 0;

volatile float flyback_vdd_applied = 15.0f;

volatile float flyback_pwm_actual_hz = 0.0f;

volatile float flyback_pwm_actual_duty = 0.0f;

volatile uint16_t flyback_pwm_config_error = 0U;

// =====================================================
// INTERNAL VARIABLES
// =====================================================

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

// =====================================================
// SWEEP DELAYS
// =====================================================

static const int16_t sweep_delays_ns[SWEEP_PULSES] =
{
    300,
    200,
    150,
    100,
     50,
    -50
};

// Positive delays only.
// Last pulse uses PREEDGE_COUNTS instead.

static const uint16_t
sweep_positive_counts[SWEEP_PULSES - 1U] =
{
    NS_TO_COUNTS(300U),
    NS_TO_COUNTS(200U),
    NS_TO_COUNTS(150U),
    NS_TO_COUNTS(100U),
    NS_TO_COUNTS(50U)
};

// =====================================================
// FLYBACK INTERNAL VARIABLES
// =====================================================

static uint32_t last_flyback_frequency = 0UL;

static float last_flyback_duty = -1.0f;

static uint16_t last_flyback_enable = 0xFFFFU;

// =====================================================
// FUNCTION DECLARATIONS
// =====================================================

__interrupt void epwm5_isr(void);

static void initPWMs(void);
static void initDPTPWM(void);
static void initFlybackPWM(void);

static void flybackSetVdd(float vdd);
static void updateFlybackPWM(void);

static void forceIN(uint16_t high);
static void forceINF(uint16_t high);

static void releaseIN(void);
static void releaseINF(void);

static void setINFCompareActions(
    uint16_t a,
    uint16_t b);

static void prepareSlowPeriod(uint32_t index);

static void startTest(void);
static void armFastSequence(void);
static void stopTest(void);

static void prepareNextSweepPeriod(
    uint16_t next_index);

// =====================================================
// MAIN
// =====================================================

void main(void)
{
    Device_init();
    Device_initGPIO();

    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_setPinConfig(GPIO_10_GPIO10);
    GPIO_setPinConfig(GPIO_14_GPIO14);

    GPIO_writePin(8U, 0U);
    GPIO_writePin(10U, 1U);
    GPIO_writePin(14U, 0U);

    GPIO_setPadConfig(8U, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(10U, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(14U, GPIO_PIN_TYPE_STD);

    GPIO_setDirectionMode(8U, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(10U, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(14U, GPIO_DIR_MODE_OUT);

    flybackSetVdd(flyback_target_vdd);

    initPWMs();

    Interrupt_initModule();
    Interrupt_initVectorTable();

    Interrupt_register(
        INT_EPWM5,
        &epwm5_isr);

    Interrupt_enable(INT_EPWM5);

    updateFlybackPWM();

    EINT;
    ERTM;

    for (;;)
    {
        if (flyback_vdd_control_enable != 0U)
        {
            flybackSetVdd(flyback_target_vdd);
        }

        updateFlybackPWM();
    }
}

// =====================================================
// PWM INITIALIZATION
// =====================================================

static void initPWMs(void)
{
    SysCtl_disablePeripheral(
        SYSCTL_PERIPH_CLK_TBCLKSYNC);

    SysCtl_enablePeripheral(
        SYSCTL_PERIPH_CLK_EPWM5);

    SysCtl_enablePeripheral(
        SYSCTL_PERIPH_CLK_EPWM6);

    SysCtl_enablePeripheral(
        SYSCTL_PERIPH_CLK_EPWM8);

    SysCtl_setEPWMClockDivider(
        SYSCTL_EPWMCLK_DIV_1);

    initDPTPWM();
    initFlybackPWM();

    GPIO_setPinConfig(GPIO_8_EPWM5_A);
    GPIO_setPinConfig(GPIO_10_EPWM6_A);
    GPIO_setPinConfig(GPIO_14_EPWM8_A);

    SysCtl_enablePeripheral(
        SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

// =====================================================
// DPT / QPT / SWEEP PWM INITIALIZATION
// =====================================================

static void initDPTPWM(void)
{
    EPWM_setTimeBaseCounterMode(
        EPWM_IN,
        EPWM_COUNTER_MODE_STOP_FREEZE);

    EPWM_setTimeBaseCounterMode(
        EPWM_INF,
        EPWM_COUNTER_MODE_STOP_FREEZE);

    EPWM_setClockPrescaler(
        EPWM_IN,
        EPWM_CLOCK_DIVIDER_1,
        EPWM_HSCLOCK_DIVIDER_1);

    EPWM_setClockPrescaler(
        EPWM_INF,
        EPWM_CLOCK_DIVIDER_1,
        EPWM_HSCLOCK_DIVIDER_1);

    // Direct period loading during initialization.

    EPWM_setPeriodLoadMode(
        EPWM_IN,
        EPWM_PERIOD_DIRECT_LOAD);

    EPWM_setPeriodLoadMode(
        EPWM_INF,
        EPWM_PERIOD_DIRECT_LOAD);

    EPWM_setTimeBasePeriod(
        EPWM_IN,
        SLOW_TBPRD);

    EPWM_setTimeBasePeriod(
        EPWM_INF,
        SLOW_TBPRD);

    EPWM_setTimeBaseCounter(EPWM_IN, 0U);
    EPWM_setTimeBaseCounter(EPWM_INF, 0U);

    // Shadow period loading at ZERO.

    EPWM_setPeriodLoadMode(
        EPWM_IN,
        EPWM_PERIOD_SHADOW_LOAD);

    EPWM_setPeriodLoadMode(
        EPWM_INF,
        EPWM_PERIOD_SHADOW_LOAD);

    EPWM_selectPeriodLoadEvent(
        EPWM_IN,
        EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);

    EPWM_selectPeriodLoadEvent(
        EPWM_INF,
        EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);

    EPWM_setTimeBasePeriod(
        EPWM_IN,
        SLOW_TBPRD);

    EPWM_setTimeBasePeriod(
        EPWM_INF,
        SLOW_TBPRD);

    // ePWM5 master; ePWM6 synchronized slave.

    EPWM_disablePhaseShiftLoad(EPWM_IN);

    EPWM_setSyncInPulseSource(
        EPWM_IN,
        EPWM_SYNC_IN_PULSE_SRC_DISABLE);

    EPWM_enableSyncOutPulseSource(
        EPWM_IN,
        EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

    EPWM_setSyncInPulseSource(
        EPWM_INF,
        EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM5);

    EPWM_setPhaseShift(EPWM_INF, 0U);

    EPWM_setCountModeAfterSync(
        EPWM_INF,
        EPWM_COUNT_MODE_UP_AFTER_SYNC);

    EPWM_enablePhaseShiftLoad(EPWM_INF);

    // AQ shadow loading.

    EPWM_setActionQualifierShadowLoadMode(
        EPWM_IN,
        EPWM_ACTION_QUALIFIER_A,
        EPWM_AQ_LOAD_ON_CNTR_PERIOD);

    EPWM_setActionQualifierShadowLoadMode(
        EPWM_INF,
        EPWM_ACTION_QUALIFIER_A,
        EPWM_AQ_LOAD_ON_CNTR_PERIOD);

    // Compare shadow loading.

    EPWM_setCounterCompareShadowLoadMode(
        EPWM_IN,
        EPWM_COUNTER_COMPARE_A,
        EPWM_COMP_LOAD_ON_CNTR_PERIOD);

    EPWM_setCounterCompareShadowLoadMode(
        EPWM_IN,
        EPWM_COUNTER_COMPARE_B,
        EPWM_COMP_LOAD_ON_CNTR_PERIOD);

    EPWM_setCounterCompareShadowLoadMode(
        EPWM_IN,
        EPWM_COUNTER_COMPARE_C,
        EPWM_COMP_LOAD_ON_CNTR_PERIOD);

    EPWM_setCounterCompareShadowLoadMode(
        EPWM_INF,
        EPWM_COUNTER_COMPARE_A,
        EPWM_COMP_LOAD_ON_CNTR_PERIOD);

    EPWM_setCounterCompareShadowLoadMode(
        EPWM_INF,
        EPWM_COUNTER_COMPARE_B,
        EPWM_COMP_LOAD_ON_CNTR_PERIOD);

    // Default IN output LOW.

    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_LOW,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

    // Default /INF output HIGH.

    EPWM_setActionQualifierAction(
        EPWM_INF,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    setINFCompareActions(
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_NO_CHANGE);

    EPWM_setActionQualifierAction(
        EPWM_INF,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

    // Initial compare values.

    EPWM_setCounterCompareValue(
        EPWM_IN,
        EPWM_COUNTER_COMPARE_A,
        FAST_ON_CMP);

    EPWM_setCounterCompareValue(
        EPWM_IN,
        EPWM_COUNTER_COMPARE_B,
        AT_US(25U));

    EPWM_setCounterCompareValue(
        EPWM_IN,
        EPWM_COUNTER_COMPARE_C,
        AT_US(6U));

    EPWM_setCounterCompareValue(
        EPWM_INF,
        EPWM_COUNTER_COMPARE_A,
        AT_US(2U));

    EPWM_setCounterCompareValue(
        EPWM_INF,
        EPWM_COUNTER_COMPARE_B,
        AT_US(6U));

    // Normal 20kHz interrupt at ZERO.

    EPWM_setInterruptSource(
        EPWM_IN,
        EPWM_INT_TBCTR_ZERO);

    EPWM_setInterruptEventCount(
        EPWM_IN,
        1U);

    EPWM_clearEventTriggerInterruptFlag(
        EPWM_IN);

    EPWM_enableInterrupt(EPWM_IN);

    // Immediate software forcing.

    EPWM_setActionQualifierContSWForceShadowMode(
        EPWM_IN,
        EPWM_AQ_SW_IMMEDIATE_LOAD);

    EPWM_setActionQualifierContSWForceShadowMode(
        EPWM_INF,
        EPWM_AQ_SW_IMMEDIATE_LOAD);

    forceIN(0U);
    forceINF(1U);

    EPWM_setTimeBaseCounterMode(
        EPWM_IN,
        EPWM_COUNTER_MODE_UP);

    EPWM_setTimeBaseCounterMode(
        EPWM_INF,
        EPWM_COUNTER_MODE_UP);
}

// =====================================================
// SOFTWARE FORCE
// =====================================================

static void forceIN(uint16_t high)
{
    EPWM_setActionQualifierContSWForceAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        high
            ? EPWM_AQ_SW_OUTPUT_HIGH
            : EPWM_AQ_SW_OUTPUT_LOW);
}

static void forceINF(uint16_t high)
{
    EPWM_setActionQualifierContSWForceAction(
        EPWM_INF,
        EPWM_AQ_OUTPUT_A,
        high
            ? EPWM_AQ_SW_OUTPUT_HIGH
            : EPWM_AQ_SW_OUTPUT_LOW);
}

static void releaseIN(void)
{
    EPWM_setActionQualifierContSWForceAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_SW_DISABLED);
}

static void releaseINF(void)
{
    EPWM_setActionQualifierContSWForceAction(
        EPWM_INF,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_SW_DISABLED);
}

// =====================================================
// /INF COMPARE ACTIONS
// =====================================================

static void setINFCompareActions(
    uint16_t a,
    uint16_t b)
{
    EPWM_setActionQualifierAction(
        EPWM_INF,
        EPWM_AQ_OUTPUT_A,
        a,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setActionQualifierAction(
        EPWM_INF,
        EPWM_AQ_OUTPUT_A,
        b,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
}

// =====================================================
// SLOW SCHEDULER
// =====================================================

static void prepareSlowPeriod(uint32_t index)
{
    uint32_t charge_end =
        wait_slow + charge_slow;

    bool charging =
        (index >= wait_slow &&
         index < charge_end);

    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        charging
            ? EPWM_AQ_OUTPUT_HIGH
            : EPWM_AQ_OUTPUT_LOW,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
}

// =====================================================
// START TEST
// =====================================================

static void startTest(void)
{
    uint64_t count;
    uint64_t done;
    uint16_t i;

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
    else
    {
        dpt_config_error = 1U;
    }

    if (dpt_config_error != 0U)
    {
        forceIN(0U);

        running_test = STOP;
        automated_test = STOP;
        sequence_state = SEQ_IDLE;

        return;
    }

    // DPT boost selection.

    dpt_boost_latched =
        passive_boost ? 1U : 0U;

    forceIN(0U);

    if (running_test == DPT)
    {
        forceINF(
            dpt_boost_latched ? 0U : 1U);
    }
    else
    {
        // QPT / SWEEP charging starts with
        // /INF HIGH.

        forceINF(1U);

        EPWM_setActionQualifierAction(
            EPWM_INF,
            EPWM_AQ_OUTPUT_A,
            EPWM_AQ_OUTPUT_HIGH,
            EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        setINFCompareActions(
            EPWM_AQ_OUTPUT_NO_CHANGE,
            EPWM_AQ_OUTPUT_NO_CHANGE);
    }

    next_slow_index = 0UL;

    sequence_state = SEQ_SLOW;

    prepareSlowPeriod(next_slow_index++);

    releaseIN();
}

// =====================================================
// ARM FAST SEQUENCE
//
// Called 25us before charging finishes.
// =====================================================

static void armFastSequence(void)
{
    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_LOW,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setCounterCompareValue(
        EPWM_IN,
        EPWM_COUNTER_COMPARE_A,
        running_test == DPT
            ? dpt_off_cmp
            : FAST_ON_CMP);

    // =================================================
    // SINGLE DPT
    // =================================================

    if (running_test == DPT)
    {
        // Original DPT:
        // 50us timebase and CMPC cleanup.

        EPWM_setActionQualifierAction(
            EPWM_IN,
            EPWM_AQ_OUTPUT_A,
            EPWM_AQ_OUTPUT_LOW,
            EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

        EPWM_setCounterCompareValue(
            EPWM_IN,
            EPWM_COUNTER_COMPARE_B,
            dpt_end_cmp);

        EPWM_setCounterCompareValue(
            EPWM_IN,
            EPWM_COUNTER_COMPARE_C,
            dpt_done_cmp);

        EPWM_setInterruptSource(
            EPWM_IN,
            EPWM_INT_TBCTR_U_CMPC);

        sequence_state = SEQ_DPT_FAST;

        return;
    }

    // =================================================
    // QPT / SWEEP: 8us PERIOD
    // =================================================

    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

    EPWM_setTimeBasePeriod(
        EPWM_IN,
        FAST_TBPRD);

    // =================================================
    // QPT: UNCHANGED FOUR-PULSE PATTERN
    // =================================================

    if (running_test == QPT)
    {
        // QPT /INF transitions:
        // 2, 6, 18, 26, 30us

        EPWM_setActionQualifierAction(
            EPWM_INF,
            EPWM_AQ_OUTPUT_A,
            EPWM_AQ_OUTPUT_NO_CHANGE,
            EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        EPWM_setActionQualifierAction(
            EPWM_INF,
            EPWM_AQ_OUTPUT_A,
            EPWM_AQ_OUTPUT_NO_CHANGE,
            EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

        setINFCompareActions(
            EPWM_AQ_OUTPUT_HIGH,
            EPWM_AQ_OUTPUT_LOW);

        EPWM_setCounterCompareValue(
            EPWM_INF,
            EPWM_COUNTER_COMPARE_A,
            AT_US(2U));

        EPWM_setCounterCompareValue(
            EPWM_INF,
            EPWM_COUNTER_COMPARE_B,
            AT_US(6U));

        qpt_fast_index = 0U;

        sequence_state = SEQ_QPT_FAST;
    }

    // =================================================
    // SWEEP: REVERSED DELAYS
    // =================================================

    else
    {
        /*
         * For SWEEP only, load AQ shadow
         * registers at ZERO rather than PERIOD.
         *
         * This is required for the final pulse:
         * its special PERIOD -> HIGH action must
         * become active at t=40us, not earlier.
         */

        EPWM_setActionQualifierShadowLoadMode(
            EPWM_INF,
            EPWM_ACTION_QUALIFIER_A,
            EPWM_AQ_LOAD_ON_CNTR_ZERO);

        // Normal sweep periods:
        //
        // ZERO: /INF LOW
        // CMPA: /INF HIGH
        // CMPB: /INF LOW

        EPWM_setActionQualifierAction(
            EPWM_INF,
            EPWM_AQ_OUTPUT_A,
            EPWM_AQ_OUTPUT_LOW,
            EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        EPWM_setActionQualifierAction(
            EPWM_INF,
            EPWM_AQ_OUTPUT_A,
            EPWM_AQ_OUTPUT_NO_CHANGE,
            EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

        setINFCompareActions(
            EPWM_AQ_OUTPUT_HIGH,
            EPWM_AQ_OUTPUT_LOW);

        // Pulse 1: +300ns
        //
        // t=0.300us: /INF HIGH
        // t=4.300us: /INF LOW

        EPWM_setCounterCompareValue(
            EPWM_INF,
            EPWM_COUNTER_COMPARE_A,
            sweep_positive_counts[0]);

        EPWM_setCounterCompareValue(
            EPWM_INF,
            EPWM_COUNTER_COMPARE_B,
            (uint16_t)(
                FAST_ON_CMP +
                sweep_positive_counts[0]));

        sweep_delay_ns_active =
            sweep_delays_ns[0];

        sweep_fast_index = 0U;

        sequence_state = SEQ_SWEEP_FAST;
    }

    // ePWM6 initially uses the same 8us period.

    EPWM_setTimeBasePeriod(
        EPWM_INF,
        FAST_TBPRD);

    // Disable repeated synchronization
    // during the fast sequence.

    EPWM_disableSyncOutPulseSource(
        EPWM_IN,
        EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

    EPWM_setInterruptSource(
        EPWM_IN,
        EPWM_INT_TBCTR_ZERO);
}

// =====================================================
// PREPARE NEXT SWEEP PERIOD
//
// Called during the preceding 8us period.
//
// Compare shadow registers load at PERIOD.
// Sweep AQ shadow registers load at ZERO.
// =====================================================

static void prepareNextSweepPeriod(
    uint16_t next_index)
{
    // =================================================
    // PULSES 2..5: POSITIVE DELAYS
    // =================================================

    if (next_index < SWEEP_PULSES - 1U)
    {
        /*
         * CMPA:
         * Previous turn-off + previous delay.
         *
         * CMPB:
         * Next turn-on + next delay.
         */

        EPWM_setCounterCompareValue(
            EPWM_INF,
            EPWM_COUNTER_COMPARE_A,
            sweep_positive_counts[next_index - 1U]);

        EPWM_setCounterCompareValue(
            EPWM_INF,
            EPWM_COUNTER_COMPARE_B,
            (uint16_t)(
                FAST_ON_CMP +
                sweep_positive_counts[next_index]));

        EPWM_setActionQualifierAction(
            EPWM_INF,
            EPWM_AQ_OUTPUT_A,
            EPWM_AQ_OUTPUT_LOW,
            EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        EPWM_setActionQualifierAction(
            EPWM_INF,
            EPWM_AQ_OUTPUT_A,
            EPWM_AQ_OUTPUT_NO_CHANGE,
            EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

        setINFCompareActions(
            EPWM_AQ_OUTPUT_HIGH,
            EPWM_AQ_OUTPUT_LOW);
    }

    // =================================================
    // PULSE 6: -50ns
    // =================================================

    else if (next_index == SWEEP_PULSES - 1U)
    {
        /*
         * This prepares t=40..48us.
         *
         * It is called at t=32us, giving the
         * hardware time to load the configuration.
         *
         * Fifth turn-off:
         *   IN LOW at 40us
         *   /INF HIGH at 40.05us
         *
         * Sixth turn-on:
         *   /INF LOW at 43.95us
         *   IN HIGH at 44us
         *
         * Sixth turn-off:
         *   /INF HIGH at 47.95us
         *   IN LOW at 48us
         *
         * Three hardware events:
         *
         *   CMPA   -> HIGH
         *   CMPB   -> LOW
         *   PERIOD -> HIGH
         *
         * ePWM6 period is shortened by 50ns.
         * ePWM5 remains unchanged.
         */

        // t=40.05us: previous turn-off +50ns

        EPWM_setCounterCompareValue(
            EPWM_INF,
            EPWM_COUNTER_COMPARE_A,
            sweep_positive_counts[SWEEP_PULSES - 2U]);

        // t=43.95us: sixth turn-on -50ns

        EPWM_setCounterCompareValue(
            EPWM_INF,
            EPWM_COUNTER_COMPARE_B,
            (uint16_t)(
                FAST_ON_CMP - PREEDGE_COUNTS));

        /*
         * Important:
         *
         * ZERO must not force LOW.
         *
         * After the shortened period event at
         * t=47.95us, ePWM6 wraps before t=48us.
         * A LOW action at that ZERO would undo
         * the required boosted turn-off state.
         */

        EPWM_setActionQualifierAction(
            EPWM_INF,
            EPWM_AQ_OUTPUT_A,
            EPWM_AQ_OUTPUT_NO_CHANGE,
            EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        // t=47.95us: sixth turn-off -50ns

        EPWM_setActionQualifierAction(
            EPWM_INF,
            EPWM_AQ_OUTPUT_A,
            EPWM_AQ_OUTPUT_HIGH,
            EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

        setINFCompareActions(
            EPWM_AQ_OUTPUT_HIGH,
            EPWM_AQ_OUTPUT_LOW);

        // Shorten only the final ePWM6 period.

        EPWM_setTimeBasePeriod(
            EPWM_INF,
            SWEEP_FINAL_INF_TBPRD);
    }
}

// =====================================================
// STOP / CLEANUP
// =====================================================

static void stopTest(void)
{
    // MOSFET safely commanded OFF.

    forceIN(0U);

    if (running_test == DPT)
    {
        forceINF(
            dpt_boost_latched ? 0U : 1U);
    }
    else
    {
        forceINF(1U);
    }

    // Restore original 50us periods.

    EPWM_setTimeBasePeriod(
        EPWM_IN,
        SLOW_TBPRD);

    EPWM_setTimeBasePeriod(
        EPWM_INF,
        SLOW_TBPRD);

    // Restore ePWM5/ePWM6 synchronization.

    EPWM_enableSyncOutPulseSource(
        EPWM_IN,
        EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

    // Restore original AQ shadow loading mode.
    // Sweep temporarily uses ZERO for ePWM6.

    EPWM_setActionQualifierShadowLoadMode(
        EPWM_INF,
        EPWM_ACTION_QUALIFIER_A,
        EPWM_AQ_LOAD_ON_CNTR_PERIOD);

    // Restore IN action qualifier.

    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_LOW,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

    // Restore /INF action qualifier.

    EPWM_setActionQualifierAction(
        EPWM_INF,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    EPWM_setActionQualifierAction(
        EPWM_INF,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

    setINFCompareActions(
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_NO_CHANGE);

    // Restore the normal 20kHz interrupt.

    EPWM_setCounterCompareValue(
        EPWM_IN,
        EPWM_COUNTER_COMPARE_B,
        AT_US(25U));

    EPWM_setInterruptSource(
        EPWM_IN,
        EPWM_INT_TBCTR_ZERO);

    running_test = STOP;
    automated_test = STOP;

    sequence_state = SEQ_IDLE;

    automated_test_cnt = 0UL;
    dpt_state_monitor = SEQ_IDLE;
}

// =====================================================
// EPWM5 INTERRUPT
// =====================================================

__interrupt void epwm5_isr(void)
{
    epwm5_isr_count++;

    // =================================================
    // IDLE
    // =================================================

    if (sequence_state == SEQ_IDLE)
    {
        if (automated_test == DPT ||
            automated_test == QPT ||
            automated_test == DPT_SWEEP)
        {
            startTest();
        }
        else
        {
            forceIN(0U);

            forceINF(
                passive_boost ? 0U : 1U);
        }
    }

    // =================================================
    // MANUAL STOP
    // =================================================

    else if (automated_test == STOP)
    {
        stopTest();
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
                releaseINF();

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
                EPWM_setActionQualifierAction(
                    EPWM_INF,
                    EPWM_AQ_OUTPUT_A,
                    EPWM_AQ_OUTPUT_LOW,
                    EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
            }
        }

        // Final 50us charging period:
        // arm the fast sequence after 25us.

        if (current_index == charge_end - 1UL)
        {
            EPWM_setInterruptSource(
                EPWM_IN,
                EPWM_INT_TBCTR_U_CMPB);

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
                setINFCompareActions(
                    EPWM_AQ_OUTPUT_NO_CHANGE,
                    EPWM_AQ_OUTPUT_NO_CHANGE);

                qpt_step = 1U;

                break;
            }

            case 1U:
            {
                setINFCompareActions(
                    EPWM_AQ_OUTPUT_HIGH,
                    EPWM_AQ_OUTPUT_NO_CHANGE);

                qpt_step = 2U;

                break;
            }

            case 2U:
            {
                setINFCompareActions(
                    EPWM_AQ_OUTPUT_LOW,
                    EPWM_AQ_OUTPUT_HIGH);

                qpt_step = 3U;

                break;
            }

            case 3U:
            {
                // Disable further turn-on events.

                EPWM_setActionQualifierAction(
                    EPWM_IN,
                    EPWM_AQ_OUTPUT_A,
                    EPWM_AQ_OUTPUT_NO_CHANGE,
                    EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

                setINFCompareActions(
                    EPWM_AQ_OUTPUT_NO_CHANGE,
                    EPWM_AQ_OUTPUT_NO_CHANGE);

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

                EPWM_setActionQualifierAction(
                    EPWM_IN,
                    EPWM_AQ_OUTPUT_A,
                    EPWM_AQ_OUTPUT_NO_CHANGE,
                    EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
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

            forceINF(1U);
            forceIN(0U);

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

    EPWM_clearEventTriggerInterruptFlag(
        EPWM_IN);

    Interrupt_clearACKGroup(
        INTERRUPT_ACK_GROUP3);
}

// =====================================================
// ORIGINAL FLYBACK CALIBRATION
// =====================================================

static void flybackSetVdd(float vdd)
{
    static const float voltage[6] =
    {
        8.0f,
        12.7f,
        15.8f,
        17.5f,
        19.0f,
        20.7f
    };

    static const float duty[6] =
    {
        0.050f,
        0.080f,
        0.100f,
        0.110f,
        0.120f,
        0.130f
    };

    uint16_t i;

    if (!(vdd >= 11.0f))
    {
        vdd = 11.0f;
    }

    if (vdd > 22.0f)
    {
        vdd = 22.0f;
    }

    if (vdd <= voltage[0])
    {
        i = 0U;
    }
    else if (vdd >= voltage[5])
    {
        i = 4U;
    }
    else
    {
        for (i = 0U; i < 5U; ++i)
        {
            if (vdd <= voltage[i + 1U])
            {
                break;
            }
        }
    }

    flyback_pwm_duty =
        duty[i] +
        (vdd - voltage[i]) *
        (duty[i + 1U] - duty[i]) /
        (voltage[i + 1U] - voltage[i]);

    flyback_vdd_applied = vdd;
}

// =====================================================
// FLYBACK PWM INITIALIZATION
// =====================================================

static void initFlybackPWM(void)
{
    uint32_t counts =
        (DEVICE_SYSCLK_FREQ +
         flyback_pwm_frequency_hz / 2UL) /
         flyback_pwm_frequency_hz;

    uint16_t compare =
        (uint16_t)(
            (float)counts *
            flyback_pwm_duty + 0.5f);

    EPWM_setTimeBaseCounterMode(
        EPWM_FLYBACK,
        EPWM_COUNTER_MODE_STOP_FREEZE);

    EPWM_setClockPrescaler(
        EPWM_FLYBACK,
        EPWM_CLOCK_DIVIDER_1,
        EPWM_HSCLOCK_DIVIDER_1);

    EPWM_disablePhaseShiftLoad(
        EPWM_FLYBACK);

    EPWM_setTimeBaseCounter(
        EPWM_FLYBACK,
        0U);

    EPWM_setPeriodLoadMode(
        EPWM_FLYBACK,
        EPWM_PERIOD_DIRECT_LOAD);

    EPWM_setTimeBasePeriod(
        EPWM_FLYBACK,
        (uint16_t)(counts - 1UL));

    EPWM_setPeriodLoadMode(
        EPWM_FLYBACK,
        EPWM_PERIOD_SHADOW_LOAD);

    EPWM_setTimeBasePeriod(
        EPWM_FLYBACK,
        (uint16_t)(counts - 1UL));

    EPWM_setCounterCompareShadowLoadMode(
        EPWM_FLYBACK,
        EPWM_COUNTER_COMPARE_A,
        EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_setCounterCompareValue(
        EPWM_FLYBACK,
        EPWM_COUNTER_COMPARE_A,
        compare);

    EPWM_setActionQualifierAction(
        EPWM_FLYBACK,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    EPWM_setActionQualifierAction(
        EPWM_FLYBACK,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_LOW,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setActionQualifierContSWForceShadowMode(
        EPWM_FLYBACK,
        EPWM_AQ_SW_IMMEDIATE_LOAD);

    EPWM_setActionQualifierContSWForceAction(
        EPWM_FLYBACK,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_SW_OUTPUT_LOW);

    EPWM_setTimeBaseCounterMode(
        EPWM_FLYBACK,
        EPWM_COUNTER_MODE_UP);
}

// =====================================================
// FLYBACK PWM UPDATE
// =====================================================

static void updateFlybackPWM(void)
{
    uint32_t frequency =
        flyback_pwm_frequency_hz;

    float duty =
        flyback_pwm_duty;

    uint16_t enabled =
        flyback_pwm_enable;

    uint32_t counts;
    uint32_t compare;

    if (frequency == last_flyback_frequency &&
        duty == last_flyback_duty &&
        enabled == last_flyback_enable)
    {
        return;
    }

    if (frequency < FLYBACK_MIN_HZ ||
        frequency > FLYBACK_MAX_HZ ||
        !(duty >= 0.0f && duty <= 1.0f))
    {
        EPWM_setActionQualifierContSWForceAction(
            EPWM_FLYBACK,
            EPWM_AQ_OUTPUT_A,
            EPWM_AQ_SW_OUTPUT_LOW);

        flyback_pwm_config_error = 1U;

        flyback_pwm_actual_hz = 0.0f;
        flyback_pwm_actual_duty = 0.0f;
    }
    else
    {
        flyback_pwm_config_error = 0U;

        counts =
            (DEVICE_SYSCLK_FREQ +
             frequency / 2UL) /
             frequency;

        compare =
            (uint32_t)(
                (float)counts * duty + 0.5f);

        if (compare > counts)
        {
            compare = counts;
        }

        EPWM_setTimeBasePeriod(
            EPWM_FLYBACK,
            (uint16_t)(counts - 1UL));

        EPWM_setCounterCompareValue(
            EPWM_FLYBACK,
            EPWM_COUNTER_COMPARE_A,
            (uint16_t)compare);

        flyback_pwm_actual_hz =
            (float)DEVICE_SYSCLK_FREQ /
            (float)counts;

        flyback_pwm_actual_duty =
            (float)compare /
            (float)counts;

        if (enabled == 0U || compare == 0U)
        {
            EPWM_setActionQualifierContSWForceAction(
                EPWM_FLYBACK,
                EPWM_AQ_OUTPUT_A,
                EPWM_AQ_SW_OUTPUT_LOW);
        }
        else if (compare >= counts)
        {
            EPWM_setActionQualifierContSWForceAction(
                EPWM_FLYBACK,
                EPWM_AQ_OUTPUT_A,
                EPWM_AQ_SW_OUTPUT_HIGH);
        }
        else
        {
            EPWM_setActionQualifierContSWForceAction(
                EPWM_FLYBACK,
                EPWM_AQ_OUTPUT_A,
                EPWM_AQ_SW_DISABLED);
        }
    }

    last_flyback_frequency = frequency;
    last_flyback_duty = duty;
    last_flyback_enable = enabled;
}


#include "TI_EPWM.h"
#include "driverlib.h"
#include "device.h"
#include <stdint.h>

#define EPWM_IN       EPWM5_BASE
#define EPWM_INF      EPWM6_BASE
#define EPWM_FLYBACK  EPWM8_BASE

#define FLYBACK_MIN_HZ 4000UL
#define FLYBACK_MAX_HZ 100000UL

volatile uint32_t epwm5_isr_count = 0UL;
volatile float flyback_vdd_applied = 15.0f;
volatile float flyback_pwm_actual_hz = 0.0f;
volatile float flyback_pwm_actual_duty = 0.0f;
volatile uint16_t flyback_pwm_config_error = 0U;

// MOSFET switching PWM period.
static uint32_t continuous_pwm_counts = 0UL;

// Flyback duty override state.
static volatile uint16_t flyback_pid_override_active = 0U;
static volatile float flyback_pid_override_duty = 0.16f;
static volatile float flyback_calibrated_duty = 0.09483871f;

static uint32_t last_flyback_frequency = 0UL;
static float last_flyback_duty = -1.0f;
static uint16_t last_flyback_enable = 0xFFFFU;

static void initDPTPWM(void);
static void initFlybackPWM(void);
static void setINFCompareActions(uint16_t a, uint16_t b);

__interrupt void epwm5_isr(void);

static TI_EPWM_TickCallback epwm5_tick_callback = 0;

// =====================================================
// INTERRUPT
// =====================================================

void TI_EPWM_RegisterInterrupt(TI_EPWM_TickCallback on_tick)
{
    if (on_tick == 0)
    {
        return;
    }

    epwm5_tick_callback = on_tick;

    Interrupt_register(INT_EPWM5, &epwm5_isr);
    Interrupt_enable(INT_EPWM5);
}

__interrupt void epwm5_isr(void)
{
    ++epwm5_isr_count;

    if (epwm5_tick_callback != 0)
    {
        epwm5_tick_callback();
    }

    EPWM_clearEventTriggerInterruptFlag(EPWM_IN);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

// =====================================================
// INITIALIZATION
// =====================================================

void TI_EPWM_Init(void)
{
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

static void initDPTPWM(void)
{
    EPWM_setTimeBaseCounterMode(
        EPWM_IN, EPWM_COUNTER_MODE_STOP_FREEZE);

    EPWM_setTimeBaseCounterMode(
        EPWM_INF, EPWM_COUNTER_MODE_STOP_FREEZE);

    EPWM_setClockPrescaler(
        EPWM_IN,
        EPWM_CLOCK_DIVIDER_1,
        EPWM_HSCLOCK_DIVIDER_1);

    EPWM_setClockPrescaler(
        EPWM_INF,
        EPWM_CLOCK_DIVIDER_1,
        EPWM_HSCLOCK_DIVIDER_1);

    // Initial direct period loading.
    EPWM_setPeriodLoadMode(
        EPWM_IN, EPWM_PERIOD_DIRECT_LOAD);

    EPWM_setPeriodLoadMode(
        EPWM_INF, EPWM_PERIOD_DIRECT_LOAD);

    EPWM_setTimeBasePeriod(EPWM_IN, SLOW_TBPRD);
    EPWM_setTimeBasePeriod(EPWM_INF, SLOW_TBPRD);

    EPWM_setTimeBaseCounter(EPWM_IN, 0U);
    EPWM_setTimeBaseCounter(EPWM_INF, 0U);

    // Shadow period loading at ZERO.
    EPWM_setPeriodLoadMode(
        EPWM_IN, EPWM_PERIOD_SHADOW_LOAD);

    EPWM_setPeriodLoadMode(
        EPWM_INF, EPWM_PERIOD_SHADOW_LOAD);

    EPWM_selectPeriodLoadEvent(
        EPWM_IN,
        EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);

    EPWM_selectPeriodLoadEvent(
        EPWM_INF,
        EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);

    EPWM_setTimeBasePeriod(EPWM_IN, SLOW_TBPRD);
    EPWM_setTimeBasePeriod(EPWM_INF, SLOW_TBPRD);

    // ePWM5 master, ePWM6 slave.
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

    // Action qualifier shadow loading.
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

    // IN default state.
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

    // /INF default state.
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
        EPWM_IN, EPWM_COUNTER_COMPARE_A, FAST_ON_CMP);

    EPWM_setCounterCompareValue(
        EPWM_IN, EPWM_COUNTER_COMPARE_B, AT_US(25U));

    EPWM_setCounterCompareValue(
        EPWM_IN, EPWM_COUNTER_COMPARE_C, AT_US(6U));

    EPWM_setCounterCompareValue(
        EPWM_INF, EPWM_COUNTER_COMPARE_A, AT_US(2U));

    EPWM_setCounterCompareValue(
        EPWM_INF, EPWM_COUNTER_COMPARE_B, AT_US(6U));

    // Normal ePWM5 interrupt at ZERO.
    EPWM_setInterruptSource(
        EPWM_IN, EPWM_INT_TBCTR_ZERO);

    EPWM_setInterruptEventCount(EPWM_IN, 1U);

    EPWM_clearEventTriggerInterruptFlag(EPWM_IN);
    EPWM_enableInterrupt(EPWM_IN);

    EPWM_setActionQualifierContSWForceShadowMode(
        EPWM_IN, EPWM_AQ_SW_IMMEDIATE_LOAD);

    EPWM_setActionQualifierContSWForceShadowMode(
        EPWM_INF, EPWM_AQ_SW_IMMEDIATE_LOAD);

    TI_EPWM_ForceIN(0U);
    TI_EPWM_ForceINF(1U);

    EPWM_setTimeBaseCounterMode(
        EPWM_IN, EPWM_COUNTER_MODE_UP);

    EPWM_setTimeBaseCounterMode(
        EPWM_INF, EPWM_COUNTER_MODE_UP);
}

// =====================================================
// GATE CONTROL
// =====================================================

void TI_EPWM_ForceIN(uint16_t high)
{
    EPWM_setActionQualifierContSWForceAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        high ? EPWM_AQ_SW_OUTPUT_HIGH
             : EPWM_AQ_SW_OUTPUT_LOW);
}

void TI_EPWM_ForceINF(uint16_t high)
{
    EPWM_setActionQualifierContSWForceAction(
        EPWM_INF,
        EPWM_AQ_OUTPUT_A,
        high ? EPWM_AQ_SW_OUTPUT_HIGH
             : EPWM_AQ_SW_OUTPUT_LOW);
}

void TI_EPWM_ForceBothLow(void)
{
    TI_EPWM_ForceIN(0U);
    TI_EPWM_ForceINF(0U);
}

void TI_EPWM_ReleaseIN(void)
{
    EPWM_setActionQualifierContSWForceAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_SW_DISABLED);
}

void TI_EPWM_ReleaseINF(void)
{
    EPWM_setActionQualifierContSWForceAction(
        EPWM_INF,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_SW_DISABLED);
}

static void setINFCompareActions(uint16_t a, uint16_t b)
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
// SLOW CHARGING SEQUENCE
// =====================================================

void TI_EPWM_PrepareSlowPeriod(uint16_t charging)
{
    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        charging ? EPWM_AQ_OUTPUT_HIGH
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

void TI_EPWM_PrepareChargingINF(void)
{
    TI_EPWM_ForceINF(1U);

    EPWM_setActionQualifierAction(
        EPWM_INF,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    setINFCompareActions(
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_NO_CHANGE);
}

void TI_EPWM_ChargingMidpointINF(void)
{
    EPWM_setActionQualifierAction(
        EPWM_INF,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_LOW,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
}

void TI_EPWM_SelectArmingInterrupt(void)
{
    EPWM_setInterruptSource(
        EPWM_IN, EPWM_INT_TBCTR_U_CMPB);
}

// =====================================================
// QPT AND SWEEP
// =====================================================

void TI_EPWM_SuppressFutureTurnOn(void)
{
    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
}

void TI_EPWM_QPTStep(uint16_t index)
{
    switch (index)
    {
        case 0U:
            setINFCompareActions(
                EPWM_AQ_OUTPUT_NO_CHANGE,
                EPWM_AQ_OUTPUT_NO_CHANGE);
            break;

        case 1U:
            setINFCompareActions(
                EPWM_AQ_OUTPUT_HIGH,
                EPWM_AQ_OUTPUT_NO_CHANGE);
            break;

        case 2U:
            setINFCompareActions(
                EPWM_AQ_OUTPUT_LOW,
                EPWM_AQ_OUTPUT_HIGH);
            break;

        case 3U:
            TI_EPWM_SuppressFutureTurnOn();

            setINFCompareActions(
                EPWM_AQ_OUTPUT_NO_CHANGE,
                EPWM_AQ_OUTPUT_NO_CHANGE);
            break;

        default:
            break;
    }
}

void TI_EPWM_ArmFastSequence(
    TI_EPWM_FastSequence fast_mode,
    uint16_t dpt_off_cmp,
    uint16_t dpt_end_cmp,
    uint16_t dpt_done_cmp,
    uint16_t first_sweep_delay_counts)
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
        fast_mode == TI_EPWM_FAST_SINGLE
            ? dpt_off_cmp
            : FAST_ON_CMP);

    // Single DPT.
    if (fast_mode == TI_EPWM_FAST_SINGLE)
    {
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

        return;
    }

    // QPT / sweep 8 us period.
    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

    EPWM_setTimeBasePeriod(EPWM_IN, FAST_TBPRD);

    if (fast_mode == TI_EPWM_FAST_FOUR_PULSE)
    {
        // QPT transitions.
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
    }
    else
    {
        // DPT_SWEEP.
        EPWM_setActionQualifierShadowLoadMode(
            EPWM_INF,
            EPWM_ACTION_QUALIFIER_A,
            EPWM_AQ_LOAD_ON_CNTR_ZERO);

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

        EPWM_setCounterCompareValue(
            EPWM_INF,
            EPWM_COUNTER_COMPARE_A,
            first_sweep_delay_counts);

        EPWM_setCounterCompareValue(
            EPWM_INF,
            EPWM_COUNTER_COMPARE_B,
            (uint16_t)(
                FAST_ON_CMP + first_sweep_delay_counts));
    }

    EPWM_setTimeBasePeriod(EPWM_INF, FAST_TBPRD);

    EPWM_disableSyncOutPulseSource(
        EPWM_IN,
        EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

    EPWM_setInterruptSource(
        EPWM_IN, EPWM_INT_TBCTR_ZERO);
}

void TI_EPWM_SweepPreparePositive(
    uint16_t previous_delay_counts,
    uint16_t next_delay_counts)
{
    EPWM_setCounterCompareValue(
        EPWM_INF,
        EPWM_COUNTER_COMPARE_A,
        previous_delay_counts);

    EPWM_setCounterCompareValue(
        EPWM_INF,
        EPWM_COUNTER_COMPARE_B,
        (uint16_t)(FAST_ON_CMP + next_delay_counts));

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

void TI_EPWM_SweepPrepareNegative(
    uint16_t previous_delay_counts)
{
    EPWM_setCounterCompareValue(
        EPWM_INF,
        EPWM_COUNTER_COMPARE_A,
        previous_delay_counts);

    EPWM_setCounterCompareValue(
        EPWM_INF,
        EPWM_COUNTER_COMPARE_B,
        (uint16_t)(FAST_ON_CMP - PREEDGE_COUNTS));

    EPWM_setActionQualifierAction(
        EPWM_INF,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    EPWM_setActionQualifierAction(
        EPWM_INF,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

    setINFCompareActions(
        EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_LOW);

    EPWM_setTimeBasePeriod(
        EPWM_INF, SWEEP_FINAL_INF_TBPRD);
}

// =====================================================
// CONTINUOUS PWM WITH VARIABLE DUTY
// =====================================================

uint16_t TI_EPWM_ContinuousPWMFrequencyValid(
    uint32_t frequency_hz)
{
    uint32_t counts;

    if (frequency_hz < 4000UL ||
        frequency_hz > 50000UL)
    {
        return 0U;
    }

    counts =
        (DEVICE_SYSCLK_FREQ + frequency_hz / 2UL) /
        frequency_hz;

    if (counts < 4UL || counts > 65535UL)
    {
        return 0U;
    }

    return 1U;
}

void TI_EPWM_SetContinuousDuty(float duty)
{
    uint32_t compare;

    if (continuous_pwm_counts == 0UL ||
        !(duty > 0.0f && duty < 1.0f))
    {
        return;
    }

    // IN LOW at ZERO, HIGH after CMPA.
    // HIGH duty = (counts - CMPA) / counts.
    compare =
        (uint32_t)(
            (float)continuous_pwm_counts *
            (1.0f - duty) + 0.5f);

    if (compare < 1UL)
    {
        compare = 1UL;
    }

    if (compare >= continuous_pwm_counts)
    {
        compare = continuous_pwm_counts - 1UL;
    }

    EPWM_setCounterCompareValue(
        EPWM_IN,
        EPWM_COUNTER_COMPARE_A,
        (uint16_t)compare);
}

uint16_t TI_EPWM_ArmContinuousDuty(
    uint32_t frequency_hz,
    float duty)
{
    uint32_t counts;

    if (TI_EPWM_ContinuousPWMFrequencyValid(
            frequency_hz) == 0U ||
        !(duty > 0.0f && duty < 1.0f))
    {
        return 0U;
    }

    counts =
        (DEVICE_SYSCLK_FREQ + frequency_hz / 2UL) /
        frequency_hz;

    continuous_pwm_counts = counts;

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

    EPWM_setActionQualifierAction(
        EPWM_IN,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

    TI_EPWM_SetContinuousDuty(duty);

    EPWM_setTimeBasePeriod(
        EPWM_IN,
        (uint16_t)(counts - 1UL));

    // PWM edges are generated in hardware.
    EPWM_setInterruptSource(
        EPWM_IN,
        EPWM_INT_TBCTR_ZERO);

    return 1U;
}

// =====================================================
// STOP AND RESTORE
// =====================================================

void TI_EPWM_StopOutputs(
    uint16_t is_single_dpt,
    uint16_t boost_latched)
{
    TI_EPWM_ForceIN(0U);
    continuous_pwm_counts = 0UL;

    if (is_single_dpt != 0U)
    {
        TI_EPWM_ForceINF(
            boost_latched ? 0U : 1U);
    }
    else
    {
        TI_EPWM_ForceINF(1U);
    }

    EPWM_setTimeBasePeriod(
        EPWM_IN, SLOW_TBPRD);

    EPWM_setTimeBasePeriod(
        EPWM_INF, SLOW_TBPRD);

    EPWM_enableSyncOutPulseSource(
        EPWM_IN,
        EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

    EPWM_setActionQualifierShadowLoadMode(
        EPWM_INF,
        EPWM_ACTION_QUALIFIER_A,
        EPWM_AQ_LOAD_ON_CNTR_PERIOD);

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

    EPWM_setCounterCompareValue(
        EPWM_IN,
        EPWM_COUNTER_COMPARE_B,
        AT_US(25U));

    EPWM_setInterruptSource(
        EPWM_IN,
        EPWM_INT_TBCTR_ZERO);
}

// =====================================================
// TEMPORARY FLYBACK PID DUTY OVERRIDE
// =====================================================

void TI_EPWM_SetFlybackPidOverride(
    uint16_t enable,
    float pid_duty)
{
    if (enable != 0U &&
        pid_duty > 0.0f &&
        pid_duty < 1.0f)
    {
        flyback_pid_override_duty = pid_duty;
        flyback_pid_override_active = 1U;

        // Temporary duty: default 0.16.
        flyback_pwm_duty = pid_duty;
    }
    else
    {
        // Restore calibrated duty for current Ug.
        flyback_pid_override_active = 0U;
        flyback_pwm_duty = flyback_calibrated_duty;
    }
}

// =====================================================
// FLYBACK VOLTAGE CALIBRATION
// =====================================================

void TI_EPWM_SetFlybackVdd(float vdd)
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

    // Disable flyback PWM below 11 V.
    if (!(vdd >= 11.0f))
    {
        flyback_pid_override_active = 0U;
        flyback_calibrated_duty = 0.0f;
        flyback_pwm_duty = 0.0f;
        flyback_vdd_applied = 0.0f;
        return;
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

    // Store the normal calibrated duty.
    flyback_calibrated_duty =
        duty[i] +
        (vdd - voltage[i]) *
        (duty[i + 1U] - duty[i]) /
        (voltage[i + 1U] - voltage[i]);

    // Main-loop updates must preserve the PID override.
    if (flyback_pid_override_active != 0U)
    {
        flyback_pwm_duty = flyback_pid_override_duty;
    }
    else
    {
        flyback_pwm_duty = flyback_calibrated_duty;
    }

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

    EPWM_disablePhaseShiftLoad(EPWM_FLYBACK);

    EPWM_setTimeBaseCounter(EPWM_FLYBACK, 0U);

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
// APPLY FLYBACK PWM UPDATES
// =====================================================

void TI_EPWM_UpdateFlybackPWM(void)
{
    uint32_t frequency = flyback_pwm_frequency_hz;
    float duty = flyback_pwm_duty;
    uint16_t enabled = flyback_pwm_enable;

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

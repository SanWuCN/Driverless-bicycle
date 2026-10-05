#include "balance_monitor.h"

#include "main.h"
#include "zero_persistence.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define TELEMETRY_PERIOD_MS             50u
#define ZERO_GATE_TIME_MS               5000u
#define ZERO_ROLL_WINDOW_DEG             2.0f
#define ZERO_ROLL_RATE_WINDOW_DPS        2.0f
#define ZERO_WHEEL_SPEED_WINDOW_TPS     15.0f
#define ZERO_WHEEL_BIAS_WINDOW_DEG       0.5f
#define ZERO_WHEEL_DEADBAND_TPS          0.5f
#define ZERO_SPEED_FILTER_TAU_S          3.0f
#define ZERO_ADAPT_GAIN                  0.001f
#define ZERO_ADAPT_RATE_LIMIT_DPS        0.003f
#define ZERO_CANDIDATE_RANGE_DEG         1.50f
#define ODRIVE_VELOCITY_LIMIT_TPS       50.0f
#define ODRIVE_VEL_RAMP_RATE_TPS2       50.0f
#define WHEEL_BIAS_LIMIT_DEG             1.5f
#define DYNAMIC_ZERO_PILOT_APPLY          1

volatile BalanceTelemetry g_balance_telemetry;

static float initial_zero_deg;
static float zero_range_center_deg;
static float zero_candidate_deg;
static float filtered_wheel_speed_tps;
static float previous_velocity_command_tps;
static float last_acceleration_command_tps2;
static uint32_t previous_update_ms;
static uint32_t zero_gate_start_ms;
static bool velocity_command_initialized;
static bool zero_gate_started;

static float clamp_float(float value, float lower, float upper)
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

void balance_monitor_init(float range_center_zero, float initial_zero)
{
    memset((void *)&g_balance_telemetry, 0, sizeof(g_balance_telemetry));
    initial_zero_deg = initial_zero;
    zero_range_center_deg = range_center_zero;
    zero_candidate_deg = initial_zero;
    filtered_wheel_speed_tps = 0.0f;
    previous_velocity_command_tps = 0.0f;
    last_acceleration_command_tps2 = 0.0f;
    previous_update_ms = HAL_GetTick();
    zero_gate_start_ms = 0u;
    velocity_command_initialized = false;
    zero_gate_started = false;
    g_balance_telemetry.base_zero_deg = initial_zero;
    g_balance_telemetry.control_zero_deg = initial_zero;
    g_balance_telemetry.zero_candidate_deg = initial_zero;
    g_balance_telemetry.persisted_zero_deg = zero_persistence_stored_zero();
    g_balance_telemetry.persistence_sequence = zero_persistence_sequence();
}

float balance_monitor_zero_for_control(void)
{
#if DYNAMIC_ZERO_PILOT_APPLY
    return zero_candidate_deg;
#else
    return initial_zero_deg;
#endif
}

void balance_monitor_update(float roll_deg,
                            float roll_rate_dps,
                            float wheel_speed_tps,
                            float velocity_command_tps,
                            float base_zero_deg,
                            float wheel_bias_deg,
                            bool control_armed,
                            bool arm_window_active,
                            bool rear_wheel_stopped,
                            bool imu_ready)
{
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t elapsed_ms = now_ms - previous_update_ms;
    const float elapsed_s = (elapsed_ms > 0u && elapsed_ms <= 10u)
                                ? ((float)elapsed_ms * 0.001f)
                                : 0.0f;
    const float active_zero_deg = balance_monitor_zero_for_control();
    uint32_t flags = 0u;
    float acceleration_command_tps2 = last_acceleration_command_tps2;
    float zero_adaptation_rate_dps = 0.0f;
    const bool finite_inputs = isfinite(roll_deg) &&
                               isfinite(roll_rate_dps) &&
                               isfinite(wheel_speed_tps) &&
                               isfinite(velocity_command_tps);

    if (velocity_command_initialized && elapsed_s > 0.0f)
    {
        acceleration_command_tps2 =
            (velocity_command_tps - previous_velocity_command_tps) / elapsed_s;
        last_acceleration_command_tps2 = acceleration_command_tps2;
        previous_velocity_command_tps = velocity_command_tps;
        previous_update_ms = now_ms;
    }
    else if (!velocity_command_initialized)
    {
        velocity_command_initialized = true;
        previous_velocity_command_tps = velocity_command_tps;
        previous_update_ms = now_ms;
    }

    if (elapsed_s > 0.0f)
    {
        const float filter_alpha = elapsed_s / (ZERO_SPEED_FILTER_TAU_S + elapsed_s);
        filtered_wheel_speed_tps += filter_alpha *
                                    (wheel_speed_tps - filtered_wheel_speed_tps);
    }

    const bool zero_gate = finite_inputs &&
                           control_armed &&
                           rear_wheel_stopped &&
                           (fabsf(roll_deg - active_zero_deg) <= ZERO_ROLL_WINDOW_DEG) &&
                           (fabsf(roll_rate_dps) <= ZERO_ROLL_RATE_WINDOW_DPS) &&
                           (fabsf(wheel_speed_tps) <= ZERO_WHEEL_SPEED_WINDOW_TPS) &&
                           (fabsf(wheel_bias_deg) <= ZERO_WHEEL_BIAS_WINDOW_DEG);

    if (zero_gate)
    {
        flags |= BALANCE_MONITOR_ZERO_GATE_ACTIVE;
        if (!zero_gate_started)
        {
            zero_gate_start_ms = now_ms;
            zero_gate_started = true;
        }
        else if ((uint32_t)(now_ms - zero_gate_start_ms) >= ZERO_GATE_TIME_MS)
        {
            float wheel_error_tps = 0.0f;

            flags |= BALANCE_MONITOR_ZERO_LEARNING;
            if (filtered_wheel_speed_tps > ZERO_WHEEL_DEADBAND_TPS)
            {
                wheel_error_tps = filtered_wheel_speed_tps - ZERO_WHEEL_DEADBAND_TPS;
            }
            else if (filtered_wheel_speed_tps < -ZERO_WHEEL_DEADBAND_TPS)
            {
                wheel_error_tps = filtered_wheel_speed_tps + ZERO_WHEEL_DEADBAND_TPS;
            }
            zero_adaptation_rate_dps = clamp_float(
                -ZERO_ADAPT_GAIN * wheel_error_tps,
                -ZERO_ADAPT_RATE_LIMIT_DPS,
                ZERO_ADAPT_RATE_LIMIT_DPS);
            zero_candidate_deg += zero_adaptation_rate_dps * elapsed_s;
            zero_candidate_deg = clamp_float(
                zero_candidate_deg,
                zero_range_center_deg - ZERO_CANDIDATE_RANGE_DEG,
                zero_range_center_deg + ZERO_CANDIDATE_RANGE_DEG);
        }
    }
    else
    {
        zero_gate_started = false;
    }

#if DYNAMIC_ZERO_PILOT_APPLY
    flags |= BALANCE_MONITOR_ZERO_APPLIED;
#endif
    if (control_armed)
    {
        flags |= BALANCE_MONITOR_CONTROL_ARMED;
    }
    if (arm_window_active)
    {
        flags |= BALANCE_MONITOR_ARM_WINDOW;
    }
    if (imu_ready)
    {
        flags |= BALANCE_MONITOR_IMU_READY;
    }
    if (fabsf(zero_candidate_deg - zero_range_center_deg) >=
        (ZERO_CANDIDATE_RANGE_DEG - 0.0001f))
    {
        flags |= BALANCE_MONITOR_ZERO_LIMIT;
    }

    if (fabsf(wheel_bias_deg) >= (WHEEL_BIAS_LIMIT_DEG - 0.001f))
    {
        flags |= BALANCE_MONITOR_WHEEL_BIAS_SATURATED;
    }
    if (fabsf(velocity_command_tps) >= ODRIVE_VELOCITY_LIMIT_TPS)
    {
        flags |= BALANCE_MONITOR_VELOCITY_SATURATED;
    }
    if (fabsf(acceleration_command_tps2) > ODRIVE_VEL_RAMP_RATE_TPS2)
    {
        flags |= BALANCE_MONITOR_ACCEL_SATURATED;
    }

    const uint32_t persistence_flags = zero_persistence_status();
    if ((persistence_flags & ZERO_PERSISTENCE_VALID) != 0u)
    {
        flags |= BALANCE_MONITOR_ZERO_PERSIST_VALID;
    }
    if ((persistence_flags & ZERO_PERSISTENCE_SAVED_THIS_BOOT) != 0u)
    {
        flags |= BALANCE_MONITOR_ZERO_PERSIST_SAVED;
    }
    if ((persistence_flags & ZERO_PERSISTENCE_ERROR) != 0u)
    {
        flags |= BALANCE_MONITOR_ZERO_PERSIST_ERROR;
    }

    g_balance_telemetry.sample_sequence++;
    g_balance_telemetry.uptime_ms = HAL_GetTick();
    g_balance_telemetry.roll_deg = roll_deg;
    g_balance_telemetry.roll_rate_dps = roll_rate_dps;
    g_balance_telemetry.wheel_speed_tps = wheel_speed_tps;
    g_balance_telemetry.velocity_command_tps = velocity_command_tps;
    g_balance_telemetry.acceleration_command_tps2 = acceleration_command_tps2;
    g_balance_telemetry.base_zero_deg = base_zero_deg;
    g_balance_telemetry.control_zero_deg = active_zero_deg;
    g_balance_telemetry.zero_candidate_deg = zero_candidate_deg;
    g_balance_telemetry.persisted_zero_deg = zero_persistence_stored_zero();
    g_balance_telemetry.persistence_sequence = zero_persistence_sequence();
    g_balance_telemetry.zero_adaptation_rate_dps = zero_adaptation_rate_dps;
    g_balance_telemetry.wheel_bias_deg = wheel_bias_deg;
    g_balance_telemetry.flags = flags;
}

void balance_monitor_get_snapshot(BalanceTelemetry *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(snapshot, (const void *)&g_balance_telemetry, sizeof(*snapshot));
    if (primask == 0u)
    {
        __enable_irq();
    }
}

static void uart7_write(const char *text, size_t length)
{
    for (size_t index = 0; index < length; index++)
    {
        while (!LL_USART_IsActiveFlag_TXE(UART7))
        {
        }
        LL_USART_TransmitData8(UART7, (uint8_t)text[index]);
    }
}

void balance_telemetry_service(void)
{
    static uint32_t last_send_ms;
    static bool header_sent;
    BalanceTelemetry snapshot;
    char line[256];
    const uint32_t now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - last_send_ms) < TELEMETRY_PERIOD_MS)
    {
        return;
    }
    last_send_ms = now_ms;

    balance_monitor_get_snapshot(&snapshot);

    if (!header_sent)
    {
        static const char header[] =
            "#B,time_ms,seq,roll_deg,roll_rate_dps,wheel_tps,vel_cmd_tps,"
            "accel_cmd_tps2,zero_base_deg,zero_control_deg,zero_candidate_deg,"
            "zero_persisted_deg,zero_persist_seq,zero_rate_dps,wheel_bias_deg,flags\r\n";
        uart7_write(header, sizeof(header) - 1u);
        header_sent = true;
    }

    const int length = snprintf(
        line,
        sizeof(line),
        "B,%lu,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%lu,%.6f,%.4f,0x%04lX\r\n",
        (unsigned long)snapshot.uptime_ms,
        (unsigned long)snapshot.sample_sequence,
        (double)snapshot.roll_deg,
        (double)snapshot.roll_rate_dps,
        (double)snapshot.wheel_speed_tps,
        (double)snapshot.velocity_command_tps,
        (double)snapshot.acceleration_command_tps2,
        (double)snapshot.base_zero_deg,
        (double)snapshot.control_zero_deg,
        (double)snapshot.zero_candidate_deg,
        (double)snapshot.persisted_zero_deg,
        (unsigned long)snapshot.persistence_sequence,
        (double)snapshot.zero_adaptation_rate_dps,
        (double)snapshot.wheel_bias_deg,
        (unsigned long)snapshot.flags);

    if (length > 0)
    {
        const size_t output_length =
            ((size_t)length < sizeof(line)) ? (size_t)length : (sizeof(line) - 1u);
        uart7_write(line, output_length);
    }
}

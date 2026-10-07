#include "balance_monitor.h"

#include "main.h"
#include "task.h"
#include "uart7_bridge.h"
#include "zero_persistence.h"
#include "steering.h"
#include "drive_control.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define TELEMETRY_PERIOD_MS             50u
#define STEERING_TELEMETRY_DIVIDER       6u
#define BASIC_TELEMETRY_DIVIDER         10u
#define COMPACT_PROTOCOL_VERSION         1u
#define COMPACT_BALANCE_FRAME_TYPE       1u
#define COMPACT_STEERING_FRAME_TYPE      2u
#define COMPACT_BASIC_FRAME_TYPE         3u
#define COMPACT_FRAME_MAGIC_0          0xA5u
#define COMPACT_FRAME_MAGIC_1          0x5Au
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
#define ODRIVE_VELOCITY_LIMIT_TPS       35.0f
#define RATE_TARGET_LIMIT_DPS            5.0f
#define WHEEL_BIAS_LIMIT_DEG             1.5f
#define DYNAMIC_ZERO_PILOT_APPLY          1

volatile BalanceTelemetry g_balance_telemetry;

static float initial_zero_deg;
static float zero_range_center_deg;
static float zero_candidate_deg;
static float filtered_wheel_speed_tps;
static uint32_t previous_update_ms;
static uint32_t zero_gate_start_ms;
static bool zero_gate_started;
static volatile BalanceTelemetryMode telemetry_mode = BALANCE_TELEMETRY_BASIC;

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
    previous_update_ms = HAL_GetTick();
    zero_gate_start_ms = 0u;
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
                            bool imu_ready,
                            bool odrive_ready,
                            bool odrive_timeout,
                            bool odrive_fault,
                            bool fall_disarm_latched,
                            float rate_target_dps,
                            float rate_error_dps,
                            float rate_p_term_tps,
                            float rate_i_term_tps,
                            float rate_d_term_tps,
                            float raw_velocity_command_tps,
                            float raw_acceleration_command_tps2,
                            float limited_acceleration_command_tps2,
                            float current_command_a,
                            float torque_command_nm,
                            bool acceleration_boost_120_active,
                            bool speed_envelope_active,
                            bool torque_control_active)
{
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t elapsed_ms = now_ms - previous_update_ms;
    const float elapsed_s = (elapsed_ms > 0u && elapsed_ms <= 10u)
                                ? ((float)elapsed_ms * 0.001f)
                                : 0.0f;
    previous_update_ms = now_ms;
    const float active_zero_deg = balance_monitor_zero_for_control();
    uint32_t flags = 0u;
    const float acceleration_command_tps2 =
        limited_acceleration_command_tps2;
    float zero_adaptation_rate_dps = 0.0f;
    const bool finite_inputs = isfinite(roll_deg) &&
                               isfinite(roll_rate_dps) &&
                               isfinite(wheel_speed_tps) &&
                               isfinite(velocity_command_tps) &&
                               isfinite(raw_velocity_command_tps) &&
                               isfinite(raw_acceleration_command_tps2) &&
                               isfinite(limited_acceleration_command_tps2) &&
                               isfinite(current_command_a) &&
                               isfinite(torque_command_nm);

    if (elapsed_s > 0.0f)
    {
        const float filter_alpha = elapsed_s / (ZERO_SPEED_FILTER_TAU_S + elapsed_s);
        filtered_wheel_speed_tps += filter_alpha *
                                    (wheel_speed_tps - filtered_wheel_speed_tps);
    }

    const bool steering_zero_learning_allowed =
        steering_allows_zero_learning();
    const bool zero_gate = finite_inputs &&
                           steering_zero_learning_allowed &&
                           control_armed &&
                           odrive_ready &&
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
    if (odrive_ready)
    {
        flags |= BALANCE_MONITOR_ODRIVE_READY;
    }
    if (odrive_timeout)
    {
        flags |= BALANCE_MONITOR_ODRIVE_TIMEOUT;
    }
    if (odrive_fault)
    {
        flags |= BALANCE_MONITOR_ODRIVE_FAULT;
    }
    if (fall_disarm_latched)
    {
        flags |= BALANCE_MONITOR_FALL_DISARM_LATCHED;
    }
    if (!steering_zero_learning_allowed)
    {
        flags |= BALANCE_MONITOR_ZERO_STEERING_BLOCKED;
    }
    if (acceleration_boost_120_active)
    {
        flags |= BALANCE_MONITOR_ACCEL_BOOST_120_ACTIVE;
    }
    if (speed_envelope_active)
    {
        flags |= BALANCE_MONITOR_SPEED_ENVELOPE_ACTIVE;
    }
    if (torque_control_active)
    {
        flags |= BALANCE_MONITOR_TORQUE_CONTROL_ACTIVE;
    }
    if (uart7_rx_count != 0u)
    {
        flags |= BALANCE_MONITOR_UART7_RX_SEEN;
    }
    if (uart7_rx_overflow_count != 0u)
    {
        flags |= BALANCE_MONITOR_UART7_RX_OVERFLOW;
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
    if (fabsf(raw_acceleration_command_tps2 -
              limited_acceleration_command_tps2) > 0.001f)
    {
        flags |= BALANCE_MONITOR_ACCEL_SATURATED;
    }
    if (fabsf(rate_target_dps) >= (RATE_TARGET_LIMIT_DPS - 0.001f))
    {
        flags |= BALANCE_MONITOR_RATE_TARGET_SATURATED;
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
    g_balance_telemetry.rate_target_dps = rate_target_dps;
    g_balance_telemetry.rate_error_dps = rate_error_dps;
    g_balance_telemetry.rate_p_term_tps = rate_p_term_tps;
    g_balance_telemetry.rate_i_term_tps = rate_i_term_tps;
    g_balance_telemetry.rate_d_term_tps = rate_d_term_tps;
    g_balance_telemetry.raw_velocity_command_tps = raw_velocity_command_tps;
    g_balance_telemetry.velocity_slew_error_tps =
        raw_velocity_command_tps - velocity_command_tps;
    g_balance_telemetry.raw_acceleration_command_tps2 =
        raw_acceleration_command_tps2;
    g_balance_telemetry.acceleration_limit_error_tps2 =
        raw_acceleration_command_tps2 - limited_acceleration_command_tps2;
    g_balance_telemetry.current_command_a = current_command_a;
    g_balance_telemetry.torque_command_nm = torque_command_nm;
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

void balance_telemetry_set_mode(BalanceTelemetryMode mode)
{
    telemetry_mode = mode;
}

BalanceTelemetryMode balance_telemetry_get_mode(void)
{
    return telemetry_mode;
}

#define UART7_TX_SPIN_LIMIT 1000000u

static bool uart7_write(const char *text, size_t length)
{
    for (size_t index = 0; index < length; index++)
    {
        uint32_t spins = 0u;
        while (!LL_USART_IsActiveFlag_TXE(UART7))
        {
            if (++spins >= UART7_TX_SPIN_LIMIT)
            {
                return false;
            }
        }
        LL_USART_TransmitData8(UART7, (uint8_t)text[index]);
    }
    return true;
}

static void put_u16_le(uint8_t *output, uint16_t value)
{
    output[0] = (uint8_t)(value & 0xFFu);
    output[1] = (uint8_t)(value >> 8u);
}

static void put_u32_le(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)(value & 0xFFu);
    output[1] = (uint8_t)((value >> 8u) & 0xFFu);
    output[2] = (uint8_t)((value >> 16u) & 0xFFu);
    output[3] = (uint8_t)(value >> 24u);
}

static int16_t quantize_i16(float value, float scale)
{
    if (!isfinite(value))
    {
        return 0;
    }
    const float scaled = roundf(value * scale);
    if (scaled > 32767.0f)
    {
        return INT16_MAX;
    }
    if (scaled < -32768.0f)
    {
        return INT16_MIN;
    }
    return (int16_t)scaled;
}

static uint16_t quantize_u16(float value, float scale)
{
    if (!isfinite(value) || value <= 0.0f)
    {
        return 0u;
    }
    const float scaled = roundf(value * scale);
    if (scaled > 65535.0f)
    {
        return UINT16_MAX;
    }
    return (uint16_t)scaled;
}

static int32_t quantize_i32(float value, float scale)
{
    if (!isfinite(value))
    {
        return 0;
    }
    const double scaled = round((double)value * (double)scale);
    if (scaled > (double)INT32_MAX)
    {
        return INT32_MAX;
    }
    if (scaled < (double)INT32_MIN)
    {
        return INT32_MIN;
    }
    return (int32_t)scaled;
}

static uint16_t telemetry_crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFu;
    for (size_t index = 0u; index < length; index++)
    {
        crc ^= (uint16_t)data[index] << 8u;
        for (uint8_t bit = 0u; bit < 8u; bit++)
        {
            crc = (crc & 0x8000u) != 0u
                      ? (uint16_t)((crc << 1u) ^ 0x1021u)
                      : (uint16_t)(crc << 1u);
        }
    }
    return crc;
}

static void compact_frame_begin(uint8_t *frame, uint8_t frame_type)
{
    frame[0] = COMPACT_FRAME_MAGIC_0;
    frame[1] = COMPACT_FRAME_MAGIC_1;
    frame[2] = COMPACT_PROTOCOL_VERSION;
    frame[3] = frame_type;
    frame[4] = 0u;
}

static bool compact_frame_finish(uint8_t *frame, size_t payload_end)
{
    const size_t total_length = payload_end + 2u;
    if (total_length > UINT8_MAX)
    {
        return false;
    }
    frame[4] = (uint8_t)total_length;
    const uint16_t crc = telemetry_crc16_ccitt(frame + 2u, payload_end - 2u);
    put_u16_le(frame + payload_end, crc);
    return uart7_write((const char *)frame, total_length);
}

static void compact_put_i16(uint8_t *frame, size_t *index, float value, float scale)
{
    put_u16_le(frame + *index, (uint16_t)quantize_i16(value, scale));
    *index += 2u;
}

static void send_compact_balance(const BalanceTelemetry *snapshot)
{
    uint8_t frame[65];
    size_t index = 5u;
    compact_frame_begin(frame, COMPACT_BALANCE_FRAME_TYPE);
    put_u32_le(frame + index, snapshot->sample_sequence);
    index += 4u;
    put_u32_le(frame + index, snapshot->uptime_ms);
    index += 4u;
    put_u32_le(frame + index, snapshot->persistence_sequence);
    index += 4u;

    compact_put_i16(frame, &index, snapshot->roll_deg, 1000.0f);
    compact_put_i16(frame, &index, snapshot->roll_rate_dps, 100.0f);
    compact_put_i16(frame, &index, snapshot->wheel_speed_tps, 100.0f);
    compact_put_i16(frame, &index, snapshot->velocity_command_tps, 100.0f);
    compact_put_i16(frame, &index, snapshot->acceleration_command_tps2, 10.0f);
    compact_put_i16(frame, &index, snapshot->base_zero_deg, 1000.0f);
    compact_put_i16(frame, &index, snapshot->control_zero_deg, 1000.0f);
    compact_put_i16(frame, &index, snapshot->zero_candidate_deg, 1000.0f);
    compact_put_i16(frame, &index, snapshot->persisted_zero_deg, 1000.0f);
    compact_put_i16(frame, &index, snapshot->zero_adaptation_rate_dps, 1000000.0f);
    compact_put_i16(frame, &index, snapshot->wheel_bias_deg, 1000.0f);
    compact_put_i16(frame, &index, snapshot->rate_target_dps, 100.0f);
    compact_put_i16(frame, &index, snapshot->rate_error_dps, 100.0f);
    compact_put_i16(frame, &index, snapshot->rate_p_term_tps, 100.0f);
    compact_put_i16(frame, &index, snapshot->rate_i_term_tps, 100.0f);
    compact_put_i16(frame, &index, snapshot->rate_d_term_tps, 100.0f);
    compact_put_i16(frame, &index, snapshot->raw_velocity_command_tps, 100.0f);
    compact_put_i16(frame, &index, snapshot->velocity_slew_error_tps, 100.0f);
    compact_put_i16(frame, &index, snapshot->raw_acceleration_command_tps2, 10.0f);
    compact_put_i16(frame, &index, snapshot->acceleration_limit_error_tps2, 10.0f);
    compact_put_i16(frame, &index, snapshot->current_command_a, 100.0f);
    put_u32_le(frame + index, snapshot->flags);
    index += 4u;
    (void)compact_frame_finish(frame, index);
}

static void send_compact_steering(void)
{
    SteeringTelemetry steering;
    uint8_t frame[34];
    size_t index = 5u;
    steering_get_telemetry(&steering);
    compact_frame_begin(frame, COMPACT_STEERING_FRAME_TYPE);
    put_u32_le(frame + index, steering.uptime_ms);
    index += 4u;
    frame[index++] = (uint8_t)steering.mode;
    put_u16_le(frame + index, quantize_u16(steering.yaw_deg, 100.0f));
    index += 2u;
    compact_put_i16(frame, &index, steering.yaw_rate_dps, 100.0f);
    put_u16_le(frame + index,
               quantize_u16(steering.target_heading_deg, 100.0f));
    index += 2u;
    compact_put_i16(frame, &index, steering.heading_error_deg, 100.0f);
    compact_put_i16(frame, &index, steering.rear_wheel_tps, 100.0f);
    compact_put_i16(frame, &index, steering.integral_deg_s, 100.0f);
    compact_put_i16(frame, &index, steering.raw_offset_us, 10.0f);
    compact_put_i16(frame, &index, steering.commanded_offset_us, 10.0f);
    put_u16_le(frame + index, steering.pulse_us);
    index += 2u;
    put_u32_le(frame + index, steering.flags);
    index += 4u;
    (void)compact_frame_finish(frame, index);
}

static void send_compact_basic(const BalanceTelemetry *balance)
{
    SteeringTelemetry steering;
    DriveTelemetry drive;
    uint8_t frame[46];
    size_t index = 5u;
    steering_get_telemetry(&steering);
    drive_get_telemetry(&drive);
    compact_frame_begin(frame, COMPACT_BASIC_FRAME_TYPE);
    put_u32_le(frame + index, drive.uptime_ms);
    index += 4u;
    put_u32_le(frame + index, balance->flags);
    index += 4u;
    frame[index++] = (uint8_t)steering.mode;
    compact_put_i16(frame, &index, steering.raw_offset_us, 10.0f);
    compact_put_i16(frame, &index, steering.commanded_offset_us, 10.0f);
    put_u16_le(frame + index, steering.pulse_us);
    index += 2u;
    put_u32_le(frame + index, steering.flags);
    index += 4u;
    frame[index++] = (uint8_t)drive.mode;
    frame[index++] = (uint8_t)drive.demo_phase;
    put_u32_le(frame + index, drive.flags);
    index += 4u;
    compact_put_i16(frame, &index, drive.requested_speed_mps, 1000.0f);
    compact_put_i16(frame, &index, drive.output_speed_mps, 1000.0f);
    compact_put_i16(frame, &index, drive.actual_speed_mps, 1000.0f);
    put_u32_le(frame + index,
               (uint32_t)quantize_i32(drive.odometry_m, 1000.0f));
    index += 4u;
    compact_put_i16(frame, &index, drive.segment_distance_m, 1000.0f);
    put_u16_le(frame + index, drive.demo_cycle_count);
    index += 2u;
    (void)compact_frame_finish(frame, index);
}

static void send_full_text_telemetry(const BalanceTelemetry *snapshot)
{
    static bool header_sent;
    static bool steering_header_sent;
    BalanceTuningParameters tuning;
    char line[512];
    balance_tuning_get(&tuning);

    if (!header_sent)
    {
        static const char header[] =
            "#B,time_ms,seq,roll_deg,roll_rate_dps,wheel_tps,vel_cmd_tps,"
            "accel_cmd_tps2,zero_base_deg,zero_control_deg,zero_candidate_deg,"
            "zero_persisted_deg,zero_persist_seq,zero_rate_dps,wheel_bias_deg,"
            "rate_target_dps,rate_error_dps,rate_p_tps,rate_i_tps,rate_d_tps,"
            "vel_raw_tps,vel_slew_error_tps,accel_raw_tps2,"
            "accel_limit_error_tps2,current_cmd_a,torque_cmd_nm,"
            "rate_kp,rate_ki,rate_kd,angle_kp,"
            "angle_ki,angle_kd,wheel_kp,wheel_ki,flags\r\n";
        header_sent = uart7_write(header, sizeof(header) - 1u);
    }

    const int length = snprintf(
        line,
        sizeof(line),
        "B,%lu,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%lu,"
        "%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,0x%08lX\r\n",
        (unsigned long)snapshot->uptime_ms,
        (unsigned long)snapshot->sample_sequence,
        (double)snapshot->roll_deg,
        (double)snapshot->roll_rate_dps,
        (double)snapshot->wheel_speed_tps,
        (double)snapshot->velocity_command_tps,
        (double)snapshot->acceleration_command_tps2,
        (double)snapshot->base_zero_deg,
        (double)snapshot->control_zero_deg,
        (double)snapshot->zero_candidate_deg,
        (double)snapshot->persisted_zero_deg,
        (unsigned long)snapshot->persistence_sequence,
        (double)snapshot->zero_adaptation_rate_dps,
        (double)snapshot->wheel_bias_deg,
        (double)snapshot->rate_target_dps,
        (double)snapshot->rate_error_dps,
        (double)snapshot->rate_p_term_tps,
        (double)snapshot->rate_i_term_tps,
        (double)snapshot->rate_d_term_tps,
        (double)snapshot->raw_velocity_command_tps,
        (double)snapshot->velocity_slew_error_tps,
        (double)snapshot->raw_acceleration_command_tps2,
        (double)snapshot->acceleration_limit_error_tps2,
        (double)snapshot->current_command_a,
        (double)snapshot->torque_command_nm,
        (double)tuning.rate_kp,
        (double)tuning.rate_ki,
        (double)tuning.rate_kd,
        (double)tuning.angle_kp,
        (double)tuning.angle_ki,
        (double)tuning.angle_kd,
        (double)tuning.wheel_kp,
        (double)tuning.wheel_ki,
        (unsigned long)snapshot->flags);

    if (length > 0)
    {
        const size_t output_length =
            ((size_t)length < sizeof(line)) ? (size_t)length : (sizeof(line) - 1u);
        (void)uart7_write(line, output_length);
    }

    if (!steering_header_sent)
    {
        static const char steering_header[] =
            "#S,time_ms,mode,yaw_deg,yaw_rate_dps,target_heading_deg,"
            "heading_error_deg,rear_wheel_tps,kp,ki,kd,integral_deg_s,"
            "raw_offset_us,commanded_offset_us,pulse_us,flags\r\n";
        steering_header_sent =
            uart7_write(steering_header, sizeof(steering_header) - 1u);
    }

    SteeringTelemetry steering;
    steering_get_telemetry(&steering);
    const int steering_length = snprintf(
        line,
        sizeof(line),
        "S,%lu,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,"
        "%.3f,%.3f,%u,0x%08lX\r\n",
        (unsigned long)steering.uptime_ms,
        (unsigned int)steering.mode,
        (double)steering.yaw_deg,
        (double)steering.yaw_rate_dps,
        (double)steering.target_heading_deg,
        (double)steering.heading_error_deg,
        (double)steering.rear_wheel_tps,
        (double)steering.kp,
        (double)steering.ki,
        (double)steering.kd,
        (double)steering.integral_deg_s,
        (double)steering.raw_offset_us,
        (double)steering.commanded_offset_us,
        (unsigned int)steering.pulse_us,
        (unsigned long)steering.flags);
    if (steering_length > 0)
    {
        const size_t output_length =
            ((size_t)steering_length < sizeof(line))
                ? (size_t)steering_length
                : (sizeof(line) - 1u);
        (void)uart7_write(line, output_length);
    }
}

void balance_telemetry_service(void)
{
    static uint32_t last_send_ms;
    static uint8_t steering_divider;
    static uint8_t basic_divider;
    BalanceTelemetry snapshot;
    const uint32_t now_ms = HAL_GetTick();

    uart7_tuning_service();

    if ((uint32_t)(now_ms - last_send_ms) < TELEMETRY_PERIOD_MS)
    {
        return;
    }
    last_send_ms = now_ms;

    balance_monitor_get_snapshot(&snapshot);
    basic_divider++;
    if (basic_divider >= BASIC_TELEMETRY_DIVIDER)
    {
        basic_divider = 0u;
        send_compact_basic(&snapshot);
    }
    if (telemetry_mode == BALANCE_TELEMETRY_BASIC)
    {
        return;
    }
    if (telemetry_mode == BALANCE_TELEMETRY_FULL_TEXT)
    {
        send_full_text_telemetry(&snapshot);
        return;
    }

    send_compact_balance(&snapshot);
    steering_divider++;
    if (steering_divider >= STEERING_TELEMETRY_DIVIDER)
    {
        steering_divider = 0u;
        send_compact_steering();
    }
}

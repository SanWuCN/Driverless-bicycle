#ifndef BALANCE_MONITOR_H
#define BALANCE_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

enum
{
    BALANCE_MONITOR_ZERO_GATE_ACTIVE = (1u << 0),
    BALANCE_MONITOR_ZERO_LEARNING = (1u << 1),
    BALANCE_MONITOR_WHEEL_BIAS_SATURATED = (1u << 2),
    BALANCE_MONITOR_VELOCITY_SATURATED = (1u << 3),
    BALANCE_MONITOR_ACCEL_SATURATED = (1u << 4),
    BALANCE_MONITOR_ZERO_APPLIED = (1u << 5),
    BALANCE_MONITOR_ZERO_LIMIT = (1u << 6),
    BALANCE_MONITOR_CONTROL_ARMED = (1u << 7),
    BALANCE_MONITOR_ARM_WINDOW = (1u << 8),
    BALANCE_MONITOR_IMU_READY = (1u << 9),
    BALANCE_MONITOR_ZERO_PERSIST_VALID = (1u << 10),
    BALANCE_MONITOR_ZERO_PERSIST_SAVED = (1u << 11),
    BALANCE_MONITOR_ZERO_PERSIST_ERROR = (1u << 12),
    BALANCE_MONITOR_RATE_SLEW_LIMITED = (1u << 13),
    BALANCE_MONITOR_ODRIVE_READY = (1u << 14),
    BALANCE_MONITOR_ODRIVE_TIMEOUT = (1u << 15),
    BALANCE_MONITOR_ODRIVE_FAULT = (1u << 16),
    BALANCE_MONITOR_UART7_RX_SEEN = (1u << 17),
    BALANCE_MONITOR_UART7_RX_OVERFLOW = (1u << 18),
    BALANCE_MONITOR_RATE_TARGET_SATURATED = (1u << 19),
    BALANCE_MONITOR_FALL_DISARM_LATCHED = (1u << 20),
    BALANCE_MONITOR_ZERO_STEERING_BLOCKED = (1u << 21),
    BALANCE_MONITOR_ACCEL_BOOST_120_ACTIVE = (1u << 22),
    BALANCE_MONITOR_SPEED_ENVELOPE_ACTIVE = (1u << 23),
    BALANCE_MONITOR_TORQUE_CONTROL_ACTIVE = (1u << 24)
};

typedef struct
{
    uint32_t sample_sequence;
    uint32_t uptime_ms;
    float roll_deg;
    float roll_rate_dps;
    float wheel_speed_tps;
    float velocity_command_tps;
    float acceleration_command_tps2;
    float base_zero_deg;
    float control_zero_deg;
    float zero_candidate_deg;
    float persisted_zero_deg;
    uint32_t persistence_sequence;
    float zero_adaptation_rate_dps;
    float wheel_bias_deg;
    float rate_target_dps;
    float rate_error_dps;
    float rate_p_term_tps;
    float rate_i_term_tps;
    float rate_d_term_tps;
    float raw_velocity_command_tps;
    float velocity_slew_error_tps;
    float raw_acceleration_command_tps2;
    float acceleration_limit_error_tps2;
    float current_command_a;
    float torque_command_nm;
    uint32_t flags;
} BalanceTelemetry;

typedef enum
{
    BALANCE_TELEMETRY_BASIC = 0,
    BALANCE_TELEMETRY_COMPACT = 1,
    BALANCE_TELEMETRY_FULL_TEXT = 2
} BalanceTelemetryMode;

extern volatile BalanceTelemetry g_balance_telemetry;

void balance_monitor_init(float range_center_zero_deg, float initial_zero_deg);
float balance_monitor_zero_for_control(void);
void balance_monitor_get_snapshot(BalanceTelemetry *snapshot);
void balance_telemetry_set_mode(BalanceTelemetryMode mode);
BalanceTelemetryMode balance_telemetry_get_mode(void);
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
                            bool torque_control_active);
void balance_telemetry_service(void);

#endif

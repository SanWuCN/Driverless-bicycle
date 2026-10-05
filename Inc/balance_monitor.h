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
    BALANCE_MONITOR_ZERO_PERSIST_ERROR = (1u << 12)
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
    uint32_t flags;
} BalanceTelemetry;

extern volatile BalanceTelemetry g_balance_telemetry;

void balance_monitor_init(float range_center_zero_deg, float initial_zero_deg);
float balance_monitor_zero_for_control(void);
void balance_monitor_get_snapshot(BalanceTelemetry *snapshot);
void balance_monitor_update(float roll_deg,
                            float roll_rate_dps,
                            float wheel_speed_tps,
                            float velocity_command_tps,
                            float base_zero_deg,
                            float wheel_bias_deg,
                            bool control_armed,
                            bool arm_window_active,
                            bool rear_wheel_stopped,
                            bool imu_ready);
void balance_telemetry_service(void);

#endif

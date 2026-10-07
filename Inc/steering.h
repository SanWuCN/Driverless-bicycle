#ifndef STEERING_H
#define STEERING_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    STEERING_MODE_DISABLED = 0,
    STEERING_MODE_CALIBRATION = 1,
    STEERING_MODE_HEADING_HOLD = 2,
    STEERING_MODE_ANGLE = 3
} SteeringMode;

enum
{
    STEERING_FLAG_OUTPUT_ACTIVE = (1u << 0),
    STEERING_FLAG_COMMAND_TIMEOUT = (1u << 1),
    STEERING_FLAG_SAFETY_GATED = (1u << 2),
    STEERING_FLAG_OUTPUT_SATURATED = (1u << 3),
    STEERING_FLAG_SLEW_LIMITED = (1u << 4),
    STEERING_FLAG_ROLL_DERATED = (1u << 5),
    STEERING_FLAG_BALANCE_ARMED = (1u << 6),
    STEERING_FLAG_IMU_READY = (1u << 7),
    STEERING_FLAG_REAR_FEEDBACK_READY = (1u << 8),
    STEERING_FLAG_BACKLASH_CENTERING = (1u << 9)
};

typedef struct
{
    uint32_t uptime_ms;
    SteeringMode mode;
    float yaw_deg;
    float yaw_rate_dps;
    float target_heading_deg;
    float heading_error_deg;
    float rear_wheel_tps;
    float kp;
    float ki;
    float kd;
    float integral_deg_s;
    float raw_offset_us;
    float commanded_offset_us;
    uint16_t pulse_us;
    uint32_t flags;
} SteeringTelemetry;

typedef enum
{
    STEERING_COMMAND_OK = 0,
    STEERING_COMMAND_BAD_VALUE,
    STEERING_COMMAND_UNSAFE_STATE
} SteeringCommandResult;

void steering_init(void);
void steering_update(float yaw_deg,
                     float yaw_rate_dps,
                     float roll_error_deg,
                     float rear_wheel_tps,
                     bool balance_armed,
                     bool imu_ready,
                     bool rear_feedback_ready,
                     bool fall_disarmed);
SteeringCommandResult steering_calibration_command(float offset_us);
SteeringCommandResult steering_angle_command(float offset_us);
SteeringCommandResult steering_heading_command(float heading_deg);
SteeringCommandResult steering_capture_heading_command(float current_heading_deg);
void steering_disable_command(void);
bool steering_keepalive_command(void);
void steering_get_telemetry(SteeringTelemetry *telemetry);
bool steering_allows_zero_learning(void);
float steering_balance_feedforward_deg(void);
bool steering_target_reached(float target_offset_us, float tolerance_us);

#endif

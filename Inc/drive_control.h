#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    DRIVE_MODE_STOPPED = 0,
    DRIVE_MODE_MANUAL = 1,
    DRIVE_MODE_DEMO = 2
} DriveMode;

typedef enum
{
    DRIVE_DEMO_IDLE = 0,
    DRIVE_DEMO_FORWARD_2M,
    DRIVE_DEMO_STOP_AFTER_2M,
    DRIVE_DEMO_STEER_RIGHT,
    DRIVE_DEMO_WAIT_RIGHT,
    DRIVE_DEMO_CENTER_AFTER_RIGHT,
    DRIVE_DEMO_STEER_LEFT,
    DRIVE_DEMO_WAIT_LEFT,
    DRIVE_DEMO_FORWARD_1M,
    DRIVE_DEMO_REVERSE_1M,
    DRIVE_DEMO_CENTER_BEFORE_RETURN,
    DRIVE_DEMO_REVERSE_2M
} DriveDemoPhase;

enum
{
    DRIVE_FLAG_OUTPUT_ACTIVE = (1u << 0),
    DRIVE_FLAG_COMMAND_TIMEOUT = (1u << 1),
    DRIVE_FLAG_SAFETY_GATED = (1u << 2),
    DRIVE_FLAG_BALANCE_ARMED = (1u << 3),
    DRIVE_FLAG_REAR_FEEDBACK_READY = (1u << 4),
    DRIVE_FLAG_DEMO_ACTIVE = (1u << 5),
    DRIVE_FLAG_SPEED_SLEW_LIMITED = (1u << 6),
    DRIVE_FLAG_POSITION_VALID = (1u << 7),
    DRIVE_FLAG_DIRECTION_MISMATCH = (1u << 8),
    DRIVE_FLAG_ENCODER_ODOMETRY = (1u << 9),
    DRIVE_FLAG_WHEEL_SLIP = (1u << 10),
    DRIVE_FLAG_IMU_ACCEL_READY = (1u << 11)
};

typedef struct
{
    uint32_t uptime_ms;
    DriveMode mode;
    DriveDemoPhase demo_phase;
    float requested_speed_mps;
    float output_speed_mps;
    float actual_speed_mps;
    float odometry_m;
    float segment_distance_m;
    uint16_t demo_cycle_count;
    uint32_t flags;
} DriveTelemetry;

typedef enum
{
    DRIVE_COMMAND_OK = 0,
    DRIVE_COMMAND_BAD_VALUE,
    DRIVE_COMMAND_BALANCE_NOT_ARMED,
    DRIVE_COMMAND_REAR_FEEDBACK_NOT_READY,
    DRIVE_COMMAND_FALL_DISARMED,
    DRIVE_COMMAND_ODOMETRY_NOT_READY,
    DRIVE_COMMAND_ALREADY_ACTIVE,
    DRIVE_COMMAND_REAR_WHEEL_MOVING
} DriveCommandResult;

void drive_control_init(void);
void drive_control_update(float rear_position_turns,
                          float rear_speed_tps,
                          uint32_t rear_feedback_ms,
                          float longitudinal_accel_g,
                          bool imu_accel_ready,
                          bool balance_armed,
                          bool rear_feedback_ready,
                          bool fall_disarmed);
DriveCommandResult drive_manual_command(float speed_mps);
DriveCommandResult drive_demo_command(float speed_mps);
void drive_stop_command(void);
bool drive_keepalive_command(void);
void drive_get_telemetry(DriveTelemetry *telemetry);

#endif

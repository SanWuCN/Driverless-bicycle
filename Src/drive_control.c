#include "drive_control.h"

#include "main.h"
#include "odrive.h"
#include "steering.h"
#include "task.h"

#include <math.h>
#include <string.h>

/* Historical bicycle calibration: one Axis-1 motor turn advances 0.077 m. */
#define DRIVE_METERS_PER_MOTOR_TURN       0.077f
#define DRIVE_MIN_SPEED_MPS               0.01f
#define DRIVE_MAX_SPEED_MPS               0.10f
#define DRIVE_DEFAULT_SPEED_MPS           0.03f
#define DRIVE_ACCEL_LIMIT_MPS2            0.025f
#define DRIVE_DECEL_LIMIT_MPS2             0.05f
#define DRIVE_JERK_LIMIT_MPS3               0.10f
#define DRIVE_SPEED_RESPONSE_S              0.40f
#define DRIVE_COMMAND_TIMEOUT_MS          3000u
#define DRIVE_UPDATE_MAX_ELAPSED_MS        20u
#define DRIVE_DISTANCE_TOLERANCE_M         0.03f
#define DRIVE_STOP_SPEED_MPS              0.005f
#define DRIVE_DIRECTION_ERROR_M            0.10f
#define DRIVE_DEMO_STEER_SETTLE_US          2.0f
#define DRIVE_DEMO_STEER_WAIT_MS           5000u
#define DRIVE_DEMO_STEER_REFRESH_MS         800u
#define DRIVE_GRAVITY_MPS2                 9.80665f
#define DRIVE_IMU_FORWARD_SIGN             1.0f
#define DRIVE_ACCEL_FILTER_TAU_S           0.08f
#define DRIVE_ACCEL_BIAS_TAU_S             2.00f
#define DRIVE_FUSION_ENCODER_GAIN          0.08f
#define DRIVE_FUSION_SLIP_GAIN             0.01f
#define DRIVE_SLIP_SPEED_ERROR_MPS        0.025f
#define DRIVE_SLIP_ACCEL_ERROR_MPS2        0.30f

static volatile DriveTelemetry drive_telemetry;
static DriveMode requested_mode;
static DriveDemoPhase demo_phase;
static float requested_speed_mps;
static float output_speed_mps;
static float output_acceleration_mps2;
static float fused_speed_mps;
static float fused_odometry_m;
static float segment_start_m;
static float segment_target_m;
static float segment_direction;
static float accel_bias_g;
static float filtered_accel_mps2;
static float previous_wheel_speed_mps;
static uint32_t previous_rear_feedback_ms;
static uint32_t last_update_ms;
static uint32_t last_command_ms;
static uint32_t phase_started_ms;
static uint32_t last_steering_refresh_ms;
static uint16_t demo_cycle_count;
static bool position_initialized;
static bool balance_ready_cached;
static bool rear_feedback_cached;
static bool imu_accel_ready_cached;
static bool fall_disarmed_cached;
static bool command_timeout_latched;
static bool direction_mismatch_latched;
static bool accel_bias_initialized;
static bool wheel_slip_active;

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

static float move_towards(float current, float target, float maximum_step)
{
    const float delta = target - current;
    if (delta > maximum_step)
    {
        return current + maximum_step;
    }
    if (delta < -maximum_step)
    {
        return current - maximum_step;
    }
    return target;
}

static float update_speed_profile(float target_speed_mps,
                                  float elapsed_s)
{
    if (elapsed_s <= 0.0f)
    {
        return output_speed_mps;
    }

    const float speed_error_mps = target_speed_mps - output_speed_mps;
    if (fabsf(speed_error_mps) < 0.0005f)
    {
        output_acceleration_mps2 = 0.0f;
        return target_speed_mps;
    }

    const bool increasing_magnitude =
        fabsf(target_speed_mps) > fabsf(output_speed_mps);
    const float acceleration_limit_mps2 = increasing_magnitude
                                              ? DRIVE_ACCEL_LIMIT_MPS2
                                              : DRIVE_DECEL_LIMIT_MPS2;
    const float desired_acceleration_mps2 = clamp_float(
        speed_error_mps / DRIVE_SPEED_RESPONSE_S,
        -acceleration_limit_mps2,
        acceleration_limit_mps2);
    output_acceleration_mps2 = move_towards(
        output_acceleration_mps2,
        desired_acceleration_mps2,
        DRIVE_JERK_LIMIT_MPS3 * elapsed_s);

    const float next_speed_mps = output_speed_mps +
                                 output_acceleration_mps2 * elapsed_s;
    if ((target_speed_mps - next_speed_mps) * speed_error_mps <= 0.0f)
    {
        output_acceleration_mps2 = 0.0f;
        return target_speed_mps;
    }
    return next_speed_mps;
}

static bool drive_safe(void)
{
    return balance_ready_cached && rear_feedback_cached &&
           !fall_disarmed_cached;
}

static void begin_distance_phase(DriveDemoPhase phase,
                                 float direction,
                                 float distance_m,
                                 uint32_t now_ms)
{
    demo_phase = phase;
    segment_start_m = fused_odometry_m;
    segment_direction = direction;
    segment_target_m = distance_m;
    phase_started_ms = now_ms;
}

static float segment_progress_m(void)
{
    return segment_direction * (fused_odometry_m - segment_start_m);
}

static bool steering_reached(float target_us)
{
    SteeringTelemetry steering;
    steering_get_telemetry(&steering);
    return fabsf(steering.commanded_offset_us - target_us) <=
               DRIVE_DEMO_STEER_SETTLE_US &&
           (steering.flags & STEERING_FLAG_SLEW_LIMITED) == 0u;
}

static void refresh_demo_steering(float target_us, uint32_t now_ms)
{
    if ((uint32_t)(now_ms - last_steering_refresh_ms) >=
        DRIVE_DEMO_STEER_REFRESH_MS)
    {
        (void)steering_angle_command(target_us);
        last_steering_refresh_ms = now_ms;
    }
}

static float distance_speed_target(void)
{
    const float progress_m = segment_progress_m();
    const float remaining_m = segment_target_m - progress_m;
    if (progress_m < -DRIVE_DIRECTION_ERROR_M)
    {
        direction_mismatch_latched = true;
        return 0.0f;
    }
    if (remaining_m <= DRIVE_DISTANCE_TOLERANCE_M)
    {
        return 0.0f;
    }

    /* Braking-distance profile prevents a full-speed command at the target. */
    const float braking_speed_mps =
        sqrtf(2.0f * DRIVE_DECEL_LIMIT_MPS2 * remaining_m);
    const float magnitude_mps = clamp_float(braking_speed_mps,
                                             DRIVE_MIN_SPEED_MPS,
                                             requested_speed_mps);
    return segment_direction * magnitude_mps;
}

static bool distance_phase_complete(float actual_speed_mps)
{
    return segment_progress_m() >=
               (segment_target_m - DRIVE_DISTANCE_TOLERANCE_M) &&
           fabsf(actual_speed_mps) <= DRIVE_STOP_SPEED_MPS;
}

static float demo_update(float actual_speed_mps,
                         uint32_t now_ms)
{
    switch (demo_phase)
    {
        case DRIVE_DEMO_FORWARD_2M:
            if (distance_phase_complete(actual_speed_mps))
            {
                demo_phase = DRIVE_DEMO_STOP_AFTER_2M;
                phase_started_ms = now_ms;
                return 0.0f;
            }
            return distance_speed_target();

        case DRIVE_DEMO_STOP_AFTER_2M:
            demo_phase = DRIVE_DEMO_STEER_RIGHT;
            phase_started_ms = now_ms;
            last_steering_refresh_ms = 0u;
            return 0.0f;

        case DRIVE_DEMO_STEER_RIGHT:
            refresh_demo_steering(-150.0f, now_ms);
            if (steering_reached(-150.0f))
            {
                demo_phase = DRIVE_DEMO_WAIT_RIGHT;
                phase_started_ms = now_ms;
            }
            return 0.0f;

        case DRIVE_DEMO_WAIT_RIGHT:
            refresh_demo_steering(-150.0f, now_ms);
            if ((uint32_t)(now_ms - phase_started_ms) >=
                DRIVE_DEMO_STEER_WAIT_MS)
            {
                demo_phase = DRIVE_DEMO_CENTER_AFTER_RIGHT;
                phase_started_ms = now_ms;
                last_steering_refresh_ms = 0u;
            }
            return 0.0f;

        case DRIVE_DEMO_CENTER_AFTER_RIGHT:
            refresh_demo_steering(0.0f, now_ms);
            if (steering_reached(0.0f))
            {
                demo_phase = DRIVE_DEMO_STEER_LEFT;
                phase_started_ms = now_ms;
                last_steering_refresh_ms = 0u;
            }
            return 0.0f;

        case DRIVE_DEMO_STEER_LEFT:
            refresh_demo_steering(150.0f, now_ms);
            if (steering_reached(150.0f))
            {
                demo_phase = DRIVE_DEMO_WAIT_LEFT;
                phase_started_ms = now_ms;
            }
            return 0.0f;

        case DRIVE_DEMO_WAIT_LEFT:
            refresh_demo_steering(150.0f, now_ms);
            if ((uint32_t)(now_ms - phase_started_ms) >=
                DRIVE_DEMO_STEER_WAIT_MS)
            {
                begin_distance_phase(DRIVE_DEMO_FORWARD_1M,
                                     1.0f,
                                     1.0f,
                                     now_ms);
            }
            return 0.0f;

        case DRIVE_DEMO_FORWARD_1M:
            refresh_demo_steering(150.0f, now_ms);
            if (distance_phase_complete(actual_speed_mps))
            {
                begin_distance_phase(DRIVE_DEMO_REVERSE_1M,
                                     -1.0f,
                                     1.0f,
                                     now_ms);
                return 0.0f;
            }
            return distance_speed_target();

        case DRIVE_DEMO_REVERSE_1M:
            refresh_demo_steering(150.0f, now_ms);
            if (distance_phase_complete(actual_speed_mps))
            {
                demo_phase = DRIVE_DEMO_CENTER_BEFORE_RETURN;
                phase_started_ms = now_ms;
                last_steering_refresh_ms = 0u;
                return 0.0f;
            }
            return distance_speed_target();

        case DRIVE_DEMO_CENTER_BEFORE_RETURN:
            refresh_demo_steering(0.0f, now_ms);
            if (steering_reached(0.0f))
            {
                begin_distance_phase(DRIVE_DEMO_REVERSE_2M,
                                     -1.0f,
                                     2.0f,
                                     now_ms);
            }
            return 0.0f;

        case DRIVE_DEMO_REVERSE_2M:
            refresh_demo_steering(0.0f, now_ms);
            if (distance_phase_complete(actual_speed_mps))
            {
                demo_cycle_count++;
                begin_distance_phase(DRIVE_DEMO_FORWARD_2M,
                                     1.0f,
                                     2.0f,
                                     now_ms);
                return 0.0f;
            }
            return distance_speed_target();

        default:
            return 0.0f;
    }
}

void drive_control_init(void)
{
    const uint32_t now_ms = HAL_GetTick();
    memset((void *)&drive_telemetry, 0, sizeof(drive_telemetry));
    requested_mode = DRIVE_MODE_STOPPED;
    demo_phase = DRIVE_DEMO_IDLE;
    requested_speed_mps = DRIVE_DEFAULT_SPEED_MPS;
    output_speed_mps = 0.0f;
    output_acceleration_mps2 = 0.0f;
    fused_speed_mps = 0.0f;
    fused_odometry_m = 0.0f;
    segment_start_m = 0.0f;
    segment_target_m = 0.0f;
    segment_direction = 0.0f;
    last_update_ms = now_ms;
    last_command_ms = now_ms;
    phase_started_ms = now_ms;
    last_steering_refresh_ms = 0u;
    demo_cycle_count = 0u;
    position_initialized = false;
    balance_ready_cached = false;
    rear_feedback_cached = false;
    imu_accel_ready_cached = false;
    fall_disarmed_cached = false;
    command_timeout_latched = false;
    direction_mismatch_latched = false;
    accel_bias_g = 0.0f;
    filtered_accel_mps2 = 0.0f;
    previous_wheel_speed_mps = 0.0f;
    previous_rear_feedback_ms = 0u;
    accel_bias_initialized = false;
    wheel_slip_active = false;
    odrive.set_speed1 = 0.0f;
    param.run_flag = 0;
}

void drive_control_update(float rear_position_turns,
                          float rear_speed_tps,
                          uint32_t rear_feedback_ms,
                          float longitudinal_accel_g,
                          bool imu_accel_ready,
                          bool balance_armed,
                          bool rear_feedback_ready,
                          bool fall_disarmed)
{
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t elapsed_ms = now_ms - last_update_ms;
    if (elapsed_ms == 0u)
    {
        return;
    }
    last_update_ms = now_ms;
    const float elapsed_s = elapsed_ms <= DRIVE_UPDATE_MAX_ELAPSED_MS
                                ? (float)elapsed_ms * 0.001f
                                : 0.0f;
    balance_ready_cached = balance_armed;
    rear_feedback_cached = rear_feedback_ready;
    imu_accel_ready_cached = imu_accel_ready;
    fall_disarmed_cached = fall_disarmed;

    const float wheel_speed_mps = rear_speed_tps *
                                  DRIVE_METERS_PER_MOTOR_TURN;
    if (rear_feedback_ready && isfinite(rear_position_turns) &&
        isfinite(wheel_speed_mps))
    {
        if (!position_initialized)
        {
            position_initialized = true;
            fused_speed_mps = wheel_speed_mps;
            previous_wheel_speed_mps = wheel_speed_mps;
            previous_rear_feedback_ms = rear_feedback_ms;
        }
    }

    const bool finite_acceleration =
        imu_accel_ready && isfinite(longitudinal_accel_g);
    if (finite_acceleration)
    {
        if (!accel_bias_initialized)
        {
            accel_bias_g = longitudinal_accel_g;
            accel_bias_initialized = true;
        }
        const bool stationary_for_bias =
            requested_mode == DRIVE_MODE_STOPPED &&
            fabsf(output_speed_mps) < 0.02f &&
            fabsf(wheel_speed_mps) < 0.03f;
        if (stationary_for_bias && elapsed_s > 0.0f)
        {
            const float bias_alpha = elapsed_s /
                                     (DRIVE_ACCEL_BIAS_TAU_S + elapsed_s);
            accel_bias_g += bias_alpha *
                            (longitudinal_accel_g - accel_bias_g);
        }
        const float linear_accel_mps2 = clamp_float(
            (longitudinal_accel_g - accel_bias_g) *
                DRIVE_GRAVITY_MPS2 * DRIVE_IMU_FORWARD_SIGN,
            -6.0f,
            6.0f);
        if (elapsed_s > 0.0f)
        {
            const float accel_alpha = elapsed_s /
                                      (DRIVE_ACCEL_FILTER_TAU_S + elapsed_s);
            filtered_accel_mps2 += accel_alpha *
                                   (linear_accel_mps2 -
                                    filtered_accel_mps2);
            fused_speed_mps += filtered_accel_mps2 * elapsed_s;
        }
    }
    else
    {
        filtered_accel_mps2 = 0.0f;
    }

    if (rear_feedback_ready && rear_feedback_ms != 0u &&
        rear_feedback_ms != previous_rear_feedback_ms)
    {
        wheel_slip_active = false;
        if (previous_rear_feedback_ms != 0u)
        {
            const uint32_t sample_elapsed_ms =
                rear_feedback_ms - previous_rear_feedback_ms;
            if (sample_elapsed_ms > 0u && sample_elapsed_ms <= 200u)
            {
                const float sample_elapsed_s =
                    (float)sample_elapsed_ms * 0.001f;
                const float wheel_accel_mps2 =
                    (wheel_speed_mps - previous_wheel_speed_mps) /
                    sample_elapsed_s;
                const float speed_error_mps =
                    fabsf(wheel_speed_mps - fused_speed_mps);
                const float accel_error_mps2 =
                    fabsf(wheel_accel_mps2 - filtered_accel_mps2);
                wheel_slip_active = finite_acceleration &&
                    fabsf(output_speed_mps) >= DRIVE_MIN_SPEED_MPS &&
                    (speed_error_mps >= DRIVE_SLIP_SPEED_ERROR_MPS ||
                     (fabsf(wheel_accel_mps2) >=
                          DRIVE_SLIP_ACCEL_ERROR_MPS2 &&
                      accel_error_mps2 >=
                          DRIVE_SLIP_ACCEL_ERROR_MPS2));
            }
        }
        const float encoder_gain = !finite_acceleration
                                       ? 1.0f
                                       : (wheel_slip_active
                                              ? DRIVE_FUSION_SLIP_GAIN
                                              : DRIVE_FUSION_ENCODER_GAIN);
        fused_speed_mps += encoder_gain *
                           (wheel_speed_mps - fused_speed_mps);
        previous_wheel_speed_mps = wheel_speed_mps;
        previous_rear_feedback_ms = rear_feedback_ms;
    }
    else if (!finite_acceleration && rear_feedback_ready)
    {
        fused_speed_mps = wheel_speed_mps;
    }

    fused_speed_mps = clamp_float(fused_speed_mps,
                                  -DRIVE_MAX_SPEED_MPS * 1.2f,
                                  DRIVE_MAX_SPEED_MPS * 1.2f);
    if (requested_mode == DRIVE_MODE_STOPPED &&
        fabsf(output_speed_mps) < 0.02f &&
        fabsf(wheel_speed_mps) < 0.03f)
    {
        fused_speed_mps = move_towards(fused_speed_mps,
                                       0.0f,
                                       0.5f * elapsed_s);
        if (fabsf(fused_speed_mps) < 0.005f)
        {
            fused_speed_mps = 0.0f;
        }
    }
    if (position_initialized && elapsed_s > 0.0f)
    {
        fused_odometry_m += fused_speed_mps * elapsed_s;
    }

    const bool command_fresh =
        requested_mode != DRIVE_MODE_STOPPED &&
        (uint32_t)(now_ms - last_command_ms) <= DRIVE_COMMAND_TIMEOUT_MS;
    const bool command_timed_out = !command_fresh &&
                                   requested_mode != DRIVE_MODE_STOPPED;
    if (command_timed_out)
    {
        requested_mode = DRIVE_MODE_STOPPED;
        demo_phase = DRIVE_DEMO_IDLE;
        command_timeout_latched = true;
        steering_disable_command();
    }
    const bool unsafe_drive = command_timed_out || !drive_safe() || direction_mismatch_latched ||
                              (requested_mode == DRIVE_MODE_DEMO && !finite_acceleration);
    if (unsafe_drive)
    {
        if (requested_mode == DRIVE_MODE_DEMO)
        {
            steering_disable_command();
        }
        requested_mode = DRIVE_MODE_STOPPED;
        demo_phase = DRIVE_DEMO_IDLE;
    }

    const float actual_speed_mps = fused_speed_mps;
    float target_speed_mps = 0.0f;
    if (requested_mode == DRIVE_MODE_MANUAL)
    {
        target_speed_mps = requested_speed_mps;
    }
    else if (requested_mode == DRIVE_MODE_DEMO && position_initialized)
    {
        target_speed_mps = demo_update(actual_speed_mps,
                                       now_ms);
    }

    const float previous_output_mps = output_speed_mps;
    if (unsafe_drive)
    {
        output_speed_mps = 0.0f;
        output_acceleration_mps2 = 0.0f;
    }
    else
    {
        output_speed_mps = update_speed_profile(target_speed_mps,
                                                elapsed_s);
    }
    if (fabsf(output_speed_mps) < 0.001f &&
        fabsf(target_speed_mps) < 0.001f)
    {
        output_speed_mps = 0.0f;
    }

    /* task.c negates set_speed1 before sending it to Axis 1. */
    odrive.set_speed1 = -output_speed_mps /
                       DRIVE_METERS_PER_MOTOR_TURN;
    param.run_flag = (requested_mode != DRIVE_MODE_STOPPED ||
                      fabsf(output_speed_mps) > 0.001f)
                         ? 1
                         : 0;

    uint32_t flags = 0u;
    if (fabsf(output_speed_mps) > 0.001f)
    {
        flags |= DRIVE_FLAG_OUTPUT_ACTIVE;
    }
    if (command_timeout_latched)
    {
        flags |= DRIVE_FLAG_COMMAND_TIMEOUT;
    }
    if (!drive_safe())
    {
        flags |= DRIVE_FLAG_SAFETY_GATED;
    }
    if (balance_armed)
    {
        flags |= DRIVE_FLAG_BALANCE_ARMED;
    }
    if (rear_feedback_ready)
    {
        flags |= DRIVE_FLAG_REAR_FEEDBACK_READY;
    }
    if (requested_mode == DRIVE_MODE_DEMO)
    {
        flags |= DRIVE_FLAG_DEMO_ACTIVE;
    }
    if (fabsf(previous_output_mps - target_speed_mps) > 0.0005f)
    {
        flags |= DRIVE_FLAG_SPEED_SLEW_LIMITED;
    }
    if (position_initialized)
    {
        flags |= DRIVE_FLAG_POSITION_VALID;
    }
    if (direction_mismatch_latched)
    {
        flags |= DRIVE_FLAG_DIRECTION_MISMATCH;
    }
    if (finite_acceleration)
    {
        flags |= DRIVE_FLAG_IMU_ACCEL_READY |
                 DRIVE_FLAG_IMU_FUSION_ACTIVE;
    }
    if (wheel_slip_active)
    {
        flags |= DRIVE_FLAG_WHEEL_SLIP;
    }

    drive_telemetry.uptime_ms = now_ms;
    drive_telemetry.mode = requested_mode;
    drive_telemetry.demo_phase = demo_phase;
    drive_telemetry.requested_speed_mps = requested_speed_mps;
    drive_telemetry.output_speed_mps = output_speed_mps;
    drive_telemetry.actual_speed_mps = actual_speed_mps;
    drive_telemetry.odometry_m = fused_odometry_m;
    drive_telemetry.segment_distance_m = position_initialized
                                            ? segment_progress_m()
                                            : 0.0f;
    drive_telemetry.demo_cycle_count = demo_cycle_count;
    drive_telemetry.flags = flags;
}

DriveCommandResult drive_manual_command(float speed_mps)
{
    if (!isfinite(speed_mps) || fabsf(speed_mps) < DRIVE_MIN_SPEED_MPS ||
        fabsf(speed_mps) > DRIVE_MAX_SPEED_MPS)
    {
        return DRIVE_COMMAND_BAD_VALUE;
    }
    if (!drive_safe())
    {
        return DRIVE_COMMAND_UNSAFE_STATE;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    requested_speed_mps = speed_mps;
    requested_mode = DRIVE_MODE_MANUAL;
    demo_phase = DRIVE_DEMO_IDLE;
    last_command_ms = HAL_GetTick();
    command_timeout_latched = false;
    direction_mismatch_latched = false;
    if (primask == 0u)
    {
        __enable_irq();
    }
    return DRIVE_COMMAND_OK;
}

DriveCommandResult drive_demo_command(float speed_mps)
{
    if (!isfinite(speed_mps) || speed_mps < DRIVE_MIN_SPEED_MPS ||
        speed_mps > DRIVE_MAX_SPEED_MPS)
    {
        return DRIVE_COMMAND_BAD_VALUE;
    }
    if (!drive_safe() || !position_initialized ||
        !imu_accel_ready_cached || !accel_bias_initialized ||
        requested_mode != DRIVE_MODE_STOPPED ||
        fabsf(output_speed_mps) > DRIVE_STOP_SPEED_MPS ||
        fabsf(fused_speed_mps) > DRIVE_STOP_SPEED_MPS)
    {
        return DRIVE_COMMAND_UNSAFE_STATE;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    requested_speed_mps = speed_mps;
    requested_mode = DRIVE_MODE_DEMO;
    begin_distance_phase(DRIVE_DEMO_FORWARD_2M,
                         1.0f,
                         2.0f,
                         HAL_GetTick());
    last_command_ms = HAL_GetTick();
    last_steering_refresh_ms = 0u;
    demo_cycle_count = 0u;
    command_timeout_latched = false;
    direction_mismatch_latched = false;
    if (primask == 0u)
    {
        __enable_irq();
    }
    return DRIVE_COMMAND_OK;
}

void drive_stop_command(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool was_demo = requested_mode == DRIVE_MODE_DEMO;
    requested_mode = DRIVE_MODE_STOPPED;
    demo_phase = DRIVE_DEMO_IDLE;
    requested_speed_mps = DRIVE_DEFAULT_SPEED_MPS;
    last_command_ms = HAL_GetTick();
    command_timeout_latched = false;
    direction_mismatch_latched = false;
    if (primask == 0u)
    {
        __enable_irq();
    }
    if (was_demo)
    {
        steering_disable_command();
    }
}

bool drive_keepalive_command(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool active = requested_mode != DRIVE_MODE_STOPPED;
    if (active)
    {
        last_command_ms = HAL_GetTick();
    }
    if (primask == 0u)
    {
        __enable_irq();
    }
    return active;
}

void drive_get_telemetry(DriveTelemetry *telemetry)
{
    if (telemetry == NULL)
    {
        return;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(telemetry, (const void *)&drive_telemetry, sizeof(*telemetry));
    if (primask == 0u)
    {
        __enable_irq();
    }
}

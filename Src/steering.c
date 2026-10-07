#include "steering.h"

#include "main.h"
#include "servo.h"
#include "task.h"

#include <math.h>
#include <string.h>

#define STEERING_UPDATE_PERIOD_MS          20u
#define STEERING_COMMAND_TIMEOUT_MS      3000u
#define STEERING_CALIBRATION_MIN_US    -150.0f
#define STEERING_CALIBRATION_MAX_US     150.0f
#define STEERING_ANGLE_MIN_US          -150.0f
#define STEERING_ANGLE_MAX_US           150.0f
#define STEERING_ANGLE_DEADBAND_US       50.0f
#define STEERING_ANGLE_STEP_US            5.0f
#define STEERING_OUTPUT_LIMIT_US        150.0f
#define STEERING_MAX_RATE_US_PER_S        20.0f
#define STEERING_MAX_ACCEL_US_PER_S2      40.0f
#define STEERING_CENTER_OVERSHOOT_US      40.0f
#define STEERING_CENTER_DWELL_MS         300u
#define STEERING_RIGHT_BALANCE_FF_DEG_PER_US (-0.0039f)
#define STEERING_LEFT_BALANCE_FF_DEG_PER_US  (-0.0034f)
#define STEERING_INTEGRAL_LIMIT_DEG_S    20.0f
#define STEERING_MIN_REAR_SPEED_TPS       0.20f
#define STEERING_ROLL_DERATE_START_DEG    5.0f
#define STEERING_ROLL_DERATE_END_DEG      8.0f
#define STEERING_ZERO_NEUTRAL_WINDOW_US    1.0f
#define STEERING_ZERO_NEUTRAL_RATE_US_S    0.5f
#define STEERING_ZERO_SETTLE_MS          5000u

static volatile SteeringTelemetry steering_telemetry;
static SteeringMode requested_mode;
static float target_heading_deg;
static float calibration_offset_us;
static float angle_offset_us;
static float commanded_offset_us;
static float commanded_velocity_us_per_s;
static float heading_integral_deg_s;
static uint32_t last_update_ms;
static uint32_t last_command_ms;
static uint32_t last_non_neutral_ms;
static bool output_started;
static bool command_timeout_latched;

typedef enum
{
    STEERING_CENTER_IDLE = 0,
    STEERING_CENTER_OVERSHOOT,
    STEERING_CENTER_DWELL,
    STEERING_CENTER_FINAL
} SteeringCenterState;

static SteeringCenterState center_state;
static float center_overshoot_target_us;
static uint32_t center_dwell_start_ms;
static bool center_return_requested;
static float center_return_sign;

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

static float wrap_heading_deg(float angle_deg)
{
    angle_deg = fmodf(angle_deg + 180.0f, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg - 180.0f;
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

static void update_command_profile(float target_offset_us, float elapsed_s)
{
    if (elapsed_s <= 0.0f)
    {
        return;
    }

    const float error_us = target_offset_us - commanded_offset_us;
    if (fabsf(error_us) < 0.01f &&
        fabsf(commanded_velocity_us_per_s) < 0.01f)
    {
        commanded_offset_us = target_offset_us;
        commanded_velocity_us_per_s = 0.0f;
        return;
    }

    /*
     * Trapezoidal position profile with braking-distance lookahead.  The
     * previous implementation jumped a fixed 4 us every 20 ms.  On a
     * linkage with static friction those discrete jumps become stick-slip.
     * This profile ramps velocity up and down and reaches the target without
     * commanding an abrupt start, stop, or reversal.
     */
    const float direction = (error_us >= 0.0f) ? 1.0f : -1.0f;
    const float braking_speed_us_per_s =
        sqrtf(2.0f * STEERING_MAX_ACCEL_US_PER_S2 * fabsf(error_us));
    const float desired_speed_us_per_s =
        direction * fminf(STEERING_MAX_RATE_US_PER_S,
                          braking_speed_us_per_s);
    commanded_velocity_us_per_s = move_towards(
        commanded_velocity_us_per_s,
        desired_speed_us_per_s,
        STEERING_MAX_ACCEL_US_PER_S2 * elapsed_s);

    float next_offset_us =
        commanded_offset_us + commanded_velocity_us_per_s * elapsed_s;
    if ((error_us > 0.0f && next_offset_us >= target_offset_us) ||
        (error_us < 0.0f && next_offset_us <= target_offset_us))
    {
        next_offset_us = target_offset_us;
        commanded_velocity_us_per_s = 0.0f;
    }
    commanded_offset_us = next_offset_us;
}

static void cancel_center_return(void)
{
    center_state = STEERING_CENTER_IDLE;
    center_overshoot_target_us = 0.0f;
    center_dwell_start_ms = 0u;
    center_return_requested = false;
}

static void request_center_return(float reference_offset_us)
{
    if (fabsf(reference_offset_us) < 0.5f)
    {
        reference_offset_us = commanded_offset_us;
    }
    if (fabsf(reference_offset_us) < 0.5f)
    {
        cancel_center_return();
        return;
    }
    center_return_sign = (reference_offset_us >= 0.0f) ? 1.0f : -1.0f;
    center_return_requested = true;
}

static float center_profile_target(uint32_t now_ms, bool center_sequence_safe)
{
    if (!center_sequence_safe)
    {
        cancel_center_return();
        return 0.0f;
    }

    if (center_state == STEERING_CENTER_IDLE)
    {
        if (!center_return_requested)
        {
            return 0.0f;
        }
        center_overshoot_target_us =
            -center_return_sign * STEERING_CENTER_OVERSHOOT_US;
        center_state = STEERING_CENTER_OVERSHOOT;
    }

    if (center_state == STEERING_CENTER_OVERSHOOT)
    {
        if (fabsf(commanded_offset_us - center_overshoot_target_us) < 0.05f &&
            fabsf(commanded_velocity_us_per_s) < 0.05f)
        {
            center_state = STEERING_CENTER_DWELL;
            center_dwell_start_ms = now_ms;
        }
        return center_overshoot_target_us;
    }

    if (center_state == STEERING_CENTER_DWELL)
    {
        if ((uint32_t)(now_ms - center_dwell_start_ms) <
            STEERING_CENTER_DWELL_MS)
        {
            return center_overshoot_target_us;
        }
        center_state = STEERING_CENTER_FINAL;
    }

    if (fabsf(commanded_offset_us) < 0.05f &&
        fabsf(commanded_velocity_us_per_s) < 0.05f)
    {
        cancel_center_return();
    }
    return 0.0f;
}

void steering_init(void)
{
    const uint32_t now_ms = HAL_GetTick();
    memset((void *)&steering_telemetry, 0, sizeof(steering_telemetry));
    requested_mode = STEERING_MODE_DISABLED;
    target_heading_deg = 0.0f;
    calibration_offset_us = 0.0f;
    angle_offset_us = 0.0f;
    commanded_offset_us = 0.0f;
    commanded_velocity_us_per_s = 0.0f;
    heading_integral_deg_s = 0.0f;
    last_update_ms = now_ms;
    last_command_ms = now_ms;
    last_non_neutral_ms = now_ms;
    output_started = false;
    command_timeout_latched = false;
    center_state = STEERING_CENTER_IDLE;
    center_overshoot_target_us = 0.0f;
    center_dwell_start_ms = 0u;
    center_return_requested = false;
    center_return_sign = 0.0f;
    servo_disable();
}

void steering_update(float yaw_deg,
                     float yaw_rate_dps,
                     float roll_error_deg,
                     float rear_wheel_tps,
                     bool balance_armed,
                     bool imu_ready,
                     bool rear_feedback_ready,
                     bool fall_disarmed)
{
    const uint32_t now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - last_update_ms) < STEERING_UPDATE_PERIOD_MS)
    {
        return;
    }
    const uint32_t elapsed_ms = now_ms - last_update_ms;
    last_update_ms = now_ms;
    const float elapsed_s = (elapsed_ms <= 100u)
                                ? ((float)elapsed_ms * 0.001f)
                                : 0.0f;
    uint32_t flags = 0u;
    float raw_offset_us = 0.0f;
    bool center_sequence_safe = false;
    const bool finite_imu = isfinite(yaw_deg) && isfinite(yaw_rate_dps) &&
                            isfinite(roll_error_deg);
    const bool command_fresh =
        (requested_mode != STEERING_MODE_DISABLED) &&
        ((uint32_t)(now_ms - last_command_ms) <= STEERING_COMMAND_TIMEOUT_MS);

    if (balance_armed)
    {
        flags |= STEERING_FLAG_BALANCE_ARMED;
    }
    if (imu_ready && finite_imu)
    {
        flags |= STEERING_FLAG_IMU_READY;
    }
    if (rear_feedback_ready)
    {
        flags |= STEERING_FLAG_REAR_FEEDBACK_READY;
    }

    if (!command_fresh && requested_mode != STEERING_MODE_DISABLED)
    {
        if (requested_mode == STEERING_MODE_ANGLE)
        {
            request_center_return(angle_offset_us);
            angle_offset_us = 0.0f;
        }
        requested_mode = STEERING_MODE_DISABLED;
        heading_integral_deg_s = 0.0f;
        command_timeout_latched = true;
    }
    if (command_timeout_latched)
    {
        flags |= STEERING_FLAG_COMMAND_TIMEOUT;
    }

    if (requested_mode == STEERING_MODE_CALIBRATION && command_fresh)
    {
        raw_offset_us = calibration_offset_us;
        steering_telemetry.heading_error_deg = 0.0f;
    }
    else if (requested_mode == STEERING_MODE_ANGLE && command_fresh)
    {
        const bool controller_ready = balance_armed &&
                                      imu_ready &&
                                      finite_imu &&
                                      !fall_disarmed;
        if (controller_ready)
        {
            float roll_scale = 1.0f;
            const float absolute_roll_error = fabsf(roll_error_deg);
            if (absolute_roll_error >= STEERING_ROLL_DERATE_END_DEG)
            {
                roll_scale = 0.0f;
            }
            else if (absolute_roll_error > STEERING_ROLL_DERATE_START_DEG)
            {
                roll_scale =
                    (STEERING_ROLL_DERATE_END_DEG - absolute_roll_error) /
                    (STEERING_ROLL_DERATE_END_DEG -
                     STEERING_ROLL_DERATE_START_DEG);
            }
            if (roll_scale < 0.999f)
            {
                flags |= STEERING_FLAG_ROLL_DERATED;
            }
            raw_offset_us = angle_offset_us * roll_scale;
            center_sequence_safe = roll_scale > 0.0f;
            steering_telemetry.heading_error_deg = 0.0f;
        }
        else
        {
            cancel_center_return();
            steering_telemetry.heading_error_deg = 0.0f;
            flags |= STEERING_FLAG_SAFETY_GATED;
        }
    }
    else if (requested_mode == STEERING_MODE_HEADING_HOLD && command_fresh)
    {
        const bool controller_ready = balance_armed &&
                                      imu_ready &&
                                      finite_imu &&
                                      rear_feedback_ready &&
                                      !fall_disarmed &&
                                      (param.run_flag != 0) &&
                                      (fabsf(rear_wheel_tps) >=
                                       STEERING_MIN_REAR_SPEED_TPS);
        if (controller_ready)
        {
            const float heading_error_deg =
                wrap_heading_deg(target_heading_deg - yaw_deg);
            float roll_scale = 1.0f;
            const float absolute_roll_error = fabsf(roll_error_deg);
            if (absolute_roll_error >= STEERING_ROLL_DERATE_END_DEG)
            {
                roll_scale = 0.0f;
            }
            else if (absolute_roll_error > STEERING_ROLL_DERATE_START_DEG)
            {
                roll_scale =
                    (STEERING_ROLL_DERATE_END_DEG - absolute_roll_error) /
                    (STEERING_ROLL_DERATE_END_DEG -
                     STEERING_ROLL_DERATE_START_DEG);
            }
            if (roll_scale < 0.999f)
            {
                flags |= STEERING_FLAG_ROLL_DERATED;
            }

            const float candidate_integral = clamp_float(
                heading_integral_deg_s + heading_error_deg * elapsed_s,
                -STEERING_INTEGRAL_LIMIT_DEG_S,
                STEERING_INTEGRAL_LIMIT_DEG_S);
            const float candidate_output =
                param.Steer_Kp * heading_error_deg +
                param.Steer_Ki * candidate_integral -
                param.Steer_Kd * yaw_rate_dps;
            const float limited_output = clamp_float(
                candidate_output,
                -STEERING_OUTPUT_LIMIT_US,
                STEERING_OUTPUT_LIMIT_US);
            const float integral_effect =
                param.Steer_Ki * heading_error_deg * elapsed_s;
            const bool allow_integrator =
                fabsf(candidate_output) <= STEERING_OUTPUT_LIMIT_US ||
                (candidate_output > STEERING_OUTPUT_LIMIT_US &&
                 integral_effect < 0.0f) ||
                (candidate_output < -STEERING_OUTPUT_LIMIT_US &&
                 integral_effect > 0.0f);
            if (allow_integrator)
            {
                heading_integral_deg_s = candidate_integral;
            }
            raw_offset_us = limited_output * roll_scale;
            steering_telemetry.heading_error_deg = heading_error_deg;
            if (fabsf(candidate_output - limited_output) > 0.01f)
            {
                flags |= STEERING_FLAG_OUTPUT_SATURATED;
            }
        }
        else
        {
            heading_integral_deg_s = 0.0f;
            steering_telemetry.heading_error_deg = 0.0f;
            flags |= STEERING_FLAG_SAFETY_GATED;
        }
    }
    else
    {
        heading_integral_deg_s = 0.0f;
        steering_telemetry.heading_error_deg = 0.0f;
    }

    if (output_started)
    {
        float profile_target_us = raw_offset_us;
        if (center_return_requested || center_state != STEERING_CENTER_IDLE)
        {
            profile_target_us = center_profile_target(
                now_ms,
                center_sequence_safe ||
                    (requested_mode == STEERING_MODE_DISABLED &&
                     balance_armed && imu_ready && finite_imu &&
                     !fall_disarmed));
            if (center_state != STEERING_CENTER_IDLE)
            {
                flags |= STEERING_FLAG_BACKLASH_CENTERING;
            }
        }
        update_command_profile(profile_target_us, elapsed_s);
        if (fabsf(commanded_offset_us - profile_target_us) > 0.01f)
        {
            flags |= STEERING_FLAG_SLEW_LIMITED;
        }
        servo_set_pulse_us((int)lroundf((float)SERVO_CENTER_PULSE_US +
                                       commanded_offset_us));
        flags |= STEERING_FLAG_OUTPUT_ACTIVE;
    }

    /* Steering load must never be learned as IMU zero drift. */
    if (requested_mode != STEERING_MODE_DISABLED ||
        fabsf(commanded_offset_us) > STEERING_ZERO_NEUTRAL_WINDOW_US ||
        fabsf(commanded_velocity_us_per_s) >
            STEERING_ZERO_NEUTRAL_RATE_US_S)
    {
        last_non_neutral_ms = now_ms;
    }

    steering_telemetry.uptime_ms = now_ms;
    steering_telemetry.mode = requested_mode;
    steering_telemetry.yaw_deg = yaw_deg;
    steering_telemetry.yaw_rate_dps = yaw_rate_dps;
    steering_telemetry.target_heading_deg = target_heading_deg;
    steering_telemetry.rear_wheel_tps = rear_wheel_tps;
    steering_telemetry.kp = param.Steer_Kp;
    steering_telemetry.ki = param.Steer_Ki;
    steering_telemetry.kd = param.Steer_Kd;
    steering_telemetry.integral_deg_s = heading_integral_deg_s;
    steering_telemetry.raw_offset_us = raw_offset_us;
    steering_telemetry.commanded_offset_us = commanded_offset_us;
    steering_telemetry.pulse_us = output_started
                                      ? (uint16_t)servo_current_pulse_us()
                                      : 0u;
    steering_telemetry.flags = flags;
}

SteeringCommandResult steering_calibration_command(float offset_us)
{
    if (!isfinite(offset_us) ||
        offset_us < STEERING_CALIBRATION_MIN_US ||
        offset_us > STEERING_CALIBRATION_MAX_US)
    {
        return STEERING_COMMAND_BAD_VALUE;
    }
    if (param.run_flag != 0)
    {
        return STEERING_COMMAND_UNSAFE_STATE;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    cancel_center_return();
    angle_offset_us = 0.0f;
    calibration_offset_us = offset_us;
    requested_mode = STEERING_MODE_CALIBRATION;
    last_command_ms = HAL_GetTick();
    output_started = true;
    heading_integral_deg_s = 0.0f;
    command_timeout_latched = false;
    if (primask == 0u)
    {
        __enable_irq();
    }
    return STEERING_COMMAND_OK;
}

SteeringCommandResult steering_angle_command(float offset_us)
{
    const float absolute_offset_us = fabsf(offset_us);
    const float quantized_offset_us =
        roundf(offset_us / STEERING_ANGLE_STEP_US) * STEERING_ANGLE_STEP_US;
    if (!isfinite(offset_us) ||
        offset_us < STEERING_ANGLE_MIN_US ||
        offset_us > STEERING_ANGLE_MAX_US ||
        (absolute_offset_us > 0.5f &&
         absolute_offset_us < STEERING_ANGLE_DEADBAND_US) ||
        fabsf(offset_us - quantized_offset_us) > 0.05f)
    {
        return STEERING_COMMAND_BAD_VALUE;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (fabsf(offset_us) < 0.5f)
    {
        request_center_return(
            (fabsf(angle_offset_us) >= 0.5f)
                ? angle_offset_us
                : commanded_offset_us);
    }
    else
    {
        cancel_center_return();
    }
    angle_offset_us = offset_us;
    requested_mode = STEERING_MODE_ANGLE;
    last_command_ms = HAL_GetTick();
    output_started = true;
    heading_integral_deg_s = 0.0f;
    command_timeout_latched = false;
    if (primask == 0u)
    {
        __enable_irq();
    }
    return STEERING_COMMAND_OK;
}

SteeringCommandResult steering_heading_command(float heading_deg)
{
    if (!isfinite(heading_deg))
    {
        return STEERING_COMMAND_BAD_VALUE;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    cancel_center_return();
    angle_offset_us = 0.0f;
    target_heading_deg = wrap_heading_deg(heading_deg);
    requested_mode = STEERING_MODE_HEADING_HOLD;
    last_command_ms = HAL_GetTick();
    output_started = true;
    command_timeout_latched = false;
    if (primask == 0u)
    {
        __enable_irq();
    }
    return STEERING_COMMAND_OK;
}

SteeringCommandResult steering_capture_heading_command(float current_heading_deg)
{
    return steering_heading_command(current_heading_deg);
}

void steering_disable_command(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (requested_mode == STEERING_MODE_ANGLE)
    {
        request_center_return(angle_offset_us);
        angle_offset_us = 0.0f;
    }
    requested_mode = STEERING_MODE_DISABLED;
    heading_integral_deg_s = 0.0f;
    last_command_ms = HAL_GetTick();
    command_timeout_latched = false;
    if (primask == 0u)
    {
        __enable_irq();
    }
}

bool steering_keepalive_command(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool active = requested_mode != STEERING_MODE_DISABLED;
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

void steering_get_telemetry(SteeringTelemetry *telemetry)
{
    if (telemetry == NULL)
    {
        return;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(telemetry,
           (const void *)&steering_telemetry,
           sizeof(*telemetry));
    if (primask == 0u)
    {
        __enable_irq();
    }
}

bool steering_allows_zero_learning(void)
{
    const uint32_t now_ms = HAL_GetTick();
    if (requested_mode != STEERING_MODE_DISABLED ||
        fabsf(commanded_offset_us) > STEERING_ZERO_NEUTRAL_WINDOW_US ||
        fabsf(commanded_velocity_us_per_s) >
            STEERING_ZERO_NEUTRAL_RATE_US_S)
    {
        return false;
    }
    return (uint32_t)(now_ms - last_non_neutral_ms) >=
           STEERING_ZERO_SETTLE_MS;
}

float steering_balance_feedforward_deg(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const float offset_us = output_started ? commanded_offset_us : 0.0f;
    if (primask == 0u)
    {
        __enable_irq();
    }

    /*
     * Identified from the static +/-100 us and right -150 us hold tests.
     * The linkage/load is asymmetric, so preserve separate left/right gains.
     * Negative PWM offset is physical right steering on the current bicycle.
     */
    return offset_us < 0.0f
               ? STEERING_RIGHT_BALANCE_FF_DEG_PER_US * offset_us
               : STEERING_LEFT_BALANCE_FF_DEG_PER_US * offset_us;
}

bool steering_target_reached(float target_offset_us, float tolerance_us)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool reached =
        fabsf(commanded_offset_us - target_offset_us) <= tolerance_us &&
        fabsf(commanded_velocity_us_per_s) < 0.05f;
    if (primask == 0u)
    {
        __enable_irq();
    }
    return reached;
}

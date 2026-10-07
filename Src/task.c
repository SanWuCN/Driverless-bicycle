#include "task.h"
#include "tim.h"
#include "imu.h"
#include "imu_data_decode.h"
#include "odrive.h"
#include "servo.h"
#include "upper.h"
#include "balance_monitor.h"
#include "packet.h"
#include "zero_persistence.h"
#include "steering.h"
#include "drive_control.h"
#include "math.h"
#include "string.h"

#define  FS 3
#if FS==1
    float fast_rate=8.0f,slow_rate=3.0f,mid_rate=5.0f;
#elif FS==2
    float fast_rate=10.0f,slow_rate=4.50f,mid_rate=6.50f;
#else
    float fast_rate=12.0f,slow_rate=5.50f,mid_rate=6.50f;
#endif

#define fly_wheel_rate_limit 40
#define dt 0.100f
#define PI 3.1415926f
#define BALANCE_ARM_ROLL_WINDOW_DEG 3.0f
#define BALANCE_ARM_RATE_WINDOW_DPS 1.0f
#define BALANCE_ARM_HOLD_MS 2000u
#define BALANCE_ARM_IMU_MIN_FRAMES 5u
#define BALANCE_ARM_IMU_FRESH_MS 100u
#define BALANCE_FALL_DISARM_ANGLE_DEG 8.0f
#define BALANCE_FACTORY_ZERO_DEG (-2.18f)
#define BALANCE_ZERO_RANGE_DEG 1.50f
#define BALANCE_ODRIVE_SPEED_LIMIT_TPS 35.0f
#define BALANCE_WHEEL_SPEED_ENVELOPE_START_TPS 25.0f
#define BALANCE_ODRIVE_ACCEL_LIMIT_TPS2 80.0f
#define BALANCE_ODRIVE_ACCEL_BOOST_LIMIT_TPS2 120.0f
#define BALANCE_ACCEL_BOOST_MIN_ROLL_DEG 0.60f
#define BALANCE_ACCEL_BOOST_MIN_RATE_DPS 0.50f
#define BALANCE_RATE_TARGET_LIMIT_DPS 5.0f
#define BALANCE_RATE_LIMIT_MAX_ELAPSED_MS 10u
#define BALANCE_ODRIVE_FEEDBACK_TIMEOUT_MS 300u
#define BALANCE_ODRIVE_STATE_REQUEST_PERIOD_MS 100u
#define BALANCE_CONTROL_DT_S 0.002f
#define BALANCE_WHEEL_CONTROL_DT_S 0.040f
#define BALANCE_RATE_INTEGRAL_LIMIT_DPS_S 5.0f
#define BALANCE_ANGLE_INTEGRAL_LIMIT_DEG_S 2.0f
#define BALANCE_WHEEL_INTEGRAL_LIMIT_TPS_S 100.0f
#define BALANCE_WHEEL_BIAS_LIMIT_DEG 1.5f
#define BALANCE_DEFAULT_RATE_KP (-20.0f)
#define BALANCE_DEFAULT_RATE_KI 0.0f
#define BALANCE_DEFAULT_RATE_KD (-0.004f)
#define BALANCE_DEFAULT_ANGLE_KP (-5.0f)
#define BALANCE_DEFAULT_ANGLE_KI 0.0f
#define BALANCE_DEFAULT_ANGLE_KD (-1.5f)
#define BALANCE_DEFAULT_WHEEL_KP 0.06f
#define BALANCE_DEFAULT_WHEEL_KI 0.003f
#define BALANCE_DEFAULT_STEER_KP 1.5f
#define BALANCE_DEFAULT_STEER_KI 0.0f
#define BALANCE_DEFAULT_STEER_KD 0.2f

paramTypeDef param;
float PWM_X,PWM_accel,PWM_Final;// PWM中间量
extern int key_times;
int cnt;//角度环计数
int cnt1;//速度环计数
int cnt_vel_callback1;//飞轮速度反馈计数
int cnt_vel_set1;//飞轮速度发送计数
int cnt_balance;//自行车平衡控制周期计数
int cnt_rate;//速度设置计数
float rate;//死区外飞轮速度
float start_yaw0;//开始积分时的偏航角
float last_rate=0;//记录上一时刻的速度

// 【修改点1】将PID积分项移至全局，方便倒地时清零
float Angle_Velocity_Last_Bias = 0;
float Angle_Velocity_Integral = 0;
float X_balance_error = 0;
float Velocity_encoder_bias_integral = 0;
static uint32_t balance_arm_start_ms;
static uint32_t balance_last_imu_frame_count;
static uint32_t balance_last_imu_frame_ms;
static uint32_t drive_last_accel_frame_count;
static uint32_t drive_last_accel_frame_ms;
static bool balance_arm_window;
static bool balance_imu_ready;
static bool balance_odrive_ready;
static bool balance_odrive_timeout;
static bool balance_odrive_fault;
static bool balance_fall_disarm_latched;
static bool angle_velocity_initialized;
static uint32_t balance_last_odrive_state_request_ms;
static float rate_target_dps;
static float rate_error_dps;
static float rate_p_term_tps;
static float rate_i_term_tps;
static float rate_d_term_tps;
static float rate_raw_velocity_tps;
static float rate_raw_acceleration_tps2;
static float rate_limited_acceleration_tps2;
static float rate_limited_velocity_tps;
static float rate_current_command_a;
static float rate_torque_command_nm;
static bool rate_acceleration_boost_active;
static bool rate_speed_envelope_active;
static uint32_t rate_limiter_previous_ms;
static bool rate_limiter_initialized;
static float wheel_feedback_bias_deg;

static float balance_clamp(float value, float lower, float upper)
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

static float balance_select_acceleration_limit_tps2(
    float roll_error_deg,
    float roll_rate_dps,
    float raw_acceleration_tps2)
{
    const bool finite_inputs = isfinite(roll_error_deg) &&
                               isfinite(roll_rate_dps) &&
                               isfinite(raw_acceleration_tps2);
    const bool large_roll_error =
        fabsf(roll_error_deg) >= BALANCE_ACCEL_BOOST_MIN_ROLL_DEG;
    const bool meaningful_roll_rate =
        fabsf(roll_rate_dps) >= BALANCE_ACCEL_BOOST_MIN_RATE_DPS;
    const bool falling_away_from_zero =
        (roll_error_deg * roll_rate_dps) > 0.0f;
    const bool corrective_output_direction =
        (roll_error_deg * raw_acceleration_tps2) > 0.0f;
    const bool base_limit_is_insufficient =
        fabsf(raw_acceleration_tps2) > BALANCE_ODRIVE_ACCEL_LIMIT_TPS2;

    rate_acceleration_boost_active = finite_inputs &&
                                     large_roll_error &&
                                     meaningful_roll_rate &&
                                     falling_away_from_zero &&
                                     corrective_output_direction &&
                                     base_limit_is_insufficient;
    return rate_acceleration_boost_active
               ? BALANCE_ODRIVE_ACCEL_BOOST_LIMIT_TPS2
               : BALANCE_ODRIVE_ACCEL_LIMIT_TPS2;
}

static float balance_integrate_acceleration_command(
    float raw_acceleration_tps2,
    float acceleration_limit_tps2,
    float wheel_speed_tps)
{
    const uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = 1u;

    if (!isfinite(raw_acceleration_tps2))
    {
        rate_raw_acceleration_tps2 = 0.0f;
        rate_limited_acceleration_tps2 = 0.0f;
        rate_raw_velocity_tps = 0.0f;
        rate_limited_velocity_tps = 0.0f;
        rate_current_command_a = 0.0f;
        rate_torque_command_nm = 0.0f;
        rate_speed_envelope_active = false;
        rate_limiter_previous_ms = now_ms;
        rate_limiter_initialized = true;
        return 0.0f;
    }

    if (rate_limiter_initialized)
    {
        elapsed_ms = now_ms - rate_limiter_previous_ms;
        if (elapsed_ms == 0u)
        {
            return rate_limited_velocity_tps;
        }
        if (elapsed_ms > BALANCE_RATE_LIMIT_MAX_ELAPSED_MS)
        {
            elapsed_ms = BALANCE_RATE_LIMIT_MAX_ELAPSED_MS;
        }
    }
    rate_limiter_previous_ms = now_ms;
    rate_limiter_initialized = true;

    rate_raw_acceleration_tps2 = raw_acceleration_tps2;
    rate_limited_acceleration_tps2 = balance_clamp(
        raw_acceleration_tps2,
        -acceleration_limit_tps2,
        acceleration_limit_tps2);

    /*
     * Preserve reaction-wheel speed headroom without weakening recovery
     * torque. Between 25 and 35 tps, only acceleration that would increase
     * measured wheel-speed magnitude is tapered to zero. Braking/reversing
     * acceleration always retains full authority.
     */
    rate_speed_envelope_active = false;
    if (isfinite(wheel_speed_tps) &&
        (wheel_speed_tps * rate_limited_acceleration_tps2) > 0.0f &&
        fabsf(wheel_speed_tps) > BALANCE_WHEEL_SPEED_ENVELOPE_START_TPS)
    {
        const float envelope_width_tps =
            BALANCE_ODRIVE_SPEED_LIMIT_TPS -
            BALANCE_WHEEL_SPEED_ENVELOPE_START_TPS;
        const float outward_scale = balance_clamp(
            (BALANCE_ODRIVE_SPEED_LIMIT_TPS - fabsf(wheel_speed_tps)) /
                envelope_width_tps,
            0.0f,
            1.0f);
        rate_limited_acceleration_tps2 *= outward_scale;
        rate_speed_envelope_active = true;
    }
    /*
     * Convert the acceleration request into an Axis 0 velocity target.  The
     * elapsed time is capped so a delayed callback cannot create a large
     * command jump after a reset or debugger pause.
     */
    rate_raw_velocity_tps = rate_limited_velocity_tps +
                            rate_limited_acceleration_tps2 *
                                ((float)elapsed_ms * 0.001f);
    rate_limited_velocity_tps = balance_clamp(
        rate_raw_velocity_tps,
        -BALANCE_ODRIVE_SPEED_LIMIT_TPS,
        BALANCE_ODRIVE_SPEED_LIMIT_TPS);
    rate_current_command_a = 0.0f;
    rate_torque_command_nm = 0.0f;
    return rate_limited_velocity_tps;
}

static void balance_reset_controller_state(void)
{
    PWM_X = 0.0f;
    PWM_accel = 0.0f;
    PWM_Final = 0.0f;
    odrive.set_speed0 = 0.0f;
    odrive.set_torque0 = 0.0f;
    cnt1 = 0;
    Angle_Velocity_Last_Bias = 0.0f;
    Angle_Velocity_Integral = 0.0f;
    X_balance_error = 0.0f;
    Velocity_encoder_bias_integral = 0.0f;
    wheel_feedback_bias_deg = 0.0f;
    angle_velocity_initialized = false;
    rate_target_dps = 0.0f;
    rate_error_dps = 0.0f;
    rate_p_term_tps = 0.0f;
    rate_i_term_tps = 0.0f;
    rate_d_term_tps = 0.0f;
    rate_raw_velocity_tps = 0.0f;
    rate_raw_acceleration_tps2 = 0.0f;
    rate_limited_acceleration_tps2 = 0.0f;
    rate_limited_velocity_tps = 0.0f;
    rate_current_command_a = 0.0f;
    rate_torque_command_nm = 0.0f;
    rate_acceleration_boost_active = false;
    rate_speed_envelope_active = false;
    rate_limiter_previous_ms = HAL_GetTick();
    rate_limiter_initialized = false;
}

static void balance_auto_arm_update(void)
{
    const uint32_t now_ms = HAL_GetTick();
    const bool imu_finite = isfinite(imu.rol) && isfinite(imu.vx);
    if (frame_count != balance_last_imu_frame_count)
    {
        balance_last_imu_frame_count = frame_count;
        balance_last_imu_frame_ms = now_ms;
    }
    balance_imu_ready = (frame_count >= BALANCE_ARM_IMU_MIN_FRAMES) &&
                        ((uint32_t)(now_ms - balance_last_imu_frame_ms) <=
                         BALANCE_ARM_IMU_FRESH_MS);
    const bool odrive_heartbeat_fresh = odrive.heartbeat_seen[0] &&
        ((uint32_t)(now_ms - odrive.last_heartbeat_ms[0]) <=
         BALANCE_ODRIVE_FEEDBACK_TIMEOUT_MS);
    const bool odrive_feedback_fresh = odrive_axis_feedback_fresh(
        0u, now_ms, BALANCE_ODRIVE_FEEDBACK_TIMEOUT_MS);
    const bool odrive_error_free = odrive.axis_error[0] == 0u;
    balance_odrive_ready = odrive_feedback_fresh &&
                            odrive_axis_closed_loop_error_free(0u);
    balance_odrive_timeout = !odrive_feedback_fresh;
    balance_odrive_fault = odrive_heartbeat_fresh &&
                           (!odrive_error_free ||
                            ((param.scope_flag == 1) &&
                             (odrive.axis_state[0] != 8u)));

    const bool inside_arm_window = balance_imu_ready &&
                                   imu_finite &&
                                   odrive_feedback_fresh &&
                                   odrive_error_free &&
                                   (fabsf(imu.rol - param.angular_zero) <=
                                    BALANCE_ARM_ROLL_WINDOW_DEG) &&
                                   (fabsf(imu.vx) <= BALANCE_ARM_RATE_WINDOW_DPS);
    const bool fall_angle_exceeded = imu_finite &&
        (fabsf(imu.rol - balance_monitor_zero_for_control()) >=
         BALANCE_FALL_DISARM_ANGLE_DEG);

    if (param.scope_flag == 1)
    {
        balance_arm_window = false;
        if (fall_angle_exceeded)
        {
            balance_fall_disarm_latched = true;
            param.scope_flag = 0;
            balance_reset_controller_state();
            (void)odrive_request_axis_state(0u, 1u);
            return;
        }
        if (!balance_imu_ready || !imu_finite || balance_odrive_fault)
        {
            param.scope_flag = 0;
            balance_reset_controller_state();
        }
        return;
    }

    balance_reset_controller_state();
    if (!inside_arm_window)
    {
        balance_arm_window = false;
        return;
    }

    if (!balance_odrive_ready)
    {
        balance_arm_window = false;
        if ((odrive.axis_state[0] == 1u) &&
            ((uint32_t)(now_ms - balance_last_odrive_state_request_ms) >=
             BALANCE_ODRIVE_STATE_REQUEST_PERIOD_MS))
        {
            balance_last_odrive_state_request_ms = now_ms;
            (void)odrive_request_axis_state(0u, 8u);
        }
        return;
    }

    if (!balance_arm_window)
    {
        balance_arm_start_ms = now_ms;
        balance_arm_window = true;
        return;
    }

    if ((uint32_t)(now_ms - balance_arm_start_ms) >= BALANCE_ARM_HOLD_MS)
    {
        balance_reset_controller_state();
        param.scope_flag = 1;
        balance_fall_disarm_latched = false;
        balance_arm_window = false;
    }
}

//定时器 2ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim3)
    {
        imu_get();//陀螺仪读取
        balance_auto_arm_update();

        const bool rear_feedback_ready = odrive_axis_feedback_fresh(
            1u, HAL_GetTick(), BALANCE_ODRIVE_FEEDBACK_TIMEOUT_MS) &&
            odrive_axis_closed_loop_error_free(1u);
        if (acceleration_frame_count != drive_last_accel_frame_count)
        {
            drive_last_accel_frame_count = acceleration_frame_count;
            drive_last_accel_frame_ms = HAL_GetTick();
        }
        const bool drive_accel_ready = acceleration_frame_count > 0u &&
            (uint32_t)(HAL_GetTick() - drive_last_accel_frame_ms) <=
                BALANCE_ARM_IMU_FRESH_MS;
        drive_control_update(odrive.now_pos1,
                             odrive.now_speed1,
                             odrive.last_encoder_ms[1],
                             imu.ax,
                             drive_accel_ready,
                             param.scope_flag == 1,
                             rear_feedback_ready,
                             balance_fall_disarm_latched);
        steering_update(imu.yaw,
                        imu.vz,
                        imu.rol - balance_monitor_zero_for_control(),
                        odrive.now_speed1,
                        param.scope_flag == 1,
                        balance_imu_ready,
                        rear_feedback_ready,
                        balance_fall_disarm_latched);

        cnt_vel_set1++;
        cnt_balance++;
        if(param.scope_flag == 1)//飞轮平衡控制周期 2ms
        {
                balance();
                cnt_balance=0;
        }
        if(cnt_vel_set1 >= 1)//odrive can通信周期 2ms
        {
                cnt_vel_callback1++;
                odrive_speed_ctrl(0,odrive.set_speed0);
                odrive_vel_callback(0);

                cnt_vel_set1 = 0;
                if(cnt_vel_callback1 == 20)
                {
                        cnt_vel_callback1 = 0;
                        odrive_speed_ctrl(1,-odrive.set_speed1);
                        odrive_vel_callback(1);
                }
        }

        /* 采集控制量，并在已启动且后轮停止时更新动态零点。 */
        balance_monitor_update(imu.rol,
                               imu.vx,
                               odrive.now_speed0,
                               odrive.set_speed0,
                               param.angular_zero,
                               PWM_accel,
                               param.scope_flag == 1,
                               balance_arm_window,
                               param.run_flag == 0,
                               balance_imu_ready,
                               balance_odrive_ready,
                               balance_odrive_timeout,
                               balance_odrive_fault,
                               balance_fall_disarm_latched,
                               rate_target_dps,
                               rate_error_dps,
                               rate_p_term_tps,
                               rate_i_term_tps,
                               rate_d_term_tps,
                               rate_raw_velocity_tps,
                               rate_raw_acceleration_tps2,
                               rate_limited_acceleration_tps2,
                               rate_current_command_a,
                               rate_torque_command_nm,
                               rate_acceleration_boost_active,
                               rate_speed_envelope_active,
                               false);
    }
}

void rate_set()
{
    /* Rear-wheel commands are owned by drive_control_update(). */
}

//pid参数初始化
void param_init(){
    // 角度环
    param.angular_kp = BALANCE_DEFAULT_ANGLE_KP;
    param.angular_ki = BALANCE_DEFAULT_ANGLE_KI;
    param.angular_kd = BALANCE_DEFAULT_ANGLE_KD;

    // 角速度环
    param.angular_v_kp = BALANCE_DEFAULT_RATE_KP;
    param.angular_v_ki = BALANCE_DEFAULT_RATE_KI;
    param.angular_v_kd = BALANCE_DEFAULT_RATE_KD;

    // 速度环
    param.fly_wheel_speed_kp = BALANCE_DEFAULT_WHEEL_KP;
    param.fly_wheel_speed_ki = BALANCE_DEFAULT_WHEEL_KI;
    param.fly_wheel_speed_kd = 0;

    param.zero_speed_kp=0;
    param.zero_speed_kd=0;
    param.zero_speed_ki=0;

    param.angular_zero = BALANCE_FACTORY_ZERO_DEG;
    zero_persistence_init(BALANCE_FACTORY_ZERO_DEG,
                          BALANCE_ZERO_RANGE_DEG,
                          &param.angular_zero);

    // 上电先保持动量轮为零；手工扶正并稳定 2 秒后自动进入平衡控制。
    param.scope_flag = 0;
    odrive.set_speed0 = 0.0f;
    odrive.set_torque0 = 0.0f;
    balance_arm_start_ms = 0u;
    balance_last_imu_frame_count = 0u;
    balance_last_imu_frame_ms = 0u;
    drive_last_accel_frame_count = 0u;
    drive_last_accel_frame_ms = 0u;
    balance_arm_window = false;
    balance_imu_ready = false;
    balance_odrive_ready = false;
    balance_odrive_timeout = true;
    balance_odrive_fault = false;
    balance_fall_disarm_latched = false;
    balance_last_odrive_state_request_ms = 0u;

    // 上电默认保持后轮停止；收到明确运行指令后再将 run_flag 置 1。
    param.run_flag = 0;
    odrive.set_speed1 = 0.0f;

    param.Steer_Kp = BALANCE_DEFAULT_STEER_KP;
    param.Steer_Ki = BALANCE_DEFAULT_STEER_KI;
    param.Steer_Kd = BALANCE_DEFAULT_STEER_KD;

    // 初始化时清空所有积分
    Angle_Velocity_Integral = 0;
    Angle_Velocity_Last_Bias = 0;
    X_balance_error = 0;
    Velocity_encoder_bias_integral = 0;
    angle_velocity_initialized = false;

    balance_monitor_init(BALANCE_FACTORY_ZERO_DEG, param.angular_zero);
    drive_control_init();
}

enum
{
    TUNING_GROUP_RATE = 1u,
    TUNING_GROUP_ANGLE = 2u,
    TUNING_GROUP_WHEEL = 3u,
    TUNING_GROUP_STEER = 4u
};

typedef struct
{
    const char *name;
    float *value;
    float minimum;
    float maximum;
    uint8_t group;
} BalanceTuningDescriptor;

static BalanceTuningDescriptor balance_tuning_parameters[] = {
    {"RATE_KP", &param.angular_v_kp, -40.0f, -0.5f, TUNING_GROUP_RATE},
    {"RATE_KI", &param.angular_v_ki, -5.0f, 0.0f, TUNING_GROUP_RATE},
    {"RATE_KD", &param.angular_v_kd, -0.05f, 0.0f, TUNING_GROUP_RATE},
    {"ANGLE_KP", &param.angular_kp, -10.0f, -0.1f, TUNING_GROUP_ANGLE},
    {"ANGLE_KI", &param.angular_ki, -0.1f, 0.1f, TUNING_GROUP_ANGLE},
    {"ANGLE_KD", &param.angular_kd, -5.0f, 0.0f, TUNING_GROUP_ANGLE},
    {"WHEEL_KP", &param.fly_wheel_speed_kp, 0.0f, 0.2f, TUNING_GROUP_WHEEL},
    {"WHEEL_KI", &param.fly_wheel_speed_ki, 0.0f, 0.1f, TUNING_GROUP_WHEEL},
    {"STEER_KP", &param.Steer_Kp, -10.0f, 10.0f, TUNING_GROUP_STEER},
    {"STEER_KI", &param.Steer_Ki, -2.0f, 2.0f, TUNING_GROUP_STEER},
    {"STEER_KD", &param.Steer_Kd, -5.0f, 5.0f, TUNING_GROUP_STEER}
};

static void balance_tuning_reset_group(uint8_t group)
{
    if (group == TUNING_GROUP_RATE)
    {
        Angle_Velocity_Integral = 0.0f;
        Angle_Velocity_Last_Bias = 0.0f;
        angle_velocity_initialized = false;
    }
    else if (group == TUNING_GROUP_ANGLE)
    {
        X_balance_error = 0.0f;
    }
    else if (group == TUNING_GROUP_WHEEL)
    {
        Velocity_encoder_bias_integral = 0.0f;
        wheel_feedback_bias_deg = 0.0f;
    }
}

BalanceTuningResult balance_tuning_set(const char *name,
                                       float requested_value,
                                       float *applied_value)
{
    if (name == NULL || !isfinite(requested_value))
    {
        return BALANCE_TUNING_OUT_OF_RANGE;
    }
    /* Online gain changes are forbidden while the rear drive wheel is running. */
    if (param.run_flag != 0)
    {
        return BALANCE_TUNING_UNSAFE_STATE;
    }

    const size_t parameter_count =
        sizeof(balance_tuning_parameters) / sizeof(balance_tuning_parameters[0]);
    for (size_t index = 0u; index < parameter_count; index++)
    {
        BalanceTuningDescriptor *descriptor = &balance_tuning_parameters[index];
        if (strcmp(name, descriptor->name) != 0)
        {
            continue;
        }
        if (requested_value < descriptor->minimum ||
            requested_value > descriptor->maximum)
        {
            return BALANCE_TUNING_OUT_OF_RANGE;
        }

        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        *descriptor->value = requested_value;
        balance_tuning_reset_group(descriptor->group);
        if (primask == 0u)
        {
            __enable_irq();
        }
        if (applied_value != NULL)
        {
            *applied_value = requested_value;
        }
        return BALANCE_TUNING_OK;
    }
    return BALANCE_TUNING_UNKNOWN_PARAMETER;
}

void balance_tuning_get(BalanceTuningParameters *parameters)
{
    if (parameters == NULL)
    {
        return;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    parameters->rate_kp = param.angular_v_kp;
    parameters->rate_ki = param.angular_v_ki;
    parameters->rate_kd = param.angular_v_kd;
    parameters->angle_kp = param.angular_kp;
    parameters->angle_ki = param.angular_ki;
    parameters->angle_kd = param.angular_kd;
    parameters->wheel_kp = param.fly_wheel_speed_kp;
    parameters->wheel_ki = param.fly_wheel_speed_ki;
    parameters->steer_kp = param.Steer_Kp;
    parameters->steer_ki = param.Steer_Ki;
    parameters->steer_kd = param.Steer_Kd;
    if (primask == 0u)
    {
        __enable_irq();
    }
}

BalanceTuningResult balance_tuning_revert(void)
{
    if (param.run_flag != 0)
    {
        return BALANCE_TUNING_UNSAFE_STATE;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    param.angular_v_kp = BALANCE_DEFAULT_RATE_KP;
    param.angular_v_ki = BALANCE_DEFAULT_RATE_KI;
    param.angular_v_kd = BALANCE_DEFAULT_RATE_KD;
    param.angular_kp = BALANCE_DEFAULT_ANGLE_KP;
    param.angular_ki = BALANCE_DEFAULT_ANGLE_KI;
    param.angular_kd = BALANCE_DEFAULT_ANGLE_KD;
    param.fly_wheel_speed_kp = BALANCE_DEFAULT_WHEEL_KP;
    param.fly_wheel_speed_ki = BALANCE_DEFAULT_WHEEL_KI;
    param.Steer_Kp = BALANCE_DEFAULT_STEER_KP;
    param.Steer_Ki = BALANCE_DEFAULT_STEER_KI;
    param.Steer_Kd = BALANCE_DEFAULT_STEER_KD;
    balance_tuning_reset_group(TUNING_GROUP_RATE);
    balance_tuning_reset_group(TUNING_GROUP_ANGLE);
    balance_tuning_reset_group(TUNING_GROUP_WHEEL);
    if (primask == 0u)
    {
        __enable_irq();
    }
    return BALANCE_TUNING_OK;
}

//角速度环pid
float Angle_Velocity(float Gyro,float Gyro_Target)
{
    float Angle_Velocity_Bias;
    float PWM_Out;
    // static float Angle_Velocity_Last_Bias,Angle_Velocity_Integral; // 【修改】移至全局
    Angle_Velocity_Bias = Gyro_Target - Gyro;
    if (!angle_velocity_initialized)
    {
        Angle_Velocity_Last_Bias = Angle_Velocity_Bias;
        angle_velocity_initialized = true;
    }
    const float derivative_dps2 =
        (Angle_Velocity_Bias - Angle_Velocity_Last_Bias) /
        BALANCE_CONTROL_DT_S;
    float candidate_integral = Angle_Velocity_Integral +
                               Angle_Velocity_Bias * BALANCE_CONTROL_DT_S;
    candidate_integral = balance_clamp(
        candidate_integral,
        -BALANCE_RATE_INTEGRAL_LIMIT_DPS_S,
        BALANCE_RATE_INTEGRAL_LIMIT_DPS_S);
    rate_target_dps = Gyro_Target;
    rate_error_dps = Angle_Velocity_Bias;
    rate_p_term_tps = param.angular_v_kp * Angle_Velocity_Bias;
    rate_d_term_tps = param.angular_v_kd * derivative_dps2;
    const float candidate_i_term = param.angular_v_ki * candidate_integral;
    const float candidate_output = rate_p_term_tps +
                                   candidate_i_term +
                                   rate_d_term_tps;
    const float integral_output_delta =
        param.angular_v_ki * Angle_Velocity_Bias * BALANCE_CONTROL_DT_S;
    const bool allow_integrator =
        fabsf(candidate_output) <= BALANCE_ODRIVE_ACCEL_LIMIT_TPS2 ||
        (candidate_output > BALANCE_ODRIVE_ACCEL_LIMIT_TPS2 &&
         integral_output_delta < 0.0f) ||
        (candidate_output < -BALANCE_ODRIVE_ACCEL_LIMIT_TPS2 &&
         integral_output_delta > 0.0f);
    if (allow_integrator)
    {
        Angle_Velocity_Integral = candidate_integral;
    }
    rate_i_term_tps = param.angular_v_ki * Angle_Velocity_Integral;
    PWM_Out = rate_p_term_tps + rate_i_term_tps + rate_d_term_tps;
    Angle_Velocity_Last_Bias = Angle_Velocity_Bias;
    return PWM_Out;
}

//角度环pid
float X_balance_Control(float Angle,float Angle_Zero,float gyro)
{
     float PWM,Bias;
     // static float error; // 【修改】移至全局 X_balance_error
     Bias=Angle-Angle_Zero;
     X_balance_error += Bias * BALANCE_CONTROL_DT_S;
     if(X_balance_error > BALANCE_ANGLE_INTEGRAL_LIMIT_DEG_S)
         X_balance_error = BALANCE_ANGLE_INTEGRAL_LIMIT_DEG_S;
     if(X_balance_error < -BALANCE_ANGLE_INTEGRAL_LIMIT_DEG_S)
         X_balance_error = -BALANCE_ANGLE_INTEGRAL_LIMIT_DEG_S;
     PWM=param.angular_kp*Bias + param.angular_ki*X_balance_error + (gyro)*param.angular_kd;
     return balance_clamp(PWM,
                          -BALANCE_RATE_TARGET_LIMIT_DPS,
                          BALANCE_RATE_TARGET_LIMIT_DPS);
}

//速度环pid
float Velocity_Control(float encoder,float target_encoder)
{
    const float encoder_bias = encoder - target_encoder;
    const bool steering_compensation_active =
        !steering_allows_zero_learning();
    if (!steering_compensation_active)
    {
        /*
         * Hand long-term trim back to the persistent dynamic-zero estimator.
         * At 25 Hz, 0.99 gives a gentle ~4 s decay instead of a roll-target
         * step when the post-steering settling gate expires.
         */
        Velocity_encoder_bias_integral *= 0.99f;
    }
    const float candidate_integral = balance_clamp(
        Velocity_encoder_bias_integral +
            encoder_bias * BALANCE_WHEEL_CONTROL_DT_S,
        -BALANCE_WHEEL_INTEGRAL_LIMIT_TPS_S,
        BALANCE_WHEEL_INTEGRAL_LIMIT_TPS_S);
    const float proportional_output =
        encoder_bias * param.fly_wheel_speed_kp;
    const float candidate_integral_output =
        candidate_integral * param.fly_wheel_speed_ki;
    const float candidate_output =
        proportional_output + candidate_integral_output;
    const float integral_output_delta =
        encoder_bias * BALANCE_WHEEL_CONTROL_DT_S *
        param.fly_wheel_speed_ki;
    const bool allow_integrator =
        fabsf(candidate_output) <= BALANCE_WHEEL_BIAS_LIMIT_DEG ||
        (candidate_output > BALANCE_WHEEL_BIAS_LIMIT_DEG &&
         integral_output_delta < 0.0f) ||
        (candidate_output < -BALANCE_WHEEL_BIAS_LIMIT_DEG &&
         integral_output_delta > 0.0f);
    if (steering_compensation_active && allow_integrator)
    {
        Velocity_encoder_bias_integral = candidate_integral;
    }
    return proportional_output +
           Velocity_encoder_bias_integral * param.fly_wheel_speed_ki;
}

void balance(void)
{
    cnt1++;

    // 速度环 25Hz (20 * 2ms)
    if(cnt1 >= 20)
    {
        if (balance_odrive_ready)
        {
            float raw_speed_output = Velocity_Control(odrive.now_speed0, 0);
            raw_speed_output = -raw_speed_output;
            wheel_feedback_bias_deg =
                wheel_feedback_bias_deg * 0.9f + raw_speed_output * 0.1f;
            PWM_accel = balance_clamp(
                steering_balance_feedforward_deg() +
                    wheel_feedback_bias_deg,
                -BALANCE_WHEEL_BIAS_LIMIT_DEG,
                BALANCE_WHEEL_BIAS_LIMIT_DEG);
        }

        cnt1 = 0;
    }

    // 直立环 500Hz
    PWM_X = X_balance_Control(imu.rol,
                              balance_monitor_zero_for_control() + PWM_accel,
                              imu.vx);
    rate_raw_acceleration_tps2 = Angle_Velocity(imu.vx, PWM_X);
    const float acceleration_limit_tps2 =
        balance_select_acceleration_limit_tps2(
            imu.rol - (balance_monitor_zero_for_control() + PWM_accel),
            imu.vx,
            rate_raw_acceleration_tps2);
    PWM_Final = balance_integrate_acceleration_command(
        rate_raw_acceleration_tps2,
        acceleration_limit_tps2,
        odrive.now_speed0);

    odrive.set_speed0 = PWM_Final;
    odrive.set_torque0 = 0.0f;

}

int my_abs(int x)
{
    if(x>=0) return x;
    else return -x;
}

float my_fabs(float x)
{
    if(x>=0) return x;
    else return -x;
}

#include "task.h"
#include "tim.h" 
#include "imu.h"
#include "odrive.h"
#include "servo.h"
#include "upper.h"
#include "balance_monitor.h"
#include "packet.h"
#include "zero_persistence.h"
#include "math.h"

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
#define BALANCE_FACTORY_ZERO_DEG (-2.18f)
#define BALANCE_ZERO_RANGE_DEG 1.50f

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
static bool balance_arm_window;
static bool balance_imu_ready;
static bool angle_velocity_initialized;

static void balance_reset_controller_state(void)
{
    PWM_X = 0.0f;
    PWM_accel = 0.0f;
    PWM_Final = 0.0f;
    odrive.set_speed0 = 0.0f;
    cnt1 = 0;
    Angle_Velocity_Last_Bias = 0.0f;
    Angle_Velocity_Integral = 0.0f;
    X_balance_error = 0.0f;
    Velocity_encoder_bias_integral = 0.0f;
    angle_velocity_initialized = false;
}

static void balance_auto_arm_update(void)
{
    const uint32_t now_ms = HAL_GetTick();
    if (frame_count != balance_last_imu_frame_count)
    {
        balance_last_imu_frame_count = frame_count;
        balance_last_imu_frame_ms = now_ms;
    }
    balance_imu_ready = (frame_count >= BALANCE_ARM_IMU_MIN_FRAMES) &&
                        ((uint32_t)(now_ms - balance_last_imu_frame_ms) <=
                         BALANCE_ARM_IMU_FRESH_MS);

    const bool inside_arm_window = balance_imu_ready &&
                                   isfinite(imu.rol) &&
                                   isfinite(imu.vx) &&
                                   (fabsf(imu.rol - param.angular_zero) <=
                                    BALANCE_ARM_ROLL_WINDOW_DEG) &&
                                   (fabsf(imu.vx) <= BALANCE_ARM_RATE_WINDOW_DPS);

    if (param.scope_flag == 1)
    {
        balance_arm_window = false;
        return;
    }

    balance_reset_controller_state();
    if (!inside_arm_window)
    {
        balance_arm_window = false;
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
        
        cnt_vel_set1++;
        cnt_balance++;  
        cnt_rate++;
        if(cnt_rate>=50)
        {
            cnt_rate=0;
            rate_set();//速度设置
        }
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
                               balance_imu_ready);
    }
}

void rate_set()
{
    if(param.run_flag==1)//运行后轮
    {
        odrive.set_speed1 = -0.7;
    }
    else
    {
        odrive.set_speed1=0;
    }
}

//pid参数初始化
void param_init(){
    // 角度环
    param.angular_kp = -5.0f;      
    param.angular_ki = 0;
    param.angular_kd = -1.5f;      
    
    // 角速度环
    param.angular_v_kp = -20.0f;   
    param.angular_v_ki = 0; 
    param.angular_v_kd =-2.0f; 
    
    // 速度环
    param.fly_wheel_speed_kp = 0.04; 
    param.fly_wheel_speed_ki = 0;
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
    balance_arm_start_ms = 0u;
    balance_last_imu_frame_count = 0u;
    balance_last_imu_frame_ms = 0u;
    balance_arm_window = false;
    balance_imu_ready = false;

    // 上电默认保持后轮停止；收到明确运行指令后再将 run_flag 置 1。
    param.run_flag = 0;
    odrive.set_speed1 = 0.0f;
        
    param.Steer_Kp = 1.5;
    param.Steer_Ki = 0.2;
    param.Steer_Kd = 0;
    
    // 初始化时清空所有积分
    Angle_Velocity_Integral = 0;
    Angle_Velocity_Last_Bias = 0;
    X_balance_error = 0;
    Velocity_encoder_bias_integral = 0;
    angle_velocity_initialized = false;

    balance_monitor_init(BALANCE_FACTORY_ZERO_DEG, param.angular_zero);
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
    Angle_Velocity_Integral+=Angle_Velocity_Bias;
    if(Angle_Velocity_Integral > 10000)
            Angle_Velocity_Integral =10000;                    
    if(Angle_Velocity_Integral < -10000) 
            Angle_Velocity_Integral = -10000;               
    PWM_Out = param.angular_v_kp * Angle_Velocity_Bias + param.angular_v_ki * Angle_Velocity_Integral + param.angular_v_kd * (Angle_Velocity_Bias - Angle_Velocity_Last_Bias);
    Angle_Velocity_Last_Bias = Angle_Velocity_Bias;
    return PWM_Out;
}

//角度环pid
float X_balance_Control(float Angle,float Angle_Zero,float gyro)
{
     float PWM,Bias;
     // static float error; // 【修改】移至全局 X_balance_error
     Bias=Angle-Angle_Zero;                                        
     X_balance_error+=Bias;                                                    
     if(X_balance_error>+30) X_balance_error=+30;                                    
     if(X_balance_error<-30) X_balance_error=-30;                                    
     PWM=param.angular_kp*Bias + param.angular_ki*X_balance_error + (gyro)*param.angular_kd;    
     return PWM;
}

//速度环pid
float Velocity_Control(float encoder,float target_encoder)
{
    float encoder_bias,Velocity;
    // static float encoder_bias_integral; // 【修改】移至全局 Velocity_encoder_bias_integral
    encoder_bias = encoder - target_encoder;
    Velocity_encoder_bias_integral += encoder_bias;
    if(Velocity_encoder_bias_integral > +200) 
            Velocity_encoder_bias_integral = +200;                    
    if(Velocity_encoder_bias_integral < -200) 
            Velocity_encoder_bias_integral = -200;                    
    Velocity = encoder_bias * param.fly_wheel_speed_kp + Velocity_encoder_bias_integral * param.fly_wheel_speed_ki/1000;
    return Velocity;
}

void balance(void)
{
    cnt1++;
    
    // 速度环 25Hz (20 * 2ms)
    if(cnt1 >= 20) 
    {
        // 1. 计算原始输出
        float raw_speed_output = Velocity_Control(odrive.now_speed0, 0);
        
        // 2. 【关键】如果之前是拽着倒，这里取反！
        // 试着加个负号（-）。如果之前有负号，就去掉。
        raw_speed_output = -raw_speed_output; 

        // 3. 【关键】低通滤波 (Low Pass Filter)
        // 速度环不要突变，要软！0.9 是旧值权重，0.1 是新值权重。
        PWM_accel = PWM_accel * 0.9f + raw_speed_output * 0.1f;
        
        cnt1 = 0;
        
        // 4. 限幅 (老生常谈，保命用的)
        if(PWM_accel > 1.5f) PWM_accel = 1.5f;
        if(PWM_accel < -1.5f) PWM_accel = -1.5f;
    }

    // 直立环 500Hz
    // 注意：这里使用的是经过 滤波 和 取反 后的 PWM_accel
    PWM_X = X_balance_Control(imu.rol,
                              balance_monitor_zero_for_control() + PWM_accel,
                              imu.vx);
    PWM_Final = Angle_Velocity(imu.vx, PWM_X);                                                                                                      
    
    odrive.set_speed0 = PWM_Final;                              
    
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

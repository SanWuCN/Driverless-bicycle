#include "servo.h"
#include "steering.h"

HAL_StatusTypeDef servo_status;

static int current_pulse_us;

void servo_init(void)
{
	MX_TIM2_Init(); //PWM OUTPUT
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // PA2 / TIM2_CH3
	steering_init();
}

void servo_set_duty(int duty)
{
    int target = SERVO_CENTER_PULSE_US + duty;
    target = target > SERVO_MAX_PULSE_US ? SERVO_MAX_PULSE_US : target;
    target = target < SERVO_MIN_PULSE_US ? SERVO_MIN_PULSE_US : target;
    servo_set_pulse_us(target);
}

void servo_set_pulse_us(int pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE_US)
        pulse_us = SERVO_MIN_PULSE_US;
    if (pulse_us > SERVO_MAX_PULSE_US)
        pulse_us = SERVO_MAX_PULSE_US;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)pulse_us);
    current_pulse_us = pulse_us;
}

void servo_disable(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0u);
    current_pulse_us = 0;
}

int servo_current_pulse_us(void)
{
    return current_pulse_us;
}
//速度限幅
int Steer_Speed_Limit(int now, int last, int limit, int times)
{
    static int cnt = 0;
    cnt++;
    if (cnt >= times)
    {
        cnt = 0;
        if ((now - last) >= limit)
            return (last + limit);
        else if ((now - last) <= -limit)
            return (last - limit);
        else
            return now;
    }
    return last;
}

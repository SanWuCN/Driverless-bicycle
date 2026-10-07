#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"
#include "tim.h"

#define SERVO_CENTER_PULSE_US 1520
#define SERVO_MIN_PULSE_US    1370
#define SERVO_MAX_PULSE_US    1670


void servo_init(void);
void servo_set_duty(int duty);
void servo_set_pulse_us(int pulse_us);
void servo_disable(void);
int servo_current_pulse_us(void);

int Steer_Speed_Limit(int now, int last, int limit, int times);
#endif



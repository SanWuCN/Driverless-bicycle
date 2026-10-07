#ifndef __ODRIVE_H__
#define __ODRIVE_H__

#include <stdbool.h>
#include <stdint.h>

#define AXIS0_CAN_NODE_ID (0x18)
#define AXIS1_CAN_NODE_ID (0x10)

#define NODE_ID(num) ((num) == 0 ? AXIS0_CAN_NODE_ID : AXIS1_CAN_NODE_ID)

// ... (剩下的 enum 和 struct 内容完全不动，保持原样) ...

typedef enum
{
    MSG_CO_NMT_CTRL = 0x000,
    MSG_ODRIVE_HEARTBEAT,
    MSG_ODRIVE_ESTOP,
    MSG_GET_MOTOR_ERROR,
    MSG_GET_ENCODER_ERROR,
    MSG_GET_SENSORLESS_ERROR,
    MSG_SET_AXIS_NODE_ID,
    MSG_SET_AXIS_REQUESTED_STATE,
    MSG_SET_AXIS_STARTUP_CONFIG,
    MSG_GET_ENCODER_ESTIMATES,
    MSG_GET_ENCODER_COUNT,
    MSG_SET_CONTROLLER_MODES,
    MSG_SET_INPUT_POS,
    MSG_SET_INPUT_VEL,
    MSG_SET_INPUT_TORQUE,
    MSG_SET_VEL_LIMIT,
    MSG_START_ANTICOGGING,
    MSG_SET_TRAJ_VEL_LIMIT,
    MSG_SET_TRAJ_ACCEL_LIMITS,
    MSG_SET_TRAJ_A_PER_CSS,
    MSG_GET_IQ,
    MSG_GET_SENSORLESS_ESTIMATES,
    MSG_RESET_ODRIVE,
    MSG_GET_VBUS_VOLTAGE,
    MSG_CLEAR_ERRORS,
    MSG_CO_HEARTBEAT_CMD = 0x700,
} OdriveMsg_t;

typedef struct
{
    float set_speed0;
    float set_speed1;
    float set_torque0;
    float fliter_speed0[3];
    float fliter_speed1[3];
    float now_speed0;
    float now_speed1;
    float now_pos0;
    float now_pos1;
    int speed0_i;
    int speed1_i;
    volatile uint32_t axis_error[2];
    volatile uint32_t last_heartbeat_ms[2];
    volatile uint32_t last_encoder_ms[2];
    volatile uint32_t tx_failure_count;
    volatile uint8_t axis_state[2];
    volatile bool heartbeat_seen[2];
    volatile bool encoder_seen[2];
}OdirveTypeDef;

extern OdirveTypeDef odrive;
void odrive_canFilter_init(void);
void odrive_init(void);
void odrive_vel_callback(unsigned char num);
bool odrive_speed_ctrl(unsigned char num, float speed);
bool odrive_torque_ctrl(unsigned char num, float torque_nm);
bool odrive_set_controller_modes(unsigned char num,
                                 uint32_t control_mode,
                                 uint32_t input_mode);
bool odrive_request_axis_state(unsigned char num, uint32_t requested_state);
bool odrive_axis_feedback_fresh(unsigned char num,
                                uint32_t now_ms,
                                uint32_t timeout_ms);
bool odrive_axis_closed_loop_error_free(unsigned char num);

#endif

#include "odrive.h"
#include "can.h"
#include <string.h>

OdirveTypeDef odrive;
int rx_packet_count = 0;

void odrive_init(void)
{
    memset(&odrive, 0, sizeof(OdirveTypeDef));
    odrive_canFilter_init();
}

void odrive_canFilter_init(void)
{
    CAN_FilterTypeDef filter;

    // 【关键修复】确保 CAN1 和 CAN2 的时钟同时开启，否则过滤器无法共享
    __HAL_RCC_CAN1_CLK_ENABLE();

    filter.FilterActivation = ENABLE;

    // 强制使用 Bank 14
    filter.FilterBank = 14;

    filter.FilterFIFOAssignment = CAN_RX_FIFO0;

    // 【全通模式】不检查任何 ID 掩码，确保物理层只要有包就能进入 rx_packet_count
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;

    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    // 【核心修复】明确告诉硬件：Bank 14 及之后的过滤器属于从机（CAN2）
    // 这一步在许多 F4 库中是必须的，否则 Bank 14 默认仍归属 CAN1
    filter.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan2, &filter);

    // 启动并激活通知
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

bool odrive_speed_ctrl(unsigned char num, float speed)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    header.RTR = CAN_RTR_DATA;
    header.IDE = CAN_ID_STD;
    header.DLC = 8;
    header.StdId = ((NODE_ID(num) << 5) | MSG_SET_INPUT_VEL);
    header.ExtId = 0;
    header.TransmitGlobalTime = DISABLE;

    memcpy(&data[0], &speed, 4);
    memset(&data[4], 0, 4);

    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(&hcan2, &header, data, &mailbox) != HAL_OK)
    {
        odrive.tx_failure_count++;
        return false;
    }
    return true;
}

bool odrive_torque_ctrl(unsigned char num, float torque_nm)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = {0};
    uint32_t mailbox;

    header.RTR = CAN_RTR_DATA;
    header.IDE = CAN_ID_STD;
    header.DLC = 8;
    header.StdId = ((NODE_ID(num) << 5) | MSG_SET_INPUT_TORQUE);
    header.ExtId = 0;
    header.TransmitGlobalTime = DISABLE;
    memcpy(&data[0], &torque_nm, sizeof(torque_nm));

    if (HAL_CAN_AddTxMessage(&hcan2, &header, data, &mailbox) != HAL_OK)
    {
        odrive.tx_failure_count++;
        return false;
    }
    return true;
}

bool odrive_set_controller_modes(unsigned char num,
                                 uint32_t control_mode,
                                 uint32_t input_mode)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    uint32_t mailbox;

    header.RTR = CAN_RTR_DATA;
    header.IDE = CAN_ID_STD;
    header.DLC = 8;
    header.StdId = ((NODE_ID(num) << 5) | MSG_SET_CONTROLLER_MODES);
    header.ExtId = 0;
    header.TransmitGlobalTime = DISABLE;
    memcpy(&data[0], &control_mode, sizeof(control_mode));
    memcpy(&data[4], &input_mode, sizeof(input_mode));

    if (HAL_CAN_AddTxMessage(&hcan2, &header, data, &mailbox) != HAL_OK)
    {
        odrive.tx_failure_count++;
        return false;
    }
    return true;
}

bool odrive_request_axis_state(unsigned char num, uint32_t requested_state)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[4];
    uint32_t mailbox;

    header.RTR = CAN_RTR_DATA;
    header.IDE = CAN_ID_STD;
    header.DLC = 4;
    header.StdId = ((NODE_ID(num) << 5) | MSG_SET_AXIS_REQUESTED_STATE);
    header.ExtId = 0;
    header.TransmitGlobalTime = DISABLE;
    memcpy(data, &requested_state, sizeof(requested_state));

    if (HAL_CAN_AddTxMessage(&hcan2, &header, data, &mailbox) != HAL_OK)
    {
        odrive.tx_failure_count++;
        return false;
    }
    return true;
}

bool odrive_clear_errors(unsigned char num)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[1] = {0u};
    uint32_t mailbox;

    header.RTR = CAN_RTR_DATA;
    header.IDE = CAN_ID_STD;
    header.DLC = 0u;
    header.StdId = ((NODE_ID(num) << 5) | MSG_CLEAR_ERRORS);
    header.ExtId = 0u;
    header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan2, &header, data, &mailbox) != HAL_OK)
    {
        odrive.tx_failure_count++;
        return false;
    }
    return true;
}

void odrive_vel_callback(unsigned char num)
{
    CAN_TxHeaderTypeDef header;
    uint8_t dummy_data[8] = {0};

    header.RTR = CAN_RTR_REMOTE;
    header.IDE = CAN_ID_STD;
    header.DLC = 0;
    header.StdId = ((NODE_ID(num) << 5) | MSG_GET_ENCODER_ESTIMATES);
    header.ExtId = 0;
    header.TransmitGlobalTime = DISABLE;

    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(&hcan2, &header, dummy_data, &mailbox) != HAL_OK)
    {
        odrive.tx_failure_count++;
    }
}

bool odrive_axis_feedback_fresh(unsigned char num,
                                uint32_t now_ms,
                                uint32_t timeout_ms)
{
    const unsigned char axis = (num == 0u) ? 0u : 1u;
    return odrive.heartbeat_seen[axis] &&
           odrive.encoder_seen[axis] &&
           ((uint32_t)(now_ms - odrive.last_heartbeat_ms[axis]) <= timeout_ms) &&
           ((uint32_t)(now_ms - odrive.last_encoder_ms[axis]) <= timeout_ms);
}

bool odrive_axis_closed_loop_error_free(unsigned char num)
{
    const unsigned char axis = (num == 0u) ? 0u : 1u;
    return (odrive.axis_error[axis] == 0u) &&
           (odrive.axis_state[axis] == 8u);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t buf[8];

    // 明确判断实例是否为 CAN2
    if(hcan->Instance == CAN2)
    {
        if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, buf) == HAL_OK)
        {
            rx_packet_count++;

            uint8_t cmd_id = header.StdId & 0x1F;
            uint8_t node_id = header.StdId >> 5;

            if ((cmd_id == MSG_ODRIVE_HEARTBEAT) && (header.DLC >= 5u))
            {
                const unsigned char axis =
                    (node_id == AXIS0_CAN_NODE_ID) ? 0u : 1u;
                if ((node_id == AXIS0_CAN_NODE_ID) ||
                    (node_id == AXIS1_CAN_NODE_ID))
                {
                    memcpy((void *)&odrive.axis_error[axis], &buf[0], 4);
                    odrive.axis_state[axis] = buf[4];
                    odrive.last_heartbeat_ms[axis] = HAL_GetTick();
                    odrive.heartbeat_seen[axis] = true;
                }
            }
            else if (cmd_id == MSG_GET_ENCODER_ESTIMATES)
            {
                float temp_pos = 0;
                float temp_vel = 0;
                memcpy(&temp_pos, &buf[0], 4);
                memcpy(&temp_vel, &buf[4], 4);

                if(node_id == AXIS0_CAN_NODE_ID)
                {
                    odrive.now_pos0 = temp_pos;
                    odrive.speed0_i = (odrive.speed0_i + 1) % 3;
                    odrive.fliter_speed0[odrive.speed0_i] = temp_vel;
                    odrive.now_speed0 = (odrive.fliter_speed0[0] + odrive.fliter_speed0[1] + odrive.fliter_speed0[2]) / 3.0f;
                    odrive.last_encoder_ms[0] = HAL_GetTick();
                    odrive.encoder_seen[0] = true;
                }
                else if(node_id == AXIS1_CAN_NODE_ID)
                {
                    odrive.now_pos1 = temp_pos;
                    odrive.speed1_i = (odrive.speed1_i + 1) % 3;
                    odrive.fliter_speed1[odrive.speed1_i] = temp_vel;
                    odrive.now_speed1 = (odrive.fliter_speed1[0] + odrive.fliter_speed1[1] + odrive.fliter_speed1[2]) / 3.0f;
                    odrive.last_encoder_ms[1] = HAL_GetTick();
                    odrive.encoder_seen[1] = true;
                }
            }
        }
    }
}

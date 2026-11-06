#include "CAN_cmd_gaoqing.h"

#include "bsp_can.h"
#include "stm32f4xx_hal.h"

/* 高擎电机用 29 位扩展帧，和 DM 的标准帧不同，所以单独一套发送数据 */
static CanCtrlData_s GAOQING_CTRL_DATA = {
    .tx_header.IDE = CAN_ID_EXT,
    .tx_header.RTR = CAN_RTR_DATA,
    .tx_header.DLC = 8,
};

/* 检查电机类型是否为高擎，返回对应 CAN 句柄 */
static hcan_t * GaoQingGetHcanPoint(Motor_s * motor)
{
    if (motor->type != GAOQING_MOTOR) {
        return NULL;
    }
    if (motor->can == 1) {
        return &hcan1;
    } else if (motor->can == 2) {
        return &hcan2;
    }
    return NULL;  // 如果不是高擎电机，返回 NULL
}

/* 扩展帧底层发送 */
static void GaoQingSendExt(hcan_t * hcan, uint32_t ext_id, const uint8_t * data, uint8_t dlc)
{
    GAOQING_CTRL_DATA.hcan = hcan;
    GAOQING_CTRL_DATA.tx_header.ExtId = ext_id;
    GAOQING_CTRL_DATA.tx_header.DLC = dlc;

    for(uint8_t i = 0; i < 8; i++)
    {
        GAOQING_CTRL_DATA.tx_data[i] = (i < dlc) ? data[i] : 0;  // 填充数据，未使用的字节填充为 0;
    }

    CAN_SendTxMessage(&GAOQING_CTRL_DATA);
}

/**
 * @brief 高擎电机力矩控制（等价于达妙 MIT 力矩 kp=kd=0，内部走力矩模式）
 * @param motor 电机结构体，motor->set.tor 为真实力矩(N·m)
 */
void GaoQingMitCtrlTorque(Motor_s * motor)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) {
        return;  // 如果不是高擎电机，直接返回
    }

    float tor_raw = motor->set.tor / GAOQING_MOTOR_K * 100.0f;  // 力矩修正
    int16_t tqe_int;
    if(tor_raw > 32767.0f) {
        tqe_int = 32767;
    } else if(tor_raw < -32768.0f) {
        tqe_int = -32768;
    } else {
        tqe_int = (int16_t)tor_raw;
    }

    uint8_t data[4];
    data[0] = 0x05;
    data[1] = 0x13;
    data[2] = (uint8_t)(tqe_int & 0xFF);
    data[3] = (uint8_t)((tqe_int >> 8) & 0xFF);

    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 4);
}

/**
 * @brief 高擎电机停止（三相断开，等效零力）
 */
void GaoQingStop(Motor_s * motor)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) {
        return;  // 如果不是高擎电机，直接返回
    }

    uint8_t data[3] = {0x01, 0x00, 0x00};
    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 3);
}

/**
 * @brief 高擎电机刹车（三相接地，阻尼）
 */
void GaoQingBrake(Motor_s * motor)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) {
        return;  // 如果不是高擎电机，直接返回
    }

    uint8_t data[3] = {0x01, 0x00, 0x0F};
    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 3);
}

/**
 * @brief 开启高擎电机周期返回状态（上电/初始化时调用一次）
 * @param t_ms 返回周期(ms)，0 为关闭。控制环 1kHz 建议 1~2ms
 */
void GaoQingSetTimedReturnStatus(Motor_s * motor, int16_t t_ms)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) {
        return;  // 如果不是高擎电机，直接返回
    }

    uint8_t data[5];
    data[0] = 0x05;
    data[1] = 0xB4;
    data[2] = 0x02;
    data[3] = (uint8_t)(t_ms & 0xFF);
    data[4] = (uint8_t)((t_ms >> 8) & 0xFF);

    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 5);
}

/**
 * @brief 高擎电机电流模式（DQ电流），motor->set.curr 单位 A
 */
void GaoQingCurCtrl(Motor_s * motor)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) {
        return;  // 如果不是高擎电机，直接返回
    }

    int16_t curr_int = (int16_t)(motor->set.curr * 10.0f);  // 电流单位 A 转换为整数

    uint8_t data[7];
    data[0] = 0x01;
    data[1] = 0x00;
    data[2] = 0x09;
    data[3] = 0x05;
    data[4] = 0x1C;
    data[5] = (uint8_t)(curr_int & 0xFF);
    data[6] = (uint8_t)((curr_int >> 8) & 0xFF);

    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 7);
}

/**
 * @brief 高擎电机电压模式（DQ电压），motor->set.volt 单位 V
 */
void GaoQingVoltCtrl(Motor_s * motor)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) {
        return;  // 如果不是高擎电机，直接返回
    }

    int16_t volt_int = (int16_t)(motor->set.volt * 10.0f);  // 电压单位 V 转换为整数

    uint8_t data[7];
    data[0] = 0x01;
    data[1] = 0x00;
    data[2] = 0x08;
    data[3] = 0x05;
    data[4] = 0x1B;
    data[5] = (uint8_t)(volt_int & 0xFF);
    data[6] = (uint8_t)((volt_int >> 8) & 0xFF);

    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 7);
}

/**
 * @brief 高擎电机位置模式，motor->set.pos 单位 rad（输出轴），motor->set.tor 为最大力矩(N·m)
 */
void GaoQingPosCtrl(Motor_s * motor)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) {
        return;  // 如果不是高擎电机，直接返回
    }

    int16_t pos_int = (int16_t)(motor->set.pos / 6.2831853f * 10000.0f);  // 位置单位 rad 转换为整数
    int16_t tqe_int = (int16_t)(motor->set.tor / GAOQING_MOTOR_K * 100.0f);  // 力矩单位 N·m 转换为整数

    uint8_t data[8];
    data[0] = 0x07;
    data[1] = 0x07;
    data[2] = (uint8_t)(pos_int & 0xFF);
    data[3] = (uint8_t)((pos_int >> 8) & 0xFF);
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = (uint8_t)(tqe_int & 0xFF);
    data[7] = (uint8_t)((tqe_int >> 8) & 0xFF);

    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 8);
}

/**
 * @brief 高擎电机速度模式，motor->set.vel 单位 rad/s（输出轴），motor->set.tor 为最大力矩(N·m)
 */
void GaoQingVelCtrl(Motor_s * motor)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) {
        return;  // 如果不是高擎电机，直接返回
    }   

    int16_t vel_int = (int16_t)(motor->set.vel / 6.2831853f * 4000.0f);  // 速度单位 rad/s 转换为整数
    int16_t tqe_int = (int16_t)(motor->set.tor / GAOQING_MOTOR_K * 100.0f);  // 力矩单位 N·m 转换为整数

    uint8_t data[8];
    data[0] = 0x07;
    data[1] = 0x07;
    data[2] = 0x00;
    data[3] = 0x80;
    data[4] = (uint8_t)(vel_int & 0xFF);
    data[5] = (uint8_t)((vel_int >> 8) & 0xFF);
    data[6] = (uint8_t)(tqe_int & 0xFF);
    data[7] = (uint8_t)((tqe_int >> 8) & 0xFF);

    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 8);
}

/**
 * @brief 高擎电机复位位置零点（当前角度为零点，断电不丢失）
 * @param motor 电机结构体
 */
void GaoQingRezeroPos(Motor_s * motor)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) {
        return;  // 如果不是高擎电机，直接返回
    }

    // 先把电机停下（零点必须在停止状态设置才有效）
    uint8_t stop[3] = {0x01, 0x00, 0x00};
    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, stop, 3);
    HAL_Delay(100);

    // 设置当前位置为零点（0x40 写寄存器）
    uint8_t zero[7] = {0x40, 0x01, 0x04, 0x64, 0x20, 0x63, 0x0A};
    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, zero, 7);
    HAL_Delay(1000);   // 例程建议延时 1s

    // 保存到 flash（否则断电丢失）
    uint8_t save[5] = {0x05, 0xB3, 0x02, 0x00, 0x00};
    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, save, 5);
}

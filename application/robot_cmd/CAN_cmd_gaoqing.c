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
    if (hcan == NULL) return;

    float tor = motor->set.tor * 100.0f;   // N·m -> 0.01 N·m（无 k 系数）
    int16_t tqe_int;
    if (tor > 32767.0f)       tqe_int = 32767;
    else if (tor < -32768.0f) tqe_int = -32768;
    else                      tqe_int = (int16_t)tor;

    uint8_t data[4];
    data[0] = 0x1B;                          // 力矩模式码
    data[1] = 0x0E;                          // 查询码(状态)
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
    if (hcan == NULL) return;
    uint8_t data[2] = {0x01, 0x0E};
    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 2);
}

/**
 * @brief 高擎电机刹车（三相接地，阻尼）
 */
void GaoQingBrake(Motor_s * motor)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) return;
    uint8_t data[2] = {0x18, 0x0E};
    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 2);
}

/**
 * @brief 开启高擎电机周期返回状态（上电/初始化时调用一次）
 * @param t_ms 返回周期(ms)，0 为关闭。控制环 1kHz 建议 1~2ms
 */
void GaoQingSetTimedReturnStatus(Motor_s * motor, int16_t t_us)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) return;

    uint8_t data[8];
    data[0] = 0x03;
    data[1] = 0x00;
    data[2] = 0x05;                          // 周期返回命令
    data[3] = 0x0E;                          // 返回查询码(0x0E状态)
    data[4] = (uint8_t)(t_us & 0xFF);        // 微秒，小端
    data[5] = (uint8_t)((t_us >> 8) & 0xFF);
    data[6] = (uint8_t)((t_us >> 16) & 0xFF);
    data[7] = (uint8_t)((t_us >> 24) & 0xFF);

    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 8);
}

/**
 * @brief 高擎电机电流模式（DQ电流），motor->set.curr 单位 A
 */
void GaoQingCurCtrl(Motor_s * motor)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) return;

    int16_t cur_int = (int16_t)(motor->set.curr * 10.0f);  // A -> 0.1A（单位需确认，见下）

    uint8_t data[6];
    data[0] = 0x1A;                          // 电流模式码
    data[1] = 0x0E;                          // 查询码
    data[2] = 0x00;                          // D轴电流固定为 0
    data[3] = 0x00;
    data[4] = (uint8_t)(cur_int & 0xFF);     // Q轴电流，小端
    data[5] = (uint8_t)((cur_int >> 8) & 0xFF);

    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 6);
}

/**
 * @brief 高擎电机电压模式（DQ电压），motor->set.volt 单位 V
 */
void GaoQingVoltCtrl(Motor_s * motor)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) return;

    int16_t vol_int = (int16_t)(motor->set.volt * 10.0f);  // V -> 0.1V（单位需确认）

    uint8_t data[6];
    data[0] = 0x19;                          // 电压模式码
    data[1] = 0x0E;                          // 查询码
    data[2] = 0x00;                          // D轴电压固定为 0
    data[3] = 0x00;
    data[4] = (uint8_t)(vol_int & 0xFF);     // Q轴电压，小端
    data[5] = (uint8_t)((vol_int >> 8) & 0xFF);

    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 6);
}

/**
 * @brief 高擎电机位置模式，motor->set.pos 单位 rad（输出轴），motor->set.tor 为最大力矩(N·m)
 */
void GaoQingPosCtrl(Motor_s * motor)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) return;

    int16_t pos_int = (int16_t)(motor->set.pos / 6.2831853f * 10000.0f);  // rad -> 圈 -> 0.0001圈

    uint8_t data[4];
    data[0] = 0x1D;                          // 位置模式码
    data[1] = 0x0E;                          // 查询码
    data[2] = (uint8_t)(pos_int & 0xFF);
    data[3] = (uint8_t)((pos_int >> 8) & 0xFF);

    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 4);
}

/**
 * @brief 高擎电机速度模式，motor->set.vel 单位 rad/s（输出轴），motor->set.tor 为最大力矩(N·m)
 */
void GaoQingVelCtrl(Motor_s * motor)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) return;

    int16_t vel_int = (int16_t)(motor->set.vel / 6.2831853f * 4000.0f);  // rad/s -> 圈/s -> 0.00025圈/s

    uint8_t data[4];
    data[0] = 0x1C;                          // 速度模式码
    data[1] = 0x0E;                          // 查询码
    data[2] = (uint8_t)(vel_int & 0xFF);
    data[3] = (uint8_t)((vel_int >> 8) & 0xFF);

    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 4);
}

/**
 * @brief 高擎电机复位位置零点（当前角度为零点，断电不丢失）
 * @param motor 电机结构体
 */
void GaoQingRezeroPos(Motor_s * motor)
{
    hcan_t * hcan = GaoQingGetHcanPoint(motor);
    if (hcan == NULL) return;

    uint8_t data[3] = {0x03, 0x03, 0x03};   // 系统命令-重设零位
    GaoQingSendExt(hcan, GAOQING_CTRL_ID_BASE | motor->id, data, 3);
}

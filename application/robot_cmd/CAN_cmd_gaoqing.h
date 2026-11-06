#ifndef CAN_CMD_GAOQING_H
#define CAN_CMD_GAOQING_H

#include "motor.h"

/* 高擎电机型号力矩修正系数 k。
 * M5047_36 = 0.35。换型号时改成对应 k（例程 convert.c 的 motor_tqe_adj 表）。
 * 必须和第 3 步解码里的 0.0035(=0.01*k) 保持一致。 */
#define GAOQING_MOTOR_K 0.35f

/* 高擎控制帧 ID 基址（扩展帧） */
#define GAOQING_CTRL_ID_BASE 0x8000u    // 普通控制帧：力矩/停止/刹车/周期返回
#define GAOQING_MIT_ID_BASE  0x18000u   // MIT 运控帧

extern void GaoQingMitCtrlTorque(Motor_s * motor);
extern void GaoQingStop(Motor_s * motor);
extern void GaoQingBrake(Motor_s * motor);
extern void GaoQingSetTimedReturnStatus(Motor_s * motor, int16_t t_ms);
extern void GaoQingCurCtrl(Motor_s * motor);   // 电流模式，motor->set.curr (A)
extern void GaoQingVoltCtrl(Motor_s * motor);  // 电压模式，motor->set.volt (V)
extern void GaoQingPosCtrl(Motor_s * motor);   // 位置模式，motor->set.pos (rad)，set.tor 为最大力矩限幅
extern void GaoQingVelCtrl(Motor_s * motor);   // 速度模式，motor->set.vel (rad/s)，set.tor 为最大力矩限幅
extern void GaoQingRezeroPos(Motor_s * motor);

#endif
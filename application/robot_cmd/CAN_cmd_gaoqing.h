#ifndef CAN_CMD_GAOQING_H
#define CAN_CMD_GAOQING_H

#include "motor.h"


/* 高擎控制帧 ID 基址（扩展帧） */
#define GAOQING_CTRL_ID_BASE 0x18000u    // 控制帧(发送) 0x18000|id
#define GAOQING_MIT_ID_BASE  0x58000u    // MIT 帧 0x58000|id（不用可不管）

extern void GaoQingMitCtrlTorque(Motor_s * motor);
extern void GaoQingStop(Motor_s * motor);
extern void GaoQingBrake(Motor_s * motor);
extern void GaoQingSetTimedReturnStatus(Motor_s * motor, int16_t t_us);
extern void GaoQingCurCtrl(Motor_s * motor);   // 电流模式，motor->set.curr (A)
extern void GaoQingVoltCtrl(Motor_s * motor);  // 电压模式，motor->set.volt (V)
extern void GaoQingPosCtrl(Motor_s * motor);   // 位置模式，motor->set.pos (rad)，set.tor 为最大力矩限幅
extern void GaoQingVelCtrl(Motor_s * motor);   // 速度模式，motor->set.vel (rad/s)，set.tor 为最大力矩限幅
extern void GaoQingRezeroPos(Motor_s * motor);

#endif

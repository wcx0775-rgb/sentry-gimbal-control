/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       lqr.c/h
  * @brief      LQR(LQI) 云台控制器：带积分增广的最优状态反馈
  * @note       状态量 x = [e, omega, e_int]^T
  *             e     = 角度误差 (rad)，由调用方预先做角度缠绕
  *             omega = 角速度 (rad/s)，与 e 同坐标系（云台侧/IMU 侧）
  *             e_int = 误差积分 (rad*s)
  *             控制律 u = -(k1*e + k2*omega + ki*e_int)
  *             增益 k1/k2/ki 离线用 MATLAB dlqr 求解，在线只做乘加，1kHz 无压力
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-10-06      汪辰旭          1. done
  *
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#ifndef LQR_H
#define LQR_H

#include "struct_typedef.h"

typedef struct 
{
    /* 反馈增益（MATLAB dlqr 离线求解，力矩模式单位 N*m） */
    fp32 k1;        //角度误差增益  [N*m/rad]
    fp32 k2;        //角速度增益    [N*m/(rad/s)]
    fp32 ki;        //积分增益（LQI, 消稳态误差）[N*m/(rad*s)]

    fp32 dt;        //控制周期（s）
    fp32 max_out;   //输出限幅（u的单位上限）
    fp32 max_iout;  //积分限幅（抗饱和）

    fp32 angle_ref; //目标角度
    fp32 angle_fdb; //反馈角度
    fp32 omega_fdb; //反馈角速度

    fp32 e_int;     //积分状态
    fp32 out;       //输出
} Lqr_t;

/**
  * @brief          LQR 初始化
  * @param[out]     lqr: LQR 结构指针
  * @param[in]      K: 增益数组 {k1, k2, ki}
  * @param[in]      max_out: 输出限幅（N*m）
  * @param[in]      max_iout: 积分限幅（rad*s）
  * @param[in]      dt: 控制周期 (s)
  */
extern void LQR_init(Lqr_t *lqr, const fp32 K[3], fp32 max_out, fp32 max_iout, fp32 dt);

/**
  * @brief          在线更新增益（调参用）
  */
extern void LQR_param_update(Lqr_t *lqr, const fp32 K[3]);

/**
  * @brief          LQR 计算
  * @param[out]     lqr: LQR 结构指针
  * @param[in]      err: 角度误差 (rad)，已由调用方做角度缠绕
  * @param[in]      omega_fdb: 角速度反馈 (rad/s)，与 err 同坐标系
  * @retval         输出（N*m，已限幅）
  */
extern fp32 LQR_calc(Lqr_t *lqr, fp32 err, fp32 omega_fdb);

/**
  * @brief          清空积分状态（切零力/失能时调用，防积分残留跳变）
  */
extern void LQR_clear(Lqr_t *lqr);




#endif

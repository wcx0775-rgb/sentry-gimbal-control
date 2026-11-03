/**
  * @file       robot_param_omni_infantry.h
  * @brief      这里是折叠全向哨兵机器人参数配置文件，包括物理参数、PID参数等
  * @history
  *  Version    Date            Author          Modification
  *  V1.1.0     2024-11-3     Harry_Wong        1. 完成云台所有基本控制
  *  V1.1.1     2025-1-15     CJH               1. 完成射击基本功能
  *  V1.1.2     2025-11-05    CJH               1. 完成底盘基本功能
  *                                             2. 完善参数   
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_OMNI_WHEEL         // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_YAW_PITCH_DIRECT      // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC_TRIGGER            // 选择发射机构类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL          // 选择控制类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE  //选择机械臂类型 


//RC parametes ---------------------
//遥控器相关参数
#define CHASSIS_RC_DEADLINE (5.0f)      // 摇杆死区
#define CHASSIS_RC_MAX_RANGE (660.0f)   //遥控器最大量程

/*-------------------- Gimbal --------------------*/
//云台电流发送参数
#define GIMBAL_CAN (2)
#define GIMBAL_STDID (0x1FF)

//gimbal_init-------------------------------
#define GIMBAL_INIT_TIME (uint32_t)1000

//mouse sensitivity ---------------------
#define MOUSE_SENSITIVITY (40000.0f)
#define MOUSE_SENSITIVITY_PITCH (38000.0f)//200000.0这里是这个值越大，越不灵敏10000偏快。 灵敏度的正负会影响PITCH轴旋转方向
#define MOUSE_SENSITIVITY_YAW (21000.0f)
//remote controller sensitivity ---------------------
#define REMOTE_CONTROLLER_SENSITIVITY (100000.0f)
#define REMOTE_CONTROLLER_MAX_DEADLINE (20.0f)
#define REMOTE_CONTROLLER_MIN_DEADLINE (-20.0f)
//motor parameters ---------------------
//电机id
#define GIMBAL_DIRECT_YAW_ID ((uint8_t)3)   //大yaw
#define GIMBAL_DIRECT_PITCH_ID ((uint8_t)4) //pitch
#define GIMBAL_DIRECT_FOLD_ID ((uint8_t)1)  //折叠
#define GIMBAL_DIRECT_SMALL_YAW_ID ((uint8_t)2) //小yaw

//电机can口
#define GIMBAL_DIRECT_YAW_CAN ((uint8_t)1)
#define GIMBAL_DIRECT_PITCH_CAN ((uint8_t)1)
#define GIMBAL_DIRECT_FOLD_CAN ((uint8_t)1)
#define GIMBAL_DIRECT_SMALL_YAW_CAN ((uint8_t)2)

//电机种类
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE ((MotorType_e)DM_4310)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE ((MotorType_e)DM_4310)
#define GIMBAL_DIRECT_FOLD_MOTOR_TYPE ((MotorType_e)DM_4310)
#define GIMBAL_DIRECT_SMALL_YAW_MOTOR_TYPE ((MotorType_e)DJI_M6020)

//旋转方向
#define GIMBAL_DIRECT_YAW_DIRECTION (1)
#define GIMBAL_DIRECT_PITCH_DIRECTION (-1)
#define GIMBAL_DIRECT_FOLD_DIRECTION (1)
#define GIMBAL_DIRECT_SMALL_YAW_DIRECTION (1)

//减速比
#define GIMBAL_DIRECT_YAW_REDUCTION_RATIO (10)
#define GIMBAL_DIRECT_PITCH_REDUCTION_RATIO (10)
#define GIMBAL_DIRECT_FOLD_REDUCTION_RATIO (10)
#define GIMBAL_DIRECT_SMALL_YAW_REDUCTION_RATIO (1)

//电机运行模式
#define GIMBAL_DIRECT_YAW_MODE (DM_MODE_MIT)
#define GIMBAL_DIRECT_PITCH_MODE (DM_MODE_MIT)
#define GIMBAL_DIRECT_FOLD_MODE (DM_MODE_MIT)
#define GIMBAL_DIRECT_SMALL_YAW_MODE (0)

//physical parameters ---------------------
#define GIMBAL_UPPER_LIMIT_PITCH (0.256f)//数据源IMU角度  
#define GIMBAL_LOWER_LIMIT_PITCH (-0.225f)//数据源IMU角度
#define GIMBAL_UPPER_LIMIT_SMALL_YAW (0.80f)//编码器角度
#define GIMBAL_LOWER_LIMIT_SMALL_YAW (-2.43f)//编码器角度

//电机角度中值设置
#define GIMBAL_DIRECT_PITCH_MID (-2.697f)  //云台初始化正对齐的时候使用的pitch轴正中心量（编码器角度）
#define GIMBAL_DIRECT_YAW_MID  (-1.693f)    //云台初始化正对齐的时候使用的yaw轴正中心量（编码器角度，给云台跟随用的）
#define GIMBAL_DIRECT_SMALL_YAW_MID (-0.77f)  //小yaw中间初始角度（编码器角度）
#define GIMBAL_DIRECT_FOLD_UP   (1.141f)  //fold抬头展开角度（编码器角度）
#define GIMBAL_DIRECT_FOLD_DOWN (-1.025f)  //fold低头收起角度（编码器角度）

//PID parameters ---------------------
//YAW ANGLE
#define KP_GIMBAL_SMALL_YAW_ANGLE (15.0f)
#define KI_GIMBAL_SMALL_YAW_ANGLE (0.0f)
#define KD_GIMBAL_SMALL_YAW_ANGLE (0.0f)
#define MAX_IOUT_GIMBAL_SMALL_YAW_ANGLE (1.0f)
#define MAX_OUT_GIMBAL_SMALL_YAW_ANGLE (15.0f)
//VELOCITY:角速度
#define KP_GIMBAL_SMALL_YAW_VELOCITY (402.52f)
#define KI_GIMBAL_SMALL_YAW_VELOCITY (50.0f)
#define KD_GIMBAL_SMALL_YAW_VELOCITY (0.0f)
#define MAX_IOUT_GIMBAL_SMALL_YAW_VELOCITY (15000.0f)
#define MAX_OUT_GIMBAL_SMALL_YAW_VELOCITY (30000.0f)



/*-------------------- Shoot --------------------*/
//physical parameters ---------------------
#define FRIC_RADIUS 0.059f              // (m)摩擦轮半径
#define BULLET_NUM 8                   // 定义拨弹盘容纳弹丸个数
#define GUN_NUM 1                      // 定义枪管个数（一个枪管2个摩擦轮）
#define TRIGGER_REDUCTION_RATIO 1.0f   // 定义英雄电机到拨弹盘的齿轮减速比

/*MOTOR paramters --------------------*/

//电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DJI_M2006)
#define FRIC_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//电机ID
#define TRIGGER_MOTOR_ID 3
#define FRIC_MOTOR_R_ID 1
#define FRIC_MOTOR_L_ID 4

//电机can口
#define TRIGGER_MOTOR_CAN 2
#define FRIC_MOTOR_R_CAN 2
#define FRIC_MOTOR_L_CAN 2

//电机std_id
#define STD_ID 0x200
//单环拨弹速度
#define TRIGGER_SPEED               (350.0f)//(700.0f)
//摩擦轮速度
#define FRIC_R_SPEED                (650.0f)  //(686.0f) 
#define FRIC_L_SPEED                (-650.0f) //(-686.0f) 
#define FRIC_SPEED_LIMIT            (600.0f) 

/*ECD parameters------------*/
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191

//电机rpm 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18

/*BLOCK&REVERSE parameters------------*/

//初版   看门狗防堵转
#define BLOCK_TRIGGER_SPEED         5.0f
#define BLOCK_TIME                  1000
#define REVERSE_TIME                1250
#define REVERSE_SPEED               (-20.0f) 

/*MIT parameters ---------------------*/

#define TRIGGER_SPEED_MIT_KD (0.0f)  //  (none)

/*PID parameters ---------------------*/

//拨弹轮电机PID速度环
#define TRIGGER_SPEED_PID_KP (50.0f)


#define TRIGGER_SPEED_PID_KI (0.01f)
#define TRIGGER_SPEED_PID_KD (0.02f)

#define TRIGGER_SPEED_PID_MAX_OUT (10000.0f)
#define TRIGGER_SPEED_PID_MAX_IOUT (1000.0f)

//拨弹轮电机PID角度环
#define TRIGGER_ANGEL_PID_KP (300.0f)
#define TRIGGER_ANGEL_PID_KI (0.5f)
#define TRIGGER_ANGEL_PID_KD (0.05f)

#define TRIGGER_ANGEL_PID_MAX_OUT (800.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (300.0f)

//摩擦轮电机PID
#define FRIC_SPEED_PID_KP (680.0f)//(651.0f)
#define FIRC_SPEED_PID_KI  (0.05f)//(0.08f)
#define FRIC_SPEED_PID_KD  (3.0f)//(4.3f)

#define FRIC_PID_MAX_OUT (16000.0f)
#define FRIC_PID_MAX_IOUT (1000.0f)

//枪口热量阈值
#define SHOOT_HEAT_REMAIN_VALUE     60

#endif /* INCLUDED_ROBOT_PARAM_H */

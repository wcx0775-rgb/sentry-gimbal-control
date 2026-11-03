/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_yaw_pitch.c/h
  * @brief      yaw_pitch云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024-04-01      Penguin         1. done
  *  V1.2.0     2025-02-26      Harry_Wong      1.删除了不需要的变量
  *                                             2.剩余更新详情请见.c文件
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "robot_param.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
#ifndef GIMBAL_YAW_PITCH_H
#define GIMBAL_YAW_PITCH_H
#include "IMU.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "user_lib.h"
#include "CAN_cmd_dji.h"
#include "detect_task.h"
#include "cmsis_os.h"
#include "can_receive.h"
#include "math.h"
#include "macro_typedef.h"
#include "gimbal.h"
#include "CAN_cmd_damiao.h"
#include "VOFA.h"
#include "usb_task.h"
#include "LQR.h"
#include "signal_generator.h"


#define GIMBAL_SMALL_YAW_ANGLE_PERIOD_MS (2)   // 角度环周期，单位 ms（1ms 的整数倍）
#define GIMBAL_FOLD_CONNECTED    (0)        //0=没接折叠电机，1=已接

/* ==================== 云台控制开关 ==================== */
// 4310(大yaw/pitch/折叠) 用 LQR 力矩；6020(小yaw) 永远走双环 PID 电压

/* LQR 增益：MATLAB dlqr 输出后填入（占位值！） */
#define K1_LQR_YAW    (-2.839323f)  /* TODO */
#define K2_LQR_YAW    (0.113066f)   /* TODO */
#define KI_LQR_YAW    (-0.028382f)   /* TODO */

#define K1_LQR_PITCH  (-1.546450f)  /* TODO */
#define K2_LQR_PITCH  (0.095673f)   /* TODO */
#define KI_LQR_PITCH  (-0.10463f)   /* TODO */

#define K1_LQR_FOLD   (-1.546450f)  /* TODO */
#define K2_LQR_FOLD   (0.095673f)   /* TODO */
#define KI_LQR_FOLD   (-0.10463f)   /* TODO */

/* 输出软保护（电机在上位机限幅为主，这里设成一致或略大） */
#define MAX_OUT_LQR_YAW   (3.0f)
#define MAX_OUT_LQR_PITCH (3.0f)
#define MAX_OUT_LQR_FOLD  (3.0f)
/* 积分限幅：建议 0.3~1.0 * (max_out/ki) */
#define MAX_IOUT_LQR_YAW   (1.0f)  /* TODO */
#define MAX_IOUT_LQR_PITCH (1.0f)  /* TODO */
#define MAX_IOUT_LQR_FOLD  (1.0f)  /* TODO */

#define LQR_GIMBAL_DT (0.001f)  /* 1kHz，与 gimbal_task.c 一致 */


/* 折叠轴前馈补偿参数 */
#define FOLD_MASS              (0.5f)   /* 折叠负载质量 kg，实测填 */
#define FOLD_ARM_LENGTH        (0.10f)  /* 质心到折叠轴距离 m，实测填 */
#define FOLD_ANGLE_HORIZONTAL  (0.0f)   /* 轴水平时 fold.fdb.pos 读数，实测填 */
#define FRICTION_COULOMB_FOLD  (0.0f)   /* 库仑摩擦 N·m，有卡顿再调 */
#define FRICTION_VISCOUS_FOLD  (0.0f)   /* 粘性摩擦 N·m/(rad/s) */

/* pitch 前馈参数（pitch 用 IMU 角度，水平位就是 IMU 读数 0 附近） */
#define PITCH_MASS              (0.8f)
#define PITCH_ARM_LENGTH        (0.12f)
#define PITCH_ANGLE_HORIZONTAL  (0.0f)
#define PITCH_FRIC_COULOMB      (0.0f)
#define PITCH_FRIC_VISCOUS      (0.0f)

/**
 * @brief 云台模式
 */
typedef enum {
    GIMBAL_ZERO_FORCE,  // 云台无力，所有控制量置0
    GIMBAL_IMU,         // 云台陀螺仪控制(角度控制)
    GIMBAL_INIT,        //云台矫正模式
    GIMBAL_DBUS_ERR,    //遥控器断联相关处理任务
    GIMBAL_GAP,         //跳出矫正进入IMU/AUTO_AIM模式之前的存储数据模式
    GIMBAL_AUTO_AIM,    //自瞄模式
    GIMBAL_MOTOR_TEST,  //电机测试模式
} GimbalMode_e;

/**
 * @brief 状态、期望和限制值
 */
typedef struct
{
    float pitch;
    float yaw;
    float fold;       /* 折叠轴 */
    float small_yaw;  /* 小yaw轴 */
} Values_t;

typedef struct
{
    pid_type_def small_yaw_angle;     /* 小yaw 角度环 */
    pid_type_def small_yaw_velocity;  /* 小yaw 速度环 */
} PID_t;

//新增dm——yaw     汪辰旭 1.17
typedef struct feedback
{
  fp32 yaw_angel_fdb;// 拨弹盘输出轴位置
  fp32 yaw_speed_fdb;// 拨弹盘输出轴速度
} Fdb;

typedef struct
{
    const RC_ctrl_t * rc;  // 遥控器指针
    GimbalMode_e mode,last_mode,mode_before_rc_err;  // 模式

    /*-------------------- Motors --------------------*/
    Motor_s yaw,pitch,fold,small_yaw;  // 电机结构体
	
	Fdb FDB;				//新增 		汪辰旭 1.17
    /*-------------------- Values --------------------*/
    Values_t reference;    // 期望值
    Values_t feedback_pos,feedback_vel;     // 状态值(目前专供给IMU数据)
    Values_t upper_limit;  // 上限值
    Values_t lower_limit;  // 下限值
	Values_t vision_reference;         // 视觉下发的目标值缓存
	
	/*-------------------- Vision Status --------------------*/
    bool vision_control_enabled;       // 视觉是否接管
    uint8_t shoot_cmd;                 // 射击指令 (0:不射, 1:射击)
    int16_t target_distance;           // 目标距离

    Values_t init_base;    //初始上电的imu  
    bool init_base_record; //是否需要记录imu初始位置 

    PID_t pid;  // PID控制器
//	PID_t aim_pid;                     // 自瞄专用PID (可选)

    float angle_zero_for_imu; //pitch电机处于中值时imupitch的角度

    uint32_t init_start_time,init_timer;

    bool init_continue; //是否继续进行校准模式
} Gimbal_s;

extern Gimbal_s gimbal_direct;
extern Lqr_t gimbal_lqr_yaw;
extern Lqr_t gimbal_lqr_pitch;
extern Lqr_t gimbal_lqr_fold;
extern PID_t gimbal_small_yaw_pid;

extern void GimbalInit(void);

extern void GimbalHandleException(void);

extern void GimbalObserver(void);

extern void GimbalReference(void);

extern void GimbalConsole(void);

extern void GimbalSendCmd(void);

#endif  // GIMBAL_YAW_PITCH_H
#endif  // GIMBAL_YAW_PITCH





/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      增强版PID - 支持多种优化算法组合
  * @note       通过配置标志位选择不同特性，保持代码统一
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V2.0.0     2026-09-10      汪辰旭           2. 添加高级特性
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"

/* ============ PID模式定义 ============ */

/* PID基础模式 */
enum PID_MODE {
    PID_POSITION = 0,      //位置式PID
    PID_DELTA,             //增量式PID
};

/* 微分结构类型 */
enum PID_DERIVATIVE_MODE {
    DERIVATIVE_ON_ERROR = 0,    //标准PID：D作用在误差上
    DERIVATIVE_ON_MEASUREMENT,   //改进PID：D作用在测量值上
};

/* 积分算法类型 */
enum PID_INTEGRAL_MODE {
    INTEGRAL_RECTANGULAR = 0,   //矩形积分
    INTEGRAL_TRAPEZOIDAL,       //梯形积分
};

/* 特性标志位（可按位或组合） */
#define PID_FEATURE_NONE                0x00  //无特性
#define PID_FEATURE_INTEGRAL_SEPARATION 0x01  //积分分离
#define PID_FEATURE_ANTI_WINDUP         0x02  //积分限幅(抗饱和)
#define PID_FEATURE_2DOF                0x04  //二自由度PID
#define PID_FEATURE_FEEDFORWARD         0x08  //前馈控制
#define PID_FEATURE_OUTPUT_FILTER       0x10  //输出低通滤波

/* ============ PID结构体 ============ */

typedef struct
{
    /* 基础参数 */
    uint8_t mode;                 //PID模式
    fp32 Kp, Ki, Kd;              //PID三参数
    fp32 N;                       //微分滤波系数[0,1]

    fp32 max_out;                 //最大输出
    fp32 max_iout;                //最大积分输出

    fp32 set, fdb;                //设定值，反馈值
    fp32 out;                     //输出值
    fp32 Pout, Iout, Dout;        //PID输出分量

    fp32 error[3];                //误差历史值[0]最新
    fp32 Dbuf[3];                 //微分缓冲 [0]最新

    /* 算法配置 */
    uint8_t derivative_mode;       //微分模式
    uint8_t integral_mode;         //积分模式
    uint16_t features;             //特性标志位

    /* 积分分离 */
    fp32 integral_separation_threshold;  //积分分离阈值

    /* 抗饱和 */
    fp32 Kb;                       //抗饱和系数

    /* 二自由度PID */
    fp32 b;                       //设定值P权重[0,1]
    fp32 c;                       //设定值D权重[0,1]
    fp32 set_last;                //上次设定值  

    /* 前馈 */
    fp32 Kf;                      //前馈增益
    
    /* 输出滤波 */
    fp32 output_filter_coef;      //输出滤波系数[0,1]
    fp32 out_last;                 //上次输出值

} pid_type_def;




/* ============ 函数声明 ============ */

/**
  * @brief          标准PID初始化（向后兼容）
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION 或 PID_DELTA
  * @param[in]      PID: 参数数组 {Kp, Ki, Kd}
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
extern void PID_init(
    pid_type_def * pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          增强PID初始化
  * @param[out]     pid: PID结构指针
  * @param[in]      mode: PID_POSITION 或 PID_DELTA
  * @param[in]      PID: 参数数组 {Kp, Ki, Kd}
  * @param[in]      max_out: 最大输出
  * @param[in]      max_iout: 最大积分输出
  * @param[in]      derivative_mode: 微分模式
  * @param[in]      integral_mode: 积分模式
  * @param[in]      features: 特性标志位组合
  */
extern void PID_init_ex(
    pid_type_def *pid, 
    uint8_t mode, 
    const fp32 PID[3], 
    fp32 max_out, 
    fp32 max_iout,
    uint8_t derivative_mode,
    uint8_t integral_mode,
    uint16_t features);


/**
  * @brief          设置微分滤波系数
  * @param[out]     pid: PID结构指针
  * @param[in]      N: 滤波系数 [0,1]，0=不滤波，越大越平滑
  */
extern void PID_set_derivative_filter(pid_type_def *pid, fp32 N);

/**
  * @brief          设置积分分离阈值
  * @param[out]     pid: PID结构指针
  * @param[in]      threshold: 误差阈值，|error| > threshold时停止积分
  */
extern void PID_set_integral_separation(pid_type_def *pid, fp32 threshold);

/**
  * @brief          设置抗饱和系数
  * @param[out]     pid: PID结构指针
  * @param[in]      Kb: 抗饱和系数，建议0.3~1.0
  */
extern void PID_set_anti_windup(pid_type_def *pid, fp32 Kb);

/**
  * @brief          设置二自由度PID参数
  * @param[out]     pid: PID结构指针
  * @param[in]      b: 设定值比例权重 [0,1]
  * @param[in]      c: 设定值微分权重 [0,1]
  */
extern void PID_set_2dof(pid_type_def *pid, fp32 b, fp32 c);

/**
  * @brief          设置前馈增益
  * @param[out]     pid: PID结构指针
  * @param[in]      Kf: 前馈增益
  */
extern void PID_set_feedforward(pid_type_def *pid, fp32 Kf);

/**
  * @brief          设置输出滤波系数
  * @param[out]     pid: PID结构指针
  * @param[in]      coef: 滤波系数 [0,1]，越大越平滑
  */
extern void PID_set_output_filter(pid_type_def *pid, fp32 coef);

/**
  * @brief          PID计算（统一接口，根据配置自动选择算法）
  * @param[out]     pid: PID结构指针
  * @param[in]      ref: 反馈值
  * @param[in]      set: 设定值
  * @retval         PID输出
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          PID输出清零
  * @param[out]     pid: PID结构指针
  */
extern void PID_clear(pid_type_def *pid);

#endif
/*------------------------------ End of File ------------------------------*/

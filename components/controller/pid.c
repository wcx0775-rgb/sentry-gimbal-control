/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      增强版PID实现 - 统一接口，多种算法组合，
  * @note       根据配置标志位自动选择优化算法
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V2.0.0     2026-09-10      汪辰旭          2. 添加高级特性 
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"

#include "main.h"

#include <math.h>
#include <stdbool.h>

#define LimitMax(input, max)       \
    {                              \
        if (input > max) {         \
            input = max;           \
        } else if (input < -max) { \
            input = -max;          \
        }                          \
    }

/* ============ 初始化函数 ============ */

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL) return;
    
    // 基础参数
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->N = 0.0f;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    
    // 清零
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->set = pid->fdb = 0.0f;
    
    // 默认配置：标准PID，无特殊特性
    pid->derivative_mode = DERIVATIVE_ON_ERROR;
    pid->integral_mode = INTEGRAL_RECTANGULAR;
    pid->features = PID_FEATURE_NONE;
    
    // 扩展参数默认值
    pid->integral_separation_threshold = 0.0f;
    pid->Kb = 0.0f;
    pid->b = 1.0f;
    pid->c = 1.0f;
    pid->set_last = 0.0f;
    pid->Kf = 0.0f;
    pid->output_filter_coef = 0.0f;
    pid->out_last = 0.0f;
}

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
 void PID_init_ex(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout,
    uint8_t derivative_mode, uint8_t integral_mode, uint16_t features)
{
    // 先调用标准初始化
    PID_init(pid, mode, PID, max_out, max_iout);

    // 设置扩展参数
    pid->derivative_mode = derivative_mode;
    pid->integral_mode = integral_mode;
    pid->features = features;
}

/* ============ 配置函数 ============ */

/*
  以下这些配置函数的注释在.h文件中已经详细说明，这里不再赘述。
  它们主要用于设置PID的高级特性，如微分滤波、积分分离、抗饱和、二自由度PID、前馈增益和输出滤波等。
  这些函数会直接修改pid结构体中的相关参数，以便在PID计算过程中使用。
*/

void PID_set_derivative_filter(pid_type_def *pid, fp32 N)
{
    if (pid == NULL) return;
    pid->N = N;
}

void PID_set_integral_separation(pid_type_def *pid, fp32 threshold)
{
    if (pid == NULL) return;
    pid->integral_separation_threshold = threshold;
    pid->features |= PID_FEATURE_INTEGRAL_SEPARATION;
}

void PID_set_anti_windup(pid_type_def *pid, fp32 Kb)
{
    if (pid == NULL) return;
    pid->Kb = Kb;
    pid->features |= PID_FEATURE_ANTI_WINDUP;
}

void PID_set_2dof(pid_type_def *pid, fp32 b, fp32 c)
{
    if (pid == NULL) return;
    pid->b = b;
    pid->c = c;
    pid->features |= PID_FEATURE_2DOF;
}

void PID_set_feedforward(pid_type_def *pid, fp32 Kf)
{
    if (pid == NULL) return;
    pid->Kf = Kf;
    pid->features |= PID_FEATURE_FEEDFORWARD;
}

void PID_set_output_filter(pid_type_def *pid, fp32 coef)
{
    if (pid == NULL) return;
    pid->output_filter_coef = coef;
    pid->features |= PID_FEATURE_OUTPUT_FILTER;
}

/* ============ PID计算核心函数 ============ */

/**
  * @brief          统一PID计算接口
  * @note           根据配置自动选择算法组合
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL) return 0.0f;

    //更新误差历史
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = set - ref;
    pid->set = set;
    // pid->fdb = ref;

    /* ========== 位置式PID ========== */
    if (pid->mode == PID_POSITION)
    {
        /* --- 1. 比例项 --- */
        if(pid->features & PID_FEATURE_2DOF)
        {
            // 二自由度：P = Kp * (b*set - ref)
            pid->Pout = pid->Kp * (pid->b * set - ref);
        }
        else
        {
            // 标准：P = Kp * error
            pid->Pout = pid->Kp * pid->error[0];
        }

        /* --- 2. 积分项 --- */
        fp32 integral_increment = 0.0f;

        // 检查积分分离
        bool should_integrate = true;
        if(pid->features & PID_FEATURE_INTEGRAL_SEPARATION)
        {
            if(fabsf(pid->error[0]) > pid->integral_separation_threshold)
            {
                should_integrate = false;
            }
        }

        if(should_integrate)
        {
            // 选择积分算法
            if(pid->integral_mode == INTEGRAL_TRAPEZOIDAL)
            {
                // 梯形积分
                integral_increment = pid->Ki * (pid->error[0] + pid->error[1]) * 0.5f;
            }
            else
            {
                // 矩形积分
                integral_increment = pid->Ki * pid->error[0];
            }

            // 抗饱和
            // if(pid->features & PID_FEATURE_ANTI_WINDUP)
            // {
            //     // 计算饱和误差（实际输出 - 理想输出）
            //     fp32 ideal_out = pid->Pout + pid->Iout + pid->Dout;
            //     fp32 set_error = pid->out - ideal_out;
            //     integral_increment += pid->Kb * set_error;
            // }

            pid->Iout += integral_increment;
            LimitMax(pid->Iout, pid->max_iout);
        }

        /* --- 3. 微分项 --- */
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];

        if(pid->derivative_mode == DERIVATIVE_ON_MEASUREMENT)
        {
            // I-PD结构：微分作用在测量值上
            if(pid->features & PID_FEATURE_2DOF)
            {
                // 二自由度PID + 微分作用在测量值：D = Kd * (c*d(set)/dt - d(fdb)/dt)
                fp32 set_diff = pid->c * (set - pid->set_last);
                fp32 fdb_diff = ref - pid->fdb;
                fp32 diff_input = set_diff - fdb_diff;
                pid->Dbuf[0] = diff_input;
            }
            else
            {
                // 标准I-PD：D = -Kd * d(fdb)/dt
                fp32 fdb_diff = ref - pid->fdb;
                pid->Dbuf[0] = -fdb_diff;
            }
        }
        else 
        {
             // 微分作用在误差上（标准结构）
            if(pid->features & PID_FEATURE_2DOF)
            {
                // 二自由度PID：D = Kd * (c*d(set)/dt - d(error)/dt)
                fp32 set_diff = pid->c * (set - pid->set_last);
                fp32 error_diff = pid->error[0] - pid->error[1];
                pid->Dbuf[0] = set_diff - error_diff;
            }
        
            else
            {
                // 标准PID：D = Kd * d(error)/dt
                pid->Dbuf[0] = pid->error[0] - pid->error[1];
            }
        }

        // 微分滤波
        if(pid->N > 0.0f)
        {
            pid->Dbuf[0] = pid->N * pid->Dbuf[1] + (1.0f - pid->N) * pid->Dbuf[0];
        }

        pid->Dout = pid->Kd * pid->Dbuf[0];

        /* --- 4. 前馈项 --- */
        fp32 feedforward = 0.0f;
        if(pid->features & PID_FEATURE_FEEDFORWARD)
        {
            feedforward = pid->Kf * set;
        }

        /* --- 5. 总输出 --- */
        pid->out = pid->Pout + pid->Iout + pid->Dout + feedforward;
        fp32 out_unsat = pid->out;              // 限幅前的理想输出
        LimitMax(pid->out, pid->max_out);
        // 抗饱和：输出被限幅时，把积分往回拉（下一周期生效）
        if (pid->features & PID_FEATURE_ANTI_WINDUP)
        {
            pid->Iout += pid->Kb * (pid->out - out_unsat);
            LimitMax(pid->Iout, pid->max_iout);
        }

        // 输出滤波
        if(pid->features & PID_FEATURE_OUTPUT_FILTER)
        {
            pid->out = pid->output_filter_coef * pid->out_last + (1.0f - pid->output_filter_coef) * pid->out;
        }

        pid->out_last = pid->out;
        pid->set_last = set;

    }
    /* ========== 增量式PID ========== */
    else if(pid->mode == PID_DELTA)
    {
        // 增量式：简化版，主要用于电机位置控制
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];

        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];

        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);

        pid->set_last = set;
    }
    pid->fdb = ref;   // 存本周期测量值，下一周期微分用 ref - fdb = 本周期 - 上一周期
    return pid->out;

}

/**
  * @brief          PID输出清零
  */
void PID_clear(pid_type_def *pid)
{
    if(pid == NULL) return;

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
    pid->set_last = 0.0f;
    pid->out_last = 0.0f;
}




/*------------------------------ End of File ------------------------------*/

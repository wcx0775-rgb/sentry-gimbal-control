/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_yaw_pitch.c/h
  * @brief      yaw_pitch云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *   V1.1.0    2024-11-3     Harry_Wong        1. 完成云台所有基本控制
  *   V1.1.1    2024-11-11    Harry_Wong        1.为云台随动添加了yaw轴偏转角度的API
  *   V1.1.2    2024-11-25    Harry_Wong        1.云台模式设置逻辑重构，准备函数给底盘表明是否处于初始化模式
  *   V1.1.3    2024-12-16    Harry_Wong        1.云台的imu获取方式被更改为函数传递，防止Subcribe（）函数停用造成影响 
                                                2.云台在遥控器断联情况下直接发送0电流，防止赛场上出现意外情况影响稳定性
  *   V1.2.0    2025-2-25     Harry_Wong        1.向上位机发送数据进行了优化，采用了电机角度差值控制：（电机实际角度 - 电机中值角度）* 电机旋转方向 （距离中间的偏移角度）
                                                2.添加了Gimbal_direct_ecd_to_imu（）函数 接受上位机返回的目标角度，并且跟发送角度做差得到角度期望偏移值并映射成为云台imu的期望角度值
                                                3. Reference（） AUTO_AIM 模式下的reference计算方法进行了更改：将返回值进行Gimbal_direct_ecd_to_imu（）运算得到结果作为云台的目标角度
                                                4. 删除了 imu_base 变量 ，现在云台已经不需要在第一次校准的时候记录imu初始值了
                                                5.Console() INIT 模式下的PID计算优化：所有云台的旋转计算参考值已经更改为imu相关数据，尽可能的减轻调教负担
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/

#include "gimbal_yaw_pitch_direct.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
Gimbal_s gimbal_direct;
Lqr_t gimbal_lqr_yaw;
Lqr_t gimbal_lqr_pitch;
Lqr_t gimbal_lqr_fold;
PID_t gimbal_small_yaw_pid;
first_order_filter_type_t small_yaw_vel_filter;
static uint16_t small_yaw_angle_cnt = 0;  // 小yaw角度环分频计数(1ms计数)
// IMU模式子状态机
typedef enum {
    IMU_FOLDING = 0,   // fold抬头中，其它轴保持
    IMU_RECENTERING,   // fold到位，大/小yaw回中
    IMU_READY,         // 回中完成，大yaw/pitch可遥控
} GimbalImuState_e;
static GimbalImuState_e gimbal_imu_state = IMU_FOLDING;

/*--------------------------------Internal functions---------------------------------------*/
/**以下函数均不会被外部调用，请注意！**/

/*----------------angle_solution--------------------*/

/**
 * @brief          解算imu和电机角度的差值（用于更新云台的限位范围）,确认新的电机中值对应的imu值
 * @param[in]      none
 * @retval         none
 */
void Angle_solution(void)
{
	float motor_feedback=gimbal_direct.pitch.fdb.pos;
	float imu_feedback=gimbal_direct.feedback_pos.pitch;
	float motor_mid=GIMBAL_DIRECT_PITCH_MID;
	float imu_mid=0.0;
	float motor_delta=GIMBAL_DIRECT_PITCH_DIRECTION*(motor_feedback-motor_mid);
	float imu_delta=imu_feedback-imu_mid;
	gimbal_direct.angle_zero_for_imu=imu_delta-motor_delta;
}

/*----------------Gimbal_direct_init_judge--------------------*/
/**
 * @brief          判断是否需要继续初始化云台校准
 * @param[in]      none
 * @retval         bool 解释是否需要继续初始化
 */
bool Gimbal_direct_init_judge (void)
{
	if ( ((gimbal_direct.reference.yaw-gimbal_direct.yaw.fdb.pos<0.003f && (-0.003f)<gimbal_direct.reference.yaw-gimbal_direct.yaw.fdb.pos) && (gimbal_direct.reference.pitch-gimbal_direct.pitch.fdb.pos<0.003f && (-0.003f)<gimbal_direct.reference.pitch-gimbal_direct.pitch.fdb.pos) ) || gimbal_direct.init_timer>=GIMBAL_INIT_TIME )
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * @brief          云台数据发布（重写weak函数）
 * @param[in]      none
 * @retval         none
 */
void GimbalPublish(void)
{
	//发送遥控器数据
	CanSendRcDataToBoard(1,BoardC1 ,0);
	//发送键鼠数据
	CanSendRcDataToBoard(1,BoardC1 ,1);
	//发送云台数据
	CanSendGimbalDataToBoard(1,BoardC1 ,0);
	//发送雷达数据
	CanSendNaviDataToBoard(1,BoardC1 ,0);
}

/**
 * @brief  重力 + 摩擦前馈补偿（通用，fold/pitch 共用）
 * @param  pos 轴角度 (rad)
 * @param  vel 轴角速度 (rad/s)
 * @param  mass 负载质量 (kg)
 * @param  arm_length 质心到轴距离 (m)
 * @param  angle_horizontal 轴水平时的角度读数 (rad)
 * @param  fric_coulomb 库仑摩擦 (N·m)
 * @param  fric_viscous 粘性摩擦 (N·m/(rad/s))
 * @return 前馈力矩 (N·m)
 */
static fp32 GravityFrictionComp(fp32 pos, fp32 vel, fp32 mass, fp32 arm_length, fp32 angle_horizontal, fp32 fric_coulomb, fp32 fric_viscous)
{
	fp32 tau = 0.0f;
	tau += mass * GRAVITY * arm_length * cosf(pos - angle_horizontal);  // 重力补偿

	// 库仑摩擦补偿
	if (vel > 0.01f)
	{
		tau += fric_coulomb;
	}
	else if (vel < -0.01f)
	{
		tau -= fric_coulomb;
	}
	tau += fric_viscous * vel;  // 粘性摩擦补偿

	return tau;
}

/*----------------Gimbal_direct_ecd_to_imu--------------------*/
/**
 * @brief          ecd角度值转换成imu角度值
 * @param[in]      axis 用于知道读取哪一个轴的角度转换
 * @param[in]      value 用于准换的值
 * @retval         float imu映射角度值
 */
float Gimbal_direct_ecd_to_imu(uint8_t axis,float value)
{
	if (axis == AX_PITCH)
	{
		return value - CmdGimbalJointState(AX_PITCH) + gimbal_direct.feedback_pos.pitch;
	}
	else if(axis == AX_YAW)
	{
		return value - CmdGimbalJointState(AX_YAW) + gimbal_direct.feedback_pos.yaw;
	}
	else 
	{
		return 0.0f;
	}
}

/**
 * @brief          从USB任务获取视觉数据并处理 (核心函数)
 */
void Gimbal_direct_process_vision_data(void)
{
    // 1. 调用 usb_task.c 提供的接口获取解析后的数据结构
    VisionCmdData_t vision_data = GetVisionCmdData();
    
    // 2. 更新控制标志位
    // control_mode: 0=不控制, 1=控制
    gimbal_direct.vision_control_enabled = (vision_data.control_mode > 0);
    
    // 3. 更新射击指令 (0xCC帧中的 shoot 字段)
    // 兼容两种逻辑：control_mode==2 或者 shoot_cmd > 0
    if (vision_data.control_mode == 2 || vision_data.shoot_cmd > 0) {
        gimbal_direct.shoot_cmd = 1;
    } else {
        gimbal_direct.shoot_cmd = 0;
    }
    
    // 4. 更新距离 (0xCC帧中的 distance 字段)
    gimbal_direct.target_distance = vision_data.distance;
    
    // 5. 更新目标角度 (usb_task已经将int16转换为float弧度)
    gimbal_direct.vision_reference.yaw = vision_data.yaw;
    gimbal_direct.vision_reference.pitch = vision_data.pitch;
}

/**
 * @brief          自瞄模式控制逻辑
 */
void Gimbal_direct_auto_aim_control(void)
{
    Gimbal_direct_process_vision_data();

    static float search_tick = 0;
    if (gimbal_direct.last_mode != GIMBAL_AUTO_AIM) search_tick = 0;

	gimbal_direct.reference.fold = GIMBAL_DIRECT_FOLD_UP;

    if (gimbal_direct.vision_control_enabled)
    {
        // --- Pitch：相对角 → 当前IMU角 + 相对角 = 绝对目标 ---
        gimbal_direct.reference.pitch = fp32_constrain(
            gimbal_direct.feedback_pos.pitch + gimbal_direct.vision_reference.pitch,
            gimbal_direct.lower_limit.pitch + gimbal_direct.angle_zero_for_imu,
            gimbal_direct.upper_limit.pitch + gimbal_direct.angle_zero_for_imu);

        // --- Yaw：在大yaw / 小yaw 之间分配 ---
        float yaw_err = gimbal_direct.vision_reference.yaw;   // 世界系相对角

        // 若小yaw 吃掉整个误差，编码器会到哪
        float small_desired = gimbal_direct.small_yaw.fdb.pos
                            + gimbal_direct.small_yaw.direction * yaw_err;

        if (small_desired > gimbal_direct.lower_limit.small_yaw &&
            small_desired < gimbal_direct.upper_limit.small_yaw)
        {
            // 小yaw 范围内：小yaw 转，大yaw 保持
            gimbal_direct.reference.small_yaw = loop_fp32_constrain(gimbal_direct.small_yaw.fdb.pos + gimbal_direct.small_yaw.direction * yaw_err, -M_PI, M_PI);
            gimbal_direct.reference.yaw = gimbal_direct.feedback_pos.yaw;
        }
        else
        {
            // 超出小yaw 范围：大yaw 转，小yaw 保持当前
            gimbal_direct.reference.yaw = loop_fp32_constrain(gimbal_direct.feedback_pos.yaw + yaw_err, -M_PI, M_PI);
            gimbal_direct.reference.small_yaw = gimbal_direct.small_yaw.fdb.pos;
        }
    }
	else
    {
        // 视觉没发角度：保持当前角度，不乱动
        gimbal_direct.reference.pitch = gimbal_direct.feedback_pos.pitch;
        gimbal_direct.reference.yaw = gimbal_direct.feedback_pos.yaw;
        gimbal_direct.reference.small_yaw = gimbal_direct.small_yaw.fdb.pos;
    }
}

/*-------------------------The end of internal functions--------------------------------------*/

/* ---------------- GetGimbalDeltaYawMid -------------------- */

/**
 * @brief          (rad) 获取yaw轴和中值的差值
 * @param[in]      none
 * @retval         float
 */
inline float GetGimbalDeltaYawMid(void)
{
	  return loop_fp32_constrain(gimbal_direct.yaw.fdb.pos-GIMBAL_DIRECT_YAW_MID,-M_PI,M_PI);
}

/* ---------------- GetGimbalInitJudgeReturn -------------------- */

/**
 * @brief          对底盘说折叠是否抬起
 * @param[in]      none
 * @retval         bool 折叠电机是否抬到指定位置
 */
inline bool GetGimbalInitJudgeReturn(void)
{
	return (fabsf(gimbal_direct.fold.fdb.pos - GIMBAL_DIRECT_FOLD_UP) < 0.1f);
}

/* --------------------- CmdGimbalJointState ------------------- */

/**
 * @brief          返回云台的imu基准值
 * @param[in]      uint8_t 轴id
 * @retval         云台的基准值返回 （float)
 */
inline float CmdGimbalJointState(uint8_t axis)//以预设固定中点为基准，算相对偏移（含方向修正，固定基准）
{
	if ( axis == AX_PITCH )
	{
		return loop_fp32_constrain(gimbal_direct.pitch.direction * (gimbal_direct.pitch.fdb.pos - GIMBAL_DIRECT_PITCH_MID),-M_PI,M_PI);
	}
	else if ( axis == AX_YAW )
	{
		return loop_fp32_constrain(gimbal_direct.yaw.direction * (gimbal_direct.yaw.fdb.pos - GIMBAL_DIRECT_YAW_MID),-M_PI,M_PI); 
	}
	else 
	{
		return 0.0;
	}
}

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void GimbalInit(void)
{
	//step1 获取所有所需变量指针
	gimbal_direct.rc = get_remote_control_point(); 
	//step2 置零所有值
    gimbal_direct.reference.pitch=0;
    gimbal_direct.reference.yaw=0;
	gimbal_direct.reference.fold=0;
	gimbal_direct.reference.small_yaw=0;
	
	gimbal_direct.feedback_pos.pitch=0;
    gimbal_direct.feedback_pos.yaw=0;
	gimbal_direct.feedback_pos.fold=0;
	gimbal_direct.feedback_pos.small_yaw=0;
	
	gimbal_direct.feedback_vel.pitch=0;
    gimbal_direct.feedback_vel.yaw=0;
	gimbal_direct.feedback_vel.fold=0;
	gimbal_direct.feedback_vel.small_yaw=0;
	
	gimbal_direct.upper_limit.pitch=GIMBAL_UPPER_LIMIT_PITCH;
	gimbal_direct.lower_limit.pitch=GIMBAL_LOWER_LIMIT_PITCH;
	gimbal_direct.upper_limit.small_yaw=GIMBAL_UPPER_LIMIT_SMALL_YAW;
	gimbal_direct.lower_limit.small_yaw=GIMBAL_LOWER_LIMIT_SMALL_YAW;
    //step3 PID数据清零，设置PID参数,以及LQR数据清零，设置LQR参数
	// 3 个 4310：LQR
	const static fp32 lqr_yaw_K[3]   = {K1_LQR_YAW,   K2_LQR_YAW,   KI_LQR_YAW};
    const static fp32 lqr_pitch_K[3] = {K1_LQR_PITCH, K2_LQR_PITCH, KI_LQR_PITCH};
    const static fp32 lqr_fold_K[3]  = {K1_LQR_FOLD,  K2_LQR_FOLD,  KI_LQR_FOLD};	
  
	LQR_init(&gimbal_lqr_yaw,   lqr_yaw_K,   MAX_OUT_LQR_YAW,   MAX_IOUT_LQR_YAW,   LQR_GIMBAL_DT);
    LQR_init(&gimbal_lqr_pitch, lqr_pitch_K, MAX_OUT_LQR_PITCH, MAX_IOUT_LQR_PITCH, LQR_GIMBAL_DT);
    LQR_init(&gimbal_lqr_fold,  lqr_fold_K,  MAX_OUT_LQR_FOLD,  MAX_IOUT_LQR_FOLD,  LQR_GIMBAL_DT);
	
	// 1 个 6020：小yaw 双环 PID（沿用原 pitch 6020 参数）
    const static fp32 small_yaw_angle[3]    = {KP_GIMBAL_SMALL_YAW_ANGLE,    KI_GIMBAL_SMALL_YAW_ANGLE,    KD_GIMBAL_SMALL_YAW_ANGLE};
    const static fp32 small_yaw_velocity[3] = {KP_GIMBAL_SMALL_YAW_VELOCITY, KI_GIMBAL_SMALL_YAW_VELOCITY, KD_GIMBAL_SMALL_YAW_VELOCITY};

   	PID_init_ex(&gimbal_small_yaw_pid.small_yaw_angle, PID_POSITION, small_yaw_angle,
            MAX_OUT_GIMBAL_SMALL_YAW_ANGLE, MAX_IOUT_GIMBAL_SMALL_YAW_ANGLE,
            DERIVATIVE_ON_MEASUREMENT, INTEGRAL_TRAPEZOIDAL, PID_FEATURE_NONE);   // I-PD
	PID_set_output_filter(&gimbal_small_yaw_pid.small_yaw_angle, 0.15f);

	PID_init_ex(&gimbal_small_yaw_pid.small_yaw_velocity, PID_POSITION, small_yaw_velocity,
            MAX_OUT_GIMBAL_SMALL_YAW_VELOCITY, MAX_IOUT_GIMBAL_SMALL_YAW_VELOCITY,
            DERIVATIVE_ON_ERROR, INTEGRAL_TRAPEZOIDAL, PID_FEATURE_NONE);
	PID_set_anti_windup(&gimbal_small_yaw_pid.small_yaw_velocity, 0.1f);
	const static fp32 small_yaw_vel_filter_param[1] = {0.004f};
	first_order_filter_init(&small_yaw_vel_filter, 0.001f, small_yaw_vel_filter_param);
	
	//step4 初始化电机
    MotorInit(&gimbal_direct.yaw,GIMBAL_DIRECT_YAW_ID,GIMBAL_DIRECT_YAW_CAN,GIMBAL_DIRECT_YAW_MOTOR_TYPE,GIMBAL_DIRECT_YAW_DIRECTION,GIMBAL_DIRECT_YAW_REDUCTION_RATIO,GIMBAL_DIRECT_YAW_MODE);
    MotorInit(&gimbal_direct.pitch,GIMBAL_DIRECT_PITCH_ID,GIMBAL_DIRECT_PITCH_CAN,GIMBAL_DIRECT_PITCH_MOTOR_TYPE,GIMBAL_DIRECT_PITCH_DIRECTION,GIMBAL_DIRECT_PITCH_REDUCTION_RATIO,GIMBAL_DIRECT_PITCH_MODE);
	MotorInit(&gimbal_direct.fold,      GIMBAL_DIRECT_FOLD_ID,      GIMBAL_DIRECT_FOLD_CAN,      GIMBAL_DIRECT_FOLD_MOTOR_TYPE,      GIMBAL_DIRECT_FOLD_DIRECTION,      GIMBAL_DIRECT_FOLD_REDUCTION_RATIO,      GIMBAL_DIRECT_FOLD_MODE);
    MotorInit(&gimbal_direct.small_yaw, GIMBAL_DIRECT_SMALL_YAW_ID, GIMBAL_DIRECT_SMALL_YAW_CAN, GIMBAL_DIRECT_SMALL_YAW_MOTOR_TYPE, GIMBAL_DIRECT_SMALL_YAW_DIRECTION, GIMBAL_DIRECT_SMALL_YAW_REDUCTION_RATIO, GIMBAL_DIRECT_SMALL_YAW_MODE);

	//step5 初始化云台初始化校准相关变量
    gimbal_direct.init_start_time=0;
    gimbal_direct.init_timer=0;
    gimbal_direct.init_continue=false;
	
	//step6 模式设置初始化
    gimbal_direct.mode=GIMBAL_ZERO_FORCE;
    gimbal_direct.last_mode = GIMBAL_ZERO_FORCE;
    gimbal_direct.mode_before_rc_err = GIMBAL_ZERO_FORCE;
	
	//step7 云台初始上电位置校准初始化  
    gimbal_direct.init_base.pitch = 0.0f;
    gimbal_direct.init_base.yaw   = 0.0f;
    gimbal_direct.init_base_record = false;
}
/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void GimbalSetMode(void)
{

    // --- 离线检查：遥控器断连 ---
    if (toe_is_error(DBUS_TOE)) 
    {
        gimbal_direct.mode = GIMBAL_DBUS_ERR;
        return;
    }

    if (switch_is_down(gimbal_direct.rc->rc.s[0])) 
    {
        gimbal_direct.mode = GIMBAL_ZERO_FORCE;
    }
    else if (switch_is_mid(gimbal_direct.rc->rc.s[0])) 
    {
        // gimbal_direct.mode = GIMBAL_MOTOR_TEST;
		gimbal_direct.mode = GIMBAL_IMU;
    }
    else if (switch_is_up(gimbal_direct.rc->rc.s[0])) 
    {
		gimbal_direct.mode = GIMBAL_AUTO_AIM;
    }
}
/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void GimbalObserver(void) 
{
	 //电机相关数据更新
	GetMotorMeasure(&gimbal_direct.yaw);
	GetMotorMeasure(&gimbal_direct.pitch);
	GetMotorMeasure(&gimbal_direct.fold);
	GetMotorMeasure(&gimbal_direct.small_yaw);
	// 速度反馈滤波
	// first_order_filter_cali(&small_yaw_vel_filter, gimbal_direct.small_yaw.fdb.vel);
	// gimbal_direct.small_yaw.fdb.vel = small_yaw_vel_filter.out;
	
	//IMU相关数据更新
	gimbal_direct.feedback_pos.pitch=GetImuAngle(AX_PITCH);
	gimbal_direct.feedback_pos.yaw = atan2f(sinf(GetImuAngle(AX_YAW)), cosf(GetImuAngle(AX_YAW)));
	gimbal_direct.feedback_vel.pitch=GetImuVelocity(AX_PITCH);
	gimbal_direct.feedback_vel.yaw=GetImuVelocity(AX_YAW);

	//fold/小yaw用自己的编码器(不同轴)
	gimbal_direct.feedback_pos.fold=gimbal_direct.fold.fdb.pos;
	gimbal_direct.feedback_vel.fold=gimbal_direct.fold.fdb.vel;
	gimbal_direct.feedback_pos.small_yaw=gimbal_direct.small_yaw.fdb.pos;
	gimbal_direct.feedback_vel.small_yaw=gimbal_direct.small_yaw.fdb.vel;
	
	Angle_solution();
	
	if(gimbal_direct.mode == GIMBAL_INIT)
	{
		if (gimbal_direct.last_mode != GIMBAL_INIT)
		{
			gimbal_direct.init_start_time=xTaskGetTickCount();
		}
		gimbal_direct.init_timer=xTaskGetTickCount()-gimbal_direct.init_start_time;
	}
	else
	{
		gimbal_direct.init_timer=0;
	}
	if (gimbal_direct.mode == GIMBAL_DBUS_ERR && gimbal_direct.last_mode != GIMBAL_DBUS_ERR )
	{
		gimbal_direct.mode_before_rc_err=gimbal_direct.last_mode;
	}
	gimbal_direct.last_mode=gimbal_direct.mode; //上一运行模式更新
}
/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void GimbalReference(void) 
{
	if (gimbal_direct.mode == GIMBAL_INIT)
	{
		gimbal_direct.reference.pitch = loop_fp32_constrain(gimbal_direct.pitch.direction * (GIMBAL_DIRECT_PITCH_MID - gimbal_direct.pitch.fdb.pos) + gimbal_direct.feedback_pos.pitch, -M_PI, M_PI);
		gimbal_direct.reference.yaw =  loop_fp32_constrain(gimbal_direct.yaw.direction * (GIMBAL_DIRECT_YAW_MID - gimbal_direct.yaw.fdb.pos) + gimbal_direct.feedback_pos.yaw, -M_PI, M_PI);
		gimbal_direct.reference.small_yaw = GIMBAL_DIRECT_SMALL_YAW_MID;
 	}
	else if(gimbal_direct.mode == GIMBAL_GAP)
	{
		gimbal_direct.reference.pitch = gimbal_direct.feedback_pos.pitch;
		gimbal_direct.reference.yaw = gimbal_direct.feedback_pos.yaw;
		gimbal_direct.reference.fold = gimbal_direct.fold.fdb.pos;
		gimbal_direct.reference.small_yaw = gimbal_direct.small_yaw.fdb.pos;
	}
	else if (gimbal_direct.mode==GIMBAL_IMU)
	{
		//fold 抬头
		gimbal_direct.reference.fold = GIMBAL_DIRECT_FOLD_UP;
		#if GIMBAL_FOLD_CONNECTED

		//small_yaw: fold 到位后再回中，否则保持当前
		if (fabsf(gimbal_direct.reference.fold - gimbal_direct.fold.fdb.pos) < 0.1f)
		{
			gimbal_direct.reference.small_yaw = GIMBAL_DIRECT_SMALL_YAW_MID;
		}
		else
		{
			gimbal_direct.reference.small_yaw = gimbal_direct.small_yaw.fdb.pos;
		}
		#else
		gimbal_direct.reference.small_yaw = GIMBAL_DIRECT_SMALL_YAW_MID;
		#endif

		if (gimbal_direct.last_mode != GIMBAL_IMU)
		{
			//进入IMU模式：大yaw/pitch先hold，等fold抬头
			// gimbal_imu_state = IMU_FOLDING;
			gimbal_imu_state = IMU_RECENTERING;
			gimbal_direct.reference.pitch = gimbal_direct.feedback_pos.pitch;
			gimbal_direct.reference.yaw = gimbal_direct.feedback_pos.yaw;
			LQR_clear(&gimbal_lqr_yaw);
			LQR_clear(&gimbal_lqr_pitch);
			LQR_clear(&gimbal_lqr_fold);
			PID_clear(&gimbal_small_yaw_pid.small_yaw_angle);
			PID_clear(&gimbal_small_yaw_pid.small_yaw_velocity);
		}

		else 
		{
			switch (gimbal_imu_state)
			{
			case IMU_FOLDING:
				//等fold抬头到位，然后大yaw回中
				if (fabsf(gimbal_direct.reference.fold - gimbal_direct.fold.fdb.pos) < 0.1f)
				{
					gimbal_imu_state = IMU_RECENTERING;
					gimbal_direct.reference.yaw = loop_fp32_constrain(gimbal_direct.yaw.direction * (GIMBAL_DIRECT_YAW_MID - gimbal_direct.yaw.fdb.pos) + gimbal_direct.feedback_pos.yaw, -M_PI, M_PI);  // 大yaw回中(MID)
				}
				break;

			case IMU_RECENTERING:
				//电机已使能、反馈有效，抓当前 yaw 再放行
    			gimbal_direct.reference.yaw = gimbal_direct.feedback_pos.yaw;
    			gimbal_imu_state = IMU_READY;
				break;

			case IMU_READY:
				//大yaw、pitch可遥控
				gimbal_direct.reference.pitch = fp32_constrain(gimbal_direct.reference.pitch - gimbal_direct.rc->mouse.y / MOUSE_SENSITIVITY_PITCH, GIMBAL_LOWER_LIMIT_PITCH + gimbal_direct.angle_zero_for_imu, GIMBAL_UPPER_LIMIT_PITCH + gimbal_direct.angle_zero_for_imu);
				gimbal_direct.reference.yaw = loop_fp32_constrain(gimbal_direct.reference.yaw - gimbal_direct.rc->mouse.x / MOUSE_SENSITIVITY_YAW, -M_PI, M_PI);
				gimbal_direct.reference.pitch = fp32_constrain(gimbal_direct.reference.pitch - fp32_deadline(gimbal_direct.rc->rc.ch[1], REMOTE_CONTROLLER_MIN_DEADLINE, REMOTE_CONTROLLER_MAX_DEADLINE) / 10000, GIMBAL_LOWER_LIMIT_PITCH + gimbal_direct.angle_zero_for_imu, GIMBAL_UPPER_LIMIT_PITCH + gimbal_direct.angle_zero_for_imu);
				gimbal_direct.reference.yaw = loop_fp32_constrain(gimbal_direct.reference.yaw - fp32_deadline(gimbal_direct.rc->rc.ch[0], REMOTE_CONTROLLER_MIN_DEADLINE, REMOTE_CONTROLLER_MAX_DEADLINE) / 10000, -M_PI, M_PI);
				break;
			}
      	}
	}
	else if (gimbal_direct.mode == GIMBAL_AUTO_AIM)
	{
//		gimbal_direct.reference.pitch = fp32_constrain(Gimbal_direct_ecd_to_imu(AX_PITCH,GetScCmdGimbalAngle(AX_PITCH)), GIMBAL_LOWER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu  , GIMBAL_UPPER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu );
//		gimbal_direct.reference.yaw   = loop_fp32_constrain(Gimbal_direct_ecd_to_imu(AX_YAW,GetScCmdGimbalAngle(AX_YAW)), -M_PI , M_PI );
		// 自瞄模式：调用专用的控制逻辑处理函数
        Gimbal_direct_auto_aim_control();
	}
	else if (gimbal_direct.mode == GIMBAL_MOTOR_TEST)
	{
		        // 调试模式：位置式，摇杆 ch[0] 推多少转多少，回中=0（中间）
        // 1.0f 是摇杆推满对应的目标角度(rad)，想转更多就改这个数
        gimbal_direct.reference.small_yaw = loop_fp32_constrain(
            fp32_deadline(gimbal_direct.rc->rc.ch[0],
                          REMOTE_CONTROLLER_MIN_DEADLINE,
                          REMOTE_CONTROLLER_MAX_DEADLINE) / 660.0f * 1.0f,
            -M_PI, M_PI);
			// fold：摇杆 ch[1] 控角度（LQR 跟踪）
			gimbal_direct.reference.fold = fp32_constrain(
    		fp32_deadline(gimbal_direct.rc->rc.ch[1],
                  		REMOTE_CONTROLLER_MIN_DEADLINE,
                  		REMOTE_CONTROLLER_MAX_DEADLINE) / 660.0f * GIMBAL_DIRECT_FOLD_UP,
    		GIMBAL_DIRECT_FOLD_DOWN, GIMBAL_DIRECT_FOLD_UP);
	}
	else
	{
		//fold 低头
		gimbal_direct.reference.fold = GIMBAL_DIRECT_FOLD_DOWN;
		gimbal_direct.reference.small_yaw = gimbal_direct.small_yaw.fdb.pos;
	}

}
/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalConsole(void)
{
	if (gimbal_direct.mode == GIMBAL_ZERO_FORCE || gimbal_direct.mode == GIMBAL_DBUS_ERR)
	{
		//yaw/pitch/小yaw 零力
		gimbal_direct.yaw.set.tor = 0;
		gimbal_direct.pitch.set.tor = 0;
		gimbal_direct.small_yaw.set.curr = 0;
		//fold 低头(编码器反馈)
		gimbal_direct.fold.set.tor = 0;
	}
	else if(gimbal_direct.mode == GIMBAL_IMU || gimbal_direct.mode == GIMBAL_GAP || gimbal_direct.mode == GIMBAL_INIT || gimbal_direct.mode == GIMBAL_AUTO_AIM)
	{
		//大yaw LQR(编码器反馈,与小yaw解耦)
		gimbal_direct.yaw.set.tor = gimbal_direct.yaw.direction * LQR_calc(&gimbal_lqr_yaw, loop_fp32_constrain(gimbal_direct.reference.yaw - gimbal_direct.feedback_pos.yaw, -M_PI, M_PI), gimbal_direct.yaw.direction * gimbal_direct.yaw.fdb.vel);
		//pitch LQR(IMU反馈)
		gimbal_direct.pitch.set.tor = gimbal_direct.pitch.direction * LQR_calc(&gimbal_lqr_pitch, gimbal_direct.reference.pitch - gimbal_direct.feedback_pos.pitch, gimbal_direct.feedback_vel.pitch);
		//fold LQR(编码器反馈)
		gimbal_direct.fold.set.tor = LQR_calc(&gimbal_lqr_fold, gimbal_direct.reference.fold - gimbal_direct.fold.fdb.pos, gimbal_direct.fold.fdb.vel);
		//小yaw 双环PID(角度环IMU反馈,2ms分频; 速度环1ms)
		small_yaw_angle_cnt++;
		if (small_yaw_angle_cnt >= GIMBAL_SMALL_YAW_ANGLE_PERIOD_MS)
		{
			small_yaw_angle_cnt = 0;
			fp32 small_yaw_ref = gimbal_direct.small_yaw.fdb.pos + loop_fp32_constrain(gimbal_direct.reference.small_yaw - gimbal_direct.small_yaw.fdb.pos, -M_PI, M_PI);
			gimbal_direct.small_yaw.set.vel = PID_calc(&gimbal_small_yaw_pid.small_yaw_angle, gimbal_direct.small_yaw.fdb.pos, small_yaw_ref);
		}
		gimbal_direct.small_yaw.set.curr = gimbal_direct.small_yaw.direction * PID_calc(&gimbal_small_yaw_pid.small_yaw_velocity, gimbal_direct.small_yaw.fdb.vel, gimbal_direct.small_yaw.set.vel);
	}
	else if(gimbal_direct.mode == GIMBAL_MOTOR_TEST)
	{
		// 调试模式：根据想要单独调试的电机解除注释，记得把不用的的电机力矩和电流都发0
        gimbal_direct.yaw.set.tor = 0;
        gimbal_direct.pitch.set.tor = 0;
        gimbal_direct.fold.set.tor = 0;
		// gimbal_direct.small_yaw.set.curr = 0;

        // 系统辨识：开环正弦电压激励
        // GenerateSinWave(幅值, 偏置, 周期s)，周期 = 1/频率
        // gimbal_direct.small_yaw.set.curr = GenerateSinWave(15000.0f, 0.0f, 1.0f);

		// 调双环的时候解除注释
		small_yaw_angle_cnt++;
		if (small_yaw_angle_cnt >= GIMBAL_SMALL_YAW_ANGLE_PERIOD_MS)
		{
			small_yaw_angle_cnt = 0;
			fp32 small_yaw_ref = gimbal_direct.small_yaw.fdb.pos + loop_fp32_constrain(gimbal_direct.reference.small_yaw - gimbal_direct.small_yaw.fdb.pos, -M_PI, M_PI);
			gimbal_direct.small_yaw.set.vel = PID_calc(&gimbal_small_yaw_pid.small_yaw_angle, gimbal_direct.small_yaw.fdb.pos, small_yaw_ref);
		}
		gimbal_direct.small_yaw.set.curr = gimbal_direct.small_yaw.direction * PID_calc(&gimbal_small_yaw_pid.small_yaw_velocity, gimbal_direct.small_yaw.fdb.vel, gimbal_direct.small_yaw.set.vel);

		// 内环(速度环)单独调参：摇杆直接给速度指令，跳过角度环
        // gimbal_direct.small_yaw.set.vel = loop_fp32_constrain(
        //     fp32_deadline(gimbal_direct.rc->rc.ch[0],
        //                   REMOTE_CONTROLLER_MIN_DEADLINE,
        //                   REMOTE_CONTROLLER_MAX_DEADLINE) / 660.0f * 15.0f,
        //     -MAX_OUT_GIMBAL_SMALL_YAW_ANGLE, MAX_OUT_GIMBAL_SMALL_YAW_ANGLE);

        // gimbal_direct.small_yaw.set.curr = gimbal_direct.small_yaw.direction *
        //     PID_calc(&gimbal_small_yaw_pid.small_yaw_velocity,
        //              gimbal_direct.small_yaw.fdb.vel, gimbal_direct.small_yaw.set.vel);

		// fold LQR 摇杆控角度（想调 fold 就取消下面这行注释）
		// gimbal_direct.fold.set.tor = LQR_calc(&gimbal_lqr_fold, gimbal_direct.reference.fold - gimbal_direct.fold.fdb.pos, gimbal_direct.fold.fdb.vel);
		// gimbal_direct.fold.set.tor = 0.3; //以恒定力矩转动，用来做系统辨识
	}
	//VOFA调试
	//  JustFloat(gimbal_direct.yaw.set.tor,gimbal_direct.yaw.fdb.tor,gimbal_direct.yaw.fdb.pos,gimbal_direct.yaw.fdb.vel);
	 JustFloat(gimbal_direct.small_yaw.set.pos,gimbal_direct.small_yaw.fdb.pos,gimbal_direct.small_yaw.fdb.curr,gimbal_direct.small_yaw.fdb.vel);
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalSendCmd(void)
{

	static bool motor_enabled = false;
	bool want_enable = !(gimbal_direct.mode == GIMBAL_ZERO_FORCE || gimbal_direct.mode == GIMBAL_DBUS_ERR);

	if (!want_enable)
	{
		//6020 发0电压
		CanCmdDjiMotor(GIMBAL_CAN, GIMBAL_STDID, 0, 0, 0, 0);

		if (motor_enabled)
		{
			//yaw/pitch 直接失能
			DmDisable(&gimbal_direct.yaw);
			DmDisable(&gimbal_direct.pitch);
			DmDisable(&gimbal_direct.fold);
			motor_enabled = false;
		}
	}
	else
	{

		//6020 发电压
		CanCmdDjiMotor(GIMBAL_CAN, GIMBAL_STDID, 0,  (int16_t)gimbal_direct.small_yaw.set.curr,0,0);
		if (!motor_enabled)
		{
			//4310 使能
			DmEnable(&gimbal_direct.yaw);
			DmEnable(&gimbal_direct.pitch);
			DmEnable(&gimbal_direct.fold);
			motor_enabled = true;
		}
		DmMitCtrlTorque(&gimbal_direct.yaw);
		DmMitCtrlTorque(&gimbal_direct.pitch);
		DmMitCtrlTorque(&gimbal_direct.fold);
	}
}



#endif  // GIMBAL_YAW_PITCH


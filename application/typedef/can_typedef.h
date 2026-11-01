/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_typedef.c/h
  * @brief      can的板间通信部分的相关定义（CAN_communication.c内部文件，不对外开放）
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-04-08      Penguin         1.done
  *
  @verbatim
  ==============================================================================
  11 位 std id 划分
    base id:   110 0000 0000
    type id:   001 1100 0000
    target id: 000 0011 1000
    index id:  000 0000 0111
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/
#ifndef CAN_TYPEDEF_H
#define CAN_TYPEDEF_H

#include "attribute_typedef.h"
#include "remote_control.h"
#include "struct_typedef.h"

// clang-format off
#define BASE_ID_OFFSET 9
#define TYPE_ID_OFFSET 6
#define TARGET_ID_OFFSET 3

// CAN通信协议相关定义
// 固定类型的数据包
#define CAN_STD_ID_PACK_BASE     ((uint16_t) 0x0400)

#define CAN_STD_ID_Test     ((uint16_t) 1)
#define CAN_STD_ID_Rc       ((uint16_t) 2)
#define CAN_STD_ID_Gimbal   ((uint16_t) 3)
#define CAN_STD_ID_Referee 	((uint16_t) 4)
#define CAN_STD_ID_Navi		((uint16_t) 5)
#define CAN_STD_ID_Chassis_Fdb ((uint16_t) 6)

#define CAN_Test_Duration   ((uint32_t)20) // ms
#define CAN_Rc_Duration     ((uint32_t)16) // ms
#define CAN_Gimbal_Duration ((uint32_t)10) // ms

// 任意类型的数据包
#define CAN_STD_ID_ANY_BASE     ((uint16_t)0x0601)

// clang-format on

/*-------------------- Send & Receive --------------------*/

// typedef union {
//     struct
//     {
//     } __attribute__((packed)) data;
// } Data_Test_u;

typedef struct
{
    struct
    {
        union {
            struct
            {
                uint16_t ch0 : 11;
                uint16_t ch1 : 11;
                uint16_t ch2 : 11;
                uint16_t ch3 : 11;
                uint16_t ch4 : 11;
                uint8_t s0 : 2;
                uint8_t s1 : 2;
                bool offline : 1;
                uint8_t reserved : 4;
            } __packed__ packed;
            struct
            {
                uint8_t data[8];
            } __packed__ raw;
        } rc;

        union {
            struct
            {
                int16_t mouse_x : 15;
                uint16_t mouse_press_l : 1;
                int16_t mouse_y : 15;
                uint16_t mouse_press_r : 1;
                int16_t mouse_z;
                int16_t key;
            } __packed__ packed;
            struct
            {
                uint8_t data[8];
            } __packed__ raw;
        } km;
        RC_ctrl_t rc_unpacked;  // 遥控器数据
    } rc_data;

    struct
    {
        union {
            struct
            {
                float yaw;
                bool offline;
                bool init_judge;
                uint16_t reserved;
            } __packed__ packed_data;
            struct
            {
                uint8_t data[8];
            } __packed__ raw;
        } gimbal;

    } gimbal_data;
	// 3. 裁判系统数据（新增：热量闭环）
    struct {
        union {
            uint8_t data[8];
            struct {
                uint16_t heat_limit; // 热量上限
                uint16_t heat;       // 当前热量
				uint8_t  robot_id;            // 自身机器人ID
                uint8_t  game_progress;       // 比赛阶段
                uint16_t stage_remain_time;   // 比赛剩余时间
            }  __packed__ packed;
        } frame0;
		
		 // Index 1: 自身机器人状态与比赛结果
        union {
            struct {
                uint16_t current_HP;          // 当前血量
                uint16_t maximum_HP;          // 最大血量
                uint16_t chassis_power_limit; // 底盘功率上限
                uint8_t  power_shooter_out;   // 发射机构24V输出状态
                uint8_t  winner;              // 比赛结果 (0平局/1红/2蓝)
            } __packed__ packed;
            uint8_t data[8];
        } frame1;
		// Index 2: 机器人位置 (X和Y坐标)
        union {
            struct {
                float x;
                float y;
            } __packed__ packed;
            uint8_t data[8];
        } frame2;

        // Index 3: 机器人朝向与增益 (Buff)
        union {
            struct {
                float    angle;               // 机器人朝向
                uint8_t  recovery_buff;       // 回血增益
                uint16_t cooling_buff;        // 冷却增益
                uint8_t  attack_buff;         // 攻击增益
            } __packed__ packed;
            uint8_t data[8];
        } frame3;

        // Index 4~7: 存活血量 (全场HP数据极其庞大，正好需要4帧=32字节)
        union {
            struct { uint16_t r1, r2, r3, r4; } __packed__ packed;
            uint8_t data[8];
        } frame4;

        union {
            struct { uint16_t r5, r7, out, base; } __packed__ packed;
            uint8_t data[8];
        } frame5;

        union {
            struct { uint16_t b1, b2, b3, b4; } __packed__ packed;
            uint8_t data[8];
        } frame6;

        union {
            struct { uint16_t b5, b7, out, base; } __packed__ packed;
            uint8_t data[8];
        } frame7;
		
    } referee_data;
	/* 4. 上位机 ROS 导航下发的底盘速度指令 */
	 struct
    {
        union {
            struct
            {
                int16_t vx;     // 期望 X 轴速度 * 1000
                int16_t vy;     // 期望 Y 轴速度 * 1000
                int16_t wz;     // 期望 Z 轴角速度 * 1000
                uint16_t null;  // 补齐剩下的 2 字节，凑满 8 字节
            } __packed__ packed; // 必须加 __packed__ 保证内存连续不留空隙
            struct
            {
                uint8_t data[8];
            } __packed__ raw;
        } navi;
    } navi_data;
	/* 5. 底盘向上板反馈的数据（真实参考速度反馈） */
    struct
    {
        union {
            struct
            {
                int16_t vx;     // 反馈 X 轴速度 * 1000
                int16_t vy;     // 反馈 Y 轴速度 * 1000
                int16_t wz;     // 反馈 Z 轴角速度 * 1000
                uint16_t null;  // 补齐 8 字节
            } __packed__ packed;
            struct
            {
                uint8_t data[8];
            } __packed__ raw;
        } speed;
    } chassis_fdb_data;
	
} CanBoardCommunicate_t;

#endif
/*------------------------------ End of File ------------------------------*/

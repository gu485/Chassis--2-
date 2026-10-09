/**
 *******************************************************************************
 * @file      : HW_can.cpp
 * @brief     : CAN 收发基础封装实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：CanFilter_Init + CAN_Send_Msg
 *                                                全量提供，接收分发回调掏空
 *******************************************************************************
 * @attention :
 *  接收中断回调 HAL_CAN_RxFifo0/1MsgPendingCallback 已在此预留，仅完成「取 8 字节 +
 *  再次使能接收中断」，具体「按帧 ID 分发」逻辑留给新人：
 *    - M3508 / DM 电机反馈：调用对应电机的 decode(can?_rx_data)
 *    - 云台 yaw 轴 DM4310(ID=2)也接在 CAN1 上，由底盘板直接控制，同样按反馈 ID 分发
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"

#include "can.h"
#include "ins_motor.hpp"             // GetMotorWheel(i), GetMotorYaw()
#include "motor_DaMiao.hpp"           // DM_J4310::GetRxId
#include "motor_M3508.hpp"            // M3508::GetRxId(i+1)
#include "ins_chassis_gimbal_comm.hpp" // GetGimbalChassisComm() / robot::kGimbalToChassisId
#include "gimbal_chassis_comm.hpp"     // robot::GimbalChassisComm 完整定义

/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];
static uint32_t p_tx_mailbox;

/* 调试用计数器/快照:Ozone 里直接挂 Watch 看,不用手动打断点 */
uint32_t g_can1_rx_isr_cnt = 0;      // CAN1 收到过多少帧(不管 ID 对不对)
uint32_t g_can1_rx_last_id = 0;      // 最近一次收到的 StdId(十六进制看)
uint32_t g_can1_rx_matched_cnt = 0;  // 其中命中任意一个轮电机 ID 的次数(汇总)
/* 命中云台→底盘双板帧(0x211)的次数。恒为 0 = 云台板没在发,
 * 或者总线上只有单板(比如云台板没上电)。 */
volatile uint32_t g_can1_g2c_cnt = 0;
uint32_t g_can1_tx_ok_cnt = 0;       // CAN_Send_Msg 成功投递邮箱的次数
uint32_t g_can1_tx_err_cnt = 0;      // 投递失败(邮箱满等)的次数
uint32_t check_Can2_Rx_Msg = 0;
/* 按 4 个轮电机分开的命中计数,idx 0~3 对应 id 1~4(反馈 ID 0x201~0x204) */
uint32_t g_can1_matched_cnt_by_id[4] = {0, 0, 0, 0};

/* 最近一帧的原始 8 字节数据快照(不管 ID 对不对都存),用来看内容像不像电调反馈 */
uint8_t g_can1_last_rx_data[8] = {0};

/* 排查"电调是不是接在 CAN2 上"用的临时计数器,阶段 4 前不会真正拿 CAN2 发东西 */
uint32_t g_can2_rx_isr_cnt = 0;
uint32_t g_can2_rx_last_id = 0;
uint32_t g_can2_rx_matched_cnt = 0;

/* yaw 电机(DM4310)调试用:CAN1 收到的帧里命中 yaw 反馈 ID 的次数,
 * 如果这个数一直是 0,说明 CAN1 根本没收到 yaw 电机的反馈帧(接线/ID/波特率问题),
 * 跟"电机没使能"或"没转"是完全不同的两类问题,要先看这个排除掉接收链路的问题。 */
uint32_t g_yaw_rx_matched_cnt = 0;

/* === 达妙电机寄存器读取(诊断 CTRL_MODE 用)===
 * 库的 enable_mode_switch=false 导致它永远不会去读电机实际的控制模式,
 * 构造函数的 current_ctrl_mode=kMIT 只是个"假设"。这里自己发读请求把真值拿回来。
 *
 * 协议(手册 P15「读取参数」):
 *   发送 ID=0x7FF, D[0]=CANID_L(0x02) D[1]=CANID_H(0x00) D[2]=0x33(读) D[3]=RID
 *   电机回 ID=MST_ID(0x12), 前 4 字节原样回带,D[4..7] 是 uint32 数据(小端)
 *
 * g_dm_ctrl_mode 初值 0xFFFFFFFF = "一次都还没读到",跟"读到 0"区分开 */
volatile uint32_t g_dm_ctrl_mode    = 0xFFFFFFFFu;  // 0x0A 寄存器读回来的值
volatile uint32_t g_dm_reg_read_cnt = 0;            // 成功解析了几次寄存器应答
volatile uint8_t  g_dm_reg_last_rid = 0;            // 最近一次读的是哪个寄存器

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief   按 StdId 把 8 字节 CAN 反馈分发给对应电机
 * @param   rx_id: 接收到的 StdId
 * @param   data:  8 字节 CAN 数据
 * @param   matched_cnt: 命中时要递增的调试计数器(区分是哪条总线收到的)
 * @retval  None
 * @note    4 个轮电机的接收 ID 是 0x201~0x204(由 M3508::GetRxId 给出)
 */
static void RouteMotorFeedback(uint32_t rx_id, const uint8_t data[8],
                               uint32_t* matched_cnt);

/* Exported function definitions ---------------------------------------------*/

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef can_filter;

  /* 全收：掩码全 0，任何 ID 均通过 */
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterActivation = ENABLE;
  /* CAN1/CAN2 共用 28 个滤波器组,此字段决定分界点:
   * bank 0~13 归 CAN1，bank 14~27 归 CAN2。必须显式设置，
   * 否则用栈上的未初始化值会导致 CAN2 的 bank14 实际不生效，
   * 报文在硬件层被丢弃，RX 中断永远不触发。 */
  can_filter.SlaveStartFilterBank = 14;

  if (hcan == &hcan1) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterBank = 0;
  } else if (hcan == &hcan2) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    can_filter.FilterBank = 14;
  } else {
    return;
  }

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }
}

void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  tx_header.StdId = id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;
  if (HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &p_tx_mailbox) == HAL_OK) {
    if (hcan == &hcan1) g_can1_tx_ok_cnt++;
  } else {
    if (hcan == &hcan1) g_can1_tx_err_cnt++;
  }
}

/**
 * @brief   CAN1 FIFO0 接收中断回调
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    HAL 头文件里这个回调是 C 链接声明的,这里必须用 extern "C" 包裹,
 *          否则 C++ 的 name mangling 会导致这个定义盖不掉 HAL 里的弱空实现,
 *          中断触发时实际跑的是那个什么都不做的默认回调。
 */
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1,
                             can1_rx_data) == HAL_OK) {
      g_can1_rx_isr_cnt++;
      g_can1_rx_last_id = rx_header1.StdId;
      for (int k = 0; k < 8; k++) g_can1_last_rx_data[k] = can1_rx_data[k];
      // 云台 yaw 轴 DM4310(ID=2)直接接在 CAN1 上,反馈 ID 固定为 id+0x10
      if (rx_header1.StdId == hello_world::motor::DM_J4310::GetRxId(2)) {
        g_yaw_rx_matched_cnt++;
        // 同一个 ID(0x12)上会有两类帧,靠 D[2] 区分:
        //   常规状态反馈帧: D[0]高4位=状态码,D[1..2]=位置,D[2] 是角度的任意字节
        //   寄存器应答帧  : D[2] 固定是 0x33(读) 或 0x55(写),D[3]=RID,D[4..7]=数据
        if (can1_rx_data[2] == 0x33 || can1_rx_data[2] == 0x55) {
          // 寄存器应答:小端解出 D[4..7] 这 4 个字节拼成的 uint32
          uint32_t val = (uint32_t)can1_rx_data[4]
                       | ((uint32_t)can1_rx_data[5] << 8)
                       | ((uint32_t)can1_rx_data[6] << 16)
                       | ((uint32_t)can1_rx_data[7] << 24);
          g_dm_reg_last_rid = can1_rx_data[3];
          if (can1_rx_data[3] == 0x0A) {   // 0x0A = CTRL_MODE 控制模式
            g_dm_ctrl_mode = val;
          }
          g_dm_reg_read_cnt++;
        } else {
          GetMotorYaw()->decode(can1_rx_data);
        }
      } else if (rx_header1.StdId == robot::kGimbalToChassisId) {
        /* 云台→底盘双板帧(0x211):解出云台世界朝向 + IMU 就绪标志。
         * 必须在电机反馈之前判,虽然这两个 ID 不会撞,但顺序上
         * 先处理协议帧,出问题时好定位。 */
        g_can1_g2c_cnt++;
        auto* comm = GetGimbalChassisComm();
        if (comm != nullptr) {
          comm->decode(can1_rx_data);
        }
      } else {
        RouteMotorFeedback(rx_header1.StdId, can1_rx_data, &g_can1_rx_matched_cnt);
      }
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   CAN2 FIFO1 接收中断回调
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    原因同上,必须 extern "C"。
 */
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2,
                             can2_rx_data) == HAL_OK) {
      g_can2_rx_isr_cnt++;
      check_Can2_Rx_Msg++;  
      // 阶段 4 前用这个计数器排查电调是不是接在 CAN2 上
      g_can2_rx_last_id = rx_header2.StdId;
      RouteMotorFeedback(rx_header2.StdId, can2_rx_data, &g_can2_rx_matched_cnt);
      // 双板通讯走 CAN1(见 HAL_CAN_RxFifo0MsgPendingCallback),CAN2 专用于轮电机
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief   按 StdId 把 8 字节 CAN 反馈分发给对应电机
 */
static void RouteMotorFeedback(uint32_t rx_id, const uint8_t data[8],
                               uint32_t* matched_cnt) {
  // 4 个轮电机的接收 ID 是 0x201~0x204(由 M3508::GetRxId 给出)
  for (int i = 0; i < 4; i++) {
    uint32_t motor_rx_id = hello_world::motor::M3508::GetRxId(static_cast<uint8_t>(i + 1));
    if (rx_id == motor_rx_id) {
      (*matched_cnt)++;
      if (matched_cnt == &g_can1_rx_matched_cnt) {
        g_can1_matched_cnt_by_id[i]++;  // 只在 CAN1 上细分到具体哪个轮子
      }
      auto* motor = GetMotorWheel(i);
      if (motor != nullptr) {
        motor->decode(data);
      }
      return;  // 命中就退出(同一帧不可能同时是多个电机)
    }
  }
  // 都不命中 → 未知帧,忽略
}

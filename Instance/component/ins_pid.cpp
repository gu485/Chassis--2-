/**
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_pid.hpp"
#include "pid.hpp"
/* Private macro -------------------------------------------------------------*/
static constexpr int kPidCount = 4;  // 4 个轮子各一个独立 PID
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::pid::Pid *GetPidMotorWheel(int idx) {
  if (idx < 0 || idx >= kPidCount) {
    return nullptr;  // 越界保护
  }

  // 函数内 static = 全局唯一 + 首次调用创建
  static hello_world::pid::Pid *pids[kPidCount] = {nullptr};

  if (pids[idx] == nullptr) {
    // 起始参数:从老 GM6020 那套 130/0.3/0 起步,M3508 实测要重调
    // M3508 电流范围 ±16384,out_limit 起步给 8000 留余量
    pids[idx] = new hello_world::pid::Pid(
        /*kp=*/       13080.0f,
        /*ki=*/         0.1f,
        /*kd=*/         0.0f,
        /*i_limit=*/  4000.0f,    // 积分限幅:不能比 out_limit 还大,否则积分抗饱和形同虚设
        /*out_limit=*/ 8000.0f);  // 输出(电流)限幅
  }
  return pids[idx];
}

hello_world::pid::Pid *GetPidFollow() {
  static hello_world::pid::Pid *p = nullptr;
  if (p == nullptr) {
    // 跟随模式是位置环(θ → 0),不建议加 I:θ 有 ±π 跳变,积分会累积出假偏差
    p = new hello_world::pid::Pid(
        /*kp=*/0.9f,
        /*ki=*/0.0f,      // ← 关键:积分关掉
        /*kd=*/0.0f,      // ← 关键:1kHz 采样下原始差分噪声太大
        /*i_limit=*/0.5f,
        /*out_limit=*/2.0f);   // ← w 限幅 2 rad/s,别让底盘追飞
  }
  return p;
}
hello_world::pid::Pid *GetPidYawRate() {
  static hello_world::pid::Pid *p = nullptr;
  if (p == nullptr) {
    // 云台 yaw 角速度环:误差 rad/s → 力矩 N·m(DM4310)。起步值,上机再调
    p = new hello_world::pid::Pid(
        /*kp=*/0.3f,
        /*ki=*/0.5f,
        /*kd=*/0.0f,
        /*i_limit=*/1.0f,
        /*out_limit=*/3.0f);   // 力矩限幅 N·m
  }
  return p;
}
/* Private function definitions ----------------------------------------------*/
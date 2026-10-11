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
    // 轮速速度环: 误差 rad/s -> 输出直接是电流 A (不再是电调原始值)。
    // 由旧的原始值参数等效换算而来 (原始值 -> 安培 系数 = 20A / 16384):
    //   kp 13080 -> 15.97 A/(rad/s)   ki 0.1 -> 0.000122
    //   i_limit 4000 -> 4.88 A        out_limit 8000 -> 9.77 A
    // M3508 + C620 额定持续电流 20 A, out_limit 给一半留余量。
    // 注意 kp 的物理含义: 轮速误差超过 out_limit/kp ≈ 0.61 rad/s 输出就顶满限幅。
    pids[idx] = new hello_world::pid::Pid(
        /*kp=*/       15.97f,
        /*ki=*/        0.000122f,
        /*kd=*/        0.0f,
        /*i_limit=*/   4.88f,     // 积分限幅(A): 不能比 out_limit 还大, 否则抗饱和形同虚设
        /*out_limit=*/ 9.77f);    // 输出(电流 A)限幅
  }
  return pids[idx];
}

hello_world::pid::Pid *GetPidFollow() {
  static hello_world::pid::Pid *p = nullptr;
  if (p == nullptr) {
    // 跟随模式是位置环(θ → 0),不建议加 I:θ 有 ±π 跳变,积分会累积出假偏差
    p = new hello_world::pid::Pid(
        /*kp=*/1.7f,
        /*ki=*/0.0f,      // ← 关键:积分关掉
        /*kd=*/0.0f,      // ← 关键:1kHz 采样下原始差分噪声太大
        /*i_limit=*/0.5f,
        /*out_limit=*/2.0f);   // ← w 限幅 2 rad/s,别让底盘追飞
  }
  return p;
}
/* Private function definitions ----------------------------------------------*/
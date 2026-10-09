/**
 *******************************************************************************
 * @file      :ins_chassis_iksolver.cpp
 * @brief     : 底盘运动学逆解器实现
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
#include "ins_chassis_iksolver.hpp"

#include <cmath>

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

namespace hw_cs = hello_world::chassis_ik_solver;

void hw_cs::ChassisIkSolver::solve(float vx, float vy, float w) {
  /* ① 存输入(Ozone 调试) */
  last_vx_ = vx;
  last_vy_ = vy;
  last_w_  = w;

  /* ② 平移向量等比例限幅 —— 保方向, 只缩大小 */
  float v_norm = std::sqrt(vx * vx + vy * vy);
  if (v_norm > kMaxV_ && v_norm > 0.0f) {
    float k = kMaxV_ / v_norm;
    vx *= k;
    vy *= k;
  }

  /* ③ w 单独限幅 */
  if      (w >  kMaxW_) w =  kMaxW_;
  else if (w < -kMaxW_) w = -kMaxW_;

  /* ④ 通用投影公式 ＋ 单轮限幅 */
  for (int i = 0; i < 4; i++) {
    float cs = std::cos(kTheta_[i]);
    float sn = std::sin(kTheta_[i]);
    float om = (vx * cs + vy * sn
                + w * (kX_[i] * sn - kY_[i] * cs))
               / kWheelRadius_;

    /* 单轮保护 */
    if      (om >  kMaxWheelVel_) om =  kMaxWheelVel_;
    else if (om < -kMaxWheelVel_) om = -kMaxWheelVel_;

    target_[i] = om;
  }
}

float hw_cs::ChassisIkSolver::targetVel(int idx) const {
  if (idx < 0 || idx >= 4) return 0.0f;
  return target_[idx];
}

/* 单例: 程序启动即构造 */
namespace {
hw_cs::ChassisIkSolver g_ik_solver;
}

hw_cs::ChassisIkSolver *GetChassisIkSolver() {
  return &g_ik_solver;
}

/* Private function definitions ----------------------------------------------*/

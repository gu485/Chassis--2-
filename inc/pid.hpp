/**
 *******************************************************************************
 * @file      : pid.hpp
 * @brief     : 位置式 PID 控制器
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-23      Hello World     1. 从老工程 pid_calc_speed 迁过来的位置式 PID
 *******************************************************************************
 * @attention :
 *  1. 位置式 PID(含积分限幅 + 输出限幅)
 *  2. 不含角度环那种 circular/needed,本工程只做速度环
 *  3. 每个轮子需要独立 PID 实例,不要共享
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#ifndef HW_COMPONENTS_ALGORITHM_PID_HPP_
#define HW_COMPONENTS_ALGORITHM_PID_HPP_

namespace hello_world {
namespace pid {

class Pid {
 public:
  /**
   * @brief       构造位置式 PID
   * @param       kp:        比例系数
   * @param       ki:        积分系数
   * @param       kd:        微分系数  [输出 / (输入/秒)]
   * @param       i_limit:   积分限幅(防积分饱和)
   * @param       out_limit: 输出限幅(防电流爆炸)
   * @param       dt:        采样周期(s)。控制循环 1kHz 就填 0.001f。
   *                         缺省 1ms,对应本工程所有 1kHz 控制回路。
   * @param       beta:      误差低通系数 (0,1],1.0 = 不滤波。
   *                         仅在 kd≠0 时起作用。
   * @retval       None
   */
  Pid(float kp, float ki, float kd, float i_limit, float out_limit,
      float dt = 0.001f, float beta = 0.2f);

  ~Pid() = default;

  /**
   * @brief       位置式 PID 计算
   * @param       target:  目标值
   * @param       measure: 实测值
   * @retval      本次输出
   */
  float calc(float target, float measure);

  /**
   * @brief       清空 PID 内部状态(切换模式时用)
   */
  void reset();


 private:
  /* 控制器参数 */
  float kp_       = 0.0f;
  float ki_       = 0.0f;
  float kd_       = 0.0f;
  float i_limit_  = 0.0f;
  float out_limit_ = 0.0f;
  float dt_       = 0.001f;
  float inv_dt_   = 1000.0f;
  float beta_     = 0.2f;

  /* 运行状态(每次 calc 更新) */
  float error_     = 0.0f;
  float last_err_  = 0.0f;
  float i_out_     = 0.0f;
  /* 误差低通结果。D 项从这里取差分,而不是从原始误差取,
   * 这样滤波作用在原始量级上,相位滞后最小。
   * 只有 kd≠0 时才会更新,ki/kd 都为 0 的回路(kp-only)不承担这份开销。 */
  float error_f_   = 0.0f;
  bool  f_valid_   = false;
};

}  // namespace pid
}  // namespace hello_world
#endif /* HW_COMPONENTS_ALGORITHM_PID_HPP_ */
/**
 *******************************************************************************
 * @file      : pid.cpp
 * @brief     : 位置式 PID 实现
 * @history   :
 *  V1.0.0      2026-09-23      Hello World     1. 从老工程 pid_calc_speed 迁过来
 *  V1.1.0      2026-10-06      gujun           1. 积分项乘 dt、微分项除以 dt,
 *                                                  参数按物理量纲归一化
 *                                      2. D 项改成"先滤波后微分",降低相位滞后
 *                                      3. 补上长期缺失的 reset() 实现
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#include "pid.hpp"

namespace hello_world {
namespace pid {

Pid::Pid(float kp, float ki, float kd, float i_limit, float out_limit,
         float dt, float beta)
    : kp_(kp), ki_(ki), kd_(kd),
      i_limit_(i_limit), out_limit_(out_limit),
      dt_(dt > 1e-6f ? dt : 1e-6f),
      inv_dt_(1.0f / (dt > 1e-6f ? dt : 1e-6f)),
      beta_(beta > 1.0f ? 1.0f : (beta < 0.0f ? 0.0f : beta)) {}

float Pid::calc(float target, float measure) {
  /* 1. 当前误差 */
  error_ = target - measure;

  /* 2. P —— 用原始误差,不做滤波。
   * P 项要求响应快,滤波只会让整体滞后。 */
  float p_out = kp_ * error_;

  /* 3. I(积分累加,带限幅防饱和)
   *
   * 乘 dt:积分项的物理意义是"误差对时间的累积"。不乘 dt 的话 ki 的
   * 量纲会随采样频率变化 —— 同样一个 ki,1kHz 下比 100Hz 下积分快 10 倍,
   * 换控制频率就得把所有 ki 重新调一遍。
   *
   * 现有三个 PID 单例(轮子/跟随/pitch)的 ki 都是 0,所以这一改动对它们
   * 没有实际影响(0 × dt 仍然是 0),不会改变已调好的行为。 */
  i_out_ += ki_ * error_ * dt_;
  if (i_out_ > i_limit_) {
    i_out_ = i_limit_;
  } else if (i_out_ < -i_limit_) {
    i_out_ = -i_limit_;
  }

  /* 4. D:先对误差低通,再取差分并除以 dt
   *
   * 除以 dt 是必须的:差分得到的是"每拍变化量"(输入/拍),
   * 除以 dt 才是"变化速度"(输入/秒),这样 kd 才有物理量纲。
   * 不除的话系数小三个数量级 —— 1kHz 下 kd=0.05 实际等效于 0.00005,
   * 阻尼形同虚设,表现为"拨杆稍多就超调,而且怎么调 kd 都没用"。
   *
   * "先滤波后微分"而不是"先微分再滤波":先差分会把信号放大 inv_dt 倍
   * (1kHz 下 1000 倍),同样的滤波系数作用在放大后的信号上,
   * 相位滞后大得多,等效阻尼达不到设计值。 */
  float d_out = 0.0f;
  if (kd_ != 0.0f) {
    if (!f_valid_) {
      /* 首拍:滤波器还没"跑起来",直接用原始误差初始化。
       * 否则差分会从 0 跳到实际误差,产生一个假的速度尖峰。 */
      error_f_ = error_;
      f_valid_ = true;
    } else {
      error_f_ = beta_ * error_ + (1.0f - beta_) * error_f_;
    }
    d_out = kd_ * (error_f_ - last_err_) * inv_dt_;
    last_err_ = error_f_;
  } else {
    /* kd=0:不维护微分状态,省掉滤波开销。
     * 轮子 PID 和跟随 PID 目前都是这种情况。 */
    last_err_ = error_;
  }

  /* 5. 总输出(带限幅) */
  float output = p_out + i_out_ + d_out;
  if (output > out_limit_) {
    output = out_limit_;
  } else if (output < -out_limit_) {
    output = -out_limit_;
  }

  return output;
}

void Pid::reset() {
  /* 清空运行状态,参数(kp/ki/kd/限幅)保持不变。
   * 必须在"控制目标突变"的时刻调用,比如:
   *   - 刚进入跟随模式(目标从"未知"跳到当前对齐位置)
   *   - 模式切换
   * 不 reset 的话,上次累积的 i_out_ 和 last_err_ 会让第一拍输出
   * 出现莫名的偏差或尖峰。
   *
   * f_valid_ 也必须清:误差滤波状态不清的话,恢复后第一拍差分会拿
   * 陈旧的 error_f_ 去算,产生不明来源的力矩尖峰。
   *
   * 注:这个函数在 pid.hpp 里声明了很久但实现一直缺失,只要没人调用
   * 就永远不会暴露,一旦有人调就在链接期报 undefined reference。 */
  error_    = 0.0f;
  last_err_ = 0.0f;
  i_out_    = 0.0f;
  error_f_  = 0.0f;
  f_valid_  = false;
}

}  // namespace pid
}  // namespace hello_world
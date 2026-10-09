/**
 *******************************************************************************
 * @file      :ins_chassis_iksolver.hpp
 * @brief     : 底盘运动学逆解器(全向轮 45° X 形)
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_INSTANCE_INS_CHASSIS_IKSOLVER_HPP_
#define CHASSIS_INSTANCE_INS_CHASSIS_IKSOLVER_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace hello_world {
namespace chassis_ik_solver {

/**
 * @brief 底盘运动学逆解器
 *        把底盘系速度 (vx, vy, w) → 4 个 M3508 轮电机目标角速度 (rad/s)
 *
 * 公式:  ω_i = [v_x cosθ_i + v_y sinθ_i + ω (x_i sinθ_i − y_i cosθ_i)] / r
 *
 * idx ↔ M3508 ID: idx=0,1,2,3 ↔ ID=1,2,3,4
 *
 * 【重要】 idx ↔ 物理位置(左前/右前/...)默认猜测, 上车实测后可能需
 *         翻转某些 θ_i 的符号. 调整方式是修改 kX_/kY_/kTheta_ 数组中
 *         某些行的符号.
 */
class ChassisIkSolver {
 public:
  ChassisIkSolver() = default;
  ~ChassisIkSolver() = default;

  /**
   * @brief 逆解入口(每 1ms 由 chassis 调用)
   * @param vx  沿车头方向速度 (m/s)
   * @param vy  沿车左侧方向速度 (m/s)
   * @param w   底盘自转角速度 (rad/s, 逆时针为正)
   */
  void solve(float vx, float vy, float w);

  /**
   * @brief 读第 idx 轮的目标角速度 (rad/s), 给 PID 当 setpoint
   */
  float targetVel(int idx) const;

  /* 调试接口: Ozone 里看 */
  float lastVx() const { return last_vx_; }
  float lastVy() const { return last_vy_; }
  float lastW()  const { return last_w_;  }

 private:
  /* 几何参数(出厂定, 全向轮 45° X 形, 考核标准) */
  static constexpr float kWheelRadius_   = 0.07786f;  /* m, 轮半径 */
  static constexpr float kHalfDiagonal_  = 0.21691f;  /* m, 轮心到旋转中心 L */

  /* 4 轮的轮心坐标(底盘系) 和 轮速方向角 θ_i (rad)
   * 默认 45° X 等距: |x_i| = |y_i| = L/√2 ≈ 0.15339
   * θ_i 默认顺序:  idx 0→135°, 1→-135°, 2→-45°, 3→45°  (逆时针为正)
   * ↑ 上车实测可能要翻某些 θ_i 的符号 */
  /* kX_/kY_ 的符号目前已经不代表真实物理坐标了,纯粹是用来单独调自转(w)项的
   * 系数:vx/vy 只由 kTheta_ 决定,跟 kX_/kY_ 完全无关(公式里 x_i/y_i 只出现在
   * w 那一项),所以可以放心单独调这两个数组来修自转方向,不会牵连平移。
   * 轮1、轮3 的 kX_/kY_ 同时翻了符号(相对最初版本):实测自转时 1、3 号轮子
   * 转向和 0、2 号反了,只翻其中一个会让贡献变 0(之前 2、3 号踩过这个坑),
   * 必须两个一起翻才能保持贡献不为 0 且方向正确。 */
  static constexpr float kX_[4]     = {+0.15339f, -0.15339f, -0.15339f, +0.15339f};
  static constexpr float kY_[4]     = {+0.15339f, +0.15339f, -0.15339f, -0.15339f};
  /* θ2/θ3 改回最初的 -45°/45°(不是 +45°/-45°):实测发现侧移(vy)时 2、3 号轮子
   * 转向和 0、1 号反了,vx 用 cosθ、vy 用 sinθ,只有 θ→-θ 能在保持 cosθ 不变(vx 不受影响)
   * 的前提下把 sinθ 反过来(修正 vy)。原来这组 θ 会导致自转(w)对 2、3 号轮贡献恒为 0,
   * 那个问题改用上面的 kY_ 符号来修,不再动 θ,避免 vx/vy 又被牵连。 */
  static constexpr float kTheta_[4] = { 2.3562f,  -2.3562f,  -0.7854f,   0.7854f}; /* 135°, -135°, -45°, 45° */

  /* 三档限幅 */
  static constexpr float kMaxV_        = 3.0f;       /* m/s, 平移速度 */
  static constexpr float kMaxW_        = 6.2832f;    /* rad/s, 自转速度 (=2π) */
  static constexpr float kMaxWheelVel_ = 60.0f;      /* rad/s, 单轮速度保护 */

  float target_[4]   = {0, 0, 0, 0};
  float last_vx_ = 0.0f;
  float last_vy_ = 0.0f;
  float last_w_  = 0.0f;
};

}  // namespace chassis_ik_solver
}  // namespace hello_world

/* Exported function prototypes ----------------------------------------------*/
hello_world::chassis_ik_solver::ChassisIkSolver *GetChassisIkSolver();

#endif /* CHASSIS_INSTANCE_INS_CHASSIS_IKSOLVER_HPP_ */

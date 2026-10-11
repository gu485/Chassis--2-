/**
 *******************************************************************************
 * @file      :chassis.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_HPP_

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/


namespace robot {
class Chassis {
 public:
  Chassis() = default;
  ~Chassis() = default;
  /* 下发底盘运动指令
   * @param vx: 前后平移速度 m/s
   * @param vy: 左右平移速度 m/s
   * @param w:  旋转角速度 rad/s
   * @param use_friction_ff: 是否叠加摩擦前馈电流
   *        分离模式传 true(摇杆推满时 target 较大,前馈只占小头,帮轮子克服静摩擦)
   *        跟随模式传 false(θ 很小时 IK 解出的 target 只有 0.01 量级,
   *        此时叠加 ~2A 固定摩擦电流会淹没 PID 输出,实测导致底盘疯转)
   *        默认 true,保证旧代码不改也能编译通过。
   */
  void setSpeed(float vx, float vy, float w, bool use_friction_ff = true);


};
}
/* ============================================================================
 * 【新人任务】声明底盘运动模块类 Chassis（namespace robot）。
 *
 * 你要做：
 *   1. 设计 Chassis 类，负责把底盘速度指令 (vx, vy, w) 逆解为每个轮的控制量：
 *     运动学逆解（4 个 M3508 轮电机），输出每个轮的轮速。
 *   2. 提供接口让 Robot 下发目标速度、读取解算结果。
 *
 * 约束 / 提示：
 *   - 几何参数（轮半径、轮距、轮心距、轮位坐标）见考核说明文档。
 *   - 逆解公式需自己推导（注意麦轮斜向布置角），网上「舵向轮 / 麦克纳姆轮逆运动学」资料可参考。
 *   - 参考：code_control/Omni/Omni/Chassis/RobotModules/chassis.cpp（只看思路）。
 * ==========================================================================*/
#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */

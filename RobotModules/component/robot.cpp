/**
 *******************************************************************************
 * @file      :robot.cpp
 * @brief     :底盘板顶层状态机:三态外壳 + SEPARATED/FOLLOW/SPIN 三模式派发
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
#include <cmath>  // fabsf, cosf, sinf

#include "main.h"                           // HAL_GetTick()
#include "robot.hpp"
#include "ins_comm.hpp"                     // GetRemoteControl()
#include "ins_motor.hpp"                    // GetMotorYaw()

/* === 引用外部计数器 / 急停锁存标志(定义在其它 cpp) === */
extern uint32_t      g_uart3_rx_cnt;        // comm_task.cpp:DT7 收到帧计数
extern volatile bool g_estop_latched;       // chassis.cpp  :左拨杆急停档锁存
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

/* === 分离模式参数 === */
static constexpr float kDbgMaxSpeed = 0.5f;   // 右摇杆打满对应的平移速度 m/s
static constexpr float kYawMaxVel   = 1.0f;   // 左摇杆打满对应的 yaw 角速度 rad/s
static constexpr float kYawKd       = 1.0f;   // MIT 模式"纯速度"用法阻尼系数(kp=0)

/* === 摇杆信号保护 === */
static constexpr float kRcDeadzone   = 0.05f; // 死区:回中误差容限
static constexpr float kRcValidRange = 1.2f;  // DT7 协议 [-1,1],超此值视为信号异常

static float ApplyDeadzone(float value) {
  return (value > -kRcDeadzone && value < kRcDeadzone) ? 0.0f : value;
}

static bool IsRcSignalValid(hello_world::remote_control::DT7* rc) {
  return rc != nullptr &&
         fabsf(rc->rc_rv()) <= kRcValidRange &&
         fabsf(rc->rc_rh()) <= kRcValidRange &&
         fabsf(rc->rc_lh()) <= kRcValidRange &&
         fabsf(rc->rc_lv()) <= kRcValidRange;
}

/* === 调试镜像(Ozone Symbols window 直接搜名字) === */
volatile int g_dbg_robot_state = 0;     // 0=DEAD 1=RESURRECTION 2=WORKING
volatile int g_dbg_robot_mode  = 0;     // 0=INIT 1=SEPARATED 2=FOLLOW 3=SPIN 4=EMERGENCY

volatile float   g_dbg_theta            = 0.0f;  // yaw 电机当前角度 rad
volatile bool    g_dbg_yaw_enabled      = false; // yaw 电机是否使能
volatile uint8_t g_dbg_yaw_status_code  = 0;     // 状态码:1=正常使能,其它=故障
volatile float   g_dbg_yaw_vel_cmd      = 0.0f;  // yaw 电机目标角速度

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
/* Private function definitions ----------------------------------------------*/

namespace robot{

  void Robot::update()
  {
    DataUpdate();

    /* 同步调试镜像(Ozone 观察) */
    g_dbg_robot_state = static_cast<int>(state_);
    g_dbg_robot_mode  = static_cast<int>(mode_);

    /* 三态外壳:DEAD/RESURRECTION 全零输出;WORKING 才进模式派发.
     * 进入新状态都先清指令,避免电机突变(任务注释约束 #1). */
    switch(state_){
      case LifeState::DEAD:{
        if (chassis_) chassis_->setSpeed(0,0,0);
        if (HAL_GetTick() > 200 && g_uart3_rx_cnt > 0) {
          state_ = LifeState::RESURRECTION;
          resurrection_enter_ms_ = HAL_GetTick();
          if (chassis_) chassis_->setSpeed(0,0,0);
        }
        return;
      }
      case LifeState::RESURRECTION:{
        if (chassis_) chassis_->setSpeed(0,0,0);
        if (HAL_GetTick() - resurrection_enter_ms_ > 100) {
          state_ = LifeState::WORKING;
          if (chassis_) chassis_->setSpeed(0,0,0);
        }
        return;
      }
      case LifeState::WORKING:{
        ModeUpdate();
        switch(mode_){
          case WorkMode::INIT:      run_init();      return;
          case WorkMode::EMERGENCY: run_emergency(); return;
          case WorkMode::SEPARATED: run_seperated(); return;
          case WorkMode::FOLLOW:    run_follow();    return;
          case WorkMode::SPIN:      run_spin();      return;
        }
        return;
      }
    }
  }

  void Robot::ModeUpdate()
  {
    int S1 = 0;
    int S2 = 0;
    S1 = GetRemoteControl()->rc_l_switch();
    S2 = GetRemoteControl()->rc_r_switch();

    if (S1 == 1) {
      mode_ = WorkMode::EMERGENCY;
      return;
    }
    switch (S2) {
      case 1: mode_ = WorkMode::SEPARATED; return;
      case 3: mode_ = WorkMode::FOLLOW;    return;
      case 2: mode_ = WorkMode::SPIN;      return;
    }
  }

  void Robot::DataUpdate() {}

  void Robot::RegisterChassis(Chassis* c) {
    chassis_ = c;
  }

  void Robot::run_init() {}

  void Robot::run_emergency() {
    /* 急停:四轮 + yaw 电机都归零(急停锁存时 chassis_->setSpeed() 内部已经置 0,
     * 这里再显式发一次确保). */
    if (chassis_) chassis_->setSpeed(0,0,0);
    GetMotorYaw()->setMitInput(0.0f, 0.0f, 0.0f, kYawKd, 0.0f);
  }

  void Robot::run_seperated()
  {
    /* === 真正的分离模式 ===
     * 核心:把摇杆系(驾驶员相对云台朝向)的速度按 yaw 角度 θ 旋转到底盘系.
     * 底盘朝向不变(w=0),云台可被左摇杆横拨单独控制.
     *
     * 数据流:
     *   rc_rv / rc_rh → vx_stick / vy_stick(死区)
     *                → 乘旋转矩阵(cos θ, sin θ)→ vx / vy(底盘系)
     *   rc_lh        → yaw_vel → MIT 模式 setMitInput(vel_des) */
    if (!chassis_) return;
    auto* rc = GetRemoteControl();
    if (!rc) {
      chassis_->setSpeed(0,0,0);
      GetMotorYaw()->setMitInput(0,0,0,kYawKd,0);
      return;
    }

    /* 信号异常保护:垃圾值全零(超出协议范围 → 接触不良/干扰/极性接反) */
    if (!IsRcSignalValid(rc)) {
      chassis_->setSpeed(0.0f, 0.0f, 0.0f);
      GetMotorYaw()->setMitInput(0.0f, 0.0f, 0.0f, kYawKd, 0.0f);
      return;
    }

    /* 摇杆系速度(死区处理):rc_rh 取负号是因为 DT7 推左为正、车系左为负 */
    float vx_stick =  kDbgMaxSpeed * ApplyDeadzone(rc->rc_rv());
    float vy_stick = -kDbgMaxSpeed * ApplyDeadzone(rc->rc_rh());

    /* 读 yaw 电机角度,做调试镜像 */
    auto* yaw = GetMotorYaw();
    float theta = yaw->angle();
    g_dbg_theta           = theta;
    g_dbg_yaw_enabled     = yaw->is_enabled();
    g_dbg_yaw_status_code = yaw->status_code();

    /* 摇杆系 → 底盘系旋转矩阵(逆时针旋转 θ):
     *   vx = vx_s·cos θ - vy_s·sin θ
     *   vy = vx_s·sin θ + vy_s·cos θ
     * 物理意义:把云台坐标系下的"前进 / 左移"分量投影到底盘坐标系. */
    float vx = vx_stick * cosf(theta) - vy_stick * sinf(theta);
    float vy = vx_stick * sinf(theta) + vy_stick * cosf(theta);

    /* 分离模式:底盘朝向不变(w=0),平移跟着云台方向走 */
    chassis_->setSpeed(vx, vy, 0.0f);

    /* 左摇杆横拨 → yaw 角速度.MIT 纯速度用法 kp=0, kd=kYawKd 只传 vel_des.
     * 急停锁存时强制归零,跟轮子共用同一个急停开关. */
    float yaw_vel = kYawMaxVel * ApplyDeadzone(rc->rc_lh());
    if (g_estop_latched) yaw_vel = 0.0f;
    g_dbg_yaw_vel_cmd = yaw_vel;
    yaw->setMitInput(0.0f, yaw_vel, 0.0f, kYawKd, 0.0f);
  }

  void Robot::run_follow() {}
  void Robot::run_spin() {}

}


/* ============================================================================
 * 【新人任务】实现 robot.hpp 中声明的状态机。
 *
 * 你要做：
 *   1. 周期入口：按「更新数据 → 状态机迁移 → 生成指令 → 下发」的顺序组织 update()。
 *   2. 三态实现：Dead 时输出全零并等待就绪条件；Resurrection 时做自检与复位（如等待
 *      电机使能、IMU 零漂收敛）；Working 时执行正常控制逻辑。
 *   3. 各工作模式（分离 / 随动 / 小陀螺）的指令生成分支。
 *
 * 约束 / 提示：
 *   - 状态切换时记得复位上一状态残留的指令，避免电机突变。
 *   - 所有下发指令统一经过本类，急停判断放在最外层，确保任何分支都先过急停。
 * ==========================================================================*/

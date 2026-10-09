/**
 *******************************************************************************
 * @file      :ins_motor.cpp
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
#include "ins_motor.hpp"
#include "motor.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
static constexpr int kWheelCount = 4;  // 4 个轮电机
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::motor::Motor *GetMotorWheel(int idx) {
  if (idx < 0 || idx >= kWheelCount) {
    return nullptr;  // 越界保护
  }

  // 函数内 static = 全局唯一 + 首次调用时创建
  static hello_world::motor::Motor *motors[kWheelCount] = {nullptr};

  if (motors[idx] == nullptr) {
    // M3508 配置:电流输入模式,正向(上车后实际测,可能要改 Dir)
    hello_world::motor::OptionalParams opt;
    opt.input_type = hello_world::motor::InputType::kCurr;
    opt.dir = hello_world::motor::kDirFwd;
    // 接 14:1 外置减速箱(从接线图确认)
    opt.ex_redu_rat = 14.0f;
    opt.remove_build_in_reducer = false;  // 保留电机自带的减速器
    // M3508 ID 是 1~4,idx 是 0~3,差 1
    motors[idx] = hello_world::motor::CreateMotor(
        hello_world::motor::MotorType::kM3508,
        static_cast<uint8_t>(idx + 1),
        opt);
  }
  return motors[idx];
}

hello_world::motor::DM_J4310 *GetMotorYaw() {
  // 函数内 static = 全局唯一 + 首次调用时创建
  static hello_world::motor::DM_J4310 *motor = nullptr;

  if (motor == nullptr) {
    // 达妙电机只认力矩输入类型(HW_ASSERT 强制检查),方向先给正向,
    // 上车实测转向不对再改成 kDirRev
    hello_world::motor::OptionalParams opt;
    opt.dir = hello_world::motor::kDirFwd;
    opt.input_type = hello_world::motor::InputType::kTorq;
    // angle_range 限制在 [-π, π):驱动开机收到第一帧反馈时,内部"过零计圈"逻辑会把
    // 上次角度的默认值(0 rad,对应 -π)和真实角度一比,大概率误判成"转了一整圈",
    // 给圈数计数器凭空加了 ±1,导致 angle() 一直带着约 ±2π 的固定偏移(实测正是
    // -5.7 左右)。用 kNegPiToPosPi 让 normAngle() 把角度自动折回 [-π, π),
    // 抵消这个偏移;分离模式只关心 θ 的 cos/sin,本来就不需要无限累计角度。
    opt.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi;
    // 实测零点补偿:把云台转到正对底盘正前方,此时 yaw->angle() ≈ -1.3 rad(~-74.5°)
    // 用 angle_offset 扣掉这个机械偏差,以后 angle() = 0 就代表云台正对正前方
    opt.angle_offset = -1.3f;
    // enable_mode_switch=false:实测速度模式(setVelInput)不可靠——怀疑总线不稳定
    // 导致模式切换的寄存器握手(写CTRL_MODE→等确认帧)经常卡死,或者电机被切到了
    // 速度模式后仍收不到可靠的控制帧。改用 MIT 模式的纯速度用法(kp=0、kd给非0、
    // 只传 vel_des)替代,不需要任何模式切换握手,默认构造就是 MIT 模式,更稳定。
    motor = new hello_world::motor::DM_J4310(
        /*id=*/2, opt, /*auto_enable=*/true, /*enable_mode_switch=*/false);
  }
  return motor;
}

/* Private function definitions ----------------------------------------------*/
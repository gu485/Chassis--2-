/**
 *******************************************************************************
 * @file      :ins_robot.cpp
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
#include "ins_robot.hpp"
#include "robot.hpp"
#include "chassis.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
namespace {
robot::Robot   g_robot;     // ← 程序启动就构造 Robot 对象
robot::Chassis g_chassis;   // ← 程序启动就构造 Chassis 对象
}

namespace robot {

Robot *GetRobot() {
  return &g_robot;  // 返回地址,全局唯一
}

Chassis *GetChassis() {
  return &g_chassis;  // 返回地址,全局唯一
}

}  // namespace robot

/* Private function definitions ----------------------------------------------*/

/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.cpp
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
#include "ins_chassis_gimbal_comm.hpp"
#include "gimbal_chassis_comm.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm *GetGimbalChassisComm() {
  static robot::GimbalChassisComm comm(
      robot::GimbalChassisComm::CodePart::Chassis,
      robot::kChassisToGimbalId,   // tx_id:本板(底盘)发送用这个 ID
      robot::kGimbalToChassisId);  // rx_id:本板(底盘)接收用这个 ID
  return &comm;
}

/* Private function definitions ----------------------------------------------*/

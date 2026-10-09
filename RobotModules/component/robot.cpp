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
#include "ins_pid.hpp"    // GetPidFollow()
#include "ins_imu.hpp"                     // GetImu()
#include "imu.hpp"                          // Imu 类完整定义(roll/pitch/roll 输出参数风格)
#include "pid.hpp"        // hello_world::pid::Pid 的完整定义(calc/reset)
#include "ins_chassis_gimbal_comm.hpp"   // GetGimbalChassisComm()
#include "gimbal_chassis_comm.hpp"       // robot::GimbalChassisComm 完整定义
/* === 引用外部计数器 / 急停锁存标志(定义在其它 cpp) === */
extern uint32_t      g_uart3_rx_cnt;        // comm_task.cpp:DT7 收到帧计数
extern volatile bool g_estop_latched;       // chassis.cpp  :左拨杆急停档锁存
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

/* === 分离模式参数 === */
static constexpr float kDbgMaxSpeed = 0.07f;   // 右摇杆打满对应的平移速度 m/s
static constexpr float kYawMaxVel   = 1.0f;   // 左摇杆打满对应的 yaw 角速度 rad/s
static constexpr float kYawKd       = 2.0f;   // MIT 模式"纯速度"用法阻尼系数(kp=0)

/* === 小陀螺模式参数 === */
/* 摇杆推满对应的自转角速度(rad/s)。3.0 rad/s ≈ 172°/s,
 * 大约 2.1 秒转一圈。麦克纳姆轮高速自旋时容易打滑,
 * 起步先用这个保守值,不够快再往上调。 */
/* 编译期标定的满量程角速度。现在 run_spin() 实际下发用的是下面的
 * g_dbg_spin_omega(volatile,Ozone 可实时改),这个 constexpr 只作为
 * "标定参考值"保留 —— 两者调好后保持一致即可。
 * 3.0 rad/s ≈ 172°/s,约 2.1 秒转一圈;麦克纳姆轮高速自旋容易打滑。 */
[[maybe_unused]] static constexpr float kSpinOmega = 1.5f;
/* 转速死区(摇杆归一化后的值)。推过这个量才给转速,
 * 避免"手一碰摇杆底盘就转"。0.25 对应摇杆约 1/4 行程。 */
static constexpr float kSpinMinStick = 0.4f;
/* w 斜率限制(rad/s 每 1ms 拍):0.003 → 约 0.5 秒从 0 加到 1.5 rad/s */
static constexpr float kSpinWRamp = 0.003f;

/* === 小陀螺目标角速度(Ozone 可实时改) ===
 *
 * g_dbg_spin_omega:摇杆推满时对应的自转角速度 rad/s。
 * 现场调转速直接改这个变量,**不用重新编译烧录**。
 *
 * 注意 run_spin() 里下发用的是这个变量,不是 constexpr 的 kSpinOmega ——
 * kSpinOmega 只是"编译期标定值",真正生效的是这个 volatile。
 * 调好之后把两者的默认值设成一致,代码读起来不困惑。
 *
 * 参考换算:
 *   1.0 rad/s ≈ 57°/s → 6.3 秒一圈
 *   1.5 rad/s ≈ 86°/s → 4.2 秒一圈
 *   2.0 rad/s ≈ 115°/s → 3.1 秒一圈
 *   3.0 rad/s ≈ 172°/s → 2.1 秒一圈 */
volatile float g_dbg_spin_omega = 1.5f;

/* === 跟随模式参数 ===
 * w = kFollowKp * θ
 * θ = 云台相对底盘的夹角(rad,已在 ins_motor.cpp 里用 angle_offset 标定为 0)
 * 底盘朝 θ 的方向转,就能把云台"追"回正前方,θ 自然收敛到 0。
 * 起步给 1.0;觉得跟太肉就加大,出现来回摆就减小。 */
static constexpr float kFollowKp    = 1.0f;

/* === 摇杆信号保护 === */
static constexpr float kRcDeadzone   = 0.05f; // 死区:回中误差容限
static constexpr float kRcValidRange = 1.2f;  // DT7 协议 [-1,1],超此值视为信号异常

/* === 跟随模式角度死区 ===
 * |θ| < 这个值就认为底盘已跟到位,角速度 w 直接归零。
 * 0.07 rad ≈ 4°,留一点机械装配误差的余量。
 * 可按机械精度要求调整:要跟得更紧就调小,底盘摆得厉害就调大。 */
static constexpr float kFollowThetaDeadZone = 0.07f;

static float ApplyDeadzone(float value) {
  return (value > -kRcDeadzone && value < kRcDeadzone) ? 0.0f : value;
}

/* === 角度回绕:把任意角度折回 [-π, π) ===
 *
 * 跟随模式的误差是"云台世界朝向 − 底盘世界朝向",两个量都在 [0, 2π)
 * 累积(IMU 积分不会自动折回),相减可能是 ±2π 附近的值。
 * 不折回的话,底盘会把一次微小的偏差当成"差了一整圈",
 * 原地掉头转一整圈。
 *
 * 这个函数同时被云台板 Gimbal::WrapAngle 用,两边逻辑必须一致,
 * 所以各自实现一份而不是放共享头(免得为了一个函数增加耦合)。 */
static constexpr float kPi = 3.14159265f;
static float WrapAngle(float a) {
  constexpr float kTwoPi = 2.0f * kPi;
  while (a >  kPi) { a -= kTwoPi; }
  while (a < -kPi) { a += kTwoPi; }
  return a;
}

static bool IsRcSignalValid(hello_world::remote_control::DT7* rc) {
  return rc != nullptr &&
         fabsf(rc->rc_rv()) <= kRcValidRange &&
         fabsf(rc->rc_rh()) <= kRcValidRange &&
         fabsf(rc->rc_lh()) <= kRcValidRange &&
         fabsf(rc->rc_lv()) <= kRcValidRange;
}

/* === 调试镜像(Ozone Symbols window 直接搜名字) === */
/* 跟随模式参数 */
/* yaw_pid_reset_pending:第一次采样 offset 时顺便复位跟随 PID。
 * 进入跟随的那一刻目标从"未知"跳到"当前对齐位置",不复位的话
 * PID 内部的 last_error 会有残值,第一拍输出一个假偏差。 */
static bool yaw_pid_reset_pending = true;

volatile int g_dbg_robot_state = 0;     // 0=DEAD 1=RESURRECTION 2=WORKING
volatile int g_dbg_robot_mode  = 0;     // 0=INIT 1=SEPARATED 2=FOLLOW 3=SPIN 4=EMERGENCY

volatile float   g_dbg_theta            = 0.0f;  // yaw 电机当前角度 rad
volatile bool    g_dbg_yaw_enabled      = false; // yaw 电机是否使能
volatile uint8_t g_dbg_yaw_status_code  = 0;     // 状态码:1=正常使能,其它=故障
/* 故障锁存:记住最后一个"非 0 非 1"的故障码。
 * enabled 会在 0/1 之间快速跳变,status_code 同理,Ozone 刷新有 100ms 间隔,
 * 很容易只采到 0 就错过故障瞬间。这个变量只写不清,故障码一旦出现就留住。 */
volatile uint8_t g_dbg_yaw_last_fault  = 0;     // 0=还没抓到过故障
volatile float   g_dbg_yaw_vel_cmd      = 0.0f;  // yaw 电机目标角速度
volatile float   g_dbg_w_cmd            = 0.0f;  // 底盘目标角速度 w(区分三模式的关键量)
volatile float g_dbg_w_err = 0.0f;
volatile float g_dbg_w_raw = 0.0f;
/* 云台电机实际角速度(rad/s,由 DM4310 反馈帧解出)。
 * 跟 g_dbg_yaw_vel_cmd(我们下发的 vel_des)对比:
 *   两者接近      → 电机跟得上指令,补偿逻辑有问题
 *   real 远小于 cmd → 电机出力不足,kd 太小 */
volatile float g_dbg_yaw_vel_real = 0.0f;
/* 跟随模式下发的云台最终下发速度指令(yaw_vel − w)。云台板接管后这个值
 * 只用于调试镜像,底盘不再把它发给电机。 */
volatile float g_dbg_yaw_final = 0.0f;

/* === 跟随模式调试(新架构:两个世界朝向的差) === */
/* psi_gimbal  : 云台世界朝向 rad(经双板通讯从云台板 IMU 传来)
 * psi_chassis : 底盘世界朝向 rad(本板 IMU)
 * err         : 回绕后的误差 rad,这是底盘 PID 的输入
 * valid       : 1 = 数据齐全且 PID 在跑;0 = 双板离线/IMU 没标定完,w 强制为 0 */
volatile float g_dbg_follow_psi_gimbal  = 0.0f;
volatile float g_dbg_follow_psi_chassis = 0.0f;
volatile float g_dbg_follow_err         = 0.0f;
volatile bool  g_dbg_follow_valid       = false;

/* === 小陀螺调试 === */
/* spin_stick : 右摇杆上下原始值(死区处理前),拨动时看它变不变
 * w_cmd      : 实际下发的自转角速度 rad/s
 *
 * 注意 w_cmd 是所有模式共用的(g_dbg_w_cmd),小陀螺下它就是自转角速度,
 * 分离/跟随下它是底盘跟随的角速度。要区分就看当前 mode 值。 */
volatile float g_dbg_spin_stick = 0.0f;
/* 小陀螺平移参考系:
 * ref_valid : 1 = 双板/IMU 数据齐全,已采到基准,平移按世界系锁定
 *             0 = 数据不可用,退化成车头方向(基准也会重采)
 * ref_yaw   : 平移参考系相对车头的偏角 rad。车身自转时它持续变化,
 *             但"推前 = 云台指的方向"这件事不变 —— 这就是世界锁定。 */
volatile bool  g_dbg_spin_ref_valid = false;
volatile float g_dbg_spin_ref_yaw   = 0.0f;

/* 小陀螺平移世界锁定的基准:进入模式时冻结的云台世界朝向(rad)。
 * 这是常量,之后不再更新 —— 云台自转期间的抖动不能影响平移参考系。 */
static bool  spin_ref_valid = false;
static float spin_ref_psi = 0.0f;

/* === IMU 调试镜像(搜 g_dbg_imu 即可找到所有) === */
/* init_err:0=通,bit0=加速度计 ID 错,bit1=陀螺仪 ID 错,bit2/3=自检失败,bit4/5=配置报错 */
volatile uint8_t g_dbg_imu_init_err     = 0;
volatile bool    g_dbg_imu_calib_done   = false;   /* 上电后 1 秒变 true */
/* 当前姿态角(rad,水平放置应全 ≈ 0)*/
volatile float g_dbg_imu_roll    = 0.0f;          /* 翻滚角 */
volatile float g_dbg_imu_pitch   = 0.0f;          /* 俯仰角 */
volatile float g_dbg_imu_yaw     = 0.0f;          /* 偏航角(给小陀螺用) */
/* 原始读数: 单位 g,水平静止 |acc|≈9.8(用来验证 BMI088 接线对不对) */
volatile float g_dbg_imu_acc[3]  = {0, 0, 0};
/* 原始陀螺仪: 单位 rad/s,静止应 ≈ 0(验证陀螺仪接线 + 标定) */
volatile float g_dbg_imu_gyro[3] = {0, 0, 0};
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

    /* IMU 调试镜像(搜 g_dbg_imu 可全找到) */
    {
      auto* imu = GetImu();
      if (imu != nullptr) {
        g_dbg_imu_init_err   = static_cast<uint8_t>(imu->init_err());
        g_dbg_imu_calib_done = imu->calib_done();
        /* 临时变量中转:避免 volatile float* 与 float* 类型不匹配 */
        float r, p, y;
        float a[3], g_[3];
        imu->roll (&r);
        imu->pitch(&p);
        imu->yaw  (&y);
        imu->acc  (a);
        imu->gyro (g_);
        g_dbg_imu_roll  = r;
        g_dbg_imu_pitch = p;
        g_dbg_imu_yaw   = y;
        for (int i = 0; i < 3; i++) {
          g_dbg_imu_acc[i]  = a[i];
          g_dbg_imu_gyro[i] = g_[i];
        }
      }
    }

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

  void Robot::DataUpdate()
  {
    /* 「无条件采集」层:不管当前是 DEAD / RESURRECTION / WORKING,也不管是哪个工作
     * 模式,每 1ms 都执行一次。所以调试镜像放在这里,任何状态下 Ozone 看到的都是
     * 真实值,而不是某个模式没跑到时的陈旧初值。
     *
     * 之前这些镜像写在 run_seperated() / run_follow() 里,一旦状态机卡在 DEAD
     * 或没切到对应档位,它们就永远停在初值(status_code=0、enabled=false),
     * 排查时会被误导成"电机失能",实际根本没采过。 */
    auto* yaw = GetMotorYaw();
    g_dbg_theta           = yaw->angle();
    g_dbg_yaw_enabled     = yaw->is_enabled();
    g_dbg_yaw_status_code = yaw->status_code();

    /* 故障锁存:status_code 是 0(失能) / 1(使能) / 8~15(故障) 三类。
     * 只把"故障"那一类(既不是 0 也不是 1)钉住不动,Ozone 里就能看到
     * 电机到底报的是过压、欠压、过流、超温还是通信丢失。 */
    const uint8_t sc = static_cast<uint8_t>(yaw->status_code());
    if (sc != 0 && sc != 1) g_dbg_yaw_last_fault = sc;

    /* 云台实际角速度(电机反馈,rad/s)。跟 g_dbg_yaw_vel_cmd(我们发出的指令)对比,
     * 就能判断"指令对不对"还是"电机跟不上指令"。
     * 排查跟随模式时很关键:云台被底盘带着走,如果是 vel 远小于 vel_cmd,
     * 说明电机出力不足(kd 太小),而不是补偿量算错了。 */
    g_dbg_yaw_vel_real = yaw->vel();

    /* 指令类镜像:先清零,真正的模式函数(如果有)会在本周期后面重新赋值。
     * 这样 DEAD / 急停 时看到的是 0,而不是上一次残留的非零值。 */
    g_dbg_yaw_vel_cmd = 0.0f;
    g_dbg_w_cmd       = 0.0f;
  }

  void Robot::RegisterChassis(Chassis* c) {
    chassis_ = c;
  }

  void Robot::run_init() {}

  void Robot::run_emergency() {
    /* 急停:四轮 + yaw 电机都归零(急停锁存时 chassis_->setSpeed() 内部已经置 0,
     * 这里再显式发一次确保). */
    if (chassis_) chassis_->setSpeed(0,0,0);
    g_dbg_w_cmd = 0.0f;
      /* 云台板接管 yaw 后,底盘不再下发电机帧:GetMotorYaw()->setMitInput(0.0f, 0.0f, 0.0f, kYawKd, 0.0f); */
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
      /* 云台板接管 yaw 后,底盘不再下发电机帧:GetMotorYaw()->setMitInput(0.0f, 0.0f, 0.0f, kYawKd, 0.0f); */
      return;
    }

    /* 信号异常保护:垃圾值全零(超出协议范围 → 接触不良/干扰/极性接反) */
    if (!IsRcSignalValid(rc)) {
      chassis_->setSpeed(0.0f, 0.0f, 0.0f);
      /* 云台板接管 yaw 后,底盘不再下发电机帧:GetMotorYaw()->setMitInput(0.0f, 0.0f, 0.0f, kYawKd, 0.0f); */
      return;
    }

    /* 摇杆系速度(死区处理):rc_rh 取负号是因为 DT7 推左为正、车系左为负 */
    float vx_stick =  kDbgMaxSpeed * ApplyDeadzone(rc->rc_rv());
    float vy_stick = -kDbgMaxSpeed * ApplyDeadzone(rc->rc_rh());

    /* yaw 角度的采集与调试镜像已移到 DataUpdate()（无条件执行,任何状态都有真值） */
    auto* yaw = GetMotorYaw();
    float theta = yaw->angle();

    /* 摇杆系 → 底盘系旋转矩阵(逆时针旋转 θ):
     *   vx = vx_s·cos θ - vy_s·sin θ
     *   vy = vx_s·sin θ + vy_s·cos θ
     * 物理意义:把云台坐标系下的"前进 / 左移"分量投影到底盘坐标系. */
    float vx = vx_stick * cosf(theta) - vy_stick * sinf(theta);
    float vy = vx_stick * sinf(theta) + vy_stick * cosf(theta);

    /* 分离模式:底盘朝向不变(w=0),平移跟着云台方向走。
     * 第四个参数 true = 叠加摩擦前馈(摇杆推满时 target 较大,前馈有用) */
    g_dbg_w_cmd = 0.0f;          // 分离模式的 w 恒为 0
    chassis_->setSpeed(vx, vy, 0.0f, /*use_friction_ff=*/true);

    /* 左摇杆横拨 → yaw 角速度.
     * 云台板已接管 yaw 电机闭环(它用自己的 IMU 做世界锁定),
     * 底盘这里只保留调试镜像,实际下发由 comm_task 通过双板通讯
     * 把 rc_lh 原始值发给云台板。绝不能在这里再 setMitInput ——
     * 两个节点同时驱同一个电机 ID 会让 CAN 仲裁随机,云台抽搐。 */
    float yaw_vel = kYawMaxVel * ApplyDeadzone(rc->rc_lh());
    if (g_estop_latched) yaw_vel = 0.0f;
    g_dbg_yaw_vel_cmd = yaw_vel;
  }

  void Robot::run_follow()
  {
    
    /* === 跟随模式 ===
     * 跟分离模式只差一处:底盘角速度 w 的算法。
     *   分离 : w = 0            → 底盘朝向锁死,只有云台在转
     *   跟随 : w = kFollowKp*θ  → 底盘朝云台方向追,θ 收敛回 0
     *
     * 原理:θ 就是"云台偏离底盘正前方多少"。底盘朝 θ 的方向转 θ 弧度,
     * 两者就重新同向。因为云台装在底盘上方,"追"的动作直接用 θ 当误差即可,
     * 不需要 IMU、不需要积分、不需要额外传感器。
     *
     * ⚠ 上机后如果底盘是"反着追"(拨左摇杆,底盘往右转),把下面 w 那行改成
     *   float w = -kFollowKp * theta;
     * 正负号取决于 yaw 电机 dir 配置和云台安装朝向,只能实测确定。 */
    
     if (!chassis_) return;
    auto* rc = GetRemoteControl();
    if (!rc) {
      chassis_->setSpeed(0,0,0);
      /* 云台板接管 yaw 后,底盘不再下发电机帧:GetMotorYaw()->setMitInput(0.0f, 0.0f, 0.0f, kYawKd, 0.0f); */
      return;
    }

    /* 信号异常保护:垃圾值全零 */
    if (!IsRcSignalValid(rc)) {
      chassis_->setSpeed(0.0f, 0.0f, 0.0f);
      /* 云台板接管 yaw 后,底盘不再下发电机帧:GetMotorYaw()->setMitInput(0.0f, 0.0f, 0.0f, kYawKd, 0.0f); */
      return;
    }

    /* 急停锁存:跟分离模式同一个开关 */
    if (g_estop_latched) {
      chassis_->setSpeed(0.0f, 0.0f, 0.0f);
      /* 云台板接管 yaw 后,底盘不再下发电机帧:GetMotorYaw()->setMitInput(0.0f, 0.0f, 0.0f, kYawKd, 0.0f); */
      return;
    }


    /* yaw 角度的采集与调试镜像已移到 DataUpdate()（无条件执行,任何状态都有真值） */
    /* 平移方向仍用云台相对底盘的编码器角 theta。
     * 为什么这个还用编码器而不用 IMU:平移指令是"底盘系 → 世界系"的旋转,
     * 关心的是两个机械轴之间的**相对**关系,而 theta 恰好就是这个相对量,
     * 且是本板直连电机反馈的,没有通信延迟。
     * (云台的世界朝向要走 CAN 往返,有 0.5~1ms 延迟,用在平移方向上
     *  会让整体运动有拖滞感。) */
    auto* yaw = GetMotorYaw();
    float theta = yaw->angle();

    /* 摇杆 → 云台目标角速度。
     * 注意:这里只发给底盘自己的调试镜像和云台板(通过双板通讯),
     * 底盘**不再**驱云台电机 —— 云台板的 Gimbal 类负责闭环。 */
    float yaw_vel = kYawMaxVel * ApplyDeadzone(rc->rc_lh());
    g_dbg_yaw_vel_cmd = yaw_vel;

    /* 摇杆系 → 底盘系(和平移模式完全一样) */
    float vx_stick =  kDbgMaxSpeed * ApplyDeadzone(rc->rc_rv());
    float vy_stick = -kDbgMaxSpeed * ApplyDeadzone(rc->rc_rh());
    float vx = vx_stick * cosf(theta) - vy_stick * sinf(theta);
    float vy = vx_stick * sinf(theta) + vy_stick * cosf(theta);

    /* === 跟随模式核心:底盘追云台的世界朝向 ===
     *
     * 架构:两个环各自闭环,完全解耦。
     *   云台环(在云台板):摇杆 → 目标角度 → 云台 PID → 力矩。
     *     误差用云台自己的 IMU,所以底盘怎么转都不影响云台的"世界朝向"。
     *   底盘环(在底盘板):云台世界朝向 − 底盘世界朝向 → w。
     *     两个角都在世界系,底盘转动会被自己的 IMU 立刻看到并反向补偿,
     *     所以云台拨到哪,底盘最终都会转到和它同向。
     *
     * 相比旧的"云台直接给云台补偿 -w"的做法,新架构的云台**不再需要**
     * 知道底盘转了多少(那个 kFollowThetaBoost ×8 的放大 hack 也就没了),
     * 因为两个环互不干扰,不需要事后补偿。 */

    float follow_err = 0.0f;
    float w = 0.0f;
    bool  follow_valid = false;
    /* 两块板 IMU 零点独立,需要一个基准把它们对齐。声明在 if 外,
     * 因为下面的 else 分支要把它作废。 */
    static bool  offset_valid = false;
    static float offset = 0.0f;

    auto* comm = GetGimbalChassisComm();
    auto* imu  = GetImu();

    if (comm != nullptr && imu != nullptr &&
        !comm->isOffline() && imu->calib_done()) {

      /* 云台世界朝向:云台板 IMU 解算,经双板通讯传来。
       * imu_ready 必须在云台板上置位后才有效 —— 标定完成前 euler_ 全是 0,
       * 发过来是假值,拿它当方向会让底盘朝错的方向猛转。 */
      if (comm->gimbal_data().imu_ready != 0u) {
        const float psi_gimbal = comm->gimbal_data().gimbal_yaw_world;

        float psi_chassis = 0.0f;
        imu->yaw(&psi_chassis);

        /* 两块板的 IMU 零点独立:云台的 ψ=0 是云台上电那一刻的朝向,
         * 底盘的 ψ=0 是底盘上电那一刻。两者装配关系和上电时刻都不保证一致,
         * 直接相减会得到一个恒定的偏差(可能几十度),底盘会一直朝一个方向转。
         *
         * 解法:进入跟随模式时采样一次基准,之后用差值。
         * 这个基准只需采一次 —— 两个 IMU 的零点一旦确定就不再变。 */
        if (!offset_valid) {
          offset = WrapAngle(psi_gimbal - psi_chassis);
          offset_valid = true;
          if (yaw_pid_reset_pending) {
            if (auto* p = GetPidFollow()) p->reset();
            yaw_pid_reset_pending = false;
          }
        }

        /* 误差(必须回绕,否则 ±2π 附近的值会让底盘掉头一圈) */
        follow_err = WrapAngle(psi_gimbal - psi_chassis - offset);
        g_dbg_follow_psi_gimbal  = psi_gimbal;
        g_dbg_follow_psi_chassis = psi_chassis;

        auto* pid = GetPidFollow();
        w = (pid != nullptr) ? -pid->calc(0.0f, follow_err) : 0.0f;

        /* 死区:|误差| < kFollowThetaDeadZone 时认为已跟到位,w 归零。
         * 不加的话误差在 0 附近抖动 → w 跟着抖 → 底盘持续小幅旋转。 */
        if (fabsf(follow_err) < kFollowThetaDeadZone) {
          w = 0.0f;
        }
        follow_valid = true;
      }
    } else {
      /* 双板离线 / 云台 IMU 没标定完 / 底盘 IMU 没标定完。
       * 此时 w 保持 0 —— 宁可底盘不跟,也不要拿假数据转。
       * offset 也作废,下次条件满足时重新采样。 */
      offset_valid = false;
      g_dbg_follow_psi_gimbal  = 0.0f;
      g_dbg_follow_psi_chassis = 0.0f;
    }

    g_dbg_follow_err    = follow_err;
    g_dbg_follow_valid  = follow_valid;
    g_dbg_w_cmd = w;

    chassis_->setSpeed(vx, vy, w, /*use_friction_ff=*/false);
  }


  void Robot::run_spin()
  {
    /* === 小陀螺模式 ===
     *
     * 行为:底盘按固定角速度自转,云台靠自己的 IMU 世界锁定保持朝向不变。
     *
     * 为什么云台什么都不用做:云台板不知道当前是什么模式,只跟着底盘
     * 经双板通讯发来的 rc_lh 走。小陀螺下底盘会额外发一个
     * "云台使能=0"的标志位,云台收到后停止累积目标角度,纯靠自己的
     * IMU 世界锁定保持朝向 —— 底盘转多少,它主动转回来补偿多少,
     * 始终指着进入小陀螺时的世界方向。这正是小陀螺模式的意义。
     *
     * 
     * 底盘这边只管一件事:输出一个恒定的 w。
     *
     * 摇杆分配:
     *   左摇杆上下 rc_lv → 前进后退
     *   左摇杆左右 rc_lh → 左右横移(本模式独占,云台让出通道)
     *   右摇杆上下 rc_rv → 自转角速度
     *   右摇杆左右 rc_rh → 空闲
     *
     * 平移方向按"世界系锁定":进模式那一刻云台指向哪,推前就一直往
     * 那个世界方向走,车身怎么转都不变。详见下面 ref_yaw 那段注释。 */
    if (!chassis_) return;
    auto* rc = GetRemoteControl();

    if (rc == nullptr || !IsRcSignalValid(rc)) {
      chassis_->setSpeed(0.0f, 0.0f, 0.0f);
      return;
    }
    if (g_estop_latched) {
      chassis_->setSpeed(0.0f, 0.0f, 0.0f);
      return;
    }

    /* 右摇杆上下 → 自转角速度。
     * 死区之外线性映射到 ±kSpinOmega,摇杆回中 w 就是 0,底盘停转。 */
    float spin_stick = ApplyDeadzone(rc->rc_rv());

    /* 转速死区 + 连续映射:
     * 推过 kSpinMinStick 才开始给转速,但从 0 平滑爬升(不是跳到 kSpinMinStick×omega)。
     * 原来的写法在 0.4 处有阶跃,一跨过去 w 瞬间跳到 0.6 rad/s,
     * 轮子速度环 kp 很大,四个轮子同时顶满电流,表现为"一推就狂转"。 */
    float w_target = 0.0f;
    const float abs_stick = fminf(fabsf(spin_stick), 1.0f);
    if (abs_stick > kSpinMinStick) {
      const float mag = (abs_stick - kSpinMinStick) / (1.0f - kSpinMinStick);
      w_target = copysignf(mag * g_dbg_spin_omega, spin_stick);
    }

    /* 斜率限制:w 每拍最多变 kSpinWRamp。目标阶跃时轮速目标也是斜坡,
     * 速度环不会饱和。断档(切走又切回)超过 5ms 没被调用就从 0 重新起爬,
     * 避免带着上次的残留 w 起步。 */
    static float w_out = 0.0f;
    static uint32_t last_ms = 0;
    const uint32_t now_ms = HAL_GetTick();
    if (now_ms - last_ms > 5) w_out = 0.0f;
    last_ms = now_ms;

    if (w_target > w_out + kSpinWRamp)      w_out += kSpinWRamp;
    else if (w_target < w_out - kSpinWRamp) w_out -= kSpinWRamp;
    else                                    w_out = w_target;
    const float w = w_out;

    /* === 平移方向:世界系锁定 ===
     *
     * 需求:进小陀螺那一刻云台指向哪,"前"就一直指哪个世界方向,
     * 车身怎么转都不变。
     *
     * 实现:进模式时把云台的世界朝向**冻结**成一个常量基准,
     * 之后每拍只用它减底盘的实时朝向:
     *
     *   ref_yaw = spin_ref_psi(常量) − ψ_chassis(实时)
     *
     * 底盘顺时针转 90° → ref_yaw 变成 −90° → 推"前"在世界系下
     * 依然指向进模式时云台指的方向。
     *
     * ⚠ 关键:基准必须是冻结的常量,不能每拍重算 (ψ_gimbal − ψ_chassis)。
     *   自转时云台被底盘带着偏、再被自己的 IMU 闭环拉回来,ψ_gimbal
     *   持续抖动;如果每拍减这个实时值,参考系就跟着抖,
     *   现象是"走一点方向就飘"。云台想指向哪在进入模式时就定了,
     *   之后不该再变。
     *
     * 为什么不能用编码器的 theta:它是"云台相对底盘"的夹角,
     * 底盘自转时剧烈变化,不能用来确定世界方向。
     *
     * 数据不可用(双板离线/IMU 没标定完)时退化成底盘系:
     * 推"前"往车头走。这比完全不动要好,至少还能开回去。 */
    float vx = 0.0f;
    float vy = 0.0f;
    float ref_yaw = 0.0f;

    auto* spin_comm = GetGimbalChassisComm();
    auto* spin_imu  = GetImu();

    if (spin_comm != nullptr && spin_imu != nullptr &&
        !spin_comm->isOffline() &&
        spin_imu->calib_done() &&
        spin_comm->gimbal_data().imu_ready != 0u) {

      const float psi_gimbal  = spin_comm->gimbal_data().gimbal_yaw_world;
      float psi_chassis = 0.0f;
      spin_imu->yaw(&psi_chassis);

      if (!spin_ref_valid) {
        /* 进模式时把云台的世界朝向"冻结"成常量基准。
         *
         * ⚠ 这里必须存 psi_gimbal 本身,而不是像跟随模式那样存
         *   (psi_gimbal − psi_chassis) 的差值。原因:
         *   自转过程中云台会被底盘带着偏、再被自己的 IMU 闭环拉回来,
         *   ψ_gimbal 是持续抖动的。如果每拍都去减这个实时值,
         *   参考系就一直飘,走一点就偏。
         *   云台"想指向哪"在进入模式那一刻就定了,之后不该再变。
         *
         * 所以:基准 = 常量(云台当时的世界朝向)
         *     实时 = 底盘当前朝向
         *     参考系偏角 = 基准 − 实时
         * 这样"推前"永远指向进模式时云台指的世界方向,与云台
         * 后续的抖动完全无关。
         *
         * ⚠ 2026-10-06 撤销"减 PI":实车验证两块板的朝向是对齐的,
         *   ref_yaw 本来就 ≈0,那个 π 是误判。留着它反而会引入一个
         *   假的 180° 旋转,和摇杆符号错误两次翻转相消,表现为"改了没反应"。
         *   "推前后退"的真因是 rc_lv() 的符号约定,见下面摇杆映射那段。 */
        spin_ref_psi = psi_gimbal;
        spin_ref_valid  = true;
        if (auto* p = GetPidFollow()) p->reset();
      }
      ref_yaw = WrapAngle(spin_ref_psi - psi_chassis);
      g_dbg_spin_ref_valid = true;
    } else {
      /* 基准作废,下次条件满足时重采 */
      spin_ref_valid = false;
      g_dbg_spin_ref_valid = false;
      ref_yaw = 0.0f;   /* 退化成车头方向 */
    }

    g_dbg_spin_ref_yaw = ref_yaw;

    /* 左杆上下 = 前进后退,左杆左右 = 横移(小陀螺下云台让出通道)
     *
     * ⚠ 两个负号是必须的,不是笔误。
     *   DR16 通道 3(右杆上下)和通道 4(左杆上下)物理朝向相反,
     *   rc_lv() 的正方向和 rc_rv() 是反的;左右同理。
     *   分离/跟随模式一直用右杆,所以约定是:
     *     vx_stick = +kDbgMaxSpeed * rc_rv()    (robot.cpp:353 / :438)
     *     vy_stick = -kDbgMaxSpeed * rc_rh()
     *   小陀螺改用左杆却不跟着翻符号,实测表现为"推左杆上下反而后退"。
     *   这里翻过来,和其他模式的杆→车体约定保持一致。 */
    float vx_stick = -kDbgMaxSpeed * ApplyDeadzone(rc->rc_lv());
    float vy_stick = -kDbgMaxSpeed * ApplyDeadzone(rc->rc_lh());

    /* 摇杆系 → 世界系:把"前"从参考系转到车体系。
     * 车体系下 vx 是车头方向、vy 是车体左侧。 */
    vx = vx_stick * cosf(ref_yaw) - vy_stick * sinf(ref_yaw);
    vy = vx_stick * sinf(ref_yaw) + vy_stick * cosf(ref_yaw);

    g_dbg_spin_stick = spin_stick;
    g_dbg_w_cmd      = w;

    /* 不叠摩擦前馈:自旋时轮速目标很小,固定 ~2A 的前馈占比过大,
     * 跟随模式同样的问题(见 chassis.cpp g_friction_ff_enable 注释)。 */
    chassis_->setSpeed(vx, vy, w, /*use_friction_ff=*/false);
  }

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

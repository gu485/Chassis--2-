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
/* 小陀螺转速系数 kw: w = kw × 右杆上下(rad/s)。Ozone 可实时改。 */
volatile float g_dbg_spin_kw = 1.5f;

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

/* 云台相对底盘的零位 theta_0:跟随模式进入时标定一次(云台对准车头的编码器读数),
 * 小陀螺模式共用同一个零位,保证两个模式的「云台坐标系」定义完全一致。
 * 没标定过(没进过跟随)时按 0 处理,即直接用 yaw 电机的 angle_offset 零点。 */
static bool  s_theta0_valid = false;
static float s_theta0       = 0.0f;

/* === 调试镜像(Ozone Symbols window 直接搜名字) === */
/* 跟随模式参数 */
/* yaw_pid_reset_pending:第一次采样 offset 时顺便复位跟随 PID。
 * 进入跟随的那一刻目标从"未知"跳到"当前对齐位置",不复位的话
 * PID 内部的 last_error 会有残值,第一拍输出一个假偏差。 */

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
    float yaw_vel = -kYawMaxVel * ApplyDeadzone(rc->rc_lh());  // 与云台板方向一致(已取反)
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

    /* === 底盘追云台(只用 yaw 电机编码器, 不用 IMU) ===
     * ① 刚进入跟随模式: 采 kThetaCalibN 次 yaw 编码器角度取平均 → theta_0
     *    (进入时云台所指方向 = 底盘正前方的基准)。采样期间底盘不动。
     * ② theta = theta_1 − theta_0 (theta_1 为当前编码器角度, 回绕到 [-π, π))
     * ③ w = PID(目标 0, 当前 theta), 再 setSpeed 下发。
     * 进入检测: run_follow 只在跟随模式下被调用, 超过 5ms 没被调用即认为
     * 中间切走过, 下次进来重新标定。 */
    constexpr int kThetaCalibN = 100;
    static bool  calib_done = false;
    static int   calib_cnt  = 0;
    static float calib_sum  = 0.0f;
    static float theta_0    = 0.0f;
    static uint32_t last_ms = 0;

    const uint32_t now_ms = HAL_GetTick();
    if (now_ms - last_ms > 5) {
      calib_done = false;
      calib_cnt  = 0;
      calib_sum  = 0.0f;
      if (auto* p = GetPidFollow()) p->reset();
    }
    last_ms = now_ms;

    const float theta_1 = yaw->angle();
    float w = 0.0f;
    float vx = 0.0f, vy = 0.0f;
    float theta_rel = 0.0f;

    if (!calib_done) {
      /* 零位标定: 底盘保持静止 */
      calib_sum += theta_1;
      if (++calib_cnt >= kThetaCalibN) {
        theta_0 = calib_sum / static_cast<float>(kThetaCalibN);
        s_theta0 = theta_0;
        s_theta0_valid = true;
        calib_done = true;
      }
    } else {
      theta_rel = WrapAngle(theta_1 - theta_0);

      /* 摇杆系 → 底盘系: 平移方向跟随云台 */
      float vx_stick =  kDbgMaxSpeed * ApplyDeadzone(rc->rc_rv());
      float vy_stick = -kDbgMaxSpeed * ApplyDeadzone(rc->rc_rh());
      /* 旋转角取 -theta_rel: 实测云台转到侧面时平移方向反了,
       * 说明编码器 θ 的正方向与底盘 w 的逆时针正方向相反。 */
      vx =  vx_stick * cosf(theta_rel) + vy_stick * sinf(theta_rel);
      vy = -vx_stick * sinf(theta_rel) + vy_stick * cosf(theta_rel);

      /* 底盘转速: 目标 0, 当前 theta。符号沿用之前实测的 w = -pid(0, θ)
       * (= +kp·θ, 底盘朝云台方向转); 若实车反着追, 去掉前面的负号。 */
      auto* pid = GetPidFollow();
      w = (pid != nullptr) ? -pid->calc(0.0f, theta_rel) : 0.0f;
      if (fabsf(theta_rel) < kFollowThetaDeadZone) w = 0.0f;
    }

    g_dbg_follow_err   = theta_rel;
    g_dbg_follow_valid = calib_done;
    g_dbg_w_cmd        = w;

    chassis_->setSpeed(vx, vy, w, /*use_friction_ff=*/false);
  }


  void Robot::run_spin()
  {
    /* === 小陀螺模式 ===
     * 底盘自转 + 平移;平移方向取云台坐标系的前后左右;云台 yaw 由右杆左右控制。
     *
     * 摇杆分配:
     *   左杆上下 rc_lv → 云台系前后    左杆左右 rc_lh → 云台系左右
     *   右杆上下 rc_rv → 小陀螺转速 w = kw × 摇杆
     *   右杆左右 rc_rh → 云台 yaw 目标角速度(经双板通讯发给云台板,
     *                    comm_task 在小陀螺模式下把 rc_rh 填进 rc_lh_q 字段,
     *                    云台板的速度环与跟随模式完全一致)
     *
     * 云台坐标系:跟随模式同款,用 yaw 电机编码器角 theta(云台相对底盘),
     * 把云台系下的"前/左"速度旋转到底盘系,底盘转到哪,"前"始终指向云台所指方向。 */
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

    /* ③ 右杆上下 → 转速 w = kw × 摇杆(死区在 ApplyDeadzone 里) */
    const float spin_stick = ApplyDeadzone(rc->rc_rv());
    const float w_target   = g_dbg_spin_kw * spin_stick;

    /* 斜率限制:w 每拍最多变 kSpinWRamp,避免阶跃让四个轮子同时顶满电流。
     * 超过 5ms 没被调用(切走又切回)从 0 重新起爬。 */
    static float w_out = 0.0f;
    static uint32_t last_ms = 0;
    const uint32_t now_ms = HAL_GetTick();
    if (now_ms - last_ms > 5) w_out = 0.0f;
    last_ms = now_ms;

    if (w_target > w_out + kSpinWRamp)      w_out += kSpinWRamp;
    else if (w_target < w_out - kSpinWRamp) w_out -= kSpinWRamp;
    else                                    w_out = w_target;
    const float w = w_out;

    /* ①② 云台坐标系 + 左杆平移。
     * 符号约定与之前一致:左杆通道与右杆物理朝向相反,故 lv、lh 取负。 */
    auto* yaw = GetMotorYaw();
    /* 与跟随模式同一个定义: theta = 编码器角 − theta_0, 回绕到 [-π, π)。 */
    const float theta = WrapAngle(yaw->angle() - (s_theta0_valid ? s_theta0 : 0.0f));
    /* yaw 电机掉使能/报故障时编码器角会冻结, 继续按旧 theta 旋转平移方向,
     * 底盘一转, 「前」就和云台对不上了。此时只自转、不平移。 */
    const bool frame_ok = yaw->is_enabled();
    const float vx_stick = -kDbgMaxSpeed * ApplyDeadzone(rc->rc_lv());
    const float vy_stick = -kDbgMaxSpeed * ApplyDeadzone(rc->rc_lh());

    /* 旋转方向与跟随模式一致(实测编码器 θ 与底盘 w 的正方向相反) */
    float vx =  vx_stick * cosf(theta) + vy_stick * sinf(theta);
    float vy = -vx_stick * sinf(theta) + vy_stick * cosf(theta);
    if (!frame_ok) { vx = 0.0f; vy = 0.0f; }

    g_dbg_spin_stick     = spin_stick;
    g_dbg_spin_ref_yaw   = theta;
    g_dbg_spin_ref_valid = frame_ok;
    g_dbg_w_cmd          = w;

    /* ⑤ 下发。自旋时轮速目标小,不叠摩擦前馈 */
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

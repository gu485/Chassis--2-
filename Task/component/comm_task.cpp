/**
 *******************************************************************************
 * @file      :comm_task.cpp
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
#include "comm_task.hpp"

#include "can.h"                         // hcan1, hcan2
#include "usart.h"                       // huart3(DT7 走 USART3)
#include "iwdg.h"                        // IWDG 刷新
#include "HW_can.hpp"                    // CAN_Send_Msg
#include "ins_motor.hpp"                 // GetMotorWheel, GetMotorYaw
#include "ins_comm.hpp"                  // GetRemoteControl(DT7 单例)
#include "ins_chassis_gimbal_comm.hpp"   // GetGimbalChassisComm() / robot::kChassisToGimbalId
#include "gimbal_chassis_comm.hpp"       // robot::GimbalChassisComm 完整定义
#include "motor_M3508.hpp"               // M3508::GetTxId
#include "imu.hpp"
#include "ins_imu.hpp"                    // GetImu()
/* 排查用:镜像电机所在 CAN 总线的硬件错误状态寄存器(ESR)。
 * bit0 EWGF=错误警告 bit1 EPVF=错误被动 bit2 BOFF=总线关闭(最严重)
 * [23:16]=发送错误计数 TEC  [31:24]=接收错误计数 REC
 * 只要没收到对方 ACK,TEC 会一直往上涨,涨到 255 就 Bus-Off。 */
uint32_t g_can_motor_esr = 0;
/* CAN1(yaw 所在总线)的错误状态寄存器,用来判断是"发不出去"还是"收不到 ACK"。
 * 高 16 位 = REC 接收错误计数,低 16 位 = TEC 发送错误计数。
 *   bit0 EWGF 错误警告   bit1 EPVF 错误被动   bit2 BOFF 总线关闭
 *   REC 接近 128 → 总线监视模式,能收不能发(tx_ok 停 / tx_err 涨就是这个现象) */
uint32_t g_can1_esr = 0;
uint32_t check_Can2_Send_Msg = 0;
uint32_t check_dt7_rx_msg = 0;

/* 外部引用:HW_can.cpp 里的 CAN1 接收分发计数(命中双板帧 0x211 的次数) */
extern volatile uint32_t g_can1_g2c_cnt;
/* chassis.cpp 里定义的急停锁存标志,跨板转发给云台用 */
extern volatile bool g_estop_latched;
/* robot.cpp 里的当前工作模式镜像。用于判断要不要关掉云台使能。
 * 小陀螺(SPIN)下左摇杆左右被征用成横移,云台必须只做世界锁定。 */
extern volatile int  g_dbg_robot_mode;


/* === 双板通讯调试 === */
/* g_dbg_c2g_frame_cnt : 底盘→云台 发帧计数,一直涨说明发送链路通
 * g_dbg_g2c_rx_cnt    : 云台→底盘 收帧计数,恒为 0 = 云台板没在发 / 双向都通不了
 * g_dbg_gimbal_yaw_rx : 收到的云台世界朝向 rad(NaN 或乱数 = 协议没对齐) */
volatile uint32_t g_dbg_c2g_frame_cnt = 0;
volatile uint32_t g_dbg_g2c_rx_cnt    = 0;
volatile float   g_dbg_gimbal_yaw_rx = 0.0f;

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

/* DT7(DBUS)调试用计数器 */
uint32_t g_uart3_rx_cnt = 0;       // 空闲中断触发次数
uint32_t g_uart3_last_size = 0;    // 最近一次接收的字节数
uint32_t g_dt7_decode_cnt = 0;     // DT7.decode() 被调次数

/* DBUS 接收缓冲区(18 字节,固定长度) */
static uint8_t g_dt7_rx_buf[18];

/* 调试用:DT7 解码后的通道值快照,每次 decode() 成功后刷新一次,
 * Ozone 里挂着看,推摇杆/拨开关时应该跟着实时变化 */
volatile float g_dbg_rc_lv = 0.0f;
volatile float g_dbg_rc_lh = 0.0f;
volatile float g_dbg_rc_rv = 0.0f;
volatile float g_dbg_rc_rh = 0.0f;
volatile float g_dbg_rc_wheel = 0.0f;
volatile uint8_t g_dbg_rc_l_switch = 0;
volatile uint8_t g_dbg_rc_r_switch = 0;

/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void) {
  /* 阶段 2 实测确认:底盘电机在 CAN2 上(从 Ozone 看到 CAN1 最大 rx_id=0x1FF,
   * 没有 M3508 反馈 ID 0x201~0x204;CAN2 才有)。
   * CAN2 = 4 个 M3508 轮电机(走 FIFO1)
   * CAN1 = 云台 yaw 轴 DM4310(ID=2,走 FIFO0),只有一块底盘C板,没有独立云台板,
   *        yaw 电机由底盘板直接控制
   * 顺序不能乱:必须先配滤波器,再启动外设,最后才打开中断通知。
   */
  CanFilter_Init(&hcan2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  CanFilter_Init(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* DT7(DBUS)UART3 接收启动:空闲 + DMA 方式
   * HAL_UARTEx_ReceiveToIdle_DMA:收到完整帧或总线空闲时触发回调
   * 接收 18 字节 DBUS 到 g_dt7_rx_buf */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dt7_rx_buf, sizeof(g_dt7_rx_buf));
  // HAL 默认开了 DMA 半传输中断(HTIE),DBUS 协议我们只关心完整帧,
  // 关掉 HTIE 避免回调被提前触发
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);

  /* TODO【新人任务,阶段 1 再补】:
   *   1. 启动 / 刷新独立看门狗 IWDG(inc/iwdg.h)。
   */
}

void CommTask(void) {
  g_can_motor_esr = hcan2.Instance->ESR;  // 每拍刷新一次电机总线的 ESR
  g_can1_esr      = hcan1.Instance->ESR;  // yaw 所在 CAN1 的 ESR
  GetImu()->update();

  /* IMU 调试镜像:紧跟在 update() 后面,读到的一定是同一拍的数据 */
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
  /* 遍历 4 个轮电机,ID 1~4 共用同一条报文(0x200),必须先把 4 个电机的数据
   * 填进同一个缓冲区,再整帧一次性发出——不能各发各的,否则后发的电机会把
   * 先前电机的字节位清零,导致电调收到的电流指令在真实值和 0 之间跳变。 */
  uint8_t data[8] = {0};
  for (int i = 0; i < 4; i++) {
    auto* motor = GetMotorWheel(i);   // 拿第 i 个电机单例(idx 0~3 对应 M3508 ID 1~4)
    if (motor == nullptr) continue;

    motor->encode(data);              // 把 target_current 装进 data 里自己的 2 字节
  }
  CAN_Send_Msg(&hcan2, data, hello_world::motor::M3508::GetTxId(1), 8);   // 电机在 CAN2,整帧只发一次
  check_Can2_Send_Msg++;

  /* =====================================================================
   * 云台 yaw 轴 DM4310(ID=2)的所有权已移交云台板。
   *
   * 架构变更:云台板用自己的 IMU 做世界锁定闭环(闭环必须和传感器同板,
   * 否则 1kHz 的误差信号走双板 CAN 要延迟半拍,PID 会不稳)。
   * 底盘只通过双板通讯把摇杆发过去,云台板自己算、自己驱电机。
   *
   * ⚠ 如果下面这段还开着,两块板会同时给同一个电机 ID 发帧,
   *   CAN 仲裁优先级随机,谁赢看运气 → 云台随机抽搐,而且极难查
   *   (两边代码单独看都是对的)。
   * ⚠ 急停也拦不住:云台板输出全零,但底盘这边还在发,总线上照样有帧。
   *
   * 要回退到"底盘控云台"的旧架构,把 kGimbalOwnsYawMotor 改回 true。 */
  static constexpr bool kGimbalOwnsYawMotor = false;  // yaw 电机由云台板驱动,底盘板不发

  if (kGimbalOwnsYawMotor) {
    auto* yaw = GetMotorYaw();
    uint8_t yaw_data[8] = {0};
    yaw->encode(yaw_data);
    CAN_Send_Msg(&hcan1, yaw_data, yaw->tx_id(), 8);
  }

  /* === 双板通讯:底盘 → 云台 (0x210) ===
   *
   * 云台板没有接 DR16,左摇杆数据只能从这里转发过去。
   * 不发的话云台板 isOffline() 一直为 true,急停锁死、云台完全不动。
   * encode() 每拍都要发,保持链路有心跳。 */
  {
    auto* comm = GetGimbalChassisComm();
    if (comm != nullptr) {
      auto& cd = comm->chassis_data();

      auto* rc = GetRemoteControl();
      /* 遥控器没接到这一侧时按 0 发,别用上次的旧值——
       * 云台板会拿旧摇杆值继续积分目标角度。 */
      /* 云台 yaw 的摇杆通道:分离/跟随用左杆左右 rc_lh。
       * 云台板协议不变,仍读 rc_lh_q,只是小陀螺时这个字段恒为 0。 */
      float lh = (rc != nullptr && g_dbg_robot_mode != robot::kModeSpin) ? rc->rc_lh() : 0.0f;
      float lv = (rc != nullptr) ? rc->rc_lv() : 0.0f;

      cd.rc_lh_q = robot::GimbalChassisComm::QuantizeStick(lh);
      cd.rc_lv_q = robot::GimbalChassisComm::QuantizeStick(lv);
      cd.heartbeat++;   /* uint8 自然回绕,只用于"链路活着"的粗略判断 */
      /* 把底盘急停状态跨板传给云台。云台板没有 DR16,拿不到拨杆,
       * 不传的话会出现"底盘已急停、云台还在转"的不一致。 */
      cd.estop = g_estop_latched ? 1u : 0u;
      /* 云台使能:恒为 1。
       * 小陀螺下不发摇杆,云台走位置环做世界锁定,不需要用这个位去禁用它。
       * (早期版本小陀螺会置 0 让云台"什么都不做",那时云台还没有独立的位置环,
       *  现在有了,语义变了:保留 1 = 云台主动锁住世界朝向。) */
      cd.gimbal_enabled = 1u;
      /* 把底盘当前工作模式发给云台板, 云台据此选择控制环路和 PID */
      cd.work_mode = static_cast<uint8_t>(g_dbg_robot_mode);

      uint8_t data[8] = {0};
      comm->encode(data);
      CAN_Send_Msg(&hcan1, data, robot::kChassisToGimbalId, 8);
      g_dbg_c2g_frame_cnt++;
    }
  }

  /* 刷看门狗前更新一下双板收帧镜像 */
  {
    auto* comm = GetGimbalChassisComm();
    if (comm != nullptr) {
      g_dbg_g2c_rx_cnt    = g_can1_g2c_cnt;
      g_dbg_gimbal_yaw_rx = comm->gimbal_data().gimbal_yaw_world;
    }
  }

  //刷看门狗 
  HAL_IWDG_Refresh(&hiwdg);

}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief   UART 接收事件回调(空闲中断触发)
 * @note    HAL_UARTEx_ReceiveToIdle_DMA 在以下两种情况触发此回调:
 *          1. 收到完整帧(DMA 传输完成)
 *          2. 总线空闲(空闲中断触发,Size 是已经收到的字节数)
 *          DBUS 协议用情况 2:1ms 一帧,空闲中断几乎每 1ms 触发一次。
 *          注意:HAL 头文件声明是 C 链接,必须 extern "C",否则中断里找不到符号。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size) {
  if (huart != &huart3) return;   // 只处理 DT7 用的 USART3

  g_uart3_rx_cnt++;
  check_dt7_rx_msg++;
  g_uart3_last_size = Size;

  // DBUS 帧固定 18 字节,验证一下,防止异常数据
  if (Size == sizeof(g_dt7_rx_buf)) {
    auto* rc = GetRemoteControl();
    rc->decode(g_dt7_rx_buf);
    g_dt7_decode_cnt++;

    g_dbg_rc_lv = rc->rc_lv();
    g_dbg_rc_lh = rc->rc_lh();
    g_dbg_rc_rv = rc->rc_rv();
    g_dbg_rc_rh = rc->rc_rh();
    g_dbg_rc_wheel = rc->rc_wheel();
    g_dbg_rc_l_switch = rc->rc_l_switch();
    g_dbg_rc_r_switch = rc->rc_r_switch();
  }

  // 重启下一轮接收(HAL 默认只触发一次)
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dt7_rx_buf, sizeof(g_dt7_rx_buf));
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}
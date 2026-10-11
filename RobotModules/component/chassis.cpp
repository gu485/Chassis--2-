/**
 *******************************************************************************
 * @file      :chassis.cpp
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
#include "chassis.hpp"

#include <cmath>  // fabsf

#include "ins_chassis_iksolver.hpp"   // GetChassisIkSolver()  — Phase 3 新增
#include "ins_comm.hpp"    // GetRemoteControl() —— 急停用
#include "ins_motor.hpp"   // GetMotorWheel(i)
#include "ins_pid.hpp"     // GetPidMotorWheel(i)
#include "pid.hpp"         // hello_world::pid::Pid 的完整定义(calc/reset)
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 急停锁存标志:左拨杆一旦拨到最上档就锁死,四个轮子强制零电流输出,
 * 必须重新上电才能解除(不允许拨回原位自动恢复,避免测试时手滑碰到拨杆误解除)。
 * volatile 方便 Ozone 里实时看有没有触发。 */
volatile bool g_estop_latched = false;
/* 阶段 2 调试用:4 个轮子各自独立的开环测试电流(安培),Ozone 里改这个数组
 * 实时生效,不用重新烧录。默认先全部给 0,按需单独调大某一路,避免像
 * 之前 4 路统一给 1.0f 导致某个轮子转太快的情况。阶段 3 接上 IK/PID 闭环后删掉。 */
volatile float g_test_wheel_curr[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/* 阻力扭矩补偿(低配版前馈):T_i = t_i * sgn(target_i),4 个轮子各自独立标定。
 * 麦轮侧移(vy)比前后直行(vx)阻力明显更大(滚轮蹭地摩擦更大),所以前后/左右
 * 分成两组独立标定的 t,按本次运动是以 vx 为主还是 vy 为主来选用对应的一组。
 * 标定方法:给底盘一个很小的对应方向目标速度(如 0.1m/s),平地/接近场地地胶
 * 材质环境下运动,观察每个电机反馈电流的稳定值,取该值作为 t(单位:安培,填绝对值,
 * 符号由代码根据 target 方向自动处理)。
 * Ozone 里改这两个数组实时生效,不用重新烧录。 */
/* ★ 临时统一值:先让四个轮子出力一致,排除"前进歪"是前馈差异导致的 ★
 * 实测前进时车往一侧歪,怀疑是四个前馈值不同导致的(0号2.1 最大,1号1.75 最小)。
 * 前馈是直接叠加到电流上的,差 0.35A 足以让车偏。
 * 先统一成 2.0,再实车走直线看还歪不歪:
 *   - 不歪了 → 就是前馈标定问题,之后按"哪个轮子出力不够就调大"逐个重新标定
 *   - 还歪  → PID 增益不对称,或者机械问题(轮子接触不均/安装角偏差)
 * 标定完之后记得把这行注释放开,恢复成每个轮子各自的值。 */
volatile float g_friction_ff_t_fwd[4] = {2.0f, 2.0f, 2.0f, 2.0f};  // 前后(vx)方向,临时统一值
volatile float g_friction_ff_t_lat[4] = {2.0f, 2.0f, 2.0f, 2.0f};  // 左右(vy)方向,临时统一值
/* ↓↓↓ 原始标定值(前进歪排查期间停用,标定完成后恢复)↓↓↓
volatile float g_friction_ff_t_fwd[4] = {2.1f, 1.75f, 1.9f, 2.0f};  // 前后(vx)方向标定值
volatile float g_friction_ff_t_lat[4] = {2.1f, 1.75f, 1.9f, 2.0f};  // 左右(vy)方向标定值 */
/* ↑↑↑ 原始标定值 ↑↑↑ */

/* 摩擦前馈总开关:Ozone 里改这个变量实时生效,不用重新烧录。
 *
 * ★ 为什么需要它 ★
 * 跟随模式实测出的问题:切换到 FOLLOW 后底盘疯转,但把前馈代码注释掉就正常。
 * 根因不是前馈本身有错,而是"固定电流"在极小目标转速下会淹没 PID 输出:
 *   跟随模式 w = kp × θ,kp 很小 → IK 解出的 target 只有 0.01 量级(rad/s)
 *   → PID 输出本来就接近 0
 *   → 再叠一个为克服静摩擦标定的 ~2A 固定电流,占比变成 100%
 *   → 电流瞬间打满,底盘疯转
 *
 * 分离模式不受影响:摇杆推满时 target 是 0.5 m/s 量级,前馈占比很小,只在起步
 * 那一瞬间帮轮子克服静摩擦,是有用的。
 *
 * ⚠ 代价:关掉之后跟随模式推摇杆前进会变肉(没有摩擦补偿,可能起步打滑)。
 *   这是"能跑"和"跑得顺"的取舍,先保证不失控。
 *   更好的做法是给前馈加幅值门槛(按 target 大小缩放),但那是下一步的事。 */
volatile bool g_friction_ff_enable = true;   // true=分离模式有前馈 / false=全模式关闭

/* 调试检查点:4 个轮子各自的"反馈转速 / PID 原始输出 / 换算后电流",
 * 每 1ms 刷新一次,Ozone 里挂数组能看到 4 个轮子的完整链路数据 */
volatile float g_wheel_target[4] = {0, 0, 0, 0};    // 节点⓪:IK 解算出的目标转速(PID setpoint)
volatile float g_wheel_vel[4] = {0, 0, 0, 0};       // 节点①:motor->vel() 反馈
volatile float g_wheel_pid_raw[4] = {0, 0, 0, 0};   // 节点②:pid->calc() 原始输出
volatile float g_wheel_curr[4] = {0, 0, 0, 0};      // 节点③:换算后的安培值(写进 setInput)

/* === 0 号轮子的标量镜像(给 Ozone 单变量 Watch 用)===
 * 数组在 Ozone 里要展开下标,新手容易找不到。这里额外导出 0 号轮的四个
 * 关键量,都是普通 float 变量,在 Symbols 窗口搜 "g_dbg_w0" 就能直接看到:
 *   g_dbg_w0_target : IK 解算出的目标转速(PID setpoint)
 *   g_dbg_w0_vel    : 电机实际反馈转速(vel(),跟电机转不转直接相关)
 *   g_dbg_w0_pidraw : PID 原始输出
 *   g_dbg_w0_curr   : 换算后的安培值,真正写进电机的量
 * 排查"轮子疯转"时重点看 g_dbg_w0_curr:正常几安培,爆了会到几百。 */
volatile float g_dbg_w0_target = 0.0f;
volatile float g_dbg_w0_vel    = 0.0f;
volatile float g_dbg_w0_pidraw = 0.0f;
volatile float g_dbg_w0_curr   = 0.0f;

/* === 1 号轮子的标量镜像,和 0 号完全对称 ===
 * 跟 0 号配对看最有用:跟随模式 w≠0 时底盘是差速旋转,
 * 0 号和 1 号的目标转速方向相反、大小不同,对比这两个数就能看出
 * "左右轮是不是在往相反方向转"(跟疯转直接相关)。
 * 搜 "g_dbg_w1" 找到。 */
volatile float g_dbg_w1_target = 0.0f;
volatile float g_dbg_w1_vel    = 0.0f;
volatile float g_dbg_w1_pidraw = 0.0f;
volatile float g_dbg_w1_curr   = 0.0f;

/* === 惯性制动前馈(解决"摇杆回中后车还在小飘")===
 * 现象:DT7 摇杆已经回中,底盘却停不下来,原地小幅飘移。
 *
 * 原因:轮子有惯性,摇杆回中瞬间 target 归零,但轮子还在按之前的速度转,
 * 速度环要等误差累积到一定程度才给出足够制动,这段时间就是"飘"。
 * PID 里的 kd 项理论上能提供阻尼,但 1kHz 采样下原始差分噪声太大
 * (相邻两拍转速能差几百),一开 kd 就是放大噪声,所以 kd 只能给 0。
 *
 * 这里用开环惯性前馈绕开:记住上一拍的轮子转速,本拍给一个反向电流。
 * 惯量小,系数 0.02 起步,Ozone 里改 kInertiaFfGain 实时生效。
 * 不引入反馈,所以不会跟 PID 抢,也不放大噪声。 */
volatile float kInertiaFfGain = 8.0f;   // 惯性制动系数,起步 0.02,大了会"急刹"
static float s_last_vel[4] = {0, 0, 0, 0};  // 上一拍各轮转速(static,不入栈)
volatile float g_dbg_brake[4] = {0, 0, 0, 0}; // 制动电流,Ozone 里看加了多少
/* External variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
/* Private function definitions ----------------------------------------------*/

extern uint32_t g_dt7_decode_cnt;  // comm_task.cpp:decode() 成功次数,判断是否已收到过真实遥控器数据

namespace robot {
void Chassis::setSpeed(float vx, float vy, float w, bool use_friction_ff) {
  // 急停检测:左拨杆拨到最上档就锁存,不会因为拨回原位而自动解除,
  // 必须重新上电才能恢复——防止测试时手滑碰到拨杆导致意外恢复输出。
  // 必须先判断 g_dt7_decode_cnt > 0(已经收到过至少一帧真实数据),
  // 否则上电瞬间还没收到第一帧 DBUS 时,rc_l_switch() 读到的是 DT7
  // 类里的默认初值(恰好就是 kSwitchStateUp),会导致开机瞬间被误锁死。
  if (g_dt7_decode_cnt > 0 &&
      GetRemoteControl()->rc_l_switch() ==
          hello_world::remote_control::kSwitchStateUp) {
    g_estop_latched = true;
  }

  auto* ik = GetChassisIkSolver();          // Phase 3 新增: 取 IK 单例
  ik->solve(vx, vy, w);                     // Phase 3 新增: 每 1ms 解算一次

  // 按本次指令是以前后(vx)还是左右(vy)为主,选用对应的一组摩擦前馈标定值。
  // 简单二选一,不做插值——对角线运动时会取较大分量所在的一组,低配版够用。
  volatile float* friction_ff = (fabsf(vy) > fabsf(vx)) ? g_friction_ff_t_lat
                                                         : g_friction_ff_t_fwd;

  for (int i = 0; i < 4; i++) {
    auto* motor = GetMotorWheel(i);
    auto* pid   = GetPidMotorWheel(i);

    if (motor == nullptr || pid == nullptr) continue;

    if (i == 0 || i == 1) {
      if (i == 0) {
        g_dbg_w0_target = ik->targetVel(0);
        g_dbg_w0_vel    = motor->vel();
      } else {
        g_dbg_w1_target = ik->targetVel(1);
        g_dbg_w1_vel    = motor->vel();
      }
    }

    g_wheel_vel[i] = motor->vel();  // 节点①:反馈转速,应该跟着手转/通电变化

    // 目标值改从 IK 类读取(Phase 3 接入):setSpeed 传进来的 vx/vy/w 经 ik->solve()
    // 解算后,每个轮子各自的目标转速用 ik->targetVel(i) 取
    float target = ik->targetVel(i);
    g_wheel_target[i] = target;     // 节点⓪:本轮目标转速

    // 反馈速度 → PID, 输出直接是电流(A)
    float curr = pid->calc(target, motor->vel());
    const float pid_out = curr;     // 保存纯 PID 输出, 后面 curr 还要叠加前馈
    g_wheel_pid_raw[i] = pid_out;   // 节点②:PID 输出(A)

    /* 惯性制动前馈:本拍给一个跟"上一拍转速"反向的电流,抵消轮子惯性。
     * 摇杆回中后 target 归零,这个项让车快速刹住,不再小飘。
     * 用上一拍的转速而不是本拍的,是因为本拍 target 已经是 0,
     * 差分项要靠 kd,而 kd 在 1kHz 下不能开(见 kInertiaFfGain 上方注释)。
     * 制动量远小于 out_limit,不会顶到限幅。 */
    float brake = -s_last_vel[i] * kInertiaFfGain;
    g_dbg_brake[i] = brake;
    curr += brake;
    s_last_vel[i] = motor->vel();   // 存本拍转速,供下一拍使用

    /* 阻力扭矩前馈(低配版):按目标转速方向叠加固定补偿电流,抵消静摩擦。
     * target==0 时不加(方向不明确,避免在零点附近来回抖动)。数组里存的是
     * 绝对值,符号在这里根据 target 的正负自动处理,不用在数组里填负数。
     *
     * 开关 = 调用方传入的 use_friction_ff AND 全局 g_friction_ff_enable。
     *   - 分离模式:use_friction_ff=true  → 分离有前馈(见 chassis.hpp 参数说明)
     *   - 跟随模式:use_friction_ff=false → 跟随无前馈,避免极小 target 下疯转
     *   - g_friction_ff_enable:Ozone 里的总闸,设 false 可临时全模式关闭
     */
    if (g_friction_ff_enable && use_friction_ff) {
      if (target > 0.0f) {
        curr += friction_ff[i];
      } else if (target < 0.0f) {
        curr -= friction_ff[i];
      }
    }

    if (g_estop_latched) {
      curr = 0.0f;
      s_last_vel[i] = 0.0f;   // 急停时清掉惯性历史,否则解除急停后第一拍会有一个
                              // 突发的制动电流(而且急停后电机已经被断电,vel 不可信)
    }
    g_wheel_curr[i] = curr;         // 节点③:实际写进电机的安培值

    if (i == 0 || i == 1) {
      if (i == 0) {
        g_dbg_w0_pidraw = pid_out;
        g_dbg_w0_curr   = curr;
      } else {
        g_dbg_w1_pidraw = pid_out;
        g_dbg_w1_curr   = curr;
      }
    }

    // 接入 PID 闭环:PID 算出来的电流才是真正写进电机的值(急停锁存时强制 0)
    // g_test_wheel_curr 保留作为开环调试手段,不用时保持全 0 即可
    motor->setInput(curr);
  }
}
}  // namespace robot
/* ============================================================================
 * 【新人任务】实现 chassis.hpp 中声明的底盘运动学。
 *
 * 你要做：
 *   1. 逆解：输入 (vx, vy, w)，输出每个轮的轮速 / 电流指令（麦轮逆运动学矩阵）。
 *   2. 对解算结果做限幅，避免超速。
 *
 * 约束 / 提示：
 *   - 解算结果会经电机组件的 encode() 下发，注意输入量的类型（电流 / 力矩）。
 * ==========================================================================*/
// ============================================================================
// mybot · 差速混控（cmd_vel → 左右轮 PWM）—— 纯函数，无 Arduino/ROS 依赖
//
// 为什么单独做成一个 header：
//   1) 这段数学是 P3 唯一"真机上一定会算错"的地方（符号、限幅、死区、NaN），
//      而它**不需要硬件就能测完** —— 用 g++ 在宿主机上跑 test_mixer.cpp，
//      同一份代码烧进 ESP32 也是它。仿真是测不出混控符号错的（diff_drive 控制器
//      替你算好了），这正是 P2 全程零硬件留下的盲区。
//   2) 默认常数直接抄仿真的 URDF（wheel_radius 0.035 / wheel_separation 0.28），
//      这样"仿真里 0.30 m/s + 1.5 rad/s 意味着轮子转多快"是可核对的，
//      也顺带变成对电机选型是否够用的检查（见 test_mixer 的 [5]）。
//
// 符号约定（与 D9/D10 一致，REP-103）：
//   x 向前为正，ω>0 = 逆时针 = **左转**；左转时右轮更快、左轮更慢。
//   轮缘线速度： v_L = v − ω·L/2 ， v_R = v + ω·L/2
// ============================================================================
#pragma once

#include <cmath>
#include <cstdint>

namespace mybot {

struct DiffLimits {
  double wheel_radius = 0.035;      // m，抄 mybot_core.urdf.xacro
  double wheel_separation = 0.28;   // m，左右轮心距，同上
  // 单个轮缘允许的最大线速度 = **电机上限**，不是控制上限。
  // 定这条的硬约束：必须 ≥ 定稿工况的轮缘需求 v_max + w_max·L/2 = 0.30 + 2.2×0.14 = 0.608 m/s，
  // 否则 mix() 会把 PD 的指令悄悄等比削掉（仿真里 w_max=2.2 是实测最优，削了就等于换了参数）。
  // MG5158 1:30 @12V 额定输出约 250 rpm → 轮缘 0.92 m/s，取 0.90 留余量；
  // 单测 [11] 会把这条约束钉死，改小立刻红。
  double wheel_max_mps = 0.90;
  double deadband_mps = 0.012;      // 低于此轮缘速度直接给 0：防静摩擦区抖振与 PWM 啸叫
  int pwm_top = 255;                // 8 bit 占空比满量程（ledcWrite 的 2^8-1）
  int min_on_duty = 46;             // 死区之后直接跳到 ~18% 占空比：TB6612+电机在低占空比下不转只叫
};

enum class Dir : uint8_t { kCoast = 0, kForward = 1, kBackward = 2, kBrake = 3 };

struct WheelOut {
  double left_mps = 0.0;            // 混控后的轮缘速度（物理量，便于日志与对账）
  double right_mps = 0.0;
  int left_duty = 0;
  int right_duty = 0;
  Dir left_dir = Dir::kCoast;
  Dir right_dir = Dir::kCoast;
  bool saturated = false;           // 任一占空比顶到 pwm_top
  bool invalid = false;             // 输入非有限（NaN/Inf）→ 已强制安全停车
};

// 唯一入口：一条 Twist 的 (v, ω) → 两个轮子的 PWM 与方向位。
// 纯函数：不读全局、不做 IO、不依赖时间，所以能在宿主机上逐条断言。
inline WheelOut mix(double v_mps, double w_rps, const DiffLimits& g) {
  WheelOut o;
  // [安全优先] 非有限输入一律"刹车"而不是"沿用上一条"。
  // D10 的丢线看门狗、D11 的 /scan 断流失效安全都是同一条规矩：
  // "没有有效数据"和"数据是 0"不是一回事，但都必须落到"不动"这个安全解释上。
  if (!std::isfinite(v_mps) || !std::isfinite(w_rps)) {
    o.invalid = true;
    o.left_dir = o.right_dir = Dir::kBrake;   // AIN1=AIN2=1：TB6612 主动短路制动
    return o;
  }

  const double half = g.wheel_separation * 0.5;
  o.left_mps = v_mps - w_rps * half;
  o.right_mps = v_mps + w_rps * half;

  // 单轮限幅：轮缘速度超过 wheel_max_mps 时按比例**一起缩**，而不是各切各的。
  // 各切各的会改变左右差 → 等于偷偷改了航向指令，这是差速小车最常见的坑。
  double scale = 1.0;
  const double amax = std::max(std::fabs(o.left_mps), std::fabs(o.right_mps));
  if (g.wheel_max_mps > 0.0 && amax > g.wheel_max_mps) {
    scale = g.wheel_max_mps / amax;
    o.saturated = true;
  }
  o.left_mps *= scale;
  o.right_mps *= scale;

  auto one = [&](double mps, int* duty, Dir* dir) {
    if (std::fabs(mps) < g.deadband_mps) {           // 死区：滑行（两个输入都拉低）
      *duty = 0;
      *dir = Dir::kCoast;
      return;
    }
    *dir = (mps > 0.0) ? Dir::kForward : Dir::kBackward;
    const double frac = std::fabs(mps) / g.wheel_max_mps;      // 0..1
    double d = g.min_on_duty + frac * (g.pwm_top - g.min_on_duty);
    if (d > g.pwm_top) { d = g.pwm_top; o.saturated = true; }
    *duty = static_cast<int>(std::lround(d));
  };
  one(o.left_mps, &o.left_duty, &o.left_dir);
  one(o.right_mps, &o.right_duty, &o.right_dir);
  return o;
}

// ---- 编码器侧的单位换算（D16 里程计要用，放这里一起测）------------------
// ticks/s → 轮缘线速度。cpr 是"电机轴每转计数"，gear 是减速比（输出轴/电机轴 = gear:1）。
inline double ticks_to_mps(double ticks_per_s, double cpr, double gear, const DiffLimits& g) {
  const double wheel_rev_per_s = ticks_per_s / (cpr * gear);
  return wheel_rev_per_s * 2.0 * M_PI * g.wheel_radius;
}

inline double mps_to_ticks(double mps, double cpr, double gear, const DiffLimits& g) {
  return mps / (2.0 * M_PI * g.wheel_radius) * cpr * gear;
}

// 里程计反解：由两个轮缘速度还原 (v, ω) —— 与 mix() 互为逆运算，测试里做往返核对。
inline void wheel_fk(double left_mps, double right_mps, const DiffLimits& g,
                     double* v_out, double* w_out) {
  *v_out = 0.5 * (left_mps + right_mps);
  *w_out = (right_mps - left_mps) / g.wheel_separation;
}

}  // namespace mybot

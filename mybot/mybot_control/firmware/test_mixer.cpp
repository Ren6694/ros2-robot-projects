// ============================================================================
// mixer.h 的宿主机单测：g++ -std=c++17 -I. test_mixer.cpp && ./a.out
//
// 为什么值得为一段"看起来很简单"的乘法写 11 组断言：
//   差速混控的错全在符号与饱和语义上，而**仿真测不出来** —— P2 全程用的是 Gazebo 的
//   diff_drive 控制器，它替我们把 v/ω 拆成两轮转速了。真机上这一层是我们自己写，
//   写反一个符号就是"命令左转它右转"，且第一次上电往往是在地上跑着的时候才发现。
//   所以这一层必须在有硬件之前就逐条钉死。
//
// 每条断言都用**解析真值**（手算出来的数），不用"跑一遍看看像不像"。
// ============================================================================
#include <cmath>
#include <cstdio>

#include "mixer.h"

static int g_fail = 0;
static int g_pass = 0;

static void ok(const char* what, bool cond) {
  if (cond) { ++g_pass; } else { ++g_fail; printf("  FAIL  %s\n", what); }
}

static void near(const char* what, double got, double want, double tol = 1e-9) {
  bool good = std::fabs(got - want) <= tol;
  if (good) { ++g_pass; } else { ++g_fail; printf("  FAIL  %s : got %.6f want %.6f\n", what, got, want); }
}

using mybot::DiffLimits;
using mybot::Dir;
using mybot::mix;
using mybot::mps_to_ticks;
using mybot::ticks_to_mps;
using mybot::wheel_fk;

int main() {
  DiffLimits g;                       // 默认值 = 仿真 URDF 的几何
  const double half = g.wheel_separation * 0.5;   // 0.14

  printf("[1] straight: v=0.30 w=0 -> both wheels 0.30, duty 116\n");
  {
    auto o = mix(0.30, 0.0, g);
    near("left_mps", o.left_mps, 0.30);
    near("right_mps", o.right_mps, 0.30);
    ok("duty 116 (hand: 46 + 0.3/0.9*209 = 115.67)", o.left_duty == 116 && o.right_duty == 116);
    ok("both forward", o.left_dir == Dir::kForward && o.right_dir == Dir::kForward);
    ok("not saturated", !o.saturated && !o.invalid);
  }

  printf("[2] pivot: v=0 w=+1.0 -> left -0.14 / right +0.14 (REP-103: w>0 = turn LEFT)\n");
  {
    auto o = mix(0.0, 1.0, g);
    near("left = -w*L/2", o.left_mps, -1.0 * half);
    near("right = +w*L/2", o.right_mps, 1.0 * half);
    ok("left backs up, right goes forward", o.left_dir == Dir::kBackward && o.right_dir == Dir::kForward);
    near("duty (hand: 46 + 0.14/0.9*209 = 78.51)", static_cast<double>(o.right_duty), 79.0, 0.5);
  }

  printf("[3] moving left turn: v=0.30 w=0.6 -> 0.216 / 0.384\n");
  {
    auto o = mix(0.30, 0.6, g);
    near("left", o.left_mps, 0.30 - 0.6 * half);
    near("right", o.right_mps, 0.30 + 0.6 * half);
    ok("right wheel faster (turns left)", o.right_duty > o.left_duty);
  }

  printf("[4] NaN / Inf -> brake, zero duty, invalid flag (never reuse last command)\n");
  {
    auto a = mix(std::nan(""), 0.5, g);
    auto b = mix(0.3, INFINITY, g);
    ok("NaN flagged", a.invalid && b.invalid);
    ok("both brake", a.left_dir == Dir::kBrake && a.right_dir == Dir::kBrake &&
                    b.left_dir == Dir::kBrake && b.right_dir == Dir::kBrake);
    ok("duty zero", a.left_duty == 0 && a.right_duty == 0);
  }

  printf("[5] saturation scales BOTH wheels -> turn radius v/w preserved (time dilation)\n");
  {
    const double v = 0.90, w = 1.0;
    auto o = mix(v, w, g);
    ok("saturated reported", o.saturated);
    near("right pinned at wheel_max", o.right_mps, g.wheel_max_mps);
    near("left scaled too (hand: 0.76*0.9/1.04)", o.left_mps, 0.76 * 0.90 / 1.04, 1e-12);
    double vv, ww;
    wheel_fk(o.left_mps, o.right_mps, g, &vv, &ww);
    near("radius preserved", vv / ww, v / w, 1e-9);
    ok("duty never exceeds top", o.left_duty <= g.pwm_top && o.right_duty <= g.pwm_top);
  }

  printf("[6] symmetry: mix(-v,-w) -> same duty per side, both directions reversed\n");
  {
    auto p = mix(0.30, 0.6, g);
    auto q = mix(-0.30, -0.6, g);
    // 反向是"两个轮子各自掉头"，不是左右互换 —— 左右互换等于把航向也翻了。
    ok("duty per side unchanged", p.left_duty == q.left_duty && p.right_duty == q.right_duty);
    ok("both wheels now back up", q.left_dir == Dir::kBackward && q.right_dir == Dir::kBackward);
    ok("left still slower (still turning left relative to travel)", q.left_mps > q.right_mps);
  }

  printf("[7] deadband: tiny command coasts (no PWM whine, no stick-slip jitter)\n");
  {
    auto o = mix(0.005, 0.0, g);
    ok("duty 0", o.left_duty == 0 && o.right_duty == 0);
    ok("coast, not brake", o.left_dir == Dir::kCoast && o.right_dir == Dir::kCoast);
  }

  printf("[8] min_on_duty step: just above deadband jumps to the minimum driving duty\n");
  {
    auto o = mix(g.deadband_mps * 1.01, 0.0, g);
    ok("duty >= min_on_duty", o.left_duty >= g.min_on_duty);
  }

  printf("[9] fk/mix round trip (unsaturated)\n");
  {
    auto o = mix(0.24, 0.8, g);
    double vv, ww;
    wheel_fk(o.left_mps, o.right_mps, g, &vv, &ww);
    near("v recovered", vv, 0.24);
    near("w recovered", ww, 0.8);
  }

  printf("[10] encoder units round trip (cpr=22, gear=30)\n");
  {
    double t = mps_to_ticks(0.30, 22, 30, g);
    near("mps recovered", ticks_to_mps(t, 22, 30, g), 0.30, 1e-12);
    // 0.30 m/s / (2*pi*0.035) = 1.3645 rev/s -> *22*30 = 900.6 ticks/s
    near("ticks/s magnitude", t, 0.30 / (2.0 * M_PI * g.wheel_radius) * 660.0, 1e-9);
  }

  printf("[11] D12 shipping gains must NOT saturate the wheels (self-consistency)\n");
  {
    // 定稿工况：v_max=0.30、w_max=2.2（D12 实测最优）→ 右轮 0.30+2.2*0.14 = 0.608 m/s
    const double v_ship = 0.30, w_ship = 2.2;
    const double need = v_ship + w_ship * half;
    printf("     shipping demand = %.3f m/s at the outer wheel; wheel_max = %.2f\n",
           need, g.wheel_max_mps);
    // 这条是"为什么 wheel_max_mps 不能随手调小"的根：低于需求就会把 PD 指令等比削掉，
    // 等于真机跑的增益和仿真里那组不是同一组 —— 而 mix() 不会报错，只会悄悄慢一点。
    ok("wheel_max covers the tuned demand with margin", g.wheel_max_mps >= need * 1.2);
    auto o = mix(v_ship, w_ship, g);
    ok("no clipping at the shipping point", !o.saturated);
    // 轮子转速：给电机选型对账用（MG5158 1:30 @12V 额定输出约 100~250 rpm）
    double rev_s = g.wheel_max_mps / (2.0 * M_PI * g.wheel_radius);
    printf("     wheel_max -> %.2f rev/s = %.0f rpm at the wheel\n", rev_s, rev_s * 60.0);
    ok("rpm inside a sane gearmotor range", rev_s * 60.0 > 60.0 && rev_s * 60.0 < 400.0);
  }

  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}

// ============================================================================
// D13 上电自检固件：点灯 + 两路 PWM，**并且 PWM 是走 mixer.h 算出来的**
//
// 为什么自检也要走混控函数，而不是直接 ledcWrite(128)：
//   验收要看的是"cmd_vel 语义 → 轮子行为"这条链，不是"GPIO 会不会输出"。
//   走同一段代码，宿主机单测（test_mixer.cpp，33 条断言）里算出来的占空比
//   就可以直接在万用表/示波器上对：串口每 2 秒换一个工况并打印预期占空比，
//   量到 46%±3 就说明 PWM 链路是好的；量不到就是驱动板/供电/共地问题。
//
// 引脚按 DevKit-C V4 + TB6612FNG 的常见接法，到货后如果不同只改这一组 #define。
// ============================================================================
#include <Arduino.h>

#include "../mixer.h"

// ---- 引脚分配（定稿进标定文档，别散落在代码里）--------------------------
#define PIN_LED        2     // 板载 LED
#define PIN_PWM_LEFT   25    // TB6612 PWMA
#define PIN_PWM_RIGHT  26    // TB6612 PWMB
#define PIN_AIN1       27    // 左轮方向
#define PIN_AIN2       14
#define PIN_BIN1        5    // 右轮方向
#define PIN_BIN2        4

#define PWM_FREQ_HZ  20000   // 20 kHz：高出人耳，且避开 TB6612 驱动感性负载的啸叫区
#define PWM_RES_BITS 8       // 与 mixer.h 的 pwm_top=255 对齐

using mybot::Dir;

static void set_dir(uint8_t pin_a, uint8_t pin_b, Dir d) {
  // TB6612 真值表：1/0 正转，0/1 反转，1/1 短路制动，0/0 滑行。
  // 这里唯一不能错的是"两个都写 0 当制动"——那是滑行，坡道上会溜车。
  digitalWrite(pin_a, (d == Dir::kForward || d == Dir::kBrake) ? HIGH : LOW);
  digitalWrite(pin_b, (d == Dir::kBackward || d == Dir::kBrake) ? HIGH : LOW);
}

// 自检工况表：{说明, v, w}
struct Step { const char* what; double v; double w; };
static const Step STEPS[] = {
  {"straight 0.30",            0.30,  0.0},
  {"pivot left  w=+1.0",       0.00,  1.0},
  {"left turn v0.30 w0.6",     0.30,  0.6},
  {"demand clipped (0.9,1.0)", 0.90,  1.0},
  {"NaN input -> BRAKE",       NAN,   0.5},
  {"coast (below deadband)",   0.005, 0.0},
};
static const int N_STEPS = int(sizeof(STEPS) / sizeof(STEPS[0]));

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  ledcSetup(0, PWM_FREQ_HZ, PWM_RES_BITS);  ledcAttachPin(PIN_PWM_LEFT, 0);
  ledcSetup(1, PWM_FREQ_HZ, PWM_RES_BITS);  ledcAttachPin(PIN_PWM_RIGHT, 1);
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("mybot D13 self-test  (PWM via mixer.h, same code as the host unit tests)");
  Serial.printf("geom: r=%.3f m  L=%.3f m  wheel_max=%.2f m/s  pwm_top=%d min_on=%d\n",
                mybot::DiffLimits{}.wheel_radius, mybot::DiffLimits{}.wheel_separation,
                mybot::DiffLimits{}.wheel_max_mps, mybot::DiffLimits{}.pwm_top,
                mybot::DiffLimits{}.min_on_duty);
}

void loop() {
  static int i = 0;
  static uint32_t t0 = 0;
  mybot::DiffLimits g;                       // 默认值 = 仿真 URDF 的几何
  const Step& s = STEPS[i];
  mybot::WheelOut o = mybot::mix(s.v, s.w, g);

  ledcWrite(0, o.left_duty);
  ledcWrite(1, o.right_duty);
  set_dir(PIN_AIN1, PIN_AIN2, o.left_dir);
  set_dir(PIN_BIN1, PIN_BIN2, o.right_dir);
  digitalWrite(PIN_LED, o.left_duty > g.pwm_top / 2);

  if (millis() - t0 >= 2000) {
    t0 = millis();
    Serial.printf("[%d] %-26s v=%+.2f w=%+.2f -> L duty=%3d (%.0f%%) %s | R duty=%3d (%.0f%%) %s%s%s\n",
                  i, s.what, s.v, s.w,
                  o.left_duty, 100.0 * o.left_duty / g.pwm_top,
                  o.left_dir == Dir::kForward ? "FWD" : o.left_dir == Dir::kBackward ? "REV"
                  : o.left_dir == Dir::kBrake ? "BRAKE" : "COAST",
                  o.right_duty, 100.0 * o.right_duty / g.pwm_top,
                  o.right_dir == Dir::kForward ? "FWD" : o.right_dir == Dir::kBackward ? "REV"
                  : o.right_dir == Dir::kBrake ? "BRAKE" : "COAST",
                  o.saturated ? " SAT" : "", o.invalid ? " INVALID" : "");
    i = (i + 1) % N_STEPS;
  }
}

#include "pwm.h"

// 初始化PWM设置
void initPWM() {
  // 初始化PWM引脚
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  pinMode(PWM3_PIN, OUTPUT);
  pinMode(PWM4_PIN, OUTPUT);

  // 1. 配置 LEDC 通道
  ledcSetup(0, 50, 13);
  ledcSetup(1, 50, 13);
  ledcSetup(2, 50, 13);
  ledcSetup(3, 50, 13);
  // 2. 将引脚绑定到通道
  ledcAttachPin(4, 0);
  ledcAttachPin(5, 1);
  ledcAttachPin(6, 2);
  ledcAttachPin(7, 3);
}

// 计算角度对应的 PWM 数值
int angleToDuty(int angle) {
  // 将 0-180 度映射到 205-1024 (对应 0.5ms-2.5ms)
  return map(angle, 0, 180, 205, 1024);
}

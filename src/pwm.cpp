#include "pwm.h"

// 初始化PWM设置
void initPWM() {
  // 初始化PWM引脚
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  pinMode(PWM3_PIN, OUTPUT);
  pinMode(PWM4_PIN, OUTPUT);
  pinMode(PWM5_PIN, OUTPUT);

  // 1. 配置 LEDC 通道
  ledcSetup(1, 50, 13);
  ledcSetup(2, 50, 13);
  ledcSetup(3, 50, 13);
  ledcSetup(4, 50, 13);
  ledcSetup(5, 50, 13);
  // 2. 将引脚绑定到通道
  ledcAttachPin(PWM1_PIN, 1);
  ledcAttachPin(PWM2_PIN, 2);
  ledcAttachPin(PWM3_PIN, 3);
  ledcAttachPin(PWM4_PIN, 4);
  ledcAttachPin(PWM5_PIN, 5);
}
 
// 计算角度对应的 PWM 数值
int angleToDuty(int angle) {
  //0到360，对应0.5ms-2.5ms 20ms

  return map(angle, 0, 360, 204, 1024);
}

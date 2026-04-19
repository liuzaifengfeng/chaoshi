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

  ledcWrite(1,angleToDuty(250) );//图像大臂完全放下，20完全抬起
  ledcWrite(2,angleToDuty(285) );//夹臂完全收回，45完全打开
  ledcWrite(3,angleToDuty(180) );//夹爪闭合，0大打开
  ledcWrite(4,angleToDuty(0) );//料台收回，270伸出
  ledcWrite(5,angleToDuty(240) );//挡板完全关闭，90完全打开

}
 
// 计算角度对应的 PWM 数值
int angleToDuty(int angle) {
  //0到360，对应0.5ms-2.5ms 20ms

  return map(angle, 0, 360, 204, 1024);
}

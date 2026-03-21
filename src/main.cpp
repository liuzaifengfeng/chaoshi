/**********************************************************
*** 超市机器人控制程序
*** 编写作者：LZF
*** 技术支持：GEMINI
**********************************************************/

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

#include "Emm_V5.h"
#include "lidar.h"

// 引脚定义
#define PWM1_PIN 4
#define PWM2_PIN 5
#define PWM3_PIN 6
#define PWM4_PIN 7



//全局h变量
RobotPose currentPose = {0, 0, 0, 0};//当前机器人位置

//位置坐标-电机脉冲转换系数（mm-脉冲）
const float X_PULSE = 18.0f;
const float Y_PULSE = 18.0f;
const float THETA_PULSE = 18.0f;

// --------------------------------------------------------
// 初始化设置
// --------------------------------------------------------
void setup() {
  // 初始化PWM引脚
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  pinMode(PWM3_PIN, OUTPUT);
  pinMode(PWM4_PIN, OUTPUT);

  // 初始化串口
  Serial.begin(115200);

  // 初始化雷达
  initLidar();
  // 初始化电机
  Emm_V5_Init();

  Serial.println("Supermarket robot initialized");

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

void loop() {

  //ledcWrite(0, angleToDuty(90));
                    
}
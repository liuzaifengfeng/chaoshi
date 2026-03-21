/**********************************************************
*** 超市机器人控制程序
*** 编写作者：LZF
*** 技术支持：GEMINI
**********************************************************/

#include <Arduino.h>
#include "Emm_V5.h"
#include "lidar.h"

// 引脚定义
#define SERIAL1_TXD_PIN 18
#define SERIAL1_RXD_PIN 17
#define SERIAL2_TXD_PIN 16
#define SERIAL2_RXD_PIN 15
#define PWM1_PIN 4
#define PWM2_PIN 5
#define PWM3_PIN 6
#define PWM4_PIN 7

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
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_TXD_PIN, SERIAL1_RXD_PIN);

  // 初始化雷达
  initLidar();

  Serial.println("Supermarket robot initialized");

  // 上电延时等待初始化完毕
  delay(1000);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
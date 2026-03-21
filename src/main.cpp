/**********************************************************
*** 超市机器人控制程序
*** 编写作者：LZF
*** 技术支持：GEMINI
**********************************************************/

#include <Arduino.h>
#include "Emm_V5.h"
#include "lidar.h"

// 引脚定义
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
  // 初始化电机
  Emm_V5_Init();

  Serial.println("Supermarket robot initialized");
}

void loop() {
  
  vTaskDelay(pdMS_TO_TICKS(1000));
  Emm_V5_Pos_Control(5, 0, 1000, 254, 3200,0,0);
  vTaskDelay(pdMS_TO_TICKS(1000));
  Emm_V5_Pos_Control(6, 0, 1000, 254, 3200,0,0);
  vTaskDelay(pdMS_TO_TICKS(1000));
  Emm_V5_Pos_Control(7, 0, 1000, 254, 3200,0,0);
  vTaskDelay(pdMS_TO_TICKS(2000));

  Emm_V5_Pos_Control(5, 0, 1000, 254, 3200,0,0);
  vTaskDelay(pdMS_TO_TICKS(100));
  Emm_V5_Pos_Control(6, 0, 1000, 254, 3200,0,0);
  vTaskDelay(pdMS_TO_TICKS(100));
  Emm_V5_Pos_Control(7, 0, 1000, 254, 3200,0,0);
  vTaskDelay(pdMS_TO_TICKS(100));

  vTaskDelay(pdMS_TO_TICKS(2000));
  Emm_V5_Pos_Control(5, 0, 1000, 254, 3200,0,1);
  vTaskDelay(pdMS_TO_TICKS(10));
  Emm_V5_Pos_Control(6, 0, 1000, 254, 3200,0,1);
  vTaskDelay(pdMS_TO_TICKS(10));
  Emm_V5_Pos_Control(7, 0, 1000, 254, 3200,0,1);
  vTaskDelay(pdMS_TO_TICKS(10));
  Emm_V5_Synchronous_motion(0);
  vTaskDelay(pdMS_TO_TICKS(1000));
}
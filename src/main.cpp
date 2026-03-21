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
#include "pwm.h"

//全局变量
RobotPose currentPose = {0, 0, 0, 0};//当前机器人位置
bool isdebug = true;//是否调试模式

//位置坐标-电机脉冲转换系数（mm-脉冲）
 float X_PULSE = 18.0f;
 float Y_PULSE = 18.0f;
 float THETA_PULSE = 18.0f;

// ================= 函数声明 =================

void Task_MainStateMachine(void *pvParameters);//主状态机任务函数（正常运行模式）
void Task_Main_Serial0_CMD(void *pvParameters);//主串口0命令任务函数
void Task_DebugStateMachine(void *pvParameters);//调试模式状态机任务函数（调试模式）
void Task_Debug_Serial0_CMD(void *pvParameters);//调试模式串口0命令任务函数


// --------------------------------------------------------
// 初始化设置
// --------------------------------------------------------
void setup() {
  // 初始化串口
  Serial.begin(115200);

  // 初始化PWM
  initPWM();
  // 初始化雷达
  initLidar();
  // 初始化电机
  Emm_V5_Init();

  //开启程序
  if (isdebug) {
    Serial.println("Debug mode ");//调试模式 开启对应任务
    //xTaskCreate(Task_DebugStateMachine, "Task_DebugStateMachine", 2048, NULL, 5, NULL);
    //xTaskCreate(Task_Debug_Serial0_CMD, "Task_Debug_Serial0_CMD", 2048, NULL, 5, NULL);
  } else {
    Serial.println("Release mode ");//正常运行 开启对应任务
    //xTaskCreate(Task_MainStateMachine, "Task_MainStateMachine", 2048, NULL, 5, NULL);
    //xTaskCreate(Task_Main_Serial0_CMD, "Task_Main_Serial0_CMD", 2048, NULL, 5, NULL);
  }

  Serial.println("Supermarket robot initialized");

}



void loop() {

  //ledcWrite(0, angleToDuty(90));
                    
}
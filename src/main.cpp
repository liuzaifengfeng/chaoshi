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
RobotPose currentPose = {0, 0, 0};//当前机器人位置
bool isdebug = true;//是否调试模式

//位置坐标-电机脉冲转换系数（mm-脉冲）
 float X_PULSE = 18.0f;
 float Y_PULSE = 18.0f;
 float THETA_PULSE = 18.0f;

// Debug命令队列
QueueHandle_t xDebugQueue;

// Debug命令结构体
typedef struct {
  char cmd[10];      // 命令类型：GOTOpose 或 GETpose
  float param1;           // 参数1
  float param2;           // 参数2
  float param3;           // 参数3
} DebugCommand_t;

// ================= 函数声明 =================

void Task_MainStateMachine(void *pvParameters);//主状态机任务函数（正常运行模式）
void Task_Main_Serial0_CMD(void *pvParameters);//主串口0命令任务函数
void Task_Debug_Mode(void *pvParameters);//调试模式任务函数（调试模式）
void Task_Debug_Serial0_CMD(void *pvParameters);//调试模式串口0命令任务函数

// 主状态机任务函数（正常运行模式）
void Task_MainStateMachine(void *pvParameters){

}

// 主串口0命令任务函数
void Task_Main_Serial0_CMD(void *pvParameters){

}

// 调试模式任务函数（调试模式）
void Task_Debug_Mode(void *pvParameters){
  DebugCommand_t cmd;
  
  while(1){
    // 从队列中接收命令
    if(xQueueReceive(xDebugQueue, &cmd, portMAX_DELAY) == pdPASS){

      if(strcmp(cmd.cmd, "GOTOpose") == 0){
        // 执行GOTOpose命令
        Serial.print("Executing GOTOpose: x=");
        Serial.print(cmd.param1);
        Serial.print(", y=");
        Serial.print(cmd.param2);
        Serial.print(", theta=");
        Serial.println(cmd.param3);
        // 使用GotoPose函数移动机器人到指定位置
        GotoPose(cmd.param1, cmd.param2, cmd.param3, true);
        
      } else if(strcmp(cmd.cmd, "GETpose") == 0){
        // 执行GETpose命令
        Serial.print("Current pose: x=");
        Serial.print(currentPose.x);
        Serial.print(", y=");
        Serial.print(currentPose.y);
        Serial.print(", theta=");
        Serial.println(currentPose.theta);

      } else if(strcmp(cmd.cmd, "GETdist") == 0){
        // 执行GETdist命令,雷达原始距离
        Serial.print("LiDAR distances: CH0=");
        Serial.print(avg_distances[0]);
        Serial.print(" mm, CH1=");
        Serial.print(avg_distances[1]);
        Serial.print(" mm, CH2=");
        Serial.print(avg_distances[2]);
        Serial.print(" mm, CH3=");
        Serial.println(avg_distances[3]);

      } else if(strcmp(cmd.cmd, "EMMpos") == 0){
        // 执行EMMpos命令,设置电机位置
        Serial.print("Executing EMMpos: addr=");
        Serial.print(cmd.param1);
        Serial.print(", dir=");
        Serial.print(cmd.param2);
        Serial.print(", clk=");
        Serial.println(cmd.param3);
        Emm_V5_Pos_Control( cmd.param1, cmd.param2, 2000, 250, cmd.param3, 0, 0);

        // 执行reset命令,重置机器人位置
      } else if(strcmp(cmd.cmd, "reset") == 0){
          Serial.println("Executing reset");
          ESP.restart();

      } else {
        // 未知命令类型
        Serial.println("Invalid command type");
      }
    }
  }
}

// 调试模式串口0命令任务函数
void Task_Debug_Serial0_CMD(void *pvParameters){
  char buffer[100];
  int bufferIndex = 0;
  
  while(1){
    if(Serial.available() > 0){
      char c = Serial.read();
      
      if(c == '\n' || c == '\r'){
        // 命令结束，处理命令
        if(bufferIndex > 0){
          buffer[bufferIndex] = '\0';
          
          // 解析命令
          DebugCommand_t cmd;
          if(sscanf(buffer, "%s %f %f %f", cmd.cmd, &cmd.param1, &cmd.param2, &cmd.param3) >= 1){
            // 发送命令到队列
            if(xQueueSend(xDebugQueue, &cmd, 100) == pdPASS){
              Serial.println("Command sent to debug queue");
            } else {
              Serial.println("Failed to send command to debug queue");
            }
          } else {
            Serial.println("Invalid command format");
          }
          
          // 重置缓冲区
          bufferIndex = 0;
        }
      } else if(bufferIndex < 99){
        // 读取字符到缓冲区
        buffer[bufferIndex++] = c;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}



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

  // 初始化调试队列
  xDebugQueue = xQueueCreate(10, sizeof(DebugCommand_t));
  if (xDebugQueue == NULL) {
    Serial.println("Failed to create debug queue");
  }

  //开启程序
  if (isdebug) {
    Serial.println("Debug mode ");//调试模式 开启对应任务
    xTaskCreate(Task_Debug_Mode, "Task_Debug_Mode", 8192, NULL, 5, NULL);
    xTaskCreate(Task_Debug_Serial0_CMD, "Task_Debug_Serial0_CMD", 8192, NULL, 5, NULL);
  } else {
    Serial.println("Release mode ");//正常运行 开启对应任务
    xTaskCreate(Task_MainStateMachine, "Task_MainStateMachine", 8192, NULL, 5, NULL);
    xTaskCreate(Task_Main_Serial0_CMD, "Task_Main_Serial0_CMD", 8192, NULL, 5, NULL);
  }

  Serial.println("Supermarket robot initialized");

}



void loop() {

  //ledcWrite(0, angleToDuty(90));
                    
}
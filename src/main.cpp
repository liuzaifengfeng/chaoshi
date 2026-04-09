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
RobotPose currentPose = {0, 0, 0};//当前理想机器人位置,中心坐标，(x,y,theta),mm,mm,度(0-360)
//RobotAngle servoPose = {0, 0, 0, 0, 0};//当前大臂高度，舵机角度,mm(0-1000),度(0-360)
bool isdebug = true;//是否调试模式

// 机器人舵机角度结构体
struct RobotAngle {
    float height;//大臂高度,mm(0-1000)
    float angle1;//通道1，
    float angle2;//通道2
    float angle3;//通道3
    float angle4;//通道4
};

//位置坐标-电机脉冲转换系数（mm-脉冲）
 float X_PULSE = 10.5f;
 float Y_PULSE = 11.4f;
 float THETA_PULSE = 84.0f;

// Debug命令队列
QueueHandle_t xDebugQueue;

// Debug命令结构体
typedef struct {
  char cmd[20];           // 命令类型
  float param1;           // 参数1
  float param2;           // 参数2
  float param3;           // 参数3
} DebugCommand_t;

// ================= 函数声明 =================

void Task_MainStateMachine(void *pvParameters);//主状态机任务函数（正常运行模式）
void Task_Main_Serial0_CMD(void *pvParameters);//主串口0命令任务函数
void Task_Debug_Mode(void *pvParameters);//调试模式任务函数（调试模式）
void Task_Debug_Serial0_CMD(void *pvParameters);//调试模式串口0命令任务函数


// 定义任务阶段
enum RobotState {
    STATE_INIT_WAIT,      // 启动后的10秒强制静止阶段 
    STATE_READ_ORDER,     // 前往小方桌识别购物需求 [cite: 56]
    STATE_PRE_REPLENISH,  // 前往货架1/2的第一层抓取补货物品 [cite: 64]
    STATE_DO_REPLENISH,   // 前往第三层完成补货任务 [cite: 64]
    STATE_GO_SHOPPING,    // 前往第二层寻找清单上的物品 [cite: 62]
    STATE_DELIVERING,     // 前往提货区投递 [cite: 62]
    STATE_FINISH_HOME     // 返回终点区并结束比赛 [cite: 66]
};

  // 购物清单与任务记录（具体结构根据实际需求补充）
  volatile bool isOrderReceived = false;//是否拍到购物需求
  int itemsPicked = 0;//已抓取物品数量
  int replenishDone = 0;//已补货物品数量

// 主状态机任务函数（正常运行模式）
void Task_MainStateMachine(void *pvParameters) {
    RobotState currentState = STATE_INIT_WAIT;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println("start"); 
    for (;;) {
        switch (currentState) {
            
            case STATE_INIT_WAIT:
                // 1. 强制静止10秒
                Serial.println("point1");
                vTaskDelay(10000 / portTICK_PERIOD_MS);//点一
                currentState = STATE_READ_ORDER;
                break;

            case STATE_READ_ORDER:
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                GotoPose(1300, 0, 0, true, false);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                Serial.println("point2");
                vTaskDelay(2000 / portTICK_PERIOD_MS);//点二
                //AdjustPose();
                for (int i = 0; i < 10; i++) {//10秒超时，未收到到购物需求，则先前往货架1/2的第一层抓取补货物品
                    if (isOrderReceived) {
                        break;
                    }
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                GotoPose(0, 350, 0 , true, false);
                //ledcWrite( 3, 50);
                GotoPose(700, 0, 0 , true, false);//点三
                Serial.println("point3");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                GotoPose(0, 1000, 0 , true, false);//点四
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                Serial.println("point4");
                GotoPose(0, -500, 0 , true, false);//点五
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                Serial.println("point5");
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                GotoPose(-700, 0, 0 , true, false);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                GotoPose(0, 0, 180 , true, false);
                vTaskDelay(3000 / portTICK_PERIOD_MS);//场地中央
                Serial.println("currentPose: " + String(currentPose.x) + ", " + String(currentPose.y) + ", " + String(currentPose.theta));
        Serial.print("REALLY pose: x=");
        Serial.print(GETRPose(avg_distances).x);
        Serial.print(", y=");
        Serial.print(GETRPose(avg_distances).y);
        Serial.print(", theta=");
        Serial.println(GETRPose(avg_distances).theta);
                AdjustPose();
                //打印当前机器人位置
                GotoPose(700, 0, 0 , true, false);//点六
                Serial.println("point6");
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                AdjustPose();
        Serial.print("REALLY pose: x=");
        Serial.print(GETRPose(avg_distances).x);
        Serial.print(", y=");
        Serial.print(GETRPose(avg_distances).y);
        Serial.print(", theta=");
        Serial.println(GETRPose(avg_distances).theta);
                //打印当前机器人位置
                Serial.println("currentPose: " + String(currentPose.x) + ", " + String(currentPose.y) + ", " + String(currentPose.theta));
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                GotoPose(0, 500, 0 , true, false);//点七
                Serial.println("point7");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                GotoPose(0, -500, 0 , true, false);
                GotoPose(0, -500, 0 , true, false);//点八
                Serial.println("point8");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                GotoPose(0, -400, 0 , true, false);
                GotoPose(0, 0, -90 , true, false);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                GotoPose(100, 0, 0 , true, false);
                Serial.println("currentPose: " + String(currentPose.x) + ", " + String(currentPose.y) + ", " + String(currentPose.theta));
        Serial.print("REALLY pose: x=");
        Serial.print(GETRPose(avg_distances).x);
        Serial.print(", y=");
        Serial.print(GETRPose(avg_distances).y);
        Serial.print(", theta=");
        Serial.println(GETRPose(avg_distances).theta);          
                AdjustPose();
                Serial.println("currentPose: " + String(currentPose.x) + ", " + String(currentPose.y) + ", " + String(currentPose.theta));
                GotoPose(0, -100, 0 , true, false);//点九
                Serial.println("point9");
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                GotoPose(0, -300, 0 , true, false);//点十
                Serial.println("point10");
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                GotoPose(0, -300, 0 , true, false);//点十一
                Serial.println("point11");
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                GotoPose(0, -300, 0 , true, false);//点十二
                Serial.println("point12");
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                GotoPose(-100, 0, 0 , true, false);
                GotoPose(0, 0, -90 , true, false);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                GotoPose(0, -800, 0 , true, false);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                GETRPose(avg_distances);//获取当前机器人位置
                //打印当前机器人位置
                Serial.println("currentPose: " + String(currentPose.x) + ", " + String(currentPose.y) + ", " + String(currentPose.theta));
                AdjustPose();
        Serial.print("REALLY pose: x=");
        Serial.print(GETRPose(avg_distances).x);
        Serial.print(", y=");
        Serial.print(GETRPose(avg_distances).y);
        Serial.print(", theta=");
        Serial.println(GETRPose(avg_distances).theta);
                //打印当前机器人理想位置
                Serial.println("currentPose: " + String(currentPose.x) + ", " + String(currentPose.y) + ", " + String(currentPose.theta));
                GotoPose(0, 500, 0, true, false);
                GotoPose(0, 500, 0, true, false);
                GotoPose(500, 0, 0, true, false);
                GotoPose(500, 0, 0 , true, false);//点十三
                Serial.println("point13");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                Serial.println("finish");
                vTaskDelete(NULL); 
                break;

            case STATE_PRE_REPLENISH:
                // 从货架1和2的第一层抓取待补货物品：锐澳、百事、旺仔、维他奶 
                // 遍历货架1/2 第一层


                break;

            case STATE_DO_REPLENISH:
                // 将抓到的4个物品放入第三层红线标记的指定位置 
                // 执行放置动作 
                currentState = STATE_GO_SHOPPING;
                break;

            case STATE_GO_SHOPPING:
                // 前往第二层寻找6个清单物品
                // 识别->回复"yes"->执行抓取动作
                break;

            case STATE_DELIVERING:
                // 前往提货区，识别头像匹配目标顾客 [cite: 62]
                currentState = STATE_FINISH_HOME;
                break;

            case STATE_FINISH_HOME:
                // 必须在8分钟内完全进入终点区
                vTaskDelete(NULL); 
                break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void Task_Main_Serial0_CMD(void *pvParameters) {
    char rxBuffer[64];
    int rxIdx = 0;

    for (;;) {
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                rxBuffer[rxIdx] = '\0';
                if (rxIdx > 0) {
                    // 处理指令
                    if (strstr(rxBuffer, "realy") != NULL) {
                        // 视觉初始化完成逻辑
                    } else if (strcmp(rxBuffer, "orderget") == 0) {
                       // 确认商品，触发抓取信号量或修改全局标志
                        isOrderReceived = true;
                    } else if (strcmp(rxBuffer, "no") == 0) {
                        // 拒绝商品，触发拒绝信号量或修改全局标志
                    }
                    rxIdx = 0;
                }
            } else if (rxIdx < 63) {
                rxBuffer[rxIdx++] = c;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// 调试模式任务函数（调试模式）
void Task_Debug_Mode(void *pvParameters){
  DebugCommand_t cmd;
  
  while(1){
    // 从队列中接收命令
    if(xQueueReceive(xDebugQueue, &cmd, portMAX_DELAY) == pdPASS){

       if(strcmp(cmd.cmd, "GOTOpose") == 0){
        // 执行GOTOposetf命令
        Serial.print("Executing GOTOpose: x=");
        Serial.print(cmd.param1);
        Serial.print(", y=");
        Serial.print(cmd.param2);
        Serial.print(", theta=");
        Serial.println(cmd.param3);
        // 使用GotoPose函数移动机器人到指定位置
        GotoPose(cmd.param1, cmd.param2, cmd.param3, true, false);
        
      } else if(strcmp(cmd.cmd, "GETCpose") == 0){
        // 执行GETCpose命令
        Serial.print("Current pose: x=");
        Serial.print(currentPose.x);
        Serial.print(", y=");
        Serial.print(currentPose.y);
        Serial.print(", theta=");
        Serial.println(currentPose.theta);

      } else if(strcmp(cmd.cmd, "GETRpose") == 0){
        // 执行GETRpose命令
        Serial.print("REALLY pose: x=");
        Serial.print(GETRPose(avg_distances).x);
        Serial.print(", y=");
        Serial.print(GETRPose(avg_distances).y);
        Serial.print(", theta=");
        Serial.println(GETRPose(avg_distances).theta);

      } else if(strcmp(cmd.cmd, "AdjustPose") == 0){
        // 执行AdjustPose命令,调整机器人位置
        Serial.print("Executing AdjustPose");
        // 使用AdjustPose函数调整机器人位置
        AdjustPose();

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
        Emm_V5_Pos_Control( cmd.param1, cmd.param2, 1000, 200, cmd.param3, 0, 0);

      } else if(strcmp(cmd.cmd, "PWM") == 0){
        // 执行PWM命令,设置PWM占空比
        Serial.print("Executing PWM: addr=");
        Serial.print(cmd.param1);
        Serial.print(", angle=");
        Serial.println(cmd.param2);
        ledcWrite(cmd.param1, angleToDuty(cmd.param2));

      } else if(strcmp(cmd.cmd, "reset") == 0){
          Serial.println("Executing reset");
          ESP.restart();//重启ESP32

      } else if(strcmp(cmd.cmd, "help") == 0){
        // 执行help命令,显示帮助信息
        Serial.println("Available commands:");
        Serial.println("GOTOposet: x y theta");
        Serial.println("GOTOposetf: x y theta");
        Serial.println("GETpose");
        Serial.println("GETdist");
        Serial.println("EMMpos addr dir clk");
        Serial.println("reset");
        Serial.println("help");
           
      } else {
        // 未知命令类型
        Serial.println("Invalid command type ,please input help");
        //echo
        Serial.println(cmd.cmd);
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

  currentPose = {250, 400, 0};//初始化机器人中心位置为(250mm,400mm,0)

  //开启程序
  if (isdebug) {
    Serial.println("Debug mode ");//调试模式 开启对应任务
    xDebugQueue = xQueueCreate(20, sizeof(DebugCommand_t));  // 初始化调试队列
    if (xDebugQueue == NULL) { Serial.println("Failed to create debug queue");}
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
  vTaskDelete(NULL);
                    
}
/**********************************************************
*** 超市机器人控制程序
*** 编写作者：LZF
*** 技术支持：GEMINI
**********************************************************/

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h> // 显式引入队列头文件
#include <FastLED.h>
#include "ota_service.h"
#include <ArduinoJson.h> 

#include "Emm_V5.h"
#include "lidar.h"
#include "pwm.h"

#define MODE_key 0
#define LED_PIN 48
#define NUM_LEDS 1
#define OTA_HOSTNAME "espchaoshi"
#define VERSION "2.1.0"

#define Speed_buhuo 15 // 补货速度，mm/s
#define Speed_tihuo 10 // 提货速度，mm/s

#define X_buhuo_1 2215
#define X_buhuo_2 880
#define X_tihuo_1 2255
#define X_tihuo_2 840

#define X_buhuo_P 80
#define X_buhuo_N 130
#define X_tihuo_P 80
#define X_tihuo_N 180

#define X_buhuo_put 200
       
#define PWM_3_helf 135 
#define PWM_3_close 170 
#define PWM_3_open 0 

// 视觉/上位机指令枚举
enum VisualCmdType {
    VISUAL_CMD_NONE,
    VISUAL_CMD_SEARCH,
    VISUAL_CMD_NEXT,
    VISUAL_CMD_NETGET,
    VISUAL_CMD_GET0,
    VISUAL_CMD_ORDERGET,
    VISUAL_CMD_OK
};

// //四个待补货商品位置（二维数组），{ x坐标, y坐标, theta角度, 是否完成}
// float buhuo[5][4] = { // 商品位置 180 * i(取1-5)，从下往上数
//     {0, 0, 0, 0}, // 商品0 空 
//     {2185, 840+180*2, 0, 0}, // 商品2 百事可乐
//     {910, 700+180*2, 180, 0}, // 商品3 旺仔牛奶
//     {910, 700+180*4, 180, 0}, // 商品4 维他奶
//     {2185, 840+180*4, 0, 0}, // 商品5（1） 锐澳水蜜桃 
// };

// 待补货商品位置
float buhuo[5][4] = { 
    {0, 0, 0, 0}, //
    {2185, 840+180*2, 0, 0}, // 商品2 百事可乐
    {2185, 840+180*4, 0, 0}, // 商品3 旺仔牛奶
    {910, 700+180*2, 180, 0}, // 商品4 维他奶
    {910, 700+180*4, 180, 0}, // 商品5（1） 锐澳水蜜桃 
};

volatile int getBuhuoIndex = 0;
CRGB leds[NUM_LEDS];
RobotPose currentPose = {0, 0, 0};

TaskHandle_t xTask_MainStateMachine_Handle = NULL;
TimerHandle_t xHomeTimer = NULL;
volatile bool isdebug = false;
volatile bool ready = false;
volatile int DX_dist = 0;
volatile int report_hz = 0;

struct RobotAngle {
    float height;
    float angle1;
    float angle2;
    float angle3;
    float angle4;
};

float X_PULSE = 10.5f;
float Y_PULSE = 11.4f;
float THETA_PULSE = 83.4f;
float HEIGHT_PULSE = 32.26f;
float DX_PULSE = 0.1f;

// 消息队列句柄
QueueHandle_t xDebugQueue;
QueueHandle_t xVisualTaskQueue; // 新增：用于主状态机接收上位机通知的队列

typedef struct {
  char cmd[20];           
  float param1;           
  float param2;           
  float param3;           
} DebugCommand_t;

// ================= 函数声明 =================
void Task_MainStateMachine(void *pvParameters);
void Task_Main_Serial0_CMD(void *pvParameters);
void Task_Debug_Mode(void *pvParameters);
void Task_Debug_Serial0_CMD(void *pvParameters);
void Task_Debug_pose(void *pvParameters);
void get0ok();
void while_get0();

enum RobotState {
    STATE_INIT_WAIT,       
    STATE_READ_ORDER,      
    STATE_PRE_REPLENISH,   
    STATE_DO_REPLENISH,    
    STATE_GO_SHOPPING,     
    STATE_DELIVERING,      
    STATE_FINISH_HOME      
};

RobotState currentState = STATE_INIT_WAIT;

// 削减掉被队列替代的全局变量，保留非队列控制的业务变量
volatile bool isOrderReceived = false; 
volatile int itemsPicked = 0;
volatile int replenishDone = 0;
volatile int deliverDone = 0;
volatile bool isCustomer = false;  
volatile bool isDelivered = false;
volatile int OK_ = 0; 

/**
 * @brief 定时器回调函数 - 超时自动触发回家逻辑
 */
void vHomeTimerCallback(TimerHandle_t xTimer) {
  if(currentState != STATE_FINISH_HOME){
    Serial.println("[TIMER] Timeout triggered! Forcing robot to go home...");
    if (xTask_MainStateMachine_Handle != NULL) {
        vTaskSuspend(xTask_MainStateMachine_Handle);
        Serial.println("[TIMER] Task_MainStateMachine suspended");
    }
    movepose(0,0,1);
    GotoHeight(520);
    ledcWrite(3, angleToDuty(180)); 
    ledcWrite(5,angleToDuty(240) );
    GotoPose(2200, 2200, 0 , false, false);
    ledcWrite(2, angleToDuty(300));
    AdjustPose();
    GotoHeight(0);
    GotoPose(2850, 2200, 0 , false, false);
    Serial.println("[TIMER] Robot arrived at home position");
    Emm_V5_En_Control_all(false);
  }
}

void Task_MainStateMachine(void *pvParameters) {
    volatile int buhuoNOW = 0;
    RobotPose lastpose = {0, 0, 0};
    volatile bool isReplenishDone_1 = false;
    volatile bool isReplenishDone_2 = false;
    volatile int caoweiNOW = 0;
    volatile int zhuaziNOW = 0;
    volatile int start_time = 0;

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while(!ready){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    while (1) {
        switch (currentState) {
            case STATE_INIT_WAIT:
                for(int i = 1; i >= 0; i--){
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    Serial.println(String(i));
                }
                start_time = millis();
                Serial.println("start"); 

                if (xHomeTimer != NULL) {
                  xTimerStart(xHomeTimer, 0);
                  Serial.println("[SYSTEM] Home timer started");
                }
                GotoHeight(0);
                currentState = STATE_READ_ORDER;
                break;

            case STATE_READ_ORDER:
                ledcWrite( 1, angleToDuty(120));
                vTaskDelay(100 / portTICK_PERIOD_MS);
                Serial.println("taiqi");
                Serial.println("[0]");
                for(int i = 0; i < 100; i++){
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    if(OK_){ OK_ = 0; break; }
                }
                GotoPose(1100, 0, 0, true, false);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                Emm_V5_En_Control_all(false);
                
                // 此处清空一次队列，防止之前的残留信号干扰
                xQueueReset(xVisualTaskQueue);

                for (int i = 0; i < 30; i++) {
                    vTaskDelay(1000 / portTICK_PERIOD_MS); 
                    Serial.println("i="+String(i));
                    if (isOrderReceived) { break; }    
                }
                Emm_V5_En_Control_all(true);
                GotoHeight(550);
                GotoPose(880, 0, 0 , true, false);
                ledcWrite( 1, angleToDuty(270));
                Serial.println("[2]");
                for(int i = 0; i < 100; i++){
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    if(OK_){ OK_ = 0; break; }
                }
                ledcWrite( 2, angleToDuty(60));
                vTaskDelay(500 / portTICK_PERIOD_MS);
                ledcWrite( 3, angleToDuty(PWM_3_open));
                GotoPose(0, 700, 0 , true, false);
                AdjustPose();
                GotoHeight(0);
                vTaskDelay(2000 / portTICK_PERIOD_MS);

                //currentState = STATE_GO_SHOPPING;
                currentState = STATE_PRE_REPLENISH;
                break;

            case STATE_PRE_REPLENISH:
                Serial.println("start replenish");
                if(!isReplenishDone_1) {
                  Serial.println("go to one replenish");
                  GotoHeight(0);
                  GotoPose(X_buhuo_1, 950, 0, false, false);
                } else if(!isReplenishDone_2) {
                  Serial.println("go to two replenish");
                  if(currentPose.theta == 0) {
                    GotoPose(-500, 0, 0 , true, false);
                    GotoHeight(0);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    GotoPose(1600, 1700, 180 , false, false); 
                    GotoPose(X_buhuo_2, 1800, 180 , false, false); 
                    AdjustPose();
                  }else{
                    GotoHeight(0);
                    GotoPose(X_buhuo_2, 1800, 180 , false, false); 
                    AdjustPose();
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                  }
                } else {
                  Serial.println("[buhuodone]");
                  currentState = STATE_GO_SHOPPING;
                  break;
                }

                getBuhuoIndex = 0;
                if(!isReplenishDone_1) {
                  movepose(1, Speed_buhuo,0);
                  getBuhuoIndex = 0;
                  while(avg_distances[0] < 1600){
                    getBuhuoIndex = 0;
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    if(getBuhuoIndex != 0) {
                      movepose(0, 0, 1);
                      lastpose = currentPose;
                      Serial.println("get buhuo"+String(getBuhuoIndex));
                      buhuoNOW = getBuhuoIndex-1;
                      lastpose = currentPose;

                      if(buhuo[buhuoNOW][2] == currentPose.theta) {
                        ledcWrite( 3, angleToDuty(PWM_3_helf));
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoPose(X_buhuo_P, 0, 0 , true, false);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(PWM_3_close));
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoHeight(50);
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        GotoPose(-X_buhuo_N, 0, 0 , true, false);
                        GotoPose(buhuo[buhuoNOW][0], buhuo[buhuoNOW][1], buhuo[buhuoNOW][2], false, false);
                        AdjustPose();
                        GotoHeight(640);
                        vTaskDelay(3000 / portTICK_PERIOD_MS);
                        GotoPose(200, 0, 0 , true, false);
                        GotoHeight(595);
                        vTaskDelay(800 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(PWM_3_helf));
                        vTaskDelay(400 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(110));
                        vTaskDelay(400 / portTICK_PERIOD_MS);
                        GotoPose(-200, 0, 0 , true, false);
                        ledcWrite( 3, angleToDuty(PWM_3_open));  
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoHeight(0);
                        vTaskDelay(2000 / portTICK_PERIOD_MS);
                        replenishDone++;
                        getBuhuoIndex = 0;
                        GotoPose(lastpose.x, lastpose.y, lastpose.theta, false, false);
                        GotoHeight(0);
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        movepose(1, Speed_buhuo,0);                          
                      }else {
                        if(caoweiNOW != 0 ){
                            ledcWrite( 3, angleToDuty(PWM_3_helf));
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            GotoPose(X_buhuo_P, 0, 0 , true, false);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            ledcWrite( 3, angleToDuty(PWM_3_close));
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            GotoHeight(50);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            GotoPose(-X_buhuo_N, 0, 0 , true, false);
                            zhuaziNOW = buhuoNOW;
                            getBuhuoIndex = 0;
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            isReplenishDone_1 = true;
                            currentState = STATE_DO_REPLENISH;
                            break;
                        }else{
                            ledcWrite( 3, angleToDuty(PWM_3_helf));
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            GotoPose(X_buhuo_P, 0, 0 , true, false);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            ledcWrite( 3, angleToDuty(PWM_3_close));
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            GotoHeight(50);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            GotoPose(-X_buhuo_N, 0, 0 , true, false);
                            GotoHeight(550);
                            vTaskDelay(4000 / portTICK_PERIOD_MS);
                            ledcWrite( 2, angleToDuty(270));
                            vTaskDelay(2000 / portTICK_PERIOD_MS);
                            GotoHeight(450);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            ledcWrite( 3, angleToDuty(PWM_3_open));
                            GotoHeight(640);
                            vTaskDelay(3000 / portTICK_PERIOD_MS);
                            ledcWrite( 2, angleToDuty(60));
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            caoweiNOW = buhuoNOW;
                            getBuhuoIndex = 0;
                            GotoHeight(0);
                            GotoPose(lastpose.x, lastpose.y, lastpose.theta, false, false);
                            vTaskDelay(2000 / portTICK_PERIOD_MS);
                            movepose(1, Speed_buhuo,0);                          
                        }
                      }
                    }
                  }
                  movepose(0, 0, 1);
                  isReplenishDone_1 = true;
                  if(caoweiNOW == 0 && zhuaziNOW == 0){
                    currentState = STATE_PRE_REPLENISH;
                  } else {
                    currentState = STATE_DO_REPLENISH;
                  }
                } else if(!isReplenishDone_2) {
                  movepose(1, Speed_buhuo,0);
                  getBuhuoIndex = 0;
                  while(avg_distances[0] < 1600){
                    getBuhuoIndex = 0;
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    if(getBuhuoIndex != 0) {
                        movepose(0, 0, 1);
                        Serial.println("get buhuo "+String(getBuhuoIndex));
                        buhuoNOW = getBuhuoIndex-1;
                        lastpose = currentPose;
                        getBuhuoIndex = 0;

                      if(buhuo[buhuoNOW][2] == currentPose.theta) {
                        ledcWrite( 3, angleToDuty(PWM_3_helf));
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoPose(X_buhuo_P, 0, 0 , true, false);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(PWM_3_close));
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoHeight(50);
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        GotoPose(-X_buhuo_N, 0, 0 , true, false);
                        GotoPose(buhuo[buhuoNOW][0], buhuo[buhuoNOW][1], buhuo[buhuoNOW][2], false, false);
                        AdjustPose();
                        GotoHeight(640);
                        vTaskDelay(3000 / portTICK_PERIOD_MS);
                        GotoPose(200, 0, 0 , true, false);
                        GotoHeight(595);
                        vTaskDelay(800 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(PWM_3_helf));
                        vTaskDelay(400 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(110));
                        vTaskDelay(400 / portTICK_PERIOD_MS);
                        GotoPose(-200, 0, 0 , true, false);
                        ledcWrite( 3, angleToDuty(PWM_3_open));  
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoHeight(0);
                        vTaskDelay(2000 / portTICK_PERIOD_MS);
                        replenishDone++;
                        getBuhuoIndex = 0;
                        GotoPose(lastpose.x, lastpose.y, lastpose.theta, false, false);
                        GotoHeight(0);
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        movepose(1, Speed_buhuo,0);                       
                      }else if(buhuo[buhuoNOW][2] != currentPose.theta) {
                        if(caoweiNOW != 0 ){
                            ledcWrite( 3, angleToDuty(PWM_3_helf));
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            GotoPose(X_buhuo_P, 0, 0 , true, false);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            ledcWrite( 3, angleToDuty(PWM_3_close));
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            GotoHeight(50);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            GotoPose(-X_buhuo_N, 0, 0 , true, false);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            zhuaziNOW = buhuoNOW;
                            getBuhuoIndex = 0;
                            currentState = STATE_DO_REPLENISH;
                            break;
                        }else{
                            ledcWrite( 3, angleToDuty(PWM_3_helf));
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            GotoPose(X_buhuo_P, 0, 0 , true, false);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            ledcWrite( 3, angleToDuty(PWM_3_close));
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            GotoHeight(50);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            GotoPose(-X_buhuo_N, 0, 0 , true, false);
                            GotoHeight(550);
                            vTaskDelay(4000 / portTICK_PERIOD_MS);
                            ledcWrite( 2, angleToDuty(270));
                            vTaskDelay(2000 / portTICK_PERIOD_MS);
                            GotoHeight(450);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            ledcWrite( 3, angleToDuty(PWM_3_open));
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            GotoHeight(640);
                            vTaskDelay(3000 / portTICK_PERIOD_MS);
                            ledcWrite( 2, angleToDuty(60));
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            caoweiNOW = buhuoNOW;
                            getBuhuoIndex = 0;
                            GotoHeight(0);
                            GotoPose(lastpose.x, lastpose.y, lastpose.theta, false, false);
                            vTaskDelay(2000 / portTICK_PERIOD_MS);
                            movepose(1, Speed_buhuo,0);      
                        }
                      }
                    }
                  }
                  movepose(0, 0, 1);
                  isReplenishDone_2 = true;
                  if(caoweiNOW == 0 && zhuaziNOW == 0){
                    currentState = STATE_GO_SHOPPING;
                    Serial.println("[buhuodone]");
                  } else {
                    currentState = STATE_DO_REPLENISH;
                  }
                } else {
                  if(caoweiNOW == 0 && zhuaziNOW == 0){
                    currentState = STATE_GO_SHOPPING;
                    Serial.println("[buhuodone]");
                  } else {
                    currentState = STATE_DO_REPLENISH;
                  }
                }
                break;

            case STATE_DO_REPLENISH:
                if(caoweiNOW != 0){ Serial.println("put  buhuo (caowei)"+String(caoweiNOW)); }
                if(zhuaziNOW != 0){ Serial.println("put  buhuo (zhuazi)"+String(zhuaziNOW)); }    
                vTaskDelay(1000 / portTICK_PERIOD_MS);

                if(isReplenishDone_2 == 0) {
                  Serial.println("put  buhuo 1");
                  GotoPose(1600, 1300, 180 , false, false);
                  vTaskDelay(1000 / portTICK_PERIOD_MS);
                  AdjustPose();
                  vTaskDelay(1000 / portTICK_PERIOD_MS);
                  if(zhuaziNOW != 0){
                        GotoPose(buhuo[zhuaziNOW][0], buhuo[zhuaziNOW][1], buhuo[zhuaziNOW][2], false, false);
                        AdjustPose();
                        GotoHeight(640);
                        vTaskDelay(3000 / portTICK_PERIOD_MS);
                        GotoPose(X_buhuo_put, 0, 0 , true, false);
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        GotoHeight(595);
                        vTaskDelay(800 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(PWM_3_helf));
                        vTaskDelay(400 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(110));
                        vTaskDelay(400 / portTICK_PERIOD_MS);
                        GotoPose(-X_buhuo_put, 0, 0 , true, false);
                        ledcWrite( 3, angleToDuty(PWM_3_open));  
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        replenishDone++;
                    }
                  if(caoweiNOW != 0){
                        GotoPose(buhuo[caoweiNOW][0], buhuo[caoweiNOW][1], buhuo[caoweiNOW][2], false, false);
                        AdjustPose();
                        GotoHeight(640);
                        vTaskDelay(2000 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(120));
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        ledcWrite( 2, angleToDuty(270));
                        vTaskDelay(2000 / portTICK_PERIOD_MS);
                        GotoHeight(470);
                        vTaskDelay(1500 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(PWM_3_close));
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoHeight(550);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        ledcWrite( 2, angleToDuty(60));
                        GotoHeight(640);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoPose(X_buhuo_put, 0, 0 , true, false);
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        GotoHeight(595);
                        vTaskDelay(800 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(PWM_3_helf));
                        vTaskDelay(400 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(110));
                        vTaskDelay(400 / portTICK_PERIOD_MS);
                        GotoPose(-X_buhuo_put, 0, 0 , true, false);
                        ledcWrite( 3, angleToDuty(PWM_3_open));  
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        replenishDone++;
                    }
                    zhuaziNOW = 0;
                    caoweiNOW = 0;
                }else {
                  Serial.println("put  buhuo 2");
                  GotoPose(1600, 1300, 0 , false, false);
                  vTaskDelay(1000 / portTICK_PERIOD_MS);
                  AdjustPose();
                  vTaskDelay(1000 / portTICK_PERIOD_MS);
                  if(zhuaziNOW != 0){
                        GotoPose(buhuo[zhuaziNOW][0], buhuo[zhuaziNOW][1], buhuo[zhuaziNOW][2] , false, false);
                        GotoHeight(640);
                        vTaskDelay(3000 / portTICK_PERIOD_MS);
                        GotoPose(X_buhuo_put, 0, 0 , true, false);
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        GotoHeight(595);
                        vTaskDelay(800 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(PWM_3_helf));
                        vTaskDelay(400 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(110));
                        vTaskDelay(400 / portTICK_PERIOD_MS);
                        GotoPose(-X_buhuo_put, 0, 0 , true, false);
                        ledcWrite( 3, angleToDuty(PWM_3_open));  
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        replenishDone++;
                    }
                  if(caoweiNOW != 0){
                        GotoPose(buhuo[caoweiNOW][0], buhuo[caoweiNOW][1], buhuo[caoweiNOW][2]  , false, false);
                        GotoHeight(640);
                        vTaskDelay(2000 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(120));
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        ledcWrite( 2, angleToDuty(270));
                        vTaskDelay(2000 / portTICK_PERIOD_MS);
                        GotoHeight(470);
                        vTaskDelay(1500 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(PWM_3_close));
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoHeight(550);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        ledcWrite( 2, angleToDuty(60));
                        GotoHeight(640);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoPose(X_buhuo_put, 0, 0 , true, false);
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        GotoHeight(595);
                        vTaskDelay(800 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(PWM_3_helf));
                        vTaskDelay(400 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(110));
                        vTaskDelay(400 / portTICK_PERIOD_MS);
                        GotoPose(-X_buhuo_put, 0, 0 , true, false);
                        ledcWrite( 3, angleToDuty(PWM_3_open));  
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        replenishDone++;
                    }
                    zhuaziNOW = 0;
                    caoweiNOW = 0;
                }

                if(isReplenishDone_1 && isReplenishDone_2) {
                    currentState = STATE_GO_SHOPPING;
                    Serial.println("[buhuodone]");
                } else {
                    currentState = STATE_PRE_REPLENISH;
                }
                break;

            case STATE_GO_SHOPPING:
                Serial.println("go to tihuo");
                Serial.println("NOW_Time: " + String(millis()-start_time) + "ms");
                Serial.println("[3]");
                vTaskDelay(6000 / portTICK_PERIOD_MS);



                // 进入提货环节前，清空队列中的过期老数据
                xQueueReset(xVisualTaskQueue);

                GotoHeight(315);

                if(currentPose.theta == 0){
                    GotoPose(X_tihuo_1, 900, 0 , false, false);
                    AdjustPose();
                    vTaskDelay(500 / portTICK_PERIOD_MS);

                    for(int i = 0; i < 100; i++){
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    if(OK_){ OK_ = 0; break; }
                    }

                    Serial.println("[3_1]");
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    
                    movepose(1, Speed_tihuo,0);
                    while_get0(); // 阻塞处理提货数据
                    movepose(0, 0, 1);
                    Serial.println("[3_0]");

                    GotoPose(1600, 1800, 180 , false, false);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    GotoPose(X_tihuo_2, 1850, 180 , false, false); 
                    AdjustPose();
                    vTaskDelay(500 / portTICK_PERIOD_MS);

                    Serial.println("[3_1]");
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    movepose(1, Speed_tihuo,0);
                    while_get0(); 
                    movepose(0, 0, 1);
                }else {
                    GotoPose(X_tihuo_2, 1800, 180 , false, false);
                    AdjustPose();
                    vTaskDelay(500 / portTICK_PERIOD_MS);

                    for(int i = 0; i < 100; i++){
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    if(OK_){ OK_ = 0; break; }
                    }

                    Serial.println("[3_1]");
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    
                    movepose(1, Speed_tihuo,0);
                    while_get0();
                    movepose(0, 0, 1);
                    Serial.println("[3_0]");

                    GotoPose(1600, 950, 0 , false, false);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    GotoPose(X_tihuo_1, 900, 0 , false, false);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    AdjustPose();

                    Serial.println("[3_1]");
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    movepose(1, Speed_tihuo,0);
                    while_get0();
                    movepose(0, 0, 1);
                }

                Serial.println("finish shopping: " + String(deliverDone));
                Serial.println("[tihuodone]");
                currentState = STATE_DELIVERING;
                break;

            case STATE_DELIVERING:
                Serial.println("NOW_Time: " + String(millis()-start_time) + "ms");
                Serial.println("[1]");
                vTaskDelay(20 / portTICK_PERIOD_MS);
                Serial.println("[1]");
                isCustomer = false;
                for(int i = 0; i < 100; i++){
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    if(OK_){ OK_ = 0; break; }
                }
                GotoHeight(630);
                GotoPose(950, 2200, 90 , false, false);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                AdjustPose();
                for(int i = 0; i <= 4; i++) {
                  vTaskDelay(2000 / portTICK_PERIOD_MS);
                  if(isCustomer) {
                    Serial.println("dump: " + String(i) + " customer");
                    GotoPose(170, -280, 0 , true, false);
                    ledcWrite( 5, angleToDuty(70));
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                    ledcWrite( 5, angleToDuty(90));
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    ledcWrite( 5, angleToDuty(70));
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    ledcWrite( 5, angleToDuty(90));
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    ledcWrite( 5, angleToDuty(70));
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                    ledcWrite( 5, angleToDuty(240));
                    break;
                  }
                  GotoPose(0, -300, 0 , true, false);
                }
                isDelivered = true;
                currentState = STATE_FINISH_HOME;
                break;

            case STATE_FINISH_HOME:
                ledcWrite( 3, angleToDuty(PWM_3_close));
                GotoHeight(520);
                ledcWrite( 4, angleToDuty(0));
                ledcWrite( 5, angleToDuty(250));
                ledcWrite( 2, angleToDuty(300));
                vTaskDelay(1500 / portTICK_PERIOD_MS);
                GotoHeight(0);

                if (isDelivered){
                  Serial.println("done,go home");
                  GotoPose(-200, 0, -90 , true, false);
                  vTaskDelay(500 / portTICK_PERIOD_MS);
                  GotoPose(0, 50, 0 , true, false);
                  GotoPose(3000, 2100, 0 , false, false);
                } else {
                  Serial.println("not done,go home");
                  GotoPose(2200, 2100, 0 , false, false);
                  vTaskDelay(500 / portTICK_PERIOD_MS);
                  AdjustPose();
                  GotoPose(3000, 2100, 0 , false, false);
                }
                
                Serial.println("finish home");
                Serial.println("consume time: " + String(millis() - start_time) + " ms");
                Emm_V5_En_Control_all(false);
                vTaskDelete(NULL);
                vTaskDelay(100000000 / portTICK_PERIOD_MS);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if(millis() - start_time > 470000) {
          Serial.println("timeout backup");
          currentState = STATE_FINISH_HOME;
        }
    }
}

// 修改后的主任务串口通信函数：将布尔和标志转变为写队列
void Task_Main_Serial0_CMD(void *pvParameters) {
    char rxBuffer[64];
    int rxIdx = 0;
    VisualCmdType msgToSend;

    for (;;) {
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                rxBuffer[rxIdx] = '\0';
                if (rxIdx > 0) {
                    msgToSend = VISUAL_CMD_NONE;

                    if (strstr(rxBuffer, "ready") != 0) {
                        Serial.println("ready->");      
                        ready = true;
                    } else if (strcmp(rxBuffer, "orderget") == 0) {
                        isOrderReceived = true;
                        Serial.println("orderget->");
                    } else if (strcmp(rxBuffer, "get1") == 0) {
                        getBuhuoIndex = 5;
                        Serial.println("get1->");
                    } else if (strcmp(rxBuffer, "get2") == 0) {
                        getBuhuoIndex = 2;
                        Serial.println("get2->");
                    } else if (strcmp(rxBuffer, "get3") == 0) {
                        getBuhuoIndex = 3;
                        Serial.println("get3->");
                    } else if (strcmp(rxBuffer, "get4") == 0) {
                        getBuhuoIndex = 4;
                        Serial.println("get4->");
                    } 
                    // --- 以下命令通过消息队列发送 ---
                    else if (strcmp(rxBuffer, "get0") == 0) {
                        msgToSend = VISUAL_CMD_GET0;
                        Serial.println("get0->");
                    } else if (strcmp(rxBuffer, "search") == 0) {
                        msgToSend = VISUAL_CMD_SEARCH;
                        Serial.println("search ing...");
                    } else if (strcmp(rxBuffer, "next") == 0) {
                        msgToSend = VISUAL_CMD_NEXT;
                        Serial.println("get next");
                    } else if (strcmp(rxBuffer, "netget") == 0) {
                        msgToSend = VISUAL_CMD_NETGET;
                        Serial.println("get netget");
                    } 
                    // --------------------------------
                    else if (strcmp(rxBuffer, "true") == 0) {
                        isCustomer = true;
                        Serial.println("true->");
                    } else if (strstr(rxBuffer, "[") != NULL && strstr(rxBuffer, "]") != NULL) {
                        int dx_value;
                        if (sscanf(rxBuffer, "[%d]", &dx_value) == 1) {
                            DX_dist = dx_value;
                        }
                    } else if (strstr(rxBuffer, "posedebug") != 0) {
                        int hz;
                        if (sscanf(rxBuffer, "posedebug %d", &hz) == 1) {
                            report_hz = hz; 
                            Serial.printf("Main Mode: Report set to %d Hz\n", hz);
                        }
                    } else if (strcmp(rxBuffer, "ok") == 0) {
                        OK_ = 1;
                        Serial.println("get ok");
                    } 

                    // 如果触发了需要队列通知的指令，则推入队列中
                    if (msgToSend != VISUAL_CMD_NONE && xVisualTaskQueue != NULL) {
                        if (xQueueSend(xVisualTaskQueue, &msgToSend, 0) != pdPASS) {
                            Serial.println("[QUEUE ERROR] Visual queue full!");
                        }
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

// 调试模式任务函数
void Task_Debug_Mode(void *pvParameters){
  DebugCommand_t cmd;
  while(1){
    if(xQueueReceive(xDebugQueue, &cmd, portMAX_DELAY) == pdPASS){
       if(strcmp(cmd.cmd, "GOTOpose") == 0){
        GotoPose(cmd.param1, cmd.param2, cmd.param3, false, false);
       } else if(strcmp(cmd.cmd, "GOTORpose") == 0){
        GotoPose(cmd.param1, cmd.param2, cmd.param3, true, false);
       } else if(strcmp(cmd.cmd,  "En_C") == 0){
        Emm_V5_En_Control_all(cmd.param1);
       } else if(strcmp(cmd.cmd, "GETCpose") == 0){
        Serial.printf("Current pose: x=%.2f, y=%.2f, theta=%.2f\n", currentPose.x, currentPose.y, currentPose.theta);
       } else if(strcmp(cmd.cmd, "GETRpose") == 0){
        Serial.printf("REALLY pose: x=%.2f, y=%.2f, theta=%.2f\n", GETRPose(avg_distances).x, GETRPose(avg_distances).y, GETRPose(avg_distances).theta);
       } else if(strcmp(cmd.cmd, "AdjustPose") == 0){
        AdjustPose();
       } else if(strcmp(cmd.cmd, "GETdist") == 0){
        Serial.printf("LiDAR distances: CH0=%d, CH1=%d, CH2=%d, CH3=%d\n", avg_distances[0], avg_distances[1], avg_distances[2], avg_distances[3]);
       } else if(strcmp(cmd.cmd, "movepose") == 0){
        movepose(cmd.param1, cmd.param2, cmd.param3);
       } else if(strcmp(cmd.cmd, "EMMpos") == 0){
        Emm_V5_Pos_Control( cmd.param1, cmd.param2, 1000, 200, cmd.param3, 0, 0);
       } else if(strcmp(cmd.cmd, "GOTOHeight") == 0){
        GotoHeight(cmd.param1);
       } else if(strcmp(cmd.cmd, "PWM") == 0){
        ledcWrite(cmd.param1, angleToDuty(cmd.param2));
       } else if(strcmp(cmd.cmd, "reset") == 0){
          ESP.restart();
       } else if(strcmp(cmd.cmd, "posedebug") == 0){
        report_hz = (int)cmd.param1;
       } else if(strcmp(cmd.cmd, "help") == 0){
        Serial.println("Available commands: GOTOpose, GOTORpose, GETCpose, GETRpose, GETdist, movepose, AdjustPose, GOTOHeight, PWM, reset");
       }
    }
  }
}

void Task_Debug_Serial0_CMD(void *pvParameters){
  char buffer[100];
  int bufferIndex = 0;
  while(1){
    if(Serial.available() > 0){
      char c = Serial.read();
      if(c == '\n' || c == '\r'){
        if(bufferIndex > 0){
          buffer[bufferIndex] = '\0';
          DebugCommand_t cmd;
          if(sscanf(buffer, "%s %f %f %f", cmd.cmd, &cmd.param1, &cmd.param2, &cmd.param3) >= 1){
            xQueueSend(xDebugQueue, &cmd, 100);
          }
          bufferIndex = 0;
        }
      } else if(bufferIndex < 99){
        buffer[bufferIndex++] = c;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void Task_Debug_pose(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xDefaultDelay = pdMS_TO_TICKS(100);
    while (1) {
        if (report_hz > 0) {
            int interval_ms = 1000 / report_hz;
            xLastWakeTime = xTaskGetTickCount();
            RobotPose realPose = GETRPose(avg_distances);
            Serial.printf("Cpose %.2f %.2f %.2f\n\r", currentPose.x, currentPose.y, currentPose.theta);
            Serial.printf("Rpose %.2f %.2f %.2f\n\r", realPose.x, realPose.y, realPose.theta);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(interval_ms));
        } else {
            vTaskDelay(xDefaultDelay);
        }
    }
}


void setup() {
  Serial.begin(115200);
  initPWM();
  initLidar();
  Emm_V5_Init();
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(10);  

  currentPose = {250, 400, 0};
  init_ota_service("null123", "1234567899", OTA_HOSTNAME);
  leds[0] = CRGB::Red;
  FastLED.show();

  pinMode(MODE_key, INPUT_PULLUP);  
  unsigned long startTime = millis();
  bool bootKeyPressed = false;
  
  while (millis() - startTime < 5000) {  
    if (digitalRead(MODE_key) == LOW) {  
      bootKeyPressed = true;
      Serial.println("MODE_key pressed, entering debug mode...");
      break;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  
  }
  
  if (bootKeyPressed) { isdebug = true; }

  // 新增：初始化用于视觉命令传递的消息队列（深度为10）
  xVisualTaskQueue = xQueueCreate(10, sizeof(VisualCmdType));

  if (isdebug) {
    Serial.println("Debug mode ");
    leds[0] = CRGB::Yellow;
    FastLED.show();
    xDebugQueue = xQueueCreate(20, sizeof(DebugCommand_t));  
    xTaskCreate(Task_Debug_Mode, "Task_Debug_Mode", 16384, NULL, 5, NULL);
    xTaskCreate(Task_Debug_Serial0_CMD, "Task_Debug_Serial0_CMD", 16384, NULL, 5, NULL);
  } else {
    Serial.println("Release mode ");
    leds[0] = CRGB::Green;
    FastLED.show();
    xTaskCreate(Task_MainStateMachine, "Task_MainStateMachine", 16384, NULL, 8, &xTask_MainStateMachine_Handle);
    xTaskCreate(Task_Main_Serial0_CMD, "Task_Main_Serial0_CMD", 16384, NULL, 8, NULL);
    
    const TickType_t xTimerPeriod = pdMS_TO_TICKS(460000); 
    xHomeTimer = xTimerCreate("HomeTimer", xTimerPeriod, pdFALSE, (void *)0, vHomeTimerCallback);
  }

  //xTaskCreate(Task_Debug_pose, "Task_Debug_pose", 4096, NULL, 4, NULL);
  //Serial.println("Supermarket robot initialized");
  //Serial.println("Version: " + String(VERSION));
}

void loop() {
    vTaskDelay(10000 / portTICK_PERIOD_MS);
}

/**
 * @brief 提货动作
 */
void get0ok(){
  Serial.println("[get0ok]");
  movepose(0, 0, 1);
  
  ledcWrite( 3, angleToDuty(PWM_3_helf));
  GotoPose(X_tihuo_P, 0, 0 , true, false);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  ledcWrite( 3, angleToDuty(PWM_3_close));
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  GotoHeight(350);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  GotoPose(-X_tihuo_N, 0, 0 , true, false);
  GotoHeight(550);
  GotoPose(0, 20, 0 , true, false);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  ledcWrite( 2, angleToDuty(185));
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  ledcWrite( 3, angleToDuty(PWM_3_helf));
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  ledcWrite( 2, angleToDuty(60));
  ledcWrite( 3, angleToDuty(PWM_3_open));
  vTaskDelay(1000 / portTICK_PERIOD_MS); 
  GotoHeight(315);
  GotoPose(X_tihuo_N - X_tihuo_P, 0, 0 , true, false);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  deliverDone++;
  movepose(1, 10,0);
  Serial.println("[get0done]");
}

/**
 * @brief 提货逻辑的底层消息中心
 * @details 替代原有 while_get0 内的轮询逻辑，全面改为事件驱动
 */
void while_get0(){
  VisualCmdType receivedCmd;
  
  while(avg_distances[0] < 1600 ){
    // 核心改动：非阻塞式读取队列（等待时间 10ms，等同于原有的循环延时）
    if (xQueueReceive(xVisualTaskQueue, &receivedCmd, pdMS_TO_TICKS(10)) == pdPASS) {
        
        // 1. 处理直接提货请求 (get0)
        if (receivedCmd == VISUAL_CMD_GET0) {
            get0ok();
        }
        
        // 2. 处理网络联网寻找逻辑 (search)
        else if (receivedCmd == VISUAL_CMD_SEARCH) {
            Serial.println("search->");
            movepose(0, 0, 1); // 先停下
            
            TickType_t searchStartTime = xTaskGetTickCount();
            const TickType_t timeoutTicks = pdMS_TO_TICKS(10000); // 10秒网络超时
            bool exitSearchLoop = false;

            // 开启内部网络数据监听阻塞循环
            while ((xTaskGetTickCount() - searchStartTime) < timeoutTicks) {
                VisualCmdType subCmd;
                // 持续阻塞读取队列新进来的二级命令
                if (xQueueReceive(xVisualTaskQueue, &subCmd, pdMS_TO_TICKS(50)) == pdPASS) {
                    if (subCmd == VISUAL_CMD_NEXT) {
                        Serial.println("next->");
                        movepose(1, Speed_tihuo, 0); // 继续恢复移动
                        exitSearchLoop = true;
                        break;
                    } 
                    else if (subCmd == VISUAL_CMD_NETGET) {
                        Serial.println("netget->");
                        get0ok();
                        exitSearchLoop = true;
                        break;
                    }
                }
            }
            
            if(!exitSearchLoop) {
                Serial.println("search timeout->");
                movepose(1, Speed_tihuo, 0); // 超时保护：恢复原移动
            }
        }
    }
  }
}
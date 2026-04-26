/**********************************************************
*** 超市机器人控制程序
*** 编写作者：LZF
*** 技术支持：GEMINI
**********************************************************/

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <FastLED.h>
#include "ota_service.h"

#include "Emm_V5.h"
#include "lidar.h"
#include "pwm.h"

#define MODE_key 0
#define LED_PIN 48
#define NUM_LEDS 1
#define OTA_HOSTNAME "espchaoshi"
#define VERSION "1.3.0"

//四个待补货商品位置（二维数组），{ x坐标, y坐标, theta角度, 是否完成}
float buhuo[4][4] = {
    {2230, 1300, 0, 0}, // 商品1 锐澳水蜜桃 
    {2230, 1480, 0, 0}, // 商品2 百事可乐
    {900, 1660, 180, 0}, // 商品3 旺仔牛奶
    {900, 1300, 180, 0}  // 商品4 维他奶
};
int getBuhuoIndex = 0;//当前需要补货的商品索引

// LED数组
CRGB leds[NUM_LEDS];

//全局变量
RobotPose currentPose = {0, 0, 0};//当前理想机器人位置,中心坐标，(x,y,theta),mm,mm,度(0-360)
//RobotAngle servoPose = {0, 0, 0, 0, 0};//当前大臂高度，舵机角度,mm(0-1000),度(0-360)
bool isdebug = false;//是否调试模式
bool ready = false;//是否准备好运行
int DX_dist = 0;//补货/提货的视觉X轴偏差

// 机器人舵机角度结构体
struct RobotAngle {
    float height;//大臂高度,mm(0-1000)
    float angle1;//通道1，
    float angle2;//通道2
    float angle3;//通道3
    float angle4;//通道4
};

//位置坐标-电机脉冲转换系数（mm*pulse = 走对应距离对应脉冲数）
 float X_PULSE = 10.5f;
 float Y_PULSE = 11.4f;
 float THETA_PULSE = 84.0f;
 float HEIGHT_PULSE = 32.26f;
 float DX_PULSE = 0.1f;

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
    STATE_READ_ORDER,     // 前往小方桌识别购物需求 
    STATE_PRE_REPLENISH,  // 前往货架1/2的第一层抓取补货物品
    STATE_DO_REPLENISH,   // 前往第三层完成补货任务 
    STATE_GO_SHOPPING,    // 前往第二层寻找清单上的物品 
    STATE_DELIVERING,     // 前往提货区投递 
    STATE_FINISH_HOME     // 返回终点区并结束比赛 
};

  // 购物清单与任务记录（具体结构根据实际需求补充）
  bool isOrderReceived = false;//是否拍到购物需求
  int itemsPicked = 0;//已抓取物品数量
  bool isinorder = false;//是否在清单上的物品
  int replenishDone = 0;//已补货物品数量
  int deliverDone = 0;//已提货物品数量
  bool isCustomer = false;  //是否识别到顾客

// 主状态机任务函数（正常运行模式）
void Task_MainStateMachine(void *pvParameters) {
    int buhuoNOW = 0;//当前爪子上面的补货商品索引
    RobotPose lastpose = {0, 0, 0};
    bool isReplenishDone_1 = false;//是否在货架1补货完成
    bool isReplenishDone_2 = false;//是否在货架2补货完成

    int start_time = 0;//开始时间，用于计算是否超时

    int caowei[2][3][7] = { //货架号，层号，槽位号
    { // 货架0
        {0, 0, 0, 0, 0, 0, 0}, // 层0的7个槽位
        {0, 0, 0, 0, 0, 0, 0}, // 层1的7个槽位
        {0, 0, 0, 0, 0, 0, 0}  // 层2的7个槽位
    },
    { // 货架1
        {0, 0, 0, 0, 0, 0, 0}, // 层0的7个槽位
        {0, 0, 0, 0, 0, 0, 0}, // 层1的7个槽位
        {0, 0, 0, 0, 0, 0, 0}  // 层2的7个槽位
    }
    };

    RobotState currentState = STATE_INIT_WAIT;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while(!ready){
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    start_time = millis();
    Serial.println("start"); 
    while (1) {
        switch (currentState) {
/**************************************************************/
//               启动后的10秒强制静止阶段 
/**************************************************************/
            case STATE_INIT_WAIT:
                // 1. 强制静止10秒
                vTaskDelay(10000 / portTICK_PERIOD_MS);//点一
                currentState = STATE_READ_ORDER;
                break;
/**************************************************************/
//               前往小方桌识别购物需求 
/**************************************************************/
            case STATE_READ_ORDER://前往小方桌识别购物需求
                ledcWrite( 1, angleToDuty(40));//图像大臂完全抬起
                vTaskDelay(100 / portTICK_PERIOD_MS);
                Serial.println("taiqi");
                GotoPose(1100, 0, 0, true, false);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                Serial.println("[0]");//开启上位机的OCR功能
                vTaskDelay(200 / portTICK_PERIOD_MS);
                for (int i = 0; i < 30; i++) {//30秒超时，未收到到购物需求，则先前往货架1/2的第一层抓取补货物品
                    vTaskDelay(1000 / portTICK_PERIOD_MS); 
                    Serial.println("i="+String(i));
                    if (isOrderReceived) {
                        break;
                    }
                }
                GotoPose(880, 0, 0 , true, false);
                ledcWrite( 1, angleToDuty(250));//图像大臂放下
                Serial.println("[2]");//开启上位机的yolo(补货)功能
                GotoPose(0, 650, 0 , true, false);//开始采集
                GotoHeight(500);
                AdjustPose();
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                ledcWrite( 2, angleToDuty(60));//夹臂完全打开
                vTaskDelay(500 / portTICK_PERIOD_MS);
                ledcWrite( 3, angleToDuty(0));//夹爪完全打开
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                GotoHeight(0);
                vTaskDelay(2000 / portTICK_PERIOD_MS);

                lastpose = currentPose;
                
               currentState = STATE_PRE_REPLENISH;
               break;
/**************************************************************/
//               前往货架1/2的第一层抓取补货物品
/**************************************************************/
            case STATE_PRE_REPLENISH://前往货架1/2的第一层抓取补货物品
                // 从货架1和2的第一层抓取待补货物品：锐澳、百事、旺仔、维他奶 
                // 遍历货架1/2 第一层

                Serial.println("into replenish" + String(lastpose.x) + String(lastpose.y) + String(lastpose.theta));

                GotoHeight(0);
                vTaskDelay(3000 / portTICK_PERIOD_MS);

                if(!isReplenishDone_1) {//货架1 的补货未完成
                  if(currentPose.theta == 0){
                    Serial.println("go to one"+String(lastpose.y));
                    GotoPose(2230, lastpose.y, 0 , false, false);
                  } else{//机器人与补货槽位货架不在同侧，先移动旋转到补货槽位货架
                    GotoPose(1600, 1300, 0 , false, false);
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    AdjustPose();
                    GotoPose(2230, lastpose.y, 0 , false, false);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                  }
                } else if(!isReplenishDone_2) {//货架1 的补货已完成，前往货架2补货
                    Serial.println("go to two"+String(lastpose.y));
                  if(currentPose.theta == 180){
                    GotoPose(880, lastpose.y, 180 , false, false);
                  } else {//机器人与补货槽位货架不在同侧，先移动旋转到补货槽位货架
                    GotoPose(1600, 1300, 180 , false, false);
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    AdjustPose();
                    GotoPose(880, lastpose.y, 180 , false, false); 
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                  }
                } else {//补全部完成
                  Serial.println("[buhuodone]");//告诉上位机确认补货完成
                  currentState = STATE_GO_SHOPPING;
                  break;
                }


                getBuhuoIndex = 0;//先清零;

                if(!isReplenishDone_1) {//货架1 的补货未完成

                  movepose(1, 10,0);
                  while(avg_distances[0] < 1500){
                    getBuhuoIndex = 0;//先清零;
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    if(getBuhuoIndex != 0) {
                        movepose(0, 0, 1);
                        lastpose = currentPose;
                        Serial.println("get buhuo"+String(getBuhuoIndex));
                        buhuoNOW = getBuhuoIndex-1;//记录当前爪子上面的补货商品索引
                        //抓取动作
                        ledcWrite( 3, angleToDuty(120));
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoPose(100, 0, 0 , true, false);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(190));//夹爪闭合
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoHeight(50);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoPose(-100, 0, 0 , true, false);
                        currentState = STATE_DO_REPLENISH;
                        break;
                    }
                  }

                  if(avg_distances[0] > 1450){//被动跳出的循环，货架遍历完成
                    movepose(0, 0, 1);
                    isReplenishDone_1 = true;
                    break;//查看完毕，前往货架2补货                  
                  } else {
                    break;//由getBuhuoIndex跳出的逻辑，在抓取中已经停止了
                  }


                } else if(!isReplenishDone_2) {//货架1 的补货已完成，前往货架2补货
                  movepose(1, 10,0);
                  while(avg_distances[0] < 1500){
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    if(getBuhuoIndex != 0) {
                        movepose(0, 0, 1);
                        Serial.println("get buhuo"+String(getBuhuoIndex));
                        buhuoNOW = getBuhuoIndex-1;//记录当前爪子上面的补货商品索引
                        //抓取动作
                        ledcWrite( 3, angleToDuty(120));
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoPose(100, 0, 0 , true, false);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        ledcWrite( 3, angleToDuty(190));//夹爪闭合
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoHeight(50);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        GotoPose(-100, 0, 0 , true, false);
                        currentState = STATE_DO_REPLENISH;
                        break;
                    }
                  }
                  
                  if(avg_distances[0] > 1450){//被动跳出的循环，货架遍历完成
                    movepose(0, 0, 1);
                    isReplenishDone_2 = true;
                    break;//全部查看完毕，补货完成
                  } else {
                    break;//由getBuhuoIndex跳出的逻辑，在抓取中已经停止了
                  }

                } else {//补货全部完成
                  currentState = STATE_GO_SHOPPING;
                  Serial.println("buhuodone");
                  break;
                }

                break;//正常不会执行到这行代码
/**************************************************************/
//               执行补货动作
/**************************************************************/
            case STATE_DO_REPLENISH://执行补货动作
                // 将抓到的补货物品放入第三层标签标记的指定位置 
                // 执行放置动作 

                //前往补货槽位
                Serial.println("go to buhuo"+String(buhuoNOW));
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                //如果机器人与补货槽位货架不在同侧，先移动旋转到补货槽位货架
                if(buhuo[buhuoNOW][2] != currentPose.theta) {
                  GotoPose(1600, 1300, buhuo[buhuoNOW][2], false, false);//先去场地中央转弯
                  vTaskDelay(1000 / portTICK_PERIOD_MS);
                  AdjustPose();
                  vTaskDelay(1000 / portTICK_PERIOD_MS);
                  GotoPose(buhuo[buhuoNOW][0], buhuo[buhuoNOW][1], buhuo[buhuoNOW][2], false, false);
                  GotoHeight(640);
                  vTaskDelay(4000 / portTICK_PERIOD_MS);
                  GotoPose(180, 0, 0 , true, false);
                  GotoHeight(600);
                  ledcWrite( 3, angleToDuty(120));//夹爪半开
                  vTaskDelay(500 / portTICK_PERIOD_MS);
                  GotoPose(-180, 0, 0 , true, false);
                  ledcWrite( 3, angleToDuty(0));//夹爪大开  
                  GotoHeight(0);
                } else{
                  GotoPose(buhuo[buhuoNOW][0], buhuo[buhuoNOW][1], buhuo[buhuoNOW][2], false, false);
                  GotoHeight(640);
                  vTaskDelay(4000 / portTICK_PERIOD_MS);
                  GotoPose(180, 0, 0 , true, false);
                  GotoHeight(600);
                  ledcWrite( 3, angleToDuty(120));//夹爪半开
                  vTaskDelay(500 / portTICK_PERIOD_MS);
                  GotoPose(-180, 0, 0 , true, false);
                  ledcWrite( 3, angleToDuty(0));//夹爪大开
                  GotoHeight(0);
                }

                replenishDone++;

                //如果所有商品都完成补货，则前往第二层寻找清单物品，否则继续补货
                if((isReplenishDone_1 && isReplenishDone_2) || replenishDone == 4) {//两货架标志位均完成，或补货完成4个商品，补货逻辑完成
                    currentState = STATE_GO_SHOPPING;
                    Serial.println("[buhuodone]");//告诉上位机确认补货完成
                }
                else {
                    currentState = STATE_PRE_REPLENISH;//继续补货，前往补货槽位
                }
                break;
/**************************************************************/
//               前往第二层寻找6个清单物品
/**************************************************************/
            case STATE_GO_SHOPPING://前往第二层寻找6个清单物品
                // 前往第二层寻找6个清单物品
                // 上位机识别->收到"get0"->执行抓取动作

                Serial.println("go to tihuo");
                //Serial.println("[3]");//开启上位机的提货（yolo）功能

                GotoHeight(300);

                //补货完成后再货架二，就近在二开始提货
                GotoPose(880, 1630, 180 , false, false);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                AdjustPose();
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                //开始货架二遍历

                movepose(1, 10,0);//开始移动
                while(avg_distances[0] < 1500 && deliverDone < 6){
                  vTaskDelay(100 / portTICK_PERIOD_MS);
                  if(isinorder != 0 ) {
                      movepose(0, 0, 1);//停下
                      Serial.println("get tihuo: " + String(isinorder));
                      //抓取动作，放置在临时货斗
                      ledcWrite( 3, angleToDuty(120));
                      GotoPose(130, 0, 0 , true, false);
                      vTaskDelay(1000 / portTICK_PERIOD_MS);
                      ledcWrite( 3, angleToDuty(190));//夹爪闭合
                      vTaskDelay(1000 / portTICK_PERIOD_MS);
                      GotoHeight(350);
                      vTaskDelay(1000 / portTICK_PERIOD_MS);
                      GotoPose(-130, 0, 0 , true, false);
                      GotoHeight(550);
                      vTaskDelay(3000 / portTICK_PERIOD_MS);
                      ledcWrite( 2, angleToDuty(180));
                      vTaskDelay(2000 / portTICK_PERIOD_MS);
                      ledcWrite( 3, angleToDuty(120));
                      vTaskDelay(1000 / portTICK_PERIOD_MS);
                      ledcWrite( 2, angleToDuty(50));
                      ledcWrite( 3, angleToDuty(0));//夹爪大开
                      GotoHeight(300);
                      vTaskDelay(1000 / portTICK_PERIOD_MS);
                      isinorder = 0;
                      deliverDone++;
                      Serial.println("[getok]");//告诉上位机成功提货一个物品
                      movepose(1, 10,0);//继续向前开
                  }
                }

                    
                //从中间过去，随便校准
                GotoPose(1600, 1300, 0 , false, false);
                vTaskDelay(3000 / portTICK_PERIOD_MS);
                AdjustPose();
                GotoPose(2230, 1030, 0 , false, false); 
                vTaskDelay(1000 / portTICK_PERIOD_MS);


                //开始货架一遍历
                movepose(1, 10,0);//开始移动
                while(avg_distances[0] < 1500){
                  vTaskDelay(100 / portTICK_PERIOD_MS);
                  if(isinorder != 0 && deliverDone < 6) {
                      movepose(0, 0, 1);//停下
                      Serial.println("get tihuo: " + String(isinorder));
                      //抓取动作，放置在临时货斗
                      ledcWrite( 3, angleToDuty(120));
                      GotoPose(130, 0, 0 , true, false);
                      vTaskDelay(1000 / portTICK_PERIOD_MS);
                      ledcWrite( 3, angleToDuty(190));//夹爪闭合
                      vTaskDelay(1000 / portTICK_PERIOD_MS);
                      GotoHeight(350);
                      vTaskDelay(1000 / portTICK_PERIOD_MS);
                      GotoPose(-130, 0, 0 , true, false);
                      GotoHeight(550);
                      vTaskDelay(3000 / portTICK_PERIOD_MS);
                      ledcWrite( 2, angleToDuty(180));
                      vTaskDelay(2000 / portTICK_PERIOD_MS);
                      ledcWrite( 3, angleToDuty(120));
                      vTaskDelay(1000 / portTICK_PERIOD_MS);
                      ledcWrite( 2, angleToDuty(50));
                      ledcWrite( 3, angleToDuty(0));//夹爪大开
                      GotoHeight(300);
                      vTaskDelay(1000 / portTICK_PERIOD_MS);
                      isinorder = 0;
                      deliverDone++;
                      Serial.println("[getok]");//告诉上位机成功提货一个物品
                      movepose(1, 10,0);//继续向前开
                  }
                }

                //打印完成情况
                Serial.println("finish shopping: " + String(deliverDone));
                Serial.println("[tihuodone]");//告诉上位机确认提货完成
                currentState = STATE_DELIVERING;
                break;
/**************************************************************/
//               执行交付动作
/**************************************************************/
            case STATE_DELIVERING:// 前往提货区，识别头像匹配目标顾客 
                Serial.println("[1]");//上位机交付功能
                GotoHeight(630);
                GotoPose(1150, 2100, 90 , false, false);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                AdjustPose();
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                GotoPose(200, 200, 0 , true, false);
                for(int i = 0; i < 4; i++) {//4个顾客
                  vTaskDelay(3000 / portTICK_PERIOD_MS);//等待1秒，确保上位机识别完成
                  if(isCustomer) {
                    //倒料
                    Serial.println("dump");
                    GotoPose(0, -100, 0 , true, false);
                    ledcWrite( 4, angleToDuty(200));
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    ledcWrite( 5, angleToDuty(90));
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    break;//倒料完成后，跳出循环
                  }
                  //向下个位置移动
                  GotoPose(0, -300, 0 , true, false);
                }
                currentState = STATE_FINISH_HOME;
                break;
/**************************************************************/
//                  执行返回终点区动作
/**************************************************************/
            case STATE_FINISH_HOME://须在8分钟内完全进入终点区

                ledcWrite( 3, angleToDuty(180));//闭合
                GotoHeight(520);
                ledcWrite( 4, angleToDuty(0));
                ledcWrite( 5, angleToDuty(240));
                ledcWrite( 2, angleToDuty(300));//夹臂完全收回
                vTaskDelay(3000 / portTICK_PERIOD_MS);
                GotoHeight(0);

                GotoPose(0, 0, 0 , false, false);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                GotoPose(2300, 2500, 0 , false, false);
                AdjustPose();
                GotoPose(200, 500, 0 , true, false);
                vTaskDelete(NULL); 
                break;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if(millis() - start_time > 450000) {//8分钟超时,预留30秒
          Serial.println("timeout");
          currentState = STATE_FINISH_HOME;
        }
    }
}

// 主任务串口通信函数（主循环）
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
                    if (strstr(rxBuffer, "ready") != 0) {
                      // 视觉初始化完成逻辑
                        Serial.println("ready");      
                        ready = true;
                    } else if (strcmp(rxBuffer, "orderget") == 0) {
                       // 确认提货订单商品
                        isOrderReceived = true;
                        Serial.println("orderget");
                    } else if (strcmp(rxBuffer, "get1") == 0) {
                        // 识别到锐澳
                        getBuhuoIndex = 1;
                        Serial.println("get1");
                    } else if (strcmp(rxBuffer, "get2") == 0) {
                        // 识别到百事
                        getBuhuoIndex = 2;
                        Serial.println("get2");
                    } else if (strcmp(rxBuffer, "get3") == 0) {
                        // 识别到旺仔
                        getBuhuoIndex = 3;
                        Serial.println("get3");
                    } else if (strcmp(rxBuffer, "get4") == 0) {
                        // 识别到维他奶
                        getBuhuoIndex = 4;
                        Serial.println("get4");
                    } else if (strcmp(rxBuffer, "get0") == 0) {
                        // 识别到提货商品
                        isinorder = true;
                        Serial.println("get0");
                    } else if (strcmp(rxBuffer, "true") == 0) {
                        // 识别到顾客
                        isCustomer = true;
                        Serial.println("true");
                    } else if (strstr(rxBuffer, "[") != NULL && strstr(rxBuffer, "]") != NULL) {
                        // 解析[dx]格式的命令
                        int dx_value;
                        if (sscanf(rxBuffer, "[%d]", &dx_value) == 1) {
                            DX_dist = dx_value;
                            //Serial.print("DX_dist set to: ");
                            Serial.println(DX_dist);
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

      }else if(strcmp(cmd.cmd, "movepose") == 0){
        // 执行movepose命令,移动机器人到指定位置
        Serial.print("Executing movepose: Y=");
        Serial.print(cmd.param1);
        Serial.print(", speed=");
        Serial.print(cmd.param2);
        Serial.print(", stop=");
        Serial.println(cmd.param3);
        // 使用movepose函数移动机器人到指定位置
        movepose(cmd.param1, cmd.param2, cmd.param3);


      } else if(strcmp(cmd.cmd, "EMMpos") == 0){
        // 执行EMMpos命令,设置电机位置
        Serial.print("Executing EMMpos: addr=");
        Serial.print(cmd.param1);
        Serial.print(", dir=");
        Serial.print(cmd.param2);
        Serial.print(", clk=");
        Serial.println(cmd.param3);
        Emm_V5_Pos_Control( cmd.param1, cmd.param2, 1000, 200, cmd.param3, 0, 0);

      }else if(strcmp(cmd.cmd, "GOTOHeight") == 0){
        // 执行GOTOHeight命令,设置机器人高度
        Serial.print("Executing GOTOHeight: height=");
        Serial.println(cmd.param1);
        // 使用GotoHeight函数设置机器人高度
        GotoHeight(cmd.param1);

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
        Serial.println("GETCpose/GETRpose");
        Serial.println("GETdist");
        Serial.println("movepose Y speed stop");
        Serial.println("AdjustPose");
        Serial.println("EMMpos addr dir clk");
        Serial.println("GOTOHeight height");
        Serial.println("PWM addr angle");
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
//                      初始化设置
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
  // 初始化FastLED
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(10);  // 设置亮度为50%

  currentPose = {250, 400, 0};//初始化机器人中心位置为(250mm,400mm,0)

  init_ota_service("null123", "1234567899", OTA_HOSTNAME);
  
  // WiFi连接成功后设为红色
  leds[0] = CRGB::Red;
  FastLED.show();


  // 初始化时延时十秒，监测boot按键
  pinMode(MODE_key, INPUT_PULLUP);  // 设置MODE_key为输入模式，启用上拉电阻
  //Serial.println("Waiting for 10 seconds, press MODE_key to enter debug mode...");
  
  unsigned long startTime = millis();
  bool bootKeyPressed = false;
  
  while (millis() - startTime < 5000) {  // 延时5秒
    if (digitalRead(MODE_key) == LOW) {  // 检测按键是否按下（低电平）
      bootKeyPressed = true;
      Serial.println("MODE_key pressed, entering debug mode...");
      break;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  // 短暂延时，避免占用过多CPU资源
  }
  
  if (bootKeyPressed) {
    isdebug = true;
  }

  //开启程序
  if (isdebug) {
    Serial.println("Debug mode ");//调试模式 开启对应任务
    // 调试模式设为黄色
    leds[0] = CRGB::Yellow;
    FastLED.show();
    xDebugQueue = xQueueCreate(20, sizeof(DebugCommand_t));  // 初始化调试队列
    if (xDebugQueue == NULL) { Serial.println("Failed to create debug queue");}
    xTaskCreate(Task_Debug_Mode, "Task_Debug_Mode", 16384, NULL, 5, NULL);
    xTaskCreate(Task_Debug_Serial0_CMD, "Task_Debug_Serial0_CMD", 16384, NULL, 5, NULL);
  } else {
    Serial.println("Release mode ");//正常运行 开启对应任务
    // 正常模式设为绿色
    leds[0] = CRGB::Green;
    FastLED.show();
    xTaskCreate(Task_MainStateMachine, "Task_MainStateMachine", 16384, NULL, 5, NULL);
    xTaskCreate(Task_Main_Serial0_CMD, "Task_Main_Serial0_CMD", 16384, NULL, 5, NULL);
  }

  Serial.println("Supermarket robot initialized");
  Serial.println("Version: " + String(VERSION));

}

void loop() {

vTaskDelay(10000 / portTICK_PERIOD_MS);
                    
}
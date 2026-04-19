#ifndef LIDAR_H
#define LIDAR_H

#include <Arduino.h>

// 引脚定义
#define SERIAL2_TXD_PIN 16
#define SERIAL2_RXD_PIN 15
#define CD4052_A 1
#define CD4052_B 2

// 机器人坐标结构体
struct RobotPose {
    float x;
    float y;
    float theta; // 航向角
};

// 位置坐标-电机脉冲转换系数（mm-脉冲）
extern float X_PULSE;
extern float Y_PULSE;
extern float THETA_PULSE;
extern float HEIGHT_PULSE;

extern bool BetweenShelves;//是否在货架货架之间
extern RobotPose currentPose;//当前机器人位置,中心坐标，(x,y,theta),mm,mm,度(0-360)


// 存储4个通道的平均距离（全局变量，用于GETdist命令）
extern int avg_distances[4];


// 场地常数定义 (单位: mm)
const int FIELD_X_MAX = 3100; // 场地最大X坐标
const int FIELD_Y_MAX = 2600; // 场地最大Y坐标
const int SHELF_WIDTH = 500;  // 货架宽度 
const int SHELF_LENGTH = 1000; // 货架长度 
//机器人常数定义 (单位: mm)
const int ROBOT_WIDTH = 390; // 机器人宽度（X,mm）
const int ROBOT_LENGTH = 610; // 机器人长度（Y,mm）   
const int LIDAR_W_1_2 = 335; //ch1-ch2宽度(mm)


#pragma pack(push, 1)
typedef struct {
    int16_t distance;   
    uint16_t noise;     
    uint32_t peak;      
    uint8_t confidence; 
    uint32_t intg;      
    int16_t reftof;     
} LidarPointTypedef;

typedef struct {
    LidarPointTypedef points[12]; 
    uint32_t timestamp;           
} LidarDataPayload;
#pragma pack(pop)

#define FRAME_LENGTH 195

// 声明 FreeRTOS 任务句柄
extern TaskHandle_t TaskLidarHandle;

// 函数声明
void TaskLidarProcess(void *pvParameters);// 处理雷达初始数据的任务
void initLidar();// 初始化雷达
RobotPose GETRPose(int dists[4]);// 计算机器人坐标
void GotoPose(float x, float y, float theta,bool isRelative,bool isAdjust);// 移动机器人到指定位置
bool AdjustPose();// 位置微调函数
void GotoHeight(float height);// 移动机器人大臂到指定高度


#endif // LIDAR_H
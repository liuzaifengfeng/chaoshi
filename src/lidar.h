#ifndef LIDAR_H
#define LIDAR_H

#include <Arduino.h>

// 引脚定义
#define SERIAL2_TXD_PIN 16
#define SERIAL2_RXD_PIN 15
#define CD4052_A 1
#define CD4052_B 2

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
void TaskLidarProcess(void *pvParameters);
void initLidar();

#endif // LIDAR_H
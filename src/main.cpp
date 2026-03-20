/**********************************************************
*** 超市机器人控制程序
*** 编写作者：LZF
*** 技术支持：GEMINI
**********************************************************/

#include <Arduino.h>
#include "Emm_V5.h"

#define SERIAL1_TXD_PIN 18
#define SERIAL1_RXD_PIN 17
#define SERIAL2_TXD_PIN 16
#define SERIAL2_RXD_PIN 15
#define PWM1_PIN 4
#define PWM2_PIN 5
#define PWM3_PIN 6
#define PWM4_PIN 7
#define CD4052_A 1
#define CD4052_B 2

// 定义接收数据数组、接收数据长度
static uint8_t rxCmd[128] = {0}; 
static uint8_t rxCount = 0;

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
TaskHandle_t TaskLidarHandle = NULL;

void TaskLidarProcess(void *pvParameters) {
    uint8_t rx_buffer[FRAME_LENGTH];
    int rx_index = 0;

    for (;;) {
        // 只要串口1有数据，就一直接收
        while (Serial2.available()) {
            rx_buffer[rx_index++] = Serial2.read();

            // 动态滑窗寻找包头 0xAAAAAAAA
            if (rx_index >= 4) {
                if (rx_buffer[0] != 0xAA || rx_buffer[1] != 0xAA || 
                    rx_buffer[2] != 0xAA || rx_buffer[3] != 0xAA) {
                    for (int i = 0; i < rx_index - 1; i++) {
                        rx_buffer[i] = rx_buffer[i + 1];
                    }
                    rx_index--;
                }
            }

            // 成功收集齐一帧
            if (rx_index == FRAME_LENGTH) {
                // 校验命令码是否为 0x02 获取数据
                if (rx_buffer[5] == 0x02) {
                    uint8_t checksum = 0;
                    for (int i = 4; i < FRAME_LENGTH - 1; i++) {
                        checksum += rx_buffer[i];
                    }
                    
                    // 校验通过，提取数据
                    if (checksum == rx_buffer[FRAME_LENGTH - 1]) {
                        LidarDataPayload* payload = (LidarDataPayload*)(&rx_buffer[10]);
                        
                        // 为了避免疯狂刷屏，这里只打印正前方的第一个点用于测试
                        if(payload->points[0].confidence > 50) {
                             Serial.printf("[LiDAR Task] Point 1 Dist: %d mm\n", payload->points[0].distance);
                        }
                        
                        // TODO: 在这里可以通过 FreeRTOS Queue (队列) 将距离数据发送给主循环的避障逻辑
                    }
                }
                rx_index = 0; // 重置索引，准备接收下一帧
            }
        }
        
        // 必须加入延时，交出 CPU 控制权，防止 Core 0 触发看门狗复位 (WDT Reset)
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

void setup() {


  // 初始化PWM引脚
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  pinMode(PWM3_PIN, OUTPUT);
  pinMode(PWM4_PIN, OUTPUT);

  // 初始化CD4052引脚
  pinMode(CD4052_A, OUTPUT);
  pinMode(CD4052_B, OUTPUT);
  digitalWrite(CD4052_A, LOW);
  digitalWrite(CD4052_B, LOW);

  // 初始化串口
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_TXD_PIN, SERIAL1_RXD_PIN);
  Serial2.begin(230400, SERIAL_8N1, SERIAL2_TXD_PIN, SERIAL2_RXD_PIN);

  xTaskCreate(TaskLidarProcess, "LidarProcess", 8192, NULL, 2, &TaskLidarHandle);

  Serial.println("Supermarket robot initialized");
  Serial2.println("Start LidarProcess Task");
  

  // 上电延时2秒等待Emm_V5.0闭环初始化完毕
  delay(1000);
}

void loop() {
  /*
  //00
  digitalWrite(CD4052_A, HIGH);//10
  delay(2000);
  digitalWrite(CD4052_B, HIGH);//11
  delay(2000);
  digitalWrite(CD4052_A, LOW);//01
  delay(2000);
  digitalWrite(CD4052_B, LOW);//00
  delay(2000);
*/

}


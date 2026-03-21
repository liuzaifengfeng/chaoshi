/**********************************************************
*** 超市机器人控制程序
*** 编写作者：LZF
*** 技术支持：GEMINI
**********************************************************/

#include <Arduino.h>
#include "Emm_V5.h"

// 引脚定义
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

// --------------------------------------------------------
// 任务：雷达数据轮询与解析 (运行在 Core 0)
// --------------------------------------------------------
void TaskLidarProcess(void *pvParameters) {
    uint8_t rx_buffer[FRAME_LENGTH];
    int avg_distances[4] = {0, 0, 0, 0}; // 存储4个通道的平均距离

    // 用于精准控制 2Hz (500ms) 的任务周期
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500); 

    for (;;) {
        // 遍历 0 到 3 号通道
        for (int ch = 0; ch < 4; ch++) {
            
            // 1. 控制 CD4052 切换通道 (A为低位，B为高位)
            digitalWrite(CD4052_A, (ch & 0x01) ? HIGH : LOW);
            digitalWrite(CD4052_B, (ch & 0x02) ? HIGH : LOW);

            // 2. 给硬件多路复用器电平切换时间，同时让传感器发送几帧新数据 (雷达120Hz，约8ms一帧)
            vTaskDelay(pdMS_TO_TICKS(20));

            // 3. 核心避坑：清空串口缓冲区，丢弃切换瞬间产生的乱码和上个通道的残余数据
            while(Serial2.available()) {
                Serial2.read();
            }

            // 4. 接收当前通道的一帧数据
            int rx_index = 0;
            bool frame_received = false;
            uint32_t start_time = millis();

            // 设置200ms超时机制 (避免某个雷达掉线导致整个系统卡死)
            while (millis() - start_time < 100 && !frame_received) {
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
                        // 校验命令码是否为 0x02
                        if (rx_buffer[5] == 0x02) {
                            uint8_t checksum = 0;
                            for (int i = 4; i < FRAME_LENGTH - 1; i++) {
                                checksum += rx_buffer[i];
                            }
                            
                            // 校验通过
                            if (checksum == rx_buffer[FRAME_LENGTH - 1]) {
                                LidarDataPayload* payload = (LidarDataPayload*)(&rx_buffer[10]);
                                
                                // 提取12个点做平均
                                int32_t sum_dist = 0;
                                int valid_count = 0;
                                
                                for (int p = 0; p < 12; p++) {
                                    // 过滤掉置信度极低或测量错误(距离为0)的噪点
                                    if (payload->points[p].confidence > 50 && payload->points[p].distance > 0) {
                                        sum_dist += payload->points[p].distance;
                                        valid_count++;
                                    }
                                }
                                
                                if (valid_count > 0) {
                                    avg_distances[ch] = sum_dist / valid_count;
                                } else {
                                    avg_distances[ch] = 0; // 无有效数据
                                }
                                
                                frame_received = true; // 标记成功接收并打断 while
                                break;
                            }
                        }
                        rx_index = 0; // 如果校验失败，清空索引重新找包头
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(1)); // 喂狗，避免死锁
            }
            
            // 如果超时未收到数据，赋值为 -1 报警
            if (!frame_received) {
                avg_distances[ch] = -1;
            }
        }

        // 5. 4个通道采集完毕，向串口0上报最终汇总结果
        Serial.printf("[LiDAR Map] CH0: %4d mm | CH1: %4d mm | CH2: %4d mm | CH3: %4d mm\n", 
                      avg_distances[0], avg_distances[1], avg_distances[2], avg_distances[3]);

        // 6. FreeRTOS 绝对延时，补齐剩余时间，确保整个大循环精准为 1000 毫秒 (1Hz)
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

// --------------------------------------------------------
// 初始化设置
// --------------------------------------------------------
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

  // 创建雷达任务 (分配 8192 字节内存，运行在 Core 0)
  xTaskCreatePinnedToCore(TaskLidarProcess, "LidarProcess", 8192, NULL, 2, &TaskLidarHandle, 0);

  Serial.println("Supermarket robot initialized");
  Serial.println("Start Lidar 4-Channel Polling Task...");

  // 上电延时等待初始化完毕
  delay(1000);
}

// --------------------------------------------------------
// 主循环 (Core 1)
// --------------------------------------------------------
void loop() {
    // 你的电机闭环控制逻辑可以写在这里
    // 雷达任务已经在后台自动帮你把四周的距离刷好了

    delay(1000);
}
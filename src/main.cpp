#include <Arduino.h>

// --------------------------------------------------------
// 1. 定义数据结构 (强制 1 字节对齐，防止内存错位)
// --------------------------------------------------------
#pragma pack(push, 1)
typedef struct {
    int16_t distance;   // 距离数据 (mm)
    uint16_t noise;     // 环境噪声
    uint32_t peak;      // 接收强度
    uint8_t confidence; // 置信度 (0-100)
    uint32_t intg;      // 积分次数
    int16_t reftof;     // 温度表征值
} LidarPointTypedef;

typedef struct {
    LidarPointTypedef points[12]; // 1帧包含12个测量点
    uint32_t timestamp;           // 时间戳
} LidarDataPayload;
#pragma pack(pop)

// --------------------------------------------------------
// 2. 硬件与变量配置
// --------------------------------------------------------
// ESP32-S3 串口1引脚定义 (你可以根据实际接线修改)
#define RX1_PIN 18 
#define TX1_PIN 17
#define LED_PIN 2

#define FRAME_LENGTH 195         // 完整一帧的长度
uint8_t rx_buffer[FRAME_LENGTH]; // 接收缓冲区
int rx_index = 0;

// --------------------------------------------------------
// 3. 解析与上报函数
// --------------------------------------------------------
void parseRadarFrame() {
    // 校验命令码是否为获取测量数据 (0x02)
    if (rx_buffer[5] != 0x02) return;

    // 计算校验和：除去数据协议头(前4字节)后的数据校验和
    uint8_t checksum = 0;
    for (int i = 4; i < FRAME_LENGTH - 1; i++) {
        checksum += rx_buffer[i];
    }
    
    // 校验对比
    if (checksum != rx_buffer[FRAME_LENGTH - 1]) {
        Serial.println("Warn: Checksum mismatch!");
        return;
    }

    // 将缓冲区偏移 10 字节的位置映射为有效数据结构体
    LidarDataPayload* payload = (LidarDataPayload*)(&rx_buffer[10]);

    // 串口0 上报：这里为了屏幕整洁，只遍历打印置信度大于 50 的有效测量点
    Serial.printf("=== Timestamp: %u ===\n", payload->timestamp);
    for (int i = 0; i < 12; i++) {
        if (payload->points[i].confidence > 50) {
            Serial.printf("Point %2d | Dist: %4d mm | Peak: %6d | Conf: %3d\n", 
                          i + 1, 
                          payload->points[i].distance, 
                          payload->points[i].peak, 
                          payload->points[i].confidence);
        }
    }
}

// --------------------------------------------------------
// 4. 主流程
// --------------------------------------------------------
void setup() {
    // 初始化串口0 (用于监视器打印)
    Serial.begin(115200);
    
    // 初始化串口1 (连接雷达)，波特率设为 230400，8位数据，无校验，1位停止
    Serial1.begin(230400, SERIAL_8N1, RX1_PIN, TX1_PIN);
    
    Serial.println("STP-23L Radar parsing started...");
}

void loop() {
    // 状态机流式读取串口1数据
    while (Serial1.available()) {
        uint8_t c = Serial1.read();
        rx_buffer[rx_index++] = c;

        // 动态滑窗寻找帧头 0xAAAAAAAA
        if (rx_index >= 4) {
            if (rx_buffer[0] != 0xAA || rx_buffer[1] != 0xAA || 
                rx_buffer[2] != 0xAA || rx_buffer[3] != 0xAA) {
                // 如果前4个字节不是合法包头，整体左移1个字节继续找
                for (int i = 0; i < rx_index - 1; i++) {
                    rx_buffer[i] = rx_buffer[i + 1];
                }
                rx_index--;
            }
        }

        // 成功收集齐完整一帧 (195字节)
        if (rx_index == FRAME_LENGTH) {
            parseRadarFrame();
            rx_index = 0; // 重置索引，准备接收下一帧
        }
    }
}
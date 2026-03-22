#include "lidar.h"
#include "Emm_V5.h"


// 定义 FreeRTOS 任务句柄
TaskHandle_t TaskLidarHandle = NULL;

// 存储4个通道的平均距离（全局变量，用于GETdist命令）
int avg_distances[4] = {0, 0, 0, 0};

// --------------------------------------------------------
// 任务：雷达数据轮询与解析 (运行在 Core 0)
// --------------------------------------------------------
void TaskLidarProcess(void *pvParameters) {
    uint8_t rx_buffer[FRAME_LENGTH];

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

            // 设置100ms超时机制 (避免某个雷达掉线导致整个系统卡死)
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
        //Serial.printf("[LiDAR Map] CH0: %4d mm | CH1: %4d mm | CH2: %4d mm | CH3: %4d mm\n",  avg_distances[0], avg_distances[1], avg_distances[2], avg_distances[3]);

        // 6. FreeRTOS 绝对延时，补齐剩余时间，确保整个大循环精准为 500 毫秒 (2Hz)
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

/**
 * @brief 移动机器人到指定位置
 * @param x 目标X坐标
 * @param y 目标Y坐标
 * @param theta 目标角度
 * @param isRelative 0:绝对坐标 1:相对坐标
 * @return void
 */
void GotoPose(float x, float y, float theta,bool isRelative) {
    if (isRelative) {//相对坐标
       if(x >= 0 || y >= 0 ) {//平行移动
        if(x > 0) {
            Emm_V5_Pos_Control( 1, 0, 1000, 100, x * X_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 2, 1, 1000, 100, x * X_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 3, 0, 1000, 100, x * X_PULSE, 0, 1);    
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 4, 1, 1000, 100, x * X_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Synchronous_motion(0);
            vTaskDelay(pdMS_TO_TICKS(3000));
        } else if(x < 0) {
            Emm_V5_Pos_Control( 1, 1, 1000, 100, -x * X_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 2, 0, 1000, 100, -x * X_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 3, 1, 1000, 100, -x * X_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 4, 0, 1000, 100, -x * X_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Synchronous_motion(0);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        
        if(y > 0) {
            Emm_V5_Pos_Control( 1, 0, 1000, 100, y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 2, 0, 1000, 100, y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 3, 1, 1000, 100, y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 4, 1, 1000, 100, y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Synchronous_motion(0);
            vTaskDelay(pdMS_TO_TICKS(3000));
        } else if(y < 0) {
            Emm_V5_Pos_Control( 1, 1, 1000, 100, -y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 2, 1, 1000, 100, -y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 3, 0, 1000, 100, -y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 4, 0, 1000, 100, -y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Synchronous_motion(0);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
       } else {//旋转移动
        Emm_V5_Pos_Control( 1, 0, 1000, 100, theta * THETA_PULSE, 0, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        Emm_V5_Pos_Control( 2, 0, 1000, 100, theta * THETA_PULSE, 0, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        Emm_V5_Pos_Control( 3, 0, 1000, 100, theta * THETA_PULSE, 0, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        Emm_V5_Pos_Control( 4, 0, 1000, 100, theta * THETA_PULSE, 0, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        Emm_V5_Synchronous_motion(0);
        vTaskDelay(pdMS_TO_TICKS(3000));
       }

    } else {//绝对坐标
        //暂不支持
        Serial.println("Error: GotoPose() absolute position not supported");
    }
}

/**
 * @brief 根据雷达数据推算坐标
 * @param dists 长度为4的数组，存储CH0-CH3的平均距离
 * @return 推算出的位置
 */
RobotPose GETPose(int dists[4]) {
    RobotPose pose = {0, 0, 0};    
    bool BetweenShelves = false;     //是否在两货架之间
    bool ParallelToEdge = false;    //是否平行场地边缘

    if (dists[0] > 0 && dists[1] > 0 && dists[2] > 0 && dists[3] > 0) {//检查是否有无效数据
    } else {
        Serial.println("Error: GETPose() no all valid data");
        return pose;
    }

    // --- 1. 计算 航向角 (利用 CH1 和 CH2) ---
    // 假设两个传感器向上指向 Y_MAX 方向
    pose.theta = (dists[1] - dists[2]) / 2.0f;
    if (pose.theta < 50) {
        ParallelToEdge = true;
    } else {
        BetweenShelves = false;
    }

    if (!ParallelToEdge) {//不平行场地边缘，直接返回
        Serial.println("Error: GETPose() not parallel to edge");
        return pose;
    }

    // --- 2. 计算 X 坐标 (利用 CH0 和 CH3) ---
    // 假设 CH0 指向 X_MINI 方向 (机器人左侧方向)

    if (dists[0] + dists[3]  + ROBOT_WIDTH > FIELD_X_MAX- 200) {//不再两货架之间
        pose.x =((dists[0])+(FIELD_X_MAX - dists[3] - ROBOT_WIDTH))/2;//取平均值
    } else if (dists[0] + dists[1]  + ROBOT_WIDTH > FIELD_X_MAX - SHELF_WIDTH * 2- 200) {//在两货架之间
        pose.x =((dists[0] + SHELF_WIDTH) +(FIELD_X_MAX - dists[3] - ROBOT_WIDTH - SHELF_WIDTH))/2;//取平均值
    } else {//错误情况
        Serial.println("Error: GETPose() no X valid data");
        Serial.println("dists[0]: " + String(dists[0]) + ", dists[3]: " + String(dists[3]));
        return pose;
    }

    // --- 3. 计算 Y 坐标 (利用 CH0 和 CH1) ---
    // 假设两个传感器向上指向 Y_MAX 方向
    pose.y = FIELD_Y_MAX - (dists[0] + dists[1]) / 2.0f;

    return pose;
}

// 初始化雷达相关设置
void initLidar() {
  // 初始化CD4052引脚
  pinMode(CD4052_A, OUTPUT);
  pinMode(CD4052_B, OUTPUT);
  digitalWrite(CD4052_A, LOW);
  digitalWrite(CD4052_B, LOW);

  // 初始化串口2（用于雷达通信）
  Serial2.begin(230400, SERIAL_8N1, SERIAL2_TXD_PIN, SERIAL2_RXD_PIN);

  // 创建雷达任务 (分配 8192 字节内存，运行在 Core 0)
  xTaskCreatePinnedToCore(TaskLidarProcess, "LidarProcess", 8192, NULL, 2, &TaskLidarHandle, 0);

  Serial.println("Start Lidar 4-Channel Polling Task...");
}
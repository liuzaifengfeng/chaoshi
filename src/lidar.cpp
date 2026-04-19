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
 * @brief 移动机器人到指定高度
 * @param height 目标高度，单位：mm 0-300-600（一层，二层，三层）
 * @return void
 */
void GotoHeight(float height) {
    int speed = 100;//移动速度  
    if(height < 0 || height > 640) {
        return;//高度超出范围
    }
    Emm_V5_Pos_Control( 5, 0, speed, 50, height * HEIGHT_PULSE, 1, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
}

/**
 * @brief 移动机器人到指定位置
 * @param x 目标X坐标
 * @param y 目标Y坐标
 * @param theta 目标角度
 * @param isRelative 0:绝对坐标 1:相对坐标
 * @param isAdjust 是否更改理想位置
 * @return void
 */
void GotoPose(float x, float y, float theta,bool isRelative,bool isAdjust) {
    int speed = 10;//移动速度  

    if (isRelative) {//相对坐标
       if(x != 0 || y != 0 ) {//平行移动

        if(x > 0) {
            Emm_V5_Pos_Control( 1, 0, speed, 50, x * X_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 2, 1, speed, 50, x * X_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 3, 0, speed, 50, x * X_PULSE, 0, 1);    
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 4, 1, speed, 50, x * X_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Synchronous_motion(0);
            vTaskDelay(pdMS_TO_TICKS(3000));
        } else if(x < 0) {
            Emm_V5_Pos_Control( 1, 1, speed, 50, -x * X_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 2, 0, speed, 50, -x * X_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 3, 1, speed, 50, -x * X_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 4, 0, speed, 50, -x * X_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Synchronous_motion(0);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        
        if(y > 0) {
            Emm_V5_Pos_Control( 1, 0, speed, 50, y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 2, 0, speed, 50, y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 3, 1, speed, 50, y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 4, 1, speed, 50, y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Synchronous_motion(0);
            vTaskDelay(pdMS_TO_TICKS(3000));
        } else if(y < 0) {
            Emm_V5_Pos_Control( 1, 1, speed, 50, -y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 2, 1, speed, 50, -y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 3, 0, speed, 50, -y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 4, 0, speed, 50, -y * Y_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Synchronous_motion(0);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }

       } else if(theta != 0) {//旋转移动
        if(theta > 0) {
            Emm_V5_Pos_Control( 1, 0, speed, 50, theta * THETA_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 2, 0, speed, 50, theta * THETA_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 3, 0, speed, 50, theta * THETA_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 4, 0, speed, 50, theta * THETA_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Synchronous_motion(0);
            vTaskDelay(pdMS_TO_TICKS(3000));
        } else if(theta <0){
            Emm_V5_Pos_Control( 1, 1, speed, 50, -theta * THETA_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 2, 1, speed, 50, -theta * THETA_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 3, 1, speed, 50, -theta * THETA_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Pos_Control( 4, 1, speed, 50, -theta * THETA_PULSE, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Synchronous_motion(0);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
       }

       if(isAdjust) {//位置微调

       }else{

        //更新当前位置
        if(currentPose.theta == 0) {
         currentPose.x += x;
         currentPose.y += y;
        } else if(currentPose.theta == 90) {
         currentPose.y += x;
         currentPose.x -= y;
        } else if(currentPose.theta == 180) {
         currentPose.x -= x;
         currentPose.y -= y;
        } else if(currentPose.theta == 270) {
         currentPose.y -= x;
         currentPose.x += y;
        }
        currentPose.theta += theta;
        if(currentPose.theta < 0) {
         currentPose.theta += 360;
        } else if(currentPose.theta > 360) {
         currentPose.theta -= 360;
        }

        }

    } else {//绝对坐标
        //先移动x轴
        if(currentPose.x - x != 0){
        GotoPose(x - currentPose.x, 0, 0 , true, false);
        vTaskDelay(pdMS_TO_TICKS(3000));
        }
        //再移动y轴
        if(currentPose.y - y != 0){
        GotoPose(0, y - currentPose.y, 0, true, false);
        vTaskDelay(pdMS_TO_TICKS(3000));
        }
        //再旋转到目标角度
        if(currentPose.theta - theta != 0){
        GotoPose(0, 0, theta - currentPose.theta, true, false);
        vTaskDelay(pdMS_TO_TICKS(3000));
        }

        //更新当前位置
        currentPose.x = x;
        currentPose.y = y;
        currentPose.theta = theta;
    }
}

/**
 * @brief 根据雷达数据推算坐标
 * @param dists 长度为4的数组，存储CH0-CH3的平均距离
 * @return 推算出的位置
 */
RobotPose GETRPose(int dists[4]) {
    RobotPose pose = {0, 0, 0};    
    bool BetweenShelves = false;     //是否在两货架之间
    bool ParallelToEdge = false;    //是否平行场地边缘

    if (dists[0] > 0 && dists[1] > 0 && dists[2] > 0 && dists[3] > 0) {//检查是否有无效数据
    } else {
        Serial.println("Error: GETPose() invalid data ");
        return {0, 0, 0};
    }

    // ---  计算坐标 ---
    if (currentPose.theta == 0) {//初始角度为0度， ch0指向Y_MINI方向，ch3指向Y_MAXI方向
        //计算Y坐标
        pose.y = (FIELD_Y_MAX + dists[0] - dists[3]) / 2.0f;
        if (pose.y > 1000 && pose.y < FIELD_Y_MAX - 1000) {//Y坐标在判断有效范围内(两货架之间)
            BetweenShelves = true;
        } else {
            BetweenShelves = false;
        }

        //计算X坐标
        if ((dists[1] - dists[2]) < 50 || (dists[1] - dists[2]) > -50) {//两个激光打到同一平面
            if(BetweenShelves) {//在两货架之间
                pose.x = FIELD_X_MAX - ((dists[1] + dists[2])/2.0f + SHELF_WIDTH + ROBOT_WIDTH/2.0f);
            } else {//不再两货架之间
                pose.x = FIELD_X_MAX - ((dists[1] + dists[2])/2.0f + ROBOT_WIDTH/2.0f);
            }

        //计算Theta坐标(度) 对边dists[1]-dists[2] 临边LIDAR_W_1_2
        pose.theta = currentPose.theta + atan2f(dists[1] - dists[2], LIDAR_W_1_2) * 180.0f / M_PI;  

        return pose;

        } else {//两个激光打到不同平面
            return {0, 0, 0};//非法位置
        }

    } else if (currentPose.theta == 90) { //角度为90度，ch0指向X_MAX方向，ch3指向X_MINI方向
        //计算X坐标
        if(dists[0] + dists[3] + ROBOT_LENGTH > FIELD_X_MAX - 100) {//X坐标在判断有效范围内(两货架之间)
            BetweenShelves = false;
            pose.x = (FIELD_X_MAX + dists[3] - dists[0]) / 2.0f;
        } else if((dists[0] + dists[3] + ROBOT_LENGTH) - (FIELD_X_MAX - SHELF_WIDTH*2) > -100 || (dists[0] + dists[3] + ROBOT_LENGTH) - (FIELD_X_MAX - SHELF_WIDTH*2) < 100) {//不在两货架之间
            BetweenShelves = true;
            pose.x = (FIELD_X_MAX + dists[3] - dists[0]) / 2.0f;
        } else {
            return {0, 0, 0};//非法位置
        }
        //计算Y坐标
        if((dists[1] - dists[2]) < 100 || (dists[1] - dists[2]) > -100) {//两个激光打到同一平面
            pose.y = FIELD_Y_MAX - (dists[1] + dists[2])/2.0f - ROBOT_WIDTH/2.0f;
        } else {
            return {0, 0, 0};//非法位置
        }
        //计算Theta坐标(度) 对边dists[1]-dists[2] 临边LIDAR_W_1_2
        pose.theta = currentPose.theta + atan2f(dists[1] - dists[2], LIDAR_W_1_2) * 180.0f / M_PI;  

        return pose;

    } else if (currentPose.theta == 180) { //角度为180度，ch0指向Y_MAX方向，ch3指向Y_MINI方向
        //计算Y坐标
        pose.y = (FIELD_Y_MAX + dists[3] - dists[0]) / 2.0f ;
        if (pose.y > 1000 && pose.y < FIELD_Y_MAX - 1000) {//Y坐标在判断有效范围内(两货架之间)
            BetweenShelves = true;
        } else {
            BetweenShelves = false;
        }

        //计算X坐标
        if ((dists[1] - dists[2]) < 50 || (dists[1] - dists[2]) > -50) {//两个激光打到同一平面
            if(BetweenShelves) {//在两货架之间
                pose.x = (dists[1] + dists[2])/2.0f + SHELF_WIDTH + ROBOT_WIDTH/2.0f;
            } else {//不再两货架之间
                pose.x = (dists[1] + dists[2])/2.0f + ROBOT_WIDTH/2.0f;
            }

        //计算Theta坐标(度) 对边dists[1]-dists[2] 临边LIDAR_W_1_2
        pose.theta = currentPose.theta + atan2f(dists[1] - dists[2], LIDAR_W_1_2) * 180.0f / M_PI;  

        return pose;

        } else {//两个激光打到不同平面
            return {0, 0, 0};
        }

    } else if (currentPose.theta == 270) { //角度为270度，ch0指向X_MINI方向，ch3指向X_MINI方向
        //计算X坐标
        if(dists[0] + dists[3] + ROBOT_LENGTH > FIELD_X_MAX - 100) {//X坐标在判断有效范围内(两货架之间)
            BetweenShelves = false;
            pose.x = (FIELD_X_MAX + dists[0] - dists[3]) / 2.0f;
        } else if((dists[0] + dists[3] + ROBOT_LENGTH) - (FIELD_X_MAX - SHELF_WIDTH*2) > -100 || (dists[0] + dists[3] + ROBOT_LENGTH) - (FIELD_X_MAX - SHELF_WIDTH*2) < 100) {//不在两货架之间
            BetweenShelves = true;
            pose.x = (FIELD_X_MAX + dists[0] - dists[3]) / 2.0f;
        } else {
            return {0, 0, 0};//非法位置
        }
        //计算Y坐标
        if((dists[1] - dists[2]) < 50 || (dists[1] - dists[2]) > -50) {//两个激光打到同一平面
            pose.y =(dists[1] + dists[2])/2.0f + ROBOT_WIDTH/2.0f;
        } else {
            return {0, 0, 0};//非法位置
        }
        //计算Theta坐标(度) 对边dists[1]-dists[2] 临边LIDAR_W_1_2
        pose.theta = currentPose.theta + atan2f(dists[1] - dists[2], LIDAR_W_1_2) * 180.0f / M_PI;  

        return pose;
    } else {
        return {0, 0, 0};
    }
}

/**
 * @brief 位置微调函数 - 对比实际位置与理想位置，超过阈值时进行微调
 * @return true: 调整成功, false: 调整失败或数据无效
 */
bool AdjustPose() {
        int retryCount = 0;

        float posThreshold = 20.0f; // 20mm
        float angleThreshold = 1.0f; // 1度
        int maxRetries = 1; // 最大重试次数
        float adjustRatio = 0.9f;  // 矫正系数比例，用于调整微调系数
    
    while (retryCount < maxRetries) {
        // 1. 获取实际位置
        RobotPose actualPose = GETRPose(avg_distances);
        
        // 检查数据有效性
        if (actualPose.x == 0 && actualPose.y == 0) {
            Serial.println("AdjustPose: Lidar data invalid, cannot adjust");
            return false;
        }
        
        // 2. 计算偏差
        float deltaX = currentPose.x - actualPose.x;
        float deltaY = currentPose.y - actualPose.y;
        float deltaTheta = currentPose.theta - actualPose.theta;
        
        // 角度归一化到 -180~180 度
        while (deltaTheta > 180) deltaTheta -= 360;
        while (deltaTheta < -180) deltaTheta += 360;
        
        // 3. 判断是否需要调整
        bool needAdjustX = fabs(deltaX) > posThreshold;
        bool needAdjustY = fabs(deltaY) > posThreshold;
        bool needAdjustTheta = fabs(deltaTheta) > angleThreshold;
        
        // 如果所有偏差都在阈值内，调整完成
        if (!needAdjustX && !needAdjustY && !needAdjustTheta) {
            Serial.printf("AdjustPose: Position calibrated (retry %d times)\n", retryCount);
            return true;
        }
        
        // 4. 执行微调
        Serial.printf("AdjustPose: Adjustment %d - X error:%.1fmm Y error:%.1fmm Angle error:%.1f degrees\n", 
                      retryCount + 1, deltaX, deltaY, deltaTheta);
        
        // 先调整角度
        if (needAdjustTheta) {
            GotoPose(0, 0, deltaTheta * adjustRatio, true, true);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        // 再调整位置
        if (needAdjustX || needAdjustY) {
            if(currentPose.theta == 0) {
                GotoPose(deltaX * adjustRatio, 0, 0, true, true);
                vTaskDelay(pdMS_TO_TICKS(500));
                GotoPose(0, deltaY * adjustRatio, 0, true, true);
                vTaskDelay(pdMS_TO_TICKS(500));
            } else if(currentPose.theta == 90) {
                GotoPose(deltaY * adjustRatio, 0, 0, true, true);
                vTaskDelay(pdMS_TO_TICKS(500));
                GotoPose(0, -deltaX * adjustRatio, 0, true, true);
                vTaskDelay(pdMS_TO_TICKS(500));
            } else if(currentPose.theta == 180) {
                GotoPose(-deltaX * adjustRatio, 0, 0, true, true);
                vTaskDelay(pdMS_TO_TICKS(500));
                GotoPose(0, -deltaY * adjustRatio, 0, true, true);
                vTaskDelay(pdMS_TO_TICKS(500));
            } else if(currentPose.theta == 270) {
                GotoPose(-deltaY * adjustRatio, 0, 0, true, true);
                vTaskDelay(pdMS_TO_TICKS(500));
                GotoPose(0, deltaX * adjustRatio, 0, true, true);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
        
        retryCount++;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    Serial.println("AdjustPose: error, max retries reached");
    return false;
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
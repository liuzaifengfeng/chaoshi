#include "lidar.h"
#include "Emm_V5.h"


// 定义 FreeRTOS 任务句柄
TaskHandle_t TaskLidarHandle = NULL;

// 存储4个通道的平均距离（全局变量，用于GETdist命令）
int avg_distances[4] = {0, 0, 0, 0};//4个通道的平均距离（mm）

/**
 * @brief 任务：解析由 STM32 汇总上报的雷达数据
 * 运行在 Core 0，频率保持 20Hz 以上即可（协议发送频率为 20Hz）
 */
void TaskLidarProcess(void *pvParameters) {
    uint8_t buffer[11]; // 汇总协议长度为 11 字节
    
    for (;;) {
        // 1. 寻找帧头 0x7B
        if (Serial2.available() > 0) {
            if (Serial2.read() == 0x7B) {
                buffer[0] = 0x7B;
                
                // 2. 读取后续 10 字节（设置 10ms 超时确保一帧读取完整）
                size_t len = Serial2.readBytes(&buffer[1], 10);
                
                if (len == 10) {
                    // 3. 校验帧尾 0x7D
                    if (buffer[10] == 0x7D) {
                        
                        // 4. 计算 XOR 校验 (前 9 字节)
                        uint8_t checksum = 0;
                        for (int i = 0; i < 9; i++) {
                            checksum ^= buffer[i];
                        }
                        
                        // 5. 校验通过则解析数据
                        if (checksum == buffer[9]) {
                            // 大端序解析 (高字节在前)
                            avg_distances[2] = (buffer[1] << 8) | buffer[2];//通道1距离（mm）
                            avg_distances[1] = (buffer[3] << 8) | buffer[4];//通道2距离（mm）
                            avg_distances[3] = (buffer[5] << 8) | buffer[6];//通道3距离（mm）
                            avg_distances[0] = (buffer[7] << 8) | buffer[8];//通道4距离（mm）
                            
                            // 调试打印（可选）
                            // Serial.printf("CH1:%d CH2:%d CH3:%d CH4:%d\n", avg_distances[0], avg_distances[1], avg_distances[2], avg_distances[3]);
                        }
                    }
                }
            }
        }
        
        // 稍微延时，防止过度占用 CPU，同时匹配上报频率 (50ms/次)
        vTaskDelay(pdMS_TO_TICKS(10));
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
* @brief 移动机器人到指定位置（位置模式）
* @param Y y轴方向，0为负向移动，1为正向移动。
* @param speed 移动速度，单位：mm/s
* @param stop 开始移动0/停止移动1
* @return void
*/
void movepose(bool Y, float speed, bool stop) {

    static int NOW_position = 0;//激光当前位置（角度）
    static int last_position = 0;//激光上次位置（角度）
    static bool isMoving = false;//是否正在移动

    for(int i = 0; i < 3; i++) {
        if(avg_distances[i] > 0) {
            NOW_position = avg_distances[i];
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }


    if(!stop) {//开始移动
        isMoving = true;//标记为正在移动
        if(Y) {
            Emm_V5_Vel_Control( 1, 0, speed, 50, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Vel_Control( 2, 0, speed, 50, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Vel_Control( 3, 1, speed, 50, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Vel_Control( 4, 1, speed, 50, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            Emm_V5_Vel_Control( 1, 1, speed, 50, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Vel_Control( 2, 1, speed, 50, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Vel_Control( 3, 0, speed, 50, 1);    
            vTaskDelay(pdMS_TO_TICKS(10));
            Emm_V5_Vel_Control( 4, 0, speed, 50, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        Emm_V5_Synchronous_motion(0);
        vTaskDelay(pdMS_TO_TICKS(10));
        last_position = NOW_position;

    } else {//停止移动
        if(isMoving) {  
            Emm_V5_Stop_Now(0, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
            // 同步到全局理想位置
            // 根据当前机器人角度更新坐标
            float distance_mm = NOW_position - last_position;
            if (currentPose.theta == 0) {
                currentPose.y += distance_mm;
            } else if (currentPose.theta == 90) {
                currentPose.x += distance_mm;
            } else if (currentPose.theta == 180) {
                currentPose.y -= distance_mm;
            } else if (currentPose.theta == 270) {
                currentPose.x -= distance_mm;
            }
            NOW_position = 0;//清空当前位置
            last_position = 0;//清空上次位置
            // 打印调试信息
            Serial.printf("Move completed: distance = %.2f mm, new position: (%.2f, %.2f, %.2f)\n", 
                            distance_mm, currentPose.x, currentPose.y, currentPose.theta);
            isMoving = false;//标记为未移动
        } else {
            Serial.println("Not moving");
        }
    }
}

/**
 * @brief 移动机器人到指定位置（速度模式）
 * @param x 目标X坐标
 * @param y 目标Y坐标
 * @param theta 目标角度
 * @param isRelative false:绝对坐标 true:相对坐标
 * @param isAdjust 是否更改理想位置,默认false
 * @return void
 */
void GotoPose(float x, float y, float theta,bool isRelative,bool isAdjust) {
    int speed = 80;//移动速度  

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
            vTaskDelay(pdMS_TO_TICKS(500+ 4*abs(x)));
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
            vTaskDelay(pdMS_TO_TICKS(500+ 4*abs(x)));
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
            vTaskDelay(pdMS_TO_TICKS(500+ 4*abs(y)));
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
            vTaskDelay(pdMS_TO_TICKS(500+ 4*abs(y)));
        }

       }
        if(theta != 0) {//旋转移动
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
            vTaskDelay(pdMS_TO_TICKS(500+ 25*abs(theta)));
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
            vTaskDelay(pdMS_TO_TICKS(500+ 25*abs(theta)));
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
        } else if(currentPose.theta >= 360) {
         currentPose.theta -= 360;
        }

        }

    } else {//绝对坐标
        //先判断方向
        if(currentPose.theta == 0) {
            if(currentPose.x - x != 0){
                GotoPose(x - currentPose.x, 0, 0 , true, false);
                vTaskDelay(pdMS_TO_TICKS(500+ 4*abs(x-currentPose.x)));
            }
            if(currentPose.y - y != 0){
                GotoPose(0, y - currentPose.y, 0 , true, false);
                vTaskDelay(pdMS_TO_TICKS(500+ 4*abs(y-currentPose.y)));
            }
            if(currentPose.theta - theta != 0){
                GotoPose(0, 0, theta - currentPose.theta, true, false);
                vTaskDelay(pdMS_TO_TICKS(500+ 25*abs(theta-currentPose.theta)));
            }

        } else if(currentPose.theta == 90){
            if(currentPose.y - x != 0){
                GotoPose(0, -x + currentPose.x, 0 , true, false);
                vTaskDelay(pdMS_TO_TICKS(500+ 4*abs(x-currentPose.y)));
            }
            if(currentPose.x - y != 0){
                GotoPose(y - currentPose.y, 0, 0, true, false);
                vTaskDelay(pdMS_TO_TICKS(500+ 4*abs(y-currentPose.x)));
            }
            if(currentPose.theta - theta != 0){
                GotoPose(0, 0, theta - currentPose.theta, true, false);
                vTaskDelay(pdMS_TO_TICKS(500+ 25*abs(theta-currentPose.theta)));
            }

        } else if(currentPose.theta == 180){
            if(currentPose.x - x != 0){
                GotoPose(currentPose.x - x , 0, 0 , true, false);
                vTaskDelay(pdMS_TO_TICKS(500+ 4*abs(x-currentPose.x)));
            }
            if(currentPose.y - y != 0){
                GotoPose(0, currentPose.y - y , 0 , true, false);
                vTaskDelay(pdMS_TO_TICKS(500+ 4*abs(y-currentPose.y)));
            }
            if(currentPose.theta - theta != 0){
                GotoPose(0, 0, theta - currentPose.theta,  true, false);
                vTaskDelay(pdMS_TO_TICKS(500+ 25*abs(theta-currentPose.theta)));
            }

        } else if(currentPose.theta == 270){
            if(currentPose.y - x != 0){
                GotoPose(0, x - currentPose.x, 0 , true, false);
                vTaskDelay(pdMS_TO_TICKS(500+ 4*abs(x-currentPose.y)));
            }
            if(currentPose.x - y != 0){
                GotoPose(-y + currentPose.y, 0, 0 , true, false);
                vTaskDelay(pdMS_TO_TICKS(500+ 4*abs(y-currentPose.x)));
            }
            if(currentPose.theta - theta != 0){
                GotoPose(0, 0, theta - currentPose.theta, true, false);
                vTaskDelay(pdMS_TO_TICKS(500+ 25*abs(theta-currentPose.theta)));
            }
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
        if(dists[0] + dists[3] > 1000){
            pose.y = (FIELD_Y_MAX + dists[0] - dists[3]) / 2.0f;
        }else{
            if(currentPose.y < 1000){
                pose.y = ((FIELD_Y_MAX-SHELF_LENGTH)/2.0f + dists[0] - dists[3]) / 2.0f ;
            }else if(currentPose.y > 1600){
                pose.y = ((FIELD_Y_MAX-SHELF_LENGTH)/2.0f + dists[0] - dists[3]) / 2.0f + SHELF_LENGTH + (FIELD_Y_MAX-SHELF_LENGTH)/2.0f;
            }
        }

        if (pose.y > 700 && pose.y < FIELD_Y_MAX - 700) {//Y坐标在判断有效范围内(两货架之间)
            BetweenShelves = true;
        } else {
            BetweenShelves = false;
        }

        //计算X坐标
        if (ABS(dists[1] - dists[2]) < 100) {//两个激光打到同一平面
            if(BetweenShelves) {//在两货架之间
                pose.x = FIELD_X_MAX - ((dists[1] + dists[2])/2.0f + SHELF_WIDTH + ROBOT_WIDTH/2.0f);
            } else {//不再两货架之间
                pose.x = FIELD_X_MAX - ((dists[1] + dists[2])/2.0f + ROBOT_WIDTH/2.0f);
            }

        //计算Theta坐标(度) 对边dists[1]-dists[2] 临边LIDAR_W_1_2
        pose.theta = currentPose.theta + atan2f(dists[1] - dists[2], LIDAR_W_1_2) * 180.0f / M_PI;  

        return pose;

        } else {//两个激光打到不同平面

        pose.x = FIELD_X_MAX - ((dists[1] + dists[2] + SHELF_WIDTH)/2.0f + ROBOT_WIDTH/2.0f);

        //计算Theta坐标(度) 对边dists[1]-dists[2] 临边LIDAR_W_1_2
        if(dists[1] - dists[2] >= 400){
            pose.theta = currentPose.theta + atan2f(dists[1] -500 - dists[2], LIDAR_W_1_2) * 180.0f / M_PI;  
        }else if(dists[2] - dists[1] >= 400){
            pose.theta = currentPose.theta + atan2f(dists[1] +500 - dists[2], LIDAR_W_1_2) * 180.0f / M_PI; 
        }
        
        return pose;

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
        if(dists[0] + dists[3] > 1000){
            pose.y = (FIELD_Y_MAX + dists[3] - dists[0]) / 2.0f ;
        }else{
            if(currentPose.y < 1000){
                pose.y = ((FIELD_Y_MAX-SHELF_LENGTH)/2.0f + dists[3] - dists[0]) / 2.0f ;
            }else if(currentPose.y > 1600){
                pose.y = ((FIELD_Y_MAX-SHELF_LENGTH)/2.0f + dists[3] - dists[0]) / 2.0f + SHELF_LENGTH + (FIELD_Y_MAX-SHELF_LENGTH)/2.0f;
            }
        }

        if (pose.y > 700 && pose.y < FIELD_Y_MAX - 700) {//Y坐标在判断有效范围内(两货架之间)
            BetweenShelves = true;
        } else {
            BetweenShelves = false;
        }

        //计算X坐标
        if (ABS(dists[1] - dists[2]) < 100) {//两个激光打到同一平面
            if(BetweenShelves) {//在两货架之间
                pose.x = (dists[1] + dists[2])/2.0f + SHELF_WIDTH + ROBOT_WIDTH/2.0f;
            } else {//不再两货架之间
                pose.x = (dists[1] + dists[2])/2.0f + ROBOT_WIDTH/2.0f;
            }

        //计算Theta坐标(度) 对边dists[1]-dists[2] 临边LIDAR_W_1_2
        pose.theta = currentPose.theta + atan2f(dists[1] - dists[2], LIDAR_W_1_2) * 180.0f / M_PI;  

        return pose;

        } else {//两个激光打到不同平面

                pose.x = (dists[1] + dists[2])/2.0f + ROBOT_WIDTH/2.0f;

        //计算Theta坐标(度) 对边dists[1]-dists[2] 临边LIDAR_W_1_2
        if(dists[1] - dists[2] >= 400){
            pose.theta = currentPose.theta + atan2f(dists[1] -500 - dists[2], LIDAR_W_1_2) * 180.0f / M_PI;  
        }else if(dists[2] - dists[1] >= 400){
            pose.theta = currentPose.theta + atan2f(dists[1] +500 - dists[2], LIDAR_W_1_2) * 180.0f / M_PI; 
        }
        
        return pose;

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

        float posThreshold = 10.0f; // 10mm
        float angleThreshold = 0.5f; // 0.5度
        int maxRetries = 1; // 最大重试次数
        float adjustRatio = 1.0f;  // 矫正系数比例，用于调整微调系数

        vTaskDelay(pdMS_TO_TICKS(1000));
    
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
        if (deltaTheta >= 30 || deltaTheta <= -30) {deltaTheta = 0; deltaX = 0; }//角度偏差超过30度，认为是0度
        
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

  // 初始化串口2（用于stm32）
  Serial2.begin(115200, SERIAL_8N1, SERIAL2_TXD_PIN, SERIAL2_RXD_PIN);

  // 创建雷达任务 (分配 8192 字节内存，运行在 Core 0)
  xTaskCreatePinnedToCore(TaskLidarProcess, "LidarProcess", 8192, NULL, 2, &TaskLidarHandle, 0);

  Serial.println("Start Lidar 4-Channel Polling Task...");
}
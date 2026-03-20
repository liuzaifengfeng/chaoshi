/**********************************************************
*** Emm_V5.0步进闭环控制例程
*** 编写作者：ZHANGDATOU
*** 技术支持：张大头闭环伺服
*** 淘宝店铺：https://zhangdatou.taobao.com
*** CSDN博客：http s://blog.csdn.net/zhangdatou666
*** qq交流群：262438510
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

void setup() {
  // put your setup code here, to run once:

  // 初始化LED灯
  //pinMode(LED_BUILTIN, OUTPUT);

  // 初始化串口
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_TXD_PIN, SERIAL1_RXD_PIN);
  Serial2.begin(115200, SERIAL_8N1, SERIAL2_TXD_PIN, SERIAL2_RXD_PIN);

  Serial.println("Supermarket robot initialized");

  // 上电延时2秒等待Emm_V5.0闭环初始化完毕
  delay(1000);
}

void loop() {

  // 定义接收数据数组、接收数据长度
  uint8_t rxCmd[128] = {0}; uint8_t rxCount = 0;
 
  // 位置模式：速度1000RPM，加速度0（不使用加减速直接启动），脉冲数3200（16细分下发送3200个脉冲电机转一圈），相对运动
  Emm_V5_Pos_Control(1, 0, 1000, 250, 3200, 0, 0);
  delay(1000);
  Emm_V5_Pos_Control(1, 1, 1000, 250, 3200, 0, 0);
  Serial.println("Motor 1 position control command sent");

  // 等待返回命令，命令数据缓存在数组rxCmd上，长度为rxCount
  //Emm_V5_Receive_Data(rxCmd, &rxCount);

  // 验证校验字节，验证成功则点亮LED灯，否则熄灭LED灯
  //if(rxCmd[rxCount - 1] == 0x6B) {  } else {  }

  // 调试使用，打印Emm_V5.0闭环返回的数据到串口
  // for(int i = 0; i < rxCount; i++) { Serial.write(rxCmd[i] + 1); } // 因为和USB下载口共用串口，所以让每个数据加1再发送出来，防止和电机地址冲突

  delay(1000);
 }


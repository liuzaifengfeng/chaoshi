#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include "ota_service.h"

// 内部任务句柄
static void ota_task(void *pvParameters) {
    const char* hostname = (const char*)pvParameters;

    // 1. 配置 mDNS
    if (MDNS.begin(hostname)) {
        Serial.printf("mDNS responder started: %s.local\n", hostname);
    }

    // 2. 配置 OTA 回调（可选，用于调试）
    ArduinoOTA.setHostname(hostname);
    
    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
    });

    // 3. 启动 OTA
    ArduinoOTA.begin();

    // 4. 任务循环：处理 OTA 请求
    for (;;) {
        ArduinoOTA.handle();
        vTaskDelay(pdMS_TO_TICKS(10)); // 给其他任务留出时间
    }
}

void init_ota_service(const char* ssid, const char* password, const char* hostname) {
    // 连接 WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected.");
    Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());

    // 创建 OTA 异步任务 (分配 4KB 堆栈，优先级设为 1)
    xTaskCreate(ota_task, "OTA_Task", 4096, (void*)hostname, 1, NULL);
}
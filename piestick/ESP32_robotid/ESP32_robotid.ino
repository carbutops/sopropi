#include <WiFi.h>
#include <esp_now.h>

// Estrutura dos dados recebidos
typedef struct {
  float joint1;
  float joint2;
  float joint3;
  float joint4;
  float joint5;
  float joint6;
} robotCommand;

robotCommand receivedCmd;

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(robotCommand)) {
    memcpy(&receivedCmd, data, sizeof(receivedCmd));

    //Serial.print("Received command: ");
    Serial.print(receivedCmd.joint1, 2); Serial.print(",");
    Serial.print(receivedCmd.joint2, 2); Serial.print(",");
    Serial.print(receivedCmd.joint3, 2); Serial.print(",");
    Serial.print(receivedCmd.joint4, 2); Serial.print(",");
    Serial.print(receivedCmd.joint5, 2); Serial.print(",");
    Serial.print(receivedCmd.joint6, 2); 

  } else {
    Serial.println("Invalid data size");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
  Serial.println("Robot ESP ready to receive joint angles via ESP-NOW!");
}

void loop() {
  // Nothing needed here — data is handled in the callback
}

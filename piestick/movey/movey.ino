#include <WiFi.h>
#include <esp_now.h>

// Estrutura compatível com o receptor
typedef struct {
  float joint1;
  float joint2;
  float joint3;
  float joint4;
  float joint5;
  float joint6;
} robotCommand;

// Posição atual (em milímetros, por exemplo)
float posX = 0;
float posY = 0;
float posZ = 0;

// Entradas digitais
#define PIN_AVANCA  18   // botão ou sinal que equivale a +1
#define PIN_RECUA   19   // botão ou sinal que equivale a -1

robotCommand cmdToSend;

// Endereço MAC da ESP receptora (substitua pelo real)
uint8_t peerAddress[] = {0xC8, 0xF0, 0x9E, 0xF7, 0xC5, 0x3C};
esp_now_peer_info_t peerInfo;

// Flag de controle de envio
volatile bool envioConcluido = true;

// -------------------------------------------------------------------
// Função de callback de envio — nova assinatura para ESP-IDF v5.x
// -------------------------------------------------------------------
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  envioConcluido = true;
  Serial.print("Status do envio: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sucesso" : "Falha");
}

// -------------------------------------------------------------------
// Função para enviar as coordenadas atuais via ESP-NOW
// -------------------------------------------------------------------
void enviaCoordenadas() {
  cmdToSend.joint1 = posX;
  cmdToSend.joint2 = posY;
  cmdToSend.joint3 = posZ;
  cmdToSend.joint4 = 0;
  cmdToSend.joint5 = 0;
  cmdToSend.joint6 = 0;

  esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&cmdToSend, sizeof(cmdToSend));

  // Envia também via Serial para depuração e leitura no receptor
  if (result == ESP_OK) {
    Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                  cmdToSend.joint1, cmdToSend.joint2, cmdToSend.joint3,
                  cmdToSend.joint4, cmdToSend.joint5, cmdToSend.joint6);
  } else {
    Serial.println("Erro ao enviar via ESP-NOW!");
  }
}

// -------------------------------------------------------------------
// Função moveY: atualiza o eixo Y e envia (aguardando envio anterior)
// -------------------------------------------------------------------
void moveY(float deltaY) {
  if (!envioConcluido) return;  // evita sobreposição
  envioConcluido = false;

  posY += deltaY;
  enviaCoordenadas();
}

// -------------------------------------------------------------------
// Setup
// -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_AVANCA, INPUT_PULLUP);
  pinMode(PIN_RECUA, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW!");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Erro ao adicionar peer!");
    return;
  }

  Serial.println("ESP32 transmissora pronta!");
  Serial.println("Use as entradas digitais para mover o eixo Y.");
}

// -------------------------------------------------------------------
// Loop principal
// -------------------------------------------------------------------
void loop() {
  if (digitalRead(PIN_AVANCA) == LOW) {
    moveY(5);
    delay(1000);
  }

  if (digitalRead(PIN_RECUA) == LOW) {
    moveY(-5);
    delay(1000);
  }
}

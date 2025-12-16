// === Leitura direta de 3 entradas analógicas na ESP32-S2 ===

// Defina aqui os GPIOs ADC que você vai usar
const int entrada1 = 1;  // Ex.: GPIO1
const int entrada2 = 10;  // Ex.: GPIO2
const int entrada3 = 17;  // Ex.: GPIO3

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Iniciando leitura direta das entradas ADC...");

  // Configura resolução do ADC da ESP32-S2 para 12 bits (0–4095)
  analogReadResolution(12);
}

void loop() {
  // Leitura direta de cada pino
  int valorADC1 = analogRead(entrada1);
  int valorADC2 = analogRead(entrada2);
  int valorADC3 = analogRead(entrada3);

  // Conversão para tensão (0–3.3V)
  float tensao1 = (valorADC1 / 4095.0) * 3.3;
  float tensao2 = (valorADC2 / 4095.0) * 3.3;
  float tensao3 = (valorADC3 / 4095.0) * 3.3;

  // Exibe resultados
  Serial.println("===== Leituras Instantaneas =====");
  Serial.print("ADC1: "); Serial.print(valorADC1); Serial.print("  ("); Serial.print(tensao1, 2); Serial.println(" V)");
  Serial.print("ADC2: "); Serial.print(valorADC2); Serial.print("  ("); Serial.print(tensao2, 2); Serial.println(" V)");
  Serial.print("ADC3: "); Serial.print(valorADC3); Serial.print("  ("); Serial.print(tensao3, 2); Serial.println(" V)");
  Serial.println("===============================");

  delay(500); // Atualiza a cada meio segundo
}

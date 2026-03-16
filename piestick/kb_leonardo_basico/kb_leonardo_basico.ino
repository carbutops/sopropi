#include <Arduino.h>
#include <Keyboard.h>
#include <Mouse.h>

const int entrada1 = A0;
const int entrada2 = A1;
const int entrada3 = A2;
const int entrada4 = A3;
const int entrada5 = A4;




int comandos[6][23] = {

  { 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54 },

  { 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45 },

  { 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68 },

  { 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91 },

  { 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114 },

  { 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137 }

};






int mode = 0;


const char* traduzASCII(int codigo){

  switch(codigo){

    case 8: return "kb_backspace";
    case 9: return "kb_tab";
    case 13: return "kb_enter";
    case 27: return "kb_escape";
    case 32: return "kb_space";

    case 48: return "kb_0";
    case 49: return "kb_1";
    case 50: return "kb_2";
    case 51: return "kb_3";
    case 52: return "kb_4";
    case 53: return "kb_5";
    case 54: return "kb_6";
    case 55: return "kb_7";
    case 56: return "kb_8";
    case 57: return "kb_9";

    case 65: return "kb_a";
    case 66: return "kb_b";
    case 67: return "kb_c";
    case 68: return "kb_d";
    case 69: return "kb_e";
    case 70: return "kb_f";
    case 71: return "kb_g";
    case 72: return "kb_h";
    case 73: return "kb_i";
    case 74: return "kb_j";
    case 75: return "kb_k";
    case 76: return "kb_l";
    case 77: return "kb_m";
    case 78: return "kb_n";
    case 79: return "kb_o";
    case 80: return "kb_p";
    case 81: return "kb_q";
    case 82: return "kb_r";
    case 83: return "kb_s";
    case 84: return "kb_t";
    case 85: return "kb_u";
    case 86: return "kb_v";
    case 87: return "kb_w";
    case 88: return "kb_x";
    case 89: return "kb_y";
    case 90: return "kb_z";

    case 97: return "kb_a";
    case 98: return "kb_b";
    case 99: return "kb_c";
    case 100: return "kb_d";
    case 101: return "kb_e";
    case 102: return "kb_f";
    case 103: return "kb_g";
    case 104: return "kb_h";
    case 105: return "kb_i";
    case 106: return "kb_j";
    case 107: return "kb_k";
    case 108: return "kb_l";
    case 109: return "kb_m";
    case 110: return "kb_n";
    case 111: return "kb_o";
    case 112: return "kb_p";
    case 113: return "kb_q";
    case 114: return "kb_r";
    case 115: return "kb_s";
    case 116: return "kb_t";
    case 117: return "kb_u";
    case 118: return "kb_v";
    case 119: return "kb_w";
    case 120: return "kb_x";
    case 121: return "kb_y";
    case 122: return "kb_z";

  }

  return "unknown";
}
// ==================== Função de comandos ====================
void executarComando(const char* cmd) {

  // ==================== TECLADO ====================
  if (strncmp(cmd, "kb_", 3) == 0) {
    const char* sub = cmd + 3;

    if (strcmp(sub, "enter") == 0) { Keyboard.press(KEY_RETURN); Keyboard.release(KEY_RETURN); return; }
    if (strcmp(sub, "escape") == 0) { Keyboard.press(KEY_ESC); Keyboard.release(KEY_ESC); return; }
    if (strcmp(sub, "tab") == 0) { Keyboard.press(KEY_TAB); Keyboard.release(KEY_TAB); return; }
    if (strcmp(sub, "backspace") == 0) { Keyboard.press(KEY_BACKSPACE); Keyboard.release(KEY_BACKSPACE); return; }
    if (strcmp(sub, "space") == 0) { Keyboard.write(' '); return; }

    if (strcmp(sub, "left_shift") == 0) { Keyboard.press(KEY_LEFT_SHIFT); delay(50); Keyboard.release(KEY_LEFT_SHIFT); return; }
    if (strcmp(sub, "left_ctrl") == 0) { Keyboard.press(KEY_LEFT_CTRL); delay(50); Keyboard.release(KEY_LEFT_CTRL); return; }
    if (strcmp(sub, "left_alt") == 0) { Keyboard.press(KEY_LEFT_ALT); delay(50); Keyboard.release(KEY_LEFT_ALT); return; }
    if (strcmp(sub, "left_gui") == 0) { Keyboard.press(KEY_LEFT_GUI); delay(50); Keyboard.release(KEY_LEFT_GUI); return; }

    if (strcmp(sub, "delete") == 0) { Keyboard.press(KEY_DELETE); Keyboard.release(KEY_DELETE); return; }

    // Letras minúsculas
    if (strlen(sub) == 1) {
      Keyboard.write(sub[0]);
      return;
    }
    Serial.println("===============================");
    return ;
  }  // <-- FECHA corretamente o bloco kb_

  // ==================== MOUSE ====================
  if (strncmp(cmd, "mouse_", 6) == 0) {
    const char* sub = cmd + 6;

    if      (strcmp(sub, "left") == 0)  Mouse.move(-10,0);
    else if (strcmp(sub, "right") == 0) Mouse.move(10,0);
    else if (strcmp(sub, "up") == 0)    Mouse.move(0,-10);
    else if (strcmp(sub, "down") == 0)  Mouse.move(0,10);
    else if (strcmp(sub, "wheel_up") == 0)   Mouse.move(0,0,1);
    else if (strcmp(sub, "wheel_down") == 0) Mouse.move(0,0,-1);
    else if (strcmp(sub, "left_button") == 0)   Mouse.click(MOUSE_LEFT);
    else if (strcmp(sub, "right_button") == 0)  Mouse.click(MOUSE_RIGHT);
    else if (strcmp(sub, "middle_button") == 0) Mouse.click(MOUSE_MIDDLE);
    else {
      Serial.print("Comando mouse desconhecido: ");
      Serial.println(sub);
    }

    return;
  }

  // ----- MODOS -----
  if (strcmp(cmd, "increment_mode") == 0) { mode++; return; }
  if (strcmp(cmd, "decrement_mode") == 0) { if (mode > 0) mode--; return; }

  Serial.print("Comando desconhecido: ");
  Serial.println(cmd);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(9600);
  Keyboard.begin();
  Mouse.begin();
  
}
// ==================== LOOP ====================
void loop() {

  int valorADC1 = analogRead(entrada1);
  int valorADC2 = analogRead(entrada2);
  int valorADC3 = analogRead(entrada3);
  int valorADC4 = analogRead(entrada4);
  int valorADC5 = analogRead(entrada5);

  float tensao1 = (valorADC1 / 1023.0) * 5.0;
  float tensao2 = (valorADC2 / 1023.0) * 5.0;
  float tensao3 = (valorADC3 / 1023.0) * 5.0;
  float tensao4 = (valorADC4 / 1023.0) * 5.0;
  float tensao5 = (valorADC5 / 1023.0) * 5.0;

  // -------- ENTRADA 1 --------
  if (tensao1 < 2) {
    if (tensao1 < 1) executarComando(traduzASCII(comandos[mode][0]));
    else executarComando(traduzASCII(comandos[mode][1]));
  }

  if (tensao1 > 3) {
    if (tensao1 > 4) executarComando(traduzASCII(comandos[mode][2]));
    else executarComando(traduzASCII(comandos[mode][3]));
  }

  // -------- ENTRADA 2 --------
  if (tensao2 < 2) {
    if (tensao2 < 1) executarComando(traduzASCII(comandos[mode][4]));
    else executarComando(traduzASCII(comandos[mode][5]));
  }

  if (tensao2 > 3) {
    if (tensao2 > 4) executarComando(traduzASCII(comandos[mode][6]));
    else executarComando(traduzASCII(comandos[mode][7]));
  }

  // -------- ENTRADA 3 --------
  if (tensao3 < 2) {
    if (tensao3 < 1) executarComando(traduzASCII(comandos[mode][8]));
    else executarComando(traduzASCII(comandos[mode][9]));
  }

  if (tensao3 > 3) {
    if (tensao3 > 4) executarComando(traduzASCII(comandos[mode][10]));
    else executarComando(traduzASCII(comandos[mode][11]));
  }

  // -------- ENTRADA 4 + 5 --------
  if (tensao4 > 4) {
    if (tensao5 > 4) executarComando(traduzASCII(comandos[mode][12]));
    if (tensao5 < 2) executarComando(traduzASCII(comandos[mode][13]));
    else executarComando(traduzASCII(comandos[mode][14]));
  }

  if (tensao4 < 2) {
    if (tensao5 > 4) executarComando(traduzASCII(comandos[mode][15]));
    if (tensao5 < 2) executarComando(traduzASCII(comandos[mode][16]));
    else executarComando(traduzASCII(comandos[mode][17]));
  }

  if (tensao4 > 3) {
    if (tensao5 > 4) executarComando(traduzASCII(comandos[mode][18]));
    if (tensao5 < 2) executarComando(traduzASCII(comandos[mode][19]));
    else executarComando(traduzASCII(comandos[mode][20]));
  }

  // -------- ENTRADA 5 --------
  if (tensao5 > 4) executarComando(traduzASCII(comandos[mode][21]));
  if (tensao5 < 2) executarComando(traduzASCII(comandos[mode][22]));

  delay(50);
}

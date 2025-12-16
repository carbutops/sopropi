#include <Arduino.h>
#include <Keyboard.h>
#include <Mouse.h>

const int entrada1 = A0;
const int entrada2 = A1;
const int entrada3 = A2;

int mode = 0;  // variável global para controlar modos

// ==================== Função de comandos ====================
void executarComando(const char* cmd) {
  // ----- TECLADO -----
  if (strncmp(cmd, "kb_", 3) == 0) {
    const char* sub = cmd + 3;

    if (strlen(sub) == 1 && sub[0] >= 'a' && sub[0] <= 'z') {
      Keyboard.write(sub[0]);
      return;
    }
    if (strlen(sub) == 1 && sub[0] >= '0' && sub[0] <= '9') {
      Keyboard.write(sub[0]);
      return;
    }

    if (strcmp(sub, "enter") == 0) { Keyboard.press(KEY_RETURN); Keyboard.release(KEY_RETURN); return; }
    if (strcmp(sub, "escape") == 0) { Keyboard.press(KEY_ESC); Keyboard.release(KEY_ESC); return; }
    if (strcmp(sub, "tab") == 0) { Keyboard.press(KEY_TAB); Keyboard.release(KEY_TAB); return; }
    if (strcmp(sub, "backspace") == 0) { Keyboard.press(KEY_BACKSPACE); Keyboard.release(KEY_BACKSPACE); return; }
    if (strcmp(sub, "space") == 0) { Keyboard.write(' '); return; }

    if (strcmp(sub, "left_shift") == 0) { Keyboard.press(KEY_LEFT_SHIFT); delay(50); Keyboard.release(KEY_LEFT_SHIFT); return; }
    if (strcmp(sub, "left_ctrl") == 0) { Keyboard.press(KEY_LEFT_CTRL); delay(50); Keyboard.release(KEY_LEFT_CTRL); return; }
    if (strcmp(sub, "left_alt") == 0) { Keyboard.press(KEY_LEFT_ALT); delay(50); Keyboard.release(KEY_LEFT_ALT); return; }
    if (strcmp(sub, "left_gui") == 0) { Keyboard.press(KEY_LEFT_GUI); delay(50); Keyboard.release(KEY_LEFT_GUI); return; }

    if (sub[0] == 'f') {
      int fnum = atoi(sub + 1);
      if (fnum >= 1 && fnum <= 12) {
        Keyboard.press(KEY_F1 + fnum - 1);
        Keyboard.release(KEY_F1 + fnum - 1);
      }
      return;
    }

    if      (strcmp(sub, "slash")     == 0) Keyboard.write('/');
    else if (strcmp(sub, "backslash") == 0) Keyboard.write('\\');
    else if (strcmp(sub, "minus")     == 0) Keyboard.write('-');
    else if (strcmp(sub, "equal")     == 0) Keyboard.write('=');
    else if (strcmp(sub, "semicolon") == 0) Keyboard.write(';');
    else if (strcmp(sub, "quote")     == 0) Keyboard.write('\'');
    else if (strcmp(sub, "comma")     == 0) Keyboard.write(',');
    else if (strcmp(sub, "dot")       == 0) Keyboard.write('.');
    else {
      Serial.print("Comando kb desconhecido: "); Serial.println(sub);
    }
    return;
  }

  // ----- MOUSE -----
  if (strncmp(cmd, "mouse_", 6) == 0) {
    const char* sub = cmd + 6;

    if      (strcmp(sub, "left")  == 0) Mouse.move(-10,0);
    else if (strcmp(sub, "right") == 0) Mouse.move(10,0);
    else if (strcmp(sub, "up")    == 0) Mouse.move(0,-10);
    else if (strcmp(sub, "down")  == 0) Mouse.move(0,10);

    else if (strcmp(sub, "wheel_up")   == 0) Mouse.move(0,0,1);
    else if (strcmp(sub, "wheel_down") == 0) Mouse.move(0,0,-1);

    else if (strcmp(sub, "left_button")   == 0) Mouse.click(MOUSE_LEFT);
    else if (strcmp(sub, "right_button")  == 0) Mouse.click(MOUSE_RIGHT);
    else if (strcmp(sub, "middle_button") == 0) Mouse.click(MOUSE_MIDDLE);

    else {
      Serial.print("Comando mouse desconhecido: "); Serial.println(sub);
    }
    return;
  }

  // ----- MODOS -----
  if (strcmp(cmd, "increment_mode") == 0) { mode++; return; }
  if (strcmp(cmd, "decrement_mode") == 0) { if (mode > 0) mode--; return; }

  // ----- FALLBACK -----
  Serial.print("Comando desconhecido: "); Serial.println(cmd);
}

void setup() {
  
  Serial.begin(9600);
  Keyboard.begin();
  Mouse.begin();
}

void loop() {

  int valorADC1 = analogRead(entrada1);
  int valorADC2 = analogRead(entrada2);
  int valorADC3 = analogRead(entrada3);

  // Conversão para tensão (0–5V)
  float tensao1 = (valorADC1 / 1023.0) * 5.0;
  float tensao2 = (valorADC2 / 1023.0) * 5.0;
  float tensao3 = (valorADC3 / 1023.0) * 5.0;

  if (tensao1 <2){
    executarComando("kb_w");
  }
  if (tensao1 >3){
    executarComando("kb_s");
  }
  if (tensao2 <2){
    executarComando("kb_a");
  }
  if (tensao2 >3){
    executarComando("kb_d");
  }
  if (tensao3 <2){
    executarComando("kb_space");
  }
  if (tensao3 >3){
    executarComando("kb_q");
  } // Pequena pausa para estabilizar leituras
}


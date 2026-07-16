#include <Arduino.h>

#include "USB.h"
#include "USBHIDKeyboard.h"
#include "USBHIDMouse.h"

#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

USBHIDKeyboard Keyboard;
USBHIDMouse Mouse;

const int entrada1 = 4;
const int entrada2 = 5;
const int entrada3 = 3;
const int entrada4 = 6;
const int entrada5 = 7;


#define VALOR_ADC_SF 2.5
#define VALOR_ADC_SS 3
#define VALOR_ADC_IF 0.8
#define VALOR_ADC_IS 1.2

#define L1 50.0
#define L2 40.0
#define L3 20.0
#define G  20.0

#define DELTA 5
#define RAD_TO_DEG 57.2957795

// limites de alcance
#define MAX_REACH (L1 + L2 + L3 + G)
#define MIN_REACH 30

// limites das juntas (graus)
#define J1_MIN -168
#define J1_MAX 168

#define J2_MIN -120
#define J2_MAX 120

#define J3_MIN -135
#define J3_MAX 135

#define J4_MIN -145
#define J4_MAX 145

#define J5_MIN -90
#define J5_MAX 90

typedef struct {
  float joint1;
  float joint2;
  float joint3;
  float joint4;
  float joint5;
  float joint6;
} robotCommand;

float posX = 0;
float posY = 50;
float posZ = 60;

robotCommand cmdToSend;

uint8_t peerAddress[] = {0x80, 0xF3, 0xDA, 0x62, 0x93, 0x6C};

esp_now_peer_info_t peerInfo;

volatile bool envioConcluido = true;



//------------------------------------------------
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
  envioConcluido = true;

  Serial.print("Status do envio: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sucesso" : "Falha");
}


//------------------------------------------------
void enviaCoordenadas()
{
  esp_err_t result = esp_now_send(peerAddress,(uint8_t *)&cmdToSend,sizeof(cmdToSend));

  if (result == ESP_OK)
  {
    Serial.printf("%.2f %.2f %.2f %.2f %.2f %.2f\n",
                  cmdToSend.joint1,
                  cmdToSend.joint2,
                  cmdToSend.joint3,
                  cmdToSend.joint4,
                  cmdToSend.joint5,
                  cmdToSend.joint6);
    Serial.printf("%.2f %.2f %.2f\n", posX,posY,posZ);
  }
  else
  {
    Serial.println("Erro ao enviar via ESP-NOW!");
  }
}



//------------------------------------------------
// FORWARD KINEMATICS
//------------------------------------------------
void forwardKinematics(robotCommand *cmd, float *x, float *y, float *z)
{
  float j1 = cmd->joint1 / RAD_TO_DEG;
  float j2 = cmd->joint2 / RAD_TO_DEG;
  float j3 = cmd->joint3 / RAD_TO_DEG;
  float j4 = cmd->joint4 / RAD_TO_DEG;
  float j5 = cmd->joint5 / RAD_TO_DEG;

  float a1 = j2;
  float a2 = j2 + j3;
  float a3 = j2 + j3 + j4;
  float a4 = j2 + j3 + j4 + j5;

  float r =
      L1 * sin(a1) +
      L2 * sin(a2) +
      L3 * sin(a3) +
      G  * sin(a4);

  *z =
      L1 * cos(a1) +
      L2 * cos(a2) +
      L3 * cos(a3) +
      G  * cos(a4);

  *x = r * cos(j1);
  *y = r * sin(j1);
}



//------------------------------------------------
// INVERSE KINEMATICS
//------------------------------------------------
void inverseKinematics(float x,float y,float z,robotCommand *cmd)
{
  float j1 = atan(y/x);

  float r = sqrt(x*x + y*y);

  float j5 = cmd->joint5 / RAD_TO_DEG;

  float r_eff = r - G * sin(j5);
  float z_eff = z - G * cos(j5);

  float L23 = L2 + L3;

  float D = (r_eff*r_eff + z_eff*z_eff - L1*L1 - L23*L23) / (2 * L1 * L23);

  if(D > 1) D = 1;
  if(D < -1) D = -1;

  float j3 = acos(D);

  float j2 = atan2(r_eff,z_eff) -
             atan2(L23*sin(j3), L1 + L23*cos(j3));

  float j4 = -(j2 + j3 + j5);

  cmd->joint1 = j1 * RAD_TO_DEG;
  cmd->joint2 = j2 * RAD_TO_DEG;
  cmd->joint3 = j3 * RAD_TO_DEG;
  cmd->joint4 = j4 * RAD_TO_DEG;
}



//------------------------------------------------
// CHECAR LIMITES DE JUNTAS
//------------------------------------------------
bool checkJointLimits(robotCommand *cmd)
{
  if(cmd->joint1 < J1_MIN || cmd->joint1 > J1_MAX) return false;
  if(cmd->joint2 < J2_MIN || cmd->joint2 > J2_MAX) return false;
  if(cmd->joint3 < J3_MIN || cmd->joint3 > J3_MAX) return false;
  if(cmd->joint4 < J4_MIN || cmd->joint4 > J4_MAX) return false;
  if(cmd->joint5 < J5_MIN || cmd->joint5 > J5_MAX) return false;

  return true;
}



//------------------------------------------------
// CHECAR WORKSPACE
//------------------------------------------------
int checkWorkspace(float a,float b,float c)
{
  float dist = sqrt(a*a + b*b + c*c);
  Serial.print("\n");
  Serial.print(dist);
  Serial.print("\n");
  Serial.print(MAX_REACH);


  if(dist > MAX_REACH) return 0;
  if(dist < MIN_REACH) return 0;


  return 1;
}



//------------------------------------------------
// MOVIMENTOS
//------------------------------------------------
void moveX(float deltaX)
{
  Serial.print("moveX");
  Serial.print(deltaX);
  if(!envioConcluido) return;

  float newX = posX + deltaX;
  float newY = posY;
  float newZ = posZ;

  if(!checkWorkspace(newX,newY,newZ))
  {
    Serial.println("Fora do alcance");
    return;
  }

  robotCommand tempCmd = cmdToSend;

  inverseKinematics(newX,newY,newZ,&tempCmd);

  if(!checkJointLimits(&tempCmd))
  {
    Serial.println("Limite de junta");
    return;
  }

  envioConcluido = false;

  posX = newX;
  cmdToSend = tempCmd;

  enviaCoordenadas();
}



void moveY(float deltaY)
{
  Serial.print("moveY");
  Serial.print(deltaY);
  if(!envioConcluido) return;

  float newX = posX;
  float newY = posY + deltaY;
  float newZ = posZ;

  if(!checkWorkspace(newX,newY,newZ))
  {
    Serial.println("Fora do alcance");
    return;
  }

  robotCommand tempCmd = cmdToSend;

  inverseKinematics(newX,newY,newZ,&tempCmd);

  if(!checkJointLimits(&tempCmd))
  {
    Serial.println("Limite de junta");
    return;
  }

  envioConcluido = false;

  posY = newY;
  cmdToSend = tempCmd;

  enviaCoordenadas();
}



void moveZ(float deltaZ)
{
  Serial.print("moveZ");
  Serial.print(deltaZ);
  if(!envioConcluido) return;

  float newX = posX;
  float newY = posY;
  float newZ = posZ + deltaZ;

  if(!checkWorkspace(newX,newY,newZ))
  {
    Serial.println("Fora do alcance");
    return;
  }

  robotCommand tempCmd = cmdToSend;

  inverseKinematics(newX,newY,newZ,&tempCmd);

  if(!checkJointLimits(&tempCmd))
  {
    Serial.println("Limite de junta");
    return;
  }

  envioConcluido = false;

  posZ = newZ;
  cmdToSend = tempCmd;

  enviaCoordenadas();
}  

void moveJ1(float delta){
  if(!envioConcluido) return;

  robotCommand temp = cmdToSend;
  temp.joint1 += delta;

  if(!checkJointLimits(&temp)) return;

  float x,y,z;
  forwardKinematics(&temp,&x,&y,&z);

  envioConcluido = false;

  cmdToSend = temp;
  posX = x; posY = y; posZ = z;

  enviaCoordenadas();
}

  
void moveJ2(float delta){
  if(!envioConcluido) return;

  robotCommand temp = cmdToSend;
  temp.joint2 += delta;

  if(!checkJointLimits(&temp)) return;

  float x,y,z;
  forwardKinematics(&temp,&x,&y,&z);

  envioConcluido = false;

  cmdToSend = temp;
  posX = x; posY = y; posZ = z;

  enviaCoordenadas();
}

  
void moveJ3(float delta){
  if(!envioConcluido) return;

  robotCommand temp = cmdToSend;
  temp.joint3 += delta;

  if(!checkJointLimits(&temp)) return;

  float x,y,z;
  forwardKinematics(&temp,&x,&y,&z);

  envioConcluido = false;

  cmdToSend = temp;
  posX = x; posY = y; posZ = z;

  enviaCoordenadas();
}  

void moveJ4(float delta){
  if(!envioConcluido) return;

  robotCommand temp = cmdToSend;
  temp.joint4 += delta;

  if(!checkJointLimits(&temp)) return;

  float x,y,z;
  forwardKinematics(&temp,&x,&y,&z);

  envioConcluido = false;

  cmdToSend = temp;
  posX = x; posY = y; posZ = z;

  enviaCoordenadas();
}  

void moveJ5(float delta){
  if(!envioConcluido) return;

  robotCommand temp = cmdToSend;
  temp.joint5 += delta;

  if(!checkJointLimits(&temp)) return;

  float x,y,z;
  forwardKinematics(&temp,&x,&y,&z);

  envioConcluido = false;

  cmdToSend = temp;
  posX = x; posY = y; posZ = z;

  enviaCoordenadas();
}

void moveJ6(float delta){
  if(!envioConcluido) return;

  robotCommand temp = cmdToSend;
  temp.joint6 += delta;

  if(!checkJointLimits(&temp)) return;

  float x,y,z;
  forwardKinematics(&temp,&x,&y,&z);

  envioConcluido = false;

  cmdToSend = temp;
  posX = x; posY = y; posZ = z;

  enviaCoordenadas();
}  



int comandos[6][23] = {

  { 0, 127, 127, 128, 128, 139, 0, 140, 0, 0, 0, 0, 0, 0, 123, 0,0, 124, 0, 0, 0, 125, 126 },

  { 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45 },

  { 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68 },

  { 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91 },

  { 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114 },

  { 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137 }

};






int mode = 0;


const char* traduzASCII(int codigo){

  switch(codigo){

    case 0: return "";
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

    case 123: return "move_x_pos";
    case 124: return "move_x_neg";

    case 125: return "move_y_pos";
    case 126: return "move_y_neg";

    case 127: return "move_z_pos";
    case 128: return "move_z_neg";

    case 129: return "move_j1_pos";
    case 130: return "move_j1_neg";

    case 131: return "move_j2_pos";
    case 132: return "move_j2_neg";

    case 133: return "move_j3_pos";
    case 134: return "move_j3_neg";

    case 135: return "move_j4_pos";
    case 136: return "move_j4_neg";

    case 137: return "move_j5_pos";
    case 138: return "move_j5_neg";

    case 139: return "move_j6_pos";
    case 140: return "move_j6_neg";
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
  } 

  if (strncmp(cmd, "move_", 5) == 0) {

    const char* sub = cmd + 5;

    // ===== MOVIMENTO CARTESIANO =====
    if      (strcmp(sub, "x_pos") == 0) { moveX(DELTA); }
    else if (strcmp(sub, "x_neg") == 0) { moveX(-DELTA); }

    else if (strcmp(sub, "y_pos") == 0) { moveY(DELTA); }
    else if (strcmp(sub, "y_neg") == 0) { moveY(-DELTA); }

    else if (strcmp(sub, "z_pos") == 0) { moveZ(DELTA); }
    else if (strcmp(sub, "z_neg") == 0) { moveZ(-DELTA); }

    // ===== JUNTAS =====
    else if (strcmp(sub, "j1_pos") == 0) { moveJ1(DELTA); }
    else if (strcmp(sub, "j1_neg") == 0) { moveJ1(-DELTA); }

    else if (strcmp(sub, "j2_pos") == 0) { moveJ2(DELTA); }
    else if (strcmp(sub, "j2_neg") == 0) { moveJ2(-DELTA); }

    else if (strcmp(sub, "j3_pos") == 0) { moveJ3(DELTA); }
    else if (strcmp(sub, "j3_neg") == 0) { moveJ3(-DELTA); }

    else if (strcmp(sub, "j4_pos") == 0) { moveJ4(DELTA); }
    else if (strcmp(sub, "j4_neg") == 0) { moveJ4(-DELTA); }

    else if (strcmp(sub, "j5_pos") == 0) { moveJ5(DELTA); }
    else if (strcmp(sub, "j5_neg") == 0) { moveJ5(-DELTA); }

    else if (strcmp(sub, "j6_pos") == 0) { moveJ6(DELTA); }
    else if (strcmp(sub, "j6_neg") == 0) { moveJ6(-DELTA); }

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
  USB.begin();
  Keyboard.begin();
  Mouse.begin();


  //Serial.begin(115200);

  cmdToSend.joint1 = 0;
  cmdToSend.joint2 = -40;
  cmdToSend.joint3 = 0;
  cmdToSend.joint4 = 0;
  cmdToSend.joint5 = 0;
  cmdToSend.joint6 = 0;

  forwardKinematics(&cmdToSend,&posX,&posY,&posZ);

  

  WiFi.mode(WIFI_STA);

  if(esp_now_init()!=ESP_OK)
  {
    Serial.println("Erro ao iniciar ESP-NOW!");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  memcpy(peerInfo.peer_addr,peerAddress,6);
  peerInfo.channel=0;
  peerInfo.encrypt=false;

  if(esp_now_add_peer(&peerInfo)!=ESP_OK)
  {
    Serial.println("Erro ao adicionar peer!");
    return;
  }

  enviaCoordenadas();

  Serial.println("ESP32 transmissora pronta!");

}
// ==================== LOOP ====================
void loop() {

  int valorADC1 = analogRead(entrada1);
  int valorADC2 = analogRead(entrada2);
  int valorADC3 = analogRead(entrada3);
  int valorADC4 = analogRead(entrada4);
  int valorADC5 = analogRead(entrada5);

  float tensao1 = (valorADC1 / 4095.0) * 3.3;
  float tensao2 = (valorADC2 / 4095.0) * 3.3;
  float tensao3 = (valorADC3 / 4095.0) * 3.3;
  float tensao4 = (valorADC4 / 4095.0) * 3.3;
  float tensao5 = (valorADC5 / 4095.0) * 3.3;

  // -------- ENTRADA 1 --------
  if (tensao1 < VALOR_ADC_IS) {
    if (tensao1 < VALOR_ADC_IF) executarComando(traduzASCII(comandos[mode][0]));
    else executarComando(traduzASCII(comandos[mode][1]));
  }

  if (tensao1 > VALOR_ADC_SS) {
    if (tensao1 > VALOR_ADC_SF) executarComando(traduzASCII(comandos[mode][2]));
    else executarComando(traduzASCII(comandos[mode][3]));
  }

  // -------- ENTRADA 2 --------
  if (tensao2 < VALOR_ADC_IS) {
    if (tensao2 < VALOR_ADC_IF) executarComando(traduzASCII(comandos[mode][4]));
    else executarComando(traduzASCII(comandos[mode][5]));
  }

  if (tensao2 > VALOR_ADC_SS) {
    if (tensao2 > VALOR_ADC_SF) executarComando(traduzASCII(comandos[mode][6]));
    else executarComando(traduzASCII(comandos[mode][7]));
  }

  // -------- ENTRADA 3 --------
  if (tensao3 < VALOR_ADC_IS) {
    if (tensao3 < VALOR_ADC_IF) executarComando(traduzASCII(comandos[mode][8]));
    else executarComando(traduzASCII(comandos[mode][9]));
  }

  if (tensao3 > VALOR_ADC_SS) {
    if (tensao3 > VALOR_ADC_SF) executarComando(traduzASCII(comandos[mode][10]));
    else executarComando(traduzASCII(comandos[mode][11]));
  }

  // -------- ENTRADA 4 + 5 --------
  if (tensao4 > VALOR_ADC_SF) {
    if (tensao5 > VALOR_ADC_SF) executarComando(traduzASCII(comandos[mode][12]));
    if (tensao5 < VALOR_ADC_IS) executarComando(traduzASCII(comandos[mode][13]));
    else executarComando(traduzASCII(comandos[mode][14]));
  }

  else if (tensao4 < VALOR_ADC_IS) {
    if (tensao5 > VALOR_ADC_SF) executarComando(traduzASCII(comandos[mode][15]));
    if (tensao5 < VALOR_ADC_IS) executarComando(traduzASCII(comandos[mode][16]));
    else executarComando(traduzASCII(comandos[mode][17]));
  }

  else if (tensao4 > VALOR_ADC_SS) {
    if (tensao5 > VALOR_ADC_SF) executarComando(traduzASCII(comandos[mode][18]));
    if (tensao5 < VALOR_ADC_IS) executarComando(traduzASCII(comandos[mode][19]));
    else executarComando(traduzASCII(comandos[mode][20]));
  }

  // -------- ENTRADA 5 --------
  else if (tensao5 > VALOR_ADC_SF) executarComando(traduzASCII(comandos[mode][21]));
  else if (tensao5 < VALOR_ADC_IS) executarComando(traduzASCII(comandos[mode][22]));



  Serial.print(posX);
  Serial.print(",");
  Serial.print(posY);
  Serial.print(",");
  Serial.print(posZ);
  

  delay(50);
}

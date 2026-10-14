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
const int entrada6 = 8;


#define VALOR_ADC_SF 2.8
#define VALOR_ADC_SS 3
#define VALOR_ADC_IF 0.8
#define VALOR_ADC_IS 1.0

#define L1 50.0
#define L2 40.0
#define L3 20.0
#define G  20.0

#define modos 6

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
bool inverseKinematics(float x, float y, float z, robotCommand *cmd)
{
  const float PI_F = 3.14159265f;
  const float TOL = 0.5f;       // tolerância de posição, em mm
  const float LAMBDA = 1.0f;    // amortecimento
  const int MAX_ITER = 120;

  // Escolhe uma representação cilíndrica compatível com J1.
  // Isso também trata casos em que a cinemática direta gera r negativo.
  float rho = sqrtf(x*x + y*y);
  float j1Atual = cmd->joint1 / RAD_TO_DEG;

  float j1A = atan2f(y, x);
  float j1B = j1A + PI_F;

  if (j1B > PI_F) j1B -= 2.0f * PI_F;

  float rTarget;
  float j1;

  bool validoA =
      j1A >= J1_MIN / RAD_TO_DEG &&
      j1A <= J1_MAX / RAD_TO_DEG;

  bool validoB =
      j1B >= J1_MIN / RAD_TO_DEG &&
      j1B <= J1_MAX / RAD_TO_DEG;

  if (rho < 0.001f) {
    j1 = j1Atual;
    rTarget = 0.0f;
  }
  else if (validoA && validoB) {
    if (fabsf(j1A - j1Atual) <= fabsf(j1B - j1Atual)) {
      j1 = j1A;
      rTarget = rho;
    } else {
      j1 = j1B;
      rTarget = -rho;
    }
  }
  else if (validoA) {
    j1 = j1A;
    rTarget = rho;
  }
  else if (validoB) {
    j1 = j1B;
    rTarget = -rho;
  }
  else {
    return false;
  }

  // Começa com os ângulos atuais.
  float j2 = cmd->joint2 / RAD_TO_DEG;
  float j3 = cmd->joint3 / RAD_TO_DEG;
  float j4 = cmd->joint4 / RAD_TO_DEG;
  float j5 = cmd->joint5 / RAD_TO_DEG;

  bool convergiu = false;

  for (int iter = 0; iter < MAX_ITER; iter++) {

    float a1 = j2;
    float a2 = j2 + j3;
    float a3 = j2 + j3 + j4;
    float a4 = j2 + j3 + j4 + j5;

    float r =
        L1 * sinf(a1) +
        L2 * sinf(a2) +
        L3 * sinf(a3) +
        G  * sinf(a4);

    float zAtual =
        L1 * cosf(a1) +
        L2 * cosf(a2) +
        L3 * cosf(a3) +
        G  * cosf(a4);

    float erroR = rTarget - r;
    float erroZ = z - zAtual;

    if (sqrtf(erroR*erroR + erroZ*erroZ) < TOL) {
      convergiu = true;
      break;
    }

    // Jacobiana: derivadas da posição em relação a J2, J3 e J4.
    float dr2 = L1*cosf(a1) + L2*cosf(a2)
              + L3*cosf(a3) + G*cosf(a4);
    float dz2 = -L1*sinf(a1) - L2*sinf(a2)
              - L3*sinf(a3) - G*sinf(a4);

    float dr3 = L2*cosf(a2) + L3*cosf(a3)
              + G*cosf(a4);
    float dz3 = -L2*sinf(a2) - L3*sinf(a3)
              - G*sinf(a4);

    float dr4 = L3*cosf(a3) + G*cosf(a4);
    float dz4 = -L3*sinf(a3) - G*sinf(a4);

    // J * J^T + lambda² * I
    float a11 = dr2*dr2 + dr3*dr3 + dr4*dr4
              + LAMBDA*LAMBDA;
    float a12 = dr2*dz2 + dr3*dz3 + dr4*dz4;
    float a22 = dz2*dz2 + dz3*dz3 + dz4*dz4
              + LAMBDA*LAMBDA;

    float det = a11*a22 - a12*a12;

    if (fabsf(det) < 0.000001f) {
      return false;
    }

    float u = (a22*erroR - a12*erroZ) / det;
    float v = (a11*erroZ - a12*erroR) / det;

    // Correções angulares em radianos.
    float d2 = dr2*u + dz2*v;
    float d3 = dr3*u + dz3*v;
    float d4 = dr4*u + dz4*v;

    // Limita cada passo para evitar saltos grandes.
    d2 = constrain(d2, -0.15f, 0.15f);
    d3 = constrain(d3, -0.15f, 0.15f);
    d4 = constrain(d4, -0.15f, 0.15f);

    // Mantém as juntas dentro dos limites mecânicos.
    j2 = constrain(j2 + d2,
                   J2_MIN / RAD_TO_DEG,
                   J2_MAX / RAD_TO_DEG);

    j3 = constrain(j3 + d3,
                   J3_MIN / RAD_TO_DEG,
                   J3_MAX / RAD_TO_DEG);

    j4 = constrain(j4 + d4,
                   J4_MIN / RAD_TO_DEG,
                   J4_MAX / RAD_TO_DEG);
  }

  // Confere o erro final, inclusive se atingiu o limite de iterações.
  float a1 = j2;
  float a2 = j2 + j3;
  float a3 = j2 + j3 + j4;
  float a4 = j2 + j3 + j4 + j5;

  float rFinal =
      L1*sinf(a1) + L2*sinf(a2) +
      L3*sinf(a3) + G*sinf(a4);

  float zFinal =
      L1*cosf(a1) + L2*cosf(a2) +
      L3*cosf(a3) + G*cosf(a4);

  float erroFinal = sqrtf(
      (rTarget - rFinal)*(rTarget - rFinal) +
      (z - zFinal)*(z - zFinal)
  );

  if (!convergiu && erroFinal >= TOL) {
    return false;
  }

  // Só modifica o comando quando encontrou uma solução.
  cmd->joint1 = j1 * RAD_TO_DEG;
  cmd->joint2 = j2 * RAD_TO_DEG;
  cmd->joint3 = j3 * RAD_TO_DEG;
  cmd->joint4 = j4 * RAD_TO_DEG;

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

bool checkJointLimits(robotCommand *cmd)
{
  return
    cmd->joint1 >= J1_MIN && cmd->joint1 <= J1_MAX &&
    cmd->joint2 >= J2_MIN && cmd->joint2 <= J2_MAX &&
    cmd->joint3 >= J3_MIN && cmd->joint3 <= J3_MAX &&
    cmd->joint4 >= J4_MIN && cmd->joint4 <= J4_MAX &&
    cmd->joint5 >= J5_MIN && cmd->joint5 <= J5_MAX;
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

  if (!inverseKinematics(newX, newY, newZ, &tempCmd)) {
  Serial.println("Nao foi possivel resolver a cinemática inversa");
  return;
  }

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

  if (!inverseKinematics(newX, newY, newZ, &tempCmd)) {
    Serial.println("Nao foi possivel resolver a cinemática inversa");
    return;
  }
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

  if (!inverseKinematics(newX, newY, newZ, &tempCmd)) {
  Serial.println("Nao foi possivel resolver a cinemática inversa");
  return;
  }

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

float calcularRaioRadial(float j2, float j3, float j4)
{
  float r = L1 * sin(j2)
          + L2 * sin(j2 + j3)
          + L3 * sin(j2 + j3 + j4);

  float z = L1 * cos(j2)
          + L2 * cos(j2 + j3)
          + L3 * cos(j2 + j3 + j4);

  return sqrt(r*r + z*z);
}


void movimentoRadial(float deltaP){
  if(!envioConcluido) return;

  robotCommand temp = cmdToSend;

  float j2 = temp.joint2 / RAD_TO_DEG;
  float j3Atual = temp.joint3 / RAD_TO_DEG;
  float j4Atual = temp.joint4 / RAD_TO_DEG;

  float gamma = j2 + j3Atual + j4Atual;
  float delta = gamma - j2;

  float pAtual = calcularRaioRadial(j2, j3Atual, j4Atual);
  float pDesejado = pAtual + deltaP;

  if(pDesejado <= 0.0f) return;

  float C = L1*L1 + L2*L2 + L3*L3
          + 2.0f*L1*L3*cos(j2 - gamma);

  float U = L1*L2 + L2*L3*cos(delta);
  float V = L2*L3*sin(delta);

  float q = (pDesejado*pDesejado - C) / 2.0f;
  float R = sqrt(U*U + V*V);

  if(R < 0.000001f || fabs(q) > R) return;

  float phi = atan2(V, U);
  float ang = acos(q / R);

  float j3A = phi + ang;
  float j3B = phi - ang;

  float j3Novo =
      (fabs(j3A - j3Atual) < fabs(j3B - j3Atual))
      ? j3A : j3B;

  float j4Novo = gamma - j2 - j3Novo;

  temp.joint3 = j3Novo * RAD_TO_DEG;
  temp.joint4 = j4Novo * RAD_TO_DEG;

  if(!checkJointLimits(&temp)) return;

  float x, y, z;
  forwardKinematics(&temp, &x, &y, &z);

  envioConcluido = false;

  cmdToSend = temp;
  posX = x;
  posY = y;
  posZ = z;

  enviaCoordenadas();
}

int comandos[modos][22] = {

//{1 S1+S20S30,2 S1+S2+S30,3 S1+S2+S3+,4 S1+S20S3+,5 S1-S20S30,6 S1-S2-S30,7 S1-S2-S3-,8 S1-S20S3-,9 S10S2+S30,10 S10S2+S3+,11 S10S2-S30,12 S10S2-S3-,13 S10S20S3+,14 S10S20S3-,15 S4+S50,16 S4+S5+,17 S4+S5-,18 S4-S50,19 S4-S5+,20 S4-S5-,21 S40S5+,22 S40S5-}
  { 131, 123,124, 125, 126, 127, 54, 55, 56, 57, 58, 59, 60, 61, 129, 63,64, 130, 67, 68, 143, 144  },

  { 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 0, 0 },

  { 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 0, 0  },

  { 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 0, 0  },

  { 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 0, 0  },

  { 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 0, 0  }

};






int mode = 0;

int changemode(int mode, int x)
{
  if (x > 0) {
    if (mode == modos-1) {
      mode= 0;
    } 
    else {
      mode++;
    }
  } 
  else {
    if (mode == 0) {
      mode = modos - 1;
    } 
    else {
      mode--;
    }
  }

  return mode;
}




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

    case 141: return "increment_mode";
    case 142: return "decrement_mode";
    
    case 143: return "move_radial_pos";
    case 144: return "move_radial_neg";
    
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

    else if (strcmp(sub, "radial_pos") == 0) { movimentoRadial(DELTA); }
    else if (strcmp(sub, "radial_neg") == 0) { movimentoRadial(-DELTA); }
    

    return;
  }

  // ----- MODOS -----
  if (strcmp(cmd, "increment_mode") == 0) {
    mode = changemode(mode, 1);
    return;
  }

  if (strcmp(cmd, "decrement_mode") == 0) {
    mode = changemode(mode, -1);
    return;
  }


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
  int valorADC6 = analogRead(entrada6);
  

  float tensao1 = (valorADC1 / 4095.0) * 3.3;
  float tensao2 = (valorADC2 / 4095.0) * 3.3;
  float tensao3 = (valorADC3 / 4095.0) * 3.3;
  float tensao4 = (valorADC4 / 4095.0) * 3.3;
  float tensao5 = (valorADC5 / 4095.0) * 3.3;
  float tensao6 = (valorADC6 / 4095.0) * 3.3;


  // -------- ENTRADA 1 --------

// { S1+S20S30, S1+S2+S30, S1+S2+S3+, S1+S20S3+,
//   S1-S20S30, S1-S2-S30, S1-S2-S3-, S1-S20S3-,
//   S10S2+S30, S10S2+S3+, S10S2-S30, S10S2-S3-,
//   S10S20S3+, S10S20S3-,
//   S4+S50, S4+S5+, S4+S5-,
//   S4-S50, S4-S5+, S4-S5-,
//   S40S5+, S40S5- }


// =====================================================
// S1+
// =====================================================

if (tensao1 > VALOR_ADC_SF) {

    // S1+ S20
    if (tensao2 >= VALOR_ADC_IS && tensao2 <= VALOR_ADC_SF) {

        // S1+ S20 S30
        if (tensao3 >= VALOR_ADC_IS && tensao3 <= VALOR_ADC_SF) {
            executarComando(traduzASCII(comandos[mode][0]));
        }

        // S1+ S20 S3+
        else if (tensao3 > VALOR_ADC_SF) {
            executarComando(traduzASCII(comandos[mode][3]));
        }
    }

    // S1+ S2+
    else if (tensao2 > VALOR_ADC_SF) {

        // S1+ S2+ S30
        if (tensao3 >= VALOR_ADC_IS && tensao3 <= VALOR_ADC_SF) {
            executarComando(traduzASCII(comandos[mode][1]));
        }

        // S1+ S2+ S3+
        else if (tensao3 > VALOR_ADC_SF) {
            executarComando(traduzASCII(comandos[mode][2]));
        }
    }
}


// =====================================================
// S1-
// =====================================================

else if (tensao1 < VALOR_ADC_IS) {

    // S1- S20
    if (tensao2 >= VALOR_ADC_IS && tensao2 <= VALOR_ADC_SF) {

        // S1- S20 S30
        if (tensao3 >= VALOR_ADC_IS && tensao3 <= VALOR_ADC_SF) {
            executarComando(traduzASCII(comandos[mode][4]));
        }

        // S1- S20 S3-
        else if (tensao3 < VALOR_ADC_IS) {
            executarComando(traduzASCII(comandos[mode][7]));
        }
    }

    // S1- S2-
    else if (tensao2 < VALOR_ADC_IS) {

        // S1- S2- S30
        if (tensao3 >= VALOR_ADC_IS && tensao3 <= VALOR_ADC_SF) {
            executarComando(traduzASCII(comandos[mode][5]));
        }

        // S1- S2- S3-
        else if (tensao3 < VALOR_ADC_IS) {
            executarComando(traduzASCII(comandos[mode][6]));
        }
    }
}


// =====================================================
// S10
// =====================================================

else {

    // S10 S2+
    if (tensao2 > VALOR_ADC_SF) {

        // S10 S2+ S30
        if (tensao3 >= VALOR_ADC_IS && tensao3 <= VALOR_ADC_SF) {
            executarComando(traduzASCII(comandos[mode][8]));
        }

        // S10 S2+ S3+
        else if (tensao3 > VALOR_ADC_SF) {
            executarComando(traduzASCII(comandos[mode][9]));
        }
    }

    // S10 S2-
    else if (tensao2 < VALOR_ADC_IS) {

        // S10 S2- S30
        if (tensao3 >= VALOR_ADC_IS && tensao3 <= VALOR_ADC_SF) {
            executarComando(traduzASCII(comandos[mode][10]));
        }

        // S10 S2- S3-
        else if (tensao3 < VALOR_ADC_IS) {
            executarComando(traduzASCII(comandos[mode][11]));
        }
    }

    // S10 S20
    else {

        // S10 S20 S3+
        if (tensao3 > VALOR_ADC_SF) {
            executarComando(traduzASCII(comandos[mode][12]));
        }

        // S10 S20 S3-
        else if (tensao3 < VALOR_ADC_IS) {
            executarComando(traduzASCII(comandos[mode][13]));
        }
    }
}


// =====================================================
// S4+
// =====================================================

if (tensao4 > VALOR_ADC_SF) {

    // S4+ S50
    if (tensao5 >= VALOR_ADC_IS && tensao5 <= VALOR_ADC_SF) {
        executarComando(traduzASCII(comandos[mode][14]));
    }

    // S4+ S5+
    else if (tensao5 > VALOR_ADC_SF) {
        executarComando(traduzASCII(comandos[mode][15]));
    }

    // S4+ S5-
    else if (tensao5 < VALOR_ADC_IS) {
        executarComando(traduzASCII(comandos[mode][16]));
    }
}


// =====================================================
// S4-
// =====================================================

else if (tensao4 < VALOR_ADC_IS) {

    // S4- S50
    if (tensao5 >= VALOR_ADC_IS && tensao5 <= VALOR_ADC_SF) {
        executarComando(traduzASCII(comandos[mode][17]));
    }

    // S4- S5+
    else if (tensao5 > VALOR_ADC_SF) {
        executarComando(traduzASCII(comandos[mode][18]));
    }

    // S4- S5-
    else if (tensao5 < VALOR_ADC_IS) {
        executarComando(traduzASCII(comandos[mode][19]));
    }
}


// =====================================================
// S40
// =====================================================

else {

    // S40 S5+
    if (tensao5 > VALOR_ADC_SF) {
        executarComando(traduzASCII(comandos[mode][20]));
    }

    // S40 S5-
    else if (tensao5 < VALOR_ADC_IS) {
        executarComando(traduzASCII(comandos[mode][21]));
    }
}
//if (tensao6 > VALOR_ADC_SF) {
//        executarComando(traduzASCII(141));
//        Serial.print(" mode up /n");
//    }
//
    // S40 S5-
//else if (tensao6 < VALOR_ADC_IS) {
//        executarComando(traduzASCII(142));
//Serial.print(" mode down \n");
//}



  Serial.print(posX);
  Serial.print(",");
  Serial.print(posY);
  Serial.print(",");
  Serial.print(posZ);
  Serial.print("\n");
  

  delay(100);
}

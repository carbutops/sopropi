#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

#define L1 120.0
#define L2 110.0
#define L3 90.0
#define G  60.0

#define DELTA 5
#define RAD_TO_DEG 57.2957795

// limites de alcance
#define MAX_REACH (L1 + L2 + L3 + G)
#define MIN_REACH 30

// limites das juntas (graus)
#define J1_MIN -180
#define J1_MAX 180

#define J2_MIN -100
#define J2_MAX 100

#define J3_MIN -135
#define J3_MAX 135

#define J4_MIN -180
#define J4_MAX 180

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
float posY = 200;
float posZ = 120;


#define PIN_AVANCA 18
#define PIN_RECUA  19


robotCommand cmdToSend;


uint8_t peerAddress[] = {0xC8, 0xF0, 0x9E, 0xF7, 0xC5, 0x3C};

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
  float j1 = atan2(y,x);

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
bool checkWorkspace(float x,float y,float z)
{
  float dist = sqrt(x*x + y*y + z*z);

  if(dist > MAX_REACH) return false;
  if(dist < MIN_REACH) return false;

  return true;
}



//------------------------------------------------
// MOVIMENTOS
//------------------------------------------------
void moveX(float deltaX)
{
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



//------------------------------------------------
// SETUP
//------------------------------------------------
void setup()
{
  Serial.begin(115200);

  cmdToSend.joint1 = 0;
  cmdToSend.joint2 = 0;
  cmdToSend.joint3 = 0;
  cmdToSend.joint4 = 0;
  cmdToSend.joint5 = 0;
  cmdToSend.joint6 = 0;

  forwardKinematics(&cmdToSend,&posX,&posY,&posZ);

  pinMode(PIN_AVANCA,INPUT_PULLUP);
  pinMode(PIN_RECUA,INPUT_PULLUP);

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



//------------------------------------------------
// LOOP
//------------------------------------------------
void loop()
{
  if(digitalRead(PIN_AVANCA)==LOW)
  {
    moveY(DELTA);
    delay(300);
  }

  if(digitalRead(PIN_RECUA)==LOW)
  {
    moveY(-DELTA);
    delay(300);
  }
}

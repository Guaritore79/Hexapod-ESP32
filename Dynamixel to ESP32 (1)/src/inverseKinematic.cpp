#include <Dynamixel2Arduino.h>
#include <PS4Controller.h>
#include <math.h>

#define PS4add "74:f2:fa:dd:f4:8c"
int8_t rawRY;
int8_t rawLY;
int8_t rawLX;

float move = 0;

#define coxa 40
#define femur 80
#define tibia 62

#define RXD2 16
#define TXD2 17
HardwareSerial mySerial(2);

#define DXL_SERIAL   Serial
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 4;

const float LEG_ANGLES[6] = { PI, PI, PI, 0, 0, 0};

  float safeAcos(float x){
    x = constrain(x, -1.0f, 1.0f);
    return acos(x);
  }


//class Satu kaki inverse kinematic
class HexapodLeg{
  private:
    float coxaLength, femurLength, tibiaLength;

    float coxaDeg, femurDeg, tibiaDeg;

  public:
    HexapodLeg(float lenC, float lenF, float lenT){
      coxaLength = lenC;
      femurLength = lenF;
      tibiaLength = lenT;
    }

    void inverseKinematic(float xInput, float yInput, float zInput){
      float yRest = 80; //posisi femur diam
      float zRest = -62; //posisi tibia diam

      float yTotal = yRest - yInput;
      float zTotal = zRest - zInput;

      float gamma = atan2(xInput, yTotal);
      this->coxaDeg = degrees(gamma);

      float totalHorizontal = sqrt(sq(xInput) + sq(yTotal));
      float effHorizontal = totalHorizontal - coxaLength;

      float l = sqrt(sq(effHorizontal) + sq(zTotal));

      float maxReach = femurLength + tibiaLength;
      if(l > maxReach) l = maxReach;
      if(l <1.0f) l =1.0f;

      // float tibiaAngle = acos((sq(femurLength) + sq(tibiaLength) - sq(l)) / (2 * femurLength * tibiaLength)); // tibiaAngle += (PI / 2.0);
      float cosTibia = (sq(femurLength) + sq(tibiaLength) - sq(l)) / (2 * femurLength * tibiaLength);

      float tibiaAngle = safeAcos(cosTibia);

      float cosFemur = (sq(femurLength) + sq(l) - sq(tibiaLength)) / (2 * femurLength * l);
      float vb = safeAcos(cosFemur);

      // float vb = acos((sq(femurLength) + sq(l) - sq(tibiaLength)) / ( 2 * femurLength * l));
      float va = atan2(zTotal, effHorizontal);
      float femurAngle = va + vb + PI;

      this->femurDeg = degrees(femurAngle);
      this->tibiaDeg = degrees(tibiaAngle);
  }

  int getValCoxa(){
    return map(coxaDeg + 170, 0, 360, 0, 4095);
  }

    int getValCoxa2(){
    return map(coxaDeg + 170, 0, 360, 4095, 0);
  }

  int getValFemur(){
    return map(femurDeg, 0, 360, 0, 4095);
  }

  int getValFemur2(){
    return map(femurDeg, 0, 360, 4095, 0);
  }

  int getValTibia(){
    return map(tibiaDeg + 120, 0, 360, 4095, 0);
  }

  int getValTibia2(){
    return map(tibiaDeg + 120, 0, 360, 0, 4095);
  }
    
};

const uint8_t DXL_ID_CNT = 18;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};
const float DXL_PROTOCOL_VERSION = 1.0;

// For AX-12 with Protocol 1.0
const uint16_t GOAL_POSITION_ADDR = 30;
const uint16_t GOAL_POSITION_LEN = 2;

typedef struct sw_data
{
  int16_t goal_position;
} __attribute__((packed)) sw_data_t;

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

Dynamixel2Arduino dxl(mySerial, DXL_DIR_PIN);

using namespace ControlTableItem;

HexapodLeg legs[6] = {
  HexapodLeg(coxa, femur, tibia),
  HexapodLeg(coxa, femur, tibia),
  HexapodLeg(coxa, femur, tibia),
  HexapodLeg(coxa, femur, tibia),
  HexapodLeg(coxa, femur, tibia),
  HexapodLeg(coxa, femur, tibia)
};

float t = 0;
float speed = 0.2;
float step_hight = 40;
float step_length = 40;

void setup()
{

  PS4.begin(PS4add);
  
  DEBUG_SERIAL.begin(115200);
  mySerial.begin(1000000), SERIAL_8N1, RXD2, TXD2;
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (uint8_t i = 0; i < DXL_ID_CNT; i++)
  {
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
    dxl.torqueOn(DXL_ID_LIST[i]);
  }

  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = GOAL_POSITION_ADDR;
  sw_infos.addr_length = GOAL_POSITION_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  for (uint8_t i = 0; i < DXL_ID_CNT; i++)
  {
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;
}

void loop()
{

  t += speed;

  if(PS4.isConnected()){
    if(PS4.R1()){
      move = 1;
    }
    
  }

  for (int i = 0; i < 6; i++){
    // Grouping Tripod gait
    bool leg_group = (i % 2 == 0);
    float phase = (leg_group) ? t : t + PI;

    // Calculate Trajectory (sine wave)
    float W_Y = (step_length * cos(phase)) * move;
    float W_X = 0;

    // Cordinate Rotation (to keep legs parallel)
    float theta = LEG_ANGLES[i];
    float lx = -W_X * sin(theta) + W_Y * cos(theta);
    float ly =  W_X * cos(theta) + W_Y * sin(theta);

    // COUNT LEG LIFTS (Z)
    float lz = 0;
    if (sin(phase) > 0 && move > 0.1){
      lz = step_hight * sin(phase);

    legs[i].inverseKinematic(lx, ly, lz);

    int idx = i * 3;

    if (i < 3){
      sw_data[idx].goal_position   = legs[i].getValCoxa();  
      sw_data[idx+1].goal_position = legs[i].getValFemur2(); 
      sw_data[idx+2].goal_position = legs[i].getValTibia2();
    }
    else {
      sw_data[idx].goal_position   = legs[i].getValCoxa2();
      sw_data[idx+1].goal_position = legs[i].getValFemur();  
      sw_data[idx+2].goal_position = legs[i].getValTibia();
    }

    }

  }

  sw_infos.is_info_changed = true;
  dxl.syncWrite(&sw_infos);


  delay(100);

}

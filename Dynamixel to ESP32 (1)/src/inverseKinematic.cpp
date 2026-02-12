#include <Dynamixel2Arduino.h>
#include <PS4Controller.h>
#include <math.h>

#define PS4add "74:f2:fa:dd:f4:8c"
int8_t rawRY;
int8_t rawLY;
int8_t rawLX;

float move_x = 0;
float move_y = 0;
float acceleration = 0.02;

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
    return map(coxaDeg + 170, 0, 360, 0, 1023);
  }

    int getValCoxa2(){
    return map(coxaDeg + 170, 0, 360, 1023, 0);
  }

  int getValFemur(){
    return map(femurDeg, 0, 360, 0, 1023);
  }

  int getValFemur2(){
    return map(femurDeg, 0, 360, 1023, 0);
  }

  int getValTibia(){
    return map(tibiaDeg + 120, 0, 360, 1023, 0);
  }

  int getValTibia2(){
    return map(tibiaDeg + 120, 0, 360, 0, 1023);
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
float speed = 0.02;
float step_hight = 50;
float step_length = 30;

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

  if (t > 2 * PI) {
    t -= 2 * PI;
  }

  float target_move_x = 0;
  float target_move_y = 0;

  if(PS4.isConnected()){
    //forward backward
    if(PS4.Up()) target_move_x = 1;
    else if(PS4.Down()) target_move_x = -1;

    //left right
    if(PS4.Right()) target_move_y = 1;
    else if(PS4.Left()) target_move_y = -1;
    
  }

  if (move_y < target_move_y){
    move_y += acceleration;
    if (move_y > target_move_y){
      move_y = target_move_y;
    }
  }else if(move_y > target_move_y){
    move_y -= acceleration;
    if(move_y < target_move_y){
      move_y = target_move_y;
    }
  }

  if (move_x < target_move_x){
    move_x += acceleration;
    if (move_x > target_move_x){
      move_x = target_move_x;
    }
  }else if(move_x > target_move_x){
    move_x -= acceleration;
    if(move_x < target_move_x){
      move_x = target_move_x;
    }
  }



  for (int i = 0; i < 6; i++){
    // Grouping Tripod gait
    bool leg_group = (i % 2 == 0);
    float phase = (leg_group) ? t : t + PI;

    float amplitude = step_length * cos(phase);

    float lx = 0;
    float W_Y = amplitude * move_y;
    float W_X = amplitude * move_x;
    
    if (i < 3){
      lx = W_X + W_Y;
    }else{
      lx = W_X - W_Y;
    }

    float ly = 0;

    // COUNT LEG LIFTS (Z)
    float lz = 0;
    if (sin(phase) > 0 && (abs(move_x) > 0.1 || abs(move_y) > 0.1)){
      lz = step_hight * sin(phase);
    }

    legs[i].inverseKinematic(lx, ly, lz);
  }
  // -- KAKI 0 (Leg 1) --
  sw_data[0].goal_position = legs[0].getValCoxa2();
  sw_data[1].goal_position = legs[0].getValFemur();
  sw_data[2].goal_position = legs[0].getValTibia();

  // -- KAKI 1 (Leg 2) --
  sw_data[3].goal_position = legs[1].getValCoxa2();
  sw_data[4].goal_position = legs[1].getValFemur();
  sw_data[5].goal_position = legs[1].getValTibia();

  // -- KAKI 2 (Leg 3) -- *Pake Femur2/Tibia2*
  sw_data[6].goal_position = legs[2].getValCoxa2();
  sw_data[7].goal_position = legs[2].getValFemur2(); 
  sw_data[8].goal_position = legs[2].getValTibia2();

  // -- KAKI 3 (Leg 4) -- *Pake Femur Normal*
  sw_data[9].goal_position  = legs[3].getValCoxa();
  sw_data[10].goal_position = legs[3].getValFemur();
  sw_data[11].goal_position = legs[3].getValTibia();

  // -- KAKI 4 (Leg 5) -- *Pake Femur2/Tibia2*
  sw_data[12].goal_position = legs[4].getValCoxa();
  sw_data[13].goal_position = legs[4].getValFemur2();
  sw_data[14].goal_position = legs[4].getValTibia2();

  // -- KAKI 5 (Leg 6) -- *Pake Femur2/Tibia2*
  sw_data[15].goal_position = legs[5].getValCoxa();
  sw_data[16].goal_position = legs[5].getValFemur2();
  sw_data[17].goal_position = legs[5].getValTibia2();

  sw_infos.is_info_changed = true;
  dxl.syncWrite(&sw_infos);


  delay(5);

}
#include <Dynamixel2Arduino.h>
#include <PS4Controller.h>
#include <math.h>

#define PS4add "74:f2:fa:dd:f4:8c"
int8_t rawRY;
int8_t rawLY;
int8_t rawLX;

#define coxa 40
#define femur 80
#define tibia 62

#define RXD2 16
#define TXD2 17
HardwareSerial mySerial(2);

#define DXL_SERIAL   Serial
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 4;

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

HexapodLeg leg1(coxa, femur, tibia);
HexapodLeg leg2(coxa, femur, tibia);
HexapodLeg leg3(coxa, femur, tibia);
HexapodLeg leg4(coxa, femur, tibia);
HexapodLeg leg5(coxa, femur, tibia);
HexapodLeg leg6(coxa, femur, tibia);



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
  if(PS4.isConnected()){
    rawRY = PS4.RStickY();
    rawLY = PS4.LStickY();
    rawLX = PS4.LStickX();
  }else{
    rawRY = 0;
    rawLY = 0;
    rawLX = 0;
  }

  int ly = map(rawLY, -128, 127, 40, -40);
  int lx = map(rawLX, -128, 127, -40, 40);
  int ry = map(rawRY, -128, 127, -40, 40);
  

  int targetX = 0;
  int targetY = 0;
  int targetZ = 0;

  leg1.inverseKinematic(ly, lx, ry);
  leg2.inverseKinematic(ly, lx, ry);
  leg3.inverseKinematic(ly, lx, ry);
  leg4.inverseKinematic(ly, lx, ry);
  leg5.inverseKinematic(ly, lx, ry);
  leg6.inverseKinematic(ly, lx, ry);


  // leg3.inverseKinematic(targetX, targetY, targetZ);
  sw_data[0].goal_position = leg1.getValCoxa2();
  sw_data[1].goal_position = leg1.getValFemur();
  sw_data[2].goal_position = leg1.getValTibia2();

  sw_data[3].goal_position = leg2.getValCoxa2();
  sw_data[4].goal_position = leg2.getValFemur();
  sw_data[5].goal_position = leg2.getValTibia();

  sw_data[6].goal_position = leg3.getValCoxa2();
  sw_data[7].goal_position = leg3.getValFemur();
  sw_data[8].goal_position = leg3.getValTibia();

  sw_data[9].goal_position = leg4.getValCoxa();
  sw_data[10].goal_position = leg4.getValFemur2();
  sw_data[11].goal_position = leg4.getValTibia2();

  sw_data[12].goal_position = leg5.getValCoxa();
  sw_data[13].goal_position = leg5.getValFemur2();
  sw_data[14].goal_position = leg5.getValTibia2();

  sw_data[15].goal_position = leg6.getValCoxa();
  sw_data[16].goal_position = leg6.getValFemur2();
  sw_data[17].goal_position = leg6.getValTibia2();

  sw_infos.is_info_changed = true;

  if(dxl.syncWrite(&sw_infos)){
    DEBUG_SERIAL.printf("Coxa1(ly->x): %d ||mm:%d\t Femur1(lx->y): %d ||mm:%d\t Tibia1(ry->z): %d ||mm:%d\n\n", 
    sw_data[0].goal_position,
    ly,
    sw_data[1].goal_position, 
    lx,
    sw_data[2].goal_position,
    ry);
    DEBUG_SERIAL.printf("Coxa3(ly->x): %d ||mm:%d\t Femur3(lx->y): %d ||mm:%d\t Tibia3(ry->z): %d ||mm:%d\n\n", 
    sw_data[6].goal_position,
    ly,
    sw_data[7].goal_position, 
    lx,
    sw_data[8].goal_position,
    ry);
    DEBUG_SERIAL.printf("Coxa4(ly->x): %d ||mm:%d\t Femur4(lx->y): %d ||mm:%d\t Tibia4(ry->z): %d ||mm:%d\n\n",
    sw_data[10].goal_position,
    ly,
    sw_data[11].goal_position,
    lx,
    sw_data[12].goal_position,
    ry);
  }

  delay(100);

}

#include <Dynamixel2Arduino.h>
#include <PS4Controller.h>
#include <math.h>

#define PS4add "f4:8c:50:5a:54:f0"
int8_t rawRY;
int8_t rawLY;
int8_t rawLX;

#define RXD2 16
#define TXD2 17
HardwareSerial mySerial(2);

#define DXL_SERIAL   Serial
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 4;


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

      float tibiaAngle = acos((sq(femurLength) + sq(tibiaLength) - sq(l)) / (2 * femurLength * tibiaLength));
      // tibiaAngle += (PI / 2.0);

      float vb = acos((sq(femurLength) + sq(l) - sq(tibiaLength)) / ( 2 * femurLength * l));
      float va = atan2(zTotal, effHorizontal);
      float femurAngle = va + vb + PI;

      this->femurDeg = degrees(femurAngle);
      this->tibiaDeg = degrees(tibiaAngle);
  }

  int getValCoxa(){
    return map(coxaDeg + 170, 0, 360, 0, 4095);
  }

  int getValFemur(){
    return map(femurDeg, 0, 360, 0, 4095);
  }

  int getValTibia(){
    return map(tibiaDeg + 120, 0, 360, 4095, 0);
  }
    
};

const uint8_t DXL_ID_CNT = 3;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {7, 8, 9};
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

//femur length, tibia length
HexapodLeg leg1(40, 80, 62);



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
  int lx = map(rawLX, -128, 127, 40, -40);
  int ry = map(rawRY, -128, 127, -40, 40);

  int targetX = 0;
  int targetY = 0;
  int targetZ = 0;

  // leg1.inverseKinematic(ly, lx, ry);
  leg1.inverseKinematic(targetX, targetY, targetZ);

  sw_data[0].goal_position = leg1.getValCoxa();
  sw_data[1].goal_position = leg1.getValFemur();
  sw_data[2].goal_position = leg1.getValTibia();

  sw_infos.is_info_changed = true;

  if(dxl.syncWrite(&sw_infos)){
    DEBUG_SERIAL.println("Femur: ");
    DEBUG_SERIAL.println(sw_data[1].goal_position);
    DEBUG_SERIAL.println("Tibia: ");
    DEBUG_SERIAL.println(sw_data[2].goal_position);
  }

  delay(100);

}

#include <Dynamixel2Arduino.h>
#include <PS4Controller.h>
#include "HexapodLeg.h"

#define PS4add "74:f2:fa:dd:f4:8c"

#define coxa 40
#define femur 80
#define tibia 62

// Konfigurasi Pin Dynamixel ESP32
#define RXD2 16
#define TXD2 17
#define DXL_DIR_PIN 4
HardwareSerial mySerial(2);
Dynamixel2Arduino dxl(mySerial, DXL_DIR_PIN);

// Struktur SyncWrite Dynamixel
const uint8_t DXL_ID_CNT = 18;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};

typedef struct {
    int16_t goal_position;
} __attribute__((packed)) sw_data_t;

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

HexapodLeg legs[6];

// Variabel Kontrol Gerak
float t = 0;
float speed = 0.05;
float step_hight = 25;
float step_length = 20;
float move_x = 0, move_y = 0;
float acceleration = 0.02;

void setup(){
    Serial.begin(115200);
    PS4.begin(PS4add);
    mySerial.begin(1000000, SERIAL_8N1, RXD2, TXD2);
    dxl.begin(1000000);
    dxl.setPortProtocolVersion(1.0);

    // Inisialisasi Servo
    for (int i = 0; i < DXL_ID_CNT; i++) {
      dxl.torqueOff(DXL_ID_LIST[i]);
      dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
      dxl.torqueOn(DXL_ID_LIST[i]);
    }

    // Setup SyncWrite
    sw_infos.addr = 30; // GOAL_POSITION_ADDR
    sw_infos.addr_length = 2;
    sw_infos.p_xels = info_xels_sw;
    sw_infos.xel_count = DXL_ID_CNT;

    for (int i = 0; i < DXL_ID_CNT; i++) {
      info_xels_sw[i].id = DXL_ID_LIST[i];
      info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
    }
}

void loop(){
    t += speed;
    if (t > 2 * PI) t -= 2 * PI;
    
    float target_move_x = 0, target_move_y = 0;
    if(PS4.isConnected()){
      if(PS4.Up()) target_move_x = 1; else if(PS4.Down()) target_move_x = -1;
      if(PS4.Right()) target_move_y = 1; else if(PS4.Left()) target_move_y = -1;
    }
  
    // Smoothing Akselerasi
    if (move_x < target_move_x) move_x = min(move_x + acceleration, target_move_x);
    else if (move_x > target_move_x) move_x = max(move_x - acceleration, target_move_x);
  
    if (move_y < target_move_y) move_y = min(move_y + acceleration, target_move_y);
    else if (move_y > target_move_y) move_y = max(move_y - acceleration, target_move_y);
  
    // Kalkulasi Gait untuk 6 Kaki
    for (int i = 0; i < 6; i++) {
      bool leg_group = (i % 2 == 0);
      float phase = leg_group ? t : t + PI;
      float amplitude = step_length * cos(phase);
    
      float lx = (i < 3) ? (amplitude * move_x + amplitude * move_y) : (amplitude * move_x - amplitude * move_y);
      float ly = 0;
      float lz = (sin(phase) > 0 && (abs(move_x) > 0.1 || abs(move_y) > 0.1)) ? (step_hight * sin(phase)) : 0;
    
      legs[i].inverseKinematic(lx, ly, lz);
    
      int idx = i * 3;
      bool invertSide = (i < 3); // Invert untuk kaki sebelah kiri
    
      sw_data[idx].goal_position   = legs[i].getValCoxa(invertSide);
      sw_data[idx+1].goal_position = legs[i].getValFemur(invertSide);
      sw_data[idx+2].goal_position = legs[i].getValTibia(invertSide);
    }
  
    sw_infos.is_info_changed = true;
    dxl.syncWrite(&sw_infos);
    delay(5);
}


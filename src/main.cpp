#include <Arduino.h>
#include "can_manager.h"
#include "wheel.h"

unsigned long last_time_50hz = 0;
Wheel wheel;
float Vl = 0; 
float Vr = 0;
float separation = 0.19;

float max_linear_speed = 0.4;   // m/s
float max_angular_speed = 1.2;  // rad/s

void setup(){
  Serial.begin(115200);
  can_init();
  wheel.init_wheel();
}

void loop(){

  unsigned long now = millis();
  wheel.update_speed();
  int32_t encoder_r = wheel.encoder_get_right();
  int32_t encoder_l = wheel.encoder_get_left();
  
  CanRcv_t msg;
  if(can_receive(msg)){
    if(msg.id == 0x001){
      float linear = 0;
      float angular = 0;

      memcpy(&linear, &msg.data[0], sizeof(float));
      memcpy(&angular, &msg.data[4], sizeof(float));

      // limit
      linear = map(linear, -(max_linear_speed), max_linear_speed, -100, 100);
      angular = map(angular, -(max_angular_speed), max_angular_speed, 500, -500);

      if (linear < 10 && linear > -10){
        linear = 0;
      }
      
      if (angular < 10 && angular > -10){
        angular = 0;
      }

      Vr = linear + (angular * separation/2);
      Vl = linear - (angular * separation/2);

      Serial.print(Vr);
      Serial.print(",");
      Serial.println(Vl);

    }else{
      Vr = 0;
      Vl = 0;
    }
  }else{
      Vr = 0;
      Vl = 0;
  }
  

  wheel.r_wheel_move(Vr);
  wheel.l_wheel_move(Vl);



  // if(wheel.l_get_speed() != 0 || wheel.r_get_speed() != 0){
  //   Serial.print(wheel.l_get_speed());
  //   Serial.print(",");
  //   Serial.println(wheel.r_get_speed());
  // }
  

  // 50Hz loop
  if(now - last_time_50hz >= 20){
    CanMsg_32t left_ticks;
    CanMsg_32t right_ticks;
    left_ticks.value = encoder_l;
    right_ticks.value = encoder_r;

    can_send(0x011, 4, right_ticks.bytes);
    can_send(0x012, 4, left_ticks.bytes);

    last_time_50hz = now;
  }
}
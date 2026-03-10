//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp_node2 --ros-domain-id 69 
/*
ros2 topic echo /enc/total 
ros2 topic echo /enc/joint_states 
*/
// position = มุมสะสมของล้อ rad
//data = ระยะทางเชิงเส้น m
//velocity = ความเร็วเชิงมุมของล้อ rad/s
#ifdef Node2
#include <Arduino.h>
#include <micro_ros_platformio.h>
extern "C" {
  #include <rmw_microros/rmw_microros.h>
}
#include "encoder_read.h"
#include "encoder_ros_pub.h"

QuadEncoderReader enc4;

void setup() {
  Serial.begin(115200);
  delay(200);

  enc4.begin(true);
  //FF, FR, RL, RR
  enc4.setInvert(false,true,false,false);
  enc4.setWheelRadius(0.0635f);
  
  set_microros_serial_transports(Serial);  
  //rmw_uros_set_domain_id(69);
  while (RMW_RET_OK != rmw_uros_ping_agent(100 /*ms*/, 50 /*ครั้ง*/)) {
    delay(100);
  }
  enc_microros_begin_serial();

}

void loop() {
  enc4.update();

  // ถ้า agent หลุด ฟังก์ชันนี้จะพยายามเชื่อมใหม่ให้เอง
  bool connected = enc_agent_check_and_reconnect();

  if (connected) {
    enc_microros_spin_some();
  }

  delay(2);
}
#endif
#ifdef Node2
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h> 

#include "CtrlManager.h"

// ตัวแปรสำหรับกลไก
PlantingManager robot;
//docker run -it --rm -v /dev:/dev --privileged --net=host -e ROS_DOMAIN_ID=69 microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -b 115200

//ros2 topic pub --once /plant_command std_msgs/msg/String "{data: '1'}" 
// micro-ROS entities
rcl_publisher_t ultra_pub;
rcl_publisher_t feedback_pub;
rcl_subscription_t command_sub;
std_msgs__msg__Int32 feedback_msg;
std_msgs__msg__Int32 command_msg; 

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

enum ros_state { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
ros_state state = WAITING_AGENT;

TaskHandle_t StepperTask;
void StepperLoop(void * pvParameters) {
  for(;;) {
    robot.update(); // ทำงานบน Core 0 ตลอดเวลา ไม่โดน ROS ขัดจังหวะ
    vTaskDelay(1);  // ให้ OS พักบ้างเล็กน้อย
  }
}

// Callback เมื่อได้รับคำสั่งเลข 0, 1, 2 จาก Topic "plant_command"
void command_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int cmd = msg->data;
  
  // ตรวจสอบค่าที่ได้รับผ่าน Serial Monitor
  Serial.print("Received Command: "); Serial.println(cmd);
  
  if (cmd == 0) robot.LoadPattern();
  else if (cmd == 1) robot.startPlantPattern();
  else if (cmd == 2) robot.testpattern();
  else if (cmd == 3) robot.mv_cam_up_pattern();
  else if (cmd == 4) robot.mv_cam_down_pattern();
  else if (cmd == 9) robot.stopAll(); // ใช้เลข 9 แทน 's'
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
    // สร้าง Task ใหม่ไปรันที่ Core 0
    xTaskCreatePinnedToCore(
        StepperLoop,    /* ชื่อฟังก์ชัน */
        "StepperTask",  /* ชื่อ Task */
        10000,          /* Stack size */
        NULL,           /* Parameter */
        1,              /* Priority */
        &StepperTask,   /* Task handle */
        0               /* Core ID (0) */
    );
  setupUltra();
  robot.begin();

  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 69);

  // รอการเชื่อมต่อ Agent
  while (rmw_uros_ping_agent(1000, 3) != RMW_RET_OK) { delay(100); }

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, "node2_ctrl", "", &support);

  // Publisher setup
  rclc_publisher_init_default(&feedback_pub, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "plant_feedback");

  // Subscriber setup (Int32)
  rclc_subscription_init_default(&command_sub, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "plant_command");

  // Executor (สำคัญ: ต้องมี 1 handle สำหรับ subscriber)
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &command_sub, &command_msg, &command_callback, ON_NEW_DATA);

  state = AGENT_CONNECTED;
}

unsigned long last_pub_time = 0;

void loop() {
  // 1. ให้ความสำคัญสูงสุดกับ Actuator (ต้องเรียกถี่ที่สุดเท่าที่จะทำได้)
//   robot.update();

  if (state == AGENT_CONNECTED) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    static unsigned long last_pub = 0;
    if (millis() - last_pub > 150) {
      feedback_msg.data = robot.getCurrentStep();
      rcl_publish(&feedback_pub, &feedback_msg, NULL);
      last_pub = millis();
    }

    // ตรวจสอบสถานะ Agent เป็นระยะ
    static unsigned long last_ping = 0;
    if (millis() - last_ping > 2000) {
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) state = AGENT_DISCONNECTED;
        last_ping = millis();
    }
  }else if (state == AGENT_DISCONNECTED) {
    // ปิด Enable ของ Stepper ก่อน Restart เพื่อความปลอดภัย
    robot.stopAll();
    ESP.restart();
  }
}
#endif
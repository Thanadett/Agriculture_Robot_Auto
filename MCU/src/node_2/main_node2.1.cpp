#ifdef Node2
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h> 

#include "CtrlManager.h"

// ตัวแปรสำหรับกลไก
PlantingManager robot;
//docker run -it --rm -v /dev:/dev --privileged --net=host -e ROS_DOMAIN_ID=69 microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -b 115200

//ros2 topic pub --once /plant_command std_msgs/msg/String "{data: '1'}" 
// micro-ROS entities
rcl_publisher_t feedback_pub;
rcl_subscription_t command_sub;
std_msgs__msg__String feedback_msg;
std_msgs__msg__String command_msg; 

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

void waitRobotStop() {
    delay(500); // Give it a moment to switch from IDLE to the new mode
    while(robot.isBusy()) { 
        // ไม่ต้องเรียก robot.update() ตรงนี้ เพราะแยกไปรันที่ Core 0 แล้ว
        vTaskDelay(20 / portTICK_PERIOD_MS); 
    }
}

// ฟังก์ชันช่วยส่ง Feedback เป็น String
void publish_feedback(const char * text) {
  feedback_msg.data.data = (char *)text;
  feedback_msg.data.size = strlen(text);
  rcl_publish(&feedback_pub, &feedback_msg, NULL);
}


// Callback เมื่อได้รับคำสั่งเลข 0, 1, 2 จาก Topic "plant_command"
void command_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  String cmd = String(msg->data.data);// แปลงจาก ros string เป็น Arduino String เพื่อความง่าย
  
  // ตรวจสอบค่าที่ได้รับผ่าน Serial Monitor
  Serial.print("Received Command: "); Serial.println(cmd);
  
  if (cmd == "DONE:planting") {
    
    // ลำดับที่ 1: Load Pattern
    publish_feedback("step 1");
    robot.LoadPattern(); // เปลี่ยน _activeMode เป็น LOADING
    waitRobotStop();

    // ลำดับที่ 2: Start Plant
    publish_feedback("step 2");
    robot.startPlantPattern(); // เปลี่ยน _activeMode เป็น PLANTING
    waitRobotStop();

    // แจ้งว่าจบทุกกระบวนการ
    publish_feedback("step finish");
  } 
  else if (cmd == "stop") {
    robot.stopAll(); // บังคับกลับเป็น IDLE ทันที
    publish_feedback("stopped");
  }
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
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/plant_feedback");

  // Subscriber setup (Int32)
  rclc_subscription_init_default(&command_sub, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/msg");
  
    // จอง Memory สำหรับ String Messages
  command_msg.data.capacity = 50; 
  command_msg.data.data = (char *)malloc(command_msg.data.capacity * sizeof(char));
  
  feedback_msg.data.capacity = 50;
  feedback_msg.data.data = (char *)malloc(feedback_msg.data.capacity * sizeof(char));
  // Executor (สำคัญ: ต้องมี 1 handle สำหรับ subscriber)
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &command_sub, &command_msg, &command_callback, ON_NEW_DATA);

  state = AGENT_CONNECTED;
}

unsigned long last_pub_time = 0;
void loop() {
  if (state == AGENT_CONNECTED) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) state = AGENT_DISCONNECTED;
  } else {
    robot.stopAll();
    ESP.restart();
  }
}
#endif
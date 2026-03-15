#ifdef Node2
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h> 
#include <std_msgs/msg/float32_multi_array.h> 

#include "CtrlManager.h"


// ตัวแปรสำหรับกลไก
PlantingManager robot;
//docker run -it --rm -v /dev:/dev --privileged --net=host -e ROS_DOMAIN_ID=69 microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -b 115200
//ros2 topic pub --once /msg std_msgs/msg/String "{data: 'DONE:planting'}"
//ros2 topic pub --once /vision_debug std_msgs/msg/Float32MultiArray "{data: [4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

// micro-ROS entities
rcl_publisher_t feedback_pub;
rcl_subscription_t command_sub;
// rcl_subscription_t vision_sub; 
std_msgs__msg__String feedback_msg;
std_msgs__msg__String command_msg; 

std_msgs__msg__Float32MultiArray vision_msg; 

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

enum ros_state { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
ros_state state = WAITING_AGENT;

enum RobotCommand {
  CMD_NONE,
  CMD_RESET,
  CMD_STOP,
  CMD_PLANT1B,
  CMD_PLANT1A,
  CMD_PLANT2,
  // CMD_CAMUP,
  CMD_UP,
  CMD_LOAD 
};

volatile RobotCommand pending_cmd = CMD_NONE;


// ฟังก์ชันช่วยส่ง Feedback เป็น String
void publish_feedback(const char * text) {
  feedback_msg.data.data = (char *)text;
  feedback_msg.data.size = strlen(text);
  rcl_publish(&feedback_pub, &feedback_msg, NULL);
}

float vision_state_val = 0;

TaskHandle_t StepperTask;
void StepperLoop(void * pvParameters) {
  for(;;) {
    robot.update(); // ทำงานบน Core 0 ตลอดเวลา ไม่โดน ROS ขัดจังหวะ
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}

void waitRobotStop() {
  delay(50);
    while(robot.isBusy()) { 
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // ← keep alive
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    publish_feedback("DEBUG:robot_stopped");

}


// Callback สำหรับ vision_debug
volatile bool cam_has_moved = false;

// void vision_callback(const void * msgin) {
//   const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
//   if (msg->data.size > 0) {
//     vision_state_val = msg->data.data[0];
//     if (vision_state_val == 4.0f && pending_cmd == CMD_NONE && !cam_has_moved) {
//       pending_cmd = CMD_CAMUP;
//       cam_has_moved = true; // ล็อคไม่ให้ขยับซ้ำ
//     }
//   }
// }


void command_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;

  String cmd = "";
  cmd.reserve(msg->data.size);

  for (size_t i = 0; i < msg->data.size; i++) {
    cmd += msg->data.data[i];
  }
  cmd.trim();

  if (cmd == "DONE:RESET") {
    pending_cmd = CMD_RESET;
  }
  else if (cmd == "DONE:STOP") {
    pending_cmd = CMD_STOP;
  }
  else if (cmd == "DONE:planting1B") {
    pending_cmd = CMD_PLANT1B;
  }
  else if (cmd == "DONE:planting1A") {
    pending_cmd = CMD_PLANT1A;
  }
  else if (cmd == "DONE:planting2") {
    pending_cmd = CMD_PLANT2;
  }
  else if (cmd == "DONE:load") { 
    pending_cmd = CMD_LOAD;
  }
  else if (cmd == "DONE:up") {
    pending_cmd = CMD_UP;
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  robot.begin();
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
  // rclc_subscription_init_default(&vision_sub, &node, 
    // ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/vision_debug");
  
    // จอง Memory สำหรับ String Messages
  command_msg.data.capacity = 50; 
  command_msg.data.data = (char *)malloc(command_msg.data.capacity * sizeof(char));
  
  feedback_msg.data.capacity = 50;
  feedback_msg.data.data = (char *)malloc(feedback_msg.data.capacity * sizeof(char));
  
  vision_msg.data.capacity = 9;
  vision_msg.data.data = (float *)malloc(vision_msg.data.capacity * sizeof(float));
  vision_msg.data.size = 0;

  // Executor (สำคัญ: ต้องมี 1 handle สำหรับ subscriber)
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &command_sub, &command_msg, &command_callback, ON_NEW_DATA);
  // rclc_executor_add_subscription(&executor, &vision_sub, &vision_msg, &vision_callback, ON_NEW_DATA);

  state = AGENT_CONNECTED;
}

unsigned long last_pub_time = 0;
void loop() {

  if (state == AGENT_CONNECTED) {

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));

    if (pending_cmd != CMD_NONE) {

      RobotCommand cmd = pending_cmd;
      pending_cmd = CMD_NONE;

      if (cmd == CMD_RESET) {
        publish_feedback("resetting system");
        cam_has_moved = false; 
        robot.resetPattern();
        waitRobotStop();
        publish_feedback("SUCCESS");
      }

      else if (cmd == CMD_STOP) {
        robot.stopAll();
        publish_feedback("SUCCESS");
      }

      else if (cmd == CMD_PLANT1B) {
        publish_feedback("moving camera up"); 
        robot.mv_cam_up_pattern();          
        waitRobotStop();    

        publish_feedback("plantingB");
        robot.startFixPattern();
        waitRobotStop();
         
        publish_feedback("SUCCESS");
      }

      else if (cmd == CMD_PLANT1A) {
        publish_feedback("moving camera up"); 
        robot.mv_cam_up_pattern();          
        waitRobotStop();    

        publish_feedback("plantingA");
        robot.startPlantpattern();
        waitRobotStop();

        publish_feedback("SUCCESS");
      }

      else if (cmd == CMD_PLANT2) {
        publish_feedback("moving camera up"); 
        robot.mv_cam_up_pattern();          
        waitRobotStop();

        publish_feedback("plant loading");
        robot.LoadPattern();
        waitRobotStop();

        publish_feedback("planting2");
        robot.startPlantpattern();
        waitRobotStop();
    

        publish_feedback("startc");
        waitRobotStop(); 
        
        publish_feedback("startc");
        waitRobotStop(); 

        publish_feedback("SUCCESS");
      }

      // else if (cmd == CMD_CAMUP) {
      //   publish_feedback("moving camera up");
      //   robot.mv_cam_up_pattern();
      //   waitRobotStop();
      //   publish_feedback("SUCCESS");
      // }

      else if (cmd == CMD_LOAD) {
        publish_feedback("loading");
        robot.LoadPattern();
        waitRobotStop();
        publish_feedback("SUCCESS");
      }

      else if (cmd == CMD_UP) {
        publish_feedback("moving camera up");
        robot.mv_cam_up_pattern();
        waitRobotStop();
        publish_feedback("SUCCESS");
      }
    }

    if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
      state = AGENT_DISCONNECTED;
  }

  else {
    robot.stopAll();
    ESP.restart();
  }
}
#endif
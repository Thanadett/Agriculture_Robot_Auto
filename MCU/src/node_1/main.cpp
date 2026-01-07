#ifdef Node1

#include <Arduino.h>
#include <math.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

#include "config.h"
#include "motor_driver.h"
#include "encoder_read.h"
#include "imu_bno055.h"
#include "pid_rate.h"

// ============================ Config ============================
#define TOPIC_CMD_VEL "cmd_vel"
#define TOPIC_IMU "imu/data"
#define TOPIC_WHEEL_TICKS "wheel_ticks"
#define TOPIC_HEARTBEAT "robot_heartbeat"
#define TOPIC_JOY_RESET "joy_reset"

#ifndef ROS_DOMAIN_ID_MCU
#define ROS_DOMAIN_ID_MCU 69
#endif

#define YAW_RATE_DEADZONE 0.02f
#define W_CMD_DEADZONE 0.05f

// ============================ micro-ROS State ============================
enum AgentState
{
    WAITING_AGENT = 0,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};
static AgentState g_state = WAITING_AGENT;

// ============================ Core ROS ============================
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_timer_t timer_ctrl;

// ============================ Pub/Sub ============================
static rcl_publisher_t pub_imu;
static rcl_publisher_t pub_ticks;
static rcl_publisher_t pub_heartbeat;

static rcl_subscription_t sub_cmd_vel;
static rcl_subscription_t sub_joy_reset;

// ============================ Messages ============================
static sensor_msgs__msg__Imu msg_imu;
static geometry_msgs__msg__Twist msg_cmd_vel;
static std_msgs__msg__Float32MultiArray msg_ticks;
static std_msgs__msg__String msg_hb;
static std_msgs__msg__Bool msg_joy_reset;

// ============================ Robot modules ============================
static IMU_BNO055 g_imu;
static QuadEncoderReader enc;

static PIDRate g_pid_wz(
    1.2f,  // Kp
    0.0f,  // Ki
    0.0f,  // Kd
    -2.5f, // output_min
    2.5f,  // output_max
    -0.4f, // I_min
    0.4f,  // I_max
    3.0f   // D low-pass cutoff (Hz)
);

static volatile float g_V_cmd = 0.0f;
static volatile float g_W_cmd = 0.0f;

// ============================ Helpers ============================
#define RCCHECK(fn)         \
    if ((fn) != RCL_RET_OK) \
    rclErrorLoop()
#define RCSOFTCHECK(fn) (void)(fn)

#define EXECUTE_EVERY_MS(ms, code) \
    do                             \
    {                              \
        static uint32_t _t = 0;    \
        uint32_t _n = millis();    \
        if (_n - _t >= ms)         \
        {                          \
            code;                  \
            _t = _n;               \
        }                          \
    } while (0)

static void rclErrorLoop()
{
    delay(200);
    ESP.restart();
}

// ============================ Callbacks ============================
static void on_cmd_vel(const void *msgin)
{
    const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
    g_V_cmd = m->linear.x;
    g_W_cmd = m->angular.z;
}

static void on_joy_reset(const void *msgin)
{
    const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
    if (m->data)
        enc.reset();
}

static void on_timer(rcl_timer_t *, int64_t)
{
    enc.update();
    motorDrive_update();

    msg_ticks.data.data[0] = enc.totalDistanceM(W_FL);
    msg_ticks.data.data[1] = enc.totalDistanceM(W_FR);
    msg_ticks.data.data[2] = enc.totalDistanceM(W_RL);
    msg_ticks.data.data[3] = enc.totalDistanceM(W_RR);

    RCSOFTCHECK(rcl_publish(&pub_ticks, &msg_ticks, NULL));

    static uint32_t hb_ts = 0;
    if (millis() - hb_ts > 200)
    {
        const char *hb = "OK";
        msg_hb.data.data = (char *)hb;
        msg_hb.data.size = strlen(hb);
        msg_hb.data.capacity = msg_hb.data.size + 1;
        RCSOFTCHECK(rcl_publish(&pub_heartbeat, &msg_hb, NULL));
        hb_ts = millis();
    }
}

// ============================ IMU publish ============================
static void publish_imu()
{
    g_imu.update();

    msg_imu.header.stamp.sec = rmw_uros_epoch_millis() / 1000ULL;
    msg_imu.header.stamp.nanosec = (rmw_uros_epoch_millis() % 1000ULL) * 1000000ULL;

    const char *frame = "imu_link";
    msg_imu.header.frame_id.data = (char *)frame;
    msg_imu.header.frame_id.size = strlen(frame);
    msg_imu.header.frame_id.capacity = msg_imu.header.frame_id.size + 1;

    msg_imu.orientation.x = g_imu.qx;
    msg_imu.orientation.y = g_imu.qy;
    msg_imu.orientation.z = g_imu.qz;
    msg_imu.orientation.w = g_imu.qw;

    msg_imu.angular_velocity.z = g_imu.wz;
    msg_imu.linear_acceleration.x = g_imu.ax;
    msg_imu.linear_acceleration.y = g_imu.ay;
    msg_imu.linear_acceleration.z = g_imu.az;

    msg_imu.orientation_covariance[0] = 1e-3;
    msg_imu.angular_velocity_covariance[0] = 1e-3;
    msg_imu.linear_acceleration_covariance[0] = 1e-2;

    RCSOFTCHECK(rcl_publish(&pub_imu, &msg_imu, NULL));
}

// ============================ Setup ============================
void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    motorDrive_begin();
    enc.begin(true);

    if (!g_imu.begin())
        rclErrorLoop();

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_base", "", &support);

    rclc_publisher_init_default(&pub_imu, &node,
                                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), TOPIC_IMU);

    rclc_publisher_init_default(&pub_ticks, &node,
                                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), TOPIC_WHEEL_TICKS);
    rosidl_runtime_c__float32__Sequence__init(&msg_ticks.data, 4);

    rclc_publisher_init_default(&pub_heartbeat, &node,
                                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), TOPIC_HEARTBEAT);

    rclc_subscription_init_default(&sub_cmd_vel, &node,
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), TOPIC_CMD_VEL);

    rclc_subscription_init_default(&sub_joy_reset, &node,
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TOPIC_JOY_RESET);

    rclc_timer_init_default(&timer_ctrl, &support, RCL_MS_TO_NS(20), on_timer);

    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_timer(&executor, &timer_ctrl);
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, on_cmd_vel, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_joy_reset, &msg_joy_reset, on_joy_reset, ON_NEW_DATA);
}

// ============================ Loop ============================
void loop()
{
    EXECUTE_EVERY_MS(20, publish_imu()); // 50 Hz IMU

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));

    float wz = (fabs(g_W_cmd) < W_CMD_DEADZONE) ? 0.0f : g_W_cmd;
    if (fabs(wz) < YAW_RATE_DEADZONE)
        g_pid_wz.reset();

    float u = g_pid_wz.step(wz, g_imu.wz, 0.02f);
    cmdVW_to_targets(g_V_cmd, wz + u);
}

#endif

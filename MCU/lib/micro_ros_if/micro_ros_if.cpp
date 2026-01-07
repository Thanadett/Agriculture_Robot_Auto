#include "micro_ros_if.h"

#include <std_msgs/msg/int32_multi_array.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rmw_microros/rmw_microros.h> // rmw_uros_ping_agent, rmw_uros_set_custom_transport (ถ้าจำเป็น)
#include <WiFi.h>
#include <string.h>

// ========================= Internal State =========================
static micro_ros_state_t g_state = MICRO_ROS_STATE_DISCONNECTED;
static bool g_debug_enabled = false;
static char g_last_error[256] = {0};

// ROS entities
static rcl_allocator_t g_allocator;
static rclc_support_t g_support;
static rcl_node_t g_node;
static rclc_executor_t g_executor;

// Publishers
static rcl_publisher_t g_pub_odom;
static rcl_publisher_t g_pub_imu;
static rcl_publisher_t g_pub_joint_state;
static rcl_publisher_t g_pub_diagnostics;
static rcl_publisher_t g_pub_heartbeat;

// Subscribers
static rcl_subscription_t g_sub_cmd_vel;
static rcl_subscription_t g_sub_command;

// Services
static rcl_service_t g_srv_reset_odom;

// Messages
static geometry_msgs__msg__Twist g_msg_cmd_vel;
static std_msgs__msg__String g_msg_command;
static nav_msgs__msg__Odometry g_msg_odom;
static sensor_msgs__msg__Imu g_msg_imu;
static sensor_msgs__msg__JointState g_msg_joint_state;
static diagnostic_msgs__msg__DiagnosticArray g_msg_diagnostics;
static std_msgs__msg__String g_msg_heartbeat;

// Service messages
static std_srvs__srv__Empty_Request g_req_empty;
static std_srvs__srv__Empty_Response g_res_empty;

// Callbacks
static twist_callback_t g_twist_callback = nullptr;
static string_callback_t g_string_callback = nullptr;

// Timing
static uint32_t g_last_heartbeat_ms = 0;
static uint32_t g_last_odom_publish_ms = 0;
static uint32_t g_last_imu_publish_ms = 0;
static uint32_t g_last_joint_publish_ms = 0;
static uint32_t g_last_diag_publish_ms = 0;

// Publisher states
static bool g_odom_publisher_created = false;
static bool g_imu_publisher_created = false;
static bool g_joint_publisher_created = false;
static bool g_diagnostics_publisher_created = false;
static bool g_heartbeat_publisher_created = false;

// Frame IDs
static char g_odom_frame_id[64] = "odom";
static char g_odom_child_frame_id[64] = "base_link";
static char g_imu_frame_id[64] = "imu_link";

// Encoder
static rcl_publisher_t g_pub_wheel_ticks;
static std_msgs__msg__Int32MultiArray g_msg_wheel_ticks;
static bool g_wheel_ticks_publisher_created = false;
static size_t g_wheel_ticks_len = 0;

// ========================= Internal Functions =========================

static void setError(const char *error_msg)
{
    const char *rcl_err = rcl_get_error_string().str;
    if (rcl_err && rcl_err[0] != '\0')
    {
        snprintf(g_last_error, sizeof(g_last_error),
                 "%s | rcl_error: %s", error_msg, rcl_err);
        rcl_reset_error();
    }
    else
    {
        strncpy(g_last_error, error_msg, sizeof(g_last_error) - 1);
        g_last_error[sizeof(g_last_error) - 1] = '\0';
    }
    if (g_debug_enabled)
    {
        Serial.printf("[micro-ROS ERROR] %s\n", g_last_error);
    }
}

static void debugPrint(const char *msg)
{
    if (g_debug_enabled)
    {
        Serial.printf("[micro-ROS] %s\n", msg);
    }
}

// Callback functions
static void cmdVelCallback(const void *msg_in)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
    if (g_twist_callback)
    {
        g_twist_callback(msg);
    }
}

static void commandCallback(const void *msg_in)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msg_in;
    if (g_string_callback)
    {
        g_string_callback(msg);
    }
}

// Service callbacks
static void resetOdomServiceCallback(const void *req, void *res)
{
    debugPrint("Reset odometry service called");
    // This would typically reset odometry - implementation depends on your needs
}

static bool initializeROS()
{
    // Initialize allocator
    g_allocator = rcl_get_default_allocator();

    // Initialize support
    if (rclc_support_init(&g_support, 0, NULL, &g_allocator) != RCL_RET_OK)
    {
        setError("Failed to initialize support");
        return false;
    }

    // Create node
    if (rclc_node_init_default(&g_node, "micro_ros_robot", "", &g_support) != RCL_RET_OK)
    {
        setError("Failed to create node");
        return false;
    }

    // Initialize executor
    if (rclc_executor_init(&g_executor, &g_support.context, MICRO_ROS_EXECUTOR_HANDLES, &g_allocator) != RCL_RET_OK)
    {
        setError("Failed to initialize executor");
        return false;
    }

    return true;
}

// ========================= Public API Implementation =========================

bool microROS_begin(const char *agent_ip, uint16_t agent_port)
{
    debugPrint("Initializing micro-ROS...");

    // --- เลือก transport ---
    if (agent_ip != nullptr)
    {
        // (ถ้าใช้ WiFi ค่อยใส่ set_microros_wifi_transports ตรงนี้)
        debugPrint("Using WiFi transport (not configured in this snippet)");
        setError("WiFi transport not configured");
        return false;
    }
    else
    {
        // Serial transport
        set_microros_serial_transports(Serial);
        debugPrint("Using Serial transport");
    }

    // --- Ping/รอ Agent ก่อนเริ่ม ROS ---
    // รอ agent 5 วินาที (ลอง 5 ครั้ง ๆ ละ 1000ms)
    const int timeout_ms = 1000;
    const int attempts = 5;
    if (rmw_uros_ping_agent(timeout_ms, attempts) != RMW_RET_OK)
    {
        setError("Agent not reachable over serial (ping failed)");
        g_state = MICRO_ROS_STATE_ERROR;
        return false;
    }
    debugPrint("Agent reachable, proceeding to init ROS");

    // --- Init ROS entities ---
    if (!initializeROS())
    {
        g_state = MICRO_ROS_STATE_ERROR;
        return false;
    }

    // เตรียม buffer string ที่ใช้ซ้ำ
    g_msg_command.data.data = (char *)malloc(256);
    g_msg_command.data.capacity = 256;
    g_msg_heartbeat.data.data = (char *)malloc(128);
    g_msg_heartbeat.data.capacity = 128;

    g_state = MICRO_ROS_STATE_CONNECTED;
    debugPrint("micro-ROS initialized successfully");
    return true;
}

void microROS_update()
{
    if (g_state != MICRO_ROS_STATE_CONNECTED)
    {
        static uint32_t last_try = 0;
        uint32_t now = millis();
        if (now - last_try > 2000)
        { // ทุก 2 วินาที
            last_try = now;
            if (rmw_uros_ping_agent(200, 1) == RMW_RET_OK)
            {
                // ลอง re-init
                if (initializeROS())
                {
                    g_state = MICRO_ROS_STATE_CONNECTED;
                    debugPrint("Reconnected to Agent");
                }
            }
        }
        return;
    }

    // ปกติ
    rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(1));

    uint32_t now = millis();
    if (g_heartbeat_publisher_created && (now - g_last_heartbeat_ms >= MICRO_ROS_HEARTBEAT_MS))
    {
        snprintf(g_msg_heartbeat.data.data, g_msg_heartbeat.data.capacity,
                 "Robot alive at %lu ms", now);
        g_msg_heartbeat.data.size = strlen(g_msg_heartbeat.data.data);
        if (rcl_publish(&g_pub_heartbeat, &g_msg_heartbeat, NULL) == RCL_RET_OK)
            g_last_heartbeat_ms = now;
    }
}

micro_ros_state_t microROS_getState()
{
    return g_state;
}

bool microROS_isConnected()
{
    return g_state == MICRO_ROS_STATE_CONNECTED;
}

// ========================= Subscribers =========================

bool microROS_subscribeCmdVel(twist_callback_t callback, const char *topic_name)
{
    if (g_state != MICRO_ROS_STATE_CONNECTED)
    {
        setError("Not connected");
        return false;
    }

    g_twist_callback = callback;

    if (rclc_subscription_init_default(&g_sub_cmd_vel, &g_node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                       topic_name) != RCL_RET_OK)
    {
        setError("Failed to create cmd_vel subscription");
        return false;
    }

    if (rclc_executor_add_subscription(&g_executor, &g_sub_cmd_vel,
                                       &g_msg_cmd_vel, &cmdVelCallback, ON_NEW_DATA) != RCL_RET_OK)
    {
        setError("Failed to add cmd_vel subscription to executor");
        return false;
    }

    debugPrint("cmd_vel subscription created");
    return true;
}

bool microROS_subscribeCommand(string_callback_t callback, const char *topic_name)
{
    if (g_state != MICRO_ROS_STATE_CONNECTED)
    {
        setError("Not connected");
        return false;
    }

    g_string_callback = callback;

    if (rclc_subscription_init_default(&g_sub_command, &g_node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                       topic_name) != RCL_RET_OK)
    {
        setError("Failed to create command subscription");
        return false;
    }

    if (rclc_executor_add_subscription(&g_executor, &g_sub_command,
                                       &g_msg_command, &commandCallback, ON_NEW_DATA) != RCL_RET_OK)
    {
        setError("Failed to add command subscription to executor");
        return false;
    }

    debugPrint("command subscription created");
    return true;
}

// ========================= Publishers =========================

bool microROS_createWheelTicksPublisher(const char *topic_name, size_t length)
{
    if (g_state != MICRO_ROS_STATE_CONNECTED)
    {
        setError("Not connected");
        return false;
    }
    if (length == 0)
    {
        setError("wheel_ticks length must be > 0");
        return false;
    }

    // 1) Create the publisher
    if (rclc_publisher_init_default(
            &g_pub_wheel_ticks, &g_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
            topic_name) != RCL_RET_OK)
    {
        setError("Failed to create wheel_ticks publisher");
        return false;
    }

    // 2) Initialize the message and allocate its data sequence once
    if (!std_msgs__msg__Int32MultiArray__init(&g_msg_wheel_ticks))
    {
        setError("Failed to init Int32MultiArray message");
        rcl_ret_t rc = rcl_publisher_fini(&g_pub_wheel_ticks, &g_node);
        if (rc != RCL_RET_OK)
        {
            setError("Failed to fini wheel_ticks publisher");
        }
        return false;
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&g_msg_wheel_ticks.data, length))
    {
        setError("Failed to allocate data sequence for wheel_ticks");
        std_msgs__msg__Int32MultiArray__fini(&g_msg_wheel_ticks);
        rcl_ret_t rc = rcl_publisher_fini(&g_pub_wheel_ticks, &g_node);
        if (rc != RCL_RET_OK)
        {
            setError("Failed to fini wheel_ticks publisher");
        }
        return false;
    }

    g_wheel_ticks_len = length;
    g_wheel_ticks_publisher_created = true;
    debugPrint("wheel_ticks publisher created");
    return true;
}

bool microROS_publishWheelTicks(const int32_t *ticks, size_t length)
{
    if (!g_wheel_ticks_publisher_created)
    {
        setError("wheel_ticks publisher not created");
        return false;
    }
    if (ticks == nullptr)
    {
        setError("ticks pointer is null");
        return false;
    }
    if (length == 0)
        length = g_wheel_ticks_len;
    if (length != g_wheel_ticks_len)
    {
        setError("ticks length mismatch");
        return false;
    }

    // Copy tick values into pre-allocated sequence; no extra allocations at runtime
    for (size_t i = 0; i < length; ++i)
    {
        g_msg_wheel_ticks.data.data[i] = ticks[i];
    }
    g_msg_wheel_ticks.data.size = length;

    return (rcl_publish(&g_pub_wheel_ticks, &g_msg_wheel_ticks, NULL) == RCL_RET_OK);
}

bool microROS_publishWheelTicks4(int32_t lf, int32_t rf, int32_t lr, int32_t rr)
{
    if (!g_wheel_ticks_publisher_created)
    {
        setError("wheel_ticks publisher not created");
        return false;
    }
    if (g_wheel_ticks_len != 4)
    {
        setError("wheel_ticks length is not 4");
        return false;
    }

    // NOTE: Define a consistent order. Here we use LF, RF, LR, RR.
    g_msg_wheel_ticks.data.data[0] = lf;
    g_msg_wheel_ticks.data.data[1] = rf;
    g_msg_wheel_ticks.data.data[2] = lr;
    g_msg_wheel_ticks.data.data[3] = rr;
    g_msg_wheel_ticks.data.size = 4;

    return (rcl_publish(&g_pub_wheel_ticks, &g_msg_wheel_ticks, NULL) == RCL_RET_OK);
}

bool microROS_createOdomPublisher(const char *topic_name, const char *frame_id, const char *child_frame_id)
{
    if (g_state != MICRO_ROS_STATE_CONNECTED)
    {
        setError("Not connected");
        return false;
    }

    strncpy(g_odom_frame_id, frame_id, sizeof(g_odom_frame_id) - 1);
    strncpy(g_odom_child_frame_id, child_frame_id, sizeof(g_odom_child_frame_id) - 1);

    if (rclc_publisher_init_default(&g_pub_odom, &g_node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                    topic_name) != RCL_RET_OK)
    {
        setError("Failed to create odometry publisher");
        return false;
    }

    // Initialize odometry message
    g_msg_odom.header.frame_id.data = g_odom_frame_id;
    g_msg_odom.header.frame_id.size = strlen(g_odom_frame_id);
    g_msg_odom.header.frame_id.capacity = sizeof(g_odom_frame_id);
    g_msg_odom.child_frame_id.data = g_odom_child_frame_id;
    g_msg_odom.child_frame_id.size = strlen(g_odom_child_frame_id);
    g_msg_odom.child_frame_id.capacity = sizeof(g_odom_child_frame_id);

    g_odom_publisher_created = true;
    debugPrint("Odometry publisher created");
    return true;
}

bool microROS_publishOdom(const odom_data_t *odom_data)
{
    if (!g_odom_publisher_created)
    {
        setError("Odometry publisher not created");
        return false;
    }

    // Fill timestamp
    microROS_usToRosTime(odom_data->timestamp_us, &g_msg_odom.header.stamp);

    // Fill pose
    g_msg_odom.pose.pose.position.x = odom_data->x;
    g_msg_odom.pose.pose.position.y = odom_data->y;
    g_msg_odom.pose.pose.position.z = 0.0;

    // Convert theta to quaternion
    double cy = cos(odom_data->theta * 0.5);
    double sy = sin(odom_data->theta * 0.5);
    g_msg_odom.pose.pose.orientation.w = cy;
    g_msg_odom.pose.pose.orientation.x = 0.0;
    g_msg_odom.pose.pose.orientation.y = 0.0;
    g_msg_odom.pose.pose.orientation.z = sy;

    // Fill twist
    g_msg_odom.twist.twist.linear.x = odom_data->vx;
    g_msg_odom.twist.twist.linear.y = odom_data->vy;
    g_msg_odom.twist.twist.linear.z = 0.0;
    g_msg_odom.twist.twist.angular.x = 0.0;
    g_msg_odom.twist.twist.angular.y = 0.0;
    g_msg_odom.twist.twist.angular.z = odom_data->vtheta;

    return rcl_publish(&g_pub_odom, &g_msg_odom, NULL) == RCL_RET_OK;
}

bool microROS_createIMUPublisher(const char *topic_name, const char *frame_id)
{
    if (g_state != MICRO_ROS_STATE_CONNECTED)
    {
        setError("Not connected");
        return false;
    }

    strncpy(g_imu_frame_id, frame_id, sizeof(g_imu_frame_id) - 1);

    if (rclc_publisher_init_default(&g_pub_imu, &g_node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                    topic_name) != RCL_RET_OK)
    {
        setError("Failed to create IMU publisher");
        return false;
    }

    // Initialize IMU message
    g_msg_imu.header.frame_id.data = g_imu_frame_id;
    g_msg_imu.header.frame_id.size = strlen(g_imu_frame_id);
    g_msg_imu.header.frame_id.capacity = sizeof(g_imu_frame_id);

    g_imu_publisher_created = true;
    debugPrint("IMU publisher created");
    return true;
}

bool microROS_publishIMU(const imu_data_t *imu_data)
{
    if (!g_imu_publisher_created)
    {
        setError("IMU publisher not created");
        return false;
    }

    // Fill timestamp
    microROS_usToRosTime(imu_data->timestamp_us, &g_msg_imu.header.stamp);

    // Fill orientation (quaternion)
    g_msg_imu.orientation.w = imu_data->quat_w;
    g_msg_imu.orientation.x = imu_data->quat_x;
    g_msg_imu.orientation.y = imu_data->quat_y;
    g_msg_imu.orientation.z = imu_data->quat_z;

    // Fill angular velocity
    g_msg_imu.angular_velocity.x = imu_data->gyro_x;
    g_msg_imu.angular_velocity.y = imu_data->gyro_y;
    g_msg_imu.angular_velocity.z = imu_data->gyro_z;

    // Fill linear acceleration
    g_msg_imu.linear_acceleration.x = imu_data->accel_x;
    g_msg_imu.linear_acceleration.y = imu_data->accel_y;
    g_msg_imu.linear_acceleration.z = imu_data->accel_z;

    return rcl_publish(&g_pub_imu, &g_msg_imu, NULL) == RCL_RET_OK;
}

bool microROS_createJointStatePublisher(const char *topic_name)
{
    if (g_state != MICRO_ROS_STATE_CONNECTED)
    {
        setError("Not connected");
        return false;
    }

    if (rclc_publisher_init_default(&g_pub_joint_state, &g_node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                    topic_name) != RCL_RET_OK)
    {
        setError("Failed to create joint state publisher");
        return false;
    }

    g_joint_publisher_created = true;
    debugPrint("Joint state publisher created");
    return true;
}

bool microROS_publishJointState(const joint_data_t *joint_data)
{
    if (!g_joint_publisher_created)
    {
        setError("Joint state publisher not created");
        return false;
    }

    // Fill timestamp
    microROS_usToRosTime(joint_data->timestamp_us, &g_msg_joint_state.header.stamp);

    // This is a simplified implementation - in practice you'd need to properly
    // allocate and manage the arrays
    // For now, assume the message arrays are properly sized

    return rcl_publish(&g_pub_joint_state, &g_msg_joint_state, NULL) == RCL_RET_OK;
}

bool microROS_createDiagnosticsPublisher(const char *topic_name)
{
    if (g_state != MICRO_ROS_STATE_CONNECTED)
    {
        setError("Not connected");
        return false;
    }

    if (rclc_publisher_init_default(&g_pub_diagnostics, &g_node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
                                    topic_name) != RCL_RET_OK)
    {
        setError("Failed to create diagnostics publisher");
        return false;
    }

    g_diagnostics_publisher_created = true;
    debugPrint("Diagnostics publisher created");
    return true;
}

bool microROS_publishDiagnostics(const diagnostic_data_t *diag_data)
{
    if (!g_diagnostics_publisher_created)
    {
        setError("Diagnostics publisher not created");
        return false;
    }

    // Fill timestamp
    microROS_usToRosTime(microROS_getCurrentTimeUs(), &g_msg_diagnostics.header.stamp);

    // This is a simplified implementation - in practice you'd need to properly
    // allocate and manage the diagnostic status arrays
    // For now, assume the message arrays are properly sized

    return rcl_publish(&g_pub_diagnostics, &g_msg_diagnostics, NULL) == RCL_RET_OK;
}

bool microROS_createHeartbeatPublisher(const char *topic_name)
{
    if (g_state != MICRO_ROS_STATE_CONNECTED)
    {
        setError("Not connected");
        return false;
    }

    if (rclc_publisher_init_default(&g_pub_heartbeat, &g_node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                    topic_name) != RCL_RET_OK)
    {
        setError("Failed to create heartbeat publisher");
        return false;
    }

    g_heartbeat_publisher_created = true;
    debugPrint("Heartbeat publisher created");
    return true;
}

// ========================= Services =========================
bool microROS_createResetOdomService(const char *service_name)
{
    if (g_state != MICRO_ROS_STATE_CONNECTED)
    {
        setError("Not connected");
        return false;
    }

    if (rclc_service_init_default(&g_srv_reset_odom, &g_node,
                                  ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty),
                                  service_name) != RCL_RET_OK)
    {
        setError("Failed to create reset odom service");
        return false;
    }

    if (rclc_executor_add_service(&g_executor, &g_srv_reset_odom,
                                  &g_req_empty, &g_res_empty,
                                  resetOdomServiceCallback) != RCL_RET_OK)
    {
        setError("Failed to add reset odom service to executor");
        return false;
    }

    debugPrint("Reset odometry service created");
    return true;
}

// ========================= Utility Functions =========================

uint64_t microROS_getCurrentTimeUs()
{
    return (uint64_t)millis() * 1000ULL + (uint64_t)(micros() % 1000ULL);
}

void microROS_usToRosTime(uint64_t time_us, builtin_interfaces__msg__Time *ros_time)
{
    ros_time->sec = (int32_t)(time_us / 1000000ULL);
    ros_time->nanosec = (uint32_t)((time_us % 1000000ULL) * 1000ULL);
}

void microROS_fillTwist(geometry_msgs__msg__Twist *msg,
                        double linear_x, double linear_y, double angular_z)
{
    msg->linear.x = linear_x;
    msg->linear.y = linear_y;
    msg->linear.z = 0.0;
    msg->angular.x = 0.0;
    msg->angular.y = 0.0;
    msg->angular.z = angular_z;
}

void microROS_setDebug(bool enable)
{
    g_debug_enabled = enable;
}

const char *microROS_getLastError()
{
    return g_last_error;
}

void microROS_reconnect()
{
    debugPrint("Attempting reconnection...");
    g_state = MICRO_ROS_STATE_CONNECTING;

    // Clean up current connection
    microROS_shutdown();

    // Reinitialize
    if (initializeROS())
    {
        g_state = MICRO_ROS_STATE_CONNECTED;
        debugPrint("Reconnection successful");
    }
    else
    {
        g_state = MICRO_ROS_STATE_ERROR;
        setError("Reconnection failed");
    }
}

void microROS_shutdown()
{
    if (g_state == MICRO_ROS_STATE_DISCONNECTED)
    {
        return;
    }

    debugPrint("Shutting down micro-ROS...");

    // Clean up publishers
    if (g_odom_publisher_created)
    {
        rcl_ret_t rc = rcl_publisher_fini(&g_pub_odom, &g_node);
        if (rc != RCL_RET_OK)
        {
            setError("Failed to fini odometry publisher");
        }
        g_odom_publisher_created = false;
    }

    if (g_imu_publisher_created)
    {
        rcl_ret_t rc = rcl_publisher_fini(&g_pub_imu, &g_node);
        if (rc != RCL_RET_OK)
        {
            setError("Failed to fini IMU publisher");
        }
        g_imu_publisher_created = false;
    }

    if (g_joint_publisher_created)
    {
        rcl_ret_t rc = rcl_publisher_fini(&g_pub_joint_state, &g_node);
        if (rc != RCL_RET_OK)
        {
            setError("Failed to fini JointState publisher");
        }
        g_joint_publisher_created = false;
    }

    if (g_diagnostics_publisher_created)
    {
        rcl_ret_t rc = rcl_publisher_fini(&g_pub_diagnostics, &g_node);
        if (rc != RCL_RET_OK)
        {
            setError("Failed to fini Diagnostics publisher");
        }
        g_diagnostics_publisher_created = false;
    }

    if (g_heartbeat_publisher_created)
    {
        rcl_ret_t rc = rcl_publisher_fini(&g_pub_heartbeat, &g_node);
        if (rc != RCL_RET_OK)
        {
            setError("Failed to fini Heartbeat publisher");
        }
        g_heartbeat_publisher_created = false;
    }

    if (g_wheel_ticks_publisher_created)
    {
        rcl_ret_t rc = rcl_publisher_fini(&g_pub_wheel_ticks, &g_node);
        if (rc != RCL_RET_OK)
        {
            setError("Failed to fini wheel_ticks publisher");
        }
        g_wheel_ticks_publisher_created = false;
    }
    // Always fini the message if it was inited
    std_msgs__msg__Int32MultiArray__fini(&g_msg_wheel_ticks);
    g_wheel_ticks_len = 0;

    // Clean up subscribers (if they were created)
    // Note: You'd need to track their creation status similar to publishers

    // Clean up services (if they were created)
    // Note: You'd need to track their creation status similar to publishers

    // Clean up executor and node
    (void)rclc_executor_fini(&g_executor);
    rcl_ret_t rc_node = rcl_node_fini(&g_node);
    if (rc_node != RCL_RET_OK)
    {
        setError("Failed to fini node");
    }
    (void)rclc_support_fini(&g_support);

    // Free allocated memory
    if (g_msg_command.data.data)
    {
        free(g_msg_command.data.data);
        g_msg_command.data.data = nullptr;
    }
    if (g_msg_heartbeat.data.data)
    {
        free(g_msg_heartbeat.data.data);
        g_msg_heartbeat.data.data = nullptr;
    }

    g_state = MICRO_ROS_STATE_DISCONNECTED;
    debugPrint("micro-ROS shutdown complete");
}
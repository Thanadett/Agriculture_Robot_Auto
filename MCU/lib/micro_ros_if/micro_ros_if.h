#ifndef MICRO_ROS_IF_H
#define MICRO_ROS_IF_H

#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Message types
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/empty.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>

// Configuration constants
#define MICRO_ROS_MAX_SUBSCRIBERS 8
#define MICRO_ROS_MAX_PUBLISHERS 8
#define MICRO_ROS_MAX_SERVICES 4
#define MICRO_ROS_MAX_TIMERS 4
#define MICRO_ROS_EXECUTOR_HANDLES (MICRO_ROS_MAX_SUBSCRIBERS + MICRO_ROS_MAX_PUBLISHERS + MICRO_ROS_MAX_SERVICES + MICRO_ROS_MAX_TIMERS)

#define MICRO_ROS_HEARTBEAT_MS 1000
#define MICRO_ROS_ODOM_PUBLISH_MS 50   // 20Hz
#define MICRO_ROS_IMU_PUBLISH_MS 20    // 50Hz
#define MICRO_ROS_JOINT_PUBLISH_MS 100 // 10Hz
#define MICRO_ROS_DIAG_PUBLISH_MS 2000 // 0.5Hz

// Status enum
typedef enum
{
    MICRO_ROS_STATE_DISCONNECTED,
    MICRO_ROS_STATE_CONNECTING,
    MICRO_ROS_STATE_CONNECTED,
    MICRO_ROS_STATE_ERROR
} micro_ros_state_t;

// Callback function types
typedef void (*twist_callback_t)(const geometry_msgs__msg__Twist *msg);
typedef void (*estop_callback_t)(const std_msgs__msg__Bool *msg);
typedef void (*string_callback_t)(const std_msgs__msg__String *msg);

// Structure to hold odometry data
typedef struct
{
    double x, y, theta;    // Position and orientation
    double vx, vy, vtheta; // Velocities
    uint64_t timestamp_us; // Timestamp in microseconds
} odom_data_t;

// Structure to hold IMU data
typedef struct
{
    double accel_x, accel_y, accel_z;      // m/s^2
    double gyro_x, gyro_y, gyro_z;         // rad/s
    double quat_w, quat_x, quat_y, quat_z; // Quaternion
    uint64_t timestamp_us;                 // Timestamp in microseconds
} imu_data_t;

// Structure to hold encoder/joint data
typedef struct
{
    float position[4];    // Joint positions (rad or m)
    float velocity[4];    // Joint velocities (rad/s or m/s)
    float effort[4];      // Joint efforts (N⋅m or N)
    const char *names[4]; // Joint names
    uint8_t count;        // Number of joints
    uint64_t timestamp_us;
} joint_data_t;

// Diagnostic data structure
typedef struct
{
    const char *hardware_id;
    uint8_t level; // 0=OK, 1=WARN, 2=ERROR, 3=STALE
    const char *name;
    const char *message;
    const char *key_values[8]; // Pairs: key1, value1, key2, value2, ...
    uint8_t kv_count;          // Number of key-value pairs
} diagnostic_data_t;

// ========================= Public API =========================

/**
 * @brief Initialize micro-ROS interface
 * @param agent_ip IP address of micro-ROS agent (NULL for USB/Serial)
 * @param agent_port Port of micro-ROS agent (default: 8888)
 * @return true if initialization successful
 */
bool microROS_begin(const char *agent_ip = nullptr, uint16_t agent_port = 8888);

/**
 * @brief Main update loop - call this regularly (e.g., in main loop)
 */
void microROS_update();

/**
 * @brief Get current connection state
 * @return Current micro-ROS state
 */
micro_ros_state_t microROS_getState();

/**
 * @brief Check if micro-ROS is connected and ready
 * @return true if connected
 */
bool microROS_isConnected();

// ========================= Subscribers =========================

/**
 * @brief Subscribe to cmd_vel topic
 * @param callback Function to call when message received
 * @param topic_name Topic name (default: "cmd_vel")
 * @return true if subscription successful
 */
bool microROS_subscribeCmdVel(twist_callback_t callback, const char *topic_name = "cmd_vel");

/**
 * @brief Subscribe to string commands
 * @param callback Function to call when message received
 * @param topic_name Topic name (default: "robot_command")
 * @return true if subscription successful
 */
bool microROS_subscribeCommand(string_callback_t callback, const char *topic_name = "robot_command");

// ========================= Publishers =========================

/**
 * @brief Create encoder publisher
 * @param topic_name Topic name (default: "wheel_ticks")
 * @param frame_id Frame ID (default: "wheel_link")
 * @return true if publisher creation successful
 */
bool microROS_createWheelTicksPublisher(const char *topic_name = "wheel_ticks", size_t length = 4);

/**
 * @brief Publish Encoder data
 * @param wheel_ticks Encoder data to publish
 * @return true if publish successful
 */
bool microROS_publishWheelTicks(const int32_t *ticks, size_t length = 0);

// Convenience: publish 4 wheels in LF, RF, LR, RR order
bool microROS_publishWheelTicks4(int32_t lf, int32_t rf, int32_t lr, int32_t rr);

/**
 * @brief Create odometry publisher
 * @param topic_name Topic name (default: "odom")
 * @param frame_id Frame ID (default: "odom")
 * @param child_frame_id Child frame ID (default: "base_link")
 * @return true if publisher creation successful
 */
bool microROS_createOdomPublisher(const char *topic_name = "odom",
                                  const char *frame_id = "odom",
                                  const char *child_frame_id = "base_link");

/**
 * @brief Publish odometry data
 * @param odom_data Odometry data to publish
 * @return true if publish successful
 */
bool microROS_publishOdom(const odom_data_t *odom_data);

/**
 * @brief Create IMU publisher
 * @param topic_name Topic name (default: "imu/data")
 * @param frame_id Frame ID (default: "imu_link")
 * @return true if publisher creation successful
 */
bool microROS_createIMUPublisher(const char *topic_name = "imu/data",
                                 const char *frame_id = "imu_link");

/**
 * @brief Publish IMU data
 * @param imu_data IMU data to publish
 * @return true if publish successful
 */
bool microROS_publishIMU(const imu_data_t *imu_data);

/**
 * @brief Create joint state publisher
 * @param topic_name Topic name (default: "joint_states")
 * @return true if publisher creation successful
 */
bool microROS_createJointStatePublisher(const char *topic_name = "joint_states");

/**
 * @brief Publish joint state data
 * @param joint_data Joint data to publish
 * @return true if publish successful
 */
bool microROS_publishJointState(const joint_data_t *joint_data);

/**
 * @brief Create diagnostics publisher
 * @param topic_name Topic name (default: "diagnostics")
 * @return true if publisher creation successful
 */
bool microROS_createDiagnosticsPublisher(const char *topic_name = "diagnostics");

/**
 * @brief Publish diagnostic data
 * @param diag_data Diagnostic data to publish
 * @return true if publish successful
 */
bool microROS_publishDiagnostics(const diagnostic_data_t *diag_data);

/**
 * @brief Create heartbeat publisher (std_msgs/String)
 * @param topic_name Topic name (default: "robot_heartbeat")
 * @return true if publisher creation successful
 */
bool microROS_createHeartbeatPublisher(const char *topic_name = "robot_heartbeat");

// ========================= Services =========================
/**
 * @brief Create reset odometry service
 * @param service_name Service name (default: "reset_odometry")
 * @return true if service creation successful
 */
bool microROS_createResetOdomService(const char *service_name = "reset_odometry");

// ========================= Utility Functions =========================

/**
 * @brief Get current time in microseconds for ROS timestamps
 * @return Current time in microseconds
 */
uint64_t microROS_getCurrentTimeUs();

/**
 * @brief Convert microseconds to ROS time
 * @param time_us Time in microseconds
 * @param ros_time Pointer to builtin_interfaces__msg__Time to fill
 */
void microROS_usToRosTime(uint64_t time_us, builtin_interfaces__msg__Time *ros_time);

/**
 * @brief Helper to create twist message from linear/angular velocities
 * @param msg Pointer to twist message to fill
 * @param linear_x Linear velocity in x (m/s)
 * @param linear_y Linear velocity in y (m/s)
 * @param angular_z Angular velocity in z (rad/s)
 */
void microROS_fillTwist(geometry_msgs__msg__Twist *msg,
                        double linear_x, double linear_y, double angular_z);

/**
 * @brief Enable/disable debug output
 * @param enable true to enable debug prints
 */
void microROS_setDebug(bool enable);

/**
 * @brief Get last error message
 * @return Last error message string
 */
const char *microROS_getLastError();

/**
 * @brief Force reconnection attempt
 */
void microROS_reconnect();

/**
 * @brief Cleanup and shutdown micro-ROS
 */
void microROS_shutdown();

#endif // MICRO_ROS_IF_H
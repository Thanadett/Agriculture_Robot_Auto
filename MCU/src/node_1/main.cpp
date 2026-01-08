#ifdef Node1

#include <Arduino.h>
#include <math.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

#include "config.h"
#include "motor_driver.h"
#include "encoder_read.h"

// ===== IMU BNO055 + PID =====
#include "imu_bno055.h"
#include "pid_rate.h"

// ---------------- Build-time config ----------------
#ifndef ROS_DOMAIN_ID_MCU
#define ROS_DOMAIN_ID_MCU 69
#endif

#ifndef USE_INNER_PID
#define USE_INNER_PID 1
#endif

#define TOPIC_WHEEL_TICKS "wheel_ticks"
#define TOPIC_HEARTBEAT "robot_heartbeat"
#define TOPIC_CMD_VEL "cmd_vel"
#define TOPIC_IMU_DATA "imu/data"
#define TOPIC_PID_DEBUG "pid_debug"
#define TOPIC_JOY_RESET "joy_reset"

// ===== PID Tuning Parameters =====
#define YAW_RATE_DEADZONE 0.015f
#define PID_KP 0.8f
#define PID_KI 0.04f
#define PID_KD 0.002f
#define PID_I_CLAMP_MIN -0.05f
#define PID_I_CLAMP_MAX 0.05f
#define PID_OUT_MIN -3.0f
#define PID_OUT_MAX 3.0f
#define PID_D_LPF 5.0f

// ============================ Debug Macros ============================
#define DEBUG_ENABLED (g_state != AGENT_CONNECTED)

#define DEBUG_PRINT(x)       \
    do                       \
    {                        \
        if (DEBUG_ENABLED)   \
            Serial.print(x); \
    } while (0)
#define DEBUG_PRINTLN(x)       \
    do                         \
    {                          \
        if (DEBUG_ENABLED)     \
            Serial.println(x); \
    } while (0)
#define DEBUG_PRINTF(...)               \
    do                                  \
    {                                   \
        if (DEBUG_ENABLED)              \
            Serial.printf(__VA_ARGS__); \
    } while (0)

// ============================ Safety Macros ============================
#define RCCHECK(fn)                                                             \
    {                                                                           \
        rcl_ret_t rc_ = (fn);                                                   \
        if (rc_ != RCL_RET_OK)                                                  \
        {                                                                       \
            DEBUG_PRINTF("[RCL] Error %d at %s:%d\n", rc_, __FILE__, __LINE__); \
            rclErrorLoop();                                                     \
        }                                                                       \
    }
#define RCSOFTCHECK(fn)                                                              \
    {                                                                                \
        rcl_ret_t rc_ = (fn);                                                        \
        if (rc_ != RCL_RET_OK)                                                       \
        {                                                                            \
            DEBUG_PRINTF("[RCL] Soft error %d at %s:%d\n", rc_, __FILE__, __LINE__); \
        }                                                                            \
    }

#define EXECUTE_EVERY_N_MS(MS, X)     \
    do                                \
    {                                 \
        static uint32_t _ts = 0;      \
        uint32_t _now = uxr_millis(); \
        if (_now - _ts >= (MS))       \
        {                             \
            {                         \
                X;                    \
            }                         \
            _ts = _now;               \
        }                             \
    } while (0)

// ============================ micro-ROS State ==========================
enum AgentState
{
    WAITING_AGENT = 0,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};
static AgentState g_state = WAITING_AGENT;

// Core objects
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_timer_t timer_ctrl;

// Pub/Sub
static rcl_publisher_t pub_ticks;
static rcl_publisher_t pub_heartbeat;
static rcl_publisher_t pub_imu;
static rcl_publisher_t pub_pid_debug;
static rcl_subscription_t sub_cmd_vel;
static rcl_subscription_t sub_joy_reset;

// Messages
static std_msgs__msg__Float32MultiArray msg_ticks;
static std_msgs__msg__String msg_hb;
static sensor_msgs__msg__Imu msg_imu;
static geometry_msgs__msg__Twist msg_cmd_vel;
static std_msgs__msg__Bool msg_joy_reset;
static std_msgs__msg__Float32MultiArray msg_pid_debug;

// Time sync
static unsigned long long time_offset_ms = 0;
static uint32_t last_time_sync_ms = 0;

// ============================ Robot Modules ============================
static QuadEncoderReader enc;
static IMU_BNO055 g_imu;
static PIDRate g_pid_wz(PID_KP, PID_KI, PID_KD,
                        PID_OUT_MIN, PID_OUT_MAX,
                        PID_I_CLAMP_MIN, PID_I_CLAMP_MAX,
                        PID_D_LPF);

extern uint32_t last_cmd_ms;

static volatile float g_V_cmd = 0.0f;
static volatile float g_W_cmd = 0.0f;
static uint32_t last_fast_us = 0;

#ifndef TRACK_W_M
#ifdef WHEEL_SEP
#define TRACK_W_M WHEEL_SEP
#else
#define TRACK_W_M 0.365f
#endif
#endif

#ifndef WHEEL_RADIUS
#define WHEEL_RADIUS 0.0635f
#endif

#ifndef ENCODER_PPR_OUTPUT_DEFAULT
#define ENCODER_PPR_OUTPUT_DEFAULT 5940.0f
#endif

static inline float round2(float x)
{
    return roundf(x * 100.0f) / 100.0f;
}
static inline float m_to_cm_2f(float m)
{
    return round2(m * 100.0f);
}

// ============================ Forward Decls ============================
static void rclErrorLoop();
static bool createEntities();
static bool destroyEntities();
static void syncTime();

static void on_cmd_vel(const void *msgin);
static void on_timer(rcl_timer_t *timer, int64_t last_call_time);
static void on_joy_reset(const void *msgin);
static void fast_loop_200hz();
static void publish_imu_100hz();

// ============================ Callbacks ============================
static void on_joy_reset(const void *msgin)
{
    const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
    static bool prev = false;
    bool now = m->data;

    if (now && !prev)
    {
        enc.reset();
        g_pid_wz.reset();
        DEBUG_PRINTLN("[RESET] wheel encoders & PID cleared");
    }
    prev = now;
}

static void on_cmd_vel(const void *msgin)
{
    const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
    g_V_cmd = (float)m->linear.x;
    g_W_cmd = (float)m->angular.z;

#if !USE_INNER_PID
    // Open-loop mode: forward directly
    cmdVW_to_targets(g_V_cmd, g_W_cmd);
#endif

    last_cmd_ms = millis();
}

static void on_timer(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/)
{
    enc.update();
    motorDrive_update();

    // Publish wheel ticks
    if (msg_ticks.data.size >= 4)
    {
        msg_ticks.data.data[0] = m_to_cm_2f(enc.totalDistanceM(W_FL));
        msg_ticks.data.data[1] = m_to_cm_2f(enc.totalDistanceM(W_FR));
        msg_ticks.data.data[2] = m_to_cm_2f(enc.totalDistanceM(W_RL));
        msg_ticks.data.data[3] = m_to_cm_2f(enc.totalDistanceM(W_RR));
        RCSOFTCHECK(rcl_publish(&pub_ticks, &msg_ticks, NULL));
    }

    // Publish heartbeat
    static uint32_t hb_ts = 0;
    uint32_t now = millis();
    if (now - hb_ts >= 200U)
    {
        const char *hb = "OK";
        msg_hb.data.data = (char *)hb;
        msg_hb.data.size = strlen(hb);
        msg_hb.data.capacity = msg_hb.data.size + 1;
        RCSOFTCHECK(rcl_publish(&pub_heartbeat, &msg_hb, NULL));
        hb_ts = now;
    }
}

// ============================ Fast Control Loop (200Hz) ============================
static void fast_loop_200hz()
{
    const uint32_t now_us = micros();
    if (last_fast_us == 0)
    {
        last_fast_us = now_us;
        return;
    }
    uint32_t dt_us = now_us - last_fast_us;
    if (dt_us < 5000U)
        return; // 200Hz = 5ms
    last_fast_us = now_us;

    const float dt = dt_us * 1e-6f;

    const uint32_t CMD_TIMEOUT_MS = 300;
    bool cmd_active = (millis() - last_cmd_ms) < CMD_TIMEOUT_MS;

    if (!cmd_active)
    {
        g_pid_wz.reset();
        cmdVW_to_targets(0.0f, 0.0f);
        return;
    }

    // 1) Read IMU yaw rate
    g_imu.update();
    float wz_actual = g_imu.wz; // rad/s from gyro

#if USE_INNER_PID
    // 2) Apply deadzone to actual rate
    float wz_filtered = wz_actual;
    if (fabs(wz_actual) < YAW_RATE_DEADZONE)
    {
        wz_filtered = 0.0f;
    }

    // 3) PID: compute correction
    float u = g_pid_wz.step(g_W_cmd, wz_filtered, dt);

    // 4) Add correction to commanded rate
    float W_eff = g_W_cmd + u;

    // 5) Clamp output
    if (W_eff > PID_OUT_MAX)
        W_eff = PID_OUT_MAX;
    if (W_eff < PID_OUT_MIN)
        W_eff = PID_OUT_MIN;

    // 6) Send to motors
    cmdVW_to_targets(g_V_cmd, W_eff);

    // 7) Debug publish ~40Hz
    static uint32_t dbg_last_ms = 0;
    const uint32_t now_ms = millis();
    if (g_state == AGENT_CONNECTED && (now_ms - dbg_last_ms) >= 40U)
    {
        if (msg_pid_debug.data.size >= 4)
        {
            msg_pid_debug.data.data[0] = g_W_cmd;     // setpoint
            msg_pid_debug.data.data[1] = wz_filtered; // actual (filtered)
            msg_pid_debug.data.data[2] = u;           // PID output
            msg_pid_debug.data.data[3] = W_eff;       // final command
            RCSOFTCHECK(rcl_publish(&pub_pid_debug, &msg_pid_debug, NULL));
        }
        dbg_last_ms = now_ms;
    }
#else
    // Open-loop: already handled in on_cmd_vel
#endif
}

// ============================ IMU Publishing (100Hz) ============================
static void publish_imu_100hz()
{
    static uint32_t last_imu_ms = 0;
    const uint32_t now = millis();

    if (g_state != AGENT_CONNECTED)
        return;
    if (now - last_imu_ms < 10)
        return; // 100Hz

    last_imu_ms = now;

    // Fill timestamp
    unsigned long long ros_time_ms = now + time_offset_ms;
    msg_imu.header.stamp.sec = (int32_t)(ros_time_ms / 1000ULL);
    msg_imu.header.stamp.nanosec = (uint32_t)((ros_time_ms % 1000ULL) * 1000000ULL);

    // Frame ID
    const char *frame = "imu_link";
    msg_imu.header.frame_id.data = (char *)frame;
    msg_imu.header.frame_id.size = strlen(frame);
    msg_imu.header.frame_id.capacity = msg_imu.header.frame_id.size + 1;

    // Orientation (quaternion)
    msg_imu.orientation.x = g_imu.qx;
    msg_imu.orientation.y = g_imu.qy;
    msg_imu.orientation.z = g_imu.qz;
    msg_imu.orientation.w = g_imu.qw;

    // Angular velocity (rad/s)
    msg_imu.angular_velocity.x = g_imu.wx;
    msg_imu.angular_velocity.y = g_imu.wy;
    msg_imu.angular_velocity.z = g_imu.wz;

    // Linear acceleration (m/s²)
    msg_imu.linear_acceleration.x = g_imu.ax;
    msg_imu.linear_acceleration.y = g_imu.ay;
    msg_imu.linear_acceleration.z = g_imu.az;

    // Covariance (BNO055 datasheet values)
    // Orientation covariance
    for (int i = 0; i < 9; i++)
        msg_imu.orientation_covariance[i] = 0.0;
    msg_imu.orientation_covariance[0] = 0.0001; // x
    msg_imu.orientation_covariance[4] = 0.0001; // y
    msg_imu.orientation_covariance[8] = 0.0001; // z

    // Angular velocity covariance
    for (int i = 0; i < 9; i++)
        msg_imu.angular_velocity_covariance[i] = 0.0;
    msg_imu.angular_velocity_covariance[0] = 0.0004;
    msg_imu.angular_velocity_covariance[4] = 0.0004;
    msg_imu.angular_velocity_covariance[8] = 0.0004;

    // Linear acceleration covariance
    for (int i = 0; i < 9; i++)
        msg_imu.linear_acceleration_covariance[i] = 0.0;
    msg_imu.linear_acceleration_covariance[0] = 0.01;
    msg_imu.linear_acceleration_covariance[4] = 0.01;
    msg_imu.linear_acceleration_covariance[8] = 0.01;

    RCSOFTCHECK(rcl_publish(&pub_imu, &msg_imu, NULL));
}

// ============================ Setup ============================
void setup()
{
    Serial.begin(115200);
    delay(100);
    DEBUG_PRINTLN("\n[BOOT] ESP32 starting...");

#ifndef MICROROS_WIFI
    set_microros_serial_transports(Serial);
    DEBUG_PRINTLN("[MICROROS] Serial transport set");
#else
#error "Wi-Fi transport not configured"
#endif

    // Hardware init
    motorDrive_begin();
    enc.begin(true);
    enc.setInvert(ENC_INV_FL < 0, ENC_INV_FR < 0, ENC_INV_RL < 0, ENC_INV_RR < 0);
    enc.setWheelRadius(ENC_WHEEL_RADIUS);
    enc.setPPR(ENCODER_PPR_OUTPUT_DEFAULT);

    // IMU init
    if (!g_imu.begin())
    {
        DEBUG_PRINTLN("[IMU] BNO055 init FAILED (check wiring)");
    }
    else
    {
        DEBUG_PRINTLN("[IMU] BNO055 init OK");
    }

#if USE_INNER_PID
    g_pid_wz.setGains(PID_KP, PID_KI, PID_KD);
    g_pid_wz.setIClamp(PID_I_CLAMP_MIN, PID_I_CLAMP_MAX);
    g_pid_wz.setOutputClamp(PID_OUT_MIN, PID_OUT_MAX);
    g_pid_wz.setDLpf(PID_D_LPF);
    g_pid_wz.reset();

    g_V_cmd = 0.0f;
    g_W_cmd = 0.0f;

    DEBUG_PRINTLN("[PID] Inner yaw-rate controller ENABLED");
    DEBUG_PRINTF("[PID] Kp=%.2f, Ki=%.3f, Kd=%.3f\n", PID_KP, PID_KI, PID_KD);
    DEBUG_PRINTF("[PID] Out=[%.1f,%.1f], I=[%.2f,%.2f], D_LPF=%.1f\n",
                 PID_OUT_MIN, PID_OUT_MAX, PID_I_CLAMP_MIN, PID_I_CLAMP_MAX, PID_D_LPF);
    DEBUG_PRINTF("[DEADZONE] Rate=%.3f rad/s\n", YAW_RATE_DEADZONE);
#else
    DEBUG_PRINTLN("[PID] Inner rate controller DISABLED (open-loop)");
#endif

    syncTime();
    last_time_sync_ms = millis();

    DEBUG_PRINTF("[INFO] Domain: %d\n", ROS_DOMAIN_ID_MCU);
    DEBUG_PRINTF("[INFO] Topics: %s, %s, %s, %s, %s\n",
                 TOPIC_WHEEL_TICKS, TOPIC_HEARTBEAT, TOPIC_CMD_VEL,
                 TOPIC_IMU_DATA, TOPIC_PID_DEBUG);
}

// ============================ Main Loop ============================
void loop()
{
    // Fast control loop (200Hz)
    fast_loop_200hz();

    // IMU publishing (100Hz)
    publish_imu_100hz();

    // Periodic time sync
    if (millis() - last_time_sync_ms > 10000U)
    {
        syncTime();
        last_time_sync_ms = millis();
    }

    // State machine
    switch (g_state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, {
            rmw_ret_t pr = rmw_uros_ping_agent(500, 4);
            if (pr == RMW_RET_OK)
            {
                g_state = AGENT_AVAILABLE;
                DEBUG_PRINTLN("[PING] agent available");
            }
            else
            {
                DEBUG_PRINTLN("[PING] no agent");
            }
        });
        break;

    case AGENT_AVAILABLE:
        DEBUG_PRINTLN("[STATE] AGENT_AVAILABLE -> creating entities...");
        if (createEntities())
        {
            g_state = AGENT_CONNECTED;
            DEBUG_PRINTLN("[STATE] AGENT_CONNECTED");
        }
        else
        {
            DEBUG_PRINTLN("[ERR] createEntities failed");
            destroyEntities();
            g_state = WAITING_AGENT;
        }
        break;

    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(300, {
            if (RMW_RET_OK != rmw_uros_ping_agent(300, 3))
            {
                DEBUG_PRINTLN("[WARN] agent lost");
                g_state = AGENT_DISCONNECTED;
            }
        });
        if (g_state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
        }
        break;

    case AGENT_DISCONNECTED:
        cmdVW_to_targets(0.f, 0.f);
        motorDrive_update();
        destroyEntities();
        g_state = WAITING_AGENT;
        DEBUG_PRINTLN("[STATE] DISCONNECTED -> WAITING_AGENT");
        break;
    }

    // Local serial command
    if (g_state != AGENT_CONNECTED)
    {
        motorDrive_handleSerialOnce();
    }
}

// ============================ Entity Management ============================
static bool createEntities()
{
    allocator = rcl_get_default_allocator();

    rcl_init_options_t init_opts = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_opts, allocator) != RCL_RET_OK)
        return false;

    rcl_ret_t drc = rcl_init_options_set_domain_id(&init_opts, ROS_DOMAIN_ID_MCU);
    if (drc != RCL_RET_OK)
    {
        DEBUG_PRINTF("[ERR] set_domain_id rc=%d\n", drc);
    }

    if (rclc_support_init_with_options(&support, 0, NULL, &init_opts, &allocator) != RCL_RET_OK)
    {
        rcl_init_options_fini(&init_opts);
        return false;
    }
    rcl_init_options_fini(&init_opts);

    // Node
    if (rclc_node_init_default(&node, "esp32_base", "", &support) != RCL_RET_OK)
    {
        return false;
    }
    DEBUG_PRINTLN("[INIT] node esp32_base created");

    // Publisher: wheel_ticks
    if (rclc_publisher_init_default(&pub_ticks, &node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                    TOPIC_WHEEL_TICKS) != RCL_RET_OK)
    {
        return false;
    }
    rosidl_runtime_c__float32__Sequence__init(&msg_ticks.data, 4);
    for (int i = 0; i < 4; i++)
        msg_ticks.data.data[i] = 0.0f;
    DEBUG_PRINTLN("[INIT] pub wheel_ticks");

    // Publisher: heartbeat
    if (rclc_publisher_init_default(&pub_heartbeat, &node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                    TOPIC_HEARTBEAT) != RCL_RET_OK)
    {
        return false;
    }
    DEBUG_PRINTLN("[INIT] pub heartbeat");

    // Publisher: imu/data
    if (rclc_publisher_init_default(&pub_imu, &node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                    TOPIC_IMU_DATA) != RCL_RET_OK)
    {
        return false;
    }
    DEBUG_PRINTLN("[INIT] pub imu/data");

    // Publisher: pid_debug
    if (rclc_publisher_init_default(&pub_pid_debug, &node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                    TOPIC_PID_DEBUG) != RCL_RET_OK)
    {
        return false;
    }
    rosidl_runtime_c__float32__Sequence__init(&msg_pid_debug.data, 4);
    for (int i = 0; i < 4; i++)
        msg_pid_debug.data.data[i] = 0.0f;
    DEBUG_PRINTLN("[INIT] pub pid_debug");

    // Subscriber: cmd_vel
    if (rclc_subscription_init_default(&sub_cmd_vel, &node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                       TOPIC_CMD_VEL) != RCL_RET_OK)
    {
        return false;
    }
    DEBUG_PRINTLN("[INIT] sub cmd_vel");

    // Subscriber: joy_reset
    if (rclc_subscription_init_default(&sub_joy_reset, &node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                       TOPIC_JOY_RESET) != RCL_RET_OK)
    {
        return false;
    }
    DEBUG_PRINTLN("[INIT] sub joy_reset");

    // Timer: 20ms (50Hz)
    const uint32_t CTRL_MS = 20;
    if (rclc_timer_init_default(&timer_ctrl, &support, RCL_MS_TO_NS(CTRL_MS), on_timer) != RCL_RET_OK)
    {
        return false;
    }
    DEBUG_PRINTLN("[INIT] timer 20ms");

    // Executor
    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK)
    {
        return false;
    }
    rclc_executor_add_timer(&executor, &timer_ctrl);
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, on_cmd_vel, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_joy_reset, &msg_joy_reset, on_joy_reset, ON_NEW_DATA);
    DEBUG_PRINTLN("[INIT] executor ready");

    syncTime();
    return true;
}

static bool destroyEntities()
{
    rcl_subscription_fini(&sub_cmd_vel, &node);
    rcl_subscription_fini(&sub_joy_reset, &node);
    rcl_publisher_fini(&pub_pid_debug, &node);
    rcl_publisher_fini(&pub_imu, &node);
    rcl_publisher_fini(&pub_heartbeat, &node);
    rcl_publisher_fini(&pub_ticks, &node);

    if (msg_ticks.data.data)
        rosidl_runtime_c__float__Sequence__fini(&msg_ticks.data);
    if (msg_pid_debug.data.data)
        rosidl_runtime_c__float__Sequence__fini(&msg_pid_debug.data);

    rcl_timer_fini(&timer_ctrl);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    DEBUG_PRINTLN("[CLEAN] entities destroyed");
    return true;
}

// ============================ Time Sync ============================
static void syncTime()
{
    RCSOFTCHECK(rmw_uros_sync_session(10));
    unsigned long now_ms = millis();
    unsigned long long agent_ms = rmw_uros_epoch_millis();
    time_offset_ms = (agent_ms > 0ULL) ? (agent_ms - now_ms) : 0ULL;
    DEBUG_PRINTF("[TIME] offset=%llu ms\n", time_offset_ms);
}

static void rclErrorLoop()
{
    DEBUG_PRINTLN("[RCL] Fatal error -> restart");
    delay(200);
    ESP.restart();
}

#endif // Node1
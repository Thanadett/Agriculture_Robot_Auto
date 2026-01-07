#include <Arduino.h>
#include <micro_ros_platformio.h>
extern "C" { 
  #include <rmw_microros/rmw_microros.h> 
}

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <std_msgs/msg/multi_array_dimension.h>
#include <rosidl_runtime_c/string_functions.h>

#include "encoder_read.h"
#include "encoder_ros_pub.h"

// ===== เดิมของคุณ =====
extern QuadEncoderReader enc4;

static rcl_allocator_t      g_alloc;
static rclc_support_t       g_support;
static rcl_node_t           g_node;
static rcl_publisher_t      g_pub_js;
static rcl_publisher_t      g_pub_total;
static rcl_timer_t          g_timer;
static rclc_executor_t      g_exec;

static sensor_msgs__msg__JointState     g_msg_js;
static std_msgs__msg__Float32MultiArray g_msg_total;

static double g_pos[4];
static double g_vel[4];
static float  g_total[4];

static uint32_t pub_fail_js = 0, pub_fail_total = 0;

// ===== เพิ่ม state สำหรับ watchdog =====
enum AgentState : uint8_t { AGENT_DISCONNECTED = 0, AGENT_CONNECTING, AGENT_CONNECTED };
static AgentState   g_agent_state = AGENT_DISCONNECTED;
static uint32_t     g_last_ping_ms = 0;
static uint32_t     g_last_reconnect_attempt_ms = 0;

// ปรับได้ตามต้องการ
static const uint32_t PING_INTERVAL_MS     = 1000;  // ping ทุก 1 วินาที
static const uint32_t RECONNECT_COOLDOWN_MS= 2000;  // เว้นระยะก่อนลองใหม่ 2 วิ
static const uint32_t PING_TIMEOUT_MS      = 100;   // ping timeout 100ms
static const uint32_t PING_ATTEMPTS        = 3;     // ลอง 3 ครั้งต่อรอบ

// flags สำหรับติดตามว่า entity ไหนสร้างสำเร็จแล้ว
static bool g_support_ok   = false;
static bool g_node_ok      = false;
static bool g_pub_js_ok    = false;
static bool g_pub_total_ok = false;
static bool g_timer_ok     = false;
static bool g_exec_ok      = false;

// ===== utils: ปิด/คืนทรัพยากรทั้งหมดอย่างปลอดภัย =====
static void enc_ros_fini_all()
{
  rcl_ret_t rc; (void)rc;

  if (g_exec_ok)   { rc = rclc_executor_fini(&g_exec);         g_exec_ok = false; }
  if (g_timer_ok)  { rc = rcl_timer_fini(&g_timer);            g_timer_ok = false; }
  if (g_pub_js_ok) { rc = rcl_publisher_fini(&g_pub_js, &g_node);   g_pub_js_ok = false; }
  if (g_pub_total_ok) { rc = rcl_publisher_fini(&g_pub_total, &g_node); g_pub_total_ok = false; }
  if (g_node_ok)   { rc = rcl_node_fini(&g_node);              g_node_ok = false; }
  if (g_support_ok){ rc = rclc_support_fini(&g_support);       g_support_ok = false; }

  // เคลียร์โครงสร้าง ป้องกันค่าเก่า
  memset(&g_exec,    0, sizeof(g_exec));
  memset(&g_timer,   0, sizeof(g_timer));
  memset(&g_pub_js,  0, sizeof(g_pub_js));
  memset(&g_pub_total,0, sizeof(g_pub_total));
  memset(&g_node,    0, sizeof(g_node));
  memset(&g_support, 0, sizeof(g_support));
}

// ===== utils: ตรวจสอบว่า layout ของ total ถูก init แล้วหรือยัง =====
static void ensure_total_layout_initialized() {
  // ถ้า dim ยังไม่พร้อม (size != 1) ให้ init ใหม่
  if (g_msg_total.layout.dim.size != 1) {
    // เคลียร์เดิมก่อน (กันหลุดหน่วยความจำถ้าเคย init)
    std_msgs__msg__Float32MultiArray__fini(&g_msg_total);
    std_msgs__msg__Float32MultiArray__init(&g_msg_total);

    // สร้าง 1 มิติชื่อ "wheel"
    std_msgs__msg__MultiArrayDimension__Sequence__init(&g_msg_total.layout.dim, 1);
    rosidl_runtime_c__String__assign(&g_msg_total.layout.dim.data[0].label, "wheel");
    g_msg_total.layout.dim.data[0].size   = 4;  // 4 ล้อ
    g_msg_total.layout.dim.data[0].stride = 4;  // 1D array → stride = size
    g_msg_total.layout.data_offset        = 0;  // เริ่มอ่านจาก index 0
  }
}

// ====== timer callback เดิม ======
static void timer_cb(rcl_timer_t * timer, int64_t) {
  (void)timer;

  // 1) บังคับให้ layout พร้อมทุกครั้ง
  ensure_total_layout_initialized();

  // 2) อัปเดตตำแหน่ง/ความเร็ว/ระยะตามเดิม
  g_pos[W_FL] = (double)enc4.positionRad(W_FL);
  g_pos[W_FR] = (double)enc4.positionRad(W_FR);
  g_pos[W_RL] = (double)enc4.positionRad(W_RL);
  g_pos[W_RR] = (double)enc4.positionRad(W_RR);

  g_vel[W_FL] = (double)enc4.velocityRadPerSec(W_FL);
  g_vel[W_FR] = (double)enc4.velocityRadPerSec(W_FR);
  g_vel[W_RL] = (double)enc4.velocityRadPerSec(W_RL);
  g_vel[W_RR] = (double)enc4.velocityRadPerSec(W_RR);

  g_total[W_FL] = enc4.totalDistanceM(W_FL);
  g_total[W_FR] = enc4.totalDistanceM(W_FR);
  g_total[W_RL] = enc4.totalDistanceM(W_RL);
  g_total[W_RR] = enc4.totalDistanceM(W_RR);

  // เติม timestamp ให้ JointState (ตามที่คุณทำ)
  int64_t ns = rmw_uros_epoch_nanos();
  if (ns > 0) {
    g_msg_js.header.stamp.sec     = (int32_t)(ns / 1000000000LL);
    g_msg_js.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
  } else {
    g_msg_js.header.stamp.sec = 0;
    g_msg_js.header.stamp.nanosec = 0;
  }

  // map buffer
  g_msg_js.position.data = g_pos; g_msg_js.position.size = 4; g_msg_js.position.capacity = 4;
  g_msg_js.velocity.data = g_vel; g_msg_js.velocity.size = 4; g_msg_js.velocity.capacity = 4;
  // effort คุณตั้งไว้เป็น 4 ช่องแล้วใน begin_serial()

  g_msg_total.data.data = g_total; g_msg_total.data.size = 4; g_msg_total.data.capacity = 4;

  // publish
  rcl_ret_t rc1 = rcl_publish(&g_pub_js, &g_msg_js, nullptr);
  if (rc1 != RCL_RET_OK) { ++pub_fail_js; }

  rcl_ret_t rc2 = rcl_publish(&g_pub_total, &g_msg_total, nullptr);
  if (rc2 != RCL_RET_OK) { ++pub_fail_total; }
}


void enc_microros_begin_serial()
{
  g_alloc = rcl_get_default_allocator();

  // ---- กำหนด QoS เอง ----
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;   // ส่งให้ครบทุกข้อความ
  qos.durability  = RMW_QOS_POLICY_DURABILITY_VOLATILE;    // ไม่เก็บ backlog
  qos.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos.depth       = 10;

  rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();
  pub_opt.qos = qos;

  // ---- Domain ID = 69 ----
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, g_alloc) != RCL_RET_OK) return;
  if (rcl_init_options_set_domain_id(&init_options, 69) != RCL_RET_OK) {
    (void) rcl_init_options_fini(&init_options);
    return;
  }

  if (rclc_support_init_with_options(&g_support, 0, NULL, &init_options, &g_alloc) != RCL_RET_OK) {
    (void) rcl_init_options_fini(&init_options);
    return;
  }
  g_support_ok = true;
  (void) rcl_init_options_fini(&init_options);

  // ---- Node ----
  if (rclc_node_init_default(&g_node, "esp32_encoder_node", "", &g_support) != RCL_RET_OK) {
    enc_ros_fini_all(); return;
  }
  g_node_ok = true;

  // ---- Publishers ใช้ QoS ที่กำหนดเอง (RELIABLE, depth=10) ----
  if (rcl_publisher_init(
        &g_pub_js, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/enc/joint_states",
        &pub_opt) != RCL_RET_OK) { enc_ros_fini_all(); return; }
  g_pub_js_ok = true;

  if (rcl_publisher_init(
        &g_pub_total, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/enc/total",
        &pub_opt) != RCL_RET_OK) { enc_ros_fini_all(); return; }
  g_pub_total_ok = true;

  // ---- เคลียร์ message struct (กันขยะใน sequence) ----
  memset(&g_msg_js, 0, sizeof(g_msg_js));
  memset(&g_msg_total, 0, sizeof(g_msg_total));
  // ---------- เตรียมค่าเริ่มต้นให้ข้อความ ----------

  // JointState: init โครงสร้าง + ตั้ง name (4 ล้อ) + frame_id
  sensor_msgs__msg__JointState__init(&g_msg_js);

  // name[] = ["wheel_fl","wheel_fr","wheel_rl","wheel_rr"]
  rosidl_runtime_c__String__Sequence__init(&g_msg_js.name, 4);
  rosidl_runtime_c__String__assign(&g_msg_js.name.data[0], "wheel_fl");
  rosidl_runtime_c__String__assign(&g_msg_js.name.data[1], "wheel_fr");
  rosidl_runtime_c__String__assign(&g_msg_js.name.data[2], "wheel_rl");
  rosidl_runtime_c__String__assign(&g_msg_js.name.data[3], "wheel_rr");

  // frame_id = "base_link"
  rosidl_runtime_c__String__assign(&g_msg_js.header.frame_id, "base_link");

  // init ข้อความรวม
  std_msgs__msg__Float32MultiArray__init(&g_msg_total);

  // init sequence ของ dim ให้มี 1 element
  std_msgs__msg__MultiArrayDimension__Sequence__init(&g_msg_total.layout.dim, 1);

  // ตั้งค่าของ dim[0]
  rosidl_runtime_c__String__assign(&g_msg_total.layout.dim.data[0].label, "wheel");
  g_msg_total.layout.dim.data[0].size   = 4;
  g_msg_total.layout.dim.data[0].stride = 4;
  g_msg_total.layout.data_offset        = 0;

  static double eff[4] = {0,0,0,0};
  g_msg_js.effort.data = eff;
  g_msg_js.effort.size = 4;
  g_msg_js.effort.capacity = 4;


  // ---- Timer + Executor ----
  const unsigned period_ms = 100;  // 10 Hz
  if (rclc_timer_init_default(&g_timer, &g_support, RCL_MS_TO_NS(period_ms), timer_cb) != RCL_RET_OK) {
    enc_ros_fini_all(); return;
  }
  g_timer_ok = true;

  if (rclc_executor_init(&g_exec, &g_support.context, 1, &g_alloc) != RCL_RET_OK) {
    enc_ros_fini_all(); return;
  }
  g_exec_ok = true;

  if (rclc_executor_add_timer(&g_exec, &g_timer) != RCL_RET_OK) {
    enc_ros_fini_all(); return;
  }

  g_agent_state = AGENT_CONNECTED;
}



// ====== spin เดิม ======
void enc_microros_spin_some()
{
  if (g_agent_state == AGENT_CONNECTED) {
    rclc_executor_spin_some(&g_exec, RCL_MS_TO_NS(5));
  }
}

// ====== watchdog API ======
void enc_agent_watchdog_begin()
{
  g_agent_state = AGENT_DISCONNECTED;
  g_last_ping_ms = g_last_reconnect_attempt_ms = millis();
}

// คืนค่า true = agent ok, false = หลุด/กำลังเชื่อมใหม่
bool enc_agent_check_and_reconnect()
{
  uint32_t now = millis();

  // ping เป็นช่วง ๆ เฉพาะตอนเชื่อมแล้ว
  if (g_agent_state == AGENT_CONNECTED) {
    if (now - g_last_ping_ms >= PING_INTERVAL_MS) {
      g_last_ping_ms = now;
      if (rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) != RMW_RET_OK) {
        // หลุด: ปิดทรัพยากร แล้วเปลี่ยนสถานะ
        enc_ros_fini_all();
        g_agent_state = AGENT_DISCONNECTED;
        g_last_reconnect_attempt_ms = now;
      }
    }
    return true; // ยังเชื่อมอยู่
  }

  // ยังไม่เชื่อม: รอคูลดาวน์ก่อนลองใหม่ (กันสแปม)
  if (now - g_last_reconnect_attempt_ms < RECONNECT_COOLDOWN_MS) {
    return false; // ยังไม่ถึงเวลาลองใหม่
  }

  g_agent_state = AGENT_CONNECTING;
  g_last_reconnect_attempt_ms = now;

 
  set_microros_serial_transports(Serial);

  // ลอง ping สั้น ๆ ก่อนสร้าง entity
  if (rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) != RMW_RET_OK) {
    g_agent_state = AGENT_DISCONNECTED;
    return false;
  }

  // agent ตอบแล้ว → init entities
  enc_microros_begin_serial();
  // ตรวจง่าย ๆ: node ถูกสร้างหรือยัง
  if (g_node.impl != nullptr) {
    g_agent_state = AGENT_CONNECTED;
    g_last_ping_ms = now;
    return true;
  } else {
    g_agent_state = AGENT_DISCONNECTED;
    return false;
  }
}



#pragma once
#include <Arduino.h>

// ---------------- Robot Params ----------------
static constexpr float WHEEL_SEP = 0.365f;         // m
static constexpr float WHEEL_RADIUS = 0.0635f;     // m
static constexpr float MAX_OMEGA_FOR_FULL = 20.0f; // rad/s -> 100% PWM

static constexpr bool INVERT_LF = false;
static constexpr bool INVERT_LR = false;
static constexpr bool INVERT_RF = false;
static constexpr bool INVERT_RR = false;

//-------- Motor Driver Pins ----------------
#define LF_IN1 32 // Left Front IN1
#define LF_IN2 23 // Left Front IN2
#define LR_IN1 27 // Left Rear IN1
#define LR_IN2 26 // Left Rear IN2
#define RF_IN1 14 // Right Front IN1
#define RF_IN2 12 // Right Front IN2
#define RR_IN1 33 // Right Rear IN1
#define RR_IN2 25 // Right Rear IN2

// ---------------- LEDC PWM ----------------
static constexpr int PWM_FREQ_HZ = 20000; // 20kHz เงียบ
static constexpr int PWM_RES_BITS = 8;    // 8-bit (0..255)
static constexpr int PWM_MAX_DUTY = (1 << PWM_RES_BITS) - 1;

// 8 ช่องไม่ซ้ำกัน
static constexpr int CH_LF_IN1 = 0;
static constexpr int CH_LF_IN2 = 1;
static constexpr int CH_LR_IN1 = 2;
static constexpr int CH_LR_IN2 = 3;
static constexpr int CH_RF_IN1 = 4;
static constexpr int CH_RF_IN2 = 5;
static constexpr int CH_RR_IN1 = 6;
static constexpr int CH_RR_IN2 = 7;

// Safety / smoothness
static constexpr uint32_t CMD_TIMEOUT_MS = 2000; // ms
static constexpr float RAMP_STEP = 0.05f;
static constexpr float IDLE_DECAY = 0.85f;

/** -------- Encoder Configuration --------
 *  JGB37-520 + เกียร์ 270:1, PPR มอเตอร์ = 11 (ก่อนเกียร์)
 *  โค้ดหลักคูณ x4 จาก quad decoding
 */
#define ENCODER_PPR_MOTOR 11.0f
#define REDUCTION_RATIO 270.0f   // gearbox ratio
#define QUAD_MULTIPLIER 2.0f     // half-quad
#define ENC_WHEEL_RADIUS 0.0635f // m (D=0.127 m ~ 5")

/** ===== แก้ไข: Encoder Pins - ใช้พินที่ปลอดภัย =====
 *  หลีกเลี่ยง GPIO 0, 2, 4, 5, 15 (boot strap pins)
 *  แนะนำใช้: 13, 16-19, 34-39 (input-only สำหรับ 34-39)
 */

// Front Left - ใช้พินเดิม (ปลอดภัย)
#define ENC_FL_A 34
#define ENC_FL_B 35 // (input-only, ปลอดภัย)

// Rear Left - ใช้พินเดิม (ปลอดภัย)
#define ENC_RL_A 18
#define ENC_RL_B 36

// Front Right - ย้ายจาก GPIO4 → GPIO34/35
#define ENC_FR_A 13
#define ENC_FR_B 17

// Rear Right - ย้ายจาก GPIO2,5 → GPIO34,36
#define ENC_RR_A 16
#define ENC_RR_B 19

// Encoder direction correction (+1 / -1)
#define ENC_INV_FL (+1)
#define ENC_INV_FR (-1)
#define ENC_INV_RL (+1)
#define ENC_INV_RR (-1)

/** -------- IMU (MPU6050) -------- **/
#define SDA 21
#define SCL 22

// ---------- IMU Settings ----------
static constexpr uint8_t MPU6050_I2C_ADDR = 0x68; // AD0=GND
static constexpr float IMU_SAMPLE_HZ = 100.0f;    // ความถี่ฟังค่าจริง
static constexpr float BETA_MADGWICK = 0.08f;     // ค่ากำกับฟิวชัน

// Accel: ±2g, Gyro: ±250 dps
static constexpr float ACCEL_SENS_2G = 16384.0f; // LSB/g
static constexpr float GYRO_SENS_250 = 131.0f;   // LSB/(deg/s)

/** ===== เพิ่ม: Watchdog และ Serial Buffer ===== */
// เพิ่ม timeout สำหรับ serial communication
#define SERIAL_TIMEOUT_MS 100
#define SERIAL_BAUD_RATE 115200

// เพิ่ม buffer size สำหรับ micro-ROS
#define MICROROS_SERIAL_BUFFER_SIZE 512
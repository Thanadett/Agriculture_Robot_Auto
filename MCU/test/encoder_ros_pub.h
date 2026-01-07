#pragma once

// มีอยู่แล้ว
void enc_microros_begin_serial();
void enc_microros_spin_some();

// ใหม่: watchdog / reconnect API
// เรียกใน setup ครั้งเดียวเพื่อเริ่มตัวแปรภายใน 
void enc_agent_watchdog_begin();

// เรียกใน loop() บ่อยๆ:
// - จะ ping agent เป็นช่วง ๆ
// - ถ้าหลุดจะปิดทรัพยากรแล้วพยายามเชื่อมใหม่
// - คืนค่า true = เชื่อมอยู่ / false = หลุด 
bool enc_agent_check_and_reconnect();

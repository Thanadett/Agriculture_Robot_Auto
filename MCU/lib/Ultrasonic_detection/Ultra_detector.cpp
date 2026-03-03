#include "Ultra_detector.h"
#include <vector>
#include <algorithm>

// const int TRIG_PIN = 12; 
// const int ECHO_PIN = 34; // ขา 34 เป็น Input-only ใช้เป็น Echo ได้
const float MAX_CHANGE = 15.0; 

static float last_valid_distance = 0;

void setupUltra() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT); 
    
    while(Serial.available()) Serial.read();
    Serial.println("\n--- Sensor Started ---");
}

float readRawDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
    if (duration == 0) return -1.0; 
    return duration * 0.034 / 2.0;
}

float getStableDistance() {
    std::vector<float> samples;
    
    for (int i = 0; i < 7; i++) {
        float d = readRawDistance();
        if (d > 2.0 && d < 400.0) {
            samples.push_back(d);
        }
        // delay(25); 
    }

    if (samples.size() < 3) return last_valid_distance;

    std::sort(samples.begin(), samples.end());
    float current_median = samples[samples.size() / 2];

    // 3. Logic ป้องกันการแกว่ง (Spike Protection)
    if (last_valid_distance > 0) {
        float difference = abs(current_median - last_valid_distance);
        
        // ถ้าค่าใหม่ต่างจากค่าเดิมเกิน 3cm ในพริบตาเดียว (เช่น 15 ไป 20)
        // เราจะมองว่าเป็นสัญญาณรบกวน และไม่เปลี่ยนค่าตามทันที
        if (difference > 5.0) { 
            // ให้ยอมรับการเปลี่ยนแปลงครึ่งหนึ่งของระยะที่ต่างกัน เพื่อให้ค่าไล่ตามทันได้เร็วขึ้น
            current_median = last_valid_distance + (current_median - last_valid_distance) * 0.7;
        }
    }
    // if (last_valid_distance > 0) {
    //     if (abs(current_median - last_valid_distance) > MAX_CHANGE) {
    //         current_median = last_valid_distance + (current_median > last_valid_distance ? 1.0 : -1.0);
    //     }
    // }
    
    last_valid_distance = current_median;
    return current_median;
}

void loopUltra() {
    float distance = getStableDistance();
    
    if (distance > 0) {
        Serial.print("Distance: ");
        Serial.print(distance, 2);
        Serial.println(" cm");
    }
    delay(150); 
}
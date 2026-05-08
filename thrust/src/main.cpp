#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"

constexpr int PIN_SDA = 4;
constexpr int PIN_SCL = 5;
constexpr uint32_t PRINT_INTERVAL_MS = 20;

static MPU6050 imu(Wire);
static uint32_t last_print_ms = 0;

void setup() {
    Serial.begin(115200);
    const uint32_t deadline = millis() + 2000;
    while (!Serial && millis() < deadline) yield();

    if (!imu.autoInit(Serial, PIN_SDA, PIN_SCL)) {
        while (true) yield();
    }
}

void loop() {
    const uint32_t now = millis();
    if (now - last_print_ms < PRINT_INTERVAL_MS) return;
    last_print_ms = now;
    imu.streamCSV(Serial);
}

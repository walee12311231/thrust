#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "MPU6050.h"
#include "MAX31856.h"

// I2C bus for MPU6050
constexpr int PIN_SDA = 4;
constexpr int PIN_SCL = 5;

// Hardware SPI0 for MAX31856 (RP2040 default SPI0 pins)
// SCK=GP2, MISO=GP0 (RX), MOSI=GP3 (TX), CS=GP1 — disjoint from I2C0 (GP4/GP5)
constexpr int PIN_SPI_SCK  = 2;
constexpr int PIN_SPI_MISO = 0;
constexpr int PIN_SPI_MOSI = 3;
constexpr int PIN_TC_CS    = 1;

constexpr uint32_t PRINT_INTERVAL_MS = 20;  // 50 Hz

static MPU6050   imu(Wire);
static MAX31856  tc(PIN_TC_CS, &SPI);
static bool      tc_ok        = false;
static uint32_t  last_print_ms = 0;

void setup() {
    Serial.begin(115200);
    const uint32_t deadline = millis() + 2000;
    while (!Serial && millis() < deadline) yield();

    // Bring up MPU6050 on I2C0 (GP4/GP5)
    if (!imu.autoInit(Serial, PIN_SDA, PIN_SCL)) {
        while (true) yield();
    }

    // Bring up hardware SPI0 on GP2/GP0/GP3 with GP1 as CS
    SPI.setSCK(PIN_SPI_SCK);
    SPI.setRX(PIN_SPI_MISO);
    SPI.setTX(PIN_SPI_MOSI);
    SPI.begin();

    tc_ok = tc.begin();
    if (tc_ok) {
        tc.setThermocoupleType(MAX31856_TCTYPE_K);
        tc.setConversionMode(MAX31856_CONTINUOUS);
        Serial.println("MAX31856 OK");
    } else {
        Serial.println("MAX31856 init FAILED");
    }
}

void loop() {
    const uint32_t now = millis();
    if (now - last_print_ms < PRINT_INTERVAL_MS) return;
    last_print_ms = now;

    MPU6050::Sample s;
    if (!imu.read(s)) return;

    Serial.print(s.accel_g.x, 4);  Serial.print(',');
    Serial.print(s.accel_g.y, 4);  Serial.print(',');
    Serial.print(s.accel_g.z, 4);  Serial.print(',');
    Serial.print(s.gyro_dps.x, 3); Serial.print(',');
    Serial.print(s.gyro_dps.y, 3); Serial.print(',');
    Serial.print(s.gyro_dps.z, 3); Serial.print(',');

    if (tc_ok) {
        const float tc_temp = tc.readThermocoupleTemperature();
        Serial.println(tc_temp, 3);
    } else {
        Serial.println("nan");
    }
}

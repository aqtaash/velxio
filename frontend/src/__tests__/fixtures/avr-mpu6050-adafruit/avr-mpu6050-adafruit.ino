// Arduino Uno + one MPU-6050 at 0x68 on Wire (SDA = A4, SCL = A5), read with
// Adafruit_MPU6050: the setup and the loop of the gallery example
// esp32-mpu6050, on a board whose firmware runs in the tab.
//
// begin() resets the chip and waits for DEVICE_RESET to clear with no
// timeout, and getEvent() divides by the sensitivity of the range the sketch
// selected. So the driver itself tells whether the model clears the bit and
// follows the range: "BEGIN" with no "READY" after it is the wait that never
// ends, and AZ=39.23 is 1 g encoded for 2 g and read as 8 g.
//
// Serial protocol (115200): BEGIN, then READY (or NOT FOUND), then once per
// loop "AZ=<m/s2> GX=<rad/s> T=<deg C>".
//
// Libraries: Adafruit MPU6050 2.2.9, Adafruit BusIO 1.17.4,
//            Adafruit Unified Sensor 1.1.15
// Rebuild: node project/board-buses-2026-09/harness/compile-fixture.mjs \
//   --fqbn arduino:avr:uno \
//   --libs "Adafruit MPU6050,Adafruit Unified Sensor,Adafruit BusIO" \
//   --out <this dir> <this dir>/avr-mpu6050-adafruit.ino
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Serial.println("BEGIN");
  if (!mpu.begin()) {
    Serial.println("NOT FOUND");
    while (true) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("READY");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print("AZ=");
  Serial.print(a.acceleration.z);
  Serial.print(" GX=");
  Serial.print(g.gyro.x, 3);
  Serial.print(" T=");
  Serial.println(temp.temperature);
  delay(100);
}

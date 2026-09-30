// Arduino Uno + one MPU-6050 at 0x68 on Wire (SDA = A4, SCL = A5), read the
// way a first sketch often reads it: straight from ACCEL_XOUT_H, with
// PWR_MGMT_1 never written. The chip powers on asleep, so on the bench this
// prints zeros for as long as it runs, whatever way the board is held.
//
// Serial protocol (115200): BOOT, then once per loop "A=<x>,<y>,<z>" in
// counts.
//
// Rebuild: node project/board-buses-2026-09/harness/compile-fixture.mjs \
//   --fqbn arduino:avr:uno --out <this dir> <this dir>/avr-mpu6050-asleep.ino
#include <Wire.h>

#define MPU_ADDR 0x68

void setup() {
  Serial.begin(115200);
  Serial.println(F("BOOT"));
  Wire.begin();
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6);
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Serial.print(F("A="));
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.println(az);
  delay(50);
}

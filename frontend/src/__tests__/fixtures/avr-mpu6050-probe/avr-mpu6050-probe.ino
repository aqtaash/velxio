// Arduino Uno + one MPU-6050 at 0x68 on Wire (SDA = A4, SCL = A5), probed
// register by register with no library in the way. It is the probe the
// staging board matrix runs on every board (project i2c-model-fidelity-2026-09,
// harness/mk.py), so what passes here is what that matrix expects of the tab
// model.
//
// Serial protocol (115200), one line per probe:
//   P0 whoami=0x68
//   P1 pwr_at_boot=0x40            the chip powers on asleep
//   P2 after_reset pwr=0x40 polls=0  DEVICE_RESET is gone by the first read
//   P3 sig_path_reset=0x0          write-only
//   P4 user_ctrl=0x0               the reset bits clear themselves
//   P5 az_raw_at_8g=4096           1 g at AFS_SEL = 2, read a byte at a time
//   P6 int_status=0x0
//   PROBE_DONE
//
// Rebuild: node project/board-buses-2026-09/harness/compile-fixture.mjs \
//   --fqbn arduino:avr:uno --out <this dir> <this dir>/avr-mpu6050-probe.ino
#include <Wire.h>

static uint8_t rd(uint8_t r) {
  Wire.beginTransmission(0x68);
  Wire.write(r);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xEE;
}

static void wr(uint8_t r, uint8_t v) {
  Wire.beginTransmission(0x68);
  Wire.write(r);
  Wire.write(v);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin();
  Serial.print("P0 whoami=0x");
  Serial.println(rd(0x75), HEX);
  Serial.print("P1 pwr_at_boot=0x");
  Serial.println(rd(0x6B), HEX);

  wr(0x6B, 0x80);
  unsigned long t = millis();
  uint8_t v = 0x80;
  int n = 0;
  while (((v = rd(0x6B)) & 0x80) && millis() - t < 500) {
    n++;
    delay(1);
  }
  Serial.print("P2 after_reset pwr=0x");
  Serial.print(v, HEX);
  Serial.print(" polls=");
  Serial.println(n);

  wr(0x68, 0x07);
  Serial.print("P3 sig_path_reset=0x");
  Serial.println(rd(0x68), HEX);
  wr(0x6A, 0x07);
  Serial.print("P4 user_ctrl=0x");
  Serial.println(rd(0x6A), HEX);

  wr(0x6B, 0x00);
  wr(0x1C, 0x10);
  uint8_t h = rd(0x3F), l = rd(0x40);
  Serial.print("P5 az_raw_at_8g=");
  Serial.println((int16_t)((h << 8) | l));
  Serial.print("P6 int_status=0x");
  Serial.println(rd(0x3A), HEX);
  Serial.println("PROBE_DONE");
}

void loop() {
  delay(1000);
}

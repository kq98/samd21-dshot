# DSHOT for SAMD21 based boards

Control up to 4 brushless motors with DSHOT enabled ESCs.

This library requires the Arduino framework to work properly.
It is intended for use with the Arduino Zero.

The DSHOT documentation can be found at: https://betaflight.com/docs/development/api/dshot

Without an external quartz, as often found on clone boards, the clock is too unstable for DSHOT 600 mode or anything faster, so DSHOT 300 is the recommended mode.

## Example


```src/main.c

#include "samd21_dshot.h"

DSHOTSetpoint setpoints;

void setup() {

  // Select protocol speed 150, 300, 600, 1200 kbit/s
  DSHOTInit(DSHOT300);

  delay(100);

  // Send commands to set LEDs, motor direction etc.
  // Only possible when the drone is disarmed
  sendDSHOTCommand(2, dshot_frame[0])
  sendDSHOTCommand(1, dshot_frame[0])

  delay(300);

  // Arm the drone, throttle values will be accepted afterwards
  DSHOTArmDrone();

}

void loop() {

  // < Read Sensors >
  ...
  // < Calculate motor setpoints >
  ..

  // Apply setpoints
  setpoints.motor1 = val_for_m1;
  setpoints.motor2 = val_for_m2;
  setpoints.motor3 = val_for_m3;
  setpoints.motor4 = val_for_m4;

  DSHOTWrite(&setpoints);

}
```
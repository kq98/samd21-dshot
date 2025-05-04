# DSHOT for SAMD21 based boards

Control up to 4 brushless motors with DSHOT enabled ESCs.

This library requires the Arduino framework to work properly.
It is intended for use with the Arduino Zero.

The DSHOT documentation can be found at: https://betaflight.com/docs/development/api/dshot

Without an external quartz the clock is too unstable for DSHOT 600 or anything faster, so DSHOT 300 is the recommended mode.

## Pinout
| Motor Nr.  | SAMD21 Package pin | Arduino Zero pin  | 
| -- | -- | -- |
| 1 | PA08 | 4 (2 on clones) |
| 2 | PA09 | 3 |
| 3 | PA20 | 6 |
| 4 | PA21 | 7 |

## Install
The library is intended to be used with PlatformIO, just add this repo to your lib_deps and let PlatformIO do the work.
```
[env:zeroUSB]
;build_flags = 
;	-DCRYSTALLESS
platform = atmelsam
board = zeroUSB
framework = arduino
...
lib_deps = https://github.com/kq98/samd21-dshot.git
...
```

## Example


```src/main.c

#include "samd21_dshot.h"

DSHOT::Setpoint setpoints;

void setup() {

  // Select protocol speed 150, 300, 600, 1200 kbit/s
  DSHOT::Init(DSHOT::DSHOT300);

  delay(300);

  // Arm the drone, throttle values will be accepted afterwards
  DSHOT::ArmDrone();

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

  DSHOT::Write(&setpoints);

}
```
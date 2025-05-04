
#ifndef SAMD21_DSHOT_H
#define SAMD21_DSHOT_H


#include <Arduino.h>
#include <wiring_private.h>

#define DSHOT_NUM_MOTOR 4
#define DSHOT_FRAME_SIZE 16

static int _writeResolution = 8;

bool DSHOT_READY = false;
bool DSHOT_SEND_CMD = false;
int DSHOT_CMD_REPEAT_CNT = 0;

enum {
  DSHOT150,
  DSHOT300,
  DSHOT600,
  DSHOT1200
};

/*
  Based on: https://betaflight.com/docs/development/api/dshot

  Clock frequency is 48 MHz with external crystal and ~46.2222 MHz without

  DSHOTX_HIGH and DSHOTX_LOW values are the number of clk cycles to wait befor pulling a PWM signal from 1 to 0. 
  They are the compare values used in the TCC pwm module.

  DSHOTX_BIT_PERIOD is the period(also known as TOP) value used in the TCC pwm module.
  It is equivalent to the number of clk cycles of a single pwm period.

  DSHOTX_num_clk_cycles = clk_freq * time_to_wait

*/

#ifdef CRYSTALLESS
  const uint8_t DSHOT150_HIGH = 231;
  const uint8_t DSHOT150_LOW = 116;
  const uint16_t DSHOT150_BIT_PERIOD = 308;

  const uint8_t DSHOT300_HIGH = 116;
  const uint8_t DSHOT300_LOW = 58;
  const uint16_t DSHOT300_BIT_PERIOD = 154;

  const uint8_t DSHOT600_HIGH = 58;
  const uint8_t DSHOT600_LOW = 29;
  const uint16_t DSHOT600_BIT_PERIOD = 77;

  const uint8_t DSHOT1200_HIGH = 29;
  const uint8_t DSHOT1200_LOW = 14;
  const uint16_t DSHOT1200_BIT_PERIOD = 38;
#else
  const uint8_t DSHOT150_HIGH = 240;
  const uint8_t DSHOT150_LOW = 120;
  const uint16_t DSHOT150_BIT_PERIOD = 320;

  const uint8_t DSHOT300_HIGH = 120;
  const uint8_t DSHOT300_LOW = 60;
  const uint16_t DSHOT300_BIT_PERIOD = 160;

  const uint8_t DSHOT600_HIGH = 60;
  const uint8_t DSHOT600_LOW = 30;
  const uint16_t DSHOT600_BIT_PERIOD = 80;

  const uint8_t DSHOT1200_HIGH = 30;
  const uint8_t DSHOT1200_LOW = 15;
  const uint16_t DSHOT1200_BIT_PERIOD = 40;
#endif

uint8_t DSHOT_HIGH;
uint8_t DSHOT_LOW;
uint16_t DSHOT_BIT_PERIOD;

struct dmaDescriptor {
  uint16_t btctrl;
  uint16_t btcnt;
  uint32_t srcaddr;
  uint32_t dstaddr;
  uint32_t descaddr;
};

volatile dmaDescriptor dmaDescriptorArray[4] __attribute__ ((aligned (16)));
dmaDescriptor dmaDescriptorWritebackArray[4] __attribute__ ((aligned (16)));

uint8_t dshot_frame[DSHOT_NUM_MOTOR][DSHOT_FRAME_SIZE+1] __attribute__ ((aligned (16))) = {0};

struct DSHOTSetpoint {
  int motor1 = 0;
  int motor2 = 0;
  int motor3 = 0; 
  int motor4 = 0;
};

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_16(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_16(Tc* TCx);

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx);

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to);

void DSHOTAnalogWrite(int pin, int value, uint8_t group, uint8_t channel);

void enable_dma_channels();
void disable_dma_channels();

void DMAC_Handler();
void setupTCC();
void setupDMA();

uint16_t calcDSHOTFrame(uint16_t throttle, bool telemetry = false);
void writeDSHOTFrame(uint16_t value, uint8_t* target);

void sendDSHOTCommand(uint8_t cmd, uint8_t *target, uint16_t delay_val = 0, uint8_t num_repeat = 0);

void DSHOTArmDrone();
void DSHOTDisarmDrone();

void DSHOTWrite(const DSHOTSetpoint* setpoints);
int DSHOTInit(uint8_t DSHOT_MODE);

#endif
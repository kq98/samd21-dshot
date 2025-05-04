
#ifndef SAMD21_DSHOT_H
#define SAMD21_DSHOT_H


#include <Arduino.h>
#include <wiring_private.h>

#define DSHOT_NUM_MOTOR 4
#define DSHOT_FRAME_SIZE 16

enum {
  DSHOT150,
  DSHOT300,
  DSHOT600,
  DSHOT1200
};

struct DSHOTSetpoint {
  int motor1 = 0;
  int motor2 = 0;
  int motor3 = 0; 
  int motor4 = 0;
};

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

#include <Arduino.h>
#include <wiring_private.h>

#define NUM_MOTOR 4

static int _writeResolution = 8;

bool SEND_CMD = false;
int CMD_REPEAT_CNT = 0;

int counter[4] = {0, 0, 0, 0};

uint8_t DSHOT600_HIGH = 60;
uint8_t DSHOT600_LOW = 30;

struct dmaDescriptor {
  uint16_t btctrl;
  uint16_t btcnt;
  uint32_t srcaddr;
  uint32_t dstaddr;
  uint32_t descaddr;
};

uint8_t dshot_frame[NUM_MOTOR][17] __attribute__ ((aligned (16))) = {0};


volatile dmaDescriptor dmaDescriptorArray[4] __attribute__ ((aligned (16)));
dmaDescriptor dmaDescriptorWritebackArray[4] __attribute__ ((aligned (16)));

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_16(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_16(Tc* TCx) {
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from == to) {
    return value;
  }
  if (from > to) {
    return value >> (from-to);
  }
  return value << (to-from);
}

void myAnalogWrite(int pin, int value, uint8_t group, uint8_t channel) {

  PinDescription pinDesc = g_APinDescription[pin];
  uint32_t attr = pinDesc.ulPinAttribute;


  value = mapResolution(value, _writeResolution, 6);

  uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
  static bool tcEnabled[TCC_INST_NUM+TC_INST_NUM];

  if (attr & PIN_ATTR_TIMER) {
    #if !(ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10603)
    // Compatibility for cores based on SAMD core <=1.6.2
    if (pinDesc.ulPinType == PIO_TIMER_ALT) {
      pinPeripheral(pin, PIO_TIMER_ALT);
    } else
    #endif
    {
      pinPeripheral(pin, PIO_TIMER);
    }
  } else {
    // We suppose that attr has PIN_ATTR_TIMER_ALT bit set...
    pinPeripheral(pin, PIO_TIMER_ALT);
  }

  if (!tcEnabled[tcNum]) {
    tcEnabled[tcNum] = true;

    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC0_TCC1));
    while (GCLK->STATUS.bit.SYNCBUSY == 1);

    // Set PORT
    // -- Configure TCC
    Tcc* TCCx = TCC0; //(Tcc*) GetTC(pinDesc.ulPWMChannel);
    // Disable TCCx
    TCCx->CTRLA.bit.ENABLE = 0;
    syncTCC(TCCx);
    // Set TCCx as normal PWM
    TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
    syncTCC(TCCx);
    // Set the initial value
    TCCx->CC[channel].reg = (uint32_t) value;
    syncTCC(TCCx);
    // Set PER to maximum counter value (resolution : 0xFFFF)
    TCCx->PER.reg = 0x0050; //48 MHz * 1,67 us = 80,16 appr. = 80 = 0x50
    syncTCC(TCCx);
    // Enable TCCx
    TCCx->CTRLA.bit.ENABLE = 1;
    syncTCC(TCCx);
  } else {
      Tcc* TCCx = TCC0; //(Tcc*) GetTC(pinDesc.ulPWMChannel);
      TCCx->CTRLBSET.bit.LUPD = 1;
      syncTCC(TCCx);
      TCCx->CCB[channel].reg = (uint32_t) value;
      syncTCC(TCCx);
      TCCx->CTRLBCLR.bit.LUPD = 1;
      syncTCC(TCCx);
  }
}

void enable_dma_channels() {
  DMAC->CTRL.reg &= ~DMAC_CTRL_DMAENABLE;

  DMAC->CHID.reg = 0; // select channel 0
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  DMAC->CHID.reg = 1; // select channel 0
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  DMAC->CHID.reg = 2; // select channel 0
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  DMAC->CHID.reg = 3; // select channel 0
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  DMAC->CTRL.reg |= DMAC_CTRL_DMAENABLE;
}

void disable_dma_channels() {
  DMAC->CHID.reg = 0; // select channel 0
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;

  DMAC->CHID.reg = 1; // select channel 0
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;

  DMAC->CHID.reg = 2; // select channel 0
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;

  DMAC->CHID.reg = 3; // select channel 0
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
}

void DMAC_Handler() {
  // Must clear this flag! Otherwise the interrupt will be triggered over and over again.
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_MASK;

  if (SEND_CMD) {
    if (CMD_REPEAT_CNT == 0) {
      SEND_CMD = false;
      // TCC0->CCB[0].reg = (uint32_t) 0x00;
      // TCC0->CCB[1].reg = (uint32_t) 0x00;
      // TCC0->CCB[2].reg = (uint32_t) 0x00;
      // TCC0->CCB[3].reg = (uint32_t) 0x00;
    } else {
      CMD_REPEAT_CNT--;
      enable_dma_channels();
    }
  } else {
    // TCC0->CCB[0].reg = (uint32_t) 0x00;
    // TCC0->CCB[1].reg = (uint32_t) 0x00;
    // TCC0->CCB[2].reg = (uint32_t) 0x00;
    // TCC0->CCB[3].reg = (uint32_t) 0x00;
  }

}

void setupDMA() {

  for (int i = 0; i < NUM_MOTOR; i++) {
    dmaDescriptorArray[i].btctrl =  (1 << 0) |  // VALID: Descriptor Valid
                                    (0 << 3) |  // BLOCKACT=NOACT: Block Action
                                    (1 << 10) | // SRCINC: Source Address Increment Enable
                                    (0 << 11) | // DSTINC: Destination Address Increment Enable
                                    (1 << 12) | // STEPSEL=SRC: Step Selection
                                    (0 << 13);  // STEPSIZE=X1: Address Increment Step Size
    dmaDescriptorArray[i].btcnt = 17; // beat count
    dmaDescriptorArray[i].dstaddr = (uint32_t) &(TCC0->CCB[i].reg);
    dmaDescriptorArray[i].srcaddr = (uint32_t) &dshot_frame[i][17];
  }

  DMAC->BASEADDR.reg = (uint32_t)dmaDescriptorArray;
  DMAC->WRBADDR.reg = (uint32_t)dmaDescriptorWritebackArray;

  PM->AHBMASK.bit.DMAC_ = 1;
  PM->APBBMASK.bit.DMAC_ = 1;
  
  for (int i = 0; i < NUM_MOTOR; i++) {
    DMAC->CHID.reg = i; // select channel i
    DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0) | DMAC_CHCTRLB_TRIGSRC(TCC0_DMAC_ID_OVF) | DMAC_CHCTRLB_TRIGACT_BEAT;
  }

  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);

  // for (int i = 0; i < NUM_MOTOR; i++) {
  //   DMAC->CHID.reg = i;
  //   DMAC->CHINTENSET.reg = DMAC_CHINTENSET_TCMPL;
  // }

  // DMAC->CHID.reg = 3;
  // DMAC->CHINTENSET.reg = DMAC_CHINTENSET_TCMPL;
  // NVIC_EnableIRQ(DMAC_IRQn);

  // enable_dma_channels();

}

uint16_t calcDSHOTFrame(uint16_t throttle) {
  uint16_t value = 0;
  uint16_t crc = 0;

  if (throttle > 2047)
    return 0xFFFF;
  else {
    value = throttle << 1; // Telemetry = 0
    crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
    value = value << 4; // Shift thottle and telemetry into upper 12 bits to make space for crc
    value = value | crc;

    return value;
  }
}

void writeDSHOTFrame(uint16_t value, uint8_t* target) {
  for (int i = 0; i < 16; i++) {
    if ( ((1 << i) & value) == (1 << i)) 
      target[15-i] = DSHOT600_HIGH;
    else
      target[15-i] = DSHOT600_LOW;
  }

  target[16] = 0;
}

void sendCommand(uint8_t cmd, uint8_t *target, uint16_t delay_val = 0, uint8_t num_repeat = 0) {
  disable_dma_channels();
  writeDSHOTFrame(calcDSHOTFrame(cmd), target);
  CMD_REPEAT_CNT = num_repeat;
  SEND_CMD = true;
  enable_dma_channels();

  // delay(delay_val);
}

void arm_drone() {
  TCC0->CCB[0].reg = (uint32_t) DSHOT600_LOW;
  TCC0->CCB[1].reg = (uint32_t) DSHOT600_LOW;
  TCC0->CCB[2].reg = (uint32_t) DSHOT600_LOW;
  TCC0->CCB[3].reg = (uint32_t) DSHOT600_LOW;
  delay(300);
  TCC0->CCB[0].reg = (uint32_t) 0x00;
  TCC0->CCB[1].reg = (uint32_t) 0x00;
  TCC0->CCB[2].reg = (uint32_t) 0x00;
  TCC0->CCB[3].reg = (uint32_t) 0x00;
  delay(1);

}

void setup() {

  // SerialUSB.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  myAnalogWrite(4, 0, PORT_PMUX_PMUXE_E, 0); //PA08
  myAnalogWrite(3, 0, PORT_PMUX_PMUXO_F, 1); //PA09
  myAnalogWrite(6, 0, PORT_PMUX_PMUXE_F, 2); //PA20
  myAnalogWrite(7, 0, PORT_PMUX_PMUXO_F, 3); //PA21

  setupDMA();

  delay(100);

  sendCommand(21, dshot_frame[0], 0, 6);

  delay(300);
  arm_drone();

  // // sendCommand(200, dshotFrames.motor1, 1);
  // // sendCommand(200, dshotFrames.motor1, 1);
  // // sendCommand(200, dshotFrames.motor1, 1);

  // // writeDSHOTFrame(calcDSHOTFrame(100), dshotFrames.motor1);
  // // DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  // // delay(500);

  // // // writeDSHOTFrame(calcDSHOTFrame(100), dshotFrames.motor1);
  // // // DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  // // // delay(500);

  // // // writeDSHOTFrame(calcDSHOTFrame(48), dshotFrames.motor1);
  // // // DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  // // DMA_DISABLE = false;

  // delay(10);

  // writeDSHOTFrame(calcDSHOTFrame(100), dshotFrames.motor1);
  // writeDSHOTFrame(calcDSHOTFrame(50), dshotFrames.motor2);
  // writeDSHOTFrame(calcDSHOTFrame(200), dshotFrames.motor3);
  // writeDSHOTFrame(calcDSHOTFrame(300), dshotFrames.motor4);
  // enable_dma_channels();
}

void loop() {
  // while (Serial.available() > 0) {

  //   // look for the next valid integer in the incoming serial stream:
  //   int red = Serial.parseInt();

  //   // look for the newline. That's the end of your sentence:
  //   if (Serial.read() == '\n') {
  //     // constrain the values to 0 - 255 and invert
  //     // if you're using a common-cathode LED, just use "constrain(color, 0, 255);"
  //     red = constrain(red, 48, 2047);

  //     // fade the red, green, and blue legs of the LED:
  //     writeDSHOTFrame(calcDSHOTFrame(red), dshotFrames.motor1);
  //     DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  //     // print the three numbers in one string as hexadecimal:
  //     Serial.print(red, HEX);

  //   }
  // }

  int x_read = analogRead(A5);
  int setpoint = constrain(round(2047/1023.0 * x_read), 48, 1047);

  // analogWrite(LED_BUILTIN, setpoint);


  writeDSHOTFrame(calcDSHOTFrame(setpoint), dshot_frame[0]);
  writeDSHOTFrame(calcDSHOTFrame(setpoint), dshot_frame[1]);
  writeDSHOTFrame(calcDSHOTFrame(setpoint), dshot_frame[2]);
  writeDSHOTFrame(calcDSHOTFrame(setpoint), dshot_frame[3]);
  enable_dma_channels();

  // delay(1);

}
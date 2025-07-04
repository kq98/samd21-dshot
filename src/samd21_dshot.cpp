
#include "samd21_dshot.h"

bool DSHOT_READY = false;
bool DSHOT_SEND_CMD = false;
int DSHOT_CMD_REPEAT_CNT = 0;

namespace DSHOT {
namespace detail {

static int _writeResolution = 8;

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
  constexpr uint8_t DSHOT150_HIGH = 231;
  constexpr uint8_t DSHOT150_LOW = 116;
  constexpr uint16_t DSHOT150_BIT_PERIOD = 308;

  constexpr uint8_t DSHOT300_HIGH = 116;
  constexpr uint8_t DSHOT300_LOW = 58;
  constexpr uint16_t DSHOT300_BIT_PERIOD = 154;

  constexpr uint8_t DSHOT600_HIGH = 58;
  constexpr uint8_t DSHOT600_LOW = 29;
  constexpr uint16_t DSHOT600_BIT_PERIOD = 77;

  constexpr uint8_t DSHOT1200_HIGH = 29;
  constexpr uint8_t DSHOT1200_LOW = 14;
  constexpr uint16_t DSHOT1200_BIT_PERIOD = 38;
#else
  constexpr uint8_t DSHOT150_HIGH = 240;
  constexpr uint8_t DSHOT150_LOW = 120;
  constexpr uint16_t DSHOT150_BIT_PERIOD = 320;

  constexpr uint8_t DSHOT300_HIGH = 120;
  constexpr uint8_t DSHOT300_LOW = 60;
  constexpr uint16_t DSHOT300_BIT_PERIOD = 160;

  constexpr uint8_t DSHOT600_HIGH = 60;
  constexpr uint8_t DSHOT600_LOW = 30;
  constexpr uint16_t DSHOT600_BIT_PERIOD = 80;

  constexpr uint8_t DSHOT1200_HIGH = 30;
  constexpr uint8_t DSHOT1200_LOW = 15;
  constexpr uint16_t DSHOT1200_BIT_PERIOD = 40;
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
volatile dmaDescriptor dmaDescriptorWritebackArray[4] __attribute__ ((aligned (16)));

uint8_t dshot_frame[DSHOT_NUM_MOTOR][DSHOT_FRAME_SIZE+1] __attribute__ ((aligned (16)));

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

void DSHOTAnalogWrite(int pin, int value, uint8_t group, uint8_t channel) {

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
    TCCx->PER.reg = DSHOT_BIT_PERIOD;
    syncTCC(TCCx);
    // Enable TCCx
    TCCx->CTRLA.bit.ENABLE = 1;
    syncTCC(TCCx);
  } else {
      Tcc* TCCx = TCC0;
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

  DMAC->CHID.reg = 0;
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  DMAC->CHID.reg = 1; 
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  DMAC->CHID.reg = 2;
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  DMAC->CHID.reg = 3;
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  DMAC->CTRL.reg |= DMAC_CTRL_DMAENABLE;
}

void disable_dma_channels() {
  DMAC->CHID.reg = 0;
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;

  DMAC->CHID.reg = 1;
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;

  DMAC->CHID.reg = 2;
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;

  DMAC->CHID.reg = 3;
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
}

void setupTCC() {
  DSHOTAnalogWrite(4, 0, PORT_PMUX_PMUXE_E, 0); //PA08
  DSHOTAnalogWrite(3, 0, PORT_PMUX_PMUXO_F, 1); //PA09
  DSHOTAnalogWrite(6, 0, PORT_PMUX_PMUXE_F, 2); //PA20
  DSHOTAnalogWrite(7, 0, PORT_PMUX_PMUXO_F, 3); //PA21
}

void setupDMA() {

  for (int i = 0; i < DSHOT_NUM_MOTOR; i++) {
    dmaDescriptorArray[i].btctrl =  (1 << 0) |  // VALID: Descriptor Valid
                                    (0 << 3) |  // BLOCKACT=NOACT: Block Action
                                    (1 << 10) | // SRCINC: Source Address Increment Enable
                                    (0 << 11) | // DSTINC: Destination Address Increment Enable
                                    (1 << 12) | // STEPSEL=SRC: Step Selection
                                    (0 << 13);  // STEPSIZE=X1: Address Increment Step Size
    dmaDescriptorArray[i].btcnt = DSHOT_FRAME_SIZE+1; // beat count
    dmaDescriptorArray[i].dstaddr = (uint32_t) &(TCC0->CCB[i].reg);
    dmaDescriptorArray[i].srcaddr = (uint32_t) &dshot_frame[i][DSHOT_FRAME_SIZE+1];
  }

  DMAC->BASEADDR.reg = (uint32_t)dmaDescriptorArray;
  DMAC->WRBADDR.reg = (uint32_t)dmaDescriptorWritebackArray;

  PM->AHBMASK.bit.DMAC_ = 1;
  PM->APBBMASK.bit.DMAC_ = 1;
  
  for (int i = 0; i < DSHOT_NUM_MOTOR; i++) {
    DMAC->CHID.reg = i; // select channel i
    DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(i) | DMAC_CHCTRLB_TRIGSRC(TCC0_DMAC_ID_OVF) | DMAC_CHCTRLB_TRIGACT_BEAT;
  }

  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);

  DMAC->CHID.reg = 3;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_TCMPL;
  NVIC_EnableIRQ(DMAC_IRQn);

  enable_dma_channels();
}

uint16_t calcFrame(uint16_t throttle, bool telemetry) {
  uint16_t value = 0;
  uint16_t crc = 0;

  if (throttle > 2047)
    return 0xFFFF;
  else {
    value = throttle << 1; // Telemetry = 0
    value |= 1 ? telemetry : 0;
    crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
    value = value << 4; // Shift thottle and telemetry into upper 12 bits to make space for crc
    value = value | crc;

    return value;
  }
}

void writeFrame(uint16_t value, uint8_t* target) {
  for (int i = 0; i < 16; i++) {
    if ( ((1 << i) & value) == (1 << i)) 
      target[15-i] = DSHOT_HIGH;
    else
      target[15-i] = DSHOT_LOW;
  }

  // set the last bit to 0 in order to terminate a DSHOT frame
  target[16] = 0;
}

}

using namespace detail;

void sendCommand(uint8_t cmd, uint8_t *target, uint16_t delay_val, uint8_t num_repeat) {
  disable_dma_channels();
  writeFrame(calcFrame(cmd,true), target);
  DSHOT_CMD_REPEAT_CNT = num_repeat;
  DSHOT_SEND_CMD = true;
  enable_dma_channels();

  delay(delay_val);
}

void ArmDrone() {
  TCC0->CCB[0].reg = (uint32_t) DSHOT_LOW;
  TCC0->CCB[1].reg = (uint32_t) DSHOT_LOW;
  TCC0->CCB[2].reg = (uint32_t) DSHOT_LOW;
  TCC0->CCB[3].reg = (uint32_t) DSHOT_LOW;
  delay(300);
  TCC0->CCB[0].reg = (uint32_t) 0x00;
  TCC0->CCB[1].reg = (uint32_t) 0x00;
  TCC0->CCB[2].reg = (uint32_t) 0x00;
  TCC0->CCB[3].reg = (uint32_t) 0x00;
  delay(1);
}

void DisarmDrone() {
  ArmDrone();
}

void Write(const Setpoint* setpoints) {
  if (DSHOT_READY) {
    DSHOT_READY = false;
    writeFrame(calcFrame(setpoints->motor1), dshot_frame[0]);
    writeFrame(calcFrame(setpoints->motor2), dshot_frame[1]);
    writeFrame(calcFrame(setpoints->motor3), dshot_frame[2]);
    writeFrame(calcFrame(setpoints->motor4), dshot_frame[3]);
    enable_dma_channels();
  }
}

int Init(MODE DSHOT_MODE) {
  switch(DSHOT_MODE) {
    case DSHOT150:
      DSHOT_HIGH = DSHOT150_HIGH;
      DSHOT_LOW  = DSHOT150_LOW;
      DSHOT_BIT_PERIOD = DSHOT150_BIT_PERIOD;
      break;
    case DSHOT300:
      DSHOT_HIGH = DSHOT300_HIGH;
      DSHOT_LOW  = DSHOT300_LOW;
      DSHOT_BIT_PERIOD = DSHOT300_BIT_PERIOD;
      break;
    case DSHOT600:
      DSHOT_HIGH = DSHOT600_HIGH;
      DSHOT_LOW  = DSHOT600_LOW;
      DSHOT_BIT_PERIOD = DSHOT600_BIT_PERIOD;
      break;
    case DSHOT1200:
      DSHOT_HIGH = DSHOT1200_HIGH;
      DSHOT_LOW  = DSHOT1200_LOW;
      DSHOT_BIT_PERIOD = DSHOT1200_BIT_PERIOD;
      break;
    default:
      return -1;
  }

  setupTCC();
  setupDMA();

  return 0;
}

}

void DMAC_Handler() {
  // Must clear this flag! Otherwise the interrupt will be triggered over and over again.
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_MASK;

  if (DSHOT_SEND_CMD) {
    if (DSHOT_CMD_REPEAT_CNT == 0) {
      DSHOT_SEND_CMD = false;
    } else {
      DSHOT_CMD_REPEAT_CNT--;
      DSHOT_READY = true;
      DSHOT::enable_dma_channels();
    }
  } else {
    DSHOT_READY = true;
  }
}

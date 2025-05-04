
#include <Arduino.h>
#include <wiring_private.h>

static int _writeResolution = 8;

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

struct DSHOTFrames {
  uint8_t motor1[16] __attribute__ ((aligned (16))) = {80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,0};
  uint8_t motor2[16] __attribute__ ((aligned (16))) = {80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,0};
  uint8_t motor3[16] __attribute__ ((aligned (16))) = {0,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80};
  uint8_t motor4[16] __attribute__ ((aligned (16))) = {0,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80};
};

DSHOTFrames dshotFrames;

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

void DMAC_Handler() {
  // Must clear this flag! Otherwise the interrupt will be triggered over and over again.
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_MASK;

  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

void setupDMA() {
  dmaDescriptorArray[0].btctrl =  (1 << 0) |  // VALID: Descriptor Valid
                                  (0 << 3) |  // BLOCKACT=NOACT: Block Action
                                  (1 << 10) | // SRCINC: Source Address Increment Enable
                                  (0 << 11) | // DSTINC: Destination Address Increment Enable
                                  (1 << 12) | // STEPSEL=SRC: Step Selection
                                  (0 << 13);  // STEPSIZE=X1: Address Increment Step Size
  dmaDescriptorArray[0].btcnt = 16; // beat count
  dmaDescriptorArray[0].dstaddr = (uint32_t) &(TCC0->CCB[0].reg);
  dmaDescriptorArray[0].srcaddr = (uint32_t) &dshotFrames.motor1[16];
  dmaDescriptorArray[0].descaddr = (uint32_t) &dmaDescriptorArray[1];

  dmaDescriptorArray[1].btctrl =  (1 << 0) |  // VALID: Descriptor Valid
                                  (0 << 3) |  // BLOCKACT=NOACT: Block Action
                                  (1 << 10) | // SRCINC: Source Address Increment Enable
                                  (0 << 11) | // DSTINC: Destination Address Increment Enable
                                  (1 << 12) | // STEPSEL=SRC: Step Selection
                                  (0 << 13);  // STEPSIZE=X1: Address Increment Step Size
  dmaDescriptorArray[1].btcnt = 16; // beat count
  dmaDescriptorArray[1].dstaddr = (uint32_t) &(TCC0->CCB[1].reg);
  dmaDescriptorArray[1].srcaddr = (uint32_t) &dshotFrames.motor2[16];
  dmaDescriptorArray[1].descaddr = (uint32_t) &dmaDescriptorArray[2];

  dmaDescriptorArray[2].btctrl =  (1 << 0) |  // VALID: Descriptor Valid
                                  (0 << 3) |  // BLOCKACT=NOACT: Block Action
                                  (1 << 10) | // SRCINC: Source Address Increment Enable
                                  (0 << 11) | // DSTINC: Destination Address Increment Enable
                                  (1 << 12) | // STEPSEL=SRC: Step Selection
                                  (0 << 13);  // STEPSIZE=X1: Address Increment Step Size
  dmaDescriptorArray[2].btcnt = 16; // beat count
  dmaDescriptorArray[2].dstaddr = (uint32_t) &(TCC0->CCB[2].reg);
  dmaDescriptorArray[2].srcaddr = (uint32_t) &dshotFrames.motor3[16];
  dmaDescriptorArray[2].descaddr = (uint32_t) &dmaDescriptorArray[3];

  dmaDescriptorArray[3].btctrl =  (1 << 0) |  // VALID: Descriptor Valid
                                  (0 << 3) |  // BLOCKACT=NOACT: Block Action
                                  (1 << 10) | // SRCINC: Source Address Increment Enable
                                  (0 << 11) | // DSTINC: Destination Address Increment Enable
                                  (1 << 12) | // STEPSEL=SRC: Step Selection
                                  (0 << 13);  // STEPSIZE=X1: Address Increment Step Size
  dmaDescriptorArray[3].btcnt = 16; // beat count
  dmaDescriptorArray[3].dstaddr = (uint32_t) &(TCC0->CCB[3].reg);
  dmaDescriptorArray[3].srcaddr = (uint32_t) &dshotFrames.motor4[16];
  dmaDescriptorArray[3].descaddr = (uint32_t) 0x00; // last of the 4 linked descriptors

  DMAC->BASEADDR.reg = (uint32_t)dmaDescriptorArray;
  DMAC->WRBADDR.reg = (uint32_t)dmaDescriptorWritebackArray;

  PM->AHBMASK.bit.DMAC_ = 1;
  PM->APBBMASK.bit.DMAC_ = 1;
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
  

  DMAC->CHID.reg = 0; // select channel 0
  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0) | DMAC_CHCTRLB_TRIGSRC(TCC0_DMAC_ID_OVF) | DMAC_CHCTRLB_TRIGACT_BEAT;

  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_TCMPL;
  NVIC_EnableIRQ(DMAC_IRQn);

  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

}

void setup() {

  myAnalogWrite(4, 0, PORT_PMUX_PMUXE_E, 0); //PA08
  myAnalogWrite(3, 0, PORT_PMUX_PMUXO_F, 1); //PA09
  myAnalogWrite(6, 0, PORT_PMUX_PMUXE_F, 2); //PA20
  myAnalogWrite(7, 0, PORT_PMUX_PMUXO_F, 3); //PA21

  setupDMA();
}

void loop() {

  //myAnalogWrite(4, counter[0], PORT_PMUX_PMUXE_E, 0); //PA08
  // myAnalogWrite(3, counter[1], PORT_PMUX_PMUXO_F, 1); //PA09
  // myAnalogWrite(6, counter[2], PORT_PMUX_PMUXE_F, 2); //PA20
  // myAnalogWrite(7, counter[3], PORT_PMUX_PMUXO_F, 3); //PA21
  // analogWrite(5,counter);
  delay ( 10 );
  if (counter[0] < 80) {
    counter[0]++;
  }
  else 
    counter[0] = 0;

  if (counter[1] < 80)
    counter[1] += 10;
  else 
    counter[1] = 0;

  if (counter[2] < 80)
    counter[2] += 20;
  else 
    counter[2] = 0;

  if (counter[3] < 80)
    counter[3] += 50;
  else 
    counter[3] = 0;

  for(int i = 0; i < 16; i++) {
    dshotFrames.motor1[i] = counter[0];
    dshotFrames.motor2[i] = counter[1];
    dshotFrames.motor3[i] = counter[2];
    dshotFrames.motor4[i] = counter[3];
  }
}
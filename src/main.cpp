
#include <Arduino.h>

static int _writeResolution = 8;

int counter = 0;

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

void myAnalogWrite(int pin, int value) {

  PinDescription pinDesc = g_APinDescription[pin];

  value = mapResolution(value, _writeResolution, 16);

  uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
  static bool tcEnabled[TCC_INST_NUM+TC_INST_NUM];

  // Get whole current setup for both odd and even pins and remove odd one
  // uint32_t temp = (PORT->Group[g_APinDescription[pin].ulPort].PMUX[g_APinDescription[pin].ulPin >> 1].reg) & PORT_PMUX_PMUXE( 0xF ) ;
  // Set new muxing
  // PORT->Group[g_APinDescription[pin].ulPort].PMUX[g_APinDescription[pin].ulPin >> 1].reg = temp|PORT_PMUX_PMUXO( PIO_TIMER ) ;

  PORT->Group[g_APinDescription[pin].ulPort].PMUX[g_APinDescription[pin].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;
  PORT->Group[g_APinDescription[pin].ulPort].PINCFG[g_APinDescription[pin].ulPin].reg |= PORT_PINCFG_PMUXEN ;

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
    TCCx->CC[3].reg = (uint32_t) value;
    syncTCC(TCCx);
    // Set PER to maximum counter value (resolution : 0xFFFF)
    TCCx->PER.reg = 0xFFFF;
    syncTCC(TCCx);
    // Enable TCCx
    TCCx->CTRLA.bit.ENABLE = 1;
    syncTCC(TCCx);
  } else {
      Tcc* TCCx = TCC0; //(Tcc*) GetTC(pinDesc.ulPWMChannel);
      TCCx->CTRLBSET.bit.LUPD = 1;
      syncTCC(TCCx);
      TCCx->CCB[3].reg = (uint32_t) value;
      syncTCC(TCCx);
      TCCx->CTRLBCLR.bit.LUPD = 1;
      syncTCC(TCCx);
  }
}

void setup() {
  myAnalogWrite(13, 128);
}

void loop() {

  myAnalogWrite(13, counter);
  delay ( 10 );
  if (counter < 255)
    counter++;
  else 
    counter = 0;
}
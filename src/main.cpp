
#include <Arduino.h>
#include <wiring_private.h>

static int _writeResolution = 8;

int counter[4] = {0, 0, 0, 0};

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


  value = mapResolution(value, _writeResolution, 16);

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

  //PORT->Group[g_APinDescription[pin].ulPort].PINCFG[g_APinDescription[pin].ulPin].reg |= PORT_PINCFG_PMUXEN ;

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
    TCCx->PER.reg = 0xFFFF;
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

void setup() {
  // myAnalogWrite(4, 128, PORT_PMUX_PMUXO_E, 0);
  // myAnalogWrite(5, 128, PORT_PMUX_PMUXO_F, 1);
  // myAnalogWrite(6, 128, PORT_PMUX_PMUXO_F, 2);
  // myAnalogWrite(7, 128, PORT_PMUX_PMUXO_F, 3);
}

void loop() {

  myAnalogWrite(4, counter[0], PORT_PMUX_PMUXE_E, 0); //PA08
  myAnalogWrite(3, counter[1], PORT_PMUX_PMUXO_F, 1); //PA09
  myAnalogWrite(6, counter[2], PORT_PMUX_PMUXE_F, 2); //PA20
  myAnalogWrite(7, counter[3], PORT_PMUX_PMUXO_F, 3); //PA21
  // analogWrite(5,counter);
  delay ( 10 );
  if (counter[0] < 255)
    counter[0]++;
  else 
    counter[0] = 0;

  if (counter[1] < 255)
    counter[1] += 10;
  else 
    counter[1] = 0;

  if (counter[2] < 255)
    counter[2] += 20;
  else 
    counter[2] = 0;

  if (counter[3] < 255)
    counter[3] += 50;
  else 
    counter[3] = 0;
}
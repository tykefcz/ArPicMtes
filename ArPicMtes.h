#if !defined(ARPICMTES_H)
#if !defined(ARDUINO_AVR_UNO) && !defined(ARDUINO_AVR_NANO) && !defined(ARDUINO_AVR_YUN) && !defined(ARDUINO_AVR_LEONARDO) && !defined(ARDUINO_AVR_MICRO)
  #error "This library only supports Arduino Uno boards"
#endif
#define ARPICMTES_H 1

#include <Arduino.h>
#include <inttypes.h>

uint16_t ticks16(void);
uint32_t ticks(void);
unsigned long ticksToMicros(uint32_t t);

typedef struct pcevent_t {
  uint8_t pin; /* 0..127 rising 128..255 falling edge */
  uint32_t stamp; /* ticks */
} PCEVENT;

extern PCEVENT eventq[];
extern volatile uint8_t event_head,event_tail;

inline bool isPcEvent() { return event_head!=event_tail;}

inline void pcEventPop() {
  if (event_head!=event_tail) event_head=(event_head+1)&15;
}

int8_t pcEventPin();
unsigned long pcEventStamp();
inline bool pcEventRiseEdge() {
  return (event_head!=event_tail && (eventq[event_head].pin & 0x80)!=0);
}

void pcAttachPin(uint8_t pin);
void pcDetachPin(uint8_t pin);
void pcBegin();
#endif

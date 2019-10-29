#include "ArPicMtes.h"
#include "pins_arduino.h"

/* Pin to interrupt map:
 * D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
 * D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
 * A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
 */
const uint16_t PROGMEM port_to_pcmask_PGM[] = {
  (uint16_t) &PCMSK0,
#if defined(PCMSK1)
  (uint16_t) &PCMSK1,  (uint16_t) &PCMSK2
#else
  NOT_A_PORT,NOT_A_PORT
#endif
};
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
  const uint8_t PROGMEM portbit_to_pin_PGM[] = {
  // B0..B5=8..13; C0..C5=14..19; D0..D7=0..7
  8,9,10,11,12,13,63,63,14,15,16,17,18,19,63,63,0,1,2,3,4,5,6,7};
  #define PCINT_PORTCD	1
#elif defined(ARDUINO_AVR_YUN) || defined(ARDUINO_AVR_LEONARDO) || defined(ARDUINO_AVR_MICRO)
  const uint8_t PROGMEM portbit_to_pin_PGM[] = {
// B0..B7                
  17,15,16,14,8,9,10,11};
  #define PCINT_PORTCD	0
#else
  #error Unsuported Arduino Variant (UNO,YUN,LEONARDO,MICRO,NANO) only
#endif
extern volatile unsigned long timer0_overflow_count;
volatile uint8_t *lobyte_t0ovc = (uint8_t*)&timer0_overflow_count;
uint16_t ticks16(void) { // like micros
  uint8_t oldSREG = SREG,rvh,rvl;
  cli();
#if defined(TCNT0)
  rvl = TCNT0;
#elif defined(TCNT0L)
  rvl = TCNT0L;
#else
  #error TIMER 0 not defined
#endif
  rvh=*lobyte_t0ovc; // low byte only
#ifdef TIFR0
  if ((TIFR0 & _BV(TOV0)) && (rvl != 255))
    rvh++;
#else
  if ((TIFR & _BV(TOV0)) && (rvl != 255))
    rvh++;
#endif
  SREG = oldSREG;
  return (rvh<<8)+rvl;
}

uint32_t ticks(void) { // like micros
  uint8_t oldSREG = SREG,rvl;
  uint32_t rvh;
  cli();
#if defined(TCNT0)
  rvl = TCNT0;
#elif defined(TCNT0L)
  rvl = TCNT0L;
#else
  #error TIMER 0 not defined
#endif
  rvh=timer0_overflow_count&0xFFFFFFUL; // low 3 bytes only
#ifdef TIFR0
  if ((TIFR0 & _BV(TOV0)) && (rvl != 255))
    rvh++;
#else
  if ((TIFR & _BV(TOV0)) && (rvl != 255))
    rvh++;
#endif
  SREG = oldSREG;
  return (rvh<<8)+rvl;
}

unsigned long ticksToMicros(uint32_t t) {
  return t * (64 / clockCyclesPerMicrosecond());
}

PCEVENT eventq[16];

volatile uint8_t event_head,event_tail;
//static 
uint8_t PCintMaskState[6]={}; // 3 mask(BCD) 3 state(BCD)
void pcAttachPin(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT)
    return;
  port -= 2;
  pcmask = (volatile uint8_t *)(pgm_read_word(port_to_pcmask_PGM + port));

  // set the mask
  *pcmask |= bit;
  if (PCintMaskState[port]==0) { // save "last" pin state on first enable
    PCintMaskState[port + 3]=*portInputRegister(port + 2);
  }
  PCintMaskState[port] |= bit;
  // enable the interrupt
  PCICR |= 0x01 << port;
//Serial.println((String)"Attachaed port" + port + " mask:" + bit + " pin=" + pin + " pcmask=" + *pcmask + " at " + (uint16_t)pcmask);
}

void pcDetachPin(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT)
    return;
  port -= 2;
  pcmask = (volatile uint8_t *)(pgm_read_word(port_to_pcmask_PGM + port));

  // disable the mask.
  *pcmask &= ~bit;
  PCintMaskState[port] &= ~bit;
  // if that's the last one, disable the interrupt.
  if (*pcmask == 0)
    PCICR &= ~(0x01 << port);
}

static void pcint_all(void) __attribute__ ((__naked__));
static void pcint_all(void) {
  // Input r16 PINx value, __tmp_reg__ = 0 portB, 1 portC, 2 portD
  asm volatile(
    "       push __zero_reg__\n"
    "       clr  __zero_reg__\n"
    "       push r28\n" // 30,31=Z;28,29=Y;26,27=X
    "       push r29\n"
    "       push r30\n" // 30,31=Z;28,29=Y;26,27=X
    "       push r31\n");
  asm volatile(
    "       push r17\n"
    "       push r18\n"
    "       push r19\n"
    "       in	 r17,%[tifr]\n"
    "       in   r18,%[tcnt0]\n"
    "       andi r17,%[bvtov0]\n"
    "       breq neniov\n"
    "       eor  r18,r18\n"
    "       dec  r18\n" // r18 = 255 if overflow
    "neniov:\n"
    "       add  r28,__tmp_reg__\n"
    "       adc  r28,__zero_reg__\n"
    "       ld   r17,Y\n"
    "       ldd  r19,Y+3\n"
    "       eor  r19,r16\n" // r19 = curr ^ last
    "       and  r17,r19\n" // r17 = mask (changed bits)
    "       breq nebit\n"
    "       std  Y+3,r16\n" // r16 = new pinx; save state
    "       lsl  __tmp_reg__\n" // port * 2
    "       lsl  __tmp_reg__\n" // port * 4
    "       lsl  __tmp_reg__\n" // port * 8
    "       ldi  r28,0x40\n"
    "       or   __tmp_reg__,r28\n" // dirty flag
    "nextbit:\n"
    "       sbrs r17,0\n"
    "       rjmp nochange\n"
    "       lds  r28,%[evt]\n"
    "       inc  r28\n"
    "       andi r28,15\n"
    "       sts  %[evt],r28\n"
    "       lds  r29,%[evh]\n"
    "       cpse r28,r29\n"
    "       rjmp nohdinc\n"
    "       inc  r29\n"
    "       andi r29,15\n"
    "       sts  %[evh],r29\n"
    "nohdinc:\n"
    "       mov  r29,r31\n"  // eventq index
    "       mov  r19,r28\n"
    "       lsl  r19\n"
    "       lsl  r19\n"
    "       add  r28,r19\n"  // r28 *= 5 ; evq_tail * 5
    "       add  r28,r30\n"  // Y=eventq[r28*5] 
    "       brcc nohii\n" // skip if carry not set
    "       inc  r29\n"
    "nohii:\n"
    "       mov  r19,__tmp_reg__\n"
    "       sbrc r16,0\n"  // falling edge not set +128
    "       ori  r19,128\n"
    "       st   Y+,r19\n"
    "       st   Y+,r18\n"
    "       lds  r19,%[t0ovc]\n"
    "       st   Y+,r19\n"
    "       lds  r19,%[t0ovc]+1\n"
    "       st   Y+,r19\n"
    "       lds  r19,%[t0ovc]+2\n"
    "       st   Y,r19\n"
    "nochange:\n"
    "       lsr  r16\n"         // curr bits
    "       inc  __tmp_reg__\n" // pin# ++
    "       lsr  r17\n"         // changed bits
    "       brne nextbit\n"     // change to save ?
    "nebit:\n"
    "       pop r19\n"
    "       pop r18\n"
    "       pop r17\n"
    "       pop r31\n"
    "       pop r30\n"
    "       pop r29\n"
    "       pop r28\n"
    "       pop __zero_reg__\n"
    "	    ret\n"
    : /* no output*/
    :[tcnt0]	"I"	(_SFR_IO_ADDR(
    #if defined(TCNT0)
    TCNT0))
    #else
    TCNT0L))
    #endif
    ,[tifr]	"I"	(_SFR_IO_ADDR(
    #ifdef TIFR0
    TIFR0))
    #else
    TIFR))
    #endif
    ,[bvtov0]	"I"	(_BV(TOV0))
    ,[evq]	"z"	(eventq)
    ,[evh]	"p"	(&event_head)
    ,[evt]	"p"	(&event_tail)
    ,[evma]	"y"	(PCintMaskState)
    ,[t0ovc]    "p"     (&timer0_overflow_count)
    );
}

ISR(PCINT0_vect, ISR_NAKED) {
    asm volatile(
    "       push r16\n"
    "       in   r16,__SREG__\n"
    "       push r16\n"
    "       push __tmp_reg__\n"
    "       ldi  r16,0\n"
    "       mov  __tmp_reg__,r16\n" // port 0 = B 8*8=64 dirty pin#
    "       in   r16,%[pinx]\n"
    : /* no output*/
    :[pinx]	"I"	(_SFR_IO_ADDR(PINB))
    );
    pcint_all();
    asm volatile(
    "       pop __tmp_reg__\n"
    "       pop r16\n"
    "       out __SREG__, r16\n"
    "       pop r16\n"  // restore previous r0
    "       reti            \n");
    
}
#if PCINT_PORTCD
ISR(PCINT1_vect, ISR_NAKED) {
    asm volatile(
    "       push r16\n"
    "       in   r16,__SREG__\n"
    "       push r16\n"
    "       push __tmp_reg__\n"
    "       ldi  r16,1\n"
    "       mov  __tmp_reg__,r16\n" // port 1 = C
    "       in   r16,%[pinx]\n"
    : /* no output*/
    :[pinx]	"I"	(_SFR_IO_ADDR(PINC))
    );
    pcint_all();
    asm volatile(
    "       pop __tmp_reg__\n"
    "       pop r16\n"
    "       out __SREG__, r16\n"
    "       pop r16\n"  // restore previous r0
    "       reti            \n");
    
}
ISR(PCINT2_vect, ISR_NAKED) {
    asm volatile(
    "       push r16\n"
    "       in   r16,__SREG__\n"
    "       push r16\n"
    "       push __tmp_reg__\n"
    "       ldi  r16,2\n"
    "       mov  __tmp_reg__,r16\n" // port 2 = D
    "       in   r16,%[pinx]\n"
    : /* no output*/
    :[pinx]	"I"	(_SFR_IO_ADDR(PIND))
    );
    pcint_all();
    asm volatile(
    "       pop __tmp_reg__\n"
    "       pop r16\n"
    "       out __SREG__, r16\n"
    "       pop r16\n"  // restore previous r0
    "       reti            \n");
    
}
#endif
/*
inline int8_t pcevent_push_ix() {
  int8_t rv=event_tail++;
  event_tail&=15;
  if (event_head == event_tail) event_head=(event_head+1)&15; // discard oldest event
  return rv;
}

SIGNAL(PCINT0_vect) {
  uint8_t ticl;
  uint8_t curr,mask;
  uint32_t tich;
#if defined(TCNT0)
  ticl = TCNT0;
#elif defined(TCNT0L)
  ticl = TCNT0L;
#else
  #error TIMER 0 not defined
#endif
  tich=timer0_overflow_count&0xFFFFFFUL; // low 3 bytes only
#ifdef TIFR0
  if ((TIFR0 & _BV(TOV0)) != 0) ticl = 255;
#else
  if ((TIFR & _BV(TOV0)) != 0) ticl = 255;
#endif
  curr = PINB; // PINC | PIND
  mask = (curr ^ PCintMaskState[3]) & PCintMaskState[0];
  PCintMaskState[3] = curr;
  if (mask==0) return; // not monitored port
  for (uint8_t pin=8,m=1;m!=_BV(6);pin++,m<<=1) {
    if ((mask & m)!=0) {
      PCEVENT *aq=&(eventq[pcevent_push_ix()]);
      aq->pin=pin | ((curr&m)!=0?128:0);
      aq->stamp=(tich<<8)+ticl;
    }
  }
}
*/

int8_t pcEventPin() {
  if (event_head==event_tail) return -1;
  if ((eventq[event_head].pin & 0x40)!=0)
    eventq[event_head].pin = 
      (eventq[event_head].pin & 0x80) +
      pgm_read_byte(portbit_to_pin_PGM + (eventq[event_head].pin & 0x3F));
  return (eventq[event_head].pin & 0x7F);
}

unsigned long pcEventStamp() {
  return eventq[event_head].stamp;
}

void pcBegin() {
  PCintMaskState[0]=PCintMaskState[1]=PCintMaskState[2]=0;
  PCintMaskState[3]=PCintMaskState[4]=PCintMaskState[5]=0;
  event_head=event_tail=0;  
}

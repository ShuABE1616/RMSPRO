#ifndef TIMERONE_cpp
#define TIMERONE_cpp

#include "TimerOne.h"

TimerOne Timer1;              // preinstatiate

/*ISR(TIMER1_OVF_vect)          // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  Timer1.isrCallback();
}*/

ISR(TIMER1_CAPT_vect)//タイマカウンタがICR1に到達すると割り込み
{
  Timer1.isrCallback();
}


void TimerOne::initialize(void)
{
  //TCCR1A = 0;                 // clear control register A 
  //TCCR1B = _BV(WGM13);        // set mode 8: phase and frequency correct pwm, stop the timer
  setPeriod();
}


void TimerOne::setPeriod(void)
{
  /*引数は無視して64分周2040マイクロ秒で固定割り込み*/
  oldSREG = SREG;
  /*タイマ割り込み禁止*/
  cli();
  /*プリスケーラ設定*/
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));//プリスケーラ設定のリセット
  clockSelectBits = _BV(CS11) | _BV(CS10);  //プリスケーラ1/64
  TCCR1B |= clockSelectBits;
  /*TOP値設定*/        
  ICR1 = 255;                
  SREG = oldSREG;
}

void TimerOne::attachInterrupt(void (*isr)())
{
  isrCallback = isr; 
  TIMSK1 = _BV(ICIE1);//ISR1に達したらタイマ割り込み, TOIE1の場合、タイマカウンタがオーバーフローしたら割り込みになる
  sei();//タイマ割り込み許可
  resume();												
}

void TimerOne::detachInterrupt()
{
  TIMSK1 &= ~_BV(ICIE1); //タイマ割り込み停止
  cli();
}

void TimerOne::resume()				// AR suggested
{ 
  TCCR1B |= clockSelectBits;
}

void TimerOne::restart()		// Depricated - Public interface to start at zero - Lex 10/9/2011
{
	start();				
}

void TimerOne::start()	// AR addition, renamed by Lex to reflect it's actual role
{
  unsigned int tcnt1;
  
  TIMSK1 &= ~_BV(TOIE1);        // AR added 
  GTCCR |= _BV(PSRSYNC);   		// AR added - reset prescaler (NB: shared with all 16 bit timers);

  oldSREG = SREG;				// AR - save status register
  cli();						// AR - Disable interrupts
  TCNT1 = 0;                	
  SREG = oldSREG;          		// AR - Restore status register
	resume();
  do {	// Nothing -- wait until timer moved on from zero - otherwise get a phantom interrupt
	oldSREG = SREG;
	cli();
	tcnt1 = TCNT1;
	SREG = oldSREG;
  } while (tcnt1==0); 
 
//  TIFR1 = 0xff;              		// AR - Clear interrupt flags
//  TIMSK1 = _BV(TOIE1);              // sets the timer overflow interrupt enable bit
}

void TimerOne::stop()
{
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));          // clears all clock selects bits
}

#endif
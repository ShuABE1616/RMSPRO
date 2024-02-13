#ifndef TIMERONE_h
#define TIMERONE_h

#include <avr/io.h>
#include <avr/interrupt.h>

#define RESOLUTION 65536    // Timer1 is 16 bit

class TimerOne
{
  public:
  
    // properties
    unsigned int pwmPeriod;
    unsigned char clockSelectBits;
	char oldSREG;

    // methods
    void initialize(void);
    void start();
    void stop();
    void restart();
	void resume();
	unsigned long read();
    void attachInterrupt(void (*isr)());
    void detachInterrupt();
    void setPeriod(void);
    void (*isrCallback)();
};

extern TimerOne Timer1;
#endif
#ifndef CURRENT__H__
#define CURRENT__H__

#include <xc.h> // processor SFR definitions
#include <sys/attribs.h> // __ISR macro
#include "NU32DIP.h"

#define NUMSAMPS 100
volatile float Waveform[NUMSAMPS];
volatile float uarray[NUMSAMPS];
volatile float ADCarray[NUMSAMPS];
void delay();
void current_StartUp();
void set_curr_kp(float kp);
void set_curr_ki(float ki);
float get_curr_kp();
float get_curr_ki();
void makeWaveform();

void current_PID(float ref_curr);

#endif 
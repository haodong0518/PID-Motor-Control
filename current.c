#include "nu32dip.h"          // constants, functions for startup and UART
#include "utilities.h"
#include "ina219.h"
#include "current.h"
#include "pos_control.h"

#define EINT_MAX 100
#define char_size 200

static int counter = 0;
static volatile float e; //error (ref - adcval)
static volatile float eint = 0;//
static volatile float current_kp = 0, current_ki= 0; 
static volatile float u = 50.0, u_new; //initialize PWM at 50%
char prints[char_size];

volatile int curr_count = 0;
float curr_array[NUMSAMPS];
float ref_array[NUMSAMPS];

void set_curr_kp(float kp);
void set_curr_ki(float ki);
float get_curr_kp();
float get_curr_ki();
void makeWaveform();

void set_curr_kp(float kp){
  current_kp = kp;
}

void set_curr_ki(float ki){
  current_ki = ki;
}

float get_curr_kp(){
  return current_kp;
}
float get_curr_ki(){
  return current_ki;
}


void delay(){
  _CP0_SET_COUNT(0);       // delay 1 seconds to see the 25% duty cycle 
  while(_CP0_GET_COUNT() < 1 * 48000000) {
    ;
  }
}

// Timer2 ISR for 5 kHz current control
void __ISR(_TIMER_2_VECTOR, IPL5SOFT) current(void) {
  switch (get_mode())
  {
  case IDLE:{
    OC1RS =0;
    break;
  }
  case PWM:{
    int pwm_input = get_pwm();
    OC1RS = pwm_input * 24;
    break;
  }
  case ITEST: {
    counter++;
    if(counter == 99 ){
      set_pwm(0);
      set_mode(IDLE);
      counter = 0;
    }
    else if(counter < 100){
      current_PID(Waveform[counter]);
      /*
      float curr = INA219_read_current();
      ADCarray[counter] = curr;
      e = curr - Waveform[counter];
      eint += e; 
      u = current_kp * e+current_ki * eint ;

      int ocmax = 2400;


      if(Waveform[counter] > 0){
        LATAbits.LATA1 = 0;
      }
      else if(Waveform[counter]<0){
        LATAbits.LATA1 = 1;
      }

      if(u<0){
        u = u*(-1);
      }

      if (u>=ocmax)
      { 
        u =  ocmax;}

      OC1RS = (int)u;*/
//**-------------------------------------------------
      // if ( u > 0 && u<ocmax) {
      //   LATAbits.LATA1 = 0;
	    // } else if (u < 0 && u>-ocmax) {
      //   u = u* (-1);
      //   LATAbits.LATA1 = 1;
	    // }

      // if (u>=ocmax)
      // { 
      //   u =  ocmax;
      //   LATAbits.LATA1 = 0;
      // }else if (u<=-ocmax)
      // {
      //   u = ocmax;
      //   LATAbits.LATA1 = 1;
      // }
//** ------------------------------------------------
    }
    break;
  }
  case HOLD:{
			//same formulation as ITEST, but we're not
			//plotting current values this time			
      // sprintf(prints, "current.c: in HOLD mode: %f", get_ref_curr_from_pos());
      // NU32DIP_WriteUART1(prints);
			current_PID(get_ref_curr_from_pos());
			//curr_count -= 1; //not using arrays for plotting
			break;
		}
  case TRACK:{
      current_PID(get_ref_curr_from_pos());
      break;
  }
  }
  IFS0bits.T2IF = 0; // Clear Timer2 interrupt flag  
}

// I need two timer here, one timer is for the pwm, for the output compare 20khz
// the other timer is for the interrupt, 5khz for current control

void current_StartUp() {
    makeWaveform();
    __builtin_disable_interrupts(); // disable interrupts at CPU
    // Set the A1 to switch direction
    TRISAbits.TRISA1 = 0;
    ANSELA = 0;

    // Timer2 
    T2CONbits.ON = 1;        // turn on Timer2
    T2CONbits.TCKPS = 0b110; // set prescalar 1:8, N = 8
    PR2 = 149; // (48m / 64 / 5000) - 1   ~ Timer2 period register, calculated based on 48 MHz clock
    TMR2 = 0; 

    // Timer3 
    T3CONbits.ON = 1;
    T3CONbits.TCKPS = 0;
    PR3 = 2399;    // 48m / 20k
    TMR3 = 0;

    OC1CONbits.ON = 1;       // turn on OC1
    OC1CONbits.OCM = 0b110;  // PWM mode 
    OC1CONbits.OCTSEL = 1;   // Use Timer 3
    RPA0Rbits.RPA0R = 0b0101;// put OC1 on RPA0
    OC1R = 0;             
    OC1RS = 0;             // duty cycle = OC1RS/(PR3+1) = OC1RS / 2400

    IPC2bits.T2IP = 5; // step 4: interrupt priority
    IPC2bits.T2IS = 0; // step 4: subp is 0, which is the default
    IFS0bits.T2IF = 0; // step 5: clear Timer2 interrupt flag
    IEC0bits.T2IE = 1; // step 6: enable Timer2 interrupt

    __builtin_enable_interrupts();  // INT step 7: enable interrupts at CPU
}


void makeWaveform() {
	
	//waveform should be between -200 and 200
	//current at any given time to create an "analog" square
	//wave of variable peak/valley values
	int i = 0;
  float center =0.0, A = 200.0; // square wave, fill in center value and amplitude
	for (i = 0; i < 100; ++i) {
		if ((i <= 25) || (i >= 50 && i <= 75)) {
			Waveform[i] = center + A;
		} else{
			Waveform[i] = center - A;
		}
	}
}


void current_PID(float ref_curr) {
	
	float current = INA219_read_current();
  ADCarray[counter] = current;
	e = current - ref_curr;

	//carry out PI controller for current
  eint += e; 
  u = current_kp * e+current_ki * eint ;
	
	//integrator anti windup
  int ocmax = 2400;

  if(ref_curr> 0){
    LATAbits.LATA1 = 0;
  }
  else if(ref_curr<0){
    LATAbits.LATA1 = 1;
  }

  if(u<0){
    u = u*(-1);
  }

  if (u>=ocmax)
  { 
    u =  ocmax;}
    
	OC1RS = (int)u;

	//store values of current in arrays. send them in a separate
	//function because sprintf() takes a while
	// curr_array[curr_count] = current;
	// ref_array[curr_count] = ref_curr;

	//setup calcs for next pass of ISR
	// curr_count++;
}

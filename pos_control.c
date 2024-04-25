#include "nu32dip.h"          // constants, functions for startup and UART
#include "utilities.h"
#include "ina219.h"
#include "current.h"
#include "encoder.h"
#include "pos_control.h"

#define POSN_OUT_MAX 1000 // don't want it to go at max 1A (1000)
#define POSN_EINT_MAX 200


static volatile float pos_kp , pos_ki, pos_kd; 
static volatile float desir_deg = 0.0;
static volatile float cur_deg; 
static volatile float ref_curr_in_pos;
char debug[200];
static volatile int traj_counter = 0;
static volatile float trajectory_PLOT_array[1000];

float error_posn = 0.0;
float eint_posn = 0.0;
float eprev_posn = 0.0;
float eder_posn = 0.0;
float u_posn;
float u_posn_old = 0.0;


// Timer4 ISR for 200 Hz position control
void __ISR(_TIMER_4_VECTOR, IPL6SOFT) PositionControlISR(void) {
    switch (get_mode())
    {
    case HOLD:
        position_PID();
        traj_counter = 0;
        // sprintf(debug, "in HOLD mode: %d", desir_deg);
        // NU32DIP_WriteUART1(debug);
        break;
    
    case TRACK:{
        while(traj_counter< get_traj_length()){
            // sprintf(debug, "in pos.c TRACK case\r\n");
            // NU32DIP_WriteUART1(debug);

            desir_deg = ref_posn(traj_counter);
            position_PID();
            traj_counter++;
        }
        // sprintf(debug, "in pos.c , set HOLD\r\n");
        // NU32DIP_WriteUART1(debug);
        set_mode(HOLD);
        break;
    }
    
    default:
        break;
    }
    IFS0bits.T4IF = 0; 
}

// duty cycle = OCxR/(PRy + 1) x 100%.
// period = 48M / [(PR +1)*N]
// put OC1 on RPA0
void PositionControl_StartUp() {

    __builtin_disable_interrupts();
    // Timer4
    T4CONbits.ON = 1;        // turn on Timer4
    T4CONbits.TCKPS = 0b110 ; // Prescaler 
    PR4 = 3749; // Timer4 period register, for 200 Hz with 48 MHz clock (48m/ 200/64) - 1
    TMR4 = 0; // Clear Timer4 counter

    IPC4bits.T4IP = 6; // Set interrupt priority
    IPC4bits.T4IS = 0; // Set Timer4 interrupt subpriority to 0
    IFS0bits.T4IF = 0; // Clear Timer4 interrupt flag
    IEC0bits.T4IE = 1; // Enable Timer4 interrupts
    __builtin_enable_interrupts();
}

void position_PID()
{   WriteUART2("a");
	
	while (!get_encoder_flag()) {
		//delay until encoder/PICO are done sending insructions
	}
	
	set_encoder_flag(0); //prepare for new instructions

    cur_deg =  get_encoder_count() / 384.0 *360.0;
    trajectory_PLOT_array[traj_counter] = cur_deg;

    error_posn = cur_deg - desir_deg;

    if (eint_posn > POSN_EINT_MAX) {
		eint_posn = POSN_EINT_MAX;
	} else if (eint_posn < -POSN_EINT_MAX) {
		eint_posn = -POSN_EINT_MAX;
	}

    u_posn = pos_kp * error_posn + pos_ki * eint_posn + pos_kd * eder_posn; 

    // if((u_posn_old - u_posn) < 100 & (u_posn_old - u_posn)>-100){
    //     u_posn = u_posn_old;
    // }
    // u_posn_old = u_posn;
    

    if (u_posn > POSN_OUT_MAX) {
		u_posn = POSN_OUT_MAX;
	} else if (u_posn < -POSN_OUT_MAX) {
		u_posn = -POSN_OUT_MAX;
	}

    ref_curr_in_pos = u_posn;

    eder_posn = error_posn - eprev_posn;
    eint_posn += error_posn;
    eprev_posn = error_posn;

    // sprintf(debug,"u_posn : %f", u_posn);
    // NU32DIP_WriteUART1(debug);
}

float get_ref_curr_from_pos(){
    return ref_curr_in_pos;
}

void set_deg(int deg){
    desir_deg = deg;
}

int read_deg(){
    return desir_deg;
}

void set_pos_kp(float p_kp){
    pos_kp = p_kp;
}

void set_pos_ki(float p_ki){
    pos_ki = p_ki;
}

void set_pos_kd(float p_kd){
    pos_kd = p_kd;
}

float get_pos_kp(){
    return pos_kp;
}

float get_pos_ki(){
    return pos_ki;
}

float get_pos_kd(){
    return pos_kd;
}

float get_plot_angel_from_posn(int num){
    return trajectory_PLOT_array[num];
}
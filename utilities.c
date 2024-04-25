#include "utilities.h"
#include "nu32dip.h"

volatile enum mode_t mode;
volatile int p; // pwm record
static int trajectoryLength;
static volatile float trajectory_REF_array[1000];

float ref_posn(int number){
  return trajectory_REF_array[number];
}

int get_traj_length(void){
  return trajectoryLength;
}


enum mode_t get_mode(){
    return mode;
}

const char* get_mode_str(){
    switch (mode)
    {
    case IDLE:{
        return "IDLE";
        break;}
    
    case PWM:{
        return "PWM";
        break;}
    
    case ITEST:{
        return "PWM";
        break;}
    
    case HOLD:{
        return "HOLD";
        break;}
    
    case TRACK:{
        return "TRACK";
        break;}
    
    default:{
        return "error";
    }
    }
}

void set_mode(enum mode_t newMode){
    mode = newMode;
}

void set_pwm(int pwm_input){
    p = pwm_input;
}

int get_pwm(){
    return p;
}

void load_trajectory() {
    char input[200];
    NU32DIP_ReadUART1(input, 200);
    sscanf(input, "%d", &trajectoryLength);

    // Store the data sent from client to trajectory_STEP_array
    for (int i=0; i<trajectoryLength; i++) {
        NU32DIP_ReadUART1(input, 200);
        sscanf(input, "%f", &trajectory_REF_array[i]);
    }

    // Print for Debugging
    // sprintf(input, "sending ref traj done\r\n");
    // NU32DIP_WriteUART1(input);

    // sprintf(input, "test %d", get_traj_length());
    // NU32DIP_WriteUART1(input);

    // Testing the Load Trajectory Function: 
    // Send the data in trajectory_STEP_array back to client
    // for (int i=0; i<trajectoryLength; i++) {
    //     sprintf(input, "%f\r\n", trajectory_REF_array[i]);
    //     NU32DIP_WriteUART1(input);
    // }
}

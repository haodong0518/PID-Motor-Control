#ifndef POS_CURRENT__H__
#define POS_CURRENT__H__

#include <xc.h> // processor SFR definitions
#include <sys/attribs.h> // __ISR macro
#include "NU32DIP.h"

void set_deg(int deg);

void set_pos_kp(float p_kp);
void set_pos_ki(float p_ki);
void set_pos_kd(float p_kd);

float get_pos_kp();
float get_pos_ki();
float get_pos_kd();

void PositionControl_StartUp();
float get_ref_curr_from_pos();
void position_PID();
int read_deg();

float get_plot_angel_from_posn(int num);
#endif
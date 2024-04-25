#ifndef UTILITIES__H__
#define UTILITIES__H__


#define BUF_SIZE 200

enum mode_t {IDLE, PWM, ITEST, HOLD, TRACK};
const char* get_mode_str();
enum mode_t get_mode();
void set_mode();
void set_pwm(int pwm_input);
int get_pwm();
void load_trajectory();
float ref_posn(int number);
int get_traj_length(void);

#endif
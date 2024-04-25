#include "encoder.h"
#include "NU32DIP.h"
#include "ina219.h"
#include "utilities.h"
#include "current.h"
#include "pos_control.h"

static volatile float curr_kp = 0, curr_ki= 0;
static volatile float position_kp = 0, position_ki = 0, position_kd = 0;
static volatile int temp_desir_deg;

int main() 
{
  char buffer[BUF_SIZE];
  NU32DIP_Startup();
  INA219_Startup();
  UART2_Startup();

  current_StartUp();
  PositionControl_StartUp();

  NU32DIP_GREEN = 0;
  NU32DIP_YELLOW = 0;      
  __builtin_disable_interrupts();
  // in future, initialize modules or peripherals here
  __builtin_enable_interrupts();

  while(1)
  {
    NU32DIP_ReadUART1(buffer,BUF_SIZE); // we expect the next character to be a menu command
    //NU32DIP_GREEN = 1;
    switch (buffer[0]) {

    // read current sensor 
    case 'b':{
      float curr = INA219_read_current();
      char m[50];
      sprintf(m, "%f\r\n",curr);
      NU32DIP_WriteUART1(m);
      break; 
    }
     // case c: Read Encoder Counts   
    case 'c':{
        WriteUART2("a");
        while (!get_encoder_flag()){};
        set_encoder_flag(0);
        char m[50];
        int p = get_encoder_count();
        sprintf(m ,"%d\r\n", p);
        NU32DIP_WriteUART1(m);
        break;
      }

    // case d: Read encoder (deg)
    case 'd':{
        WriteUART2("a");
        while (!get_encoder_flag()){};
        set_encoder_flag(0);
        char m[50];
        int p = get_encoder_count();
        float deg = ((float)p/384.0)*360.0;
        sprintf(m ,"%f\r\n", deg );
        NU32DIP_WriteUART1(m);
        break;
    }

    // case e: reset the encoder
    case 'e':{
        WriteUART2("b");
        break;
      }

    // case f: Set PWM (-100 to 100)--- face to the gear, clockwise
    // LATAbits.LATA1 = 1; it rotate in the opposite direction
    case 'f':{
      set_mode(PWM);
      sprintf(buffer,"\r\n");
      NU32DIP_WriteUART1(buffer);

      int pwm_input =0;
			NU32DIP_ReadUART1(buffer, BUF_SIZE);
			sscanf(buffer,"%d", &pwm_input);
      sprintf(buffer,"%d\r\n", pwm_input);

      if (pwm_input<0){
        pwm_input = pwm_input* (-1);  
        LATAbits.LATA1 = 1;
      }

      else if(pwm_input > 0){
        LATAbits.LATA1 = 0;
      }

      set_pwm(pwm_input);
      NU32DIP_WriteUART1(buffer);
			break;
    }

    // case g: Set current gains
    case 'g':{
      // sprintf(buffer, "your desired Kp current gain:\r\n");
      // NU32DIP_WriteUART1(buffer);
      NU32DIP_ReadUART1(buffer, BUF_SIZE);
			sscanf(buffer,"%f", &curr_kp);

      // sprintf(buffer,"Enter your desired Ki current gain:\r\n");
      // NU32DIP_WriteUART1(buffer);
      NU32DIP_ReadUART1(buffer, BUF_SIZE);
			sscanf(buffer,"%f", &curr_ki);

      // sprintf(buffer,"Sending Kp = %f and Ki = %f to the current controller.\r\n", curr_kp,curr_ki);
      // NU32DIP_WriteUART1(buffer);
      set_curr_ki(curr_ki);
      set_curr_kp(curr_kp);
      break;
    }
    // case h: Get current gains
    case 'h':{
      sprintf(buffer, "%f %f \r\n", get_curr_kp(),get_curr_ki());
      NU32DIP_WriteUART1(buffer);
      break;
    }

    // case i : Set position gains
    case 'i':{
      NU32DIP_ReadUART1(buffer, BUF_SIZE);
			sscanf(buffer,"%f", &position_kp);

      NU32DIP_ReadUART1(buffer, BUF_SIZE);
			sscanf(buffer,"%f", &position_ki);

      NU32DIP_ReadUART1(buffer, BUF_SIZE);
			sscanf(buffer,"%f", &position_kd);

      set_pos_kp(position_kp);
      set_pos_ki(position_ki);
      set_pos_kd(position_kd);
      break;
    }

    // case j: Get position gains
    case 'j':{
      sprintf(buffer, "%f %f %f\r\n", get_pos_kp(),get_pos_ki(), get_pos_kd());
      NU32DIP_WriteUART1(buffer);
      break;
    }

    // case k: Test current control
    case 'k': {
      set_mode(ITEST);
      char m[50];
      for (int j=0; j<100; j++) { // send plot data to MATLAB
			// when first number sent = 1, MATLAB knows we’re done
			  sprintf(m, "%d %d %d\r\n", 100-j,(int)ADCarray[j], (int)Waveform[j]);
			  NU32DIP_WriteUART1(m);
      }
      break;
    }

    //case l: Go to angle (deg)
    case 'l':{
      NU32DIP_ReadUART1(buffer, BUF_SIZE);
			sscanf(buffer,"%d", &temp_desir_deg);
      set_deg(temp_desir_deg);
      set_mode(HOLD);
      // sprintf(buffer,"Sending desired degree = %d to the current controller.\r\n", temp_desir_deg);
      // NU32DIP_WriteUART1(buffer);
      break;
    }

    case 'v':{
      sprintf(buffer, "%d \r\n", read_deg());
      NU32DIP_WriteUART1(buffer);
      break;
    }
    // case m: Load step trajectory
    case 'm':{
			//load step trajectory
			load_trajectory();
			break;
		}

		// case n: Load cubic trajectory
		case 'n':
		{
			load_trajectory();
			break;
		}
    // case o: Execute trajectory
    case 'o':{
      // __builtin_disable_interrupts(); // keep ISR disabled as briefly as possible
      // eint = 0;
      // pos_eint = 0;
      // pos_eprev = 0;
      // WriteUART2("b");
      // __builtin_enable_interrupts();  // set ISR back to enabled
      float f;
      set_mode(TRACK);
      while (get_mode() == TRACK) {
        // sprintf(buffer, "in TRACK\r\n");
        // NU32DIP_WriteUART1(buffer);
      }

      // sprintf(buffer, "while done %d\r\n", get_traj_length());
      // NU32DIP_WriteUART1(buffer);
      
      sprintf(buffer, "%d\r\n", get_traj_length());
      NU32DIP_WriteUART1(buffer);

      for (int i=0; i<get_traj_length(); i++) {
          f = get_traj_length()-i;
          // print format: reference current [space] actual current
          sprintf(buffer, "%f %f %f\n",f,get_plot_angel_from_posn(i), ref_posn(i));
          NU32DIP_WriteUART1(buffer);
      }
			break;
		}
    
    // case p: Unpower the motor
    case 'p': {
      set_mode(IDLE);
      set_pwm(0);
      sprintf(buffer, "\r\n");
      NU32DIP_WriteUART1(buffer);
      break;
    }

    // case r: Get mode
    case 'r': {
      sprintf(buffer, "%s \r\n",get_mode_str());
      NU32DIP_WriteUART1(buffer);
      break;
    }


    // q: Quit client
    case 'q':
      {
        // handle q for quit. Later you may want to return to IDLE mode here. 
        break;
      }

        // dummy command for demonstration purposes
    case 'z':{
        int n = 0;
        NU32DIP_ReadUART1(buffer,BUF_SIZE);
        sscanf(buffer, "%d", &n);
        sprintf(buffer,"%d\r\n", n + 1); // return the number + 1
        NU32DIP_WriteUART1(buffer);
        break;
      }

    case 'x':{
        for (int i=0; i<get_traj_length(); i++) {
          sprintf(buffer, "%f\r\n", ref_posn(i));
          NU32DIP_WriteUART1(buffer);
    }
      break;
    }

    default:
      {
        NU32DIP_GREEN = 1;
        NU32DIP_YELLOW = 0;  // turn on LED2 to indicate an error
        break;
      }
    }
  }
  return 0;
}


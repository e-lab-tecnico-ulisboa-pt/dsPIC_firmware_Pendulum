#ifndef MCPWM_H
#define	MCPWM_H

void mcpwm1_init();
void mcpwm2_init();
void mcpwm_init();
void set_pxdcx_12(unsigned int t, unsigned int n, unsigned int val);
void set_pxdcx_34(unsigned int t, unsigned int n, unsigned int val);
void print_lookup_table(unsigned int t);
void restore_phase_and_hold_motor();
void release_motor();
void set_motor_phase(unsigned int phase, unsigned int duty_cap);
void change_omega();
void spin_one_phase();
void spin(double turns, int direction, double omega, double alpha);

//Variable to allow the motor to spin (for protection)
extern int volatile _allow_motor_spin;

//E.g., for Fpwm = 20 kHz and Fcy = 3686.400 kHz
//PWM_PERIOD = Fcy / Fpwm = 184
//If PWM is in continous up/down count mode the PWM_PERIOD calculated as above
//corresponds to half of the actual PWM period, in this way a division by 2 is
//necessary to obtained the desired PWM frequency, e.g., 184 is now 92
#define PWM_PERIOD 92       //~20 kHz        
#define DUTY_CYCLE_RES 184  //=2x PWM_PERIOD (because it is edge of Tcy), ensure it is < 2^14
//#define SINE_CORRECTION 63  //used to correct sine wave (to compensate detent?), see mcpwm.c, should be lower than DUTY_CYCLE_RES
//The SINE_RESOLUTION defines the number of sub-steps (micro-steps) performed
//on a full sine period
#define SINE_RESOLUTION 48  //must be even, >= 4 and <= 8x PWM_PERIOD

//Lookup tables
#define NUMBER_OF_LOOKUP_TABLES 4  //unsigned int, >= 1
#define INITIAL_LOOKUP_TABLE 1        //int, see MAX_POWER definitions above

#define TIMER_ENABLE T1CONbits.TON
#define TIMER_PRESCALER 8  //1->1/1, 8->1/8, 64->1/64 and 256->1/256...

#define PI 3.141592

//This is a mechanical characteristic of the step-motor, see datasheet
//1 step = 7.5 degrees
#define STEPS_PER_SINE 4    //one full sine period corresponds to 4 steps
#define ANGLE_PER_FULL_SINE_PERIOD 30   //degrees
#define MAX_FREQ 3000.0    //Hz, double

#define ACCELERATION_RAMP_RESOLUTION 50    //Integer, must be > 1 and < 2^16 (65536)
#define MIN_ALPHA 1.0    //double
#define MAX_ALPHA 1000000.0    //double

#endif

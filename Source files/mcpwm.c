#include "mcpwm.h"
#include <p33FJ128MC804.h>  //defines dspic register locations and structures definitions
#include <math.h>
#include <stdio.h>
#include "world_pendulum.h"
#include "delays.h"
#include "physical.h"

//VectorS that contains the conversion from phase to duty cylce for a full
//period of a sine wave, associated to each one of the PWM
//For DRV8844
static unsigned int phase_to_P1DC1_P1DC2[NUMBER_OF_LOOKUP_TABLES][SINE_RESOLUTION];
static unsigned int phase_to_PxOUTxH_12[SINE_RESOLUTION];
static unsigned int phase_to_P1DC3_P2DC1[NUMBER_OF_LOOKUP_TABLES][SINE_RESOLUTION];
static unsigned int phase_to_PxOUTxH_34[SINE_RESOLUTION];

//Stores the phase at which the step-motor is, after it has spinned and allows
//to restore the last phase
//Initialises the phase at zero, instead of every time mcpwm_init() is called
//this will allow to call mcpwm_init() during runtime without losing the phase
static unsigned int volatile _current_phase = 0;

//Variable to allow the motor to spin (for protection)
int volatile _allow_motor_spin = NO;

//Variables to be used in spin_one_phase() and spin() functions
unsigned long int full_turns, micro_steps, micro_steps_per_turn;
unsigned long int acc_turns_cap, acc_micro_steps_cap, de_acc_turns_cap, de_acc_micro_steps_cap;
int accelerating, phase_inc;
unsigned int dts[ACCELERATION_RAMP_RESOLUTION]; //time period of each acceleration step (to be used in PRx register for comparison with timer register)
unsigned int dts_n[ACCELERATION_RAMP_RESOLUTION]; //number of time periods spent in each acceleration step 
unsigned int current_dts_index, dts_counting, min_dt_for_lookup_table_2, min_dt_for_lookup_table_3, lookup_table;

void mcpwm1_init() {
    P1TCONbits.PTMOD = 2;   //PWM time base operates in a Continuous Up/Down Count mode
    
    P1TMRbits.PTMR = 0; //PWM Time Base Register Count Value bits
    
    P1TPERbits.PTPER = PWM_PERIOD;  //PWM Time Base Period Value bits
    
    //P1SECMP
    
    PWM1CON1bits.PMOD1 = 1;     //PWM I/O pin pair is in Independent Output mode
    PWM1CON1bits.PMOD2 = 1;     //PWM I/O pin pair is in Independent Output mode
    PWM1CON1bits.PMOD3 = 1;     //PWM I/O pin pair is in Independent Output mode
    PWM1CON1bits.PEN1L = 1;     //pin is enabled for PWM output
    PWM1CON1bits.PEN2L = 1;     //pin is enabled for PWM output
    PWM1CON1bits.PEN3L = 1;     //pin is enabled for PWM output
    PWM1CON1bits.PEN1H = 1;     //pin is enabled for PWM output
    PWM1CON1bits.PEN2H = 1;     //pin is enabled for PWM output
    PWM1CON1bits.PEN3H = 1;     //pin is enabled for PWM output
    
    PWM1CON2bits.IUE = 1;   // immediate updates to the active PxDCy registers
    PWM1CON2bits.OSYNC = 1; //output overrides through the PxOVDCON(1) register are synchronised to the PWM time base
    PWM1CON2bits.UDIS = 0;  //Updates from duty cycle and period buffer registers are enabled
    
    //P1DTCON1
    
    //P1DTCON2
    
    //P1FLTACON
    
    P1OVDCONbits.POVD3H = 0;    //Output on PWMx I/O pin is controlled by the value in the corresponding POUTxH:POUTxL bit
    P1OVDCONbits.POVD2H = 0;    //Output on PWMx I/O pin is controlled by the value in the corresponding POUTxH:POUTxL bit
    P1OVDCONbits.POVD1H = 0;    //Output on PWMx I/O pin is controlled by the value in the corresponding POUTxH:POUTxL bit
    
    P1DC1 = 0;
    P1DC2 = 0;
    P1DC3 = 0;
}

void mcpwm2_init() {
    P2TCONbits.PTMOD = 2;   //PWM time base operates in a Continuous Up/Down Count mode
    
    P2TMRbits.PTMR = 0; //PWM Time Base Register Count Value bits
    
    P2TPERbits.PTPER = PWM_PERIOD;  //PWM Time Base Period Value bits
    
    //P2SECMP
    
    PWM2CON1bits.PMOD1 = 1;     //PWM I/O pin pair is in Independent Output mode
    PWM2CON1bits.PEN1L = 1;     //pin is enabled for PWM output 
    PWM2CON1bits.PEN1H = 1;     //pin is enabled for PWM output
    
    PWM2CON2bits.IUE = 1;   //immediate updates to the active PxDCy registers
    PWM2CON2bits.OSYNC = 1; //output overrides through the PxOVDCON(1) register are synchronised to the PWM time base
    PWM2CON2bits.UDIS = 0;  //Updates from duty cycle and period buffer registers are enabled
    
    //P2DTCON1
    
    //P2DTCON2
    
    //P2FLTACON
    
    P2OVDCONbits.POVD1H = 0;    //Output on PWMx I/O pin is controlled by the value in the corresponding POUTxH:POUTxL bit
    
    P2DC1 = 0;
}

void mcpwm_init() {
	mcpwm1_init();
	mcpwm2_init();
    
    int n, ni, nf;
    unsigned int sine_res_1_8, sine_res_1_4, sine_res_1_2;
    double ref_1[12], ref_2[12];
    double x, y1, y2, max_power, max_power_high, a1, a2, ratio1, ratio2;
    double a;
    
    //Experimental values to obtain more constant load free d_theta at 100% power (12 V)
    ref_1[0] = 0.4961; ref_1[1] = 0.5748; ref_1[2] = 0.6693; ref_1[3] = 0.9055; ref_1[4] = 0.9449; ref_1[5] = 0.9843; ref_1[6] = 1.0;    ref_1[7] = 0.9843; ref_1[8] = 0.9291; ref_1[9] = 0.7953; ref_1[10] = 0.6142; ref_1[11] = 0.5512;
    ref_2[0] = 1.0;    ref_2[1] = 0.9843; ref_2[2] = 0.9291; ref_2[3] = 0.7953; ref_2[4] = 0.6142; ref_2[5] = 0.5512; ref_2[6] = 0.5354; ref_2[7] = 0.5906; ref_2[8] = 0.6772; ref_2[9] = 0.9055; ref_2[10] = 0.9449; ref_2[11] = 0.9843;    
    
    sine_res_1_8 = SINE_RESOLUTION/8;  //12.5%
    sine_res_1_4 = SINE_RESOLUTION/4;  //25%
    sine_res_1_2 = SINE_RESOLUTION/2;  //50%
    max_power = (double) (DUTY_CYCLE_RES - getSineCorrection()) * getMaxPower_total();
    max_power_high = max_power + getSineCorrection(); //does not take into account the sine correction
    ratio1 = 1.0 - ref_1[0];
    ratio2 = 1.0 - ref_2[6];
    
    //The lookup tables calculated in the cycle below derive from the experimental value defined above (ref_1, ref_2) 
    for(n=0; n<sine_res_1_2; n++){
        x = (double)(n) / ((double)(SINE_RESOLUTION) / 24.0); //24 = 2 * 12 because experimental data is for half period only
        ni = (int) (x);
        nf = (int) (x+0.5);
        if(nf >= 12) nf = 0;
        y1 = (ref_1[nf] - ref_1[ni]) * (double)(x - (int)(x)) + ref_1[ni];
        y2 = (ref_2[nf] - ref_2[ni]) * (double)(x - (int)(x)) + ref_2[ni];
        a1 = (y1 - ref_1[0])/ratio1 * max_power;
        a2 = (y2 - ref_2[6])/ratio2 * max_power;
        //speed 0 -> static, to hold the motor
        phase_to_P1DC1_P1DC2[0][n] = (unsigned int) (a1 * getMaxPower_0()) + getSineCorrection();
        phase_to_P1DC3_P2DC1[0][n] = (unsigned int) (a2 * getMaxPower_0()) + getSineCorrection();
        phase_to_P1DC1_P1DC2[0][n+sine_res_1_2] = (unsigned int) (a1 * getMaxPower_0()) + getSineCorrection();
        phase_to_P1DC3_P2DC1[0][n+sine_res_1_2] = (unsigned int) (a2 * getMaxPower_0()) + getSineCorrection();        
        //speed 1 -> spin at low speed (e.g., 50%)
        phase_to_P1DC1_P1DC2[1][n] = (unsigned int) (a1 * getMaxPower_1()) + getSineCorrection();
        phase_to_P1DC3_P2DC1[1][n] = (unsigned int) (a2 * getMaxPower_1()) + getSineCorrection();
        phase_to_P1DC1_P1DC2[1][n+sine_res_1_2] = (unsigned int) (a1 * getMaxPower_1()) + getSineCorrection();
        phase_to_P1DC3_P2DC1[1][n+sine_res_1_2] = (unsigned int) (a2 * getMaxPower_1()) + getSineCorrection();
        //speed 3 -> spin at medium speed (e.g., 100%) sine wave (circle in complex plane)
        phase_to_P1DC1_P1DC2[2][n] = (unsigned int) (a1 * getMaxPower_2()) + getSineCorrection();
        phase_to_P1DC3_P2DC1[2][n] = (unsigned int) (a2 * getMaxPower_2()) + getSineCorrection();
        phase_to_P1DC1_P1DC2[2][n+sine_res_1_2] = (unsigned int) (a1 * getMaxPower_2()) + getSineCorrection();
        phase_to_P1DC3_P2DC1[2][n+sine_res_1_2] = (unsigned int) (a2 * getMaxPower_2()) + getSineCorrection();
        //POUT
        phase_to_PxOUTxH_12[n] = 1; phase_to_PxOUTxH_12[n+sine_res_1_2] = 0;
    }
    //The lookup table calculated below uses theoretical values and does not use the sine correction
    for(n=0; n<sine_res_1_4; n++){
        a = (double) n * PI / (double) sine_res_1_4;
        a1 = sin(a) * max_power_high;
        a2 = cos(a) * max_power_high;
        if(a2 < 0) a2 = -a2;
        //speed 3-> spin at high speed (100%) with maximum torque (square in complex plane)
        if(n < sine_res_1_8) { //1,2,5&6
            phase_to_P1DC1_P1DC2[3][n] = (unsigned int) (a1);
            phase_to_P1DC3_P2DC1[3][n] = (unsigned int) (max_power_high);
            phase_to_P1DC1_P1DC2[3][n+sine_res_1_8] = (unsigned int) (max_power_high);
            phase_to_P1DC3_P2DC1[3][n+sine_res_1_8] = (unsigned int) (a2);
            phase_to_P1DC1_P1DC2[3][n+sine_res_1_2] = (unsigned int) (a1);
            phase_to_P1DC3_P2DC1[3][n+sine_res_1_2] = (unsigned int) (max_power_high);
            phase_to_P1DC1_P1DC2[3][n+sine_res_1_2+sine_res_1_8] = (unsigned int) (max_power_high);
            phase_to_P1DC3_P2DC1[3][n+sine_res_1_2+sine_res_1_8] = (unsigned int) (a2);
        }
        else{ //3,4,7&8
            phase_to_P1DC1_P1DC2[3][n+sine_res_1_8] = (unsigned int) (max_power_high);
            phase_to_P1DC3_P2DC1[3][n+sine_res_1_8] = (unsigned int) (a2);
            phase_to_P1DC1_P1DC2[3][n+sine_res_1_4] = (unsigned int) (a1);
            phase_to_P1DC3_P2DC1[3][n+sine_res_1_4] = (unsigned int) (max_power_high);
            phase_to_P1DC1_P1DC2[3][n+sine_res_1_2+sine_res_1_8] = (unsigned int) (max_power_high);
            phase_to_P1DC3_P2DC1[3][n+sine_res_1_2+sine_res_1_8] = (unsigned int) (a2);
            phase_to_P1DC1_P1DC2[3][n+sine_res_1_2+sine_res_1_4] = (unsigned int) (a1);
            phase_to_P1DC3_P2DC1[3][n+sine_res_1_2+sine_res_1_4] = (unsigned int) (max_power_high);
        }
        //POUT
        phase_to_PxOUTxH_34[n] = 0; phase_to_PxOUTxH_34[n+sine_res_1_4] = 1; phase_to_PxOUTxH_34[n+sine_res_1_2] = 1; phase_to_PxOUTxH_34[n+sine_res_1_2+sine_res_1_4] = 0;
    }
   
    //Global variables initialisations
    _allow_motor_spin = NO;
}

void set_pxdcx_12(unsigned int t, unsigned int n, unsigned int val) {
    unsigned int sine_res_1_2 = SINE_RESOLUTION/2;  //50%
    if(t >= NUMBER_OF_LOOKUP_TABLES) t = NUMBER_OF_LOOKUP_TABLES - 1; //prevent segmentation-fault
    if(n >= sine_res_1_2) n = sine_res_1_2 - 1; //prevent segmentation-fault
    if(val > DUTY_CYCLE_RES) val = DUTY_CYCLE_RES;
    phase_to_P1DC1_P1DC2[t][n] = val;
    phase_to_P1DC1_P1DC2[t][(n + sine_res_1_2)%SINE_RESOLUTION] = val;
}

void set_pxdcx_34(unsigned int t, unsigned int n, unsigned int val) {
    unsigned int sine_res_1_2 = SINE_RESOLUTION/2;  //50%
    if(t >= NUMBER_OF_LOOKUP_TABLES) t = NUMBER_OF_LOOKUP_TABLES - 1; //prevent segmentation-fault
    if(n >= sine_res_1_2) n = sine_res_1_2 - 1; //prevent segmentation-fault
    if(val > DUTY_CYCLE_RES) val = DUTY_CYCLE_RES;
    phase_to_P1DC3_P2DC1[t][n] = val;
    phase_to_P1DC3_P2DC1[t][(n + sine_res_1_2)%SINE_RESOLUTION] = val;
}

void print_lookup_table(unsigned int t) {
    int i;
    if(t >= NUMBER_OF_LOOKUP_TABLES) t = NUMBER_OF_LOOKUP_TABLES - 1; //prevent segmentation-fault
    for(i=0; i<SINE_RESOLUTION; i++)
        printf("i=%u\t%u\t%u\t%u\t%u\t%u\t%u\r", i, phase_to_PxOUTxH_12[i], !phase_to_PxOUTxH_12[i], phase_to_P1DC1_P1DC2[t][i], phase_to_PxOUTxH_34[i], !phase_to_PxOUTxH_34[i], phase_to_P1DC3_P2DC1[t][i]);
}

void restore_phase_and_hold_motor() {
    P1TCONbits.PTEN = 1;
    P2TCONbits.PTEN = 1;
    _allow_motor_spin = YES;
    set_motor_phase(_current_phase, 0);
}

void release_motor() {
    _allow_motor_spin = NO;
    //Switch to slow decay mode to avoid voltage spike on power supply
    P1OVDCONbits.POUT1H = 0;
    P1OVDCONbits.POUT2H = 0;
    P1DC1 = 0xFFFF;
    P1DC2 = 0xFFFF;
    P1OVDCONbits.POUT3H = 0;
    P2OVDCONbits.POUT1H = 0;    
    P1DC3 = 0xFFFF;
    P2DC1 = 0xFFFF;
    //Wait for current to go down and then turn OFF
    delay_ms(5);
    P1DC1 = 0;
    P1DC2 = 0;
    P1DC3 = 0;
    P2DC1 = 0;
    P1TCONbits.PTEN = 0;
    P2TCONbits.PTEN = 0;
}

void set_motor_phase(unsigned int phase, unsigned int t) {
    //Set the PWM duty in accordance to the phase of the sinusoidal wave and 
    //the lookup table (which depends on angular velocity)
    //For DRV8844
    //This function takes about ~18 us at Fcy = 3686400 Hz
    //the phase is defined between 0 and SINE_RESOLUTION-1 (defined in mcpwm.h)
    unsigned int pwm12, pwm34;
    unsigned int pout12, pout34;
    
    pwm12 = phase_to_P1DC1_P1DC2[t][phase];
    pwm34 = phase_to_P1DC3_P2DC1[t][phase];
    pout12 = phase_to_PxOUTxH_12[phase];
    pout34 = phase_to_PxOUTxH_34[phase];
    //The following piece of code (until end of function) takes about 6-7 us to execute
    //This uses the fast decay of DRV8844
    if(pout12 == 0){
        P1OVDCONbits.POUT1H = 0;
        P1OVDCONbits.POUT2H = 1;
        P1DC1 = pwm12;
        P1DC2 = pwm12;
    }
    else{
        P1OVDCONbits.POUT1H = 1;
        P1OVDCONbits.POUT2H = 0;
        P1DC1 = pwm12;
        P1DC2 = pwm12;
    }
    if(pout34 == 0){
        P1OVDCONbits.POUT3H = 0;
        P2OVDCONbits.POUT1H = 1;
        P1DC3 = pwm34;
        P2DC1 = pwm34;
    }
    else{
        P1OVDCONbits.POUT3H = 1;
        P2OVDCONbits.POUT1H = 0;
        P1DC3 = pwm34;
        P2DC1 = pwm34;
    }
    _current_phase = phase;
    //printf("%u\t%u\t%u\t%u\t%u\r", phase, P1DC1, P1DC2, P1DC3, P2DC1);
}

void change_omega(){
    //Check if motor should be de-accelerating
    if( (micro_steps <= de_acc_micro_steps_cap) && (full_turns <= de_acc_turns_cap) ) accelerating = -1;
        
    //If accelerating change timer period counter        
    if(accelerating == 1) {
        if(dts_counting == dts_n[current_dts_index]) {
            //printf("----\r%lu\t%lu\t%lu\t%lu\t%lu\t%lu\t\r", full_turns, acc_turns_cap, de_acc_turns_cap, micro_steps, acc_micro_steps_cap, de_acc_micro_steps_cap);
            //printf("%u\t%d\t%u\t\r", dts_counting, accelerating, current_dts_index);
            current_dts_index++;
            if(current_dts_index >= ACCELERATION_RAMP_RESOLUTION){
                current_dts_index = ACCELERATION_RAMP_RESOLUTION - 1;
                dts_counting--;
            }
            else dts_counting = 0;

            TIMER_ENABLE = 0;       //OFF
            //TMR1 = 0x0000;     //Reset TMR
            PR1 = dts[current_dts_index];  //Set PMR to new value
            TIMER_ENABLE = 1;       //ON

            if(dts[current_dts_index] <= min_dt_for_lookup_table_3) lookup_table = 3;
            else if(dts[current_dts_index] <= min_dt_for_lookup_table_2) lookup_table = 2;
            else lookup_table = INITIAL_LOOKUP_TABLE;
        }
        dts_counting++;
        //printf("%u, ", dts_counting);
        if( (micro_steps <= acc_micro_steps_cap) && (full_turns <= acc_turns_cap) ) accelerating = 0;
    }
    if(accelerating == -1) {
        if(dts_counting == 0) {
            //printf("----\r%lu\t%lu\t%lu\t%lu\t%lu\t%lu\t\r", full_turns, acc_turns_cap, de_acc_turns_cap, micro_steps, acc_micro_steps_cap, de_acc_micro_steps_cap);
            //printf("%u\t%d\t%u\t\r", dts_counting, accelerating, current_dts_index);
            current_dts_index--;
            if((int)current_dts_index < 0) current_dts_index = 0;

            TIMER_ENABLE = 0;       //OFF
            //TMR1 = 0x0000;     //Reset TMR
            PR1 = dts[current_dts_index];  //Set PMR to new value
            TIMER_ENABLE = 1;       //ON

            dts_counting = dts_n[current_dts_index];

            if(dts[current_dts_index] <= min_dt_for_lookup_table_3) lookup_table = 3;
            else if(dts[current_dts_index] <= min_dt_for_lookup_table_2) lookup_table = 2;
            else lookup_table = INITIAL_LOOKUP_TABLE;
        }
        dts_counting--;
        //printf("%u, ", dts_counting);
    }    
}

//Spin the motor by one phase only (to be called inside Timer interrupt)
void spin_one_phase(){
    int phase;
    if( (full_turns > 0) || (micro_steps > 0) ){
        //check acceleration state and update angular velocity in accordance
        change_omega();
        
        //Change phase
        phase = (int) _current_phase;
        phase = phase + phase_inc;
        if(phase < 0) phase = SINE_RESOLUTION - 1;
        if(phase >= SINE_RESOLUTION) phase = 0;
        set_motor_phase(phase, lookup_table);
    
        //check number of turns and micro steps
        if( (micro_steps == 0) && (full_turns > 0) ){
            micro_steps = micro_steps_per_turn;
            full_turns = full_turns - 1;
        }
        micro_steps = micro_steps - 1;
    }
    else _allow_motor_spin = NO;
}

//Makes the motor spin a number of turns
void spin(double turns, int direction, double omega, double alpha) {
    //turns is the number of turns that the motor will perform
    //direction is the rotation direction. It can be 88 or 99, check header file
    //omega is the angular speed (rad/s)
    //alpha is the angular acceleration (rad/s/s)

    ///////////////////////////////////////
    // Truncate inputs (omega and alpha) //
    ///////////////////////////////////////
    //minimum angle possible (corresponds to one micro-step)
    double d_theta_min = PI * (double)ANGLE_PER_FULL_SINE_PERIOD / (180.0 * (double)SINE_RESOLUTION);
    //maximum time per micro-step
    double dt_max = (65535.0 * (double)TIMER_PRESCALER) / (double)FCY;  //0xFFFF - 1
    //calculates minimum omega based on minium angel and maximum dt possible
    double omega_min = d_theta_min / dt_max;
    if(omega < omega_min) omega = omega_min; //truncate omega to minimum if input is lower
    //calculates maximum omega based on the maximum motor pulse (step) per second (pps) and angle per pulse
    double omega_max = 2.0*PI*MAX_FREQ / ( (360.0 * (double)STEPS_PER_SINE) / (double)ANGLE_PER_FULL_SINE_PERIOD );
    if(omega > omega_max) omega = omega_max; //truncate omega to maximum if input is higher
    if(alpha < MIN_ALPHA) alpha = MIN_ALPHA;
    if(alpha > MAX_ALPHA) alpha = MAX_ALPHA;
    
    ////////////////////////////////////////////////////////////////////
    // Calculate number of full turns and micro-steps to be performed //
    ////////////////////////////////////////////////////////////////////
    full_turns = (unsigned long int) turns; //number of full turns rounded down to integer
    double fraction_turn = turns - (double) full_turns;  //remaining fraction of a turn (if any)
    unsigned int periods = 360 / ANGLE_PER_FULL_SINE_PERIOD;  //number of full sine periods in one full turn
    //calculates the number of micro-steps in the remaining fraction of turn
    micro_steps = (unsigned long int) (fraction_turn * (double) periods * (double) SINE_RESOLUTION);
    micro_steps_per_turn = periods * SINE_RESOLUTION;
    
    ///////////////////////////////////////////
    // Calculates the full acceleration ramp //
    ///////////////////////////////////////////
    unsigned long int dts_n_total = 0; //total number of time periods for the full acceleration ramp
    unsigned int max_dts_n = 0xFFFF / ACCELERATION_RAMP_RESOLUTION; //maximum number of time periods per acceleration step to prevent overflow of dts_n_total
    double tf = (omega - omega_min) / alpha; //total acceleration time
    double dt = tf / (double) (ACCELERATION_RAMP_RESOLUTION - 1); //time period (s) of each acceleration step
    double t; //time (elapsed time)
    double denominator; //auxiliary variable, corresponds to omega
    double timer_period = (double)TIMER_PRESCALER / (double)FCY; //timer period unit
    double dts_aux;
    min_dt_for_lookup_table_2 = 0; //stores dt as threshold to use lookup table no. 2
    min_dt_for_lookup_table_3 = 0; //stores dt as threshold to use lookup table no. 3
    unsigned int n;
    for(n=0; n<ACCELERATION_RAMP_RESOLUTION; n++) {
        t = (double) n * dt; //time (elapsed time)
        denominator = (omega_min + alpha*t); //current omega
        dts_aux = (d_theta_min/denominator) / timer_period; //calculate the number of timer period units, i.e., omega for the current acceleration step
        if(dts_aux > (double)0xFFFF) dts_aux = (double)0xFFFF;
        dts[n] = (unsigned int) dts_aux;
        if(dts[n] == 0) dts[n] = 1;
        if((denominator >= getOmega_2()) && (min_dt_for_lookup_table_2 == 0)) min_dt_for_lookup_table_2 = dts[n];
        if((denominator >= getOmega_3()) && (min_dt_for_lookup_table_3 == 0)) min_dt_for_lookup_table_3 = dts[n];
        dts_n[n] = (unsigned int) ( dt / (d_theta_min/denominator) ); //calculate the number of time periods to perform, i.e., how long it stays at the current omega
        if(dts_n[n] == 0) dts_n[n] = 1;
        if(dts_n[n] > max_dts_n) dts_n[n] = max_dts_n;
        dts_n_total += dts_n[n]; //calculates the total number of time periods for the full acceleration ramp
        //printf("%u\t%u\t%lf\r", dts[n], dts_n[n], denominator);
    }
    //printf("min_dt_for_duty_cap_0 = %u\r", min_dt_for_duty_cap_0);
    //printf("dts_n_total = %lu\r", dts_n_total);
    
    ////////////////////////////////////////////////////////////////////////////////////
    // Check if requested turns allow for full acceleration and de-acceleration ramps //
    ////////////////////////////////////////////////////////////////////////////////////
    //If not, then calculates the acceleration ramp cap
    unsigned long int acc_ramp_cap;
    //if the full acceleration + de-acceleration ramps are smaller than the total number of turns
    //printf("%lf\t%lf\r", turns * (double)micro_steps_per_turn, 2.0 * (double)dts_n_total);
    if( (turns * (double)micro_steps_per_turn) > (2.0 * (double)dts_n_total) ) {
        de_acc_turns_cap = (unsigned long int) ( dts_n_total / micro_steps_per_turn );
        de_acc_micro_steps_cap = (unsigned long int) ( dts_n_total - (de_acc_turns_cap * micro_steps_per_turn) );
        acc_turns_cap = full_turns - de_acc_turns_cap;
        if(micro_steps >= de_acc_micro_steps_cap) acc_micro_steps_cap = micro_steps - de_acc_micro_steps_cap;
        else{
            acc_micro_steps_cap = micro_steps_per_turn - de_acc_micro_steps_cap + micro_steps;
            acc_turns_cap--;
        }
        de_acc_micro_steps_cap++; //Not sure why I had do increment by one
    }
    //if the full acceleration + de-acceleration ramps are larger than the total number of turns
    else{
        acc_ramp_cap = (unsigned long int) ( (turns * (double)micro_steps_per_turn) / 2.0 );
        de_acc_turns_cap = (unsigned long int) ( turns / 2.0 );
        de_acc_micro_steps_cap = (unsigned long int) ( acc_ramp_cap - (de_acc_turns_cap * micro_steps_per_turn) );
        acc_turns_cap = de_acc_turns_cap;
        acc_micro_steps_cap = de_acc_micro_steps_cap;
    }
    //printf("turns_cap = %lu\t%lu\r", acc_turns_cap, de_acc_turns_cap);
    //printf("micro_steps_cap = %lu\t%lu\r", acc_micro_steps_cap, de_acc_micro_steps_cap);

    
    ///////////////////////////////////////////////
    // Some more definitions and initialisations //
    ///////////////////////////////////////////////
    phase_inc = 0;
    if(direction == 88) phase_inc = -1;
    if(direction == 99) phase_inc = 1;
    lookup_table = INITIAL_LOOKUP_TABLE;
    current_dts_index = 0;
    dts_counting = 1;   //should start at 1
    accelerating = 1;   //1:yes, 0:no, -1:de-accelerating
    
    ///////////////////////////////////////////////////////
    // Cycle to wait until the motor has finished moving //
    ///////////////////////////////////////////////////////
    restore_phase_and_hold_motor();
    TMR1 = 0x0000;     //Reset TMR
    PR1 = dts[0];
    TIMER_ENABLE = 1; //ON
    
    while(_allow_motor_spin == YES){
        asm("clrwdt");
    }
    
    TIMER_ENABLE = 0; //OFF
    release_motor();    
}

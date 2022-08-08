#include <p33FJ128MC804.h>
#include <math.h>
#include "world_pendulum.h"
#include "shovel.h"
#include "delays.h"
#include "ball.h"
#include "laser.h"
#include "physical.h"
#include "etc.h"
#include "mcpwm.h"

#include <stdio.h>

static int volatile _Gl_dir;
static int volatile _move_called_from_move_to_photodiode_function = NO;
static int volatile _Gl_ball_and_shovel_at_photodiode = YES;
static int _Gl_distance = 0;
static int volatile in_prepare_launch = NO;

double last_v_launch = 0.0;
double last_a_launch = 0.0;
double last_v_catch  = 0.0;
double last_a_catch  = 0.0;

void prepare_launch(double cm) {
    double d;
	if(cm > (double)getDeltaXMax_CM()) cm = (double)getDeltaXMax_CM();
	if(cm < 1.0) cm = 1.0;
    //printf("cm=%lf\r", cm);
    
    in_prepare_launch = YES; //To be used in stop_ball()
	stop_ball();
    
    //d = cm + 1.0; //+1.0 is the margin given in stop_ball()
    d = cm - getInitialOffset_CM();
    //printf("d=%lf\r", d);
	if (d > 0) move (d, DIRECTION_FORWARD, 2.0, ACCELERATION_WITHOUT_SLIDING);
    else move (-d, DIRECTION_BACKWARD, 2.0, ACCELERATION_WITHOUT_SLIDING);

    //Prevents the shovel to slip due to the horizontal force from the pendulum
    restore_phase_and_hold_motor();
    
    //Allows the ball to stabilise before launching
	delay_ms(5000);
	_Gl_distance = cm;
}

void launch_ball() {
    double theta, a, v;
    
    theta = asin( (_Gl_distance) / (getPendulumLength_M() * 100.0) );
    a = 980.0 * _Gl_distance / (getPendulumLength_M() * 100.0); //tangential acceleration at launching position
    a = a * cos(theta); //horizontal acceleration at launching position (launching angle must be <= 45 deg)
    a = a * 1.05; //5% higher for margin
    if(a < 20.0) a = 20.0; //prevent taking too long to go to origin
    v = sqrt(19.6 * getPendulumLength_M() * (1.0 - cos(theta)) ) * 100.0; //2*g = 19.6, maximum velocity cm/s^2
    v = v * 1.05; //5% higher for margin
    if(v < 5.0) v = 5.0; //prevent taking too long to go to origin
    //printf("v=%lf\ta=%lf\r", v, a);
    last_v_launch = v;
    last_a_launch = a;
    move(_Gl_distance - getInitialOffset_CM() + getDistanceLaserToStart_CM() - 0.5, DIRECTION_BACKWARD, v, a);
    
	go_to_origin(2.0, 130.0);
}

void stop_ball() {
	static int ok;
	static int st;
    double theta, a_max, a_min, v_max, aux;
    double d, d_aux, v, v_ball, dt, a; 
    
	st = laser_is_on();
	laser_on();

    //Piece of code to double check if the shovel is at photodiode
    //e.g., executes at boot
    ok = 0;
	if(_Gl_ball_and_shovel_at_photodiode == YES) {
        //2.0 cm accounts for the 1.0 cm given at the end of this function and
        //another 1.0 cm to ensure that it full passes over the laser/photodiode
		move (getInitialOffset_CM() + 2.0, DIRECTION_FORWARD, 4.0, 1.0);
		delay_ms(50);
		if(photodiode_is_on() == YES) ok++;
	
        //d+0.5 ensures that it crosses the photodiode/laser "line"
		move (2.0, DIRECTION_BACKWARD, 4.0, 4.0);
		delay_ms(50);
		if(photodiode_is_on() == NO) ok++;
	
        //d+0.5 ensures that it crosses the photodiode/laser "line"
		move (2.0, DIRECTION_FORWARD, 4.0, 4.0);
		delay_ms(50);
		if(photodiode_is_on() == YES) ok++;
	
		if(ok == 3) {
			move_to_photodiode(4.0, 2.0);
			//delay_ms(1000);
            //If stop_ball() was called from prepare_launch()
			if (in_prepare_launch == NO) {
                //Leave ball at vertical position so that it can rest freely, 1.0 cm for margin
                move(getInitialOffset_CM() + 1.0, DIRECTION_BACKWARD, 4.0, 1.0);
                //Tweak, because shovel is no longer at photodiode but I want the stop_ball function to think that it is
                _Gl_ball_and_shovel_at_photodiode = YES;
            }
            else in_prepare_launch = NO;
            //Leave the laser at the same state as before calling stop_ball()
			if(st == NO) laser_off();
			return;
		}
	}

    //If it reached this point this is because the shovel is not at the
    //photodiode
	go_to_origin(20.0, 40.0);

	if(ball_is_stopped() != YES) delay_ms(2000 * getExpectedPeriod_S());

    //printf("%d\r", get_oscillation_mode());
	if(get_oscillation_mode() == OSC_LONG) {
        
        //Calculations to limit v and a to the maximum ones used in launch_ball()
        theta = asin( getDeltaXMax_CM() / (getPendulumLength_M() * 100.0) );
        a_max = 980.0 * getDeltaXMax_CM() / (getPendulumLength_M() * 100.0);
        a_max = a_max * cos(theta);
        a_max = a_max * 1.05; //5% higher for margin
        v_max = sqrt(19.6 * getPendulumLength_M() * (1.0 - cos(theta)) ) * 100.0; //2*g = 19.6, maximum velocity
        v_max = v_max * 1.05; //5% higher for margin
        
		while (get_ball_direction() != DIRECTION_BACKWARD && get_ball_direction() != DIRECTION_FORWARD);
        asm("clrwdt");
		while (get_ball_direction() == DIRECTION_BACKWARD);
        asm("clrwdt");
		while (get_ball_direction() == DIRECTION_FORWARD);
        asm("clrwdt");
        
        //Get ball velocity and truncate its value if needed
        v_ball = get_ball_velocityCM();
        if(v_ball > v_max) v_ball = v_max;
        theta = asin( getSphereDiameter_CM() / (getPendulumLength_M() * 100.0) ); //auxiliary calculation in case v_ball <= 0
        if(v_ball <= 0.0) v_ball = sqrt( 19.6 * getPendulumLength_M() * (1.0 - cos(theta)) ) * 100.0; //takes into account marginal case where the ball stops crossing the laser twice
        //Calculate distance
        theta = v_ball * v_ball / 100.0; //auxiliary calculation to avoid large numbers
        theta = acos( 1.0 - ( theta / (1960.0 * getPendulumLength_M()) ) ); //2*g*100 = 1960
        d = sin(theta) * getPendulumLength_M() * 100; //maximum amplitude in relation to vertical
        //-2.0 cm and 0.1*d are a margin so that the shovel stops before the maximum position of the ball
        d_aux = getVerticalPosition_CM() - getOriginPosition_CM() + d - 2.0;// - 0.1*d;
        if( d_aux > (getMaximumPosition_CM() - getOriginPosition_CM()) ) {
            d_aux = getMaximumPosition_CM() - getOriginPosition_CM() - 0.5; //0.5 cm for margin
        }
        //Calculate acceleration
        a_min = 16.0 * d_aux / (get_oscillation_period() * get_oscillation_period());
        a = (a_max + a_min) / 2.0; //take the average to prevent selecting extreme values
        //printf("a=%lf\ta_min=%lf\ta_max=%lf\r", a, a_min, a_max);
        if(a > a_max) a = a_max;
        //Calculate velocity, it is a 2nd order polynomial, so two velocities are possible
        aux = (get_oscillation_period() / 2.0);
        //printf("V=%lf\t%lf\r", (-aux + sqrt(aux*aux - (4.0*d_aux/a))) / (-2.0/a), (-aux - sqrt(aux*aux - (4.0*d_aux/a))) / (-2.0/a) );
        v = (-aux + sqrt(aux*aux - (4.0*d_aux/a))) / (-2.0/a); //the other solution is (-aux - sqrt(aux*aux - (4.0*d_aux/a))) / (-2.0/a)
        //printf("v=%lf\tv_max=%lf\r", v, v_max);        
        if(v > v_max) v = v_max;
        //in case v is not calculated properly (may happen if experiment parameters are not configured yet)
        if(isNaN((float*)&v)) v = v_ball;
        if(v <= 0.0) v = v_ball;
        //dt is the time to wait before shovel starts to catch the ball
        dt = get_part_2_of_period() * 1000.0 / 2.0; //ms
        if(v_ball > 0.0) dt = dt + ((getInitialOffset_CM()-1.0) / v_ball) * 1000; //-1.0 cm for shovel thickness, this is a small correction
        if(dt < 0.0) dt = 0.0;
        
        //Catch ball
        delay_ms((int) dt);
        last_v_catch  = v;
        last_a_catch  = a;
        //printf("v=%lf\ta=%lf\r", v, a);
        move(d_aux, DIRECTION_FORWARD, v, a);
        delay_ms(500); //For the ball to settle before going further
        move(getMaximumPosition_CM() - d_aux - 10.0, DIRECTION_FORWARD, 4.0, 4.0);
        
        if(test_laser() == NOT_OK) panic(ERR_1);
	}
    else if (get_oscillation_mode() == OSC_SHORT) {
        //Calculate distance to get closer to the ball
        d_aux = getVerticalPosition_CM() - getOriginPosition_CM() - ( getSphereDiameter_CM() - getInitialOffset_CM() ) - 2.0; //-2.0 for margin
        move(d_aux, DIRECTION_FORWARD, 5.0, 5.0);
        //dt is the time to wait before shovel starts to catch the ball
        dt = get_oscillation_period() * 500.0; //* 500.0 = * 1000.0 / 2.0 ms
        dt = dt * 1.1; //10% larger to not hit the ball when oscillation is maximum
        if(dt < 0.0) dt = 0.0;
        //Calculate new distance to catch the ball -> corresponds to maximum oscillation amplitude when OSC_SHORT
        d_aux = 2.0 * (getSphereDiameter_CM() - getInitialOffset_CM()) + 2.0;
        //Calculate velocity to reach that point before ball starts to go backward
        v = d_aux / ( (get_oscillation_period() / 2.0) );
        //Define acceleration
        a = (980.0 * getSphereDiameter_CM()) / (getPendulumLength_M()*100.0);
        //check PHOTODIODE_PIN states in schematic (should be the same as in
        //ball.c interrupt function)
        while(PHOTODIODE_PIN == 1); //dark, ball is going forward then backward
        asm("clrwdt");
        while(PHOTODIODE_PIN == 0); //light, ball is going forward then backward
        asm("clrwdt");
        while(PHOTODIODE_PIN == 1); //dark, ball is going forward then backward
        asm("clrwdt");
        
        //Catch ball
        delay_ms((int) dt);
        //printf("d = %f\r", d_aux);
        //printf("v = %f\r", v);
        last_v_catch  = v;
        last_a_catch  = a;
        move(d_aux, DIRECTION_FORWARD, v, a);
        delay_ms(500); //For the ball to settle before going further
        //Stops 10.0 cm before reaching maximum position
        d_aux = (getVerticalPosition_CM() - getOriginPosition_CM()) + (getSphereDiameter_CM() - getInitialOffset_CM());
        move(getMaximumPosition_CM() - d_aux - 10.0, DIRECTION_FORWARD, 5.0, 5.0);
        
        if(test_laser() == NOT_OK) panic(ERR_1);
    }
	else {
        //Calculate distance to get closer to the ball
        d_aux = getVerticalPosition_CM() - getOriginPosition_CM() - 2.0*getInitialOffset_CM() - 2.0; //-2.0 for margin
        move(d_aux, DIRECTION_FORWARD, 5.0, 5.0);
        
        //Catch ball
        //Stops 10.0 cm before reaching maximum position
        last_v_catch  = 4.0;
        last_a_catch  = 4.0;
        move(getMaximumPosition_CM() - d_aux - 10.0, DIRECTION_FORWARD, 4.0, 4.0);
        
        if(test_laser() == NOT_OK) panic(ERR_1);
	}
    //printf("last\r");
	delay_ms(2000); //For ball to settle
	move_to_photodiode(2.0, 2.0);
	if(st == NO) laser_off();
    //If stop_ball() was called from prepare_launch()
	if (in_prepare_launch == NO) {
        //Leave ball at vertical position so that it can rest freely, 1.0 cm for margin
        move(getInitialOffset_CM() + 1.0, DIRECTION_BACKWARD, 4.0, 1.0);
        //Tweak, because shovel is no longer at photodiode but I want the stop_ball function to think that it is
        _Gl_ball_and_shovel_at_photodiode = YES;
    }
    else in_prepare_launch = NO;
}

void go_to_origin(double v, double a) {
	if(shovel_is_at_origin() == YES) return;
    
    move(getMaximumPosition_CM() - getOriginPosition_CM(), DIRECTION_BACKWARD, v, a);
    
    move(getMaximumPosition_CM() - getOriginPosition_CM(), DIRECTION_BACKWARD, 1.0, 1.0);
    
	if(shovel_is_at_origin() == NO) panic(ERR_2);
}

void move(double cm, int direction, double v, double a) {
    //v is in cm/s
    //a is in cm/s^2
	if(direction == DIRECTION_FORWARD || direction == DIRECTION_BACKWARD) _Gl_dir = direction;
	else return;
	if(cm > (getMaximumPosition_CM() - getOriginPosition_CM()) )  cm = getMaximumPosition_CM() - getOriginPosition_CM();
	if(cm <= 0) return;
	if(shovel_is_at_origin() == YES && _Gl_dir == DIRECTION_BACKWARD) return;

    spin(convert_cm_to_turns(cm), _Gl_dir, convert_v_to_omega(v), convert_a_to_alpha(a));

}

void move_to_photodiode(double v, double a) {
    //If the shovel is between photodiode and origin, this function sends the 
    //shovel to the origin if nothing stands between the laser and photodiode
    //If photodiode is dark then this function does not move the shovel
	static int st;
	if(shovel_is_at_origin() == YES) return;

	st = laser_is_on();
	laser_on();
	
    _move_called_from_move_to_photodiode_function = YES;
	move(getMaximumPosition_CM() - getOriginPosition_CM(), DIRECTION_BACKWARD, v, a);

	if(st == NO) laser_off(); //keeps the laser at same state before calling this function
}

int shovel_is_at_origin() {
	if(MICROSWITCH == 1) { LED2_OFF; return NO; }
	else { LED2_ON; return YES; }
}

int shovel_is_at_photodiode() {
    return _Gl_ball_and_shovel_at_photodiode;
}

double convert_cm_to_turns(double cm) {
    return ( cm / (PI * getPulleyDiameter_cm()) );
}

//Convert cm/s to rad/s
double convert_v_to_omega(double v) {
    return ( v / (getPulleyDiameter_cm()/2) );
}

//Convert cm/(s^2) to rad/(s^2)
double convert_a_to_alpha(double a) {
    return ( a / (getPulleyDiameter_cm()/2) );
}

//stepping
void __attribute__((__interrupt__, __no_auto_psv__)) _T1Interrupt(void) {
    //This interrupt function takes about 50 us to execute with Fcy=3686400 Hz
	IFS0bits.T1IF = 0;
    
    //"if" to stop at the photodiode only if the move() function was called 
    //from the move_to_photodiode() function
    if( (_move_called_from_move_to_photodiode_function == YES) && (photodiode_is_on() == NO) ) {
        _move_called_from_move_to_photodiode_function = NO;
        _allow_motor_spin = NO;
        _Gl_ball_and_shovel_at_photodiode = YES;
        return;
    }
    
	if(shovel_is_at_origin() == YES && _Gl_dir == DIRECTION_BACKWARD) {
        _allow_motor_spin = NO;        
		return;
	}

    if(_allow_motor_spin == YES) spin_one_phase();
     
	_Gl_ball_and_shovel_at_photodiode = NO;
}

#include <p33FJ128MC804.h>
#include <math.h>
#include "world_pendulum.h"
#include "delays.h"
#include "ball.h"
#include "laser.h"
#include "shovel.h"
#include "physical.h"
#include "state_machine.h"
#include "pendulum_N_oscs.h"

#include <stdio.h>

static volatile double valOscillationPeriod;
static volatile double valLinearVelocityCM;
static volatile double calculatedG;
static volatile int ballDirection = NOT_AVAIL;
static volatile int _Gl_ball_is_stopped = NOT_AVAIL;
static volatile int oscillationMode = NOT_AVAIL;
static volatile int movementCounter;
static volatile int dataPointAvailable = NO;
static volatile int RESTART = YES;
static volatile int acquisitionOngoing = NO;
static volatile double valPeriodPART1;
static volatile double valPeriodPART2;

void input_capture_1_init(){
    //photodiode: RP16 (AN6)
    AD1PCFGLbits.PCFG6 = 1;     //set pin to digital
	TRISCbits.TRISC0 = 1;       //set pin as input
    RPINR7bits.IC1R = 0b10000;  //RP16 is remapped to Input Capture 1
	IPC0bits.IC1IP = 6;			//set priority of IC1 interruption
	IC1CONbits.ICTMR = 1;		//Input Capture Timer Select bits: 1 = TMR2, 0 = TMR3
	IC1CONbits.ICM = 0b001;		//Capture mode, every edge (rising and falling)    
	IFS0bits.IC1IF = 0;
	IEC0bits.IC1IE = 1;
}

//void __attribute__((__interrupt__, __no_auto_psv__)) _CNInterrupt(void) {

void __attribute__((__interrupt__, __no_auto_psv__)) _IC1Interrupt(void) {
    //This interrupt function takes between 200-250 us to execute with Fcy=3686400 Hz
    static unsigned int u1, u2, u3, u4, dummy;
    static unsigned long myLong;
    //static double valPeriodPART1;
    //static double valPeriodPART2;
    static double myDouble;
    static double TTT;
    static int counter;
    static int photoDiode;

    //	IFS0bits.CNIF = 0;
    IFS0bits.IC1IF = 0;

    dummy = IC1BUF;

    /*
        WPA: PHOTODIODE_PIN=0 => light
             PHOTODIODE_PIN=1 => dark
     */
    if (PHOTODIODE_PIN == 0) {
        photoDiode = LIGHT;
        LED1_ON;
    } else {
        photoDiode = DARK;
        LED1_OFF;
    }

    movementCounter = 50;
    _Gl_ball_is_stopped = NO;

    if (RESTART == YES && photoDiode == DARK) {
        RESTART = NO;
        TMR3HLD = 0;
        TMR2 = 0;
        valPeriodPART2 = NOT_AVAIL;
        valPeriodPART1 = NOT_AVAIL;
        ballDirection = NOT_AVAIL;
        counter = 0;
        return;
    }

    if (photoDiode == DARK) {
        u1 = TMR2;
        u2 = TMR3HLD;
        TMR3HLD = 0;
        TMR2 = 0;
        myLong = u2;
        myLong = myLong << 16;
        myLong |= u1;
        valPeriodPART2 = valPeriodPART1;
        valPeriodPART1 = (double) myLong / getClockFrequency_Hz_fast();

        if (valPeriodPART1 > 0 && valPeriodPART2 > 0) {
            TTT = 0.9 * getExpectedPeriod_S();
            //printf("TTT = %f\r", TTT);
            //printf("valPeriodPART1 = %f\r", valPeriodPART1);
            //printf("valPeriodPART2 = %f\r", valPeriodPART2);
            if (valPeriodPART1 >= TTT) {
                oscillationMode = OSC_SHORT;
                valOscillationPeriod = valPeriodPART1;
                ballDirection = NOT_AVAIL;
                counter = 2;
            } else if (valPeriodPART2 >= TTT) {
                oscillationMode = OSC_SHORT;
                ballDirection = NOT_AVAIL;
                counter = 0;
            } else if (valPeriodPART2 < TTT) {
                oscillationMode = OSC_LONG;
                valOscillationPeriod = valPeriodPART1 + valPeriodPART2;
                counter++;
                //F 1.583326 1.080748
                if (valPeriodPART1 > valPeriodPART2) {
                    //ballDirection = DIRECTION_FORWARD;
                    ballDirection = DIRECTION_BACKWARD;
                    //printf("F %lf %lf\r", valPeriodPART1, valPeriodPART2);
                }
                //B 1.080748 1.582652
                else {
                    //ballDirection = DIRECTION_BACKWARD;
                    ballDirection = DIRECTION_FORWARD;
                    //printf("B %lf %lf\r", valPeriodPART1, valPeriodPART2);
                }
            }
        }

        if (counter == 2) {
            //incGlobalNumberOfOscs();
            counter = 0;
            if (acquisitionOngoing == YES) {
                dataPointAvailable = YES;
                //g = 4 * pi^2 * L / T^2
                calculatedG = 39.478417604 * getPendulumLength_M() / valOscillationPeriod / valOscillationPeriod;
            }
        }
    }

    //velocity measurement
    if (photoDiode == LIGHT) {
        u3 = TMR2;
        u4 = TMR3HLD;
        myLong = u4;
        myLong = myLong << 16;
        myLong |= u3;
        myDouble = (double) myLong / (double) getClockFrequency_Hz_fast();
        TTT = 0.9 * getExpectedPeriod_S();
        if (myDouble < TTT && myDouble > 0 && oscillationMode == OSC_LONG) 
            valLinearVelocityCM = getSphereDiameter_CM() / myDouble;
        else valLinearVelocityCM = 0;
    }
    //end of velocity measurement
}

int is_data_point_available() {
    static int retVal;
    IEC0bits.IC1IE = 0;
    asm("nop");
    retVal = dataPointAvailable;
    dataPointAvailable = NO;
    IEC0bits.IC1IE = 1;
    return retVal;
}

double get_oscillation_period() {
    static double retVal;
    IEC0bits.IC1IE = 0;
    asm("nop");
    retVal = valOscillationPeriod;
    IEC0bits.IC1IE = 1;
    return retVal;
}

double get_ball_velocityCM() {
    static double retVal;
    IEC0bits.IC1IE = 0;
    asm("nop");
    retVal = valLinearVelocityCM;
    IEC0bits.IC1IE = 1;
    return retVal;
}

double get_calculated_G() {
    static double retVal;
    IEC0bits.IC1IE = 0;
    asm("nop");
    retVal = calculatedG;
    IEC0bits.IC1IE = 1;
    return retVal;
}

void start_acquisition() {
    acquisitionOngoing = YES;
}

void reset_acquisition() {
    IEC0bits.IC1IE = 0;
    asm("nop");
    RESTART = YES;
    oscillationMode = NOT_AVAIL;
    dataPointAvailable = NO;
    acquisitionOngoing = NO;
    calculatedG = 0;
    IEC0bits.IC1IE = 1;
}

void stop_acquisition() {
    IEC0bits.IC1IE = 0;
    asm("nop");
    dataPointAvailable = NO;
    acquisitionOngoing = NO;
    calculatedG = 0;
    IEC0bits.IC1IE = 1;
}

int get_ball_direction() {
    return ballDirection;
}

int ball_is_stopped() {
    return _Gl_ball_is_stopped;
}

int photodiode_is_on() {
    if (PHOTODIODE_PIN == 0) return YES;
    else return NO;
}

int get_oscillation_mode() {
    return oscillationMode;
}

double get_part_1_of_period() {
    static double retVal;
    IEC0bits.IC1IE = 0;
    asm("nop");
    retVal = valPeriodPART1;
    IEC0bits.IC1IE = 1;
    return retVal;
}

double get_part_2_of_period() {
    static double retVal;
    IEC0bits.IC1IE = 0;
    asm("nop");
    retVal = valPeriodPART2;
    IEC0bits.IC1IE = 1;
    return retVal;
}

void doAdvancedCalculations(double G) {
    static double calculatedG_Arr[50];
    static double averageG, sigmaG;
    static int Gptr;
    static int cnt;
    static int i;

    calculatedG_Arr[Gptr++] = G;
    if (Gptr == 50) Gptr = 0;

    cnt = 0;
    averageG = 0;
    for (i = 0; i < 50; i++) if (calculatedG_Arr[i] > 0) {
            averageG += calculatedG_Arr[i];
            cnt++;
        }
    if (cnt > 0) averageG = averageG / cnt;
    else averageG = 0;

    cnt = 0;
    sigmaG = 0;
    for (i = 0; i < 50; i++) if (calculatedG_Arr[i] > 0) {
            sigmaG += pow(calculatedG_Arr[i] - averageG, 2);
            cnt++;
        }
    if (cnt > 0) sigmaG /= cnt;
    else sigmaG = 0;
    sigmaG = sqrt(sigmaG);
}


//T5 f=5Hz
void __attribute__((__interrupt__, __no_auto_psv__)) _T5Interrupt(void) {
    static int laser_ON;

    IFS1bits.T5IF = 0;

    laser_ON = laser_is_on();

    IEC0bits.IC1IE = 0;
    asm("nop");

    if (laser_ON == YES && movementCounter > 0) movementCounter--;

    IEC0bits.IC1IE = 1;

    if (laser_ON == NO) {
        movementCounter = 50;
        _Gl_ball_is_stopped = NOT_AVAIL;
        ballDirection = NOT_AVAIL;
        oscillationMode = NOT_AVAIL;
    }

    if (movementCounter == 0) {
        _Gl_ball_is_stopped = YES;
        ballDirection = NOT_AVAIL;
        oscillationMode = NOT_AVAIL;
    }

}

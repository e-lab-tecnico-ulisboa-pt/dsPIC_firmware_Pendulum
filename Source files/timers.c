#include "timers.h"
#include <p33FJ128MC804.h>
#include "world_pendulum.h"
#include "ball.h"
#include "delays.h"

static unsigned int currentTestClockOutPinADPCFG;
static unsigned int currentTestClockOutPinTRIS;
static unsigned int currentTestClockOutPinREAD;
static unsigned int testClockInPeriodFound;
static unsigned int long testClockInperiod;

//For setp motor
void timer_1_init(){
    T1CONbits.TCKPS = 0b01; //prescaler for timer 1:1 1:8 1:64 1:256
	T1CONbits.TGATE = 0;
	T1CONbits.TSYNC = 0;
	T1CONbits.TCS = 0;
	TMR1 = 0;
	PR1 = 0xFFFF;
	IFS0bits.T1IF = 0;
    IPC0bits.T1IP = 7; //Highest priority to prevent motor from stalling at high speed
	IEC0bits.T1IE = 1;
	T1CONbits.TON = 0; //Enabled in move() function in shovel.c
}

//For period measurement (32 bits), uses timer 3 too
void timer_2_init(){
    T2CONbits.TCKPS = 0b00; //prescaler for timer 1:1 1:8 1:64 1:256
	T2CONbits.TGATE = 0;
	T2CONbits.TCS = 0;
	//TMR3HLD = 0;
	TMR2 = 0;
	TMR3 = 0;
	PR3 = 0xFFFF;
	PR2 = 0xFFFF;
	T2CONbits.T32 = 1;
	IFS0bits.T3IF = 0;
	IEC0bits.T3IE = 0;
	T2CONbits.TON = 1;
}

//For messaging
void timer_4_init(){
    T4CONbits.TCKPS = 0b11; //prescaler for timer 1:1 1:8 1:64 1:256
	T4CONbits.TGATE = 0;
	T4CONbits.TCS = 0;
	TMR4 = 0;
	PR4 = FCY / 256 / 2 - 1;
	IFS1bits.T4IF = 0;
	IEC1bits.T4IE = 1;
	T4CONbits.TON = 1;
}

//For checking if ball is stopped (f~5Hz)
void timer_5_init(){
    T5CONbits.TCKPS = 0b11;     //prescaler for timer 1:1 1:8 1:64 1:256
	T5CONbits.TGATE = 0;
	T5CONbits.TCS = 0;
	TMR5 = 0;
	PR5 = FCY / 256 / 5 - 1;
	IFS1bits.T5IF = 0;
	IEC1bits.T5IE = 1;
	T5CONbits.TON = 1;
}

//The dsPic receives a square wave as input and returns its period
//The pin that receives the input wave is the same as the one for the
//photodiode RP16 (AN6)
void start_test_clock_in(unsigned int prescaler){
    testClockInPeriodFound = 0;
    IC1CONbits.ICM = 0; //turn off input capture module 1
    T2CONbits.TON = 0; //Stop timer 2
    if(prescaler > 3) prescaler = 3;
    T2CONbits.TCKPS = prescaler; //0=1:1, 1=1:8, 2=1:64, 3=1:256
	T2CONbits.TGATE = 0;
	T2CONbits.TCS = 0;
	//TMR3HLD = 0;
	TMR2 = 0;
	TMR3 = 0;
	PR3 = 0xFFFF;
	PR2 = 0xFFFF;
	T2CONbits.T32 = 1;
    IPC2bits.T3IP = 0;
	IFS0bits.T3IF = 0;
	IEC0bits.T3IE = 0;
	T2CONbits.TON = 1; //Starts timer
    //Input Capture 2 configuration
    AD1PCFGLbits.PCFG6 = 1;     //set pin to digital
	TRISCbits.TRISC0 = 1;       //set pin as input
    RPINR7bits.IC2R = 0b10000;  //RP16 is remapped to Input Capture 2
	IC2CONbits.ICTMR = 1;		//Input Capture Timer Select bits: 1 = TMR2, 0 = TMR3
    IPC1bits.IC2IP = 7;			//set priority of IC2 interruption
	IFS0bits.IC2IF = 0;
	IEC0bits.IC2IE = 1;
    IC2CONbits.ICM = 0b011;		//Capture mode, every rising edge    
}
void stop_test_clock_in(){
    IC2CONbits.ICM = 0; //turn off input capture module 2
    IPC1bits.IC2IP = 4;			//set priority of IC2 interruption back to default
    input_capture_1_init();
    timer_2_init();
}
void __attribute__((__interrupt__, __no_auto_psv__)) _IC2Interrupt(void) {
    static unsigned int u1, u2;
    IFS0bits.IC2IF = 0;
    u1 = TMR2;
    u2 = TMR3HLD;
    TMR3HLD = 0;
    TMR2 = 0;
    testClockInperiod = u2;
    testClockInperiod = testClockInperiod << 16;
    testClockInperiod |= u1;
    testClockInPeriodFound = 1; 
}
unsigned int long get_test_clock_in_period(){
    if(testClockInPeriodFound == 1){
        testClockInPeriodFound = 0;
        return testClockInperiod;
    }
    else return 0;
}

//The dsPic produces a square wave with a given half period
void start_test_clock_out(unsigned int prescaler, unsigned int pr2, unsigned int pr3){
    T2CONbits.TON = 0; //Stop timer 2
    if(prescaler > 3) prescaler = 3;
    T2CONbits.TCKPS = prescaler; //0=1:1, 1=1:8, 2=1:64, 3=1:256
	T2CONbits.TGATE = 0;
	T2CONbits.TCS = 0;
	//TMR3HLD = 0;
	TMR2 = 0;
	TMR3 = 0;
	PR3 = pr3;
	PR2 = pr2;
	T2CONbits.T32 = 1;
    IPC2bits.T3IP = 7;
	IFS0bits.T3IF = 0;
	IEC0bits.T3IE = 1;
    //Get current pin configuration
    currentTestClockOutPinADPCFG = READ_TEST_CLOCK_OUT_PIN_ADPCFG; //Analog (0) or digital (1) mode
    if(currentTestClockOutPinADPCFG == 1){
        currentTestClockOutPinTRIS = TEST_CLOCK_OUT_PIN_TRIS;
        currentTestClockOutPinREAD = READ_TEST_CLOCK_OUT_PIN;
    }
    //Set pin as output
    SET_TEST_CLOCK_OUT_PIN_ADPCFG; //Set pin in digital (1)
    TEST_CLOCK_OUT_PIN_TRIS = 0;   //Input (1) or Output (0) mode
    asm("nop");
    WRITE_TEST_CLOCK_OUT_PIN = 0;  //Initialise pin at low voltage
    delay_ms(1000);
    WRITE_TEST_CLOCK_OUT_PIN = 1;  //goes to high to trigger a transition
    delay_ms(1000);
    WRITE_TEST_CLOCK_OUT_PIN = 0;  //goes to low to trigger another transition just before starting the timer
    //Starts timer
	T2CONbits.TON = 1;
}
void stop_test_clock_out(){
    T2CONbits.TON = 0;
    IPC2bits.T3IP = 4;
    //Set clock pin configurations back
    if(currentTestClockOutPinADPCFG == 0){
        TEST_CLOCK_OUT_PIN_TRIS = 1;
        CLR_TEST_CLOCK_OUT_PIN_ADPCFG;
        asm("nop");
    }
    else{
        TEST_CLOCK_OUT_PIN_TRIS = currentTestClockOutPinTRIS;
        SET_TEST_CLOCK_OUT_PIN_ADPCFG;        
        asm("nop");
        WRITE_TEST_CLOCK_OUT_PIN = currentTestClockOutPinREAD;
    }
    timer_2_init();
}
void __attribute__((__interrupt__, __no_auto_psv__)) _T3Interrupt(void) {
    IFS0bits.T3IF = 0;
    TOGGLE_TEST_CLOCK_OUT_PIN;
}
 /* 
 * File:   main.c
 * Initial author is tpereira for world pendulum
 * Created on 10 de Novembro de 2012, 12:53
 *
 * Adapted to dsPIC33FJ128MC804 and World Pendulum Alliance hardware from May 
 * 2019 by rhenriques
 * 
 * Code for the World Pendulum Alliance (WPA)
 *
 * Pinout:
 * 
 * Photogate on pin ----------- RP16 (to be used with Input Capture on RP16)
 * End microswitch on pin ----- AN0/RA0 (to be used as digital input pin on RA0)
 * LM35 on pin ---------------- AN1/RA1 (to be used as analog input for ADC, pin
 *                                      configuration performed in adc_init())
 * Laser ---------------------- RA8 (to be used as digital output pin)
 * Light bulb ----------------- RA10 (to be used as digital output pin)
 * Push button ---------------- RA7 (to be used as digital input pin)
 * 
 * Bipolar stepper: (pin to be used as digital outputs)
 * IN1 ------------------------ RB14
 * EN1 ------------------------ RB15
 * IN2 ------------------------ RB12
 * EN2 ------------------------ RB13
 * IN3 ------------------------ RB10
 * EN3 ------------------------ RB11
 * IN4 ------------------------ RC6
 * EN4 ------------------------ RC7
 *
 * Display: (pin to be used as digital outputs)
 * RCK ------------------------ RA4 (X2,22)
 * SCK ------------------------ RA9 (X2,9)
 * DIO ------------------------ RB4 (X2,21)
 * GND ------------------------ (X2,8)
 * 5V ------------------------- (X2,20)
 * 
 * RS232:
 * U2RX ----------------------- RP19 (X2,4)
 * U2TX ----------------------- RP20 (X2,3)
 */

#include "world_pendulum.h"
#include "physical.h"
#include "shovel.h"
#include "delays.h"
#include "ball.h"
#include "laser.h"
#include "msg.h"
#include "adc.h"
#include "state_machine.h"
#include "d7seg_max7219.h"
#include "pendulum_N_oscs.h"
#include "memory.h"
#include "mcpwm.h"
#include "timers.h"

#include <p33FJ128MC804.h>
#include <stdio.h>
#include <string.h>
#include <libpic30.h>
#include <math.h>

//Configuration bits
#pragma config FNOSC = PRI          // Oscillator Source (Primary Oscillator w/o PLL)
#pragma config IESO = OFF           // Two-speed Oscillator Start-up Enable bit
#pragma config POSCMD = XT          // Primary Oscillator Mode (XT)
#pragma config FCKSM = CSDCMD       // Clock Switching and Monitor (Sw Disabled, Mon Disabled)
#pragma config WDTPOST = PS4096     // Set WDT postscaler to 1:4096, 250/4096=0.061035 Hz -> 16.384 s
#pragma config WDTPRE = PR128       // Set WDT presclaer to 1:128, 32/128 = 250 Hz (32 kHz is LPRC oscillator)
#pragma config WINDIS = OFF         // WDT in Non-Window mode
#pragma config FWDTEN = ON          // Watchdog Timer enabled (~16 s)
#pragma config JTAGEN = OFF         // For some reason "picpgm -p_cfg file.hex" puts JTAGE=ON 
                                    // and this creates troubles to the dsPiC program execution.
                                    // This line forces JTAGEN bit to be set to 0, as it is from default 

//Variable to store the reset source
unsigned int last_reset = 0;

/* main code */
int main(void) {
	static int pushButtonFlag;
	static int i;

    last_reset = RCON;  //Stores the reset source
    RCON = 0;           //Clears the reset source register
    
	//~9ms delay (not sure what this delay is for... why not?)
	asm("repeat #16383");
	asm("nop");
	asm("repeat #16383");   //repeated twice because FCY of 33F is twice the one of 30F
	asm("nop");
               
	pushButtonFlag = 0;
    
	//LEDs
    //Input (1) or Output (0) mode
	TRISCbits.TRISC1 = 0;   //LED1 (AN7/RC1)
	TRISCbits.TRISC2 = 0;   //LED2 (AN8/RC2)
    //Analog (0) or digital (1) mode
    AD1PCFGLbits.PCFG7 = 1; //LED1 (AN7/RC1)
    AD1PCFGLbits.PCFG8 = 1; //LED2 (AN8/RC2)
    //Initial state
    LED1_ON;
    LED2_OFF;
    
	for (i=0; i<10; i++) {
		TOGGLE_LED1;
		TOGGLE_LED2;
		delay_ms(200);
	}    
    LED1_OFF;
    LED2_OFF;

	//Enable interrupt nesting
	INTCON1bits.NSTDIS = 0;
	
	//Laser and illuminating light bulb
	laser_off();
	light_bulb_off();
	TRISAbits.TRISA8 = 0;   //laser
	TRISAbits.TRISA10 = 0;  //light bulb
	
	//limit switch
	AD1PCFGLbits.PCFG0 = 1; //set pin to digital
	TRISAbits.TRISA0 = 1;   //set pin as input
    
	//start experiment button
	TRISAbits.TRISA7 = 1;
	
    //This function must be run before enabling the Input Compare interrupt
    //so that it loads the clock frequency variable for faster reading
    getClockFrequency_Hz(); //function defined in physical.c
    
    //Initialise input capture on RP16 for period measurement
	input_capture_1_init(); //function defined in ball.c
	
	//Motor control PWM
    mcpwm_init();
	
	//Initialise ADC
	adc_init();
	
	//T1 for stepping
	timer_1_init();
	
	//T2 (32bit) for period measurement
    timer_2_init();
	
	//T4 for messaging (f=2Hz)
	timer_4_init();
	
	//T5 for checking if ball is stopped (f=5Hz)
	timer_5_init();
	
    //UART 1 for communication via FTDI (USB)
    uart_1_init();
    
    //UART 2 for communication via DB25 (flat cable)
    uart_2_init();
    
    //Define which UART will be used with stdio library, printf, etc...
    //UART1 and UART2 are both initiated as listener (they both can receive 
    //commands and execute them but only one can reply). It can be change 
    //during runtime via the advanced help menu.
    //Set the previously used UART for printf output. If first time, the default
    //value is defined in world_pendulum.h
    __C30_UART = getC30Uart();

    //Set state machine in RESET state	
	set_state(STATE_RESET, NO_ECHO);
	reset_idmsg_timer();
    
    delay_ms(10);
	init_d7seg_max7219(INITIAL_LCD_INTENSITY);
    delay_ms(10);
    
	retrieveGlobalNumberOfOscs();

	printf("WP START (RCON=%u)\r", last_reset);

    delay_ms(10);
    write_unsigned_long_int_to_display(getGlobalNumberOfOscs());
    delay_ms(10);
    
	while(1) {
		asm("clrwdt");
        
		if(PUSH_BUTTON == 0 && pushButtonFlag == 0) {
			pushButtonFlag = 1;
			delay_ms(10);
            init_d7seg_max7219(getLCDIntensity());
            write_unsigned_long_int_to_display(getGlobalNumberOfOscs());
			if(get_state() == STATE_RESET || get_state() == STATE_STOPPED) {
				//This will start a local experiment
				printf("CFG\t10\t20\r");
				set_deltaX(10);
				set_Noscillations(20);
				set_state(STATE_CONFIGURED, ECHO);
				state_machine();
				delay_ms(1000);
				printf("STR\r");
				set_state(STATE_STARTED, ECHO);
			}
		}
		
		if(PUSH_BUTTON == 1) pushButtonFlag = 0;

		processMessage(NORMAL_MODE);
		state_machine();
	}
}//End of main

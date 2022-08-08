#include <p33FJ128MC804.h>
#include "world_pendulum.h"
#include "delays.h"
#include "ball.h"
#include "laser.h"
#include "etc.h"

int test_laser() {
	static int good;
	static int i;
	static int st;
	static int retval;

	//IEC0bits.CNIE = 0;
	//IEC0bits.IC1IE = 0;
	//asm("nop");

	st = laser_is_on();
	good = 0;

	for (i=0; i<5; i++) {
        //printf("%d\r", i);
		laser_on(); if(photodiode_is_on() == YES) {
            //printf("on\r");
            good++;
        }
		laser_off(); if(photodiode_is_on() == NO) {
            //printf("off\r");
            good++;
        }
	}
    //printf("%d\r", good);
	if(st == YES) laser_on();

	//IFS0bits.CNIF = 0;
	//IEC0bits.CNIE = 1;
	//IFS0bits.IC1IF = 0;
	//IEC0bits.IC1IE = 1;

	if(good == 10) retval = OK;
	else retval = NOT_OK;
    
	if (retval == OK) relax(ERR_1);

	return retval;
}

int laser_is_on() {
	if(PORTAbits.RA8 == 1) return YES;
	else return NO;
}

int laser_is_off() {
	if(PORTAbits.RA8 == 1) return NO;
	else return YES;
}

void laser_on() {
    asm("bset LATA, #8");
	delay_ms(50);   //necessary because of rc filter on photodiode (4k7, 2u2, tau~10ms)
    //printf("lsr on\r");
}

void laser_off() {
    asm("bclr LATA, #8");
	delay_ms(50);   //necessary because of rc filter on photodiode (4k7, 2u2, tau~10ms)
    //printf("lsr off\r");
}

void laser_toggle() {
    asm("btg LATA, #8");
	delay_ms(50);   //necessary because of rc filter on photodiode (4k7, 2u2, tau~10ms)
}

void light_bulb_on() {
    asm("bset LATA, #10");
  	delay_ms(50);   //maybe necessary because of current peak
}

void light_bulb_off() {
    asm("bclr LATA, #10");
  	delay_ms(50);   //maybe necessary because of current peak
}

void light_bulb_toggle() {
    asm("btg LATA, #10");
  	delay_ms(50);   //maybe necessary because of current peak
}

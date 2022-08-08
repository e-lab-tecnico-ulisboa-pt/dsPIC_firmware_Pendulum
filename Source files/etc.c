#include <p33FJ128MC804.h>
#include <stdio.h>

#include "world_pendulum.h"
#include "msg.h"
#include "laser.h"
#include "delays.h"

static int exit_err_1 = 0;

void panic(int error) {
/*
	ERR 1 -> problem with laser/photodiode
	ERR 2 -> problem with microswitch
*/
	static int counter1;
	static int counter2;
	static long counter3;
	static int flag;
	
	counter1 = 0;
	counter2 = 0;
	counter3 = 0;
	flag = 0;

	laser_off();
	light_bulb_off();

	while(1) {
		if(exit_err_1 == 1) {
			exit_err_1 = 0;
			return;
		}

		if(counter1 == 0) { TOGGLE_LED1; TOGGLE_LED2; }
		if(counter2 == 0) {
			if (error == ERR_1) printf("ERR 1\r");
			else if(error == ERR_2) printf("ERR 2\r");
		}
		counter1++;
		counter2++;
		counter3++;
		if(counter1 == 10) counter1 = 0;		//1 second
		if(counter2 == 50) counter2 = 0;		//5 seconds
		if(counter3 == 432000) asm("reset");	//12 hours
		ClrWdt();
		processMessage(PANIC_MODE);

		if(PUSH_BUTTON == 0 && flag == 0) {
			flag = 1;
			laser_toggle();
			if(laser_is_on() == YES) light_bulb_on();
			else light_bulb_off();
		}
		if(PUSH_BUTTON == 1) flag = 0;

		delay_ms(100);
	}
}

void relax(int error) {
	if(error == ERR_1) exit_err_1 = 1;
}

int isNaN (const float* f) {
	const int* rep = ((const int*) f) + 1;
	return ((*rep & 0x7F00) == 0x7F00);
}

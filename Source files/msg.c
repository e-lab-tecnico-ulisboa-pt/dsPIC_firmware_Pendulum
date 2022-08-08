#include <p33FJ128MC804.h>
#include <stdio.h>
#include <string.h>
#include <libpic30.h>   //For __C30_UART

#include "world_pendulum.h"
#include "msg.h"
#include "physical.h"
#include "shovel.h"
#include "laser.h"
#include "delays.h"
#include "state_machine.h"
#include "pendulum_N_oscs.h"
#include "ball.h"
#include "adc.h"
#include "flash.h"
#include "d7seg_max7219.h"
#include "timers.h"
#include "mcpwm.h"

#define SERIAL_BUFFER_SIZE 64

static char RXbuffer[SERIAL_BUFFER_SIZE];		//buffer used to store message from serial port
static volatile int messageReceivedFlag = NO;	//indicates arrival of message through serial port~
extern unsigned int last_reset;                 //Variable to store the reset source defined once at boot in main.c
extern double last_v_launch;    //Variable to store the last velocity used to launch the ball (shovel.c)
extern double last_a_launch;    //Variable to store the last acceleration used to launch the ball (shovel.c)
extern double last_v_catch;     //Variable to store the last velocity used to catch the ball (shovel.c)
extern double last_a_catch;     //Variable to store the last acceleration used to catch the ball (shovel.c)

//UART2 communicates via the FTDI (USB)
void uart_1_init() {
    //Pinout configuration for UART 1
    TRISCbits.TRISC9 = 0;   //RP25 is output
	TRISCbits.TRISC8 = 1;   //RP24 is input   
    RPOR12bits.RP25R = 3;           //RP25 is U1TX
    RPINR18bits.U1RXR = 0b11000;    //RP24 is U1RX    
	//Configuration of UART2
	U1BRG  = BRGVAL;   /* defined in world_pendulum.h */
	U1MODE = 0x8000;   /* Reset UART to 8-n-1, usual pins, and enable */
	U1STA  = 0x0400;   /* Reset status register and enable TX */
	IFS0bits.U1RXIF = 0;
	IEC0bits.U1RXIE = 1;
}

//UART2 communicates via the DB25
void uart_2_init() {
	//Pinout configuration for UART 2
    TRISCbits.TRISC3 = 1;   //RP19 is input
	TRISCbits.TRISC4 = 0;   //RP20 is output
    RPINR19bits.U2RXR = 0b10011;    //RP19 is U2RX
    RPOR10bits.RP20R = 5;           //RP20 is U2TX    
	//Configuration of UART2
	U2BRG  = BRGVAL;   /* defined in world_pendulum.h */
	U2MODE = 0x8000;   /* Reset UART to 8-n-1, usual pins, and enable */
	U2STA  = 0x0400;   /* Reset status register and enable TX */
	IFS1bits.U2RXIF = 0;
	IEC1bits.U2RXIE = 1;
}

//Check if a messages has been received (i.e., if a  '\r' has been sent)
int check_message_receival(char * message) {
    static int flag;
    IEC0bits.U1RXIE = 0;
	IEC1bits.U2RXIE = 0;
	asm("nop");
	flag = messageReceivedFlag;
	messageReceivedFlag = NO;
	if(flag == YES) strcpy(message, RXbuffer);
	IEC0bits.U1RXIE = 1;
    IEC1bits.U2RXIE = 1;
    return flag;
}

void processMessage(int mode) {
	static int flag;
	static char buf[32];
	static char message[SERIAL_BUFFER_SIZE];
	static unsigned int u1;
	static unsigned int u2;
	static unsigned long myULong;
    static double f1, f2, f3;
	static int myInt;
    static unsigned int myU1, myU2, myU3;
    static int prescaler;
    static unsigned int long period = 0;
    
    flag = check_message_receival(message);

	if(flag == NO) return;

	if(mode == NORMAL_MODE) {
		//cur
		if(strcmp(message, "cur") == 0) {
			if(get_state() == STATE_SENDING_DATA) return;
			printf("CUR\t%u\t%u\r", get_deltaX(), get_Noscillations());
			reset_idmsg_timer();
		}
		
		//str
		else if(strcmp(message, "str") == 0) {
			printf("STR\r");
			set_state(STATE_STARTED, ECHO);
		}
		
		//stp
		else if(strcmp(message, "stp") == 0) {
			printf("STP\r");
			set_state(STATE_STOPPED, ECHO);
		}
		
		//rst
		else if(strcmp(message, "rst") == 0) {
			printf("RST\r");
			set_state(STATE_RESET, ECHO);
		}
		
		//ids
		else if(strcmp(message, "ids") == 0) {
			if(get_state() == STATE_SENDING_DATA) return;
			schedule_send_id();
		}
		
		//cfg \t ## \t ####
		else if(sscanf(message, "cfg\t%u\t%u", &u1, &u2) == 2) {
			sprintf(buf, "cfg\t%u\t%u", u1, u2);
			if(strcmp(buf, message) == 0 ) {
				sprintf(buf, "CFG\t%u\t%u", u1, u2);
				printf("%s\r", buf);
				if(get_state() == STATE_STARTED || get_state() == STATE_SENDING_DATA) return;
				if(u1 < getDeltaXMin_CM()) u1 = getDeltaXMin_CM();
				if(u1 > getDeltaXMax_CM()) u1 = getDeltaXMax_CM();
				if(u2 < NOSC_MIN) u2 = NOSC_MIN;
				if(u2 > NOSC_MAX) u2 = NOSC_MAX;
				set_deltaX(u1);
				set_Noscillations(u2);
				set_state(STATE_CONFIGURED, ECHO);
			}
		}
	}
	
	if(strcmp(message, "stop ball") == 0) {
		stop_ball();
		printf("STOP BALL OK\r");
	}
	
	else if(sscanf(message, "prepare launch %lf", &f1) == 1) {
		prepare_launch(f1);
		printf("PREPARE LAUNCH %lf OK\r", f1);
	}
	
	else if(strcmp(message, "launch ball") == 0) {
		launch_ball();
		printf("LAUNCH BALL OK\r");
	}

	else if(strcmp(message, "test laser") == 0) {
		if(test_laser() == OK) printf("LASER IS OK\r");
		else printf("LASER IS NOT OK\r");
	}

	else if(sscanf(message, "go to origin %lf %lf", &f1, &f2) == 2) {
		go_to_origin(f1, f2);
		printf("GO TO ORIGIN %lf %lf OK\r", f1, f2);
	}

	else if(sscanf(message, "move forward %lf %lf %lf", &f1, &f2, &f3) == 3) {
		move(f1, DIRECTION_FORWARD, f2, f3);
		printf("MOVE FORWARD %lf %lf %lf OK\r", f1, f2, f3);
	}

	else if(sscanf(message, "move backward %lf %lf %lf", &f1, &f2, &f3) == 3) {
		move(f1, DIRECTION_BACKWARD, f2, f3);
		printf("MOVE BACKWARD %lf %lf %lf OK\r", f1, f2, f3);
	}
	
	else if(sscanf(message, "move to photodiode %lf %lf", &f1, &f2) == 2) {
		move_to_photodiode(f1, f2);
		printf("MOVE TO PHOTODIODE %lf %lf OK\r", f1, f2);
	}

	else if(sscanf(message, "set sphere diameter %lf", &f1) == 1) {
		saveSphereDiameter_CM(f1);
		printf("SPHERE DIAMETER: %lf cm OK\r", getSphereDiameter_CM());
	}

	else if(sscanf(message, "set maximum position %lf", &f1) == 1) {
		saveMaximumPosition_CM(f1);
		printf("MAXIMUM POSITION: %lf cm OK\r", getMaximumPosition_CM());
	}
	
	else if(sscanf(message, "set pendulum length %lf", &f1) == 1) {
		savePendulumLength_M(f1);
		printf("PENDULUM LENGTH: %lf m OK\r", getPendulumLength_M());
	}
	
	else if(sscanf(message, "set origin position %lf", &f1) == 1) {
		saveOriginPosition_CM(f1);
		printf("ORIGIN POSITION: %lf cm OK\r", getOriginPosition_CM());
	}
    
    else if(sscanf(message, "set vertical position %lf", &f1) == 1) {
		saveVerticalPosition_CM(f1);
		printf("VERTICAL POSITION: %lf cm OK\r", getVerticalPosition_CM());
	}
    
    else if(sscanf(message, "set photodiode position %lf", &f1) == 1) {
		savePhotodiodePosition_CM(f1);
		printf("PHOTODIODE POSITION: %lf cm OK\r", getPhotodiodePosition_CM());
	}    
	
//	else if(sscanf(message, "set catch ball delay %d", &myInt) == 1) {
//		if(myInt < 1) myInt = 1;
//		if(myInt > 2000) myInt = 2000;
//		saveCatchDelay_MS(myInt);
//		printf("CATCH BALL DELAY: %d ms OK\r", getCatchDelay_MS());
//	}
	
	else if(sscanf(message, "set pulley diameter %lf", &f1) == 1) {
		savePulleyDiameter_cm(f1);
		printf("PULLEY DIAMETER: %lf cm OK\r", getPulleyDiameter_cm());
	}
    
    else if(sscanf(message, "set display intensity %d", &myInt) == 1) {
		saveLCDIntensity(myInt);
        init_d7seg_max7219(getLCDIntensity());
        write_unsigned_long_int_to_display(getGlobalNumberOfOscs());
		printf("DISPLAY INTENSITY: %d OK\r", getLCDIntensity());
	}
	
	else if(strcmp(message, "light bulb on") == 0) {
		light_bulb_on();
            printf("LIGHT BULB ON OK\r");
	}
	
	else if(strcmp(message, "light bulb off") == 0) {
		light_bulb_off();
		printf("LIGHT BULB OFF OK\r");
	}
	
	else if(strcmp(message, "laser on") == 0) {
		laser_on();
		printf("LASER ON OK\r");
	}
	
	else if(strcmp(message, "laser off") == 0) {
		laser_off();
		printf("LASER OFF OK\r");
	}

	else if(strcmp(message, "reboot") == 0) {
		setGlobalNumberOfOscs(getGlobalNumberOfOscs());
		printf("REBOOTING...\r");
		delay_ms(300);
		asm("reset");
	}

	else if(sscanf(message, "set global oscillation counter %lu", &myULong) == 1) {
		setGlobalNumberOfOscs(myULong);
        write_unsigned_long_int_to_display(getGlobalNumberOfOscs());
		printf("GLOBAL OSCILLATION COUNTER: %lu OK\r", getGlobalNumberOfOscs());
	}

	else if(strncmp(message, "set ID string ", strlen("set ID string ")) == 0) {
		if(strlen(message) > strlen("set ID string ")) {
			strncpy(buf, message + strlen("set ID string "), 32);
			buf[31] = 0;
			saveIDstring_CHAR(buf);
			printf("ID STRING: %s OK\r", getIDstring_CHAR(buf));
		}
	}
    
    else if(message[0] == '?' && strlen(message) == 1) sendHelp();
    
    else if(strcmp(message, "help") == 0) sendHelp();
    
    ///////////////////
    // Advanced help //
    ///////////////////
    else if(sscanf(message, "print page %d", &myInt) == 1) {
        //19456 -> page of all settable variables
        //20480 -> 1st page of global number of oscillations
        //21504 -> 2nd page of global number of oscillations
		printPage(PAGE_ADDRESS, myInt);
		printf("\rPRINT PAGE: %d OK\r", myInt);
	}
    
    else if(sscanf(message, "test clock out %u %u %u", &myU1, &myU2, &myU3) == 3) {
        printf("Type \"stop\" + enter (\\r) to exit the test.\r");
        if(myU1 == 0) prescaler = 1;
        if(myU1 == 1) prescaler = 8;
        if(myU1 == 2) prescaler = 64;
        if(myU1 >= 3) prescaler = 256;
        f1 = (double) myU2 + (double) myU3 * 65536.0;
        printf("Generating a square wave with half period of: %lf s\r", f1 * (double) prescaler / (double) FCY);
		start_test_clock_out(myU1, myU2, myU3);
        while(1){
            //for cycle to take about 2 s long
            for(myInt=0; myInt<10; myInt++){
                delay_ms(200);
                check_message_receival(message);
                if(strcmp(message, "stop") == 0) break;
                asm("clrwdt");
            }
            //Outputs the temperature every 2 s, approximately
            printf("temperature: %.2f degC\r", get_temperature());
            if(strcmp(message, "stop") == 0) break;            
        }
        stop_test_clock_out();
		printf("TEST CLOCK OUT OK\r");
	}
    
    else if(sscanf(message, "test clock in %u", &myU1) == 1) {
        if(myU1 == 0) prescaler = 1;
        if(myU1 == 1) prescaler = 8;
        if(myU1 == 2) prescaler = 64;
        if(myU1 >= 3) prescaler = 256;
        printf("Type \"stop\" + enter (\\r) to exit the test.\r");
        start_test_clock_in(myU1);
        while(1){
            period = get_test_clock_in_period();
            if(period != 0){
                printf("%lf s\r", (double) ((double) period * (double) prescaler) / (double) FCY);
                period = 0;
            }
            check_message_receival(message);
            if(strcmp(message, "stop") == 0) break;
            asm("clrwdt");
        }
        stop_test_clock_in();
		printf("TEST CLOCK IN OK\r");
	}
    
	else if(sscanf(message, "set clock freq %lu", &myULong) == 1) {
		saveClockFrequency_Hz((double) myULong);
		printf("CLOCK FREQ: %lu OK\r", (unsigned long) getClockFrequency_Hz_fast());
	}
    
    else if(strcmp(message, "tx via db25") == 0) {
        printf("TX VIA DB25 OK\r");
        saveC30Uart(2);
        __C30_UART = getC30Uart();
    }
    
    else if(strcmp(message, "tx via ftdi") == 0) {
        printf("TX VIA FTDI OK\r");
        saveC30Uart(1);
        __C30_UART = getC30Uart();
    }
    
    else if(sscanf(message, "sm %lf %u %lf %lf", &f1, &myU1, &f2, &f3) == 4) {
		spin(f1, myU1, f2, f3);
		printf("SPIN MOTOR: %lf %u %lf %lf OK\r", f1, myU1, f2, f3);
	}
    
    else if(sscanf(message, "smp %u %u", &myU1, &myU2) == 2) {
        restore_phase_and_hold_motor();
        if(myU1 >= (unsigned)SINE_RESOLUTION) myU1 = (unsigned)SINE_RESOLUTION - 1;
        if(myU2 >= NUMBER_OF_LOOKUP_TABLES) myU2 = NUMBER_OF_LOOKUP_TABLES - 1;
        set_motor_phase(myU1, myU2);
        delay_ms(2000);
		release_motor();
		printf("SET MOTOR PHASE: %u %u OK\r", myU1, myU2);
	}
    
    else if(sscanf(message, "plt %u", &myU1) == 1) {
        if(myU1 >= NUMBER_OF_LOOKUP_TABLES) myU1 = NUMBER_OF_LOOKUP_TABLES - 1;
        print_lookup_table(myU1);
        printf("PRINT LOOKUP TABLE: %u OK\r", myU1);
    }
    
    else if(sscanf(message, "sp12 %u %u %u", &myU1, &myU2, &myU3) == 3) {
        set_pxdcx_12(myU1, myU2, myU3);
		printf("SET PXDCX 12: %u %u %u OK\r", myU1, myU2, myU3);
	}
    
    else if(sscanf(message, "sp34 %u %u %u", &myU1, &myU2, &myU3) == 3) {
        set_pxdcx_34(myU1, myU2, myU3);
		printf("SET PXDCX 34: %u %u %u OK\r", myU1, myU2, myU3);
	}
    
    else if(sscanf(message, "set sine correction %d", &myInt) == 1) {
        saveSineCorrection(myInt);
        mcpwm_init();
		printf("SET SINE CORRECTION: %d OK\r", getSineCorrection());
	}
    
    else if(sscanf(message, "set max power total %lf", &f1) == 1) {
        saveMaxPower_total(f1);
        mcpwm_init();
		printf("SET MAX POWER TOTAL: %lf OK\r", getMaxPower_total());
	}
    
    else if(sscanf(message, "set max power 0 %lf", &f1) == 1) {
        saveMaxPower_0(f1);
        mcpwm_init();
		printf("SET MAX POWER 0: %lf OK\r", getMaxPower_0());
	}
    
    else if(sscanf(message, "set max power 1 %lf", &f1) == 1) {
        saveMaxPower_1(f1);
        mcpwm_init();
		printf("SET MAX POWER 1: %lf OK\r", getMaxPower_1());
	}
    
    else if(sscanf(message, "set max power 2 %lf", &f1) == 1) {
        saveMaxPower_2(f1);
        mcpwm_init();
		printf("SET MAX POWER 2: %lf OK\r", getMaxPower_2());
	}
    
    else if(sscanf(message, "set omega for lt 2 %lf", &f1) == 1) {
        saveOmega_2(f1);
        mcpwm_init();
		printf("SET OMEGA FOR LT 2: %lf OK\r", getOmega_2());
	}
    
    else if(sscanf(message, "set omega for lt 3 %lf", &f1) == 1) {
        saveOmega_3(f1);
        mcpwm_init();
		printf("SET OMEGA FOR LT 3: %lf OK\r", getOmega_3());
	}
    
    else if(strcmp(message, "???") == 0) {
        sendAdvancedHelp();
        if(getC30Uart() == 1) {
            __C30_UART = 2;
            asm("nop");
            sendAdvancedHelp();
            __C30_UART = 1;
        }
        else {
            __C30_UART = 1;
            asm("nop");
            sendAdvancedHelp();
            __C30_UART = 2;
        }
    }
    
    else if(strcmp(message, "advanced help") == 0) {
        sendAdvancedHelp();
        if(getC30Uart() == 1) {
            __C30_UART = 2;
            asm("nop");
            sendAdvancedHelp();
            __C30_UART = 1;
        }
        else {
            __C30_UART = 1;
            asm("nop");
            sendAdvancedHelp();
            __C30_UART = 2;
        }
    }
}




void sendHelp() {
	static char idstring[32];
	getIDstring_CHAR(idstring);

	printf("\r\r\r");
	printf("HELP:\r");
	printf("\r");
	printf("cur\r");
	printf("str\r");
	printf("stp\r");
	printf("rst\r");
	printf("ids\r");
	printf("cfg\tdeltaX[%d:%d]\tN[%u:%u]\r", getDeltaXMin_CM(), getDeltaXMax_CM(), NOSC_MIN, NOSC_MAX);
	printf("\r");

	printf("stop ball\r");
	printf("prepare launch deltaX[%d:%d]\r", getDeltaXMin_CM(), getDeltaXMax_CM());
	printf("launch ball\r");
	printf("test laser\r");
	printf("go to origin speed[cm/s, >0] acceleration[cm/s^2, >0]\r");
	printf("move forward deltaX[cm, >0] speed[cm/s, >0] acceleration[cm/s^2, >0]\r");
	printf("move backward deltaX[cm, >0] speed[cm/s, >0] acceleration[cm/s^2, >0]\r");
	printf("move to photodiode speed[cm/s, >0] acceleration[cm/s^2, >0]\r");
	printf("\r");

	printf("light bulb on\r");
	printf("light bulb off\r");
	printf("laser on\r");
	printf("laser off\r");
	printf("reboot\r");
	printf("\r");

	printf("set maximum position %%lf [%.1lf %.1lf] (cm)\r", MAXIMUM_POSITION_MIN, MAXIMUM_POSITION_MAX);
	printf("set sphere diameter %%lf [%.1lf %.1lf] (cm)\r", SPHERE_DIAMETER_MIN, SPHERE_DIAMETER_MAX);
	printf("set pendulum length %%lf [%.1lf %.1lf] (m)\r", PENDULUM_LENGTH_MIN, PENDULUM_LENGTH_MAX);
	printf("set origin position %%lf [%.1lf %.1lf] (cm)\r", ORIGIN_POSITION_MIN, ORIGIN_POSITION_MAX);
    printf("set vertical position %%lf [%.1lf %.1lf] (cm)\r", VERTICAL_POSITION_MIN, VERTICAL_POSITION_MAX);
    printf("set photodiode position %%lf [%.1lf %.1lf] (cm)\r", PHOTODIODE_POSITION_MIN, PHOTODIODE_POSITION_MAX);
	//printf("set catch ball delay %%d [%d %d] (ms)\r", CATCH_DELAY_MIN, CATCH_DELAY_MAX);
	printf("set pulley diameter %%lf [%.1lf %.1lf] (cm)\r", PULLEY_DIAMETER_MIN, PULLEY_DIAMETER_MAX);
    printf("set display intensity %%d [%d %d]\r", LCD_INTENSITY_MIN, LCD_INTENSITY_MAX);
	printf("set global oscillation counter %%lu [%lu %lu]\r", (long unsigned int) 0, (long unsigned int) (2147483647-NOSC_MAX-1));
	printf("set ID string %%s [maximum length of %d characters]\r", ID_STRING_MAX_LENGTH);
	printf("\r");

	printf("maximum position: %lf cm\r",            getMaximumPosition_CM());
	printf("sphere diameter: %lf cm\r",             getSphereDiameter_CM());
	printf("pendulum length: %lf m\r",              getPendulumLength_M());
	printf("origin position: %lf cm\r",             getOriginPosition_CM());
    printf("vertical position: %lf cm\r",           getVerticalPosition_CM());
    printf("photodiode position: %lf cm\r",         getPhotodiodePosition_CM());
	//printf("catch ball delay: %d ms\r",             getCatchDelay_MS());
	printf("pulley diameter: %lf cm\r",             getPulleyDiameter_cm());
    printf("display intensity: %d\r",               getLCDIntensity());
	printf("expected period: %lf s\r",              getExpectedPeriod_S());
	printf("global oscillation counter: %lu\r",     getGlobalNumberOfOscs());
	printf("uptime: %ld s\r",                       get_uptime());
	printf("ID string: %s\r",                       idstring);
	printf("shovel is at origin: %s\r",             shovel_is_at_origin() == YES ? "yes" : "no");
	printf("shovel is at photodiode: %s\r",         shovel_is_at_photodiode() == YES ? "yes" : "no");
	printf("laser is on: %s\r",                     laser_is_on() == YES ? "yes" : "no");
	printf("photodiode is on: %s\r",                photodiode_is_on() == YES ? "yes" : "no");
	printf("temperature: %.2fC\r",                  get_temperature());
	printf("\r");
	printf("ERR 1 -> problem with laser/photodiode\r");
	printf("ERR 2 -> problem with microswitch\r");
	printf("\r");
    printf("code version: %s %s\r",                 __DATE__, __TIME__);
    printf("\r");
    printf("help\r");
	printf("?\r\r");
}

void sendAdvancedHelp(){
    printf("\r\r\r");
	printf("ADVANCED HELP:\r");
	printf("\r");
    
    printf("print page %%d [e.g.: 19456, 20480 or 21504]\r");
    printf("test clock out %%u %%u %%u [prescaler:0-3] [pr2:0-65364] [pr3:0-65364] (Tcy=271.3ns)\r");
    printf("test clock in %%u [prescaler:0-3]\r");
    printf("set clock freq %%lu [%lu %lu] (Hz, nominal value: 3686400 Hz)\r", (unsigned long)CLOCK_FREQUENCY_MIN, (unsigned long)CLOCK_FREQUENCY_MAX);
    printf("tx via db25 (default)\r");
    printf("tx via ftdi\r");
    printf("sm %%lf %%u %%lf %%lf [turns] [direction: 88 or 99] [omega] [alpha] (spin motor)\r");
    printf("smp %%u %%u [phase:0-%u] [table: 0,...,%u] (set motor phase)\r", (unsigned)SINE_RESOLUTION, (unsigned)NUMBER_OF_LOOKUP_TABLES);
    printf("plt %%u [table: 0,...,%u] (print lookup table)\r", (unsigned)NUMBER_OF_LOOKUP_TABLES-1);
    printf("sp12 %%u %%u %%u [table: 0,...,%u] [position:0-%u] [value:0-%u] (set pxdcx 12)\r", (unsigned)NUMBER_OF_LOOKUP_TABLES-1, SINE_RESOLUTION/2 - 1, DUTY_CYCLE_RES-1);
    printf("sp34 %%u %%u %%u [table: 0,...,%u] [position:0-%u] [value:0-%u] (set pxdcx 34)\r", (unsigned)NUMBER_OF_LOOKUP_TABLES-1, SINE_RESOLUTION-1, DUTY_CYCLE_RES-1);
    printf("set sine correction %%d [%d %d]\r", SINE_CORRECTION_MIN, DUTY_CYCLE_RES);
    printf("set max power total %%lf [%.1lf %.1lf]\r", MAX_POWER_MIN, MAX_POWER_MAX);
    printf("set max power 0 %%lf [%.1lf %.1lf]\r", MAX_POWER_0_MIN, MAX_POWER_0_MAX);
    printf("set max power 1 %%lf [%.1lf %.1lf]\r", MAX_POWER_1_MIN, MAX_POWER_1_MAX);
    printf("set max power 2 %%lf [%.1lf %.1lf]\r", MAX_POWER_2_MIN, MAX_POWER_2_MAX);
    printf("set omega for lt 2 %%lf [%.1lf %.1lf]\r", OMEGA_FOR_LT_2_MIN, OMEGA_FOR_LT_2_MAX);
    printf("set omega for lt 3 %%lf [%.1lf %.1lf]\r", OMEGA_FOR_LT_3_MIN, OMEGA_FOR_LT_3_MAX);
    printf("\r");
    
    printf("clock source (OSCCON.COSC): %u\r",       OSCCONbits.COSC);
    printf("clock freq: %lu Hz\r",                   (unsigned long) getClockFrequency_Hz_fast());
    if(__C30_UART == 1) printf("tx via: ftdi\r");
    if(__C30_UART == 2) printf("tx via: db25\r");
    printf("reset source (RCON): %u\r",              last_reset);
    printf("sine correction: %d/%d\r",               getSineCorrection(), DUTY_CYCLE_RES);
    printf("max power total: %lf\r",                 getMaxPower_total());
    printf("max power 0: %lf\r",                     getMaxPower_0());
    printf("max power 1: %lf\r",                     getMaxPower_1());
    printf("max power 2: %lf\r",                     getMaxPower_2());
    printf("omega for lookup table 2: %lf\r",        getOmega_2());
    printf("omega for lookup table 3: %lf\r",        getOmega_3());
    printf("\r");
    
    printf("last used v and omega to launch the ball: %lf cm/s and %lf rad/s\r", last_v_launch, convert_v_to_omega(last_v_launch));
    printf("last used a and alpha to launch the ball: %lf cm/s^2 and %lf rad/s^2\r", last_a_launch, convert_a_to_alpha(last_a_launch));
    printf("last used v and omega to catch the ball: %lf cm/s and %lf rad/s\r", last_v_catch, convert_v_to_omega(last_v_catch));
    printf("last used a and alpha to catch the ball: %lf cm/s^2 and %lf rad/s^2\r", last_a_catch, convert_a_to_alpha(last_a_catch));
    printf("\r");
    
    printf("advanced help\r");
	printf("???\r\r");
}

/* This is UART1 receive ISR */
void __attribute__((__interrupt__, __no_auto_psv__)) _U1RXInterrupt(void) {
	static char RXb[SERIAL_BUFFER_SIZE];
	static unsigned int counter;
	static int o;
	static char c;
	
   	IFS0bits.U1RXIF = 0;
	
	o = U1STA & 0x0002;		/*OERR bit*/
	
    while(U1STAbits.URXDA) {
	    c = U1RXREG;
	    RXb[counter++] = c;

	    if(c == '\r') {
		    memcpy(RXbuffer, RXb, counter);
		    RXbuffer[counter - 1] = 0;
		    counter = 0;
		    messageReceivedFlag = YES;
		}
		if(counter == SERIAL_BUFFER_SIZE) counter = 0;
	}

	if(o) U1STAbits.OERR = 0;
}

/* This is UART2 receive ISR */
void __attribute__((__interrupt__, __no_auto_psv__)) _U2RXInterrupt(void) {
	static char RXb[SERIAL_BUFFER_SIZE];
	static unsigned int counter;
	static int o;
	static char c;
	
   	IFS1bits.U2RXIF = 0;
	
	o = U2STA & 0x0002;		/*OERR bit*/
	
    while(U2STAbits.URXDA) {
	    c = U2RXREG;
	    RXb[counter++] = c;

	    if(c == '\r') {
		    memcpy(RXbuffer, RXb, counter);
		    RXbuffer[counter - 1] = 0;
		    counter = 0;
		    messageReceivedFlag = YES;
		}
		if(counter == SERIAL_BUFFER_SIZE) counter = 0;
	}

	if(o) U2STAbits.OERR = 0;
}

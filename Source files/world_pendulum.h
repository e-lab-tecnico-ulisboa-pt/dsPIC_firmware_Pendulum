#ifndef WORLD_PENDULUM
#define WORLD_PENDULUM

//#define FCY 1843200UL	//PLL=1 instruction frequency in Hz (used by the pic30 libraries in the function __delay_ms)
//#define FCY 7372800UL	//PLL=4 instruction frequency in Hz (used by the pic30 libraries in the function __delay_ms)
//#define FCY 29491200UL	//PLL=16 instruction frequency in Hz (used by the pic30 libraries in the function __delay_ms)

#define XTFREQ              7372800						//On-board Crystal frequency
#define PLLMODE             1							//On-chip PLL setting (not used for 33FJ128MC804))
#define FCY                 (XTFREQ*PLLMODE/2)			//Instruction Cycle Frequency (3686400 Hz -> 271.3 ns)
#define BAUDRATE            115200						//zorg darg
#define BRGVAL              (((FCY/BAUDRATE)/16)-1)		//used in main.c, (3686400/115200/16)-1=1 used in traps.c -> change it manually

#define TOGGLE_LED1			asm("btg LATC, #1")			//macro for toggling led1
#define LED1				LATCbits.LATC1				//macro for led1
#define LED1_ON				asm("bclr LATC, #1")		//macro for led1 on
#define LED1_OFF			asm("bset LATC, #1")		//macro for led1 off

#define TOGGLE_LED2			asm("btg LATC, #2")			//macro for toggling led2
#define LED2				LATCbits.LATC2				//macro for led2
#define LED2_ON				asm("bclr LATC, #2")		//macro for led2 on
#define LED2_OFF			asm("bset LATC, #2")		//macro for led2 off

#define PUSH_BUTTON			PORTAbits.RA7				//start experiment
#define MICROSWITCH			PORTAbits.RA0               //limit switch

#define PHOTODIODE_PIN		PORTCbits.RC0				//use with input capture IC1

#define OK							33
#define NOT_OK						34

#define YES							13
#define NO							14
#define NOT_AVAIL					-1

#define MAXIMUM_POSITION_MIN        35.0    //double (cm)
#define MAXIMUM_POSITION_MAX		55.0    //double (cm)
#define NOSC_MIN					2       //int
#define NOSC_MAX					500    //int
#define LCD_INTENSITY_MIN           0       //int
#define LCD_INTENSITY_MAX           15      //int
#define ORIGIN_POSITION_MIN         -3.0    //double
#define ORIGIN_POSITION_MAX         4.0     //double
#define SPHERE_DIAMETER_MIN         1.0     //double (cm)
#define SPHERE_DIAMETER_MAX         15.0    //double (cm)
#define PENDULUM_LENGTH_MIN         0.3     //double (m)
#define PENDULUM_LENGTH_MAX         20.0    //double (m)
#define PULLEY_DIAMETER_MIN         1.0     //double (cm)
#define PULLEY_DIAMETER_MAX         5.0     //double (cm)
#define VERTICAL_POSITION_MIN       15.0    //double (cm)
#define VERTICAL_POSITION_MAX       30.0    //double (cm)
#define PHOTODIODE_POSITION_MIN     15.0    //double (cm)
#define PHOTODIODE_POSITION_MAX     35.0    //double (cm)

#define INITIAL_PULLEY_DIAMETER_CM  3.0     //double (cm)
#define INITIAL_VERTICAL_POSITION   27.5    //double (cm)
#define INITIAL_PHOTODIODE_POSITION 30.5    //double (cm)
#define INITIAL_ORIGIN_POSITION     0.0     //double (cm)
#define INITIAL_SPHERE_DIAMETER     8.0     //double (cm)
#define INITIAL_PENDULUM_LENGTH     2.8     //double (m)
#define INITIAL_LCD_INTENSITY       2       //int between LCD_INTENSITY_MIN and LCD_INTENSITY_MAX
#define INITIAL_MAXIMUM_POSITION    45.0    //double (cm) between MAXIMUM_POSITION_MIN and MAXIMUM_POSITION_MAX

//Advanced parameters
#define INITIAL_CLOCK_FREQUENCY     3686400.0   //double (Hz), same as FCY defined above
#define CLOCK_FREQUENCY_MIN         3502080.0   //double (Hz), FCY - 5%
#define CLOCK_FREQUENCY_MAX         3870720.0   //double (Hz), FCY + 5%
#define INITIAL_C30_UART            2           
#define C30_UART_MIN                1           //int, printf output via FTDI
#define C30_UART_MAX                2           //int, printf output via DB25
#define INITIAL_SINE_CORRECTION     105          //int, value between 0 and DUTY_CYCLE_RES, used to correct sine wave (to compensate detent?), see mcpwm.c
#define SINE_CORRECTION_MIN         0           //int,
//#define SINE_CORRECTION_MAX       DUTY_CYCLE_RES in mcpwm.h
#define INITIAL_MAX_POWER           1.0         //double, 0.0 < MAX_POWER <= 1.0, power to spin at high speed, global power cap
#define MAX_POWER_MIN               0.0         //double, 
#define MAX_POWER_MAX               1.0         //double,
#define INITIAL_MAX_POWER_0         0.25        //double, 0.0 < MAX_POWER_0 < MAX_POWER_1, power to hold the motor
#define MAX_POWER_0_MIN             0.0         //double, 
#define MAX_POWER_0_MAX             1.0         //double,
#define INITIAL_MAX_POWER_1         0.5         //double, MAX_POWER_0 < MAX_POWER_1 < MAX_POWER_2, power to spin at low speed
#define MAX_POWER_1_MIN             0.0         //double, 
#define MAX_POWER_1_MAX             1.0         //double,
#define INITIAL_MAX_POWER_2         1.0         //double, MAX_POWER_1 < MAX_POWER_2 <= 1.0, power to spin at medium speed
#define MAX_POWER_2_MIN             0.0         //double, 
#define MAX_POWER_2_MAX             1.0         //double,
#define INITIAL_OMEGA_FOR_LT_2      5.0         //double, omega above which lookup table 2 is used
#define OMEGA_FOR_LT_2_MIN          0.0         //double, 
#define OMEGA_FOR_LT_2_MAX          1000.0      //double,
#define INITIAL_OMEGA_FOR_LT_3      30.0        //double, omega above which lookup table 3 is used
#define OMEGA_FOR_LT_3_MIN          0.0         //double, 
#define OMEGA_FOR_LT_3_MAX          1000.0      //double,

//AN2,RB0 corresponds to gpio 2 (100 ohm) & 4 (470 ohm) on Raspberry Pi
#define READ_TEST_CLOCK_OUT_PIN_ADPCFG  AD1PCFGLbits.PCFG2       //Analog (0) or digital (1) mode, in non-analog pins use: 1
#define SET_TEST_CLOCK_OUT_PIN_ADPCFG   AD1PCFGLbits.PCFG2 = 1   //Analog (0) or digital (1) mode, in non-analog pins use: asm("nop")
#define CLR_TEST_CLOCK_OUT_PIN_ADPCFG   AD1PCFGLbits.PCFG2 = 0   //Analog (0) or digital (1) mode, in non-analog pins use: asm("nop")
#define TEST_CLOCK_OUT_PIN_TRIS         TRISBbits.TRISB0         //Input (1) or Output (0) mode
#define READ_TEST_CLOCK_OUT_PIN         PORTBbits.RB0     
#define WRITE_TEST_CLOCK_OUT_PIN        LATBbits.LATB0
#define TOGGLE_TEST_CLOCK_OUT_PIN       asm("btg LATB, #0")

#define NUMBER_OF_ACQUISITIONS_FOR_TEMPERATURE_AVERAGING 32

#define ACCELERATION_WITHOUT_SLIDING    1.0     //double cm/(s^2)

#define DIRECTION_FORWARD			88
#define DIRECTION_BACKWARD			99

#define ERR_1						200
#define ERR_2						201

///////////////////////////////////////////////////
//Assignment of memory addresses to store variables
///////////////////////////////////////////////////
#define PAGE_ADDRESS                    0x0001  //same page for all variables
#define ORIGIN_POSITION_ADDRESS         0x4C00  //stores a double, initial address of the 3rd page before the end of the program memory
#define SPHERE_DIAMETER_ADDRESS         0x4C04  //stores a double
#define PENDULUM_LENGTH_ADDRESS         0x4C08  //stores a double
#define PULLEY_DIAMETER_ADDRESS         0x4C12  //stores a double
#define MAXIMUM_POSITION_ADDRESS        0x4C16  //stores a double
#define SINE_CORRECTION_ADDRESS         0x4C20  //stores a int
#define LCD_INTENSITY_ADDRESS           0x4C22  //stores a int
#define VERTICAL_POSITION_ADDRESS       0x4C24  //stores a double
#define PHOTODIODE_POSITION_ADDRESS     0x4C28  //stores a double
#define CLOCK_FREQUENCY_ADDRESS         0x4C32  //stores a double
#define C30_UART_ADDRESS                0x4C36  //stores a int
#define ID_STRING_ADDRESS               0x4C38  //stores 32 char, the last one is '\0'
#define ID_STRING_MAX_LENGTH            32      //ends at 0x4C78
#define MAX_POWER_TOTAL_ADDRESS         0X4C80  //stores a double
#define MAX_POWER_0_ADDRESS             0X4C84  //stores a double
#define MAX_POWER_1_ADDRESS             0X4C88  //stores a double
#define MAX_POWER_2_ADDRESS             0X4C92  //stores a double
#define OMEGA_FOR_LT_2_ADDRESS          0X4C96  //stores a double
#define OMEGA_FOR_LT_3_ADDRESS          0X4CA0  //stores a double

//The GlobalNumberOfOscs will use the last two pages of memory to prevent
//running out of W/E operations on the same cell. In this way the page will only
//be erased once full and writing will start at the beginning of the page again.
#define GLOBAL_NUMBER_OF_OSCS_ADDRESS   0x5000  //stores a unsigned int, initial address of the 2nd page before the end of the program memory

#endif

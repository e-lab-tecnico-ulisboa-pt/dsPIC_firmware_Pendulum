#include <p33FJ128MC804.h>
#include <math.h>
#include "flash.h"
#include "etc.h"
#include "world_pendulum.h"
#include "mcpwm.h"

static int SphereDiameter_FLAG;
static int PendulumLength_FLAG;
static int ExpectedPeriod_FLAG;
static int OriginPosition_FLAG;
static int VerticalPosition_FLAG;
static int PhotodiodePosition_FLAG;
static int PulleyDiameter_FLAG;
static int IdString_FLAG;
static int MaximumPosition_FLAG;
static int LCDIntensity_FLAG;
static int ClockFrequency_FLAG;
static int C30Uart_FLAG;
static int SineCorrection_FLAG;
static int MaxPowerTotal_FLAG;
static int MaxPower0_FLAG;
static int MaxPower1_FLAG;
static int MaxPower2_FLAG;
static int Omega2_FLAG;
static int Omega3_FLAG;

static double SphereDiameter_CM;
static double PendulumLength_M;
static double ExpectedPeriod_S;
static double OriginPosition_CM;
static double VerticalPosition_CM;
static double PhotodiodePosition_CM;
static double PulleyDiameter_cm;
static double MaximumPosition_CM;
static int    LCDIntensity;
static double ClockFrequency_Hz;
static int    C30Uart;
static int    SineCorrection_Tcy;
static double MaxPower_total;
static double MaxPower_0;
static double MaxPower_1;
static double MaxPower_2;
static double Omega_2;
static double Omega_3;

static char IdString[32];

//LCD intensity level
void saveLCDIntensity(int intensity) {
	if(intensity < LCD_INTENSITY_MIN) intensity = LCD_INTENSITY_MIN;
	if(intensity > LCD_INTENSITY_MAX) intensity = LCD_INTENSITY_MAX;
    flashSaveInt(PAGE_ADDRESS, LCD_INTENSITY_ADDRESS, intensity);
	LCDIntensity = intensity;
	LCDIntensity_FLAG = 1;
}
//LCD intensity level
int getLCDIntensity() {
	static int intensity;

	if(LCDIntensity_FLAG == 1) return LCDIntensity;
	
	intensity = flashGetInt(PAGE_ADDRESS, LCD_INTENSITY_ADDRESS);

	if(intensity < LCD_INTENSITY_MIN) { saveLCDIntensity(INITIAL_LCD_INTENSITY); intensity = INITIAL_LCD_INTENSITY; }
	else if(intensity > LCD_INTENSITY_MAX) { saveLCDIntensity(INITIAL_LCD_INTENSITY); intensity = INITIAL_LCD_INTENSITY; }

	LCDIntensity = intensity;
	LCDIntensity_FLAG = 1;
	return intensity;
}

//position of shovel when at origin
void saveOriginPosition_CM(double originPosition) {
	if(originPosition < ORIGIN_POSITION_MIN) originPosition = ORIGIN_POSITION_MIN;
	if(originPosition > ORIGIN_POSITION_MAX) originPosition = ORIGIN_POSITION_MAX;
	flashSaveDouble(PAGE_ADDRESS, ORIGIN_POSITION_ADDRESS, originPosition);
	OriginPosition_CM = originPosition;
	OriginPosition_FLAG = 1;
}
//position of shovel when at origin
double getOriginPosition_CM() {
	static double originPosition;

	if(OriginPosition_FLAG == 1) return OriginPosition_CM;
	
	originPosition = flashGetDouble(PAGE_ADDRESS, ORIGIN_POSITION_ADDRESS);

	if(originPosition < ORIGIN_POSITION_MIN) { saveOriginPosition_CM(INITIAL_ORIGIN_POSITION); originPosition = INITIAL_ORIGIN_POSITION; }
	else if(originPosition > ORIGIN_POSITION_MAX) { saveOriginPosition_CM(INITIAL_ORIGIN_POSITION); originPosition = INITIAL_ORIGIN_POSITION; }
	else if(isNaN((float*)&originPosition)) { saveOriginPosition_CM(INITIAL_ORIGIN_POSITION); originPosition = INITIAL_ORIGIN_POSITION; }

	OriginPosition_CM = originPosition;
	OriginPosition_FLAG = 1;
	return originPosition;
}

//diameter of sphere in cm
void saveSphereDiameter_CM(double diameter) {
	if(diameter < SPHERE_DIAMETER_MIN) diameter = SPHERE_DIAMETER_MIN;
	if(diameter > SPHERE_DIAMETER_MAX) diameter = SPHERE_DIAMETER_MAX;
	flashSaveDouble(PAGE_ADDRESS, SPHERE_DIAMETER_ADDRESS, diameter);
	SphereDiameter_CM = diameter;
	SphereDiameter_FLAG = 1;
}
//diameter of sphere in cm
double getSphereDiameter_CM() {
	static double diameter;

	if(SphereDiameter_FLAG == 1) return SphereDiameter_CM;
	
	diameter = flashGetDouble(PAGE_ADDRESS, SPHERE_DIAMETER_ADDRESS);

	if(diameter < SPHERE_DIAMETER_MIN) { saveSphereDiameter_CM(INITIAL_SPHERE_DIAMETER); diameter = INITIAL_SPHERE_DIAMETER; }
	else if(diameter > SPHERE_DIAMETER_MAX) { saveSphereDiameter_CM(INITIAL_SPHERE_DIAMETER); diameter = INITIAL_SPHERE_DIAMETER; }
	else if(isNaN((float*)&diameter)) { saveSphereDiameter_CM(INITIAL_SPHERE_DIAMETER); diameter = INITIAL_SPHERE_DIAMETER; }
	SphereDiameter_CM = diameter;
	SphereDiameter_FLAG = 1;
	return diameter;
}


//length of pendulum in meters
void savePendulumLength_M(double length) {
	if(length < PENDULUM_LENGTH_MIN) length = PENDULUM_LENGTH_MIN;
	if(length > PENDULUM_LENGTH_MAX) length = PENDULUM_LENGTH_MAX;
	flashSaveDouble(PAGE_ADDRESS, PENDULUM_LENGTH_ADDRESS, length);
	PendulumLength_M = length;
	PendulumLength_FLAG = 1;
	ExpectedPeriod_S = 2 * 3.14 * sqrt(PendulumLength_M / 9.8);
	ExpectedPeriod_FLAG = 1;
}
//length of pendulum in meters
double getPendulumLength_M() {
	static double length;

	if(PendulumLength_FLAG == 1) return PendulumLength_M;
	
	length = flashGetDouble(PAGE_ADDRESS, PENDULUM_LENGTH_ADDRESS);

	if(length < PENDULUM_LENGTH_MIN) { savePendulumLength_M(INITIAL_PENDULUM_LENGTH); length = INITIAL_PENDULUM_LENGTH; }
	else if(length > PENDULUM_LENGTH_MAX) { savePendulumLength_M(INITIAL_PENDULUM_LENGTH); length = INITIAL_PENDULUM_LENGTH; }
	else if(isNaN((float*)&length)) { savePendulumLength_M(INITIAL_PENDULUM_LENGTH); length = INITIAL_PENDULUM_LENGTH; }
	PendulumLength_M = length;
	PendulumLength_FLAG = 1;
	ExpectedPeriod_S = 2 * 3.14 * sqrt(PendulumLength_M / 9.8);
	ExpectedPeriod_FLAG = 1;
	return length;
}

//expected period in seconds
double getExpectedPeriod_S() {
	if(ExpectedPeriod_FLAG == 1) return ExpectedPeriod_S;
	ExpectedPeriod_S = 2 * 3.14 * sqrt(getPendulumLength_M() / 9.8);
	ExpectedPeriod_FLAG = 1;
	return ExpectedPeriod_S;
}


//pulley diameter in cm
void savePulleyDiameter_cm(double pulleyDiam) {
	if(pulleyDiam < PULLEY_DIAMETER_MIN) pulleyDiam = PULLEY_DIAMETER_MIN;
	if(pulleyDiam > PULLEY_DIAMETER_MAX) pulleyDiam = PULLEY_DIAMETER_MAX;
	flashSaveDouble(PAGE_ADDRESS, PULLEY_DIAMETER_ADDRESS, pulleyDiam);
	PulleyDiameter_cm = pulleyDiam;
	PulleyDiameter_FLAG = 1;
}
//pulley diameter in cm
double getPulleyDiameter_cm() {
	static double pulleyDiam;

	if(PulleyDiameter_FLAG == 1) return PulleyDiameter_cm;

	pulleyDiam = flashGetDouble(PAGE_ADDRESS, PULLEY_DIAMETER_ADDRESS);

	if(pulleyDiam < PULLEY_DIAMETER_MIN) { savePulleyDiameter_cm(INITIAL_PULLEY_DIAMETER_CM); pulleyDiam = INITIAL_PULLEY_DIAMETER_CM; }
	else if(pulleyDiam > PULLEY_DIAMETER_MAX) { savePulleyDiameter_cm(INITIAL_PULLEY_DIAMETER_CM); pulleyDiam = INITIAL_PULLEY_DIAMETER_CM; }
    else if(isNaN((float*)&pulleyDiam)) { savePulleyDiameter_cm(INITIAL_PULLEY_DIAMETER_CM); pulleyDiam = INITIAL_PULLEY_DIAMETER_CM; }
	PulleyDiameter_cm = pulleyDiam;
	PulleyDiameter_FLAG = 1;
	return pulleyDiam;
}

//save ID string
void saveIDstring_CHAR(char *str) {
    static int i;
    flashSaveString(PAGE_ADDRESS, ID_STRING_ADDRESS, str, ID_STRING_MAX_LENGTH);
    for(i=0; i<ID_STRING_MAX_LENGTH; i++) IdString[i] = str[i]; 
	IdString_FLAG = 1;
}
//get ID string
char* getIDstring_CHAR(char *str) {
    static int i;
    static int ok;
    if(IdString_FLAG == 1) {
        ok = 1;
        for(i=0; i<ID_STRING_MAX_LENGTH; i++) str[i] = IdString[i];
        str[ID_STRING_MAX_LENGTH-1] = 0; //redundant
        return str;
    }
    flashGetString(PAGE_ADDRESS, ID_STRING_ADDRESS, str, ID_STRING_MAX_LENGTH);
    for(i=0; i<ID_STRING_MAX_LENGTH; i++) IdString[i] = str[i];
    str[ID_STRING_MAX_LENGTH-1] = 0; //redundant
	IdString_FLAG = 1;
	return str;
}


//maximum position of shovel in cm
void saveMaximumPosition_CM(double maximumPosition) {
	if(maximumPosition < MAXIMUM_POSITION_MIN) maximumPosition = MAXIMUM_POSITION_MIN;
	if(maximumPosition > MAXIMUM_POSITION_MAX) maximumPosition = MAXIMUM_POSITION_MAX;
	flashSaveDouble(PAGE_ADDRESS, MAXIMUM_POSITION_ADDRESS, maximumPosition);
	MaximumPosition_CM = maximumPosition;
	MaximumPosition_FLAG = 1;
}
//maximum position of shovel in cm
double getMaximumPosition_CM() {
	static double maximumPosition;

	if(MaximumPosition_FLAG == 1) return MaximumPosition_CM;

	maximumPosition = flashGetDouble(PAGE_ADDRESS, MAXIMUM_POSITION_ADDRESS);

	if(maximumPosition < MAXIMUM_POSITION_MIN) { saveMaximumPosition_CM(INITIAL_MAXIMUM_POSITION); maximumPosition = INITIAL_MAXIMUM_POSITION; }
	else if(maximumPosition > MAXIMUM_POSITION_MAX) { saveMaximumPosition_CM(INITIAL_MAXIMUM_POSITION); maximumPosition = INITIAL_MAXIMUM_POSITION; }
    else if(isNaN((float*)&maximumPosition)) { saveMaximumPosition_CM(INITIAL_MAXIMUM_POSITION); maximumPosition = INITIAL_MAXIMUM_POSITION; }
	MaximumPosition_CM = maximumPosition;
	MaximumPosition_FLAG = 1;
	return maximumPosition;
}


//position of shovel when pendulum wire is perfectly vertical
void saveVerticalPosition_CM(double verticalPosition) {
	if(verticalPosition < VERTICAL_POSITION_MIN) verticalPosition = VERTICAL_POSITION_MIN;
	if(verticalPosition > VERTICAL_POSITION_MAX) verticalPosition = VERTICAL_POSITION_MAX;
	flashSaveDouble(PAGE_ADDRESS, VERTICAL_POSITION_ADDRESS, verticalPosition);
	VerticalPosition_CM = verticalPosition;
	VerticalPosition_FLAG = 1;
}
//position of shovel when pendulum wire is perfectly vertical
double getVerticalPosition_CM() {
	static double verticalPosition;

	if(VerticalPosition_FLAG == 1) return VerticalPosition_CM;
	
	verticalPosition = flashGetDouble(PAGE_ADDRESS, VERTICAL_POSITION_ADDRESS);

	if(verticalPosition < VERTICAL_POSITION_MIN) { saveVerticalPosition_CM(INITIAL_VERTICAL_POSITION); verticalPosition = INITIAL_VERTICAL_POSITION; }
	else if(verticalPosition > VERTICAL_POSITION_MAX) { saveVerticalPosition_CM(INITIAL_VERTICAL_POSITION); verticalPosition = INITIAL_VERTICAL_POSITION; }
	else if(isNaN((float*)&verticalPosition)) { saveVerticalPosition_CM(INITIAL_VERTICAL_POSITION); verticalPosition = INITIAL_VERTICAL_POSITION; }
	VerticalPosition_CM = verticalPosition;
	VerticalPosition_FLAG = 1;
	return verticalPosition;
}


//position of shovel when at photodiode
void savePhotodiodePosition_CM(double photodiodePosition) {
	if(photodiodePosition < PHOTODIODE_POSITION_MIN) photodiodePosition = PHOTODIODE_POSITION_MIN;
	if(photodiodePosition > PHOTODIODE_POSITION_MAX) photodiodePosition = PHOTODIODE_POSITION_MAX;
	flashSaveDouble(PAGE_ADDRESS, PHOTODIODE_POSITION_ADDRESS, photodiodePosition);
	PhotodiodePosition_CM = photodiodePosition;
	PhotodiodePosition_FLAG = 1;
}
//position of shovel when at photodiode
double getPhotodiodePosition_CM() {
	static double photodiodePosition;

	if(PhotodiodePosition_FLAG == 1) return PhotodiodePosition_CM;
	
	photodiodePosition = flashGetDouble(PAGE_ADDRESS, PHOTODIODE_POSITION_ADDRESS);

	if(photodiodePosition < PHOTODIODE_POSITION_MIN) { savePhotodiodePosition_CM(INITIAL_PHOTODIODE_POSITION); photodiodePosition = INITIAL_PHOTODIODE_POSITION; }
	else if(photodiodePosition > PHOTODIODE_POSITION_MAX) { savePhotodiodePosition_CM(INITIAL_PHOTODIODE_POSITION); photodiodePosition = INITIAL_PHOTODIODE_POSITION; }
	else if(isNaN((float*)&photodiodePosition)) { savePhotodiodePosition_CM(INITIAL_PHOTODIODE_POSITION); photodiodePosition = INITIAL_PHOTODIODE_POSITION; }
	PhotodiodePosition_CM = photodiodePosition;
	PhotodiodePosition_FLAG = 1;
	return photodiodePosition;
}


//clock frequency for pendulum period calculation
void saveClockFrequency_Hz(double clockFrequency) {
	if(clockFrequency < CLOCK_FREQUENCY_MIN) clockFrequency = CLOCK_FREQUENCY_MIN;
	if(clockFrequency > CLOCK_FREQUENCY_MAX) clockFrequency = CLOCK_FREQUENCY_MAX;
	flashSaveDouble(PAGE_ADDRESS, CLOCK_FREQUENCY_ADDRESS, clockFrequency);
	ClockFrequency_Hz = clockFrequency;
	ClockFrequency_FLAG = 1;
}
//clock frequency for pendulum period calculation
double getClockFrequency_Hz() {
	static double clockFrequency;

	if(ClockFrequency_FLAG == 1) return ClockFrequency_Hz;
	
	clockFrequency = flashGetDouble(PAGE_ADDRESS, CLOCK_FREQUENCY_ADDRESS);

	if(clockFrequency < CLOCK_FREQUENCY_MIN) { saveClockFrequency_Hz(INITIAL_CLOCK_FREQUENCY); clockFrequency = INITIAL_CLOCK_FREQUENCY; }
	else if(clockFrequency > CLOCK_FREQUENCY_MAX) { saveClockFrequency_Hz(INITIAL_CLOCK_FREQUENCY); clockFrequency = INITIAL_CLOCK_FREQUENCY; }
	else if(isNaN((float*)&clockFrequency)) { saveClockFrequency_Hz(INITIAL_CLOCK_FREQUENCY); clockFrequency = INITIAL_CLOCK_FREQUENCY; }
	ClockFrequency_Hz = clockFrequency;
	ClockFrequency_FLAG = 1;
	return clockFrequency;
}
double getClockFrequency_Hz_fast(){
    return ClockFrequency_Hz;
}


//printf output to C30 UART x
void saveC30Uart(int c30uart) {
	if(c30uart < C30_UART_MIN) c30uart = C30_UART_MIN;
	if(c30uart > C30_UART_MAX) c30uart = C30_UART_MAX;
    flashSaveInt(PAGE_ADDRESS, C30_UART_ADDRESS, c30uart);
	C30Uart = c30uart;
	C30Uart_FLAG = 1;
}
//printf output to C30 UART x
int getC30Uart() {
	static int c30uart;

	if(C30Uart_FLAG == 1) return C30Uart;
	
	c30uart = flashGetInt(PAGE_ADDRESS, C30_UART_ADDRESS);

	if(c30uart < C30_UART_MIN) { saveC30Uart(INITIAL_C30_UART); c30uart = INITIAL_C30_UART; }
	else if(c30uart > C30_UART_MAX) { saveC30Uart(INITIAL_C30_UART); c30uart = INITIAL_C30_UART; }

	C30Uart = c30uart;
	C30Uart_FLAG = 1;
	return c30uart;
}


//Sine correction
void saveSineCorrection(int sineCorrection) {
	if(sineCorrection < SINE_CORRECTION_MIN) sineCorrection = SINE_CORRECTION_MIN;
	if(sineCorrection > DUTY_CYCLE_RES) sineCorrection = DUTY_CYCLE_RES;
    flashSaveInt(PAGE_ADDRESS, SINE_CORRECTION_ADDRESS, sineCorrection);
	SineCorrection_Tcy = sineCorrection;
	SineCorrection_FLAG = 1;
}
//Sine correction
int getSineCorrection() {
	static int sineCorrection;

	if(SineCorrection_FLAG == 1) return SineCorrection_Tcy;
	
	sineCorrection = flashGetInt(PAGE_ADDRESS, SINE_CORRECTION_ADDRESS);

	if(sineCorrection < SINE_CORRECTION_MIN) { saveSineCorrection(INITIAL_SINE_CORRECTION); sineCorrection = INITIAL_SINE_CORRECTION; }
	else if(sineCorrection > DUTY_CYCLE_RES) { saveSineCorrection(INITIAL_SINE_CORRECTION); sineCorrection = INITIAL_SINE_CORRECTION; }

	SineCorrection_Tcy = sineCorrection;
	SineCorrection_FLAG = 1;
	return sineCorrection;
}


//Maximum power (cap for total power fed to the motor)
void saveMaxPower_total(double maxPowerTotal) {
	if(maxPowerTotal < MAX_POWER_MIN) maxPowerTotal = MAX_POWER_MIN;
	if(maxPowerTotal > MAX_POWER_MAX) maxPowerTotal = MAX_POWER_MAX;
	flashSaveDouble(PAGE_ADDRESS, MAX_POWER_TOTAL_ADDRESS, maxPowerTotal);
	MaxPower_total = maxPowerTotal;
	MaxPowerTotal_FLAG = 1;
}
//Maximum power (cap for total power fed to the motor)
double getMaxPower_total() {
	static double maxPowerTotal;

	if(MaxPowerTotal_FLAG == 1) return MaxPower_total;
	
	maxPowerTotal = flashGetDouble(PAGE_ADDRESS, MAX_POWER_TOTAL_ADDRESS);

	if(maxPowerTotal < MAX_POWER_MIN) { saveMaxPower_total(INITIAL_MAX_POWER); maxPowerTotal = INITIAL_MAX_POWER; }
	else if(maxPowerTotal > MAX_POWER_MAX) { saveMaxPower_total(INITIAL_MAX_POWER); maxPowerTotal = INITIAL_MAX_POWER; }
	else if(isNaN((float*)&maxPowerTotal)) { saveMaxPower_total(INITIAL_MAX_POWER); maxPowerTotal = INITIAL_MAX_POWER; }
	MaxPower_total = maxPowerTotal;
	MaxPowerTotal_FLAG = 1;
	return maxPowerTotal;
}


//Maximum power for lookup table 0 (hold the motor)
void saveMaxPower_0(double maxPower0) {
	if(maxPower0 < MAX_POWER_0_MIN) maxPower0 = MAX_POWER_0_MIN;
	if(maxPower0 > MAX_POWER_0_MAX) maxPower0 = MAX_POWER_0_MAX;
	flashSaveDouble(PAGE_ADDRESS, MAX_POWER_0_ADDRESS, maxPower0);
	MaxPower_0 = maxPower0;
	MaxPower0_FLAG = 1;
}
//Maximum power for lookup table 0 (hold the motor)
double getMaxPower_0() {
	static double maxPower0;

	if(MaxPower0_FLAG == 1) return MaxPower_0;
	
	maxPower0 = flashGetDouble(PAGE_ADDRESS, MAX_POWER_0_ADDRESS);

	if(maxPower0 < MAX_POWER_0_MIN) { saveMaxPower_0(INITIAL_MAX_POWER_0); maxPower0 = INITIAL_MAX_POWER_0; }
	else if(maxPower0 > MAX_POWER_0_MAX) { saveMaxPower_0(INITIAL_MAX_POWER_0); maxPower0 = INITIAL_MAX_POWER_0; }
	else if(isNaN((float*)&maxPower0)) { saveMaxPower_0(INITIAL_MAX_POWER_0); maxPower0 = INITIAL_MAX_POWER_0; }
	MaxPower_0 = maxPower0;
	MaxPower0_FLAG = 1;
	return maxPower0;
}


//Maximum power for lookup table 1 (spin motor at low speed)
void saveMaxPower_1(double maxPower1) {
	if(maxPower1 < MAX_POWER_1_MIN) maxPower1 = MAX_POWER_1_MIN;
	if(maxPower1 > MAX_POWER_1_MAX) maxPower1 = MAX_POWER_1_MAX;
	flashSaveDouble(PAGE_ADDRESS, MAX_POWER_1_ADDRESS, maxPower1);
	MaxPower_1 = maxPower1;
	MaxPower1_FLAG = 1;
}
//Maximum power for lookup table 1 (spin motor at low speed)
double getMaxPower_1() {
	static double maxPower1;

	if(MaxPower1_FLAG == 1) return MaxPower_1;
	
	maxPower1 = flashGetDouble(PAGE_ADDRESS, MAX_POWER_1_ADDRESS);

	if(maxPower1 < MAX_POWER_1_MIN) { saveMaxPower_1(INITIAL_MAX_POWER_1); maxPower1 = INITIAL_MAX_POWER_1; }
	else if(maxPower1 > MAX_POWER_1_MAX) { saveMaxPower_1(INITIAL_MAX_POWER_1); maxPower1 = INITIAL_MAX_POWER_1; }
	else if(isNaN((float*)&maxPower1)) { saveMaxPower_1(INITIAL_MAX_POWER_1); maxPower1 = INITIAL_MAX_POWER_1; }
	MaxPower_1 = maxPower1;
	MaxPower1_FLAG = 1;
	return maxPower1;
}


//Maximum power for lookup table 2 (spin motor at medium speed)
void saveMaxPower_2(double maxPower2) {
	if(maxPower2 < MAX_POWER_2_MIN) maxPower2 = MAX_POWER_2_MIN;
	if(maxPower2 > MAX_POWER_2_MAX) maxPower2 = MAX_POWER_2_MAX;
	flashSaveDouble(PAGE_ADDRESS, MAX_POWER_2_ADDRESS, maxPower2);
	MaxPower_2 = maxPower2;
	MaxPower2_FLAG = 1;
}
//Maximum power for lookup table 2 (spin motor at medium speed)
double getMaxPower_2() {
	static double maxPower2;

	if(MaxPower2_FLAG == 1) return MaxPower_2;
	
	maxPower2 = flashGetDouble(PAGE_ADDRESS, MAX_POWER_2_ADDRESS);

	if(maxPower2 < MAX_POWER_2_MIN) { saveMaxPower_2(INITIAL_MAX_POWER_2); maxPower2 = INITIAL_MAX_POWER_2; }
	else if(maxPower2 > MAX_POWER_2_MAX) { saveMaxPower_2(INITIAL_MAX_POWER_2); maxPower2 = INITIAL_MAX_POWER_2; }
	else if(isNaN((float*)&maxPower2)) { saveMaxPower_2(INITIAL_MAX_POWER_2); maxPower2 = INITIAL_MAX_POWER_2; }
	MaxPower_2 = maxPower2;
	MaxPower2_FLAG = 2;
	return maxPower2;
}


//Omega for lookup table 2 (omega above which lookup table no. 2 is used)
void saveOmega_2(double omega2) {
	if(omega2 < OMEGA_FOR_LT_2_MIN) omega2 = OMEGA_FOR_LT_2_MIN;
	if(omega2 > OMEGA_FOR_LT_2_MAX) omega2 = OMEGA_FOR_LT_2_MAX;
	flashSaveDouble(PAGE_ADDRESS, OMEGA_FOR_LT_2_ADDRESS, omega2);
	Omega_2 = omega2;
	Omega2_FLAG = 1;
}
//Omega for lookup table 2 (omega above which lookup table no. 2 is used)
double getOmega_2() {
	static double omega2;

	if(Omega2_FLAG == 1) return Omega_2;
	
	omega2 = flashGetDouble(PAGE_ADDRESS, OMEGA_FOR_LT_2_ADDRESS);

	if(omega2 < OMEGA_FOR_LT_2_MIN) { saveOmega_2(INITIAL_OMEGA_FOR_LT_2); omega2 = INITIAL_OMEGA_FOR_LT_2; }
	else if(omega2 > OMEGA_FOR_LT_2_MAX) { saveOmega_2(INITIAL_OMEGA_FOR_LT_2); omega2 = INITIAL_OMEGA_FOR_LT_2; }
	else if(isNaN((float*)&omega2)) { saveOmega_2(INITIAL_OMEGA_FOR_LT_2); omega2 = INITIAL_OMEGA_FOR_LT_2; }
	Omega_2 = omega2;
	Omega2_FLAG = 2;
	return omega2;
}


//Omega for lookup table 3 (omega above which lookup table no. 3 is used)
void saveOmega_3(double omega3) {
	if(omega3 < OMEGA_FOR_LT_3_MIN) omega3 = OMEGA_FOR_LT_3_MIN;
	if(omega3 > OMEGA_FOR_LT_3_MAX) omega3 = OMEGA_FOR_LT_3_MAX;
	flashSaveDouble(PAGE_ADDRESS, OMEGA_FOR_LT_3_ADDRESS, omega3);
	Omega_3 = omega3;
	Omega3_FLAG = 1;
}
//Omega for lookup table 3 (omega above which lookup table no. 3 is used)
double getOmega_3() {
	static double omega3;

	if(Omega3_FLAG == 1) return Omega_3;
	
	omega3 = flashGetDouble(PAGE_ADDRESS, OMEGA_FOR_LT_3_ADDRESS);

	if(omega3 < OMEGA_FOR_LT_3_MIN) { saveOmega_3(INITIAL_OMEGA_FOR_LT_3); omega3 = INITIAL_OMEGA_FOR_LT_3; }
	else if(omega3 > OMEGA_FOR_LT_3_MAX) { saveOmega_3(INITIAL_OMEGA_FOR_LT_3); omega3 = INITIAL_OMEGA_FOR_LT_3; }
	else if(isNaN((float*)&omega3)) { saveOmega_3(INITIAL_OMEGA_FOR_LT_3); omega3 = INITIAL_OMEGA_FOR_LT_3; }
	Omega_3 = omega3;
	Omega3_FLAG = 2;
	return omega3;
}


double getInitialOffset_CM() {
    return (getPhotodiodePosition_CM() - getVerticalPosition_CM());
}
double getDistanceLaserToStart_CM() {
    return (getPhotodiodePosition_CM() - getOriginPosition_CM());
}
int getDeltaXMin_CM() {
    //+0.9999999 to make sure x min >= initial offset
    return (int) (getInitialOffset_CM() + 0.999999);
}
int getDeltaXMax_CM() {
    //-0.5 to allow some uncertainty on the vertical position measurement
    double dXMax = getMaximumPosition_CM() - getVerticalPosition_CM() - 0.5;
    //Prevent the ball hitting the shovel at the origin when requesting the maximum deltaX
    if( dXMax >= (getVerticalPosition_CM() - getOriginPosition_CM() - 0.9) ) //0.9 cm for margin
        dXMax = getVerticalPosition_CM() - getOriginPosition_CM() - 0.9;
    return (int) (dXMax);
}

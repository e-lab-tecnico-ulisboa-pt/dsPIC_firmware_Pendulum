#ifndef _PHYSICAL_H_
#define _PHYSICAL_H_

//void saveCatchDelay_MS(int delay);
//int getCatchDelay_MS();

void saveOriginPosition_CM(double originPosition);
double getOriginPosition_CM();

void saveSphereDiameter_CM(double);
double getSphereDiameter_CM();

void savePendulumLength_M(double);
double getPendulumLength_M();

double getExpectedPeriod_S();

void savePulleyDiameter_cm(double pulleyDiam);
double getPulleyDiameter_cm();

void saveIDstring_CHAR(char *str);
char* getIDstring_CHAR(char *str);

void saveMaximumPosition_CM(double maximumPosition);
double getMaximumPosition_CM();

void saveVerticalPosition_CM(double verticalPosition);
double getVerticalPosition_CM();

void saveLCDIntensity(int intensity);
int getLCDIntensity();

void savePhotodiodePosition_CM(double photodiodePosition);
double getPhotodiodePosition_CM();

void saveClockFrequency_Hz(double clockFrequency);
double getClockFrequency_Hz();
double getClockFrequency_Hz_fast();

void saveC30Uart(int c30uart);
int getC30Uart();

void saveSineCorrection(int sineCorrection);
int getSineCorrection();

void saveMaxPower_total(double maxPowerTotal);
double getMaxPower_total();

void saveMaxPower_0(double maxPower0);
double getMaxPower_0();

void saveMaxPower_1(double maxPower1);
double getMaxPower_1();

void saveMaxPower_2(double maxPower2);
double getMaxPower_2();

void saveOmega_2(double omega2);
double getOmega_2();

void saveOmega_3(double omega3);
double getOmega_3();

double getDistanceLaserToStart_CM();
double getInitialOffset_CM();
int getDeltaXMin_CM();
int getDeltaXMax_CM();

#endif

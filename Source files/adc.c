#include <p33FJ128MC804.h>		//defines dspic register locations and structures definitions
#include "world_pendulum.h"
#include <stdio.h>

void __attribute__((__interrupt__, __no_auto_psv__)) _ADC1Interrupt (void);

//temperature in Celsius
unsigned int temperatureAcquisitionReadings[NUMBER_OF_ACQUISITIONS_FOR_TEMPERATURE_AVERAGING];

//Initializes all the necessary stuff for acquiring with the adc
//Uses AN1 for temperature measurement from LM35
void adc_init () {
    AD1CON1bits.ADSIDL = 1;		//Stop in Idle Mode bit
								//1 = Discontinue module operation when device enters Idle mode
								//0 = Continue module operation in Idle mode
							
    AD1CON1bits.ADDMABM = 0;     //DMA Buffer Build Mode bit
                                //1 = DMA buffers are written in the order of conversion. The module provides an address to the DMA
                                //channel that is the same as the address used for the non-DMA stand-alone buffer
                                //0 = DMA buffers are written in Scatter/Gather mode. The module provides a scatter/gather address
                                //to the DMA channel, based on the index of the analog input and the size of the DMA buffer

	AD1CON1bits.AD12B = 1;      //10-bit or 12-bit Operation Mode bit
                                //1 =12-bit, 1-channel ADC operation
                                //0 = 10-bit, 4-channel ADC operation

    AD1CON1bits.FORM = 0b00;    //FORM<1:0>: Data Output Format bits
                                //11 = Signed fractional (DOUT = sddd dddd dddd 0000)
                                //10 = Fractional (DOUT = dddd dddd dddd 0000)
                                //01 = Signed integer (DOUT = ssss sddd dddd dddd)
                                //00 = Integer (DOUT = 0000 dddd dddd dddd)

    AD1CON1bits.SSRC = 0b111;	//SSRC<2:0>: Conversion Trigger Source Select bits
                                //111 = Internal counter ends sampling and starts conversion (auto convert)
                                //110 = Reserved
                                //101 = Reserved
                                //100 = Reserved
                                //011 = Motor Control PWM interval ends sampling and starts conversion
                                //010 = General purpose Timer3 compare ends sampling and starts conversion
                                //001 = Active transition on INT0 pin ends sampling and starts conversion
                                //000 = Clearing SAMP bit ends sampling and starts conversion
                                
    AD1CON1bits.SIMSAM = 0;     //Simultaneous Sample Select bit (only applicable when CHPS<1:0> = 01 or 1x)
                                //When AD12B = 1, SIMSAM is: U-0, Unimplemented, Read as ?0?
                                //1 = Samples CH0, CH1, CH2, CH3 simultaneously (when CHPS<1:0> = 1x); or
                                //Samples CH0 and CH1 simultaneously (when CHPS<1:0> = 01)
                                //0 = Samples multiple channels individually in sequence
								
    AD1CON1bits.ASAM = 1;	//A/D Sample Auto-Start bit
    						//1 = Sampling begins immediately after last conversion completes. SAMP bit is auto set.
							//0 = Sampling begins when SAMP bit set

    AD1CON1bits.SAMP = 1;	//A/D Sample Enable bit
							//1 = At least one A/D sample/hold amplifier is sampling
							//0 = A/D sample/hold amplifiers are holding
							//When ASAM = 0, writing '1' to this bit will start sampling.
							//When SSRC = 000, writing '0' to this bit will end sampling and start conversion.

    AD1CON2bits.VCFG = 0b100;	//Voltage Reference Configuration bits
                                //A/D VREFH						A/D VREFL
                                //000 AVDD						AVSS
                                //001 External VREF+ pin		AVSS
                                //010 AVDD						External VREF- pin
                                //011 External VREF+ pin		External VREF- pin
                                //1xx AVDD						AVSS

    AD1CON2bits.CSCNA = 0;  //Scan Input Selections for CH0+ during Sample A bit
                            //1 = Scan inputs
                            //0 = Do not scan inputs
                            
    AD1CON2bits.CHPS = 0b00;    //<1:0>: Selects Channels Utilized bits
                                //When AD12B = 1, CHPS<1:0> is: U-0, Unimplemented, Read as ?0?
                                //1x = Converts CH0, CH1, CH2 and CH3
                                //01 = Converts CH0 and CH1
                                //00 = Converts CH0

    AD1CON2bits.BUFS = 0;   //Buffer Fill Status bit (only valid when BUFM = 1)
                            //1 = ADC is currently filling buffer 0x8-0xF, user should access data in 0x0-0x7
                            //0 = ADC is currently filling buffer 0x0-0x7, user should access data in 0x8-0xF
    
    AD1CON2bits.SMPI = 0b1111;	//SMPI<3:0>: Selects Increment Rate for DMA Addresses bits or number of sample/conversion
                                //operations per interrupt
                                //1111 = Increments the DMA address or generates interrupt after completion of every 16th sample/
                                //conversion operation
                                //1110 = Increments the DMA address or generates interrupt after completion of every 15th sample/
                                //conversion operation
                                //...
                                //0001 = Increments the DMA address after completion of every 2nd sample/conversion operati

    AD1CON2bits.BUFM = 0;   //BUFM: Buffer Fill Mode Select bit
                            //1 = Starts buffer filling at address 0x0 on first interrupt and 0x8 on next interrupt
                            //0 = Always starts filling buffer at address 0x0

    AD1CON2bits.ALTS = 0;   //ALTS: Alternate Input Sample Mode Select bit
                            //1 = Uses channel input selects for Sample A on first sample and Sample B on next sample
                            //0 = Always uses channel input selects for Sample A

    AD1CON3bits.ADRC = 0;   //ADRC: ADC Conversion Clock Source bit
                            //1 = ADC internal RC clock
                            //0 = Clock derived from system clock
    
    AD1CON3bits.SAMC = 0b11111;	//SAMC<4:0>: Auto Sample Time bits
                                //11111 = 31 TAD
                                //·····
                                //00001 = 1 TAD
                                //00000 = 0 TAD

    AD1CON3bits.ADCS = 0b00111111;  //ADCS<7:0>: ADC Conversion Clock Select bits(2)
                                    //11111111 = Reserved
                                    //...
                                    //01000000 = Reserved
                                    //00111111 = TCY · (ADCS<7:0> + 1) = 64 · TCY = TAD
                                    //...
                                    //00000010 = TCY · (ADCS<7:0> + 1) = 3 · TCY = TAD
                                    //00000001 = TCY · (ADCS<7:0> + 1) = 2 · TCY = TAD

    AD1CON4bits.DMABL = 0b000;      //DMABL<2:0>: Selects Number of DMA Buffer Locations per Analog Input bits
                                    //111 = Allocates 128 words of buffer to each analog input
                                    //110 = Allocates 64 words of buffer to each analog input
                                    //101 = Allocates 32 words of buffer to each analog input
                                    //100 = Allocates 16 words of buffer to each analog input
                                    //011 = Allocates 8 words of buffer to each analog input
                                    //010 = Allocates 4 words of buffer to each analog input
                                    //001 = Allocates 2 words of buffer to each analog input
                                    //000 = Allocates 1 word of buffer to each analog input
    
    //AD1CHS123: ADC1 INPUT CHANNEL 1, 2, 3 SELECT REGISTER
    //No need to fill because AD12B = 1
    
    AD1CHS0bits.CH0NB = 0;      //CH0NB: Channel 0 Negative Input Select for Sample B bit
                                //1 = Channel 0 negative input is AN1
                                //0 = Channel 0 negative input is VREF-
    
    AD1CHS0bits.CH0SB = 0b00000;    //CH0SB<4:0>: Channel 0 Positive Input Select for Sample B bits
                                    //dsPIC33FJ32MC304, dsPIC33FJ64MC204/804 and dsPIC33FJ128MC204/804 devices only:
                                    //01000 = Channel 0 positive input is AN8
                                    //...
                                    //00010 = Channel 0 positive input is AN2
                                    //00001 = Channel 0 positive input is AN1
                                    //00000 = Channel 0 positive input is AN0
    
    AD1CHS0bits.CH0NA = 0;  //CH0NA: Channel 0 Negative Input Select for Sample A bit
                            //1 = Channel 0 negative input is AN1
                            //0 = Channel 0 negative input is VREF-
    
    AD1CHS0bits.CH0SA = 0b00001;    //CH0SA<4:0>: Channel 0 Positive Input Select for Sample A bits
                                    //dsPIC33FJ32MC304, dsPIC33FJ64MC204/804 and dsPIC33FJ128MC204/804 devices only:
                                    //01000 = Channel 0 positive input is AN8
                                    //...
                                    //00010 = Channel 0 positive input is AN2
                                    //00001 = Channel 0 positive input is AN1
                                    //00000 = Channel 0 positive input is AN0
    
    AD1CSSL = 0b0000000000000000;	//CSSL<15:0>: A/D Input Pin Scan Selection bits
									//1 = Select ANx for input scan
									//0 = Skip ANx for input scan
    
    AD1PCFGL &= 0b1111111111111101; //PCFG<15:0>: Analog Input Pin Configuration Control bits
                                    //1 = Analog input pin in Digital mode, port read input enabled, A/D input multiplexer input connected to AVSS
                                    //0 = Analog input pin in Analog mode, port read input disabled, A/D samples pin voltage
    
    TRISA |= 0b0000000000000010;	//Set analog pins to inputs

    AD1CON1bits.ADON = 1;	//enable ad converter
    IFS0bits.AD1IF = 0;		//clear adc interrupt flag
    IEC0bits.AD1IE = 1;		//enable adc end of sampling/conversion interrupts
    
    //AD1CON1bits.DONE		//DONE is '0' if adc is converting
    						//DONE is '1' if adc ended the conversion
    						
    //AD1CON1bits.BUFS 		//Buffer Fill Status bit
							//Only valid when BUFM = 1 (ADRES split into 2 x 8-word buffers).
							//1 = A/D is currently filling buffer 0x8-0xF, user should access data in 0x0-0x7
							//0 = A/D is currently filling buffer 0x0-0x7, user should access data in 0x8-0xF
}


//ISR for adc. This is called when the adc finishes a conversion
void  __attribute__((__interrupt__, __no_auto_psv__)) _ADC1Interrupt (void) {
    //This interrupt function takes about 4 us to execute with Fcy=3686400 Hz
	static unsigned int i;
    IFS0bits.AD1IF = 0;
    temperatureAcquisitionReadings[i++] = ADC1BUF0;
    if(i >= NUMBER_OF_ACQUISITIONS_FOR_TEMPERATURE_AVERAGING) i = 0;
}

double get_temperature() {
    //This function takes between 300-350 us to execute with Fcy=3686400 Hz
	unsigned long int meanVal;
    unsigned int j;
    double retVal;
	/*adc calibration (includes a 220 Ohm added input resistance: between LM35 and ANx)
		LM35 conversion is 1 degC / 10 mV
		40.5 bits -> 5 degC
		104 bits -> 10 degC
		167 bits -> 15 degC
		232 bits -> 20 degC
		294.5 bits -> 25 degC
		358 bits -> 30 degC
		421 bits -> 35 degC
		485 bits -> 40 degC
		547 bits -> 45 degC
		611 bits -> 50 degC
		682 bits -> 55 degC
		745 bits -> 60 degC
		807.5 bits -> 65 degC
		871.5 bits -> 70 degC
		Resulting linear conversion factor is 0.078155 degC / bit
	*/  
    IEC0bits.AD1IE = 0;
	asm("nop");
    meanVal = 0;
    for(j=0; j<NUMBER_OF_ACQUISITIONS_FOR_TEMPERATURE_AVERAGING; j++)
        meanVal += temperatureAcquisitionReadings[j];
	retVal = (double)(meanVal) * 0.078155 / (double)(NUMBER_OF_ACQUISITIONS_FOR_TEMPERATURE_AVERAGING);
    IEC0bits.AD1IE = 1;
	return retVal;
}
#include "d7seg_max7219.h"
#include <p33FJ128MC804.h>		//defines dspic registers
#include "delays.h"

void init_d7seg_max7219(unsigned int intensity) {
    //Define pins as output
    D7SEG_DIN_TRIS = 0;
    D7SEG_CLK_TRIS = 0;
    D7SEG_CS_TRIS = 0;
    asm("nop");
    //CS should start at high because it is active low
    D7SEG_CS = 1;
    //CKL should start at low because bits are transfered at rising edge
    D7SEG_CLK = 0;
    //Set shutdown mode to normal operation
    set_shutdown_mode_register(1);
    //Set decode mode to use all digits
    set_decode_mode_register(0xFF);
    //Clear the whole display
    clear_display();
    //Set display intensity: 0-15
    set_display_intensity(intensity);
    //Set how many digits are displayed: 0-7
    set_scan_limit_mode_register(7);
    //Test function back to normal
    write_word_to_max7219(0xFFFE);;
}

//Send 2 bytes word to max7219
void write_word_to_max7219(unsigned int word) {
    unsigned int n;  //mask
    D7SEG_CLK = 0;
    asm("nop");
    D7SEG_CS = 0;
    for(n=0b1000000000000000; n; n>>=1){
        asm("nop");
        D7SEG_DIN = ((word & n) ? 1 : 0);
        asm("nop");
        D7SEG_CLK = 1;
        asm("nop");
        D7SEG_CLK = 0;        
    }
    asm("nop");
    D7SEG_CS = 1;
}

//Set or remove the display from shutdown mode
void set_shutdown_mode_register(unsigned int sm) {
    //sm = 0 -> shutdown
    //sm = 1 -> normal operation
    write_word_to_max7219((0b00001100 << 8) + sm);
}

//Allow to enable or disable the usage of the data bits
//Decode mode register should be set after every initial power-up
void set_decode_mode_register(unsigned int dm) {
    //dm = 0xFF -> decode all bits
    write_word_to_max7219((0b00001001 << 8) + dm);
}

//Set scan limit mode to set how many digits are displayed
void set_scan_limit_mode_register(unsigned int slm) {
    //slm = 0 -> digit 0 only
    //slm = 1 -> digit 0 & 1
    //slm = 2 -> digit 0, 1 & 2
    //...
    //slm = 7 -> all digits: 0, 1, 2, 3, 4, 5, 6 & 7
    write_word_to_max7219((0b00001011 << 8) + slm);
}

//Light all the segments of display
void test_display() {
    write_word_to_max7219(0xFFFF);
    delay_ms(1000);
    write_word_to_max7219(0xFFFE);
}

//Set display intensity, levels are: 0 to 15 integer
void set_display_intensity(unsigned int intensity) {
    write_word_to_max7219((0b00001010 << 8) + intensity);
}

//Write digit to one 7 segment display
void write_digit_to_7seg_display(unsigned int digit, unsigned int position) {
    //position is defined between 1 to 8
    write_word_to_max7219((position << 8) + digit);
}

//Clear the whole display by setting blank at every digit
void clear_display() {
    int n;
    for(n=1; n<9; n++) write_digit_to_7seg_display(0xF, n);
}

//Write an unsigned long int to the display, max = 99999999
void write_unsigned_long_int_to_display(unsigned long int number) {
    if(number > 99999999) number = number - 100000000;
    int n;
    unsigned long int aux = 10;
    unsigned long int digit;
    for(n=1; n<=8; n++) {
        if(number > (aux/10-1)) {
            digit = number % aux;
            digit = digit/(aux/10);
            write_digit_to_7seg_display((unsigned int) digit, n);
        }
        else if( (number == 0) && (n == 1)) write_digit_to_7seg_display(0, n);
        else write_digit_to_7seg_display(0xF, n);
        aux = aux * 10;
    }
}
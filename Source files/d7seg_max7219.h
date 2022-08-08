#ifndef D7SEG_MAX7219_H
#define	D7SEG_MAX7219_H

#define D7SEG_DIN LATBbits.LATB4
#define D7SEG_CLK LATAbits.LATA4
#define D7SEG_CS LATAbits.LATA9

#define D7SEG_DIN_TRIS TRISBbits.TRISB4
#define D7SEG_CLK_TRIS TRISAbits.TRISA4
#define D7SEG_CS_TRIS TRISAbits.TRISA9

void init_d7seg_max7219(unsigned int intensity);
void write_word_to_max7219(unsigned int word);
void set_shutdown_mode_register(unsigned int sm);
void set_decode_mode_register(unsigned int dm);
void set_scan_limit_mode_register(unsigned int slm);
void test_display();
void set_display_intensity(unsigned int intensity);
void write_digit_to_7seg_display(unsigned int digit, unsigned int position);
void clear_display();
void write_unsigned_long_int_to_display(unsigned long int number);

#endif	/* D7SEG_MAX7219_H */


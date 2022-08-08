#ifndef TIMERS_H
#define	TIMERS_H

void timer_1_init();
void timer_2_init();
void timer_4_init();
void timer_5_init();
void start_test_clock_in(unsigned int prescaler);
void stop_test_clock_in();
void __attribute__((__interrupt__, __no_auto_psv__)) _IC2Interrupt(void);
unsigned int long get_test_clock_in_period();
void start_test_clock_out(unsigned int prescaler, unsigned int pr2, unsigned int pr3);
void stop_test_clock_out();

#endif


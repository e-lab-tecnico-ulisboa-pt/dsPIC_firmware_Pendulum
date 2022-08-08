#ifndef _MSG_H_
#define _MSG_H_

#define NORMAL_MODE 200
#define PANIC_MODE 201

void uart_1_init();
void uart_2_init();
int check_message_receival(char *);
void processMessage(int mode);
void sendHelp();
void sendAdvancedHelp();

#endif

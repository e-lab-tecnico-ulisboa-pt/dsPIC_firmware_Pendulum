#ifndef FLASH_H
#define	FLASH_H

#define PAGE_SIZE   512     //number of instructions per page (0x0200)
#define ROW_SIZE    64      //number of instructions per row

//Functions to be used externally to flash.c
extern int flashGetInt(int page, int offset);
extern long flashGetLong(int page, int offset);
extern double flashGetDouble(int page, int offset);
void flashGetString(int page, int offset, char* str, int max_str_length);

extern void eraseFlashPage(int page, int offset);

extern void flashSaveInt(int page, int offset, int value);
extern void flashSaveLong(int page, int offset, long value);
extern void flashSaveDouble(int page, int offset, double value);
extern void flashSaveString(int page, int offset, char *str, int max_str_length);

void flashWriteLong(int page, int offset, long value);

//Functions to be used internally by flash.c
int getPageStartAddress(int offset);
void loadPageToBuffer(int page, int offset, int *buf);
void writeIntToBuffer(int offset, int *buf, int value);
void writeLongToBuffer(int offset, int *buf, long value);
void writeDoubleToBuffer(int offset, int *buf, double value);
void writeStringToBuffer(int offset, int *buf, char *str, int str_length);
void writeBufferToFlashPage(int page, int offset, int *buf);

//Functions for debug
extern void printPage(int page, int offset);

#endif	/* FLASH_H */


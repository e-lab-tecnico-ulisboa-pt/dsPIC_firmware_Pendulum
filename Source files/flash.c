#include <p33FJ128MC804.h>
#include "flash.h"
#include "memory.h"
#include <stdio.h>

//Buffer to save the lower 16 bits of the instrunction before erasing the page
//and writing a new value
static int buffer[PAGE_SIZE];

///////////////////////////////////////////////
// FUNCTIONS TO BE USED EXTERNALY TO FLASH.C //
///////////////////////////////////////////////

//Read an int from the flash memory
int flashGetInt(int page, int offset) {
	static int myInt;
	myInt = ReadFlashWord(page, offset);
	return myInt;
}

//Read a long from the flash memory
long flashGetLong(int page, int offset) {
    static int i;
	static long l;
	i = ReadFlashWord(page, offset);
	*((int*)(&l) + 0) = i;
	i = ReadFlashWord(page, offset + 2);
	*((int*)(&l) + 1) = i;
	return l;
}

//Read a double from the flash memory
double flashGetDouble(int page, int offset) {
	static double myDouble;
	static unsigned int ui;	
	ui = ReadFlashWord(page, offset);
	*((unsigned int*)(&myDouble) + 0) = ui;
	ui = ReadFlashWord(page, offset + 2);
	*((unsigned int*)(&myDouble) + 1) = ui;
	return myDouble;
}

//Read a string from the flash memory into an input char pointer (str))
void flashGetString(int page, int offset, char* str, int max_str_length) {
	static int n;
    static unsigned int ui;
    for(n=0; n<max_str_length; n++){
        ui = ReadFlashWord(page, offset);
        str[n] = (char) ui;
        offset = offset + 2;
    }
    str[max_str_length-1] = 0;
    return;
}

//Erase a full page (512 instructions)
void eraseFlashPage(int page, int offset) {
	EraseFlashPage(page, offset);
    return;
}

//Write an int value to flash memory
void flashSaveInt(int page, int offset, int value) {
    loadPageToBuffer(page, getPageStartAddress(offset), buffer);
    writeIntToBuffer(offset, buffer, value);
    EraseFlashPage(page, getPageStartAddress(offset));
    writeBufferToFlashPage(page, getPageStartAddress(offset), buffer);
    return;
}

//Write a long value to flash memory without erasing the page (used of global number of oscillations)
void flashWriteLong(int page, int offset, long value) {
	static int i;
	i = *((int*)(&value) + 0);
	WriteFlashWord(page, offset, i);
	i = *((int*)(&value) + 1);
	WriteFlashWord(page, offset + 2, i);
    return;
}

//Save a long value to flash memory
void flashSaveLong(int page, int offset, long value) {
    loadPageToBuffer(page, getPageStartAddress(offset), buffer);
    writeLongToBuffer(offset, buffer, value);
    EraseFlashPage(page, getPageStartAddress(offset));
    writeBufferToFlashPage(page, getPageStartAddress(offset), buffer);
    return;
}

//Write a double value to flash memory
void flashSaveDouble(int page, int offset, double value) {
    loadPageToBuffer(page, getPageStartAddress(offset), buffer);
    writeDoubleToBuffer(offset, buffer, value);
    EraseFlashPage(page, getPageStartAddress(offset));
    writeBufferToFlashPage(page, getPageStartAddress(offset), buffer);
    return;
}

//Write a string to flash memory
void flashSaveString(int page, int offset, char *str, int max_str_length) {
    loadPageToBuffer(page, getPageStartAddress(offset), buffer);
    writeStringToBuffer(offset, buffer, str, max_str_length);
    EraseFlashPage(page, getPageStartAddress(offset));
    writeBufferToFlashPage(page, getPageStartAddress(offset), buffer);
    return;
}

////////////////////////////////////////////////
// FUNCTIONS TO BE USED INTERNALLY BY FLASH.C //
////////////////////////////////////////////////

//Load the least significant 16 bits of the instructions of a full page to a 
//buffer
void loadPageToBuffer(int page, int offset, int *buf) {
    static int n;
    for(n=0; n<PAGE_SIZE; n++) {
        buf[n] = ReadFlashWord(page, offset);
        offset = offset + 2;    //+2 because of memory indexing (one instruction is 24 bits -> 2 memory location: lower (16 lsb) and upper(8 msb))
    }
    return;
}

//Returns the address of the beginning of a page
int getPageStartAddress(int offset) {
    return (int) (offset / (2*PAGE_SIZE)) * (2*PAGE_SIZE);  //*2 because of memory indexing
}

//Write the buffer (least significant 16 bits of each instruction only) to memory flash page
//The writing process is performed using row programming operation
void writeBufferToFlashPage(int page, int offset, int *buf) {
    static int row;
    for(row=0; row<PAGE_SIZE; row=row+ROW_SIZE) {
        WriteSingleRow(page, offset, &buf[row]);
        offset = offset + 2*ROW_SIZE;   //*2 because of memory indexing
    }
    return;
}

//Writes an integer value to the buffer at the corresponding memory location given by offset
void writeIntToBuffer(int offset, int *buf, int value) {
    static int n;
    static int offset_aux;
    offset_aux = getPageStartAddress(offset);   //offset_aux starts at the beginning of the page
    //This for cycle contains a break statement
    for(n=0; n<PAGE_SIZE; n++) {
        if(offset_aux == offset) {
            buf[n] = value; //write the new integer value to buf
            break;          //exits the cycle once the value has been written
        }
        offset_aux = offset_aux + 2;    //+2 because of memory indexing
    }
    return;
}

//Writes a long value to the buffer at the corresponding memory location given by offset
void writeLongToBuffer(int offset, int *buf, long value) {
    static int n;
    static int offset_aux;
    offset_aux = getPageStartAddress(offset);   //offset_aux starts at the beginning of the page
    //This for cycle contains a break statement
    for(n=0; n<PAGE_SIZE; n++) {
        if(offset_aux == offset) {
            buf[n]   = *((int*)(&value) + 0); //write the 16 lsb of value to buf
            buf[n+1] = *((int*)(&value) + 1); //write the 16 msb of value to buf
            break;          //exits the cycle once the value has been written
        }
        offset_aux = offset_aux + 2;    //+2 because of memory indexing
    }
    return;
}

//Writes a double value to the buffer at the corresponding memory location given by offset
void writeDoubleToBuffer(int offset, int *buf, double value) {
    static int n;
    static int offset_aux;
    offset_aux = getPageStartAddress(offset);   //offset_aux starts at the beginning of the page
    //This for cycle contains a break statement
    for(n=0; n<PAGE_SIZE; n++) {
        if(offset_aux == offset) {
            buf[n]   = *((unsigned int*)(&value) + 0); //write the 16 lsb of value to buf
            buf[n+1] = *((unsigned int*)(&value) + 1); //write the 16 msb of value to buf
            break;          //exits the cycle once the value has been written
        }
        offset_aux = offset_aux + 2;    //+2 because of memory indexing
    }
    return;
}

//Writes a string to the buffer at the corresponding memory location given by offset
void writeStringToBuffer(int offset, int *buf, char *str, int max_str_length) {
    static int n;
    static int offset_aux;
    static int m;
    offset_aux = getPageStartAddress(offset);   //offset_aux starts at the beginning of the page
    //This for cycle contains a break statement
    for(n=0; n<PAGE_SIZE; n++) {
        if(offset_aux == offset) {
            for(m=0; m<max_str_length; m++){
                buf[n+m] = (int)str[m];
            }
            buf[n+max_str_length-1] = 0;    //NULL character for string ending
            break;          //exits the cycle once the string has been written
        }
        offset_aux = offset_aux + 2;    //+2 because of memory indexing
    }
    return;
}

/////////////////////
// DEBUG FUNCTIONS //
/////////////////////

//Print the 16 LSBs of one memory page (512 instructions) to the terminal
void printPage(int page, int offset) {
    offset = getPageStartAddress(offset);
    static int n;
    for(n=0; n<PAGE_SIZE; n=n+8) {
        printf("%d:\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r", n, flashGetInt(page, offset), flashGetInt(page, offset+2), flashGetInt(page, offset+4), flashGetInt(page, offset+6), flashGetInt(page, offset+8), flashGetInt(page, offset+10), flashGetInt(page, offset+12), flashGetInt(page, offset+14));
        offset = offset + 16;   //Each instruction is two memory locations, hence +16 and not +8
    }
    return;
}

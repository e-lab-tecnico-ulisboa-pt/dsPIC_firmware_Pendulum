#include <p33FJ128MC804.h>
#include "pendulum_N_oscs.h"
#include "flash.h"
#include "world_pendulum.h"

static unsigned long N;

unsigned long getGlobalNumberOfOscs() {
	return N;
}

void saveGlobalNumberOfOscs() {
    static unsigned long noOfOscs;
	static int offset_end;
    static int n;
    static unsigned long noOfOscs_max;
    static int offset_found;
    static unsigned long val;
    noOfOscs = N;
    offset_found = GLOBAL_NUMBER_OF_OSCS_ADDRESS;
    //2 pages dedicated to GlobalNoOfOscs
    offset_end = GLOBAL_NUMBER_OF_OSCS_ADDRESS + 2*2*PAGE_SIZE;    //each instruction corresponds to two in memory address
    
    //To perform reset or prevent overflow
    if( (noOfOscs == 0) || (noOfOscs >= (2147483647-NOSC_MAX)) ) {  //2^31 = 2147483648
        noOfOscs = 0;
        eraseFlashPage(PAGE_ADDRESS, GLOBAL_NUMBER_OF_OSCS_ADDRESS);
        eraseFlashPage(PAGE_ADDRESS, GLOBAL_NUMBER_OF_OSCS_ADDRESS+(2*PAGE_SIZE));
        flashWriteLong(PAGE_ADDRESS, GLOBAL_NUMBER_OF_OSCS_ADDRESS, (long) noOfOscs);
    }
    //Otherwise proceed with the normal writing procedure
    else {
        noOfOscs_max = 0;
        //Searches for the last written memory location with noOfOscs by finding the maximum
        for(n=GLOBAL_NUMBER_OF_OSCS_ADDRESS; n<offset_end; n=n+4) {
            val = (unsigned long) flashGetLong(PAGE_ADDRESS, n);
            if( (val > noOfOscs_max) && (val < (2147483647-NOSC_MAX)) ) { //takes into account the cast of -1 (memory signed integer value after erase)
                noOfOscs_max = val;
                offset_found = n;
            }
        }
        if(noOfOscs > noOfOscs_max){
            offset_found = offset_found + 4; //next memory location to write to (+4 because it is 16+16 bits: 2 instructions)
            if(offset_found == offset_end) offset_found = GLOBAL_NUMBER_OF_OSCS_ADDRESS; //When it reaches the end of the 2nd page starts rewriting on the 1st one
            if(offset_found == GLOBAL_NUMBER_OF_OSCS_ADDRESS) eraseFlashPage(PAGE_ADDRESS, GLOBAL_NUMBER_OF_OSCS_ADDRESS); //Erase the full 1st page before starting to (re)write on it
            if(offset_found == (GLOBAL_NUMBER_OF_OSCS_ADDRESS+(2*PAGE_SIZE))) eraseFlashPage(PAGE_ADDRESS, GLOBAL_NUMBER_OF_OSCS_ADDRESS+(2*PAGE_SIZE)); //Erase the full 2nd page before starting to (re)write on it
            flashWriteLong(PAGE_ADDRESS, offset_found, (long) noOfOscs);            
        }        
    }
    N = noOfOscs;
}

void incGlobalNumberOfOscs() {
	N++;
}

void retrieveGlobalNumberOfOscs() {
    int offset_end;
    int n;
    unsigned long noOfOscs_max;
    static unsigned long val;
    //offset = getPageStartAddress(offset); //redundant because offset should correspond to the beginning of a page
    //2 pages dedicated to GlobalNoOfOscs   
    offset_end = GLOBAL_NUMBER_OF_OSCS_ADDRESS + 2*2*PAGE_SIZE;
    noOfOscs_max = 0;
    //Searches for the last written memory location with noOfOscs by finding the maximum
    for(n=GLOBAL_NUMBER_OF_OSCS_ADDRESS; n<offset_end; n=n+4) {
        val = (unsigned long) flashGetLong(PAGE_ADDRESS, n);
        if( (val > noOfOscs_max) && (val < (2147483647-NOSC_MAX)) ) { //takes into account the cast of -1 (memory signed integer value after erase)
            noOfOscs_max = val;
        }
    }   
	N = noOfOscs_max;
}

void resetGlobalNumberOfOscs() {
	N = 0;
	saveGlobalNumberOfOscs();
}

void setGlobalNumberOfOscs(unsigned long val) {    
    if(val < N) {
        //reset first in order to clear the pages before writing the new value: 
        //val is lower than the current Global Number of Oscillations
        resetGlobalNumberOfOscs();
        N = val;
        saveGlobalNumberOfOscs();
    }
    else{
        N = val;
        saveGlobalNumberOfOscs();
    }
}

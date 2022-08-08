#ifndef MEMORY_H
#define	MEMORY_H

extern int ReadFlashWord(int page, int offset);
extern void EraseFlashPage(int page, int offset);
extern void WriteFlashWord(int page, int offset, int value);
extern void WriteSingleRow(int page, int offset, int *buf);

#endif	/* MEMORY_H */


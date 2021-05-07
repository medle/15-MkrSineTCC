/*
 * MkrUtil.h
 *
 * Created: 07.05.2021 14:13:38
 *  Author: SL
 */ 

#ifndef MKRUTIL_H_
#define MKRUTIL_H_

/*
 * Panic management functions.
 */
#define CHECK_PANICIF
//#undef CHECK_PANICIF

void panicAt(int code, const char *file, int line);
#define panic(code) panicAt(code, __FILE__, __LINE__)

#ifdef CHECK_PANICIF 
 #define panicIf(what) { int __x = (what); if(__x != 0) panic(__x); }
#else
 #define panicIf(what) ((void)0)
#endif

void blink(int numBlinks, int msDelayEach);

#endif /* MKRUTIL_H_ */
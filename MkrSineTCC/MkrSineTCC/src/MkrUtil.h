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
#define ENABLE_EXPECT0
//#undef ENABLE_EXPECT0

void panicAt(int code, const char *file, int line);
#define panic(code) panicAt(code, __FILE__, __LINE__)

#ifdef ENABLE_EXPECT0 
 #define expect0(what) { int __x = (what); if(__x != 0) panic(__x); }
#else
 #define expect0(what) (what)
#endif

void blink(int numBlinks, int msDelayEach);

int convertHertzToCycleMicroseconds(int hertz);
int convertCycleMicrosecondsToClocksPerCycle(int cycleMicroseconds);

#endif /* MKRUTIL_H_ */
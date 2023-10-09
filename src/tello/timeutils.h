/*
 * File name: timeutils.h
 * Date:      2010/11/25 10:10
 * Author:    Jan Chudoba
 */

#ifndef __TIMEUTILS_H__
#define __TIMEUTILS_H__

#include <stdint.h>
#include <time.h>

uint32_t getTimeMs();

class CRealTime
{
public:
   CRealTime() { t = NULL; }
   CRealTime(CRealTime & rt) { rawtime = rt.rawtime; if (rt.t) t=localtime(&rawtime); else t=NULL; }
   CRealTime(CRealTime & rt, int offset_ms) { rawtime = rt.rawtime + (offset_ms/1000); t=localtime(&rawtime); }

   void now();
   void increment_ms(int time_delay_ms) { rawtime += time_delay_ms / 1000; t=localtime(&rawtime); }

   char * strTimeStamp(char * buffer);
   char * strTime(char * buffer);
   char * strDate(char * buffer);

   bool valid() { return (t!=NULL); }

   int year() { return 1900 + t->tm_year; }
   int month() { return 1 + t->tm_mon; }
   int day() { return t->tm_mday; }

   int hour() { return t->tm_hour; }
   int min() { return t->tm_min; }
   int sec() { return t->tm_sec; }

   int weekDay() { return t->tm_wday; }

   time_t getRawTime() { return rawtime; }
protected:
   time_t rawtime;
   struct tm * t;
};

class CScopedStopwatch
{
public:
   CScopedStopwatch(int alarm, const char * label=NULL) { t0 = getTimeMs(); treshold=alarm; this->label=label; }
   int getTime() { return getTimeMs() - t0; }
   void check();
   ~CScopedStopwatch() { check(); }
private:
   uint32_t t0;
   uint32_t treshold;
   const char * label;
};

#endif

/* end of timeutils.h */

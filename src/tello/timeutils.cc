/*
 * File name: timeutils.cc
 * Date:      2010/11/25 10:10
 * Author:    Jan Chudoba
 */

#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>

#include "timeutils.h"

uint32_t getTimeMs()
{
	uint32_t t;
	struct timeval now;
	gettimeofday(&now, NULL);
	t = now.tv_sec * 1000 + now.tv_usec / 1000;
	return t;
}

void CRealTime::now()
{
   time ( &rawtime );
	t = localtime ( &rawtime );
}

char * CRealTime::strTimeStamp(char * buffer)
{
   sprintf ( buffer, "%d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), min(), sec());
   return buffer;
}

char * CRealTime::strTime(char * buffer)
{
   sprintf ( buffer, "%02d:%02d:%02d", hour(), min(), sec());
   return buffer;
}

char * CRealTime::strDate(char * buffer)
{
   sprintf ( buffer, "%d-%02d-%02d", year(), month(), day());
   return buffer;
}

// =============================================================================

void CScopedStopwatch::check()
{
   int dt = getTime();
   if (dt > (int)treshold) {
      printf("ALARM: stopwatch '%s' time %d ms\n", ((label)?label:"-") , dt);
   }
}


/* end of timeutils.cc */

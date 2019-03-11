#include "cros_defs.h"
#include "cros_clock.h"
#include <limits.h>

uint64_t cRosClockGetTimeMs()
{
  uint64_t ms_since_epoch;

  PRINT_VDEBUG ( "cRosClockGetTimeMs()\n" );

#ifdef _WIN32
  const uint64_t epoch_filetime = UINT64CONST(116444736000000000); // FILETIME on Jan 1 1970 00:00:00
  FILETIME    cur_filetime;
  SYSTEMTIME  cur_system_time;
  ULARGE_INTEGER cur_filetime_large;

  GetSystemTime(&cur_system_time);
  SystemTimeToFileTime(&cur_system_time, &cur_filetime);
  // It is recomended that the calculations are done using a ULARGE_INTEGER
  cur_filetime_large.LowPart = cur_filetime.dwLowDateTime;
  cur_filetime_large.HighPart = cur_filetime.dwHighDateTime;

  tp->tv_sec = (long) ((cur_filetime_large.QuadPart - epoch_filetime) / 10000000L);
  tp->tv_usec = (long) (system_time.wMilliseconds * 1000);

  ms_since_epoch = (uint64_t)(((cur_filetime_large.QuadPart - epoch_filetime) / 10000L) + system_time.wMilliseconds);
#else
  struct timeval tv;

  gettimeofday( &tv, NULL );
  ms_since_epoch = (uint64_t)tv.tv_sec*1000 + (uint64_t)tv.tv_usec/1000;
#endif

  return(ms_since_epoch);
}

struct timeval cRosClockGetTimeVal( uint64_t msec )
{
  PRINT_VDEBUG ( "cRosClockGetTimeVal() msec: %lu\n", msec );
  struct timeval tv;
  if (msec > ( LONG_MAX * 1000ULL ))
  {
    // Given timespan would overflow timeval
    tv.tv_sec = LONG_MAX;
    tv.tv_usec = 999999L;
  }
  else
  {
    tv.tv_sec = (long)(msec / 1000);
    tv.tv_usec = (long)(msec - tv.tv_sec * 1000ULL ) * 1000;
  }

  return tv;
}

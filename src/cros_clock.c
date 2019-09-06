#include <limits.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#  include <winsock2.h>
#else
#  include <time.h>
#  include <sys/time.h>
#endif

#include "cros_defs.h"
#include "cros_clock.h"

struct timeval cRosClockGetTimeSecUsec( void )
{
  struct timeval time_since_epoch;
  int ret_val;

  PRINT_VVDEBUG ( "cRosClockGetTimeSecUsec()\n" );

#ifdef _WIN32
  const uint64_t epoch_filetime = 116444736000000000ULL; // FILETIME on Jan 1 1970 00:00:00
  FILETIME    cur_filetime;
  SYSTEMTIME  cur_system_time;

  GetSystemTime(&cur_system_time);

  if(SystemTimeToFileTime(&cur_system_time, &cur_filetime) != 0) // If Success:
  {
    ULARGE_INTEGER cur_filetime_large;

    // It is recomended that the calculations are done using a ULARGE_INTEGER
    cur_filetime_large.LowPart = cur_filetime.dwLowDateTime;
    cur_filetime_large.HighPart = cur_filetime.dwHighDateTime;

    time_since_epoch.tv_sec = (long) ((cur_filetime_large.QuadPart - epoch_filetime) / 10000000L);
    time_since_epoch.tv_usec = (long) (cur_system_time.wMilliseconds * 1000);
    ret_val = 0;
  }
  else
    ret_val = -1;
#else
  ret_val = gettimeofday(&time_since_epoch, NULL);
#endif

  if(ret_val == -1) // Failure obtaning the time: set default values:
  {
    time_since_epoch.tv_sec = 0;
    time_since_epoch.tv_usec = 0;
  }

  return(time_since_epoch);
}

uint64_t cRosClockGetTimeMs( void )
{
  uint64_t ms_since_epoch;
  struct timeval tv;

  PRINT_VVDEBUG ( "cRosClockGetTimeMs()\n" );

  tv = cRosClockGetTimeSecUsec();
  ms_since_epoch = (uint64_t)tv.tv_sec*1000 + (uint64_t)tv.tv_usec/1000;

  return(ms_since_epoch);
}

struct timeval cRosClockGetTimeVal( uint64_t msec )
{
  PRINT_VVDEBUG ( "cRosClockGetTimeVal() msec: %lu\n", msec );
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

int64_t cRosClockGetTimeStamp(void)
{
  int64_t time_stamp_ret;
#ifdef _WIN32
  LARGE_INTEGER time_stamp_init;
  QueryPerformanceCounter(&time_stamp_init);
  time_stamp_ret = time_stamp_init.QuadPart;
#else
  struct timespec time_stamp_init;
  int fn_ret_val;
  fn_ret_val = clock_gettime(CLOCK_MONOTONIC, &time_stamp_init);
  if (fn_ret_val == 0) // clock_gettime success
    time_stamp_ret = time_stamp_init.tv_sec * 1000000000LL + time_stamp_init.tv_nsec;
  else
    time_stamp_ret = 0;
#endif
  return(time_stamp_ret);
}

double cRosClockTimeStampToUSec(int64_t time_stamp)
{
  double time_sec;
#ifdef _WIN32
  LARGE_INTEGER counter_freq;
  BOOL fn_ret_val;
  fn_ret_val = QueryPerformanceFrequency(&counter_freq);
  if (fn_ret_val) // QueryPerformanceFrequency success
    time_sec = (time_stamp * 1.0e6L) / counter_freq.QuadPart;
  else
    time_sec = 0;
#else
  time_sec = time_stamp / 1.0e3;
#endif
  return(time_sec);
}

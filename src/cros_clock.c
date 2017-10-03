#include "cros_defs.h"
#include "cros_clock.h"
#include <limits.h>

uint64_t cRosClockGetTimeMs()
{
  PRINT_VDEBUG ( "cRosClockGetTimeMs()\n" );
#ifdef _WIN32 || _WIN64
  return (uint64_t)timeGetTime();
#else
  struct timeval tv;
  gettimeofday( &tv, NULL );
  return (uint64_t)tv.tv_sec*1000 + (uint64_t)tv.tv_usec/1000;
#endif
}

struct timeval cRosClockGetTimeVal( uint64_t msec )
{
  PRINT_VDEBUG ( "cRosClockGetTimeVal() msec: %u\n", msec);
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

#include "cros_defs.h"
#include "cros_clock.h"

uint64_t cRosClockGetTimeMs()
{
  PRINT_VDEBUG ( "cRosClockGetTimeMs()\n" );
  struct timeval tv;
  gettimeofday( &tv, NULL );
  return (uint64_t)tv.tv_sec*1000 + (uint64_t)tv.tv_usec/1000;
}

struct timeval cRosClockGetTimeVal( uint64_t msec )
{
  PRINT_VDEBUG ( "cRosClockGetTimeVal()\n" );
  struct timeval tv;
  tv.tv_sec = (int)(msec / 1000);
  tv.tv_usec = (int)(1000 * (msec % 1000));
  return tv;
}
#ifndef _CROS_CLOCK_H_
#define _CROS_CLOCK_H_

#ifdef _WIN32 || _WIN64
#  include <winsock.h>
#else
#  include <sys/time.h>
#endif

#include <stdint.h>

/*! \defgroup cros_clock cROS clock
 *
 *  Utility functions for time management
 */

/*! \addtogroup cros_clock
 *  @{
 */

/*! \brief Return the current time, expressed as milliseconds since the Epoch
 * 
 *  \return The current time
 */
uint64_t cRosClockGetTimeMs();

/*! \brief Convert an interval expressed as milliseconds in a timeval structure, 
 *         that express the same interval as seconds and microseconds
 * 
 *  \return The time interval express with a timeval structure
 */
struct timeval cRosClockGetTimeVal( uint64_t msec );

/*! @}*/

#endif

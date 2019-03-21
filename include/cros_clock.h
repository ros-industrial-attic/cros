#ifndef _CROS_CLOCK_H_
#define _CROS_CLOCK_H_

// declaration of struct timeval:
#ifdef _WIN32
#  include <winsock2.h>
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

/*! \brief Return the current time, expressed as seconds and microseconds since the Epoch
 *
 *  \return The current time
 */
struct timeval cRosClockGetTimeSecUsec( void );

/*! \brief Return the current time, expressed as milliseconds since the Epoch
 *
 *  \return The current time
 */
uint64_t cRosClockGetTimeMs( void );

/*! \brief Convert an interval expressed as milliseconds in a timeval structure,
 *         that express the same interval as seconds and microseconds
 *
 *  \return The time interval express with a timeval structure
 */
struct timeval cRosClockGetTimeVal( uint64_t msec );

/*! @}*/

#endif

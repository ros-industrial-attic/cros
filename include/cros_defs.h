#ifndef _CROS_DEFS_H_
#define _CROS_DEFS_H_

#include <stdio.h>
#include "cros_node.h"

/*! \defgroup cros_defs Cros defintions */

/*! \addtogroup cros_defs
 *  @{
 */

#define LITTLE_ENDIAN_ARC 1

#ifndef CROS_DEBUG_LEVEL
#  define CROS_DEBUG_LEVEL 3
#endif

#if CROS_DEBUG_LEVEL >= 1
#  define PRINT_INFO(...) fprintf(cRosOutStreamGet(),__VA_ARGS__)
#else
#  define PRINT_INFO(...)
#endif

#if CROS_DEBUG_LEVEL >= 2
#  define PRINT_DEBUG(...) printf(__VA_ARGS__)
#else
#  define PRINT_DEBUG(...)
#endif

#if CROS_DEBUG_LEVEL >= 3
#  define PRINT_VDEBUG(...) printf(__VA_ARGS__)
#else
#  define PRINT_VDEBUG(...)
#endif

#if CROS_DEBUG_LEVEL >= 4
#  define PRINT_VVDEBUG(...) printf(__VA_ARGS__)
#else
#  define PRINT_VVDEBUG(...)
#endif

#define PRINT_ERROR(...) fprintf(cRosOutStreamGet(), __VA_ARGS__)

#if LITTLE_ENDIAN_ARC
#  define HOST_TO_ROS_UINT32( val, converted_val ) converted_val = (val)
#else
#define HOST_TO_ROS_UINT32( val, converted_val )\
{\
  converted_val =  (((val)>>24)&0xff) |   \
                    (((val)<<8)&0XFF0000) | \
                    (((val)>>8)&0XFF00) |   \
                    (((val)<<24)&0XFF000000;\
}
#endif

#define ROS_TO_HOST_UINT32( val, converted_val ) HOST_TO_ROS_UINT32( val, converted_val )

/*! @}*/

#endif

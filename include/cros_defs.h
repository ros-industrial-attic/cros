#ifndef _CROS_DEFS_H_
#define _CROS_DEFS_H_

#include <stdio.h>
#include "cros_node.h"

/*! \defgroup cros_defs Cros defintions */

/*! \addtogroup cros_defs
 *  @{
 */

#ifndef CROS_DEBUG_LEVEL
#  define CROS_DEBUG_LEVEL 1
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

#define FLUSH_PRINT() fflush(cRosOutStreamGet())

// macros for detecting and converting endianness

// We ignore the existence of mixed endianness
#ifdef _WIN32
#  define LITTLE_ENDIAN_ARC 1
#else
#  ifdef __APPLE__
#    ifndef __BIG_ENDIAN__
#      define LITTLE_ENDIAN_ARC 1
#    endif
#  else
#    include <endian.h>
#    if __BYTE_ORDER == __LITTLE_ENDIAN
#      define LITTLE_ENDIAN_ARC 1
#    endif
#  endif
#endif

// We only support processors with sizeof(char)==1
#if LITTLE_ENDIAN_ARC
#  define HOST_TO_ROS_UINT32(val) (val)
#else
#  define HOST_TO_ROS_UINT32(val) (((val)>>24)&0x000000FFUL) | \
                                  (((val)<<8)&0X00FF0000UL)  | \
                                  (((val)>>8)&0X0000FF00UL)  | \
                                  (((val)<<24)&0XFF000000UL)
#endif

#define ROS_TO_HOST_UINT32 HOST_TO_ROS_UINT32

/*! @}*/

#endif

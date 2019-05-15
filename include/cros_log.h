/*! \file cros_log.h
 *  \brief This file declares the function and macros (ROS_INFO, ROS_DEBUG, ROS_WARN, ROS_ERROR and ROS_FATAL)
 *         for printing messages using the ROS log. These messages are sent through the /rosout topic to the /rosout node.
 *
 * These macros must not be confused with the macros for printing messages localy (either in a local log file or local console) which
 * are defined in cros_defs.h: PRINT_INFO, PRINT_DEBUG, PRINT_VDEBUG, PRINT_VVDEBUG and PRINT_ERROR
 */

#ifndef _CROS_LOG_H_
#define _CROS_LOG_H_

#include <stdio.h>
#include <stdint.h>

#define PRINT_LOG(node,log_level,...) \
     cRosLogPrint(node,\
                  log_level,\
                  __FILE__,\
                  __FUNCTION__,\
                  __LINE__,\
                  __VA_ARGS__)

#define ROS_INFO(node,...) PRINT_LOG(node, CROS_LOGLEVEL_INFO, __VA_ARGS__)
#define ROS_DEBUG(node,...) PRINT_LOG(node, CROS_LOGLEVEL_DEBUG, __VA_ARGS__)
#define ROS_WARN(node,...) PRINT_LOG(node, CROS_LOGLEVEL_WARN, __VA_ARGS__)
#define ROS_ERROR(node,...) PRINT_LOG(node, CROS_LOGLEVEL_ERROR, __VA_ARGS__)
#define ROS_FATAL(node,...) PRINT_LOG(node, CROS_LOGLEVEL_FATAL, __VA_ARGS__)

typedef struct CrosLog CrosLog;
typedef struct CrosLogNode CrosLogNode;
typedef struct CrosLogQueue CrosLogQueue;

struct CrosNode; // We forward declare CrosNode struct since it is used by cRosLogPrint() before it is declared

struct CrosLog
{
  uint8_t level;   //! debug level
  char* name;      //! name of the node
  char* msg;       //! message
  char* file;      //! file the message came from
  char* function;  //! function the message came from
  uint32_t line;   //! line the message came from
  char** pubs;     //! topic names that the node publishes
  uint32_t secs;
  uint32_t nsecs;
  size_t n_pubs;
};

struct CrosLogNode
{
  CrosLog *call;
  CrosLogNode* next;
};

struct CrosLogQueue
{
  CrosLogNode* head;
  CrosLogNode* tail;
  size_t count;
};

typedef enum CrosLogLevel //!Logging levels
{
  CROS_LOGLEVEL_DEBUG = 1,
  CROS_LOGLEVEL_INFO = 2,
  CROS_LOGLEVEL_WARN = 4,
  CROS_LOGLEVEL_ERROR = 8,
  CROS_LOGLEVEL_FATAL = 16
} CrosLogLevel;

CrosLog *cRosLogNew(void);
void cRosLogFree(CrosLog *log);

void cRosLogPrint(struct CrosNode *node,
                  CrosLogLevel level,   // debug level
                  const char *file,     // file the message came from
                  const char *function, // function the message came from
                  uint32_t line,
                  const char *msg, ...);

CrosLogQueue *cRosLogQueueNew(void);
void cRosLogQueueInit(CrosLogQueue *queue);
int cRosLogQueueEnqueue(CrosLogQueue *queue, CrosLog* log);
CrosLog *cRosLogQueuePeek(CrosLogQueue *queue);
CrosLog *cRosLogQueueDequeue(CrosLogQueue *queue);
void cRosLogQueueRelease(CrosLogQueue *queue);
size_t cRosLogQueueCount(CrosLogQueue *queue);
int cRosLogQueueIsEmpty(CrosLogQueue *queue);

#endif //_CROS_LOG_H

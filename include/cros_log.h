#ifndef _CROS_LOG_H_
#define _CROS_LOG_H_

#include <stdint.h>
#include "cros_node.h"

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

CrosLog * cRosLogNew();
void cRosLogFree(CrosLog *log);

void cRosLogPrint(CrosNode* node,
                  CrosLogLevel level,   // debug level
                  const char* file,     // file the message came from
                  const char* function, // function the message came from
                  uint32_t line,
                  const char* msg, ...);

CrosLogQueue* cRosLogQueueNew();
void cRosLogQueueInit(CrosLogQueue *queue);
int cRosLogQueueEnqueue(CrosLogQueue *queue, CrosLog* log);
CrosLog * cRosLogQueuePeek(CrosLogQueue *queue);
CrosLog * cRosLogQueueDequeue(CrosLogQueue *queue);
void cRosLogQueueRelease(CrosLogQueue *queue);
size_t cRosLogQueueCount(CrosLogQueue *queue);
int cRosLogQueueIsEmpty(CrosLogQueue *queue);

#endif //_CROS_LOG_H_

#ifndef _CROS_LOG_H_
#define _CROS_LOG_H_

#include <stdint.h>

#define PRINT_LOG(log_level, ...) \
    cRosLogPrint(log_level,\
                 __FILE__,\
                 __FUNCTION__,\
                 __LINE__,\
                 __VA_ARGS__)\

#define ROS_INFO(...) PRINT_LOG(CROS_LOGLEVEL_INFO, __VA_ARGS__)
#define ROS_DEBUG(...) PRINT_LOG(CROS_LOGLEVEL_DEBUG, __VA_ARGS__)
#define ROS_WARN(...) PRINT_LOG(CROS_LOGLEVEL_WARN, __VA_ARGS__)
#define ROS_ERROR(...) PRINT_LOG(CROS_LOGLEVEL_ERROR, __VA_ARGS__)
#define ROS_FATAL(...) PRINT_LOG(CROS_LOGLEVEL_FATAL, __VA_ARGS__)

typedef struct CrosLog CrosLog;
typedef struct CrosLogNode CrosLogNode;
typedef struct CrosLogQueue CrosLogQueue;

typedef enum CrosLogLevel //!Logging levels
{
  CROS_LOGLEVEL_DEBUG = 1,
  CROS_LOGLEVEL_INFO = 2,
  CROS_LOGLEVEL_WARN = 4,
  CROS_LOGLEVEL_ERROR = 8,
  CROS_LOGLEVEL_FATAL = 16
} CrosLogLevel;

struct CrosLog
{
  uint8_t level;    //! debug level
  char* name;       //! name of the node
  char* msg;        //! message
  char* file;       //! file the message came from
  char* function;   //! function the message came from
  uint32_t line;    //! line the message came from
  char** pubs;    //! topic names that the node publishes
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

CrosLog * cRosLogNew();
void cRosLogFree(CrosLog *log);

void cRosLogPrint(CrosLogLevel level,         // debug level
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

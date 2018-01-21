#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <sys/time.h>

#include "cros_log.h"
#include "cros_defs.h"
#include "cros_node.h"

FILE **Msg_output = &stdout; //! The pointer to file stream used to print all messages (except debug messages)

FILE *cRosOutStreamGet(void)
{
  return *Msg_output;
}

void cRosOutStreamSet(FILE *new_stream)
{
  static FILE *msg_out_stream;
  msg_out_stream = new_stream;
  Msg_output = &msg_out_stream;
}

CrosLog *cRosLogNew(void)
{
  CrosLog *ret = (CrosLog *)malloc(sizeof(CrosLog));
  ret->file = NULL;
  ret->level = CROS_LOGLEVEL_INFO;
  ret->function = NULL;
  ret->msg = NULL;
  ret->name = NULL;
  ret->pubs = NULL;
  ret->n_pubs = 0;
  return ret;
}

void cRosLogFree(CrosLog *log)
{
  free(log->file);
  free(log->function);
  free(log->msg);
  free(log->name);
  int i;
  for(i = 0; i < log->n_pubs; i++)
  {
    free(log->pubs[i]);
  }
  free(log->pubs);
  free(log);
}

CrosLogQueue* cRosLogQueueNew(void)
{
  CrosLogQueue* new_queue = calloc(1,sizeof(CrosLogQueue));
  cRosLogQueueInit(new_queue);
  return new_queue;
}

void cRosLogQueueInit(CrosLogQueue *queue)
{
  queue->tail = NULL;
  queue->head = NULL;
  queue->count = 0;
}

CrosLog * cRosLogQueuePeek(CrosLogQueue *queue)
{
  if (queue->head == NULL)
    return NULL;

  return queue->head->call;
}

int cRosLogQueueEnqueue(CrosLogQueue *queue, CrosLog* CrosLog)
{
  CrosLogNode* node = malloc(sizeof(CrosLogNode));

  if(node == NULL)
  {
    PRINT_ERROR("enqueueCrosLog() : Can't enqueue call\n");
    return -1;
  }

  node->call = CrosLog;
  node->next = NULL;

  if(queue->head == NULL)
  {
    queue->head = node;
    queue->tail = node;
  }
  else
  {
    queue->tail->next = node;
    queue->tail = node;
  }

  queue->count++;

  return 0;
}

CrosLog * cRosLogQueueDequeue(CrosLogQueue *queue)
{
  CrosLogNode* head = queue->head;
  queue->head = head->next;
  if(queue->head == NULL)
    queue->tail = queue->head;

  CrosLog *call = head->call;
  free(head);

  queue->count--;

  return call;
}

void cRosLogQueueRelease(CrosLogQueue *queue)
{
  CrosLogNode *current = queue->head;
  while(current != NULL)
  {
    cRosLogFree(current->call);
    CrosLogNode *next = current->next;
    free(current);
    current = next;
  }

  cRosLogQueueInit(queue);
}

size_t cRosLogQueueCount(CrosLogQueue *queue)
{
  return queue->count;
}

int cRosLogQueueIsEmpty(CrosLogQueue *queue)
{
  return queue->count == 0;
}

void cRosLogPrint(CrosNode* node,
                  CrosLogLevel level,   // debug level
                  const char* file,     // file the message came from
                  const char* function, // function the message came from
                  uint32_t line,
                  const char* msg, ...) // message
{
  char* log_msg = NULL;
  va_list args;

  va_start(args,msg);

  struct timeval wall_time;

  if(gettimeofday(&wall_time, NULL) == -1)
  {
    wall_time.tv_sec = 0;
    wall_time.tv_usec = 0;
  }


  if(node == NULL)
  {

    fprintf(cRosOutStreamGet(), "\n[%d,%ld] ", (int)wall_time.tv_sec, (long)wall_time.tv_usec);
    size_t msg_size = strlen(msg) + 512;

    log_msg = calloc(msg_size + 1, sizeof(char));
    vsprintf(log_msg,msg,args);

    switch(level)
    {
      case CROS_LOGLEVEL_INFO:
      case CROS_LOGLEVEL_WARN:
      {
        PRINT_INFO("%s", log_msg);
        break;
      }
      case CROS_LOGLEVEL_DEBUG:
      {
        PRINT_DEBUG("%s", log_msg);
        break;
      }
      case CROS_LOGLEVEL_ERROR:
      case CROS_LOGLEVEL_FATAL:
      {
        PRINT_ERROR("%s", log_msg);
        break;
      }
    }

    va_end(args);
    free(log_msg);
    return;
  }

  // We need to use the argument list twice (the 1st time to call vprintf and the 2nd time to call vsprintf), so we make a copy
  va_list args_copy;
  va_copy(args_copy, args);

  if(level != node->log_level &&
      (level != CROS_LOGLEVEL_ERROR || level != CROS_LOGLEVEL_FATAL))
    return;

  CrosLog* log = cRosLogNew();

  log->secs = wall_time.tv_sec;
  log->nsecs = (uint32_t)wall_time.tv_usec;

  log->level = level;

  log->file =  calloc(strlen(file)+1, sizeof(char));
  strncpy(log->file, file,strlen(file));

  log->function =  calloc(strlen(function)+1, sizeof(char));
  strncpy(log->function, function,strlen(function));

  log->line = line;

  int i;
  log->n_pubs = node->n_pubs;
  log->pubs = (char**) calloc(log->n_pubs,sizeof(char*));

  for(i = 0; i <node->n_pubs; i++)
  {
    if(node->pubs[i].topic_name != NULL)
    {
      log->pubs[i] = calloc(strlen(node->pubs[i].topic_name) + 1, sizeof(char));
      strncpy(log->pubs[i], node->pubs[i].topic_name,strlen(node->pubs[i].topic_name));
    }
    else
      log->pubs[i] = NULL;
  }

  fprintf(cRosOutStreamGet(),"\n[%d,%d] ",log->secs, log->nsecs);

  size_t msg_size = vfprintf(cRosOutStreamGet(),msg,args) + 512;

  log_msg = calloc(msg_size + 1, sizeof(char));
  vsprintf(log_msg,msg,args_copy);
  log->msg = log_msg;

  va_end(args);
  va_end(args_copy);
  cRosLogQueueEnqueue(node->log_queue, log);
}

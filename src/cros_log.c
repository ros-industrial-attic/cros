#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "cros_log.h"
#include "cros_defs.h"
#include "cros_node.h"
#include "cros_clock.h"
#include "cros_message.h"
#include "cros_api.h"

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
  size_t i;
  for(i = 0; i < log->n_pubs; i++)
  {
    free(log->pubs[i]);
  }
  free(log->pubs);
  free(log);
}

cRosMessage *cRosLogToMessage(CrosNode* node, CrosLog* log)
{
  cRosMessage *message;
  size_t pub_ind;

  message = cRosApiCreatePublisherMessage(node, node->rosout_pub_idx);
  if(message != NULL)
  {
    cRosMessageField* header_field = cRosMessageGetField(message, "header");
    cRosMessage* header_msg = header_field->data.as_msg;
    cRosMessageField* seq_id = cRosMessageGetField(header_msg, "seq");
    seq_id->data.as_uint32 = node->log_last_id++;
    cRosMessageField* time_field = cRosMessageGetField(header_msg, "stamp");
    cRosMessage* time_msg = time_field->data.as_msg;
    cRosMessageField* time_secs = cRosMessageGetField(time_msg, "secs");
    time_secs->data.as_uint32 = log->secs;
    cRosMessageField* time_nsecs = cRosMessageGetField(time_msg, "nsecs");
    time_nsecs->data.as_uint32 = log->nsecs;
    cRosMessageSetFieldValueString(cRosMessageGetField(header_msg, "frame_id"), "0");


    cRosMessageField* level = cRosMessageGetField(message, "level");
    level->data.as_uint8 = log->level;

    cRosMessageField* name = cRosMessageGetField(message, "name"); // name of the node
    cRosMessageSetFieldValueString(name, node->name);

    cRosMessageField* msg = cRosMessageGetField(message, "msg"); // message
    cRosMessageSetFieldValueString(msg, log->msg);

    cRosMessageField* file = cRosMessageGetField(message, "file"); // file the message came from
    cRosMessageSetFieldValueString(file, log->file);

    cRosMessageField* function = cRosMessageGetField(message, "function"); // function the message came from
    cRosMessageSetFieldValueString(function, log->function);

    cRosMessageField* line = cRosMessageGetField(message, "line"); // line the message came from
    line->data.as_uint32 = log->line;

    cRosMessageField* topics = cRosMessageGetField(message, "topics"); // topic names that the node publishes

    for(pub_ind = 0; pub_ind < log->n_pubs; pub_ind++)
    {
      cRosMessageFieldArrayPushBackString(topics, log->pubs[pub_ind]);
    }
  }
  return(message);
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

  wall_time = cRosClockGetTimeSecUsec();

  if(node == NULL)
  {

    fprintf(cRosOutStreamGet(), "\n[%d,%ld] ", (int)wall_time.tv_sec, (long)wall_time.tv_usec);
    size_t msg_size = strlen(msg) + 512;

    log_msg = (char *)calloc(msg_size + 1, sizeof(char));
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
        PRINT_VDEBUG("%s", log_msg);
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

  log->file =  (char *)calloc(strlen(file)+1, sizeof(char));
  strncpy(log->file, file,strlen(file));

  log->function =  (char *)calloc(strlen(function)+1, sizeof(char));
  strncpy(log->function, function,strlen(function));

  log->line = line;

  int i;
  log->n_pubs = node->n_pubs;
  log->pubs = (char **)calloc(log->n_pubs,sizeof(char*));

  for(i = 0; i <node->n_pubs; i++)
  {
    if(node->pubs[i].topic_name != NULL)
    {
      log->pubs[i] = (char *)calloc(strlen(node->pubs[i].topic_name) + 1, sizeof(char));
      strncpy(log->pubs[i], node->pubs[i].topic_name,strlen(node->pubs[i].topic_name));
    }
    else
      log->pubs[i] = NULL;
  }

  fprintf(cRosOutStreamGet(),"\n[%d,%d] ",log->secs, log->nsecs);

  size_t msg_size = vfprintf(cRosOutStreamGet(),msg,args) + 512;

  log_msg = (char *)calloc(msg_size + 1, sizeof(char));
  vsprintf(log_msg,msg,args_copy);
  log->msg = log_msg;

  va_end(args);
  va_end(args_copy);

  cRosMessage *topic_msg;
  topic_msg = cRosLogToMessage(node, log);
  if(topic_msg != NULL)
  {
    cRosErrCodePack err_cod;

    //err_cod = cRosNodeSendTopicMsg(node, node->rosout_pub_idx, topic_msg, 0);cRosNodeQueueTopicMsg
    err_cod = cRosNodeQueueTopicMsg(node, node->rosout_pub_idx, topic_msg);
    if (err_cod != CROS_SUCCESS_ERR_PACK)
      cRosPrintErrCodePack(err_cod, "cRosLogPrint() : A message of /rosout topic could not be sent");
    cRosMessageFree(topic_msg);
  }
  else
  {
    PRINT_ERROR ( "cRosLogPrint() : A message of /rosout topic could not be created to be sent.\n" );
  }
  cRosLogFree(log);
}

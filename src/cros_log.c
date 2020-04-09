#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#ifdef _WIN32
#  define strcasecmp _stricmp // This is the POSIX verion of strnicmp
#endif

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

int stringToLogLevel(const char* level_str, CrosLogLevel *level_num)
{
  int ret;
  ret = 0; // Default return value: success
  if(strcasecmp("Info",level_str) == 0)
  {
    *level_num = CROS_LOGLEVEL_INFO;
  }
  else if(strcasecmp("Debug",level_str) == 0)
  {
    *level_num = CROS_LOGLEVEL_DEBUG;
  }
  else if(strcasecmp("Warn",level_str) == 0)
  {
    *level_num = CROS_LOGLEVEL_WARN;
  }
  else if(strcasecmp("Error",level_str) == 0)
  {
    *level_num = CROS_LOGLEVEL_ERROR;
  }
  else if(strcasecmp("Fatal",level_str) == 0)
  {
    *level_num = CROS_LOGLEVEL_FATAL;
  }
  else
  {
    ret = -1; // Unknown input level string: return error
  }
  return ret;
}

const char *LogLevelToString(CrosLogLevel log_level)
{
  char *ret;

  switch(log_level)
  {
    case CROS_LOGLEVEL_INFO:
      ret = "INFO";
      break;
    case CROS_LOGLEVEL_DEBUG:
      ret = "DEBUG";
      break;
    case CROS_LOGLEVEL_WARN:
      ret = "WARN";
      break;
    case CROS_LOGLEVEL_ERROR:
      ret = "ERROR";
      break;
    case CROS_LOGLEVEL_FATAL:
      ret = "FATAL";
      break;
    default:
      ret = "UNKNOWN";
  }
  return ret;
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

    cRosMessageFieldArrayClear(topics);
    for(pub_ind = 0; pub_ind < log->n_pubs; pub_ind++)
      cRosMessageFieldArrayPushBackString(topics, log->pubs[pub_ind]);
  }
  return(message);
}

void cRosLogPrint(CrosNode* node,
                  CrosLogLevel level,   // debug level
                  const char* file,     // file the message came from
                  const char* function, // function the message came from
                  uint32_t line,
                  const char* msg_fmt_str, ...) // message
{
  // Only print the message localy if its priority level is equal or higher than the current log
  // priority set for this ROS node or we cannot find out the node's prioroty level
  if(node == NULL || level >= node->log_level)
  {
    struct timeval wall_time;
    int msg_str_size;
    va_list msg_str_args;

    wall_time = cRosClockGetTimeSecUsec();

    fprintf(cRosOutStreamGet(), "[%s] [%d,%ld] ", LogLevelToString(level), (int)wall_time.tv_sec, (long)wall_time.tv_usec*1000);
    va_start(msg_str_args, msg_fmt_str);
    msg_str_size = vfprintf(cRosOutStreamGet(), msg_fmt_str, msg_str_args);
    va_end(msg_str_args);

    // Only print the message in rosout if its priority level is equal or higher than the node's current log priority
    if(node != NULL)
    {
      int pub_ind;

      CrosLog* log = cRosLogNew();

      log->secs = wall_time.tv_sec;
      log->nsecs = (uint32_t)wall_time.tv_usec*1000;

      log->level = level;

      log->file = strdup(file); // We need to make a string memory copy since it is freed later in the cRosLogFree() call

      log->function = strdup(function);

      log->line = line;

      log->n_pubs = node->n_pubs;
      log->pubs = (char **)calloc(log->n_pubs,sizeof(char*));

      for(pub_ind = 0; pub_ind < node->n_pubs; pub_ind++)
      {
        if(node->pubs[pub_ind].topic_name != NULL)
          log->pubs[pub_ind] = strdup(node->pubs[pub_ind].topic_name);
        else
          log->pubs[pub_ind] = NULL;
      }

      log->msg = (char *)malloc((msg_str_size + 1)*sizeof(char));

      // We need to use the argument list twice (the 1st time to call vfprintf and the 2nd time to call vsprintf), so we start the arg list again here
      va_start(msg_str_args, msg_fmt_str);
      vsprintf(log->msg, msg_fmt_str, msg_str_args);
      va_end(msg_str_args);

      cRosMessage *topic_msg;
      topic_msg = cRosLogToMessage(node, log);
      if(topic_msg != NULL)
      {
        cRosErrCodePack err_cod;
        int rosout_pub_idx = node->rosout_pub_idx;

        //err_cod = cRosNodeSendTopicMsg(node, rosout_pub_idx, topic_msg, 0);
        err_cod = cRosNodeQueueTopicMsg(node, rosout_pub_idx, topic_msg);
        if (err_cod != CROS_SUCCESS_ERR_PACK)
        {
          // Check if the rossout publisher has any TCP process associated, that is, check if a node is subscribed to this topic
          int srv_proc_ind, subs_node;
          subs_node = 0;
          for(srv_proc_ind=0;srv_proc_ind<CN_MAX_TCPROS_SERVER_CONNECTIONS && subs_node==0;srv_proc_ind++)
            if(node->tcpros_server_proc[srv_proc_ind].topic_idx == rosout_pub_idx)
              subs_node = 1; // there is a subscriber
          // Only print the error message if there is a subscriber node, that is, if there is a node that should receive the rosout message
          if(subs_node == 1)
             cRosPrintErrCodePack(err_cod, "cRosLogPrint() : A message of /rosout topic could not be sent");
        }
        cRosMessageFree(topic_msg);
      }
      else
      {
        PRINT_ERROR ( "cRosLogPrint() : A message of /rosout topic could not be created to be sent.\n" );
      }
      cRosLogFree(log);
    }
  }
}

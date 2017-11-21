#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <time.h>
#include <unistd.h>

#include <cros_defs.h>
#include <cros_node.h>
#include <cros_api.h>
#include <cros_api_internal.h>
#include <cros_clock.h>
#include <cros_gentools.h>
#include <cros_node_api.h>
#include <cros_message.h>
#include <cros_service.h>

// TODO Signal handler

static void printHelp( char *cmd_name )
{
  printf("Usage: %s [OPTION] ... \n", cmd_name);
  printf("Options:\n");
  printf("\t-name  <node_host>       Set the node name (default: /test_node)\n");
  printf("\t-host  <node_host>       Set the node host (default: 127.0.0.1)\n");
  printf("\t-chost <roscore host>    Set the roscore host (default: 127.0.0.1)\n");
  printf("\t-cport <roscore port>    Set the roscore port (default: 11311)\n");
  printf("\t-h                       Print this help\n");
}

CrosNode *node;

static int counter = 0;
static uint64_t start_timer = 0;

static char message_buffer_counter[100];
static char message_buffer_clock[100];

#define DOUBLE_VECTOR_SIZE 10
static double message_buffer_double_vector[DOUBLE_VECTOR_SIZE];

static cRosErrCodePack gripperstatus_callback(DynBuffer *buffer, void* data_context)
{
  const unsigned char* data = dynBufferGetData(buffer);

  uint32_t PowerStatus;
  uint32_t ClampStatus[4];
  uint32_t NAlarmCodes;
  uint32_t Configuration;

  size_t pos = 0;
  PowerStatus = *(uint32_t *)(data + pos);
  pos += sizeof(uint32_t);
  memcpy(ClampStatus, data + pos, sizeof(ClampStatus));
  pos += 4*sizeof(uint32_t);
  NAlarmCodes = *(uint32_t *)(data + pos);
  pos += sizeof(uint32_t);

  uint32_t *AlarmCodes = (uint32_t *)malloc(NAlarmCodes*sizeof(uint32_t));
  if (!AlarmCodes)
  {
    PRINT_ERROR("Out of memory while reading from socket");
    exit(1);
  }

  memcpy(AlarmCodes, data + pos, NAlarmCodes*sizeof(uint32_t));
  pos += NAlarmCodes*sizeof(uint32_t);
  Configuration = *(uint32_t *)(data + pos);

  printf("PowerStatus: %d\n", PowerStatus);
  printf("ClampStatus: [%d %d %d %d]\n", ClampStatus[0], ClampStatus[1],
         ClampStatus[2], ClampStatus[3]);
  printf("AlarmCodes: [");
  int first = 1;
  int it = 0;
  for (; it < NAlarmCodes; it++)
  {
    if (first)
      first = 0;
    else
      printf(" ");

    printf("%d", AlarmCodes[it]);
  }
  printf("]\n");
  printf("Configuration: %d\n", Configuration);
  fflush(stdout);

  free(AlarmCodes);

  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack gripperjointstate_callback(DynBuffer *buffer, void* data_context)
{
  const unsigned char* data = dynBufferGetData(buffer);

  double Position[9];
  memcpy(Position, &data[0], sizeof(Position));

  printf("Position: [%f %f %f %f %f %f %f %f %f]\n", Position[0], Position[1], Position[2],
    Position[3], Position[4], Position[5], Position[6], Position[7], Position[8]);

  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static CallbackResponse gripperjointstate_callbacknewapi(cRosMessage *message, void* context)
{
  return 0;
}

static cRosErrCodePack bool_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  // Maps to uint8
  dynBufferPushBackUInt8( buffer, 1 );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack byte_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  // Deprecated: alias for int8
  dynBufferPushBackInt8( buffer, -3 );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack char_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  // Deprecated: alias for uint8
  dynBufferPushBackUInt8( buffer, 'a' );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack duration_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  dynBufferPushBackInt32( buffer, 2 );
  dynBufferPushBackInt32( buffer, 3 );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack header_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  dynBufferPushBackUInt32( buffer, 2 );
  dynBufferPushBackUInt32( buffer, 6 );
  dynBufferPushBackUInt32( buffer, 21 );
  dynBufferPushBackUInt32( buffer, 4 );
  dynBufferPushBackBuf( buffer, (unsigned char*)"Ciao", sizeof("Ciao") - 1 );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack int16_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  dynBufferPushBackInt16( buffer, -1024 );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack int32_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  dynBufferPushBackInt32( buffer, -10000000 );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack int64_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  dynBufferPushBackInt64( buffer, -10000000001 );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack int8_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  dynBufferPushBackInt8( buffer, -5 );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack time_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  dynBufferPushBackUInt32( buffer, 4 );
  dynBufferPushBackUInt32( buffer, 12 );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack uint16_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  dynBufferPushBackUInt16( buffer, 1024 );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack uint32_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  dynBufferPushBackUInt32( buffer, 10000000 );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack uint64_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  dynBufferPushBackUInt64( buffer, 10000000001 );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack uint8_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  dynBufferPushBackUInt8( buffer, 5 );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack float32_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  dynBufferPushBackFloat32( buffer, 0.3f );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack float64_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  dynBufferPushBackFloat64( buffer, 0.5 );
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack bool_sub_callback(DynBuffer *buffer, void* data_context)
{
  // Maps to uint8
  printf("Read: %u\n", *((uint8_t *)buffer->data));
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack byte_sub_callback(DynBuffer *buffer, void* data_context)
{
  // Deprecated: alias for int8
  printf("Read: %i\n", *((int8_t *)buffer->data));
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack char_sub_callback(DynBuffer *buffer, void* data_context)
{
  // Deprecated: alias for uint8
  printf("Read: %u\n", *((uint8_t *)buffer->data));
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack duration_sub_callback( DynBuffer *buffer, void* data_context )
{
  printf("Read: sec=%i , nsec=%i\n", *((int32_t *)buffer->data), *(int32_t *)(buffer->data + sizeof(int32_t)));
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack header_sub_callback(DynBuffer *buffer, void* data_context)
{
  int pos = 0;
  char *data = (char *)buffer->data;
  uint32_t seq = *((uint32_t *)data + pos);
  printf("Read: seq=%u , ", seq);
  pos += sizeof(uint32_t);
  uint32_t stamp_sec = *(uint32_t *)(data + pos);
  printf("stamp.sec=%u , ", stamp_sec);
  pos += sizeof(uint32_t);
  uint32_t stamp_nsec = *(uint32_t *)(data + pos);
  printf("stamp.nsec=%u , ", stamp_nsec);
  fflush(stdout);
  pos += sizeof(uint32_t);
  uint32_t strlen =  *(uint32_t *)(data + pos);
  pos += sizeof(uint32_t);
  char string[100];
  memcpy(string, data + pos, strlen);
  string[strlen] = '\0';
  printf("frame_id=%s\n", string);
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack int16_sub_callback(DynBuffer *buffer, void* data_context)
{
  printf("Read: %i\n", *(int16_t *)buffer->data);
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack int32_sub_callback(DynBuffer *buffer, void* data_context)
{
  printf("Read: %i\n", *(int32_t *)buffer->data);
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack int64_sub_callback(DynBuffer *buffer, void* data_context)
{
  printf("Read: %lli\n", (long long)*(int64_t *)buffer->data);
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack int8_sub_callback(DynBuffer *buffer, void* data_context)
{
  printf("Read: %i\n", *(int8_t *)buffer->data);
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack time_sub_callback(DynBuffer *buffer, void* data_context)
{
  printf("Read: sec=%u , nsec=%u\n", *((uint32_t *)buffer->data), *(uint32_t *)(buffer->data + sizeof(uint32_t)));
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack uint16_sub_callback(DynBuffer *buffer, void* data_context)
{
  printf("Read: %u\n", *(uint16_t *)buffer->data);
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack uint32_sub_callback(DynBuffer *buffer, void* data_context)
{
  printf("Read: %u\n", *(uint32_t *)buffer->data);
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack uint64_sub_callback(DynBuffer *buffer, void* data_context)
{
  printf("Read: %llu\n", (long long unsigned)*(uint64_t *)buffer->data);
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack uint8_sub_callback(DynBuffer *buffer, void* data_context)
{
  printf("Read: %u\n", *(uint8_t *)buffer->data);
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack float32_sub_callback(DynBuffer *buffer, void* data_context)
{
  printf("Read: %f\n", *(float *)buffer->data);
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack float64_sub_callback(DynBuffer *buffer, void* data_context)
{
  printf("Read: %f\n", *(double *)buffer->data);
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack callback_srv_add_two_ints(DynBuffer *request, DynBuffer *response, void* data_contex)
{
  const unsigned char* data = dynBufferGetData(request);

  int32_t values[2];
  values[0] = *(int32_t *)data;
  values[1] = *(int32_t *)(data + sizeof(int32_t));

  int32_t sum = values[0] + values[1];
  dynBufferPushBackInt32(response, sum);

  printf("Service add_two ints. Arguments: %d %d . Response %d \n", values[0], values[1], sum);
  fflush(stdout);

  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack counter_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  snprintf(message_buffer_counter, 100, "Increasing counter, now : %d",counter++);
  uint32_t len = (uint32_t)strlen(message_buffer_counter);
  dynBufferPushBackUInt32( buffer, len );
  dynBufferPushBackBuf(buffer, (unsigned char *)message_buffer_counter, len);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack clock_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  uint64_t cur_timer = cRosClockGetTimeMs();
  snprintf(message_buffer_clock, 100, "Time elapsed since start of node execution : %llu msec",
          (long long unsigned)(cur_timer - start_timer) );
  size_t len = strlen(message_buffer_clock);
  dynBufferPushBackUInt32( buffer, (uint32_t)len );
  dynBufferPushBackBuf(buffer, (unsigned char *)message_buffer_clock, len);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack double_vector_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  int i = 0;
  for( i = 0; i < DOUBLE_VECTOR_SIZE; i++ )
    message_buffer_double_vector[i] = (double)i;
  dynBufferPushBackUInt32( buffer, DOUBLE_VECTOR_SIZE );
  dynBufferPushBackBuf(buffer, (unsigned char *)message_buffer_clock, sizeof(message_buffer_double_vector));
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack doublevector_subscription_callback(DynBuffer *buffer, void* data_context)
{
  const unsigned char* data = dynBufferGetData(buffer);
  uint32_t count = *(uint32_t *)data;
  double *vector = (double *)malloc(count*sizeof(double));
  memcpy(vector, data+sizeof(uint32_t), count);
  printf("Vector: [");
  int first = 1;
  int it = 0;
  for (; it < count; it++)
  {
    if (first)
      first = 0;
    else
      printf(" ");

    printf("%f", vector[it]);
  }
  printf("]\n");
  fflush(stdout);
  free(vector);
  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack test_subscription_callback(DynBuffer *buffer, void* data_context)
{
  const unsigned char* data = dynBufferGetData(buffer);
  char string[100];
  memcpy(string, data+4, buffer->size - 4);
  string[buffer->size - 4] = '\0';
  printf("%s\n", string);
  fflush(stdout);
  return CROS_SUCCESS_ERR_PACK;
}

static char *xmlrpc_host;
static int xmlrpc_port;

void getSystemStateCallback(int callid, GetSystemStateResult *result, void *context)
{
  if (0)
    cRosNodeUnregisterSubscriber(node, 0);
  else
    cRosApiUnregisterSubscriber(node, 2);
}

void getTopicTypesCallback(int callid, GetTopicTypesResult *result, void *context)
{
  cRosApiGetSystemState(node, getSystemStateCallback, NULL);
}

void getPublishedTopicsCallback(int callid, GetPublishedTopicsResult *result, void *context)
{
  cRosApiGetTopicTypes(node, getTopicTypesCallback, NULL);
}

void getBusStatsCallback(int callid, GetBusStatsResult *result, void *context)
{
  cRosApiGetPublishedTopics(node, NULL, getPublishedTopicsCallback, NULL);
}

void getPidCallback(int callid, GetPidResult *result, void *context)
{
  printf("Pid: %d\n", result->server_process_pid);
  fflush(stdout);
  cRosApiGetBusStats(node, xmlrpc_host, xmlrpc_port, getBusStatsCallback, NULL);
}

static void getNodeStatusCallback(CrosNodeStatusUsr *status, void* context)
{
  if (status->xmlrpc_host == NULL)
  {
    printf("Subscriber unregistered\n");
    fflush(stdout);
  }
  else
  {
    xmlrpc_host = (char *)malloc(strlen(status->xmlrpc_host) + 1);
    strcpy(xmlrpc_host, status->xmlrpc_host);
    xmlrpc_port = status->xmlrpc_port;
    cRosApiGetPid(node, xmlrpc_host, xmlrpc_port, getPidCallback, NULL);
  }
}

static cRosErrCodePack point_sub_callback(DynBuffer *buffer, void* data_context)
{
  cRosMessage *msg_point;

  cRosMessageNewBuild("/home/nico/Desktop/test_msgs/geometry_msgs", "Point", &msg_point);
  size_t buffer_offset = 0;
  cRosMessageDeserialize(msg_point, buffer);
  cRosMessageFree(msg_point);

  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack manyStrings_sub_callback(DynBuffer *buffer, void* data_context)
{
  cRosMessage *manyStrings_msg;

  cRosMessageNewBuild("/home/nico/Desktop/test_msgs/gripping_robot", "StringMixes", &manyStrings_msg);
  size_t buffer_offset = 0;
  cRosMessageDeserialize(manyStrings_msg, buffer);
  cRosMessageFree(manyStrings_msg);

  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack manyPoints_sub_callback(DynBuffer *buffer, void* data_context)
{
  cRosMessage *manyPoints_msg;

  cRosMessageNewBuild("/home/nico/Desktop/test_msgs/gripping_robot", "PointMixes", &manyPoints_msg);
  size_t buffer_offset = 0;
  cRosMessageDeserialize(manyPoints_msg, buffer);
  cRosMessageFree(manyPoints_msg);

  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack point_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  cRosMessage *point_msg;

  cRosMessageNewBuild("/home/nico/Desktop/test_msgs/geometry_msgs", "Point", &point_msg);
  cRosMessageField* point_x = cRosMessageGetField(point_msg,"x");
  point_x->data.as_float64 = 1.5;
  cRosMessageField* point_y = cRosMessageGetField(point_msg,"y");
  point_y->data.as_float64 = 1.6;
  cRosMessageField* point_z = cRosMessageGetField(point_msg,"z");
  point_z->data.as_float64 = 1.57;
  cRosMessageSerialize(point_msg, buffer);
  cRosMessageFree(point_msg);

  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack manyStrings_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  cRosMessage* manyStrings_msg;

  cRosMessageNewBuild("/home/nico/Desktop/test_msgs/gripping_robot", "StringMixes", &manyStrings_msg);
  cRosMessageField* limitedStrings = cRosMessageGetField(manyStrings_msg,"limitedStrings");

  cRosMessageFieldArrayPushBackString(limitedStrings, "Ciao");
  cRosMessageFieldArrayPushBackString(limitedStrings, "amico");
  cRosMessageFieldArrayPushBackString(limitedStrings, "mio");

  //  cRosMessageField* unlimitedStrings = cRosMessageGetField(manyStrings_msg,"unlimitedStrings");
  //
  //  cRosMessageFieldArrayPushBackString(unlimitedStrings, "come");
  //  cRosMessageFieldArrayPushBackString(unlimitedStrings, "stai?");

  cRosMessageSerialize(manyStrings_msg, buffer);
  cRosMessageFree(manyStrings_msg);

  return CROS_SUCCESS_ERR_PACK;
}

static cRosErrCodePack manyPoints_pub_callback(DynBuffer *buffer, int non_period_msg, void* data_context)
{
  cRosMessage *manyPoints_msg;

  cRosMessageNewBuild("/home/nico/Desktop/test_msgs/gripping_robot", "PointMixes", &manyPoints_msg);

  cRosMessageField* limitedPoints = cRosMessageGetField(manyPoints_msg,"limitedPoints");

  cRosMessage* point_1_msg;
  cRosMessageNewBuild("/home/nico/Desktop/test_msgs/geometry_msgs", "Point", &point_1_msg);
  cRosMessageField* point_1x = cRosMessageGetField(point_1_msg,"x");
  point_1x->data.as_float64 = 1.5;
  cRosMessageField* point_1y = cRosMessageGetField(point_1_msg,"y");
  point_1y->data.as_float64 = 1.6;
  cRosMessageField* point_1z = cRosMessageGetField(point_1_msg,"z");
  point_1z->data.as_float64 = 1.57;

  cRosMessageFieldArrayPushBackMsg(limitedPoints, point_1_msg);
  cRosMessage* test = NULL;
  test = cRosMessageFieldArrayAtMsgGet(limitedPoints, 0);

  cRosMessage* point_2_msg;
  cRosMessageNewBuild("/home/nico/Desktop/test_msgs/geometry_msgs", "Point", &point_2_msg);
  cRosMessageField* point_2x = cRosMessageGetField(point_2_msg,"x");
  point_2x->data.as_float64 = 2.5;
  cRosMessageField* point_2y = cRosMessageGetField(point_2_msg,"y");
  point_2y->data.as_float64 = 2.6;
  cRosMessageField* point_2z = cRosMessageGetField(point_2_msg,"z");
  point_2z->data.as_float64 = 2.57;

  cRosMessage* point_3_msg;
  cRosMessageNewBuild("/home/nico/Desktop/test_msgs/geometry_msgs", "Point", &point_3_msg);
  cRosMessageField* point_3x = cRosMessageGetField(point_3_msg,"x");
  point_3x->data.as_float64 = 1.5;
  cRosMessageField* point_3y = cRosMessageGetField(point_3_msg,"y");
  point_3y->data.as_float64 = 1.6;
  cRosMessageField* point_3z = cRosMessageGetField(point_3_msg,"z");
  point_3z->data.as_float64 = 1.57;

  cRosMessageFieldArrayPushBackMsg(limitedPoints, point_2_msg);
  cRosMessageFieldArrayPushBackMsg(limitedPoints, point_3_msg);

  cRosMessageSerialize(manyPoints_msg, buffer);
  cRosMessageFree(manyPoints_msg);

  return CROS_SUCCESS_ERR_PACK;
}

int main(int argc, char **argv)
{
  char *default_node_name = "/test_node",
       *node_name = default_node_name;
  char *default_host = "127.0.0.1",
       *node_host = default_host,
       *roscore_host = default_host;
  unsigned short roscore_port = 11311;

  if(argc == 2)
  {
    printHelp( argv[0] );
    return EXIT_SUCCESS;
  }

  srand (time(NULL));

  int i = 1;
  for( ; i < argc - 1; i+=2)
  {
    if( strcmp(argv[i],"-name") == 0)
      node_name = argv[i+1];
    else if( strcmp(argv[i],"-host") == 0)
      node_host = argv[i+1];
    else if( strcmp(argv[i],"-chost") == 0)
      roscore_host = argv[i+1];
    else if( strcmp(argv[i],"-cport") == 0)
    {
      int i_port = atoi(argv[i+1]);
      if( i_port < 0 || i_port > USHRT_MAX )
      {
        fprintf(stderr,"Invalid port %d\n",i_port);
        exit(EXIT_FAILURE);
      }
      roscore_port = (unsigned short)i_port;
    }
    else
    {
      fprintf(stderr,"Invalid option %s\n",argv[i]);
      exit(EXIT_FAILURE);
    }
  }

  printf("Running node \"%s\" with host : %s, roscore host : %s and roscore port : %d\n",
          node_name, node_host, roscore_host, roscore_port );
  printf("To set a different node/host/port, take a look at the options: ");
  printf("%s -h\n", argv[0]);

  char path[1024];
  getcwd(path, sizeof(path));
  strncat(path, "/rosdb", sizeof(path));
  node = cRosNodeCreate( node_name, node_host, roscore_host, roscore_port, path, NULL);

  int rc;
  cRosErrCodePack err_cod;

#ifndef CROS_MESSAGE_SERIALIZATION_TEST
  rc = cRosNodeRegisterPublisher ( node, "string data\n\n", "/topic_clock", "std_msgs/String",
                                   "992ce8a1687cec8c8bd883ec73ca41d1", 1000, clock_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, "float64[] val\n\n", "/double_vector", "cros_testbed/DoubleVector",
                                   "65ac3f59e35977c61c27adccf4c68288", 1000, double_vector_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, "string data\n\n", "/topic_counter", "std_msgs/String",
                                   "992ce8a1687cec8c8bd883ec73ca41d1", 500, counter_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber(node, "string data\n\n", "/test", "std_msgs/String",
                                  "992ce8a1687cec8c8bd883ec73ca41d1", test_subscription_callback, getNodeStatusCallback, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

  char *gripperjointstatus_def = "uint32 PowerStatus\n"
                                 "uint32[4] ClampStatus\n"
                                 "uint32[] AlarmCodes\n"
                                 "uint32 Configuration\n\n";

  rc = cRosNodeRegisterSubscriber(node, gripperjointstatus_def, "/gripperstatus", "gripping_robot/GripperStatus",
                                  "bba69b1f07d275244b4a697ef69e1b2e", gripperstatus_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

  err_cod = cRosApiRegisterSubscriber(node, "/gripperjoints", "gripping_robot/GripperJoints",
                                 gripperjointstate_callbacknewapi, getNodeStatusCallback, NULL, 0, NULL);
  if (err_cod != CROS_SUCCESS_ERR_PACK)
    return EXIT_FAILURE;

/*
  rc = cRosNodeRegisterSubscriber(node, "float64[9] Position\n\n", "/gripperjoints", "gripping_robot/GripperJoints",
                                  "8351d4f138c16ac67e83ce06ead5a2f3", gripperjointstate_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber(node, "float64[] val\n\n", "/double_vector", "cros_testbed/DoubleVector",
                                  "65ac3f59e35977c61c27adccf4c68288", doublevector_subscription_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;
*/

  rc = cRosNodeRegisterServiceProvider(node,"/add_two_ints","cros_testbed/add_two_ints",
                                       "f0b6d69ea10b0cf210cb349d58d59e8f", callback_srv_add_two_ints, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

#else
  cRosService service;
  cRosServiceInit(&service);
  cRosServiceBuild(&service,"/home/nico/Desktop/test_msgs/nav_msgs/GetPlan.srv");


  cRosService serviceClamps;
  cRosServiceInit(&serviceClamps);
  cRosServiceBuild(&serviceClamps,"/home/nico/Desktop/test_msgs/gripping_robot/CloseGripper.srv");

  cRosMessage msgPoint;

  cRosMessageInit(&msgPoint);
  cRosMessageBuild(&msgPoint,"/home/nico/Desktop/test_msgs/geometry_msgs/Point.msg");
  cRosMessage msgStringMixes;

  cRosMessageInit(&msgStringMixes);
  cRosMessageBuild(&msgStringMixes,"/home/nico/Desktop/test_msgs/gripping_robot/StringMixes.msg");

  cRosMessage msgPointMixes;

  cRosMessageInit(&msgPointMixes);
  cRosMessageBuild(&msgPointMixes,"/home/nico/Desktop/test_msgs/gripping_robot/PointMixes.msg");

  rc = cRosNodeRegisterPublisher ( node, msgPoint.msgDef->plain_text, "/point_test", "geometry_msgs/Point",
                                   msgPoint.md5sum, 1000, point_pub_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;


  rc = cRosNodeRegisterPublisher ( node, msgPointMixes.msgDef->plain_text, "/many_points", "gripping_robot/PointMixes",
                                    msgPointMixes.md5sum, 1000, manyPoints_pub_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;


  rc = cRosNodeRegisterPublisher ( node, msgStringMixes.msgDef->plain_text, "/many_strings", "gripping_robot/StringMixes",
                                   msgStringMixes.md5sum, 1000, manyStrings_pub_callback, NULL, NULL);
#endif

#ifdef STD_MSGS_TEST
  char *header_def = "# Standard metadata for higher-level stamped data types.\n"
                     "# This is generally used to communicate timestamped data \n"
                     "# in a particular coordinate frame.\n"
                     "# \n"
                     "# sequence ID: consecutively increasing ID \n"
                     "uint32 seq\n"
                     "#Two-integer timestamp that is expressed as:\n"
                     "# * stamp.secs: seconds (stamp_secs) since epoch\n"
                     "# * stamp.nsecs: nanoseconds since stamp_secs\n"
                     "# time-handling sugar is provided by the client library\n"
                     "time stamp\n"
                     "#Frame this data is associated with\n"
                     "# 0: no frame\n"
                     "# 1: global frame\n"
                     "string frame_id\n\n";

  rc = cRosNodeRegisterPublisher ( node, "bool data\n\n", "/bool", "std_msgs/Bool",
                                   "8b94c1b53db61fb6aed406028ad6332a", 1000, bool_pub_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, "byte data\n\n", "/byte", "std_msgs/Byte",
                                   "ad736a2e8818154c487bb80fe42ce43b", 1000, byte_pub_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, "char data\n\n", "/char", "std_msgs/Char",
                                   "1bf77f25acecdedba0e224b162199717", 1000, char_pub_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, "duration data\n\n", "/duration", "std_msgs/Duration",
                                   "3e286caf4241d664e55f3ad380e2ae46", 1000, duration_pub_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, header_def, "/header", "std_msgs/Header",
                                   "2176decaecbce78abc3b96ef049fabed", 1000, header_pub_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, "int16 data\n\n", "/int16", "std_msgs/Int16",
                                   "8524586e34fbd7cb1c08c5f5f1ca0e57", 1000, int16_pub_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, "int32 data\n\n", "/int32", "std_msgs/Int32",
                                   "da5909fbe378aeaf85e547e830cc1bb7", 1000, int32_pub_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, "int64 data\n\n", "/int64", "std_msgs/Int64",
                                   "34add168574510e6e17f5d23ecc077ef", 1000, int64_pub_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, "int8 data\n\n", "/int8", "std_msgs/Int8",
                                   "27ffa0c9c4b8fb8492252bcad9e5c57b", 1000, int8_pub_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, "time data\n\n", "/time", "std_msgs/Time",
                                   "cd7166c74c552c311fbcc2fe5a7bc289", 1000, time_pub_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, "uint16 data\n\n", "/uint16", "std_msgs/UInt16",
                                   "1df79edf208b629fe6b81923a544552d", 1000, uint16_pub_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, "uint32 data\n\n", "/uint32", "std_msgs/UInt32",
                                   "304a39449588c7f8ce2df6e8001c5fce", 1000, uint32_pub_callback, NULL, NULL);
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, "uint64 data\n\n", "/uint64", "std_msgs/UInt64",
                                   "1b2a79973e8bf53d7b53acb71299cb57", 1000, uint64_pub_callback, NULL, NULL);
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, "uint8 data\n\n", "/uint8", "std_msgs/UInt8",
                                   "7c8164229e7d2c17eb95e9231617fdee", 1000, uint8_pub_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;


  rc = cRosNodeRegisterPublisher ( node, "float32 data\n\n", "/float32", "std_msgs/Float32",
                                   "73fcbf46b49191e672908e50842a83d4", 1000, float32_pub_callback, NULL, NULL);
    return EXIT_FAILURE;

  rc = cRosNodeRegisterPublisher ( node, "float64 data\n\n", "/float64", "std_msgs/Float64",
                                   "fdb28210bfa9d7c91146260178d9a584", 1000, float64_pub_callback, NULL, NULL);
  if (rc == -1)
    return EXIT_FAILURE;

  // STD_MSGS Subscribers
  rc = cRosNodeRegisterSubscriber ( node, "bool data\n\n", "/bool", "std_msgs/Bool",
                                   "8b94c1b53db61fb6aed406028ad6332a", bool_sub_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber ( node, "byte data\n\n", "/byte", "std_msgs/Byte",
                                   "ad736a2e8818154c487bb80fe42ce43b", byte_sub_callback, NULL, NULL, 0);
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber ( node, "char data\n\n", "/char", "std_msgs/Char",
                                   "1bf77f25acecdedba0e224b162199717", char_sub_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber ( node, "duration data\n\n", "/duration", "std_msgs/Duration",
                                   "3e286caf4241d664e55f3ad380e2ae46", duration_sub_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber ( node, header_def, "/header", "std_msgs/Header",
                                   "2176decaecbce78abc3b96ef049fabed", header_sub_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber ( node, "int16 data\n\n", "/int16", "std_msgs/Int16",
                                   "8524586e34fbd7cb1c08c5f5f1ca0e57", int16_sub_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber ( node, "int32 data\n\n", "/int32", "std_msgs/Int32",
                                   "da5909fbe378aeaf85e547e830cc1bb7", int32_sub_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber ( node, "int64 data\n\n", "/int64", "std_msgs/Int64",
                                   "34add168574510e6e17f5d23ecc077ef", int64_sub_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber ( node, "int8 data\n\n", "/int8", "std_msgs/Int8",
                                   "27ffa0c9c4b8fb8492252bcad9e5c57b", int8_sub_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber ( node, "time data\n\n", "/time", "std_msgs/Time",
                                   "cd7166c74c552c311fbcc2fe5a7bc289", time_sub_callback, NULL, NULL, 0);
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber ( node, "uint16 data\n\n", "/uint16", "std_msgs/UInt16",
                                   "1df79edf208b629fe6b81923a544552d", uint16_sub_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber ( node, "uint32 data\n\n", "/uint32", "std_msgs/UInt32",
                                   "304a39449588c7f8ce2df6e8001c5fce", uint32_sub_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;
  rc = cRosNodeRegisterSubscriber ( node, "uint64 data\n\n", "/uint64", "std_msgs/UInt64",
                                   "1b2a79973e8bf53d7b53acb71299cb57", uint64_sub_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber ( node, "uint8 data\n\n", "/uint8", "std_msgs/UInt8",
                                   "7c8164229e7d2c17eb95e9231617fdee", uint8_sub_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber ( node, "float32 data\n\n", "/float32", "std_msgs/Float32",
                                   "73fcbf46b49191e672908e50842a83d4", float32_sub_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

  rc = cRosNodeRegisterSubscriber ( node, "float64 data\n\n", "/float64", "std_msgs/Float64",
                                   "fdb28210bfa9d7c91146260178d9a584", float64_sub_callback, NULL, NULL, 0);
  if (rc == -1)
    return EXIT_FAILURE;

#endif // STD_MSGS_TEST

  unsigned char exit = 0;
  cRosNodeStart( node, &exit );
  cRosNodeDestroy( node );

  return EXIT_SUCCESS;
}

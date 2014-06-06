#ifndef _CROS_API_CALL_H_
#define _CROS_API_CALL_H_

#include "xmlrpc_params_vector.h"

#define CROS_API_TCPROS_STRING "TCPROS"

typedef void (*ResultCallback)(void *result, void *data);
typedef void * (*FetchResultCallback)(XmlrpcParamVector *response);

typedef struct RosApiCall RosApiCall;
typedef struct ApiCallNode ApiCallNode;
typedef struct ApiCallQueue ApiCallQueue;

typedef enum
{
  CROS_API_NONE = 0,
  CROS_API_REGISTER_SERVICE,
  CROS_API_UNREGISTER_SERVICE,
  CROS_API_UNREGISTER_PUBLISHER,
  CROS_API_UNREGISTER_SUBSCRIBER,
  CROS_API_REGISTER_PUBLISHER,
  CROS_API_REGISTER_SUBSCRIBER,
  CROS_API_LOOKUP_NODE,
  CROS_API_GET_PUBLISHED_TOPICS,
  CROS_API_GET_TOPIC_TYPES,
  CROS_API_GET_SYSTEM_STATE,
  CROS_API_GET_URI,
  CROS_API_LOOKUP_SERVICE,
  CROS_API_GET_BUS_STATS,
  CROS_API_GET_BUS_INFO,
  CROS_API_GET_MASTER_URI,
  CROS_API_SHUTDOWN,
  CROS_API_GET_PID,
  CROS_API_GET_SUBSCRIPTIONS,
  CROS_API_GET_PUBLICATIONS,
  CROS_API_PARAM_UPDATE,
  CROS_API_PUBLISHER_UPDATE,
  CROS_API_REQUEST_TOPIC
} CROS_API_METHOD;

struct RosApiCall
{
  CROS_API_METHOD method;                     //! ROS api method
  XmlrpcParamVector params;                   //! Method arguments
  int provider_idx;                           //! Provider (sub, pub or service provider) index
  ResultCallback result_callback;             //! Response callback
  void *context_data;                         //! Result callback context
  FetchResultCallback fetch_result_callback;  //! Callback to fetch the result
};

struct ApiCallNode
{
  RosApiCall *call;
  ApiCallNode* next;
};

struct ApiCallQueue
{
  ApiCallNode* head;
  ApiCallNode* tail;
  size_t count;
};

RosApiCall * newRosApiCall();
void freeRosApiCall(RosApiCall *call);

void initApiCallQueue(ApiCallQueue *queue);
void enqueueApiCall(ApiCallQueue *queue, RosApiCall* apiCall);
RosApiCall * dequeueApiCall(ApiCallQueue *queue);
void releaseApiCallQueue(ApiCallQueue *queue);
size_t getQueueCount(ApiCallQueue *queue);

const char * getMethodName(CROS_API_METHOD method);
CROS_API_METHOD getMethodCode(const char *method);

#endif // _CROS_API_CALL_H_
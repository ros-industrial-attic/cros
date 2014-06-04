#ifndef _CROS_API_CALL_H_
#define _CROS_API_CALL_H_

#include "xmlrpc_params_vector.h"

#define CROS_API_TCPROS_STRING "TCPROS"

typedef int (*ApiCallback)();
typedef int (*DeserializeResponseCallback)();

typedef struct RosApiCall RosApiCall;
typedef struct ApiCallNode ApiCallNode;
typedef struct ApiCallQueue ApiCallQueue;

typedef enum
{
  CROS_API_NONE = 0,
  CROS_API_GET_PID,
  CROS_API_GET_PUBLISHED_TOPICS,
  CROS_API_REGISTER_PUBLISHER,
  CROS_API_REGISTER_SUBSCRIBER,
  CROS_API_REGISTER_SERVICE,
  CROS_API_REQUEST_TOPIC,
  CROS_API_UNREGISTER_PUBLISHER,
  CROS_API_UNREGISTER_SUBSCRIBER,
  CROS_API_UNREGISTER_SERVICE,
  CROS_API_PUBLISHER_UPDATE,
  CROS_API_GET_SUBSCRIPTIONS,
  CROS_API_GET_PUBLICATIONS
} CROS_API_METHOD;

struct RosApiCall
{
  CROS_API_METHOD method;
  XmlrpcParamVector params;
  int provider_idx;
  ApiCallback Callback;
  void *ContextData;
  DeserializeResponseCallback DeserializeResponse;
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
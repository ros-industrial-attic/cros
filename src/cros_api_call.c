#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "cros_api_call.h"

RosApiCall * newRosApiCall()
{
  RosApiCall *ret = (RosApiCall *)malloc(sizeof(RosApiCall));
  ret->provider_idx = -1;
  xmlrpcParamVectorInit(&ret->params);
  ret->result_callback = NULL;
  ret->context_data = NULL;
  ret->fetch_result_callback = NULL;
  return ret;
}

void freeRosApiCall(RosApiCall *call)
{
  xmlrpcParamVectorRelease(&call->params);
  free(call);
}

void initApiCallQueue(ApiCallQueue *queue)
{
  queue->tail = NULL;
  queue->head = NULL;
  queue->count = 0;
}

void enqueueApiCall(ApiCallQueue *queue, RosApiCall* apiCall)
{
  ApiCallNode* node = malloc(sizeof(ApiCallNode));

  if(node == NULL)
    exit(1);

  node->call = apiCall;
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
}

RosApiCall * dequeueApiCall(ApiCallQueue *queue)
{
  ApiCallNode* head = queue->head;
  queue->head = head->next;
  if(queue->head == NULL)
    queue->tail = queue->head;

  RosApiCall *call = head->call;
  free(head);

  queue->count--;

  return call;
}

void releaseApiCallQueue(ApiCallQueue *queue)
{
  ApiCallNode *current = queue->head;
  while(current != NULL)
  {
    ApiCallNode *next = current->next;
    free(current);
    current = next;
  }

  initApiCallQueue(queue);
}

size_t getQueueCount(ApiCallQueue *queue)
{
  return queue->count;
}

const char * getMethodName(CROS_API_METHOD method)
{
  switch (method)
  {
    case CROS_API_NONE:
      return NULL;
    case CROS_API_REGISTER_SERVICE:
      return "registerService";
    case CROS_API_UNREGISTER_SERVICE:
      return "unregisterService";
    case CROS_API_REGISTER_SUBSCRIBER:
      return "registerSubscriber";
    case CROS_API_UNREGISTER_SUBSCRIBER:
      return "unregisterSubscriber";
    case CROS_API_REGISTER_PUBLISHER:
      return "registerPublisher";
    case CROS_API_UNREGISTER_PUBLISHER:
      return "unregisterPublisher";
    case CROS_API_LOOKUP_NODE:
      return "lookupNode";
    case CROS_API_GET_PUBLISHED_TOPICS:
      return "getPublishedTopics";
    case CROS_API_GET_TOPIC_TYPES:
      return "getTopicTypes";
    case CROS_API_GET_SYSTEM_STATE:
      return "getSystemState";
    case CROS_API_GET_URI:
      return "getUri";
    case CROS_API_LOOKUP_SERVICE:
      return "lookupService";
    case CROS_API_GET_BUS_STATS:
      return "getBusStats";
    case CROS_API_GET_BUS_INFO:
      return "getBusInfo";
    case CROS_API_GET_MASTER_URI:
      return "getMasterUri";
    case CROS_API_SHUTDOWN:
      return "shutdown";
    case CROS_API_GET_PID:
      return "getPid";
    case CROS_API_GET_SUBSCRIPTIONS:
      return "getSubscriptions";
    case CROS_API_GET_PUBLICATIONS:
      return "getPublications";
    case CROS_API_PARAM_UPDATE:
      return "paramUpdate";
    case CROS_API_PUBLISHER_UPDATE:
      return "publisherUpdate";
    case CROS_API_REQUEST_TOPIC:
      return "requestTopic";
    default:
      assert(0);
  }
}

CROS_API_METHOD getMethodCode(const char *method)
{
  if (strcmp(method, "registerService") == 0)
    return CROS_API_REGISTER_SERVICE;
  else if (strcmp(method, "unregisterService") == 0)
    return CROS_API_UNREGISTER_SERVICE;
  else if (strcmp(method, "registerSubscriber") == 0)
    return CROS_API_REGISTER_SUBSCRIBER;
  else if (strcmp(method, "unregisterSubscriber") == 0)
    return CROS_API_UNREGISTER_SUBSCRIBER;
    else if (strcmp(method, "registerPublisher") == 0)
    return CROS_API_REGISTER_PUBLISHER;
  else if (strcmp(method, "unregisterPublisher") == 0)
    return CROS_API_UNREGISTER_PUBLISHER;
  else if (strcmp(method, "lookupNode") == 0)
    return CROS_API_LOOKUP_NODE;
  else if (strcmp(method, "getPublishedTopics") == 0)
    return CROS_API_GET_PUBLISHED_TOPICS;
  else if (strcmp(method, "getTopicTypes") == 0)
    return CROS_API_GET_TOPIC_TYPES;
  else if (strcmp(method, "getSystemState") == 0)
    return CROS_API_GET_SYSTEM_STATE;
  else if (strcmp(method, "getUri") == 0)
    return CROS_API_GET_URI;
  else if (strcmp(method, "lookupService") == 0)
    return CROS_API_LOOKUP_SERVICE;
  else if (strcmp(method, "getBusStats") == 0)
    return CROS_API_GET_BUS_STATS;
  else if (strcmp(method, "getBusInfo") == 0)
    return CROS_API_GET_BUS_INFO;
  else if (strcmp(method, "getMasterUri") == 0)
    return CROS_API_GET_MASTER_URI;
  else if (strcmp(method, "shutdown") == 0)
    return CROS_API_SHUTDOWN;
  else if (strcmp(method, "getPid") == 0)
    return CROS_API_GET_PID;
  else if (strcmp(method, "getSubscriptions") == 0)
    return CROS_API_GET_SUBSCRIPTIONS;
  else if (strcmp(method, "getPublications") == 0)
    return CROS_API_GET_PUBLICATIONS;
  else if (strcmp(method, "paramUpdate") == 0)
    return CROS_API_PARAM_UPDATE;
  else if (strcmp(method, "publisherUpdate") == 0)
    return CROS_API_PUBLISHER_UPDATE;
  else if (strcmp(method, "requestTopic") == 0)
    return CROS_API_REQUEST_TOPIC;
  else
    return CROS_API_NONE;
}

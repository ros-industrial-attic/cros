#include <stdlib.h>
#include <string.h>

#include "cros_api_call.h"
#include "cros_defs.h"

RosApiCall * newRosApiCall(void)
{
  RosApiCall *ret = (RosApiCall *)malloc(sizeof(RosApiCall));
  ret->id = -1;
  ret->user_call = 0;
  ret->provider_idx = -1;
  ret->host = NULL;
  ret->port = -1;
  xmlrpcParamVectorInit(&ret->params);
  ret->result_callback = NULL;
  ret->context_data = NULL;
  ret->fetch_result_callback = NULL;
  ret->free_result_callback = NULL;
  return ret;
}

void freeRosApiCall(RosApiCall *call)
{
  xmlrpcParamVectorRelease(&call->params);
  if (call->host != NULL)
    free(call->host);
  free(call);
}

void initApiCallQueue(ApiCallQueue *queue)
{
  queue->tail = NULL;
  queue->head = NULL;
  queue->count = 0;
}

RosApiCall * peekApiCallQueue(ApiCallQueue *queue)
{
  if (queue->head == NULL)
    return NULL;

  return queue->head->call;
}

int enqueueApiCall(ApiCallQueue *queue, RosApiCall* apiCall)
{
  ApiCallNode *node = (ApiCallNode *)malloc(sizeof(ApiCallNode));

  if(node == NULL)
  {
    PRINT_ERROR("enqueueApiCall() : Can't enqueue call\n");
    return -1;
  }

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

  return 0;
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
    freeRosApiCall(current->call);
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

int isQueueEmpty(ApiCallQueue *queue)
{
  return queue->count == 0;
}

#include <stdlib.h>

#include "cros_defs.h"
#include "cros_node_api.h"
#include "cros_node_internal.h"
#include "xmlrpc_process.h"

static LookupNodeResult * fetchLookupNodeResult(XmlrpcParamVector *response);
static GetPublishedTopicsResult * fetchGetPublishedTopicsResult(XmlrpcParamVector *response);
static GetTopicTypesResult * fetchGetTopicTypesResult(XmlrpcParamVector *response);
static GetSystemStateResult * fetchGetSystemStateResult(XmlrpcParamVector *response);
static GetUriResult * fetchGetUriResult(XmlrpcParamVector *response);
static LookupServiceResult * fetchLookupServiceResult(XmlrpcParamVector *response);
static GetBusStatsResult * fetchGetBusStatsResult(XmlrpcParamVector *response);
static GetBusInfoResult * fetchGetBusInfoResult(XmlrpcParamVector *response);
static GetMasterUriResult * fetchGetMasterUriResult(XmlrpcParamVector *response);
static ShutdownResult * fetchShutdownResult(XmlrpcParamVector *response);
static GetPidResult * fetchGetPidResult(XmlrpcParamVector *response);
static GetSubscriptionsResult * fetchGetSubscriptionsResult(XmlrpcParamVector *response);
static GetPublicationsResult * fetchGetPublicationsResult(XmlrpcParamVector *response);

static void freeLookupNodeResult(LookupNodeResult *result);
static void freeGetPublishedTopicsResult(GetPublishedTopicsResult *result);
static void freeGetTopicTypesResult(GetTopicTypesResult *result);
static void freeGetSystemStateResult(GetSystemStateResult *result);
static void freeGetUriResult(GetUriResult *result);
static void freeLookupServiceResult(LookupServiceResult *result);
static void freeGetBusStatsResult(GetBusStatsResult *result);
static void freeGetBusInfoResult(GetBusInfoResult *result);
static void freeGetMasterUriResult(GetMasterUriResult *result);
static void freeShutdownResult(ShutdownResult *result);
static void freeGetPidResult(GetPidResult *result);
static void freeGetSubscriptionsResult(GetSubscriptionsResult *result);
static void freeGetPublicationsResult(GetPublicationsResult *result);

typedef enum NodeType
{
  CROS_SUBSCRIBER,
  CROS_PUBLISHER,
  CROS_SERVICE_PROVIDER
} NodeType;

typedef struct NodeContext
{
  NodeType type;
  CrosMessage incoming;
  CrosMessage outgoing;
  void *api_callback;
  SlaveStatusCallback slave_callback;
  void *context;
  char *message_definition;
  char *md5sum;
} NodeContext;

static NodeContext * newProviderContext(const char *provider_type, NodeType type)
{
  // TODO provider_type: nome qualificato con package di topic o servizio. Popolare message_definition, md5sum
  // TODO inizializzare incoming e/o outgoing per consentire set/get in base al message/service type
  NodeContext *providerContext = (NodeContext *)malloc(sizeof(NodeContext));
  providerContext->api_callback = NULL;
  providerContext->slave_callback = NULL;
  return providerContext;
}

static void freeProviderContext(NodeContext *context)
{
  // TODO
  free(context);
}

static void deserializeRosMessage(CrosMessage *message, DynBuffer *buffer)
{
  // TODO
}

static CallbackResponse cRosNodeSubscriberCallback(DynBuffer *buffer, void* context_)
{
  NodeContext *context = (NodeContext *)context_;
  deserializeRosMessage(&context->incoming, buffer);

  // Cast to the appropriate public api callback and invoke it on the user context
  SubscriberApiCallback subscriberApiCallback = (SubscriberApiCallback)context->api_callback;
  return subscriberApiCallback(&context->incoming, context->context);
}

static void cRosNodeSlaveCallback(CrosSlaveStatus *status, void* context_)
{
  NodeContext *context = (NodeContext *)context_;
  context->slave_callback(status, context->context);
}

int cRosApiRegisterService(CrosNode *node, const char *service_name, const char *service_type, ServiceProviderApiCallback callback, void *context)
{
  return 0;
}

int cRosApisUnegisterService(CrosNode *node, int svcidx)
{
  return 0;
}

int cRosApiRegisterSubscriber(CrosNode *node, const char *topic_name, const char *topic_type, SubscriberApiCallback callback, SlaveStatusCallback slave_callback, void *context)
{
  NodeContext *nodeContext = newProviderContext(topic_type, CROS_SUBSCRIBER);
  nodeContext->api_callback = callback;
  nodeContext->slave_callback = slave_callback;
  nodeContext->context = context;

  // NB: Pass the private NodeContext to the private api, not the user context
  int rc = cRosNodeRegisterSubscriber(node, nodeContext->message_definition, topic_name, topic_type,
                                  nodeContext->md5sum, cRosNodeSubscriberCallback,
                                  slave_callback == NULL ? NULL : cRosNodeSlaveCallback, nodeContext);
  return rc;
}

int cRosApiUnregisterSubscriber(CrosNode *node, int subidx)
{
  SubscriberNode *sub = &node->subs[subidx];
  NodeContext *context = (NodeContext *)sub->context;
  int rc = cRosNodeUnregisterSubscriber(node, subidx);
  if (rc != -1)
    freeProviderContext(context);

  return rc;
}

int cRosApiRegisterPublisher(CrosNode *node, const char *topic_name, const char *topic_type, PublisherApiCallback callback, SlaveStatusCallback slave_callback, void *context)
{
  return 0;
}

int cRosApiUnregisterPublisher(CrosNode *node, int pubidx)
{
  return 0;
}

int cRosApicRosApiLookupNode(CrosNode *node, const char *node_name, LookupNodeCallback *callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_LOOKUP_NODE;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchLookupNodeResult;
  call->free_result_callback = (FreeResultCallback)freeLookupNodeResult;

  return enqueueMasterApiCall(node, call);
}

int cRosApiGetPublishedTopics(CrosNode *node, const char *subgraph, GetPublishedTopicsCallback *callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_PUBLISHED_TOPICS;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetPublishedTopicsResult;
  call->free_result_callback = (FreeResultCallback)freeGetPublishedTopicsResult;

  return enqueueMasterApiCall(node, call);
}

int cRosApiGetTopicTypes(CrosNode *node, GetTopicTypesCallback *callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_TOPIC_TYPES;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetTopicTypesResult;
  call->free_result_callback = (FreeResultCallback)freeGetTopicTypesResult;

  return enqueueMasterApiCall(node, call);
}

int cRosApiGetSystemState(CrosNode *node, GetSystemStateCallback *callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_SYSTEM_STATE;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetSystemStateResult;
  call->free_result_callback = (FreeResultCallback)freeGetSystemStateResult;

  return enqueueMasterApiCall(node, call);
}

int cRosApiGetUri(CrosNode *node, GetUriCallback *callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_URI;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetUriResult;
  call->free_result_callback = (FreeResultCallback)freeGetUriResult;

  return enqueueMasterApiCall(node, call);
}

int cRosApiLookupService(CrosNode *node, const char *service, LookupServiceCallback *callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_LOOKUP_SERVICE;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchLookupServiceResult;
  call->free_result_callback = (FreeResultCallback)freeLookupServiceResult;

  return enqueueMasterApiCall(node, call);
}

int cRosApiGetBusStats(CrosNode *node, int *subidx, const char* host, int port,
                       GetBusStatsCallback *callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_BUS_STATS;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetBusStatsResult;
  call->free_result_callback = (FreeResultCallback)freeGetBusStatsResult;

  return enqueueSlaveApiCall(node, call);
}

int cRosApiGetBusInfo(CrosNode *node, int *subidx, const char* host, int port,
                      GetBusInfoCallback *callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_BUS_INFO;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetBusInfoResult;
  call->free_result_callback = (FreeResultCallback)freeGetBusInfoResult;

  return enqueueSlaveApiCall(node, call);
}

int cRosApiGetMasterUri(CrosNode *node, int *subidx, const char* host, int port,
                        GetMasterUriCallback *callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_MASTER_URI;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetMasterUriResult;
  call->free_result_callback = (FreeResultCallback)freeGetMasterUriResult;

  return enqueueSlaveApiCall(node, call);
}

int cRosApiShutdown(CrosNode *node, int *subidx, const char* host, int port, const char *msg,
                    GetMasterUriCallback *callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_SHUTDOWN;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchShutdownResult;
  call->free_result_callback = (FreeResultCallback)freeShutdownResult;

  return enqueueSlaveApiCall(node, call);
}

int cRosApiGetPid(CrosNode *node, int *subidx, const char* host, int port,
                  GetPidCallback *callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_PID;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetPidResult;
  call->free_result_callback = (FreeResultCallback)freeGetPidResult;

  return enqueueSlaveApiCall(node, call);
}

int cRosApiGetSubscriptions(CrosNode *node, int *subidx, const char* host, int port,
                            GetSubscriptionsCallback *callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_SUBSCRIPTIONS;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetSubscriptionsResult;
  call->free_result_callback = (FreeResultCallback)freeGetSubscriptionsResult;

  return enqueueSlaveApiCall(node, call);
}

int cRosApiGetPublications(CrosNode *node, int *subidx, const char* host, int port,
                    GetSubscriptionsCallback *callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_PUBLICATIONS;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetPublicationsResult;
  call->free_result_callback = (FreeResultCallback)freeGetPublicationsResult;

  return enqueueSlaveApiCall(node, call);
}

LookupNodeResult * fetchLookupNodeResult(XmlrpcParamVector *response)
{
  return NULL;
}

GetPublishedTopicsResult * fetchGetPublishedTopicsResult(XmlrpcParamVector *response)
{
  return NULL;
}

GetTopicTypesResult * fetchGetTopicTypesResult(XmlrpcParamVector *response)
{
  return NULL;
}

GetSystemStateResult * fetchGetSystemStateResult(XmlrpcParamVector *response)
{
  return NULL;
}

GetUriResult * fetchGetUriResult(XmlrpcParamVector *response)
{
  return NULL;
}

LookupServiceResult * fetchLookupServiceResult(XmlrpcParamVector *response)
{
  return NULL;
}

GetBusStatsResult * fetchGetBusStatsResult(XmlrpcParamVector *response)
{
  return NULL;
}

GetBusInfoResult * fetchGetBusInfoResult(XmlrpcParamVector *response)
{
  return NULL;
}

GetMasterUriResult * fetchGetMasterUriResult(XmlrpcParamVector *response)
{
  return NULL;
}

ShutdownResult * fetchShutdownResult(XmlrpcParamVector *response)
{
  return NULL;
}

GetPidResult * fetchGetPidResult(XmlrpcParamVector *response)
{
  return NULL;
}

GetSubscriptionsResult * fetchGetSubscriptionsResult(XmlrpcParamVector *response)
{
  return NULL;
}

GetPublicationsResult * fetchGetPublicationsResult(XmlrpcParamVector *response)
{
  return NULL;
}

void freeLookupNodeResult(LookupNodeResult *result)
{

}

void freeGetPublishedTopicsResult(GetPublishedTopicsResult *result)
{

}

void freeGetTopicTypesResult(GetTopicTypesResult *result)
{

}

void freeGetSystemStateResult(GetSystemStateResult *result)
{

}

void freeGetUriResult(GetUriResult *result)
{

}

void freeLookupServiceResult(LookupServiceResult *result)
{

}

void freeGetBusStatsResult(GetBusStatsResult *result)
{

}

void freeGetBusInfoResult(GetBusInfoResult *result)
{

}

void freeGetMasterUriResult(GetMasterUriResult *result)
{

}

void freeShutdownResult(ShutdownResult *result)
{

}

void freeGetPidResult(GetPidResult *result)
{

}

void freeGetSubscriptionsResult(GetSubscriptionsResult *result)
{

}

void freeGetPublicationsResult(GetPublicationsResult *result)
{

}

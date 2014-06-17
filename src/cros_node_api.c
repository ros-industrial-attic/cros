#include <stdlib.h>

#include "cros_defs.h"
#include "cros_node_api.h"
#include "cros_node_internal.h"
#include "xmlrpc_process.h"

static void * fetchLookupNodeResult(XmlrpcParamVector *response);
static void * fetchGetPublishedTopicsResult(XmlrpcParamVector *response);
static void * fetchGetTopicTypesResult(XmlrpcParamVector *response);
static void * fetchGetSystemStateResult(XmlrpcParamVector *response);
static void * fetchGetUriResult(XmlrpcParamVector *response);
static void * fetchLookupServiceResult(XmlrpcParamVector *response);
static void * fetchGetBusStatsResult(XmlrpcParamVector *response);
static void * fetchGetBusInfoResult(XmlrpcParamVector *response);
static void * fetchGetMasterUriResult(XmlrpcParamVector *response);
static void * fetchRequestShutdownResult(XmlrpcParamVector *response);
static void * fetchGetPidResult(XmlrpcParamVector *response);
static void * fetchGetSubscriptionsResult(XmlrpcParamVector *response);
static void * fetchGetPublicationsResult(XmlrpcParamVector *response);

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
  call->fetch_result_callback = fetchLookupNodeResult;

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
  call->fetch_result_callback = fetchGetPublishedTopicsResult;

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
  call->fetch_result_callback = fetchGetTopicTypesResult;

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
  call->fetch_result_callback = fetchGetSystemStateResult;

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
  call->fetch_result_callback = fetchGetUriResult;

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
  call->fetch_result_callback = fetchLookupServiceResult;

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
  call->fetch_result_callback = fetchGetBusStatsResult;

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
  call->fetch_result_callback = fetchGetBusInfoResult;

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
  call->fetch_result_callback = fetchGetMasterUriResult;

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
  call->fetch_result_callback = fetchRequestShutdownResult;

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
  call->fetch_result_callback = fetchGetPidResult;

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
  call->fetch_result_callback = fetchGetSubscriptionsResult;

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
  call->fetch_result_callback = fetchGetPublicationsResult;

  return enqueueSlaveApiCall(node, call);
}

void * fetchLookupNodeResult(XmlrpcParamVector *response)
{
  return NULL;
}

void * fetchGetPublishedTopicsResult(XmlrpcParamVector *response)
{
  return NULL;
}

void * fetchGetTopicTypesResult(XmlrpcParamVector *response)
{
  return NULL;
}

void * fetchGetSystemStateResult(XmlrpcParamVector *response)
{
  return NULL;
}

void * fetchGetUriResult(XmlrpcParamVector *response)
{
  return NULL;
}

void * fetchLookupServiceResult(XmlrpcParamVector *response)
{
  return NULL;
}

void * fetchGetBusStatsResult(XmlrpcParamVector *response)
{
  return NULL;
}

void * fetchGetBusInfoResult(XmlrpcParamVector *response)
{
  return NULL;
}

void * fetchGetMasterUriResult(XmlrpcParamVector *response)
{
  return NULL;
}

void * fetchRequestShutdownResult(XmlrpcParamVector *response)
{
  return NULL;
}

void * fetchGetPidResult(XmlrpcParamVector *response)
{
  return NULL;
}

void * fetchGetSubscriptionsResult(XmlrpcParamVector *response)
{
  return NULL;
}

void * fetchGetPublicationsResult(XmlrpcParamVector *response)
{
  return NULL;
}

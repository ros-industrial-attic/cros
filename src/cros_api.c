#include <stdlib.h>

#include <assert.h>
#include <string.h>

#include "cros_defs.h"
#include "cros_api.h"
#include "cros_api_internal.h"
#include "cros_message_internal.h"
#include "cros_service.h"
#include "cros_service_internal.h"
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
static DeleteParamResult * fetchDeleteParamResult(XmlrpcParamVector *response);
static SetParamResult * fetchSetParamResult(XmlrpcParamVector *response);
static GetParamResult * fetchGetParamResult(XmlrpcParamVector *response);
static SearchParamResult * fetchSearchParamResult(XmlrpcParamVector *response);
static HasParamResult * fetchHasParamResult(XmlrpcParamVector *response);
static GetParamNamesResult * fetchGetParamNamesResult(XmlrpcParamVector *response);

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
static void freeDeleteParamResult(DeleteParamResult *result);
static void freeSetParamResult(SetParamResult *result);
static void freeGetParamResult(GetParamResult *result);
static void freeSearchParamResult(SearchParamResult *result);
static void freeHasParamResult(HasParamResult *result);
static void freeGetParamNamesResult(GetParamNamesResult *result);

typedef enum ProviderType
{
  CROS_SUBSCRIBER,
  CROS_PUBLISHER,
  CROS_SERVICE_PROVIDER
} ProviderType;

typedef struct ProviderContext
{
  ProviderType type;
  cRosMessage *incoming;
  cRosMessage *outgoing;
  char *message_definition;
  char *md5sum;
  NodeStatusCallback status_callback;
  void *api_callback;
  void *context;
  int unregistering;
} ProviderContext;

static void freeProviderContext(ProviderContext *context)
{
  cRosMessageFree(context->incoming);
  cRosMessageFree(context->outgoing);
  free(context->md5sum);
  free(context);
}

static ProviderContext * newProviderContext(const char *provider_path, ProviderType type)
{
  ProviderContext *context = (ProviderContext *)calloc(1, sizeof(ProviderContext));
  if (context == NULL)
    goto clean;

  context->type = type;
  context->md5sum = (char*) calloc(1, 33);// 32 chars + '\0';
  if (context->md5sum == NULL)
    goto clean;

  int rc;
  switch(type)
  {
    case CROS_SUBSCRIBER:
    {
      context->incoming = cRosMessageNew();
      if (context->incoming == NULL)
        goto clean;

      rc = cRosMessageBuild(context->incoming, provider_path);
      if (rc != 0)
        goto clean;

      strcpy(context->md5sum, context->incoming->md5sum);
      context->message_definition = context->incoming->msgDef->plain_text;
      break;
    }
    case CROS_PUBLISHER:
    {
      context->outgoing = cRosMessageNew();
      if (context->outgoing == NULL)
        goto clean;

      rc = cRosMessageBuild(context->outgoing, provider_path);
      if (rc != 0)
        goto clean;

      strcpy(context->md5sum, context->outgoing->md5sum);
      context->message_definition = context->outgoing->msgDef->plain_text;
      break;
    }
    case CROS_SERVICE_PROVIDER:
    {
      context->incoming = cRosMessageNew();
      if (context->incoming == NULL)
        goto clean;
      context->outgoing = cRosMessageNew();
      if (context->outgoing == NULL)
        goto clean;

      rc = cRosServiceBuildInner(context->incoming, context->outgoing, context->md5sum, provider_path);
      if (rc != 0)
        goto clean;

      break;
    }
    default:
      assert(0);
  }

  return context;

clean:
  freeProviderContext(context);
  return NULL;
}

static CallbackResponse cRosNodePublisherCallback(DynBuffer *buffer, void* context_)
{
  ProviderContext *context = (ProviderContext *)context_;

  // Cast to the appropriate public api callback and invoke it on the user context
  PublisherApiCallback publisherApiCallback = (SubscriberApiCallback)context->api_callback;
  CallbackResponse rc = publisherApiCallback(context->outgoing, context->context);

  cRosMessageSerialize(context->outgoing, buffer);

  return rc;
}

static CallbackResponse cRosNodeSubscriberCallback(DynBuffer *buffer, void* context_)
{
  ProviderContext *context = (ProviderContext *)context_;
  cRosMessageDeserialize(context->incoming, buffer);

  // Cast to the appropriate public api callback and invoke it on the user context
  SubscriberApiCallback subscriberApiCallback = (SubscriberApiCallback)context->api_callback;
  return subscriberApiCallback(context->incoming, context->context);
}

static CallbackResponse cRosNodeServiceProviderCallback(DynBuffer *request, DynBuffer *response, void* contex_)
{
  ProviderContext *context = (ProviderContext *)contex_;
  cRosMessageDeserialize(context->incoming, request);

  ServiceProviderApiCallback serviceProviderApiCallback = (ServiceProviderApiCallback)context->api_callback;
  CallbackResponse rc = serviceProviderApiCallback(context->incoming, context->outgoing, context->context);

  cRosMessageSerialize(context->outgoing, response);

  return rc;
}

static void cRosNodeStatusCallback(CrosNodeStatusUsr *status, void* context_)
{
  ProviderContext *context = (ProviderContext *)context_;
  context->status_callback(status, context->context);
  if (context->unregistering)
    freeProviderContext(context);
}

int cRosApiRegisterServiceProvider(CrosNode *node, const char *service_name, const char *service_type,
                                   ServiceProviderApiCallback callback, NodeStatusCallback status_callback, void *context)
{
  char path[256];
  getSrvFilePath(node, path, 256, service_type);
  ProviderContext *nodeContext = newProviderContext(path, CROS_SERVICE_PROVIDER);
  if (nodeContext == NULL)
    return -1;

  nodeContext->api_callback = callback;
  nodeContext->status_callback = status_callback;
  nodeContext->context = context;

  // NB: Pass the private ProviderContext to the private api, not the user context
  int rc = cRosNodeRegisterServiceProvider(node, service_name, service_type, nodeContext->md5sum,
                                           cRosNodeServiceProviderCallback, cRosNodeStatusCallback,
                                           nodeContext);
  return rc;
}

int cRosApisUnegisterServiceProvider(CrosNode *node, int svcidx)
{
  ServiceProviderNode *service = &node->services[svcidx];
  ProviderContext *context = (ProviderContext *)service->context;
  int rc = cRosNodeUnregisterSubscriber(node, svcidx);
  if (rc != -1)
    context->unregistering = 1;

  return rc;
}

int cRosApiRegisterSubscriber(CrosNode *node, const char *topic_name, const char *topic_type,
                              SubscriberApiCallback callback, NodeStatusCallback status_callback, void *context)
{
  char path[256];
  cRosGetMsgFilePath(node, path, 256, topic_type);
  ProviderContext *nodeContext = newProviderContext(path, CROS_SUBSCRIBER);
  if (nodeContext == NULL)
    return -1;

  nodeContext->api_callback = callback;
  nodeContext->status_callback = status_callback;
  nodeContext->context = context;

  // NB: Pass the private ProviderContext to the private api, not the user context
  int rc = cRosNodeRegisterSubscriber(node, nodeContext->message_definition, topic_name, topic_type,
                                  nodeContext->md5sum, cRosNodeSubscriberCallback,
                                  status_callback == NULL ? NULL : cRosNodeStatusCallback, nodeContext);
  return rc;
}

int cRosApiUnregisterSubscriber(CrosNode *node, int subidx)
{
  SubscriberNode *sub = &node->subs[subidx];
  ProviderContext *context = (ProviderContext *)sub->context;
  int rc = cRosNodeUnregisterSubscriber(node, subidx);
  if (rc != -1)
    context->unregistering = 1;

  return rc;
}

int cRosApiRegisterPublisher(CrosNode *node, const char *topic_name, const char *topic_type, int loop_period,
                             PublisherApiCallback callback, NodeStatusCallback status_callback, void *context)
{
  char path[256];
  cRosGetMsgFilePath(node, path, 256, topic_type);
  ProviderContext *nodeContext = newProviderContext(path, CROS_PUBLISHER);
  if (nodeContext == NULL)
    return -1;

  nodeContext->api_callback = callback;
  nodeContext->status_callback = status_callback;
  nodeContext->context = context;

  // NB: Pass the private ProviderContext to the private api, not the user context
  int rc = cRosNodeRegisterPublisher(node, nodeContext->message_definition, topic_name, topic_type,
                                  nodeContext->md5sum, loop_period, cRosNodePublisherCallback,
                                  status_callback == NULL ? NULL : cRosNodeStatusCallback, nodeContext);
  return rc;
}

int cRosApiUnregisterPublisher(CrosNode *node, int pubidx)
{
  PublisherNode *pub = &node->pubs[pubidx];
  ProviderContext *context = (ProviderContext *)pub->context;
  int rc = cRosNodeUnregisterSubscriber(node, pubidx);
  if (rc != -1)
    context->unregistering = 1;

  return rc;
}

int cRosApicRosApiLookupNode(CrosNode *node, const char *node_name, LookupNodeCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApicRosApiLookupNode() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_LOOKUP_NODE;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchLookupNodeResult;
  call->free_result_callback = (FreeResultCallback)freeLookupNodeResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, node_name);

  return enqueueMasterApiCall(node, call);
}

int cRosApiGetPublishedTopics(CrosNode *node, const char *subgraph, GetPublishedTopicsCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetPublishedTopics() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_PUBLISHED_TOPICS;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetPublishedTopicsResult;
  call->free_result_callback = (FreeResultCallback)freeGetPublishedTopicsResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, subgraph == NULL ? "" : subgraph);

  return enqueueMasterApiCall(node, call);
}

int cRosApiGetTopicTypes(CrosNode *node, GetTopicTypesCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetTopicTypes() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_TOPIC_TYPES;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetTopicTypesResult;
  call->free_result_callback = (FreeResultCallback)freeGetTopicTypesResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  return enqueueMasterApiCall(node, call);
}

int cRosApiGetSystemState(CrosNode *node, GetSystemStateCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetSystemState() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_SYSTEM_STATE;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetSystemStateResult;
  call->free_result_callback = (FreeResultCallback)freeGetSystemStateResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  return enqueueMasterApiCall(node, call);
}

int cRosApiGetUri(CrosNode *node, GetUriCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetUri() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_URI;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetUriResult;
  call->free_result_callback = (FreeResultCallback)freeGetUriResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  return enqueueMasterApiCall(node, call);
}

int cRosApiLookupService(CrosNode *node, const char *service, LookupServiceCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiLookupService() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_LOOKUP_SERVICE;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchLookupServiceResult;
  call->free_result_callback = (FreeResultCallback)freeLookupServiceResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, service);

  return enqueueMasterApiCall(node, call);
}

int cRosApiGetBusStats(CrosNode *node, const char* host, int port,
                       GetBusStatsCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetBusStats() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_BUS_STATS;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetBusStatsResult;
  call->free_result_callback = (FreeResultCallback)freeGetBusStatsResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  return enqueueSlaveApiCall(node, call, host, port);
}

int cRosApiGetBusInfo(CrosNode *node, const char* host, int port,
                      GetBusInfoCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetBusInfo() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_BUS_INFO;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetBusInfoResult;
  call->free_result_callback = (FreeResultCallback)freeGetBusInfoResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  return enqueueSlaveApiCall(node, call, host, port);
}

int cRosApiGetMasterUri(CrosNode *node, const char* host, int port,
                        GetMasterUriCallback callback, void *context)
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

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  return enqueueSlaveApiCall(node, call, host, port);
}

int cRosApiShutdown(CrosNode *node, const char* host, int port, const char *msg,
                    GetMasterUriCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiShutdown() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_SHUTDOWN;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchShutdownResult;
  call->free_result_callback = (FreeResultCallback)freeShutdownResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, msg == NULL ? "" : msg);

  return enqueueSlaveApiCall(node, call, host, port);
}

int cRosApiGetPid(CrosNode *node, const char* host, int port,
                  GetPidCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetPid() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_PID;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetPidResult;
  call->free_result_callback = (FreeResultCallback)freeGetPidResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  return enqueueSlaveApiCall(node, call, host, port);
}

int cRosApiGetSubscriptions(CrosNode *node, const char* host, int port,
                            GetSubscriptionsCallback callback, void *context)
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

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  return enqueueSlaveApiCall(node, call, host, port);
}

int cRosApiGetPublications(CrosNode *node, const char* host, int port,
                    GetSubscriptionsCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetPublications() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_PUBLICATIONS;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetPublicationsResult;
  call->free_result_callback = (FreeResultCallback)freeGetPublicationsResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  return enqueueSlaveApiCall(node, call, host, port);
}

int cRosApiDeleteParam(CrosNode *node, const char *key, DeleteParamCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiDeleteParam() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_DELETE_PARAM;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchDeleteParamResult;
  call->free_result_callback = (FreeResultCallback)freeDeleteParamResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, key);

  return enqueueMasterApiCall(node, call);
}

int cRosApiSetParam(CrosNode *node, const char *key, XmlrpcParam *value, SetParamCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiSetParam() : Can't allocate memory\n");
    return -1;
  }

  XmlrpcParam param;
  int rc = xmlrpcParamCopy(&param, value);
  if (rc == -1)
  {
    PRINT_ERROR ( "cRosApiSetParam() : Can't allocate memory\n");
    freeRosApiCall(call);
    return -1;
  }

  call->method = CROS_API_SET_PARAM;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchSetParamResult;
  call->free_result_callback = (FreeResultCallback)freeSetParamResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, key);
  rc = xmlrpcParamVectorPushBack(&call->params, &param);
  if (rc == -1)
  {
    freeRosApiCall(call);
    xmlrpcParamRelease(&param);
    return -1;
  }

  return enqueueMasterApiCall(node, call);
}

int cRosApiGetParam(CrosNode *node, const char *key, GetParamCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetParam() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_PARAM;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetParamResult;
  call->free_result_callback = (FreeResultCallback)freeGetParamResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, key);

  return enqueueMasterApiCall(node, call);
}

int cRosApiSearchParam(CrosNode *node, const char *key, SearchParamCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiSearchParam() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_SEARCH_PARAM;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchSearchParamResult;
  call->free_result_callback = (FreeResultCallback)freeSearchParamResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, key);

  return enqueueMasterApiCall(node, call);
}

int cRosApiHasParam(CrosNode *node, const char *key, HasParamCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiHasParam() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_HAS_PARAM;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchHasParamResult;
  call->free_result_callback = (FreeResultCallback)freeHasParamResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, key);

  return enqueueMasterApiCall(node, call);
}

int cRosApiGetParamNames(CrosNode *node, GetParamNamesCallback callback, void *context)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetParamNames() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_PARAM_NAMES;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetParamNamesResult;
  call->free_result_callback = (FreeResultCallback)freeGetParamNamesResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  return enqueueMasterApiCall(node, call);
}

LookupNodeResult * fetchLookupNodeResult(XmlrpcParamVector *response)
{
  LookupNodeResult *ret = (LookupNodeResult *)calloc(1, sizeof(LookupNodeResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);

  XmlrpcParam* uri = xmlrpcParamArrayGetParamAt(array, 2);
  ret->uri = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret == NULL)
    goto clean;
  strcpy(ret->uri, uri->data.as_string);

  return ret;

clean:
  freeLookupNodeResult(ret);
  return NULL;
}

GetPublishedTopicsResult * fetchGetPublishedTopicsResult(XmlrpcParamVector *response)
{
  GetPublishedTopicsResult *ret = (GetPublishedTopicsResult *)calloc(1, sizeof(GetPublishedTopicsResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);

  XmlrpcParam* topics = xmlrpcParamArrayGetParamAt(array, 2);
  ret->topics = (struct TopicTypePair *)calloc(topics->array_n_elem, sizeof(struct TopicTypePair));
  if (ret->topics == NULL)
    goto clean;
  ret->topic_count = topics->array_n_elem;

  int it = 0;
  for (; it < topics->array_n_elem; it++)
  {
    struct TopicTypePair *pair = &ret->topics[it];
    XmlrpcParam* pair_xml = xmlrpcParamArrayGetParamAt(topics, it);
    XmlrpcParam* topic = xmlrpcParamArrayGetParamAt(pair_xml, 0);
    pair->topic = (char *)malloc(strlen(topic->data.as_string) + 1);
    if (pair->topic == NULL)
      goto clean;
    strcpy(pair->topic, topic->data.as_string);

    XmlrpcParam* type = xmlrpcParamArrayGetParamAt(pair_xml, 1);
    pair->type = (char *)malloc(strlen(type->data.as_string) + 1);
    if (pair->type == NULL)
      goto clean;
    strcpy(pair->type, type->data.as_string);
  }

  return ret;

clean:
  freeGetPublishedTopicsResult(ret);
  return NULL;
}

GetTopicTypesResult * fetchGetTopicTypesResult(XmlrpcParamVector *response)
{
  GetTopicTypesResult *ret = (GetTopicTypesResult *)calloc(1, sizeof(GetTopicTypesResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);

  XmlrpcParam* topics = xmlrpcParamArrayGetParamAt(array, 2);
  ret->topics = (struct TopicTypePair *)calloc(topics->array_n_elem, sizeof(struct TopicTypePair));
  if (ret->topics == NULL)
    goto clean;
  ret->topic_count = topics->array_n_elem;

  int it = 0;
  for (; it < topics->array_n_elem; it++)
  {
    struct TopicTypePair *pair = &ret->topics[it];
    XmlrpcParam* pair_xml = xmlrpcParamArrayGetParamAt(topics, it);
    XmlrpcParam* topic = xmlrpcParamArrayGetParamAt(pair_xml, 0);
    pair->topic = (char *)malloc(strlen(topic->data.as_string) + 1);
    if (pair->topic == NULL)
      goto clean;
    strcpy(pair->topic, topic->data.as_string);

    XmlrpcParam* type = xmlrpcParamArrayGetParamAt(pair_xml, 1);
    pair->type = (char *)malloc(strlen(type->data.as_string) + 1);
    if (pair->type == NULL)
      goto clean;
    strcpy(pair->type, type->data.as_string);
  }

  return ret;

clean:
  freeGetTopicTypesResult(ret);
  return NULL;
}

GetSystemStateResult * fetchGetSystemStateResult(XmlrpcParamVector *response)
{
  GetSystemStateResult *ret = (GetSystemStateResult *)calloc(1, sizeof(GetSystemStateResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);

  if (array->array_n_elem < 3)
    return ret;

  XmlrpcParam* publishers = xmlrpcParamArrayGetParamAt(array, 2);
  ret->publishers = (struct ProviderState *)calloc(publishers->array_n_elem, sizeof(struct ProviderState));
  if (ret->publishers == NULL)
    goto clean;
  ret->pub_count = publishers->array_n_elem;

  int it1;
  for (it1 = 0; it1 < publishers->array_n_elem; it1++)
  {
    struct ProviderState *state = &ret->publishers[it1];
    XmlrpcParam* state_xml = xmlrpcParamArrayGetParamAt(publishers, it1);
    XmlrpcParam* name = xmlrpcParamArrayGetParamAt(state_xml, 0);
    state->provider_name = (char *)malloc(strlen(name->data.as_string) + 1);
    if (state->provider_name == NULL)
      goto clean;
    strcpy(state->provider_name, name->data.as_string);

    XmlrpcParam* users_xml = xmlrpcParamArrayGetParamAt(state_xml, 1);
    state->users = (char **)calloc(1, users_xml->array_n_elem * sizeof(char *));
    if (state->users == NULL)
      goto clean;

    int it2 = 0;
    for (; it2 < users_xml->array_n_elem; it2++)
    {
      char *user = state->users[it2];
      XmlrpcParam* user_xml = xmlrpcParamArrayGetParamAt(users_xml, it2);
      user = (char*)malloc(strlen(user_xml->data.as_string) + 1);
      if (user == NULL)
        goto clean;
      strcpy(user, user_xml->data.as_string);
    }
  }

  if (array->array_n_elem < 4)
    return ret;

  XmlrpcParam* subscribers = xmlrpcParamArrayGetParamAt(array, 3);
  ret->subscribers = (struct ProviderState *)calloc(subscribers->array_n_elem, sizeof(struct ProviderState));
  if (ret->subscribers == NULL)
    goto clean;
  ret->sub_count = subscribers->array_n_elem;

  for (it1 = 0; it1 < subscribers->array_n_elem; it1++)
  {
    struct ProviderState *state = &ret->subscribers[it1];
    XmlrpcParam* state_xml = xmlrpcParamArrayGetParamAt(subscribers, it1);
    XmlrpcParam* name = xmlrpcParamArrayGetParamAt(state_xml, 0);
    state->provider_name = (char *)malloc(strlen(name->data.as_string) + 1);
    if (state->provider_name == NULL)
      goto clean;
    strcpy(state->provider_name, name->data.as_string);

    XmlrpcParam* users_xml = xmlrpcParamArrayGetParamAt(state_xml, 1);
    state->users = (char **)calloc(users_xml->array_n_elem, sizeof(char *));
    if (state->users == NULL)
      goto clean;

    int it2 = 0;
    for (; it2 < users_xml->array_n_elem; it2++)
    {
      char *user = state->users[it2];
      XmlrpcParam* user_xml = xmlrpcParamArrayGetParamAt(users_xml, it2);
      user = (char*)malloc(strlen(user_xml->data.as_string) + 1);
      if (user == NULL)
        goto clean;
      strcpy(user, user_xml->data.as_string);
    }
  }

  if (array->array_n_elem < 5)
    return ret;

  XmlrpcParam* services = xmlrpcParamArrayGetParamAt(array, 4);
  ret->service_providers = (struct ProviderState *)calloc(services->array_n_elem, sizeof(struct ProviderState));
  if (ret->service_providers == NULL)
    goto clean;
  ret->svc_count = services->array_n_elem;

  for (it1 = 0; it1 < services->array_n_elem; it1++)
  {
    struct ProviderState *state = &ret->service_providers[it1];
    XmlrpcParam* state_xml = xmlrpcParamArrayGetParamAt(services, it1);
    XmlrpcParam* name = xmlrpcParamArrayGetParamAt(state_xml, 0);
    state->provider_name = (char *)malloc(strlen(name->data.as_string) + 1);
    if (state->provider_name == NULL)
      goto clean;
    strcpy(state->provider_name, name->data.as_string);

    XmlrpcParam* users_xml = xmlrpcParamArrayGetParamAt(state_xml, 1);
    state->users = (char **)calloc(users_xml->array_n_elem, sizeof(char *));
    if (state->users == NULL)
      goto clean;

    int it2 = 0;
    for (; it2 < users_xml->array_n_elem; it2++)
    {
      char *user = state->users[it2];
      XmlrpcParam* user_xml = xmlrpcParamArrayGetParamAt(users_xml, it2);
      user = (char*)malloc(strlen(user_xml->data.as_string) + 1);
      if (user == NULL)
        goto clean;
      strcpy(user, user_xml->data.as_string);
    }
  }

  return ret;

clean:
  freeGetSystemStateResult(ret);
  return NULL;
}

GetUriResult * fetchGetUriResult(XmlrpcParamVector *response)
{
  GetUriResult *ret = (GetUriResult *)calloc(1, sizeof(GetUriResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);

  XmlrpcParam* uri = xmlrpcParamArrayGetParamAt(array, 2);
  ret->master_uri = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret == NULL)
    goto clean;
  strcpy(ret->master_uri, uri->data.as_string);

  return ret;

clean:
  freeGetUriResult(ret);
  return NULL;
}

LookupServiceResult * fetchLookupServiceResult(XmlrpcParamVector *response)
{
  LookupServiceResult *ret = (LookupServiceResult *)calloc(1, sizeof(LookupServiceResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);

  XmlrpcParam* service = xmlrpcParamArrayGetParamAt(array, 2);
  ret->service_result = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret == NULL)
    goto clean;
  strcpy(ret->service_result, service->data.as_string);

  return ret;

clean:
  freeLookupServiceResult(ret);
  return NULL;
}

GetBusStatsResult * fetchGetBusStatsResult(XmlrpcParamVector *response)
{
  GetBusStatsResult *ret = (GetBusStatsResult *)calloc(1, sizeof(GetBusStatsResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);

  if (array->array_n_elem < 3)
    return ret;

  XmlrpcParam* pubs_stats = xmlrpcParamArrayGetParamAt(array, 2);
  ret->stats.pub_stats = (struct TopicPubStats *)calloc(pubs_stats->array_n_elem, sizeof(struct TopicPubStats));
  if (ret->stats.pub_stats == NULL)
    goto clean;
  ret->stats.pub_stats_count = pubs_stats->array_n_elem;

  int it1;
  for (it1 = 0; it1 < pubs_stats->array_n_elem; it1++)
  {
    struct TopicPubStats *pub_stats = &ret->stats.pub_stats[it1];
    XmlrpcParam* pub_stats_xml = xmlrpcParamArrayGetParamAt(pubs_stats, it1);
    if (pub_stats_xml->array_n_elem < 3)
      goto clean;

    XmlrpcParam* name_xml = xmlrpcParamArrayGetParamAt(pub_stats_xml, 0);
    pub_stats->topic_name = (char *)malloc(strlen(name_xml->data.as_string) + 1);
    if (pub_stats->topic_name == NULL)
      goto clean;
    strcpy(pub_stats->topic_name, name_xml->data.as_string);

    XmlrpcParam* message_data_sent = xmlrpcParamArrayGetParamAt(pub_stats_xml, 1);
    pub_stats->message_data_sent = (size_t)message_data_sent->data.as_int;

    XmlrpcParam* pub_datas = xmlrpcParamArrayGetParamAt(pub_stats_xml, 2);
    pub_stats->datas = (struct PubConnectionData *)calloc(pub_datas->array_n_elem, sizeof(struct PubConnectionData));
    if (pub_stats->datas == NULL)
      goto clean;
    pub_stats->datas_count = pub_datas->array_n_elem;

    int it2 = 0;
    for (; it2 < pub_datas->array_n_elem; it2++)
    {
      struct PubConnectionData *pub_data = &pub_stats->datas[it2];
      XmlrpcParam* pub_data_xml = xmlrpcParamArrayGetParamAt(pub_datas, it2);
      XmlrpcParam* connection_id = xmlrpcParamArrayGetParamAt(pub_data_xml, 0);
      XmlrpcParam* bytes_sent = xmlrpcParamArrayGetParamAt(pub_data_xml, 1);
      XmlrpcParam* num_sent = xmlrpcParamArrayGetParamAt(pub_data_xml, 2);
      XmlrpcParam* connected = xmlrpcParamArrayGetParamAt(pub_data_xml, 3);
      pub_data->connection_id = connection_id->data.as_int;
      pub_data->bytes_sent = (size_t)bytes_sent->data.as_int;
      pub_data->num_sent = (size_t)num_sent->data.as_int;
      pub_data->connected = connected->data.as_int;
    }
  }

  if (array->array_n_elem < 4)
    return ret;

  XmlrpcParam* subs_stats = xmlrpcParamArrayGetParamAt(array, 3);
  ret->stats.sub_stats = (struct TopicSubStats *)calloc(subs_stats->array_n_elem, sizeof(struct TopicSubStats));
  if (ret->stats.sub_stats == NULL)
    goto clean;
  ret->stats.sub_stats_count = subs_stats->array_n_elem;

  for (it1 = 0; it1 < subs_stats->array_n_elem; it1++)
  {
    struct TopicSubStats *sub_stats = &ret->stats.sub_stats[it1];
    XmlrpcParam *sub_stats_xml = xmlrpcParamArrayGetParamAt(subs_stats, it1);
    if (sub_stats_xml->array_n_elem < 3)
      goto clean;

    XmlrpcParam *name_xml = xmlrpcParamArrayGetParamAt(sub_stats_xml, 0);
    sub_stats->topic_name = (char *)malloc(strlen(name_xml->data.as_string) + 1);
    if (sub_stats->topic_name == NULL)
      goto clean;
    strcpy(sub_stats->topic_name, name_xml->data.as_string);

    XmlrpcParam* sub_datas = xmlrpcParamArrayGetParamAt(sub_stats_xml, 1);
    sub_stats->datas = (struct SubConnectionData *)calloc(sub_datas->array_n_elem, sizeof(struct SubConnectionData));
    if (sub_stats->datas == NULL)
      goto clean;
    sub_stats->datas_count = sub_datas->array_n_elem;

    int it2 = 0;
    for (; it2 < sub_datas->array_n_elem; it2++)
    {
      struct SubConnectionData *sub_data = &sub_stats->datas[it2];
      XmlrpcParam *sub_data_xml = xmlrpcParamArrayGetParamAt(sub_datas, it2);
      XmlrpcParam *connection_id = xmlrpcParamArrayGetParamAt(sub_data_xml, 0);
      XmlrpcParam *bytes_received = xmlrpcParamArrayGetParamAt(sub_data_xml, 1);
      XmlrpcParam *drop_estimate = xmlrpcParamArrayGetParamAt(sub_data_xml, 2);
      XmlrpcParam *connected = xmlrpcParamArrayGetParamAt(sub_data_xml, 3);
      sub_data->connection_id = connection_id->data.as_int;
      sub_data->bytes_received = (size_t)bytes_received->data.as_int;
      sub_data->drop_estimate = drop_estimate->data.as_int;
      sub_data->connected = connected->data.as_bool;
    }
  }

  if (array->array_n_elem < 5)
    return ret;

  XmlrpcParam *services_stats = xmlrpcParamArrayGetParamAt(array, 4);
  XmlrpcParam *numRequests = xmlrpcParamArrayGetParamAt(services_stats, 0);
  XmlrpcParam *bytesReceived = xmlrpcParamArrayGetParamAt(services_stats, 1);
  XmlrpcParam *bytesSent = xmlrpcParamArrayGetParamAt(services_stats, 2);

  ret->stats.service_stats.num_requests = (size_t)numRequests->data.as_int;
  ret->stats.service_stats.bytes_received = (size_t)bytesReceived->data.as_int;
  ret->stats.service_stats.bytes_sent = (size_t)bytesSent->data.as_int;

  return ret;

clean:
  freeGetBusStatsResult(ret);
  return NULL;
}

GetBusInfoResult * fetchGetBusInfoResult(XmlrpcParamVector *response)
{
  GetBusInfoResult *ret = (GetBusInfoResult *)calloc(1, sizeof(GetBusInfoResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);

  if (array->array_n_elem < 3)
    return ret;

  XmlrpcParam* businfos = xmlrpcParamArrayGetParamAt(array, 2);
  ret->bus_infos = (struct BusInfo *)calloc(businfos->array_n_elem, sizeof(struct BusInfo));
  if (ret->bus_infos == NULL)
    goto clean;
  ret->bus_infos_count = businfos->array_n_elem;

  int it = 0;
  for (; it < businfos->array_n_elem; it++)
  {
    struct BusInfo *businfo = &ret->bus_infos[it];
    XmlrpcParam *businfo_xml = xmlrpcParamArrayGetParamAt(businfos, it);

    XmlrpcParam *connectionId = xmlrpcParamArrayGetParamAt(businfo_xml, 0);
    XmlrpcParam *destinationId = xmlrpcParamArrayGetParamAt(businfo_xml, 1);
    XmlrpcParam *direction = xmlrpcParamArrayGetParamAt(businfo_xml, 2);
    XmlrpcParam *transport = xmlrpcParamArrayGetParamAt(businfo_xml, 3);
    XmlrpcParam *topic = xmlrpcParamArrayGetParamAt(businfo_xml, 4);
    XmlrpcParam *connected = xmlrpcParamArrayGetParamAt(businfo_xml, 5);

    businfo->topic = (char *)malloc(strlen(topic->data.as_string) + 1);
    if (businfo->topic == NULL)
      goto clean;
    strcpy(businfo->topic, topic->data.as_string);
    businfo->connectionId = connectionId->data.as_int;
    businfo->destinationId = connectionId->data.as_int;
    switch (connectionId->data.as_string[0])
    {
      case 'i':
        businfo->direction = CROS_TRANSPORT_DIRECTION_IN;
        break;
      case 'o':
        businfo->direction = CROS_TRANSPORT_DIRECTION_OUT;
        break;
      case 'b':
        businfo->direction = CROS_TRANSPORT_DIRECTION_BOTH;
        break;
      default:
        goto clean;
    }
    businfo->transport = (CrosTransportType)connectionId->data.as_int;
    businfo->connected = connectionId->data.as_int;
  }

  return ret;

clean:
  freeGetBusInfoResult(ret);
  return NULL;
}

GetMasterUriResult * fetchGetMasterUriResult(XmlrpcParamVector *response)
{
  GetMasterUriResult *ret = (GetMasterUriResult *)calloc(1, sizeof(GetMasterUriResult));
  if (ret == NULL)
    return ret;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);

  XmlrpcParam* uri = xmlrpcParamArrayGetParamAt(array, 2);
  ret->master_uri = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret == NULL)
    goto clean;
  strcpy(ret->master_uri, uri->data.as_string);

  return ret;

clean:
  freeGetMasterUriResult(ret);
  return NULL;
}

ShutdownResult * fetchShutdownResult(XmlrpcParamVector *response)
{
  ShutdownResult *ret = (ShutdownResult *)calloc(1, sizeof(ShutdownResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);

  XmlrpcParam* ignore = xmlrpcParamArrayGetParamAt(array, 2);
  ret->ignore = ignore->data.as_bool;

  return ret;

clean:
  freeShutdownResult(ret);
  return NULL;
}

GetPidResult * fetchGetPidResult(XmlrpcParamVector *response)
{
  GetPidResult *ret = (GetPidResult *)calloc(1, sizeof(GetPidResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);

  XmlrpcParam* roscore_pid_param = xmlrpcParamArrayGetParamAt(array, 2);
  ret->server_process_pid = roscore_pid_param->data.as_int;
  return ret;

clean:
  freeGetPidResult(ret);
  return NULL;
}

GetSubscriptionsResult * fetchGetSubscriptionsResult(XmlrpcParamVector *response)
{
  GetSubscriptionsResult *ret = (GetSubscriptionsResult *)calloc(1, sizeof(GetSubscriptionsResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);

  XmlrpcParam* topics = xmlrpcParamArrayGetParamAt(array, 2);
  ret->topic_list = (struct TopicTypePair *)calloc(topics->array_n_elem, sizeof(struct TopicTypePair));
  if (ret->topic_list == NULL)
    goto clean;
  ret->topic_count = topics->array_n_elem;

  int it = 0;
  for (; it < topics->array_n_elem; it++)
  {
    struct TopicTypePair *pair = &ret->topic_list[it];
    XmlrpcParam* pair_xml = xmlrpcParamArrayGetParamAt(topics, it);
    XmlrpcParam* topic = xmlrpcParamArrayGetParamAt(pair_xml, 0);
    pair->topic = (char *)malloc(strlen(topic->data.as_string) + 1);
    if (pair->topic == NULL)
      goto clean;
    strcpy(pair->topic, topic->data.as_string);

    XmlrpcParam* type = xmlrpcParamArrayGetParamAt(pair_xml, 1);
    pair->type = (char *)malloc(strlen(type->data.as_string) + 1);
    if (pair->type == NULL)
      goto clean;
    strcpy(pair->type, type->data.as_string);
  }

  return ret;

clean:
  freeGetSubscriptionsResult(ret);
  return NULL;
}

GetPublicationsResult * fetchGetPublicationsResult(XmlrpcParamVector *response)
{
  GetPublicationsResult *ret = (GetPublicationsResult *)calloc(1, sizeof(GetPublicationsResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);

  XmlrpcParam* topics = xmlrpcParamArrayGetParamAt(array, 2);
  ret->topic_list = (struct TopicTypePair *)calloc(topics->array_n_elem, sizeof(struct TopicTypePair));
  if (ret->topic_list == NULL)
    goto clean;
  ret->topic_count = topics->array_n_elem;

  int it = 0;
  for (; it < topics->array_n_elem; it++)
  {
    struct TopicTypePair *pair = &ret->topic_list[it];
    XmlrpcParam* pair_xml = xmlrpcParamArrayGetParamAt(topics, it);
    XmlrpcParam* topic = xmlrpcParamArrayGetParamAt(pair_xml, 0);
    pair->topic = (char *)malloc(strlen(topic->data.as_string) + 1);
    if (pair->topic == NULL)
      goto clean;
    strcpy(pair->topic, topic->data.as_string);

    XmlrpcParam* type = xmlrpcParamArrayGetParamAt(pair_xml, 1);
    pair->type = (char *)malloc(strlen(type->data.as_string) + 1);
    if (pair->type == NULL)
      goto clean;
    strcpy(pair->type, type->data.as_string);
  }

  return ret;

clean:
  freeGetPublicationsResult(ret);
  return NULL;
}

static DeleteParamResult * fetchDeleteParamResult(XmlrpcParamVector *response)
{
  DeleteParamResult *ret = (DeleteParamResult *)calloc(1, sizeof(DeleteParamResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);
  XmlrpcParam* ignore = xmlrpcParamArrayGetParamAt(array, 2);
  ret->ignore = ignore->data.as_bool;

  return ret;

clean:
  freeDeleteParamResult(ret);
  return NULL;
}

static SetParamResult * fetchSetParamResult(XmlrpcParamVector *response)
{
  SetParamResult *ret = (SetParamResult *)calloc(1, sizeof(SetParamResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);
  XmlrpcParam* ignore = xmlrpcParamArrayGetParamAt(array, 2);
  ret->ignore = ignore->data.as_bool;

  return ret;

clean:
  freeSetParamResult(ret);
  return NULL;
}

static GetParamResult * fetchGetParamResult(XmlrpcParamVector *response)
{
  GetParamResult *ret = (GetParamResult *)calloc(1, sizeof(GetParamResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);
  XmlrpcParam* value = xmlrpcParamArrayGetParamAt(array, 2);
  ret->value = xmlrpcParamClone(value);
  if (ret->status == NULL)
    goto clean;

  return ret;

clean:
  freeGetParamResult(ret);
  return NULL;
}

static SearchParamResult * fetchSearchParamResult(XmlrpcParamVector *response)
{
  SearchParamResult *ret = (SearchParamResult *)calloc(1, sizeof(SearchParamResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);
  XmlrpcParam* found_key = xmlrpcParamArrayGetParamAt(array, 2);
  ret->found_key = (char *)malloc(strlen(found_key->data.as_string) + 1);
  if (ret->found_key == NULL)
    goto clean;
  strcpy(ret->found_key, found_key->data.as_string);

  return ret;

clean:
  freeSearchParamResult(ret);
  return NULL;
}

static HasParamResult * fetchHasParamResult(XmlrpcParamVector *response)
{
  HasParamResult *ret = (HasParamResult *)calloc(1, sizeof(HasParamResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);
  XmlrpcParam* ignore = xmlrpcParamArrayGetParamAt(array, 2);
  ret->has_param = ignore->data.as_bool;

  return ret;

clean:
  freeHasParamResult(ret);
  return NULL;
}

static GetParamNamesResult * fetchGetParamNamesResult(XmlrpcParamVector *response)
{
  GetParamNamesResult *ret = (GetParamNamesResult *)calloc(1, sizeof(GetParamNamesResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array = xmlrpcParamVectorAt(response, 0);
  XmlrpcParam* code = xmlrpcParamArrayGetParamAt(array, 0);
  ret->code  = code->data.as_int;
  XmlrpcParam* status = xmlrpcParamArrayGetParamAt(array, 1);
  ret->status = (char *)malloc(strlen(status->data.as_string) + 1);
  if (ret->status == NULL)
    goto clean;
  strcpy(ret->status, status->data.as_string);
  XmlrpcParam* param_names = xmlrpcParamArrayGetParamAt(array, 2);
  ret->parameter_names = (char **)calloc(param_names->array_n_elem, sizeof(char *));
  if (ret->parameter_names== NULL)
    goto clean;

  int it = 0;
  for (; it < param_names->array_n_elem; it++)
  {
    ret->parameter_names[it] = malloc(strlen(param_names->data.as_array[it].data.as_string) + 1);
    if (ret->parameter_names[it] == NULL)
      goto clean;

    strcpy(ret->parameter_names[it], param_names->data.as_array[it].data.as_string);
  }
  ret->parameter_count = (size_t)param_names->array_n_elem;

  return ret;

clean:
  freeGetParamNamesResult(ret);
  return NULL;
}


void freeLookupNodeResult(LookupNodeResult *result)
{
  free(result->status);
  free(result->uri);
  free(result);
}

void freeGetPublishedTopicsResult(GetPublishedTopicsResult *result)
{
  free(result->status);
  int it = 0;
  for (; it < result->topic_count; it++)
  {
    free(result->topics[it].topic);
    free(result->topics[it].type);
  }
  free(result->topics);
  free(result);
}

void freeGetTopicTypesResult(GetTopicTypesResult *result)
{
  free(result->status);
  int it = 0;
  for (; it < result->topic_count; it++)
  {
    free(result->topics[it].topic);
    free(result->topics[it].type);
  }
  free(result->topics);
  free(result);
}

void freeGetSystemStateResult(GetSystemStateResult *result)
{
  free(result->status);
  int it1 = 0;
  for (; it1 < result->sub_count; it1++)
  {
    free(result->publishers[it1].provider_name);
    int it2 = 0;
    for (; it2 < result->publishers[it1].user_count; it2)
      free(result->publishers[it1].users[it2]);
    free(result->publishers[it1].users);
  }
  free(result->publishers);

  for (; it1 < result->sub_count; it1++)
  {
    free(result->subscribers[it1].provider_name);
    int it2 = 0;
    for (; it2 < result->subscribers[it1].user_count; it2)
      free(result->subscribers[it1].users[it2]);
    free(result->subscribers[it1].users);
  }
  free(result->subscribers);

  for (; it1 < result->svc_count; it1++)
  {
    free(result->service_providers[it1].provider_name);
    int it2 = 0;
    for (; it2 < result->service_providers[it1].user_count; it2)
      free(result->service_providers[it1].users[it2]);
    free(result->service_providers[it1].users);
  }
  free(result->service_providers);

  free(result);
}

void freeGetUriResult(GetUriResult *result)
{
  free(result->status);
  free(result->master_uri);
  free(result);
}

void freeLookupServiceResult(LookupServiceResult *result)
{
  free(result->status);
  free(result->service_result);
  free(result);
}

void freeGetBusStatsResult(GetBusStatsResult *result)
{
  free(result->status);
  int it1 = 0;
  for (; it1 < result->stats.pub_stats_count; it1++)
    free(result->stats.pub_stats[it1].topic_name);
  free(result->stats.pub_stats);

  for (; it1 < result->stats.sub_stats_count; it1++)
    free(result->stats.pub_stats[it1].topic_name);
  free(result->stats.sub_stats);

  free(result);
}

void freeGetBusInfoResult(GetBusInfoResult *result)
{
  free(result->status);
  int it = 0;
  for (; it < result->bus_infos_count; it++)
    free(result->bus_infos[it].topic);
  free(result->bus_infos);
  free(result);
}

void freeGetMasterUriResult(GetMasterUriResult *result)
{
  free(result->status);
  free(result->master_uri);
  free(result);
}

void freeShutdownResult(ShutdownResult *result)
{
  free(result->status);
  free(result);
}

void freeGetPidResult(GetPidResult *result)
{
  free(result->status);
  free(result);
}

void freeGetSubscriptionsResult(GetSubscriptionsResult *result)
{
  free(result->status);
  int it = 0;
  for (; it < result->topic_count; it++)
  {
    free(result->topic_list[it].topic);
    free(result->topic_list[it].type);
  }
  free(result->topic_list);
  free(result);
}

void freeGetPublicationsResult(GetPublicationsResult *result)
{
  free(result->status);
  int it = 0;
  for (; it < result->topic_count; it++)
  {
    free(result->topic_list[it].topic);
    free(result->topic_list[it].type);
  }
  free(result->topic_list);
  free(result);
}

static void freeDeleteParamResult(DeleteParamResult *result)
{
  free(result->status);
  free(result);
}

static void freeSetParamResult(SetParamResult *result)
{
  free(result->status);
  free(result);
}

static void freeGetParamResult(GetParamResult *result)
{
  free(result->status);
  xmlrpcParamFree(result->value);
  free(result);
}

static void freeSearchParamResult(SearchParamResult *result)
{
  free(result->status);
  free(result->found_key);
  free(result);
}

static void freeHasParamResult(HasParamResult *result)
{
  free(result->status);
  free(result);
}

static void freeGetParamNamesResult(GetParamNamesResult *result)
{
  free(result->status);
  int it = 0;
  for (; it < result->parameter_count; it++)
    free(result->parameter_names[it]);
  free(result->parameter_names);
  free(result);
}


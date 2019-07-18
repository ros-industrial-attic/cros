#include <stdlib.h>
#include <string.h>
#include <limits.h> // for PATH_MAX

#include "cros_defs.h"
#include "cros_api.h"
#include "cros_api_internal.h"
#include "cros_message_internal.h"
#include "cros_service.h"
#include "cros_service_internal.h"
#include "cros_message_queue.h"
#include "xmlrpc_process.h"

#ifdef _WIN32
#  define OS_MAX_PATH _MAX_PATH
#else
#  define OS_MAX_PATH PATH_MAX
#endif

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
  CROS_SERVICE_CALLER,
  CROS_SERVICE_PROVIDER
} ProviderType;

typedef struct ProviderContext
{
  ProviderType type; //! Type of role: provider, subscriber, service provider or service caller
  cRosMessage *incoming; //! Message that has been received
  cRosMessage *outgoing; //! Message that will be sent
  char *message_definition;
  char *md5sum;
  void *api_callback; //! The application-defined callback function called to generate outgoing data or to handle the received data
  NodeStatusApiCallback status_api_callback; //! The application-defined callback function called when the state of the role has chnaged
  cRosMessageQueue *msg_queue; //! It is just a reference to the queue declared in node. For the publisher: it is msgs to send. For the subscriber: it is msgs received. For the svc caller: it is first svc request and then svc response
  void *context; //! Context parameter specified by the application and that will be passed to the application-defined callback functions
} ProviderContext;

static void initProviderContext(ProviderContext *context)
{
  context->type=-1;
  context->incoming=NULL;
  context->outgoing=NULL;
  context->message_definition=NULL;
  context->md5sum=NULL;
  context->status_api_callback=NULL;
  context->api_callback=NULL;
  context->msg_queue=NULL;
  context->context=NULL;
}

static void freeProviderContext(ProviderContext *context)
{
  if(context != NULL)
  {
    cRosMessageFree(context->incoming);
    cRosMessageFree(context->outgoing);
    free(context->message_definition);
    free(context->md5sum);
    free(context);
  }
}

static cRosErrCodePack newProviderContext(const char *provider_path, ProviderType type, ProviderContext **context_ptr)
{
  cRosErrCodePack ret_err;

  ret_err = CROS_SUCCESS_ERR_PACK; // Default return value: success

  ProviderContext *context = (ProviderContext *)malloc(sizeof(ProviderContext));
  if (context == NULL)
    return CROS_MEM_ALLOC_ERR;

  initProviderContext(context);
  context->type = type;
  context->md5sum = (char *)calloc(sizeof(char), 33);// 32 chars + '\0';
  if (context->md5sum == NULL)
  {
    free(context);
    return CROS_MEM_ALLOC_ERR;
  }

  switch(type)
  {
    case CROS_SUBSCRIBER:
    {
      ret_err = cRosMessageNewBuild(NULL, provider_path, &context->incoming);
      if (ret_err == CROS_SUCCESS_ERR_PACK)
      {
        strcpy(context->md5sum, context->incoming->md5sum);
        context->message_definition = strdup(context->incoming->msgDef->plain_text); // Alloc new mem so that it can be freed independently
      }
      break;
    }
    case CROS_PUBLISHER:
    {
      ret_err = cRosMessageNewBuild(NULL, provider_path, &context->outgoing);
      if (ret_err == CROS_SUCCESS_ERR_PACK)
      {
        strcpy(context->md5sum, context->outgoing->md5sum);
        context->message_definition = strdup(context->outgoing->msgDef->plain_text);
      }
      break;
    }
    case CROS_SERVICE_PROVIDER:
    {
      ret_err = cRosServiceBuildInner(&context->incoming, &context->outgoing, NULL, context->md5sum, provider_path);
      break;
    }
    case CROS_SERVICE_CALLER:
    {
      ret_err = cRosServiceBuildInner(&context->outgoing, &context->incoming, &context->message_definition, context->md5sum, provider_path);
      break;
    }
    default:
    {
      PRINT_ERROR ( "newProviderContext() : Unknown ProviderType specified\n" );
      ret_err = CROS_BAD_PARAM_ERR;
    }
  }

  if(ret_err == CROS_SUCCESS_ERR_PACK)
    *context_ptr = context; // No error occurred: return the created context
  else
    freeProviderContext(context); // An error occurred: free the allocated memory and return the error code

  return ret_err;
}

cRosErrCodePack cRosNodeSerializeOutgoingMessage(DynBuffer *buffer, void *context_)
{
  cRosErrCodePack ret_err;
  ProviderContext *context = (ProviderContext *)context_;

  ret_err = cRosMessageSerialize(context->outgoing, buffer);
  return(ret_err);
}

cRosErrCodePack cRosNodeDeserializeIncomingPacket(DynBuffer *buffer, void *context_)
{
  cRosErrCodePack ret_err;
  ProviderContext *context = (ProviderContext *)context_;

  ret_err = cRosMessageDeserialize(context->incoming, buffer);
  return(ret_err);
}

cRosErrCodePack cRosNodePublisherCallback(void *context_)
{
  cRosErrCodePack ret_err;
  ProviderContext *context = (ProviderContext *)context_;

  ret_err = CROS_SUCCESS_ERR_PACK; // Default return value

  if(cRosMessageQueueUsage(context->msg_queue) > 0) // An inmediate message is waiting to be sent
  {
    if(cRosMessageQueueExtract(context->msg_queue, context->outgoing) != 0)
      ret_err = CROS_EXTRACT_MSG_INT_ERR;
  }
  else // It is time to send a periodic message
  {
    CallbackResponse ret_cb;

    // Cast to the appropriate public api callback and invoke it on the user context
    PublisherApiCallback publisher_user_callback = (PublisherApiCallback)context->api_callback;
    if(publisher_user_callback != NULL)
    {
      ret_cb = publisher_user_callback(context->outgoing, context->context); // Use the callback if it is available and no msg is waiting in the queue
      if(ret_cb != 0)
        ret_err = CROS_TOP_PUB_CALLBACK_ERR;
    }
  }

  if(ret_err != CROS_SUCCESS_ERR_PACK)
    cRosPrintErrCodePack(ret_err, "cRosNodePublisherCallback() failed encoding the packet to send");

  return ret_err;
}

cRosErrCodePack cRosNodeSubscriberCallback(void *context_)
{
  cRosErrCodePack ret_err;
  ProviderContext *context = (ProviderContext *)context_;
  cRosMessageQueueAdd(context->msg_queue, context->incoming);

  // Cast to the appropriate public api callback and invoke it on the user context
  SubscriberApiCallback subs_user_callback_fn = (SubscriberApiCallback)context->api_callback;
  if(subs_user_callback_fn != NULL)
  {
    CallbackResponse ret_cb = subs_user_callback_fn(context->incoming, context->context);
    if(ret_cb == 0)
      ret_err = CROS_SUCCESS_ERR_PACK;
    else
      ret_err = CROS_TOP_SUB_CALLBACK_ERR;
  }
  else
    ret_err = CROS_SUCCESS_ERR_PACK;

  return ret_err;
}

cRosErrCodePack cRosNodeServiceCallerCallback(int call_resp_flag, void* contex_)
{
  cRosErrCodePack ret_err;
  CallbackResponse ret_cb;
  ServiceCallerApiCallback svc_call_user_callback_fn;
  ProviderContext *context = (ProviderContext *)contex_;

  svc_call_user_callback_fn = (ServiceCallerApiCallback)context->api_callback;

  if(call_resp_flag) // Process service response
  {
    // If msg_queue is empty, this is a periodic call (send msg response to callback fn), otherwise this is a non-periodic call (put msg response in the queue)
    if(cRosMessageQueueUsage(context->msg_queue) == 0) // Periodic service call
    {
      if(svc_call_user_callback_fn != NULL)
      {
        ret_cb = svc_call_user_callback_fn(context->outgoing, context->incoming, call_resp_flag, context->context);
        if(ret_cb != 0) // The callback indicated and error in the return value when processing the service response
          ret_err=CROS_SVC_RES_CALLBACK_ERR;
        else
          ret_err = CROS_SUCCESS_ERR_PACK;
      }
      else
        ret_err = CROS_SUCCESS_ERR_PACK;
    }
    else // Non-periodic service call
    {
      if(cRosMessageQueueAdd(context->msg_queue, context->incoming) == 0) // Add response msg to the queue (in case the svc was called non periodically)
        ret_err = CROS_SUCCESS_ERR_PACK; // A new message has been aded to the queue to later store the received response
      else
        ret_err = CROS_MEM_ALLOC_ERR;
    }
    if(ret_err != CROS_SUCCESS_ERR_PACK)
      cRosPrintErrCodePack(ret_err, "cRosNodeServiceCallerCallback() failed decoding the received service response packet");
  }
  else // Generate service request
  {
    // If msg_queue is empty, this is a periodic call (get the msg request from the callback fn), otherwise this is a non-periodic call (get msg request from the queue)
    if(cRosMessageQueueUsage(context->msg_queue) == 0) // Periodic service call
    {
      if(svc_call_user_callback_fn != NULL)
      {
        ret_cb = svc_call_user_callback_fn(context->outgoing, context->incoming, call_resp_flag, context->context);
        if(ret_cb == 0) // The callback returned success when generating the service request: send the service request
          ret_err = CROS_SUCCESS_ERR_PACK;
        else // The callback indicated and error in the return value when generating the service request
          ret_err = CROS_SVC_REQ_CALLBACK_ERR;
      }
      else
        ret_err = CROS_SUCCESS_ERR_PACK;
    }
    else // Non-periodic service call
      ret_err = (cRosMessageQueueGet(context->msg_queue, context->outgoing) == 0)?CROS_SUCCESS_ERR_PACK:CROS_EXTRACT_MSG_INT_ERR; // Copy the message request from the queue msg

    if(ret_err != CROS_SUCCESS_ERR_PACK)
      cRosPrintErrCodePack(ret_err, "cRosNodeServiceCallerCallback() failed getting the service request packet");
  }

  return ret_err;
}

cRosErrCodePack cRosNodeServiceProviderCallback(void *context_)
{
  cRosErrCodePack ret_err;
  ProviderContext *context = (ProviderContext *)context_;

  ServiceProviderApiCallback serviceProviderApiCallback = (ServiceProviderApiCallback)context->api_callback;
  CallbackResponse ret_cb = serviceProviderApiCallback(context->incoming, context->outgoing, context->context);

  if(ret_cb == 0)
    ret_err = CROS_SUCCESS_ERR_PACK;
  else
    ret_err = CROS_SVC_SER_CALLBACK_ERR;

  return ret_err;
}

void cRosNodeStatusCallback(CrosNodeStatusUsr *status, void* context_)
{
  ProviderContext *context = (ProviderContext *)context_;
  if(context->status_api_callback != NULL) // If the application defined a status callback function, call it
    context->status_api_callback(status, context->context);
}

cRosErrCodePack cRosApiRegisterServiceCaller(CrosNode *node, const char *service_name, const char *service_type, int loop_period,
                                   ServiceCallerApiCallback callback, NodeStatusApiCallback status_callback, void *context, int persistent, int tcp_nodelay, int *svcidx_ptr)
{
  cRosErrCodePack ret_err;
  char path[OS_MAX_PATH];
  ProviderContext *nodeContext = NULL;
  int svcidx;

  if(loop_period >= 0 && callback == NULL)
    return CROS_BAD_PARAM_ERR;

  getSrvFilePath(node, path, OS_MAX_PATH, service_type);
  ret_err = newProviderContext(path, CROS_SERVICE_CALLER, &nodeContext);
  if (ret_err == CROS_SUCCESS_ERR_PACK)
  {
    nodeContext->api_callback = callback;
    nodeContext->status_api_callback = status_callback;
    nodeContext->context = context;

    // NB: Pass the private ProviderContext to the private api, not the user context
    svcidx = cRosNodeRegisterServiceCaller(node, nodeContext->message_definition, service_name, service_type, nodeContext->md5sum,
                                           loop_period, nodeContext, persistent, tcp_nodelay);
    if(svcidx >= 0) // Success
    {
      if(svcidx_ptr != NULL)
        *svcidx_ptr = svcidx; // Return the index of the created service caller
      nodeContext->msg_queue = &node->service_callers[svcidx].msg_queue;
    }
    else
      ret_err=CROS_MEM_ALLOC_ERR;
  }
  return ret_err;
}

void cRosApiReleaseServiceCaller(CrosNode *node, int svcidx)
{
  ServiceCallerNode *svc = &node->service_callers[svcidx];
  ProviderContext *context = (ProviderContext *)svc->context;
  freeProviderContext(context);
  cRosNodeReleaseServiceCaller(svc);
}

cRosErrCodePack cRosApiRegisterServiceProvider(CrosNode *node, const char *service_name, const char *service_type,
                                   ServiceProviderApiCallback callback, NodeStatusApiCallback status_callback, void *context, int *svcidx_ptr)
{
  cRosErrCodePack ret_err;
  char path[OS_MAX_PATH];
  ProviderContext *nodeContext = NULL;
  int svcidx;

  if (callback == NULL)
    return CROS_BAD_PARAM_ERR;

  getSrvFilePath(node, path, OS_MAX_PATH, service_type);
  ret_err = newProviderContext(path, CROS_SERVICE_PROVIDER, &nodeContext);
  if (ret_err == CROS_SUCCESS_ERR_PACK)
  {
    nodeContext->api_callback = callback;
    nodeContext->status_api_callback = status_callback;
    nodeContext->context = context;

    // NB: Pass the private ProviderContext to the private api, not the user context
    svcidx = cRosNodeRegisterServiceProvider(node, service_name, service_type, nodeContext->md5sum, nodeContext);
    if(svcidx >= 0) // Success
    {
        if(svcidx_ptr != NULL)
            *svcidx_ptr = svcidx; // Return the index of the created service provider
    }
    else
        ret_err=CROS_MEM_ALLOC_ERR;
  }
  return ret_err;
}

cRosErrCodePack cRosApiUnregisterServiceProvider(CrosNode *node, int svcidx)
{
  int ret_err;
  if (svcidx < 0 || svcidx >= CN_MAX_SERVICE_PROVIDERS)
    return CROS_BAD_PARAM_ERR;

  ServiceProviderNode *service = &node->service_providers[svcidx];
  if (service->service_name == NULL)
    return CROS_TOPIC_SUB_IND_ERR;

  ret_err = cRosNodeUnregisterServiceProvider(node, svcidx);

  return (ret_err != -1)? CROS_SUCCESS_ERR_PACK: CROS_UNSPECIFIED_ERR;
}

void cRosApiReleaseServiceProvider(CrosNode *node, int svcidx)
{
  ServiceProviderNode *svc = &node->service_providers[svcidx];
  ProviderContext *context = (ProviderContext *)svc->context;
  freeProviderContext(context);
  cRosNodeReleaseServiceProvider(svc);
}

cRosErrCodePack cRosApiRegisterSubscriber(CrosNode *node, const char *topic_name, const char *topic_type,
                              SubscriberApiCallback callback, NodeStatusApiCallback status_callback, void *context, int tcp_nodelay, int *subidx_ptr)
{
  cRosErrCodePack ret_err;
  char path[OS_MAX_PATH];
  ProviderContext *nodeContext = NULL;
  int subidx;

  cRosGetMsgFilePath(node, path, OS_MAX_PATH, topic_type);
  ret_err = newProviderContext(path, CROS_SUBSCRIBER, &nodeContext);
  if (ret_err == CROS_SUCCESS_ERR_PACK)
  {
    nodeContext->api_callback = callback;
    nodeContext->status_api_callback = status_callback;
    nodeContext->context = context;

  // NB: Pass the private ProviderContext to the private api, not the user context
    subidx = cRosNodeRegisterSubscriber(node, nodeContext->message_definition, topic_name, topic_type,
                                  nodeContext->md5sum, nodeContext, tcp_nodelay);
    if(subidx >= 0) // Success
    {
      nodeContext->msg_queue = &node->subs[subidx].msg_queue; // Allow the callback functions to access the msg queue
      if(subidx_ptr != NULL)
        *subidx_ptr = subidx; // Return the index of the created service caller
    }
    else
      ret_err=CROS_MEM_ALLOC_ERR;
  }
  return ret_err;
}

cRosErrCodePack cRosApiUnregisterSubscriber(CrosNode *node, int subidx)
{
  int ret_err;
  if (subidx < 0 || subidx >= CN_MAX_SUBSCRIBED_TOPICS)
    return CROS_BAD_PARAM_ERR;

  SubscriberNode *sub = &node->subs[subidx];
  if (sub->topic_name == NULL)
    return CROS_TOPIC_SUB_IND_ERR;

  ret_err = cRosNodeUnregisterSubscriber(node, subidx);

  return (ret_err != -1)? CROS_SUCCESS_ERR_PACK: CROS_UNSPECIFIED_ERR;
}

void cRosApiReleaseSubscriber(CrosNode *node, int subidx)
{
  SubscriberNode *sub = &node->subs[subidx];
  ProviderContext *context = (ProviderContext *)sub->context;
  freeProviderContext(context);
  cRosNodeReleaseSubscriber(sub);
}

cRosErrCodePack cRosApiRegisterPublisher(CrosNode *node, const char *topic_name, const char *topic_type, int loop_period,
                             PublisherApiCallback callback, NodeStatusApiCallback status_callback, void *context, int *pubidx_ptr)
{
  cRosErrCodePack ret_err;
  char path[OS_MAX_PATH];
  ProviderContext *nodeContext = NULL;
  int pubidx;

  if(loop_period >= 0 && callback == NULL)
    return CROS_BAD_PARAM_ERR;

  cRosGetMsgFilePath(node, path, OS_MAX_PATH, topic_type);
  ret_err = newProviderContext(path, CROS_PUBLISHER, &nodeContext);
  if (ret_err == CROS_SUCCESS_ERR_PACK)
  {
    nodeContext->api_callback = callback;
    nodeContext->status_api_callback = status_callback;
    nodeContext->context = context;

    // NB: Pass the private ProviderContext to the private api, not the user context
    pubidx = cRosNodeRegisterPublisher(node, nodeContext->message_definition, topic_name, topic_type,
                                  nodeContext->md5sum, loop_period, nodeContext);
    if(pubidx >= 0) // Success
    {
      // Allow the callback functions to access the msg queue and send-now flag
      nodeContext->msg_queue = &node->pubs[pubidx].msg_queue;
      if(pubidx_ptr != NULL)
        *pubidx_ptr = pubidx; // Return the index of the created service caller
    }
    else
      ret_err=CROS_MEM_ALLOC_ERR;
  }
  return ret_err;
}

cRosErrCodePack cRosApiUnregisterPublisher(CrosNode *node, int pubidx)
{
  int ret_err;
  if (pubidx < 0 || pubidx >= CN_MAX_PUBLISHED_TOPICS)
    return CROS_BAD_PARAM_ERR;

  PublisherNode *pub = &node->pubs[pubidx];
  if (pub->topic_name == NULL)
    return CROS_TOPIC_PUB_IND_ERR;

  ret_err = cRosNodeUnregisterPublisher(node, pubidx);

  return (ret_err != -1)? CROS_SUCCESS_ERR_PACK: CROS_UNSPECIFIED_ERR;
}

void cRosApiReleasePublisher(CrosNode *node, int pubidx)
{
  PublisherNode *pub = &node->pubs[pubidx];
  ProviderContext *context = (ProviderContext *)pub->context;
  freeProviderContext(context);
  cRosNodeReleasePublisher(pub);
}

cRosErrCodePack cRosApicRosApiLookupNode(CrosNode *node, const char *node_name, LookupNodeCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApicRosApiLookupNode() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_LOOKUP_NODE;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchLookupNodeResult;
  call->free_result_callback = (FreeResultCallback)freeLookupNodeResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, node_name);

  caller_id = enqueueMasterApiCall(node, call);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiGetPublishedTopics(CrosNode *node, const char *subgraph, GetPublishedTopicsCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetPublishedTopics() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_GET_PUBLISHED_TOPICS;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetPublishedTopicsResult;
  call->free_result_callback = (FreeResultCallback)freeGetPublishedTopicsResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, subgraph == NULL ? "" : subgraph);

  caller_id = enqueueMasterApiCall(node, call);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiGetTopicTypes(CrosNode *node, GetTopicTypesCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetTopicTypes() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_GET_TOPIC_TYPES;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetTopicTypesResult;
  call->free_result_callback = (FreeResultCallback)freeGetTopicTypesResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  caller_id = enqueueMasterApiCall(node, call);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiGetSystemState(CrosNode *node, GetSystemStateCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetSystemState() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_GET_SYSTEM_STATE;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetSystemStateResult;
  call->free_result_callback = (FreeResultCallback)freeGetSystemStateResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  caller_id = enqueueMasterApiCall(node, call);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiGetUri(CrosNode *node, GetUriCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetUri() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_GET_URI;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetUriResult;
  call->free_result_callback = (FreeResultCallback)freeGetUriResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  caller_id = enqueueMasterApiCall(node, call);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiLookupService(CrosNode *node, const char *service, LookupServiceCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiLookupService() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_LOOKUP_SERVICE;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchLookupServiceResult;
  call->free_result_callback = (FreeResultCallback)freeLookupServiceResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, service);

  caller_id = enqueueMasterApiCall(node, call);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiGetBusStats(CrosNode *node, const char* host, int port,
                       GetBusStatsCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetBusStats() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_GET_BUS_STATS;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetBusStatsResult;
  call->free_result_callback = (FreeResultCallback)freeGetBusStatsResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  caller_id = enqueueSlaveApiCall(node, call, host, port);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiGetBusInfo(CrosNode *node, const char* host, int port,
                      GetBusInfoCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetBusInfo() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_GET_BUS_INFO;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetBusInfoResult;
  call->free_result_callback = (FreeResultCallback)freeGetBusInfoResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  caller_id = enqueueSlaveApiCall(node, call, host, port);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiGetMasterUri(CrosNode *node, const char* host, int port,
                        GetMasterUriCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_GET_MASTER_URI;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetMasterUriResult;
  call->free_result_callback = (FreeResultCallback)freeGetMasterUriResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  caller_id = enqueueSlaveApiCall(node, call, host, port);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiShutdown(CrosNode *node, const char* host, int port, const char *msg,
                    GetMasterUriCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiShutdown() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_SHUTDOWN;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchShutdownResult;
  call->free_result_callback = (FreeResultCallback)freeShutdownResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, msg == NULL ? "" : msg);

  caller_id = enqueueSlaveApiCall(node, call, host, port);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiGetPid(CrosNode *node, const char* host, int port,
                  GetPidCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetPid() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_GET_PID;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetPidResult;
  call->free_result_callback = (FreeResultCallback)freeGetPidResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  caller_id = enqueueSlaveApiCall(node, call, host, port);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiGetSubscriptions(CrosNode *node, const char* host, int port,
                            GetSubscriptionsCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_GET_SUBSCRIPTIONS;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetSubscriptionsResult;
  call->free_result_callback = (FreeResultCallback)freeGetSubscriptionsResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  caller_id = enqueueSlaveApiCall(node, call, host, port);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiGetPublications(CrosNode *node, const char* host, int port,
                    GetSubscriptionsCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetPublications() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_GET_PUBLICATIONS;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetPublicationsResult;
  call->free_result_callback = (FreeResultCallback)freeGetPublicationsResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  caller_id = enqueueSlaveApiCall(node, call, host, port);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiDeleteParam(CrosNode *node, const char *key, DeleteParamCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;

  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiDeleteParam() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_DELETE_PARAM;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchDeleteParamResult;
  call->free_result_callback = (FreeResultCallback)freeDeleteParamResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, key);

  caller_id=enqueueMasterApiCall(node, call);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiSetParam(CrosNode *node, const char *key, XmlrpcParam *value, SetParamCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id, vec_size;

  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiSetParam() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  XmlrpcParam param;
  int rc = xmlrpcParamCopy(&param, value);
  if (rc == -1)
  {
    PRINT_ERROR ( "cRosApiSetParam() : Can't allocate memory\n");
    freeRosApiCall(call);
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_SET_PARAM;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchSetParamResult;
  call->free_result_callback = (FreeResultCallback)freeSetParamResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, key);
  vec_size = xmlrpcParamVectorPushBack(&call->params, &param);
  if (vec_size == -1)
  {
    freeRosApiCall(call);
    xmlrpcParamRelease(&param);
    return CROS_MEM_ALLOC_ERR;
  }

  caller_id=enqueueMasterApiCall(node, call);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiGetParam(CrosNode *node, const char *key, GetParamCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;

  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetParam() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_GET_PARAM;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetParamResult;
  call->free_result_callback = (FreeResultCallback)freeGetParamResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, key);

  caller_id = enqueueMasterApiCall(node, call);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiSearchParam(CrosNode *node, const char *key, SearchParamCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;

  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiSearchParam() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_SEARCH_PARAM;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchSearchParamResult;
  call->free_result_callback = (FreeResultCallback)freeSearchParamResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, key);

  caller_id = enqueueMasterApiCall(node, call);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiHasParam(CrosNode *node, const char *key, HasParamCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;

  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiHasParam() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_HAS_PARAM;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchHasParamResult;
  call->free_result_callback = (FreeResultCallback)freeHasParamResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);
  xmlrpcParamVectorPushBackString(&call->params, key);

  caller_id = enqueueMasterApiCall(node, call);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosApiGetParamNames(CrosNode *node, GetParamNamesCallback callback, void *context, int *caller_id_ptr)
{
  int caller_id;
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosApiGetParamNames() : Can't allocate memory\n");
    return CROS_MEM_ALLOC_ERR;
  }

  call->method = CROS_API_GET_PARAM_NAMES;
  call->result_callback = (ResultCallback)callback;
  call->context_data = context;
  call->fetch_result_callback = (FetchResultCallback)fetchGetParamNamesResult;
  call->free_result_callback = (FreeResultCallback)freeGetParamNamesResult;

  xmlrpcParamVectorPushBackString(&call->params, node->name);

  caller_id = enqueueMasterApiCall(node, call);
  if(caller_id_ptr != NULL)
    *caller_id_ptr = caller_id;

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

XmlrpcParam *GetMethodResponseStatus(XmlrpcParamVector *response, int *status_code, char **status_msg)
{
  XmlrpcParam *resp_param;

  resp_param = xmlrpcParamVectorAt(response, 0);
  if (resp_param != NULL)
  {
    XmlrpcParam *status_code_param;
    status_code_param = xmlrpcParamArrayGetParamAt(resp_param, 0); // resp_param must be a vector or structure in order to be accessed by index
    if (status_code_param != NULL)
    {
      if( status_code_param->type == XMLRPC_PARAM_INT )
      {
        XmlrpcParam *status_msg_param;

        *status_code = status_code_param->data.as_int;
        status_msg_param = xmlrpcParamArrayGetParamAt(resp_param, 1);
        if (status_msg_param != NULL)
        {
          if( status_msg_param->type == XMLRPC_PARAM_STRING )
            *status_msg = strdup(status_msg_param->data.as_string);
          else
            *status_msg = NULL; // status_msg = NULL: No human-readable string describing the returned status has been found
        }
        else
          *status_msg = NULL; // The array or structure returned in the ROS master response does not contain a second element (the human-readable string describing the status)
      }
      else
      {
        PRINT_ERROR ( "GetMethodResponseStatus() : The status code integer cannot be found in the responsed returned by the ROS master.\n");
        resp_param = NULL;
      }
    }
    else
    {
      PRINT_ERROR ( "GetMethodResponseStatus() : The responsed returned by the ROS master does not contain a non-empty array or structure.\n");
      resp_param = NULL;
    }
  }
  else
    PRINT_ERROR ( "GetMethodResponseStatus() : The ROS master returned an XML response with an unexpected format.\n");

  return(resp_param);
}

LookupNodeResult * fetchLookupNodeResult(XmlrpcParamVector *response)
{
  LookupNodeResult *ret = (LookupNodeResult *)calloc(1, sizeof(LookupNodeResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchLookupNodeResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeLookupNodeResult(ret);
    return NULL;
  }

  XmlrpcParam* uri = xmlrpcParamArrayGetParamAt(array, 2);
  ret->uri = strdup(uri->data.as_string);
  if (ret->uri == NULL)
  {
    freeLookupNodeResult(ret);
    return NULL;
  }

  return ret;
}

GetPublishedTopicsResult * fetchGetPublishedTopicsResult(XmlrpcParamVector *response)
{
  int it;

  GetPublishedTopicsResult *ret = (GetPublishedTopicsResult *)calloc(1, sizeof(GetPublishedTopicsResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchGetPublishedTopicsResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeGetPublishedTopicsResult(ret);
    return NULL;
  }

  XmlrpcParam* topics = xmlrpcParamArrayGetParamAt(array, 2);
  ret->topics = (struct TopicTypePair *)calloc(topics->array_n_elem, sizeof(struct TopicTypePair));
  if (ret->topics == NULL)
    goto clean;
  ret->topic_count = topics->array_n_elem;

  for (it = 0; it < topics->array_n_elem; it++)
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
  int it;

  GetTopicTypesResult *ret = (GetTopicTypesResult *)calloc(1, sizeof(GetTopicTypesResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchGetTopicTypesResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeGetTopicTypesResult(ret);
    return NULL;
  }

  XmlrpcParam* topics = xmlrpcParamArrayGetParamAt(array, 2);
  ret->topics = (struct TopicTypePair *)calloc(topics->array_n_elem, sizeof(struct TopicTypePair));
  if (ret->topics == NULL)
    goto clean;
  ret->topic_count = topics->array_n_elem;

  for (it = 0; it < topics->array_n_elem; it++)
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
  int it1;

  GetSystemStateResult *ret = (GetSystemStateResult *)calloc(1, sizeof(GetSystemStateResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchGetSystemStateResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeGetSystemStateResult(ret);
    return NULL;
  }

  if (array->array_n_elem < 3)
    return ret;

  XmlrpcParam* publishers = xmlrpcParamArrayGetParamAt(array, 2);
  ret->publishers = (struct ProviderState *)calloc(publishers->array_n_elem, sizeof(struct ProviderState));
  if (ret->publishers == NULL)
    goto clean;
  ret->pub_count = publishers->array_n_elem;

  for (it1 = 0; it1 < publishers->array_n_elem; it1++)
  {
    int it2;

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

    for (it2 = 0; it2 < users_xml->array_n_elem; it2++)
    {
      char *user = state->users[it2];
      XmlrpcParam* user_xml = xmlrpcParamArrayGetParamAt(users_xml, it2);
      user = (char *)malloc(strlen(user_xml->data.as_string) + 1);
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
    int it2;

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

    for (it2 = 0; it2 < users_xml->array_n_elem; it2++)
    {
      char *user = state->users[it2];
      XmlrpcParam* user_xml = xmlrpcParamArrayGetParamAt(users_xml, it2);
      user = (char *)malloc(strlen(user_xml->data.as_string) + 1);
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
    int it2;

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

    for (it2 = 0; it2 < users_xml->array_n_elem; it2++)
    {
      char *user = state->users[it2];
      XmlrpcParam* user_xml = xmlrpcParamArrayGetParamAt(users_xml, it2);
      user = (char *)malloc(strlen(user_xml->data.as_string) + 1);
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

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchGetUriResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeGetUriResult(ret);
    return NULL;
  }

  XmlrpcParam* uri = xmlrpcParamArrayGetParamAt(array, 2);
  ret->master_uri = strdup(uri->data.as_string);
  if (ret == NULL)
  {
    freeGetUriResult(ret);
    return NULL;
  }

  return ret;
}

LookupServiceResult * fetchLookupServiceResult(XmlrpcParamVector *response)
{
  LookupServiceResult *ret = (LookupServiceResult *)calloc(1, sizeof(LookupServiceResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchLookupServiceResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeLookupServiceResult(ret);
    return NULL;
  }

  XmlrpcParam* service = xmlrpcParamArrayGetParamAt(array, 2);
  ret->service_result = strdup(service->data.as_string);
  if (ret == NULL)
  {
    freeLookupServiceResult(ret);
    return NULL;
  }

  return ret;
}

GetBusStatsResult * fetchGetBusStatsResult(XmlrpcParamVector *response)
{
  int it1;

  GetBusStatsResult *ret = (GetBusStatsResult *)calloc(1, sizeof(GetBusStatsResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchGetBusStatsResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeGetBusStatsResult(ret);
    return NULL;
  }

  if (array->array_n_elem < 3)
    return ret;

  XmlrpcParam* pubs_stats = xmlrpcParamArrayGetParamAt(array, 2);
  ret->stats.pub_stats = (struct TopicPubStats *)calloc(pubs_stats->array_n_elem, sizeof(struct TopicPubStats));
  if (ret->stats.pub_stats == NULL)
    goto clean;
  ret->stats.pub_stats_count = pubs_stats->array_n_elem;

  for (it1 = 0; it1 < pubs_stats->array_n_elem; it1++)
  {
    int it2;

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

    for (it2 = 0; it2 < pub_datas->array_n_elem; it2++)
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
    int it2;

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

    for (it2 = 0; it2 < sub_datas->array_n_elem; it2++)
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
  int it;

  GetBusInfoResult *ret = (GetBusInfoResult *)calloc(1, sizeof(GetBusInfoResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchGetBusInfoResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeGetBusInfoResult(ret);
    return NULL;
  }

  if (array->array_n_elem < 3)
    return ret;

  XmlrpcParam* businfos = xmlrpcParamArrayGetParamAt(array, 2);
  ret->bus_infos = (struct BusInfo *)calloc(businfos->array_n_elem, sizeof(struct BusInfo));
  if (ret->bus_infos == NULL)
    goto clean;
  ret->bus_infos_count = businfos->array_n_elem;

  for (it = 0; it < businfos->array_n_elem; it++)
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

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchGetMasterUriResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeGetMasterUriResult(ret);
    return NULL;
  }

  XmlrpcParam* uri = xmlrpcParamArrayGetParamAt(array, 2);
  ret->master_uri = strdup(uri->data.as_string);
  if (ret->master_uri == NULL)
  {
    freeGetMasterUriResult(ret);
    return NULL;
  }

  return ret;
}

ShutdownResult * fetchShutdownResult(XmlrpcParamVector *response)
{
  ShutdownResult *ret = (ShutdownResult *)calloc(1, sizeof(ShutdownResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchShutdownResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeShutdownResult(ret);
    return NULL;
  }

  XmlrpcParam *ignore = xmlrpcParamArrayGetParamAt(array, 2);
  if(ignore != NULL)
    ret->ignore = ignore->data.as_bool;
  else
  {
    PRINT_ERROR ( "fetchShutdownResult() : The ROS master response does not contain the 'ignore' parameter.\n" );
    freeShutdownResult(ret);
    return NULL;
  }

  return ret;
}

GetPidResult * fetchGetPidResult(XmlrpcParamVector *response)
{
  GetPidResult *ret = (GetPidResult *)calloc(1, sizeof(GetPidResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchGetPidResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeGetPidResult(ret);
    return NULL;
  }

  XmlrpcParam* roscore_pid_param = xmlrpcParamArrayGetParamAt(array, 2);
  if(roscore_pid_param != NULL)
    ret->server_process_pid = roscore_pid_param->data.as_int;
  else
  {
    PRINT_ERROR ( "fetchGetPidResult() : The ROS master response does not contain the requested PID.\n" );
    freeGetPidResult(ret);
    return NULL;
  }

  return ret;
}

GetSubscriptionsResult * fetchGetSubscriptionsResult(XmlrpcParamVector *response)
{
  int it;

  GetSubscriptionsResult *ret = (GetSubscriptionsResult *)calloc(1, sizeof(GetSubscriptionsResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchGetSubscriptionsResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeGetSubscriptionsResult(ret);
    return NULL;
  }

  XmlrpcParam* topics = xmlrpcParamArrayGetParamAt(array, 2);
  ret->topic_list = (struct TopicTypePair *)calloc(topics->array_n_elem, sizeof(struct TopicTypePair));
  if (ret->topic_list == NULL)
    goto clean;
  ret->topic_count = topics->array_n_elem;

  for (it = 0; it < topics->array_n_elem; it++)
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
  int it;

  GetPublicationsResult *ret = (GetPublicationsResult *)calloc(1, sizeof(GetPublicationsResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchGetPublicationsResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeGetPublicationsResult(ret);
    return NULL;
  }

  XmlrpcParam* topics = xmlrpcParamArrayGetParamAt(array, 2);
  ret->topic_list = (struct TopicTypePair *)calloc(topics->array_n_elem, sizeof(struct TopicTypePair));
  if (ret->topic_list == NULL)
    goto clean;
  ret->topic_count = topics->array_n_elem;

  for (it = 0; it < topics->array_n_elem; it++)
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

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchDeleteParamResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeDeleteParamResult(ret);
    return NULL;
  }

  XmlrpcParam* ignore = xmlrpcParamArrayGetParamAt(array, 2);
  if(ignore != NULL)
    ret->ignore = ignore->data.as_bool;
  else
  {
    PRINT_ERROR ( "fetchDeleteParamResult() : The ROS master response does not contain the 'ignore' parameter.\n" );
    freeDeleteParamResult(ret);
    return NULL;
  }

  return ret;
}

static SetParamResult * fetchSetParamResult(XmlrpcParamVector *response)
{
  SetParamResult *ret = (SetParamResult *)calloc(1, sizeof(SetParamResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchSetParamResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeSetParamResult(ret);
    return NULL;
  }

  XmlrpcParam* ignore = xmlrpcParamArrayGetParamAt(array, 2);
  if(ignore != NULL)
    ret->ignore = ignore->data.as_bool;
  else
  {
    PRINT_ERROR ( "fetchSetParamResult() : The ROS master response does not contain the 'ignore' parameter.\n" );
    freeSetParamResult(ret);
    return NULL;
  }

  return ret;
}

static GetParamResult * fetchGetParamResult(XmlrpcParamVector *response)
{
  GetParamResult *ret = (GetParamResult *)calloc(1, sizeof(GetParamResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchGetParamResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeGetParamResult(ret);
    return NULL;
  }

  XmlrpcParam* value = xmlrpcParamArrayGetParamAt(array, 2);
  if( value != NULL )
    ret->value = xmlrpcParamClone(value);
  else
  {
    if(ret->code != 1) // Only if ret->code is not 1, parameterValue can be ignored.
    {
      PRINT_ERROR ( "fetchGetParamResult() : The ROS master response does not contain the 'parameterValue' parameter.\n" );
      freeGetParamResult(ret);
      return NULL;
    }
  }

  return ret;
}

static SearchParamResult * fetchSearchParamResult(XmlrpcParamVector *response)
{
  SearchParamResult *ret = (SearchParamResult *)calloc(1, sizeof(SearchParamResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchSearchParamResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeSearchParamResult(ret);
    return NULL;
  }

  XmlrpcParam* found_key = xmlrpcParamArrayGetParamAt(array, 2);
  if(found_key != NULL)
  {
    ret->found_key = strdup(found_key->data.as_string);
    if(ret->found_key == NULL)
    {
      PRINT_ERROR ( "fetchSearchParamResult() : Error allocating memory for the 'foundKey' parameter.\n" );
      freeSearchParamResult(ret);
      return NULL;
    }
  }
  else
  {
    PRINT_ERROR ( "fetchSearchParamResult() : The ROS master response does not contain the 'foundKey' parameter.\n" );
    freeSearchParamResult(ret);
    return NULL;
  }

  return ret;
}

static HasParamResult * fetchHasParamResult(XmlrpcParamVector *response)
{
  HasParamResult *ret = (HasParamResult *)calloc(1, sizeof(HasParamResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchHasParamResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeHasParamResult(ret);
    return NULL;
  }

  XmlrpcParam* has_param = xmlrpcParamArrayGetParamAt(array, 2);
  if(has_param != NULL)
    ret->has_param = has_param->data.as_bool;
  else
  {
    PRINT_ERROR ( "fetchHasParamResult() : The ROS master response does not contain the 'has_param' parameter.\n" );
    freeHasParamResult(ret);
    return NULL;
  }

  return ret;
}

static GetParamNamesResult * fetchGetParamNamesResult(XmlrpcParamVector *response)
{
  int it;

  GetParamNamesResult *ret = (GetParamNamesResult *)calloc(1, sizeof(GetParamNamesResult));
  if (ret == NULL)
    return NULL;

  XmlrpcParam *array;
  array = GetMethodResponseStatus(response, &ret->code, &ret->status);
  if (array == NULL)
  {
    free(ret);
    return NULL;
  }
  if(ret->code == -1 || ret->code == 0)
  {
    PRINT_ERROR ( "fetchGetParamNamesResult() : The ROS master returned a (%s) %i status code.\n", (ret->code==-1)?"ERROR":"FAILURE" ,ret->code);
    freeGetParamNamesResult(ret);
    return NULL;
  }

  XmlrpcParam* param_names = xmlrpcParamArrayGetParamAt(array, 2);
  ret->parameter_names = (char **)calloc(param_names->array_n_elem, sizeof(char *));
  if (ret->parameter_names== NULL)
    goto clean;

  for (it = 0; it < param_names->array_n_elem; it++)
  {
    ret->parameter_names[it] = (char *)malloc(strlen(param_names->data.as_array[it].data.as_string) + 1);
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

cRosMessage *cRosApiCreatePublisherMessage(CrosNode *node, int pubidx)
{
  cRosMessage *new_msg;
  ProviderContext *pub_context;
  PublisherNode *pub;

  if (pubidx < 0 || pubidx >= CN_MAX_PUBLISHED_TOPICS)
    return NULL;

  pub = &node->pubs[pubidx];
  if (pub->topic_name == NULL)
    return NULL;

  pub_context = pub->context;
  new_msg = cRosMessageCopy(pub_context->outgoing);

  return new_msg;
}

cRosMessage *cRosApiCreateServiceCallerRequest(CrosNode *node, int svcidx)
{
  cRosMessage *new_msg;
  ServiceCallerNode *svc_caller;
  ProviderContext *caller_context;

  if (svcidx < 0 || svcidx >= CN_MAX_SERVICE_CALLERS)
    return NULL;

  svc_caller = &node->service_callers[svcidx];
  if (svc_caller->service_name == NULL)
    return NULL;

  caller_context = svc_caller->context;
  new_msg = cRosMessageCopy(caller_context->outgoing);

  return new_msg;
}


void freeLookupNodeResult(LookupNodeResult *result)
{
  free(result->status);
  free(result->uri);
  free(result);
}

void freeGetPublishedTopicsResult(GetPublishedTopicsResult *result)
{
  size_t it;

  free(result->status);
  for (it = 0; it < result->topic_count; it++)
  {
    free(result->topics[it].topic);
    free(result->topics[it].type);
  }
  free(result->topics);
  free(result);
}

void freeGetTopicTypesResult(GetTopicTypesResult *result)
{
  size_t it;

  free(result->status);
  for (it = 0; it < result->topic_count; it++)
  {
    free(result->topics[it].topic);
    free(result->topics[it].type);
  }
  free(result->topics);
  free(result);
}

void freeGetSystemStateResult(GetSystemStateResult *result)
{
  size_t it1;

  free(result->status);
  for (it1 = 0; it1 < result->sub_count; it1++)
  {
    size_t it2;

    free(result->publishers[it1].provider_name);
    for (it2 = 0; it2 < result->publishers[it1].user_count; it2++)
      free(result->publishers[it1].users[it2]);
    free(result->publishers[it1].users);
  }
  free(result->publishers);

  for (it1 = 0; it1 < result->sub_count; it1++)
  {
    size_t it2;

    free(result->subscribers[it1].provider_name);
    for (it2 = 0; it2 < result->subscribers[it1].user_count; it2++)
      free(result->subscribers[it1].users[it2]);
    free(result->subscribers[it1].users);
  }
  free(result->subscribers);

  for (it1 = 0; it1 < result->svc_count; it1++)
  {
    size_t it2;

    free(result->service_providers[it1].provider_name);
    for (it2 = 0; it2 < result->service_providers[it1].user_count; it2++)
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
  size_t it1;

  free(result->status);
  for (it1 = 0; it1 < result->stats.pub_stats_count; it1++)
    free(result->stats.pub_stats[it1].topic_name);
  free(result->stats.pub_stats);

  for (it1 = 0; it1 < result->stats.sub_stats_count; it1++)
    free(result->stats.sub_stats[it1].topic_name);
  free(result->stats.sub_stats);

  free(result);
}

void freeGetBusInfoResult(GetBusInfoResult *result)
{
  size_t it;

  free(result->status);
  for (it = 0; it < result->bus_infos_count; it++)
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
  size_t it;

  free(result->status);
  for (it = 0; it < result->topic_count; it++)
  {
    free(result->topic_list[it].topic);
    free(result->topic_list[it].type);
  }
  free(result->topic_list);
  free(result);
}

void freeGetPublicationsResult(GetPublicationsResult *result)
{
  size_t it;

  free(result->status);
  for (it = 0; it < result->topic_count; it++)
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
  size_t it;

  free(result->status);
  for (it = 0; it < result->parameter_count; it++)
    free(result->parameter_names[it]);
  free(result->parameter_names);
  free(result);
}


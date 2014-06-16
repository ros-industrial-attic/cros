#include "cros_node_api.h"
#include "cros_defs.h"
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

int lookupNode(CrosNode *node, const char *node_name, LookupNodeCallback *callback, void *data)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_LOOKUP_NODE;
  call->result_callback = (ResultCallback)callback;
  call->context_data = data;
  call->fetch_result_callback = fetchLookupNodeResult;

  enqueueApiCall(&node->master_api_queue, call);

  return 0;
}

int getPublishedTopics(CrosNode *node, const char *subgraph, GetPublishedTopicsCallback *callback, void *data)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_PUBLISHED_TOPICS;
  call->result_callback = (ResultCallback)callback;
  call->context_data = data;
  call->fetch_result_callback = fetchGetPublishedTopicsResult;

  enqueueApiCall(&node->master_api_queue, call);

  return 0;
}

int getTopicTypes(CrosNode *node, GetTopicTypesCallback *callback, void *data)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_TOPIC_TYPES;
  call->result_callback = (ResultCallback)callback;
  call->context_data = data;
  call->fetch_result_callback = fetchGetTopicTypesResult;

  enqueueApiCall(&node->master_api_queue, call);

  return 0;
}

int getSystemState(CrosNode *node, GetSystemStateCallback *callback, void *data)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_SYSTEM_STATE;
  call->result_callback = (ResultCallback)callback;
  call->context_data = data;
  call->fetch_result_callback = fetchGetSystemStateResult;

  enqueueApiCall(&node->master_api_queue, call);

  return 0;
}

int getUri(CrosNode *node, GetUriCallback *callback, void *data)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_URI;
  call->result_callback = (ResultCallback)callback;
  call->context_data = data;
  call->fetch_result_callback = fetchGetUriResult;

  enqueueApiCall(&node->master_api_queue, call);

  return 0;
}

int lookupService(CrosNode *node, const char *service, LookupServiceCallback *callback, void *data)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_LOOKUP_SERVICE;
  call->result_callback = (ResultCallback)callback;
  call->context_data = data;
  call->fetch_result_callback = fetchLookupServiceResult;

  enqueueApiCall(&node->master_api_queue, call);

  return 0;
}

int getBusStats(CrosNode *node, int client_idx, GetBusStatsCallback *callback, void *data)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_BUS_STATS;
  call->result_callback = (ResultCallback)callback;
  call->context_data = data;
  call->fetch_result_callback = fetchGetBusStatsResult;

  enqueueApiCall(&node->slave_api_queue, call);

  return 0;
}

int getBusInfo(CrosNode *node, int client_idx, GetBusInfoCallback *callback, void *data)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_BUS_INFO;
  call->result_callback = (ResultCallback)callback;
  call->context_data = data;
  call->fetch_result_callback = fetchGetBusInfoResult;

  enqueueApiCall(&node->slave_api_queue, call);

  return 0;
}

int getMasterUri(CrosNode *node, int client_idx, GetMasterUriCallback *callback, void *data)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_MASTER_URI;
  call->result_callback = (ResultCallback)callback;
  call->context_data = data;
  call->fetch_result_callback = fetchGetMasterUriResult;

  enqueueApiCall(&node->slave_api_queue, call);

  return 0;
}

int requestShutdown(CrosNode *node, int client_idx, const char *msg, GetMasterUriCallback *callback, void *data)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_SHUTDOWN;
  call->result_callback = (ResultCallback)callback;
  call->context_data = data;
  call->fetch_result_callback = fetchRequestShutdownResult;

  enqueueApiCall(&node->slave_api_queue, call);

  return 0;
}

int getPid(CrosNode *node, int client_idx, GetPidCallback *callback, void *data)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_PID;
  call->result_callback = (ResultCallback)callback;
  call->context_data = data;
  call->fetch_result_callback = fetchGetPidResult;

  enqueueApiCall(&node->slave_api_queue, call);

  return 0;
}

int getSubscriptions(CrosNode *node, int client_idx, GetSubscriptionsCallback *callback, void *data)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_SUBSCRIPTIONS;
  call->result_callback = (ResultCallback)callback;
  call->context_data = data;
  call->fetch_result_callback = fetchGetSubscriptionsResult;

  enqueueApiCall(&node->slave_api_queue, call);

  return 0;
}

int getPublications(CrosNode *node, int client_idx, GetSubscriptionsCallback *callback, void *data)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_GET_PUBLICATIONS;
  call->result_callback = (ResultCallback)callback;
  call->context_data = data;
  call->fetch_result_callback = fetchGetPublicationsResult;

  enqueueApiCall(&node->slave_api_queue, call);

  return 0;
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

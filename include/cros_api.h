#ifndef _CROS_API_H_
#define _CROS_API_H_

#include "xmlrpc_params.h"
#include "cros_node.h"
#include "cros_message.h"
#include "cros_err_codes.h"

#define CROS_INFINITE_TIMEOUT ~0UL

typedef enum CrosTransportType
{
  CROS_TRANSPORT_TCPROS,
  CROS_TRANSPORT_UPDROS
} CrosTransportType;

typedef enum CrosTransportDirection
{
  CROS_TRANSPORT_DIRECTION_IN,
  CROS_TRANSPORT_DIRECTION_OUT,
  CROS_TRANSPORT_DIRECTION_BOTH
} CrosTransportDirection;

typedef struct LookupNodeResult LookupNodeResult;
typedef struct GetPublishedTopicsResult GetPublishedTopicsResult;
typedef struct GetTopicTypesResult GetTopicTypesResult;
typedef struct GetSystemStateResult GetSystemStateResult;
typedef struct GetUriResult GetUriResult;
typedef struct LookupServiceResult LookupServiceResult;
typedef struct GetBusStatsResult GetBusStatsResult;
typedef struct GetBusInfoResult GetBusInfoResult;
typedef struct GetMasterUriResult GetMasterUriResult;
typedef struct ShutdownResult ShutdownResult;
typedef struct GetPidResult GetPidResult;
typedef struct GetSubscriptionsResult GetSubscriptionsResult;
typedef struct GetPublicationsResult GetPublicationsResult;

struct LookupNodeResult
{
  int code;
  char *status;
  char *uri;
};

struct TopicTypePair;

struct GetPublishedTopicsResult
{
  int code;
  char *status;
  struct TopicTypePair *topics;
  size_t topic_count;
};

struct GetTopicTypesResult
{
  int code;
  char *status;
  struct TopicTypePair *topics;
  size_t topic_count;
};

struct ProviderState;

struct GetSystemStateResult
{
  int code;
  char *status;
  struct ProviderState *publishers;
  size_t pub_count;
  struct ProviderState *subscribers;
  size_t sub_count;
  struct ProviderState *service_providers;
  size_t svc_count;
};

struct GetUriResult
{
  int code;
  char *status;
  char *master_uri;
};

struct LookupServiceResult
{
  int code;
  char *status;
  char *service_result;
};

struct PubConnectionData;

struct TopicPubStats
{
  char *topic_name;
  size_t message_data_sent;
  struct PubConnectionData *datas;
  size_t datas_count;
};

struct SubConnectionData;

struct TopicSubStats
{
  char *topic_name;
  struct SubConnectionData *datas;
  size_t datas_count;
};

struct ServiceStats
{
  size_t num_requests;
  size_t bytes_received;
  size_t bytes_sent;
};

struct BusStats
{
  struct TopicPubStats *pub_stats;
  size_t pub_stats_count;
  struct TopicSubStats *sub_stats;
  size_t sub_stats_count;
  struct ServiceStats service_stats;
};

struct GetBusStatsResult
{
  int code;
  char *status;
  struct BusStats stats;
};

struct BusInfo;

struct GetBusInfoResult
{
  int code;
  char *status;
  struct BusInfo *bus_infos;
  size_t bus_infos_count;
};

struct GetMasterUriResult
{
  int code;
  char *status;
  char *master_uri;
};

struct ShutdownResult
{
  int code;
  char *status;
  int ignore;
};

struct GetPidResult
{
  int code;
  char *status;
  int server_process_pid;
};

struct GetSubscriptionsResult
{
  int code;
  char *status;
  struct TopicTypePair *topic_list;
  size_t topic_count;
};

struct GetPublicationsResult
{
  int code;
  char *status;
  struct TopicTypePair *topic_list;
  size_t topic_count;
};

struct DeleteParamResult
{
  int code;
  char *status;
  int ignore;
};

struct SetParamResult
{
  int code;
  char *status;
  int ignore;
};

struct GetParamResult
{
  int code;
  char *status;
  XmlrpcParam *value;
};

struct SearchParamResult
{
  int code;
  char *status;
  char *found_key;
};

struct HasParamResult
{
  int code;
  char *status;
  int has_param;
};

struct GetParamNamesResult
{
  int code;
  char *status;
  char **parameter_names;
  size_t parameter_count;
};

struct TopicTypePair
{
  char *topic;
  char *type;
};

struct ProviderState
{
  char *provider_name;
  char **users;
  size_t user_count;
};

struct PubConnectionData
{
  int connection_id;
  size_t bytes_sent;
  size_t num_sent;
  int connected;
};

struct SubConnectionData
{
  int connection_id;
  size_t bytes_received;
  int drop_estimate;
  int connected;
};

struct BusInfo
{
  int connectionId;
  int destinationId;
  CrosTransportDirection direction;
  CrosTransportType transport;
  char *topic;
  int connected;
};

typedef struct LookupNodeResult LookupNodeResult;
typedef struct GetPublishedTopicsResult GetPublishedTopicsResult;
typedef struct GetTopicTypesResult GetTopicTypesResult;
typedef struct GetSystemStateResult GetSystemStateResult;
typedef struct GetUriResult GetUriResult;
typedef struct LookupServiceResult LookupServiceResult;
typedef struct GetBusStatsResult GetBusStatsResult;
typedef struct GetBusInfoResult GetBusInfoResult;
typedef struct GetMasterUriResult GetMasterUriResult;
typedef struct ShutdownResult ShutdownResult;
typedef struct GetPidResult GetPidResult;
typedef struct GetSubscriptionsResult GetSubscriptionsResult;
typedef struct GetPublicationsResult GetPublicationsResult;
typedef struct DeleteParamResult DeleteParamResult;
typedef struct SetParamResult SetParamResult;
typedef struct GetParamResult GetParamResult;
typedef struct SearchParamResult SearchParamResult;
typedef struct HasParamResult HasParamResult;
typedef struct GetParamNamesResult GetParamNamesResult;

typedef void (*LookupNodeCallback)(int callid, LookupNodeResult *result, void *context);
typedef void (*GetPublishedTopicsCallback)(int callid, GetPublishedTopicsResult *result, void *context);
typedef void (*GetTopicTypesCallback)(int callid, GetTopicTypesResult *result, void *context);
typedef void (*GetSystemStateCallback)(int callid, GetSystemStateResult *result, void *context);
typedef void (*GetUriCallback)(int callid, GetUriResult *result, void *context);
typedef void (*LookupServiceCallback)(int callid, LookupServiceResult *result, void *context);
typedef void (*GetBusStatsCallback)(int callid, GetBusStatsResult *result, void *context);
typedef void (*GetBusInfoCallback)(int callid, GetBusInfoResult *result, void *context);
typedef void (*GetMasterUriCallback)(int callid, GetMasterUriResult *result, void *context);
typedef void (*RequestShutdownCallback)(int callid, ShutdownResult *result, void *context);
typedef void (*GetPidCallback)(int callid, GetPidResult *result, void *context);
typedef void (*GetSubscriptionsCallback)(int callid, GetSubscriptionsResult *result, void *context);
typedef void (*GetPublicationsCallback)(int callid, GetPublicationsResult *result, void *context);
typedef void (*DeleteParamCallback)(int callid, DeleteParamResult *result, void *context);
typedef void (*SetParamCallback)(int callid, SetParamResult *result, void *context);
typedef void (*GetParamCallback)(int callid, GetParamResult *result, void *context);
typedef void (*SearchParamCallback)(int callid, SearchParamResult *result, void *context);
typedef void (*HasParamCallback)(int callid, HasParamResult *result, void *context);
typedef void (*GetParamNamesCallback)(int callid, GetParamNamesResult *result, void *context);

typedef uint8_t CallbackResponse;
typedef CallbackResponse (*ServiceCallerApiCallback)(cRosMessage *request, cRosMessage *response, int call_resp_flag, void *context);
typedef CallbackResponse (*ServiceProviderApiCallback)(cRosMessage *request, cRosMessage *response, void *context);
typedef CallbackResponse (*SubscriberApiCallback)(cRosMessage *message,  void *context);
/*! \brief Application-defined callback function which is called by the library */
typedef CallbackResponse (*PublisherApiCallback)(cRosMessage *message, void *context);

// Transfer data from message buffer (context_) of the Publisher/Service caller to the output ROS packet buffer (buffer)
cRosErrCodePack cRosNodeSerializeOutgoingMessage(DynBuffer *buffer, void *context_);
// Transfer data from packet buffer (buffer) of the Service caller to the input mesage buffer (context_)
cRosErrCodePack cRosNodeDeserializeIncomingPacket(DynBuffer *buffer, void *context_);

// Intermediary functions that call the user callback functions
// context is a structure (object) opaque for the caller function
cRosErrCodePack cRosNodeSubscriberCallback(void *context_);
cRosErrCodePack cRosNodePublisherCallback(void *context_);
cRosErrCodePack cRosNodeServiceCallerCallback(int call_resp_flag, void* contex_);
cRosErrCodePack cRosNodeServiceProviderCallback(void *context_);
void cRosNodeStatusCallback(CrosNodeStatusUsr *status, void* context_);

// Master api: register/unregister methods
cRosErrCodePack cRosApiRegisterServiceCaller(CrosNode *node, const char *service_name, const char *service_type, int loop_period, ServiceCallerApiCallback callback, NodeStatusApiCallback status_callback, void *context, int persistent, int tcp_nodelay, int *svcidx_ptr);
void cRosApiReleaseServiceCaller(CrosNode *node, int svcidx);
cRosErrCodePack cRosApiRegisterServiceProvider(CrosNode *node, const char *service_name, const char *service_type, ServiceProviderApiCallback callback, NodeStatusApiCallback status_callback, void *context, int *svcidx_ptr);
cRosErrCodePack cRosApiUnregisterServiceProvider(CrosNode *node, int svcidx);
void cRosApiReleaseServiceProvider(CrosNode *node, int svcidx);
cRosErrCodePack cRosApiRegisterSubscriber(CrosNode *node, const char *topic_name, const char *topic_type, SubscriberApiCallback callback, NodeStatusApiCallback status_callback, void *context, int tcp_nodelay, int *subidx_ptr);
cRosErrCodePack cRosApiUnregisterSubscriber(CrosNode *node, int subidx);
void cRosApiReleaseSubscriber(CrosNode *node, int subidx);
cRosErrCodePack cRosApiRegisterPublisher(CrosNode *node, const char *topic_name, const char *topic_type, int loop_period, PublisherApiCallback callback, NodeStatusApiCallback status_callback, void *context, int *pubidx_ptr);
cRosErrCodePack cRosApiUnregisterPublisher(CrosNode *node, int pubidx);
void cRosApiReleasePublisher(CrosNode *node, int pubidx);

// Master api: name service and system state
cRosErrCodePack cRosApiLookupNode(CrosNode *node, const char *node_name, LookupNodeCallback callback, void *context, int *caller_id_ptr);
cRosErrCodePack cRosApiGetPublishedTopics(CrosNode *node, const char *subgraph, GetPublishedTopicsCallback callback, void *context, int *caller_id_ptr);
cRosErrCodePack cRosApiGetTopicTypes(CrosNode *node, GetTopicTypesCallback callback, void *context, int *caller_id_ptr);
cRosErrCodePack cRosApiGetSystemState(CrosNode *node, GetSystemStateCallback callback, void *context, int *caller_id_ptr);
cRosErrCodePack cRosApiGetUri(CrosNode *node, GetUriCallback callback, void *context, int *caller_id_ptr);
cRosErrCodePack cRosApiLookupService(CrosNode *node, const char *service, LookupServiceCallback callback, void *context, int *caller_id_ptr);

// Slave API

/*!
 *  \param if host == NULL, it will contact ROS Master
 *  \param So, host and port are only used if host != NULL
 */
cRosErrCodePack cRosApiGetBusStats(CrosNode *node, const char* host, int port, GetBusStatsCallback callback, void *context, int *caller_id_ptr);

/*!
 *  \param if host == NULL, it will contact ROS Master
 *  \param So, host and port are only used if host != NULL
 */
cRosErrCodePack cRosApiGetBusInfo(CrosNode *node, const char* host, int port, GetBusInfoCallback callback, void *context, int *caller_id_ptr);

/*!
 *  \param if host == NULL, it will contact ROS Master
 *  \param So, host and port are only used if host != NULL
 */
cRosErrCodePack cRosApiGetMasterUri(CrosNode *node, const char* host, int port, GetMasterUriCallback callback, void *context, int *caller_id_ptr);

/*!
 *  \param if host == NULL, it will contact ROS Master
 *  \param So, host and port are only used if host != NULL
 */
cRosErrCodePack cRosApiShutdown(CrosNode *node, const char* host, int port, const char *msg, GetMasterUriCallback callback, void *context, int *caller_id_ptr);

/*!
 *  \param if host == NULL, it will contact ROS Master
 *  \param So, host and port are only used if host != NULL
 */
cRosErrCodePack cRosApiGetPid(CrosNode *node, const char* host, int port, GetPidCallback callback, void *context, int *caller_id_ptr);

/*!
 *  \param if host == NULL, it will contact ROS Master
 *  \param So, host and port are only used if host != NULL
 */
cRosErrCodePack cRosApiGetSubscriptions(CrosNode *node, const char* host, int port, GetSubscriptionsCallback callback, void *context, int *caller_id_ptr);

/*!
 *  \param if host == NULL, it will contact ROS Master
 *  \param So, host and port are only used if host != NULL
 */
cRosErrCodePack cRosApiGetPublications(CrosNode *node, const char* host, int port, GetSubscriptionsCallback callback, void *context, int *caller_id_ptr);

// Parameter Server API: subscribe/unsubscribe params
cRosErrCodePack cRosApiSubscribeParam(CrosNode *node, const char *key, NodeStatusApiCallback callback, void *context, int *paramsubidx_ptr);
cRosErrCodePack cRosApiUnsubscribeParam(CrosNode *node, int paramsubidx);

// Parameter Server API: other methods
cRosErrCodePack cRosApiDeleteParam(CrosNode *node, const char *key, DeleteParamCallback callback, void *context, int *caller_id_ptr);
cRosErrCodePack cRosApiSetParam(CrosNode *node, const char *key, XmlrpcParam *value, SetParamCallback callback, void *context, int *caller_id_ptr);
cRosErrCodePack cRosApiGetParam(CrosNode *node, const char *key, GetParamCallback callback, void *context, int *caller_id_ptr);
cRosErrCodePack cRosApiSearchParam(CrosNode *node, const char *key, SearchParamCallback callback, void *context, int *caller_id_ptr);
cRosErrCodePack cRosApiHasParam(CrosNode *node, const char *key, HasParamCallback callback, void *context, int *caller_id_ptr);
cRosErrCodePack cRosApiGetParamNames(CrosNode *node, GetParamNamesCallback callback, void *context, int *caller_id_ptr);

// Message polling
cRosErrCodePack cRosNodeReceiveTopicMsg(CrosNode *node, int subidx, cRosMessage *msg, unsigned char *buff_overflow, unsigned long time_out);
cRosErrCodePack cRosNodeQueueTopicMsg( CrosNode *node, int pubidx, cRosMessage *msg );
cRosErrCodePack cRosNodeSendTopicMsg(CrosNode *node, int pubidx, cRosMessage *msg, unsigned long time_out);
cRosErrCodePack cRosNodeServiceCall(CrosNode *node, int svcidx, cRosMessage *req_msg, cRosMessage *resp_msg, unsigned long time_out);
cRosMessage *cRosApiCreatePublisherMessage(CrosNode *node, int pubidx);
cRosMessage *cRosApiCreateServiceCallerRequest(CrosNode *node, int svcidx);

#endif // _CROS_API_H_

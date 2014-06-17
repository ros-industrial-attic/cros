#ifndef _CROS_NODE_API_H_
#define _CROS_NODE_API_H_

#include "cros_node.h"

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
  char *message_data_sent;
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
  struct TopicPubStats topic_pub_stats;
  struct TopicSubStats topib_sub_stats;
  struct ServiceStats service_stats;
};

struct GetBusStatsResult
{
  int code;
  char *status;
  struct BusStats stats;
};

struct GetBusInfoResult
{
  int code;
  char *status;
};

struct GetMasterUriResult
{
  int code;
  char *status;
  char *master_uri;
};

struct RequestShutdownResult
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

typedef void (*LookupNodeCallback)(LookupNodeResult *result, void *context);
typedef void (*GetPublishedTopicsCallback)(GetPublishedTopicsResult *result, void *context);
typedef void (*GetTopicTypesCallback)(GetTopicTypesResult *result, void *context);
typedef void (*GetSystemStateCallback)(GetSystemStateResult *result, void *context);
typedef void (*GetUriCallback)(GetUriResult *result, void *context);
typedef void (*LookupServiceCallback)(LookupServiceResult *result, void *context);
typedef void (*GetBusStatsCallback)(GetBusStatsResult *result, void *context);
typedef void (*GetBusInfoCallback)(GetBusInfoResult *result, void *context);
typedef void (*GetMasterUriCallback)(GetMasterUriResult *result, void *context);
typedef void (*RequestShutdownCallback)(ShutdownResult *result, void *context);
typedef void (*GetPidCallback)(GetPidResult *result, void *context);
typedef void (*GetSubscriptionsCallback)(GetSubscriptionsResult *result, void *context);
typedef void (*GetPublicationsCallback)(GetPublicationsResult *result, void *context);

typedef struct CrosMessage
{
} CrosMessage;

typedef CallbackResponse (*ServiceProviderApiCallback)(CrosMessage *messageRequest, CrosMessage *messageResponse, void* context);
typedef CallbackResponse (*SubscriberApiCallback)(CrosMessage *message,  void* context);
typedef CallbackResponse (*PublisherApiCallback)(CrosMessage *message, void* context);

// Master api: register/unregister methods
int cRosApiRegisterService(CrosNode *node, const char *service_name, const char *service_type, ServiceProviderApiCallback callback, void *context);
int cRosApisUnegisterService(CrosNode *node, int svcidx);
int cRosApiRegisterSubscriber(CrosNode *node, const char *topic_name, const char *topic_type, SubscriberApiCallback callback, SlaveCallback slave_callback, void *context);
int cRosApiUnregisterSubscriber(CrosNode *node, int subidx);
int cRosApiRegisterPublisher(CrosNode *node, const char *topic_name, const char *topic_type, PublisherApiCallback callback, SlaveCallback slave_callback, void *context);
int cRosApiUnregisterPublisher(CrosNode *node, int pubidx);

// Master api: name service and system state
int cRosApiLookupNode(CrosNode *node, const char *node_name, LookupNodeCallback *callback, void *context);
int cRosApiGetPublishedTopics(CrosNode *node, const char *subgraph, GetPublishedTopicsCallback *callback, void *context);
int cRosApiGetTopicTypes(CrosNode *node, GetTopicTypesCallback *callback, void *context);
int cRosApiGetSystemState(CrosNode *node, GetSystemStateCallback *callback, void *context);
int cRosApiGetUri(CrosNode *node, GetUriCallback *callback, void *context);
int cRosApiLookupService(CrosNode *node, const char *service, LookupServiceCallback *callback, void *context);

// Slave api

/*!
 *  \param subidx If >= 0 it will be contacted the xmlrpc server of the topic being subcribed
 *                If == -1 it will be contacted the roscore
 *                If == NULL it will fallback on the host and port provided
 *  \param host Used only if subidx == NULL
 *  \param port Used only if subidx == NULL
 */
int cRosApiGetBusStats(CrosNode *node, int *subidx, const char* host, int port, GetBusStatsCallback *callback, void *context);

/*!
 *  \param subidx If >= 0 it will be contacted the xmlrpc server of the topic being subcribed
 *                If == -1 it will be contacted the roscore
 *                If == NULL it will fallback on the host and port provided
 *  \param host Used only if subidx == NULL
 *  \param port Used only if subidx == NULL
 */
int cRosApiGetBusInfo(CrosNode *node, int *subidx, const char* host, int port, GetBusInfoCallback *callback, void *context);

/*!
 *  \param subidx If >= 0 it will be contacted the xmlrpc server of the topic being subcribed
 *                If == -1 it will be contacted the roscore
 *                If == NULL it will fallback on the host and port provided
 *  \param host Used only if subidx == NULL
 *  \param port Used only if subidx == NULL
 */
int cRosApiGetMasterUri(CrosNode *node, int *subidx, const char* host, int port, GetMasterUriCallback *callback, void *context);

/*!
 *  \param subidx If >= 0 it will be contacted the xmlrpc server of the topic being subcribed
 *                If == -1 it will be contacted the roscore
 *                If == NULL it will fallback on the host and port provided
 *  \param host Used only if subidx == NULL
 *  \param port Used only if subidx == NULL
 */
int cRosApiShutdown(CrosNode *node, int *subidx, const char* host, int port, const char *msg, GetMasterUriCallback *callback, void *context);

/*!
 *  \param subidx If >= 0 it will be contacted the xmlrpc server of the topic being subcribed
 *                If == -1 it will be contacted the roscore
 *                If == NULL it will fallback on the host and port provided
 *  \param host Used only if subidx == NULL
 *  \param port Used only if subidx == NULL
 */
int cRosApiGetPid(CrosNode *node, int *subidx, const char* host, int port, GetPidCallback *callback, void *context);

/*!
 *  \param subidx If >= 0 it will be contacted the xmlrpc server of the topic being subcribed
 *                If == -1 it will be contacted the roscore
 *                If == NULL it will fallback on the host and port provided
 *  \param host Used only if subidx == NULL
 *  \param port Used only if subidx == NULL
 */
int cRosApiGetSubscriptions(CrosNode *node, int *subidx, const char* host, int port, GetSubscriptionsCallback *callback, void *context);

/*!
 *  \param subidx If >= 0 it will be contacted the xmlrpc server of the topic being subcribed
 *                If == -1 it will be contacted the roscore
 *                If == NULL it will fallback on the host and port provided
 *  \param host Used only if subidx == NULL
 *  \param port Used only if subidx == NULL
 */
int cRosApiGetPublications(CrosNode *node, int *subidx, const char* host, int port, GetSubscriptionsCallback *callback, void *context);

#endif // _CROS_NODE_API_H_

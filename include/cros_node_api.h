#ifndef _CROS_NODE_API_H_
#define _CROS_NODE_API_H_

#include "cros_node.h"
#include "cros_message.h"

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

typedef CallbackResponse (*ServiceProviderApiCallback)(cRosMessage *request, cRosMessage *response, void *context);
typedef CallbackResponse (*SubscriberApiCallback)(cRosMessage *message,  void *context);
typedef CallbackResponse (*PublisherApiCallback)(cRosMessage *message, void *context);

// Master api: register/unregister methods
int cRosApiRegisterServiceProvider(CrosNode *node, const char *service_name, const char *service_type, ServiceProviderApiCallback callback, NodeStatusCallback status_callback, void *context);
int cRosApisUnegisterServiceProvider(CrosNode *node, int svcidx);
int cRosApiRegisterSubscriber(CrosNode *node, const char *topic_name, const char *topic_type, SubscriberApiCallback callback, NodeStatusCallback status_callback, void *context);
int cRosApiUnregisterSubscriber(CrosNode *node, int subidx);
int cRosApiRegisterPublisher(CrosNode *node, const char *topic_name, const char *topic_type, int loop_period, PublisherApiCallback callback, NodeStatusCallback status_callback, void *context);
int cRosApiUnregisterPublisher(CrosNode *node, int pubidx);

// Master api: name service and system state
int cRosApiLookupNode(CrosNode *node, const char *node_name, LookupNodeCallback callback, void *context);
int cRosApiGetPublishedTopics(CrosNode *node, const char *subgraph, GetPublishedTopicsCallback callback, void *context);
int cRosApiGetTopicTypes(CrosNode *node, GetTopicTypesCallback callback, void *context);
int cRosApiGetSystemState(CrosNode *node, GetSystemStateCallback callback, void *context);
int cRosApiGetUri(CrosNode *node, GetUriCallback callback, void *context);
int cRosApiLookupService(CrosNode *node, const char *service, LookupServiceCallback callback, void *context);

// Slave api

/*!
 *  \param host host == NULL will contact ros core
 *  \param port Used only if host != NULL
 */
int cRosApiGetBusStats(CrosNode *node, const char* host, int port, GetBusStatsCallback callback, void *context);

/*!
 *  \param host host == NULL will contact ros core
 *  \param port Used only if host == NULL
 */
int cRosApiGetBusInfo(CrosNode *node, const char* host, int port, GetBusInfoCallback callback, void *context);

/*!
 *  \param host host == NULL will contact ros core
 *  \param port Used only if host == NULL
 */
int cRosApiGetMasterUri(CrosNode *node, const char* host, int port, GetMasterUriCallback callback, void *context);

/*!
 *  \param host host == NULL will contact ros core
 *  \param port Used only if host == NULL
 */
int cRosApiShutdown(CrosNode *node, const char* host, int port, const char *msg, GetMasterUriCallback callback, void *context);

/*!
 *  \param host host == NULL will contact ros core
 *  \param port Used only if host == NULL
 */
int cRosApiGetPid(CrosNode *node, const char* host, int port, GetPidCallback callback, void *context);

/*!
 *  \param host host == NULL will contact ros core
 *  \param port Used only if host == NULL
 */
int cRosApiGetSubscriptions(CrosNode *node, const char* host, int port, GetSubscriptionsCallback callback, void *context);

/*!
 *  \param host host == NULL will contact ros core
 *  \param port Used only if host == NULL
 */
int cRosApiGetPublications(CrosNode *node, const char* host, int port, GetSubscriptionsCallback callback, void *context);

#endif // _CROS_NODE_API_H_

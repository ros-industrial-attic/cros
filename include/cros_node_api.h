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

typedef void (*LookupNodeCallback)(LookupNodeResult *result, void *data);
typedef void (*GetPublishedTopicsCallback)(GetPublishedTopicsResult *result, void *data);
typedef void (*GetTopicTypesCallback)(GetTopicTypesResult *result, void *data);
typedef void (*GetSystemStateCallback)(GetSystemStateResult *result, void *data);
typedef void (*GetUriCallback)(GetUriResult *result, void *data);
typedef void (*LookupServiceCallback)(LookupServiceResult *result, void *data);
typedef void (*GetBusStatsCallback)(GetBusStatsResult *result, void *data);
typedef void (*GetBusInfoCallback)(GetBusInfoResult *result, void *data);
typedef void (*GetMasterUriCallback)(GetMasterUriResult *result, void *data);
typedef void (*RequestShutdownCallback)(ShutdownResult *result, void *data);
typedef void (*GetPidCallback)(GetPidResult *result, void *data);
typedef void (*GetSubscriptionsCallback)(GetSubscriptionsResult *result, void *data);
typedef void (*GetPublicationsCallback)(GetPublicationsResult *result, void *data);

// Master api
int lookupNode(CrosNode *node, const char *node_name, LookupNodeCallback *callback, void *data);
int getPublishedTopics(CrosNode *node, const char *subgraph, GetPublishedTopicsCallback *callback, void *data);
int getTopicTypes(CrosNode *node, GetTopicTypesCallback *callback, void *data);
int getSystemState(CrosNode *node, GetSystemStateCallback *callback, void *data);
int getUri(CrosNode *node, GetUriCallback *callback, void *data);
int lookupService(CrosNode *node, const char *service, LookupServiceCallback *callback, void *data);

// Slave api
int getBusStats(CrosNode *node, int client_idx, GetBusStatsCallback *callback, void *data);
int getBusInfo(CrosNode *node, int client_idx, GetBusInfoCallback *callback, void *data);
int getMasterUri(CrosNode *node, int client_idx, GetMasterUriCallback *callback, void *data);
int requestShutdown(CrosNode *node, int client_idx, const char *msg, GetMasterUriCallback *callback, void *data);
int getPid(CrosNode *node, int client_idx, GetPidCallback *callback, void *data);
int getSubscriptions(CrosNode *node, int client_idx, GetSubscriptionsCallback *callback, void *data);
int getPublications(CrosNode *node, int client_idx, GetSubscriptionsCallback *callback, void *data);

#endif // _CROS_NODE_API_H_

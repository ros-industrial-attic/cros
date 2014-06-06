#include "cros_node_api.h"

int lookupNode(CrosNode *node, const char *node_name, LookupNodeCallback *callback, void *data)
{
  return 0;
}

int getPublishedTopics(CrosNode *node, const char *subgraph, GetPublishedTopicsCallback *callback, void *data)
{
  return 0;
}

int getTopicTypes(CrosNode *node, GetTopicTypesCallback *callback, void *data)
{
  return 0;
}

int getSystemState(CrosNode *node, GetSystemStateCallback *callback, void *data)
{
  return 0;
}

int getUri(CrosNode *node, GetUriCallback *callback, void *data)
{
  return 0;
}

int lookupService(CrosNode *node, const char *service, LookupServiceCallback *callback, void *data)
{
  return 0;
}

int getBusStats(CrosNode *node, int client_idx, GetBusStatsCallback *callback, void *data)
{
  return 0;
}

int getBusInfo(CrosNode *node, int client_idx, GetBusInfoCallback *callback, void *data)
{
  return 0;
}

int getMasterUri(CrosNode *node, int client_idx, GetMasterUriCallback *callback, void *data)
{
  return 0;
}

int requestShutdown(CrosNode *node, int client_idx, const char *msg, GetMasterUriCallback *callback, void *data)
{
  return 0;
}

int getPid(CrosNode *node, int client_idx, GetPidCallback *callback, void *data)
{
  return 0;
}

int getSubscriptions(CrosNode *node, int client_idx, GetSubscriptionsCallback *callback, void *data)
{
  return 0;
}

int getPublications(CrosNode *node, int client_idx, GetSubscriptionsCallback *callback, void *data)
{
  return 0;
}

#ifndef _CROS_NODE_INTERNAL_H_
#define _CROS_NODE_INTERNAL_H_

#include "cros_node.h"

/*! \brief Register a new topic to be published by a node.
 *  \param n Pointer to CrosNode structure that has previously been created with cRosNodeCreate
 *  \param message_definition Full text of message definition (output of gendeps --cat)
 *  \param topic_name The published topic namespace
 *  \param topic_type The published topic data type (e.g., std_msgs/String, ...)
 *  \param md5sum The MD5 sum of the message typedef
 *  \param loop_period Period (in msec) for publication cycle
 *  \param data_context Pointer to user data than will be passed to the callback function as context information. Can be NULL
 *  \return Returns the index of the created publisher on success, -1 on failure (e.g., the maximum number of publisher topics has been reached)
 */
int cRosNodeRegisterPublisher(CrosNode *n, const char *message_definition, const char *topic_name,
                              const char *topic_type, const char *md5sum, int loop_period, void *data_context);

/*! \brief Register the node in roscore as topic subscriber.
 *  \param n Pointer to CrosNode structure that has previously been created with cRosNodeCreate
 *  \param message_definition Full text of message definition
 *  \param topic_name The published topic namespace
 *  \param topic_type The published topic data type (e.g., std_msgs/String, ...)
 *  \param md5sum The MD5 sum of the message typedef
 *  \param data_context Pointer to user data than will be passed to the callback function as context information. Can be NULL
 *  \param tcp_nodelay If this parameter is 1, the publisher is asked to disable the Nagle algorithm for the socket,
 *         so small packets are sent immediately, reducing the latency but increasing the bandwidth usage.
 *  \return Returns the index of the created subscriber on success, -1 on failure (e.g., the maximum number of subscriber topics has been reached)
 */
int cRosNodeRegisterSubscriber(CrosNode *n, const char *message_definition, const char *topic_name, const char *topic_type,
                               const char *md5sum, void *data_context, int tcp_nodelay);

/*! \brief Register the service provider in roscore
 *  \param n Pointer to CrosNode structure that has previously been created with cRosNodeCreate
 *  \param service_name The published service namespace
 *  \param service_type The published service data type (e.g., roscpp_tutorials/TwoInts)
 *  \param md5sum The MD5 sum of the message typedef
 *  \param data_context Pointer to user data than will be passed to the callback function as context information. Can be NULL
 *  \return Returns the index of the created service provider on success, -1 on failure (e.g., the maximum number of service providers has been reached)
 */
int cRosNodeRegisterServiceProvider(CrosNode *n, const char *service_name,
                                    const char *service_type, const char *md5sum, void *data_context);

/*! \brief Register the service caller (not in roscore)
 *  \param n Pointer to CrosNode structure that has previously been created with cRosNodeCreate
 *  \param service_name The published service namespace
 *  \param service_type The published service data type (e.g., roscpp_tutorials/TwoInts)
 *  \param md5sum The MD5 sum of the message typedef
 *  \param data_context Pointer to user data than will be passed to the callback function as context information. Can be NULL
 *  \param persistent If this parameter is 1, the RPCROS connection is kept opened for multiple calls.
 *         This reduces bandwidth usage and latency. Otherwise the parameter value should be to 0.
 *  \param tcp_nodelay If this parameter is 1, the service provider is asked to disable the Nagle algorithm for the socket,
 *         so small packets are sent immediately, reducing the latency but increasing the bandwidth usage.
 *  \return Returns the index of the created service caller on success, -1 on failure (e.g., the maximum number of services has been reached)
 */
int cRosNodeRegisterServiceCaller(CrosNode *n, const char *message_definition, const char *service_name,
                                    const char *service_type, const char *md5sum, int loop_period,
                                    void *data_context, int persistent, int tcp_nodelay);

/*! \brief Unregister the topic subscriber
 *
 *  \param subidx Index of the subscriber
 */
int cRosNodeUnregisterSubscriber(CrosNode *node, int subidx);

/*! \brief Unregister the topic publisher
 *
 *  \param pubidx Index of the topic publisher
 */
int cRosNodeUnregisterPublisher(CrosNode *node, int pubidx);

/*! \brief Unregister the service provider
 *
 *  \param serviceidx Index of the service provider
 */
int cRosNodeUnregisterServiceProvider(CrosNode *node, int serviceidx);


/*! \brief Frees the memory of a publisher node
 *
 *  \param node Pointer to the publisher node to be freed
 */
void cRosNodeReleasePublisher(PublisherNode *node);

/*! \brief Frees the memory of a subscriber node
 *
 *  \param node Pointer to the subscriber node to be freed
 */
void cRosNodeReleaseSubscriber(SubscriberNode *node);

/*! \brief Frees the memory of a service-provider node
 *
 *  \param node Pointer to the service node to be freed
 */
void cRosNodeReleaseServiceProvider(ServiceProviderNode *node);

/*! \brief Frees the memory of a service-caller node
 *
 *  \param node Pointer to the service node to be freed
 */
void cRosNodeReleaseServiceCaller(ServiceCallerNode *node);

/*! \brief Frees the memory of a parameter subscription
 *
 *  \param node Pointer to the parameter subscription to be freed
 */
void cRosNodeReleaseParameterSubscrition(ParameterSubscription *subscription);

/*! \brief Search for a Tcpros client proc that is currently not assigned
 *         to any subscriber and assign it to the specified subscriber
 *
 *  \param node Pointer to CrosNode structure that has previously been created with cRosNodeCreate
 *  \param subidx Index of the subscriber
 *  \return Returns the found Tcpros client index on success or -1 on failure (e.g., No Tcpros client available)
 */
int cRosNodeRecruitTcprosClientProc(CrosNode *node, int subidx);

/*! \brief Search for a Tcpros client proc that is currently assigned
 *         to the specified subscriber and is connected to the specified hostname and port
 *
 *  subidx, tcpros_hostname and tcpros_port set the search condition for the Tcpros client proc
 *  \param node Pointer to CrosNode structure that has previously been created with cRosNodeCreate
 *  \param subidx Index of the subscriber or -1 for any subscriber
 *  \param tcpros_hostname Pointer to the hostname string or NULL for any hostname
 *  \param tcpros_port Port number of -1 for any port
 *  \return Returns the found Tcpros client index on success or -1 on failure (e.g., No Tcpros client matching the search criteria)
 */
int cRosNodeFindFirstTcprosClientProc(CrosNode *node, int subidx, const char *tcpros_hostname, int tcpros_port);

void restartAdversing(CrosNode* node);
int enqueueRequestTopic(CrosNode *node, int subidx, const char *host, int port);
int enqueueMasterApiCall(CrosNode *node, RosApiCall *call);
int enqueueSlaveApiCall(CrosNode *node, RosApiCall *call, const char *host, int port);

void getMsgFilePath(CrosNode *node, char *buffer, size_t bufsize, const char *topic_type);
void getSrvFilePath(CrosNode *node, char *buffer, size_t bufsize, const char *service_type);

#endif // _CROS_NODE_INTERNAL_H_

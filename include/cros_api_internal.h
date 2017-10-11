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
 *  \param callback Pointer to the callback function that will be called to generate the (raw) packet data of type topic_type
 *  \param status_callback Pointer to the status callback function
 *  \param data_context Pointer to user data than will be passed to the callback function as context information. Can be NULL
 *  \return Returns 0 on success, -1 on failure (e.g., the maximum number of publisher topics has been reached)
 */
int cRosNodeRegisterPublisher(CrosNode *n, const char *message_definition, const char *topic_name,
                              const char *topic_type, const char *md5sum, int loop_period,
                              PublisherCallback callback, NodeStatusCallback status_callback, void *data_context);

/*! \brief Register the node in roscore as topic subscriber.
 *  \param n Pointer to CrosNode structure that has previously been created with cRosNodeCreate
 *  \param message_definition Full text of message definition
 *  \param topic_name The published topic namespace
 *  \param topic_type The published topic data type (e.g., std_msgs/String, ...)
 *  \param md5sum The MD5 sum of the message typedef
 *  \param callback Pointer to the callback function that will be called when a received new message is ready
 *  \param status_callback Pointer to the status callback function
 *  \param data_context Pointer to user data than will be passed to the callback function as context information. Can be NULL
 *  \param tcp_nodelay If this parameter is 1, the publisher is asked to disable the Nagle algorithm for the socket,
 *         so small packets are sent immediately, reducing the latency but increasing the bandwidth usage.
 *  \return Returns 0 on success, -1 on failure (e.g., the maximum number of subscriber topics has been reached)
 */
int cRosNodeRegisterSubscriber(CrosNode *n, const char *message_definition,
                               const char *topic_name, const char *topic_type, const char *md5sum,
                               SubscriberCallback callback, NodeStatusCallback status_callback, void *data_context, int tcp_nodelay);

/*! \brief Register the service provider in roscore
 *  \param n Pointer to CrosNode structure that has previously been created with cRosNodeCreate
 *  \param service_name The published service namespace
 *  \param service_type The published service data type (e.g., roscpp_tutorials/TwoInts)
 *  \param md5sum The MD5 sum of the message typedef
 *  \param callback Pointer to the callback function that will be called when a service request is received
 *  \param status_callback Pointer to the status callback function
 *  \param data_context Pointer to user data than will be passed to the callback function as context information. Can be NULL
 *  \return Returns 0 on success, -1 on failure (e.g., the maximum number of subscriber topics has been reached)
 */
int cRosNodeRegisterServiceProvider(CrosNode *n, const char *service_name,
                                    const char *service_type, const char *md5sum,
                                    ServiceProviderCallback callback, NodeStatusCallback status_callback,
                                    void *data_context);

/*! \brief Register the service caller (not in roscore)
 *  \param n Pointer to CrosNode structure that has previously been created with cRosNodeCreate
 *  \param service_name The published service namespace
 *  \param service_type The published service data type (e.g., roscpp_tutorials/TwoInts)
 *  \param md5sum The MD5 sum of the message typedef
 *  \param callback Pointer to the callback function that will be called when a service request is received
 *  \param status_callback Pointer to the status callback function
 *  \param data_context Pointer to user data than will be passed to the callback function as context information. Can be NULL
 *  \param persistent If this parameter is 1, the RPCROS connection is kept opened for multiple calls.
 *         This reduces bandwidth usage and latency. Otherwise the parameter value should be to 0.
 *  \param tcp_nodelay If this parameter is 1, the service provider is asked to disable the Nagle algorithm for the socket,
 *         so small packets are sent immediately, reducing the latency but increasing the bandwidth usage.
 *  \return Returns 0 on success, -1 on failure (e.g., the maximum number of services has been reached)
 */
int cRosNodeRegisterServiceCaller(CrosNode *n, const char *message_definition, const char *service_name,
                                    const char *service_type, const char *md5sum, int loop_period,
                                    ServiceCallerCallback callback, NodeStatusCallback status_callback,
                                    void *data_context, int persistent, int tcp_nodelay);

/*! \brief Unregister the topic subscriber
 *
 *  \param subidx Index of the subscriber
 */
int cRosNodeUnregisterSubscriber(CrosNode *node, int subidx);

/*! \brief Unregister the topic publisher
 *
 *  \param subidx Index of the topic publisher
 */
int cRosNodeUnregisterPublisher(CrosNode *node, int pubidx);

/*! \brief Unregister the service provider
 *
 *  \param subidx Index of the service provider
 */
int cRosNodeUnregisterServiceProvider(CrosNode *node, int serviceidx);

/*! \brief Unregister the service caller
 *
 *  \param subidx Index of the service provider
 */
int cRosNodeUnregisterServiceCaller(CrosNode *node, int serviceidx);

void restartAdversing(CrosNode* node);
int enqueueRequestTopic(CrosNode *node, int subidx);
int enqueueMasterApiCall(CrosNode *node, RosApiCall *call);
int enqueueSlaveApiCall(CrosNode *node, RosApiCall *call, const char *host, int port);

void getMsgFilePath(CrosNode *node, char *buffer, size_t bufsize, const char *topic_type);
void getSrvFilePath(CrosNode *node, char *buffer, size_t bufsize, const char *service_type);

#endif // _CROS_NODE_INTERNAL_H_

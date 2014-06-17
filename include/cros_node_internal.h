#ifndef _CROS_NODE_INTERNAL_H_
#define _CROS_NODE_INTERNAL_H_

#include "cros_node.h"

/*! \brief Register a new topic to be published by a node.
 *
 *  \param message_definition Full text of message definition (output of gendeps --cat)
 *  \param topic_name The published topic namespace
 *  \param topic_type The published topic data type (e.g., std_msgs/String, ...)
 *  \param md5sum The md5sum of the message typedef
 *  \param loop_period Period (in msec) for publication cycle
 *  \param publisherDataCallback The callback called to generate the (raw) packet data
 *                                of type topic_type
 *  \param slave_callback Callback that gives feedback on connected xmlrpc clients. Can be NULL
 *  \return Returns 0 on success, -1 on failure (e.g., the maximu number of
 *          published topics has been reached )
 */
int cRosNodeRegisterPublisher(CrosNode *n, const char *message_definition, const char *topic_name,
                              const char *topic_type, const char *md5sum, int loop_period,
                              PublisherCallback callback, SlaveStatusCallback slave_callback, void *data_context);

/*! \brief Register the node in roscore as topic subscriber.
 *  \param slave_callback Callback that gives feedback on available xmlrpc servers. Can be NULL
 *  \param TODO review doxy documentation
 */
int cRosNodeRegisterSubscriber(CrosNode *n, const char *message_definition,
                               const char *topic_name, const char *topic_type, const char *md5sum,
                               SubscriberCallback callback, SlaveStatusCallback slave_callback, void *data_context);

/*! \brief Register the service provider in roscore
 *
 *  \param TODO review doxy documentation
 */
int cRosNodeRegisterServiceProvider(CrosNode *n, const char *service_name,
                                    const char *service_type, const char *md5sum,
                                    ServiceProviderCallback callback, void *data_context);

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
int cRosNodeUnregisterService(CrosNode *node, int serviceidx);

void restartAdversing(CrosNode* node);
int enqueueRequestTopic(CrosNode *node, int subidx);
int enqueueMasterApiCall(CrosNode *node, RosApiCall *call);
int enqueueSlaveApiCall(CrosNode *node, RosApiCall *call);

#endif // _CROS_NODE_INTERNAL_H_

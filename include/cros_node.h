/*! \file cros_node.h
 *  \brief This file declares the CrosNode structure, which codifies a ROS node in cROS. It also declares
 *         associated substructures, functions and macros.
 *
 *  This ROS node can implement at the same time several:
 *  - Topic subscribers
 *  - Topic publishers
 *  - Service providers (servers)
 *  - Service callers (clients)
 *  - Parameter subscriber
 */

#ifndef _CROS_NODE_H_
#define _CROS_NODE_H_

#include <stdio.h>
#include <stdint.h>

#include "cros_log.h"
#include "xmlrpc_process.h"
#include "tcpros_process.h"
#include "cros_api_call.h"
#include "cros_message_queue.h"
#include "cros_err_codes.h"

/*! \defgroup cros_node cROS Node */

/*! \addtogroup cros_node
 *  @{
 */

/*! Max num published topics */
#define CN_MAX_PUBLISHED_TOPICS 5

/*! Max num subscribed topics */
#define CN_MAX_SUBSCRIBED_TOPICS 5

/*! Max num service providers */
#define CN_MAX_SERVICE_PROVIDERS 8

/*! Max num service callers */
#define CN_MAX_SERVICE_CALLERS 8

/*! Max num parameter subscriptions */
#define CN_MAX_PARAMETER_SUBSCRIPTIONS 20

/*! Max num serving XMLRPC connections */
#define CN_MAX_XMLRPC_SERVER_CONNECTIONS 5

/*! Max num serving TCPROS connections */
#define CN_MAX_TCPROS_SERVER_CONNECTIONS 5

/*! Max num serving RPCROS connections */
#define CN_MAX_RPCROS_SERVER_CONNECTIONS CN_MAX_SERVICE_PROVIDERS

/*!
 * Max num XMLRPC connections against another subscribed nodes
 *  (first connection index reserved to roscore)
 * */
#define CN_MAX_XMLRPC_CLIENT_CONNECTIONS 1 + CN_MAX_SUBSCRIBED_TOPICS

/*!
 * Max num TCPROS connections against another subscribed nodes
 * */
#define CN_MAX_TCPROS_CLIENT_CONNECTIONS CN_MAX_SUBSCRIBED_TOPICS

/*!
 * Max num RPCROS connections against other service-providing nodes
 * (service calls are one to one, so, one TcprosProcess per ServiceCallerNode)
 * */
#define CN_MAX_RPCROS_CLIENT_CONNECTIONS CN_MAX_SERVICE_CALLERS

/*! Node automatic XMLRPC ping cycle period (in msec) */
#define CN_PING_LOOP_PERIOD 1000

/*! Maximum I/O operations timeout (in msec) */
#define CN_IO_TIMEOUT 3000

/*! Maximum time that the node will wait for unregistering all publishers, subscribers, servicer providers... in the ROS master (in msec) */
#define CN_UNREGISTRATION_TIMEOUT 3000

typedef struct PublisherNode PublisherNode;
typedef struct SubscriberNode SubscriberNode;
typedef struct ServiceProviderNode ServiceProviderNode;
typedef struct ServiceCallerNode ServiceCallerNode;
typedef struct ParameterSubscription ParameterSubscription;

typedef enum CrosNodeStatus
{
  // TODO: this is a work in progress
  CROS_STATUS_NONE = 0,
  CROS_STATUS_PUBLISHER_UNREGISTERED,
  CROS_STATUS_SUBSCRIBER_UNREGISTERED,
  CROS_STATUS_SERVICE_PROVIDER_UNREGISTERED,
  CROS_STATUS_PARAM_UNSUBSCRIBED,
  CROS_STATUS_PARAM_SUBSCRIBED,
  CROS_STATUS_PARAM_UPDATE,
} CrosNodeStatus;

typedef struct CrosNodeStatusUsr
{
  // FIXME: this is a work in progress
  // int callid; // This may be useful to track register/unregister
  CrosNodeStatus state; // May be useful to understand what the node, or the particular sub/pub/svc is doing
  int provider_idx;
  const char *xmlrpc_host;
  int xmlrpc_port;
  const char *parameter_key;
  XmlrpcParam *parameter_value;
} CrosNodeStatusUsr;

/*! \brief Callback to communicate publisher or subscriber status */
typedef void (*NodeStatusApiCallback)(CrosNodeStatusUsr *status, void* context);

/*! Structure that define a published topic */
struct PublisherNode
{
  char *topic_name;                   //! The published topic name
  char *topic_type;                   //! The published topic data type (e.g., std_msgs/String, ...)
  char *md5sum;                       //! The MD5 sum of the message type
  char *message_definition;           //! Full text of message definition (output of gendeps --cat)
  int   tcpros_id_list[CN_MAX_TCPROS_SERVER_CONNECTIONS+1]; //! List of node->tcpros_server_proc IDs allocated for this publisher. The last element of the list is always -1 (sentinel)
  void *context;
  int loop_period;                    //! Period (in msec) for publication cycle
  uint64_t wake_up_time;              //! The time for the next automatic message publication (in msec, since the Epoch)
  cRosMessageQueue msg_queue;         //! Messages on this topic wait in this queue to be send for every process
};

/*! Structure that define a subscribed topic */
struct SubscriberNode
{
  char *message_definition;           //! Full text of message definition (output of gendeps --cat)
  char *topic_name;                   //! The subscribed topic name
  char *topic_type;                   //! The subscribed topic data type (e.g., std_msgs/String, ...)
  char *md5sum;                       //! The MD5 sum of the message type
  unsigned char tcp_nodelay;          //! If 1, the publisher should set TCP_NODELAY on the socket, if possible
  void *context;                      //! Pointer to an internal library structure that stores received messages and its type
  cRosMessageQueue msg_queue;         //! Each time a message on this topic is received it is queued here
  unsigned char msg_queue_overflow;   //! If 1, the subscriber tried to insert a message in the queue but it was full
};

struct ServiceProviderNode
{
  char *service_name;
  char *service_type;
  char *servicerequest_type;
  char *serviceresponse_type;
  char *md5sum;
  void *context;
};

struct ServiceCallerNode
{
  char *service_name;
  char *service_type;
  char *servicerequest_type;
  char *serviceresponse_type;
  char *md5sum;
  char *message_definition;           //! Full text of message definition (output of gendeps --cat)
  int   rpcros_id;                    //! Index of the node->service_callers allocated for this ServiceCallerNode
  char *service_host;                 //! The hostname of the service provider.
  int   service_port;                 //! The host port of the the service provider.
  unsigned char persistent;           //! If 1, the service RPCROS connection should be kept open for multiple requests
  unsigned char tcp_nodelay;          //! If 1, the service caller should set TCP_NODELAY on the socket, if possible
  void *context;
  int loop_period;                    //! Period (in msec) for service-call cycle
  uint64_t wake_up_time;              //! The time for the next automatic service call (in msec, since the Epoch)
  cRosMessageQueue msg_queue;         //! Service requests and service responses for this service wait in this queue to be send
};

struct ParameterSubscription
{
  char *parameter_key;
  XmlrpcParam parameter_value;
  void *context;
  NodeStatusApiCallback status_api_callback;
};

/*! \brief CrosNode object. Don't modify its internal members: use the related functions instead */
typedef struct CrosNode CrosNode;
struct CrosNode
{
  char *name;                   //! The node name: it is the absolute name, i.e. it includes the namespace
  char *host;                   //! The node host (ipv4, e.g. 192.168.0.2)
  unsigned short xmlrpc_port;   //! The node port for the XMLRPC protocol
  unsigned short tcpros_port;   //! The node port for the TCPROS protocol
  unsigned short rpcros_port;   //! The node port for the RPCROS protocol

  int pid;                      //! cROS node process ID
  int roscore_pid;              //! Roscore PID

  char *roscore_host;           //! The roscore host (ipv4, e.g. 192.168.0.1)
  unsigned short roscore_port;  //! The roscore port

  char *message_root_path;      //! Directory with the message register

  CrosLogLevel log_level;
  int rosout_pub_idx;           //! Index of the publisher of the /rosout topic for ROS log messages

  uint64_t xmlrpc_master_wake_up_time; //! The time (in msec, since the Epoch) for the next automatic operation cycle of the xmlrpc_client_proc[0] (xmlrpc master-node client proc)

  uint32_t log_last_id;         //! Sequence number of the last transmitted rosout log message

  unsigned int next_call_id;
  ApiCallQueue master_api_queue;
  ApiCallQueue slave_api_queue;

  //! Manage connections for XMLRPC calls from this node to others
  XmlrpcProcess xmlrpc_client_proc[CN_MAX_XMLRPC_CLIENT_CONNECTIONS];
  XmlrpcProcess xmlrpc_listner_proc;   //! Accept new XMLRPC connections from roscore or other nodes
  /*! Manage connections for XMLRPC calls from roscore or other nodes to this node */
  XmlrpcProcess xmlrpc_server_proc[CN_MAX_XMLRPC_SERVER_CONNECTIONS];

  //! Manage connections for TCPROS calls from this node to others
  TcprosProcess tcpros_client_proc[CN_MAX_TCPROS_CLIENT_CONNECTIONS];
  TcprosProcess tcpros_listner_proc;   //! Accept new TCPROS connections from roscore or other nodes

  /*! Manage connections for TCPROS between this and other nodes  */
  TcprosProcess tcpros_server_proc[CN_MAX_TCPROS_SERVER_CONNECTIONS];

  //! Manage connections for RPCROS calls from this node to others
  TcprosProcess rpcros_client_proc[CN_MAX_RPCROS_CLIENT_CONNECTIONS];
  TcprosProcess rpcros_listner_proc;   //! Accept new TCPROS connections from roscore or other nodes

  /*! Manage connections for RPCROS between this and other nodes  */
  TcprosProcess rpcros_server_proc[CN_MAX_RPCROS_SERVER_CONNECTIONS];

  PublisherNode pubs[CN_MAX_PUBLISHED_TOPICS];            //! All the published topic, defined by PublisherNode structures
  SubscriberNode subs[CN_MAX_SUBSCRIBED_TOPICS];          //! All the subscribed topic, defined by PublisherNode structures
  ServiceProviderNode service_providers[CN_MAX_SERVICE_PROVIDERS]; //! All the provided services to register
  ServiceCallerNode service_callers[CN_MAX_SERVICE_CALLERS]; //! All the services to call
  ParameterSubscription paramsubs[CN_MAX_PARAMETER_SUBSCRIPTIONS];

  int n_pubs;                   //! Number of node's published topics
  int n_subs;                   //! Number of node's subscribed topics
  int n_service_providers;      //! Number of registered services to provide
  int n_service_callers;        //! Number of services to call
  int n_paramsubs;
};

/*! \brief Resolve the namespace of the resource name
 *
 *  \param node the CrosNode which is owner of the resource. If NULL the resource is a node as well.
 *  \param resource_name the name of the resource.
 *
 *  \return A string with the resource name.
 */
char *cRosNamespaceBuild(CrosNode *node, const char *resource_name);

void cRosGetMsgFilePath(CrosNode *node, char *buffer, size_t bufsize, const char *topic_type);

/*! \brief Dynamically create a CrosNode instance. This is the right way to create a CrosNode object.
 *         Once finished, the CrosNode should be released using cRosNodeDestroy()
 *
 *  \param node_name The node name: it is the absolute name, i.e. it should includes the namespace
 *  \param node_host The node host (ipv4, e.g. 192.168.0.2)
 *  \param roscore_host The roscore host (ipv4, e.g. 192.168.0.1)
 *  \param roscore_port The roscore port
 *  \param select_timeout_ms Max timeout for the select() in ms. NULL defaults to UINT64_MAX
 *
 *  \return A pointer to the new CrosNode on success, NULL on failure
 */
CrosNode *cRosNodeCreate(const char *node_name, const char *node_host, const char *roscore_host, unsigned short roscore_port,
                         const char *message_root_path);

/*! \brief Unregister from ROS master and release all the internal allocated memory for a CrosNode
 *          object previously crated with cRosNodeCreate()
 *
 *  \param n A pointer to the CrosNode object to be released
 *  \return CROS_SUCCESS_ERR_PACK (0) on success. Otherwise an error code pack containing one or more error codes
 */
cRosErrCodePack cRosNodeDestroy( CrosNode *n );

/*! \brief Perform a loop of the cROS node main cycle
 *
 *  \param n A pointer to a CrosNode object (e.g., created with cRosNodeCreate())
 *  \param max_timeout Maximum time in milliseconds that this function will take to finish (it may finish before).
 *
 *  cRosNodeDoEventsLoop() perform a file-based event loop: check the read and write operation availability
 *  for considered sockets, start new write and read actions, open new connections, close dropped connections,
 *  manage the internal state of the node (i.e., advertise new topics, ...)
 *
 *  cRosNodeDoEventsLoop() should be used inside a cycle, e.g.:
 *
 *  while(1)
 *  {
 *    // If you want, do here something
 *    cRosNodeDoEventsLoop( node );
 *  }
 *  \return CROS_SUCCESS_ERR_PACK (0) on success
 */
cRosErrCodePack cRosNodeDoEventsLoop( CrosNode *n, uint64_t max_timeout );

/*! \brief Run the cROS node for a specific time while the exit flag provided by the user is 0
 *
 *  This function repeatedly calls cRosNodeDoEventsLoop() while these three conditions are met:
 *  No error occurs, the exit_flag variable is 0 and the timeout is not reached.
 *  \param n A pointer to a CrosNode object (e.g., created with cRosNodeCreate())
 *  \param time_out Time in milliseconds that this function will block running the node. If CROS_INFINITE_TIMEOUT
 *         is specified, the function will not finish until an error occurs or the exit flag becomes different
 *         from 0.
 *  \param exit_flag Pointer to an unsigned char variable: the function will exit if this variable becomes
 *         different from zero. If this pointer is NULL, the node will be run until the timeout is reached or an
 *         an error occurs.
 *  \return CROS_SUCCESS_ERR_PACK (0) on success
 */
cRosErrCodePack cRosNodeStart( CrosNode *n, unsigned long time_out, unsigned char *exit_flag );

XmlrpcParam *cRosNodeGetParameterValue( CrosNode *n, const char *key);
/*! @}*/

/*! \brief Waits until a network port is open is a host address.
 *
 *  This function repeatedly tries to connect to the specified port until it succeed, an error occurs or the
 *  specified timeout is reached. When the connection is established it is closed immediately.
 *  \param host_addr The target address
 *  \param host_port The target port
 *  \param time_out Maximum time in milliseconds that this function will block trying to connect. If
 *         CROS_INFINITE_TIMEOUT is specified, the function will not finish until an error occurs or the
 *         the port connection is established.
 *  \return CROS_SUCCESS_ERR_PACK if the port could be connected. CROS_SOCK_OPEN_CONN_ERR is an error
 *          occurred when trying to connect or CROS_SOCK_OPEN_TIMEOUT_ERR if the timeout was reached before
 *          the port could be connected.
 */
cRosErrCodePack cRosWaitPortOpen(const char *host_addr, unsigned short host_port, unsigned long time_out);

/*! \brief Set the file stream used when printing messages locally, that is, through macros:
 *         PRINT_INFO, PRINT_DEBUG, PRINT_VDEBUG, PRINT_VVDEBUG and PRINT_ERROR.
 *
 *  There are two types of message printing:
 *   - local message printing (which are printed to a file or to stdout depending on the file stream specified
 *     when calling this function).
 *   - ROS log messages. These messages are not affected by this function.
 *  \param new_stream The file stream used. It must be a valid file stream or NULL if stdout must be used.
 */
void cRosOutStreamSet(FILE *new_stream);

/*! \brief Get the file stream used for printing local messages.
 *
 *  \return The file stream used. If NULL was specified when calling cRosOutStreamSet(), stdout is returned.
 */
FILE *cRosOutStreamGet(void);

#endif

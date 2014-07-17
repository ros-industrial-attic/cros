#ifndef _CROS_NODE_H_
#define _CROS_NODE_H_

#include <stdint.h>
#include "xmlrpc_process.h"
#include "tcpros_process.h"
#include "cros_api_call.h"
#include "cros_log.h"

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
 *  (first connection index reserved to roscore)
 * */
#define CN_MAX_TCPROS_CLIENT_CONNECTIONS 1 + CN_MAX_SUBSCRIBED_TOPICS

/*! Node automatic XMLRPC ping cycle period (in msec) */
#define CN_PING_LOOP_PERIOD 1000

/*! Maximum I/O operations timeout (in msec) */
#define CN_IO_TIMEOUT 2000

typedef struct PublisherNode PublisherNode;
typedef struct SubscriberNode SubscriberNode;
typedef struct ServiceProviderNode ServiceProviderNode;
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
  CrosNodeStatus state; // May be useful to udnerstand what the node, or the particular sub/pub/svc is doing
  int provider_idx;
  const char *xmlrpc_host;
  int xmlrpc_port;
  const char *parameter_key;
  XmlrpcParam *parameter_value;
} CrosNodeStatusUsr;

/*! \brief Callback to communicate publisher or subscriber status
 */
typedef void (*NodeStatusCallback)(CrosNodeStatusUsr *status, void* context);

typedef uint8_t CallbackResponse;
typedef CallbackResponse (*PublisherCallback)(DynBuffer *buffer, void* context);

/*! Structure that define a published topic */
struct PublisherNode
{
  char *topic_name;                             //! The published topic name
  char *topic_type;                             //! The published topic data type (e.g., std_msgs/String, ...)
  char *md5sum;                                 //! The md5sum of the message type
  char *message_definition;                     //! Full text of message definition (output of gendeps --cat)
  int client_tcpros_id;
  void *context;
  PublisherCallback callback;                   //! The callback called to generate the (raw) packet data of type topic_type
  NodeStatusCallback status_callback;
  int loop_period;                              //! Period (in msec) for publication cycle 
};

typedef CallbackResponse (*SubscriberCallback)(DynBuffer *buffer,  void* context);

/*! Structure that define a subscribed topic 
 * WARNING : not implemented!
 */
struct SubscriberNode
{
  char *message_definition;                     //! Full text of message definition (output of gendeps --cat)
  char *topic_name;                             //! The subscribed topic name
  char *topic_type;                             //! The subscribed topic data type (e.g., std_msgs/String, ...)
  char *md5sum;                                 //! The md5sum of the message type
  char *topic_host;                             //! The hostname of the topic already contacted.
  int   topic_port;                             //! The host-port of the topic already contacted.
  int   client_xmlrpc_id;                       //! The xmlrpc client that manages the subscription
  int   client_tcpros_id;
  int   tcpros_port;
  void *context;
  SubscriberCallback callback;
  NodeStatusCallback status_callback;
};

typedef CallbackResponse (*ServiceProviderCallback)(DynBuffer *bufferRequest, DynBuffer *bufferResponse, void* context);

struct ServiceProviderNode
{
  char *service_name;
  char *service_type;
  char *servicerequest_type;
  char *serviceresponse_type;
  char *md5sum;
  void *context;
  ServiceProviderCallback callback;
  NodeStatusCallback status_callback;
};

struct ParameterSubscription
{
  char *parameter_key;
  XmlrpcParam parameter_value;
  void *context;
  NodeStatusCallback status_callback;
};

/*! \brief CrosNode object. Don't modify its internal members: use
 *         the related functions instead */
typedef struct CrosNode CrosNode;
struct CrosNode
{
  char *name;                   //! The node name: it is the absolute name, i.e. it includes the namespace
  char *host;                   //! The node host (ipv4, e.g. 192.168.0.2)
  unsigned short xmlrpc_port;          //! The node port for the XMLRPC protocol
  unsigned short tcpros_port;          //! The node port for the TCPROS protocol
  unsigned short rpcros_port;          //! The node port for the RPCROS protocol

  uint64_t select_timeout;      //! Select max timeout (in ms)
  int pid;                      //! Process ID
  int roscore_pid;              //! Roscore PID

  char *roscore_host;           //! The roscore host (ipv4, e.g. 192.168.0.1)
  unsigned short roscore_port;  //! The roscore port

  char *message_root_path;      //! Directory with the message register

  CrosLogLevel log_level;
  CrosLogQueue* log_queue;
  uint32_t log_last_id;

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
  TcprosProcess rpcros_listner_proc;   //! Accept new TCPROS connections from roscore or other nodes

  /*! Manage connections for RPCROS between this and other nodes  */
  TcprosProcess rpcros_server_proc[CN_MAX_RPCROS_SERVER_CONNECTIONS];

  PublisherNode pubs[CN_MAX_PUBLISHED_TOPICS];            //! All the published topic, defined by PublisherNode structures
  SubscriberNode subs[CN_MAX_SUBSCRIBED_TOPICS];          //! All the subscribed topic, defined by PublisherNode structures
  ServiceProviderNode services[CN_MAX_SERVICE_PROVIDERS]; //! All the services to register
  ParameterSubscription paramsubs[CN_MAX_PARAMETER_SUBSCRIPTIONS];

  int n_pubs;                   //! Number of node's published topics
  int n_subs;                   //! Number of node's subscribed topics
  int n_services;               //! Number of registered services
  int n_paramsubs;
};

/*! \brief Resolve the namespace of the resource name
 *
 *  \param node the CrosNode which is owner of the resource. If NULL the resource is a node as well.
 *  \param resource_name the name of the resource.
 *
 *  \return A string with the resource name.
 */
char* cRosNamespaceBuild(CrosNode* node, const char* resource_name);

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
CrosNode *cRosNodeCreate(char* node_name, char *node_host, char *roscore_host, unsigned short roscore_port,
                         char *message_root_path, uint64_t const *select_timeout_ms );

/*! \brief Release all the internal allocated memory for a CrosNode object previously crated with
 *         cRosNodeCreate()
 * 
 *  \param n A pointer to the CrosNode object to be released
 */
void cRosNodeDestroy( CrosNode *n );

/*! \brief Perform a loop of the cROS node main cycle 
 * 
 *  \param n A pointer to a CrosNode object (e.g., created with cRosNodeCreate())
 * 
 *  cRosNodeDoEventsLoop() perform a file-based event loop: check al the read and write network 
 *  interfaces, start new write and read actions, open new connections, close dropped connections, 
 *  manage the internal state of the node (i.e., advertise new topisc, ...)
 * 
 *  cRosNodeDoEventsLoop() should be used inside a cycle, e.g.:
 * 
 *  while(1)
 *  {
 *    // If you want, do here something 
 *    cRosNodeDoEventsLoop( node );
 *  }
 */
void cRosNodeDoEventsLoop( CrosNode *n );

/*! \brief Start the cROS node main cycle
 * 
 *  \param n A pointer to a CrosNode object (e.g., created with cRosNodeCreate())
 *  \param exit Pointer to an unsigned char variable: cRosNodeStart() exit if this variable 
 *              is not zero
 * 
 *  It is an utiility function that call cRosNodeDoEventsLoop() inside a endless cycle
 */
void cRosNodeStart( CrosNode *n, unsigned char *exit );

CrosNode* cRosNodeGetCurrent();

XmlrpcParam * cRosNodeGetParameterValue( CrosNode *n, const char *key);
/*! @}*/

#endif

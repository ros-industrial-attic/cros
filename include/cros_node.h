#ifndef _CROS_NODE_H_
#define _CROS_NODE_H_

#include "xmlrpc_process.h"
#include "tcpros_process.h"

/*! \defgroup cros_node cROS Node */

/*! \addtogroup cros_node
 *  @{
 */


/*! Max num published topics */
#define CN_MAX_PUBLISHED_TOPICS 5

/*! Max num subscribed topics */
#define CN_MAX_SUBSCRIBED_TOPICS 5

/*! Max num service providers */
#define CN_MAX_SERVICE_PROVIDERS 7

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

typedef uint8_t CallbackResponse;

typedef enum
{
  CN_STATE_NONE = 0,
  CN_STATE_PING_ROSCORE = 1,
  CN_STATE_ADVERTISE_PUBLISHER = 2,
  CN_STATE_ADVERTISE_SUBSCRIBER = 4,
  CN_STATE_ADVERTISE_SERVICE = 8,
  CN_STATE_ASK_FOR_CONNECTION = 16
}CrosNodeState;


typedef CallbackResponse (*PublisherCallback)(DynBuffer *buffer);

/*! Structure that define a published topic */
struct PublisherNode
{
  char *topic_name;                             //! The published topic name
  char *topic_type;                             //! The published topic data type (e.g., std_msgs/String, ...)
  char *md5sum;                                 //! The md5sum of the message type
  char *message_definition;                     //! Full text of message definition (output of gendeps --cat)
  /*! The callback called to generate the (raw) packet data of type topic_type */
  PublisherCallback callback;
  int loop_period;                              //! Period (in msec) for publication cycle 
};

typedef CallbackResponse (*SubscriberCallback)(DynBuffer *buffer);

/*! Structure that define a subscribed topic 
 * WARNING : not implemented!
 */
struct SubscriberNode
{
  char *message_definition;                     //! Full text of message definition (output of gendeps --cat)
  char *topic_name;                             //! The subscribed topic name
  char *topic_type;                             //! The subscribed topic data type (e.g., std_msgs/String, ...)
  char *md5sum;                                 //! The md5sum of the message type
  char *topic_host;								              //! The hostname of the topic already contacted.
  int   topic_port;								              //! The host-port of the topic already contacted.
  int   client_xmlrpc_id;                       //! The xmlrpc client that manages the subscription
  int		client_tcpros_id;
  int   tcpros_port;
  SubscriberCallback callback;
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
};

/*! \brief CrosNode object. Don't modify its internal members: use
 *         the related functions instead */
typedef struct CrosNode CrosNode;
struct CrosNode
{
  CrosNodeState state;          //! The node state: it specified if we nood to advertise or subscribe somethings,...
    
  char *name;                   //! The node name: it is the absolute name, i.e. it should includes the namespace
  char *host;                   //! The node host (ipv4, e.g. 192.168.0.2)
  unsigned short xmlrpc_port;          //! The node port for the XMLRPC protocol
  unsigned short tcpros_port;          //! The node port for the TCPROS protocol
  unsigned short rpcros_port;          //! The node port for the RPCROS protocol

  uint64_t select_timeout;      //! Select max timeout (in ms)
  int pid;                      //! Process ID

  char *roscore_host;           //! The roscore host (ipv4, e.g. 192.168.0.1)
  unsigned short roscore_port;  //! The roscore port

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
  
  int n_pubs;                   //! Number of node's published topics
  int n_advertised_pubs;        //! Number of published topics yet advertised
  int n_subs;                   //! Number of node's subscribed topics
  int n_advertised_subs;        //! Number of topic subscriptions yet advertised
  int n_services;               //! Number of registered services
  int n_advertised_services;    //! Number of services already advertised
};


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
CrosNode *cRosNodeCreate( char* node_name, char *node_host, char *roscore_host,
                          unsigned short roscore_port, uint64_t const *select_timeout_ms );

/*! \brief Release all the internal allocated memory for a CrosNode object previously crated with
 *         cRosNodeCreate()
 * 
 *  \param n A pointer to the CrosNode object to be released
 */
void cRosNodeDestroy( CrosNode *n );

/*! \brief Register a new topic to be published by a node.
 * 
 *  \param message_definition Full text of message definition (output of gendeps --cat)
 *  \param topic_name The published topic namespace
 *  \param topic_type The published topic data type (e.g., std_msgs/String, ...)
 *  \param md5sum The md5sum of the message typedef
 *  \param loop_period Period (in msec) for publication cycle 
 *  \param publisherDataCallback The callback called to generate the (raw) packet data 
 *                                of type topic_type
 * 
 *  \return Returns 1 on success, 0 on failure (e.g., the maximu number of 
 *          published topics has been reached )
 */
int cRosNodeRegisterPublisher( CrosNode *n, char *message_definition, char *topic_name, 
                               char *topic_type, char *md5sum, int loop_period, 
                               PublisherCallback callback );

/*! \brief Register the node in roscore as topic subscriber.
 *
 *  \param TODO review doxy documentation
 */
int cRosNodeRegisterSubscriber(CrosNode *n, char *message_definition,
                               char *topic_name, char *topic_type, char *md5sum,
                               SubscriberCallback callback);

/*! \brief Register the service provider in roscore
 *
 *  \param TODO review doxy documentation
 */
int cRosNodeRegisterServiceProvider( CrosNode *n, char *service_name,
                               char *service_type, char *md5sum,
                                ServiceProviderCallback callback, void *data_context);

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

/*! @}*/

#endif

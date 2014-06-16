#ifndef _CROS_API_H_
#define _CROS_API_H_

typedef enum
{
  CROS_API_NONE = 0,
  CROS_API_REGISTER_SERVICE,
  CROS_API_UNREGISTER_SERVICE,
  CROS_API_UNREGISTER_PUBLISHER,
  CROS_API_UNREGISTER_SUBSCRIBER,
  CROS_API_REGISTER_PUBLISHER,
  CROS_API_REGISTER_SUBSCRIBER,
  CROS_API_LOOKUP_NODE,
  CROS_API_GET_PUBLISHED_TOPICS,
  CROS_API_GET_TOPIC_TYPES,
  CROS_API_GET_SYSTEM_STATE,
  CROS_API_GET_URI,
  CROS_API_LOOKUP_SERVICE,
  CROS_API_GET_BUS_STATS,
  CROS_API_GET_BUS_INFO,
  CROS_API_GET_MASTER_URI,
  CROS_API_SHUTDOWN,
  CROS_API_GET_PID,
  CROS_API_GET_SUBSCRIPTIONS,
  CROS_API_GET_PUBLICATIONS,
  CROS_API_PARAM_UPDATE,
  CROS_API_PUBLISHER_UPDATE,
  CROS_API_REQUEST_TOPIC
} CROS_API_METHOD;

/*! \defgroup cros_api cROS APIs
 * 
 * Internal implemenation (via XMLRPC protocol) of the  ROS  
 * Master, Parameter Server and Slave APIs
 * NOTE: this are cROS internal functions, usually you don't need to use them.
 */

/*! \addtogroup cros_api
 *  @{
 */

struct CrosNode;

/*! \brief Prepare a XMLRPC message for a ROS XMLRPC protocol function call,
 *         This function modify the internal member of the XmlrpcProcess client_proc
 *         of a cROS node. The function to call is chosen checking the currente 
 *         node's internal state (enum CrosNodeState) ,
 * 
 *  \param n Pointer to the CrosNode object
 *  \param client_idx The client id that manage the request
 */
void cRosApiPrepareRequest(struct CrosNode *n, int client_idx );


/*! \brief Parse a XMLRPC response generated from a previous API function call ( see cRosApiPrepareRequest() ),
 *         eventually changing the internal state of the node n
 * 
 *  \param n Ponter to the CrosNode object
 *  \param client_idx The client id that manage the response
 * 
 *  \return Returns 0 on success, -1 on failure
 */
int cRosApiParseResponse(struct CrosNode *n, int client_idx  );

/*! \brief Parse a XMLRPC request (i.e., an API function call generated from roscore or anther node), given 
 *         the XmlrpcProcess server_proc with index server_idx.
 *         Eventually this function perfomrs the requested actions, eventually changing the internal state of the node n.
 *         Moreover, it prepares the related XMLRPC response.
 * 
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the XmlrpcProcess ( xmlrpc_server_proc[server_idx] ) to be considered for the parsing and
 *                    the response generation
 *
 *  \return Returns 0 on success, -1 on failure
 */
int cRosApiParseRequestPrepareResponse(struct CrosNode *n, int server_idx );

const char * getMethodName(CROS_API_METHOD method);
CROS_API_METHOD getMethodCode(const char *method);
int isRosMasterApi(CROS_API_METHOD method);
int isRosSlaveApi(CROS_API_METHOD method);

/*! @}*/

#endif

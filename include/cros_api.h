#ifndef _CROS_API_H_
#define _CROS_API_H_

#include "cros_node.h"

/*! \defgroup cros_api cROS APIs
 * 
 * Internal implemenation (via XMLRPC protocol) of the  ROS  
 * Master, Parameter Server and Slave APIs
 * NOTE: this are cROS internal functions, usually you don't need to use them.
 */

/*! \addtogroup cros_api
 *  @{
 */

/*! \brief Prepare a XMLRPC message for a ROS XMLRPC protocol function call,
 *         This function modify the internal member of the XmlrpcProcess client_proc
 *         of a cROS node. The function to call is chosen checking the currente 
 *         node's internal state (enum CrosNodeState) ,
 * 
 *  \param n Pointer to the CrosNode object
 *  \param client_idx The client id that manage the request
 */
void cRosApiPrepareRequest( CrosNode *n, int client_idx );


/*! \brief Parse a XMLRPC response generated from a previous API function call ( see cRosApiPrepareRequest() ),
 *         eventually changing the internal state of the node n
 * 
 *  \param n Ponter to the CrosNode object
 *  \param client_idx The client id that manage the response
 * 
 *  \return Returns 1 on success, 0 on failure
 */
int cRosApiParseResponse( CrosNode *n, int client_idx  );

/*! \brief Parse a XMLRPC request (i.e., an API function call generated from roscore or anther node), given 
 *         the XmlrpcProcess server_proc with index server_idx.
 *         Eventually this function perfomrs the requested actions, eventually changing the internal state of the node n.
 *         Moreover, it prepares the related XMLRPC response.
 * 
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the XmlrpcProcess ( xmlrpc_server_proc[server_idx] ) to be considered for the parsing and
 *                    the response generation
 */
void cRosApiParseRequestPrepareResponse( CrosNode *n, int server_idx );

/*! @}*/

#endif

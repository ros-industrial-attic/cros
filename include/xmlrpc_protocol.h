#ifndef _XMLRPC_PROTOCOL_H_
#define _XMLRPC_PROTOCOL_H_

#include "xmlrpc_params_vector.h"
#include "dyn_string.h"

/*! \defgroup xmlrpc_protocol XMLRPC protocol */

/*! \addtogroup xmlrpc_protocol 
 *  @{
 */

typedef enum
{ 
  XMLRPC_MESSAGE_REQUEST,
  XMLRPC_MESSAGE_RESPONSE,
  XMLRPC_MESSAGE_UNKNOWN
}XmlrpcMessageType;

typedef enum
{
  XMLRPC_PARSER_ERROR,
  XMLRPC_PARSER_INCOMPLETE,
  XMLRPC_PARSER_DONE
}XmlrpcParserState;


/*! \brief Generate a XMLRPC over HTTP message and store it into a dynamic string
 * 
 *  \param host The hostname where the master runs
 *  \param port The port where the master runs
 *  \param type The message type (XMLRPC_MESSAGE_REQUEST or XMLRPC_MESSAGE_RESPONSE )
 *  \param method The RPC method to invoke ( used only if type == XMLRPC_MESSAGE_REQUEST )
 *  \param params Vector of arguments to the RPC call
 *  \param message Pointer to the (output) dynamic string that will contain the generated message
 */
void generateXmlrpcMessage( const char*host, unsigned short port, XmlrpcMessageType type, 
                            const char *method, XmlrpcParamVector *params, DynString *message );

/*! \brief Parse a XMLRPC over HTTP message
 * 
 *  \param message Pointer to the input dynamic string that will contain the message to be parsed
 *  \param type The message type (XMLRPC_MESSAGE_REQUEST or XMLRPC_MESSAGE_RESPONSE )
 *  \param method The RPC method to invoke ( used only if type == XMLRPC_MESSAGE_REQUEST )
 *  \param response Output vector of XMLRPC parameters with the set of arguments to the RPC call
 * 
 *  \return XMLRPC_PARSER_DONE if the message has ben successfully, 
 *          XMLRPC_PARSER_INCOMPLETE if the message is incomplete,
 *          XMLRPC_PARSER_ERROR on failure
 */
XmlrpcParserState parseXmlrpcMessage(DynString *message, XmlrpcMessageType *type,
                                     DynString *method, XmlrpcParamVector *response,
                                     char host[256], int *port);

/*! @}*/
#endif
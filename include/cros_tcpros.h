#ifndef _CROS_TCPROS_H_
#define _CROS_TCPROS_H_

#include "cros_node.h"

typedef enum
{
  TCPROS_PARSER_ERROR = 0,
  TCPROS_PARSER_HEADER_INCOMPLETE,
  TCPROS_PARSER_DATA_INCOMPLETE,
  TCPROS_PARSER_DONE
} TcprosParserState;

enum
{
    TCPROS_OK_BYTE_FAIL = 0,
    TCPROS_OK_BYTE_SUCCESS = 1
};

/*! \brief Parse a TCPROS header sent initially from a subscriber
 *
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the TcprosProcess ( tcpros_server_proc[server_idx] ) to be considered for the parsing
 *
 *  \return Returns TCPROS_PARSER_DONE if the header is successfully parsed,
 *          TCPROS_PARSER_HEADER_INCOMPLETE if the header is incomplete,
 *          or TCPROS_PARSER_ERROR on failure
 */
TcprosParserState cRosMessageParseSubcriptionHeader( CrosNode *n, int server_idx );

/*! \brief Parse a TCPROS header sent from a publisher
 *
 *  \param n Ponter to the CrosNode object
 *  \param client_idx Index of the TcprosProcess ( tcpros_client_proc[server_idx] ) to be considered for the parsing
 *
 *  \return Returns TCPROS_PARSER_DONE if the header is successfully parsed,
 *          TCPROS_PARSER_HEADER_INCOMPLETE if the header is incomplete,
 *          or TCPROS_PARSER_ERROR on failure
 */
TcprosParserState cRosMessageParsePublicationHeader( CrosNode *n, int client_idx );

/*! \brief Prepare a TCPROS header to be initially sent to a publisher
 *
 *  \param n Ponter to the CrosNode object
 *  \param client_idx Index of the TcprosProcess ( tcpros_client_proc[server_idx] ) to be considered
 */
void cRosMessagePrepareSubcriptionHeader( CrosNode *n, int client_idx );

/*! \brief Prepare a TCPROS header to be initially sent back to a subscriber
 *
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the TcprosProcess ( tcpros_server_proc[server_idx] ) to be considered
 */
void cRosMessagePreparePublicationHeader( CrosNode *n, int server_idx );

/*! \brief Prepare a TCPROS message (with data) to be sent to a subscriber
 *
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the TcprosProcess ( tcpros_server_proc[server_idx] ) to be considered
 *  \return CROS_SUCCESS_ERR_PACK on success, otherwise an error code
 */
cRosErrCodePack cRosMessagePreparePublicationPacket( CrosNode *n, int server_idx );

/*! \brief Read the TCPROS message (with data) received from the publisher
 *
 *  \param n Ponter to the CrosNode object
 *  \param client_idx Index of the TcprosProcess ( tcpros_client_proc[server_idx] ) to be considered for the parsing
 *  \return CROS_SUCCESS_ERR_PACK on success, otherwise an error code
 */
cRosErrCode cRosMessageParsePublicationPacket( CrosNode *n, int client_idx );

/*! \brief Parse a RCPROS header sent from a service caller
 *
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the TcprosProcess ( rpcros_server_proc[server_idx] ) to be considered for the parsing
 *
 *  \return Returns TCPROS_PARSER_DONE if the header is successfully parsed,
 *          or TCPROS_PARSER_ERROR on failure
 */
TcprosParserState cRosMessageParseServiceCallerHeader( CrosNode *n, int server_idx);

/*! \brief Parse a RCPROS header sent by a service provider
 *
 *  \param n Ponter to the CrosNode object
 *  \param client_idx Index of the TcprosProcess ( rpcros_server_proc[server_idx] ) to be considered for the parsing
 *
 *  \return Returns TCPROS_PARSER_DONE if the header is successfully parsed,
 *          or TCPROS_PARSER_ERROR on failure
 */
TcprosParserState cRosMessageParseServiceProviderHeader( CrosNode *n, int client_idx);

/*! \brief Prepare a RCPROS header to be sent to a service provider
 *
 *  \param n Ponter to the CrosNode object
 *  \param client_idx Index of the TcprosProcess ( rpcros_client_proc[client_idx] ) to be considered
 */
void cRosMessagePrepareServiceCallHeader( CrosNode *n, int client_idx);

/*! \brief Prepare a RCPROS packet to be sent to a service provider
 *
 *  \param n Ponter to the CrosNode object
 *  \param client_idx Index of the TcprosProcess ( rpcros_client_proc[client_idx] ) to be considered
 *  \return CROS_SUCCESS_ERR_PACK on success, otherwise an error code
 */
cRosErrCodePack cRosMessagePrepareServiceCallPacket( CrosNode *n, int client_idx );

/*! \brief Prepare a RCPROS header to be initially sent back to a service caller
 *
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the TcprosProcess ( rpcros_server_proc[server_idx] ) to be considered
 */
void cRosMessagePrepareServiceProviderHeader( CrosNode *n, int server_idx);

/*! \brief Read the RPCROS packet (with data) received from the service provider
 *
 *  \param n Ponter to the CrosNode object
 *  \param client_idx Index of the TcprosProcess ( rpcros_client_proc[client_idx] ) to be considered for the parsing
 *  \return CROS_SUCCESS_ERR_PACK on success, otherwise an error code
 */
cRosErrCodePack cRosMessageParseServiceResponsePacket( CrosNode *n, int client_idx );

/*! \brief Prepare a RCPROS response to be sent back to a service caller
 *
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the TcprosProcess ( rpcros_server_proc[server_idx] ) to be considered
 *  \return CROS_SUCCESS_ERR_PACK on success, otherwise an error code
 */
cRosErrCodePack cRosMessagePrepareServiceResponsePacket( CrosNode *n, int server_idx);

/*! \brief Prepare a RCPROS header to be initially sent to a service provider
 *
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the TcprosProcess ( rpcros_client_proc[client_idx] ) to be considered
 */
void cRosMessagePrepareServiceCallerHeader(CrosNode *n, int server_idx);

#endif // _CROS_TCPROS_H_

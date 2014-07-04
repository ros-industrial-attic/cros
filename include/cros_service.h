#ifndef _CROS_SERVICE_H_
#define _CROS_SERVICE_H_

#include "cros_node.h"
#include "cros_message.h"

/*! \defgroup cros_message cROS TCPROS 
 * 
 *  Implemenation of the TCPROS protocol for topic message exchanges
 */

/*! \addtogroup cros_message
 *  @{
 */
static const char* FILEEXT_SRV = "srv";

// string that denotes the separation between request/response service parts
static const char* SRV_DELIMITER = "---";

struct t_srvDef
{
    char* name;
    char* package;
    char* root_dir;
    char* plain_text;
    cRosMessageDef* request;
    cRosMessageDef* response;
};

typedef struct t_srvDef cRosSrvDef;

struct cRosService
{
  cRosMessage request;
  cRosMessage response;
  char* md5sum;
};

typedef struct cRosService cRosService;

cRosService * cRosServiceNew();
void cRosServiceInit(cRosService* service);
void cRosServiceBuild(cRosService* service, const char* filepath);
void cRosServiceRelease(cRosService* service);
void cRosServiceFree(cRosService* service);

void cRosServiceBuildInner(cRosMessage *request, cRosMessage *response, char *md5sum, const char* filepath);
void initCrosSrv(cRosSrvDef* srv);

int getFileDependenciesSrv(char* filename, cRosSrvDef* srv, msgDep* deps);

//  Compute full text of service, including text of embedded
//  types.  The text of the main srv is listed first. Embedded
//  srv files are denoted first by an 80-character '=' separator,
//  followed by a type declaration line,'MSG: pkg/type', followed by
//  the text of the embedded type.
char* computeFullTextSrv(cRosSrvDef* srv, msgDep* deps);

int loadFromFileSrv(char* filename, cRosSrvDef* srv);

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
 */
void cRosMessagePreparePublicationPacket( CrosNode *n, int server_idx );

/*! \brief Read the TCPROS message (with data) received from the publisher
 *
 *  \param n Ponter to the CrosNode object
 *  \param client_idx Index of the TcprosProcess ( tcpros_client_proc[server_idx] ) to be considered for the parsing
 */
void cRosMessageParsePublicationPacket( CrosNode *n, int client_idx );

/*! \brief Parse a RCPROS header sent from a service caller
 *
 *  \param n Ponter to the CrosNode object
 *  \param client_idx Index of the TcprosProcess ( rpcros_server_proc[server_idx] ) to be considered for the parsing
 *
 *  \return Returns TCPROS_PARSER_DONE if the header is successfully parsed,
 *          TCPROS_PARSER_HEADER_INCOMPLETE if the header is incomplete,
 *          or TCPROS_PARSER_ERROR on failure
 */
TcprosParserState cRosMessageParseServiceCallerHeader( CrosNode *n, int server_idx);

/*! \brief Prepare a RCPROS header to be initially sent back to a service caller
 *
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the TcprosProcess ( rpcros_server_proc[server_idx] ) to be considered
 */
void cRosMessagePrepareServiceProviderHeader( CrosNode *n, int server_idx);

/*! \brief Prepare a RCPROS response to be sent back to a service caller
 *
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the TcprosProcess ( rpcros_server_proc[server_idx] ) to be considered
 */
void cRosMessagePrepareServiceResponsePacket( CrosNode *n, int server_idx);

/*! @}*/

#endif

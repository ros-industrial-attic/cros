#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN // This define speeds up the build process by excluding some parts of the Windows header
#  include <windows.h>
#else
#  include <unistd.h>
#endif

#include "cros_node.h"
#include "cros_api_internal.h"
#include "cros_api.h"
#include "cros_message.h"
#include "cros_clock.h"
#include "cros_defs.h"
#include "cros_node_api.h"
#include "cros_tcpros.h"
#include "tcpip_socket.h"
#include "cros_log.h"

static void initPublisherNode(PublisherNode *node);
static void initSubscriberNode(SubscriberNode *node);
static void initServiceProviderNode(ServiceProviderNode *node);
static void initServiceCallerNode(ServiceCallerNode *node);
static void initParameterSubscrition(ParameterSubscription *subscription);
static int enqueueSubscriberAdvertise(CrosNode *node, int subidx);
static int enqueuePublisherAdvertise(CrosNode *node, int pubidx);
static int enqueueServiceAdvertise(CrosNode *node, int servivceidx);
static int enqueueServiceLookup(CrosNode *node, int serviceidx);
static int enqueueParameterSubscription(CrosNode *node, int parameteridx);
static int enqueueParameterUnsubscription(CrosNode *node, int parameteridx);
static void getIdleXmplrpcClients(CrosNode *node, int array[], size_t *count);
static int enqueueSlaveApiCallInternal(CrosNode *node, RosApiCall *call);
static int enqueueMasterApiCallInternal(CrosNode *node, RosApiCall *call);
static void printNodeProcState( CrosNode *n );

FILE *Msg_output = NULL; //! The pointer to file stream used to print local messages (except debug messages). If it is NULL (default value), stdout is used.

static int openXmlrpcClientSocket( CrosNode *n, int i )
{
  int ret;
  if( !tcpIpSocketOpen( &(n->xmlrpc_client_proc[i].socket) ) ||
      !tcpIpSocketSetReuse( &(n->xmlrpc_client_proc[i].socket) ) ||
      !tcpIpSocketSetNonBlocking( &(n->xmlrpc_client_proc[i].socket) ) )
  {
    PRINT_ERROR("openXmlrpcClientSocket() at index %d failed", i);
    ret=-1;
  }
  else
  {
    ret=0; // success
  }
  return(ret);
}

static int openTcprosClientSocket( CrosNode *n, int i )
{
  int ret;
  if( !tcpIpSocketOpen( &(n->tcpros_client_proc[i].socket) ) ||
      !tcpIpSocketSetReuse( &(n->tcpros_client_proc[i].socket) ) ||
      !tcpIpSocketSetNonBlocking( &(n->tcpros_client_proc[i].socket) ) )
  {
    PRINT_ERROR("openTcprosClientSocket() at index %d failed", i);
    ret=-1;
  }
  else
  {
    ret=0; // success
  }
  return(ret);
}

static int openRpcrosClientSocket( CrosNode *n, int i )
{
  int ret;
  if( !tcpIpSocketOpen( &(n->rpcros_client_proc[i].socket) ) ||
      !tcpIpSocketSetReuse( &(n->rpcros_client_proc[i].socket) ) ||
      !tcpIpSocketSetNonBlocking( &(n->rpcros_client_proc[i].socket) ) )
  {
    PRINT_ERROR("openRpcrosClientSocket() at index %d failed", i);
    ret=-1;
  }
  else
  {
    ret=0; // success
  }
  return(ret);
}

static int openXmlrpcListnerSocket( CrosNode *n )
{
  int ret;
  if( !tcpIpSocketOpen( &(n->xmlrpc_listner_proc.socket) ) ||
      !tcpIpSocketSetReuse( &(n->xmlrpc_listner_proc.socket) ) ||
      !tcpIpSocketSetNonBlocking( &(n->xmlrpc_listner_proc.socket) ) ||
      !tcpIpSocketBindListen( &(n->xmlrpc_listner_proc.socket), n->host, 0, CN_MAX_XMLRPC_SERVER_CONNECTIONS ) )
  {
    PRINT_ERROR("openXmlrpcListnerSocket() failed");
    ret=-1;
  }
  else
  {
    n->xmlrpc_port = tcpIpSocketGetPort( &(n->xmlrpc_listner_proc.socket) );
    PRINT_VDEBUG ( "openXmlrpcListnerSocket () : Accepting xmlrpc connections at port %d\n", n->xmlrpc_port );
    ret=0; // success
  }
  return(ret);
}

static int openRpcrosListnerSocket( CrosNode *n )
{
  int ret;
  if( !tcpIpSocketOpen( &(n->rpcros_listner_proc.socket) ) ||
      !tcpIpSocketSetReuse( &(n->rpcros_listner_proc.socket) ) ||
      !tcpIpSocketSetNonBlocking( &(n->rpcros_listner_proc.socket) ) ||
      !tcpIpSocketBindListen( &(n->rpcros_listner_proc.socket), n->host, 0, CN_MAX_RPCROS_SERVER_CONNECTIONS ) )
  {
    PRINT_ERROR("openRpcrosListnerSocket() failed");
    ret=-1;
  }
  else
  {
    n->rpcros_port = tcpIpSocketGetPort( &(n->rpcros_listner_proc.socket) );
    PRINT_VDEBUG ( "openRpcrosListnerSocket() : Accepting rcpros connections at port %d\n", n->rpcros_port );
    ret=0; // success
  }
  return(ret);
}

static int openTcprosListnerSocket( CrosNode *n )
{
  int ret;
  if( !tcpIpSocketOpen( &(n->tcpros_listner_proc.socket) ) ||
      !tcpIpSocketSetReuse( &(n->tcpros_listner_proc.socket) ) ||
      !tcpIpSocketSetNonBlocking( &(n->tcpros_listner_proc.socket) ) ||
      !tcpIpSocketBindListen( &(n->tcpros_listner_proc.socket), n->host, 0, CN_MAX_TCPROS_SERVER_CONNECTIONS ) )
  {
    PRINT_ERROR("openTcprosListnerSocket() failed");
    ret=-1;
  }
  else
  {
    n->tcpros_port = tcpIpSocketGetPort( &(n->tcpros_listner_proc.socket) );
    PRINT_VDEBUG ( "openTcprosListnerSocket() : Accepting tcpros connections at port %d\n", n->tcpros_port );
    ret=0; // success
  }
  return(ret);
}

static void closeTcprosProcess(TcprosProcess *process)
{
  tcpIpSocketClose(&process->socket);
  tcprosProcessReset(process);
}

static void closeXmlrpcProcess(XmlrpcProcess *process)
{
  tcpIpSocketClose(&process->socket);
  xmlrpcProcessReset(process);
  xmlrpcProcessChangeState(process, XMLRPC_PROCESS_STATE_IDLE);
}

// This method is used to communicate that some api calls at least attempted
// to complete, like in the case of unregistration when a gracefully shutdown
// is requested
static void handleApiCallAttempt(CrosNode *node, RosApiCall *call)
{
  CrosNodeStatusUsr status;

  initCrosNodeStatus(&status);
  status.provider_idx = call->provider_idx;

  switch(call->method)
  {
    case CROS_API_UNREGISTER_PUBLISHER:
    {
      if (call->provider_idx == -1)
        break;

      PublisherNode *pub = &node->pubs[call->provider_idx];
      status.state = CROS_STATUS_PUBLISHER_UNREGISTERED;
      cRosNodeStatusCallback(&status, pub->context); // calls the publisher application-defined callback function (if specified when creating the publisher)

      // Finally release publisher
      cRosApiReleasePublisher(node, call->provider_idx);
      initPublisherNode(pub);
      call->provider_idx = -1;
      break;
    }
    case CROS_API_UNREGISTER_SUBSCRIBER:
    {
      if (call->provider_idx == -1)
        break;

      SubscriberNode *sub = &node->subs[call->provider_idx];
      status.state = CROS_STATUS_SUBSCRIBER_UNREGISTERED;
      cRosNodeStatusCallback(&status, sub->context);

      // Finally release subscriber
      cRosApiReleaseSubscriber(node, call->provider_idx);
      initSubscriberNode(sub);
      call->provider_idx = -1;
      break;
    }
    case CROS_API_UNREGISTER_SERVICE:
    {
      if (call->provider_idx == -1)
        break;

      ServiceProviderNode *service = &node->service_providers[call->provider_idx];
      status.state = CROS_STATUS_SERVICE_PROVIDER_UNREGISTERED;
      cRosNodeStatusCallback(&status, service->context);

      // Finally release service provider
      cRosApiReleaseServiceProvider(node, call->provider_idx);
      initServiceProviderNode(service);
      call->provider_idx = -1;
      break;
    }
    case CROS_API_UNSUBSCRIBE_PARAM:
    {
      if (call->provider_idx == -1)
        break;

      ParameterSubscription *subscription = &node->paramsubs[call->provider_idx];
      status.state = CROS_STATUS_PARAM_UNSUBSCRIBED;
      status.parameter_key = subscription->parameter_key;
      subscription->status_api_callback(&status, subscription->context);

      // Finally release parameter subscription
      cRosNodeReleaseParameterSubscrition(subscription);
      initParameterSubscrition(subscription);
      call->provider_idx = -1;
      break;
    }
    default:
    {
      break;
    }
  }
}

static void cleanApiCallState(CrosNode *node, RosApiCall *call)
{
  switch(call->method)
  {
    case CROS_API_REQUEST_TOPIC:
    {
      // transitory xmlrpc client processes are checked and cleared one by one in the subscriber unregistration function
      break;
    }
    default:
    {
      break;
    }
  }
}

static void handleXmlrpcClientError(CrosNode *node, int i)
{
  XmlrpcProcess *proc = &node->xmlrpc_client_proc[i];
  RosApiCall *call = proc->current_call;

  switch (call->method)
  {
    case CROS_API_REGISTER_SERVICE:
    case CROS_API_REGISTER_PUBLISHER:
    case CROS_API_REGISTER_SUBSCRIBER:
    case CROS_API_SUBSCRIBE_PARAM:
    {
      proc->current_call = NULL;
      enqueueApiCall(&node->master_api_queue, call);
      break;
    }
    default:
    {
      ResultCallback callback = call->result_callback;
      if (callback != NULL)
      {
        // Notifies an error with a NULL result
        callback(call->id, NULL, call->context_data);
      }
      break;
    }
  }

  handleApiCallAttempt(node, call);
  cleanApiCallState(node, call);
  closeXmlrpcProcess(proc);
}

static void handleTcprosClientError(CrosNode *n, int i)
{
  TcprosProcess *process = &n->tcpros_client_proc[i];
  closeTcprosProcess(process);
  // CHECK-ME Riaccoda register subscriber?
}

static void handleXmlrpcServerError(CrosNode *n, int i)
{
  XmlrpcProcess *process = &n->xmlrpc_server_proc[i];
  closeXmlrpcProcess(process);
}

static void handleTcprosServerError(CrosNode *n, int proc_idx)
{
  int list_elem;
  TcprosProcess *process = &n->tcpros_server_proc[proc_idx];
  PublisherNode *pub = &n->pubs[process->topic_idx];

  // Look for the tcpros_server_proc index (proc_idx) in the publisher tcpros_server_proc list (to remove it)
  for(list_elem=0;pub->tcpros_id_list[list_elem]!=-1 && pub->tcpros_id_list[list_elem] != proc_idx;list_elem++);
  if(pub->tcpros_id_list[list_elem] != proc_idx) // tcpros_server_proc index (proc_idx) found
  {
    // Remove index
    for(;pub->tcpros_id_list[list_elem]!=-1;list_elem++)
      pub->tcpros_id_list[list_elem] = pub->tcpros_id_list[list_elem+1];
  }
  else
    PRINT_ERROR("handleTcprosServerError() : TcprosProcess index %i has not been found in Publisher %i", proc_idx, process->topic_idx);

  closeTcprosProcess(process);
}

static void handleRpcrosClientError(CrosNode *n, int i)
{
  TcprosProcess *process = &n->rpcros_client_proc[i];
  closeTcprosProcess(process);
}

static void handleRpcrosServerError(CrosNode *n, int i)
{
  TcprosProcess *process = &n->rpcros_server_proc[i];
  closeTcprosProcess(process);
}

static cRosErrCodePack xmlrpcClientConnect(CrosNode *n, int i)
{
  cRosErrCodePack ret_err;
  PRINT_VVDEBUG ( "xmlrpcClientConnect()\n" );

  ret_err = CROS_SUCCESS_ERR_PACK;
  XmlrpcProcess *client_proc = &(n->xmlrpc_client_proc[i]);

  PRINT_VDEBUG ( "xmlrpcClientConnect() : Connecting\n" );

  if( !client_proc->socket.connected )
  {
    TcpIpSocketState conn_state;
    RosApiCall *xml_call;

    if(!client_proc->socket.open)
      openXmlrpcClientSocket(n, i);

    xml_call = client_proc->current_call;
    if(xml_call == NULL)
    {
      PRINT_ERROR ( "xmlrpcClientConnect() : Invalid XMLRPC call\n" );
      return CROS_UNSPECIFIED_ERR;
    }

    if (i == 0 || isRosMasterApi(xml_call->method))
    {
      conn_state = tcpIpSocketConnect( &(client_proc->socket),
                                         n->roscore_host, n->roscore_port );
    }
    else // slave api or client xmlrpc to be invoked from a subscriber
    {
      conn_state = tcpIpSocketConnect(&client_proc->socket,
                                      xml_call->host, xml_call->port);
    }

    if( conn_state == TCPIPSOCKET_DONE)
    {
      xmlrpcProcessChangeState( client_proc, XMLRPC_PROCESS_STATE_WRITING );
    }
    else if( conn_state == TCPIPSOCKET_IN_PROGRESS )
    {
      PRINT_VDEBUG ( "xmlrpcClientConnect() : Wait: connection is established asynchronously\n" ); // Not connected yet
    }
    else if( conn_state == TCPIPSOCKET_REFUSED )
    {
      PRINT_ERROR("xmlrpcClientConnect() : Connection of XMLRPC client number %i was refused (is the target port not open?)\n", i);
      handleXmlrpcClientError( n, i);
      ret_err = CROS_XMLRPC_CLI_REFUS_ERR;
    }
    else
    {
      if( conn_state != TCPIPSOCKET_FAILED )
         PRINT_ERROR ( "tcprosClientConnect() : invalid connection state\n" );
      PRINT_ERROR("xmlrpcClientConnect() : XMLRPC client number %i can't connect\n", i);
      handleXmlrpcClientError( n, i);
      ret_err = CROS_XMLRPC_CLI_CONN_ERR;
    }
  }

  return ret_err;
}

static cRosErrCodePack doWithXmlrpcClientSocket(CrosNode *n, int i)
{
  cRosErrCodePack ret_err;
  XmlrpcProcess *client_proc;
  PRINT_VVDEBUG ( "doWithXmlrpcClientSocket()\n" );

  ret_err = CROS_SUCCESS_ERR_PACK;
  client_proc = &(n->xmlrpc_client_proc[i]);

  switch( client_proc->state )
  {
    case XMLRPC_PROCESS_STATE_CONNECTING:
    {
      ret_err = xmlrpcClientConnect(n, i);
      break;
    }
    case XMLRPC_PROCESS_STATE_WRITING:
    {
      PRINT_VDEBUG ( "doWithXmlrpcClientSocket() : writing. Xmlrpc client index: %d\n", i);

      cRosApiPrepareRequest( n, i );

      TcpIpSocketState sock_state =  tcpIpSocketWriteString( &(client_proc->socket),
                                                             &(client_proc->message) );
      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          {
          // NB: Node release is done by handleApiCallAttempt when the ros master response is received
          xmlrpcProcessClear(client_proc);
          xmlrpcProcessChangeState( client_proc, XMLRPC_PROCESS_STATE_READING );
          break;
          }

        case TCPIPSOCKET_IN_PROGRESS:
      {
          PRINT_VDEBUG ( "doWithXmlrpcClientSocket() : Write in progress...\n" );
          break;
          }

        case TCPIPSOCKET_DISCONNECTED:
        case TCPIPSOCKET_FAILED:
        default:
          {
          PRINT_ERROR("doWithXmlrpcClientSocket() : Unexpected failure writing request\n");
          handleXmlrpcClientError( n, i );
          ret_err = CROS_XMLRPC_CLI_WRITE_ERR;
          break;
          }
      }
      break;
    }
    case XMLRPC_PROCESS_STATE_READING:
    {
      PRINT_VDEBUG ( "doWithXmlrpcClientSocket() : Reading. Xmlrpc client index: %d\n", i);
      TcpIpSocketState sock_state = tcpIpSocketReadString( &client_proc->socket,
                                                           &client_proc->message );
      XmlrpcParserState parser_state = XMLRPC_PARSER_INCOMPLETE;

      int disconnected = 0;
      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          {
          parser_state = parseXmlrpcMessage( &client_proc->message,
                                             &client_proc->message_type,
                                             NULL,
                                             &client_proc->response,
                                             client_proc->host,
                                             &client_proc->port);
          break;
          }

        case TCPIPSOCKET_IN_PROGRESS:
          break;

        case TCPIPSOCKET_DISCONNECTED:
          {
          parser_state = parseXmlrpcMessage( &client_proc->message,
                                             &client_proc->message_type,
                                             NULL,
                                             &client_proc->response,
                                             client_proc->host,
                                             &client_proc->port);
          disconnected = 1;
          break;
      }

        case TCPIPSOCKET_FAILED:
        default:
          {
          PRINT_ERROR("doWithXmlrpcClientSocket() : Unexpected failure reading response\n" );
          handleXmlrpcClientError( n, i );
          ret_err = CROS_XMLRPC_CLI_READ_ERR;
          break;
          }
      }

      switch ( parser_state )
      {
        case XMLRPC_PARSER_DONE:
          {
          PRINT_VDEBUG ( "doWithXmlrpcClientSocket() : Done with no error\n" );

          int rc = cRosApiParseResponse( n, i );
          handleApiCallAttempt(n, client_proc->current_call);
          if (rc != 0)
          {
            handleXmlrpcClientError( n, i );
          }
          else
          {
            cleanApiCallState(n, client_proc->current_call);
            closeXmlrpcProcess(client_proc);
          }
          break;
          }

        case XMLRPC_PARSER_INCOMPLETE:
          {
          if (disconnected)
            handleXmlrpcClientError( n, i );
          break;
          }

        case XMLRPC_PARSER_ERROR:
        default:
          {
          handleXmlrpcClientError( n, i );
          break;
          }
      }
      break;
    }
    default:
    {
      PRINT_VDEBUG ( "doWithXmlrpcClientSocket() : Invalid XMLRPC process state\n" );
    }
  }
  return ret_err;
}

static cRosErrCodePack doWithXmlrpcServerSocket( CrosNode *n, int i )
{
  cRosErrCodePack ret_err;
  PRINT_VVDEBUG ( "doWithXmlrpcServerSocket()\n" );

  ret_err = CROS_SUCCESS_ERR_PACK;
  XmlrpcProcess *server_proc = &(n->xmlrpc_server_proc[i]);

  if( server_proc->state == XMLRPC_PROCESS_STATE_READING )
  {
    PRINT_VDEBUG ( "doWithXmlrpcServerSocket() : Reading. Xmlrpc server index: %d\n", i );
    TcpIpSocketState sock_state = tcpIpSocketReadString( &(server_proc->socket),
                                                         &(server_proc->message) );
    XmlrpcParserState parser_state = XMLRPC_PARSER_INCOMPLETE;

    switch ( sock_state )
    {
      case TCPIPSOCKET_DONE:
        parser_state = parseXmlrpcMessage( &server_proc->message,
                                           &server_proc->message_type,
                                           &server_proc->method,
                                           &server_proc->params,
                                           server_proc->host,
                                           &server_proc->port);
        break;

      case TCPIPSOCKET_IN_PROGRESS:
        break;

      case TCPIPSOCKET_DISCONNECTED:
        xmlrpcProcessReset( &(n->xmlrpc_server_proc[i]) );
        xmlrpcProcessChangeState( &(n->xmlrpc_server_proc[i]), XMLRPC_PROCESS_STATE_IDLE );
        tcpIpSocketClose( &(n->xmlrpc_server_proc[i].socket) );
        break;
      case TCPIPSOCKET_FAILED:
      default:
        PRINT_ERROR("doWithXmlrpcServerSocket() : Unexpected failure reading request\n");
        handleXmlrpcServerError( n, i );
        break;
    }

    switch ( parser_state )
    {
      case XMLRPC_PARSER_DONE:
        {

        PRINT_VDEBUG ( "doWithXmlrpcServerSocket() : Done reading and parsing with no error\n" );
        int rc = cRosApiParseRequestPrepareResponse( n, i );
        if (rc == -1)
        {
          handleXmlrpcServerError( n, i );
          break;
        }
        xmlrpcProcessChangeState( server_proc, XMLRPC_PROCESS_STATE_WRITING );
        break;
        }
      case XMLRPC_PARSER_INCOMPLETE:
        break;

      case XMLRPC_PARSER_ERROR:
      default:
        {
        handleXmlrpcServerError( n, i );
        break;
        }
    }
  }
  else if( server_proc->state == XMLRPC_PROCESS_STATE_WRITING )
  {
    PRINT_VDEBUG ( "doWithXmlrpcServerSocket() : writing. Xmlrpc server index %d \n", i );
    TcpIpSocketState sock_state =  tcpIpSocketWriteString( &(server_proc->socket),
                                                           &(server_proc->message) );
    switch ( sock_state )
    {
      case TCPIPSOCKET_DONE:
        xmlrpcProcessReset( server_proc );
        xmlrpcProcessChangeState( server_proc, XMLRPC_PROCESS_STATE_READING );
        break;

      case TCPIPSOCKET_IN_PROGRESS:
        break;

      case TCPIPSOCKET_DISCONNECTED:
        xmlrpcProcessReset( server_proc );
        xmlrpcProcessChangeState( server_proc, XMLRPC_PROCESS_STATE_IDLE );
        tcpIpSocketClose( &(server_proc->socket) );
        break;

      case TCPIPSOCKET_FAILED:
      default:
        PRINT_ERROR("doWithXmlrpcServerSocket() : Unexpected failure writing response\n");
        handleXmlrpcServerError( n, i );
        break;
    }
  }
  return ret_err;
}

static cRosErrCodePack tcprosClientConnect( CrosNode *n, int client_idx)
{
  cRosErrCodePack ret_err;
  PRINT_VVDEBUG ( "tcprosClientConnect()\n" );

  ret_err = CROS_SUCCESS_ERR_PACK;
  TcprosProcess *client_proc = &(n->tcpros_client_proc[client_idx]);

  if(!client_proc->socket.open)
    openTcprosClientSocket(n, client_idx);

  tcprosProcessClear( client_proc ); // clear packet buffer and variable indicating bytes left to receive (left_to_recv)
  TcpIpSocketState conn_state = tcpIpSocketConnect( &(client_proc->socket),
                                       client_proc->sub_tcpros_host, client_proc->sub_tcpros_port );
  switch (conn_state)
  {
    case TCPIPSOCKET_DONE:
    {
      tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_WRITING_HEADER );
      break;
    }
    case TCPIPSOCKET_IN_PROGRESS:
    {
      PRINT_VDEBUG ( "tcprosClientConnect() : Wait: connection is established asynchronously\n" );
      // Wait: connection is established asynchronously
      break;
    }
    case TCPIPSOCKET_REFUSED:
    {
      PRINT_ERROR("tcprosClientConnect() : Connection of TCPROS client number %i was refused (is the target port not open?)\n", client_idx);
      handleTcprosClientError( n, client_idx);
      ret_err = CROS_TCPROS_CLI_REFUS_ERR;
      break;
    }
    default:
    {
      PRINT_ERROR ( "tcprosClientConnect() : invalid connection state\n" );
      // no break execute next case too
    }
    case TCPIPSOCKET_FAILED:
    {
      PRINT_ERROR ( "tcprosClientConnect() : An error occurred when TCPROS client number %i was trying to connect\n", client_idx);
      handleTcprosClientError( n, client_idx);
      ret_err = CROS_TCPROS_CLI_CONN_ERR;
      break;
    }
  }

  return ret_err;
}

static cRosErrCodePack doWithTcprosClientSocket( CrosNode *n, int client_idx)
{
  cRosErrCodePack ret_err;
  PRINT_VVDEBUG ( "doWithTcprosClientSocket()\n" );

  ret_err = CROS_SUCCESS_ERR_PACK;
  TcprosProcess *client_proc = &(n->tcpros_client_proc[client_idx]);

  switch ( client_proc->state )
  {
    case TCPROS_PROCESS_STATE_CONNECTING:
    {
      ret_err = tcprosClientConnect( n, client_idx);
      break;
    }
    case TCPROS_PROCESS_STATE_WRITING_HEADER:
    {
      cRosMessagePrepareSubcriptionHeader( n, client_idx);

      TcpIpSocketState sock_state =  tcpIpSocketWriteBuffer( &(client_proc->socket),
                                                           &(client_proc->packet) );
      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          tcprosProcessClear( client_proc );
          client_proc->left_to_recv = sizeof(uint32_t);
          tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_READING_HEADER_SIZE );

          break;

        case TCPIPSOCKET_IN_PROGRESS:
          break;

        case TCPIPSOCKET_DISCONNECTED:
          closeTcprosProcess( client_proc );
          break;

        case TCPIPSOCKET_FAILED:
        default:
          handleTcprosClientError( n, client_idx );
          break;
      }
      break;
    }
    case TCPROS_PROCESS_STATE_READING_HEADER_SIZE:
    {
      size_t n_reads;
      TcpIpSocketState sock_state = tcpIpSocketReadBufferEx( &(client_proc->socket),
                                                          &(client_proc->packet),
                                                          client_proc->left_to_recv,
                                                          &n_reads);

      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          client_proc->left_to_recv -= n_reads;
          if (client_proc->left_to_recv == 0)
          {
            const unsigned char *data = dynBufferGetCurrentData(&client_proc->packet);
            uint32_t header_size;
            header_size = ROS_TO_HOST_UINT32(*(uint32_t *)data);
            tcprosProcessClear( client_proc );
            client_proc->left_to_recv = header_size;
            tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_READING_HEADER);
            goto read_header;
          }
          break;
        case TCPIPSOCKET_IN_PROGRESS:
          break;
        case TCPIPSOCKET_DISCONNECTED:
        case TCPIPSOCKET_FAILED:
        default:
          handleTcprosClientError( n, client_idx );
          break;
      }

      break;
    }
    read_header:
    case TCPROS_PROCESS_STATE_READING_HEADER:
    {
      size_t n_reads;
      TcpIpSocketState sock_state = tcpIpSocketReadBufferEx( &(client_proc->socket),
                                                          &(client_proc->packet),
                                                          client_proc->left_to_recv,
                                                          &n_reads);
      TcprosParserState parser_state = TCPROS_PARSER_ERROR;

      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          client_proc->left_to_recv -= n_reads;
          if (client_proc->left_to_recv == 0)
          {
            parser_state = cRosMessageParsePublicationHeader( n, client_idx );
            break;
          }
          parser_state = TCPROS_PARSER_HEADER_INCOMPLETE;
          break;
        case TCPIPSOCKET_IN_PROGRESS:
          parser_state = TCPROS_PARSER_HEADER_INCOMPLETE;
          break;
        case TCPIPSOCKET_DISCONNECTED:
        case TCPIPSOCKET_FAILED:
        default:
          handleTcprosClientError( n, client_idx );
          break;
      }

      switch ( parser_state )
      {
        case TCPROS_PARSER_DONE:
          tcprosProcessClear( client_proc );
          client_proc->left_to_recv = sizeof(uint32_t);
          tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_READING_SIZE );
          break;
        case TCPROS_PARSER_HEADER_INCOMPLETE:
          break;
        case TCPROS_PARSER_ERROR:
        default:
          handleTcprosClientError( n, client_idx );
          break;
      }
      break; // To avoid blocking ???
    }
    case TCPROS_PROCESS_STATE_READING_SIZE:
    {
      size_t n_reads;
      TcpIpSocketState sock_state = tcpIpSocketReadBufferEx( &(client_proc->socket),
                                                          &(client_proc->packet),
                                                          client_proc->left_to_recv,
                                                          &n_reads);

      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          client_proc->left_to_recv -= n_reads;
          if (client_proc->left_to_recv == 0)
          {
            const unsigned char *data = dynBufferGetCurrentData(&client_proc->packet);
            uint32_t msg_size = 0;
            msg_size = ROS_TO_HOST_UINT32(*(uint32_t *)data);
            tcprosProcessClear( client_proc );
            client_proc->left_to_recv = msg_size;
            tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_READING);
            goto read_msg;
          }
          break;
        case TCPIPSOCKET_IN_PROGRESS:
          break;
        case TCPIPSOCKET_DISCONNECTED:
        case TCPIPSOCKET_FAILED:
        default:
          handleTcprosClientError( n, client_idx );
          break;
      }
      break;
    }
    read_msg:
    case TCPROS_PROCESS_STATE_READING:
    {
      size_t n_reads;
      TcpIpSocketState sock_state = tcpIpSocketReadBufferEx( &(client_proc->socket),
                                                          &(client_proc->packet),
                                                          client_proc->left_to_recv,
                                                          &n_reads);

      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          client_proc->left_to_recv -= n_reads;
          if (client_proc->left_to_recv == 0)
          {
              ret_err = cRosMessageParsePublicationPacket(n, client_idx);
              tcprosProcessClear( client_proc );
              client_proc->left_to_recv = sizeof(uint32_t);
              tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_READING_SIZE );
          }
          break;
        case TCPIPSOCKET_IN_PROGRESS:
          break;
        case TCPIPSOCKET_DISCONNECTED:
        case TCPIPSOCKET_FAILED:
        default:
          handleTcprosClientError( n, client_idx );
          break;
      }
      break;
    }
    default:
    {
      PRINT_ERROR( "doWithTcprosClientSocket() : Invalid TCPROS process state\n" );
    }

  }
  return ret_err;
}

static cRosErrCodePack doWithTcprosServerSocket( CrosNode *n, int i )
{
  cRosErrCodePack ret_err;
  PRINT_VVDEBUG ( "doWithTcprosServerSocket()\n" );

  ret_err = CROS_SUCCESS_ERR_PACK; // Default return value
  TcprosProcess *server_proc = &(n->tcpros_server_proc[i]);

  if( server_proc->state == TCPROS_PROCESS_STATE_READING_HEADER)
  {
    PRINT_VDEBUG ( "doWithTcprosServerSocket() : Reading header. Tcpros server index %d \n", i );
    tcprosProcessClear( server_proc );
    TcpIpSocketState sock_state = tcpIpSocketReadBuffer( &(server_proc->socket),
                                                         &(server_proc->packet) );
    TcprosParserState parser_state = TCPROS_PARSER_ERROR;

    switch ( sock_state )
    {
      case TCPIPSOCKET_DONE:
        parser_state = cRosMessageParseSubcriptionHeader( n, i );
        break;

      case TCPIPSOCKET_IN_PROGRESS:
      	parser_state = TCPROS_PARSER_HEADER_INCOMPLETE;
        break;

      case TCPIPSOCKET_DISCONNECTED:
      case TCPIPSOCKET_FAILED:
      default:
        PRINT_INFO( "doWithTcprosServerSocket() : Client disconnected?\n" );
        handleTcprosServerError( n, i );
        break;
    }

    switch ( parser_state )
    {
      case TCPROS_PARSER_DONE:

        PRINT_VDEBUG ( "doWithTcprosServerSocket() : Done reading and parsing with no error\n" );
        tcprosProcessClear( server_proc );
        cRosMessagePreparePublicationHeader( n, i );
        tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_WRITING ); // Proceed to write the header
        break;
      case TCPROS_PARSER_HEADER_INCOMPLETE:
        break;

      case TCPROS_PARSER_ERROR:
      default:
        PRINT_INFO( "doWithTcprosServerSocket() : Parser error\n" );
        handleTcprosServerError( n, i );
        break;
    }
  }
  else if( server_proc->state == TCPROS_PROCESS_STATE_START_WRITING ||
           server_proc->state == TCPROS_PROCESS_STATE_WRITING ) // It is time to write a message or header
  {
    PRINT_VDEBUG ( "doWithTcprosServerSocket() : writing. Tcpros server index: %d \n", i );
    if( server_proc->state == TCPROS_PROCESS_STATE_START_WRITING ) // Start to publish a message
    {
      tcprosProcessClear( server_proc );
      ret_err = cRosMessagePreparePublicationPacket( n, i );
      tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_WRITING );
    }
    TcpIpSocketState sock_state =  tcpIpSocketWriteBuffer( &(server_proc->socket),
                                                           &(server_proc->packet) );

    switch ( sock_state )
    {
      case TCPIPSOCKET_DONE:
        PRINT_VDEBUG ( "doWithTcprosServerSocket() : Done writing with no error\n" );
        tcprosProcessClear( server_proc );
        tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_WAIT_FOR_WRITING ); // Wait before publishing a new message
        break;

      case TCPIPSOCKET_IN_PROGRESS:
        break;

      case TCPIPSOCKET_DISCONNECTED:
      case TCPIPSOCKET_FAILED:
      default:
        PRINT_INFO( "doWithTcprosServerSocket() : Client disconnected?\n" );
        handleTcprosServerError(n, i);
        break;
    }
  }
  return ret_err;
}

static cRosErrCodePack rpcrosClientConnect(CrosNode *n, int client_idx)
{
  cRosErrCodePack ret_err;
  PRINT_VVDEBUG ( "rpcrosClientConnect()\n" );

  ret_err = CROS_SUCCESS_ERR_PACK;
  TcprosProcess *client_proc = &(n->rpcros_client_proc[client_idx]);

  if(!client_proc->socket.open)
    openRpcrosClientSocket(n, client_idx);

  ServiceCallerNode *service_caller = &(n->service_callers[client_proc->service_idx]);
  tcprosProcessClear( client_proc );
  TcpIpSocketState conn_state = tcpIpSocketConnect( &(client_proc->socket), service_caller->service_host, service_caller->service_port );
  switch (conn_state)
  {
    case TCPIPSOCKET_DONE:
    {
      tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_WRITING_HEADER );
      break;
    }
    case TCPIPSOCKET_IN_PROGRESS:
    {
      PRINT_VDEBUG ( "rpcrosClientConnect() : Wait: connection is established asynchronously\n" );
      // Wait: connection is established asynchronously
      break;
    }
    case TCPIPSOCKET_REFUSED:
    {
      PRINT_ERROR("xmlrpcClientConnect() : Connection of RPCROS client number %i was refused (is the target port not open?)\n", client_idx);
      handleRpcrosClientError( n, client_idx);
      ret_err = CROS_RPCROS_CLI_REFUS_ERR;
      break;
    }
    default:
    {
      PRINT_ERROR ( "rpcrosClientConnect() : invalid connection state\n" );
      // no break execute next case too
    }
    case TCPIPSOCKET_FAILED:
    {
      PRINT_ERROR ( "rpcrosClientConnect() : An error occurred when RPCROS client number %i was trying to connect\n", client_idx);
      handleRpcrosClientError( n, client_idx);
      ret_err = CROS_RPCROS_CLI_CONN_ERR;
      break;
    }
  }

  return ret_err;
}

static cRosErrCodePack doWithRpcrosClientSocket(CrosNode *n, int client_idx)
{
  cRosErrCodePack ret_err;
  PRINT_VVDEBUG ( "doWithRpcrosClientSocket()\n" );

  ret_err = CROS_SUCCESS_ERR_PACK;
  TcprosProcess *client_proc = &(n->rpcros_client_proc[client_idx]);

  switch ( client_proc->state )
  {
    case  TCPROS_PROCESS_STATE_CONNECTING:
    {
      ret_err = rpcrosClientConnect(n, client_idx);
      break;
    }
    case TCPROS_PROCESS_STATE_WRITING_HEADER:
    {
      cRosMessagePrepareServiceCallHeader(n, client_idx);

      TcpIpSocketState sock_state =  tcpIpSocketWriteBuffer( &(client_proc->socket), &(client_proc->packet) );

      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          tcprosProcessClear( client_proc );
          client_proc->left_to_recv = sizeof(uint32_t);
          tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_READING_HEADER_SIZE );

          break;

        case TCPIPSOCKET_IN_PROGRESS:
          break;

        case TCPIPSOCKET_DISCONNECTED:
          closeTcprosProcess( client_proc );
          break;

        case TCPIPSOCKET_FAILED:
        default:
          handleRpcrosClientError( n, client_idx );
          break;
      }
      break;
    }
    case TCPROS_PROCESS_STATE_READING_HEADER_SIZE:
    {
      size_t n_reads;
      TcpIpSocketState sock_state = tcpIpSocketReadBufferEx( &(client_proc->socket),
                                                          &(client_proc->packet),
                                                          client_proc->left_to_recv,
                                                          &n_reads);

      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          client_proc->left_to_recv -= n_reads;
          if (client_proc->left_to_recv == 0)
          {
            const unsigned char *data = dynBufferGetCurrentData(&client_proc->packet);
            uint32_t header_size;
            header_size = ROS_TO_HOST_UINT32(*(uint32_t *)data);
            tcprosProcessClear( client_proc );
            client_proc->left_to_recv = header_size;
            tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_READING_HEADER);
            goto read_header;
          }
          break;
        case TCPIPSOCKET_IN_PROGRESS:
          break;
        case TCPIPSOCKET_DISCONNECTED:
          tcpIpSocketClose(&client_proc->socket);
          tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_WAIT_FOR_CONNECTING);
          break;
        case TCPIPSOCKET_FAILED:
        default:
          handleRpcrosClientError( n, client_idx );
          break;
      }

      break;
    }
    read_header:
    case TCPROS_PROCESS_STATE_READING_HEADER:
    {
      size_t n_reads;
      TcpIpSocketState sock_state = tcpIpSocketReadBufferEx( &(client_proc->socket),
                                                          &(client_proc->packet),
                                                          client_proc->left_to_recv,
                                                          &n_reads);
      TcprosParserState parser_state = TCPROS_PARSER_ERROR;

      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          client_proc->left_to_recv -= n_reads;
          if (client_proc->left_to_recv == 0)
          {
            parser_state = cRosMessageParseServiceProviderHeader( n, client_idx );
            break;
          }
          parser_state = TCPROS_PARSER_HEADER_INCOMPLETE;
          break;
        case TCPIPSOCKET_IN_PROGRESS:
          parser_state = TCPROS_PARSER_HEADER_INCOMPLETE;
          break;
        case TCPIPSOCKET_DISCONNECTED:
          tcpIpSocketClose(&client_proc->socket);
          tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_WAIT_FOR_CONNECTING);
          break;
        case TCPIPSOCKET_FAILED:
        default:
          handleRpcrosClientError( n, client_idx );
          break;
      }

      switch ( parser_state )
      {
        case TCPROS_PARSER_DONE:
          tcprosProcessClear( client_proc );
          tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_WAIT_FOR_WRITING );
          break;
        case TCPROS_PARSER_HEADER_INCOMPLETE:
          break;
        case TCPROS_PARSER_ERROR:
        default:
          handleRpcrosClientError( n, client_idx );
          break;
      }
      break;
    }

    case TCPROS_PROCESS_STATE_START_WRITING:
    {
      tcprosProcessClear( client_proc );
      ret_err = cRosMessagePrepareServiceCallPacket(n, client_idx);
      tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_WRITING );
    }

    case TCPROS_PROCESS_STATE_WRITING:
    {
      PRINT_VDEBUG ( "doWithRpcrosClientSocket() : writing. Rpcros client index: %d \n", client_idx );
      TcpIpSocketState sock_state =  tcpIpSocketWriteBuffer( &(client_proc->socket),
                                                               &(client_proc->packet) );

      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          PRINT_VDEBUG ( "doWithRpcrosClientSocket() : Done writing with no error\n" );
          tcprosProcessClear( client_proc ); // Clears only packet and left_to_recv vars
          client_proc->left_to_recv = sizeof(uint32_t) + sizeof(uint8_t);
          tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_READING_SIZE );
          break;

        case TCPIPSOCKET_IN_PROGRESS:
          break;

        case TCPIPSOCKET_DISCONNECTED:
          tcpIpSocketClose(&client_proc->socket);
          tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_WAIT_FOR_CONNECTING);
          break;
        case TCPIPSOCKET_FAILED:
        default:
          PRINT_INFO( "doWithRpcrosClientSocket() : Client disconnected\n" );
          handleRpcrosClientError(n, client_idx);
          break;
      }
      break;
    }

    case TCPROS_PROCESS_STATE_READING_SIZE: // Read 'ok' byte and response-packet size field
    {
      size_t n_reads;
      TcpIpSocketState sock_state = tcpIpSocketReadBufferEx( &(client_proc->socket),
                                                          &(client_proc->packet),
                                                          client_proc->left_to_recv,
                                                          &n_reads);
      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          client_proc->left_to_recv -= n_reads;
          if (client_proc->left_to_recv == 0)
          {
            const uint8_t *data = (const uint8_t *)dynBufferGetCurrentData(&client_proc->packet);
            uint32_t msg_size;
            client_proc->ok_byte = *data;
            msg_size = ROS_TO_HOST_UINT32(*(uint32_t *)(data+1));
            tcprosProcessClear( client_proc );
            client_proc->left_to_recv = msg_size;
            tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_READING);
            goto read_msg;
          }
          break;
        case TCPIPSOCKET_IN_PROGRESS:
          break;
        case TCPIPSOCKET_DISCONNECTED:
          tcpIpSocketClose(&client_proc->socket);
          tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_WAIT_FOR_CONNECTING);
          break;
        case TCPIPSOCKET_FAILED:
        default:
          handleRpcrosClientError( n, client_idx );
          break;
      }
      break;
    }
    read_msg:
    case TCPROS_PROCESS_STATE_READING:
    {
      size_t n_reads;
      TcpIpSocketState sock_state = tcpIpSocketReadBufferEx( &(client_proc->socket),
                                                          &(client_proc->packet),
                                                          client_proc->left_to_recv,
                                                          &n_reads);
      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          client_proc->left_to_recv -= n_reads;
          if (client_proc->left_to_recv == 0)
          {
              ret_err = cRosMessageParseServiceResponsePacket(n, client_idx);
              if(client_proc->persistent)
              {
                tcprosProcessClear( client_proc );
                tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_WAIT_FOR_WRITING );
              }
              else
              {
                tcprosProcessClear( client_proc );
                tcpIpSocketClose( &(client_proc->socket) );
                openRpcrosClientSocket(n, client_idx);
                // The master should be checked before contacting the service provider again,
                // that is, before going to state TCPROS_PROCESS_STATE_CONNECTING again.
                // Services are stateless (unless the persistent parameter is set to 1)
                tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_IDLE );
                if(enqueueServiceLookup(n, client_proc->service_idx) == -1)
                  handleRpcrosClientError( n, client_idx );
              }
          }
          break;
        case TCPIPSOCKET_IN_PROGRESS:
          break;
        case TCPIPSOCKET_DISCONNECTED:
          tcpIpSocketClose(&client_proc->socket);
          tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_WAIT_FOR_CONNECTING);
          break;
        case TCPIPSOCKET_FAILED:
        default:
          handleRpcrosClientError( n, client_idx );
          break;
      }
      break;
    }
    default:
    {
      PRINT_ERROR ( "doWithRpcrosClientSocket() : Invalid RPC ROS process state\n" );
    }

  }
  return ret_err;
}

static cRosErrCodePack doWithRpcrosServerSocket(CrosNode *n, int i)
{
  cRosErrCodePack ret_err;
  PRINT_VVDEBUG ( "doWithRpcrosServerSocket()\n" );

  ret_err = CROS_SUCCESS_ERR_PACK;
  TcprosProcess *server_proc = &(n->rpcros_server_proc[i]);

  switch (server_proc->state)
  {
    case TCPROS_PROCESS_STATE_READING_HEADER_SIZE:
    {
      size_t n_reads;
      if (server_proc->left_to_recv == 0)
        server_proc->left_to_recv = sizeof(uint32_t);

      TcpIpSocketState sock_state = tcpIpSocketReadBufferEx( &(server_proc->socket),
                                                          &(server_proc->packet),
                                                          server_proc->left_to_recv,
                                                          &n_reads);
      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          server_proc->left_to_recv -= n_reads;
          if (server_proc->left_to_recv == 0)
          {
            const unsigned char *data = dynBufferGetCurrentData(&server_proc->packet);
            uint32_t header_size;
            header_size = ROS_TO_HOST_UINT32(*(uint32_t *)data);
            tcprosProcessClear( server_proc );
            server_proc->left_to_recv = header_size;
            tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_READING_HEADER);
            goto read_header;
          }
          break;
        case TCPIPSOCKET_IN_PROGRESS:
          break;
        case TCPIPSOCKET_DISCONNECTED:
          closeTcprosProcess( server_proc );
          break;
        case TCPIPSOCKET_FAILED:
        default:
          handleRpcrosServerError( n, i );
          break;
      }

      break;
    }
    read_header:
    case TCPROS_PROCESS_STATE_READING_HEADER:
    {
      size_t n_reads;
      TcpIpSocketState sock_state = tcpIpSocketReadBufferEx( &(server_proc->socket),
                                                          &(server_proc->packet),
                                                          server_proc->left_to_recv,
                                                          &n_reads);
      TcprosParserState parser_state = TCPROS_PARSER_ERROR;

      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          server_proc->left_to_recv -= n_reads;
          if (server_proc->left_to_recv == 0)
          {
            parser_state = cRosMessageParseServiceCallerHeader( n, i );
            break;
          }
          parser_state = TCPROS_PARSER_HEADER_INCOMPLETE;
          break;
        case TCPIPSOCKET_IN_PROGRESS:
          parser_state = TCPROS_PARSER_HEADER_INCOMPLETE;
          break;
        case TCPIPSOCKET_DISCONNECTED:
        case TCPIPSOCKET_FAILED:
        default:
          handleRpcrosServerError( n, i );
          break;
      }

      switch ( parser_state )
      {
        case TCPROS_PARSER_DONE:
          tcprosProcessClear( server_proc );
          tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_WRITING_HEADER );
          PRINT_VDEBUG("doWithRpcrosServerSocket() : Header parsed. Probing session: %d. Rpcros server index: %d \n", server_proc->probe, i);
          break;
        case TCPROS_PARSER_HEADER_INCOMPLETE:
          break;
        case TCPROS_PARSER_ERROR:
        default:
          handleRpcrosServerError( n, i );
          break;
      }

      break;
    }
    case TCPROS_PROCESS_STATE_WRITING_HEADER:
    {
      PRINT_VDEBUG ( "doWithRpcrosServerSocket() : Writing header. RpcrosServer index: %d \n", i );

      cRosMessagePrepareServiceProviderHeader( n, i );

      TcpIpSocketState sock_state =  tcpIpSocketWriteBuffer( &(server_proc->socket),
                                                            &(server_proc->packet) );

      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          PRINT_VDEBUG ( "doWithRpcrosServerSocket() : Done writing header with no error. RpcrosServer index: %d \n", i);
          if(server_proc->probe)
          {
            if(server_proc->persistent)
            {
              tcprosProcessClear( server_proc );
              tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_READING_HEADER_SIZE);
              server_proc->left_to_recv = sizeof(uint32_t);
            }
            else
              closeTcprosProcess( server_proc );
          }
          else
          {
            tcprosProcessClear( server_proc );
            server_proc->left_to_recv = sizeof(uint32_t);
            tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_READING_SIZE );
          }
          break;

        case TCPIPSOCKET_IN_PROGRESS:
          break;

        case TCPIPSOCKET_DISCONNECTED:
          closeTcprosProcess( server_proc );
          break;

        case TCPIPSOCKET_FAILED:
        default:
          handleRpcrosServerError( n, i );
          tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_IDLE );
          break;
      }

      break;
    }
    case TCPROS_PROCESS_STATE_READING_SIZE:
    {
      size_t n_reads;
      TcpIpSocketState sock_state = tcpIpSocketReadBufferEx( &(server_proc->socket),
                                                          &(server_proc->packet),
                                                          server_proc->left_to_recv,
                                                          &n_reads);

      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          server_proc->left_to_recv -= n_reads;
          if (server_proc->left_to_recv == 0)
          {
            const uint32_t *data = (const uint32_t *)dynBufferGetCurrentData(&server_proc->packet);
            uint32_t msg_size;
            msg_size = ROS_TO_HOST_UINT32(*data);
            tcprosProcessClear( server_proc );
            if (msg_size == 0)
            {
              PRINT_VDEBUG ( "doWithRpcrosServerSocket() : Done reading size with no error\n" );
              ret_err = cRosMessagePrepareServiceResponsePacket(n, i);
              tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_WRITING);
              goto write_msg;
            }
            else
            {
              server_proc->left_to_recv = msg_size;
              tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_READING);
              goto read_msg;
            }
          }
          break;
        case TCPIPSOCKET_IN_PROGRESS:
          break;
        case TCPIPSOCKET_DISCONNECTED:
        case TCPIPSOCKET_FAILED:
        default:
          handleRpcrosServerError( n, i );
          break;
      }
      break;
    }
    read_msg:
    case TCPROS_PROCESS_STATE_READING:
    {
      PRINT_VDEBUG ( "doWithRpcrosServerSocket() : reading. RpcrosServer index: %d\n", i );

      size_t n_reads;
      TcpIpSocketState sock_state = tcpIpSocketReadBufferEx( &(server_proc->socket),
                                                          &(server_proc->packet),
                                                          server_proc->left_to_recv,
                                                          &n_reads);

      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          server_proc->left_to_recv -= n_reads;
          if (server_proc->left_to_recv == 0)
          {
              PRINT_VDEBUG ( "doWithRpcrosServerSocket() : Done reading with no error\n" );
              ret_err = cRosMessagePrepareServiceResponsePacket(n, i);
              tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_WRITING );
          }
          break;
        case TCPIPSOCKET_IN_PROGRESS:
          break;
        case TCPIPSOCKET_DISCONNECTED:
          closeTcprosProcess( server_proc );
          break;
        case TCPIPSOCKET_FAILED:
        default:
          handleRpcrosServerError( n, i );
          break;
      }

      break;
    }
    write_msg:
    case TCPROS_PROCESS_STATE_WRITING:
    {
      PRINT_VDEBUG ( "doWithRpcrosServerSocket() : writing. Rpcros server index %d \n", i );

      TcpIpSocketState sock_state;

      //If the rpc response is empty, e.g. log procedures
      //if(server_proc->packet.size > 0)
      //{
        sock_state =  tcpIpSocketWriteBuffer( &(server_proc->socket), &(server_proc->packet) );
      //}
      //else
      //{
      //sock_state = TCPIPSOCKET_DONE;
      //}
      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          PRINT_VDEBUG ( "doWithRpcrosServerSocket() : Done writing with no error\n" );
          if(server_proc->persistent)
          {
            server_proc->left_to_recv = sizeof(uint32_t);
            tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_READING_SIZE);
          }
          else
            closeTcprosProcess( server_proc );
          break;

        case TCPIPSOCKET_IN_PROGRESS:
          break;

        case TCPIPSOCKET_DISCONNECTED:
          closeTcprosProcess( server_proc );
          break;

        case TCPIPSOCKET_FAILED:
        default:
          handleRpcrosServerError( n, i );
          break;
      }

      break;
    }
    default:
    {
      PRINT_ERROR ( "doWithRpcrosServerSocket() : Invalid RPCROS server process state\n" );
    }
  }
  return ret_err;
}

/*
 * Callback functions for the service callS of the logging mechanism
 */
static CallbackResponse callback_srv_set_logger_level(cRosMessage *request, cRosMessage *response, void* context)
{
  cRosMessageField* level = cRosMessageGetField(request, "level");
  const char* level_str = level->data.as_string;
  CrosNode* node = (CrosNode*) context;
  if(stringToLogLevel(level_str, &node->log_level) == 0)
    PRINT_INFO( "callback_srv_set_logger_level() : Node log priority level changed to %s\n", LogLevelToString(node->log_level));
  return 0;
}

static CallbackResponse callback_srv_get_loggers(cRosMessage *request, cRosMessage *response, void* context)
{
  const char *log_level_str;
  cRosMessage *logger_msg;
  cRosMessageField *loggers_field, *level_field;
  CrosNode *node = (CrosNode*) context;

  loggers_field = cRosMessageGetField(response, "loggers");

  logger_msg = cRosMessageFieldArrayAtMsgGet(loggers_field, 0);
  if(logger_msg == NULL) // If the message array is empty (it is the first time that the callback fn is called), create a new message (array element)
  {
    cRosMessageField *logger_field;

    cRosMessageNewBuild(node->message_root_path, "roscpp/Logger", &logger_msg);
    logger_field = cRosMessageGetField(logger_msg, "name");
    cRosMessageSetFieldValueString(logger_field, "ros.cros_node");

    cRosMessageFieldArrayPushBackMsg(loggers_field, logger_msg);
  }
  level_field = cRosMessageGetField(logger_msg, "level");
  log_level_str = LogLevelToString(node->log_level);
  cRosMessageSetFieldValueString(level_field, log_level_str);

  return 0;
}

int checkNamespaceFormat(const char* name_space)
{
  const char* it_ns = name_space;

  if(*it_ns == '~')
    it_ns++;

  while(*it_ns != '\0')
  {
    if(*it_ns == '/')
    {
      if(*(it_ns + 1) == '\0' ||
         !isalpha(*(it_ns + 1)))
        return 0;
      it_ns++;
    }
    else
    {
      if(!isalnum(*it_ns) && *it_ns != '_')
        return 0;
    }
    it_ns++;
  }

  return 1;
}

char* cRosNamespaceBuild(CrosNode* node, const char* resource_name)
{
  char *resolved_name = NULL;

  if(!checkNamespaceFormat(resource_name))
    return NULL;

  if(node == NULL)
  {
    if(resource_name[0] == '/')
    {
      resolved_name = (char *)calloc(strlen(resource_name) + 1, sizeof(char));
      strncat(resolved_name, resource_name,strlen(resource_name));
    }
    else
    {
      resolved_name = (char *)calloc(strlen(resource_name) + strlen("/") + 1, sizeof(char));
      strncat(resolved_name, "/",strlen("/"));
      strncat(resolved_name, resource_name,strlen(resource_name));
    }
  }
  else
  {
    char* node_name = node->name;
    switch(resource_name[0])
    {
      case '/':
      {
        //global namespace
       if(node != NULL || 1)
       {
         resolved_name = (char *)calloc(strlen(resource_name) + 1,
                                sizeof(char));
         strncat(resolved_name,resource_name,strlen(resource_name));
       }
        break;
      }
      case '~':
      {
         //private namespace
        if(node != NULL || 1)
        {
          resolved_name = (char *)calloc(strlen(node_name) +
                                 strlen("/") +
                                 strlen(resource_name) + 1,
                                 sizeof(char));
          strncat(resolved_name,node_name,strlen(node_name));
          strncat(resolved_name,"/",strlen("/"));
          strncat(resolved_name,resource_name + 1,strlen(resource_name) - 1 );
        }
        break;
      }
      default:
      {
        // the resource has a name that is not global or private
        if(node != NULL || 1)
        {
          char *node_namespace = (char *)calloc(strlen(node_name) + 1, sizeof(char));
          strcpy(node_namespace, node_name);
          char *it = node_namespace + strlen(node_name);
          while(*(it--) != '/');
          *(it + 2) = '\0';
          resolved_name = (char *)calloc(strlen(node_namespace) + strlen(resource_name) + 1,sizeof(char));
          strncat(resolved_name,node_namespace,strlen(node_namespace));
          strncat(resolved_name,resource_name,strlen(resource_name));
        }
        break;
      }
    }
  }
  return resolved_name;
}

int cRosInitializeLibrary(void)
{
  int ret;
  ret = tcpIpSocketStartUp(); // Initialize the socket library (it's mandatory on Windows)
  // Enable escape character processing for console messages so that debug messages can use colors
  // This processing is supported on Windows 10 build 16257 and later (and on Linux).
#ifdef _WIN32
  HANDLE stdout_handle = GetStdHandle(STD_OUTPUT_HANDLE);
  if (stdout_handle != INVALID_HANDLE_VALUE)
  {
    DWORD console_mode = 0;
    if (GetConsoleMode(stdout_handle, &console_mode))
    {
      // Set the console output mode to process virtual-terminal sequences
      if (!SetConsoleMode(stdout_handle, console_mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING))
        PRINT_ERROR("cRosInitializeLibrary() : Can't set console-output mode (Error code: %lu).\n", GetLastError());
    }
    else
      PRINT_ERROR("cRosInitializeLibrary() : Can't obatin console-output mode (Error code: %lu).\n", GetLastError());
  }
  else
    PRINT_ERROR("cRosInitializeLibrary() : Can't obatin console handle to configure output mode (Error code: %lu).\n", GetLastError());
#endif
  return(ret);
}

CrosNode *cRosNodeCreate (const char *node_name, const char *node_host, const char *roscore_host, unsigned short roscore_port,
                          const char *message_root_path)
{
  CrosNode *new_n; // Value to be returned by this function. NULL on failure
  PRINT_VVDEBUG ( "cRosNodeCreate()\n" );

  if(node_name == NULL || node_host == NULL || roscore_host == NULL )
  {
    PRINT_ERROR ( "cRosNodeCreate() : NULL parameters\n" );
    return NULL;
  }

  if(cRosInitializeLibrary() != 0)
    return NULL; // Initialization of the library failed: exit

  new_n = ( CrosNode * ) malloc ( sizeof ( CrosNode ) );

  if ( new_n == NULL )
  {
    PRINT_ERROR ( "cRosNodeCreate() : Can't allocate memory\n" );
    return NULL;
  }

  new_n->name = new_n->host = new_n->roscore_host = NULL;

  new_n->name = cRosNamespaceBuild(NULL, node_name);
  new_n->host = ( char * ) malloc ( ( strlen ( node_host ) + 1 ) *sizeof ( char ) );
  new_n->roscore_host = ( char * ) malloc ( ( strlen ( roscore_host ) + 1 ) *sizeof ( char ) );
  new_n->message_root_path = ( char * ) malloc ( ( strlen ( message_root_path ) + 1 ) *sizeof ( char ) );

  if (new_n->name == NULL || new_n->host == NULL
      || new_n->roscore_host == NULL || new_n->message_root_path == NULL )
  {
    PRINT_ERROR ( "cRosNodeCreate() : Can't allocate memory\n" );
    cRosNodeDestroy ( new_n );
    return NULL;
  }

  strcpy ( new_n->host, node_host );
  strcpy ( new_n->roscore_host, roscore_host );
  strcpy ( new_n->message_root_path, message_root_path );

  new_n->log_level = CROS_LOGLEVEL_INFO;
  new_n->xmlrpc_port = 0;
  new_n->tcpros_port = 0;
  new_n->roscore_port = roscore_port;
  new_n->roscore_pid = -1;

  xmlrpcProcessInit( &(new_n->xmlrpc_listner_proc) );

  new_n->next_call_id = 0;
  initApiCallQueue(&new_n->master_api_queue);
  initApiCallQueue(&new_n->slave_api_queue);

  new_n->xmlrpc_master_wake_up_time = 0;

  int i, fn_ret;
  for (i = 0 ; i < CN_MAX_XMLRPC_SERVER_CONNECTIONS; i++)
    xmlrpcProcessInit( &(new_n->xmlrpc_server_proc[i]) );

  for ( i = 0 ; i < CN_MAX_XMLRPC_CLIENT_CONNECTIONS; i++)
    xmlrpcProcessInit( &(new_n->xmlrpc_client_proc[i]) );

  tcprosProcessInit( &(new_n->tcpros_listner_proc) );

  for ( i = 0; i < CN_MAX_TCPROS_SERVER_CONNECTIONS; i++)
    tcprosProcessInit( &(new_n->tcpros_server_proc[i]) );

  for ( i = 0; i < CN_MAX_TCPROS_CLIENT_CONNECTIONS; i++)
    tcprosProcessInit( &(new_n->tcpros_client_proc[i]) );

  tcprosProcessInit( &(new_n->rpcros_listner_proc) );

  for ( i = 0; i < CN_MAX_RPCROS_SERVER_CONNECTIONS; i++)
    tcprosProcessInit( &(new_n->rpcros_server_proc[i]) );

  for ( i = 0; i < CN_MAX_RPCROS_CLIENT_CONNECTIONS; i++)
    tcprosProcessInit( &(new_n->rpcros_client_proc[i]) );

  for ( i = 0; i < CN_MAX_PUBLISHED_TOPICS; i++)
    initPublisherNode(&new_n->pubs[i]);
  new_n->n_pubs = 0;

  for ( i = 0; i < CN_MAX_SUBSCRIBED_TOPICS; i++)
    initSubscriberNode(&new_n->subs[i]);
  new_n->n_subs = 0;

  for ( i = 0; i < CN_MAX_SERVICE_PROVIDERS; i++)
    initServiceProviderNode(&new_n->service_providers[i]);
  new_n->n_service_providers = 0;

  for ( i = 0; i < CN_MAX_SERVICE_CALLERS; i++)
    initServiceCallerNode(&new_n->service_callers[i]);
  new_n->n_service_callers = 0;

  for ( i = 0; i < CN_MAX_PARAMETER_SUBSCRIPTIONS; i++)
    initParameterSubscrition(&new_n->paramsubs[i]);
  new_n->n_paramsubs = 0;

#ifdef _WIN32
  new_n->pid = (int)GetCurrentProcessId();
#else
  new_n->pid = (int)getpid();
#endif

  fn_ret = 0;
  for(i = 0; i < CN_MAX_XMLRPC_CLIENT_CONNECTIONS && fn_ret == 0; i++)
  {
    fn_ret = openXmlrpcClientSocket( new_n, i );
  }

  for(i = 0; i < CN_MAX_TCPROS_CLIENT_CONNECTIONS && fn_ret == 0; i++)
  {
    fn_ret = openTcprosClientSocket( new_n, i );
  }

  for(i = 0; i < CN_MAX_RPCROS_CLIENT_CONNECTIONS && fn_ret == 0; i++)
  {
    fn_ret = openRpcrosClientSocket( new_n, i );
  }
  if(fn_ret == 0)
    fn_ret = openXmlrpcListnerSocket( new_n );
  if(fn_ret == 0)
    fn_ret = openTcprosListnerSocket( new_n );
  if(fn_ret == 0)
    fn_ret = openRpcrosListnerSocket( new_n );

  if(fn_ret != 0)
  {
    PRINT_ERROR ( "cRosNodeCreate() : socket could not be opened\n" );
    cRosNodeDestroy ( new_n );
    return NULL;
  }

  new_n-> log_last_id = 0;

  /*
   * Registering logging callback
   */

  cRosErrCodePack ret_err;

  // Create a publisher of the topic /rosout of type "rosgraph_msgs/Log".
  // The messages of this topic will not be periodically sent but on demand (loop_period = -1).
  ret_err = cRosApiRegisterPublisher(new_n,"/rosout","rosgraph_msgs/Log", -1, NULL, NULL, new_n, &new_n->rosout_pub_idx);
  if (ret_err != CROS_SUCCESS_ERR_PACK)
  {
    PRINT_ERROR ( "cRosNodeCreate(): Error registering rosout.\n");
    cRosPrintErrCodePack(ret_err, "cRosNodeCreate()");
  }

  ret_err = cRosApiRegisterServiceProvider(new_n,"~get_loggers","roscpp/GetLoggers",
                                      callback_srv_get_loggers, NULL, (void *)new_n, NULL);
  if (ret_err != CROS_SUCCESS_ERR_PACK)
  {
    PRINT_ERROR ( "cRosNodeCreate(): Error registering loggers.\n");
    cRosPrintErrCodePack(ret_err, "cRosNodeCreate()");
  }

  ret_err = cRosApiRegisterServiceProvider(new_n,"~set_logger_level","roscpp/SetLoggerLevel",
                                      callback_srv_set_logger_level, NULL, (void *)new_n, NULL);
  if (ret_err != CROS_SUCCESS_ERR_PACK)
  {
    PRINT_ERROR ( "cRosNodeCreate(): Error registering loggers.\n");
    cRosPrintErrCodePack(ret_err, "cRosNodeCreate()");
  }

  return new_n;
}

int cRosUnregistrationCompleted(CrosNode *n)
{
  int i, unreg_finished;

  unreg_finished = 1;

  for ( i = 0; i < CN_MAX_PUBLISHED_TOPICS && unreg_finished == 1; i++)
    if(n->pubs[i].topic_name != NULL)
      unreg_finished = 0;

  for ( i = 0; i < CN_MAX_SUBSCRIBED_TOPICS && unreg_finished == 1; i++)
    if(n->subs[i].topic_name != NULL)
      unreg_finished = 0;

  for ( i = 0; i < CN_MAX_SERVICE_PROVIDERS && unreg_finished == 1; i++)
    if(n->service_providers[i].service_name != NULL)
      unreg_finished = 0;

  for ( i = 0; i < CN_MAX_PARAMETER_SUBSCRIPTIONS && unreg_finished == 1; i++)
    if(n->paramsubs[i].parameter_key != NULL)
      unreg_finished = 0;

  return(unreg_finished);
}

int cRosOutputQueuesEmpty(CrosNode *n)
{
  int i, queues_empty;

  queues_empty = 1;

  for ( i = 0; i < CN_MAX_PUBLISHED_TOPICS && queues_empty == 1; i++)
    if(n->pubs[i].topic_name != NULL && cRosMessageQueueUsage(&n->pubs[i].msg_queue) > 0)
      queues_empty = 0;

  for ( i = 0; i < CN_MAX_SERVICE_CALLERS && queues_empty == 1; i++)
    if(n->service_callers[i].service_name != NULL && cRosMessageQueueUsage(&n->service_callers[i].msg_queue) > 0)
      queues_empty = 0;

  return(queues_empty);
}

// This function continues to run the cROS node until the specified function returns true (1)
cRosErrCodePack cRosNodeWaitUntilFnRetTrue(CrosNode *n, int (*fn_to_check)(CrosNode *n))
{
  int unreg_incomp;
  cRosErrCodePack ret_err = CROS_SUCCESS_ERR_PACK;
  uint64_t start_time, elapsed_time;

  start_time = cRosClockGetTimeMs();
  while( (unreg_incomp = !fn_to_check(n)) && (elapsed_time=cRosClockGetTimeMs()-start_time) <= CN_UNREGISTRATION_TIMEOUT && ret_err == CROS_SUCCESS_ERR_PACK)
    ret_err=cRosNodeDoEventsLoop( n, CN_UNREGISTRATION_TIMEOUT - elapsed_time);

  if(ret_err == CROS_SUCCESS_ERR_PACK && unreg_incomp)
    ret_err = CROS_UNREG_TIMEOUT_ERR; // the unregistration process could not be completed

  return ret_err;
}

void cRosNodePauseAllCallersPublishers(CrosNode *n)
{
  int i;

  for ( i = 0; i < CN_MAX_PUBLISHED_TOPICS; i++)
    if(n->pubs[i].topic_name != NULL)
      n->pubs[i].loop_period = -1;

  for ( i = 0; i < CN_MAX_SERVICE_CALLERS; i++)
    if(n->service_callers[i].service_name != NULL)
      n->service_callers[i].loop_period = -1;
}

cRosErrCodePack cRosNodeUnregisterAll(CrosNode *n)
{
  int i;
  cRosErrCodePack ret_err=CROS_SUCCESS_ERR_PACK;

  for ( i = 0; i < CN_MAX_PUBLISHED_TOPICS && ret_err == CROS_SUCCESS_ERR_PACK; i++)
    if(n->pubs[i].topic_name != NULL)
      ret_err = cRosApiUnregisterPublisher(n, i);

  for ( i = 0; i < CN_MAX_SUBSCRIBED_TOPICS && ret_err == CROS_SUCCESS_ERR_PACK; i++)
    if(n->subs[i].topic_name != NULL)
      ret_err = cRosApiUnregisterSubscriber(n, i);

  for ( i = 0; i < CN_MAX_SERVICE_PROVIDERS && ret_err == CROS_SUCCESS_ERR_PACK; i++)
    if(n->service_providers[i].service_name != NULL)
      ret_err = cRosApiUnregisterServiceProvider(n, i);

  for ( i = 0; i < CN_MAX_PARAMETER_SUBSCRIPTIONS && ret_err == CROS_SUCCESS_ERR_PACK; i++)
    if(n->paramsubs[i].parameter_key != NULL)
      ret_err = cRosApiUnsubscribeParam(n, i);

  return ret_err;
}

cRosErrCodePack cRosNodeDestroy ( CrosNode *n )
{
  cRosErrCodePack ret_err;

  PRINT_VVDEBUG ( "cRosNodeDestroy()\n" );

  if ( n == NULL )
    return CROS_BAD_PARAM_ERR;

  cRosNodeWaitUntilFnRetTrue(n, cRosOutputQueuesEmpty); // Wait until all pendind topic messages and service call have been sent

  cRosNodePauseAllCallersPublishers(n);
  ret_err = cRosNodeUnregisterAll(n);
  if(ret_err == CROS_SUCCESS_ERR_PACK)
    ret_err = cRosNodeWaitUntilFnRetTrue(n, cRosUnregistrationCompleted); // Wait until all roles have been unsuscribed

  xmlrpcProcessRelease( &(n->xmlrpc_listner_proc) );

  releaseApiCallQueue(&n->master_api_queue);
  releaseApiCallQueue(&n->slave_api_queue);

  int i;
  for (i = 0; i < CN_MAX_XMLRPC_SERVER_CONNECTIONS; i++)
    xmlrpcProcessRelease( &(n->xmlrpc_server_proc[i]) );

  for(i = 0; i < CN_MAX_XMLRPC_CLIENT_CONNECTIONS; i++)
    xmlrpcProcessRelease( &(n->xmlrpc_client_proc[i]) );

  tcprosProcessRelease( &(n->tcpros_listner_proc) );

  for ( i = 0; i < CN_MAX_TCPROS_SERVER_CONNECTIONS; i++)
    tcprosProcessRelease( &(n->tcpros_server_proc[i]) );

  for ( i = 0; i < CN_MAX_TCPROS_CLIENT_CONNECTIONS; i++)
    tcprosProcessRelease( &(n->tcpros_client_proc[i]) );

  for ( i = 0; i < CN_MAX_RPCROS_SERVER_CONNECTIONS; i++)
    tcprosProcessRelease( &(n->rpcros_server_proc[i]) );

  for ( i = 0; i < CN_MAX_RPCROS_CLIENT_CONNECTIONS; i++)
    tcprosProcessRelease( &(n->rpcros_client_proc[i]) );

  if ( n->name != NULL ) free ( n->name );
  if ( n->host != NULL ) free ( n->host );
  if ( n->roscore_host != NULL ) free ( n->roscore_host );

  for ( i = 0; i < CN_MAX_PUBLISHED_TOPICS; i++)
    cRosApiReleasePublisher(n, i);

  for ( i = 0; i < CN_MAX_SUBSCRIBED_TOPICS; i++)
    cRosApiReleaseSubscriber(n, i);

  for ( i = 0; i < CN_MAX_SERVICE_PROVIDERS; i++)
    cRosApiReleaseServiceProvider(n, i);

  for ( i = 0; i < CN_MAX_SERVICE_CALLERS; i++)
    cRosApiReleaseServiceCaller(n, i);

  for ( i = 0; i < CN_MAX_PARAMETER_SUBSCRIPTIONS; i++)
    cRosNodeReleaseParameterSubscrition(&n->paramsubs[i]);

  tcpIpSocketCleanUp();

  return ret_err;
}

  int cRosNodeRegisterPublisher (CrosNode *node, const char *message_definition, const char *topic_name,
                               const char *topic_type, const char *md5sum, int loop_period, void *data_context)
{
  PRINT_VVDEBUG ( "cRosNodeRegisterPublisher()\n" );

  if ( node->n_pubs >= CN_MAX_PUBLISHED_TOPICS )
  {
    PRINT_ERROR ( "cRosNodeRegisterPublisher() : Can't register a new publisher: \
                 reached the maximum number of published topics\n");
    return -1;
  }

  char *pub_message_definition = ( char * ) malloc ( ( strlen ( message_definition ) + 1 ) * sizeof ( char ) );
  char *pub_topic_name = cRosNamespaceBuild(node, topic_name);
  char *pub_topic_type = ( char * ) malloc ( ( strlen ( topic_type ) + 1 ) * sizeof ( char ) );
  char *pub_md5sum = ( char * ) malloc ( ( strlen ( md5sum ) + 1 ) * sizeof ( char ) );

  if ( pub_message_definition == NULL || pub_topic_name == NULL ||
       pub_topic_type == NULL || pub_md5sum == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterPublisher() : Can't allocate memory\n" );
    return -1;
  }

  strcpy ( pub_message_definition, message_definition );
  strcpy ( pub_topic_type, topic_type );
  strcpy ( pub_md5sum, md5sum );

  PRINT_INFO ( "Publishing topic %s type %s \n", pub_topic_name, pub_topic_type );

  int pubidx = -1;
  int it = 0;
  for (; it < CN_MAX_PUBLISHED_TOPICS; it++)
  {
    if (node->pubs[it].topic_name == NULL)
    {
      pubidx = it;
      break;
    }
  }

  PublisherNode *pub = &node->pubs[pubidx];
  pub->message_definition = pub_message_definition;
  pub->topic_name = pub_topic_name;
  pub->topic_type = pub_topic_type;
  pub->md5sum = pub_md5sum;

  pub->loop_period = loop_period;
  pub->wake_up_time = 0;
  pub->context = data_context;
  cRosMessageQueueClear(&pub->msg_queue);

  node->n_pubs++;

  int rc = enqueuePublisherAdvertise(node, pubidx);
  if (rc == -1)
    return -1;

  return pubidx;
}

int cRosNodeRegisterServiceProvider(CrosNode *node, const char *service_name,
                                    const char *service_type, const char *md5sum, void *data_context)
{
  PRINT_VVDEBUG ( "cRosNodeRegisterServiceProvider()\n" );

  if (node->n_service_providers >= CN_MAX_SERVICE_PROVIDERS)
  {
    PRINT_ERROR ( "cRosNodeRegisterServiceProvider() : Can't register a new service provider: \
                 reached the maximum number of service providers\n");
    return -1;
  }

  char *srv_service_name =  cRosNamespaceBuild(node, service_name);
  char *srv_service_type = ( char * ) malloc ( ( strlen ( service_type ) + 1 ) * sizeof ( char ) );
  char *srv_servicerequest_type = ( char * ) malloc ( ( strlen ( service_type ) + strlen("Request") + 1 ) * sizeof ( char ) );
  char *srv_serviceresponse_type = ( char * ) malloc ( ( strlen ( service_type ) + strlen("Response") + 1 ) * sizeof ( char ) );
  char *srv_md5sum = ( char * ) malloc ( ( strlen ( md5sum ) + 1 ) * sizeof ( char ) );

  if ( srv_service_name == NULL || srv_service_type == NULL ||
       srv_md5sum == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterServiceProvider() : Can't allocate memory\n" );
    return -1;
  }

  strncpy ( srv_service_type, service_type, strlen ( service_type ) + 1 );
  strncpy ( srv_servicerequest_type, service_type, strlen ( service_type ) + 1 );
  strncat ( srv_servicerequest_type, "Request", strlen("Request") + 1 );
  strncpy ( srv_serviceresponse_type, service_type, strlen ( service_type ) + 1 );
  strncat ( srv_serviceresponse_type, "Response", strlen("Response") +1 );
  strncpy ( srv_md5sum, md5sum, strlen(md5sum) + 1 );

  PRINT_INFO ( "Registering service provider %s type %s \n", srv_service_name, srv_service_type);

  int serviceidx = -1;
  int it = 0;
  for (; it < CN_MAX_SERVICE_PROVIDERS; it++)
  {
    if (node->service_providers[it].service_name == NULL)
    {
      serviceidx = it;
      break;
    }
  }

  ServiceProviderNode *service = &(node->service_providers[serviceidx]);

  service->service_name = srv_service_name;
  service->service_type = srv_service_type;
  service->servicerequest_type = srv_servicerequest_type;
  service->serviceresponse_type = srv_serviceresponse_type;
  service->md5sum = srv_md5sum;
  service->context = data_context;

  node->n_service_providers++;

  int rc = enqueueServiceAdvertise(node, serviceidx);
  if (rc == -1)
    return -1;

  return serviceidx;
}

int cRosNodeRecruitTcprosClientProc(CrosNode *node, int subidx)
{
  int ret; // Return value: -1 on error, or the recruited proc index on success

  SubscriberNode *sub;
  TcprosProcess *client_proc;

  // Look for a free Tcpros client proc
  int clientidx;
  client_proc=NULL;
  sub = &node->subs[subidx];
  ret=-1; // Default return value (No free Tcpros client proc is available)
  for(clientidx=0;clientidx<CN_MAX_TCPROS_CLIENT_CONNECTIONS && ret==-1;clientidx++)
  {
    client_proc = &node->tcpros_client_proc[clientidx];
    if(client_proc->topic_idx == -1) // A free Tcpros client proc has been found
    {
      client_proc->topic_idx = subidx;
      client_proc->tcp_nodelay = (unsigned char)sub->tcp_nodelay;
      ret=clientidx; // Exit loop
    }
  }
  return ret;
}

int cRosNodeFindFirstTcprosClientProc(CrosNode *node, int subidx, const char *tcpros_hostname, int tcpros_port)
{
  int ret; // Return value: -1 on error, or the found proc index on success
  int clientidx;

  // Look for the first Tcpros client proc that was recruited for subidx subscriber and a specific publisher host and port
  ret=-1;
  for(clientidx=0;clientidx<CN_MAX_TCPROS_CLIENT_CONNECTIONS && ret==-1;clientidx++)
  {
    TcprosProcess *cur_cli = &node->tcpros_client_proc[clientidx];
    if((subidx == -1 || cur_cli->topic_idx == subidx) &&
       (tcpros_port == -1 || cur_cli->sub_tcpros_port == tcpros_port) &&
       (tcpros_hostname == NULL || strcmp(cur_cli->sub_tcpros_host,tcpros_hostname)==0)
       )
      ret=clientidx;
  }
  return ret;
}

int cRosNodeRegisterSubscriber(CrosNode *node, const char *message_definition, const char *topic_name,
                               const char *topic_type, const char *md5sum, void *data_context, int tcp_nodelay)
{
  PRINT_VVDEBUG ( "cRosNodeRegisterSubscriber()\n" );

  if (node->n_subs >= CN_MAX_SUBSCRIBED_TOPICS)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't register a new subscriber: \
                  reached the maximum number of published topics\n");
    return -1;
  }

  char *pub_message_definition = ( char * ) malloc ( ( strlen ( message_definition ) + 1 ) * sizeof ( char ) );
  char *pub_topic_name = cRosNamespaceBuild(node, topic_name);
  char *pub_topic_type = ( char * ) malloc ( ( strlen ( topic_type ) + 1 ) * sizeof ( char ) );
  char *pub_md5sum = ( char * ) malloc ( ( strlen ( md5sum ) + 1 ) * sizeof ( char ) );

  if ( pub_topic_name == NULL || pub_topic_type == NULL )
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n" );
    return -1;
  }

  strcpy ( pub_message_definition, message_definition );
  strcpy ( pub_topic_name, topic_name );
  strcpy ( pub_topic_type, topic_type );
  strcpy ( pub_md5sum, md5sum );

  PRINT_INFO ( "Subscribing to topic %s type %s \n", pub_topic_name, pub_topic_type );

  int subidx = node->n_subs;
  int it = 0;
  for (; it < CN_MAX_SUBSCRIBED_TOPICS; it++)
  {
    if (node->subs[it].topic_name == NULL)
    {
      subidx = it;
      break;
    }
  }

  SubscriberNode *sub = &node->subs[subidx];
  sub->message_definition = pub_message_definition;
  sub->topic_name = pub_topic_name;
  sub->topic_type = pub_topic_type;
  sub->md5sum = pub_md5sum;
  sub->context = data_context;
  sub->tcp_nodelay = (unsigned char)tcp_nodelay;
  sub->msg_queue_overflow = 0;
  cRosMessageQueueClear(&sub->msg_queue);

  node->n_subs++;

  int rc = enqueueSubscriberAdvertise(node, subidx);
  if (rc == -1)
    return -1;

  return subidx;
}

int cRosNodeRegisterServiceCaller(CrosNode *node, const char *message_definition, const char *service_name,
                                    const char *service_type, const char *md5sum, int loop_period,
                                    void *data_context, int persistent, int tcp_nodelay)
{
  PRINT_VVDEBUG ( "cRosNodeRegisterServiceCaller()\n" );

  if (node->n_service_callers >= CN_MAX_SERVICE_CALLERS)
  {
    PRINT_ERROR ( "cRosNodeRegisterServiceCaller() : Can't register a new service caller: \
                 reached the maximum number of service callers\n");
    return -1;
  }

  char *srv_message_definition = ( char * ) malloc ( ( strlen ( message_definition ) + 1 ) * sizeof ( char ) );
  char *srv_service_name =  cRosNamespaceBuild(node, service_name);
  char *srv_service_type = ( char * ) malloc ( ( strlen ( service_type ) + 1 ) * sizeof ( char ) );
  char *srv_servicerequest_type = ( char * ) malloc ( ( strlen ( service_type ) + strlen("Request") + 1 ) * sizeof ( char ) );
  char *srv_serviceresponse_type = ( char * ) malloc ( ( strlen ( service_type ) + strlen("Response") + 1 ) * sizeof ( char ) );
  char *srv_md5sum = ( char * ) malloc ( ( strlen ( md5sum ) + 1 ) * sizeof ( char ) );

  if ( srv_service_name == NULL || srv_service_type == NULL ||
       srv_md5sum == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterServiceCaller() : Can't allocate memory\n" );
    return -1;
  }

  strcpy ( srv_message_definition, message_definition );
  strncpy ( srv_service_type, service_type, strlen ( service_type ) + 1 );
  strncpy ( srv_servicerequest_type, service_type, strlen ( service_type ) + 1 );
  strncat ( srv_servicerequest_type, "Request", strlen("Request") + 1 );
  strncpy ( srv_serviceresponse_type, service_type, strlen ( service_type ) + 1 );
  strncat ( srv_serviceresponse_type, "Response", strlen("Response") +1 );
  strncpy ( srv_md5sum, md5sum, strlen(md5sum) + 1 );

  PRINT_INFO ( "Starting service caller %s type %s \n", srv_service_name, srv_service_type);

  int serviceidx = -1;
  int it = 0;
  for (; it < CN_MAX_SERVICE_CALLERS; it++)
  {
    if (node->service_callers[it].service_name == NULL)
    {
      serviceidx = it;
      break;
    }
  }

  ServiceCallerNode *service = &(node->service_callers[serviceidx]);
  service->message_definition = srv_message_definition;
  service->service_name = srv_service_name;
  service->service_type = srv_service_type;
  service->servicerequest_type = srv_servicerequest_type;
  service->serviceresponse_type = srv_serviceresponse_type;
  service->md5sum = srv_md5sum;
  service->context = data_context;
  service->loop_period = loop_period;
  service->wake_up_time = 0; // cRosClockGetTimeMs() + 10;
  service->persistent = (unsigned char)persistent;
  service->tcp_nodelay = (unsigned char)tcp_nodelay;

  int clientidx = serviceidx; // node->service_callers[0] is assigned node->rpcros_client_proc[0] and so on
  service->rpcros_id = clientidx;

  TcprosProcess *client_proc = &node->rpcros_client_proc[clientidx];
  client_proc->service_idx = serviceidx;
  client_proc->persistent = (unsigned char)persistent;
  client_proc->tcp_nodelay = (unsigned char)tcp_nodelay;

  node->n_service_callers++;

  int rc = enqueueServiceLookup(node, serviceidx);
  if (rc == -1)
    return -1;

  return serviceidx;
}


int cRosNodeUnregisterSubscriber(CrosNode *node, int subidx)
{
  int client_tcpros_ind, client_xmlrpc_ind;
  if (subidx < 0 || subidx >= CN_MAX_SUBSCRIBED_TOPICS)
    return -1;

  SubscriberNode *sub = &node->subs[subidx];
  if (sub->topic_name == NULL)
    return -1;

  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeUnRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  while((client_tcpros_ind=cRosNodeFindFirstTcprosClientProc(node, subidx, NULL, -1)) != -1)
  {
    TcprosProcess *tcprosProc = &node->tcpros_client_proc[client_tcpros_ind];
    closeTcprosProcess(tcprosProc);
  }

  // Check if any xmlrpc_client_proc is working for the subscriber being unregistered and if so, close them
  for(client_xmlrpc_ind=0;client_xmlrpc_ind < CN_MAX_XMLRPC_CLIENT_CONNECTIONS;client_xmlrpc_ind++)
  {
    XmlrpcProcess *xmlrpcProc = &node->xmlrpc_client_proc[client_xmlrpc_ind];
    if(xmlrpcProc->state != XMLRPC_PROCESS_STATE_IDLE && xmlrpcProc->current_call != NULL &&
       xmlrpcProc->current_call->method == CROS_API_REQUEST_TOPIC && xmlrpcProc->current_call->provider_idx == subidx)
     closeXmlrpcProcess(xmlrpcProc);
  }

  XmlrpcProcess *coreproc = &node->xmlrpc_client_proc[0];
  if (coreproc->current_call != NULL
      && coreproc->current_call->method == CROS_API_REGISTER_SUBSCRIBER
      && coreproc->current_call->provider_idx == subidx)
  {
    // Delist current registration
    closeXmlrpcProcess(coreproc);
  }

  call->method = CROS_API_UNREGISTER_SUBSCRIBER;
  call->provider_idx = subidx;

  xmlrpcParamVectorPushBackString( &call->params, node->name);
  xmlrpcParamVectorPushBackString( &call->params, sub->topic_name );
  char node_uri[256];
  snprintf( node_uri, 256, "http://%s:%d/", node->host, node->xmlrpc_port);
  xmlrpcParamVectorPushBackString( &call->params, node_uri );

  // NB: Node release is done in handleApiCallAttempt()

  if(enqueueMasterApiCallInternal(node, call) < 0)
    return -1;
  else
    return 0;
}

int cRosNodeUnregisterPublisher(CrosNode *node, int pubidx)
{
  int list_elem;
  if (pubidx < 0 || pubidx >= CN_MAX_PUBLISHED_TOPICS)
    return -1;

  PublisherNode *pub = &node->pubs[pubidx];
  if (pub->topic_name == NULL)
    return -1;

  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeUnRegisterPublisher() : Can't allocate memory\n");
    return -1;
  }

  for(list_elem=0;pub->tcpros_id_list[list_elem]!=-1;list_elem++)
  {
    TcprosProcess *tcprosProc = &node->tcpros_server_proc[pub->tcpros_id_list[list_elem]];
    closeTcprosProcess(tcprosProc);
  }

  XmlrpcProcess *coreproc = &node->xmlrpc_client_proc[0];
  if (coreproc->current_call != NULL
      && coreproc->current_call->method == CROS_API_REGISTER_PUBLISHER
      && coreproc->current_call->provider_idx == pubidx)
  {
    // Delist current registration
    closeXmlrpcProcess(coreproc);
  }

  call->method = CROS_API_UNREGISTER_PUBLISHER;
  call->provider_idx = pubidx;

  xmlrpcParamVectorPushBackString( &call->params, node->name);
  xmlrpcParamVectorPushBackString( &call->params, pub->topic_name );
  char node_uri[256];
  snprintf( node_uri, 256, "http://%s:%d/", node->host, node->xmlrpc_port);
  xmlrpcParamVectorPushBackString( &call->params, node_uri );

  // NB: Node release is done in handleApiCallAttempt()

  if(enqueueMasterApiCallInternal(node, call) < 0)
    return -1;
  else
    return 0;
}

int cRosNodeUnregisterServiceProvider(CrosNode *node, int serviceidx)
{
  if (serviceidx < 0 || serviceidx >= CN_MAX_SERVICE_PROVIDERS)
    return -1;

  ServiceProviderNode *svc = &node->service_providers[serviceidx];
  if (svc->service_name == NULL)
    return -1;

  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeUnregisterServiceProvider() : Can't allocate memory\n");
    return -1;
  }

  XmlrpcProcess *coreproc = &node->xmlrpc_client_proc[0];
  if (coreproc->current_call != NULL
      && coreproc->current_call->method == CROS_API_REGISTER_SERVICE
      && coreproc->current_call->provider_idx == serviceidx)
  {
    // Delist current registration
    closeXmlrpcProcess(coreproc);
  }

  call->method = CROS_API_UNREGISTER_SERVICE;
  call->provider_idx = serviceidx;
  // 3 parameters are expected for this method
  xmlrpcParamVectorPushBackString( &call->params, node->name);
  xmlrpcParamVectorPushBackString( &call->params, svc->service_name);
  char uri[256];
  snprintf( uri, 256, "rosrpc://%s:%d/", node->host, node->rpcros_port);
  xmlrpcParamVectorPushBackString( &call->params, uri );
//  snprintf( uri, 256, "http://%s:%d/", node->host, node->xmlrpc_port);
//  xmlrpcParamVectorPushBackString( &call->params, uri );

  // NB: Node release is done in handleApiCallAttempt()

  if(enqueueMasterApiCallInternal(node, call) < 0)
    return -1;
  else
    return 0;
}

cRosErrCodePack cRosApiSubscribeParam(CrosNode *node, const char *key, NodeStatusApiCallback callback, void *context, int *paramsubidx_ptr)
{
  PRINT_VVDEBUG ( "cRosApiSubscribeParam()\n" );
  PRINT_INFO ( "Subscribing to parameter %s\n", key);

  if (node->n_paramsubs >= CN_MAX_PARAMETER_SUBSCRIPTIONS)
  {
    PRINT_ERROR ( "cRosApiSubscribeParam() : Can't register a new parameter subscription: \
                  reached the maximum number of parameters\n");
    return CROS_MANY_PARAM_ERR;
  }

  char *parameter_key = ( char * ) malloc ( ( strlen ( key ) + 1 ) * sizeof ( char ) );
  if (parameter_key == NULL)
  {
    PRINT_ERROR ( "cRosApiSubscribeParam() : Can't allocate memory\n" );
    return CROS_MEM_ALLOC_ERR;
  }

  strcpy (parameter_key, key);

  int paramsubidx = -1; // This value should never be used
  int it;
  for (it = 0; it < CN_MAX_PARAMETER_SUBSCRIPTIONS; it++)
  {
    if (node->paramsubs[it].parameter_key == NULL)
    {
      paramsubidx = it;
      break;
    }
  }

  ParameterSubscription *sub = &node->paramsubs[paramsubidx];
  sub->parameter_key = parameter_key;
  sub->context = context;
  sub->status_api_callback = callback;

  node->n_paramsubs++;

  int callid = enqueueParameterSubscription(node, paramsubidx);
  if (callid == -1)
  {
    free(parameter_key);
    sub->parameter_key = NULL;
    node->n_paramsubs--;
    return CROS_MEM_ALLOC_ERR;
  }

  if(paramsubidx_ptr != NULL)
    *paramsubidx_ptr = paramsubidx;

  return CROS_SUCCESS_ERR_PACK;
}

cRosErrCodePack cRosApiUnsubscribeParam(CrosNode *node, int paramsubidx)
{
  int caller_id;

  if (paramsubidx < 0 || paramsubidx >= CN_MAX_PARAMETER_SUBSCRIPTIONS)
    return CROS_BAD_PARAM_ERR;

  ParameterSubscription *sub = &node->paramsubs[paramsubidx];
  if (sub->parameter_key == NULL)
    return CROS_PARAM_SUB_IND_ERR;

  XmlrpcProcess *coreproc = &node->xmlrpc_client_proc[0];
  if (coreproc->current_call != NULL
      && coreproc->current_call->method == CROS_API_SUBSCRIBE_PARAM
      && coreproc->current_call->provider_idx == paramsubidx)
  {
    // Delist current registration
    closeXmlrpcProcess(coreproc);
  }

  caller_id = enqueueParameterUnsubscription(node, paramsubidx);

  return (caller_id != -1)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
}

cRosErrCodePack cRosNodeTriggerPublishersWriting( CrosNode *n, uint64_t cur_time )
{
  cRosErrCodePack ret_err;
  int pub_idx;

  ret_err = CROS_SUCCESS_ERR_PACK; // Default return value: success
  // Check whether it is time to send a new topic message and trigger the corresponding TcprosProcesses
  for(pub_idx = 0; pub_idx < CN_MAX_PUBLISHED_TOPICS; pub_idx++)
  {
    PublisherNode *cur_pub = &n->pubs[pub_idx];
    if(cur_pub->topic_name != NULL) // Is this publisher active?
    {
      if((cur_pub->loop_period >= 0 && cur_pub->wake_up_time <= cur_time) || cRosMessageQueueUsage(&cur_pub->msg_queue) > 0) // Is it time to publish a message (periodic or immediate)?
      {
        int list_elem, all_procs_ready;
        // Check whether all tcpProcess are ready to start writing a new message (or idle)
        all_procs_ready = 1;
        for(list_elem=0;cur_pub->tcpros_id_list[list_elem]!=-1;list_elem++)
        {
          TcprosProcess *server_proc = &n->tcpros_server_proc[cur_pub->tcpros_id_list[list_elem]];
          if(server_proc->state != TCPROS_PROCESS_STATE_WAIT_FOR_WRITING) // server_proc->state != TCPROS_PROCESS_STATE_IDLE &&
          {
            all_procs_ready = 0;
            break;
          }
        }
        if(all_procs_ready && cur_pub->tcpros_id_list[0]!=-1) // All associated processes are ready to write and there is at least one associated process
        {
          if(cRosMessageQueueUsage(&cur_pub->msg_queue) == 0) // There is no immediate message waiting, so a periodic message must be sent
            cur_pub->wake_up_time = cur_time + cur_pub->loop_period;

          // The next function will store the next message to be sent in cur_pub->context->outgoing
          ret_err = cRosNodePublisherCallback(cur_pub->context); // Calls the publisher application-defined callback

          // Make all the waiting processes start writing
          for(list_elem=0;cur_pub->tcpros_id_list[list_elem]!=-1;list_elem++)
          {
            TcprosProcess *server_proc = &n->tcpros_server_proc[cur_pub->tcpros_id_list[list_elem]];
            // if(server_proc->state == TCPROS_PROCESS_STATE_WAIT_FOR_WRITING)
              tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_START_WRITING );
          }
        }
      }
    }
  }
  return(ret_err);
}


cRosErrCodePack cRosNodeTriggerServiceCallersWriting( CrosNode *n, uint64_t cur_time )
{
  cRosErrCodePack ret_err;
  int caller_idx;

  ret_err = CROS_SUCCESS_ERR_PACK; // Default return value: success
  // Check whether it is time to make a service call, and if so, trigger the corresponding TcprosProcesses
  for(caller_idx = 0; caller_idx < CN_MAX_SERVICE_CALLERS; caller_idx++)
  {
    ServiceCallerNode *cur_caller = &n->service_callers[caller_idx];
    if(cur_caller->service_name != NULL) // Is this caller active?
    {
      if((cur_caller->loop_period >= 0 && cur_caller->wake_up_time <= cur_time) || cRosMessageQueueUsage(&cur_caller->msg_queue) == 1) // Is it time to make a call (periodic or immediate)?
      {
        // Check whether the corresponding process is ready to start making a new call
        TcprosProcess *caller_proc = &n->rpcros_client_proc[cur_caller->rpcros_id];
        if(caller_proc->state == TCPROS_PROCESS_STATE_WAIT_FOR_WRITING) // caller_proc->state == TCPROS_PROCESS_STATE_IDLE ||
        {
          if(cRosMessageQueueUsage(&cur_caller->msg_queue) == 0) // There is no immediate call waiting, so a periodic call will be made
            cur_caller->wake_up_time = cur_time + cur_caller->loop_period;

          // Now the service-call parameters are stored in cur_caller->context->outgoing
          ret_err = cRosNodeServiceCallerCallback( 0, cur_caller->context); // calls the service-caller application-defined callback function to generate the service request

          //if(caller_proc->state == TCPROS_PROCESS_STATE_WAIT_FOR_WRITING)
          {
            tcprosProcessChangeState( caller_proc, TCPROS_PROCESS_STATE_START_WRITING );
          }
        }
      }
    }
  }
  return(ret_err);
}

#define NODE_STATUS_STR_MAX_LEN (CN_MAX_XMLRPC_CLIENT_CONNECTIONS+CN_MAX_XMLRPC_SERVER_CONNECTIONS+CN_MAX_TCPROS_CLIENT_CONNECTIONS+CN_MAX_TCPROS_SERVER_CONNECTIONS+CN_MAX_RPCROS_CLIENT_CONNECTIONS+CN_MAX_RPCROS_SERVER_CONNECTIONS+6*3+3*4+2)
void printNodeProcState( CrosNode *n )
{
  static char prev_stat_str[NODE_STATUS_STR_MAX_LEN]="\0";
  char stat_str[NODE_STATUS_STR_MAX_LEN];
  int i;

  sprintf(stat_str, "XL%X XC",n->xmlrpc_listner_proc.state);
  for(i = 0; i < CN_MAX_XMLRPC_CLIENT_CONNECTIONS; i++ )
    sprintf(stat_str+strlen(stat_str), "%X",n->xmlrpc_client_proc[i].state);

  sprintf(stat_str+strlen(stat_str), " XS");
  for( i = 0; i < CN_MAX_XMLRPC_SERVER_CONNECTIONS; i++ )
    sprintf(stat_str+strlen(stat_str), "%X",n->xmlrpc_server_proc[i].state);

  sprintf(stat_str+strlen(stat_str), " TL%X TC",n->tcpros_listner_proc.state);
  for(i = 0; i < CN_MAX_TCPROS_CLIENT_CONNECTIONS; i++ )
    sprintf(stat_str+strlen(stat_str), "%X",n->tcpros_client_proc[i].state);

  sprintf(stat_str+strlen(stat_str), " TS");
  for( i = 0; i < CN_MAX_TCPROS_SERVER_CONNECTIONS; i++ )
    sprintf(stat_str+strlen(stat_str), "%X",n->tcpros_server_proc[i].state);

  sprintf(stat_str+strlen(stat_str), " RL%X RC",n->rpcros_listner_proc.state);
  for(i = 0; i < CN_MAX_RPCROS_CLIENT_CONNECTIONS; i++ )
    sprintf(stat_str+strlen(stat_str), "%X",n->rpcros_client_proc[i].state);

  sprintf(stat_str+strlen(stat_str), " RS");
  for( i = 0; i < CN_MAX_RPCROS_SERVER_CONNECTIONS; i++ )
    sprintf(stat_str+strlen(stat_str), "%X",n->rpcros_server_proc[i].state);

  if (strcmp(prev_stat_str, stat_str) != 0) // If the node status has changed:
  {
    // Print a compact string indicating the state of all the node processed for debug purposes
    PRINT_DEBUG("Node status: %s\n", stat_str);
    strcpy(prev_stat_str, stat_str);
  }
}

uint64_t cRosNodeCalculateSelectTimeout(CrosNode *n, uint64_t max_timeout)
{
  uint64_t wakeup_timeout, select_timeout, cur_time;
  int pub_idx, svc_idx;

  select_timeout =  max_timeout;
  cur_time = cRosClockGetTimeMs();

  if( n->xmlrpc_master_wake_up_time > cur_time )
    wakeup_timeout = n->xmlrpc_master_wake_up_time - cur_time;
  else
    wakeup_timeout = 0;

  if( wakeup_timeout < select_timeout )
    select_timeout = wakeup_timeout;

  for (pub_idx = 0;pub_idx < CN_MAX_PUBLISHED_TOPICS;pub_idx++) // < n_pubs?
  {
    PublisherNode *cur_pub = &n->pubs[pub_idx];
    if(cur_pub->topic_name != NULL && cur_pub->loop_period >= 0) // Is this publisher active and automatically publishing messages?
    {
      if( cur_pub->wake_up_time > cur_time ) // Is not it time to publish a new message yet?
        wakeup_timeout = cur_pub->wake_up_time - cur_time;
      else
        wakeup_timeout = 0;

      if( wakeup_timeout < select_timeout )
        select_timeout = wakeup_timeout;
    }
  }

  for (svc_idx = 0;svc_idx < CN_MAX_SERVICE_CALLERS;svc_idx++) // <n_service_callers?
  {
    ServiceCallerNode *cur_svc_caller = &n->service_callers[svc_idx];
    if(cur_svc_caller->service_name != NULL && cur_svc_caller->loop_period >= 0) // Is this service caller active and automatically publishing messages?
    {
      if( cur_svc_caller->wake_up_time > cur_time ) // Is not it time to make a service call yet?
        wakeup_timeout = cur_svc_caller->wake_up_time - cur_time;
      else
        wakeup_timeout = 0;

      if( wakeup_timeout < select_timeout )
      {
        //printf("[i:%i cur time: %lu wake:%lu new timeout: %lu prev timeout:%lu]", i, cur_time,  cur_svc_caller->wake_up_time, wakeup_timeout, select_timeout);
        select_timeout = wakeup_timeout;
      }
    }
  }

  return(select_timeout);
}

cRosErrCodePack cRosNodeDoEventsLoop( CrosNode *n, uint64_t max_timeout )
{
  cRosErrCodePack ret_err, new_errors;
  uint64_t cur_time, select_timeout;
  int nfds = -1;
  fd_set r_fds, w_fds, err_fds;
  int i;

  PRINT_VVDEBUG ( "cRosNodeDoEventsLoop ()\n" );

  if(n == NULL)
    return CROS_BAD_PARAM_ERR;

  #if CROS_DEBUG_LEVEL >= 2
  PRINT_DEBUG("*");
  FLUSH_PRINT();
  printNodeProcState( n );
  #endif

  cur_time = cRosClockGetTimeMs();

  ret_err = cRosNodeTriggerPublishersWriting( n, cur_time );

  new_errors = cRosNodeTriggerServiceCallersWriting( n, cur_time );
  ret_err = cRosAddErrCodePackIfErr(ret_err, new_errors);

  FD_ZERO( &r_fds );
  FD_ZERO( &w_fds );
  FD_ZERO( &err_fds );

  int xmlrpc_listner_fd = tcpIpSocketGetFD( &(n->xmlrpc_listner_proc.socket) );
  int tcpros_listner_fd = tcpIpSocketGetFD( &(n->tcpros_listner_proc.socket) );
  int rpcros_listner_fd = tcpIpSocketGetFD( &(n->rpcros_listner_proc.socket) );

  XmlrpcProcess *coreproc = &n->xmlrpc_client_proc[0];
  if (coreproc->state == XMLRPC_PROCESS_STATE_IDLE && !isQueueEmpty(&n->master_api_queue))
  {
    RosApiCall *call = dequeueApiCall(&n->master_api_queue);
    coreproc->current_call = call;
    xmlrpcProcessChangeState( coreproc, XMLRPC_PROCESS_STATE_CONNECTING );
  }

  size_t idle_client_count;
  int idle_clients[CN_MAX_XMLRPC_CLIENT_CONNECTIONS];
  getIdleXmplrpcClients(n, idle_clients, &idle_client_count);

  int next_idle_client_idx = 0;
  while (!isQueueEmpty(&n->slave_api_queue) && next_idle_client_idx != idle_client_count)
  {
    int idle_client_idx = idle_clients[next_idle_client_idx];
    RosApiCall *call = dequeueApiCall(&n->slave_api_queue);

    XmlrpcProcess *proc =  &n->xmlrpc_client_proc[idle_client_idx];
    proc->current_call = call;
    xmlrpcProcessChangeState( proc, XMLRPC_PROCESS_STATE_CONNECTING );

    next_idle_client_idx++;
  }

  /* If active (not idle state), add to the tcpIpSocketSelect() the XMLRPC clients */
  for(i = 0; i < CN_MAX_XMLRPC_CLIENT_CONNECTIONS; i++)
  {
    fd_set *fdset = NULL;
    if( n->xmlrpc_client_proc[i].state == XMLRPC_PROCESS_STATE_CONNECTING )
    {
      new_errors =  xmlrpcClientConnect(n, i);
      ret_err = cRosAddErrCodePackIfErr(ret_err, new_errors);
      fdset = &w_fds; // tcpIpSocketSelect() will acknowledge the socket connection completion in the file descriptors checked for writing
    }
    else if( n->xmlrpc_client_proc[i].state == XMLRPC_PROCESS_STATE_WRITING )
      fdset = &w_fds;
    else if( n->xmlrpc_client_proc[i].state == XMLRPC_PROCESS_STATE_READING )
      fdset = &r_fds;

    if (fdset != NULL)
    {
      int xmlrpc_client_fd = tcpIpSocketGetFD( &(n->xmlrpc_client_proc[i].socket) );
      if(xmlrpc_client_fd != -1)
      {
        if( xmlrpc_client_fd > nfds )
          nfds = xmlrpc_client_fd;

        FD_SET( xmlrpc_client_fd, fdset);
        FD_SET( xmlrpc_client_fd, &err_fds);
      }
    }
  }

  //printf("FD_SET COUNT. R: %d W: %d\n", r_count, w_count);

  /* Add to the tcpIpSocketSelect() the active XMLRPC servers */
  int next_xmlrpc_server_i = -1;
  for( i = 0; i < CN_MAX_XMLRPC_SERVER_CONNECTIONS; i++ )
  {
    int server_fd = tcpIpSocketGetFD( &(n->xmlrpc_server_proc[i].socket) );

    if( next_xmlrpc_server_i < 0 &&
        n->xmlrpc_server_proc[i].state == XMLRPC_PROCESS_STATE_IDLE )
    {
      next_xmlrpc_server_i = i;
    }
    else if( n->xmlrpc_server_proc[i].state == XMLRPC_PROCESS_STATE_READING )
    {
      FD_SET( server_fd, &r_fds);
      FD_SET( server_fd, &err_fds);
      if( server_fd > nfds ) nfds = server_fd;
    }
    else if( n->xmlrpc_server_proc[i].state == XMLRPC_PROCESS_STATE_WRITING )
    {
      FD_SET( server_fd, &w_fds);
      FD_SET( server_fd, &err_fds);
      if( server_fd > nfds ) nfds = server_fd;
    }
  }

  /* If one XMLRPC server is active at least, add to the tcpIpSocketSelect() the listener socket */
  if( next_xmlrpc_server_i >= 0)
  {
    if(xmlrpc_listner_fd != -1) // If the listener socket is still opened, add it to the tcpIpSocketSelect() file descriptors
    {
      FD_SET( xmlrpc_listner_fd, &r_fds);
      FD_SET( xmlrpc_listner_fd, &err_fds);
      if( xmlrpc_listner_fd > nfds ) nfds = xmlrpc_listner_fd;
    }
  }

  /*
   *
   * TCPROS PROCESSES tcpIpSocketSelect() MANAGEMENT
   *
   */

  /* If active (not idle state), add to the tcpIpSocketSelect() the TCPROS clients */
  int next_tcpros_client_i = -1; // Unused ???
  for(i = 0; i < CN_MAX_TCPROS_CLIENT_CONNECTIONS; i++)
  {
    TcprosProcess *client_proc = &(n->tcpros_client_proc[i]);
    int tcpros_client_fd;

    if( next_tcpros_client_i < 0 &&
        i != 0 && //the zero-index is reserved to the roscore communications
        client_proc->state == TCPROS_PROCESS_STATE_IDLE )
    {
      next_tcpros_client_i = i;
    }
    else if(client_proc->state == TCPROS_PROCESS_STATE_CONNECTING)
    {
      new_errors =  tcprosClientConnect(n, i);
      ret_err = cRosAddErrCodePackIfErr(ret_err, new_errors);

      tcpros_client_fd = tcpIpSocketGetFD( &(client_proc->socket) );
      FD_SET( tcpros_client_fd, &w_fds);
      FD_SET( tcpros_client_fd, &err_fds);
      if( tcpros_client_fd > nfds ) nfds = tcpros_client_fd;
    }
    else if(client_proc->state == TCPROS_PROCESS_STATE_WRITING_HEADER)
    {
      tcpros_client_fd = tcpIpSocketGetFD( &(client_proc->socket) );
      FD_SET( tcpros_client_fd, &w_fds);
      FD_SET( tcpros_client_fd, &err_fds);
      if( tcpros_client_fd > nfds ) nfds = tcpros_client_fd;
    }
    else if(client_proc->state == TCPROS_PROCESS_STATE_READING_HEADER_SIZE ||
            client_proc->state == TCPROS_PROCESS_STATE_READING_HEADER ||
            client_proc->state == TCPROS_PROCESS_STATE_READING_SIZE ||
            client_proc->state == TCPROS_PROCESS_STATE_READING)
    {
      tcpros_client_fd = tcpIpSocketGetFD( &(client_proc->socket) );
      FD_SET( tcpros_client_fd, &r_fds);
      FD_SET( tcpros_client_fd, &err_fds);
      if( tcpros_client_fd > nfds ) nfds = tcpros_client_fd;
    }
  }

  /* Add to the tcpIpSocketSelect() the active TCPROS servers */
  int next_tcpros_server_i = -1; // Index of the first server that is idle (ready to be activated). -1 = no one is idle
  for( i = 0; i < CN_MAX_TCPROS_SERVER_CONNECTIONS; i++ )
  {
    int server_fd = tcpIpSocketGetFD( &(n->tcpros_server_proc[i].socket) );

    if( next_tcpros_server_i < 0 &&
        n->tcpros_server_proc[i].state == TCPROS_PROCESS_STATE_IDLE )
    {
      next_tcpros_server_i = i;
    }
    else if( n->tcpros_server_proc[i].state == TCPROS_PROCESS_STATE_READING_HEADER )
    {
      FD_SET( server_fd, &r_fds);
      FD_SET( server_fd, &err_fds);
      if( server_fd > nfds ) nfds = server_fd;
    }
    else if( n->tcpros_server_proc[i].state == TCPROS_PROCESS_STATE_START_WRITING ||
             n->tcpros_server_proc[i].state == TCPROS_PROCESS_STATE_WRITING )
    {
      FD_SET( server_fd, &w_fds);
      FD_SET( server_fd, &err_fds);
      if( server_fd > nfds ) nfds = server_fd;
    }
    else if( n->tcpros_server_proc[i].state == TCPROS_PROCESS_STATE_WAIT_FOR_WRITING )
    {
      FD_SET( server_fd, &err_fds);
      if( server_fd > nfds ) nfds = server_fd;
    }
  }

  /* If one TCPROS server is available at least, add to the tcpIpSocketSelect() the listener socket */
  if( next_tcpros_server_i >= 0)
  {
    if(tcpros_listner_fd != -1) // If the listener socket is still opened
    {
      FD_SET( tcpros_listner_fd, &r_fds);
      FD_SET( tcpros_listner_fd, &err_fds);
      if( tcpros_listner_fd > nfds ) nfds = tcpros_listner_fd;
    }
  }


  /*
   *
   * RPCROS PROCESSES tcpIpSocketSelect() MANAGEMENT
   *
   */

  /* If active (not idle state), add to the tcpIpSocketSelect() the TCPROS clients */
  for(i = 0; i < CN_MAX_RPCROS_CLIENT_CONNECTIONS; i++)
  {
    int rpcros_client_fd;

    rpcros_client_fd = tcpIpSocketGetFD( &(n->rpcros_client_proc[i].socket) );
    if(n->rpcros_client_proc[i].state == TCPROS_PROCESS_STATE_CONNECTING)
    {
      new_errors =  rpcrosClientConnect(n, i);
      ret_err = cRosAddErrCodePackIfErr(ret_err, new_errors);

      rpcros_client_fd = tcpIpSocketGetFD( &(n->rpcros_client_proc[i].socket) ); // update file descriptor after connecting
      FD_SET( rpcros_client_fd, &w_fds);
      FD_SET( rpcros_client_fd, &err_fds);
      if( rpcros_client_fd > nfds ) nfds = rpcros_client_fd;
    }
    else if(n->rpcros_client_proc[i].state == TCPROS_PROCESS_STATE_WRITING_HEADER ||
       n->rpcros_client_proc[i].state == TCPROS_PROCESS_STATE_START_WRITING ||
       n->rpcros_client_proc[i].state == TCPROS_PROCESS_STATE_WRITING)
    {
      FD_SET( rpcros_client_fd, &w_fds);
      FD_SET( rpcros_client_fd, &err_fds);
      if( rpcros_client_fd > nfds ) nfds = rpcros_client_fd;
    }
    else if(n->rpcros_client_proc[i].state == TCPROS_PROCESS_STATE_READING_HEADER_SIZE ||
            n->rpcros_client_proc[i].state == TCPROS_PROCESS_STATE_READING_HEADER ||
            n->rpcros_client_proc[i].state == TCPROS_PROCESS_STATE_READING_SIZE ||
            n->rpcros_client_proc[i].state == TCPROS_PROCESS_STATE_READING)
    {
      FD_SET( rpcros_client_fd, &r_fds);
      FD_SET( rpcros_client_fd, &err_fds);
      if( rpcros_client_fd > nfds ) nfds = rpcros_client_fd;
    }
    else if(n->rpcros_client_proc[i].state == TCPROS_PROCESS_STATE_WAIT_FOR_WRITING)
    {
      FD_SET( rpcros_client_fd, &err_fds);
      if( rpcros_client_fd > nfds ) nfds = rpcros_client_fd;
    }
  }


  /* Add to the tcpIpSocketSelect() the active RPCROS servers */
  int next_rpcros_server_i = -1;

  for( i = 0; i < CN_MAX_RPCROS_SERVER_CONNECTIONS; i++ )
  {
    int server_fd = tcpIpSocketGetFD( &(n->rpcros_server_proc[i].socket) );

    if( next_rpcros_server_i < 0 &&
        n->rpcros_server_proc[i].state == TCPROS_PROCESS_STATE_IDLE )
    {
    	next_rpcros_server_i = i;
    }
    else if (n->rpcros_server_proc[i].state == TCPROS_PROCESS_STATE_READING_HEADER_SIZE ||
             n->rpcros_server_proc[i].state == TCPROS_PROCESS_STATE_READING_HEADER ||
             n->rpcros_server_proc[i].state == TCPROS_PROCESS_STATE_READING_SIZE ||
             n->rpcros_server_proc[i].state == TCPROS_PROCESS_STATE_READING)
    {
      FD_SET( server_fd, &r_fds);
      FD_SET( server_fd, &err_fds);
      if( server_fd > nfds ) nfds = server_fd;
    }
    else if( n->rpcros_server_proc[i].state == TCPROS_PROCESS_STATE_WRITING_HEADER ||
             n->rpcros_server_proc[i].state == TCPROS_PROCESS_STATE_WRITING )
    {
      FD_SET( server_fd, &w_fds);
      FD_SET( server_fd, &err_fds);
      if( server_fd > nfds ) nfds = server_fd;
    }
  }

  /* If one RPCROS server is available at least, add to the tcpIpSocketSelect() the listner socket */
  if( next_rpcros_server_i >= 0)
  {
    if(rpcros_listner_fd != -1) // If the listener socket is still opened
    {
      FD_SET( rpcros_listner_fd, &r_fds);
      FD_SET( rpcros_listner_fd, &err_fds);
      if( rpcros_listner_fd > nfds ) nfds = rpcros_listner_fd;
    }
  }

  if (nfds + 1 == 0)
  {
    PRINT_VDEBUG("cRosNodeDoEventsLoop() : Warning: tcpIpSocketSelect() is being called with no file descriptors to monitor.\n");
  }

  select_timeout = cRosNodeCalculateSelectTimeout(n, max_timeout);

  // The node waits here until the specified file descriptors become ready for the corresponding I/O operation or the timeout is up
  // ------------------------------------------------------------------------------------------------------------------------------
  int n_set = tcpIpSocketSelect(nfds + 1, &r_fds, &w_fds, &err_fds, select_timeout);

  cur_time = cRosClockGetTimeMs(); // Update current time after select()
  if (n_set == -1)
  {
    PRINT_ERROR("cRosNodeDoEventsLoop() : tcpIpSocketSelect() function failed.\n");
    ret_err = CROS_SELECT_FD_ERR;
  }
  else if( n_set == 0 )
  {
    PRINT_VDEBUG ("cRosNodeDoEventsLoop() : tcpIpSocketSelect() finished due to timeout (parameter: %llu ms) or it was interrupted\n", (long long unsigned)select_timeout);

    XmlrpcProcess *rosproc = &n->xmlrpc_client_proc[0];
    if(n->xmlrpc_master_wake_up_time <= cur_time ) // It's time to wakeup, ping master, and maybe look up in master for pending services
    {
      PRINT_VDEBUG("cRosNodeDoEventsLoop() : It is ime to wake up the Master XML RPC process. Current time: %lu Wake up time: %lu\n", cur_time, n->xmlrpc_master_wake_up_time);
      if(rosproc->state == XMLRPC_PROCESS_STATE_IDLE)
      {
        // Prepare to ping roscore ...
        PRINT_VDEBUG("cRosNodeDoEventsLoop() : Sending ping to ROS Master\n");

        RosApiCall *call = newRosApiCall();
        if (call != NULL)
        {
          call->method = CROS_API_GET_PID;
          int rc = xmlrpcParamVectorPushBackString(&call->params, n->name);
          if(rc >= 0)
          {
            rosproc->message_type = XMLRPC_MESSAGE_REQUEST;
            generateXmlrpcMessage( n->roscore_host, n->roscore_port, rosproc->message_type,
                                  getMethodName(call->method), &call->params, &rosproc->message );

            rosproc->current_call = call;
            xmlrpcProcessChangeState(rosproc, XMLRPC_PROCESS_STATE_CONNECTING );
          }
          else
          {
            PRINT_ERROR ( "cRosNodeDoEventsLoop() : Can't allocate memory\n");
            ret_err=CROS_MEM_ALLOC_ERR;
          }

          // The ROS master does not warn us when a new service is registered, so we have to
          // continuously check for the required service
          for(i = 0; i < CN_MAX_RPCROS_CLIENT_CONNECTIONS; i++ )
          {
             TcprosProcess *client_proc = &(n->rpcros_client_proc[i]);
             if( client_proc->state == TCPROS_PROCESS_STATE_WAIT_FOR_CONNECTING)
             {
               tcprosProcessChangeState(client_proc, TCPROS_PROCESS_STATE_IDLE);
               enqueueServiceLookup(n, client_proc->service_idx);
             }
          }
          n->xmlrpc_master_wake_up_time = cur_time + CN_PING_LOOP_PERIOD; // The process completed doing what it should, so wake up again CN_PING_LOOP_PERIOD milliseconds later
        }
        else
        {
          PRINT_ERROR ( "cRosNodeDoEventsLoop() : Can't allocate memory\n");
          ret_err=CROS_MEM_ALLOC_ERR;
        }
      }
      else
        n->xmlrpc_master_wake_up_time = cur_time + CN_PING_LOOP_PERIOD/50; // The process is busy, so try to wake up again soon (CN_PING_LOOP_PERIOD/50 milliseconds later) to do what is pending
    }
    if( rosproc->state != XMLRPC_PROCESS_STATE_IDLE && cRosClockGetTimeMs() - rosproc->last_change_time > CN_IO_TIMEOUT ) // last_change_time is updated when changing process state
    {
      // Timeout between I/O operations... close the socket and re-advertise
      PRINT_VDEBUG ( "cRosNodeDoEventsLoop() : XMLRPC client I/O timeout\n");
      handleXmlrpcClientError( n, 0 );
    }


    for( i = 0; i < CN_MAX_TCPROS_SERVER_CONNECTIONS && ret_err==CROS_SUCCESS_ERR_PACK; i++ )
    {
      if( (n->tcpros_server_proc[i].state == TCPROS_PROCESS_STATE_READING_HEADER ||
                n->tcpros_server_proc[i].state == TCPROS_PROCESS_STATE_WRITING ) &&
               cur_time - n->tcpros_server_proc[i].last_change_time > CN_IO_TIMEOUT )
      {
        // Timeout between I/O operations
        PRINT_VDEBUG ( "cRosNodeDoEventsLoop() : TCPROS server I/O timeout\n");
        handleTcprosServerError( n, i );
      }
    }

    for( i = 0; i < CN_MAX_RPCROS_CLIENT_CONNECTIONS && ret_err==CROS_SUCCESS_ERR_PACK; i++ )
    {
      if( (n->rpcros_client_proc[i].state == TCPROS_PROCESS_STATE_READING_HEADER || // Add more states to the condition???
                n->rpcros_client_proc[i].state == TCPROS_PROCESS_STATE_WRITING ) &&
               cur_time - n->rpcros_client_proc[i].last_change_time > CN_IO_TIMEOUT )
      {
        // Timeout between I/O operations
        PRINT_VDEBUG ( "cRosNodeDoEventsLoop() : TCPROS server I/O timeout\n");
        handleTcprosServerError( n, i );
      }
    }
  }
  else
  {
    PRINT_VDEBUG ( "cRosNodeDoEventsLoop() : tcpIpSocketSelect() finished with num. fd set: %i (timeout parameter was: %llu ms)\n", n_set, (long long unsigned)select_timeout);
    for(i = 0; i < CN_MAX_XMLRPC_CLIENT_CONNECTIONS; i++ )
    {
      XmlrpcProcess *client_proc;
      int xmlrpc_client_fd;

      client_proc = &n->xmlrpc_client_proc[i];
      xmlrpc_client_fd = tcpIpSocketGetFD( &client_proc->socket );
      if( client_proc->state != XMLRPC_PROCESS_STATE_IDLE && FD_ISSET(xmlrpc_client_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : XMLRPC client socket error\n" );
        handleXmlrpcClientError( n, i );
      }
      /* Check what is the socket unblocked by tcpIpSocketSelect(), and start the requested operations */
      else if( ( client_proc->state == XMLRPC_PROCESS_STATE_CONNECTING && FD_ISSET(xmlrpc_client_fd, &w_fds) ) ||
          ( client_proc->state == XMLRPC_PROCESS_STATE_WRITING && FD_ISSET(xmlrpc_client_fd, &w_fds) ) ||
          ( client_proc->state == XMLRPC_PROCESS_STATE_READING && FD_ISSET(xmlrpc_client_fd, &r_fds) ) )
      {
        new_errors = doWithXmlrpcClientSocket( n, i );;
        ret_err = cRosAddErrCodePackIfErr(ret_err, new_errors);
      }
    }

    if ( next_xmlrpc_server_i >= 0 && xmlrpc_listner_fd != -1) // Check that there is an available free (idle) xmlrpx process and that the listener socket is still open
    {
      if( FD_ISSET( xmlrpc_listner_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : XMLRPC server listener-socket error\n" );
      }
      else if( FD_ISSET( xmlrpc_listner_fd, &r_fds) )
      {
        PRINT_VDEBUG ( "cRosNodeDoEventsLoop() : XMLRPC server listener-socket ready\n" );
        if( tcpIpSocketAccept( &(n->xmlrpc_listner_proc.socket),
            &(n->xmlrpc_server_proc[next_xmlrpc_server_i].socket) ) == TCPIPSOCKET_DONE &&
            tcpIpSocketSetReuse( &(n->xmlrpc_server_proc[next_xmlrpc_server_i].socket) ) &&
            tcpIpSocketSetNonBlocking( &(n->xmlrpc_server_proc[next_xmlrpc_server_i].socket ) ) )

          xmlrpcProcessChangeState( &(n->xmlrpc_server_proc[next_xmlrpc_server_i]), XMLRPC_PROCESS_STATE_READING );
      }
    }

    for( i = 0; i < CN_MAX_XMLRPC_SERVER_CONNECTIONS; i++ )
    {
      XmlrpcProcess *server_proc;
      int server_fd;

      server_proc = &n->xmlrpc_server_proc[i];
      server_fd = tcpIpSocketGetFD( &server_proc->socket );
      if( server_proc->state != XMLRPC_PROCESS_STATE_IDLE && FD_ISSET(server_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : XMLRPC server socket error\n" );
        tcpIpSocketClose( &(n->xmlrpc_server_proc[i].socket) );
        xmlrpcProcessChangeState( &(n->xmlrpc_server_proc[next_xmlrpc_server_i]), XMLRPC_PROCESS_STATE_IDLE ); // maybe server_proc instead of &(n->xmlrpc_server_proc[next_xmlrpc_server_i]) ???
      }
      else if( ( server_proc->state == XMLRPC_PROCESS_STATE_WRITING && FD_ISSET(server_fd, &w_fds) ) ||
               ( server_proc->state == XMLRPC_PROCESS_STATE_READING && FD_ISSET(server_fd, &r_fds) ) )
      {
        new_errors = doWithXmlrpcServerSocket( n, i );
        ret_err = cRosAddErrCodePackIfErr(ret_err, new_errors);
      }
    }

    for(i = 0; i < CN_MAX_TCPROS_CLIENT_CONNECTIONS; i++ )
    {
      TcprosProcess *client_proc = &(n->tcpros_client_proc[i]);
      int tcpros_client_fd = tcpIpSocketGetFD( &(client_proc->socket) );

      if( client_proc->state != TCPROS_PROCESS_STATE_IDLE && FD_ISSET(tcpros_client_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : XMLRPC client socket error\n" );
        handleTcprosClientError( n, i );
      }

      if( (client_proc->state == TCPROS_PROCESS_STATE_CONNECTING && FD_ISSET(tcpros_client_fd, &w_fds) ) || // tcpIpSocketSelect() indicates connection completion through write-fd
          ( client_proc->state == TCPROS_PROCESS_STATE_WRITING_HEADER && FD_ISSET(tcpros_client_fd, &w_fds) ) ||
          ( client_proc->state == TCPROS_PROCESS_STATE_READING_SIZE && FD_ISSET(tcpros_client_fd, &r_fds) ) ||
          ( client_proc->state == TCPROS_PROCESS_STATE_READING && FD_ISSET(tcpros_client_fd, &r_fds) ) ||
          ( client_proc->state == TCPROS_PROCESS_STATE_READING_HEADER_SIZE && FD_ISSET(tcpros_client_fd, &r_fds) ) ||
          ( client_proc->state == TCPROS_PROCESS_STATE_READING_HEADER && FD_ISSET(tcpros_client_fd, &r_fds) ) )
      {
        new_errors = doWithTcprosClientSocket( n, i );
        ret_err = cRosAddErrCodePackIfErr(ret_err, new_errors);
      }
    }

    if ( next_tcpros_server_i >= 0 && tcpros_listner_fd != -1)
    {
      if( FD_ISSET( tcpros_listner_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : TCPROS listener-socket error\n" );
      }
      else if( FD_ISSET( tcpros_listner_fd, &r_fds) )
      {
        PRINT_VDEBUG ( "cRosNodeDoEventsLoop() : TCPROS listener ready\n" );
        if( tcpIpSocketAccept( &(n->tcpros_listner_proc.socket),
            &(n->tcpros_server_proc[next_tcpros_server_i].socket) ) == TCPIPSOCKET_DONE &&
            tcpIpSocketSetReuse( &(n->tcpros_server_proc[next_tcpros_server_i].socket) ) &&
            tcpIpSocketSetNonBlocking( &(n->tcpros_server_proc[next_tcpros_server_i].socket ) ) &&
            tcpIpSocketSetKeepAlive( &(n->tcpros_server_proc[next_tcpros_server_i].socket ), 60, 10, 9 ) )
        {
          tcprosProcessChangeState( &(n->tcpros_server_proc[next_tcpros_server_i]), TCPROS_PROCESS_STATE_READING_HEADER ); // A TCPROS process has been activated to attend the connection
        }
      }
    }

    for( i = 0; i < CN_MAX_TCPROS_SERVER_CONNECTIONS; i++ )
    {
      TcprosProcess *server_proc;
      int server_fd;

      server_proc = &n->tcpros_server_proc[i];
      server_fd = tcpIpSocketGetFD( &server_proc->socket );
      if( server_proc->state != TCPROS_PROCESS_STATE_IDLE && FD_ISSET(server_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : TCPROS server socket error\n" );
        tcpIpSocketClose( &(server_proc->socket) );
        tcprosProcessChangeState( &(n->tcpros_server_proc[next_tcpros_server_i]), TCPROS_PROCESS_STATE_IDLE );
      }
      else if( ( server_proc->state == TCPROS_PROCESS_STATE_READING_HEADER && FD_ISSET(server_fd, &r_fds) ) ||
        ( server_proc->state == TCPROS_PROCESS_STATE_START_WRITING && FD_ISSET(server_fd, &w_fds) ) ||
        ( server_proc->state == TCPROS_PROCESS_STATE_WRITING && FD_ISSET(server_fd, &w_fds) ) )
      {
        new_errors = doWithTcprosServerSocket( n, i );
        ret_err = cRosAddErrCodePackIfErr(ret_err, new_errors);
      }
    }

    for(i = 0; i < CN_MAX_RPCROS_CLIENT_CONNECTIONS; i++ )
    {
      TcprosProcess *client_proc = &(n->rpcros_client_proc[i]);
      int rpcros_client_fd = tcpIpSocketGetFD( &(client_proc->socket) );

      if( client_proc->state != TCPROS_PROCESS_STATE_IDLE && FD_ISSET(rpcros_client_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : RPCROS client socket error\n" );
        handleRpcrosClientError( n, i );
      }

      if( ( client_proc->state == TCPROS_PROCESS_STATE_CONNECTING && FD_ISSET(rpcros_client_fd, &w_fds) ) ||
          ( client_proc->state == TCPROS_PROCESS_STATE_WRITING_HEADER && FD_ISSET(rpcros_client_fd, &w_fds) ) ||
          ( client_proc->state == TCPROS_PROCESS_STATE_READING_SIZE && FD_ISSET(rpcros_client_fd, &r_fds) ) ||
          ( client_proc->state == TCPROS_PROCESS_STATE_READING && FD_ISSET(rpcros_client_fd, &r_fds) ) ||
          ( client_proc->state == TCPROS_PROCESS_STATE_READING_HEADER_SIZE && FD_ISSET(rpcros_client_fd, &r_fds) ) ||
          ( client_proc->state == TCPROS_PROCESS_STATE_READING_HEADER && FD_ISSET(rpcros_client_fd, &r_fds) ) ||
          ( client_proc->state == TCPROS_PROCESS_STATE_START_WRITING && FD_ISSET(rpcros_client_fd, &w_fds) ) ||
          ( client_proc->state == TCPROS_PROCESS_STATE_WRITING && FD_ISSET(rpcros_client_fd, &w_fds) ) )
      {
        new_errors = doWithRpcrosClientSocket( n, i );
        ret_err = cRosAddErrCodePackIfErr(ret_err, new_errors);
      }
    }

    if ( next_rpcros_server_i >= 0 && rpcros_listner_fd != -1)
    {
      if( FD_ISSET( rpcros_listner_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : TCPROS listener-socket error\n" );
      }
      else if( next_rpcros_server_i >= 0 && FD_ISSET( rpcros_listner_fd, &r_fds) )
      {
        PRINT_VDEBUG ( "cRosNodeDoEventsLoop() : TCPROS listener ready\n" );
        if( tcpIpSocketAccept( &(n->rpcros_listner_proc.socket),
            &(n->rpcros_server_proc[next_rpcros_server_i].socket) ) == TCPIPSOCKET_DONE &&
            tcpIpSocketSetReuse( &(n->rpcros_server_proc[next_rpcros_server_i].socket) ) &&
            tcpIpSocketSetNonBlocking( &(n->rpcros_server_proc[next_rpcros_server_i].socket ) ) &&
            tcpIpSocketSetKeepAlive( &(n->rpcros_server_proc[next_rpcros_server_i].socket ), 60, 10, 9 ) )
        {
          tcprosProcessChangeState( &(n->rpcros_server_proc[next_rpcros_server_i]), TCPROS_PROCESS_STATE_READING_HEADER_SIZE );
        }
      }
    }

    for( i = 0; i < CN_MAX_RPCROS_SERVER_CONNECTIONS; i++ )
    {
      TcprosProcess *server_proc = &(n->rpcros_server_proc[i]);
      int server_fd = tcpIpSocketGetFD( &(server_proc->socket) );

      if( server_proc->state != TCPROS_PROCESS_STATE_IDLE && FD_ISSET(server_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : TCPROS server socket error\n" );
        tcpIpSocketClose( &(server_proc->socket) );
        tcprosProcessChangeState( &(n->rpcros_server_proc[next_rpcros_server_i]), TCPROS_PROCESS_STATE_IDLE );
      }
      else if( ( server_proc->state == TCPROS_PROCESS_STATE_READING_HEADER_SIZE && FD_ISSET(server_fd, &r_fds) ) ||
        ( server_proc->state == TCPROS_PROCESS_STATE_READING_HEADER && FD_ISSET(server_fd, &r_fds) ) ||
        ( server_proc->state == TCPROS_PROCESS_STATE_READING_SIZE && FD_ISSET(server_fd, &r_fds) ) ||
        ( server_proc->state == TCPROS_PROCESS_STATE_READING && FD_ISSET(server_fd, &r_fds) ) ||
        ( server_proc->state == TCPROS_PROCESS_STATE_WRITING_HEADER && FD_ISSET(server_fd, &w_fds) ) ||
        ( server_proc->state == TCPROS_PROCESS_STATE_WRITING && FD_ISSET(server_fd, &w_fds) ) )
      {
        new_errors = doWithRpcrosServerSocket( n, i );
        ret_err = cRosAddErrCodePackIfErr(ret_err, new_errors);
      }
    }
  }
  return ret_err;
}

cRosErrCodePack cRosNodeStart( CrosNode *n, unsigned long time_out, unsigned char *exit_flag )
{
  uint64_t start_time, elapsed_time;
  cRosErrCodePack ret_err;
  PRINT_VVDEBUG ( "cRosNodeStart ()\n" );

  start_time = cRosClockGetTimeMs();
  ret_err = CROS_SUCCESS_ERR_PACK;
  while(ret_err == CROS_SUCCESS_ERR_PACK && (exit_flag==NULL || !(*exit_flag)) && (time_out == CROS_INFINITE_TIMEOUT || (elapsed_time=cRosClockGetTimeMs()-start_time) <= time_out))
    ret_err = cRosNodeDoEventsLoop( n, (time_out == CROS_INFINITE_TIMEOUT)? UINT64_MAX : time_out-elapsed_time);

  return ret_err;
}

cRosErrCodePack cRosNodeReceiveTopicMsg( CrosNode *node, int subidx, cRosMessage *msg, unsigned char *buff_overflow, unsigned long time_out )
{
  cRosErrCodePack ret_err;
  SubscriberNode *subs_node;
  uint64_t start_time, elapsed_time = 0; // Initialized just to avoid a compiler warning
  PRINT_VVDEBUG ( "cRosNodeReceiveTopicMsg ()\n" );

  if(subidx < 0 || subidx >= CN_MAX_SUBSCRIBED_TOPICS)
    return CROS_BAD_PARAM_ERR;

  subs_node = &node->subs[subidx];
  if(subs_node->topic_name == NULL)
    return CROS_BAD_PARAM_ERR;

  if(buff_overflow != NULL)
    *buff_overflow = subs_node->msg_queue_overflow;
  subs_node->msg_queue_overflow = 0; // Reset overflow flag

  start_time = cRosClockGetTimeMs();
  ret_err = CROS_SUCCESS_ERR_PACK; // default return value
  // While the buffer is empty and the timeout is not reached wait
  while(cRosMessageQueueUsage(&subs_node->msg_queue) == 0 && ret_err == CROS_SUCCESS_ERR_PACK && (time_out == CROS_INFINITE_TIMEOUT || (elapsed_time=cRosClockGetTimeMs()-start_time) <= time_out))
  {
    ret_err = cRosNodeDoEventsLoop ( node, time_out - elapsed_time);
  }
  if(ret_err == CROS_SUCCESS_ERR_PACK)
  {
    if(cRosMessageQueueUsage(&subs_node->msg_queue) > 0) // If no error and there is at least one message in the queue, get the message
      ret_err = (cRosMessageQueueExtract(&subs_node->msg_queue, msg) == 0)? CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
    else
      ret_err = CROS_RCV_TOP_TIMEOUT_ERR;
  }
  return ret_err;
}

cRosErrCodePack cRosNodeQueueTopicMsg( CrosNode *node, int pubidx, cRosMessage *msg )
{
  cRosErrCodePack ret_err;
  PublisherNode *pub_node;

  pub_node = &node->pubs[pubidx];
  {
    if(cRosMessageQueueVacancies(&pub_node->msg_queue) > 0) // If no error and there is space in the queue, put the new message
    {
      if(cRosMessageQueueAdd(&pub_node->msg_queue, msg) == 0)
        ret_err = CROS_SUCCESS_ERR_PACK;
      else
        ret_err = CROS_MEM_ALLOC_ERR;
    }
    else
      ret_err = CROS_SEND_TOP_TIMEOUT_ERR;
  }
  return ret_err;
}

cRosErrCodePack cRosNodeSendTopicMsg( CrosNode *node, int pubidx, cRosMessage *msg, unsigned long time_out )
{
  cRosErrCodePack ret_err;
  PublisherNode *pub_node;
  uint64_t start_time, elapsed_time = 0; // Initialized just to avoid a compiler warning
  PRINT_VVDEBUG ( "cRosNodeSendTopicMsg ()\n" );

  if(pubidx < 0 || pubidx >= CN_MAX_PUBLISHED_TOPICS)
    return CROS_BAD_PARAM_ERR;

  pub_node = &node->pubs[pubidx];
  if(pub_node->topic_name == NULL)
    return CROS_BAD_PARAM_ERR;

  start_time = cRosClockGetTimeMs();
  ret_err = CROS_SUCCESS_ERR_PACK; // default return value
  // While the buffer is full and the timeout is not reached wait
  while(cRosMessageQueueVacancies(&pub_node->msg_queue) == 0 && ret_err == CROS_SUCCESS_ERR_PACK && (time_out == CROS_INFINITE_TIMEOUT || (elapsed_time=cRosClockGetTimeMs()-start_time) <= time_out))
  {
    ret_err = cRosNodeDoEventsLoop ( node, time_out - elapsed_time);
  }

  if(ret_err == CROS_SUCCESS_ERR_PACK)
     cRosNodeQueueTopicMsg( node, pubidx, msg );

  return ret_err;
}


cRosErrCodePack cRosNodeServiceCall( CrosNode *node, int svcidx, cRosMessage *req_msg, cRosMessage *resp_msg, unsigned long time_out)
{
  cRosErrCodePack ret_err;
  ServiceCallerNode *caller_node;
  TcprosProcess *svc_client_proc;
  uint64_t start_time, elapsed_time = 0; // Initialized just to avoid a compiler warning
  PRINT_VVDEBUG ( "cRosNodeServiceCall ()\n" );

  if(svcidx < 0 || svcidx >= CN_MAX_SERVICE_CALLERS)
    return CROS_BAD_PARAM_ERR;

  caller_node = &node->service_callers[svcidx];
  if(caller_node->service_name == NULL)
    return CROS_BAD_PARAM_ERR;

  svc_client_proc = &node->rpcros_client_proc[caller_node->rpcros_id];

  start_time = cRosClockGetTimeMs();
  // Wait until the RPCROS process has finished the current call and the timeout is not reached wait
  ret_err = CROS_SUCCESS_ERR_PACK;
  while(svc_client_proc->state != TCPROS_PROCESS_STATE_WAIT_FOR_WRITING && ret_err == CROS_SUCCESS_ERR_PACK && (time_out == CROS_INFINITE_TIMEOUT || (elapsed_time=cRosClockGetTimeMs()-start_time) <= time_out))
  {
    ret_err = cRosNodeDoEventsLoop ( node, time_out - elapsed_time);
  }

  if(ret_err == CROS_SUCCESS_ERR_PACK && svc_client_proc->state != TCPROS_PROCESS_STATE_WAIT_FOR_WRITING)
    ret_err = CROS_CALL_INI_TIMEOUT_ERR;
  if(ret_err != CROS_SUCCESS_ERR_PACK)
    return ret_err; // If an error occurred or the process is not in TCPROS_PROCESS_STATE_WAIT_FOR_WRITING state, exit

  if(cRosMessageQueueUsage(&caller_node->msg_queue) > 0) // Unfinished service call?
  {
    PRINT_ERROR ( "cRosNodeServiceCall () : The service caller is not ready to make a new call. Overwriting previous state and trying anyway...\n" );
    cRosMessageQueueClear(&caller_node->msg_queue);
  }

  if(cRosMessageQueueAdd(&caller_node->msg_queue, req_msg) != 0) // Put service-call request msg in the queue
    ret_err = CROS_MEM_ALLOC_ERR;

  // Wait while the buffer is full and the timeout is not reached
  while(cRosMessageQueueUsage(&caller_node->msg_queue) < 2 && ret_err == CROS_SUCCESS_ERR_PACK && (time_out == CROS_INFINITE_TIMEOUT || (elapsed_time=cRosClockGetTimeMs()-start_time) <= time_out))
  {
    ret_err = cRosNodeDoEventsLoop ( node, time_out - elapsed_time);
  }

  if(ret_err == CROS_SUCCESS_ERR_PACK)
  {
    if(cRosMessageQueueUsage(&caller_node->msg_queue) > 1) // If no error and the response is in the buffer
    {
      cRosMessageQueueRemove(&caller_node->msg_queue); // Remove request msg from queue
      if(resp_msg != NULL)
      {
        int queue_ret_val;
        queue_ret_val = cRosMessageQueueExtract(&caller_node->msg_queue, resp_msg); // Extract response msg from queue
        if(queue_ret_val == 0)
          ret_err = CROS_SUCCESS_ERR_PACK;
        else
          ret_err = CROS_MEM_ALLOC_ERR;
      }
      else // The user is not interested in the service response, just remove the msg from queue
        cRosMessageQueueRemove(&caller_node->msg_queue);
    }
    else
      ret_err = CROS_CALL_SVC_TIMEOUT_ERR;
  }

  return ret_err;
}

int enqueueSubscriberAdvertise(CrosNode *node, int subidx)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->provider_idx= subidx;
  call->method = CROS_API_REGISTER_SUBSCRIBER;

  SubscriberNode *sub = &node->subs[subidx];
  xmlrpcParamVectorPushBackString( &call->params, node->name );
  xmlrpcParamVectorPushBackString( &call->params, sub->topic_name );
  xmlrpcParamVectorPushBackString( &call->params, sub->topic_type );
  char node_uri[256];
  snprintf( node_uri, 256, "http://%s:%d/", node->host, node->xmlrpc_port);
  xmlrpcParamVectorPushBackString( &call->params, node_uri );

  return enqueueMasterApiCallInternal(node, call);
}

int enqueuePublisherAdvertise(CrosNode *node, int pubidx)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterPublisher() : Can't allocate memory\n");
    return -1;
  }

  call->provider_idx= pubidx;
  call->method = CROS_API_REGISTER_PUBLISHER;

  PublisherNode *publiser = &node->pubs[pubidx];
  xmlrpcParamVectorPushBackString( &call->params, node->name);
  xmlrpcParamVectorPushBackString( &call->params, publiser->topic_name );
  xmlrpcParamVectorPushBackString( &call->params, publiser->topic_type );
  char node_uri[256];
  snprintf( node_uri, 256, "http://%s:%d/", node->host, node->xmlrpc_port);
  xmlrpcParamVectorPushBackString( &call->params, node_uri );

  return enqueueMasterApiCallInternal(node, call);
}

int enqueueServiceAdvertise(CrosNode *node, int serviceidx)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterServiceProvider() : Can't allocate memory\n");
    return -1;
  }

  call->provider_idx = serviceidx;
  call->method = CROS_API_REGISTER_SERVICE;

  ServiceProviderNode *service = &node->service_providers[serviceidx];
  xmlrpcParamVectorPushBackString( &call->params, node->name );
  xmlrpcParamVectorPushBackString( &call->params, service->service_name );
  char uri[256];
  snprintf( uri, 256, "rosrpc://%s:%d/", node->host, node->rpcros_port);
  xmlrpcParamVectorPushBackString( &call->params, uri );
  snprintf( uri, 256, "http://%s:%d/", node->host, node->xmlrpc_port);
  xmlrpcParamVectorPushBackString( &call->params, uri );

  return enqueueMasterApiCallInternal(node, call);
}

int enqueueServiceLookup(CrosNode *node, int serviceidx)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "enqueueServiceLookup() : Can't allocate memory\n");
    return -1;
  }

  call->provider_idx= serviceidx;
  call->method = CROS_API_LOOKUP_SERVICE;

  ServiceCallerNode *service = &node->service_callers[serviceidx];
  xmlrpcParamVectorPushBackString( &call->params, node->name );
  xmlrpcParamVectorPushBackString( &call->params, service->service_name );

  return enqueueMasterApiCallInternal(node, call);
}

int enqueueParameterSubscription(CrosNode *node, int parameteridx)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterServiceProvider() : Can't allocate memory\n");
    return -1;
  }

  call->provider_idx = parameteridx;
  call->method = CROS_API_SUBSCRIBE_PARAM;

  ParameterSubscription *subscrition = &node->paramsubs[parameteridx];
  xmlrpcParamVectorPushBackString( &call->params, node->name );
  char node_uri[256];
  snprintf( node_uri, 256, "http://%s:%d/", node->host, node->xmlrpc_port);
  xmlrpcParamVectorPushBackString( &call->params, node_uri );
  xmlrpcParamVectorPushBackString( &call->params, subscrition->parameter_key);

  return enqueueMasterApiCallInternal(node, call);
}

int enqueueParameterUnsubscription(CrosNode *node, int parameteridx)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterServiceProvider() : Can't allocate memory\n");
    return -1;
  }

  call->method = CROS_API_UNSUBSCRIBE_PARAM;
  call->provider_idx = parameteridx;

  ParameterSubscription *subscrition = &node->paramsubs[parameteridx];
  xmlrpcParamVectorPushBackString( &call->params, node->name);
  char node_uri[256];
  snprintf( node_uri, 256, "http://%s:%d/", node->host, node->xmlrpc_port);
  xmlrpcParamVectorPushBackString( &call->params, node_uri );
  xmlrpcParamVectorPushBackString( &call->params, subscrition->parameter_key );

  // NB: Node release is done in handleApiCallAttempt()
  return enqueueMasterApiCallInternal(node, call);
}

int enqueueRequestTopic(CrosNode *node, int subidx, const char *host, int port)
{
  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterSubscriber() : Can't allocate memory\n");
    return -1;
  }

  call->provider_idx = subidx;
  call->method = CROS_API_REQUEST_TOPIC;

  SubscriberNode *sub = &node->subs[subidx];

  CrosNodeStatusUsr status;
  initCrosNodeStatus(&status);
  status.provider_idx = subidx;
  status.xmlrpc_host = host;
  status.xmlrpc_port = port;
  cRosNodeStatusCallback(&status, sub->context); // calls the subscriber-status application-defined callback function (if specified when creating the subscriber). Undocumented status callback?

  xmlrpcParamVectorPushBackString(&call->params, node->name );
  xmlrpcParamVectorPushBackString(&call->params, sub->topic_name );
  xmlrpcParamVectorPushBackArray(&call->params);
  XmlrpcParam* array_param = xmlrpcParamVectorAt(&call->params,2);
  xmlrpcParamArrayPushBackArray(array_param);
  xmlrpcParamArrayPushBackString(xmlrpcParamArrayGetParamAt(array_param,0),CROS_TRANSPORT_TCPROS_STRING);

  return enqueueSlaveApiCall(node, call, host, port);
}

void restartAdversing(CrosNode* n)
{
  int it;
  for(it = 0; it < n->n_pubs; it++)
  {
    if (n->pubs[it].topic_name == NULL)
      continue;

    enqueuePublisherAdvertise(n, it);
  }

  for(it = 0; it < n->n_subs; it++)
  {
    if (n->subs[it].topic_name == NULL)
      continue;

    enqueueSubscriberAdvertise(n, it);
  }

  for(it = 0; it < n->n_service_providers; it++)
  {
    if (n->service_providers[it].service_name == NULL)
      continue;

    enqueueServiceAdvertise(n, it);
  }
}

void initPublisherNode(PublisherNode *pub)
{
  pub->message_definition = NULL;
  pub->topic_name = NULL;
  pub->topic_type = NULL;
  pub->md5sum = NULL;
  pub->context = NULL;
  pub->tcpros_id_list[0] = -1; // Empty list of TcprosProcess indices
  pub->loop_period = -1; // Publication paused
  pub->wake_up_time = 0;
  cRosMessageQueueInit(&pub->msg_queue);
}

void initSubscriberNode(SubscriberNode *sub)
{
  sub->message_definition = NULL;
  sub->topic_name = NULL;
  sub->topic_type = NULL;
  sub->md5sum = NULL;
  sub->context = NULL;
  sub->tcp_nodelay = 0;
  sub->msg_queue_overflow = 0;
  cRosMessageQueueInit(&sub->msg_queue);
}

void initServiceProviderNode(ServiceProviderNode *srv_prov)
{
  srv_prov->service_name = NULL;
  srv_prov->service_type = NULL;
  srv_prov->md5sum = NULL;
  srv_prov->context = NULL;
  srv_prov->servicerequest_type = NULL;
  srv_prov->serviceresponse_type = NULL;
}

void initServiceCallerNode(ServiceCallerNode *srv_caller)
{
  srv_caller->service_name = NULL;
  srv_caller->service_type = NULL;
  srv_caller->service_host = NULL;
  srv_caller->service_port = -1;
  srv_caller->md5sum = NULL;
  srv_caller->message_definition = NULL;
  srv_caller->rpcros_id = -1;
  srv_caller->context = NULL;
  srv_caller->servicerequest_type = NULL;
  srv_caller->serviceresponse_type = NULL;
  srv_caller->persistent = 0;
  srv_caller->tcp_nodelay = 0;
  srv_caller->loop_period = -1; // Calling paused
  srv_caller->wake_up_time = 0;
  cRosMessageQueueInit(&srv_caller->msg_queue);
}

void initParameterSubscrition(ParameterSubscription *subscription)
{
  subscription->parameter_key = NULL;
  xmlrpcParamInit(&subscription->parameter_value);
  subscription->status_api_callback = NULL;
  subscription->context = NULL;
}

void cRosNodeReleasePublisher(PublisherNode *node)
{
  free(node->message_definition);
  free(node->topic_name);
  free(node->topic_type);
  free(node->md5sum);
  cRosMessageQueueRelease(&node->msg_queue);
}

void cRosNodeReleaseSubscriber(SubscriberNode *node)
{
  free(node->message_definition);
  free(node->topic_name);
  free(node->topic_type);
  free(node->md5sum);
  cRosMessageQueueRelease(&node->msg_queue);
}

void cRosNodeReleaseServiceProvider(ServiceProviderNode *node)
{
  free(node->service_name);
  free(node->service_type);
  free(node->servicerequest_type);
  free(node->serviceresponse_type);
  free(node->md5sum);
}

void cRosNodeReleaseServiceCaller(ServiceCallerNode *node)
{
  free(node->service_name);
  free(node->service_type);
  free(node->servicerequest_type);
  free(node->serviceresponse_type);
  free(node->md5sum);
  free(node->message_definition);
  free(node->service_host);
}

void initCrosNodeStatus(CrosNodeStatusUsr *status)
{
  status->state = CROS_STATUS_NONE;
  status->xmlrpc_port = -1;
  status->xmlrpc_host = NULL;
  status->provider_idx = -1;
  status->parameter_key = NULL;
  status->parameter_value = NULL;
}

void cRosNodeReleaseParameterSubscrition(ParameterSubscription *subscription)
{
  free(subscription->parameter_key);
  xmlrpcParamRelease(&subscription->parameter_value);
}

void getIdleXmplrpcClients(CrosNode *node, int idle_clients[], size_t *idle_client_count)
{
  int client_it = 1; // client_it = 0 is for roscore only
  int idle_it = 0;
  *idle_client_count = 0;
  for(; client_it < CN_MAX_XMLRPC_CLIENT_CONNECTIONS; client_it++)
  {
    if (node->xmlrpc_client_proc[client_it].state == XMLRPC_PROCESS_STATE_IDLE)
    {
      idle_clients[idle_it] = client_it;
      (*idle_client_count)++;
      idle_it++;
    }
  }
}

int enqueueMasterApiCall(CrosNode *node, RosApiCall *call)
{
  call->user_call = 1;
  return enqueueSlaveApiCallInternal(node, call);
}

int enqueueSlaveApiCall(CrosNode *node, RosApiCall *call, const char *host, int port)
{
  call->user_call = 1;
  if (host == NULL)
  {
    call->host = node->roscore_host;
    call->port = node->roscore_port;
  }
  else
  {
    call->host = (char *)malloc(strlen(host) + 1);
    if (call->host == NULL)
    {
      PRINT_ERROR("enqueueSlaveApiCall() : Not enough memory\n");
      return -1;
    }

    strcpy(call->host, host);
    call->port = port;
  }

  return enqueueSlaveApiCallInternal(node, call);
}

int enqueueMasterApiCallInternal(CrosNode *node, RosApiCall *call)
{
  int callid = (int)node->next_call_id;
  call->id = callid;
  int rc = enqueueApiCall(&node->master_api_queue, call);
  if (rc == -1)
    return -1;

  node->next_call_id++;
  return callid;
}

int enqueueSlaveApiCallInternal(CrosNode *node, RosApiCall *call)
{
  int callid = (int)node->next_call_id;
  call->id = callid;
  int rc = enqueueApiCall(&node->slave_api_queue, call);
  if (rc == -1)
    return -1;

  node->next_call_id++;
  return callid;
}

void cRosGetMsgFilePath(CrosNode *node, char *buffer, size_t bufsize, const char *topic_type)
{
  snprintf(buffer, bufsize, "%s/%s.msg", node->message_root_path, topic_type);
}

void getSrvFilePath(CrosNode *node, char *buffer, size_t bufsize, const char *service_type)
{
  snprintf(buffer, bufsize, "%s/%s.srv", node->message_root_path, service_type);
}

XmlrpcParam * cRosNodeGetParameterValue( CrosNode *node, const char *key)
{
  int it = 0;
  for (it = 0 ; it < node->n_paramsubs; it++)
  {
    if (node->paramsubs[it].parameter_key == NULL)
      continue;

    if (strcmp(node->paramsubs[it].parameter_key, key) == 0)
      return &node->paramsubs[it].parameter_value;
  }

  return NULL;
}

#define MAX_PORT_OPEN_CHECK_PERIOD 1000 //! Maximum time to wait in ms until the target port is checked again by cRosWaitPortOpen()
cRosErrCodePack cRosWaitPortOpen(const char *host_addr, unsigned short host_port, unsigned long time_out)
{
  uint64_t start_time, elapsed_time;
  cRosErrCodePack ret_err;
  TcpIpSocketState socket_state;
  PRINT_VVDEBUG ( "cRosWaitPortOpen ()\n" );

  start_time = cRosClockGetTimeMs();
  do
  {
    elapsed_time = cRosClockGetTimeMs()-start_time;
    socket_state = tcpIpSocketCheckPort (host_addr, host_port);
    // socket_state can be TCPIPSOCKET_FAILED, TCPIPSOCKET_DONE, TCPIPSOCKET_IN_PROGRESS or TCPIPSOCKET_REFUSED
    if(socket_state == TCPIPSOCKET_REFUSED)
    {
      unsigned int pause_ms;
      if(time_out == CROS_INFINITE_TIMEOUT || time_out-elapsed_time > MAX_PORT_OPEN_CHECK_PERIOD)
        pause_ms = MAX_PORT_OPEN_CHECK_PERIOD;
      else
        pause_ms = (unsigned int)(time_out-elapsed_time);
#ifdef _WIN32
      Sleep(pause_ms);
#else
      usleep(pause_ms*1000);
#endif
    }
  }
  while(socket_state == TCPIPSOCKET_REFUSED && (time_out == CROS_INFINITE_TIMEOUT || elapsed_time <= time_out));
  // We contnue iterating if the connection is refused and the time is not out
  if(socket_state == TCPIPSOCKET_FAILED)
    ret_err = CROS_SOCK_OPEN_CONN_ERR;
  else if(socket_state == TCPIPSOCKET_REFUSED)
    ret_err = CROS_SOCK_OPEN_TIMEOUT_ERR;
  else // socket_state == TCPIPSOCKET_DONE or TCPIPSOCKET_IN_PROGRESS
    ret_err = CROS_SUCCESS_ERR_PACK; // Port is ready

  return ret_err;
}

FILE *cRosOutStreamGet(void)
{
  return((Msg_output == NULL)?stdout:Msg_output);
}

void cRosOutStreamSet(FILE *new_stream)
{
  Msg_output = new_stream;
}


#include <stdio.h>
#ifndef __APPLE__
  #include <malloc.h>
#endif
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>
#include <ctype.h>

#include "cros_node.h"
#include "cros_api.h"
#include "cros_message.h"
#include "cros_clock.h"
#include "cros_defs.h"
#include "cros_node_api.h"
#include "cros_tcpros.h"
#include "cros_log.h"

static void initPublisherNode(PublisherNode *node);
static void initSubscriberNode(SubscriberNode *node);
static void initServiceProviderNode(ServiceProviderNode *node);
static void initParameterSubscrition(ParameterSubscription *subscription);
static void releasePublisherNode(PublisherNode *node);
static void releaseSubscriberNode(SubscriberNode *node);
static void releaseServiceProviderNode(ServiceProviderNode *node);
static void releaseParameterSubscrition(ParameterSubscription *subscription);
static int enqueueSubscriberAdvertise(CrosNode *node, int subidx);
static int enqueuePublisherAdvertise(CrosNode *node, int pubidx);
static int enqueueServiceAdvertise(CrosNode *node, int servivceidx);
static int enqueueParameterSubscription(CrosNode *node, int parameteridx);
static int enqueueParameterUnsubscription(CrosNode *node, int parameteridx);
static void getIdleXmplrpcClients(CrosNode *node, int array[], size_t *count);
static int enqueueSlaveApiCallInternal(CrosNode *node, RosApiCall *call);
static int enqueueMasterApiCallInternal(CrosNode *node, RosApiCall *call);

static void openXmlrpcClientSocket( CrosNode *n, int i )
{
  if( !tcpIpSocketOpen( &(n->xmlrpc_client_proc[i].socket) ) ||
      !tcpIpSocketSetReuse( &(n->xmlrpc_client_proc[i].socket) ) ||
      !tcpIpSocketSetNonBlocking( &(n->xmlrpc_client_proc[i].socket) ) )
  {
    PRINT_ERROR("openXmlrpcClientSocket() at index %d failed", i);
    exit( EXIT_FAILURE );
  }
}

static void openTcprosClientSocket( CrosNode *n, int i )
{
  if( !tcpIpSocketOpen( &(n->tcpros_client_proc[i].socket) ) ||
      !tcpIpSocketSetReuse( &(n->tcpros_client_proc[i].socket) ) ||
      !tcpIpSocketSetNonBlocking( &(n->tcpros_client_proc[i].socket) ) )
  {
    PRINT_ERROR("openTcprosClientSocket() at index %d failed", i);
    exit( EXIT_FAILURE );
  }
}

static void openXmlrpcListnerSocket( CrosNode *n )
{
  if( !tcpIpSocketOpen( &(n->xmlrpc_listner_proc.socket) ) ||
      !tcpIpSocketSetReuse( &(n->xmlrpc_listner_proc.socket) ) ||
      !tcpIpSocketSetNonBlocking( &(n->xmlrpc_listner_proc.socket) ) ||
      !tcpIpSocketBindListen( &(n->xmlrpc_listner_proc.socket), n->host, 0, CN_MAX_XMLRPC_SERVER_CONNECTIONS ) )
  {
    PRINT_ERROR("openXmlrpcListnerSocket() failed");
    exit( EXIT_FAILURE );
  }
  else
  {
    n->xmlrpc_port = tcpIpSocketGetPort( &(n->xmlrpc_listner_proc.socket) );
    PRINT_DEBUG ( "openXmlrpcListnerSocket () : Accepting xmlrpc connections at port %d\n", n->xmlrpc_port );
  }
}

static void openRpcrosListnerSocket( CrosNode *n )
{
  if( !tcpIpSocketOpen( &(n->rpcros_listner_proc.socket) ) ||
      !tcpIpSocketSetReuse( &(n->rpcros_listner_proc.socket) ) ||
      !tcpIpSocketSetNonBlocking( &(n->rpcros_listner_proc.socket) ) ||
      !tcpIpSocketBindListen( &(n->rpcros_listner_proc.socket), n->host, 0, CN_MAX_RPCROS_SERVER_CONNECTIONS ) )
  {
    PRINT_ERROR("openRpcrosListnerSocket() failed");
    exit( EXIT_FAILURE );
  }
  else
  {
    n->rpcros_port = tcpIpSocketGetPort( &(n->rpcros_listner_proc.socket) );
    PRINT_DEBUG ( "openRpcrosListnerSocket() : Accepting rcpros connections at port %d\n", n->rpcros_port );
  }
}

static void openTcprosListnerSocket( CrosNode *n )
{
  if( !tcpIpSocketOpen( &(n->tcpros_listner_proc.socket) ) ||
      !tcpIpSocketSetReuse( &(n->tcpros_listner_proc.socket) ) ||
      !tcpIpSocketSetNonBlocking( &(n->tcpros_listner_proc.socket) ) ||
      !tcpIpSocketBindListen( &(n->tcpros_listner_proc.socket), n->host, 0, CN_MAX_TCPROS_SERVER_CONNECTIONS ) )
  {
    PRINT_ERROR("openTcprosListnerSocket() failed");
    exit( EXIT_FAILURE );
  }
  else
  {
    n->tcpros_port = tcpIpSocketGetPort( &(n->tcpros_listner_proc.socket) );
    PRINT_DEBUG ( "openTcprosListnerSocket() : Accepting tcpros connections at port %d\n", n->tcpros_port );
  }
}

static void closeTcprosProcess(TcprosProcess *process)
{
  tcpIpSocketClose(&process->socket);
  tcprosProcessClear(process, 1);
  tcprosProcessChangeState(process, TCPROS_PROCESS_STATE_IDLE );
}

static void closeXmlrpcProcess(XmlrpcProcess *process)
{
  tcpIpSocketClose(&process->socket);
  xmlrpcProcessClear(process, 1);
  xmlrpcProcessChangeState(process, XMLRPC_PROCESS_STATE_IDLE);
}

// This method is used to communicate that some api calls at least attempted
// to complete, like in the case of unregistration when a gracefully shutdown
// is requested
static void handleApiCallAttempt(CrosNode *node, RosApiCall *call)
{
  switch(call->method)
  {
    case CROS_API_UNREGISTER_PUBLISHER:
    {
      if (call->provider_idx == -1)
        break;

      PublisherNode *pub = &node->pubs[call->provider_idx];
      NodeStatusCallback callback = pub->status_callback;
      if (callback != NULL)
      {
        CrosNodeStatusUsr status;
        initCrosNodeStatus(&status);
        callback(&status, pub->context);
      }

      // Finally release publisher
      releasePublisherNode(pub);
      initPublisherNode(pub);
      call->provider_idx = -1;
      break;
    }
    case CROS_API_UNREGISTER_SUBSCRIBER:
    {
      if (call->provider_idx == -1)
        break;

      SubscriberNode *sub = &node->subs[call->provider_idx];
      NodeStatusCallback callback = sub->status_callback;
      if (callback != NULL)
      {
        CrosNodeStatusUsr status;
        initCrosNodeStatus(&status);
        callback(&status, sub->context);
      }

      // Finally release subscriber
      releaseSubscriberNode(sub);
      initSubscriberNode(sub);
      call->provider_idx = -1;
      break;
    }
    case CROS_API_UNREGISTER_SERVICE:
    {
      if (call->provider_idx == -1)
        break;

      ServiceProviderNode *service = &node->services[call->provider_idx];
      NodeStatusCallback callback = service->status_callback;
      if (callback != NULL)
      {
        CrosNodeStatusUsr status;
        initCrosNodeStatus(&status);
        callback(&status, service->context);
      }

      // Finally release service provider
      releaseServiceProviderNode(service);
      initServiceProviderNode(service);
      call->provider_idx = -1;
      break;
    }
    case CROS_API_UNSUBSCRIBE_PARAM:
    {
      if (call->provider_idx == -1)
        break;

      ParameterSubscription *subscription = &node->paramsubs[call->provider_idx];
      NodeStatusCallback callback = subscription->status_callback;
      if (callback != NULL)
      {
        CrosNodeStatusUsr status;
        initCrosNodeStatus(&status);
        status.state = CROS_STATUS_PARAM_UNSUBSCRIBED;
        status.provider_idx = call->provider_idx;
        status.parameter_key = subscription->parameter_key;
        callback(&status, subscription->context);
      }

      // Finally release parameter subscription
      releaseParameterSubscrition(subscription);
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
      // This is needed to clean transitory xmlrpc client process that is set on
      // the subscriber
      node->subs[call->provider_idx].client_xmlrpc_id = -1;
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

static void handleTcprosServerError(CrosNode *n, int i)
{
  TcprosProcess *process = &n->tcpros_server_proc[i];
  PublisherNode *pub = &n->pubs[process->topic_idx];
  pub->client_tcpros_id = -1;
  closeTcprosProcess(process);
}

static void handleRpcrosServerError(CrosNode *n, int i)
{
  TcprosProcess *process = &n->rpcros_server_proc[i];
  closeTcprosProcess(process);
}

static void doWithXmlrpcClientSocket(CrosNode *n, int i)
{
  PRINT_VDEBUG ( "doWithXmlrpcClientSocket()\n" );

  XmlrpcProcess *xmlrpc_client_proc = &(n->xmlrpc_client_proc[i]);

  if( xmlrpc_client_proc->state == XMLRPC_PROCESS_STATE_WRITING )
  {
    PRINT_DEBUG ( "doWithXmlrpcClientSocket() : writing\n" );

    if( !xmlrpc_client_proc->socket.connected )
    {

      TcpIpSocketState conn_state = TCPIPSOCKET_FAILED;

      RosApiCall *call = xmlrpc_client_proc->current_call;
      assert(call != NULL);
      if (i == 0 || isRosMasterApi(call->method))
      {
    	  conn_state = tcpIpSocketConnect( &(xmlrpc_client_proc->socket),
                                       	   n->roscore_host, n->roscore_port );
      }
      else // Is slave api
      {
        if (call->provider_idx == -1)
        {
          conn_state = tcpIpSocketConnect(&xmlrpc_client_proc->socket,
                                          call->host, call->port);
        }
        else // It's client xmlrpc to be invoked from a subscriber
        {
          SubscriberNode *sub = &n->subs[call->provider_idx];
          conn_state = tcpIpSocketConnect(&xmlrpc_client_proc->socket,
                                          sub->topic_host, sub->topic_port);
        }
      }

      if( conn_state == TCPIPSOCKET_IN_PROGRESS )
      {
        PRINT_DEBUG ( "doWithXmlrpcClientSocket() : Wait: connection is established asynchronously\n" );
        // Wait: connection is established asynchronously
        return;
      }
      else if( conn_state == TCPIPSOCKET_FAILED )
      {
        PRINT_ERROR("doWithXmlrpcClientSocket() : Can't connect\n" );
        handleXmlrpcClientError( n, i);
        return;
      }
    }

    cRosApiPrepareRequest( n, i );

    TcpIpSocketState sock_state =  tcpIpSocketWriteString( &(xmlrpc_client_proc->socket), 
                                                           &(xmlrpc_client_proc->message) );
    switch ( sock_state )
    {
      case TCPIPSOCKET_DONE:
        {
        handleApiCallAttempt(n, xmlrpc_client_proc->current_call);
        xmlrpcProcessClear(xmlrpc_client_proc, 0);
        xmlrpcProcessChangeState( xmlrpc_client_proc, XMLRPC_PROCESS_STATE_READING );
        break;
        }

      case TCPIPSOCKET_IN_PROGRESS:
		{
        PRINT_DEBUG ( "doWithXmlrpcClientSocket() : Write in progress...\n" );
        break;
        }

      case TCPIPSOCKET_DISCONNECTED:
      case TCPIPSOCKET_FAILED:
      default:
        {
        PRINT_ERROR("doWithXmlrpcClientSocket() : Unexpected failure writing request\n");
        handleXmlrpcClientError( n, i );
        break;
        }
    }
  }
  else if( xmlrpc_client_proc->state == XMLRPC_PROCESS_STATE_READING )
  {
    PRINT_DEBUG ( "doWithXmlrpcClientSocket() : Reading()\n" );
    TcpIpSocketState sock_state = tcpIpSocketReadString( &xmlrpc_client_proc->socket,
                                                         &xmlrpc_client_proc->message );
    //printf("%s\n", xmlrpc_client_proc->message.data);
    XmlrpcParserState parser_state = XMLRPC_PARSER_INCOMPLETE;

    int disconnected = 0;
    switch ( sock_state )
    {
      case TCPIPSOCKET_DONE:
        {
        parser_state = parseXmlrpcMessage( &xmlrpc_client_proc->message,
                                           &xmlrpc_client_proc->message_type,
                                           NULL,
                                           &xmlrpc_client_proc->response,
                                           xmlrpc_client_proc->host,
                                           &xmlrpc_client_proc->port);
        break;
        }

      case TCPIPSOCKET_IN_PROGRESS:
        break;

      case TCPIPSOCKET_DISCONNECTED:
        {
        parser_state = parseXmlrpcMessage( &xmlrpc_client_proc->message,
                                           &xmlrpc_client_proc->message_type,
                                           NULL,
                                           &xmlrpc_client_proc->response,
                                           xmlrpc_client_proc->host,
                                           &xmlrpc_client_proc->port);
        disconnected = 1;
        break;
    }

      case TCPIPSOCKET_FAILED:
      default:
        {
        PRINT_ERROR("doWithXmlrpcClientSocket() : Unexpected failure reading response\n" );
        handleXmlrpcClientError( n, i );
        break;
        }
    }

    switch ( parser_state )
    {
      case XMLRPC_PARSER_DONE:
{
        PRINT_DEBUG ( "doWithXmlrpcClientSocket() : Done with no error\n" );

        int rc = cRosApiParseResponse( n, i );
        if (rc != 0)
        {
          handleXmlrpcClientError( n, i );
        }
        else
        {
          cleanApiCallState(n, xmlrpc_client_proc->current_call);
          closeXmlrpcProcess(xmlrpc_client_proc);
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
  }
}

static void doWithXmlrpcServerSocket( CrosNode *n, int i )
{
  PRINT_VDEBUG ( "doWithXmlrpcServerSocket()\n" );

  XmlrpcProcess *server_proc = &(n->xmlrpc_server_proc[i]);

  if( server_proc->state == XMLRPC_PROCESS_STATE_READING )
  {
    PRINT_DEBUG ( "doWithXmlrpcServerSocket() : Reading() index %d \n", i );
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
        xmlrpcProcessClear( &(n->xmlrpc_server_proc[i]), 1);
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

        PRINT_DEBUG ( "doWithXmlrpcServerSocket() : Done read() and parse() with no error\n" );
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
    PRINT_DEBUG ( "doWithXmlrpcServerSocket() : writing() index %d \n", i );
    TcpIpSocketState sock_state =  tcpIpSocketWriteString( &(server_proc->socket), 
                                                           &(server_proc->message) );
    switch ( sock_state )
    {
      case TCPIPSOCKET_DONE:
        xmlrpcProcessClear( server_proc, 1);
        xmlrpcProcessChangeState( server_proc, XMLRPC_PROCESS_STATE_READING );
        break;

      case TCPIPSOCKET_IN_PROGRESS:
        break;

      case TCPIPSOCKET_DISCONNECTED:
        xmlrpcProcessClear( server_proc, 1);
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
}

static void doWithTcprosClientSocket( CrosNode *n, int client_idx)
{
  PRINT_VDEBUG ( "doWithTcprosSubscriberNode()\n" );

  TcprosProcess *client_proc = &(n->tcpros_client_proc[client_idx]);

  switch ( client_proc->state )
  {
    case  TCPROS_PROCESS_STATE_CONNECTING:
    {
      SubscriberNode *sub = &(n->subs[client_proc->topic_idx]);
      tcprosProcessClear( client_proc, 0 );
      TcpIpSocketState conn_state = tcpIpSocketConnect( &(client_proc->socket),
                                           sub->topic_host, sub->tcpros_port );
      switch (conn_state)
      {
        case TCPIPSOCKET_DONE:
        {
          tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_WRITING_HEADER );
          break;
        }
        case TCPIPSOCKET_IN_PROGRESS:
        {
          PRINT_DEBUG ( "doWithXmlrpcClientSocket() : Wait: connection is established asynchronously\n" );
          // Wait: connection is established asynchronously
          break;
        }
        case TCPIPSOCKET_FAILED:
        {
          PRINT_DEBUG ( "doWithXmlrpcClientSocket() : error\n" );
          handleTcprosClientError( n, client_idx);
          break;
        }
        default:
        {
          assert(0);
          break;
        }
      }

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
          tcprosProcessClear( client_proc, 0);
          client_proc->left_to_recv = sizeof(uint32_t);
          tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_READING_HEADER_SIZE );

          break;

        case TCPIPSOCKET_IN_PROGRESS:
          break;

        case TCPIPSOCKET_DISCONNECTED:
          tcprosProcessClear( client_proc, 1);
          tcprosProcessChangeState( client_proc, TCPROS_PROCESS_STATE_IDLE );
          tcpIpSocketClose( &(client_proc->socket) );
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
            ROS_TO_HOST_UINT32(*((uint32_t *)data), header_size);
            tcprosProcessClear( client_proc, 0 );
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
          tcprosProcessClear( client_proc, 0);
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
            ROS_TO_HOST_UINT32(*((uint32_t *)data), msg_size);
            tcprosProcessClear( client_proc, 0);
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
              cRosMessageParsePublicationPacket(n, client_idx);
              tcprosProcessClear( client_proc, 0);
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
      // Invalid flow
      assert(0);
    }

  }
}

static void doWithTcprosServerSocket( CrosNode *n, int i )
{
  PRINT_VDEBUG ( "doWithTcprosServerSocket()\n" );
  
  TcprosProcess *server_proc = &(n->tcpros_server_proc[i]);

  if( server_proc->state == TCPROS_PROCESS_STATE_READING_HEADER)
  {
    PRINT_DEBUG ( "doWithTcprosServerSocket() : Reading header index %d \n", i );
    tcprosProcessClear( server_proc, 0);
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
        PRINT_INFO( "doWithTcprosServerSocket() : Client disconnected\n" );
        handleTcprosServerError( n, i );
        break;
    }
    
    switch ( parser_state )
    {
      case TCPROS_PARSER_DONE:
                               
        PRINT_DEBUG ( "doWithTcprosServerSocket() : Done read() and parse() with no error\n" );
        tcprosProcessClear( server_proc, 0);
        cRosMessagePreparePublicationHeader( n, i );
        tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_WRITING );
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
           server_proc->state == TCPROS_PROCESS_STATE_WRITING )
  {
    PRINT_DEBUG ( "doWithTcprosServerSocket() : writing() index %d \n", i );
    if( server_proc->state == TCPROS_PROCESS_STATE_START_WRITING )
    {
      tcprosProcessClear( server_proc, 0 );
      cRosMessagePreparePublicationPacket( n, i );
      tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_WRITING );      
    }
    TcpIpSocketState sock_state =  tcpIpSocketWriteBuffer( &(server_proc->socket), 
                                                           &(server_proc->packet) );
    
    switch ( sock_state )
    {
      case TCPIPSOCKET_DONE:
        PRINT_DEBUG ( "doWithTcprosServerSocket() : Done write() with no error\n" );
        tcprosProcessClear( server_proc, 0);
        tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_WAIT_FOR_WRITING );
        break;

      case TCPIPSOCKET_IN_PROGRESS:
        break;

      case TCPIPSOCKET_DISCONNECTED:
      case TCPIPSOCKET_FAILED:
      default:
        PRINT_INFO( "doWithTcprosServerSocket() : Client disconnected\n" );
        handleTcprosServerError(n, i);
        break;
    }
  }
}

static void doWithRpcrosServerSocket(CrosNode *n, int i)
{
  PRINT_VDEBUG ( "doWithRpcrosServerSocket()\n" );

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
            ROS_TO_HOST_UINT32(*((uint32_t *)data), header_size);
            tcprosProcessClear( server_proc, 0 );
            server_proc->left_to_recv = header_size;
            tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_READING_HEADER);
            goto read_header;
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
          tcprosProcessClear( server_proc, 0);
          tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_WRITING_HEADER );
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
      PRINT_DEBUG ( "doWithRpcrosServerSocket() : writing() index %d \n", i );

      cRosMessagePrepareServiceProviderHeader( n, i );

      TcpIpSocketState sock_state =  tcpIpSocketWriteBuffer( &(server_proc->socket),
                                                            &(server_proc->packet) );

      switch ( sock_state )
      {
        case TCPIPSOCKET_DONE:
          PRINT_DEBUG ( "doWithRpcrosServerSocket() : Done write() with no error\n" );
          if(server_proc->probe)
          {
            tcprosProcessClear( server_proc, 1 );
            tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_IDLE );
          }
          else
          {
            tcprosProcessClear( server_proc, 0 );
            server_proc->left_to_recv = sizeof(uint32_t);
            tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_READING_SIZE );
          }
          break;

        case TCPIPSOCKET_IN_PROGRESS:
          break;

        case TCPIPSOCKET_DISCONNECTED:
          tcprosProcessClear( server_proc , 1);
          tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_IDLE );
          tcpIpSocketClose( &(server_proc->socket) );
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
            const unsigned char *data = dynBufferGetCurrentData(&server_proc->packet);
            uint32_t msg_size = (uint32_t)*data;
            tcprosProcessClear( server_proc, 0);
            if (msg_size == 0)
            {
              PRINT_DEBUG ( "doWithRpcrosServerSocket() : Done read() with no error\n" );
              cRosMessagePrepareServiceResponsePacket(n, i);
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
      PRINT_DEBUG ( "doWithRpcrosServerSocket() : reading() index %d \n", i );

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
              PRINT_DEBUG ( "doWithRpcrosServerSocket() : Done read() with no error\n" );
              cRosMessagePrepareServiceResponsePacket(n, i);
              tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_WRITING );
          }
          break;
        case TCPIPSOCKET_IN_PROGRESS:
          break;
        case TCPIPSOCKET_DISCONNECTED:
          tcprosProcessClear( server_proc, 1 );
          tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_IDLE );
          tcpIpSocketClose( &(server_proc->socket) );
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
      PRINT_DEBUG ( "doWithRpcrosServerSocket() : writing() index %d \n", i );

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
          PRINT_DEBUG ( "doWithRpcrosServerSocket() : Done write() with no error\n" );
          tcprosProcessClear( server_proc, 1 );
          tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_IDLE);
          break;

        case TCPIPSOCKET_IN_PROGRESS:
          break;

        case TCPIPSOCKET_DISCONNECTED:
          tcprosProcessClear( server_proc, 1 );
          tcprosProcessChangeState( server_proc, TCPROS_PROCESS_STATE_IDLE );
          tcpIpSocketClose( &(server_proc->socket) );
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
      // Invalid flow
      assert(0);
    }
  }
}

/*
 * Services, publisher and support functions for the logging feature
 */
static CrosLogLevel stringToLogLevel(const char* level_str)
{
  if(strncmp("Info",level_str, strlen(level_str)) == 0)
  {
    return CROS_LOGLEVEL_INFO;
  }
  else if(strncmp("Debug",level_str, strlen(level_str)) == 0)
  {
    return CROS_LOGLEVEL_DEBUG;
  }
  else if(strncmp("Warn",level_str, strlen(level_str)) == 0)
  {
    return CROS_LOGLEVEL_WARN;
  }
  else if(strncmp("Error",level_str, strlen(level_str)) == 0)
  {
    return CROS_LOGLEVEL_ERROR;
  }
  else // Fatal
  {
    return CROS_LOGLEVEL_FATAL;
  }
}

static char* LogLevelToString(CrosLogLevel log_level)
{
  char* ret = NULL;

  switch(log_level)
  {
    case CROS_LOGLEVEL_INFO:
    {
      ret = calloc(strlen("INFO") + 1, sizeof(char));
      strncpy(ret, "INFO",strlen("INFO"));
      return ret;
    }
    case CROS_LOGLEVEL_DEBUG:
    {
      ret = calloc(strlen("DEBUG") + 1, sizeof(char));
      strncpy(ret, "DEBUG",strlen("DEBUG"));
      return ret;
    }
    case CROS_LOGLEVEL_WARN:
    {
      ret = calloc(strlen("WARN") + 1, sizeof(char));
      strncpy(ret, "WARN",strlen("WARN"));
      return  ret;
    }
    case CROS_LOGLEVEL_ERROR:
    {
      ret = calloc(strlen("ERROR") + 1, sizeof(char));
      strncpy(ret, "ERROR",strlen("ERROR"));
      return ret;
    }
    case CROS_LOGLEVEL_FATAL:
    {
      ret = calloc(strlen("FATAL") + 1, sizeof(char));
      strncpy(ret, "FATAL",strlen("FATAL"));
      return ret;
    }
    default:
    {
      assert(0);
    }
  }
}

static CallbackResponse callback_pub_log(cRosMessage *message, void* data_context)
{
  CrosNode* node = (CrosNode*) data_context;
  CrosLogQueue* queue = node->log_queue;

  if(!cRosLogQueueIsEmpty(queue))
  {
    CrosLog* log = cRosLogQueueDequeue(queue);

    cRosMessageField* header_field = cRosMessageGetField(message, "header");
    cRosMessage* header_msg = header_field->data.as_msg;
    cRosMessageField* seq_id = cRosMessageGetField(header_msg, "seq");
    seq_id->data.as_uint32 = node->log_last_id++;
    cRosMessageField* time_field = cRosMessageGetField(header_msg, "stamp");
    cRosMessage* time_msg = time_field->data.as_msg;
    cRosMessageField* time_secs = cRosMessageGetField(time_msg, "secs");
    time_secs->data.as_uint32 = log->secs;
    cRosMessageField* time_nsecs = cRosMessageGetField(time_msg, "nsecs");
    time_nsecs->data.as_uint32 = log->nsecs;
    cRosMessageSetFieldValueString(cRosMessageGetField(header_msg, "frame_id"), "0");


    cRosMessageField* level = cRosMessageGetField(message, "level");
    level->data.as_uint8 = log->level;

    cRosMessageField* name = cRosMessageGetField(message, "name"); //name of the node
    cRosMessageSetFieldValueString(name, node->name);

    cRosMessageField* msg = cRosMessageGetField(message, "msg"); //message
    cRosMessageSetFieldValueString(msg, log->msg);

    cRosMessageField* file = cRosMessageGetField(message, "file"); //file the message came from
    cRosMessageSetFieldValueString(file, log->file);

    cRosMessageField* function = cRosMessageGetField(message, "function"); //function the message came from
    cRosMessageSetFieldValueString(function, log->function);

    cRosMessageField* line = cRosMessageGetField(message, "line"); //line the message came from
    line->data.as_uint32 = log->line;

    cRosMessageField* topics = cRosMessageGetField(message, "topics"); //topic names that the node publishes
    int i;
    for(i = 0; i < log->n_pubs; i++)
    {
      cRosMessageFieldArrayPushBackString(topics, log->pubs[i]);
    }

    cRosLogFree(log);
  }
  return 0;
}

static CallbackResponse callback_srv_set_logger_level(cRosMessage *request, cRosMessage *response, void* context)
{
  cRosMessageField* level = cRosMessageGetField(request, "level");
  const char* level_str = level->data.as_string;
  CrosNode* node = (CrosNode*) context;
  node->log_level = stringToLogLevel(level_str);
  return 0;
}

static CallbackResponse callback_srv_get_loggers(cRosMessage *request, cRosMessage *response, void* context)
{
  cRosMessageField* loggers = cRosMessageGetField(response, "loggers");

  cRosMessage* logger_msg = cRosMessageNew();

  CrosNode* node = (CrosNode*) context;
  char path[256];

  cRosGetMsgFilePath(node, path, 256, "roscpp/Logger");
  cRosMessageBuild(logger_msg,path);

  cRosMessageField* logger = cRosMessageGetField(logger_msg, "name");
  cRosMessageSetFieldValueString(logger, "ros.cros_node");
  cRosMessageField* level = cRosMessageGetField(logger_msg, "level");
  char* str_level = LogLevelToString(node->log_level);
  cRosMessageSetFieldValueString(level, str_level);
  free(str_level);

  cRosMessageFieldArrayPushBackMsg(loggers,logger_msg);
  return 0;
}

int checkNamespaceFormat(const char* namespace)
{
  const char* it_ns = namespace;

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
  char* resolved_name = NULL;

  if(!checkNamespaceFormat(resource_name))
    return NULL;

  if(node == NULL)
  {
    if(resource_name[0] == '/')
    {
      resolved_name = calloc(strlen(resource_name) + 1, sizeof(char));
      strncat(resolved_name, resource_name,strlen(resource_name));
    }
    else
    {
      resolved_name = calloc(strlen(resource_name) + strlen("/") + 1, sizeof(char));
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
         resolved_name = calloc(strlen(resource_name) + 1,
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
          resolved_name = calloc(strlen(node_name) +
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
        //the resource has a name that is not global or private
        if(node != NULL || 1)
        {
          char* node_namespace = calloc(strlen(node_name) + 1, sizeof(char));
          strcpy(node_namespace, node_name);
          char* it = node_namespace + strlen(node_name);
          while(*(it--) != '/');
          *(it + 2) = '\0';
          resolved_name = calloc(strlen(node_namespace) + strlen(resource_name) + 1,sizeof(char));
          strncat(resolved_name,node_namespace,strlen(node_namespace));
          strncat(resolved_name,resource_name,strlen(resource_name));
        }
        break;
      }
    }
  }
  return resolved_name;
}

CrosNode *cRosNodeCreate (char* node_name, char *node_host, char *roscore_host, unsigned short roscore_port,
                          char *message_root_path, uint64_t const *select_timeout_ms)
{
  PRINT_VDEBUG ( "cRosNodeCreate()\n" );

  signal(SIGPIPE, SIG_IGN);
  
  if(node_name == NULL || node_host == NULL || roscore_host == NULL )
  {
    PRINT_ERROR ( "cRosNodeCreate() : NULL parameters\n" );
    return NULL;
  }

  CrosNode *new_n = ( CrosNode * ) malloc ( sizeof ( CrosNode ) );

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

  int i;
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

  for ( i = 0; i < CN_MAX_PUBLISHED_TOPICS; i++)
    initPublisherNode(&new_n->pubs[i]);
  new_n->n_pubs = 0;

  for ( i = 0; i < CN_MAX_SUBSCRIBED_TOPICS; i++)
    initSubscriberNode(&new_n->subs[i]);
  new_n->n_subs = 0;

  for ( i = 0; i < CN_MAX_SERVICE_PROVIDERS; i++)
    initServiceProviderNode(&new_n->services[i]);
  new_n->n_services = 0;

  for ( i = 0; i < CN_MAX_PARAMETER_SUBSCRIPTIONS; i++)
    initParameterSubscrition(&new_n->paramsubs[i]);
  new_n->n_paramsubs = 0;

  if (select_timeout_ms == NULL)
    new_n->select_timeout = UINT64_MAX;
  else
    new_n->select_timeout = *select_timeout_ms;
  new_n->pid = (int)getpid();

  for(i = 0; i < CN_MAX_XMLRPC_CLIENT_CONNECTIONS; i++)
  {
    openXmlrpcClientSocket( new_n, i );
  }

  for(i = 0; i < CN_MAX_TCPROS_CLIENT_CONNECTIONS; i++)
  {
    openTcprosClientSocket( new_n, i );
  }

  openXmlrpcListnerSocket( new_n );
  openTcprosListnerSocket( new_n );
  openRpcrosListnerSocket( new_n );


  new_n->log_queue = cRosLogQueueNew();
  new_n-> log_last_id = 0;

  /*
   * Registering logging callback
   */

  int rc = 0;

  rc = cRosApiRegisterPublisher(new_n,"/rosout","rosgraph_msgs/Log", 100,
                                  callback_pub_log, NULL, new_n);
  if (rc == -1)
  {
    PRINT_ERROR ( "cRosNodeCreate(): Error registering rosout\n" );
  }

  rc = cRosApiRegisterServiceProvider(new_n,"~get_loggers","roscpp/GetLoggers",
                                      callback_srv_get_loggers, NULL, (void*) new_n);
  if (rc == -1)
  {
    PRINT_ERROR ( "cRosNodeCreate(): Error registering loggers\n" );
  }

  rc = cRosApiRegisterServiceProvider(new_n,"~set_logger_level","roscpp/SetLoggerLevel",
                                      callback_srv_set_logger_level, NULL, (void*) new_n);
  if (rc == -1)
  {
    PRINT_ERROR ( "cRosNodeCreate(): Error registering loggers\n" );
  }

  //current_node = new_n;

  return new_n;
}

void cRosNodeDestroy ( CrosNode *n )
{
  PRINT_VDEBUG ( "cRosNodeDestroy()\n" );

  if ( n == NULL )
    return;

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
    tcprosProcessRelease( &(n->tcpros_server_proc[i]) ); 

  for ( i = 0; i < CN_MAX_RPCROS_SERVER_CONNECTIONS; i++)
    tcprosProcessRelease( &(n->rpcros_server_proc[i]) );

  if ( n->name != NULL ) free ( n->name );
  if ( n->host != NULL ) free ( n->host );
  if ( n->roscore_host != NULL ) free ( n->roscore_host );

  for ( i = 0; i < CN_MAX_PUBLISHED_TOPICS; i++)
    releasePublisherNode(&n->pubs[i]);

  for ( i = 0; i < CN_MAX_SUBSCRIBED_TOPICS; i++)
    releaseSubscriberNode(&n->subs[i]);

  for ( i = 0; i < CN_MAX_SERVICE_PROVIDERS; i++)
    releaseServiceProviderNode(&n->services[i]);

  for ( i = 0; i < CN_MAX_PARAMETER_SUBSCRIPTIONS; i++)
    releaseParameterSubscrition(&n->paramsubs[i]);
}

int cRosNodeRegisterPublisher (CrosNode *node, const char *message_definition,
                               const char *topic_name, const char *topic_type, const char *md5sum, int loop_period,
                               PublisherCallback callback, NodeStatusCallback status_callback, void *data_context)
{
  PRINT_VDEBUG ( "cRosNodeRegisterPublisher()\n" );

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
  pub->callback = callback;
  pub->status_callback = status_callback;
  pub->context = data_context;

  node->n_pubs++;

  int rc = enqueuePublisherAdvertise(node, pubidx);
  if (rc == -1)
    return -1;

  return pubidx;
}

int cRosNodeRegisterServiceProvider(CrosNode *node, const char *service_name,
                                    const char *service_type, const char *md5sum,
                                    ServiceProviderCallback callback, NodeStatusCallback status_callback,
                                    void *data_context)
{
  PRINT_VDEBUG ( "cRosNodeRegisterServiceProvider()\n" );

  if (node->n_services >= CN_MAX_SERVICE_PROVIDERS)
  {
    PRINT_ERROR ( "cRosNodeRegisterPublisher() : Can't register a new service provider: \
                 reached the maximum number of services\n");
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

  PRINT_INFO ( "Registering service %s type %s \n", srv_service_name, srv_service_type);

  int serviceidx = -1;
  int it = 0;
  for (; it < CN_MAX_SERVICE_PROVIDERS; it++)
  {
    if (node->services[it].service_name == NULL)
    {
      serviceidx = it;
      break;
    }
  }

  ServiceProviderNode *service = &(node->services[serviceidx]);

  service->service_name = srv_service_name;
  service->service_type = srv_service_type;
  service->servicerequest_type = srv_servicerequest_type;
  service->serviceresponse_type = srv_serviceresponse_type;
  service->md5sum = srv_md5sum;
  service->callback = callback;
  service->status_callback = status_callback;
  service->context = data_context;

  node->n_services++;

  int rc = enqueueServiceAdvertise(node, serviceidx);
  if (rc == -1)
    return -1;

  return serviceidx;
}

int cRosNodeRegisterSubscriber(CrosNode *node, const char *message_definition,
                               const char *topic_name, const char *topic_type, const char *md5sum,
                               SubscriberCallback callback, NodeStatusCallback status_callback, void *data_context)
{
  PRINT_VDEBUG ( "cRosNodeRegisterSubscriber()\n" );

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
  sub->status_callback = status_callback;
  sub->callback = callback;
  sub->context = data_context;

  int clientidx = subidx + 1;
  sub->client_tcpros_id = clientidx;

  TcprosProcess *client_proc = &node->tcpros_client_proc[clientidx];
  client_proc->topic_idx = subidx;

  node->n_subs++;

  int rc = enqueueSubscriberAdvertise(node, subidx);
  if (rc == -1)
    return -1;

  return subidx;
}

int cRosNodeUnregisterSubscriber(CrosNode *node, int subidx)
{
  if (subidx < 0 || subidx >= CN_MAX_SUBSCRIBED_TOPICS)
    return -1;

  SubscriberNode *sub = &node->subs[subidx];
  if (sub->topic_name == NULL)
    return -1;

  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterPublisher() : Can't allocate memory\n");
    return -1;
  }

  TcprosProcess *tcprosProc = &node->tcpros_client_proc[sub->client_tcpros_id];
  closeTcprosProcess(tcprosProc);
  if (sub->client_xmlrpc_id != -1)
  {
    XmlrpcProcess *xmlrpcProc = &node->xmlrpc_client_proc[sub->client_xmlrpc_id];
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

  return enqueueMasterApiCallInternal(node, call);
}

int cRosNodeUnregisterPublisher(CrosNode *node, int pubidx)
{
  if (pubidx < 0 || pubidx >= CN_MAX_PUBLISHED_TOPICS)
    return -1;

  PublisherNode *pub = &node->pubs[pubidx];
  if (pub->topic_name == NULL)
    return -1;

  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterPublisher() : Can't allocate memory\n");
    return -1;
  }

  TcprosProcess *tcprosProc = &node->tcpros_server_proc[pub->client_tcpros_id];
  closeTcprosProcess(tcprosProc);

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

  return enqueueMasterApiCallInternal(node, call);
}

int cRosNodeUnregisterService(CrosNode *node, int serviceidx)
{
  if (serviceidx < 0 || serviceidx >= CN_MAX_SERVICE_PROVIDERS)
    return -1;

  ServiceProviderNode *svc = &node->services[serviceidx];
  if (svc->service_name == NULL)
    return -1;

  RosApiCall *call = newRosApiCall();
  if (call == NULL)
  {
    PRINT_ERROR ( "cRosNodeRegisterPublisher() : Can't allocate memory\n");
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

  xmlrpcParamVectorPushBackString( &call->params, node->name);
  xmlrpcParamVectorPushBackString( &call->params, svc->service_name);
  char uri[256];
  snprintf( uri, 256, "rosrpc://%s:%d/", node->host, node->rpcros_port);
  xmlrpcParamVectorPushBackString( &call->params, uri );
  snprintf( uri, 256, "http://%s:%d/", node->host, node->xmlrpc_port);
  xmlrpcParamVectorPushBackString( &call->params, uri );

  // NB: Node release is done in handleApiCallAttempt()

  return enqueueMasterApiCallInternal(node, call);
}

int cRosApiSubscribeParam(CrosNode *node, const char *key, NodeStatusCallback callback, void *context)
{
  PRINT_VDEBUG ( "cRosApiSubscribeParam()\n" );
  PRINT_INFO ( "Subscribing to parameter %s\n", key);

  if (node->n_paramsubs >= CN_MAX_PARAMETER_SUBSCRIPTIONS)
  {
    PRINT_ERROR ( "cRosApiSubscribeParam() : Can't register a new parameter subscription: \
                  reached the maximum number of parameters\n");
    return -1;
  }

  char *parameter_key = ( char * ) malloc ( ( strlen ( key ) + 1 ) * sizeof ( char ) );
  if (parameter_key == NULL)
  {
    PRINT_ERROR ( "cRosApiUnsubscribeParam() : Can't allocate memory\n" );
    return -1;
  }

  strcpy (parameter_key, key);

  int paramsubidx;
  int it = 0;
  for (; it < CN_MAX_PARAMETER_SUBSCRIPTIONS; it++)
  {
    if (node->subs[it].topic_name == NULL)
    {
      paramsubidx = it;
      break;
    }
  }

  ParameterSubscription *sub = &node->paramsubs[paramsubidx];
  sub->parameter_key = parameter_key;
  sub->context = context;
  sub->status_callback = callback;

  node->n_paramsubs++;

  int rc = enqueueParameterSubscription(node, paramsubidx);
  if (rc == -1)
    return -1;

  return 0;
}

int cRosApiUnsubscribeParam(CrosNode *node, int paramsubidx)
{
  if (paramsubidx < 0 || paramsubidx >= CN_MAX_PARAMETER_SUBSCRIPTIONS)
    return -1;

  ParameterSubscription *sub = &node->paramsubs[paramsubidx];
  if (sub->parameter_key == NULL)
    return -1;

  XmlrpcProcess *coreproc = &node->xmlrpc_client_proc[0];
  if (coreproc->current_call != NULL
      && coreproc->current_call->method == CROS_API_SUBSCRIBE_PARAM
      && coreproc->current_call->provider_idx == paramsubidx)
  {
    // Delist current registration
    closeXmlrpcProcess(coreproc);
  }

  int rc = enqueueParameterUnsubscription(node, paramsubidx);
  if (rc == -1)
    return -1;

  return 0;
}

void cRosNodeDoEventsLoop ( CrosNode *n )
{
  PRINT_VDEBUG ( "cRosNodeDoEventsLoop ()\n" );

  int nfds = -1;
  fd_set r_fds, w_fds, err_fds;
  int i = 0;

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
    xmlrpcProcessChangeState( coreproc, XMLRPC_PROCESS_STATE_WRITING );
  }

  size_t idle_client_count;
  int idle_clients[CN_MAX_XMLRPC_CLIENT_CONNECTIONS];
  getIdleXmplrpcClients(n, idle_clients, &idle_client_count);

  int next_idle_client_idx = 0;
  while (!isQueueEmpty(&n->slave_api_queue) && next_idle_client_idx != idle_client_count)
  {
    int idle_client_idx = idle_clients[next_idle_client_idx];
    RosApiCall *call = dequeueApiCall(&n->slave_api_queue);
    if (call->method == CROS_API_REQUEST_TOPIC)
      n->subs[call->provider_idx].client_xmlrpc_id = idle_client_idx;

    XmlrpcProcess *proc =  &n->xmlrpc_client_proc[idle_client_idx];
    proc->current_call = call;
    xmlrpcProcessChangeState( proc, XMLRPC_PROCESS_STATE_WRITING );

    next_idle_client_idx++;
  }

  /* If active (not idle state), add to the select() the XMLRPC clients */
  for(i = 0; i < CN_MAX_XMLRPC_CLIENT_CONNECTIONS; i++)
  {
    int xmlrpc_client_fd = tcpIpSocketGetFD( &(n->xmlrpc_client_proc[i].socket) );
    if ( xmlrpc_client_fd < 0 ) continue;
    fd_set *fdset = NULL;
    if( n->xmlrpc_client_proc[i].state == XMLRPC_PROCESS_STATE_WRITING )
    {
      fdset = &w_fds;
      if( xmlrpc_client_fd > nfds ) nfds = xmlrpc_client_fd;
    }
    else if( n->xmlrpc_client_proc[i].state == XMLRPC_PROCESS_STATE_READING )
    {
      fdset = &r_fds;
      if( xmlrpc_client_fd > nfds ) nfds = xmlrpc_client_fd;
    }

    if (fdset != NULL)
    {
      if(!n->xmlrpc_client_proc[i].socket.open)
        openXmlrpcClientSocket(n, i);

      FD_SET( xmlrpc_client_fd, fdset);
      FD_SET( xmlrpc_client_fd, &err_fds);
    }
  }

  //printf("FD_SET COUNT. R: %d W: %d\n", r_count, w_count);

  /* Add to the select() the active XMLRPC servers */
  int next_xmlrpc_server_i = -1;
  for( i = 0; i < CN_MAX_XMLRPC_SERVER_CONNECTIONS; i++ )
  {
    int server_fd = tcpIpSocketGetFD( &(n->xmlrpc_server_proc[i].socket) );
    if ( server_fd < 0 ) continue;

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

  /* If one XMLRPC server is active at least, add to the select() the listener socket */
  if( next_xmlrpc_server_i >= 0)
  {
    FD_SET( xmlrpc_listner_fd, &r_fds);
    FD_SET( xmlrpc_listner_fd, &err_fds);
    if( xmlrpc_listner_fd > nfds ) nfds = xmlrpc_listner_fd;
  }

  /*
   *
   * TCPROS PROCESSES SELECT() MANAGEMENT
   *
   */

  /* If active (not idle state), add to the select() the TCPROS clients */
  int next_tcpros_client_i = -1;
  for(i = 0; i < CN_MAX_TCPROS_CLIENT_CONNECTIONS; i++)
  {
    int tcpros_client_fd = tcpIpSocketGetFD( &(n->tcpros_client_proc[i].socket) );

    if( next_tcpros_client_i < 0 &&
        i != 0 && //the zero-index is reserved to the roscore communications
        n->tcpros_client_proc[i].state == TCPROS_PROCESS_STATE_IDLE )
    {
      next_tcpros_client_i = i;
    }
    else if(n->tcpros_client_proc[i].state == TCPROS_PROCESS_STATE_WRITING_HEADER)
    {
      FD_SET( tcpros_client_fd, &w_fds);
      FD_SET( tcpros_client_fd, &err_fds);
      if( tcpros_client_fd > nfds ) nfds = tcpros_client_fd;
    }
    else if(n->tcpros_client_proc[i].state == TCPROS_PROCESS_STATE_READING_HEADER_SIZE ||
            n->tcpros_client_proc[i].state == TCPROS_PROCESS_STATE_READING_HEADER ||
            n->tcpros_client_proc[i].state == TCPROS_PROCESS_STATE_READING_SIZE ||
            n->tcpros_client_proc[i].state == TCPROS_PROCESS_STATE_READING)
    {
      FD_SET( tcpros_client_fd, &r_fds);
      FD_SET( tcpros_client_fd, &err_fds);
      if( tcpros_client_fd > nfds ) nfds = tcpros_client_fd;
    }
  }

  /* Add to the select() the active TCPROS servers */
  int next_tcpros_server_i = -1;  
  for( i = 0; i < CN_MAX_TCPROS_SERVER_CONNECTIONS; i++ )
  {
    int server_fd = tcpIpSocketGetFD( &(n->tcpros_server_proc[i].socket) );
    if ( server_fd < 0 ) continue;

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

  /* If one TCPROS server is available at least, add to the select() the listner socket */
  if( next_tcpros_server_i >= 0)
  {
    FD_SET( tcpros_listner_fd, &r_fds);
    FD_SET( tcpros_listner_fd, &err_fds);
    if( tcpros_listner_fd > nfds ) nfds = tcpros_listner_fd;
  }

  uint64_t timeout = n->select_timeout;
  uint64_t tmp_timeout, cur_time = cRosClockGetTimeMs();

  if( n->xmlrpc_client_proc[0].wake_up_time_ms > cur_time )
    tmp_timeout = n->xmlrpc_client_proc[0].wake_up_time_ms - cur_time;
  else
    tmp_timeout = 0;

  if( tmp_timeout < timeout )
    timeout = tmp_timeout;

  for( i = 0; i < CN_MAX_TCPROS_SERVER_CONNECTIONS; i++ )
  {
    if( n->tcpros_server_proc[i].state == TCPROS_PROCESS_STATE_WAIT_FOR_WRITING)
    {
      if( n->tcpros_server_proc[i].wake_up_time_ms > cur_time )
        tmp_timeout = n->tcpros_server_proc[i].wake_up_time_ms - cur_time;
      else
        tmp_timeout = 0;

      if( tmp_timeout < timeout )
        timeout = tmp_timeout;
    }
  }

  /*
   *
   * RPCROS PROCESSES SELECT() MANAGEMENT
   *
   */

  /* Add to the select() the active RPCROS servers */

  int next_rpcros_server_i = -1;

  for( i = 0; i < CN_MAX_RPCROS_SERVER_CONNECTIONS; i++ )
  {
    int server_fd = tcpIpSocketGetFD( &(n->rpcros_server_proc[i].socket) );
    if ( server_fd < 0 ) continue;

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
    else if( n->rpcros_server_proc[i].state == TCPROS_PROCESS_STATE_WAIT_FOR_WRITING )
    {
      FD_SET( server_fd, &err_fds);
      if( server_fd > nfds ) nfds = server_fd;
    }
  }

  /* If one RPCROS server is available at least, add to the select() the listner socket */
  if( next_rpcros_server_i >= 0)
  {
    FD_SET( rpcros_listner_fd, &r_fds);
    FD_SET( rpcros_listner_fd, &err_fds);
    if( rpcros_listner_fd > nfds ) nfds = rpcros_listner_fd;
  }

#ifdef DEBUG
  assert(timeout <= n->select_timeout);
#endif

  struct timeval tv = cRosClockGetTimeVal( timeout );

  int n_set = select(nfds + 1, &r_fds, &w_fds, &err_fds, &tv);

  if (n_set == -1)
  {
    if (errno == EINTR)
    {
      PRINT_INFO("cRosNodeDoEventsLoop() : select() returned EINTR\n");
    }
    else
    {
      perror("cRosNodeDoEventsLoop() ");
      exit( EXIT_FAILURE );
    }
  }
  else if( n_set == 0 )
  {
    PRINT_DEBUG ("cRosNodeDoEventsLoop() : select() timeout\n");

    uint64_t cur_time = cRosClockGetTimeMs();

    XmlrpcProcess *rosproc = &n->xmlrpc_client_proc[0];
    if(rosproc->state == XMLRPC_PROCESS_STATE_IDLE && rosproc->wake_up_time_ms <= cur_time )
    {
      rosproc->wake_up_time_ms = cur_time + CN_PING_LOOP_PERIOD;

      /* Prepare to ping roscore ... */
      PRINT_DEBUG("cRosApiPrepareRequest() : ping roscore\n");

      RosApiCall *call = newRosApiCall();
      if (call == NULL)
      {
        PRINT_ERROR ( "cRosApiPrepareRequest() : Can't allocate memory\n");
        exit(1);
      }

      call->method = CROS_API_GET_PID;
      int rc = xmlrpcParamVectorPushBackString(&call->params, "/rosout");

      rosproc->message_type = XMLRPC_MESSAGE_REQUEST;
      generateXmlrpcMessage( n->host, n->roscore_port, rosproc->message_type,
                          getMethodName(call->method), &call->params, &rosproc->message );

      rosproc->current_call = call;
      xmlrpcProcessChangeState(rosproc, XMLRPC_PROCESS_STATE_WRITING );

    }
    else if( n->xmlrpc_client_proc[0].state != XMLRPC_PROCESS_STATE_IDLE &&
             cur_time - n->xmlrpc_client_proc[0].last_change_time > CN_IO_TIMEOUT )
    {
      /* Timeout between I/O operations... close the socket and re-advertise */
      PRINT_DEBUG ( "cRosNodeDoEventsLoop() : XMLRPC client I/O timeout\n");
      handleXmlrpcClientError( n, 0 );
    }

    for( i = 0; i < CN_MAX_TCPROS_SERVER_CONNECTIONS; i++ )
    {
      int server_fd = tcpIpSocketGetFD( &(n->tcpros_server_proc[i].socket) );
      if ( server_fd < 0 ) continue;
      if( n->tcpros_server_proc[i].state == TCPROS_PROCESS_STATE_WAIT_FOR_WRITING &&
            n->tcpros_server_proc[i].wake_up_time_ms <= cur_time )
      {
        n->tcpros_server_proc[i].wake_up_time_ms = cur_time + n->pubs[n->tcpros_server_proc[i].topic_idx].loop_period;
        tcprosProcessChangeState( &(n->tcpros_server_proc[i]), TCPROS_PROCESS_STATE_START_WRITING );
      }
      else if( (n->tcpros_server_proc[i].state == TCPROS_PROCESS_STATE_READING_HEADER || 
                n->tcpros_server_proc[i].state == TCPROS_PROCESS_STATE_WRITING ) &&
               cur_time - n->tcpros_server_proc[i].last_change_time > CN_IO_TIMEOUT )
      {
        /* Timeout between I/O operations */
        PRINT_DEBUG ( "cRosNodeDoEventsLoop() : TCPROS server I/O timeout\n");
        handleTcprosServerError( n, i );
      }
    }
  }
  else
  {

    PRINT_DEBUG ( "cRosNodeDoEventsLoop() : select() unblocked\n" );

    for(i = 0; i < CN_MAX_XMLRPC_CLIENT_CONNECTIONS; i++ )
    {
      int xmlrpc_client_fd = tcpIpSocketGetFD( &(n->xmlrpc_client_proc[i].socket) );
      if( xmlrpc_client_fd < 0 ) continue;

      if( FD_ISSET(xmlrpc_client_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : XMLRPC Client error\n" );
        handleXmlrpcClientError( n, i );
      }

      /* Check what is the socket unblocked by the select, and start the requested operations */
      else if( ( n->xmlrpc_client_proc[i].state == XMLRPC_PROCESS_STATE_WRITING && FD_ISSET(xmlrpc_client_fd, &w_fds) ) ||
          ( n->xmlrpc_client_proc[i].state == XMLRPC_PROCESS_STATE_READING && FD_ISSET(xmlrpc_client_fd, &r_fds) ) )
      {
        doWithXmlrpcClientSocket( n, i );
      }
    }

    if ( next_xmlrpc_server_i >= 0 )
    {
      if( FD_ISSET( xmlrpc_listner_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : XMLRPC  listner error\n" ); 
      }
      else if( FD_ISSET( xmlrpc_listner_fd, &r_fds) )
      {
        PRINT_DEBUG ( "cRosNodeDoEventsLoop() : XMLRPC listner ready\n" );
        if( tcpIpSocketAccept( &(n->xmlrpc_listner_proc.socket), 
            &(n->xmlrpc_server_proc[next_xmlrpc_server_i].socket) ) == TCPIPSOCKET_DONE &&
            tcpIpSocketSetReuse( &(n->xmlrpc_server_proc[next_xmlrpc_server_i].socket) ) && 
            tcpIpSocketSetNonBlocking( &(n->xmlrpc_server_proc[next_xmlrpc_server_i].socket ) ) )

          xmlrpcProcessChangeState( &(n->xmlrpc_server_proc[next_xmlrpc_server_i]), XMLRPC_PROCESS_STATE_READING );        
      }
    }

    for( i = 0; i < CN_MAX_XMLRPC_SERVER_CONNECTIONS; i++ )
    {
      int server_fd = tcpIpSocketGetFD( &(n->xmlrpc_server_proc[i].socket) );
      if ( server_fd < 0 ) continue;
      if( FD_ISSET(server_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : XMLRPC server error\n" );
        tcpIpSocketClose( &(n->xmlrpc_server_proc[i].socket) );
        xmlrpcProcessChangeState( &(n->xmlrpc_server_proc[next_xmlrpc_server_i]), XMLRPC_PROCESS_STATE_IDLE ); 
      }
      else if( ( n->xmlrpc_server_proc[i].state == XMLRPC_PROCESS_STATE_WRITING && FD_ISSET(server_fd, &w_fds) ) || 
               ( n->xmlrpc_server_proc[i].state == XMLRPC_PROCESS_STATE_READING && FD_ISSET(server_fd, &r_fds) ) )
      {
        doWithXmlrpcServerSocket( n, i );
      }
    }

    for(i = 0; i < CN_MAX_TCPROS_CLIENT_CONNECTIONS; i++ )
    {
      TcprosProcess *client_proc = &(n->tcpros_client_proc[i]);
      int tcpros_client_fd = tcpIpSocketGetFD( &(client_proc->socket) );

      if( client_proc->state != TCPROS_PROCESS_STATE_IDLE && FD_ISSET(tcpros_client_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : XMLRPC Client error\n" );
        handleTcprosClientError( n, i );
      }

      if( client_proc->state == TCPROS_PROCESS_STATE_CONNECTING ||
          ( client_proc->state == TCPROS_PROCESS_STATE_WRITING_HEADER && FD_ISSET(tcpros_client_fd, &w_fds) ) ||
          ( client_proc->state == TCPROS_PROCESS_STATE_READING_SIZE && FD_ISSET(tcpros_client_fd, &r_fds) ) ||
          ( client_proc->state == TCPROS_PROCESS_STATE_READING && FD_ISSET(tcpros_client_fd, &r_fds) ) ||
          ( client_proc->state == TCPROS_PROCESS_STATE_READING_HEADER_SIZE && FD_ISSET(tcpros_client_fd, &r_fds) ) ||
          ( client_proc->state == TCPROS_PROCESS_STATE_READING_HEADER && FD_ISSET(tcpros_client_fd, &r_fds) ) )
      {
        doWithTcprosClientSocket( n, i );
      }
    }

    if ( next_tcpros_server_i >= 0 )
    {
      if( FD_ISSET( tcpros_listner_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : TCPROS listner error\n" ); 
      }
      else if( FD_ISSET( tcpros_listner_fd, &r_fds) )
      {
        PRINT_DEBUG ( "cRosNodeDoEventsLoop() : TCPROS listner ready\n" );
        if( tcpIpSocketAccept( &(n->tcpros_listner_proc.socket), 
            &(n->tcpros_server_proc[next_tcpros_server_i].socket) ) == TCPIPSOCKET_DONE &&
            tcpIpSocketSetReuse( &(n->tcpros_server_proc[next_tcpros_server_i].socket) ) && 
            tcpIpSocketSetNonBlocking( &(n->tcpros_server_proc[next_tcpros_server_i].socket ) ) &&
            tcpIpSocketSetKeepAlive( &(n->tcpros_server_proc[next_tcpros_server_i].socket ), 60, 10, 9 ) )
        {

          tcprosProcessChangeState( &(n->tcpros_server_proc[next_tcpros_server_i]), TCPROS_PROCESS_STATE_READING_HEADER );
          uint64_t cur_time = cRosClockGetTimeMs();
          n->tcpros_server_proc[next_tcpros_server_i].wake_up_time_ms = cur_time;
        }
      }
    }

    for( i = 0; i < CN_MAX_TCPROS_SERVER_CONNECTIONS; i++ )
    {
      int server_fd = tcpIpSocketGetFD( &(n->tcpros_server_proc[i].socket) );
      if ( server_fd < 0 ) continue;
      if( FD_ISSET(server_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : TCPROS server error\n" );
        tcpIpSocketClose( &(n->tcpros_server_proc[i].socket) );
        tcprosProcessChangeState( &(n->tcpros_server_proc[next_tcpros_server_i]), TCPROS_PROCESS_STATE_IDLE ); 
      }
      else if( ( n->tcpros_server_proc[i].state == TCPROS_PROCESS_STATE_READING_HEADER && FD_ISSET(server_fd, &r_fds) ) ||
        ( n->tcpros_server_proc[i].state == TCPROS_PROCESS_STATE_START_WRITING && FD_ISSET(server_fd, &w_fds) ) || 
        ( n->tcpros_server_proc[i].state == TCPROS_PROCESS_STATE_WRITING && FD_ISSET(server_fd, &w_fds) ) )
      {
        doWithTcprosServerSocket( n, i );    
      }
    }

    if ( next_rpcros_server_i >= 0 )
    {
      if( FD_ISSET( rpcros_listner_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : TCPROS listner error\n" );
      }
      else if( next_rpcros_server_i >= 0 && FD_ISSET( rpcros_listner_fd, &r_fds) )
      {
        PRINT_DEBUG ( "cRosNodeDoEventsLoop() : TCPROS listner ready\n" );
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
      int server_fd = tcpIpSocketGetFD( &(n->rpcros_server_proc[i].socket) );
      if ( server_fd < 0 ) continue;
      if( FD_ISSET(server_fd, &err_fds) )
      {
        PRINT_ERROR ( "cRosNodeDoEventsLoop() : TCPROS server error\n" );
        tcpIpSocketClose( &(n->rpcros_server_proc[i].socket) );
        tcprosProcessChangeState( &(n->rpcros_server_proc[next_rpcros_server_i]), TCPROS_PROCESS_STATE_IDLE );
      }
      else if( ( n->rpcros_server_proc[i].state == TCPROS_PROCESS_STATE_READING_HEADER_SIZE && FD_ISSET(server_fd, &r_fds) ) ||
        ( n->rpcros_server_proc[i].state == TCPROS_PROCESS_STATE_READING_HEADER && FD_ISSET(server_fd, &r_fds) ) ||
        ( n->rpcros_server_proc[i].state == TCPROS_PROCESS_STATE_READING_SIZE && FD_ISSET(server_fd, &r_fds) ) ||
        ( n->rpcros_server_proc[i].state == TCPROS_PROCESS_STATE_READING && FD_ISSET(server_fd, &r_fds) ) ||
        ( n->rpcros_server_proc[i].state == TCPROS_PROCESS_STATE_WRITING_HEADER && FD_ISSET(server_fd, &w_fds) ) ||
        ( n->rpcros_server_proc[i].state == TCPROS_PROCESS_STATE_WRITING && FD_ISSET(server_fd, &w_fds) ) )
      {
        doWithRpcrosServerSocket( n, i );
      }
    }

  }
}

void cRosNodeStart( CrosNode *n, unsigned char *exit )
{
  PRINT_VDEBUG ( "cRosNodeStart ()\n" );
  while( !(*exit) )
    cRosNodeDoEventsLoop( n );
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

  ServiceProviderNode *service = &node->services[serviceidx];
  xmlrpcParamVectorPushBackString( &call->params, node->name );
  xmlrpcParamVectorPushBackString( &call->params, service->service_name );
  char uri[256];
  snprintf( uri, 256, "rosrpc://%s:%d/", node->host, node->rpcros_port);
  xmlrpcParamVectorPushBackString( &call->params, uri );
  snprintf( uri, 256, "http://%s:%d/", node->host, node->xmlrpc_port);
  xmlrpcParamVectorPushBackString( &call->params, uri );

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

int enqueueRequestTopic(CrosNode *node, int subidx)
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
  if (sub->status_callback != NULL)
  {
    CrosNodeStatusUsr status;
    initCrosNodeStatus(&status);
    status.xmlrpc_host = sub->topic_host;
    status.xmlrpc_port = sub->topic_port;
    sub->status_callback(&status, sub->context);
  }

  xmlrpcParamVectorPushBackString(&call->params, node->name );
  xmlrpcParamVectorPushBackString(&call->params, sub->topic_name );
  xmlrpcParamVectorPushBackArray(&call->params);
  XmlrpcParam* array_param = xmlrpcParamVectorAt(&call->params,2);
  xmlrpcParamArrayPushBackArray(array_param);
  xmlrpcParamArrayPushBackString(xmlrpcParamArrayGetParamAt(array_param,0),CROS_TRANSPORT_TCPROS_STRING);

  return enqueueSlaveApiCallInternal(node, call);
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

  for(it = 0; it < n->n_services; it++)
  {
    if (n->subs[it].topic_name == NULL)
      continue;

    enqueueServiceAdvertise(n, it);
  }
}

void initPublisherNode(PublisherNode *node)
{
  node->message_definition = NULL;
  node->topic_name = NULL;
  node->topic_type = NULL;
  node->md5sum = NULL;
  node->callback = NULL;
  node->status_callback = NULL;
  node->context = NULL;
  node->client_tcpros_id = -1;
  node->loop_period = 1000;
}

void initSubscriberNode(SubscriberNode *node)
{
  node->message_definition = NULL;
  node->topic_host = NULL;
  node->topic_port = -1;
  node->topic_name = NULL;
  node->topic_type = NULL;
  node->md5sum = NULL;
  node->callback = NULL;
  node->status_callback = NULL;
  node->context = NULL;
  node->client_xmlrpc_id = -1;
  node->client_tcpros_id = -1;
  node->tcpros_port = -1;
}

void initServiceProviderNode(ServiceProviderNode *node)
{
  node->service_name = NULL;
  node->service_type = NULL;
  node->md5sum = NULL;
  node->callback = NULL;
  node->status_callback = NULL;
  node->context = NULL;
  node->servicerequest_type = NULL;
  node->serviceresponse_type = NULL;
}

void initParameterSubscrition(ParameterSubscription *subscription)
{
  subscription->parameter_key = NULL;
  xmlrpcParamInit(&subscription->parameter_value);
  subscription->status_callback = NULL;
  subscription->context = NULL;
}

void releasePublisherNode(PublisherNode *node)
{
  free(node->message_definition);
  free(node->topic_name);
  free(node->topic_type);
  free(node->md5sum);
}

void releaseSubscriberNode(SubscriberNode *node)
{
  free(node->message_definition);
  free(node->topic_name);
  free(node->topic_type);
  free(node->md5sum);
  free(node->topic_host);
}

void releaseServiceProviderNode(ServiceProviderNode *node)
{
  free(node->service_name);
  free(node->service_type);
  free(node->servicerequest_type);
  free(node->serviceresponse_type);
  free(node->md5sum);
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

void releaseParameterSubscrition(ParameterSubscription *subscription)
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
    call->host = malloc(strlen(host) + 1);
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

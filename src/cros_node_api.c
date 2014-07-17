#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <assert.h>

#include "cros_node_api.h"
#include "cros_api.h"
#include "cros_api_internal.h"
#include "cros_defs.h"
#include "xmlrpc_params.h"

int
lookup_host (const char *host, char *ip)
{
  struct addrinfo hints, *res;
  int errcode;
  char addrstr[100];
  void *ptr = NULL;

  memset (&hints, 0, sizeof (hints));
  hints.ai_family = PF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags |= AI_CANONNAME;

  errcode = getaddrinfo (host, NULL, &hints, &res);
  if (errcode != 0)
    {
      perror ("getaddrinfo");
      return -1;
    }

  printf ("Host: %s\n", host);
  while (res)
    {
      inet_ntop (res->ai_family, res->ai_addr->sa_data, addrstr, 100);

      switch (res->ai_family)
        {
        case AF_INET:
          ptr = &((struct sockaddr_in *) res->ai_addr)->sin_addr;
          break;
#ifdef AF_INET6
        case AF_INET6:
          ptr = &((struct sockaddr_in6 *) res->ai_addr)->sin6_addr;
          break;
#endif
        default:
          break;
        }
      if (ptr == NULL)
      {
        PRINT_ERROR ("getaddrinfo unsupported ai_family");
        return -1;
      }

      inet_ntop (res->ai_family, ptr, addrstr, 100);
      PRINT_VDEBUG ("IPv%d address: %s (%s)\n", res->ai_family == PF_INET6 ? 6 : 4,
              addrstr, res->ai_canonname);
      res = res->ai_next;
      strcpy(ip, (char*)&addrstr);
    }

  return 0;
}

// TODO Improve this
static int checkResponseValue( XmlrpcParamVector *params )
{
  XmlrpcParam *param = xmlrpcParamVectorAt( params, 0 );
  
  // TODO
  if( param == NULL ||
      ( xmlrpcParamGetType( param ) != XMLRPC_PARAM_INT &&
      xmlrpcParamGetType( param ) != XMLRPC_PARAM_ARRAY ) )
  {
    return 0;
  }
  
  int res = 0;
  
  if( xmlrpcParamGetType( param ) == XMLRPC_PARAM_INT )
    res = param->data.as_int;
  else if ( xmlrpcParamGetType( param ) == XMLRPC_PARAM_ARRAY &&
            param->array_n_elem > 0 )
  {
    XmlrpcParam *array_param = param->data.as_array;
    if( xmlrpcParamGetType( &( array_param[0]) ) != XMLRPC_PARAM_INT )
      res = 0;
    else
      res = array_param[0].data.as_int;
  }
  
  return res;
}

void cRosApiPrepareRequest( CrosNode *n, int client_idx )
{
  PRINT_VDEBUG ( "cRosApiPrepareRequest()\n" );

  XmlrpcProcess *client_proc = &(n->xmlrpc_client_proc[client_idx]);

  client_proc->message_type = XMLRPC_MESSAGE_REQUEST;

  assert(client_proc->current_call != NULL);
  RosApiCall *call = client_proc->current_call;
  if(client_idx == 0) //requests managed by the xmlrpc client connected to roscore
  {
    generateXmlrpcMessage(n->host, n->roscore_port, XMLRPC_MESSAGE_REQUEST,
                          getMethodName(call->method), &call->params, &client_proc->message);
  }
  else // client_idx > 0
  {
    if (call->provider_idx == -1)
    {
      generateXmlrpcMessage(n->host, call->port, XMLRPC_MESSAGE_REQUEST,
                            getMethodName(call->method), &call->params, &client_proc->message);
    }
    else
    {
      SubscriberNode *subscriber_node = &n->subs[call->provider_idx];
      generateXmlrpcMessage(n->host, subscriber_node->topic_port, XMLRPC_MESSAGE_REQUEST,
                            getMethodName(call->method), &call->params, &client_proc->message);
    }
  }
}


/*
 * XMLRPC Return values are lists in the format: [statusCode, statusMessage, value]
 *
 * statusCode (int): An integer indicating the completion condition of the method. Current values:
 *      -1: ERROR: Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.
 *       0: SUCCESS: Method completed successfully.
 *       Individual methods may assign additional meaning/semantics to statusCode.
 *
 * statusMessage (str): a human-readable string describing the return status
 *
 * value (anytype): return value is further defined by individual API calls.
 */
int cRosApiParseResponse( CrosNode *n, int client_idx )
{
  PRINT_VDEBUG ( "cRosApiParseResponse()\n" );
  XmlrpcProcess *client_proc = &(n->xmlrpc_client_proc[client_idx]);
  int ret = -1;

  assert(client_proc->current_call != NULL);
  RosApiCall *call = client_proc->current_call;
  if(client_idx == 0 && call->user_call == 0) //xmlrpc client connected to roscore
  {
    if( client_proc->message_type != XMLRPC_MESSAGE_RESPONSE )
    {
      PRINT_ERROR ( "cRosApiParseResponse() : Not a response message \n" );
      return ret;
    }

    switch (call->method)
    {
      case CROS_API_REGISTER_PUBLISHER:
      {
        PRINT_DEBUG ( "cRosApiParseResponse() : registerPublisher response \n" );
        if( checkResponseValue( &client_proc->response ) )
          ret = 0;

        break;
      }
      case CROS_API_REGISTER_SERVICE:
      {
        PRINT_DEBUG ( "cRosApiParseResponse() : registerService response \n" );

        if( checkResponseValue( &client_proc->response ) )
          ret = 0;

        break;
      }
      case CROS_API_REGISTER_SUBSCRIBER:
      {
        PRINT_DEBUG ( "cRosApiParseResponse() : registerSubscriber response \n" );
        //xmlrpcParamVectorPrint( &(client_proc->params) );

        //Get the next subscriber without a topic host
        assert(call->provider_idx != -1);
        int subidx = call->provider_idx;
        SubscriberNode* requesting_subscriber = &(n->subs[subidx]);

        if(checkResponseValue( &client_proc->response ) )
        {
          ret = 0;

          //Check if there is some publishers for the subscription
          XmlrpcParam *param = xmlrpcParamVectorAt(&client_proc->response, 0);
          XmlrpcParam *array = xmlrpcParamArrayGetParamAt(param,2);
          int available_pubs_n = xmlrpcParamArrayGetSize(array);

          if (available_pubs_n > 0)
          {
            int i ;
            for (i = 0; i < available_pubs_n; i++)
            {
              XmlrpcParam* pub_host = xmlrpcParamArrayGetParamAt(array, i);
              char* pub_host_string = xmlrpcParamGetString(pub_host);
              //manage string for exploit informations
              //removing the 'http://' and the last '/'
              int dirty_string_len = strlen(pub_host_string);
              char* clean_string = (char *)calloc(dirty_string_len-8+1,sizeof(char));
              if (clean_string == NULL)
                exit(1);
              strncpy(clean_string,pub_host_string+7,dirty_string_len-8);
              char * progress = NULL;
              char* hostname = strtok_r(clean_string,":",&progress);
              if(requesting_subscriber->topic_host == NULL)
              {
                requesting_subscriber->topic_host = (char *)calloc(100,sizeof(char)); //deleted in cRosNodeDestroy
                if (requesting_subscriber->topic_host == NULL)
                  exit(1);
              }

              int rc = lookup_host(hostname, requesting_subscriber->topic_host);
              if (rc)
                return ret;

              requesting_subscriber->topic_port = atoi(strtok_r(NULL,":",&progress));
              enqueueRequestTopic(n, subidx);

              break;
            }
          }
        }

        break;
      }
      case CROS_API_SUBSCRIBE_PARAM:
      {
        int paramsubidx = call->provider_idx;
        ParameterSubscription *subscription = &n->paramsubs[paramsubidx];

        if(checkResponseValue( &client_proc->response ) )
        {
          XmlrpcParam *array = xmlrpcParamVectorAt(&client_proc->response,0);
          XmlrpcParam *status_msg = xmlrpcParamArrayGetParamAt(array, 1);
          XmlrpcParam *value = xmlrpcParamArrayGetParamAt(array, 2);

          XmlrpcParam copy;
          int rc = xmlrpcParamCopy(&copy, value);
          if (rc < 0)
            break;

          ret = 0;
          xmlrpcParamRelease(&subscription->parameter_value);
          subscription->parameter_value = copy;
        }

        break;
      }
      case CROS_API_GET_PID:
      {
        PRINT_DEBUG ( "cRosApiParseResponse() : ping response \n" );
        //xmlrpcParamVectorPrint( &(client_proc->params) );

        if( checkResponseValue( &client_proc->response ) )
        {
          ret = 0;
          XmlrpcParam* roscore_pid_param =
              xmlrpcParamArrayGetParamAt(xmlrpcParamVectorAt(&client_proc->response,0),2);

          if(n->roscore_pid == -1)
          {
            n->roscore_pid = roscore_pid_param->data.as_int;
          }
          else if (n->roscore_pid != roscore_pid_param->data.as_int)
          {
            n->roscore_pid = roscore_pid_param->data.as_int;
            restartAdversing(n);
          }
        }
        else
        {
          restartAdversing(n);
        }

        xmlrpcProcessChangeState(client_proc,XMLRPC_PROCESS_STATE_IDLE);
        break;
      }
      case CROS_API_UNREGISTER_SERVICE:
      {
        //xmlrpcParamVectorPrint( &(client_proc->params) );

        if( checkResponseValue( &client_proc->response ) )
        {
          ret = 0;
          XmlrpcParam* status_msg = xmlrpcParamVectorAt( &client_proc->response, 1);
          XmlrpcParam* unreg_num = xmlrpcParamVectorAt( &client_proc->response, 2);
        }

        xmlrpcProcessChangeState(client_proc,XMLRPC_PROCESS_STATE_IDLE);
        break;
      }
      case CROS_API_UNREGISTER_PUBLISHER:
      {
        if( checkResponseValue( &client_proc->response ) )
        {
          ret = 0;
          XmlrpcParam *array = xmlrpcParamVectorAt(&client_proc->response,0);
          XmlrpcParam* status_msg = xmlrpcParamArrayGetParamAt(array, 1);
          XmlrpcParam* unreg_num = xmlrpcParamArrayGetParamAt(array, 2);
        }

        xmlrpcProcessChangeState(client_proc,XMLRPC_PROCESS_STATE_IDLE);
        break;
      }
      case CROS_API_UNREGISTER_SUBSCRIBER:
      {
        if( checkResponseValue( &client_proc->response ) )
        {
          ret = 0;
          XmlrpcParam *array = xmlrpcParamVectorAt(&client_proc->response, 0);
          XmlrpcParam* status_msg = xmlrpcParamArrayGetParamAt(array, 1);
          XmlrpcParam* unreg_num = xmlrpcParamArrayGetParamAt(array, 2);
        }

        xmlrpcProcessChangeState(client_proc,XMLRPC_PROCESS_STATE_IDLE);
        break;
      }
      case CROS_API_UNSUBSCRIBE_PARAM:
      {
        if( checkResponseValue( &client_proc->response ) )
        {
          ret = 0;
          XmlrpcParam *array = xmlrpcParamVectorAt(&client_proc->response, 0);
          XmlrpcParam* status_msg = xmlrpcParamArrayGetParamAt(array, 1);
          XmlrpcParam* unreg_num = xmlrpcParamArrayGetParamAt(array, 2);
        }

        xmlrpcProcessChangeState(client_proc,XMLRPC_PROCESS_STATE_IDLE);
        break;
      }
      default:
      {
        assert(0);
      }
    }
  }
  else // client_idx > 0 || user_call = 1
  {
    void *result = NULL;
    switch (call->method)
    {
      case CROS_API_LOOKUP_NODE:
      case CROS_API_GET_PUBLISHED_TOPICS:
      case CROS_API_GET_TOPIC_TYPES:
      case CROS_API_GET_SYSTEM_STATE:
      case CROS_API_GET_URI:
      case CROS_API_LOOKUP_SERVICE:
      case CROS_API_GET_BUS_STATS:
      case CROS_API_GET_BUS_INFO:
      case CROS_API_GET_MASTER_URI:
      case CROS_API_SHUTDOWN:
      case CROS_API_GET_PID:
      case CROS_API_GET_SUBSCRIPTIONS:
      case CROS_API_GET_PUBLICATIONS:
      case CROS_API_DELETE_PARAM:
      case CROS_API_SET_PARAM:
      case CROS_API_GET_PARAM:
      case CROS_API_SEARCH_PARAM:
      case CROS_API_HAS_PARAM:
      case CROS_API_GET_PARAM_NAMES:
      {
        ret = 0;

        ResultCallback callback = call->result_callback;
        if (callback != NULL)
        {
          void *result = call->fetch_result_callback(&client_proc->response);
          callback(call->id, result, call->context_data);
          if (result != NULL)
            call->free_result_callback(result);
        }

        break;
      }
      case CROS_API_REQUEST_TOPIC:
      {
        if( checkResponseValue( &client_proc->response ) )
        {
          ret = 0;

          XmlrpcParam* param_array = xmlrpcParamVectorAt(&client_proc->response,0);
          XmlrpcParam* nested_array = xmlrpcParamArrayGetParamAt(param_array,2);
          XmlrpcParam* tcp_port = xmlrpcParamArrayGetParamAt(nested_array,2);

          RosApiCall *call = client_proc->current_call;

          int tcp_port_print = tcp_port->data.as_int;
          SubscriberNode* sub = &n->subs[call->provider_idx];
          sub->tcpros_port = tcp_port_print;

          TcprosProcess* tcpros_proc = &n->tcpros_client_proc[sub->client_tcpros_id];
          tcpros_proc->topic_idx = call->provider_idx;

          //need to be checked because maybe the connection went down suddenly.
          if(!tcpros_proc->socket.open)
          {
            tcpIpSocketOpen(&(tcpros_proc->socket));
          }

          PRINT_DEBUG( "cRosApiParseResponse() : requestTopic response [tcp port: %d]\n", tcp_port_print);
          xmlrpcProcessChangeState(client_proc,XMLRPC_PROCESS_STATE_IDLE);

          //set the process to open the socket with the desired host
          tcprosProcessChangeState(tcpros_proc, TCPROS_PROCESS_STATE_CONNECTING);
        }

        break;
      }
      default:
      {
        assert(0);
      }
    }
  }

  return ret;
}

int cRosApiParseRequestPrepareResponse( CrosNode *n, int server_idx )
{
  PRINT_DEBUG ( "cRosApiParseRequestPrepareResponse()\n" );

  XmlrpcProcess *server_proc = &(n->xmlrpc_server_proc[server_idx]);

  if( server_proc->message_type != XMLRPC_MESSAGE_REQUEST)
  {
    PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Not a request message \n" );
    return -1;
  }

  server_proc->message_type = XMLRPC_MESSAGE_RESPONSE;

  XmlrpcParamVector params;
  xmlrpcParamVectorInit(&params);

  CrosApiMethod method = getMethodCode(dynStringGetData(&server_proc->method));
  switch (method)
  {
    case CROS_API_GET_PID:
    {
      xmlrpcParamVectorPushBackInt( &params, n->pid );
      break;
    }
    case CROS_API_PUBLISHER_UPDATE:
    {
      PRINT_INFO("publisherUpdate()\n");
      // TODO Store the subscribed node name
      XmlrpcParam *caller_id_param, *topic_param, *publishers_param;

#ifdef DEBUG
      xmlrpcParamVectorPrint( &server_proc->params );
#endif

      caller_id_param = xmlrpcParamVectorAt(&server_proc->params, 0);
      topic_param = xmlrpcParamVectorAt( &server_proc->params, 1);
      publishers_param = xmlrpcParamVectorAt(&server_proc->params, 2);

      if( xmlrpcParamGetType( caller_id_param ) != XMLRPC_PARAM_STRING ||
        xmlrpcParamGetType( topic_param ) != XMLRPC_PARAM_STRING ||
        xmlrpcParamGetType( publishers_param ) != XMLRPC_PARAM_ARRAY  )
      {
        PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Wrong publisherUpdate message \n" );
        xmlrpcParamVectorPushBackString( &params, "Unable to resolve hostname" );
        break;
      }
      else
      {
        int array_size = xmlrpcParamArrayGetSize( publishers_param );
        XmlrpcParam *uri;
        XmlrpcParam *proto_name;
        int sub_idx = -1;
        int uri_found = 0;
        SubscriberNode* requesting_subscriber = NULL;

        int i = 0;
        for(; i < n->n_subs; i++)
        {
          if (n->subs[i].topic_name == NULL)
            continue;

          if( strcmp( xmlrpcParamGetString( topic_param ), n->subs[i].topic_name ) == 0)
          {
            sub_idx = i;
            requesting_subscriber = &(n->subs[i]);
            break;
          }
        }

        if(array_size > 0)
        {
          uri = xmlrpcParamArrayGetParamAt ( publishers_param, 0 );
          if(xmlrpcParamGetType(uri) == XMLRPC_PARAM_STRING)
          {
            uri_found = 1;
          }
        }

        if (sub_idx != -1 && uri_found && requesting_subscriber->tcpros_port == -1)
        {
          // Subscriber that is still waiting for a tcpros connection found
          publishers_param = xmlrpcParamVectorAt(&server_proc->params, 2);
          int available_pubs_n = xmlrpcParamArrayGetSize(publishers_param);

          if(available_pubs_n > 0)
          {
            //TODO: We just consider the first host. In the remote future we should consider the others as well

            XmlrpcParam* pub_host = xmlrpcParamArrayGetParamAt(publishers_param, 0);
            char* pub_host_string = xmlrpcParamGetString(pub_host);
            //manage string for exploit informations
            //removing the 'http://' and the last '/'
            int dirty_string_len = strlen(pub_host_string);
            char* clean_string = (char *)calloc(dirty_string_len-8+1,sizeof(char));
            if (clean_string == NULL)
              exit(1);
            strncpy(clean_string,pub_host_string+7,dirty_string_len-8);
            char * progress = NULL;
            char* hostname = strtok_r(clean_string,":",&progress);
            if(requesting_subscriber->topic_host == NULL)
            {
              requesting_subscriber->topic_host = (char *)calloc(100,sizeof(char)); //deleted in cRosNodeDestroy
              if (requesting_subscriber->topic_host == NULL)
                exit(1);
            }
            int rc = lookup_host(hostname, requesting_subscriber->topic_host);
            if (rc)
            {
              PRINT_ERROR ( "lookup_host() : Unable to resolve hostname \n" );
              xmlrpcParamVectorPushBackString( &params, "Unable to resolve hostname" );
              break;
            }

            requesting_subscriber->topic_port = atoi(strtok_r(NULL,":",&progress));
            enqueueRequestTopic(n, sub_idx);
          }

          xmlrpcParamVectorPushBackArray(&params);
          XmlrpcParam *array = xmlrpcParamVectorAt(&params, 0);
          xmlrpcParamArrayPushBackInt(array, 1);
          xmlrpcParamArrayPushBackString(array, "");
          xmlrpcParamArrayPushBackInt(array, 0);
        }
        else
        {
          PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Topic not available or protocol for publisherUpdate() not supported\n" );

          //Resetting tcpros infos
          requesting_subscriber->tcpros_port = -1;

          xmlrpcParamVectorPushBackString( &params, "Topic not available or protocol for publisherUpdate() not supported" );
        }
      }

      break;
    }
    case CROS_API_REQUEST_TOPIC:
    {
      XmlrpcParam *node_param, *topic_param, *protocols_param;

      node_param = xmlrpcParamVectorAt( &(server_proc->params), 0);
      topic_param = xmlrpcParamVectorAt( &(server_proc->params), 1);
      protocols_param = xmlrpcParamVectorAt( &(server_proc->params), 2);

      if( xmlrpcParamGetType( node_param ) != XMLRPC_PARAM_STRING ||
          xmlrpcParamGetType( topic_param ) != XMLRPC_PARAM_STRING ||
          xmlrpcParamGetType( protocols_param ) != XMLRPC_PARAM_ARRAY  )
      {
        PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Wrong requestTopic message \n" );
        xmlrpcParamVectorPushBackString( &params, "Wrong requestTopic message" );
        break;
      }
      else
      {
        int array_size = xmlrpcParamArrayGetSize( protocols_param );
        XmlrpcParam *proto, *proto_name;
        int i = 0, topic_found = 0, protocol_found = 0;

        for( i = 0 ; i < n->n_pubs; i++)
        {
          PublisherNode *pub = &n->pubs[i];
          if (pub->topic_name == NULL)
            continue;

          if( strcmp( xmlrpcParamGetString( topic_param ), pub->topic_name ) == 0)
          {
            topic_found = 1;
            if (pub->status_callback != NULL && strlen(server_proc->host) != 0)
            {
              CrosNodeStatusUsr status;
              initCrosNodeStatus(&status);
              status.xmlrpc_host = server_proc->host;
              status.xmlrpc_port = server_proc->port;
              pub->status_callback(&status, pub->context);
            }

            break;
          }
        }

        for( i = 0 ; i < array_size; i++)
        {
          proto = xmlrpcParamArrayGetParamAt ( protocols_param, i );

          if( xmlrpcParamGetType( proto ) == XMLRPC_PARAM_ARRAY &&
              ( proto_name = xmlrpcParamArrayGetParamAt ( proto, 0 ) ) != NULL &&
              xmlrpcParamGetType( proto_name ) == XMLRPC_PARAM_STRING &&
              strcmp( xmlrpcParamGetString( proto_name ), CROS_TRANSPORT_TCPROS_STRING) == 0 )
          {
            protocol_found = 1;
            break;
          }
        }

        if( topic_found && protocol_found)
        {
          xmlrpcParamVectorPushBackArray(&params);
          XmlrpcParam *array1 = xmlrpcParamVectorAt(&params, 0);
          xmlrpcParamArrayPushBackInt(array1, 1);
          xmlrpcParamArrayPushBackString(array1, "");
          XmlrpcParam* array2 = xmlrpcParamArrayPushBackArray(array1);
          xmlrpcParamArrayPushBackString( array2, CROS_TRANSPORT_TCPROS_STRING );
          xmlrpcParamArrayPushBackString( array2, n->host );
          xmlrpcParamArrayPushBackInt( array2, n->tcpros_port );
        }
        else
        {
          PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Topic or protocol for requestTopic not supported\n" );
          xmlrpcParamVectorPushBackString( &params, "Topic or protocol for requestTopic not supported");
          break;
        }
      }

      break;
    }
    case CROS_API_GET_BUS_STATS:
    {
      // CHECK-ME What to answer here?
      xmlrpcParamVectorPushBackArray(&params);
      XmlrpcParam *array = xmlrpcParamVectorAt(&params, 0);
      xmlrpcParamArrayPushBackInt(array, 0);
      xmlrpcParamArrayPushBackString(array, "");
      break;
    }
    case CROS_API_GET_BUS_INFO:
    {
      // CHECK-ME What to answer here?
      xmlrpcParamVectorPushBackArray(&params);
      XmlrpcParam *array = xmlrpcParamVectorAt(&params, 0);
      xmlrpcParamArrayPushBackInt(array, 0);
      xmlrpcParamArrayPushBackString(array, "");
      break;
    }
    case CROS_API_GET_MASTER_URI:
    {
      xmlrpcParamVectorPushBackArray(&params);
      XmlrpcParam *array = xmlrpcParamVectorAt(&params, 0);
      xmlrpcParamArrayPushBackInt(array, 1);
      xmlrpcParamArrayPushBackString(array, "");
      char node_uri[256];
      snprintf(node_uri, 256, "http://%s:%d/", n->roscore_host, n->roscore_port);
      xmlrpcParamArrayPushBackString(array, node_uri);
      break;
    }
    case CROS_API_SHUTDOWN:
    {
      xmlrpcParamVectorPushBackArray(&params);
      XmlrpcParam *array = xmlrpcParamVectorAt(&params, 0);
      xmlrpcParamArrayPushBackInt(array, 1);
      xmlrpcParamArrayPushBackString(array, "");
      xmlrpcParamArrayPushBackInt(array, 1);
      break;
    }
    case CROS_API_PARAM_UPDATE:
    {
      XmlrpcParam *node_param = xmlrpcParamVectorAt(&server_proc->params, 0);
      XmlrpcParam *key_param = xmlrpcParamVectorAt(&server_proc->params, 1);
      XmlrpcParam *value_param = xmlrpcParamVectorAt(&server_proc->params, 2);
      if (xmlrpcParamGetType(key_param) != XMLRPC_PARAM_STRING)
      {
        PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Wrong paramUpdate message\n" );
        goto PrepareResponse;
      }

      int paramsubidx = -1;
      char *parameter_key = xmlrpcParamGetString(key_param);
      int it = 0;
      for (it = 0 ; it < n->n_paramsubs; it++)
      {
        if (n->paramsubs[it].parameter_key == NULL)
          continue;

        if (strcmp(parameter_key, n->paramsubs[it].parameter_key) == 0)
        {
          paramsubidx = it;

          break;
        }
      }

      ParameterSubscription* subscription = NULL;
      if (paramsubidx != -1)
      {
        if (subscription->status_callback != NULL)
        {
          CrosNodeStatusUsr status;
          initCrosNodeStatus(&status);
          status.state = CROS_STATUS_PARAM_UPDATE;
          status.provider_idx = it;
          status.parameter_key = subscription->parameter_key;
          status.parameter_value = value_param;
          subscription->status_callback(&status, subscription->context);
        }

        subscription = &n->paramsubs[it];
        XmlrpcParam param;
        int rc = xmlrpcParamCopy(&param, value_param);
        if (rc != -1)
        {
          xmlrpcParamRelease(&subscription->parameter_value);
          subscription->parameter_value = param;
        }
      }

    PrepareResponse:
      xmlrpcParamVectorPushBackArray(&params);
      XmlrpcParam *array = xmlrpcParamVectorAt(&params, 0);
      xmlrpcParamArrayPushBackInt(array, 1);
      xmlrpcParamArrayPushBackString(array, "");
      if (subscription == NULL)
        xmlrpcParamArrayPushBackInt(array, 1);
      else
        xmlrpcParamArrayPushBackInt(array, 0);
      break;
    }
    case CROS_API_GET_SUBSCRIPTIONS:
    {
      xmlrpcParamVectorPushBackArray(&params);
      XmlrpcParam *array = xmlrpcParamVectorAt(&params, 0);
      xmlrpcParamArrayPushBackInt(array, 1);
      xmlrpcParamArrayPushBackString(array, "");
      XmlrpcParam* param_array = xmlrpcParamArrayPushBackArray(array);

      int i = 0;
      for(i = 0; i < n->n_subs; i++)
      {
        XmlrpcParam* sub_array = xmlrpcParamArrayPushBackArray(param_array);
        xmlrpcParamArrayPushBackString(sub_array, n->subs[i].topic_name);
        xmlrpcParamArrayPushBackString(sub_array, n->subs[i].topic_type);
      }

      break;
    }
    case CROS_API_GET_PUBLICATIONS:
    {
      xmlrpcParamVectorPushBackArray(&params);
      XmlrpcParam *array = xmlrpcParamVectorAt(&params, 0);
      xmlrpcParamArrayPushBackInt(array, 1);
      xmlrpcParamArrayPushBackString(array, "");
      XmlrpcParam* param_array = xmlrpcParamArrayPushBackArray(array);

      int i = 0;
      for(i = 0; i < n->n_pubs; i++)
      {
        XmlrpcParam* sub_array = xmlrpcParamArrayPushBackArray(param_array);
        xmlrpcParamArrayPushBackString(sub_array, n->pubs[i].topic_name);
        xmlrpcParamArrayPushBackString(sub_array, n->pubs[i].topic_type);
      }

      break;
    }
    default:
    {
      PRINT_ERROR("cRosApiParseRequestPrepareResponse() : Unknown method \n Message : \n %s",
                  dynStringGetData( &(server_proc->message)));
      xmlrpcParamVectorPushBackString( &params, "Unknown method");
      break;
    }
  }

  xmlrpcProcessClear(server_proc, 0);
  generateXmlrpcMessage(n->roscore_host, n->xmlrpc_port, server_proc->message_type,
                        dynStringGetData(&server_proc->method), &params, &server_proc->message);
  xmlrpcParamVectorRelease(&params);

  return 0;
}

const char * getMethodName(CrosApiMethod method)
{
  switch (method)
  {
    case CROS_API_NONE:
      return NULL;
    case CROS_API_REGISTER_SERVICE:
      return "registerService";
    case CROS_API_UNREGISTER_SERVICE:
      return "unregisterService";
    case CROS_API_REGISTER_SUBSCRIBER:
      return "registerSubscriber";
    case CROS_API_UNREGISTER_SUBSCRIBER:
      return "unregisterSubscriber";
    case CROS_API_REGISTER_PUBLISHER:
      return "registerPublisher";
    case CROS_API_UNREGISTER_PUBLISHER:
      return "unregisterPublisher";
    case CROS_API_LOOKUP_NODE:
      return "lookupNode";
    case CROS_API_GET_PUBLISHED_TOPICS:
      return "getPublishedTopics";
    case CROS_API_GET_TOPIC_TYPES:
      return "getTopicTypes";
    case CROS_API_GET_SYSTEM_STATE:
      return "getSystemState";
    case CROS_API_GET_URI:
      return "getUri";
    case CROS_API_LOOKUP_SERVICE:
      return "lookupService";
    case CROS_API_GET_BUS_STATS:
      return "getBusStats";
    case CROS_API_GET_BUS_INFO:
      return "getBusInfo";
    case CROS_API_GET_MASTER_URI:
      return "getMasterUri";
    case CROS_API_SHUTDOWN:
      return "shutdown";
    case CROS_API_GET_PID:
      return "getPid";
    case CROS_API_GET_SUBSCRIPTIONS:
      return "getSubscriptions";
    case CROS_API_GET_PUBLICATIONS:
      return "getPublications";
    case CROS_API_PARAM_UPDATE:
      return "paramUpdate";
    case CROS_API_PUBLISHER_UPDATE:
      return "publisherUpdate";
    case CROS_API_REQUEST_TOPIC:
      return "requestTopic";
    case CROS_API_DELETE_PARAM:
      return "deleteParam";
    case CROS_API_SET_PARAM:
      return "setParam";
    case CROS_API_GET_PARAM:
      return "getParam";
    case CROS_API_SEARCH_PARAM:
      return "searchParam";
    case CROS_API_SUBSCRIBE_PARAM:
      return "subscribeParam";
    case CROS_API_UNSUBSCRIBE_PARAM:
      return "unsubscribeParam";
    case CROS_API_HAS_PARAM:
      return "hasParam";
    case CROS_API_GET_PARAM_NAMES:
      return "getParamNames";
    default:
      assert(0);
  }
}

CrosApiMethod getMethodCode(const char *method)
{
  if (strcmp(method, "registerService") == 0)
    return CROS_API_REGISTER_SERVICE;
  else if (strcmp(method, "unregisterService") == 0)
    return CROS_API_UNREGISTER_SERVICE;
  else if (strcmp(method, "registerSubscriber") == 0)
    return CROS_API_REGISTER_SUBSCRIBER;
  else if (strcmp(method, "unregisterSubscriber") == 0)
    return CROS_API_UNREGISTER_SUBSCRIBER;
    else if (strcmp(method, "registerPublisher") == 0)
    return CROS_API_REGISTER_PUBLISHER;
  else if (strcmp(method, "unregisterPublisher") == 0)
    return CROS_API_UNREGISTER_PUBLISHER;
  else if (strcmp(method, "lookupNode") == 0)
    return CROS_API_LOOKUP_NODE;
  else if (strcmp(method, "getPublishedTopics") == 0)
    return CROS_API_GET_PUBLISHED_TOPICS;
  else if (strcmp(method, "getTopicTypes") == 0)
    return CROS_API_GET_TOPIC_TYPES;
  else if (strcmp(method, "getSystemState") == 0)
    return CROS_API_GET_SYSTEM_STATE;
  else if (strcmp(method, "getUri") == 0)
    return CROS_API_GET_URI;
  else if (strcmp(method, "lookupService") == 0)
    return CROS_API_LOOKUP_SERVICE;
  else if (strcmp(method, "getBusStats") == 0)
    return CROS_API_GET_BUS_STATS;
  else if (strcmp(method, "getBusInfo") == 0)
    return CROS_API_GET_BUS_INFO;
  else if (strcmp(method, "getMasterUri") == 0)
    return CROS_API_GET_MASTER_URI;
  else if (strcmp(method, "shutdown") == 0)
    return CROS_API_SHUTDOWN;
  else if (strcmp(method, "getPid") == 0)
    return CROS_API_GET_PID;
  else if (strcmp(method, "getSubscriptions") == 0)
    return CROS_API_GET_SUBSCRIPTIONS;
  else if (strcmp(method, "getPublications") == 0)
    return CROS_API_GET_PUBLICATIONS;
  else if (strcmp(method, "paramUpdate") == 0)
    return CROS_API_PARAM_UPDATE;
  else if (strcmp(method, "publisherUpdate") == 0)
    return CROS_API_PUBLISHER_UPDATE;
  else if (strcmp(method, "requestTopic") == 0)
    return CROS_API_REQUEST_TOPIC;
  else if (strcmp(method, "deleteParam") == 0)
    return CROS_API_DELETE_PARAM;
  else if (strcmp(method, "setParam") == 0)
    return CROS_API_SET_PARAM;
  else if (strcmp(method, "getParam") == 0)
    return CROS_API_GET_PARAM;
  else if (strcmp(method, "searchParam") == 0)
    return CROS_API_SEARCH_PARAM;
  else if (strcmp(method, "subscribeParam") == 0)
    return CROS_API_SUBSCRIBE_PARAM;
  else if (strcmp(method, "unsubscribeParam") == 0)
    return CROS_API_UNSUBSCRIBE_PARAM;
  else if (strcmp(method, "hasParam") == 0)
    return CROS_API_HAS_PARAM;
  else if (strcmp(method, "getParamNames") == 0)
    return CROS_API_GET_PARAM_NAMES;
  else
    return CROS_API_NONE;
}

int isRosMasterApi(CrosApiMethod method)
{
  switch (method)
  {
    case CROS_API_NONE:
      return 0;
    case CROS_API_REGISTER_SERVICE:
    case CROS_API_UNREGISTER_SERVICE:
    case CROS_API_REGISTER_SUBSCRIBER:
    case CROS_API_UNREGISTER_SUBSCRIBER:
    case CROS_API_REGISTER_PUBLISHER:
    case CROS_API_UNREGISTER_PUBLISHER:
    case CROS_API_LOOKUP_NODE:
    case CROS_API_GET_PUBLISHED_TOPICS:
    case CROS_API_GET_TOPIC_TYPES:
    case CROS_API_GET_SYSTEM_STATE:
    case CROS_API_GET_URI:
    case CROS_API_LOOKUP_SERVICE:
      return 1;
    case CROS_API_GET_BUS_STATS:
    case CROS_API_GET_BUS_INFO:
    case CROS_API_GET_MASTER_URI:
    case CROS_API_SHUTDOWN:
    case CROS_API_GET_PID:
    case CROS_API_GET_SUBSCRIPTIONS:
    case CROS_API_GET_PUBLICATIONS:
    case CROS_API_PARAM_UPDATE:
    case CROS_API_PUBLISHER_UPDATE:
    case CROS_API_REQUEST_TOPIC:
      return 0;
    case CROS_API_DELETE_PARAM:
    case CROS_API_SET_PARAM:
    case CROS_API_GET_PARAM:
    case CROS_API_SEARCH_PARAM:
    case CROS_API_SUBSCRIBE_PARAM:
    case CROS_API_UNSUBSCRIBE_PARAM:
    case CROS_API_HAS_PARAM:
    case CROS_API_GET_PARAM_NAMES:
      return 1;
    default:
      assert(0);
  }
}

int isRosSlaveApi(CrosApiMethod method)
{
  switch (method)
  {
    case CROS_API_NONE:
      return 0;
    case CROS_API_REGISTER_SERVICE:
    case CROS_API_UNREGISTER_SERVICE:
    case CROS_API_REGISTER_SUBSCRIBER:
    case CROS_API_UNREGISTER_SUBSCRIBER:
    case CROS_API_REGISTER_PUBLISHER:
    case CROS_API_UNREGISTER_PUBLISHER:
    case CROS_API_LOOKUP_NODE:
    case CROS_API_GET_PUBLISHED_TOPICS:
    case CROS_API_GET_TOPIC_TYPES:
    case CROS_API_GET_SYSTEM_STATE:
    case CROS_API_GET_URI:
    case CROS_API_LOOKUP_SERVICE:
      return 0;
    case CROS_API_GET_BUS_STATS:
    case CROS_API_GET_BUS_INFO:
    case CROS_API_GET_MASTER_URI:
    case CROS_API_SHUTDOWN:
    case CROS_API_GET_PID:
    case CROS_API_GET_SUBSCRIPTIONS:
    case CROS_API_GET_PUBLICATIONS:
    case CROS_API_PARAM_UPDATE:
    case CROS_API_PUBLISHER_UPDATE:
    case CROS_API_REQUEST_TOPIC:
      return 1;
    case CROS_API_DELETE_PARAM:
    case CROS_API_SET_PARAM:
    case CROS_API_GET_PARAM:
    case CROS_API_SEARCH_PARAM:
    case CROS_API_SUBSCRIBE_PARAM:
    case CROS_API_UNSUBSCRIBE_PARAM:
    case CROS_API_HAS_PARAM:
    case CROS_API_GET_PARAM_NAMES:
      return 0;
    default:
      assert(0);
  }
}

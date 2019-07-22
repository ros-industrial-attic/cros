#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef _WIN32
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  define strtok_r strtok_s
#else
#  include <sys/socket.h>
#  include <arpa/inet.h>
#  include <netdb.h>
#endif

#include "cros_node_api.h"
#include "cros_api.h"
#include "cros_api_internal.h"
#include "cros_defs.h"
#include "xmlrpc_params.h"
#include "tcpip_socket.h"

int lookup_host (const char *host, char *ip_addr_buff, size_t ip_addr_buff_size)
{
  struct addrinfo hints, *res, *res_it;
  int errcode;
  int ret;

  memset (&hints, 0, sizeof (hints));
  hints.ai_family = PF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags |= AI_CANONNAME;

  errcode = getaddrinfo (host, NULL, &hints, &res);
  if (errcode != 0)
  {
    PRINT_ERROR ("lookup_host() : getaddrinfo failed and returned the error code:: %i", errcode);
    return -1;
  }

  ret=0; // Default return value=0: no error
  res_it = res;
  while (res_it != NULL && ret == 0)
  {
    void *src_addr = NULL;
    switch (res_it->ai_family)
    {
      case AF_INET:
        src_addr = &((struct sockaddr_in *) res_it->ai_addr)->sin_addr;
        break;
#ifdef AF_INET6
      case AF_INET6:
        src_addr = &((struct sockaddr_in6 *) res_it->ai_addr)->sin6_addr;
        break;
#endif
      default:
        break;
    }
    if (src_addr != NULL)
    {
      if(inet_ntop (res_it->ai_family, src_addr, ip_addr_buff, ip_addr_buff_size) != NULL)
      {
        PRINT_VVDEBUG ("IPv%d address: %s (%s)\n", res_it->ai_family == PF_INET6 ? 6 : 4,
                      ip_addr_buff, res_it->ai_canonname);
        res_it = res_it->ai_next;
      }
      else
      {
        int fn_error_code;

        fn_error_code = tcpIpSocketGetError();
        if(fn_error_code == FN_ERROR_INVALID_PARAMETER)
          PRINT_ERROR ("lookup_host() : buffer for host address too small");
        else
          PRINT_ERROR ("lookup_host() : error executing inet_ntop(). Error code = %i", fn_error_code);
        ret = -1;
      }
    }
    else
    {
      PRINT_ERROR ("lookup_host() : unsupported ai_family from getaddrinfo()");
      ret = -1;
    }
  }
  freeaddrinfo(res);
  return ret;
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
  PRINT_VVDEBUG ( "cRosApiPrepareRequest()\n" );

  XmlrpcProcess *client_proc = &(n->xmlrpc_client_proc[client_idx]);

  client_proc->message_type = XMLRPC_MESSAGE_REQUEST;

  if(client_proc->current_call == NULL)
  {
    PRINT_ERROR ( "cRosApiPrepareRequest() : Invalid call corresponding to XmlrpcProcess\n" );
    return;
  }

  RosApiCall *call = client_proc->current_call;
  if(client_idx == 0) //requests managed by the xmlrpc client connected to roscore
  {
    generateXmlrpcMessage(n->roscore_host, n->roscore_port, XMLRPC_MESSAGE_REQUEST,
                          getMethodName(call->method), &call->params, &client_proc->message);
  }
  else // client_idx > 0
  {
    generateXmlrpcMessage(call->host, call->port, XMLRPC_MESSAGE_REQUEST,
                          getMethodName(call->method), &call->params, &client_proc->message);
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
  PRINT_VVDEBUG ( "cRosApiParseResponse()\n" );
  XmlrpcProcess *client_proc = &(n->xmlrpc_client_proc[client_idx]);
  int ret = -1;

  if( client_proc->current_call == NULL )
  {
    PRINT_ERROR ( "cRosApiParseResponse() : Invalid current call for XMLRPC process\n" );
    return ret;
  }

  RosApiCall *call = client_proc->current_call;
  if(client_idx == 0 && call->user_call == 0) // xmlrpc client connected to roscore (master)
  {
    if( client_proc->message_type != XMLRPC_MESSAGE_RESPONSE )
    {
      PRINT_ERROR ( "cRosApiParseResponse() : Not a response message \n" );
      return ret;
    }
    // xmlrpcParamVectorPrint(&client_proc->response); ////

    switch (call->method)
    {
      case CROS_API_REGISTER_PUBLISHER:
      {
        PRINT_VDEBUG ( "cRosApiParseResponse() : registerPublisher response \n" );
        if( checkResponseValue( &client_proc->response ) )
          ret = 0;

        break;
      }
      case CROS_API_REGISTER_SERVICE:
      {
        PRINT_VDEBUG ( "cRosApiParseResponse() : registerService response \n" );

        if( checkResponseValue( &client_proc->response ) )
          ret = 0;

        break;
      }
      case CROS_API_REGISTER_SUBSCRIBER:
      {
        PRINT_VDEBUG ( "cRosApiParseResponse() : registerSubscriber response \n" );

        //Get the next subscriber without a topic host
        if(call->provider_idx == -1)
        {
          PRINT_ERROR ( "cRosApiParseResponse() : Invalid provider index in call CROS_API_REGISTER_SUBSCRIBER for XMLRPC process\n" );
          return ret;
        }
        int subidx = call->provider_idx;

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
              if (clean_string != NULL)
              {
                char topic_host_addr[MAX_HOST_NAME_LEN+1];
                int topic_host_port;
                strncpy(clean_string,pub_host_string+7,dirty_string_len-8);
                char *progress = NULL;
                char *hostname = strtok_r(clean_string,":",&progress);

                int rc = lookup_host(hostname, topic_host_addr, sizeof(topic_host_addr));
                if (rc == 0)
                {
                  topic_host_port = atoi(strtok_r(NULL,":",&progress));
                  if(enqueueRequestTopic(n, subidx, topic_host_addr, topic_host_port) == -1)
                    ret=-1;
                }
                else
                  ret=-1;
                free(clean_string);
              }
              else
                ret=-1;
            }
          }
        }
        break;
      }
      case CROS_API_LOOKUP_SERVICE:
      {
        PRINT_VDEBUG ( "cRosApiParseResponse() : Lookup Service response\n" );

        //Get the next service caller without a topic host
        if(call->provider_idx == -1)
        {
          PRINT_ERROR ( "cRosApiParseResponse() : Invalid provider index in call CROS_API_LOOKUP_SERVICE for XMLRPC process\n" );
          return ret;
        }

        int srvcalleridx = call->provider_idx;
        ServiceCallerNode* requesting_service_caller = &(n->service_callers[srvcalleridx]);

        TcprosProcess* rpcros_proc = &n->rpcros_client_proc[requesting_service_caller->rpcros_id];
        rpcros_proc->service_idx = call->provider_idx;

        if(checkResponseValue( &client_proc->response ) == 1)
        {
          ret = 0;
          //Check if there is some publishers for the subscription
          XmlrpcParam *param = xmlrpcParamVectorAt(&client_proc->response, 0);
          if(xmlrpcParamArrayGetSize(param) >= 3)
          {
              XmlrpcParam *service_prov_host = xmlrpcParamArrayGetParamAt(param,2);
              char* service_prov_host_string = xmlrpcParamGetString(service_prov_host);
              //manage string for exploit informations
              //removing the 'rosrpc://' and the last '/'
              int dirty_string_len = strlen(service_prov_host_string);
              char* clean_string = (char *)calloc(dirty_string_len-10+1,sizeof(char));
              if (clean_string != NULL)
              {
                strncpy(clean_string,service_prov_host_string+9,dirty_string_len-10);
                char * progress = NULL;
                char* hostname = strtok_r(clean_string,":",&progress);
                if(requesting_service_caller->service_host == NULL)
                {
                  requesting_service_caller->service_host = (char *)calloc(MAX_HOST_NAME_LEN+1,sizeof(char)); //deleted in cRosNodeDestroy
                }

                if (requesting_service_caller->service_host != NULL)
                {
                  int rc = lookup_host(hostname, requesting_service_caller->service_host, (MAX_HOST_NAME_LEN+1)*sizeof(char));
                  if (rc == 0)
                  {
                    requesting_service_caller->service_port = atoi(strtok_r(NULL,":",&progress));

                    //need to be checked because maybe the connection went down suddenly.
                    if(!rpcros_proc->socket.open)
                    {
                      tcpIpSocketOpen(&(rpcros_proc->socket));
                    }

                    PRINT_VDEBUG( "cRosApiParseResponse() : Lookup Service response [tcp port: %d]\n", requesting_service_caller->service_port);

                    //set the process to open the socket with the desired host
                    tcprosProcessChangeState(rpcros_proc, TCPROS_PROCESS_STATE_CONNECTING);
                  }
                  else
                    ret=-1;
                }
                else
                  ret=-1;
                free(clean_string);
              }
              else
                ret=-1;
          }
          else
          {
             PRINT_ERROR ( "cRosApiParseResponse() : Invalid response from ROS master when Looking up services (Not enough parameters in response)\n");
             ret=-1;
          }
        }
        else
        {
            tcprosProcessChangeState(rpcros_proc, TCPROS_PROCESS_STATE_WAIT_FOR_CONNECTING);
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
          // XmlrpcParam *status_msg = xmlrpcParamArrayGetParamAt(array, 1);
          XmlrpcParam *value = xmlrpcParamArrayGetParamAt(array, 2);

          XmlrpcParam copy;
          int rc = xmlrpcParamCopy(&copy, value);
          if (rc < 0)
            break;

          subscription = &n->paramsubs[paramsubidx];

          CrosNodeStatusUsr status;
          initCrosNodeStatus(&status);
          status.state = CROS_STATUS_PARAM_SUBSCRIBED;
          status.provider_idx = paramsubidx;
          status.parameter_key = subscription->parameter_key;
          status.parameter_value = value;
          subscription->status_api_callback(&status, subscription->context); // calls the parameter-subscriber application-defined status callback function (if specified when creating the publisher).

          ret = 0;
          xmlrpcParamRelease(&subscription->parameter_value);
          subscription->parameter_value = copy;
        }

        break;
      }
      case CROS_API_GET_PID:
      {
        PRINT_VDEBUG ( "cRosApiParseResponse() : get PID response \n" );

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
        PRINT_VDEBUG ( "cRosApiParseResponse() : unregister service response \n" );

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
      case CROS_API_UNREGISTER_PUBLISHER:
      {
        PRINT_VDEBUG ( "cRosApiParseResponse() : unregister publisher response \n" );
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
        PRINT_VDEBUG ( "cRosApiParseResponse() : unregister subscriber response \n" );
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
         PRINT_ERROR ( "cRosApiParseResponse() : Invalid call method for XML RPC process connected to ROS master\n" );
         ret=-1;
      }
    }
  }
  else // client_idx > 0 || user_call = 1
  {
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

        // xmlrpcParamVectorPrint(&client_proc->response); ////

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
          int client_tcpros_ind;
          char tcpros_host[MAX_HOST_NAME_LEN+1];
          ret = 0;

          XmlrpcParam* param_array = xmlrpcParamVectorAt(&client_proc->response,0);
          XmlrpcParam* nested_array = xmlrpcParamArrayGetParamAt(param_array,2);
          XmlrpcParam* tcp_host = xmlrpcParamArrayGetParamAt(nested_array,1);
          XmlrpcParam* tcp_port = xmlrpcParamArrayGetParamAt(nested_array,2);

          RosApiCall *call = client_proc->current_call;
          int sub_ind = call->provider_idx;

          int tcp_port_print = tcp_port->data.as_int;
          SubscriberNode* sub = &n->subs[sub_ind];

          PRINT_VDEBUG( "cRosApiParseResponse() : requestTopic response [tcp port: %d]\n", tcp_port_print);
          xmlrpcProcessChangeState(client_proc,XMLRPC_PROCESS_STATE_IDLE);

          int rc = lookup_host(tcp_host->data.as_string, tcpros_host, sizeof(tcpros_host));
          if (rc == 0)
          {
            // Check if a Tcpros client is already connected to the received hostname and port for the current subscriber node
            client_tcpros_ind = cRosNodeFindFirstTcprosClientProc(n, sub_ind, tcpros_host, tcp_port_print);
            if(client_tcpros_ind == -1) // No Tcpros client is already connected to this hostname and port for the current subscriber node, recruit one:
            {
              client_tcpros_ind = cRosNodeRecruitTcprosClientProc(n, sub_ind);
              if(client_tcpros_ind != -1) // A Tcpros client has been recruited to be used
              {
                TcprosProcess* tcpros_proc = &n->tcpros_client_proc[client_tcpros_ind];
                tcpros_proc->topic_idx = sub_ind;

                // need to be checked because maybe the connection went down suddenly.
                if(!tcpros_proc->socket.open)
                  tcpIpSocketOpen(&(tcpros_proc->socket));

                // set the process to open the socket with the desired host
                tcpros_proc->sub_tcpros_host = (char *)realloc(tcpros_proc->sub_tcpros_host, strlen(tcpros_host)+sizeof(char)); // If already allocated, realloc() does nothing
                if (tcpros_proc->sub_tcpros_host != NULL)
                {
                  strcpy(tcpros_proc->sub_tcpros_host, tcpros_host);
                  tcpros_proc->sub_tcpros_port = tcp_port_print;
                  tcprosProcessChangeState(tcpros_proc, TCPROS_PROCESS_STATE_CONNECTING);
                  // printf("HOST: %s:%i\n",tcpros_proc->sub_tcpros_host,tcpros_proc->sub_tcpros_port);
                }
                else
                {
                  PRINT_ERROR ( "cRosApiParseResponse() : Not enough memory allocating the publisher hostname string\n");
                  ret=-1;
                }
              }
              else
              {
                PRINT_ERROR ( "cRosApiParseResponse() : No TCPROS client process is available to be used to subscribe to the topic. Allocate more TCPROS client processes.\n");
                ret=-1;
              }
            }
          }
          else
          {
            PRINT_ERROR ( "cRosApiParseResponse() : Cannot resolve the publisher hostname received in response of REQUEST_TOPIC\n");
            ret=-1;
          }
        }
        break;
      }
      default:
      {
         PRINT_ERROR ( "cRosApiParseResponse() : Invalid call method for XML RPC process connected to ROS node\n" );
         ret=-1;
      }
    }
  }

  return ret;
}

// return value is different from 0 only when a response message cannot be generated
int cRosApiParseRequestPrepareResponse( CrosNode *n, int server_idx )
{
  int ret;
  PRINT_VDEBUG ( "cRosApiParseRequestPrepareResponse()\n" );

  XmlrpcProcess *server_proc = &(n->xmlrpc_server_proc[server_idx]);

  if( server_proc->message_type != XMLRPC_MESSAGE_REQUEST)
  {
    PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Not a request message \n" );
    return -1;
  }

  ret=0; // Default return value
  server_proc->message_type = XMLRPC_MESSAGE_RESPONSE;

  XmlrpcParamVector params;
  xmlrpcParamVectorInit(&params);

  CrosApiMethod method = getMethodCode(dynStringGetData(&server_proc->method));
  switch (method)
  {
    case CROS_API_GET_PID:
    {
      xmlrpcParamVectorPushBackArray(&params);
      XmlrpcParam *array = xmlrpcParamVectorAt(&params, 0);
      xmlrpcParamArrayPushBackInt(array, 1);
      xmlrpcParamArrayPushBackString(array, "");
      xmlrpcParamArrayPushBackInt(array, n->pid);
      break;
    }
    case CROS_API_PUBLISHER_UPDATE:
    {
      PRINT_VDEBUG("publisherUpdate()\n");
      // TODO Store the subscribed node name
      XmlrpcParam *caller_id_param, *topic_param, *publishers_param;

#if CROS_DEBUG_LEVEL >= 3
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
        int array_size;
        char *topic_name_param;
        XmlrpcParam *uri;
        int sub_idx = -1;
        int uri_found = 0;
        int i;

        topic_name_param = xmlrpcParamGetString( topic_param );
        array_size = xmlrpcParamArrayGetSize( publishers_param );

        for(i = 0; i < n->n_subs; i++)
        {
          if (n->subs[i].topic_name == NULL)
            continue;

          if( strcmp( topic_name_param, n->subs[i].topic_name ) == 0)
          {
            sub_idx = i;
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

        // The condition to be met is:
        // Topic name must be registered and
        // either some node is unregistering the topic or
        // the host is found and the tcpros port is not assigned
        if (sub_idx != -1 && (array_size == 0 || uri_found))
        {
          // Subscriber that is still waiting for a tcpros connection found
          publishers_param = xmlrpcParamVectorAt(&server_proc->params, 2);
          int available_pubs_n = xmlrpcParamArrayGetSize(publishers_param);

          {
            int pub_host_ind;
            // Iterate through all the hosts while no error occurs
            for(pub_host_ind=0;pub_host_ind<available_pubs_n && ret==0;pub_host_ind++)
            {
              XmlrpcParam* pub_host = xmlrpcParamArrayGetParamAt(publishers_param, pub_host_ind);
              char* pub_host_string = xmlrpcParamGetString(pub_host);
              //manage string for exploit informations
              //removing the 'http://' and the last '/'
              int dirty_string_len = strlen(pub_host_string);
              char* clean_string = (char *)calloc(dirty_string_len-8+1,sizeof(char));
              if (clean_string != NULL)
              {
                char topic_host_addr[MAX_HOST_NAME_LEN+1];
                int topic_host_port;
                strncpy(clean_string,pub_host_string+7,dirty_string_len-8);
                char * progress = NULL;
                char* hostname = strtok_r(clean_string,":",&progress);

                int rc = lookup_host(hostname, topic_host_addr, sizeof(topic_host_addr));
                if (rc == 0)
                {
                  topic_host_port = atoi(strtok_r(NULL,":",&progress));
                  if(enqueueRequestTopic(n, sub_idx, topic_host_addr, topic_host_port) == -1)
                  {
                    PRINT_ERROR ( "enqueueRequestTopic() : Unable to enqueue request (Not enough memory?)\n" );
                  }
                }
                else
                {
                  PRINT_ERROR ( "lookup_host() : Unable to resolve hostname\n" );
                  xmlrpcParamVectorPushBackString( &params, "Unable to resolve hostname" );
                }
                free(clean_string);
              }
              else
              {
                PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Error allocating memory\n" );
                ret=-1;
              }
            }
          }
          if(ret == 0) // No errors so far
          {
            xmlrpcParamVectorPushBackArray(&params);
            XmlrpcParam *array = xmlrpcParamVectorAt(&params, 0);
            xmlrpcParamArrayPushBackInt(array, 1);
            xmlrpcParamArrayPushBackString(array, "");
            xmlrpcParamArrayPushBackInt(array, 0);
          }
        }
        else
        {
          if(sub_idx == -1)
            PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Unknown topic name in call: %s\n", topic_name_param );
          else
            PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Host parameter not found\n" );

          xmlrpcParamVectorPushBackString( &params, "Unknown topic in publisherUpdate(), host not found or more than one publisher per topic" );
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
            if (strlen(server_proc->host) != 0)
            {
              CrosNodeStatusUsr status;
              initCrosNodeStatus(&status);
              status.xmlrpc_host = server_proc->host;
              status.xmlrpc_port = server_proc->port;
              cRosNodeStatusCallback(&status, pub->context); // calls the publisher-status application-defined callback function (if specified when creating the publisher). Undocumented status callback?
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
      int proc_idx;
      char tcpros_url_msg[MAX_HOST_NAME_LEN+60];
      XmlrpcParam *ret_parm_arr, *ret_businfo_arr, *ret_connect_arr;

      // Answer the transport/topic connection information
      xmlrpcParamVectorPushBackArray(&params);
      ret_parm_arr = xmlrpcParamVectorAt(&params, 0);
      if(ret_parm_arr != NULL)
      {
        xmlrpcParamArrayPushBackInt(ret_parm_arr, 1);
        xmlrpcParamArrayPushBackString(ret_parm_arr, "");
        ret_businfo_arr = xmlrpcParamArrayPushBackArray(ret_parm_arr);
        if(ret_businfo_arr == NULL)
          ret=-1;
      }
      else
        ret=-1;

      for(proc_idx=0;proc_idx<CN_MAX_TCPROS_CLIENT_CONNECTIONS && ret==0;proc_idx++)
      {
        TcprosProcess *cur_cli_proc = &n->tcpros_client_proc[proc_idx];
        if(cur_cli_proc->topic_idx != -1)
        {
          ret_connect_arr = xmlrpcParamArrayPushBackArray(ret_businfo_arr);
          if(ret_connect_arr != NULL)
          {
            xmlrpcParamArrayPushBackInt(ret_connect_arr, proc_idx);
            snprintf(tcpros_url_msg, sizeof(tcpros_url_msg), "http://%s:%hu/", tcpIpSocketGetRemoteAddress(&cur_cli_proc->socket), tcpIpSocketGetRemotePort(&cur_cli_proc->socket));
 // This URL could also be the name of the node that receives the topic messages
            xmlrpcParamArrayPushBackString(ret_connect_arr, tcpros_url_msg);
 // In this case, this node is the subscriber, so the connection direction marked as inbound. This means that data (messages) is transmitted
 // from other node to this node: published -> subscriber (although the connection is established from this node to other node: subscriber -> publisher)
            xmlrpcParamArrayPushBackString(ret_connect_arr, "i");
            xmlrpcParamArrayPushBackString(ret_connect_arr, "TCPROS");
            xmlrpcParamArrayPushBackString(ret_connect_arr, dynStringGetData(&cur_cli_proc->topic));
            xmlrpcParamArrayPushBackInt(ret_connect_arr, 1);
            snprintf(tcpros_url_msg, sizeof(tcpros_url_msg), "TCPROS connection on port %hu to [%s:%hu on socket %i]", tcpIpSocketGetPort(&cur_cli_proc->socket), cur_cli_proc->sub_tcpros_host, cur_cli_proc->sub_tcpros_port, tcpIpSocketGetFD(&cur_cli_proc->socket));
 // example of tcpros_url_msg sniffed from a ROS subscriber node (/rosout): TCPROS connection on port 55636 to [host_name:49463 on socket 14]
 // 55636 is the TCPROS local port in this (subscriber) node that is connected to the listening port of the other (remote) node (publisher)
 // host_name is the address of the other (remote node), which is the publisher. In this case it is also a local address
 // 49463 is the listening port of the other (remote) node, which is the publisher node
 // 14 is the local file despcriptor of this TCPROS connection, that is a file descriptor of the /rousout node process
            xmlrpcParamArrayPushBackString(ret_connect_arr, tcpros_url_msg);
          }
          else
            ret=-1;
        }
      }

      for(proc_idx=0;proc_idx<CN_MAX_TCPROS_SERVER_CONNECTIONS && ret==0;proc_idx++)
      {
        TcprosProcess *cur_ser_proc = &n->tcpros_server_proc[proc_idx];
        if(cur_ser_proc->topic_idx != -1)
        {
          ret_connect_arr = xmlrpcParamArrayPushBackArray(ret_businfo_arr);
          if(ret_connect_arr != NULL)
          {
            xmlrpcParamArrayPushBackInt(ret_connect_arr, proc_idx);
            snprintf(tcpros_url_msg, sizeof(tcpros_url_msg), "http://%s:%hu/", tcpIpSocketGetRemoteAddress(&cur_ser_proc->socket), tcpIpSocketGetRemotePort(&cur_ser_proc->socket));
            xmlrpcParamArrayPushBackString(ret_connect_arr, tcpros_url_msg);
 // In this case, this node is the publisher, so the connection direction is marked as outbound. This means that data (messages) is transmitted
 // from this node to other node: published -> subscriber (although the connection is established from other node to this node: subscriber -> publisher)
            xmlrpcParamArrayPushBackString(ret_connect_arr, "o");
            xmlrpcParamArrayPushBackString(ret_connect_arr, "TCPROS");
            xmlrpcParamArrayPushBackString(ret_connect_arr, dynStringGetData(&cur_ser_proc->topic));
            xmlrpcParamArrayPushBackInt(ret_connect_arr, 1);
            snprintf(tcpros_url_msg, sizeof(tcpros_url_msg), "TCPROS connection on port %hu to [%s:%hu on socket %i]", tcpIpSocketGetPort(&cur_ser_proc->socket), tcpIpSocketGetRemoteAddress(&cur_ser_proc->socket), tcpIpSocketGetRemotePort(&cur_ser_proc->socket), tcpIpSocketGetFD(&cur_ser_proc->socket));
 // example of tcpros_url_msg sniffed from a ROS publisher node (/turtlesim): TCPROS connection on port 49463 to [127.0.0.1:55636 on socket 26]
 // 49463 is the TCPROS local listening port of this (publisher) node, which received the incoming connection
 // 127.0.0.1 is the address of the other (remote node). In this case it is also a local address
 // 55636 is the other-(remote)-node port, which corresponds to the socket created for this TCPROS connection
 // 26 is the local file despcriptor of this TCPROS connection
 // So this XML call returns information about the TCPROS connection between publisher and subscriber, although this information is tramsitted through the XMLRPC port connetion
            xmlrpcParamArrayPushBackString(ret_connect_arr, tcpros_url_msg);
          }
          else
            ret=-1;
        }
      }

//      xmlrpcParamVectorPrint( &params );
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
      PRINT_INFO ( "cRosApiParseRequestPrepareResponse() : Received shutdown request through ROS slave API. This call will be ignored.\n" );
      break;
    }
    case CROS_API_PARAM_UPDATE:
    {
      ParameterSubscription* subscription = NULL;

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

        if (strncmp(parameter_key, n->paramsubs[it].parameter_key, strlen(n->paramsubs[it].parameter_key)) == 0)
        {
          paramsubidx = it;

          break;
        }
      }

      if (paramsubidx != -1)
      {
        subscription = &n->paramsubs[it];
        CrosNodeStatusUsr status;
        initCrosNodeStatus(&status);
        status.state = CROS_STATUS_PARAM_UPDATE;
        status.provider_idx = it;
        status.parameter_key = parameter_key;
        status.parameter_value = value_param;
        subscription->status_api_callback(&status, subscription->context); // calls the parameter-subscriber-status application-defined callback function (if specified when creating the subscriber).

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

  xmlrpcProcessClear(server_proc);
  generateXmlrpcMessage(NULL, 0, server_proc->message_type,
                        dynStringGetData(&server_proc->method), &params, &server_proc->message); // host and port are not used in XMLRPC_MESSAGE_RESPONSE
  xmlrpcParamVectorRelease(&params);

  return ret;
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
      PRINT_ERROR( "getMethodName() : Invalid CrosApiMethod value specified\n" );
      return NULL;
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
      PRINT_ERROR ( "isRosMasterApi() : Invalid CrosApiMethod value specified\n" );
      return -1;
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
      PRINT_ERROR ( "isRosSlaveApi() : Invalid CrosApiMethod value specified\n" );
      return -1;
  }
}

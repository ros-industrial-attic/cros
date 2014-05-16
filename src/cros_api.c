#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "cros_api.h"
#include "cros_defs.h"

enum
{
  CROS_API_GET_PID,
  CROS_API_GET_PUBLISHED_TOPICS,
  CROS_API_REGISTER_PUBLISHER,
  CROS_API_REGISTER_SUBSCRIBER,
  CROS_API_REGISTER_SERVICE,
  CROS_API_REQUEST_TOPIC,
  CROS_API_UNREGISTER_PUBLISHER,
  CROS_API_UNREGISTER_SUBSCRIBER,
  CROS_API_UNREGISTER_SERVICE,
};

static const char *CROS_API_TCPROS_STRING = "TCPROS";

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

static void fillErrorParams ( XmlrpcParamVector *params, char *err_msg )
{
  xmlrpcParamVectorPushBackInt( params, 0 );  
  xmlrpcParamVectorPushBackString( params, err_msg );
  xmlrpcParamVectorPushBackInt( params, 0 );  
}

static void restartAdversing (CrosNode* n)
{
  if( n->n_pubs )
  {
    n->state = (CrosNodeState)(n->state | CN_STATE_ADVERTISE_PUBLISHER);
    n->n_advertised_pubs = 0;
  }

  if( n->n_subs )
  {
    n->state = (CrosNodeState)(n->state | CN_STATE_ADVERTISE_SUBSCRIBER);
    n->n_advertised_subs = 0;
  }

  if( n->n_services )
  {
    n->state = (CrosNodeState)(n->state | CN_STATE_ADVERTISE_SERVICE);
    n->n_advertised_services = 0;
  }
}

void cRosApiPrepareRequest( CrosNode *n, int client_idx )
{
  PRINT_VDEBUG ( "cRosApiPrepareRequest()\n" );
  
  XmlrpcProcess *client_proc = &(n->xmlrpc_client_proc[client_idx]);
  
  xmlrpcProcessClear( client_proc );

  client_proc->message_type = XMLRPC_MESSAGE_REQUEST;

  if(client_idx == 0) //requests managed by the xmlrpc client connected to roscore
  {
    if( n->state & CN_STATE_ADVERTISE_PUBLISHER && n->n_pubs )
    {
      dynStringPushBackStr( &(client_proc->method), "registerPublisher" );

      xmlrpcParamVectorPushBackString( &(client_proc->params), n->name );
      xmlrpcParamVectorPushBackString( &(client_proc->params), n->pubs[n->n_advertised_pubs].topic_name );
      xmlrpcParamVectorPushBackString( &(client_proc->params), n->pubs[n->n_advertised_pubs].topic_type );
      char node_uri[256];
      sprintf( node_uri, "http://%s:%d/", n->host, n->xmlrpc_port);
      xmlrpcParamVectorPushBackString( &(client_proc->params), node_uri );

      client_proc->request_id = CROS_API_REGISTER_PUBLISHER;
      generateXmlrpcMessage( n->host, n->roscore_port, client_proc->message_type,
                          &(client_proc->method), &(client_proc->params), &(client_proc->message) );

    }
    else if( n->state & CN_STATE_ADVERTISE_SUBSCRIBER && n->n_subs )
    {
    PRINT_DEBUG("cRosApiPrepareRequest() : registerSubscriber\n");
    dynStringPushBackStr( &(client_proc->method), "registerSubscriber" );

    xmlrpcParamVectorPushBackString( &(client_proc->params), n->name );
    xmlrpcParamVectorPushBackString( &(client_proc->params), n->subs[n->n_advertised_subs].topic_name );
    xmlrpcParamVectorPushBackString( &(client_proc->params), n->subs[n->n_advertised_subs].topic_type );
    char node_uri[256];
    sprintf( node_uri, "http://%s:%d/", n->host, n->xmlrpc_port);
    xmlrpcParamVectorPushBackString( &(client_proc->params), node_uri );

    client_proc->request_id = CROS_API_REGISTER_SUBSCRIBER;

    generateXmlrpcMessage( n->host, n->roscore_port, client_proc->message_type,
                        &(client_proc->method), &(client_proc->params), &(client_proc->message) );
    }
    else if( n->state & CN_STATE_ADVERTISE_SERVICE && n->n_services )
    {
      PRINT_INFO("cRosApiPrepareRequest() : registerService\n");
      dynStringPushBackStr( &(client_proc->method), "registerService" );

      xmlrpcParamVectorPushBackString( &(client_proc->params), n->name );
      xmlrpcParamVectorPushBackString( &(client_proc->params), n->services[n->n_advertised_services].service_name );
      char uri[256];
      sprintf( uri, "rosrpc://%s:%d/", n->host, n->rpcros_port);
      xmlrpcParamVectorPushBackString( &(client_proc->params), uri );
      sprintf( uri, "http://%s:%d/", n->host, n->xmlrpc_port);
      xmlrpcParamVectorPushBackString( &(client_proc->params), uri );

      client_proc->request_id = CROS_API_REGISTER_SERVICE;

      generateXmlrpcMessage( n->host, n->roscore_port, client_proc->message_type,
                          &(client_proc->method), &(client_proc->params), &(client_proc->message) );
    }
    else if( n->state & CN_STATE_ROSCORE_REQ)
    {
      switch(client_proc->request_id)
      {
        case CROS_API_UNREGISTER_SERVICE:
        {
          PRINT_INFO("cRosApiPrepareRequest() : unregisterService\n");
          dynStringPushBackStr( &(client_proc->method), "unregisterService" );

          DynString service_name; /* here will be filled the service name to be unsubscribed */
          xmlrpcParamVectorPushBackString( &(client_proc->params), n->name );
          xmlrpcParamVectorPushBackString( &(client_proc->params), service_name.data );
          char uri[256];
          sprintf( uri, "rosrpc://%s:%d/", n->host, n->rpcros_port);
          xmlrpcParamVectorPushBackString( &(client_proc->params), uri );

          generateXmlrpcMessage( n->host, n->roscore_port, client_proc->message_type,
                              &(client_proc->method), &(client_proc->params), &(client_proc->message) );
          break;
        }
        case CROS_API_UNREGISTER_SUBSCRIBER:
        {
          PRINT_INFO("cRosApiPrepareRequest() : unregisterSubscriber\n");
          dynStringPushBackStr( &(client_proc->method), "unregisterSubscriber" );

          DynString sub_name; /* here will be filled the subscriber name to be unsubscribed */
          xmlrpcParamVectorPushBackString( &(client_proc->params), n->name );
          xmlrpcParamVectorPushBackString( &(client_proc->params), sub_name.data );
          char uri[256];
          sprintf( uri, "http://%s:%d/", n->host, n->xmlrpc_port);
          xmlrpcParamVectorPushBackString( &(client_proc->params), uri );

          generateXmlrpcMessage( n->host, n->roscore_port, client_proc->message_type,
                              &(client_proc->method), &(client_proc->params), &(client_proc->message) );
          break;
        }
        case CROS_API_UNREGISTER_PUBLISHER:
        {
          PRINT_INFO("cRosApiPrepareRequest() : unregisterPublisher\n");
          dynStringPushBackStr( &(client_proc->method), "unregisterPublisher" );

          DynString pub_name; /* here will be filled the publisher name to be unsubscribed */
          xmlrpcParamVectorPushBackString( &(client_proc->params), n->name );
          xmlrpcParamVectorPushBackString( &(client_proc->params), pub_name.data );
          char uri[256];
          sprintf( uri, "http://%s:%d/", n->host, n->xmlrpc_port);
          xmlrpcParamVectorPushBackString( &(client_proc->params), uri );

          generateXmlrpcMessage( n->host, n->roscore_port, client_proc->message_type,
                              &(client_proc->method), &(client_proc->params), &(client_proc->message) );
          break;
        }
        case CROS_API_GET_PUBLISHED_TOPICS:
        {
          PRINT_INFO("cRosApiPrepareRequest() : getPublishedTopics\n");
          dynStringPushBackStr( &(client_proc->method), "getPublishedTopics" );

          DynString subgraph; /* here will be filled with a string topic names filter */
          xmlrpcParamVectorPushBackString( &(client_proc->params), n->name );
          xmlrpcParamVectorPushBackString( &(client_proc->params), subgraph.data );

          generateXmlrpcMessage( n->host, n->roscore_port, client_proc->message_type,
                              &(client_proc->method), &(client_proc->params), &(client_proc->message) );
          break;
        }
      }
    }
    else
    {
      PRINT_DEBUG("cRosApiPrepareRequest() : ping roscore\n");

      // Default behavior: ping roscore (actually, ping a node of roscore, i.e. default /rosout )

      dynStringPushBackStr( &(client_proc->method), "getPid" );
      xmlrpcParamVectorPushBackString( &(client_proc->params), "/rosout");

      client_proc->request_id = CROS_API_GET_PID;

      generateXmlrpcMessage( n->host, n->roscore_port, client_proc->message_type,
                          &(client_proc->method), &(client_proc->params), &(client_proc->message) );
    }
  }
  else
  {
    if( n->state & CN_STATE_ASK_FOR_CONNECTION)
    {
      //1)check if there are other xmlrpc processes waiting for connection response then skip
      //2)if none is waiting a response look for if there are subscribers without requestTopic call performed yet.
      //3)if yes, erase the state CN_STATE_ASK_FOR_CONNECTION
      //4)if not, start a new requestTopic workflow

      int i;
      SubscriberNode* subscriber_node = NULL;

      for(i = 0; i < n->n_subs; i++)
      {
        //this xmlrpc process is associated to a subscriber but the requestTopic call is not done yet.
        if(n->subs[i].client_xmlrpc_id == client_idx && n->subs[i].tcpros_port == -1)
        {
          subscriber_node = &(n->subs[i]);
        }
      }

      if(subscriber_node != NULL)
      {
        PRINT_INFO("cRosApiPrepareRequest() : requestTopic\n");
        dynStringPushBackStr( &(client_proc->method), "requestTopic" );

        xmlrpcParamVectorPushBackString( &(client_proc->params), n->name );
        xmlrpcParamVectorPushBackString( &(client_proc->params), subscriber_node->topic_name );
        xmlrpcParamVectorPushBackArray( &(client_proc->params));
        XmlrpcParam*  array_param =  xmlrpcParamVectorAt(&(client_proc->params),2);
        xmlrpcParamArrayPushBackArray(array_param);
        xmlrpcParamArrayPushBackString(xmlrpcParamArrayGetParamAt(array_param,0),CROS_API_TCPROS_STRING);

        client_proc->request_id = CROS_API_REQUEST_TOPIC;

        generateXmlrpcMessage(n->host, subscriber_node->topic_port, client_proc->message_type,
                              &(client_proc->method), &(client_proc->params), &(client_proc->message) );
      }
    }
  }
}


/*
 * XMLRPC Return values are lists in the format: [statusCode, statusMessage, value]
 *
 * statusCode (int): An integer indicating the completion condition of the method. Current values:
 *      -1: ERROR: Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.
 *       0: FAILURE: Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.
 *       1: SUCCESS: Method completed successfully.
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
  const char *method = dynStringGetData( &(client_proc->method) );
  int ret = 0;

  if(client_idx == 0) //xmlrpc client connected to roscore
  {
    if( client_proc->message_type != XMLRPC_MESSAGE_RESPONSE )
    {
      PRINT_ERROR ( "cRosApiParseResponse() : Not a response message \n" );
    }
    else if( client_proc->request_id == CROS_API_REGISTER_PUBLISHER )
    {
      PRINT_DEBUG ( "cRosApiParseResponse() : registerPublisher response \n" );
      //xmlrpcParamVectorPrint( &(client_proc->params) );
      if( checkResponseValue( &(client_proc->params) ) )
      {
        ret = 1;
        if(++(n->n_advertised_pubs) >= n->n_pubs )
          n->state = (CrosNodeState)(n->state & ~CN_STATE_ADVERTISE_PUBLISHER);
      }
    }
    else if( client_proc->request_id == CROS_API_REGISTER_SERVICE )
    {
      PRINT_DEBUG ( "cRosApiParseResponse() : registerService response \n" );

      if( checkResponseValue( &(client_proc->params) ) )
      {
        ret = 1;
        if(++(n->n_advertised_services) >= n->n_services )
          n->state = (CrosNodeState)(n->state & ~CN_STATE_ADVERTISE_SERVICE);
      }
    }
    else if( client_proc->request_id == CROS_API_REGISTER_SUBSCRIBER )
    {
      PRINT_DEBUG ( "cRosApiParseResponse() : registerSubscriber response \n" );
      //xmlrpcParamVectorPrint( &(client_proc->params) );

      //Get the next subscriber without a topic host
      SubscriberNode* requesting_subscriber = NULL;
      int i ;
      requesting_subscriber = &(n->subs[n->n_advertised_subs]);

      //Check if there is some publishers for the subscription
      XmlrpcParam *param = xmlrpcParamVectorAt(&(client_proc->params), 0);
      XmlrpcParam *array = xmlrpcParamArrayGetParamAt(param,2);
      int available_pubs_n = xmlrpcParamArrayGetSize(array);

      if(available_pubs_n > 0)
      {
        for(i = 0; i < available_pubs_n; i++)
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
          lookup_host(hostname, requesting_subscriber->topic_host);
          requesting_subscriber->topic_port = atoi(strtok_r(NULL,":",&progress));
        }
      }

      if( checkResponseValue( &(client_proc->params) ) )
      {
        ret = 1;
        if(++(n->n_advertised_subs) >= n->n_subs )
        {
          n->state = (CrosNodeState)(n->state & ~CN_STATE_ADVERTISE_SUBSCRIBER);
          n->state = (CrosNodeState)(n->state | CN_STATE_ASK_FOR_CONNECTION);
          for(i = 0; i < n->n_subs; i++)
          {
            //wake up all the xmlrpc clients able to do the requestTopic call
            if(n->subs[i].topic_host != NULL && n->subs[i].topic_port != -1)
            {
              xmlrpcProcessChangeState(&(n->xmlrpc_client_proc[n->subs[i].client_xmlrpc_id]),XMLRPC_PROCESS_STATE_WRITING);
            }
          }
        }
      }
    }
    else if( client_proc->request_id == CROS_API_GET_PID )
    {
      PRINT_DEBUG ( "cRosApiParseResponse() : ping response \n" );
      //xmlrpcParamVectorPrint( &(client_proc->params) );

      if( checkResponseValue( &(client_proc->params) ) )
      {
        ret = 1;
        XmlrpcParam* roscore_pid_param =
        		xmlrpcParamArrayGetParamAt(xmlrpcParamVectorAt(&(client_proc->params),0),2);

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
    }
    else if( client_proc->request_id == CROS_API_UNREGISTER_SERVICE )
    {
      PRINT_DEBUG ( "cRosApiParseResponse() : ping response \n" );
      //xmlrpcParamVectorPrint( &(client_proc->params) );

      if( checkResponseValue( &(client_proc->params) ) )
      {
        ret = 1;
        XmlrpcParam* status_msg = xmlrpcParamVectorAt( &(client_proc->params), 1);
        XmlrpcParam* unreg_num = xmlrpcParamVectorAt( &(client_proc->params), 2);
      }

      n->state = (CrosNodeState)(n->state & ~CN_STATE_ROSCORE_REQ);
      xmlrpcProcessChangeState(client_proc,XMLRPC_PROCESS_STATE_IDLE);
    }
    else if( client_proc->request_id == CROS_API_UNREGISTER_PUBLISHER )
    {
      PRINT_DEBUG ( "cRosApiParseResponse() : ping response \n" );
      //xmlrpcParamVectorPrint( &(client_proc->params) );

      if( checkResponseValue( &(client_proc->params) ) )
      {
        ret = 1;
        XmlrpcParam* status_msg = xmlrpcParamVectorAt( &(client_proc->params), 1);
        XmlrpcParam* unreg_num = xmlrpcParamVectorAt( &(client_proc->params), 2);
      }
      n->state = (CrosNodeState)(n->state & ~CN_STATE_ROSCORE_REQ);
      xmlrpcProcessChangeState(client_proc,XMLRPC_PROCESS_STATE_IDLE);
    }
    else if( client_proc->request_id == CROS_API_UNREGISTER_SUBSCRIBER )
    {
      PRINT_DEBUG ( "cRosApiParseResponse() : ping response \n" );
      //xmlrpcParamVectorPrint( &(client_proc->params) );

      if( checkResponseValue( &(client_proc->params) ) )
      {
        ret = 1;
        XmlrpcParam* status_msg = xmlrpcParamVectorAt( &(client_proc->params), 1);
        XmlrpcParam* unreg_num = xmlrpcParamVectorAt( &(client_proc->params), 2);
      }
      n->state = (CrosNodeState)(n->state & ~CN_STATE_ROSCORE_REQ);
      xmlrpcProcessChangeState(client_proc,XMLRPC_PROCESS_STATE_IDLE);
    }
    else if( client_proc->request_id == CROS_API_GET_PUBLISHED_TOPICS )
    {
      PRINT_DEBUG ( "cRosApiParseResponse() : ping response \n" );
      //xmlrpcParamVectorPrint( &(client_proc->params) );

      if( checkResponseValue( &(client_proc->params) ) )
      {
        ret = 1;
        XmlrpcParam* status_msg = xmlrpcParamVectorAt( &(client_proc->params), 1);
        XmlrpcParam* topic_arr = xmlrpcParamVectorAt( &(client_proc->params), 2); //[[topic_name, topic_type]*]
      }
      n->state = (CrosNodeState)(n->state & ~CN_STATE_ROSCORE_REQ);
      xmlrpcProcessChangeState(client_proc,XMLRPC_PROCESS_STATE_IDLE);
    }
  }
  else if( client_idx > 0 && client_proc->request_id == CROS_API_REQUEST_TOPIC )
  {
    //xmlrpcParamVectorPrint( &(client_proc->params) );
    XmlrpcParam* param_array = xmlrpcParamVectorAt(&(client_proc->params),0);
    XmlrpcParam* nested_array = xmlrpcParamArrayGetParamAt(param_array,2);
    XmlrpcParam* tcp_port = xmlrpcParamArrayGetParamAt(nested_array,2);

    SubscriberNode* sub = NULL;
    TcprosProcess* tcpros_proc = NULL;

    int i;
    int tcp_port_print = -1;

    for(i = 0; i < n->n_subs; i++)
    {
      if(n->subs[i].client_xmlrpc_id == client_idx)
      {
      	sub = &(n->subs[i]);
        n->subs[i].tcpros_port = tcp_port->data.as_int;
        tcp_port_print = n->subs[i].tcpros_port;
        break;
      }
    }

    tcpros_proc = &(n->tcpros_client_proc[sub->client_tcpros_id]);
    tcpros_proc->topic_idx = i;

    //need to be checked because maybe the connection went down suddenly.
    if(!tcpros_proc->socket.open)
    {
    	tcpIpSocketOpen(&(tcpros_proc->socket));
    }

    PRINT_INFO( "cRosApiParseResponse() : requestTopic response [tcp port: %d]\n", tcp_port_print);
    xmlrpcProcessChangeState(client_proc,XMLRPC_PROCESS_STATE_IDLE);

    //set the process to open the socket with the desired host
    tcprosProcessChangeState(tcpros_proc, TCPROS_PROCESS_STATE_CONNECTING);
  }
  else
  {
    PRINT_ERROR ( "cRosApiParseResponse() : unknown response \n" );
    xmlrpcParamVectorPrint( &(client_proc->params) );
  }
  
  client_proc->request_id = -1;
  
  return ret;
}

void cRosApiParseRequestPrepareResponse( CrosNode *n, int server_idx )
{
  PRINT_DEBUG ( "cRosApiParseRequestPrepareResponse()\n" );
  
  XmlrpcProcess *server_proc = &(n->xmlrpc_server_proc[server_idx]);
  const char *method = dynStringGetData( &(server_proc->method) );
    
  if( server_proc->message_type != XMLRPC_MESSAGE_REQUEST)
  {
    PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Not a request message \n" );
    xmlrpcProcessClear( server_proc );
    server_proc->message_type = XMLRPC_MESSAGE_RESPONSE;
    fillErrorParams ( &(server_proc->params), "" );
    return;
  }

  server_proc->message_type = XMLRPC_MESSAGE_RESPONSE;
      
  if( strcmp( method, "getPid") == 0 )
  {
    xmlrpcProcessClear( server_proc );

    xmlrpcParamVectorPushBackInt( &(server_proc->params), 1 );  
    xmlrpcParamVectorPushBackString( &(server_proc->params), "" );  
    xmlrpcParamVectorPushBackInt( &(server_proc->params), n->pid );  

    generateXmlrpcMessage( n->roscore_host, n->roscore_port, server_proc->message_type, 
                    &(server_proc->method), &(server_proc->params), &(server_proc->message) );
  }
  else if(strcmp(method, "publisherUpdate") == 0)
  {
    PRINT_INFO("publisherUpdate()\n");
    // TODO Store the subscribed node name
    XmlrpcParam *caller_id_param, *topic_param, *publishers_param;

    xmlrpcParamVectorPrint( &(server_proc->params) );

    caller_id_param = xmlrpcParamVectorAt( &(server_proc->params), 0);
    topic_param = xmlrpcParamVectorAt( &(server_proc->params), 1);
    publishers_param = xmlrpcParamVectorAt( &(server_proc->params), 2);

    if( xmlrpcParamGetType( caller_id_param ) != XMLRPC_PARAM_STRING ||
      xmlrpcParamGetType( topic_param ) != XMLRPC_PARAM_STRING ||
       xmlrpcParamGetType( publishers_param ) != XMLRPC_PARAM_ARRAY  )
    {
      PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Wrong publisherUpdate message \n" );
      xmlrpcProcessClear( server_proc );
      fillErrorParams ( &(server_proc->params), "" );
    }
    else
    {
      int array_size = xmlrpcParamArrayGetSize( publishers_param );
      XmlrpcParam *uri, *proto_name;
      int i = 0, topic_found = 0, uri_found = 0;
      SubscriberNode* requesting_subscriber = NULL;

      for( i = 0 ; i < n->n_subs; i++)
      {
        if( strcmp( xmlrpcParamGetString( topic_param ), n->subs[i].topic_name ) == 0)
        {
          topic_found = 1;
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

      if( topic_found && uri_found )
      {

        xmlrpcParamVectorPushBackInt( &(server_proc->params), 1 );
        xmlrpcParamVectorPushBackString( &(server_proc->params), "" );
        xmlrpcParamVectorPushBackInt( &(server_proc->params), n->pid );
        publishers_param = xmlrpcParamVectorAt( &(server_proc->params), 2);
        int available_pubs_n = xmlrpcParamArrayGetSize(publishers_param);

        if(available_pubs_n > 0)
        {

        	//TODO: We just consider the first host. In the remote future we should consider the others as well
          i = 0;
					XmlrpcParam* pub_host = xmlrpcParamArrayGetParamAt(publishers_param, i);
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
					lookup_host(hostname, requesting_subscriber->topic_host);
					requesting_subscriber->topic_port = atoi(strtok_r(NULL,":",&progress));
					xmlrpcProcessChangeState(&(n->xmlrpc_client_proc[requesting_subscriber->client_xmlrpc_id]),XMLRPC_PROCESS_STATE_WRITING);
					n->state = (CrosNodeState)(n->state | CN_STATE_ASK_FOR_CONNECTION);
        }

      // WARNING DEBUG CODE
      //xmlrpcParamVectorPrint( &(server_proc->params) );

      }
      else
      {
        PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Topic not available or protocol for publisherUpdate() not supported\n" );

        //Resetting the xmlrpc connection

        XmlrpcProcess* sub_xmlrpc_proc = &(n->xmlrpc_client_proc[requesting_subscriber->client_xmlrpc_id]);
        xmlrpcProcessClear(sub_xmlrpc_proc);
        xmlrpcProcessRelease(sub_xmlrpc_proc);
        xmlrpcProcessInit(sub_xmlrpc_proc);

        //TODO: Evaluate if it's correct reopen the connection here
        if( !tcpIpSocketOpen( &(sub_xmlrpc_proc->socket) ) ||
            !tcpIpSocketSetReuse( &(sub_xmlrpc_proc->socket) ) ||
            !tcpIpSocketSetNonBlocking( &(sub_xmlrpc_proc->socket) ) )
        {
          PRINT_ERROR("openXmlrpcClientSocket() at index %d failed", i);
          exit( EXIT_FAILURE );
        }

        //Resetting tcpros infos

        requesting_subscriber->tcpros_port = -1;

        xmlrpcProcessClear( server_proc );
        fillErrorParams ( &(server_proc->params), "" );
      }

    }

    generateXmlrpcMessage( n->roscore_host, n->roscore_port, server_proc->message_type,
                        &(server_proc->method), &(server_proc->params), &(server_proc->message) );
  }
  else if( strcmp( method, "requestTopic") == 0 )
  {

    XmlrpcParam *node_param, *topic_param, *protocols_param;
    
    //xmlrpcParamVectorPrint( &(server_proc->params) );
    
    node_param = xmlrpcParamVectorAt( &(server_proc->params), 0);
    topic_param = xmlrpcParamVectorAt( &(server_proc->params), 1);
    protocols_param = xmlrpcParamVectorAt( &(server_proc->params), 2);
    
    if( xmlrpcParamGetType( node_param ) != XMLRPC_PARAM_STRING || 
        xmlrpcParamGetType( topic_param ) != XMLRPC_PARAM_STRING ||
         xmlrpcParamGetType( protocols_param ) != XMLRPC_PARAM_ARRAY  )
    {
      PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Wrong requestTopic message \n" );
      xmlrpcProcessClear( server_proc );
      fillErrorParams ( &(server_proc->params), "" );
    }
    else
    { 
      int array_size = xmlrpcParamArrayGetSize( protocols_param );
      XmlrpcParam *proto, *proto_name;
      int i = 0, topic_found = 0, protocol_found = 0;
      
      for( i = 0 ; i < n->n_pubs; i++)
      {
        if( strcmp( xmlrpcParamGetString( topic_param ), n->pubs[i].topic_name ) == 0)
        {
          topic_found = 1;
          break;
        }
      }
      
      for( i = 0 ; i < array_size; i++)
      {
        proto = xmlrpcParamArrayGetParamAt ( protocols_param, i );
        
        if( xmlrpcParamGetType( proto ) == XMLRPC_PARAM_ARRAY && 
            ( proto_name = xmlrpcParamArrayGetParamAt ( proto, 0 ) ) != NULL &&
            xmlrpcParamGetType( proto_name ) == XMLRPC_PARAM_STRING &&
            strcmp( xmlrpcParamGetString( proto_name ), CROS_API_TCPROS_STRING) == 0 )
        {
          protocol_found = 1;
          break;
        }
      }
      
      if( topic_found && protocol_found)
      {
        xmlrpcProcessClear( server_proc );
        xmlrpcParamVectorPushBackArray( &(server_proc->params) );
        xmlrpcParamArrayPushBackInt( xmlrpcParamVectorAt( &(server_proc->params), 0 ), 1 );
        // TODO Add statusMessage here
        xmlrpcParamArrayPushBackString( xmlrpcParamVectorAt( &(server_proc->params), 0 ), "" );
        xmlrpcParamArrayPushBackArray( xmlrpcParamVectorAt( &(server_proc->params), 0 ));
        XmlrpcParam *array = xmlrpcParamArrayGetParamAt( xmlrpcParamVectorAt( &(server_proc->params), 0 ), 2 );
        xmlrpcParamArrayPushBackString( array, CROS_API_TCPROS_STRING );
        xmlrpcParamArrayPushBackString( array, n->host );
        xmlrpcParamArrayPushBackInt( array, n->tcpros_port );
        
        // WARNING DEBUG CODE
        //xmlrpcParamVectorPrint( &(server_proc->params) );
        
      }
      else
      {
        PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Topic or protocol for requestTopic not supported\n" );
        xmlrpcProcessClear( server_proc );
        fillErrorParams ( &(server_proc->params), "" );
      }
      
    }
    
    generateXmlrpcMessage( n->roscore_host, n->roscore_port, server_proc->message_type, 
                    &(server_proc->method), &(server_proc->params), &(server_proc->message) );
  }
  else if( strcmp( method, "getSubscriptions") == 0 )
  {
  	int i = 0;

    xmlrpcProcessClear( server_proc );

    xmlrpcParamVectorPushBackInt( &(server_proc->params), 1 );
    xmlrpcParamVectorPushBackString( &(server_proc->params), "" );
    xmlrpcParamVectorPushBackArray(&(server_proc->params));
    XmlrpcParam* param_array = xmlrpcParamVectorAt(&(server_proc->params), 2);

  	for(i = 0; i < n->n_subs; i++)
  	{
  		XmlrpcParam* sub_array = xmlrpcParamArrayPushBackArray(param_array);
  		xmlrpcParamArrayPushBackString(sub_array, n->subs[i].topic_name);
  		xmlrpcParamArrayPushBackString(sub_array, n->subs[i].topic_type);
  	}

    generateXmlrpcMessage( n->host, n->xmlrpc_port, server_proc->message_type,
                    &(server_proc->method), &(server_proc->params), &(server_proc->message) );
  }
  else if( strcmp( method, "getPublications") == 0 )
  {
  	int i = 0;

    xmlrpcProcessClear( server_proc );

    xmlrpcParamVectorPushBackInt( &(server_proc->params), 1 );
    xmlrpcParamVectorPushBackString( &(server_proc->params), "" );
    xmlrpcParamVectorPushBackArray(&(server_proc->params));
    XmlrpcParam* param_array = xmlrpcParamVectorAt(&(server_proc->params), 2);

  	for(i = 0; i < n->n_pubs; i++)
  	{
  		XmlrpcParam* sub_array = xmlrpcParamArrayPushBackArray(param_array);
  		xmlrpcParamArrayPushBackString(sub_array, n->pubs[i].topic_name);
  		xmlrpcParamArrayPushBackString(sub_array, n->pubs[i].topic_type);
  	}

    generateXmlrpcMessage( n->host, n->xmlrpc_port, server_proc->message_type,
                    &(server_proc->method), &(server_proc->params), &(server_proc->message) );
  }
  else
  {
    PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Unknown method \n Message : \n %s",
                  dynStringGetData( &(server_proc->message) ) );
    
    xmlrpcParamVectorPrint( &(server_proc->params) );
    
    xmlrpcProcessClear( server_proc );
    server_proc->message_type = XMLRPC_MESSAGE_RESPONSE;
    fillErrorParams ( &(server_proc->params), "" );
  }        
}

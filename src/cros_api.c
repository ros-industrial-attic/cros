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
  CROS_API_REGISTER_PUBLISHER,
  CROS_API_REGISTER_SUBSCRIBER,
  CROS_API_REQUEST_TOPIC,
};

static const char *CROS_API_TCPROS_STRING = "TCPROS";

int
lookup_host (const char *host, char *ip)
{
  struct addrinfo hints, *res;
  int errcode;
  char addrstr[100];
  void *ptr;

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
        case AF_INET6:
          ptr = &((struct sockaddr_in6 *) res->ai_addr)->sin6_addr;
          break;
        }
      inet_ntop (res->ai_family, ptr, addrstr, 100);
      printf ("IPv%d address: %s (%s)\n", res->ai_family == PF_INET6 ? 6 : 4,
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
      generateXmlrpcMessage( n->roscore_host, n->roscore_port, client_proc->message_type,
                          &(client_proc->method), &(client_proc->params), &(client_proc->message) );

    }
    else if( n->state & CN_STATE_ADVERTISE_SUBSCRIBER && n->n_subs )
    {
    PRINT_INFO("cRosApiPrepareRequest() : registerSubscriber\n");
    dynStringPushBackStr( &(client_proc->method), "registerSubscriber" );

    xmlrpcParamVectorPushBackString( &(client_proc->params), n->name );
    xmlrpcParamVectorPushBackString( &(client_proc->params), n->subs[n->n_advertised_subs].topic_name );
    xmlrpcParamVectorPushBackString( &(client_proc->params), n->subs[n->n_advertised_subs].topic_type );
    char node_uri[256];
    sprintf( node_uri, "http://%s:%d/", n->host, n->xmlrpc_port);
    xmlrpcParamVectorPushBackString( &(client_proc->params), node_uri );

    client_proc->request_id = CROS_API_REGISTER_SUBSCRIBER;

    generateXmlrpcMessage( n->roscore_host, n->roscore_port, client_proc->message_type,
                        &(client_proc->method), &(client_proc->params), &(client_proc->message) );
    }
    else
    {
      PRINT_INFO("cRosApiPrepareRequest() : ping roscore\n");

      // Default behavior: ping roscore (actually, ping a node of roscore, i.e. default /rosout )

      dynStringPushBackStr( &(client_proc->method), "getPid" );
      xmlrpcParamVectorPushBackString( &(client_proc->params), "/rosout");

      client_proc->request_id = CROS_API_GET_PID;

      generateXmlrpcMessage( n->roscore_host, n->roscore_port, client_proc->message_type,
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

        generateXmlrpcMessage(subscriber_node->topic_host, subscriber_node->topic_port, client_proc->message_type,
                              &(client_proc->method), &(client_proc->params), &(client_proc->message) );
      }
    }
  }
}

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
          char* clean_string = calloc(dirty_string_len-8,sizeof(char));
          strncpy(clean_string,pub_host_string+7,dirty_string_len-8);
          char* hostname = strtok(clean_string,":");
          requesting_subscriber->topic_host = calloc(100,sizeof(char)); //deleted in cRosNodeDestroy
          lookup_host(hostname, requesting_subscriber->topic_host);
          requesting_subscriber->topic_port = atoi(strtok(NULL,":"));
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
      }
      else
      {
        if( n->n_pubs )
        {
          n->state = (CrosNodeState)(n->state | CN_STATE_ADVERTISE_PUBLISHER);
          n->n_advertised_pubs = 0;
        }
      }
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
      }
    }
    tcpros_proc = &(n->tcpros_client_proc[sub->client_tcpros_id]);
    PRINT_DEBUG( "cRosApiParseResponse() : requestTopic response [tcp port: %d]\n", tcp_port_print);
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

    //xmlrpcParamVectorPrint( &(server_proc->params) );

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
        int available_pubs_n = xmlrpcParamArrayGetSize(publishers_param);

        if(available_pubs_n > 0)
        {
          for(i = 0; i < available_pubs_n; i++)
          {
            XmlrpcParam* pub_host = xmlrpcParamArrayGetParamAt(publishers_param, i);
            char* pub_host_string = xmlrpcParamGetString(pub_host);
            //manage string for exploit informations
            //removing the 'http://' and the last '/'
            int dirty_string_len = strlen(pub_host_string);
            char* clean_string = calloc(dirty_string_len-8,sizeof(char));
            strncpy(clean_string,pub_host_string+7,dirty_string_len-8);
            char* hostname = strtok(clean_string,":");
            requesting_subscriber->topic_host = calloc(100,sizeof(char)); //deleted in cRosNodeDestroy
            lookup_host(hostname, requesting_subscriber->topic_host);
            requesting_subscriber->topic_port = atoi(strtok(NULL,":"));
            xmlrpcProcessChangeState(&(n->xmlrpc_client_proc[requesting_subscriber->client_xmlrpc_id]),XMLRPC_PROCESS_STATE_WRITING);
            n->state = (CrosNodeState)(n->state | CN_STATE_ASK_FOR_CONNECTION);
          }
        }
      // WARNING DEBUG CODE
      //xmlrpcParamVectorPrint( &(server_proc->params) );

      }
      else
      {
        PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Topic or protocol for publisherUpdate() not supported\n" );
        xmlrpcProcessClear( server_proc );
        fillErrorParams ( &(server_proc->params), "" );
      }

    }

    generateXmlrpcMessage( n->roscore_host, n->roscore_port, server_proc->message_type,
                        &(server_proc->method), &(server_proc->params), &(server_proc->message) );
  }
  else if( strcmp( method, "requestTopic") == 0 )
  {
    
    // TODO Store the subscribed node name
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

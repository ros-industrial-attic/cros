#include <string.h>

#include "cros_api.h"
#include "cros_defs.h"

enum
{
  CROS_API_GET_PID,
  CROS_API_REGISTER_PUBLISHER,
};

static const char *CROS_API_TCPROS_STRING = "TCPROS";

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

void cRosApiPrepareRequest( CrosNode *n )
{
  PRINT_VDEBUG ( "cRosApiPrepareRequest()\n" );
  
  XmlrpcProcess *client_proc = &(n->xmlrpc_client_proc);
  
  xmlrpcProcessClear( client_proc );
      
  if( n->state & CN_STATE_ADVERTISE_PUBLISCER && n->n_pubs )
  {   
    dynStringPushBackStr( &(client_proc->method), "registerPublisher" );

    xmlrpcParamVectorPushBackString( &(client_proc->params), n->name );
    xmlrpcParamVectorPushBackString( &(client_proc->params), n->pubs[n->n_advertised_pubs].topic_name );
    xmlrpcParamVectorPushBackString( &(client_proc->params), n->pubs[n->n_advertised_pubs].topic_type );
    char node_uri[256];;
    sprintf( node_uri, "http://%s:%d/", n->host, n->xmlrpc_port);
    xmlrpcParamVectorPushBackString( &(client_proc->params), node_uri ); 
    
    client_proc->request_id = CROS_API_REGISTER_PUBLISHER;
    
  }
  else
  {
    PRINT_INFO("cRosApiPrepareRequest() : ping roscore\n");
    
    // Default behavior: ping roscore (actually, ping a node of roscore, i.e. default /rosout )
    
    dynStringPushBackStr( &(client_proc->method), "getPid" );
    xmlrpcParamVectorPushBackString( &(client_proc->params), "/rosout");
    
    client_proc->request_id = CROS_API_GET_PID;
  }
  client_proc->message_type = XMLRPC_MESSAGE_REQUEST;
  
    // TODO Not only rosore!
  generateXmlrpcMessage( n->roscore_host, n->roscore_port, client_proc->message_type, 
                        &(client_proc->method), &(client_proc->params), &(client_proc->message) );  
}

int cRosApiParseResponse( CrosNode *n )
{
  PRINT_VDEBUG ( "cRosApiParseResponse()\n" );
  XmlrpcProcess *client_proc = &(n->xmlrpc_client_proc);
  const char *method = dynStringGetData( &(client_proc->method) );
  int ret = 0;
  
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
        n->state = (CrosNodeState)(n->state & ~CN_STATE_ADVERTISE_PUBLISCER);
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
        n->state = (CrosNodeState)(n->state | CN_STATE_ADVERTISE_PUBLISCER);
        n->n_advertised_pubs = 0;
      }
    }
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
  PRINT_VDEBUG ( "cRosApiParseRequestPrepareResponse()\n" );
  
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
      
      if( topic_found && protocol_found )
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
    
//     xmlrpcProcessClear( server_proc );
//     xmlrpcParamVectorPushBackInt( &(server_proc->params), 1 );  
//     xmlrpcParamVectorPushBackString( &(server_proc->params), "" );  
//     xmlrpcParamVectorPushBackInt( &(server_proc->params), n->pid );  
// 
    generateXmlrpcMessage( n->roscore_host, n->roscore_port, server_proc->message_type, 
                    &(server_proc->method), &(server_proc->params), &(server_proc->message) );
  }
  else
  {
    PRINT_ERROR ( "cRosApiParseRequestPrepareResponse() : Unknown metohd \n Message : \n %s", 
                  dynStringGetData( &(server_proc->message) ) );
    
    xmlrpcParamVectorPrint( &(server_proc->params) );
    
    xmlrpcProcessClear( server_proc );
    server_proc->message_type = XMLRPC_MESSAGE_RESPONSE;
    fillErrorParams ( &(server_proc->params), "" );
  }        
}
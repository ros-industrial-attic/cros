#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "xmlrpc_protocol.h"
#include "xmlrpc_tags.h"
#include "cros_defs.h"

static XmlrpcParserState parseXmlrpcMessageParams ( const char *params_body, int params_body_len,
    XmlrpcParamVector *params )
{
  PRINT_VDEBUG ( "parseXmlrpcMessageParams()\n" );

  int i = 0;
  const char *c = params_body;
  int found_params = 0;

  for ( ; i < params_body_len; i++, c++ )
  {
    if ( params_body_len - i >= XMLRPC_PARAMS_TAG.dim &&
         strncmp ( c, XMLRPC_PARAMS_TAG.str, XMLRPC_PARAMS_TAG.dim ) == 0 )
    {
      c += XMLRPC_PARAMS_TAG.dim;
      i += XMLRPC_PARAMS_TAG.dim;
      found_params = 1;
      break;
    }
  }

  if ( !found_params )
  {
    PRINT_ERROR ( "parseXmlrpcMessageParams() : params not found\n" );
    return XMLRPC_PARSER_ERROR;
  }

  DynString param_str;
  dynStringInit ( &param_str );
  int param_str_len = 0;
  const char *param_init = NULL;

  for ( ; i < params_body_len; i++, c++ )
  {
    if ( params_body_len - i >= XMLRPC_PARAMS_ETAG.dim &&
         strncmp ( c, XMLRPC_PARAMS_ETAG.str, XMLRPC_PARAMS_ETAG.dim ) == 0 )
    {
      c += XMLRPC_PARAMS_ETAG.dim;
      i += XMLRPC_PARAMS_ETAG.dim;
      break;
    }

    if ( param_init == NULL )
    {
      if ( params_body_len - i >= XMLRPC_PARAM_TAG.dim &&
           strncmp ( c, XMLRPC_PARAM_TAG.str, XMLRPC_PARAM_TAG.dim ) == 0 )
      {
        c += XMLRPC_PARAM_TAG.dim - 1;
        i += XMLRPC_PARAM_TAG.dim - 1;
        param_init = c + 1;
        param_str_len = 0;
      }
    }
    else
    {
      if ( params_body_len - i >= XMLRPC_PARAM_ETAG.dim &&
           strncmp ( c, XMLRPC_PARAM_ETAG.str, XMLRPC_PARAM_ETAG.dim ) == 0 )
      {
        if ( param_str_len )
        {
          dynStringClear ( &param_str );
          dynStringPushBackStrN ( &param_str, param_init, param_str_len );

          XmlrpcParam param;

          if ( !xmlrpcParamFromXml ( &param_str, &param ) ||
               xmlrpcParamVectorPushBack ( params, &param ) < 0 )
            return XMLRPC_PARSER_ERROR;
        }

        c += XMLRPC_PARAM_ETAG.dim - 1;
        i += XMLRPC_PARAM_ETAG.dim - 1;
        param_init = NULL;
        param_str_len = 0;
      }
      else
        param_str_len++;
    }
  }

  return XMLRPC_PARSER_DONE;
}

static XmlrpcParserState parseXmlrpcMessageBody ( const char *body, int body_len, XmlrpcMessageType *type,
    DynString *method, XmlrpcParamVector *params )
{
  PRINT_VDEBUG ( "parseXmlrpcMessageBody()\n" );

  *type = XMLRPC_MESSAGE_UNKNOWN;
  int i = 0;
  const char *c = body;

  for ( ; i < body_len; i++, c++ )
  {
    if ( body_len - i >= XMLRPC_REQUEST_BEGIN.dim &&
         strncmp ( c, XMLRPC_REQUEST_BEGIN.str, XMLRPC_REQUEST_BEGIN.dim ) == 0 )
    {
      c += XMLRPC_REQUEST_BEGIN.dim;
      i += XMLRPC_REQUEST_BEGIN.dim;
      *type = XMLRPC_MESSAGE_REQUEST;
      break;
    }
    else if ( body_len - i >= XMLRPC_RESPONSE_BEGIN.dim &&
              strncmp ( c, XMLRPC_RESPONSE_BEGIN.str, XMLRPC_RESPONSE_BEGIN.dim ) == 0 )
    {
      c += XMLRPC_RESPONSE_BEGIN.dim;
      i += XMLRPC_RESPONSE_BEGIN.dim;
      *type = XMLRPC_MESSAGE_RESPONSE;
      break;
    }
  }

  if ( *type == XMLRPC_MESSAGE_UNKNOWN )
  {
    PRINT_ERROR ( "parseXmlrpcMessageBody() : unknown message type\n" );
    return XMLRPC_PARSER_ERROR;
  }

  if ( *type == XMLRPC_MESSAGE_REQUEST )
  {
    const char *method_name_init = NULL, *method_name_end = NULL;
    int method_name_size = 0;
    for ( ; i < body_len; i++, c++ )
    {
      if ( body_len - i >= XMLRPC_METHODNAME_BEGIN.dim &&
           strncmp ( c, XMLRPC_METHODNAME_BEGIN.str, XMLRPC_METHODNAME_BEGIN.dim ) == 0 )
      {
        c += XMLRPC_METHODNAME_BEGIN.dim;
        i += XMLRPC_METHODNAME_BEGIN.dim;
        method_name_init = c;
        break;
      }
    }

    if ( method_name_init == NULL )
    {
      PRINT_ERROR ( "parseXmlrpcMessageBody() : missing method name\n" );
      return XMLRPC_PARSER_ERROR;
    }

    for ( ; i < body_len; i++, c++ )
    {
      if ( body_len - i >= XMLRPC_METHODNAME_END.dim &&
           strncmp ( c, XMLRPC_METHODNAME_END.str, XMLRPC_METHODNAME_END.dim ) == 0 )
      {
        method_name_end = c - 1;
        c += XMLRPC_METHODNAME_END.dim;
        i += XMLRPC_METHODNAME_END.dim;
        break;
      }
      else
        method_name_size++;
    }

    if ( method_name_end == NULL )
    {
      PRINT_ERROR ( "parseXmlrpcMessageBody() : missing method name\n" );
      return XMLRPC_PARSER_ERROR;
    }

    if ( dynStringPushBackStrN(method, method_name_init, method_name_size ) < 0 )
      return XMLRPC_PARSER_ERROR;
  }
  
  if( *type == XMLRPC_MESSAGE_REQUEST )
    PRINT_DEBUG("Received request message, method : %s\n", dynStringGetData( method ));
  else if ( *type == XMLRPC_MESSAGE_RESPONSE )
    PRINT_DEBUG("Received response message\n");
  
  return parseXmlrpcMessageParams ( c, body_len - i, params );
}

void generateXmlrpcMessage ( const char*host, unsigned short port, XmlrpcMessageType type,
                             const char *method, XmlrpcParamVector *params, DynString *message )
{
  PRINT_VDEBUG ( "generateXmlrpcMessage()\n" );

  dynStringClear ( message );

  if( type == XMLRPC_MESSAGE_REQUEST )
  {
    dynStringPushBackStr ( message, "POST " );
    dynStringPushBackStr ( message, "/" ); // uri
    dynStringPushBackStr ( message, " HTTP/1.1\r\n" );
    dynStringPushBackStr ( message, "User-Agent: " );
    dynStringPushBackStr ( message, XMLRPC_VERSION.str );
    dynStringPushBackStr ( message, "\r\nHost: " );
    dynStringPushBackStr ( message, host );

    char port_str[50];
    snprintf ( port_str, 50, ":%d\r\n", port );

    dynStringPushBackStr ( message, port_str );
  }
  else if( type == XMLRPC_MESSAGE_RESPONSE )
  {
    dynStringPushBackStr ( message, "HTTP/1.1 200 OK\r\n" );
    dynStringPushBackStr ( message, "Server: " );
    dynStringPushBackStr ( message, XMLRPC_VERSION.str );
    dynStringPushBackStr ( message, "\r\n" );
  }
  else
  {
    PRINT_ERROR ( "generateXmlrpcMessage() : Unknown message type\n" );
  }
  
  dynStringPushBackStr ( message, "Content-Type: text/xml\r\nContent-length: " );

  int content_len_init = dynStringGetLen ( message );

  dynStringPushBackStr ( message, "0000000000000\r\n\r\n" );
  int content_init = dynStringGetLen ( message );

  dynStringPushBackStr ( message, XMLRPC_MESSAGE_BEGIN.str );

  if ( type == XMLRPC_MESSAGE_REQUEST )
  {
    dynStringPushBackStr ( message, XMLRPC_REQUEST_BEGIN.str );
    dynStringPushBackStr ( message, XMLRPC_METHODNAME_BEGIN.str );
    dynStringPushBackStr ( message, method );
    dynStringPushBackStr ( message, XMLRPC_METHODNAME_END.str );
  }
  else if ( type == XMLRPC_MESSAGE_RESPONSE )
  {
    dynStringPushBackStr ( message, XMLRPC_RESPONSE_BEGIN.str );
  }
  int n_params = xmlrpcParamVectorGetSize ( params );
  if ( n_params > 0 )
  {
    char param_str[256];
    int i = 0;
    dynStringPushBackStr ( message, XMLRPC_PARAMS_TAG.str );

    for ( i = 0; i < n_params; i++ )
    {
      dynStringPushBackStr ( message, XMLRPC_PARAM_TAG.str );
      xmlrpcParamToXml ( xmlrpcParamVectorAt ( params, i ), message );
      dynStringPushBackStr ( message, XMLRPC_PARAM_ETAG.str );
    }
    dynStringPushBackStr ( message, XMLRPC_PARAMS_ETAG.str );
  }

  if ( type == XMLRPC_MESSAGE_REQUEST )
    dynStringPushBackStr ( message, XMLRPC_REQUEST_END.str );
  else if ( type == XMLRPC_MESSAGE_RESPONSE )
    dynStringPushBackStr ( message, XMLRPC_RESPONSE_END.str );

  dynStringPushBackStr ( message, XMLRPC_MESSAGE_END.str );

  int content_end = dynStringGetLen ( message );
  int content_len = content_end - content_init;

  char content_len_str[15];
  snprintf ( content_len_str, 15, "%.13d", content_len );

  dynStringPatch ( message, content_len_str, content_len_init );
}

XmlrpcParserState parseXmlrpcMessage(DynString *message, XmlrpcMessageType *type,
                                     DynString *method, XmlrpcParamVector *params,
                                     char host[256], int *port)
{
  PRINT_VDEBUG ( "parseXmlrpcMessage()\n" );

  int msg_len = dynStringGetLen ( message );
  char *msg = ( char * ) dynStringGetData ( message );
  char *body_len_init = NULL;
  char *body_init = NULL;
  char *host_init = NULL;

  int i;
  for ( i = 0; i < msg_len; i++, msg++ )
  {
    if ( msg_len - i >= 16 && strncasecmp ( msg, "Content-length: ", 16 ) == 0 )
    {
      body_len_init = msg + 16;
    }
    if ( msg_len - i >= 6 && strncasecmp ( msg, "Host: ", 16 ) == 0 )
    {
      host_init = msg + 6;
    }
    else if ( msg_len - i >= 4 && strncmp ( msg, "\r\n\r\n", 4 ) == 0 )
    {
      body_init = msg + 4;
      break;
    }
    else if ( msg_len - i >= 2 && strncmp ( msg, "\n\n", 2 ) == 0 )
    {
      body_init = msg + 2;
      break;
    }
  }

  if ( body_init == NULL )
  {
    PRINT_DEBUG ( "parseXmlrpcMessage() : message incomplete\n" );
    return XMLRPC_PARSER_INCOMPLETE;
  }
  else if ( body_len_init == NULL )
  {
    PRINT_ERROR ( "parseXmlrpcMessage() : Content-length not present\n" );
    return XMLRPC_PARSER_ERROR;
  }

  int body_len = 0;
  if ( sscanf ( body_len_init,"%d", &body_len ) != 1 )
  {
    PRINT_ERROR ( "parseXmlrpcMessage() : Content-length not valid\n" );
    return XMLRPC_PARSER_ERROR;
  }

  if ( body_len > (int)strlen ( body_init ) )
  {
    PRINT_DEBUG ( "parseXmlrpcMessage() : message incomplete\n" );
    return XMLRPC_PARSER_INCOMPLETE;
  }

  if (host_init == NULL)
  {
    memset(host, 0, sizeof(256));
    *port = -1;
  }
  else
  {
    char temp_host[256];
    snprintf(temp_host, 256, "%s", host_init); // Temporary copy
    char full_host[256];
    sscanf(full_host, "%s", temp_host); // Remove spaces and newlines
    strncpy(temp_host,full_host+7,strlen(full_host)-8); // Remove 'http://' and '/'
    char * progress = NULL;
    char* host_ = strtok_r(temp_host,":",&progress);
    strcpy(host, host_);
    *port = atoi(strtok_r(NULL,":",&progress));
  }

  PRINT_DEBUG ( "parseXmlrpcMessage() : body len : %d\n", body_len );

  return parseXmlrpcMessageBody ( body_init, body_len, type, method, params );

}

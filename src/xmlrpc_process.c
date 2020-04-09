#include <stddef.h>
#include <string.h>

#include "xmlrpc_process.h"
#include "cros_clock.h"

void xmlrpcProcessInit( XmlrpcProcess *p )
{
  p->current_call = NULL;
  p->state = XMLRPC_PROCESS_STATE_IDLE;
  tcpIpSocketInit( &(p->socket) );
  p->message_type = XMLRPC_MESSAGE_UNKNOWN;
  dynStringInit( &(p->method) );
  dynStringInit( &(p->message) );
  xmlrpcParamVectorInit( &(p->params) );
  xmlrpcParamVectorInit( &(p->response) );
  p->last_change_time = 0;
  memset(p->host, 0, sizeof(p->host));
  p->port = -1;
}

void xmlrpcProcessRelease( XmlrpcProcess *p )
{
  if( p->socket.connected )
    tcpIpSocketDisconnect( &(p->socket) );

  if (p->current_call != NULL)
    freeRosApiCall(p->current_call);

  tcpIpSocketClose( &(p->socket) );
  dynStringRelease( &(p->method) );
  dynStringRelease( &(p->message) );
  xmlrpcParamVectorRelease( &(p->params) );
  xmlrpcParamVectorRelease( &(p->response) );
}

void xmlrpcProcessClear( XmlrpcProcess *p)
{
  dynStringClear(&p->message);
}

void xmlrpcProcessReset( XmlrpcProcess *p)
{
  xmlrpcProcessClear(p);

  if (p->current_call != NULL)
    freeRosApiCall(p->current_call);

  p->current_call = NULL;
  dynStringClear(&p->method);
  xmlrpcParamVectorRelease(&p->params);
  xmlrpcParamVectorRelease(&p->response);
  p->message_type = XMLRPC_MESSAGE_UNKNOWN;
  memset(p->host, 0, sizeof(p->host));
  p->port = -1;
}

void xmlrpcProcessChangeState( XmlrpcProcess *p, XmlrpcProcessState state )
{
  p->state = state;
  p->last_change_time = cRosClockGetTimeMs();
}

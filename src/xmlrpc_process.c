#include "xmlrpc_process.h"
#include "cros_clock.h"

void xmlrpcProcessInit( XmlrpcProcess *p )
{
  p->state = XMLRPC_PROCESS_STATE_IDLE;
  p->request_id = -1;
  tcpIpSocketInit( &(p->socket) );
  p->message_type = XMLRPC_MESSAGE_UNKNOWN;
  dynStringInit( &(p->method) );
  dynStringInit( &(p->message) );
  xmlrpcParamVectorInit( &(p->params) );
  p->last_change_time = 0;
  p->wake_up_time_ms = 0;
}

void xmlrpcProcessRelease( XmlrpcProcess *p )
{
  if( p->socket.connected )
    tcpIpSocketDisconnect( &(p->socket) );
  
  tcpIpSocketClose( &(p->socket) );
  dynStringRelease( &(p->method) );
  dynStringRelease( &(p->message) );
  xmlrpcParamVectorRelease( &(p->params) );
}

void xmlrpcProcessClear( XmlrpcProcess *p )
{
  dynStringClear( &(p->method) );
  dynStringClear( &(p->message) );
  xmlrpcParamVectorRelease( &(p->params) );
}

void xmlrpcProcessChangeState( XmlrpcProcess *p, XmlrpcProcessState state )
{
  p->state = state;
  p->last_change_time = cRosClockGetTimeMs();
}
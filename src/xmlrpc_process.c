#include "xmlrpc_process.h"
#include "cros_clock.h"

void xmlrpcProcessInit( XmlrpcProcess *p )
{
  initApiCallQueue(&p->api_calls_queue);
  p->state = XMLRPC_PROCESS_STATE_IDLE;
  tcpIpSocketInit( &(p->socket) );
  p->message_type = XMLRPC_MESSAGE_UNKNOWN;
  dynStringInit( &(p->method) );
  p->method_id = CROS_API_NONE;
  p->provider_idx = -1;
  dynStringInit( &(p->message) );
  xmlrpcParamVectorInit( &(p->params) );
  xmlrpcParamVectorInit( &(p->response) );
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
  xmlrpcParamVectorRelease( &(p->response) );
}

void xmlrpcProcessClear( XmlrpcProcess *p, int fullclear)
{
  dynStringClear(&p->message);
  if (fullclear)
  {
    dynStringClear(&p->method);
    xmlrpcParamVectorRelease(&p->params);
    xmlrpcParamVectorRelease(&p->response);
    p->method_id = CROS_API_NONE;
    p->provider_idx = -1;
    p->message_type = XMLRPC_MESSAGE_UNKNOWN;
  }
}

void xmlrpcProcessChangeState( XmlrpcProcess *p, XmlrpcProcessState state )
{
  p->state = state;
  p->last_change_time = cRosClockGetTimeMs();
}
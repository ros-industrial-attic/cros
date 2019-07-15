#include "tcpros_process.h"
#include "cros_clock.h"
#include <stdlib.h>

void tcprosProcessInit( TcprosProcess *p )
{
  p->state = TCPROS_PROCESS_STATE_IDLE;
  tcpIpSocketInit( &(p->socket) );
  dynStringInit( &(p->topic) );
  dynStringInit( &(p->caller_id) );
  dynStringInit( &(p->service) );
  dynStringInit( &(p->type) );
  dynStringInit( &(p->servicerequest_type) );
  dynStringInit( &(p->serviceresponse_type) );
  dynStringInit( &(p->md5sum) );
  dynBufferInit( &(p->packet) );
  p->latching = p->tcp_nodelay = p->persistent = 0;
  p->probe = 0;
  p->last_change_time = 0;
  p->topic_idx = -1;
  p->service_idx = -1;
  p->ok_byte = 0;
  p->left_to_recv = 0;
  p->sub_tcpros_host = NULL;
  p->sub_tcpros_port = -1;
}

void tcprosProcessRelease( TcprosProcess *p )
{
  if( p->socket.connected )
    tcpIpSocketDisconnect( &(p->socket) );

  dynStringRelease( &(p->topic) );
  dynStringRelease( &(p->service) );
  dynStringRelease( &(p->caller_id) );
  dynStringRelease( &(p->type) );
  dynStringRelease( &(p->servicerequest_type) );
  dynStringRelease( &(p->serviceresponse_type) );
  dynStringRelease( &(p->md5sum) );
  dynBufferRelease( &(p->packet) );
  free(p->sub_tcpros_host);
}

void tcprosProcessClear( TcprosProcess *p)
{
  dynBufferClear( &(p->packet) );
  p->left_to_recv = 0;
}

void tcprosProcessReset( TcprosProcess *p)
{
  tcprosProcessClear( p );

  dynStringClear( &(p->topic) );
  dynStringClear( &(p->caller_id) );
  dynStringClear( &(p->service) );
  dynStringClear( &(p->type) );
  dynStringClear( &(p->servicerequest_type) );
  dynStringClear( &(p->serviceresponse_type) );
  dynStringClear( &(p->md5sum) );
  p->latching = 0;
  p->tcp_nodelay = 0;
  p->persistent = 0;
  p->probe = 0;
  p->topic_idx = -1;
  p->service_idx = -1;
  p->ok_byte = 0;
  free(p->sub_tcpros_host);
  p->sub_tcpros_host = NULL;
  p->sub_tcpros_port = -1;

  tcprosProcessChangeState( p, TCPROS_PROCESS_STATE_IDLE );
}

void tcprosProcessChangeState( TcprosProcess *p, TcprosProcessState state )
{
  p->state = state;
  p->last_change_time = cRosClockGetTimeMs();
}

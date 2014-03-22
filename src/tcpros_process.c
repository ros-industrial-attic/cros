#include "tcpros_process.h"
#include "cros_clock.h"

void tcprosProcessInit( TcprosProcess *p )
{
  p->state = TCPROS_PROCESS_STATE_IDLE;
  tcpIpSocketInit( &(p->socket) );
  dynStringInit( &(p->topic) );
  dynStringInit( &(p->caller_id) );
  dynStringInit( &(p->type) );
  dynStringInit( &(p->md5sum) );
  dynBufferInit( &(p->packet) );
  p->latching = p->tcp_nodelay = p->persistent = 0;
  p->last_change_time = 0;
  p->wake_up_time_ms = 0;
  p->topic_idx = -1;
  p->left_to_recv = 0;
}

void tcprosProcessRelease( TcprosProcess *p )
{
  if( p->socket.connected )
    tcpIpSocketDisconnect( &(p->socket) );
  
  dynStringRelease( &(p->topic) );
  dynStringRelease( &(p->caller_id) );
  dynStringRelease( &(p->type) );
  dynStringRelease( &(p->md5sum) );
  dynBufferRelease( &(p->packet) );
}

void tcprosProcessClear( TcprosProcess *p )
{
  dynStringClear( &(p->topic) );
  dynStringClear( &(p->caller_id) );
  dynStringClear( &(p->service) );
  dynStringClear( &(p->type) );
  dynStringClear( &(p->md5sum) );
  dynBufferClear( &(p->packet) );
  p->latching = p->tcp_nodelay = p->persistent = 0;
  p->last_change_time = 0;
  p->wake_up_time_ms = 0;
  p->left_to_recv = 0;
}

void tcprosProcessChangeState( TcprosProcess *p, TcprosProcessState state )
{
  p->state = state;
  p->last_change_time = cRosClockGetTimeMs();
}

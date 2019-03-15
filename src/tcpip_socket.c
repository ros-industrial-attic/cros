#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN // speed up the build process by excluding parts of the Windows header
#  include <windows.h>
#  include <winsock2.h>
#else
#  include <unistd.h>
#  include <fcntl.h>
#  include <signal.h>
#  include <sys/socket.h>
#  include <netinet/tcp.h>
#  include <arpa/inet.h>
#  include <errno.h>
#  define closesocket close
#endif

#include "tcpip_socket.h"
#include "cros_defs.h"
#include "cros_log.h"
#include "cros_clock.h"

#define TCPIP_SOCKET_READ_BUFFER_SIZE 2048
// Definitions for debug messages only. Console terminal color sequence:
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

// This global variable is incremented for each call to tcpIpSocketStartUp() that has been successfuly executed,
// and it decremented after each successful call to tcpIpSocketCleanUp()
int socket_lib_initialized = 0; // Default value: 0 = library not initialized

void tcpIpSocketInit ( TcpIpSocket *s )
{
  PRINT_VDEBUG ( "tcpIpSocketInit()\n" );
  s->fd = FN_INVALID_SOCKET;
  s->port = 0;
  memset ( & ( s->adr ), 0, sizeof ( struct sockaddr_in ) );
  s->open = 0;
  s->connected = 0;
  s->listening = 0;
  s->is_nonblocking = 0;
}

int tcpIpSocketOpen ( TcpIpSocket *s )
{
  int ret_success;

  PRINT_VDEBUG ( "tcpIpSocketOpen()\n" );
  if ( s->open )
    return(1);

  s->fd = socket ( AF_INET, SOCK_STREAM, IPPROTO_TCP );
  if ( s->fd == FN_INVALID_SOCKET )
  {
    PRINT_ERROR ( "tcpIpSocketOpen() : Can't open a socket. Error code: %i\n", tcpIpSocketGetError());
    ret_success = 0;
  }
  else
  {
    s->open = 1;
    ret_success = 1;
  }

  return(ret_success);
}

int tcpIpSocketClose ( TcpIpSocket *s )
{
  int ret_success;

  PRINT_VDEBUG ( "tcpIpSocketClose()\n");

  if ( !s->open )
    return(1);

  PRINT_DEBUG ( "tcpIpSocketClose(): Closing socket FD: %i\n", s->fd);
  if ( s->fd != FN_INVALID_SOCKET )
  {
    int close_ret_val;

    close_ret_val = closesocket( s->fd );
    ret_success = (close_ret_val != FN_SOCKET_ERROR);
  }
  else
  {
    PRINT_ERROR ( "tcpIpSocketClose(): Invalid file descriptor: %i\n", s->fd);
    tcpIpSocketInit ( s );
    ret_success = 0;
  }

  return(ret_success);
}

int tcpIpSocketSetNonBlocking ( TcpIpSocket *s )
{
  int ret_fn_ctl;

  PRINT_VDEBUG ( "tcpIpSocketSetNonBlocking()\n" );

  if ( !s->open )
  {
    PRINT_ERROR ( "tcpIpSocketSetNonBlocking() : Socket not opened\n" );
    return(0);
  }

#ifdef _WIN32
  u_long enable_non_blocking = 1;
  ret_fn_ctl = ioctlsocket(s->fd, FIONBIO, &enable_non_blocking);
#else
  int prev_flags;
  prev_flags = fcntl( s->fd, F_GETFL, 0 );
  if(prev_flags < 0)
  {
    PRINT_ERROR ( "tcpIpSocketSetNonBlocking() : fcntl() failed getting the socket flags\n" );
    prev_flags = 0; // Try to continue anyway
  }
  ret_fn_ctl = fcntl ( s->fd, F_SETFL, prev_flags | O_NONBLOCK );
#endif

  if(ret_fn_ctl == 0)
  {
    s->is_nonblocking = 1;
    return(1);
  }
  else
  {
    PRINT_ERROR ( "tcpIpSocketSetNonBlocking() : fcntl()/ ioctlsocket() failed configuring socket as non blocking. Error code: %i\n", tcpIpSocketGetError());
    return(0);
  }
}

int tcpIpSocketSetNoDelay ( TcpIpSocket *s )
{
  PRINT_VDEBUG ( "tcpIpSocketSetNoDelay()\n" );

  if ( !s->open )
  {
    PRINT_ERROR ( "tcpIpSocketSetNoDelay() : Socket not opened\n" );
    return(0);
  }

  int enable_no_delay = 1;
  int ret = setsockopt ( s->fd, IPPROTO_TCP, TCP_NODELAY, (const void *)&enable_no_delay, sizeof(enable_no_delay) );

  if ( ret == 0 )
  {
    return(1);
  }
  else
  {
    PRINT_ERROR ( "tcpIpSocketSetNoDelay() : setsockopt() with TCP_NODELAY failed. System error code: %i \n", tcpIpSocketGetError());
    return(0);
  }
}

int tcpIpSocketSetReuse ( TcpIpSocket *s )
{
  PRINT_VDEBUG ( "tcpIpSocketSetReuse()\n" );

  if ( !s->open )
  {
    PRINT_ERROR ( "tcpIpSocketSetReuse() : Socket not opened\n" );
    return(0);
  }

  int enable_reuse_addr = 1;
  if ( setsockopt ( s->fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&enable_reuse_addr, sizeof(enable_reuse_addr) ) != 0 )
  {
    PRINT_ERROR ( "tcpIpSocketSetReuse() : setsockopt() with SO_REUSEADDR option failed. System error code: %i \n", tcpIpSocketGetError());
    return(0);
  }
  return(1);
}

int tcpIpSocketSetKeepAlive ( TcpIpSocket *s, unsigned int idle, unsigned int interval, unsigned int count )
{
  PRINT_VDEBUG ( "tcpIpSocketSetKeepAlive()\n" );

  if ( !s->open )
  {
    PRINT_ERROR ( "tcpIpSocketSetKeepAlive() : Socket not opened\n" );
    return(0);
  }

  int sock_opt_val = 1;
  if ( setsockopt ( s->fd, SOL_SOCKET, SO_KEEPALIVE, (const char *)&sock_opt_val, sizeof ( sock_opt_val ) ) != 0 )
  {
    PRINT_ERROR ( "tcpIpSocketSetKeepAlive() : setsockopt() with SO_KEEPALIVE option failed. System error code: %i \n", tcpIpSocketGetError());
    return(0);
  }

  // TCP_KEEPIDLE on Linux is equivalent to TCP_KEEPALIVE option on OS X
  // see https://www.winehq.org/pipermail/wine-devel/2015-July/108583.html
  sock_opt_val = idle;
#ifdef __APPLE__
  if ( setsockopt( s->fd, IPPROTO_TCP, TCP_KEEPALIVE, (void *)&sock_opt_val, sizeof ( sock_opt_val ) ) != 0 )
  {
    PRINT_ERROR ( "tcpIpSocketSetKeepAlive() : setsockopt() with TCP_KEEPALIVE option failed \n" );
    return 0;
  }
#else
  if ( setsockopt ( s->fd, IPPROTO_TCP, TCP_KEEPIDLE, (const void *)&sock_opt_val, sizeof ( sock_opt_val ) ) != 0 )
  {
    PRINT_ERROR ( "tcpIpSocketSetKeepAlive() : setsockopt() with SO_KEEPALIVE option failed. System error code: %i \n", tcpIpSocketGetError());
    return(0);
  }
#endif

  sock_opt_val = interval;
  if ( setsockopt ( s->fd, IPPROTO_TCP, TCP_KEEPINTVL, (const void *)&sock_opt_val, sizeof ( sock_opt_val ) ) != 0 )
  {
    PRINT_ERROR ( "tcpIpSocketSetKeepAlive() : setsockopt() with TCP_KEEPINTVL option failed. System error code: %i \n", tcpIpSocketGetError());
    return(0);
  }

  // On Windows this option is available since Windows 10 only
  sock_opt_val = count;
  if ( setsockopt ( s->fd, IPPROTO_TCP, TCP_KEEPCNT, (const void *)&sock_opt_val, sizeof ( sock_opt_val ) ) != 0 )
  {
    PRINT_ERROR ( "tcpIpSocketSetKeepAlive() : setsockopt() with TCP_KEEPCNT option failed. System error code: %i \n", tcpIpSocketGetError());
    return(0);
  }

  return(1);
}

TcpIpSocketState tcpIpSocketConnect ( TcpIpSocket *s, const char *host_addr, unsigned short host_port )
{
  int connect_ret;
  struct sockaddr_in adr;
  int fn_error_code;

  PRINT_VDEBUG ( "tcpIpSocketConnect():\n" );

  if ( !s->open )
  {
    PRINT_ERROR ( "tcpIpSocketConnect() : Socket not opened\n" );
    return TCPIPSOCKET_FAILED;
  }

  if( s->connected )
    return TCPIPSOCKET_DONE;

  memset ( &adr, 0, sizeof ( struct sockaddr_in ) );

  adr.sin_family = AF_INET;
  adr.sin_port = htons ( host_port );
  if ( inet_pton ( AF_INET, host_addr, &adr.sin_addr ) <= 0 )
  {
    PRINT_ERROR ( "tcpIpSocketConnect() : Invalid network address: %s. It cannot be converted to a binary address. System error code: %i \n", host_addr, tcpIpSocketGetError());
    s->connected = 0;
    return TCPIPSOCKET_FAILED;
  }

  connect_ret = connect ( s->fd, ( struct sockaddr * ) &adr, sizeof ( struct sockaddr ) );
  fn_error_code = tcpIpSocketGetError();
  if ( connect_ret == FN_SOCKET_ERROR && fn_error_code != FN_EISCONN ) // The connection is not established so far
  {
    if ( s->is_nonblocking &&
       ( fn_error_code == FN_EINPROGRESS || fn_error_code == FN_EALREADY || fn_error_code == FN_EWOULDBLOCK) )
    {
      PRINT_DEBUG ( "tcpIpSocketConnect() : Connection in progress to %s:%i through FD:%i\n", host_addr, host_port, s->fd);
      return TCPIPSOCKET_IN_PROGRESS;
    }
    else
    {
      s->connected = 0;
      if(fn_error_code == FN_ECONNREFUSED)
      {
         PRINT_ERROR ( "tcpIpSocketConnect() : Connection to %s:%i through FD:%i was refused\n", host_addr, host_port, s->fd);
         return TCPIPSOCKET_REFUSED;
      }
      else
      {
         PRINT_ERROR ( "tcpIpSocketConnect() : Connection to %s:%i through FD:%i failed due to error code: %i\n", host_addr, host_port, s->fd, fn_error_code);
         return TCPIPSOCKET_FAILED;
      }
    }
  }
  PRINT_DEBUG ( "tcpIpSocketConnect() : connection done to %s:%i through FD:%i\n", host_addr, host_port, s->fd);

  s->port = host_port;
  s->adr = adr;
  s->connected = 1;

  return TCPIPSOCKET_DONE;
}

int tcpIpSocketDisconnect ( TcpIpSocket *s )
{
  PRINT_VDEBUG ( "tcpIpSocketDisconnect()\n" );

  if ( !s->connected )
    return 1;

  s->connected = 0;
  if ( shutdown ( s->fd, FN_SHUT_RDWR ) != 0 )
  {
    PRINT_ERROR ( "tcpIpSocketDisconnect() : shutdown failed. System error code: %i \n", tcpIpSocketGetError());
    return 0;
  }
  return 1;
}

TcpIpSocketState tcpIpSocketCheckPort ( const char *host_addr, unsigned short host_port )
{
   TcpIpSocketState port_open, fn_ret;
   TcpIpSocket socket_struct;

   tcpIpSocketInit ( &socket_struct );
   fn_ret = tcpIpSocketOpen ( &socket_struct );
   if(fn_ret)
   {
      TcpIpSocketState socket_stat;
      socket_stat = tcpIpSocketConnect ( &socket_struct, host_addr, host_port );
      if ( socket_stat == TCPIPSOCKET_IN_PROGRESS )
         port_open = TCPIPSOCKET_FAILED;
      else
      {
         port_open = socket_stat;
         if ( socket_stat == TCPIPSOCKET_DONE )
            tcpIpSocketDisconnect (  &socket_struct );
      }

      tcpIpSocketClose ( &socket_struct );
   }
   else
      port_open = TCPIPSOCKET_FAILED;

   return port_open;
}

int tcpIpSocketBindListen( TcpIpSocket *s, const char *host_addr, unsigned short port, int backlog )
{
  PRINT_VDEBUG ( "tcpIpSocketBindListen()\n" );

  if ( !s->open )
  {
    PRINT_ERROR ( "tcpIpSocketBindListen() : Socket not opened\n" );
    return 0;
  }

  if ( !s->listening )
  {
    struct sockaddr_in adr;

    memset ( &adr, 0, sizeof ( struct sockaddr_in ) );

    adr.sin_family = AF_INET;
    adr.sin_port = htons ( port );
    adr.sin_addr.s_addr = htonl(INADDR_ANY);

    if ( inet_pton ( AF_INET, host_addr, &adr.sin_addr ) <= 0 )
    {
      PRINT_ERROR ( "tcpIpSocketBindListen() : Invalid network address: %s. It cannot be converted to a binary address. System error code: %i \n", host_addr, tcpIpSocketGetError());
      return 0;
    }


    if ( bind ( s->fd, ( struct sockaddr * ) &adr, sizeof ( struct sockaddr ) ) == FN_SOCKET_ERROR )
    {
      PRINT_ERROR ( "tcpIpSocketBindListen() : Socket bind failed. System error code: %i \n", tcpIpSocketGetError());
      return 0;
    }

    if ( listen ( s->fd, backlog ) == FN_SOCKET_ERROR )
    {
      PRINT_ERROR ( "tcpIpSocketBindListen() : Socket listen failed. System error code: %i \n", tcpIpSocketGetError());
      return 0;
    }

    struct sockaddr sa;
    socklen_t sa_len = sizeof( struct sockaddr );
    if ( getsockname(s->fd, (struct sockaddr *)&sa, &sa_len) == FN_SOCKET_ERROR )
    {
      PRINT_ERROR ( "tcpIpSocketBindListen() : getsockname() failed. System error code: %i \n", tcpIpSocketGetError());
      return 0;
    }

    struct sockaddr_in *sin = (struct sockaddr_in *)&sa;

    s->port = ntohs(sin->sin_port);
    s->adr = adr;
    s->listening = 1;
  }

  return 1;
}

TcpIpSocketState tcpIpSocketAccept ( TcpIpSocket *s, TcpIpSocket *new_s )
{
  PRINT_VDEBUG ( "tcpIpSocketAccept()\n" );

  TcpIpSocketState state = TCPIPSOCKET_DONE;

  if ( !s->open )
  {
    PRINT_ERROR ( "tcpIpSocketAccept() : Failed: Socket is not opened\n" );
    return TCPIPSOCKET_FAILED;
  }

  if ( !s->listening )
  {
    PRINT_ERROR ( "tcpIpSocketAccept() : Failed: Socket is not listening\n" );
    return TCPIPSOCKET_FAILED;
  }

  struct sockaddr_in new_adr;
  socklen_t new_adr_len = sizeof(struct sockaddr);

  int new_fd = accept ( s->fd, ( struct sockaddr * ) &new_adr, &new_adr_len );

  if ( new_fd == FN_INVALID_SOCKET )
  {
    int fn_error_code;

    fn_error_code = tcpIpSocketGetError();
    if ( s->is_nonblocking &&
       ( fn_error_code == FN_EWOULDBLOCK || fn_error_code == FN_EINPROGRESS || fn_error_code == FN_EAGAIN ) )
    {
      PRINT_DEBUG ( "tcpIpSocketAccept() : Accept in progress from port %i, new FD:%i\n", new_adr.sin_port, new_fd);
      state = TCPIPSOCKET_IN_PROGRESS;
    }
    else
    {
      PRINT_ERROR ( "tcpIpSocketAccept() : accept() failed. Error code: %d\n", fn_error_code );
      return TCPIPSOCKET_FAILED;
    }
  }

  if( new_s->open )
    tcpIpSocketClose ( new_s );

  new_s->fd = new_fd;
  new_s->adr = new_adr;
  new_s->port = s->port;
  new_s->open = 1;
  new_s->connected = 1;

  return state;
}

void printTransmissionBuffer(const char *buffer, const char *msg_info, int msg_fd, int buf_len)
{
  int i;
  PRINT_DEBUG(ANSI_COLOR_CYAN"\n%s (%i bytes fd: %i) ["ANSI_COLOR_RESET, msg_info, buf_len, msg_fd);
  for(i=0;i<buf_len;i++)
  {
    char ch=buffer[i];
    if(ch=='\0' || ch=='\t' || ch=='\r' || ch=='\n' || ch=='\a')
      ch='.';
    PRINT_DEBUG("%c",ch);
  }
  PRINT_DEBUG(ANSI_COLOR_CYAN"]\n"ANSI_COLOR_RESET);
}

TcpIpSocketState tcpIpSocketWriteBuffer ( TcpIpSocket *s, DynBuffer *d_buf )
{
  PRINT_VDEBUG ( "tcpIpSocketWriteBuffer()\n" );

  const char *data = (const char *)dynBufferGetCurrentData ( d_buf );
  int data_size = dynBufferGetRemainingDataSize ( d_buf );

  if ( !s->connected )
  {
    PRINT_ERROR ( "tcpIpSocketWriteBuffer() : Socket not connected\n" );
    return TCPIPSOCKET_FAILED;
  }

  #if CROS_DEBUG_LEVEL >= 2
  printTransmissionBuffer(data, "tcpIpSocketWriteBuffer() : Buffer", s->fd, data_size);
  #endif
  while ( data_size > 0 )
  {
    int n_written , fn_error_code;

    n_written = send ( s->fd, data, data_size, 0 );
    fn_error_code = tcpIpSocketGetError();
    if ( n_written > 0 )
    {
      dynBufferMovePoseIndicator ( d_buf, n_written );
      data = (const char *)dynBufferGetCurrentData ( d_buf );
      data_size = dynBufferGetRemainingDataSize ( d_buf );
    }
    else if ( s->is_nonblocking &&
              ( fn_error_code == FN_EWOULDBLOCK || fn_error_code == FN_EINPROGRESS || fn_error_code == FN_EAGAIN ) )
    {
      PRINT_DEBUG ( "tcpIpSocketWriteBuffer() : write in progress, %d remaining bytes\n", data_size );
      return TCPIPSOCKET_IN_PROGRESS;
    }
    else if ( fn_error_code == FN_ENOTCONN || fn_error_code == FN_ECONNRESET )
    {
      PRINT_DEBUG ( "tcpIpSocketWriteBuffer() : socket disconnected\n" );
      s->connected = 0;
      return  TCPIPSOCKET_DISCONNECTED;
    }
    else
    {
      PRINT_ERROR ( "tcpIpSocketWriteBuffer() : Write failed. Error code: %i\n", fn_error_code);
      return TCPIPSOCKET_FAILED;
    }
  }

  return TCPIPSOCKET_DONE;
}

TcpIpSocketState tcpIpSocketWriteString ( TcpIpSocket *s, DynString *d_str )
{
  PRINT_VDEBUG ( "tcpIpSocketWriteString()\n" );

  const char *data = dynStringGetCurrentData ( d_str );
  int data_size = dynStringGetRemainingDataSize ( d_str );

  if ( !s->connected )
  {
    PRINT_ERROR ( "tcpIpSocketWriteString() : Socket not connected\n" );
    return TCPIPSOCKET_FAILED;
  }
  #if CROS_DEBUG_LEVEL >= 2
  printTransmissionBuffer(data, "tcpIpSocketWriteString() : Buffer", s->fd, data_size);
  #endif
  while ( data_size > 0 )
  {
    int n_written , fn_error_code;

    n_written = send ( s->fd, data, data_size, 0 );
    fn_error_code = tcpIpSocketGetError();
    if ( n_written > 0 )
    {
      dynStringMovePoseIndicator ( d_str, n_written );
      data = dynStringGetCurrentData ( d_str );
      data_size = dynStringGetRemainingDataSize ( d_str );
    }
    else if ( s->is_nonblocking &&
              ( fn_error_code == FN_EWOULDBLOCK || fn_error_code == FN_EINPROGRESS || fn_error_code == FN_EAGAIN ) )
    {
      PRINT_DEBUG ( "tcpIpSocketWriteString() : write in progress, %d remaining bytes\n", data_size );
      return TCPIPSOCKET_IN_PROGRESS;
    }
    else if ( fn_error_code == FN_ENOTCONN || fn_error_code == FN_ECONNRESET )
    {
      PRINT_DEBUG ( "tcpIpSocketWriteString() : socket disconnectd\n" );
      s->connected = 0;
      return  TCPIPSOCKET_DISCONNECTED;
    }
    else
    {
      PRINT_ERROR ( "tcpIpSocketWriteString() : Write failed. Error code: %i\n", fn_error_code);
      return TCPIPSOCKET_FAILED;
    }
  }

  return TCPIPSOCKET_DONE;
}

TcpIpSocketState tcpIpSocketReadBuffer ( TcpIpSocket *s, DynBuffer *d_buf )
{
  size_t n_read;
  return tcpIpSocketReadBufferEx(s, d_buf, TCPIP_SOCKET_READ_BUFFER_SIZE, &n_read);
}

TcpIpSocketState tcpIpSocketReadBufferEx( TcpIpSocket *s, DynBuffer *d_buf, size_t max_size, size_t *n_reads)
{
  int recv_ret, fn_error_code;

  PRINT_VDEBUG ( "tcpIpSocketReadBufferEx()\n" );

  *n_reads = 0;
  if ( !s->connected )
  {
    PRINT_ERROR ( "tcpIpSocketReadBufferEx() : Socket not connected\n" );
    return TCPIPSOCKET_FAILED;
  }

  char *read_buf = (char *)malloc(max_size);
  if (read_buf == NULL)
  {
    PRINT_ERROR("tcpIpSocketReadBufferEx() : Out of memory allocating %lu bytes before reading from socket", (unsigned long)max_size);
    return TCPIPSOCKET_FAILED;
  }

  TcpIpSocketState state = TCPIPSOCKET_UNKNOWN;
  recv_ret = recv ( s->fd, read_buf, max_size, 0);
  fn_error_code = tcpIpSocketGetError();
  if ( recv_ret == 0 )
  {
    PRINT_DEBUG ( "tcpIpSocketReadBufferEx() : socket disconnectd\n" );
    s->connected = 0;
    state = TCPIPSOCKET_DISCONNECTED;
  }
  else if ( recv_ret > 0 )
  {
    PRINT_DEBUG ( "tcpIpSocketReadBufferEx() : read %d bytes \n", recv_ret );
    #if CROS_DEBUG_LEVEL >= 2
    printTransmissionBuffer((const char *)read_buf, "tcpIpSocketReadBufferEx() : Buffer", s->fd, recv_ret);
    #endif

    dynBufferPushBackBuf ( d_buf, (const unsigned char *)read_buf, recv_ret );
    state = TCPIPSOCKET_DONE;
    *n_reads = recv_ret;
  }
  else if ( s->is_nonblocking &&
            ( fn_error_code == FN_EWOULDBLOCK || fn_error_code == FN_EINPROGRESS || fn_error_code == FN_EAGAIN ) )
  {
    PRINT_DEBUG ( "tcpIpSocketReadBufferEx() : read in progress\n" );
    state = TCPIPSOCKET_IN_PROGRESS;
  }
  else if ( fn_error_code == FN_ENOTCONN || fn_error_code == FN_ECONNRESET )
  {
    PRINT_DEBUG ( "tcpIpSocketReadBufferEx() : socket disconnectd\n" );
    s->connected = 0;
    state = TCPIPSOCKET_DISCONNECTED;
  }
  else
  {
    PRINT_ERROR ( "tcpIpSocketReadBufferEx() : Read through socket failed. Error code: %i\n", fn_error_code);
    state = TCPIPSOCKET_FAILED;
  }

  free(read_buf);

  return state;
}

TcpIpSocketState tcpIpSocketReadString ( TcpIpSocket *s, DynString *d_str )
{
  int recv_ret, fn_error_code;

  PRINT_VDEBUG ( "tcpIpSocketReadString()\n" );

  if ( !s->connected )
  {
    PRINT_ERROR ( "tcpIpSocketReadString() : Socket not connected\n" );
    return TCPIPSOCKET_FAILED;
  }

  char read_buf[TCPIP_SOCKET_READ_BUFFER_SIZE];

  TcpIpSocketState state = TCPIPSOCKET_UNKNOWN;

  recv_ret = recv ( s->fd, read_buf, TCPIP_SOCKET_READ_BUFFER_SIZE , 0 );
  fn_error_code = tcpIpSocketGetError();
  if ( recv_ret == 0 )
  {
    PRINT_DEBUG ( "tcpIpSocketReadString() : socket disconnectd\n" );
    s->connected = 0;
    state = TCPIPSOCKET_DISCONNECTED;
  }
  else if ( recv_ret > 0 )
  {
    PRINT_DEBUG ( "tcpIpSocketReadString() : read %d bytes \n", recv_ret );
    #if CROS_DEBUG_LEVEL >= 2
    printTransmissionBuffer(read_buf, "tcpIpSocketReadString() : Buffer", s->fd, recv_ret);
    #endif

    dynStringPushBackStrN ( d_str, read_buf, recv_ret );
    state = TCPIPSOCKET_DONE;
  }
  else if ( s->is_nonblocking &&
            ( fn_error_code == FN_EWOULDBLOCK || fn_error_code == FN_EINPROGRESS || fn_error_code == FN_EAGAIN ) )
  {
    PRINT_DEBUG ( "tcpIpSocketReadString() : read in progress\n" );
    state = TCPIPSOCKET_IN_PROGRESS;
  }
  else if ( fn_error_code == FN_ENOTCONN || fn_error_code == FN_ECONNRESET )
  {
    PRINT_DEBUG ( "tcpIpSocketReadString() : socket disconnectd\n" );
    s->connected = 0;
    state = TCPIPSOCKET_DISCONNECTED;
  }
  else
  {
    PRINT_ERROR ( "tcpIpSocketReadString() : Read failed. Error code: %i\n", fn_error_code);
    state = TCPIPSOCKET_FAILED;
  }

  return state;
}

int tcpIpSocketGetFD ( TcpIpSocket *s )
{
  return s->fd;
}

unsigned short tcpIpSocketGetPort( TcpIpSocket *s )
{
  return s->port;
}

int tcpIpSocketSelect( int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, uint64_t time_out )
{
  struct timeval timeout_tv;
  int nfds_set;

  timeout_tv = cRosClockGetTimeVal( time_out );

  nfds_set = select(nfds, readfds, writefds, exceptfds, &timeout_tv);

  if(nfds_set == FN_SOCKET_ERROR) // It is not needed, but it is recommended on Windows
    nfds_set = -1;

  if(nfds_set == -1)
  {
    int socket_err_num;

    socket_err_num = tcpIpSocketGetError();

    if(socket_err_num == FN_EINTR)
    {
      PRINT_DEBUG("tcpIpSocketSelect() : select() returned EINTR error code\n");
      nfds_set = 0;
    }
  }

  return(nfds_set);
}

#define REQUESTED_WS_HIGH_VER 2
#define REQUESTED_WS_LOW_VER 0
int tcpIpSocketStartUp( void )
{
  int ret_val;

  if(socket_lib_initialized == 0) // If the library is not aready initialized:
  {
#ifdef _WIN32
    WORD ws_ver_requested;
    WSADATA ws_ver_obtained;

    ws_ver_requested = MAKEWORD(REQUESTED_WS_HIGH_VER, REQUESTED_WS_LOW_VER); // Codify the requested version of Winsock
    ret_val = WSAStartup(ws_ver_requested, &ws_ver_obtained);
    if(ret_val != 0)
    {
      // Ckeck that WinSock supports the requeste version
      if(LOBYTE(ws_ver_obtained.wVersion) != REQUESTED_WS_HIGH_VER || HIBYTE(ws_ver_obtained.wVersion) != REQUESTED_WS_LOW_VER)
        PRINT_ERROR("tcpIpSocketStartUp(): Could not find the required version of Winsock: %i.%i.\n", REQUESTED_WS_HIGH_VER, REQUESTED_WS_LOW_VER);
    }
    else
      PRINT_ERROR ( "tcpIpSocketStartUp(): Loading of Winsock failed with error code: %i.\n", ret_val);
#else
    // Ignore the SIGPIPE signal. That is, prevent the process from being terminated when it tries to write to a closed socket
    signal(SIGPIPE, SIG_IGN);
    // In Windows this default behavior does not occur, so nothing has to be done
    ret_val = 0; // The socket library does not have to be initialized on Linux or OS X
#endif
    if(ret_val == 0)
      socket_lib_initialized++;
  }
  else
    ret_val = 0;

  return(ret_val);
}

int tcpIpSocketCleanUp( void )
{
  int ret_val;

  if(socket_lib_initialized != 0) // If the library is initialized:
  {
#ifdef _WIN32
    ret_val = WSACleanup();
#else
    ret_val = 0; // The socket library does not need to be initialized on Linux or OS X
#endif
    if(ret_val == 0)
      socket_lib_initialized--;
  }
  else
    ret_val = 0;

  return(ret_val);
}

int tcpIpSocketGetError( void )
{
  int socket_err_num;

#ifdef _WIN32
  socket_err_num = WSAGetLastError();
#else
  socket_err_num = errno;
#endif

  return(socket_err_num);
}

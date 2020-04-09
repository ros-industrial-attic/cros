#ifndef _TCPIP_SOCKET_H_
#define _TCPIP_SOCKET_H_

// _WIN32 is defined when compiling 32 bit and 64 bit applications, so _WIN64 does not have to be checked here
#ifdef _WIN32
#  include <winsock2.h>
#  define FN_SOCKET_ERROR SOCKET_ERROR
#  define FN_INVALID_SOCKET INVALID_SOCKET

#  define FN_ERROR_INVALID_PARAMETER ERROR_INVALID_PARAMETER
#  define MAX_HOST_NAME_LEN 256
#else
#  include <netinet/in.h>
#  include <errno.h>
#  ifndef __USE_POSIX
#    define __USE_POSIX
#  endif
#  include <limits.h>
#  define MAX_HOST_NAME_LEN _POSIX_HOST_NAME_MAX
#  define FN_SOCKET_ERROR (-1)
#  define FN_INVALID_SOCKET (-1)

#  define FN_ERROR_INVALID_PARAMETER ENOSPC
#  define MAX_HOST_NAME_LEN _POSIX_HOST_NAME_MAX
#endif

#include "dyn_string.h"
#include "dyn_buffer.h"

/*! \defgroup tcpip_socket TcpIp socket */

/*! \addtogroup tcpip_socket
 *  @{
 */

typedef enum
{
  TCPIPSOCKET_FAILED = 0,
  TCPIPSOCKET_IN_PROGRESS,
  TCPIPSOCKET_DISCONNECTED,
  TCPIPSOCKET_DONE,

  TCPIPSOCKET_UNKNOWN,
  TCPIPSOCKET_REFUSED
} TcpIpSocketState;

/*! \brief TcpIpSocket object. Don't modify directly its internal members: use
 *         the related functions instead */
typedef struct TcpIpSocket TcpIpSocket;
struct TcpIpSocket
{
  int fd; //! File descriptor for the new socket
  struct sockaddr_in rem_addr; //! Adress of the (remote) host to which this socket is connected or address to which the socket is binded (if it is listening)
  unsigned char open; //! It is 1 if the socket has been already created (successful socket() funciton call). Otherwise it is 0
  unsigned char connected; //! It is 1 if the socket is connected (inbound or outbound). Otherwise it is 0
  unsigned char listening; //! It is 1 if the socket is already in the listening state (ready to accept connections). Otherwise it is 0
  unsigned char is_nonblocking; //! It is 1 if the socket has been configured as non blocking. Otherwise it is 0
};

/*! \brief Initialize the TcpIpSocket object with default values
 *
 *  \param s Pointer to a TcpIpSocket object
 */
void tcpIpSocketInit( TcpIpSocket *s );

/*! \brief Open a TCP/IP4 socket
 *
 *  \param s Pointer to a TcpIpSocket object
 *
 *  \return Returns 1 on success, 0 on failure
 */
int tcpIpSocketOpen( TcpIpSocket *s );

/*! \brief Close a socket
 *
 *  \param s Pointer to a TcpIpSocket object
 *
 *  \return Returns 1 on success, 0 on failure
 */
int tcpIpSocketClose( TcpIpSocket *s );

/*! \brief Set non-blocking operation for a TCP/IP4 socket
 *
 *  \param s Pointer to a TcpIpSocket object
 *
 *  \return Returns 1 on success, 0 on failure
 */
int tcpIpSocketSetNonBlocking( TcpIpSocket *s );

/*! \brief Set no-delay operation for a socket, that is, disables the Nagle's algorithm.
 *         This option is intended for applications that require low latency on every packet sent
 *
 *  \param s Pointer to a TcpIpSocket object
 *
 *  \return Returns 1 on success, 0 on failure
 */
int tcpIpSocketSetNoDelay ( TcpIpSocket *s );

/*! \brief Set a TCP/IP4 socket to be re-bound immediately without timeout
 *
 *  \param s Pointer to a TcpIpSocket object
 *
 *  \return Returns 1 on success, 0 on failure
 */
int tcpIpSocketSetReuse ( TcpIpSocket *s );

/*! \brief Set a TCP/IP4 socket to prevent disconnection
 *
 *  \param s Pointer to a TcpIpSocket object
 *  \param idle The interval between the last data packet sent and the first keepalive probe
 *  \param interval The interval between subsequential keepalive probes
 *  \param count The number of unacknowledged probes to send before considering the connection dead
 *
 *  \return Returns 1 on success, 0 on failure
 */
int tcpIpSocketSetKeepAlive( TcpIpSocket *s, unsigned int idle, unsigned int interval, unsigned int count );

/*! \brief Connect a TCP/IP4 socket to a server
 *
 *  \param s Pointer to a TcpIpSocket object
 *  \param host The server address
 *  \param port The server port

 *  \return Returns TCPIPSOCKET_DONE on success,
 *          TCPIPSOCKET_IN_PROGRESS if the connection is not yet completed,
 *          or TCPIPSOCKET_FAILED on failure
 */
TcpIpSocketState tcpIpSocketConnect( TcpIpSocket *s, const char *host, unsigned short port );

/*! \brief Checks if a network port is open is a host address.
 *
 *  This function tries to connect to a target port and reports the success. If the connection
 *  is established it is closed immediately.
 *  \param host_addr The target address
 *  \param host_port The target port

 *  \return Returns TCPIPSOCKET_DONE if the port is open,
 *          TCPIPSOCKET_REFUSED if the connection was refused (e.g. the target port is closed),
 *          or TCPIPSOCKET_FAILED on failure
 */
TcpIpSocketState tcpIpSocketCheckPort ( const char *host_addr, unsigned short host_port );

/*! \brief Bind and listen for TCP/IP4  socket connections
 *
 *  \param s Pointer to a TcpIpSocket object
 *  \param host The address to be bound to the socket
 *  \param port The port to be bound to the socket
 *  \param backlog the maximum length of the listen queue
 *
 *  \return Returns 1 on success, 0 on failure
 */
int tcpIpSocketBindListen( TcpIpSocket *s, const char *host, unsigned short port, int backlog );

/*! \brief Accept a new connection on a TCP/IP4  socket
 *
 *  \param s Pointer to a TcpIpSocket object used to accept new connection
 *           (it should be opened and listening)
 *  \param new_s Pointer to a TcpIpSocket object used to return the accepted connection
 *
 *  \return Returns TCPIPSOCKET_DONE on success,
 *          TCPIPSOCKET_IN_PROGRESS if the connection is not yet completed,
 *          or TCPIPSOCKET_FAILED on failure
 */
TcpIpSocketState tcpIpSocketAccept( TcpIpSocket *s, TcpIpSocket *new_s );

/*! \brief Shutdown a connectd TCP/IP4 socket to a server
 *
 *  \param s Pointer to a TcpIpSocket object
 *
 *  \return Returns 1 on success, 0 on failure
 */
int tcpIpSocketDisconnect( TcpIpSocket *s );

/*! \brief Send a binary message on a connected socket
 *
 *  \param s Pointer to a TcpIpSocket object
 *  \param d_buf The dynamic buffer to be written
 *
 *  \return Returns TCPIPSOCKET_DONE on success,
 *          TCPIPSOCKET_IN_PROGRESS (only if the socket is non-blocking)
 *          if the write operation is not yet completed,
 *          TCPIPSOCKET_DISCONNECTED if the socket has been disconnectd,
 *          or TCPIPSOCKET_FAILED on failure
 */
TcpIpSocketState tcpIpSocketWriteBuffer( TcpIpSocket *s, DynBuffer *d_buf );

/*! \brief Send a string on a connected socket
 *
 *  \param s Pointer to a TcpIpSocket object
 *  \param d_str The dynamic string to be written
 *
 *  \return Returns TCPIPSOCKET_DONE on success,
 *          TCPIPSOCKET_IN_PROGRESS (only if the socket is non-blocking)
 *          if the write operation is not yet completed,
 *          TCPIPSOCKET_DISCONNECTED if the socket has been disconnectd,
 *          or TCPIPSOCKET_FAILED on failure
 */
TcpIpSocketState tcpIpSocketWriteString( TcpIpSocket *s, DynString *d_str );

/*! \brief Receive a binary message from a connected socket
 *
 *  \param s Pointer to a TcpIpSocket object
 *  \param d_buf Pointer to the input dynamic string
 *
 *  \return Returns TCPIPSOCKET_DONE on success,
 *          TCPIPSOCKET_IN_PROGRESS (only if the socket is non-blocking)
 *          if the read operation would block,
 *          TCPIPSOCKET_DISCONNECTED if the socket has been disconnectd,
 *          or TCPIPSOCKET_FAILED on failure
 */
TcpIpSocketState tcpIpSocketReadBufferEx( TcpIpSocket *s, DynBuffer *d_buf, size_t length, size_t *reads);

/*! \brief Receive a binary message from a connected socket
 *
 *  \param s Pointer to a TcpIpSocket object
 *  \param d_buf Pointer to the input dynamic string
 *
 *  \return Returns TCPIPSOCKET_DONE on success,
 *          TCPIPSOCKET_IN_PROGRESS (only if the socket is non-blocking)
 *          if the read operation would block,
 *          TCPIPSOCKET_DISCONNECTED if the socket has been disconnectd,
 *          or TCPIPSOCKET_FAILED on failure
 */
TcpIpSocketState tcpIpSocketReadBuffer( TcpIpSocket *s, DynBuffer *d_buf );

/*! \brief Receive a string from a connected socket
 *
 *  \param s Pointer to a TcpIpSocket object
 *  \param d_str Pointer to the input dynamic string
 *
 *  \return Returns TCPIPSOCKET_DONE on success,
 *          TCPIPSOCKET_IN_PROGRESS (only if the socket is non-blocking)
 *          if the read operation would block,
 *          TCPIPSOCKET_DISCONNECTED if the socket has been disconnectd,
 *          or TCPIPSOCKET_FAILED on failure
 */
TcpIpSocketState tcpIpSocketReadString( TcpIpSocket *s, DynString *d_str );

/*! \brief Return the file descriptor associated with the TcpIpSocket object
 *
 *  \param s A TcpIpSocket object
 *
 *  \return Returns the file descriptor
 */
int tcpIpSocketGetFD( TcpIpSocket *s );

/*! \brief Return the current (local) port to which the TcpIpSocket object is associated.
 *
 *  \param s A TcpIpSocket object
 *
 *  \return Returns the TCP/IP port number.
 */
unsigned short tcpIpSocketGetPort( TcpIpSocket *s );

/*! \brief Return the (remote) port to which the TcpIpSocket object is connected, or the
 *         port to which the socket is binded in case of being a listening port.
 *
 *  \param s A TcpIpSocket object.
 *
 *  \return Returns the TCP/IP port number.
 */
unsigned short tcpIpSocketGetRemotePort( TcpIpSocket *s );

/*! \brief Return the (remote) address to which the TcpIpSocket object is connected, or the
 *         address to which the socket is binded in case of being a listening port.
 *
 *  \param s A TcpIpSocket object.
 *
 *  \return Returns a pointer to the address string or NULL if the address string could not be obtained.
 */
const char *tcpIpSocketGetRemoteAddress( TcpIpSocket *s );

/*! \brief Check several file descriptors simultaneously waiting until at least one of them is ready
 *          for reading, writing or atteding an exceptional condition. That is, the select() function is called.
 *
 *  \return Returns the error code
 */
int tcpIpSocketSelect( int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, uint64_t time_out );

/*! \brief Initiate the use of the TcpIpSocket library.
 *         This function must be called when before using the tcpIpSocketStartUp library functions.
 *
 *  \return Returns 0 on success or an error code in case of failure.
 */
int tcpIpSocketStartUp( void );

/*! \brief Terminate the use of the TcpIpSocket library.
 *         This function must be called when the application called tcpIpSocketStartUp and it does not need to use the library anymore to free resources.
 *
 *  \return Returns 0 on success or FN_SOCKET_ERROR in case of failure.
 */
int tcpIpSocketCleanUp( void );

/*! \brief Get the error code returned by the last socket-function call that failed.
 *         So this code is significant only when the return value of the call indicated an error.
 *
 *  \return Returns the number of file descriptors that this function has set sets in the fd_set variables or -1 on error.
 */
int tcpIpSocketGetError( void );


/*! @}*/

#endif

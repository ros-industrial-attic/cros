#ifndef _TCPIP_SOCKET_H_
#define _TCPIP_SOCKET_H_

# include <arpa/inet.h>

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
  
  TCPIPSOCKET_UNKNOWN
} TcpIpSocketState;

/*! \brief TcpIpSocket object. Don't modify directly its internal members: use
 *         the related functions instead */
typedef struct TcpIpSocket TcpIpSocket;
struct TcpIpSocket
{
  int fd; 
  unsigned short port;
  struct sockaddr_in adr;
  unsigned char open, connected, 
                listening, is_nonblocking;
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
 */
void tcpIpSocketClose( TcpIpSocket *s );

/*! \brief Set non-blocking operation for a TCP/IP4 socket 
 * 
 *  \param s Pointer to a TcpIpSocket object
 * 
 *  \return Returns 1 on success, 0 on failure
 */
int tcpIpSocketSetNonBlocking( TcpIpSocket *s );

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
TcpIpSocketState  tcpIpSocketAccept( TcpIpSocket *s, TcpIpSocket *new_s );

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

/*! \brief Return the port associated with the TcpIpSocket object
 * 
 *  \param s A TcpIpSocket object
 * 
 *  \return Returns the tcp/ip port
 */
unsigned short tcpIpSocketGetPort( TcpIpSocket *s );

/*! @}*/

#endif
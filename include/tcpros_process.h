#ifndef _TCPROS_PROCESS_H_
#define _TCPROS_PROCESS_H_

#include "tcpip_socket.h"

/*! \defgroup tcpros_process TCPROS process */

/*! \addtogroup tcpros_process
 *  @{
 */

typedef enum
{
  TCPROS_PROCESS_STATE_IDLE, // 0
  TCPROS_PROCESS_STATE_WAIT_FOR_CONNECTING, // 1
  TCPROS_PROCESS_STATE_CONNECTING, // 2
  TCPROS_PROCESS_STATE_READING_HEADER_SIZE, // 3
  TCPROS_PROCESS_STATE_READING_HEADER, // 4
  TCPROS_PROCESS_STATE_WRITING_HEADER, // 5
  TCPROS_PROCESS_STATE_WAIT_FOR_WRITING, // 6
  TCPROS_PROCESS_STATE_START_WRITING, // 7
  TCPROS_PROCESS_STATE_READING_SIZE, // 8
  TCPROS_PROCESS_STATE_READING, // 9
  TCPROS_PROCESS_STATE_WRITING // A
} TcprosProcessState;

/*! \brief The TcprosProcess object represents a client or server connection used to manage
 *         peer to peer TCPROS connections between nodes. It is internally used to emulate the
 *         "process descriptor" in a multi-task system (here used in a mono task system), including
 *         the process file descriptors (i.e., a socket), process memory and the state.
 *         NOTE: this is a cROS internal object, usually you don't need to use it.
 */
typedef struct TcprosProcess TcprosProcess;
struct TcprosProcess
{
  TcprosProcessState state;             //! The state of the process. When it is TCPROS_PROCESS_STATE_WAIT_FOR_WRITING the publisher/caller waits until a new message must be sent (periodic or immediate sending)
  TcpIpSocket socket;                   //! The socket used for the TCPROS or RPCROS communication
  DynString topic;                      //! The name of the topic
  DynString service;                    //! The name of the service
  DynString type;                       //! The message/service type
  DynString servicerequest_type;        //! The service request type
  DynString serviceresponse_type;       //! The service response type
  DynString md5sum;                     //! The MD5 sum of the message type
  DynString caller_id;                  //! The name of subscriber or service caller
  unsigned char latching;               //! If 1, the publisher is sending latched messages. Otherwise 0
  unsigned char tcp_nodelay;            //! If 1, the publisher should set TCP_NODELAY on the socket, if possible. Otherwise 0
  unsigned char persistent;             //! If 1, the service connection should be kept open for multiple requests. Otherwise it should be 0
  DynBuffer packet;                     //! The incoming/outgoing TCPROS packet
  uint64_t last_change_time;            //! Last state change time (in ms)
  int topic_idx;                        //! Index used to associate the process to a publisher or a subscriber
  int service_idx;                      //! Index used to associate the process to a service provider or a service client
  size_t left_to_recv;                  //! Remaining to receive
  uint8_t ok_byte;						          //! 'ok' byte send by a service provider in response to the last service request
  int probe;							              //! The current session is a probing one
  int sub_tcpros_port;                  //! Port (obtained from a publisher node) to which the process must connect
  char *sub_tcpros_host;                //! Host (obtained from a publisher node) to which the process must connect
};


/*! \brief Initialize an TcprosProcess object, starting to allocate memory
 *         and settings default values for the objects' fields
 *
 *  \param s Pointer to TcprosProcess object to be initialized
 */
void tcprosProcessInit( TcprosProcess *p );

/*! \brief Release all the internally allocated memory of an TcprosProcess object
 *
 *  \param s Pointer to TcprosProcess object to be released
 */
void tcprosProcessRelease( TcprosProcess *p );

/*! \brief Clear internal i/o data buffer (packet) of a TcprosProcess object
 *
 *  \param s Pointer to TcprosProcess object
 */
void tcprosProcessClear( TcprosProcess *p );

/*! \brief Reset the state and clear all the content of a TcprosProcess object (the
 *         internal memory IS NOT released, so we can continue using the process)
 *
 *  \param s Pointer to TcprosProcess object
 */
void tcprosProcessReset( TcprosProcess *p );

/*! \brief Change the internal state of an TcprosProcess object, and update its timer
 *
 *  \param s Pointer to TcprosProcess object
 *  \param state The new state
 */
void tcprosProcessChangeState( TcprosProcess *p, TcprosProcessState state );

/*! @}*/

#endif

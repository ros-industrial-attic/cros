#ifndef _TCPROS_PROCESS_H_
#define _TCPROS_PROCESS_H_

#include "tcpip_socket.h"

/*! \defgroup tcpros_process TCPROS process */

/*! \addtogroup tcpros_process
 *  @{
 */

typedef enum
{
  TCPROS_PROCESS_STATE_IDLE,
  TCPROS_PROCESS_STATE_CONNECTING,
  TCPROS_PROCESS_STATE_READING_HEADER_SIZE,
  TCPROS_PROCESS_STATE_READING_HEADER,
  TCPROS_PROCESS_STATE_WRITING_HEADER,
  TCPROS_PROCESS_STATE_WAIT_FOR_WRITING,
  TCPROS_PROCESS_STATE_START_WRITING,
  TCPROS_PROCESS_STATE_READING_SIZE,
  TCPROS_PROCESS_STATE_READING,
  TCPROS_PROCESS_STATE_WRITING
}TcprosProcessState;

/*! \brief The TcprosProcess object represents a client or server connection used to manage 
 *         peer to peer TCPROS connections between nodes. It is internally used to emulate the 
 *         "precess descriptor" in a multitask system (here used in a mono task system), including 
 *         the process file descriptors (i.e., a socket), proecess memory and the state.
 *         NOTE: this is a cROS internal object, usually you don't need to use it.
 */
typedef struct TcprosProcess TcprosProcess;
struct TcprosProcess
{
  TcprosProcessState state;             //! The state
  TcpIpSocket socket;                   //! The socket used for the TCPROS communication
  DynString topic;                      //! The name of the topic
  DynString type;                       //! The message type
  DynString md5sum;                     //! The md5sum of the message type
  DynString caller_id;                  //! The name of subscriber
  unsigned char latching;               //! If 1, the publisher is sending latched messages
  unsigned char tcp_nodelay;            //! If 1, the publisher should set TCP_NODELAY on the socket, if possible.
  unsigned char persistent;             //! If 1, the service connection should be kept open for multiple requests
  DynBuffer packet;                     //! The incoming/outoming TCPROS packet
  uint64_t last_change_time;            //! Last state change time (in ms)
  uint64_t wake_up_time_ms;             //! The time for the next automatic cycle (in msec, since the Epoch) 
  int topic_idx;                         //! Index used to associate the process to a publisher or a subscribed
  size_t left_to_recv;                  //! Remaining to recevice
};


/*! \brief Initialize an TcprosProcess object, starting to allocate memory 
 *         and settins default values for the objects' fields
 * 
 *  \param s Pointer to TcprosProcess object to be initialized
 */
void tcprosProcessInit( TcprosProcess *p );

/*! \brief Release all the internally allocated memory of an TcprosProcess object
 * 
 *  \param s Pointer to TcprosProcess object to be released
 */
void tcprosProcessRelease( TcprosProcess *p );

/*! \brief Clear internal data of an TcprosProcess object (the internal memory IS NOT released)
 * 
 *  \param s Pointer to TcprosProcess object
 */
void tcprosProcessClear( TcprosProcess *p );

/*! \brief Change the internal state of an TcprosProcess object, and update its timer
 * 
 *  \param s Pointer to TcprosProcess object
 *  \param state The new state
 */
void tcprosProcessChangeState( TcprosProcess *p, TcprosProcessState state );

/*! @}*/

#endif

#ifndef _XMLRPC_PROCESS_H_
#define _XMLRPC_PROCESS_H_

#include "tcpip_socket.h"
#include "xmlrpc_protocol.h"

/*! \defgroup xmlrpc_process XMLRPC process */

/*! \addtogroup xmlrpc_process
 *  @{
 */

typedef enum
{
  XMLRPC_PROCESS_STATE_IDLE,
  XMLRPC_PROCESS_STATE_READING,
  XMLRPC_PROCESS_STATE_WRITING
}XmlrpcProcessState;

/*! \brief The XmlrpcProcess object represents a client or server connection between 
 *         the node and the roscore node or other nodes. It is internally used to emulate the 
 *         "precess descriptor" in a multitask system (here used in a mono task system), including 
 *         the process file descriptors (i.e., a socket), proecess memory and the state.
 *         NOTE: this is a cROS internal object, usually you don't need to use it.
 */
typedef struct XmlrpcProcess XmlrpcProcess;
struct XmlrpcProcess
{
  XmlrpcProcessState state;             //! The state
  TcpIpSocket socket;                   //! The socket used for the XMLRPC communication
  /*! Indentification number used to identify the called function (i.e., the request)
   *  after the response */
  int request_id;             
  XmlrpcMessageType message_type;       //! The incoming/outoming XMLRPC message type
  DynString method;                     //! The incoming/outoming XMLRPC method
  XmlrpcParamVector params;             //! The incoming/outoming XMLRPC parameters
   /*! The incoming/outoming XMLRPC message 
    *  (e.g., generated using generateXmlrpcMessage() ) */
  DynString message;                   
  uint64_t last_change_time;            //! Last state change time (in ms)
  uint64_t wake_up_time_ms;             //! The time for the next automatic cycle (in msec, since the Epoch) 
};


/*! \brief Initialize an XmlrpcProcess object, starting to allocate memory 
 *         and settins default values for the objects' fields
 * 
 *  \param s Pointer to XmlrpcProcess object to be initialized
 */
void xmlrpcProcessInit( XmlrpcProcess *p );

/*! \brief Release all the internally allocated memory of an XmlrpcProcess object
 * 
 *  \param s Pointer to XmlrpcProcess object to be released
 */
void xmlrpcProcessRelease( XmlrpcProcess *p );

/*! \brief Clear internal data of an XmlrpcProcess object (the internal memory IS NOT released)
 * 
 *  \param s Pointer to XmlrpcProcess object
 */
void xmlrpcProcessClear( XmlrpcProcess *p );

/*! \brief Change the internal state of an XmlrpcProcess object, and update its timer
 * 
 *  \param s Pointer to XmlrpcProcess object
 *  \param state The new state
 */
void xmlrpcProcessChangeState( XmlrpcProcess *p, XmlrpcProcessState state );

/*! @}*/

#endif
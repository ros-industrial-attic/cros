#ifndef _TCPROS_TAGS_H_
#define _TCPROS_TAGS_H_

/*! \defgroup tcpros_tags TCPROS tags */

/*! \addtogroup tcpros_tags
 *  @{
 */

/*! \brief Structure that contains a TCPROS tag string and the related string length
 */
typedef struct 
{
  char *str;
  int dim;
}TcprosTagStrDim;

static TcprosTagStrDim TCPROS_CALLERID_TAG = { "callerid=", 9 };
static TcprosTagStrDim TCPROS_TOPIC_TAG = { "topic=", 6 };
static TcprosTagStrDim TCPROS_TYPE_TAG = { "type=", 5 };
static TcprosTagStrDim TCPROS_MD5SUM_TAG = { "md5sum=", 7 };
static TcprosTagStrDim TCPROS_MESSAGE_DEFINITION_TAG = { "message_definition=", 19 };
static TcprosTagStrDim TCPROS_SERVICE_TAG = { "service=", 8 }; // WARNING Not implemented
static TcprosTagStrDim TCPROS_TCP_NODELAY_TAG = { "tcp_nodelay=", 12 };
static TcprosTagStrDim TCPROS_LATCHING_TAG = { "latching=", 9 }; // WARNING Not implemented
static TcprosTagStrDim TCPROS_PERSISTENT_TAG = { "persistent=", 11 }; // WARNING Not implemented
static TcprosTagStrDim TCPROS_ERROR_TAG = { "error=", 6 };

enum
{
  TCPROS_CALLER_ID_FLAG = 0x1,
  TCPROS_TOPIC_FLAG = 0x2,
  TCPROS_TYPE_FLAG = 0x4,
  TCPROS_MD5SUM_FLAG = 0x8,  
  TCPROS_MESSAGE_DEFINITION_FLAG = 0x10,
  TCPROS_SERVICE_FLAG = 0x20,
  TCPROS_TCP_NODELAY_FLAG = 0x40,
  TCPROS_LATCHING_FLAG = 0x80,
  TCPROS_PERSISTENT_FLAG = 0x100,
  TCPROS_ERROR_FLAG = 0x200,
  TCPROS_DATA_FLAG = 0x400  
};

static const uint32_t TCPROS_SUBCRIPTION_HEADER_FLAGS = TCPROS_MESSAGE_DEFINITION_FLAG |
                                                        TCPROS_CALLER_ID_FLAG |
                                                        TCPROS_TOPIC_FLAG |
                                                        TCPROS_MD5SUM_FLAG |
                                                        TCPROS_TYPE_FLAG;

// http://wiki.ros.org/ROS/TCPROS doesn't mention message_definition, caller_id
// or the topic as compulsory
static const uint32_t TCPROS_PUBLICATION_HEADER_FLAGS = TCPROS_TYPE_FLAG |
                                                        TCPROS_MD5SUM_FLAG;
                                    
static const uint32_t TCPROS_PUBLICATION_PACKET_FLAGS = TCPROS_CALLER_ID_FLAG |
                                                        TCPROS_TOPIC_FLAG |
                                                        TCPROS_TYPE_FLAG |
                                                        TCPROS_MD5SUM_FLAG |
                                                        TCPROS_MESSAGE_DEFINITION_FLAG | 
                                                        TCPROS_DATA_FLAG;
                                                        
/*! @}*/

#endif
#ifndef _CROS_MESSAGE_H_
#define _CROS_MESSAGE_H_

#include "cros_node.h"

#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>

/*! \defgroup cros_message cROS TCPROS 
 * 
 *  Implemenation of the TCPROS protocol for topic message exchanges
 */

/*! \addtogroup cros_message
 *  @{
 */

static const char* FILEEXT_MSG = "msg";

// e.g. std_msgs/String
static const char* CHAR_SEP = "/";

// character that designates a constant assignment rather than a field
static const char* CHAR_CONST = "=";

// char that denotes the comment line start
static const char* CHAR_COMMENT = "#";

static const char* HEADER_DEFAULT_PACK = "std_msgs";
static const char* HEADER_DEFAULT_NAME = "Header";
static const char* HEADER_DEFAULT_TYPE = "std_msgs/Header";

static const char* HEADER_DEFAULT_TYPEDEF = "\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data\n\
# in a particular coordinate frame.\n\
#\n\
# sequence ID: consecutively increasing ID\n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";

typedef enum CrosMessageType
{
  CROS_CUSTOM_TYPE = 0,
  CROS_STD_MSGS_INT8,
  CROS_STD_MSGS_UINT8,
  CROS_STD_MSGS_INT16,
  CROS_STD_MSGS_UINT16,
  CROS_STD_MSGS_INT32,
  CROS_STD_MSGS_UINT32,
  CROS_STD_MSGS_INT64,
  CROS_STD_MSGS_UINT64,
  CROS_STD_MSGS_FLOAT32,
  CROS_STD_MSGS_FLOAT64,
  CROS_STD_MSGS_STRING,
  CROS_STD_MSGS_BOOL,
  CROS_STD_MSGS_TIME,
  CROS_STD_MSGS_DURATION,
  CROS_STD_MSGS_HEADER,
  // deprecated
  CROS_STD_MSGS_CHAR,
  CROS_STD_MSGS_BYTE
} CrosMessageType;

struct t_msgFieldDef
{
    CrosMessageType type;
    char* type_s;
    char* name;
    int is_array;
    int array_size;
    struct t_msgFieldDef* prev;
    struct t_msgFieldDef* next;
};

typedef struct t_msgFieldDef msgFieldDef;

struct t_msgConst
{
    CrosMessageType type;
    char* type_s;
    char* name;
    char* value;
    struct t_msgConst* prev;
    struct t_msgConst* next;
};

typedef struct t_msgConst msgConst;

struct t_msgDef
{
    char* name;
    char* package;
    char* root_dir;
    char* plain_text;
    msgFieldDef* fields;
    msgFieldDef* first_field;
    msgConst* constants;
    msgConst* first_const;
};

typedef struct t_msgDef cRosMessageDef;

struct t_msgDep
{
    cRosMessageDef* msg;
    struct t_msgDep* prev;
    struct t_msgDep* next;
};

typedef struct t_msgDep msgDep;

typedef struct cRosMessageField cRosMessageField;
typedef struct cRosMessage cRosMessage;

struct cRosMessageField
{
    char *name;

    union data
    {
      uint8_t opaque[8];
      int8_t as_int8;
      uint8_t as_uint8;
      int16_t as_int16;
      uint16_t as_uint16;
      int32_t as_int32;
      uint32_t as_uint32;
      int64_t as_int64;
      uint64_t as_uint64;
      float as_float32;
      double as_float64;
      char *as_string;
      cRosMessage *as_msg;
      int8_t *as_int8_array;
      uint8_t *as_uint8_array;
      int16_t *as_int16_array;
      uint16_t *as_uint16_array;
      int32_t *as_int32_array;
      uint32_t *as_uint32_array;
      int64_t *as_int64_array;
      uint64_t *as_uint64_array;
      float *as_float32_array;
      double *as_float64_array;
      char **as_string_array;
      cRosMessage **as_msg_array;
      void *as_array;
    } data;
    int size;
    int is_const;
    int is_array;
    int is_fixed_array;
    int array_size;
    int array_capacity;
    CrosMessageType type;
    char *type_s;
};

struct cRosMessage
{
    cRosMessageField **fields;
    cRosMessageDef* msgDef;
    char* md5sum;
    int n_fields;
};

int getFileDependenciesMsg(char* filename, cRosMessageDef* msg, msgDep* deps);

//  Compute full text of message, including text of embedded
//  types.  The text of the main msg is listed first. Embedded
//  msg files are denoted first by an 80-character '=' separator,
//  followed by a type declaration line,'MSG: pkg/type', followed by
//  the text of the embedded type.
char* computeFullTextMsg(cRosMessageDef* msg, msgDep* deps);

int getDependenciesMsg(cRosMessageDef* msg, msgDep* msgDeps);

void cRosMD5Readable(unsigned char* data, DynString* output);

void getMD5Txt(cRosMessageDef* msg, DynString* buffer);

void initCrosMsg(cRosMessageDef* msg);

void initMsgConst(msgConst *msg);

void initCrosDep(msgDep* dep);

void initFieldDef(msgFieldDef* field);

int loadFromStringMsg(char* text, cRosMessageDef* msg);

int loadFromFileMsg(char* filename, cRosMessageDef* msg);

cRosMessage * cRosMessageNew();

void cRosMessageInit(cRosMessage *message);

void cRosMessageBuild(cRosMessage* message, const char* message_path);

void cRosMessageBuildFromDef(cRosMessage* message, cRosMessageDef* msg_def );

void cRosMessageFree(cRosMessage *message);

void cRosMessageRelease(cRosMessage *message);

cRosMessageField * cRosMessageFieldNew();

void cRosMessageFieldInit(cRosMessageField *field);

void cRosMessageFieldRelease(cRosMessageField *field);

void cRosMessageFieldFree(cRosMessageField *field);

cRosMessageField* cRosMessageGetField(cRosMessage *message, char *field);

int cRosMessageSetFieldValueString(cRosMessageField* field, const char* value);

int cRosMessageFieldArrayPushBackInt8(cRosMessageField *field, int8_t val);

int cRosMessageFieldArrayPushBackInt16(cRosMessageField *field, int16_t val);

int cRosMessageFieldArrayPushBackInt32(cRosMessageField *field, int32_t val);

int cRosMessageFieldArrayPushBackInt64(cRosMessageField *field, int64_t val);

int cRosMessageFieldArrayPushBackUInt8(cRosMessageField *field, uint8_t val);

int cRosMessageFieldArrayPushBackUInt16(cRosMessageField *field, uint16_t val);

int cRosMessageFieldArrayPushBackUInt32(cRosMessageField *field, uint32_t val);

int cRosMessageFieldArrayPushBackUInt64(cRosMessageField *field, uint64_t val);

int cRosMessageFieldArrayPushBackFloat32(cRosMessageField *field, float val);

int cRosMessageFieldArrayPushBackFloat64(cRosMessageField *field, double val);

int cRosMessageFieldArrayPushBackString(cRosMessageField *field, const char* val);

int cRosMessageFieldArrayPushBackMsg(cRosMessageField *field, cRosMessage* msg);

int8_t * cRosMessageFieldArrayAtInt8(cRosMessageField *field, int position);

int16_t * cRosMessageFieldArrayAtInt16(cRosMessageField *field, int position);

int32_t * cRosMessageFieldArrayAtInt32(cRosMessageField *field, int position);

int64_t * cRosMessageFieldArrayAtInt64(cRosMessageField *field, int position);

uint8_t * cRosMessageFieldArrayAtUInt8(cRosMessageField *field, int position);

uint16_t * cRosMessageFieldArrayAtUInt16(cRosMessageField *field, int position);

uint32_t * cRosMessageFieldArrayAtUInt32(cRosMessageField *field, int position);

uint64_t * cRosMessageFieldArrayAtUInt64(cRosMessageField *field, int position);

float * cRosMessageFieldArrayAtFloat32(cRosMessageField *field, int position);

double * cRosMessageFieldArrayAtFloat64(cRosMessageField *field, int position);

int cRosMessageFieldArrayAtStringGet(cRosMessageField *field, int position, const char** ptr);

int cRosMessageFieldArrayAtStringSet(cRosMessageField *field, int position, const char* val);

int cRosMessageFieldArrayAtMsgGet(cRosMessageField *field, int position, cRosMessage** ptr);

int cRosMessageFieldArrayAtMsgSet(cRosMessageField *field, int position, cRosMessage* val);

int cRosMessageFieldArrayClear(cRosMessageField *field);

size_t cRosMessageSize(cRosMessage *message);

void cRosMessageSerialize(cRosMessage *message, DynBuffer* buffer);

void cRosMessageDeserialize(cRosMessage *message, DynBuffer *buffer);

CrosMessageType getMessageType(const char* type);

const char * getMessageTypeString(CrosMessageType type);

const char * getMessageTypeDeclaration(CrosMessageType type);

size_t getMessageTypeSizeOf(CrosMessageType type);

int is_builtin_type(CrosMessageType type);

typedef enum
{
  TCPROS_PARSER_ERROR = 0,
  TCPROS_PARSER_HEADER_INCOMPLETE,
  TCPROS_PARSER_DATA_INCOMPLETE,
  TCPROS_PARSER_DONE
} TcprosParserState;

/*! \brief Parse a TCPROS header sent initially from a subscriber
 * 
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the TcprosProcess ( tcpros_server_proc[server_idx] ) to be considered for the parsing
 * 
 *  \return Returns TCPROS_PARSER_DONE if the header is successfully parsed,
 *          TCPROS_PARSER_HEADER_INCOMPLETE if the header is incomplete,  
 *          or TCPROS_PARSER_ERROR on failure
 */
TcprosParserState cRosMessageParseSubcriptionHeader( CrosNode *n, int server_idx );

/*! \brief Parse a TCPROS header sent from a publisher
 *
 *  \param n Ponter to the CrosNode object
 *  \param client_idx Index of the TcprosProcess ( tcpros_client_proc[server_idx] ) to be considered for the parsing
 *
 *  \return Returns TCPROS_PARSER_DONE if the header is successfully parsed,
 *          TCPROS_PARSER_HEADER_INCOMPLETE if the header is incomplete,
 *          or TCPROS_PARSER_ERROR on failure
 */
TcprosParserState cRosMessageParsePublicationHeader( CrosNode *n, int client_idx );

/*! \brief Prepare a TCPROS header to be initially sent to a publisher
 *
 *  \param n Ponter to the CrosNode object
 *  \param client_idx Index of the TcprosProcess ( tcpros_client_proc[server_idx] ) to be considered
 */
void cRosMessagePrepareSubcriptionHeader( CrosNode *n, int client_idx );

/*! \brief Prepare a TCPROS header to be initially sent back to a subscriber
 * 
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the TcprosProcess ( tcpros_server_proc[server_idx] ) to be considered
 */
void cRosMessagePreparePublicationHeader( CrosNode *n, int server_idx );

/*! \brief Prepare a TCPROS message (with data) to be sent to a subscriber
 * 
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the TcprosProcess ( tcpros_server_proc[server_idx] ) to be considered
 */
void cRosMessagePreparePublicationPacket( CrosNode *n, int server_idx );

/*! \brief Read the TCPROS message (with data) received from the publisher
 *
 *  \param n Ponter to the CrosNode object
 *  \param client_idx Index of the TcprosProcess ( tcpros_client_proc[server_idx] ) to be considered for the parsing
 */
void cRosMessageParsePublicationPacket( CrosNode *n, int client_idx );

/*! \brief Parse a RCPROS header sent from a service caller
 *
 *  \param n Ponter to the CrosNode object
 *  \param client_idx Index of the TcprosProcess ( rpcros_server_proc[server_idx] ) to be considered for the parsing
 *
 *  \return Returns TCPROS_PARSER_DONE if the header is successfully parsed,
 *          TCPROS_PARSER_HEADER_INCOMPLETE if the header is incomplete,
 *          or TCPROS_PARSER_ERROR on failure
 */
TcprosParserState cRosMessageParseServiceCallerHeader( CrosNode *n, int server_idx);

/*! \brief Prepare a RCPROS header to be initially sent back to a service caller
 *
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the TcprosProcess ( rpcros_server_proc[server_idx] ) to be considered
 */
void cRosMessagePrepareServiceProviderHeader( CrosNode *n, int server_idx);

/*! \brief Prepare a RCPROS response to be sent back to a service caller
 *
 *  \param n Ponter to the CrosNode object
 *  \param server_idx Index of the TcprosProcess ( rpcros_server_proc[server_idx] ) to be considered
 */
void cRosMessagePrepareServiceResponsePacket( CrosNode *n, int server_idx);

/*! @}*/

#endif

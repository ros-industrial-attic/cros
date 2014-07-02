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

typedef enum
{
  TCPROS_PARSER_ERROR = 0,
  TCPROS_PARSER_HEADER_INCOMPLETE,
  TCPROS_PARSER_DATA_INCOMPLETE,
  TCPROS_PARSER_DONE
}TcprosParserState;

struct t_msgFieldDef{
    char* type;
    char* name;
    int is_array;
    int array_size;
    struct t_msgFieldDef* prev;
    struct t_msgFieldDef* next;
};

typedef struct t_msgFieldDef msgFieldDef;

struct t_msgConst{
    char* type;
    char* name;
    char* value;
    struct t_msgConst* prev;
    struct t_msgConst* next;
};

typedef struct t_msgConst msgConst;

struct t_msgDef{
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

struct t_msgDep{
    cRosMessageDef* msg;
    struct t_msgDep* prev;
    struct t_msgDep* next;
};

typedef struct t_msgDep msgDep;

static const char* PRIMITIVE_TYPES[] = {
        "int8","uint8","int16","uint16","int32","uint32","int64","uint64","float32","float64",
    "string", "bool",
    //deprecated:
    "char","byte",
        //time and duration
    "time","duration"};

struct cRosMessageField
{
    char *name;
    unsigned char as_bool;
    int as_int;
    int64_t as_int64;
    float as_float;
    double as_double;
    char* as_string;
    void *data;
    size_t size;
    int is_builtin;
    int array_max_size;
    int array_size;
    int array_capacity;
    void* vector_data;
    unsigned char is_const;
    unsigned char is_array;
    char *type;
};

typedef struct cRosMessageField cRosMessageField;

struct cRosMessage
{
    cRosMessageField **fields;
    cRosMessageDef* msgDef;
    char* md5sum;
    size_t n_fields;
};

typedef struct cRosMessage cRosMessage;

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

void initCrosDep(msgDep* dep);

void initFieldDef(msgFieldDef* field);

int loadFromStringMsg(char* text, cRosMessageDef* msg);

int loadFromFileMsg(char* filename, cRosMessageDef* msg);

void cRosMessageInit(cRosMessage *message);

void cRosMessageBuild(cRosMessage* message, const char* message_path);

void cRosMessageFree(cRosMessage *message);

void cRosMessageFieldInit(cRosMessageField *field);

cRosMessageField* cRosMessageGetField(cRosMessage *message, char *field);

int cRosMessageSetFieldValueBool(cRosMessageField* field, unsigned char value);

int cRosMessageSetFieldValueInt(cRosMessageField* field, int value);

int cRosMessageSetFieldValueDouble(cRosMessageField* field, double value);

int cRosMessageSetFieldValueString(cRosMessageField* field, const char* value);

int cRosMessageSetFieldValueMsg(cRosMessageField* field, cRosMessage* value);

int cRosMessageFieldArrayPushBackInt8(cRosMessageField *field, int8_t val);

int cRosMessageFieldArrayPushBackInt16(cRosMessageField *field, int16_t val);

int cRosMessageFieldArrayPushBackInt32(cRosMessageField *field, int32_t val);

int cRosMessageFieldArrayPushBackInt64(cRosMessageField *field, int64_t val);

int cRosMessageFieldArrayPushBackUint8(cRosMessageField *field, uint8_t val);

int cRosMessageFieldArrayPushBackUint16(cRosMessageField *field, uint16_t val);

int cRosMessageFieldArrayPushBackUint32(cRosMessageField *field, uint32_t val);

int cRosMessageFieldArrayPushBackUint64(cRosMessageField *field, uint64_t val);

int cRosMessageFieldArrayPushBackSingle(cRosMessageField *field, float val);

int cRosMessageFieldArrayPushBackDouble(cRosMessageField *field, double val);

int cRosMessageFieldArrayAtInt8(cRosMessageField *field, int position, int8_t* val);

int cRosMessageFieldArrayAtInt16(cRosMessageField *field, int position, int16_t* val);

int cRosMessageFieldArrayAtInt32(cRosMessageField *field, int position, int32_t* val);

int cRosMessageFieldArrayAtInt64(cRosMessageField *field, int position, int64_t* val);

int cRosMessageFieldArrayAtUint8(cRosMessageField *field, int position, uint8_t* val);

int cRosMessageFieldArrayAtUint16(cRosMessageField *field, int position, uint16_t* val);

int cRosMessageFieldArrayAtUint32(cRosMessageField *field, int position, uint32_t* val);

int cRosMessageFieldArrayAtUint64(cRosMessageField *field, int position, uint64_t* val);

int cRosMessageFieldArrayAtSingle(cRosMessageField *field, int position, float* val);

int cRosMessageFieldArrayAtDouble(cRosMessageField *field, int position, double* val);

size_t cRosMessageSize(cRosMessage *message);

void cRosMessageSerialize(cRosMessage *message, DynBuffer* buffer);

void cRosMessageDeserialize(cRosMessage *message, DynBuffer *buffer);

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

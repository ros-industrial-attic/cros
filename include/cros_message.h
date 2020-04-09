#ifndef _CROS_MESSAGE_H_
#define _CROS_MESSAGE_H_

#include "dyn_buffer.h"
#include "cros_err_codes.h"

/*! \defgroup cros_message cROS TCPROS
 *
 *  Implemenation of the TCPROS protocol for topic message exchanges
 */

/*! \addtogroup cros_message
 *  @{
 */

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

typedef struct cRosMessageField cRosMessageField;
typedef struct cRosMessage cRosMessage;

struct cRosMessageField
{
    char *name;

    union data_t
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

typedef struct t_msgDef cRosMessageDef;

struct cRosMessage
{
    cRosMessageField **fields;
    cRosMessageDef *msgDef;
    char *md5sum;
    int n_fields;
};

cRosMessage * cRosMessageNew(void);

void cRosMessageInit(cRosMessage *message);

cRosErrCodePack cRosMessageDefBuild(cRosMessageDef **msg_def_ptr, const char *msg_root_dir, const char *msg_type);

cRosErrCodePack cRosMessageNewBuild(const char *msg_root_dir, const char *msg_type, cRosMessage **new_msg_ptr);

void cRosMessageFieldsPrint(cRosMessage *msg, int n_indent);

int cRosMessageFieldCopy(cRosMessageField *new_field, cRosMessageField *orig_field);

int cRosMessageFieldsCopy(cRosMessage *m_dst, cRosMessage *m_src);

cRosMessage *cRosMessageCopyWithoutDef(cRosMessage *m_src);

cRosMessage *cRosMessageCopy(cRosMessage *m_src);

cRosErrCodePack cRosMessageBuildFromDef(cRosMessage **message, cRosMessageDef *msg_def );

void cRosMessageFree(cRosMessage *message);

void cRosMessageFieldsFree(cRosMessage *message);

void cRosMessageRelease(cRosMessage *message);

cRosMessageField * cRosMessageFieldNew(void);

void cRosMessageFieldInit(cRosMessageField *field);

void cRosMessageFieldRelease(cRosMessageField *field);

void cRosMessageFieldFree(cRosMessageField *field);

cRosMessageField * cRosMessageGetField(cRosMessage *message, const char *field);

int cRosMessageSetFieldValueString(cRosMessageField *field, const char *value);

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

int cRosMessageFieldArrayPushBackString(cRosMessageField *field, const char *val);

int cRosMessageFieldArrayPushBackZero(cRosMessage *msg, int n_field);

int cRosMessageFieldArrayPushBackMsg(cRosMessageField *field, cRosMessage *msg);

cRosMessage *cRosMessageFieldArrayRemoveLastMsg(cRosMessageField *field);

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

char *cRosMessageFieldArrayAtStringGet(cRosMessageField *field, int position);

int cRosMessageFieldArrayAtStringSet(cRosMessageField *field, int position, const char *val);

cRosMessage *cRosMessageFieldArrayAtMsgGet(cRosMessageField *field, int position);

int cRosMessageFieldArrayAtMsgSet(cRosMessageField *field, int position, cRosMessage *val);

int cRosMessageFieldArrayClear(cRosMessageField *field);

size_t cRosMessageSize(cRosMessage *message);

cRosErrCodePack cRosMessageSerialize(cRosMessage *message, DynBuffer *buffer);

cRosErrCodePack cRosMessageDeserialize(cRosMessage *message, DynBuffer *buffer);

CrosMessageType getMessageType(const char *type);

const char * getMessageTypeString(CrosMessageType type);

const char * getMessageTypeDeclaration(CrosMessageType type);

size_t getMessageTypeSizeOf(CrosMessageType type);

int isBuiltinMessageType(CrosMessageType type);

/*! @}*/

#endif

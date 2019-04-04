#ifndef _XMLRPC_TAGS_H_
#define _XMLRPC_TAGS_H_

/*! \defgroup xmlrpc_tags XMLRPC tags */

/*! \addtogroup xmlrpc_tags
 *  @{
 */

/*! \brief Structure that contains a XMLRPC tag string and the related string length
 */
typedef struct
{
  char *str;
  int dim;
}XmlrpcTagStrDim;

static XmlrpcTagStrDim XMLRPC_VERSION = { "Custom XMLRPC", 13 };
static XmlrpcTagStrDim XMLRPC_MESSAGE_BEGIN = { "<?xml version=\"1.0\"?>", 21 };
static XmlrpcTagStrDim XMLRPC_MESSAGE_END = { "", 0 };
static XmlrpcTagStrDim XMLRPC_REQUEST_BEGIN = { "<methodCall>", 12 };
static XmlrpcTagStrDim XMLRPC_REQUEST_END = { "</methodCall>", 13 };
static XmlrpcTagStrDim XMLRPC_METHODNAME_BEGIN = { "<methodName>", 12 };
static XmlrpcTagStrDim XMLRPC_METHODNAME_END = { "</methodName>", 13 };
static XmlrpcTagStrDim XMLRPC_RESPONSE_BEGIN = { "<methodResponse>", 16 };
static XmlrpcTagStrDim XMLRPC_RESPONSE_END = { "</methodResponse>", 17 };

static XmlrpcTagStrDim XMLRPC_PARAMS_TAG = { "<params>", 8 };
static XmlrpcTagStrDim XMLRPC_PARAMS_ETAG = { "</params>", 9 };
static XmlrpcTagStrDim XMLRPC_PARAM_TAG = { "<param>", 7 };
static XmlrpcTagStrDim XMLRPC_PARAM_ETAG = { "</param>", 8 };
static XmlrpcTagStrDim XMLRPC_FAULT_TAG = { "<fault>", 7 };
static XmlrpcTagStrDim XMLRPC_FAULT_ETAG = { "</fault>", 8 };

static XmlrpcTagStrDim XMLRPC_VALUE_TAG = { "<value>", 7 };
static XmlrpcTagStrDim XMLRPC_VALUE_ETAG = { "</value>", 8 };

static XmlrpcTagStrDim XMLRPC_BOOLEAN_TAG = { "<boolean>", 9 };
static XmlrpcTagStrDim XMLRPC_BOOLEAN_ETAG = { "</boolean>", 10 };
static XmlrpcTagStrDim XMLRPC_DOUBLE_TAG = { "<double>", 8 };
static XmlrpcTagStrDim XMLRPC_DOUBLE_ETAG = { "</double>", 9 };
static XmlrpcTagStrDim XMLRPC_I4_TAG = { "<i4>", 4 };
static XmlrpcTagStrDim XMLRPC_I4_ETAG = { "</i4>", 5 };
static XmlrpcTagStrDim XMLRPC_INT_TAG = { "<int>", 5 };
static XmlrpcTagStrDim XMLRPC_INT_ETAG = { "</int>", 6 };
static XmlrpcTagStrDim XMLRPC_STRING_TAG = { "<string>", 8 };
static XmlrpcTagStrDim XMLRPC_STRING_ETAG = { "</string>", 9 };
static XmlrpcTagStrDim XMLRPC_DATETIME_TAG = { "<dateTime.iso8601>", 18 };
static XmlrpcTagStrDim XMLRPC_DATETIME_ETAG = { "</dateTime.iso8601>", 19 };
static XmlrpcTagStrDim XMLRPC_BASE64_TAG = { "<base64>", 8 };
static XmlrpcTagStrDim XMLRPC_BASE64_ETAG = { "</base64>", 9 };
static XmlrpcTagStrDim XMLRPC_ARRAY_TAG = { "<array>", 7 };
static XmlrpcTagStrDim XMLRPC_ARRAY_ETAG = { "</array>", 8 };
static XmlrpcTagStrDim XMLRPC_DATA_TAG = { "<data>", 6 }; // start-tag
static XmlrpcTagStrDim XMLRPC_DATA_ETAG = { "</data>", 7 }; // end-tag
static XmlrpcTagStrDim XMLRPC_DATA_NTAG = { "<data/>", 7 }; // empty-element tag (no content)
static XmlrpcTagStrDim XMLRPC_STRUCT_TAG = { "<struct>", 8 };
static XmlrpcTagStrDim XMLRPC_STRUCT_ETAG = { "</struct>", 9 };
static XmlrpcTagStrDim XMLRPC_STRUCT_NTAG = { "<struct/>", 9 };
static XmlrpcTagStrDim XMLRPC_MEMBER_TAG = { "<member>", 8 };
static XmlrpcTagStrDim XMLRPC_MEMBER_ETAG = { "</member>", 9 };
static XmlrpcTagStrDim XMLRPC_NAME_TAG = { "<name>", 6 };
static XmlrpcTagStrDim XMLRPC_NAME_ETAG = { "</name>", 7 };

/*! @}*/

#endif

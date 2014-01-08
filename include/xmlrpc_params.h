#ifndef _XMLRPC_PARAMS_H_
#define _XMLRPC_PARAMS_H_

#include "dyn_string.h"

/*! \defgroup xmlrpc_param XMLRPC parameters */

/*! \addtogroup xmlrpc_param 
 *  @{
 */

typedef enum
{ 
  XMLRPC_PARAM_BOOL,
  XMLRPC_PARAM_INT,
  XMLRPC_PARAM_DOUBLE,
  XMLRPC_PARAM_STRING,
  XMLRPC_PARAM_ARRAY,
  XMLRPC_PARAM_DATETIME, /* WARNING: Currently unsupported */
  XMLRPC_PARAM_BINARY, /* WARNING: Currently unsupported */
  XMLRPC_PARAM_STRUCT, /* WARNING: Currently unsupported */

  XMLRPC_PARAM_UNKNOWN
}XmlrpcParamType;
 
/*! \brief Struct used to store a input/oputput XMLRPC param.
 *         To modify its internal members, you could use the related functions
 */
typedef struct XmlrpcParam XmlrpcParam;
struct XmlrpcParam
{
  XmlrpcParamType type; //! Param type 
  union
  {
    unsigned char as_bool;
    int as_int;
    double as_double;
    char* as_string;
    XmlrpcParam *as_array;
    void* as_time; /* WARNING: Currently unsupported */
    void* as_binary; /* WARNING: Currently unsupported */
    void* as_struct; /* WARNING: Currently unsupported */
  } data; //! Param data
  int array_n_elem; //! Used only if type is XMLRPC_PARAM_ARRAY: it stores the array size
  int array_max_elem; //! Used only if type is XMLRPC_PARAM_ARRAY: it stores the current max size
};

/*! \brief Return an XMLRPC parameter as a boolena value (i.e., an
 *         unisigned char value either 0 : false or 1 : true ).
 *         No type control are performed. If the XMLRPC parameter 
 *         is not boolean, return value is undefined
 * 
 *  \param param Pointer to a XMLRPC parameter
 * 
 *  \return An unisigned char value ( 0 : false, 1 : true )
 */
unsigned char xmlrpcParamGetBool( XmlrpcParam *param );

/*! \brief Return an XMLRPC parameter as a integer value.
 *         No type control are performed. If the XMLRPC parameter 
 *         is not integer, return value is undefined
 * 
 *  \param param Pointer to a XMLRPC parameter
 * 
 *  \return The integer value
 */
int xmlrpcParamGetInt( XmlrpcParam *param );

/*! \brief Return an XMLRPC parameter as a double value.
 *         No type control are performed. If the XMLRPC parameter 
 *         is not double, return value is undefined
 * 
 *  \param param Pointer to a XMLRPC parameter
 * 
 *  \return The double value
 */
double xmlrpcParamGetDouble( XmlrpcParam *param );

/*! \brief Return an XMLRPC parameter as a string value.
 *         No type control are performed. If the XMLRPC parameter 
 *         is not a string, return value is undefined
 * 
 *  \param param Pointer to a XMLRPC parameter
 * 
 *  \return Pointer to the string, passed as a pointer to the internal memory
 */
char *xmlrpcParamGetString( XmlrpcParam *param );

/*! \brief Setup a XMLRPC unknown parameter (e.g., in case of errors)
 * 
 *  \param param Pointer to a XMLRPC parameter
 */
void xmlrpcParamSetUnknown( XmlrpcParam *param );

/*! \brief Setup a XMLRPC boolean parameter with the given value
 * 
 *  \param param Pointer to a XMLRPC parameter
 *  \param val An integer value (0 : false, true otherwise)
 */
void xmlrpcParamSetBool( XmlrpcParam *param, int val );

/*! \brief Setup a XMLRPC integer parameter with the given value
 * 
 *  \param param Pointer to a XMLRPC parameter
 *  \param val An integer value
 */
void xmlrpcParamSetInt( XmlrpcParam *param, int val );

/*! \brief Setup a XMLRPC double floating point parameter with the given value
 * 
 *  \param param Pointer to a XMLRPC parameter
 *  \param val An double value
 */
void xmlrpcParamSetDouble( XmlrpcParam *param, double val );

/*! \brief Setup a XMLRPC string parameter with the given string. 
 *         The string is copied inside a dynamically allocated memory
 * 
 *  \param param Pointer to a XMLRPC parameter
 *  \param val Pointer to a string
 */
void xmlrpcParamSetString( XmlrpcParam *param, const char *val );

/*! \brief As xmlrpcParamSetString(), but limits the string length to n characters
 * 
 *  \param param Pointer to a XMLRPC parameter
 *  \param val Pointer to a string
 *  \param n The string length
 */
void xmlrpcParamSetStringN( XmlrpcParam *param, const char *val, int n );

/*! \brief Setup an empty array XMLRPC parameter, starting to allocate internal memory  
 * 
 *  \param param Pointer to a XMLRPC parameter
 */
void xmlrpcParamSetArray( XmlrpcParam *param );

/*! \brief Return the data type for a given XMLRPC parameter
 * 
 *  \param param Pointer to a XMLRPC parameter
 * 
 *  \return The data type
 */
XmlrpcParamType xmlrpcParamGetType( XmlrpcParam *param );

/*! \brief Return the size of a XMLRPC array parameter
 * 
 *  \param param Pointer to a XMLRPC parameter
 * 
 *  \return The array size, or -1 if the XMLRPC parameter is not an array
 */
int xmlrpcParamArrayGetSize( XmlrpcParam *param );

/*! \brief Return the idx-th element of a XMLRPC array parameter
 * 
 *  \param param Pointer to a XMLRPC parameter
 *  \param idx The element index 
 * 
 *  \return Pointer to the XMLRPC parameter, or NULL 
 *          if the XMLRPC parameter is not an array or if idx exceeds the array size
 */
XmlrpcParam *xmlrpcParamArrayGetParamAt( XmlrpcParam *param, int idx );

/*! \brief Append to an array XMLRPC parameter a bool parameter
 * 
 *  \param param Pointer to an array XMLRPC parameter
 *  \param val An integer value (false if it is equal to 0, true otherwise)
 */
void xmlrpcParamArrayPushBackBool( XmlrpcParam *param, int val );

/*! \brief Append to an array XMLRPC parameter an integer parameter
 * 
 *  \param param Pointer to an array XMLRPC parameter
 *  \param val An integer value
 */
void xmlrpcParamArrayPushBackInt( XmlrpcParam *param, int val );

/*! \brief Append to an array XMLRPC parameter a double parameter
 * 
 *  \param param Pointer to an array XMLRPC parameter
 *  \param val An double value
 */
void xmlrpcParamArrayPushBackDouble( XmlrpcParam *param, double val );

/*! \brief Append to an array XMLRPC parameter a string parameter
 * 
 *  \param param Pointer to an array XMLRPC parameter
 *  \param val Pointer to a string
 */
void xmlrpcParamArrayPushBackString( XmlrpcParam *param, const char *val );

/*! \brief As xmlrpcParamArrayPushBackString(), but limits the string length to n characters
 * 
 *  \param param Pointer to an array XMLRPC parameter
 *  \param val Pointer to a string
 *  \param n The string length
 */
void xmlrpcParamArrayPushBackStringN( XmlrpcParam *param, const char *val, int n );

/*! \brief Append to an array XMLRPC parameter an empty array parameter
 * 
 *  \param param Pointer to an array XMLRPC parameter
 * 
 *  \return A pointer to the new pushed array XMLRPC parameter, or NULL on failure
 */
XmlrpcParam *xmlrpcParamArrayPushBackArray( XmlrpcParam *param );

/*! \brief Release internal data dynamically allocated (e.g., string and arrays)
 * 
 *  \param param Pointer to a XMLRPC parameter
 */
void xmlrpcParamReleaseData( XmlrpcParam *param );

/*! \brief Append to a dynamic string the parameters given in input, converted in XML
 * 
 *  \param param Pointer to the param to be converted in XML
 *  \param message Pointer to the dynamic string where the parameter will be appended
 */
void xmlrpcParamToXml( XmlrpcParam *param, DynString *message );

/*! \brief Look for a XMLRPC parameter inside a dynamic string, and store in a XmlrpcParam object
 * 
 *  \param message Pointer to the dynamic string to be parsed
 *  \param param Pointer to the output parameter
 * 
 *  \return Returns 1 on success, 0 on failure
 */
int xmlrpcParamFromXml( DynString *message, XmlrpcParam *param );

/*! \brief Print XMLRPC parameter to stdout in human readable form
 * 
 *  \param param Pointer to the output parameter
 */
void xmlrpcParamPrint( XmlrpcParam *param );

/*! @}*/

#endif
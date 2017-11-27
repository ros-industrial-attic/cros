#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <locale.h>
#include <assert.h>

#include "xmlrpc_params.h"
#include "xmlrpc_tags.h"
#include "cros_defs.h"
#include "cros_log.h"

enum { XMLRPC_ARRAY_INIT_SIZE = 4, XMLRPC_ARRAY_GROW_RATE = 2 };

typedef enum ParamContainerType
{
  PARAM_CONTAINER_NONE = 0,
  PARAM_CONTAINER_ARRAY = 1,
  PARAM_CONTAINER_STRUCT = 2
} ParamContainerType;

static int paramFromXml ( DynString *message, XmlrpcParam *param, ParamContainerType container);
static int paramValueFromXml ( DynString *message, XmlrpcParam *param,  ParamContainerType container);
static int structMemberFromXml ( DynString *message, XmlrpcParam *param);
static int arrayFromXml ( DynString *message, XmlrpcParam *param);
static int structFromXml ( DynString *message, XmlrpcParam *param);
static XmlrpcParam * arrayAddElem ( XmlrpcParam *param );
static int paramSetMemberName ( XmlrpcParam *param, const char *name );

static void boolToXml ( unsigned char val, DynString *message )
{
  dynStringPushBackStr ( message, XMLRPC_VALUE_TAG.str );
  dynStringPushBackStr ( message, XMLRPC_BOOLEAN_TAG.str );
  dynStringPushBackStr ( message, val != 0?"1":"0" );
  dynStringPushBackStr ( message, XMLRPC_BOOLEAN_ETAG.str );
  dynStringPushBackStr ( message, XMLRPC_VALUE_ETAG.str );
}

static void intToXml ( int val, DynString *message )
{
  dynStringPushBackStr ( message, XMLRPC_VALUE_TAG.str );
  dynStringPushBackStr ( message, XMLRPC_INT_TAG.str );
  char num_str[15];
  snprintf ( num_str, 15, "%.13d", val );
  dynStringPushBackStr ( message, num_str );
  dynStringPushBackStr ( message, XMLRPC_INT_ETAG.str );
  dynStringPushBackStr ( message, XMLRPC_VALUE_ETAG.str );
}

static void doubleToXml ( double val, DynString *message )
{
  setlocale ( LC_NUMERIC, "" );
  dynStringPushBackStr ( message, XMLRPC_VALUE_TAG.str );
  dynStringPushBackStr ( message, XMLRPC_DOUBLE_TAG.str );
  char num_str[256];
  snprintf ( num_str, 256, "%.17f", val );
  dynStringPushBackStr ( message, num_str );
  dynStringPushBackStr ( message, XMLRPC_DOUBLE_ETAG.str );
  dynStringPushBackStr ( message, XMLRPC_VALUE_ETAG.str );
}

static void stringToXml ( char *val, DynString *message )
{
  dynStringPushBackStr ( message, XMLRPC_VALUE_TAG.str );
  dynStringPushBackStr ( message, XMLRPC_STRING_TAG.str );

  int str_len = strlen ( val );
  int i = 0;
  for ( i = 0; i < str_len; i++ )
  {
    char c = val[i];
    if ( c == '<' )
      dynStringPushBackStr ( message, "&lt;" );
    else if ( c == '>' )
      dynStringPushBackStr ( message, "&gt;" );
    else if ( c == '&' )
      dynStringPushBackStr ( message, "&amp;" );
    else if ( c == '\'' )
      dynStringPushBackStr ( message, "&apos;" );
    else if ( c == '\"' )
      dynStringPushBackStr ( message, "&quot;" );
    else
      dynStringPushBackChar ( message, c );
  }

  dynStringPushBackStr ( message, XMLRPC_STRING_ETAG.str );
  dynStringPushBackStr ( message, XMLRPC_VALUE_ETAG.str );
}

static void structToXml ( XmlrpcParam *val, DynString *message )
{
  dynStringPushBackStr ( message, XMLRPC_VALUE_TAG.str );
  dynStringPushBackStr ( message, XMLRPC_STRUCT_TAG.str );

  int i;
  for ( i = 0; i < val->array_n_elem; i++ )
    xmlrpcParamToXml ( & ( val->data.as_array[i] ), message );

  dynStringPushBackStr ( message, XMLRPC_STRUCT_ETAG.str );
  dynStringPushBackStr ( message, XMLRPC_VALUE_ETAG.str );
}

static void arrayToXml ( XmlrpcParam *val, DynString *message )
{
  dynStringPushBackStr ( message, XMLRPC_VALUE_TAG.str );
  dynStringPushBackStr ( message, XMLRPC_ARRAY_TAG.str );
  dynStringPushBackStr ( message, XMLRPC_DATA_TAG.str );

  int i;
  for ( i = 0; i < val->array_n_elem; i++ )
    xmlrpcParamToXml ( & ( val->data.as_array[i] ), message );

  dynStringPushBackStr ( message, XMLRPC_DATA_ETAG.str );
  dynStringPushBackStr ( message, XMLRPC_ARRAY_ETAG.str );
  dynStringPushBackStr ( message, XMLRPC_VALUE_ETAG.str );
}

static void timeToXml ( void *val, DynString *message )
{
  PRINT_ERROR ( "timeToXml() : ERROR: Not yet implemented!\n" );
}

static void binaryToXml ( void *val, DynString *message )
{
  PRINT_ERROR ( "binaryToXml() : ERROR: Not yet implemented!\n" );
}

int paramFromXml (DynString *message, XmlrpcParam *param,  ParamContainerType container)
{
  //PRINT_VDEBUG ( "paramFromXml(), is_array : %s \n", is_array?"TRUE":"FALSE" );

  int rc;
  const char *c = dynStringGetCurrentData ( message );
  int len = dynStringGetLen ( message );
  int i = dynStringGetPoseIndicatorOffset ( message );
  const char *param_begin =  NULL;
  const char *param_end = NULL;
  for (; i < len; i++, c++ )
  {
    if ((container == PARAM_CONTAINER_NONE || container == PARAM_CONTAINER_ARRAY)
          && len - i >= XMLRPC_VALUE_TAG.dim
          && strncmp ( c, XMLRPC_VALUE_TAG.str, XMLRPC_VALUE_TAG.dim ) == 0 )
    {
      c += XMLRPC_VALUE_TAG.dim;
      i += XMLRPC_VALUE_TAG.dim;
      param_begin = c;
      break;
    }
    else if (container == PARAM_CONTAINER_STRUCT && len - i >= XMLRPC_MEMBER_TAG.dim
             && strncmp ( c, XMLRPC_MEMBER_TAG.str, XMLRPC_MEMBER_TAG.dim ) == 0 )
    {
      c += XMLRPC_MEMBER_TAG.dim;
      i += XMLRPC_MEMBER_TAG.dim;
      param_begin = c;
      break;
    }
    else if (container == PARAM_CONTAINER_ARRAY && len - i >= XMLRPC_DATA_ETAG.dim
             && strncmp ( c, XMLRPC_DATA_ETAG.str, XMLRPC_DATA_ETAG.dim ) == 0 )
    {
      PRINT_DEBUG ( "paramFromXml() : reach end of array\n" );
      return -1;
    }
    else if (container == PARAM_CONTAINER_STRUCT && len - i >= XMLRPC_STRUCT_ETAG.dim
             && strncmp ( c, XMLRPC_STRUCT_ETAG.str, XMLRPC_STRUCT_ETAG.dim ) == 0 )
    {
      PRINT_DEBUG ( "paramFromXml() : reach end of struct\n" );
      return -1;
    }
  }

  if (param_begin == NULL)
  {
    PRINT_ERROR ( "paramFromXml() : no param tag found\n" );
    return -1;
  }

  dynStringSetPoseIndicator ( message, i);

  switch (container)
  {
    case PARAM_CONTAINER_NONE:
    case PARAM_CONTAINER_ARRAY:
    {
      rc = paramValueFromXml(message, param, container);
      break;
    }
    case PARAM_CONTAINER_STRUCT:
    {
      param = arrayAddElem (param);
      if (param == NULL)
        return -1;

      rc = structMemberFromXml(message, param);
      break;
    }
    default:
    {
      assert(0);
    }
  }

  if (rc < 0)
    return rc;

  c = dynStringGetCurrentData ( message );
  i = dynStringGetPoseIndicatorOffset ( message );

  for (; i < len; i++, c++ )
  {
    switch (container)
    {
      case PARAM_CONTAINER_NONE:
      case PARAM_CONTAINER_ARRAY:
      {
        if ( len - i >= XMLRPC_VALUE_ETAG.dim &&
              strncmp ( c, XMLRPC_VALUE_ETAG.str, XMLRPC_VALUE_ETAG.dim ) == 0 )
        {
          param_end = c;
          c += XMLRPC_VALUE_ETAG.dim;
          i += XMLRPC_VALUE_ETAG.dim;
          goto paramFromXml_exit;
        }
        break;
      }
      case PARAM_CONTAINER_STRUCT:
      {
        if ( len - i >= XMLRPC_MEMBER_ETAG.dim &&
            strncmp ( c, XMLRPC_MEMBER_ETAG.str, XMLRPC_MEMBER_ETAG.dim ) == 0 )
        {
          param_end = c;
          c += XMLRPC_MEMBER_ETAG.dim;
          i += XMLRPC_MEMBER_ETAG.dim;
          goto paramFromXml_exit;
        }
        break;
      }
      default:
      {
        assert(0);
      }
    }
  }

paramFromXml_exit:
  if ( param_end == NULL )
  {
    PRINT_ERROR ( "arrayFromXml() : no param end tag found\n" );
    return -1;
  }

  dynStringSetPoseIndicator ( message, i);

  return 0;
}

int arrayFromXml(DynString *message, XmlrpcParam *param)
{
  PRINT_VDEBUG ( "arrayFromXml()\n" );

  const char *c = dynStringGetCurrentData ( message );
  int len = dynStringGetLen ( message );
  int i = dynStringGetPoseIndicatorOffset ( message );
  const char *data_init = NULL;
  const char *data_end = NULL;

  for ( ; i < len; i++, c++ )
  {
    if ( len - i >= XMLRPC_DATA_TAG.dim &&
         strncmp ( c, XMLRPC_DATA_TAG.str, XMLRPC_DATA_TAG.dim ) == 0 )
    {
      c += XMLRPC_DATA_TAG.dim;
      i += XMLRPC_DATA_TAG.dim;
      data_init = c;
      break;
    }
  }

  if ( data_init == NULL )
  {
    PRINT_ERROR ( "arrayFromXml() : no data tag found\n" );
    return -1;
  }

  dynStringSetPoseIndicator ( message, i);

  while (paramFromXml (message, param, PARAM_CONTAINER_ARRAY ) != -1);

  c = dynStringGetCurrentData ( message );
  i = dynStringGetPoseIndicatorOffset ( message );

  for ( ; i < len; i++, c++ )
  {
    if ( len - i >= XMLRPC_DATA_ETAG.dim &&
         strncmp ( c, XMLRPC_DATA_ETAG.str, XMLRPC_DATA_ETAG.dim ) == 0 )
    {
      data_end = c;
      c += XMLRPC_DATA_ETAG.dim;
      i += XMLRPC_DATA_ETAG.dim;
      break;
    }
  }

  if ( data_end == NULL )
  {
    PRINT_ERROR ( "arrayFromXml() : no end data tag found\n" );
    return -1;
  }

  dynStringSetPoseIndicator ( message, i );

  return 0;
}

int structFromXml(DynString *message, XmlrpcParam *param)
{
  PRINT_VDEBUG ( "structFromXml()\n" );

  while (paramFromXml (message, param, PARAM_CONTAINER_STRUCT ) != -1);

  return 0;
}

int paramValueFromXml (DynString *message, XmlrpcParam *param,  ParamContainerType container)
{
  int ret = 0; // Default return value=0
  const char *c = dynStringGetCurrentData ( message );
  int len = dynStringGetLen ( message );
  int i = dynStringGetPoseIndicatorOffset ( message );

  XmlrpcParamType p_type = XMLRPC_PARAM_UNKNOWN;
  const char *value_init = NULL;
  const char *type_init = NULL;
  const char *type_end = NULL;

  for ( ; i < len; i++, c++ )
  {
    if ( len - i >= XMLRPC_BOOLEAN_TAG.dim &&
         strncmp ( c, XMLRPC_BOOLEAN_TAG.str, XMLRPC_BOOLEAN_TAG.dim ) == 0 )
    {
      c += XMLRPC_BOOLEAN_TAG.dim;
      i += XMLRPC_BOOLEAN_TAG.dim;
      type_init = c;
      p_type = XMLRPC_PARAM_BOOL;
      break;
    }
    else if ( len - i >= XMLRPC_I4_TAG.dim &&
              strncmp ( c, XMLRPC_I4_TAG.str, XMLRPC_I4_TAG.dim ) == 0 )
    {
      c += XMLRPC_I4_TAG.dim;
      i += XMLRPC_I4_TAG.dim;
      type_init = c;
      p_type = XMLRPC_PARAM_INT;
      break;
    }
    else if ( len - i >= XMLRPC_INT_TAG.dim &&
              strncmp ( c, XMLRPC_INT_TAG.str, XMLRPC_INT_TAG.dim ) == 0 )
    {
      c += XMLRPC_INT_TAG.dim;
      i += XMLRPC_INT_TAG.dim;
      type_init = c;
      p_type = XMLRPC_PARAM_INT;
      break;
    }
    else if ( len - i >= XMLRPC_DOUBLE_TAG.dim &&
              strncmp ( c, XMLRPC_DOUBLE_TAG.str, XMLRPC_DOUBLE_TAG.dim ) == 0 )
    {
      c += XMLRPC_DOUBLE_TAG.dim;
      i += XMLRPC_DOUBLE_TAG.dim;
      type_init = c;
      p_type = XMLRPC_PARAM_DOUBLE;
      break;
    }
    else if ( len - i >= XMLRPC_STRING_TAG.dim &&
              strncmp ( c, XMLRPC_STRING_TAG.str, XMLRPC_STRING_TAG.dim ) == 0 )
    {
      c += XMLRPC_STRING_TAG.dim;
      i += XMLRPC_STRING_TAG.dim;
      type_init = c;
      p_type = XMLRPC_PARAM_STRING;
      break;
    }
    else if ( len - i >= XMLRPC_ARRAY_TAG.dim &&
              strncmp ( c, XMLRPC_ARRAY_TAG.str, XMLRPC_ARRAY_TAG.dim ) == 0 )
    {
      c += XMLRPC_ARRAY_TAG.dim;
      i += XMLRPC_ARRAY_TAG.dim;
      type_init = c;
      p_type = XMLRPC_PARAM_ARRAY;
      break;
    }
    else if ( len - i >= XMLRPC_DATETIME_TAG.dim &&
              strncmp ( c, XMLRPC_DATETIME_TAG.str, XMLRPC_DATETIME_TAG.dim ) == 0 )
    {
      PRINT_ERROR ( "paramFromXml() : ERROR: Tag %s not yet implemented!\n", XMLRPC_DATETIME_TAG.str );
      ret=-1;

      c += XMLRPC_DATETIME_TAG.dim;
      i += XMLRPC_DATETIME_TAG.dim;
      type_init = c;
      p_type = XMLRPC_PARAM_DATETIME;
      break;
    }
    else if ( len - i >= XMLRPC_BASE64_TAG.dim &&
              strncmp ( c, XMLRPC_BASE64_TAG.str, XMLRPC_BASE64_TAG.dim ) == 0 )
    {
      PRINT_ERROR ( "paramFromXml() : ERROR: Tag %s not yet implemented!\n", XMLRPC_BASE64_TAG.str );
      ret=-1;

      c += XMLRPC_BASE64_TAG.dim;
      i += XMLRPC_BASE64_TAG.dim;
      type_init = c;
      p_type = XMLRPC_PARAM_BINARY;
      break;
    }
    else if ( len - i >= XMLRPC_STRUCT_TAG.dim &&
              strncmp ( c, XMLRPC_STRUCT_TAG.str, XMLRPC_STRUCT_TAG.dim ) == 0 )
    {
      c += XMLRPC_STRUCT_TAG.dim;
      i += XMLRPC_STRUCT_TAG.dim;
      type_init = c;
      p_type = XMLRPC_PARAM_STRUCT;
      break;
    }
    else if ( len - i >= XMLRPC_VALUE_ETAG.dim &&
         strncmp ( c, XMLRPC_VALUE_ETAG.str, XMLRPC_VALUE_ETAG.dim ) == 0 )
    {
      value_init = dynStringGetCurrentData ( message );
      i = dynStringGetPoseIndicatorOffset ( message );
      break;
    }
  }

  PRINT_DEBUG("paramFromXml() : Param type %d \n", p_type );

  // Update pose indicator (useful in case it is an array parameter)
  dynStringSetPoseIndicator ( message, i );

  const char *str_init = NULL;
  int str_len = 0;

  switch (p_type)
  {
    case XMLRPC_PARAM_BOOL:
    {
      int val;
      if ( sscanf ( c, "%d", &val ) == 1 )
      {
        if (container == PARAM_CONTAINER_ARRAY)
          xmlrpcParamArrayPushBackBool ( param, val );
        else
          xmlrpcParamSetBool ( param, val );
      }
      else
      {
        PRINT_ERROR ( "paramFromXml() : not valid value\n" );
        xmlrpcParamSetUnknown ( param );
        return -1;
      }
      break;
    }
    case XMLRPC_PARAM_INT:
    {
      int val;
      if ( sscanf ( c, "%d", &val ) == 1 )
      {
        if (container == PARAM_CONTAINER_ARRAY)
          xmlrpcParamArrayPushBackInt ( param, val );
        else
          xmlrpcParamSetInt ( param, val );
      }
      else
      {
        PRINT_ERROR ( "paramFromXml() : not valid value\n" );
        xmlrpcParamSetUnknown ( param );
        return -1;
      }
      break;
    }
    case XMLRPC_PARAM_DOUBLE:
    {
      double val;
      if ( sscanf ( c, "%lf", &val ) == 1 )
      {
        if (container == PARAM_CONTAINER_ARRAY)
          xmlrpcParamArrayPushBackDouble ( param, val );
        else
          xmlrpcParamSetDouble ( param, val );
      }
      else
      {
        PRINT_ERROR ( "paramFromXml() : not valid value\n" );
        xmlrpcParamSetUnknown ( param );
        return -1;
      }
      break;
    }
    case XMLRPC_PARAM_STRING:
    {
      str_init = type_init;
      break;
    }
    case XMLRPC_PARAM_ARRAY:
    {
      if (container == PARAM_CONTAINER_ARRAY)
        // Pushed new param array: start to parse for this new element
        param = xmlrpcParamArrayPushBackArray ( param );
      else
        xmlrpcParamSetArray ( param );

      if (param == NULL)
        return -1;

      int rc = arrayFromXml(message, param);
      if (rc < 0)
        return rc;

      break;
    }
    case XMLRPC_PARAM_DATETIME:
    {
      // TODO Implement me!
      break;
    }
    case XMLRPC_PARAM_BINARY:
    {
      // TODO Implement me!
      break;
    }
    case XMLRPC_PARAM_STRUCT:
    {
      if (container == PARAM_CONTAINER_ARRAY)
        // Pushed new param array: start to parse for this new element
        param = xmlrpcParamArrayPushBackStruct ( param );
      else
        xmlrpcParamSetStruct ( param );

      if (param == NULL)
        return -1;

      int rc = structFromXml(message, param);
      if (rc < 0)
        return rc;

      break;
    }
    default:
    {
      // Maybe string without <string> and </string> tabs
      str_init = value_init;
      break;
    }
  }

  c = dynStringGetCurrentData ( message );
  i = dynStringGetPoseIndicatorOffset ( message );
  for ( ; i < len; i++, c++ )
  {
    if ( p_type == XMLRPC_PARAM_BOOL && len - i >= XMLRPC_BOOLEAN_ETAG.dim &&
         strncmp ( c, XMLRPC_BOOLEAN_ETAG.str, XMLRPC_BOOLEAN_ETAG.dim ) == 0 )
    {
      type_end = c;

      c += XMLRPC_BOOLEAN_ETAG.dim;
      i += XMLRPC_BOOLEAN_ETAG.dim;
      break;
    }
    else if ( p_type == XMLRPC_PARAM_INT && len - i >= XMLRPC_I4_ETAG.dim &&
         strncmp ( c, XMLRPC_I4_ETAG.str, XMLRPC_I4_ETAG.dim ) == 0 )
    {
      type_end = c;

      c += XMLRPC_I4_ETAG.dim;
      i += XMLRPC_I4_ETAG.dim;
      break;
    }
    else if ( p_type == XMLRPC_PARAM_INT && len - i >= XMLRPC_INT_ETAG.dim &&
         strncmp ( c, XMLRPC_INT_ETAG.str, XMLRPC_INT_ETAG.dim ) == 0 )
    {
      type_end = c;

      c += XMLRPC_INT_ETAG.dim;
      i += XMLRPC_INT_ETAG.dim;
      break;
    }
    else if ( p_type == XMLRPC_PARAM_STRING && len - i >= XMLRPC_STRING_ETAG.dim &&
         strncmp ( c, XMLRPC_STRING_ETAG.str, XMLRPC_STRING_ETAG.dim ) == 0 )
    {
      type_end = c;

      c += XMLRPC_STRING_ETAG.dim;
      i += XMLRPC_STRING_ETAG.dim;
      break;
    }
    else if ( p_type == XMLRPC_PARAM_ARRAY && len - i >= XMLRPC_ARRAY_ETAG.dim &&
         strncmp ( c, XMLRPC_ARRAY_ETAG.str, XMLRPC_ARRAY_ETAG.dim ) == 0 )
    {
      type_end = c;

      c += XMLRPC_ARRAY_ETAG.dim;
      i += XMLRPC_ARRAY_ETAG.dim;
      break;
    }
    else if ( p_type == XMLRPC_PARAM_STRUCT && len - i >= XMLRPC_STRUCT_ETAG.dim &&
         strncmp ( c, XMLRPC_STRUCT_ETAG.str, XMLRPC_STRUCT_ETAG.dim ) == 0 )
    {
      type_end = c;

      c += XMLRPC_STRUCT_ETAG.dim;
      i += XMLRPC_STRUCT_ETAG.dim;
      break;
    }
    else if ( p_type == XMLRPC_PARAM_DATETIME && len - i >= XMLRPC_DATETIME_ETAG.dim &&
         strncmp ( c, XMLRPC_DATETIME_ETAG.str, XMLRPC_DATETIME_ETAG.dim ) == 0 )
    {
      type_end = c;

      c += XMLRPC_DATETIME_ETAG.dim;
      i += XMLRPC_DATETIME_ETAG.dim;
      break;
    }
    else if ( p_type == XMLRPC_PARAM_BINARY && len - i >= XMLRPC_BASE64_ETAG.dim &&
         strncmp ( c, XMLRPC_BASE64_ETAG.str, XMLRPC_BASE64_ETAG.dim ) == 0 )
    {
      type_end = c;

      c += XMLRPC_BASE64_ETAG.dim;
      i += XMLRPC_BASE64_ETAG.dim;
      break;
    }
    else if ( p_type == XMLRPC_PARAM_STRUCT && len - i >= XMLRPC_STRUCT_ETAG.dim &&
         strncmp ( c, XMLRPC_STRUCT_ETAG.str, XMLRPC_STRUCT_ETAG.dim ) == 0 )
    {
      type_end = c;

      c += XMLRPC_STRUCT_ETAG.dim;
      i += XMLRPC_STRUCT_ETAG.dim;
      break;
    }
    else if ( p_type == XMLRPC_PARAM_UNKNOWN && len - i >= XMLRPC_VALUE_ETAG.dim &&
         strncmp ( c, XMLRPC_VALUE_ETAG.str, XMLRPC_VALUE_ETAG.dim ) == 0 )
    {
      type_end = c;

      // End value tag is feeded outside
      break;
    }
    else
      str_len++;
  }

  // Update pose indicator (useful in case it is an array parameter)
  dynStringSetPoseIndicator ( message, i );

  if ( type_end == NULL )
  {
    PRINT_ERROR ( "paramFromXml() : no end type tag found\n" );
    return -1;
  }

  if( p_type == XMLRPC_PARAM_STRING || p_type == XMLRPC_PARAM_UNKNOWN )
  {
    if (container == PARAM_CONTAINER_ARRAY)
      xmlrpcParamArrayPushBackStringN ( param, str_init, str_len  );
    else
      xmlrpcParamSetStringN ( param, str_init, str_len );
  }

  return ret;
}

int structMemberFromXml ( DynString *message, XmlrpcParam *param)
{
  int rc;
  int len = dynStringGetLen ( message );
  const char *c = dynStringGetCurrentData ( message );
  int i = dynStringGetPoseIndicatorOffset ( message );

  const char *name_begin = NULL;
  const char *name_end = NULL;
  for ( ; i < len; i++, c++ )
  {
    if ( len - i >= XMLRPC_NAME_TAG.dim &&
         strncmp ( c, XMLRPC_NAME_TAG.str, XMLRPC_NAME_TAG.dim ) == 0 )
    {
      c += XMLRPC_NAME_TAG.dim;
      i += XMLRPC_NAME_TAG.dim;
      name_begin = c;
      break;
    }
  }

  for ( ; i < len; i++, c++ )
  {
    if ( len - i >= XMLRPC_NAME_ETAG.dim &&
         strncmp ( c, XMLRPC_NAME_ETAG.str, XMLRPC_NAME_ETAG.dim ) == 0 )
    {
      name_end = c;
      c += XMLRPC_NAME_ETAG.dim;
      i += XMLRPC_NAME_ETAG.dim;
      break;
    }
  }

  if (name_begin == NULL || name_end == NULL)
  {
    PRINT_ERROR ( "paramMemberFromXml() : no name type tag found\n" );
    return -1;
  }

  size_t name_len = name_end - name_begin;
  param->member_name = malloc(name_len + 1);
  if (param->member_name == NULL)
  {
    PRINT_ERROR ( "paramMemberFromXml() : Can't allocate memory\n" );
    return -1;
  }

  memcpy(param->member_name, name_begin, name_len);
  param->member_name[name_len] = '\0';

  const char *value_begin = NULL;
  const char *value_end = NULL;
  for ( ; i < len; i++, c++ )
  {
    if ( len - i >= XMLRPC_VALUE_TAG.dim &&
         strncmp ( c, XMLRPC_VALUE_TAG.str, XMLRPC_VALUE_TAG.dim ) == 0 )
    {
      c += XMLRPC_VALUE_TAG.dim;
      i += XMLRPC_VALUE_TAG.dim;
      value_begin = c;
      break;
    }
  }

  if (value_begin == NULL)
  {
    PRINT_ERROR ( "paramMemberFromXml() : no value tag found\n" );
    return -1;
  }

  dynStringSetPoseIndicator ( message, i);

  rc = paramValueFromXml(message, param, PARAM_CONTAINER_STRUCT );
  if (rc < 0)
    return rc;

  c = dynStringGetCurrentData ( message );
  i = dynStringGetPoseIndicatorOffset ( message );

  for ( ; i < len; i++, c++ )
  {
    if ( len - i >= XMLRPC_VALUE_ETAG.dim &&
        strncmp ( c, XMLRPC_VALUE_ETAG.str, XMLRPC_VALUE_ETAG.dim ) == 0 )
    {
      value_end = c;
      c += XMLRPC_VALUE_ETAG.dim;
      i += XMLRPC_VALUE_ETAG.dim;
      break;
    }
  }

  if ( value_end == NULL )
  {
    PRINT_ERROR ( "paramMemberFromXml() : no value end tag found\n" );
    return -1;
  }

  dynStringSetPoseIndicator ( message, i);

  return 0;
}

XmlrpcParam * arrayAddElem ( XmlrpcParam *param )
{
  if ( param->type != XMLRPC_PARAM_ARRAY && param->type != XMLRPC_PARAM_STRUCT )
  {
    PRINT_ERROR ( "arrayAddElem() : Not array or struct type param \n" );
    return NULL;
  }

  if ( param->array_n_elem == param->array_max_elem )
  {
    PRINT_DEBUG ( "arrayAddElem() : reallocate memory\n" );
    XmlrpcParam *new_param = ( XmlrpcParam * ) realloc ( param->data.as_array,
                             ( XMLRPC_ARRAY_GROW_RATE * param->array_max_elem ) * sizeof ( XmlrpcParam ) );
    if ( new_param == NULL )
    {
      PRINT_ERROR ( "arrayAddElem() : Can't allocate more memory\n" );
      return NULL;
    }
    param->array_max_elem *= XMLRPC_ARRAY_GROW_RATE;
    param->data.as_array = new_param;
  }

  XmlrpcParam *ret = &param->data.as_array[param->array_n_elem++];
  xmlrpcParamInit(ret);
  return ret;
}


int xmlrpcParamGetBool( XmlrpcParam *param )
{
  return param->data.as_bool?1:0;
}

int32_t xmlrpcParamGetInt( XmlrpcParam *param )
{
  return param->data.as_int;
}

double xmlrpcParamGetDouble( XmlrpcParam *param )
{
  return param->data.as_double;
}
char *xmlrpcParamGetString( XmlrpcParam *param )
{
  return param->data.as_string;
}

void xmlrpcParamSetUnknown ( XmlrpcParam *param )
{
  PRINT_VDEBUG ( "xmlrpcParamSetUnknown()\n" );

  param->type = XMLRPC_PARAM_UNKNOWN;
}

void xmlrpcParamSetBool ( XmlrpcParam *param, int val )
{
  PRINT_VDEBUG ( "xmlrpcSetBool()\n" );

  param->type = XMLRPC_PARAM_BOOL;
  param->data.as_bool = ( ( val != 0 ) ?1:0 );
  PRINT_DEBUG( "xmlrpcSetBool() : Set: %s\n", param->data.as_bool?"TRUE":"FALSE" );
}

void xmlrpcParamSetInt ( XmlrpcParam *param, int32_t val )
{
  PRINT_VDEBUG ( "xmlrpcSetInt()\n" );

  param->type = XMLRPC_PARAM_INT;
  param->data.as_int = val;
  PRINT_DEBUG( "xmlrpcSetInt() : Set: %d\n", param->data.as_int );
}

void xmlrpcParamSetDouble ( XmlrpcParam *param, double val )
{
  PRINT_VDEBUG ( "xmlrpcSetDouble()\n" );

  param->type = XMLRPC_PARAM_DOUBLE;
  param->data.as_double = val;
  PRINT_DEBUG ( "xmlrpcSetDouble() : Set: %f\n", param->data.as_double );
}

void xmlrpcParamSetString ( XmlrpcParam *param, const char *val )
{
  PRINT_VDEBUG ( "xmlrpcSetString()\n" );

  int str_len = strlen ( val );
  xmlrpcParamSetStringN ( param, val, str_len );
}

void xmlrpcParamSetStringN ( XmlrpcParam *param, const char *val, int n )
{
  PRINT_VDEBUG ( "xmlrpcSetStringN()\n" );

  param->type = XMLRPC_PARAM_STRING;
  param->data.as_string = ( char * ) malloc ( ( n + 1 ) *sizeof ( char ) );
  if ( param->data.as_string == NULL )
  {
    PRINT_ERROR ( "xmlrpcSetStringN() : Can't allocate memory\n" );
    return;
  }
  int i = 0;
  const char *c = val;
  for( ; i < n; i++, c++)
  {
    if ( n - i >= 4 && strncmp ( c, "&lt;", 4 ) == 0 )
      param->data.as_string[i] = '<';
    else if ( n - i >= 4 && strncmp ( c, "&gt;", 4 ) == 0 )
      param->data.as_string[i] = '>';
    else if ( n - i >= 5 && strncmp ( c, "&amp;", 5 ) == 0 )
      param->data.as_string[i] = '&';
    else if ( n - i >= 6 && strncmp ( c, "&apos;", 6 ) == 0 )
      param->data.as_string[i] = '\'';
    else if ( n - i >= 6 && strncmp ( c, "&quot;", 6 ) == 0 )
      param->data.as_string[i] = '\"';
    else
      param->data.as_string[i] = *c;
  }
  param->data.as_string[i] = '\0';

  PRINT_DEBUG ( "xmlrpcSetStringN() : Set: %s\n", param->data.as_string );
}

int xmlrpcParamArrayGetSize( XmlrpcParam *param )
{
  if( param->type == XMLRPC_PARAM_ARRAY)
    return param->array_n_elem;
  else
    return -1;
}

XmlrpcParam *xmlrpcParamArrayGetParamAt( XmlrpcParam *param, int idx )
{
  if( param->type == XMLRPC_PARAM_ARRAY && idx < param->array_n_elem )
    return &(param->data.as_array[idx]);
  else
    return NULL;
}

void xmlrpcParamSetArray ( XmlrpcParam *param )
{
  PRINT_VDEBUG ( "xmlrpcSetArray()\n" );
  param->type = XMLRPC_PARAM_ARRAY;

  param->data.as_array = ( XmlrpcParam * ) malloc ( XMLRPC_ARRAY_INIT_SIZE*sizeof ( XmlrpcParam ) );
  if ( param->data.as_array == NULL )
  {
    PRINT_ERROR ( "xmlrpcSetArray() : Can't allocate memory\n" );
    param->array_max_elem = param->array_n_elem = 0;
    return;
  }
  param->array_n_elem = 0;
  param->array_max_elem = XMLRPC_ARRAY_INIT_SIZE;
}

void xmlrpcParamSetStruct( XmlrpcParam *param )
{
  PRINT_VDEBUG ( "xmlrpcSetArray()\n" );
  param->type = XMLRPC_PARAM_STRUCT;

  param->data.as_array = ( XmlrpcParam * ) malloc ( XMLRPC_ARRAY_INIT_SIZE*sizeof ( XmlrpcParam ) );
  if ( param->data.as_array == NULL )
  {
    PRINT_ERROR ( "xmlrpcSetArray() : Can't allocate memory\n" );
    param->array_max_elem = param->array_n_elem = 0;
    return;
  }
  param->array_n_elem = 0;
  param->array_max_elem = XMLRPC_ARRAY_INIT_SIZE;
}


XmlrpcParamType xmlrpcParamGetType ( XmlrpcParam *param )
{
  return param->type;
}

XmlrpcParam * xmlrpcParamArrayPushBackBool ( XmlrpcParam *param, int val )
{
  PRINT_VDEBUG ( "xmlrpcParamArrayPushBackBool()\n" );
  XmlrpcParam *new_param = arrayAddElem ( param );
  if ( new_param == NULL )
    return NULL;

  xmlrpcParamSetBool ( new_param, val );
  return new_param;
}

XmlrpcParam * xmlrpcParamArrayPushBackInt ( XmlrpcParam *param, int32_t val )
{
  PRINT_VDEBUG ( "xmlrpcParamArrayPushBackInt()\n" );
  XmlrpcParam *new_param = arrayAddElem ( param );
  if ( new_param == NULL )
    return NULL;

  xmlrpcParamSetInt ( new_param, val );
  return new_param;
}

XmlrpcParam * xmlrpcParamArrayPushBackDouble ( XmlrpcParam *param, double val )
{
  PRINT_VDEBUG ( "xmlrpcParamArrayPushBackDouble()\n" );
  XmlrpcParam *new_param = arrayAddElem ( param );
  if ( new_param == NULL )
    return NULL;

  xmlrpcParamSetDouble ( new_param, val );
  return new_param;
}

XmlrpcParam * xmlrpcParamArrayPushBackString ( XmlrpcParam *param, const char *val )
{
  PRINT_VDEBUG ( "xmlrpcParamArrayPushBackString()\n" );
  XmlrpcParam *new_param = arrayAddElem ( param );
  if ( new_param == NULL )
    return NULL;

  xmlrpcParamSetString ( new_param, val );
  return new_param;
}

XmlrpcParam * xmlrpcParamArrayPushBackStringN ( XmlrpcParam *param, const char *val, int n )
{
  PRINT_VDEBUG ( "xmlrpcParamArrayPushBackStringN()\n" );
  XmlrpcParam *new_param = arrayAddElem ( param );
  if ( new_param == NULL )
    return NULL;

  xmlrpcParamSetStringN ( new_param, val, n );
  return new_param;
}

XmlrpcParam *xmlrpcParamArrayPushBackArray ( XmlrpcParam *param )
{
  PRINT_VDEBUG ( "xmlrpcParamArrayPushBackArray()\n" );
  XmlrpcParam *new_param = arrayAddElem ( param );
  if ( new_param == NULL )
    return NULL;

  xmlrpcParamSetArray ( new_param );
  return new_param;
}

XmlrpcParam * xmlrpcParamArrayPushBackStruct ( XmlrpcParam *param )
{
  PRINT_VDEBUG ( "xmlrpcParamArrayPushBackStruct()\n" );
  XmlrpcParam *new_param = arrayAddElem ( param );
  if ( new_param == NULL )
    return NULL;

  xmlrpcParamSetStruct ( new_param );
  return new_param;
}

XmlrpcParam * xmlrpcParamStructGetParam( XmlrpcParam *param, const char *name )
{
  if ( param->type != XMLRPC_PARAM_STRUCT )
  {
    PRINT_ERROR ( "xmlrpcParamStructGetParam() : Not a struct type param \n" );
    return NULL;
  }

  int it = 0;
  for (; it < param->array_n_elem; it++)
  {
    XmlrpcParam *param_arr = &param->data.as_array[it];
    if (strcmp(param_arr->member_name, name) == 0)
      return param_arr;
  }

  return NULL;
}

XmlrpcParam * xmlrpcParamStructPushBackBool( XmlrpcParam *param, const char *name, int val )
{
  PRINT_VDEBUG ( "xmlrpcParamStructPushBackBool()\n" );
  XmlrpcParam *new_param = arrayAddElem ( param );
  if ( new_param == NULL )
    return NULL;

  paramSetMemberName(new_param, name);
  xmlrpcParamSetBool ( new_param, val );
  return new_param;
}

XmlrpcParam * xmlrpcParamStructPushBackInt( XmlrpcParam *param, const char *name, int32_t val )
{
  PRINT_VDEBUG ( "xmlrpcParamStructPushBackInt()\n" );
  XmlrpcParam *new_param = arrayAddElem ( param );
  if ( new_param == NULL )
    return NULL;

  paramSetMemberName(new_param, name);
  xmlrpcParamSetInt ( new_param, val );
  return new_param;
}

XmlrpcParam * xmlrpcParamStructPushBackDouble( XmlrpcParam *param, const char *name, double val )
{
  PRINT_VDEBUG ( "xmlrpcParamStructPushBackDouble()\n" );
  XmlrpcParam *new_param = arrayAddElem ( param );
  if ( new_param == NULL )
    return NULL;

  paramSetMemberName(new_param, name);
  xmlrpcParamSetDouble ( new_param, val );
  return new_param;
}

XmlrpcParam * xmlrpcParamStructPushBackString( XmlrpcParam *param, const char *name, const char *val )
{
  PRINT_VDEBUG ( "xmlrpcParamStructPushBackString()\n" );
  XmlrpcParam *new_param = arrayAddElem ( param );
  if ( new_param == NULL )
    return NULL;

  paramSetMemberName(new_param, name);
  xmlrpcParamSetString ( new_param, val );
  return new_param;
}

XmlrpcParam * xmlrpcParamStructPushBackStringN( XmlrpcParam *param, const char *name, const char *val, int n )
{
  PRINT_VDEBUG ( "xmlrpcParamStructPushBackStringN()\n" );
  XmlrpcParam *new_param = arrayAddElem ( param );
  if ( new_param == NULL )
    return NULL;

  paramSetMemberName(new_param, name);
  xmlrpcParamSetStringN ( new_param, val, n );
  return new_param;
}

XmlrpcParam * xmlrpcParamStructPushBackArray( XmlrpcParam *param, const char *name )
{
  PRINT_VDEBUG ( "xmlrpcParamStructPushBackArray()\n" );
  XmlrpcParam *new_param = arrayAddElem ( param );
  if ( new_param == NULL )
    return NULL;

  paramSetMemberName(new_param, name);
  xmlrpcParamSetArray ( new_param );
  return new_param;
}

XmlrpcParam * xmlrpcParamStructPushBackStruct ( XmlrpcParam *param, const char *name )
{
  PRINT_VDEBUG ( "xmlrpcParamStructPushBackStruct()\n" );
  XmlrpcParam *new_param = arrayAddElem ( param );
  if ( new_param == NULL )
    return NULL;

  paramSetMemberName(new_param, name);
  xmlrpcParamSetStruct ( new_param );
  return new_param;
}

int paramSetMemberName ( XmlrpcParam *param, const char *name )
{
  param->member_name = (char *)malloc(strlen(name) + 1);
  if (param->member_name == NULL)
    return -1;
  strcpy(param->member_name, name);

  return 0;
}

void xmlrpcParamInit( XmlrpcParam *param )
{
  param->type = XMLRPC_PARAM_UNKNOWN;
  param->member_name =  NULL;
  memset(param->data.opaque, 0, sizeof(param->data.opaque));
  param->array_n_elem = -1;
  param->array_max_elem = -1;
}

void xmlrpcParamRelease ( XmlrpcParam *param )
{
  PRINT_VDEBUG ( "xmlrpcParamReleaseData()\n" );

  free(param->member_name);

  switch ( param->type )
  {
  case XMLRPC_PARAM_BOOL:
  case XMLRPC_PARAM_INT:
  case XMLRPC_PARAM_DOUBLE:
    break;

  case XMLRPC_PARAM_STRING:
    if ( param->data.as_string != NULL )
    {
      free ( param->data.as_string );
      param->data.as_string = NULL;
    }
    break;

  case XMLRPC_PARAM_ARRAY:
  case XMLRPC_PARAM_STRUCT:
    if ( param->data.as_array != NULL )
    {
      int i;
      for ( i = 0; i< param->array_n_elem; i++ )
        xmlrpcParamRelease ( & ( param->data.as_array[i] ) );
      free ( param->data.as_array );
      param->data.as_array = NULL;
    }
    param->array_n_elem = 0;
    param->array_max_elem = 0;
    break;

  case XMLRPC_PARAM_DATETIME: /* WARNING: Currently unsupported */
    PRINT_ERROR ( "xmlrpcParamReleaseData() : ERROR: Parameter type datetime not yet supported\n" );
    break;
  case XMLRPC_PARAM_BINARY: /* WARNING: Currently unsupported */
    PRINT_ERROR ( "xmlrpcParamReleaseData() : ERROR: Parameter type binary not yet supported\n" );
    break;
  case XMLRPC_PARAM_UNKNOWN:
    break;
  default:
    PRINT_DEBUG ( "xmlrpcParamReleaseData() : Unknown parameter \n" );
    break;
  }
}

void xmlrpcParamToXml ( XmlrpcParam *param, DynString *message )
{
  PRINT_VDEBUG ( "xmlrpcParamToXml()\n" );

  int struct_member = 0;
  if (param->member_name != NULL)
  {
    dynStringPushBackStr ( message, XMLRPC_MEMBER_TAG.str );
    dynStringPushBackStr ( message, XMLRPC_NAME_TAG.str );
    dynStringPushBackStr ( message, param->member_name );
    dynStringPushBackStr ( message, XMLRPC_NAME_ETAG.str );
    struct_member = 1;
  }

  switch ( param->type )
  {
  case XMLRPC_PARAM_BOOL:
    boolToXml ( param->data.as_bool, message );
    break;
  case XMLRPC_PARAM_INT:
    intToXml ( param->data.as_int, message );
    break;
  case XMLRPC_PARAM_DOUBLE:
    doubleToXml ( param->data.as_double, message );
    break;
  case XMLRPC_PARAM_STRING:
    stringToXml ( param->data.as_string, message );
    break;
  case XMLRPC_PARAM_ARRAY:
    arrayToXml ( param, message );
    break;
  case XMLRPC_PARAM_STRUCT:
    structToXml ( param, message );
    break;
  case XMLRPC_PARAM_DATETIME:
    timeToXml ( param->data.as_time, message );
    break;
  case XMLRPC_PARAM_BINARY:
    binaryToXml ( param->data.as_binary, message );
    break;
  case XMLRPC_PARAM_UNKNOWN:
    break;
  default:
    PRINT_ERROR ( "xmlrpcParamToXml() : Unsupported type\n" );
    break;
  }

  if (struct_member)
    dynStringPushBackStr ( message, XMLRPC_MEMBER_ETAG.str );
}

int xmlrpcParamFromXml ( DynString *message, XmlrpcParam *param )
{
  PRINT_VDEBUG ( "xmlrpcParamFromXml()\n" );

  /* Save position indicator: it will be restored */
  int initial_pos_idx = dynStringGetPoseIndicatorOffset ( message );
  dynStringRewindPoseIndicator ( message );

  int ret = paramFromXml ( message, param, PARAM_CONTAINER_NONE );

  /* Restore position indicator */
  dynStringSetPoseIndicator ( message, initial_pos_idx );

  return ret;
}

static void paramPrint( XmlrpcParam *param, char *head )
{
  int i;
  switch ( param->type )
  {
  case XMLRPC_PARAM_BOOL:
    printf("%s type : boolean Value : [%s]\n", head, param->data.as_bool?"TRUE":"FALSE" );
    break;
  case XMLRPC_PARAM_INT:
    printf("%s type : integer Value : [%d]\n", head, param->data.as_int );
    break;
  case XMLRPC_PARAM_DOUBLE:
    printf("%s type : double Value : [%f]\n", head, param->data.as_double );
    break;
  case XMLRPC_PARAM_STRING:
    printf("%s type : string Value : [%s]\n", head,
           (param->data.as_string != NULL)?param->data.as_string:"NULL" );
    break;

  case XMLRPC_PARAM_ARRAY:
    printf("%s type : array\n\n========Start array========\n", head);
    if ( param->data.as_array != NULL && param->array_n_elem )
    {
      for ( i = 0; i< param->array_n_elem; i++ )
      {
        char elem_head[20];
        snprintf(elem_head, 20, "Elem [%d]",i);
        paramPrint( & ( param->data.as_array[i] ), elem_head );
      }
    }
    else
      printf("Empty array\n");

    printf("=========End array=========\n\n");
    break;

  case XMLRPC_PARAM_DATETIME: /* WARNING: Currently unsupported */
  case XMLRPC_PARAM_BINARY: /* WARNING: Currently unsupported */
  case XMLRPC_PARAM_STRUCT: /* WARNING: Currently unsupported */
    PRINT_ERROR ( "xmlrpcParamPrint() : Parameter type not yet supported\n" );
    break;

  default:
    PRINT_ERROR ( "xmlrpcParamPrint() : Unknown parameter \n" );
    break;
  }
}

void xmlrpcParamPrint( XmlrpcParam *param )
{
  paramPrint( param, "XMLRPC parameter" );
}

XmlrpcParam * xmlrpcParamNew()
{
  XmlrpcParam *ret = (XmlrpcParam *)malloc(sizeof(XmlrpcParam));
  if (ret == NULL)
    return NULL;

  xmlrpcParamInit(ret);

  return ret;
}

void xmlrpcParamFree( XmlrpcParam *param )
{
  if (param == NULL)
    return;

  xmlrpcParamRelease(param);
  free(param);
}

XmlrpcParam * xmlrpcParamClone(XmlrpcParam *source)
{
  XmlrpcParam *ret = (XmlrpcParam *)malloc(sizeof(XmlrpcParam));
  if (ret == NULL)
    return NULL;

  int rc = xmlrpcParamCopy(ret, source);
  if (rc == -1)
  {
    free(ret);
    return NULL;
  }

  return ret;
}

int xmlrpcParamCopy(XmlrpcParam *dest, XmlrpcParam *source)
{
  memcpy(dest, source, sizeof(XmlrpcParam));
  if (source->member_name != NULL)
  {
    dest->member_name = (char *)malloc(strlen(source->member_name) + 1);
    strcpy(dest->member_name, source->member_name);
  }

  switch ( source->type )
  {
    case XMLRPC_PARAM_BOOL:
    case XMLRPC_PARAM_INT:
    case XMLRPC_PARAM_DOUBLE:
      break;
    case XMLRPC_PARAM_STRING:
      dest->data.as_string = (char *)malloc(strlen(source->data.as_string) + 1);
      if (dest->data.as_string == NULL)
        goto clean;
      strcpy(dest->data.as_string, source->data.as_string);
      break;
    case XMLRPC_PARAM_ARRAY:
    case XMLRPC_PARAM_STRUCT:
      dest->data.as_array = (XmlrpcParam *)calloc(source->array_n_elem, sizeof(XmlrpcParam));
      if (dest->data.as_array == NULL)
        goto clean;

      int it = 0;
      for (; it < source->array_n_elem; it++)
      {
        int rc = xmlrpcParamCopy(&dest->data.as_array[it], &source->data.as_array[it]);
        if (rc == -1)
          goto clean;
      }
      break;
    case XMLRPC_PARAM_DATETIME:
    case XMLRPC_PARAM_BINARY:
      PRINT_ERROR ( "xmlrpcParamToXml() : Unsupported type\n" );
      assert(0);
      break;
    case XMLRPC_PARAM_UNKNOWN:
      break;
    default:
      PRINT_ERROR ( "xmlrpcParamToXml() : Unsupported type\n" );
      assert(0);
  }

  return 0;

clean:
  xmlrpcParamRelease(dest);
  return -1;
}

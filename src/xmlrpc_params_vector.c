#include <stdlib.h>

#include "xmlrpc_params_vector.h"
#include "cros_defs.h"
#include "cros_log.h"

enum { XMLRPC_VECTOR_INIT_SIZE = 4, XMLRPC_VECTOR_GROW_RATE = 2 };

void xmlrpcParamVectorInit ( XmlrpcParamVector *p_vec )
{
  PRINT_VVDEBUG ( "xmlrpcParamVectorInit()\n" );

  p_vec->data = NULL;
  p_vec->size = 0;
  p_vec->max = 0;
}

void xmlrpcParamVectorRelease ( XmlrpcParamVector *p_vec )
{
  PRINT_VVDEBUG ( "xmlrpcParamVectorRelease()\n" );

  if ( p_vec->data == NULL )
    return;

  int i;
  for ( i = 0; i < p_vec->size; i++ )
    xmlrpcParamRelease ( & ( p_vec->data[i] ) );

  free ( p_vec->data );
  p_vec->data = NULL;

  p_vec->size = p_vec->max = 0;
}

int xmlrpcParamVectorPushBackBool ( XmlrpcParamVector *p_vec, int val )
{
  PRINT_VVDEBUG ( "xmlrpcParamVectorPushBackBool()\n" );

  XmlrpcParam param;
  xmlrpcParamInit(&param);
  xmlrpcParamSetBool ( &param, val );
  return xmlrpcParamVectorPushBack ( p_vec, &param );
}

int xmlrpcParamVectorPushBackInt ( XmlrpcParamVector *p_vec, int32_t val )
{
  PRINT_VVDEBUG ( "xmlrpcParamVectorPushBackInt()\n" );

  XmlrpcParam param;
  xmlrpcParamInit(&param);
  xmlrpcParamSetInt ( &param, val );
  return xmlrpcParamVectorPushBack ( p_vec, &param );
}

int xmlrpcParamVectorPushBackDouble ( XmlrpcParamVector *p_vec, double val )
{
  PRINT_VVDEBUG ( "xmlrpcParamVectorPushBackDouble()\n" );

  XmlrpcParam param;
  xmlrpcParamInit(&param);
  xmlrpcParamSetDouble ( &param, val );
  return xmlrpcParamVectorPushBack ( p_vec, &param );
}

int xmlrpcParamVectorPushBackString ( XmlrpcParamVector *p_vec, const char *val )
{
  PRINT_VVDEBUG ( "xmlrpcParamVectorPushBackString()\n" );

  XmlrpcParam param;
  xmlrpcParamInit(&param);
  xmlrpcParamSetString ( &param, val );
  return xmlrpcParamVectorPushBack ( p_vec, &param );
}

int xmlrpcParamVectorPushBackArray ( XmlrpcParamVector *p_vec )
{
  PRINT_VVDEBUG ( "xmlrpcParamVectorPushBackArray()\n" );

  XmlrpcParam param;
  xmlrpcParamInit(&param);
  xmlrpcParamSetArray ( &param );
  return xmlrpcParamVectorPushBack ( p_vec, &param );
}

int xmlrpcParamVectorPushBackStruct ( XmlrpcParamVector *p_vec )
{
  PRINT_VVDEBUG ( "xmlrpcParamVectorPushBackStruct()\n" );

  XmlrpcParam param;
  xmlrpcParamInit(&param);
  xmlrpcParamSetStruct ( &param );
  return xmlrpcParamVectorPushBack ( p_vec, &param );
}

int xmlrpcParamVectorPushBack ( XmlrpcParamVector *p_vec, XmlrpcParam *param )
{
  PRINT_VVDEBUG ( "xmlrpcParamVectorPushBack()\n" );

  if ( param == NULL )
  {
    PRINT_ERROR ( "xmlrpcParamVectorPushBack() : Invalid new param\n" );
    return -1;
  }

  if ( p_vec->data == NULL )
  {
    PRINT_VVDEBUG ( "xmlrpcParamVectorPushBack() : allocate memory for the first time\n" );
    p_vec->data = ( XmlrpcParam * ) malloc ( XMLRPC_VECTOR_INIT_SIZE * sizeof ( XmlrpcParam ) );

    if ( p_vec->data == NULL )
    {
      PRINT_ERROR ( "xmlrpcParamVectorPushBack() : Can't allocate memory\n" );
      return -1;
    }

    p_vec->size = 0;
    p_vec->max = XMLRPC_VECTOR_INIT_SIZE;
  }

  while ( p_vec->size == p_vec->max )
  {
    PRINT_VVDEBUG ( "xmlrpcParamVectorPushBack() : reallocate memory\n" );
    XmlrpcParam *new_p_vec = ( XmlrpcParam * ) realloc ( p_vec->data,
                             ( XMLRPC_VECTOR_GROW_RATE* p_vec->max ) * sizeof ( XmlrpcParam ) );
    if ( new_p_vec == NULL )
    {
      PRINT_ERROR ( "xmlrpcParamVectorPushBack() : Can't allocate more memory\n" );
      return -1;
    }
    p_vec->max *= XMLRPC_VECTOR_GROW_RATE;
    p_vec->data = new_p_vec;
  }

  p_vec->data[p_vec->size] = *param;
  p_vec->size += 1;

  return p_vec->size;
}

int xmlrpcParamVectorGetSize ( XmlrpcParamVector *p_vec )
{
  PRINT_VVDEBUG ( "xmlrpcParamVectorGetSize()\n" );

  return p_vec->size;
}

XmlrpcParam *xmlrpcParamVectorAt ( XmlrpcParamVector *p_vec, int pos )
{
  PRINT_VVDEBUG ( "xmlrpcParamVectorAt()\n" );

  if ( pos < 0 || pos >= p_vec->size )
  {
    PRINT_ERROR ( "xmlrpcParamVectorAt() : index out of range\n" );
    return NULL;
  }

  return & ( p_vec->data[pos] );
}

void xmlrpcParamVectorPrint( XmlrpcParamVector *p_vec )
{
  int elem_ind;

  PRINT_VDEBUG ( "XMLRPC parameter vector (size : %d):\n", p_vec->size);

  for(elem_ind = 0 ; elem_ind < p_vec->size; elem_ind++)
  {
    PRINT_VDEBUG ( "Vec. elem [%i]: ", elem_ind );
    xmlrpcParamPrint( xmlrpcParamVectorAt( p_vec, elem_ind ) );
  }
}

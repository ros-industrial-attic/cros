#include <malloc.h>
#include <string.h>

#include "dyn_buffer.h"
#include "cros_defs.h"

enum { DYNBUFFER_INIT_SIZE = 256, DYNBUFFER_GROW_RATE = 2 };

void dynBufferInit ( DynBuffer *d_buf )
{
  PRINT_VDEBUG ( "dynBufferInit()\n" );

  d_buf->data = NULL;
  d_buf->size = 0;
  d_buf->pos_offset = 0;
  d_buf->max = 0;
}

void dynBufferRelease ( DynBuffer *d_buf )
{
  PRINT_VDEBUG ( "dynBufferRelease()\n" );

  if ( d_buf->data != NULL )
    free ( d_buf->data );

  d_buf->size = 0;
  d_buf->pos_offset = 0;
  d_buf->max = 0;
}

int dynBufferPushBackBuf ( DynBuffer *d_buf, const unsigned char *new_buf, int n )
{
  PRINT_VDEBUG ( "dynBufferPushBackBuf()\n" );

  if ( new_buf == NULL || n < 0 )
  {
    PRINT_ERROR ( "dynBufferPushBackBuf() : Invalid new buffer\n" );
    return -1;
  }

  if ( d_buf->data == NULL )
  {
    PRINT_DEBUG ( "dynBufferPushBackBuf() : allocate memory for the first time\n" );
    d_buf->data = ( unsigned char * ) malloc ( DYNBUFFER_INIT_SIZE * sizeof ( unsigned char ) );

    if ( d_buf->data == NULL )
    {
      PRINT_ERROR ( "dynBufferPushBackBuf() : Can't allocate memory\n" );
      return -1;
    }

    d_buf->size = 0;
    d_buf->max = DYNBUFFER_INIT_SIZE;
  }

  while ( d_buf->size + n > d_buf->max )
  {
    PRINT_DEBUG ( "dynBufferPushBackBuf() : reallocate memory\n" );
    unsigned char *new_d_buf = ( unsigned char * ) realloc ( d_buf->data, ( DYNBUFFER_GROW_RATE * d_buf->max ) * 
                                                                             sizeof ( unsigned char ) );
    if ( new_d_buf == NULL )
    {
      PRINT_ERROR ( "dynBufferPushBackBuf() : Can't allocate more memory\n" );
      return -1;
    }
    d_buf->max *= DYNBUFFER_GROW_RATE;
    d_buf->data = new_d_buf;
  }

  memcpy ( ( void * ) ( d_buf->data + d_buf->size ), ( void * ) new_buf, ( size_t ) n );
  d_buf->size += n;

  return d_buf->size;
}

int dynBufferPushBackUint32 ( DynBuffer *d_buf, uint32_t val )
{
  PRINT_VDEBUG ( "dynBufferPushBack()\n" );

  return dynBufferPushBackBuf ( d_buf, ( unsigned char * ) ( &val ), sizeof ( uint32_t ) );
}

int dynBufferPushBackDouble ( DynBuffer *d_buf, double val )
{
  PRINT_VDEBUG ( "dynBufferPushBack()\n" );

  return dynBufferPushBackBuf ( d_buf, ( unsigned char * ) ( &val ), sizeof ( double ) );
}

void dynBufferClear ( DynBuffer *d_buf )
{
  PRINT_VDEBUG ( "dynBufferClear()\n" );

  if ( d_buf->data == NULL )
    return;

  d_buf->size = 0;
  d_buf->pos_offset = 0;
}

int dynBufferGetSize ( DynBuffer *d_buf )
{
  PRINT_VDEBUG ( "dynBufferGetSize()\n" );

  return d_buf->size;
}

const unsigned char *dynBufferGetData ( DynBuffer *d_buf )
{
  PRINT_VDEBUG ( "dynBufferGetData()\n" );

  return ( const unsigned char * ) d_buf->data;
}

void dynBufferMovePoseIndicator ( DynBuffer *d_buf, int offset )
{
  PRINT_VDEBUG ( "dynBufferMovePoseIndicator()\n" );

  d_buf->pos_offset += offset;

  if ( d_buf->pos_offset < 0 )
    d_buf->pos_offset = 0;
  if ( d_buf->pos_offset > d_buf->size )
    d_buf->pos_offset = d_buf->size;
}

void dynBufferSetPoseIndicator( DynBuffer *d_buf, int pos )
{
  PRINT_VDEBUG ( "dynBufferSetPoseIndicator()\n" );

  d_buf->pos_offset = pos;
  if ( d_buf->pos_offset < 0 )
    d_buf->pos_offset = 0;
  if ( d_buf->pos_offset > d_buf->size )
    d_buf->pos_offset = d_buf->size;  
}

void dynBufferRewindPoseIndicator ( DynBuffer *d_buf )
{
  PRINT_VDEBUG ( "dynBufferRewindPoseIndicator()\n" );

  d_buf->pos_offset = 0;
}

int dynBufferGetPoseIndicatorOffset ( DynBuffer *d_buf )
{
  PRINT_VDEBUG ( "dynBufferGetPoseIndicatorOffset()\n" );

  return d_buf->pos_offset;
}

const unsigned char *dynBufferGetCurrentData ( DynBuffer *d_buf )
{
  PRINT_VDEBUG ( "dynBufferGetCurrentData()\n" );

  return ( const unsigned char * ) ( d_buf->data + d_buf->pos_offset );
}

int dynBufferGetRemainingDataSize ( DynBuffer *d_buf )
{
  PRINT_VDEBUG ( "dynBufferGetRemainingDataSize()\n" );

  return d_buf->size - d_buf->pos_offset;
}

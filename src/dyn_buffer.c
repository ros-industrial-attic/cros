#ifndef __APPLE__
  #include <malloc.h>
#endif
#include <string.h>

#include "dyn_buffer.h"
#include "cros_defs.h"
#include "cros_log.h"

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

int dynBufferPushBackBuf ( DynBuffer *d_buf, const unsigned char *new_buf, size_t n )
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

  memcpy ( ( void * ) ( d_buf->data + d_buf->size ), ( void * ) new_buf, n );
  d_buf->size += n;

  return d_buf->size;
}

int dynBufferPushBackInt8( DynBuffer *d_buf, int8_t val )
{
  PRINT_VDEBUG ( "dynBufferPushBack()\n" );

  return dynBufferPushBackBuf ( d_buf, ( unsigned char * ) ( &val ), sizeof ( int8_t ) );
}

int dynBufferPushBackInt16( DynBuffer *d_buf, int16_t val )
{
  PRINT_VDEBUG ( "dynBufferPushBack()\n" );

  return dynBufferPushBackBuf ( d_buf, ( unsigned char * ) ( &val ), sizeof ( int16_t ) );
}

int dynBufferPushBackInt32 ( DynBuffer *d_buf, int32_t val )
{
  PRINT_VDEBUG ( "dynBufferPushBack()\n" );

  return dynBufferPushBackBuf ( d_buf, ( unsigned char * ) ( &val ), sizeof ( int32_t ) );
}

int dynBufferPushBackInt64( DynBuffer *d_buf, int64_t val )
{
  PRINT_VDEBUG ( "dynBufferPushBack()\n" );

  return dynBufferPushBackBuf ( d_buf, ( unsigned char * ) ( &val ), sizeof ( int64_t ) );
}

int dynBufferPushBackUInt8( DynBuffer *d_buf, uint8_t val )
{
  PRINT_VDEBUG ( "dynBufferPushBack()\n" );

  return dynBufferPushBackBuf ( d_buf, ( unsigned char * ) ( &val ), sizeof ( uint8_t ) );
}

int dynBufferPushBackUInt16( DynBuffer *d_buf, uint16_t val )
{
  PRINT_VDEBUG ( "dynBufferPushBack()\n" );

  return dynBufferPushBackBuf ( d_buf, ( unsigned char * ) ( &val ), sizeof ( uint16_t ) );
}

int dynBufferPushBackUInt32 ( DynBuffer *d_buf, uint32_t val )
{
  PRINT_VDEBUG ( "dynBufferPushBack()\n" );

  return dynBufferPushBackBuf ( d_buf, ( unsigned char * ) ( &val ), sizeof ( uint32_t ) );
}

int dynBufferPushBackUInt64( DynBuffer *d_buf, uint64_t val )
{
  PRINT_VDEBUG ( "dynBufferPushBack()\n" );

  return dynBufferPushBackBuf ( d_buf, ( unsigned char * ) ( &val ), sizeof ( uint64_t ) );
}

int dynBufferPushBackFloat32( DynBuffer *d_buf, float val )
{
  PRINT_VDEBUG ( "dynBufferPushBack()\n" );

  return dynBufferPushBackBuf ( d_buf, ( unsigned char * ) ( &val ), sizeof ( float ) );
}

int dynBufferPushBackFloat64( DynBuffer *d_buf, double val )
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

size_t dynBufferGetSize ( DynBuffer *d_buf )
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

  size_t curr = d_buf->pos_offset;
  if (offset > 0 && curr >= (d_buf->size - offset))
    d_buf->pos_offset = d_buf->size;
  else if ( offset < 0 && -offset >= curr)
    d_buf->pos_offset = 0;
  else
    d_buf->pos_offset += (size_t)offset;
}

void dynBufferSetPoseIndicator( DynBuffer *d_buf, size_t pos )
{
  PRINT_VDEBUG ( "dynBufferSetPoseIndicator()\n" );

  if ( pos > d_buf->size )
    d_buf->pos_offset = d_buf->size;
  else
    d_buf->pos_offset = pos;
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

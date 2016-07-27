#include <string.h>
#ifndef __APPLE__
  #include <malloc.h>
#endif

#include "dyn_string.h"
#include "cros_defs.h"
#include "cros_log.h"

enum { DYNSTRING_INIT_SIZE = 256, DYNSTRING_GROW_RATE = 2 };

void dynStringInit ( DynString *d_str )
{
  PRINT_VDEBUG ( "dynStringInit()\n" );

  d_str->data = NULL;
  d_str->len = 0;
  d_str->pos_offset = 0;
  d_str->max = 0;
}

void dynStringRelease ( DynString *d_str )
{
  PRINT_VDEBUG ( "dynStringRelease()\n" );

  if ( d_str->data != NULL )
    free ( d_str->data );

  d_str->len = 0;
  d_str->pos_offset = 0;
  d_str->max = 0;
}

int dynStringPushBackStr ( DynString *d_str, const char *new_str )
{
  PRINT_VDEBUG ( "dynStringPushBackStr()\n" );

  return dynStringPushBackStrN ( d_str, new_str, strlen ( new_str ) );
}

int dynStringPushBackStrN ( DynString *d_str, const char *new_str, int n )
{
  PRINT_VDEBUG ( "dynStringPushBackStrN()\n" );

  if ( new_str == NULL )
  {
    PRINT_ERROR ( "dynStringPushBackStrN() : Invalid new string\n" );
    return -1;
  }

  if ( d_str->data == NULL )
  {
    PRINT_DEBUG ( "dynStringPushBackStrN() : allocate memory for the first time\n" );
    d_str->data = ( char * ) malloc ( DYNSTRING_INIT_SIZE * sizeof ( char ) );

    if ( d_str->data == NULL )
    {
      PRINT_ERROR ( "dynStringPushBackStrN() : Can't allocate memory\n" );
      return -1;
    }

    d_str->data[0] = '\0';
    d_str->len = 0;
    d_str->max = DYNSTRING_INIT_SIZE;
  }

  while ( d_str->len + n + 1 > d_str->max )
  {
    PRINT_DEBUG ( "dynStringPushBackStrN() : reallocate memory\n" );
    char *n_d_str = ( char * ) realloc ( d_str->data, ( DYNSTRING_GROW_RATE * d_str->max ) * 
                                                                          sizeof ( char ) );
    if ( n_d_str == NULL )
    {
      PRINT_ERROR ( "dynStringPushBackStrN() : Can't allocate more memory\n" );
      return -1;
    }
    d_str->max *= DYNSTRING_GROW_RATE;
    d_str->data = n_d_str;
  }

  memcpy ( ( void * ) ( d_str->data + d_str->len ), ( void * ) new_str, ( size_t ) n );
  d_str->len += n;
  d_str->data[d_str->len] = '\0';

  return d_str->len;
}

int dynStringPushBackChar ( DynString *d_str, const char c )
{
  PRINT_VDEBUG ( "dynStringPushBackChar()\n" );

  if ( d_str->data == NULL )
  {
    PRINT_DEBUG ( "dynStringPushBackChar() : allocate memory for the first time\n" );
    d_str->data = ( char * ) malloc ( DYNSTRING_INIT_SIZE * sizeof ( char ) );

    if ( d_str->data == NULL )
    {
      PRINT_ERROR ( "dynStringPushBackChar() : Can't allocate memory\n" );
      return -1;
    }

    d_str->data[0] = '\0';
    d_str->len = 0;
    d_str->max = DYNSTRING_INIT_SIZE;
  }

  if ( d_str->len + 1 > d_str->max )
  {
    PRINT_DEBUG ( "dynStringPushBackChar() : reallocate memory\n" );
    char *n_d_str = ( char * ) realloc ( d_str->data, ( DYNSTRING_GROW_RATE * d_str->max ) * 
                                                        sizeof ( char ) );
    if ( n_d_str == NULL )
    {
      PRINT_ERROR ( "dynStringPushBackChar() : Can't allocate more memory\n" );
      return -1;
    }
    d_str->max *= DYNSTRING_GROW_RATE;
    d_str->data = n_d_str;
  }

  d_str->data[d_str->len] = c;
  d_str->len += 1;
  d_str->data[d_str->len] = '\0';

  return d_str->len;
}

int dynStringPatch ( DynString *d_str, const char *new_str, int pos )
{
  PRINT_VDEBUG ( "dynStringPatch()\n" );

  if ( new_str == NULL )
  {
    PRINT_ERROR ( "dynStringPatch() : Invalid new string\n" );
    return -1;
  }

  if ( d_str->data == NULL )
  {
    PRINT_ERROR ( "dynStringPatch() : Empty dynamic string (NULL data)\n" );
    return -1;
  }

  if ( pos >= d_str->len )
  {
    PRINT_ERROR ( "dynStringPatch() : The starting position for the copy must be smaller \
                  than the current dynamic string lenght\n" );
    return -1;
  }

  int new_str_len = strlen ( new_str );
  int str_final_len = pos + new_str_len;

  while ( str_final_len + 1 > d_str->max )
  {
    PRINT_DEBUG ( "dynStringPatch() : reallocate memory\n" );
    char *n_d_str = ( char * ) realloc ( d_str->data, ( DYNSTRING_GROW_RATE * d_str->max ) * 
                                                        sizeof ( char ) );
    if ( n_d_str == NULL )
    {
      PRINT_ERROR ( "dynStringPatch() : Can't allocate more memory\n" );
      return -1;
    }
    d_str->max *= DYNSTRING_GROW_RATE;
    d_str->data = n_d_str;
  }

  memcpy ( ( void * ) ( d_str->data + pos ), ( void * ) new_str, ( size_t ) new_str_len );

  if ( str_final_len > d_str->len )
  {
    d_str->data[str_final_len] = '\0';
    d_str->len = str_final_len;
  }

  return d_str->len;
}
void dynStringClear ( DynString *d_str )
{
  PRINT_VDEBUG ( "dynStringClear()\n" );

  if ( d_str->data == NULL )
    return;

  d_str->data[0] = '\0';
  d_str->pos_offset = 0;
  d_str->len = 0;
}

int dynStringGetLen ( DynString *d_str )
{
  PRINT_VDEBUG ( "dynStringGetLen()\n" );

  return d_str->len;
}

const char *dynStringGetData ( DynString *d_str )
{
  PRINT_VDEBUG ( "getDynStringBuf()\n" );

  return ( const char * ) d_str->data;
}

void dynStringMovePoseIndicator ( DynString *d_str, int offset )
{
  PRINT_VDEBUG ( "dynStringMovePoseIndicator()\n" );

  d_str->pos_offset += offset;
  if ( d_str->pos_offset < 0 )
    d_str->pos_offset = 0;
  if ( d_str->pos_offset > d_str->len )
    d_str->pos_offset = d_str->len;
}

void dynStringSetPoseIndicator ( DynString *d_str, int pos )
{
  PRINT_VDEBUG ( "dynStringSetPoseIndicator()\n" );

  d_str->pos_offset = pos;
  if ( d_str->pos_offset < 0 )
    d_str->pos_offset = 0;
  if ( d_str->pos_offset > d_str->len )
    d_str->pos_offset = d_str->len;
}

void dynStringRewindPoseIndicator ( DynString *d_str )
{
  PRINT_VDEBUG ( "dynStringRewindPoseIndicator()\n" );

  d_str->pos_offset = 0;
}

int dynStringGetPoseIndicatorOffset ( DynString *d_str )
{
  PRINT_VDEBUG ( "dynStringGetPoseIndicatorOffset()\n" );

  return d_str->pos_offset;
}

const char *dynStringGetCurrentData ( DynString *d_str )
{
  PRINT_VDEBUG ( "dynStringGetCurrentData()\n" );

  return ( const char * ) ( d_str->data + d_str->pos_offset );
}

int dynStringGetRemainingDataSize ( DynString *d_str )
{
  PRINT_VDEBUG ( "dynStringGetRemainingDataSize()\n" );

  return d_str->len - d_str->pos_offset;
}

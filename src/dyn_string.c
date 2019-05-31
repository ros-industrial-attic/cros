#include <stdlib.h>
#include <string.h>

#include "dyn_string.h"
#include "cros_defs.h"
#include "cros_log.h"

enum { DYNSTRING_INIT_SIZE = 256, DYNSTRING_GROW_RATE = 2 };

void dynStringInit ( DynString *d_str )
{
  PRINT_VVDEBUG ( "dynStringInit()\n" );

  d_str->data = NULL;
  d_str->len = 0;
  d_str->pos_offset = 0;
  d_str->max = 0;
}

void dynStringRelease ( DynString *d_str )
{
  PRINT_VVDEBUG ( "dynStringRelease()\n" );

  if ( d_str->data != NULL )
  {
    free ( d_str->data );
    d_str->data = NULL;
  }
  d_str->len = 0;
  d_str->pos_offset = 0;
  d_str->max = 0;
}

int dynStringPushBackStr ( DynString *d_str, const char *new_str )
{
  PRINT_VVDEBUG ( "dynStringPushBackStr()\n" );

  return dynStringPushBackStrN ( d_str, new_str, strlen ( new_str ) );
}

int dynStringPushBackStrN ( DynString *d_str, const char *new_str, int n )
{
  PRINT_VVDEBUG ( "dynStringPushBackStrN()\n" );

  if ( new_str == NULL )
  {
    PRINT_ERROR ( "dynStringPushBackStrN() : Invalid new string\n" );
    return -1;
  }

  if ( d_str->data == NULL )
  {
    PRINT_VVDEBUG ( "dynStringPushBackStrN() : allocate memory for the first time\n" );
    d_str->data = ( char * ) malloc ( (DYNSTRING_INIT_SIZE + n + 1) * sizeof ( char ) ); // Reserve one char at the end of the array (+1) to store the \0 terminating char

    if ( d_str->data == NULL )
    {
      PRINT_ERROR ( "dynStringPushBackStrN() : Can't allocate memory\n" );
      return -1;
    }

    d_str->len = 0;
    d_str->max = DYNSTRING_INIT_SIZE + n;
  }
  else
    if(d_str->len + n > d_str->max)
    {
      PRINT_VVDEBUG ( "dynStringPushBackStrN() : reallocate memory\n" );
      char *n_d_str = ( char * ) realloc ( d_str->data, ( DYNSTRING_GROW_RATE * d_str->max + n + 1) * sizeof ( char ) );
      if ( n_d_str == NULL )
      {
        PRINT_ERROR ( "dynStringPushBackStrN() : Can't reallocate more memory\n" );
        return -1;
      }
      d_str->max =  DYNSTRING_GROW_RATE * d_str->max + n;
      d_str->data = n_d_str;
    }

  memcpy ( ( void * ) ( d_str->data + d_str->len ), ( void * ) new_str, ( size_t ) n );
  d_str->len += n;
  d_str->data[d_str->len] = '\0';

  return d_str->len;
}

int dynStringReplaceWithStrN ( DynString *d_str, const char *new_str, int n )
{
    int new_len;

    d_str->len=0; // Clear old content
    d_str->pos_offset=0;
    new_len = dynStringPushBackStrN ( d_str, new_str, n );

    return new_len;
}

int dynStringPushBackChar ( DynString *d_str, const char c )
{
  PRINT_VVDEBUG ( "dynStringPushBackChar()\n" );

  if ( d_str->data == NULL )
  {
    PRINT_VVDEBUG ( "dynStringPushBackChar() : allocate memory for the first time\n" );
    d_str->data = ( char * ) malloc ( (DYNSTRING_INIT_SIZE + 2) * sizeof ( char ) );

    if ( d_str->data == NULL )
    {
      PRINT_ERROR ( "dynStringPushBackChar() : Can't allocate memory\n" );
      return -1;
    }

    d_str->data[0] = '\0';
    d_str->len = 0;
    d_str->max = DYNSTRING_INIT_SIZE + 1;
  }
  else
    if ( d_str->len + 1 > d_str->max ) // If there is not enogh space in the array to store a new char, allocate more meory
    {
      PRINT_VVDEBUG ( "dynStringPushBackChar() : reallocate memory\n" );
      char *n_d_str = ( char * ) realloc ( d_str->data, ( DYNSTRING_GROW_RATE * d_str->max + 2) *
                                                          sizeof ( char ) );
      if ( n_d_str == NULL )
      {
        PRINT_ERROR ( "dynStringPushBackChar() : Can't allocate more memory\n" );
        return -1;
      }
      d_str->max = DYNSTRING_GROW_RATE * d_str->max + 1;
      d_str->data = n_d_str;
    }

  d_str->data[d_str->len] = c;
  d_str->len += 1;
  d_str->data[d_str->len] = '\0';

  return d_str->len;
}

int dynStringPatch ( DynString *d_str, const char *new_str, int pos )
{
  PRINT_VVDEBUG ( "dynStringPatch()\n" );

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

  if ( pos > d_str->len )
  {
    PRINT_ERROR ( "dynStringPatch() : The starting position for the copy must be smaller \
                  or equal to the current dynamic string lenght\n" );
    return -1;
  }

  int new_str_len = strlen ( new_str );
  int str_final_len = pos + new_str_len;

  if ( str_final_len > d_str->max )
  {
    PRINT_VVDEBUG ( "dynStringPatch() : reallocate memory\n" );
    char *n_d_str = ( char * ) realloc ( d_str->data, ( DYNSTRING_GROW_RATE * d_str->max + new_str_len + 1) *
                                                        sizeof ( char ) );
    if ( n_d_str == NULL )
    {
      PRINT_ERROR ( "dynStringPatch() : Can't allocate more memory\n" );
      return -1;
    }
    d_str->max = DYNSTRING_GROW_RATE * d_str->max + new_str_len;
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
  PRINT_VVDEBUG ( "dynStringClear()\n" );

  if ( d_str->data == NULL )
    return;

  d_str->data[0] = '\0';
  d_str->pos_offset = 0;
  d_str->len = 0;
}

void dynStringReduce ( DynString *d_str, int rem_left, int rem_right)
{
  PRINT_VVDEBUG ( "dynStringReduce()\n" );

  if ( d_str->data == NULL )
    return;

  if (rem_left > d_str->len)
    rem_left = d_str->len;
  if (rem_left + rem_right > d_str->len)
    rem_right = d_str->len - rem_left;

  d_str->len -= rem_left + rem_right;
  memmove(d_str->data, d_str->data+rem_left, d_str->len);
  d_str->data[d_str->len] = '\0';
  d_str->pos_offset -= rem_left;
  if(d_str->pos_offset < 0)
    d_str->pos_offset = 0;
  else
    if(d_str->pos_offset > d_str->len)
      d_str->pos_offset = d_str->len;
}

int dynStringGetLen ( DynString *d_str )
{
  PRINT_VVDEBUG ( "dynStringGetLen()\n" );

  return d_str->len;
}

const char *dynStringGetData ( DynString *d_str )
{
  PRINT_VVDEBUG ( "getDynStringBuf()\n" );

  return ( const char * ) d_str->data;
}

void dynStringMovePoseIndicator ( DynString *d_str, int offset )
{
  PRINT_VVDEBUG ( "dynStringMovePoseIndicator()\n" );

  d_str->pos_offset += offset;
  if ( d_str->pos_offset < 0 )
    d_str->pos_offset = 0;
  if ( d_str->pos_offset > d_str->len )
    d_str->pos_offset = d_str->len;
}

void dynStringSetPoseIndicator ( DynString *d_str, int pos )
{
  PRINT_VVDEBUG ( "dynStringSetPoseIndicator()\n" );

  d_str->pos_offset = pos;
  if ( d_str->pos_offset < 0 )
    d_str->pos_offset = 0;
  if ( d_str->pos_offset > d_str->len )
    d_str->pos_offset = d_str->len;
}

void dynStringRewindPoseIndicator ( DynString *d_str )
{
  PRINT_VVDEBUG ( "dynStringRewindPoseIndicator()\n" );

  d_str->pos_offset = 0;
}

int dynStringGetPoseIndicatorOffset ( DynString *d_str )
{
  PRINT_VVDEBUG ( "dynStringGetPoseIndicatorOffset()\n" );

  return d_str->pos_offset;
}

const char *dynStringGetCurrentData ( DynString *d_str )
{
  PRINT_VVDEBUG ( "dynStringGetCurrentData()\n" );

  return ( const char * ) ( d_str->data + d_str->pos_offset );
}

int dynStringGetRemainingDataSize ( DynString *d_str )
{
  PRINT_VVDEBUG ( "dynStringGetRemainingDataSize()\n" );

  return d_str->len - d_str->pos_offset;
}

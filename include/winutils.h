#pragma once

#include <string.h>

#ifdef WIN32 || WIN64
static inline void unix2win_filepath(char* filepath)
{
  int i;
  for(i = 0; i < strlen(filepath); i++)
  {
    if(filepath[i] == '/')
      filepath[i] = '\\';
  }
}
#endif
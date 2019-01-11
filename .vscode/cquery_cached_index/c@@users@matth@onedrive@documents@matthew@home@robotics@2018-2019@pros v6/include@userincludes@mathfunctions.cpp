#ifndef mathFunctions_cpp
#define mathFunctions_cpp

#include "headers/mathFunctions.h"

int sgn( float a)
{
  if(a > 0)
  {
    return 1;
  }
  else if(a < 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

#endif

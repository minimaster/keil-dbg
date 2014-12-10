#include <stdio.h>
#include "utility/trace.h"

struct __FILE
{
  int handle;

  /* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};

/* FILE is typedef’d in stdio.h. */

FILE __stdout;

int fputc(int ch, FILE *f) 
{
  DBGU_PutChar(ch);
  return ch;
}
int ferror(FILE *f)
{
  return 0;
}



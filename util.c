
#include "util.h"

#include "ch.h"
#include "hal.h"

#include "chprintf.h"
#include <errno.h>
#include <string.h>
#include <ctype.h>

// strip trailing stripchar from string
char * rtrim(char * str, char trimchar)
{
  char *pos;
  pos=strrchr(str, trimchar);
  if (pos != NULL)
      *pos = '\0';

  return str;
}
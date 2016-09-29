
#include "util.h"

#include "ch.h"
#include "hal.h"

#include "chprintf.h"
#include <errno.h>
#include <string.h>
#include <ctype.h>

void hexdump(BaseSequentialStream * bss, void *mem, unsigned int len)
{
    unsigned int i, j;
    if (bss)
    {
        for(i = 0; i < len + ((len % 16) ? (16 - len % 16) : 0); i++)
        {
            /* print offset */
            if(i % 16 == 0)
                chprintf(bss, "0x%06x: ", i);

            /* print hex data */
            if(i < len)
                chprintf(bss,"%02x ", 0xFF & ((char*)mem)[i]);
            else /* end of block, just aligning for ASCII dump */
                chprintf(bss, "   ");

            /* print ASCII dump */
            if(i % 16 == (16 - 1))
            {
                for(j = i - (16 - 1); j <= i; j++)
                {
                    if(j >= len) /* end of block, not really printing */
                        chprintf(bss," ");
                    else if( ((((char*)mem)[j]) > 0x20) && ((((char*)mem)[j]) < 0x7F)) /* printable char */
                        chprintf(bss, "%c", 0xFF & ((char*)mem)[j]);        
                    else /* other char */
                        chprintf(bss, ".");
                }
                chprintf(bss, "\r\n");
            }
        }
        chprintf(bss,"\r\n");
    }
}

// strip trailing stripchar from string
char * rtrim(char * str, char trimchar)
{
  char *pos;
  pos=strrchr(str, trimchar);
  if (pos != NULL)
      *pos = '\0';

  return str;
}
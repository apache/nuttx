#include <stdint.h>
#include <string.h>
#include <time.h>
#include <semaphore.h>
#include <debug.h>
#include <stdio.h>

#include "bcmf_utils.h"

#define LINE_LEN 16

/****************************************************************************
 * Name: bcmf_hexdump
 ****************************************************************************/

void bcmf_hexdump(uint8_t *data, unsigned int len, unsigned long offset)
{
  unsigned int i;
  unsigned int char_count = 0;
  char char_line[20];
  char hex_line[64];

  for(i = 0; i < len; i++)
    {
      if (char_count >= LINE_LEN)
      {
        /* Flush line */

        wlinfo("%08x: %s%s\n", offset+i-char_count, hex_line, char_line);
        char_count = 0;
      }

      sprintf(hex_line+3*char_count, "%02x ", data[i]);
      sprintf(char_line+char_count, "%c",
              data[i] < 0x20 || data[i] >= 0x7f? '.': data[i]);
      char_count ++;
    }

  if (char_count > 0)
    {
      /* Flush last line */

      memset(hex_line+3*char_count, ' ', 3*(LINE_LEN-char_count));
      hex_line[3*LINE_LEN] = 0;
      wlinfo("%08x: %s%s\n", offset+i-char_count, hex_line, char_line);
    }
}

/****************************************************************************
 * Name: bcmf_sem_wait
 ****************************************************************************/

int bcmf_sem_wait(sem_t *sem, unsigned int timeout_ms)
{
  struct timespec abstime;

  /* Get the current time */

  (void)clock_gettime(CLOCK_REALTIME, &abstime);

  abstime.tv_nsec += 1000 * 1000* timeout_ms;

  if (abstime.tv_nsec >= 1000 * 1000 * 1000)
    {
      abstime.tv_sec++;
      abstime.tv_nsec -= 1000 * 1000 * 1000;
    }

  return sem_timedwait(sem, &abstime);
}
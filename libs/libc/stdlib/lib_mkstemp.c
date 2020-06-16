/****************************************************************************
 * libs/libc/stdlib/lib_mkstemp.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define MAX_XS        6
#define MIN_NUMERIC   0    /* 0-9:   Numeric */
#define MAX_NUMERIC   9
#define MIN_UPPERCASE 10   /* 10-35: Upper case */
#define MAX_UPPERCASE 35
#define MIN_LOWERCASE 36   /* 36-61: Lower case */
#define MAX_LOWERCASE 61
#define MAX_BASE62    MAX_LOWERCASE

/* 62**1 = 62
 * 62**2 = 3844
 * 62**3 = 238328
 * 62**4 = 14776336
 * 62**5 = 916132832
 * 62**6 = 56800235584 > UINT32_MAX
 */

#define BIG_XS 5

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_base62[MAX_XS];
static sem_t g_b62sem = SEM_INITIALIZER(1);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: base62_to_char
 *
 * Description:
 *   Convert a base62 value to a printable character.
 *
 ****************************************************************************/

static char base62_to_char(uint8_t base62)
{
  if (base62 <= MAX_NUMERIC)
    {
      return '0' + base62;
    }
  else if (base62 <= MAX_UPPERCASE)
    {
      return 'A' + base62 - MIN_UPPERCASE;
    }
  else /* if (base62 <= MAX_LOWERCASE) */
    {
      DEBUGASSERT(base62 <= MAX_LOWERCASE);
      return 'a' + base62 - MIN_LOWERCASE;
    }
}

/****************************************************************************
 * Name: incr_base62
 *
 * Description:
 *   increment the base62 value array.
 *
 ****************************************************************************/

static void incr_base62(void)
{
  int i;

  for (i = MAX_XS - 1; i >= 0; i--)
    {
      if (g_base62[i] < MAX_LOWERCASE)
        {
          g_base62[i]++;
          return;
        }
      else
        {
          g_base62[i] = 0;
        }
    }
}

/****************************************************************************
 * Name: get_base62
 *
 * Description:
 *   Atomically copy and increment the base62 array.
 *
 ****************************************************************************/

static void get_base62(FAR uint8_t *ptr)
{
  int ret;

  while ((ret = _SEM_WAIT(&g_b62sem)) < 0)
    {
      DEBUGASSERT(_SEM_ERRNO(ret) == EINTR || _SEM_ERRNO(ret) == ECANCELED);
    }

  memcpy(ptr, g_base62, MAX_XS);
  incr_base62();
  _SEM_POST(&g_b62sem);
}

/****************************************************************************
 * Name: copy_base62
 *
 * Description:
 *   Copy the base62 array into the template filename, converting each
 *   base62 value to a printable character.
 *
 ****************************************************************************/

static void copy_base62(FAR const uint8_t *src, FAR char *dest, int len)
{
  if (len < MAX_XS)
    {
      src += MAX_XS - len;
    }

  for (; len > 0; len--)
    {
      *dest++ = base62_to_char(*src++);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkstemp
 *
 * Description:
 *   The mkstemp() function replaces the contents of the string pointed to
 *   by path_template by a unique filename, and returns a file descriptor
 *   for the file open for reading and writing. The function thus prevents
 *   any possible race condition between testing whether the file exists and
 *   opening it for use. The string in path_template should look like a
 *   filename with six trailing 'X' s; mkstemp() replaces each 'X' with a
 *   character from the portable filename character set. The characters are
 *   chosen such that the resulting name does not duplicate the name of an
 *   existing file at the time of a call to mkstemp().
 *
 * Input Parameters:
 *   path_template - The base file name that will be modified to produce
 *     the unique file name.  This must be a full path beginning with /tmp.
 *     This function will modify only the first XXXXXX characters within
 *     that full path.
 *
 * Returned Value:
 *   Upon successful completion, mkstemp() returns an open file descriptor.
 *   Otherwise, -1 is returned if no suitable file could be created.
 *
 ****************************************************************************/

int mkstemp(FAR char *path_template)
{
  uint8_t base62[MAX_XS];
  uint32_t retries;
  FAR char *xptr;
  int xlen;
  int fd;
  int i;

  /* Count the number of X's at the end of the template */

  xptr = &path_template[strlen(path_template)];
  for (xlen = 0; xlen < MAX_XS && path_template < xptr && *(xptr - 1) == 'X';
       xlen++, xptr--);

  if (xlen == 0)
    {
      /* No Xs?  There should always really be 6 */

      return open(path_template, O_RDWR | O_CREAT | O_EXCL, 0666);
    }

  /* Ignore any X's after the sixth */

  if (xlen > MAX_XS)
    {
      xptr += xlen - MAX_XS;
      xlen = MAX_XS;
    }

  /* If xlen is small, then we need to determine the maximum number of
   * retries before the values will repeat.
   */

  if (xlen >= BIG_XS)
    {
      retries = UINT32_MAX;
    }
  else
    {
      for (i = 1, retries = 62; i < xlen; i++, retries *= 62);
    }

  /* Then loop until we find a unique file name */

  while (retries > 0)
    {
      /* Sample and increment the base62 counter */

      get_base62(base62);

      /* Form the candidate file name */

      copy_base62(base62, xptr, xlen);

      /* Attempt to open the candidate file -- creating it exclusively
       *
       * REVISIT: This prohibits the use of this function to create unique
       * directories
       */

      fd = open(path_template, O_RDWR | O_CREAT | O_EXCL, 0666);
      if (fd >= 0)
        {
          /* We have it... return the file descriptor */

          return fd;
        }

      retries--;
    }

  /* We could not find an unique filename */

  return ERROR;
}


/****************************************************************************
 * libs/libc/stdlib/lib_mktemp.c
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
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <nuttx/mutex.h>

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
static mutex_t g_b62lock = NXMUTEX_INITIALIZER;

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
  nxmutex_lock(&g_b62lock);
  memcpy(ptr, g_base62, MAX_XS);
  incr_base62();
  nxmutex_unlock(&g_b62lock);
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
 * Name: mktemp
 *
 * Description:
 *   The mktemp() function generates a unique temporary filename from
 *   template. The last six characters of template must be XXXXXX and these
 *   are replaced with a string that makes the filename unique. Since it
 *   will be modified, template must not be a string constant, but should be
 *   declared as a character array.
 *
 ****************************************************************************/

FAR char *mktemp(FAR char *path_template)
{
  uint8_t base62[MAX_XS];
  uint32_t retries;
  struct stat buf;
  FAR char *xptr;
  int xlen;
  int ret;
  int i;

  /* Count the number of X's at the end of the template */

  xptr = &path_template[strlen(path_template)];
  for (xlen = 0; xlen < MAX_XS && path_template < xptr && *(xptr - 1) == 'X';
       xlen++, xptr--);

  if (xlen == 0)
    {
      /* No Xs?  There should always really be 6 */

      return path_template;
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

      /* Attempt to stat the candidate file */

      ret = stat(path_template, &buf);
      if (ret < 0 && get_errno() == ENOENT)
        {
          /* We have it... Clear the errno and return the template */

          set_errno(0);
          return path_template;
        }

      retries--;
    }

  /* We could not find an unique filename */

  set_errno(EINVAL);

  return NULL;
}

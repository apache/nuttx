/****************************************************************************
 * libs/libc/stdlib/lib_arc4random.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <sys/random.h>
#include <time.h>

#include <nuttx/hashtable.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_DEV_URANDOM) || defined(CONFIG_DEV_RANDOM)
static int getrandom_all(FAR void *buf, size_t size, int flags)
{
  FAR char *tmp = buf;

  while (size > 0)
    {
      ssize_t ret = getrandom(tmp, size, flags);
      if (ret < 0)
        {
          if (get_errno() == EINTR)
            {
              continue;
            }

          return ret;
        }

      tmp += ret;
      size -= ret;
    }

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arc4random_buf
 *
 * Description:
 *   Fill a buffer of arbitrary length with randomness. This is the
 *   preferred interface for getting random numbers. The traditional
 *   /dev/random approach is susceptible for things like the attacker
 *   exhausting file descriptors on purpose.
 *
 *   Note that this function cannot fail, other than by asserting.
 *
 * Input Parameters:
 *   bytes  - Buffer for returned random bytes
 *   nbytes - Number of bytes requested.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arc4random_buf(FAR void *bytes, size_t nbytes)
{
#if defined(CONFIG_DEV_URANDOM) || defined(CONFIG_DEV_RANDOM)
  if (getrandom_all(bytes, nbytes, GRND_RANDOM) >= 0)
    {
      return;
    }

  if (getrandom_all(bytes, nbytes, 0) >= 0)
    {
      return;
    }
#endif

  /* Fallback to hash of clock_systime_ticks(), minus nbytes to avoid getting
   * same tick count when looping more than once.
   */

  while (nbytes > 0)
    {
      uint32_t hash  = HASH(clock() - nbytes, 32);
      size_t   ncopy = MIN(nbytes, sizeof(hash));

      memcpy(bytes, &hash, ncopy);

      nbytes -= ncopy;
      bytes   = (FAR uint8_t *)bytes + ncopy;
    }
}

/****************************************************************************
 * Name: arc4random
 *
 * Description:
 *   Returns a single 32-bit value. This is the preferred interface for
 *   getting random numbers. The traditional /dev/random approach is
 *   susceptible for things like the attacker exhausting file
 *   descriptors on purpose.
 *
 *   Note that this function cannot fail, other than by asserting.
 *
 * Returned Value:
 *   a random 32-bit value.
 *
 ****************************************************************************/

uint32_t arc4random(void)
{
  uint32_t ret;

  arc4random_buf(&ret, sizeof(ret));
  return ret;
}

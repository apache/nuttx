/****************************************************************************
 * net/utils/net_getrandom.c
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

#include <stdint.h>
#include <string.h>
#include <sys/param.h>
#include <sys/random.h>

#include <nuttx/clock.h>
#include <nuttx/hashtable.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_getrandom
 *
 * Description:
 *   Fill a buffer of arbitrary length with randomness. This function is
 *   guaranteed to be success.
 *
 * Input Parameters:
 *   bytes  - Buffer for returned random bytes
 *   nbytes - Number of bytes requested.
 *
 ****************************************************************************/

void net_getrandom(FAR void *bytes, size_t nbytes)
{
#if defined(CONFIG_DEV_URANDOM) || defined(CONFIG_DEV_RANDOM)
  ssize_t ret = getrandom(bytes, nbytes, 0);

  if (ret < 0)
    {
      ret = getrandom(bytes, nbytes, GRND_RANDOM);
    }

  if (ret == nbytes)
    {
      return;
    }
#endif

  /* Fallback to hash of clock_systime_ticks(), minus nbytes to avoid getting
   * same tick count when looping more than once.
   */

  while (nbytes > 0)
    {
      uint32_t hash  = HASH(clock_systime_ticks() - nbytes, 32);
      size_t   ncopy = MIN(nbytes, sizeof(hash));

      memcpy(bytes, &hash, ncopy);

      nbytes -= ncopy;
      bytes   = (FAR uint8_t *)bytes + ncopy;
    }
}

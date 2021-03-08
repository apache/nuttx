/****************************************************************************
 * libs/libc/spawn/lib_psa_dump.c
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

/* Output debug info even if debug output is not selected. */

#undef  CONFIG_DEBUG_ERROR
#undef  CONFIG_DEBUG_WARN
#undef  CONFIG_DEBUG_INFO
#define CONFIG_DEBUG_ERROR 1
#define CONFIG_DEBUG_WARN 1
#define CONFIG_DEBUG_INFO 1

#include <spawn.h>
#include <stdint.h>
#include <debug.h>

#ifdef CONFIG_DEBUG_FEATURES

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawnattr_dump
 *
 * Description:
 *   Show the current attributes.
 *
 * Input Parameters:
 *   attr - The address of the spawn attributes to be dumped.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void posix_spawnattr_dump(posix_spawnattr_t *attr)
{
#ifdef CONFIG_DEBUG_ERROR
  _err("attr[%p]:\n", attr);
  _err("  flags:    %04x\n", attr->flags);
  if (attr->flags == 0)
    {
      _err("            None\n");
    }
  else
    {
      if ((attr->flags & POSIX_SPAWN_RESETIDS) != 0)
        {
          _err("            POSIX_SPAWN_RESETIDS\n");
        }

      if ((attr->flags & POSIX_SPAWN_SETPGROUP) != 0)
        {
          _err("            POSIX_SPAWN_SETPGROUP\n");
        }

      if ((attr->flags & POSIX_SPAWN_SETSCHEDPARAM) != 0)
        {
          _err("            POSIX_SPAWN_SETSCHEDPARAM\n");
        }

      if ((attr->flags & POSIX_SPAWN_SETSCHEDULER) != 0)
        {
          _err("            POSIX_SPAWN_SETSCHEDULER\n");
        }

      if ((attr->flags & POSIX_SPAWN_SETSIGDEF) != 0)
        {
          _err("            POSIX_SPAWN_SETSIGDEF\n");
        }

      if ((attr->flags & POSIX_SPAWN_SETSIGMASK) != 0)
        {
          _err("            POSIX_SPAWN_SETSIGMASK\n");
        }
    }

  _err("  priority: %d\n", attr->priority);

  _err("  policy:   %d\n", attr->policy);
  if (attr->policy == SCHED_FIFO)
    {
      _err("            SCHED_FIFO\n");
    }
  else if (attr->policy == SCHED_RR)
    {
      _err("            SCHED_RR\n");
    }
  else
    {
      _err("            Unrecognized\n");
    }

  _err("  sigmask:  %08jx\n", (uintmax_t)attr->sigmask);
#endif /* CONFIG_DEBUG_ERROR */
}

#endif /* CONFIG_DEBUG_FEATURES */

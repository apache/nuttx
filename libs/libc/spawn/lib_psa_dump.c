/****************************************************************************
 * libs/libc/string/lib_psa_dump.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#ifndef CONFIG_DISABLE_SIGNALS
  _err("  sigmask:  %08x\n", attr->sigmask);
#endif
#endif /* CONFIG_DEBUG_ERROR */
}

#endif /* CONFIG_DEBUG_FEATURES */

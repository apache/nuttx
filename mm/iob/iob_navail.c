/****************************************************************************
 * mm/iob/iob_navail.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>

#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_navail
 *
 * Description:
 *   Return the number of available IOBs.
 *
 ****************************************************************************/

int iob_navail(bool throttled)
{
  int navail = 0;
  int ret;

#if CONFIG_IOB_NBUFFERS > 0
  /* Get the value of the IOB counting semaphores */

  ret = nxsem_get_value(&g_iob_sem, &navail);
  if (ret >= 0)
    {
      ret = navail;

#if CONFIG_IOB_THROTTLE > 0
      /* Subtract the throttle value is so requested */

      if (throttled)
        {
          ret -= CONFIG_IOB_THROTTLE;
        }
#endif

      if (ret < 0)
        {
          ret = 0;
        }
    }

#else
  ret = navail;
#endif

  return ret;
}

/****************************************************************************
 * Name: iob_qentry_navail
 *
 * Description:
 *   Return the number of available IOB chains.
 *
 ****************************************************************************/

int iob_qentry_navail(void)
{
  int navail = 0;
  int ret;

#if CONFIG_IOB_NCHAINS > 0
  /* Get the value of the IOB chain qentry counting semaphores */

  ret = nxsem_get_value(&g_qentry_sem, &navail);
  if (ret >= 0)
    {
      ret = navail;
      if (ret < 0)
        {
          ret = 0;
        }
    }

#else
  ret = navail;
#endif

  return ret;
}

/****************************************************************************
 * mm/mm_gran/mm_grancritical.c
 *
 *   Copyright (C) 2012, 2016-2017 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/semaphore.h>
#include <nuttx/irq.h>
#include <nuttx/mm/gran.h>

#include "mm_gran/mm_gran.h"

#ifdef CONFIG_GRAN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gran_enter_critical and gran_leave_critical
 *
 * Description:
 *   Critical section management for the granule allocator.
 *
 * Input Parameters:
 *   priv - Pointer to the gran state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gran_enter_critical(FAR struct gran_s *priv)
{
#ifdef CONFIG_GRAN_INTR
  priv->irqstate = enter_critical_section();
#else
  nxsem_wait_uninterruptible(&priv->exclsem);
#endif
}

void gran_leave_critical(FAR struct gran_s *priv)
{
#ifdef CONFIG_GRAN_INTR
  leave_critical_section(priv->irqstate);
#else
  nxsem_post(&priv->exclsem);
#endif
}


#endif /* CONFIG_GRAN */

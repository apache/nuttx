/****************************************************************************
 * mm/mm_gran/mm_grancritical.c
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

#include <stdlib.h>
#include <assert.h>
#include <errno.h>

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
 *   gran_enter_critical() may return any error reported by
 *   nxsem_wait_uninterruptible()
 *
 ****************************************************************************/

int gran_enter_critical(FAR struct gran_s *priv)
{
#ifdef CONFIG_GRAN_INTR
  priv->irqstate = enter_critical_section();
  return OK;
#else
  return nxmutex_lock(&priv->lock);
#endif
}

void gran_leave_critical(FAR struct gran_s *priv)
{
#ifdef CONFIG_GRAN_INTR
  leave_critical_section(priv->irqstate);
#else
  nxmutex_unlock(&priv->lock);
#endif
}

#endif /* CONFIG_GRAN */

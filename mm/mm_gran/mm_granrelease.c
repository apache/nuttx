/****************************************************************************
 * mm/mm_gran/mm_granrelease.c
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

#include <assert.h>
#include <errno.h>

#include <nuttx/mm/gran.h>
#include <nuttx/kmalloc.h>

#include "mm_gran/mm_gran.h"

#ifdef CONFIG_GRAN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gran_release
 *
 * Description:
 *   Uninitialize a gram memory allocator and release resources held by the
 *   allocator.
 *
 * Input Parameters:
 *   handle - The handle previously returned by gran_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void gran_release(GRAN_HANDLE handle)
{
  FAR struct gran_s *priv = (FAR struct gran_s *)handle;

  DEBUGASSERT(priv != NULL);

#ifndef CONFIG_GRAN_INTR
  nxmutex_destroy(&priv->lock);
#endif
  kmm_free(priv);
}

#endif /* CONFIG_GRAN */

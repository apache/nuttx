/****************************************************************************
 * mm/umm_heap/umm_sem.c
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

#include <nuttx/mm/mm.h>

#include "umm_heap/umm_heap.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: umm_trysemaphore
 *
 * Description:
 *   This is a simple wrapper for the mm_trysemaphore() function.  This
 *   function is exported from the user-space blob so that the kernel
 *   can manage the user-mode allocator.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int umm_trysemaphore(void)
{
  return mm_trysemaphore(USR_HEAP);
}

/****************************************************************************
 * Name: umm_givesemaphore
 *
 * Description:
 *   This is a simple wrapper for the mm_givesemaphore() function.  This
 *   function is exported from the user-space blob so that the kernel
 *   can manage the user-mode allocator.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

void umm_givesemaphore(void)
{
  mm_givesemaphore(USR_HEAP);
}

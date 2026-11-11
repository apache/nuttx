/****************************************************************************
 * libs/libc/unistd/lib_domainname.c
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

#include <nuttx/config.h>

#include <errno.h>
#include <limits.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

/* The domain name is a system property, as the host name is.  In the
 * protected and kernel builds it is kept only in the kernel, and user
 * space calls these functions through system calls.
 */

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_domainname[HOST_NAME_MAX + 1];
static spinlock_t g_domainname_lock = SP_UNLOCKED;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getdomainname
 *
 * Description:
 *   Get the NIS domain name of the system.  The name is empty until
 *   setdomainname() sets it.
 *
 * Input Parameters:
 *   name    - The buffer for the name.
 *   namelen - The size of the buffer in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success.  -1 (ERROR) and errno EINVAL if the name and its
 *   terminating null byte do not fit in namelen bytes.
 *
 ****************************************************************************/

int getdomainname(FAR char *name, size_t namelen)
{
  irqstate_t flags;
  size_t len;

  flags = spin_lock_irqsave(&g_domainname_lock);
  len = strlcpy(name, g_domainname, namelen);
  spin_unlock_irqrestore(&g_domainname_lock, flags);

  if (len >= namelen)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: setdomainname
 *
 * Description:
 *   Set the NIS domain name of the system to the namelen bytes at name.
 *   The name does not need a terminating null byte.
 *
 * Input Parameters:
 *   name    - The new name.
 *   namelen - The length of the name in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success.  -1 (ERROR) and errno EINVAL if namelen is more
 *   than HOST_NAME_MAX.
 *
 ****************************************************************************/

int setdomainname(FAR const char *name, size_t namelen)
{
  irqstate_t flags;

  if (namelen > HOST_NAME_MAX)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  flags = spin_lock_irqsave(&g_domainname_lock);
  memcpy(g_domainname, name, namelen);
  g_domainname[namelen] = '\0';
  spin_unlock_irqrestore(&g_domainname_lock, flags);

  return OK;
}

#endif /* CONFIG_BUILD_FLAT || __KERNEL__ */

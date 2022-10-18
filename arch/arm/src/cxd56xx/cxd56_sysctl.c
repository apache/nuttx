/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_sysctl.c
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

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include "cxd56_icc.h"
#include "cxd56_sysctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_CXD56_SYSCTL_TIMEOUT
#  define SYSCTL_TIMEOUT CONFIG_CXD56_SYSCTL_TIMEOUT
#else
#  define SYSCTL_TIMEOUT 5000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  sysctl_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  sysctl_rxhandler(int cpuid, int protoid,
                             uint32_t pdata, uint32_t data,
                             void *userdata);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_lock;
static sem_t g_sync;
static int g_errcode = 0;

static const struct file_operations g_sysctlfops =
{
  .ioctl = sysctl_ioctl
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sysctl_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      default:
        {
          _err("cmd %x(%lx)\n", cmd, arg);
          ret = cxd56_sysctlcmd(cmd & 0xff, arg);
        }
    }

  return ret;
}

static int sysctl_rxhandler(int cpuid, int protoid,
                            uint32_t pdata, uint32_t data,
                            void *userdata)
{
  DEBUGASSERT(cpuid == 0);
  DEBUGASSERT(protoid == CXD56_PROTO_SYSCTL);

  g_errcode = (int)data;

  nxsem_post(&g_sync);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cxd56_sysctlcmd(uint8_t id, uint32_t data)
{
  iccmsg_t msg;
  int ret;

  /* Get exclusive access */

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  msg.cpuid = 0;
  msg.msgid = id;
  msg.data  = data;

  /* Send any message to system CPU */

  ret = cxd56_iccsend(CXD56_PROTO_SYSCTL, &msg, SYSCTL_TIMEOUT);
  if (ret < 0)
    {
      nxmutex_unlock(&g_lock);
      _err("Timeout.\n");
      return ret;
    }

  /* Wait for reply message from system CPU */

  ret = nxsem_wait_uninterruptible(&g_sync);
  if (ret < 0)
    {
      nxmutex_unlock(&g_lock);
      return ret;
    }

  /* Get the error code returned from system cpu */

  ret = g_errcode;

  nxmutex_unlock(&g_lock);
  return ret;
}

void cxd56_sysctlinitialize(void)
{
  cxd56_iccinit(CXD56_PROTO_SYSCTL);

  nxmutex_init(&g_lock);
  nxsem_init(&g_sync, 0, 0);

  cxd56_iccregisterhandler(CXD56_PROTO_SYSCTL, sysctl_rxhandler, NULL);

  register_driver("/dev/sysctl", &g_sysctlfops, 0666, NULL);
}

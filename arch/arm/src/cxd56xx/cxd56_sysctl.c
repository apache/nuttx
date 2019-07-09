/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_sysctl.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <semaphore.h>

#include "cxd56_icc.h"
#include "cxd56_sysctl.h"

#ifdef CONFIG_CXD56_SYSCTL_TIMEOUT
#  define SYSCTL_TIMEOUT CONFIG_CXD56_SYSCTL_TIMEOUT
#else
#  define SYSCTL_TIMEOUT 5000
#endif

static int sysctl_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static sem_t g_exc;
static sem_t g_sync;
static int g_errcode = 0;

static const struct file_operations g_sysctlfops =
{
  .ioctl = sysctl_ioctl,
};

static int sysctl_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      default:
        {
          _err("cmd %x(%x)\n", cmd, arg);
          ret = cxd56_sysctlcmd(cmd & 0xff, arg);
          if (ret)
            {
              set_errno(ret);
            }
        }
    }

  return ret;
}

static void sysctl_semtake(sem_t *semid)
{
  while (sem_wait(semid) != 0)
    {
      ASSERT(errno == EINTR);
    }
}

static void sysctl_semgive(sem_t *semid)
{
  sem_post(semid);
}

static int sysctl_rxhandler(int cpuid, int protoid,
                            uint32_t pdata, uint32_t data,
                            FAR void *userdata)
{
  DEBUGASSERT(cpuid == 0);
  DEBUGASSERT(protoid == CXD56_PROTO_SYSCTL);

  g_errcode = (int)data;

  sysctl_semgive(&g_sync);

  return OK;
}

int cxd56_sysctlcmd(uint8_t id, uint32_t data)
{
  iccmsg_t msg;
  int ret;

  /* Get exclusive access */

  sysctl_semtake(&g_exc);

  msg.cpuid = 0;
  msg.msgid = id;
  msg.data  = data;

  /* Send any message to system CPU */

  ret = cxd56_iccsend(CXD56_PROTO_SYSCTL, &msg, SYSCTL_TIMEOUT);
  if (ret < 0)
    {
      _err("Timeout.\n");
      return ret;
    }

  /* Wait for reply message from system CPU */

  sysctl_semtake(&g_sync);

  /* Get the error code returned from system cpu */

  ret = g_errcode;

  sysctl_semgive(&g_exc);

  return ret;
}

void cxd56_sysctlinitialize(void)
{
  cxd56_iccinit(CXD56_PROTO_SYSCTL);

  sem_init(&g_exc, 0, 1);
  sem_init(&g_sync, 0, 0);

  cxd56_iccregisterhandler(CXD56_PROTO_SYSCTL, sysctl_rxhandler, NULL);

  (void)register_driver("/dev/sysctl", &g_sysctlfops, 0666, NULL);
}

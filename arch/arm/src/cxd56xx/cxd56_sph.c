/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_sph.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include "arm_arch.h"
#include "chip.h"

#include "hardware/cxd56_sph.h"
#include "cxd56_sph.h"

#ifdef CONFIG_CXD56_SPH

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_ID (CXD56_CPU_BASE + 0x40)

#define NR_HSEMS 16

#define sph_state_unlocked(sts) (STS_STATE(sts) == STATE_IDLE)
#define sph_state_locked(sts)   (STS_STATE(sts) == STATE_LOCKED)
#define sph_state_busy(sts)     (STS_STATE(sts) == STATE_LOCKEDANDRESERVED)

#ifdef CONFIG_CXD56_SPH_DEBUG
#  define hsinfo(fmt, ...)  _info(fmt, ## __VA_ARGS__)
#else
#  define hsinfo(fmt, ...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sph_dev_s
{
  int id;
  sem_t wait;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sph_open(FAR struct file *filep);
static int sph_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int sph_semtake(sem_t *id);
static void sph_semgive(sem_t *id);
static int sph_lock(FAR struct sph_dev_s *priv);
static int sph_trylock(FAR struct sph_dev_s *priv);
static inline int sph_unlock(FAR struct sph_dev_s *priv);
static int cxd56_sphirqhandler(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations sph_fops =
{
  .open  = sph_open,
  .close = 0,
  .read  = 0,
  .write = 0,
  .seek  = 0,
  .ioctl = sph_ioctl,
};

static struct sph_dev_s g_sphdev[NR_HSEMS];
static int g_cpuid;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sph_open(FAR struct file *filep)
{
  /* Exclusive access */

  if (filep->f_inode->i_crefs > 1)
    {
      return ERROR;
    }

  return OK;
}

static int sph_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct sph_dev_s *priv =
    (FAR struct sph_dev_s *)filep->f_inode->i_private;
  int ret = -ENOTTY;

  hsinfo("cmd = %x\n", cmd);

  if (!_HSIOCVALID(cmd))
    {
      return ret;
    }

  switch (cmd)
    {
      case HSLOCK:
        ret = sph_lock(priv);
        break;

      case HSUNLOCK:
        ret = sph_unlock(priv);
        break;

      case HSTRYLOCK:
        ret = sph_trylock(priv);
        break;

      default:
        break;
    }

  return ret;
}

static int sph_semtake(sem_t *id)
{
  return nxsem_wait_uninterruptible(id);
}

static void sph_semgive(sem_t *id)
{
  nxsem_post(id);
}

static int sph_lock(FAR struct sph_dev_s *priv)
{
  uint32_t sts;
  int ret;

  ret = sph_trylock(priv);
  if (ret == OK)
    {
      return OK;
    }

  for (; ; )
    {
      putreg32(REQ_RESERVE, CXD56_SPH_REQ(priv->id));
      hsinfo("hsem%d is locked.\n", priv->id);

      sts = getreg32(CXD56_SPH_STS(priv->id));
      if (sph_state_busy(sts) && RESV_OWNER(sts) == g_cpuid)
        {
          /* If successfully reserved, wait for semaphore unlocked. */

          sts = getreg32(CXD56_SPH_STS(priv->id));
          if (sph_state_busy(sts))
            {
              sph_semtake(&priv->wait);
            }

          /* Get latest status for determining locked owner. */

          sts = getreg32(CXD56_SPH_STS(priv->id));
        }

      /* Confirm locked CPU is me. */

      if (sph_state_locked(sts) && LOCK_OWNER(sts) == g_cpuid)
        {
          break;
        }
    }

  return OK;
}

static int sph_trylock(FAR struct sph_dev_s *priv)
{
  uint32_t sts;

  sts = getreg32(CXD56_SPH_STS(priv->id));
  if (sph_state_unlocked(sts))
    {
      putreg32(REQ_LOCK, CXD56_SPH_REQ(priv->id));
      hsinfo("hsem%d is locked.\n", priv->id);

      sts = getreg32(CXD56_SPH_STS(priv->id));
      if (sph_state_locked(sts) && LOCK_OWNER(sts) == g_cpuid)
        {
          return OK;
        }
    }

  return -EBUSY;
}

static inline int sph_unlock(FAR struct sph_dev_s *priv)
{
  putreg32(REQ_UNLOCK, CXD56_SPH_REQ(priv->id));
  hsinfo("hsem%d is unlocked.\n", priv->id);
  return OK;
}

static inline int cxd56_sphdevinit(FAR const char *devname, int num)
{
  FAR struct sph_dev_s *priv = &g_sphdev[num];
  char fullpath[64];
  int ret;

  snprintf(fullpath, sizeof(fullpath), "/dev/%s%d", devname, num);

  ret = register_driver(fullpath, &sph_fops, 0666, (FAR void *)priv);
  if (ret != 0)
    {
      return ERROR;
    }

  nxsem_init(&priv->wait, 0, 0);
  nxsem_set_protocol(&priv->wait, SEM_PRIO_NONE);
  priv->id = num;

  irq_attach(CXD56_IRQ_SPH0 + num, cxd56_sphirqhandler, NULL);
  up_enable_irq(CXD56_IRQ_SPH0 + num);

  return OK;
}

static int cxd56_sphirqhandler(int irq, FAR void *context, FAR void *arg)
{
  int id;

  /* Calculate hardware semaphore ID from IRQ number */

  id = irq - CXD56_IRQ_SPH0;

  DEBUGASSERT(id >= 0 && id <= 16);

  /* Clear interrupt */

  putreg32(REQ_INTRCLR, CXD56_SPH_REQ(id));

  /* Give semaphore for hardware semaphore is locked */

  sph_semgive(&g_sphdev[id].wait);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cxd56_sphinitialize(FAR const char *devname)
{
  int ret;
  int i;

  /* No. 0-2 and (14)-15 semaphores are reserved by other system. */

#ifdef CONFIG_CXD56_TESTSET
  for (i = 3; i < 14; i++)
#else
  for (i = 3; i < 15; i++)
#endif
    {
      ret = cxd56_sphdevinit(devname, i);
      if (ret != OK)
        {
          return ERROR;
        }
    }

  g_cpuid = getreg32(CPU_ID);

  return OK;
}

#endif

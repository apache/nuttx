/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_ge2d.c
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

#include <queue.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include "arm_arch.h"
#include "chip.h"
#include "cxd56_clock.h"

#include "hardware/cxd56_ge2d.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ge2d_open(FAR struct file *filep);
static int ge2d_close(FAR struct file *filep);
static ssize_t ge2d_read(FAR struct file *filep, FAR char *buffer,
                         size_t len);
static ssize_t ge2d_write(FAR struct file *filep, FAR const char *buffer,
                          size_t len);
static int ge2d_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int ge2d_semtake(sem_t *id);
static void ge2d_semgive(sem_t *id);
static int ge2d_irqhandler(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ge2dfops =
{
  .open  = ge2d_open,
  .close = ge2d_close,
  .read  = ge2d_read,
  .write = ge2d_write,
  .seek  = 0,
  .ioctl = ge2d_ioctl,
};

static sem_t g_wait;
static sem_t g_lock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ge2d_semtake
 ****************************************************************************/

static int ge2d_semtake(sem_t *id)
{
  return nxsem_wait_uninterruptible(id);
}

/****************************************************************************
 * Name: ge2d_semgive
 ****************************************************************************/

static void ge2d_semgive(sem_t *id)
{
  nxsem_post(id);
}

/****************************************************************************
 * Name: ge2d_open
 ****************************************************************************/

static int ge2d_open(FAR struct file *filep)
{
  return 0;
}

/****************************************************************************
 * Name: ge2d_close
 ****************************************************************************/

static int ge2d_close(FAR struct file *filep)
{
  return 0;
}

/****************************************************************************
 * Name: ge2d_read
 ****************************************************************************/

static ssize_t ge2d_read(FAR struct file *filep,
                         FAR char *buffer,
                         size_t len)
{
  return 0;
}

/****************************************************************************
 * Name: ge2d_write
 ****************************************************************************/

static ssize_t ge2d_write(FAR struct file *filep,
                          FAR const char *buffer,
                          size_t len)
{
  uint32_t bits;

  /* GE2D wants 16 byte aligned address for operation buffer. */

  if (((uintptr_t)buffer & 0xf) != 0)
    {
      set_errno(EINVAL);
      return 0;
    }

  /* Get exclusive access */

  ge2d_semtake(&g_lock);

  /* Set operation buffer and start processing.
   * Descriptor start address bit 0 is select to bus, always 1 (memory),
   * can't set except 1 in this chip.
   */

  putreg32((uint32_t)(uintptr_t)buffer | 1, GE2D_ADDRESS_DESCRIPTOR_START);
  putreg32(GE2D_EXEC, GE2D_CMD_DESCRIPTOR);

  /* Enable error and completion interrupts. */

  bits = GE2D_INTR_WR_ERR |
         GE2D_INTR_RD_ERR |
         GE2D_INTR_NDE |
         GE2D_INTR_DSD |
         GE2D_INTR_NDF;

  putreg32(bits, GE2D_INTR_ENABLE);

  /* Wait for interrupts for processing done. */

  ge2d_semtake(&g_wait);

  /* Disable interrupts */

  putreg32(0, GE2D_INTR_ENABLE);

  ge2d_semgive(&g_lock);

  return len;
}

/****************************************************************************
 * Name: ge2d_ioctl
 ****************************************************************************/

static int ge2d_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = -ENOTTY;

  /* TODO: Should be implement features:
   *
   * - stop execution
   * - debug for raster operation
   */

  switch (cmd)
    {
      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: ge2d_irqhandler
 ****************************************************************************/

static int ge2d_irqhandler(int irq, FAR void *context, FAR void *arg)
{
  uint32_t stat;

  /* Clear interrupts */

  stat = getreg32(GE2D_INTR_STAT);
  putreg32(stat, GE2D_INTR_STAT);

  /* TODO: output status to syslog */

  /* Release semaphore anyway */

  ge2d_semgive(&g_wait);

  return OK;
}

/****************************************************************************
 * Name: cxd56_ge2dinitialize
 ****************************************************************************/

int cxd56_ge2dinitialize(FAR const char *devname)
{
  int ret;

  nxsem_init(&g_lock, 0, 1);
  nxsem_init(&g_wait, 0, 0);
  nxsem_set_protocol(&g_wait, SEM_PRIO_NONE);

  ret = register_driver(devname, &g_ge2dfops, 0666, NULL);
  if (ret != 0)
    {
      return ERROR;
    }

  cxd56_img_ge2d_clock_enable();

  /* Disable interrupts */

  putreg32(0, GE2D_INTR_ENABLE);

  irq_attach(CXD56_IRQ_GE2D, ge2d_irqhandler, NULL);
  up_enable_irq(CXD56_IRQ_GE2D);

  return OK;
}

/****************************************************************************
 * Name: cxd56_ge2duninitialize
 ****************************************************************************/

void cxd56_ge2duninitialize(FAR const char *devname)
{
  up_disable_irq(CXD56_IRQ_GE2D);
  irq_detach(CXD56_IRQ_GE2D);

  cxd56_img_ge2d_clock_disable();

  nxsem_destroy(&g_lock);
  nxsem_destroy(&g_wait);

  unregister_driver(devname);
}

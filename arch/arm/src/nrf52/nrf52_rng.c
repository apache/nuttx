/****************************************************************************
 * arch/arm/src/nrf52/nrf52_rng.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/drivers/drivers.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/nrf52_utils.h"
#include "hardware/nrf52_rng.h"

#if defined(CONFIG_NRF52_RNG)
#if defined(CONFIG_DEV_RANDOM) || defined(CONFIG_DEV_URANDOM_ARCH)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nrf52_rng_initialize(void);
static int nrf52_rng_irqhandler(int irq, void *context, void *arg);
static ssize_t nrf52_rng_read(struct file *filep, char *buffer,
                              size_t buflen);
static int nrf52_rng_open(struct file *filep);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng_dev_s
{
  uint8_t *rd_buf;
  size_t   rd_count;
  size_t   buflen;
  sem_t    rd_sem;         /* semaphore for read RNG data */
  mutex_t  lock;           /* mutex for access RNG dev */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rng_dev_s g_rngdev;

static const struct file_operations g_rngops =
{
  .open  = nrf52_rng_open,       /* open */
  .read  = nrf52_rng_read,       /* read */
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

static void nrf52_rng_start(void)
{
  irqstate_t flag;
  flag = enter_critical_section();

  nrf52_event_clear(NRF52_RNG_EVENTS_RDY);

  putreg32(1, NRF52_RNG_CONFIG);
  nrf52_interrupt_enable(NRF52_RNG_INTSET, RNG_INT_RDY);
  nrf52_task_trigger(NRF52_RNG_TASKS_START);

  up_enable_irq(NRF52_IRQ_RNG);

  leave_critical_section(flag);
}

static void nrf52_rng_stop(void)
{
  irqstate_t flag;
  flag = enter_critical_section();

  up_disable_irq(NRF52_IRQ_RNG);

  nrf52_task_trigger(NRF52_RNG_TASKS_STOP);
  nrf52_interrupt_disable(NRF52_RNG_INTCLR, RNG_INT_RDY);

  nrf52_event_clear(NRF52_RNG_EVENTS_RDY);

  leave_critical_section(flag);
}

static int nrf52_rng_initialize(void)
{
  static bool first_flag = true;

  if (false == first_flag)
    return OK;

  first_flag = false;

  _info("Initializing RNG\n");

  memset(&g_rngdev, 0, sizeof(struct rng_dev_s));

  nxsem_init(&g_rngdev.rd_sem, 0, 0);
  nxmutex_init(&g_rngdev.lock);

  _info("Ready to stop\n");
  nrf52_rng_stop();

  if (irq_attach(NRF52_IRQ_RNG, nrf52_rng_irqhandler, NULL) != 0)
    {
      /* We could not attach the ISR to the interrupt */

      _warn("Could not attach NRF52_IRQ_RNG.\n");

      return -EAGAIN;
    }

  return OK;
}

static int nrf52_rng_irqhandler(int irq, void *context, void *arg)
{
  struct rng_dev_s *priv = (struct rng_dev_s *) &g_rngdev;
  uint8_t *addr;

  if (getreg32(NRF52_RNG_EVENTS_RDY) == RNG_INT_RDY)
    {
      nrf52_event_clear(NRF52_RNG_EVENTS_RDY);
      if (priv->rd_count < priv->buflen)
        {
          addr = priv->rd_buf + priv->rd_count++;
          *addr = getreg32(NRF52_RNG_VALUE);
          irqwarn("%d\n", *addr);
        }

      if (priv->rd_count == priv->buflen)
        {
          nrf52_rng_stop();
          nxsem_post(&priv->rd_sem);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_rng_open
 ****************************************************************************/

static int nrf52_rng_open(struct file *filep)
{
  /* O_NONBLOCK is not supported */

  if (filep->f_oflags & O_NONBLOCK)
    {
      _err("nRF52 rng didn't support O_NONBLOCK mode.\n");
      return -EPERM;
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_rng_read
 ****************************************************************************/

static ssize_t nrf52_rng_read(struct file *filep, char *buffer,
                              size_t buflen)
{
  struct rng_dev_s *priv = (struct rng_dev_s *)&g_rngdev;
  ssize_t read_len;

  if (nxmutex_lock(&priv->lock) != OK)
    {
      return -EBUSY;
    }

  priv->rd_buf = (uint8_t *) buffer;
  priv->buflen = buflen;
  priv->rd_count = 0;

  /* start RNG and Wait until the buffer is filled */

  nrf52_rng_start();

  nxsem_wait(&priv->rd_sem);
  read_len = priv->rd_count;

  if (priv->rd_count > priv->buflen)
    {
      _err("Bad rd_count: Too much data, exceeds buffer size: %d\n",
           priv->rd_count);
    }

  /* Now , got data, and release rd_sem for next read */

  nxmutex_unlock(&priv->lock);
  return read_len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Initialize the RNG hardware and register the /dev/random driver.
 *   Must be called BEFORE devurandom_register.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_RANDOM
void devrandom_register(void)
{
  nrf52_rng_initialize();
  register_driver("/dev/random", &g_rngops, 0444, NULL);
}
#endif

/****************************************************************************
 * Name: devurandom_register
 *
 * Description:
 *   Register /dev/urandom
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_URANDOM_ARCH
void devurandom_register(void)
{
#ifndef CONFIG_DEV_RANDOM
  nrf52_rng_initialize();
#endif
  register_driver("dev/urandom", &g_rngops, 0444, NULL);
}
#endif

#endif /* CONFIG_DEV_RANDOM || CONFIG_DEV_URANDOM_ARCH */
#endif /* CONFIG_NRF52_RNG */

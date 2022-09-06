/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_rng.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>

#include "hardware/stm32_rng.h"
#include "arm_internal.h"

#if defined(CONFIG_STM32F0L0G0_RNG)
#if defined(CONFIG_DEV_RANDOM) || defined(CONFIG_DEV_URANDOM_ARCH)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_rng_initialize(void);
static int stm32_rng_interrupt(int irq, void *context, void *arg);
static void stm32_rng_enable(void);
static void stm32_rng_disable(void);
static ssize_t stm32_rng_read(struct file *filep, char *buffer, size_t);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng_dev_s
{
  mutex_t rd_devlock;   /* Threads can only exclusively access the RNG */
  sem_t rd_readsem;     /* To block until the buffer is filled  */
  char *rd_buf;
  size_t rd_buflen;
  uint32_t rd_lastval;
  bool rd_first;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rng_dev_s g_rngdev;

static const struct file_operations g_rngops =
{
  NULL,            /* open */
  NULL,            /* close */
  stm32_rng_read,  /* read */
  NULL,            /* write */
  NULL,            /* seek */
  NULL,            /* ioctl */
  NULL             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rng_initialize
 ****************************************************************************/

static int stm32_rng_initialize(void)
{
  uint32_t regval;

  _info("Initializing RNG\n");

  memset(&g_rngdev, 0, sizeof(struct rng_dev_s));
  nxmutex_init(&g_rngdev.rd_devlock);

  if (irq_attach(STM32_IRQ_RNG, stm32_rng_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      _info("Could not attach IRQ.\n");

      return -EAGAIN;
    }

  /* Enable interrupts */

  regval = getreg32(STM32_RNG_CR);
  regval |=  RNG_CR_IE;
  putreg32(regval, STM32_RNG_CR);

  up_enable_irq(STM32_IRQ_RNG);

  return OK;
}

/****************************************************************************
 * Name: stm32_rng_enable
 ****************************************************************************/

static void stm32_rng_enable(void)
{
  uint32_t regval;

  g_rngdev.rd_first = true;

  regval = getreg32(STM32_RNG_CR);
  regval |= RNG_CR_RNGEN;
  putreg32(regval, STM32_RNG_CR);
}

/****************************************************************************
 * Name: stm32_rng_disable
 ****************************************************************************/

static void stm32_rng_disable(void)
{
  uint32_t regval;
  regval = getreg32(STM32_RNG_CR);
  regval &= ~RNG_CR_RNGEN;
  putreg32(regval, STM32_RNG_CR);
}

/****************************************************************************
 * Name: stm32_rng_interrupt
 ****************************************************************************/

static int stm32_rng_interrupt(int irq, void *context, void *arg)
{
  uint32_t rngsr;
  uint32_t data;

  rngsr = getreg32(STM32_RNG_SR);

  if ((rngsr & (RNG_SR_SEIS | RNG_SR_CEIS)) /* Check for error bits */
                 || !(rngsr & RNG_SR_DRDY)) /* Data ready must be set */
    {
      /* This random value is not valid, we will try again. */

      return OK;
    }

  data = getreg32(STM32_RNG_DR);

  /* As required by the FIPS PUB (Federal Information Processing Standard
   * Publication) 140-2, the first random number generated after setting the
   * RNGEN bit should not be used, but saved for comparison with the next
   * generated random number. Each subsequent generated random number has to
   * be compared with the previously generated number. The test fails if any
   * two compared numbers are equal
   * (continuous random number generator test).
   */

  if (g_rngdev.rd_first)
    {
      g_rngdev.rd_first = false;
      g_rngdev.rd_lastval = data;
      return OK;
    }

  if (g_rngdev.rd_lastval == data)
    {
      /* Two subsequent same numbers, we will try again. */

      return OK;
    }

  /* If we get here, the random number is valid. */

  g_rngdev.rd_lastval = data;

  if (g_rngdev.rd_buflen >= 4)
    {
      g_rngdev.rd_buflen -= 4;
      *(uint32_t *)&g_rngdev.rd_buf[g_rngdev.rd_buflen] = data;
    }
  else
    {
      while (g_rngdev.rd_buflen > 0)
        {
          g_rngdev.rd_buf[--g_rngdev.rd_buflen] = (char)data;
          data >>= 8;
        }
    }

  if (g_rngdev.rd_buflen == 0)
    {
      /* Buffer filled, stop further interrupts. */

      stm32_rng_disable();
      nxsem_post(&g_rngdev.rd_readsem);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_rng_read
 ****************************************************************************/

static ssize_t stm32_rng_read(struct file *filep,
                              char *buffer, size_t buflen)
{
  int ret;

  ret = nxmutex_lock(&g_rngdev.rd_devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* We've got the semaphore. */

  /* Initialize the operation semaphore with 0 for blocking until the
   * buffer is filled from interrupts.  The readsem semaphore is used
   * for signaling and, hence, should not have priority inheritance
   * enabled.
   */

  nxsem_init(&g_rngdev.rd_readsem, 0, 0);
  nxsem_set_protocol(&g_rngdev.rd_readsem, SEM_PRIO_NONE);

  g_rngdev.rd_buflen = buflen;
  g_rngdev.rd_buf = buffer;

  /* Enable RNG with interrupts */

  stm32_rng_enable();

  /* Wait until the buffer is filled */

  ret = nxsem_wait(&g_rngdev.rd_readsem);

  /* Free RNG for next use */

  nxmutex_unlock(&g_rngdev.rd_devlock);
  return ret < 0 ? ret : buflen;
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
  stm32_rng_initialize();
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
  stm32_rng_initialize();
#endif
  register_driver("/dev/urandom", &g_rngops, 0444, NULL);
}
#endif

#endif /* CONFIG_DEV_RANDOM || CONFIG_DEV_URANDOM_ARCH */
#endif /* CONFIG_STM32F0L0G0_RNG */

/****************************************************************************
 * arch/arm/src/sama5/sam_trng.c
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/semaphore.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "sam_periphclks.h"
#include "sam_trng.h"

#if defined(CONFIG_SAMA5_TRNG)
#if defined(CONFIG_DEV_RANDOM) || defined(CONFIG_DEV_URANDOM_ARCH)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupts */

static int sam_interrupt(int irq, void *context, FAR void *arg);

/* Character driver methods */

static ssize_t sam_read(struct file *filep, char *buffer, size_t);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct trng_dev_s
{
  sem_t exclsem;            /* Enforces exclusive access to the TRNG */
  sem_t waitsem;            /* Wait for buffer full  */
  uint32_t *samples;        /* Current buffer being filled */
  size_t maxsamples;        /* Size of the current buffer (in 32-bit words) */
  volatile size_t nsamples; /* Number of samples currently buffered */
  volatile bool first;      /* The first random number must be handled differently */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct trng_dev_s g_trngdev;

static const struct file_operations g_trngops =
{
  NULL,            /* open */
  NULL,            /* close */
  sam_read,        /* read */
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
 * Name: sam_interrupt
 *
 * Description:
 *   The TRNG interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 *
 ****************************************************************************/

static int sam_interrupt(int irq, void *context, FAR void *arg)
{
  uint32_t odata;

  /* Loop where there are samples available to be read and/or until the user
   * buffer is filled.  Each sample requires only 84 clocks it is likely
   * that we will loop here.
   */

  for (; ; )
    {
      /* Read the random sample (before checking DATRDY -- but probably not
       * necessary)
       */

      odata = getreg32(SAM_TRNG_ODATA);

      /* Verify that sample data is available (DATARDY is cleared when the
       * interrupt status register is read)
       */

      if ((getreg32(SAM_TRNG_ISR) & TRNG_INT_DATRDY) == 0)
        {
          /* No?  Then return and continue processing on the next
           * interrupt.
           */

          return OK;
        }

      /* As required by the FIPS PUB (Federal Information Processing Standard
       * Publication) 140-2, the first random number generated after setting
       * the RNGEN bit should not be used, but saved for comparison with the
       * next generated random number. Each subsequent generated random
       * number has to be compared with the previously generated number. The
       * test fails if any two compared numbers are equal (continuous random
       * number generator test).
       */

      if (g_trngdev.nsamples == 0)
        {
          /* This is the first sample we have taken.  Save it for subsequent
           * comparison.
           */

          g_trngdev.samples[0] = odata;
          g_trngdev.nsamples   = 1;
          continue;
        }

      /* This is not the first sample.  Check if the new sample differs from
       * the preceding sample.
       */

      else if (odata == g_trngdev.samples[g_trngdev.nsamples - 1])
        {
          /* Two samples with the same value.  Discard this one and try
           * again.
           */

          continue;
        }

      /* This sample differs from the previous value.  Have we discarded the
       * first sample yet?
       */

      if (g_trngdev.first)
        {
          /* No, discard it now by replacing it with the new sample */

          g_trngdev.samples[0] = odata;
          g_trngdev.nsamples   = 1;
          g_trngdev.first      = false;
        }

      /* Yes.. the first sample has been discarded */

      else
        {
          /* Add the new random number to the buffer */

          g_trngdev.samples[g_trngdev.nsamples] = odata;
          g_trngdev.nsamples++;
        }

      /* Have all of the requested samples been saved? */

      if (g_trngdev.nsamples == g_trngdev.maxsamples)
        {
          /* Yes.. disable any further interrupts */

          putreg32(TRNG_INT_DATRDY, SAM_TRNG_IDR);

          /* Disable the TRNG */

          putreg32(TRNG_CR_DISABLE | TRNG_CR_KEY, SAM_TRNG_CR);

          /* And wakeup the waiting read thread. */

          nxsem_post(&g_trngdev.waitsem);
          return OK;
        }
    }
}

/****************************************************************************
 * Name: sam_read
 *
 * Description:
 *   This is the standard, NuttX character driver read method
 *
 * Input Parameters:
 *   filep - The VFS file instance
 *   buffer - Buffer in which to return the random samples
 *   buflen - The length of the buffer
 *
 * Returned Value:
 *
 ****************************************************************************/

static ssize_t sam_read(struct file *filep, char *buffer, size_t buflen)
{
  ssize_t retval;
  int ret;

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Get exclusive access to the TRNG hardware */

  ret = nxsem_wait(&g_trngdev.exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Save the buffer information. */

  DEBUGASSERT(((uintptr_t)buffer & 3) == 0);

  g_trngdev.samples    = (uint32_t *)buffer;
  g_trngdev.maxsamples = buflen >> 2;
  g_trngdev.nsamples   = 0;
  g_trngdev.first      = true;

  /* Enable the TRNG */

  putreg32(TRNG_CR_ENABLE | TRNG_CR_KEY, SAM_TRNG_CR);

  /* Clear any pending TRNG interrupts by reading the interrupt status
   * register
   */

  getreg32(SAM_TRNG_ISR);

  /* Enable TRNG interrupts */

  putreg32(TRNG_INT_DATRDY, SAM_TRNG_IER);

  /* Wait until the buffer is filled */

  while (g_trngdev.nsamples < g_trngdev.maxsamples)
    {
      ret = nxsem_wait(&g_trngdev.waitsem);

      finfo("Awakened: nsamples=%d maxsamples=%d ret=%d\n",
            g_trngdev.nsamples, g_trngdev.maxsamples, ret);

      if (ret < 0)
        {
          /* We must have been awakened by a signal */

          if (g_trngdev.nsamples > 0)
            {
              break;
            }
          else
            {
              retval = ret;
              goto errout;
            }
        }
    }

  /* Success... calculate the number of bytes to return */

  retval = g_trngdev.nsamples << 2;

errout:

  /* Disable TRNG interrupts */

  putreg32(TRNG_INT_DATRDY, SAM_TRNG_IDR);

  /* Disable the TRNG */

  putreg32(TRNG_CR_DISABLE | TRNG_CR_KEY, SAM_TRNG_CR);

  /* Release our lock on the TRNG hardware */

  nxsem_post(&g_trngdev.exclsem);

  finfo("Return %d\n", (int)retval);
  return retval;
}

/****************************************************************************
 * Name: sam_rng_initialize
 *
 * Description:
 *   Initialize the TRNG hardware.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sam_rng_initialize(void)
{
  int ret;

  finfo("Initializing TRNG hardware\n");

  /* Initialize the device structure */

  memset(&g_trngdev, 0, sizeof(struct trng_dev_s));

  /* Initialize semphores */

  nxsem_init(&g_trngdev.exclsem, 0, 1);
  nxsem_init(&g_trngdev.waitsem, 0, 0);

  /* The waitsem semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_set_protocol(&g_trngdev.waitsem, SEM_PRIO_NONE);

  /* Enable clocking to the TRNG */

  sam_trng_enableclk();

  /* Initialize the TRNG interrupt */

  ret = irq_attach(SAM_IRQ_TRNG, sam_interrupt, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to attach to IRQ%d\n", SAM_IRQ_TRNG);
      return ret;
    }

  /* Disable the interrupts at the TRNG */

  putreg32(TRNG_INT_DATRDY, SAM_TRNG_IDR);

  /* Disable the TRNG */

  putreg32(TRNG_CR_DISABLE | TRNG_CR_KEY, SAM_TRNG_CR);

  /* Enable the TRNG interrupt at the AIC */

  up_enable_irq(SAM_IRQ_TRNG);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Initialize the TRNG hardware and register the /dev/random driver.
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
  int ret;

  ret = sam_rng_initialize();
  if (ret >= 0)
    {
      ret = register_driver("/dev/random", &g_trngops, 0444, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to register /dev/random\n");
        }
    }
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
  int ret;

#ifndef CONFIG_DEV_RANDOM
  ret = sam_rng_initialize();
  if (ret >= 0)
#endif
    {
      ret = register_driver("/dev/urandom", &g_trngops, 0444, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to register /dev/urandom\n");
        }
    }
}
#endif

#endif /* CONFIG_DEV_RANDOM || CONFIG_DEV_URANDOM_ARCH */
#endif /* CONFIG_SAMA5_TRNG */

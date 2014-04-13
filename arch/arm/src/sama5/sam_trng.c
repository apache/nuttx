/****************************************************************************
 * arch/arm/src/sama5/sam_trng.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives, in part, from Max Holtzberg's STM32 RNG Nuttx driver:
 *
 *   Copyright (C) 2012 Max Holtzberg. All rights reserved.
 *   Author: Max Holtzberg <mh@uvc.de>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "up_arch.h"
#include "up_internal.h"

#include "sam_periphclks.h"
#include "sam_trng.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupts */

static int sam_interrupt(int irq, void *context);

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
  0,               /* open */
  0,               /* close */
  sam_read,        /* read */
  0,               /* write */
  0,               /* seek */
  0                /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  ,0               /* poll */
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

static int sam_interrupt(int irq, void *context)
{
  uint32_t odata;

  /* Loop where there are samples available to be read and/or until the user
   * buffer is filled.  Each sample requires only 84 clocks it is likely
   * that we will loop here.
   */

  for (;;)
    {
      /* Read the random sample (before checking DATRDY -- but probably not
       * necessary)
       */

      odata = getreg32(SAM_TRNG_ODATA);

      /* Verify that sample data is available (DATARDY is cleared when the
       * interrupt status regiser is read)
       */

      if ((getreg32(SAM_TRNG_ISR) & TRNG_INT_DATRDY) == 0)
        {
           /* No?  Then return and continue processing on the next interrupt. */

           return OK;
        }

      /* As required by the FIPS PUB (Federal Information Processing Standard
       * Publication) 140-2, the first random number generated after setting
       * the RNGEN bit should not be used, but saved for comparison with the
       * next generated random number. Each subsequent generated random number
       * has to be compared with the previously generated number. The test
       * fails if any two compared numbers are equal (continuous random number
       * generator test).
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
          /* Two samples with the same value.  Discard this one and try again. */

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

      /* Yes.. the first sample has been dicarded */

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

          sem_post(&g_trngdev.waitsem);
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

  fvdbg("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Get exclusive access to the TRNG harware */

  if (sem_wait(&g_trngdev.exclsem) != OK)
    {
      /* This is probably -EINTR meaning that we were awakened by a signal */

      return -errno;
    }

  /* Save the buffer information. */

  DEBUGASSERT(((uintptr_t)buffer & 3) == 0);

  g_trngdev.samples    = (uint32_t*)buffer;
  g_trngdev.maxsamples = buflen >> 2;
  g_trngdev.nsamples   = 0;
  g_trngdev.first      = true;

  /* Enable the TRNG */

  putreg32(TRNG_CR_ENABLE | TRNG_CR_KEY, SAM_TRNG_CR);

  /* Clear any pending TRNG interrupts by reading the interrupt status
   * register
   */

  (void)getreg32(SAM_TRNG_ISR);

  /* Enable TRNG interrupts */

  putreg32(TRNG_INT_DATRDY, SAM_TRNG_IER);

  /* Wait until the buffer is filled */

  while (g_trngdev.nsamples < g_trngdev.maxsamples)
    {
      ret = sem_wait(&g_trngdev.waitsem);

      fvdbg("Awakened: nsamples=%d maxsamples=%d ret=%d\n",
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
              retval = -errno;
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

  sem_post(&g_trngdev.exclsem);

  fvdbg("Return %d\n", (int)retval);
  return retval;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rnginitialize
 *
 * Description:
 *   Initialize the TRNG hardware and register the /dev/randome driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_rnginitialize(void)
{
  int ret;

  fvdbg("Initializing TRNG hardware\n");

  /* Initialize the device structure */

  memset(&g_trngdev, 0, sizeof(struct trng_dev_s));
  sem_init(&g_trngdev.exclsem, 0, 1);
  sem_init(&g_trngdev.waitsem, 0, 0);

  /* Enable clocking to the TRNG */

  sam_trng_enableclk();

  /* Initialize the TRNG interrupt */

  if (irq_attach(SAM_IRQ_TRNG, sam_interrupt))
    {
      fdbg("ERROR: Failed to attach to IRQ%d\n", SAM_IRQ_TRNG);
      return;
    }

  /* Disable the interrupts at the TRNG */

  putreg32(TRNG_INT_DATRDY, SAM_TRNG_IDR);

  /* Disable the TRNG */

  putreg32(TRNG_CR_DISABLE | TRNG_CR_KEY, SAM_TRNG_CR);

  /* Register the character driver */

  ret = register_driver("/dev/random", &g_trngops, 0644, NULL);
  if (ret < 0)
    {
      fdbg("ERROR: Failed to register /dev/random\n");
      return;
    }

  /* Enable the TRNG interrupt at the AIC */

  up_enable_irq(SAM_IRQ_TRNG);
}

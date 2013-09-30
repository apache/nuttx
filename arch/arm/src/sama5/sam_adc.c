/************************************************************************************
 * arch/arm/src/sama5/sam_adc.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "chip/sam_adc.h"
#include "sam_adc.h"

#if defined(CONFIG_SAMA5_ADC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Get the set of channel interrupts to enable */

#define SAMA5_CHAN0_ENABLE    0
#define SAMA5_CHAN1_ENABLE    0
#define SAMA5_CHAN2_ENABLE    0
#define SAMA5_CHAN3_ENABLE    0
#define SAMA5_CHAN4_ENABLE    0
#define SAMA5_CHAN5_ENABLE    0
#define SAMA5_CHAN6_ENABLE    0
#define SAMA5_CHAN7_ENABLE    0
#define SAMA5_CHAN8_ENABLE    0
#define SAMA5_CHAN9_ENABLE    0
#define SAMA5_CHAN10_ENABLE   0
#define SAMA5_CHAN11_ENABLE   0

#if defined(CONFIG_SAMA5_ADC_CHAN0)
#  undef SAMA5_CHAN0_ENABLE
#  define SAMA5_CHAN0_ENABLE  ADC_INT_EOC0
#elif defined(CONFIG_SAMA5_ADC_CHAN1)
#  undef SAMA5_CHAN1_ENABLE
#  define SAMA5_CHAN1_ENABLE  ADC_INT_EOC1
#elif defined(CONFIG_SAMA5_ADC_CHAN2)
#  undef SAMA5_CHAN2_ENABLE
#  define SAMA5_CHAN2_ENABLE  ADC_INT_EOC2
#elif defined(CONFIG_SAMA5_ADC_CHAN3)
#  undef SAMA5_CHAN3_ENABLE
#  define SAMA5_CHAN3_ENABLE  ADC_INT_EOC3
#elif defined(CONFIG_SAMA5_ADC_CHAN4)
#  undef SAMA5_CHAN4_ENABLE
#  define SAMA5_CHAN4_ENABLE  ADC_INT_EOC4
#elif defined(CONFIG_SAMA5_ADC_CHAN5)
#  undef SAMA5_CHAN5_ENABLE
#  define SAMA5_CHAN5_ENABLE  ADC_INT_EOC5
#elif defined(CONFIG_SAMA5_ADC_CHAN6)
#  undef SAMA5_CHAN6_ENABLE
#  define SAMA5_CHAN6_ENABLE  ADC_INT_EOC6
#elif defined(CONFIG_SAMA5_ADC_CHAN7)
#  undef SAMA5_CHAN7_ENABLE
#  define SAMA5_CHAN7_ENABLE  ADC_INT_EOC7
#elif defined(CONFIG_SAMA5_ADC_CHAN8)
#  undef SAMA5_CHAN8_ENABLE
#  define SAMA5_CHAN8_ENABLE  ADC_INT_EOC8
#elif defined(CONFIG_SAMA5_ADC_CHAN9)
#  undef SAMA5_CHAN9_ENABLE
#  define SAMA5_CHAN9_ENABLE  ADC_INT_EOC9
#elif defined(CONFIG_SAMA5_ADC_CHAN10)
#  undef SAMA5_CHAN10_ENABLE
#  define SAMA5_CHAN10_ENABLE  ADC_INT_EOC10
#elif defined(CONFIG_SAMA5_ADC_CHAN11)
#  undef SAMA5_CHAN11_ENABLE
#  define SAMA5_CHAN11_ENABLE  ADC_INT_EOC11
#endif

#define SAMA5_CHAN_ENABLE \
  (SAMA5_CHAN0_ENABLE  || SAMA5_CHAN1_ENABLE  || SAMA5_CHAN2_ENABLE  || \
   SAMA5_CHAN3_ENABLE  || SAMA5_CHAN4_ENABLE  || SAMA5_CHAN5_ENABLE  || \
   SAMA5_CHAN6_ENABLE  || SAMA5_CHAN7_ENABLE  || SAMA5_CHAN8_ENABLE  || \
   SAMA5_CHAN9_ENABLE  || SAMA5_CHAN10_ENABLE || SAMA5_CHAN11_ENABLE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the overall state of the ADC */

struct sam_adc_s
{
#ifdef SAMA5_ADC_HAVE_CHANNELS
  struct adc_dev_s dev; /* The external via of the ADC device */
#endif

  /* Debug stuff */

#ifdef CONFIG_SAMA5_ADC_REGDEBUG
   bool wrlast;         /* Last was a write */
   uintptr_t addrlast;  /* Last address */
   uint32_t vallast;    /* Last value */
   int ntimes;          /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#if defined(CONFIG_SAMA5_ADC_REGDEBUG) && defined(CONFIG_DEBUG)
static bool sam_adc_checkreg(struct sam_adc_s *priv, bool wr,
                             uint32_t regval, uintptr_t address);
#endif

/* ADC interrupt handling */

#ifdef SAMA5_ADC_HAVE_CHANNELS
static void sam_adc_endconversion(struct sam_adc_s *priv, uint32_t pending);
#endif
static int  sam_adc_interrupt(int irq, void *context);

/* ADC methods */

#ifdef SAMA5_ADC_HAVE_CHANNELS
static void sam_adc_reset(struct adc_dev_s *dev);
static int  sam_adc_setup(struct adc_dev_s *dev);
static void sam_adc_shutdown(struct adc_dev_s *dev);
static void sam_adc_rxint(struct adc_dev_s *dev, bool enable);
static int  sam_adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef SAMA5_ADC_HAVE_CHANNELS
/* ADC lower half device operations */

static const struct adc_ops_s g_adcops =
{
  .ao_reset    = sam_adc_reset,
  .ao_setup    = sam_adc_setup,
  .ao_shutdown = sam_adc_shutdown,
  .ao_rxint    = sam_adc_rxint,
  .ao_ioctl    = sam_adc_ioctl,
};
#endif

/* ADC internal state */

static struct sam_adc_s g_adcpriv;

#ifdef SAMA5_ADC_HAVE_CHANNELS
/* ADC device instance */

static struct adc_dev_s g_adcdev =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv.dev,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_adc_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   regval  - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_REGDEBUG
static bool sam_adc_checkreg(struct sam_adc_s *priv, bool wr,
                             uint32_t regval, uintptr_t address)
{
  if (wr      == priv->wrlast &&   /* Same kind of access? */
      regval  == priv->vallast &&  /* Same value? */
      address == priv->addrlast)   /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          lldbg("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast   = wr;
      priv->vallast  = regval;
      priv->addrlast = address;
      priv->ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

#ifdef SAMA5_ADC_HAVE_CHANNELS

/****************************************************************************
 * Name: sam_adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before sam_adc_setup() and on error conditions.
 *
 ****************************************************************************/

static void sam_adc_reset(struct adc_dev_s *dev)
{
  struct sam_adc_s *priv = (struct sam_adc_s *)dev->ad_priv;
  irqstate_t flags;
  uint32_t regval;

  /* Reset the ADC controller */

  flags = irqsave();
  sam_adc_putreg(priv, SAM_ADC_CR, ADC_CR_SWRST);

  /* Reset Mode Register */

  sam_adc_putreg(priv, SAM_ADC_MR, 0);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.  Interrupts
 *   are all disabled upon return.
 *
 ****************************************************************************/

static int sam_adc_setup(struct adc_dev_s *dev)
{
  struct sam_adc_s *priv = (struct sam_adc_s *)dev->ad_priv;
  int ret;

  /* Attach the ADC interrupt */

  ret = irq_attach(SAM_IRQ_ADC, sam_adc_interrupt);
  if (ret < 0)
    {
      adbg("ERROR: Failed to attach IRQ %d: %d\n", SAM_IRQ_ADC, ret);
      return ret;
    }

  /* Enable the ADC interrupt */

  up_enable_irq(SAM_IRQ_ADC);
  return OK;
}

/****************************************************************************
 * Name: sam_adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void sam_adc_shutdown(struct adc_dev_s *dev)
{
  struct sam_adc_s *priv = (struct sam_adc_s *)dev->ad_priv;

  /* Disable ADC interrupts, both at the level of the ADC device and at the
   * level of the NVIC.
   */

  sam_adc_putreg32(priv, SAM_ADC_IDR, ADC_TSD_INTS);
  up_disable_irq(SAM_IRQ_ADC);

  /* Then detach the ADC interrupt handler. */

  irq_detach(SAM_IRQ_ADC);
}

/****************************************************************************
 * Name: sam_adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void sam_adc_rxint(struct adc_dev_s *dev, bool enable)
{
  struct sam_adc_s *priv = (struct sam_adc_s *)dev->ad_priv;

  /* Are we enabling or disabling? */

  if (enable)
    {
      /* Enable channel interrupts */

      sam_adc_putreg32(priv, SAM_ADC_IER, SAMA5_CHAN_ENABLE);
    }
  else
    {
      /* Disable channel interrupts */

      sam_adc_putreg32(priv, SAM_ADC_IDR, ADC_INT_EOCALL);
    }
}

/****************************************************************************
 * Name: sam_adc_ioctl
 *
 * Description:
 *  All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int sam_adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  /* No ioctl commands supported */

  return -ENOTTY;
}

/****************************************************************************
 * Name: sam_adc_endconversion
 *
 * Description:
 *   End of conversion interrupt handler
 *
 ****************************************************************************/

static void sam_adc_endconversion(struct sam_adc_s *priv, uint32_t pending)
{
  uint32_t regval;
  int chan;

  /* Check for the end of conversion event on each channel */

  for (chan = 0; chan < SAM_ADC_NCHANNELS && pending != 0; chan++)
    {
      uint32_t bit = ADC_INT_EOC(chan);
      if ((pending & bit) != 0)
        {
          /* Read the ADC sample and pass it to the upper half */

          regval   = sam_adc_getreg(priv, SAM_ADC_CDR(chan));
          ret      = adc_receive(&priv->dev, chan, regval & ADC_CDR_DATA_MASK);
          pending &= ~bit;
        }
    }
}

#endif /* SAMA5_ADC_HAVE_CHANNELS */

/****************************************************************************
 * Name: sam_adc_interrupt
 *
 * Description:
 *   ADC interrupt handler
 *
 ****************************************************************************/

static int sam_adc_interrupt(int irq, void *context)
{
  struct sam_adc_s *priv = (struct sam_adc_s *)g_adcdev.ad_priv;
  uint32_t regval;
  struct sam_adc_s *priv = &g_adcpriv;
  uint32_t isr;
  uint32_t imr;
  uint32_t pending;

  /* Get the set of unmasked, pending ADC interrupts */

  isr = sam_adc_getreg(priv, SAM_ADC_ISR);
  imr = sam_adc_getreg(priv, SAM_ADC_IMR);
  pending = isr & imr;

  /* Handle pending touchscreen interrupts */

#ifdef CONFIG_SAMA5_TOUCHSCREEN
  if ((pending & ADC_TSD_INTS) != 0)
    {
      /* Let the touchscreen handle its interrupts */

      sam_tsd_interrupt(pending);
      pending &= ~ADC_TSD_INTS;
    }
#endif

#ifdef SAMA5_ADC_HAVE_CHANNELS
  /* Check for end-of-conversion interrupts */

  if ((pending & ADC_INT_EOCALL) != 0)
    {
      /* Let the touchscreen handle its interrupts */

      sam_adc_endconversion(pending);
      pending &= ~ADC_INT_EOCALL;
    }
#endif

  /* Make sure that all interrupts were handled */

  DEBUGASSERT(pending == 0);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_adcinitialize
 *
 * Description:
 *   Initialize the adc
 *
 * Returned Value:
 *   Valid can device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *sam_adc_initialize(void)
{
  /* Enable the ADC peripheral clock*/

  sam_adc_enableclk();

  /* Configure ADC pins */

#ifdef CONFIG_SAMA5_ADC_CHAN0
  sam_configpio(PIO_ADC_AD0);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN1
  sam_configpio(PIO_ADC_AD1);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN2
  sam_configpio(PIO_ADC_AD2);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN3
  sam_configpio(PIO_ADC_AD3);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN4
  sam_configpio(PIO_ADC_AD4);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN5
  sam_configpio(PIO_ADC_AD5);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN6
  sam_configpio(PIO_ADC_AD6);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN7
  sam_configpio(PIO_ADC_AD7);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN8
  sam_configpio(PIO_ADC_AD8);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN9
  sam_configpio(PIO_ADC_AD9);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN10
  sam_configpio(PIO_ADC_AD10);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN11
  sam_configpio(PIO_ADC_AD11);
#endif
#if 0
  sam_configpio(PIO_ADC_TRG);
#endif

  /* Reset the ADC controller */

  sam_adc_putreg(priv, SAM_ADC_CR, ADC_CR_SWRST);

  /* Reset Mode Register */

  sam_adc_putreg(priv, SAM_ADC_MR, 0);

  /* Return a pointer to the device structure */

  return &g_adcdev;
}

/****************************************************************************
 * Name: sam_adc_getreg
 *
 * Description:
 *  Read any 32-bit register using an absolute address.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_REGDEBUG
static uint32_t sam_adc_getreg(ADC_HANDLE handle, uintptr_t address)
{
  struct sam_adc_s *priv = (struct sam_adc_s *)handle;
  uint32_t regval = getreg32(address);

  if (sam_adc_checkreg(priv, false, regval, address))
    {
      lldbg("%08x->%08x\n", address, regval);
    }

  return regval;
}
#endif

/****************************************************************************
 * Name: sam_adc_putreg
 *
 * Description:
 *  Write to any 32-bit register using an absolute address.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_REGDEBUG
void sam_adc_putreg(ADC_HANDLE handle, uintptr_t address, uint32_t regval)
{
  struct sam_adc_s *priv = (struct sam_adc_s *)handle;

  if (sam_adc_checkreg(priv, true, regval, address))
    {
      lldbg("%08x<-%08x\n", address, regval);
    }

  putreg32(regval, address);
}
#endif

#endif /* CONFIG_SAMA5_ADC */

/****************************************************************************
 * arch/arm/src/sama5/sam_pwm.c
 *
 *   Copyright (C) 2013, 2016 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <fixedmath.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/pwm.h>

#include "hardware/sam_pinmap.h"
#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "sam_periphclks.h"
#include "sam_pio.h"
#include "sam_pwm.h"

#ifdef CONFIG_SAMA5_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_DEBUG_PWM_INFO
#  undef CONFIG_SAMA5_PWM_REGDEBUG
#endif

/* Currently, we support only a single PWM peripheral.  However, the hooks
 * are in place to support multiple PWM peripherals.
 */

#define PWM_SINGLE 1

/* Pulse counting is not supported by this driver */

#ifdef CONFIG_PWM_PULSECOUNT
#  warning CONFIG_PWM_PULSECOUNT no supported by this driver.
#endif

/* Are we using CLKA? CLKB?  If so, at what frequency? */

#if defined(CONFIG_SAMA5_PWM_CLKA) && !defined(CONFIG_SAMA5_PWM_CLKA_FREQUENCY)
#    error CONFIG_SAMA5_PWM_CLKA_FREQUENCY is not defined
#endif

#if defined(CONFIG_SAMA5_PWM_CLKB) && !defined(CONFIG_SAMA5_PWM_CLKB_FREQUENCY)
#    error CONFIG_SAMA5_PWM_CLKB_FREQUENCY is not defined
#endif

#ifdef CONFIG_SAMA5_PWM_CHAN0
#  if defined(CONFIG_SAMA5_PWM_CHAN0_MCK)
#    undef CONFIG_SAMA5_PWM_CHAN0_CLKA
#    undef CONFIG_SAMA5_PWM_CHAN0_CLKB
#    if CONFIG_SAMA5_PWM_CHAN0_MCKDIV == 1
#      define SAMA5_PWM_CHAN0_MCKDIV_LOG2  0
#    elif CONFIG_SAMA5_PWM_CHAN0_MCKDIV == 2
#      define SAMA5_PWM_CHAN0_MCKDIV_LOG2  1
#    elif CONFIG_SAMA5_PWM_CHAN0_MCKDIV == 4
#      define SAMA5_PWM_CHAN0_MCKDIV_LOG2  2
#    elif CONFIG_SAMA5_PWM_CHAN0_MCKDIV == 8
#      define SAMA5_PWM_CHAN0_MCKDIV_LOG2  3
#    elif CONFIG_SAMA5_PWM_CHAN0_MCKDIV == 16
#      define SAMA5_PWM_CHAN0_MCKDIV_LOG2  4
#    elif CONFIG_SAMA5_PWM_CHAN0_MCKDIV == 32
#      define SAMA5_PWM_CHAN0_MCKDIV_LOG2  5
#    elif CONFIG_SAMA5_PWM_CHAN0_MCKDIV == 64
#      define SAMA5_PWM_CHAN0_MCKDIV_LOG2  6
#    elif CONFIG_SAMA5_PWM_CHAN0_MCKDIV == 128
#      define SAMA5_PWM_CHAN0_MCKDIV_LOG2  7
#    elif CONFIG_SAMA5_PWM_CHAN0_MCKDIV == 256
#      define SAMA5_PWM_CHAN0_MCKDIV_LOG2  8
#    elif CONFIG_SAMA5_PWM_CHAN0_MCKDIV == 512
#      define SAMA5_PWM_CHAN0_MCKDIV_LOG2  9
#    elif CONFIG_SAMA5_PWM_CHAN0_MCKDIV == 1024
#      define SAMA5_PWM_CHAN0_MCKDIV_LOG2  10
#    else
#      error Unsupported MCK divider value
#    endif

#  elif defined(CONFIG_SAMA5_PWM_CHAN0_CLKA)
#    undef CONFIG_SAMA5_PWM_CHAN0_CLKB

#  elif !defined(CONFIG_SAMA5_PWM_CHAN0_CLKB)
#    error CHAN0 clock source not defined

#  endif
#endif

#ifdef CONFIG_SAMA5_PWM_CHAN1
#  if defined(CONFIG_SAMA5_PWM_CHAN1_MCK)
#    undef CONFIG_SAMA5_PWM_CHAN1_CLKA
#    undef CONFIG_SAMA5_PWM_CHAN1_CLKB
#    if CONFIG_SAMA5_PWM_CHAN1_MCKDIV == 1
#      define SAMA5_PWM_CHAN1_MCKDIV_LOG2  0
#    elif CONFIG_SAMA5_PWM_CHAN1_MCKDIV == 2
#      define SAMA5_PWM_CHAN1_MCKDIV_LOG2  1
#    elif CONFIG_SAMA5_PWM_CHAN1_MCKDIV == 4
#      define SAMA5_PWM_CHAN1_MCKDIV_LOG2  2
#    elif CONFIG_SAMA5_PWM_CHAN1_MCKDIV == 8
#      define SAMA5_PWM_CHAN1_MCKDIV_LOG2  3
#    elif CONFIG_SAMA5_PWM_CHAN1_MCKDIV == 16
#      define SAMA5_PWM_CHAN1_MCKDIV_LOG2  4
#    elif CONFIG_SAMA5_PWM_CHAN1_MCKDIV == 32
#      define SAMA5_PWM_CHAN1_MCKDIV_LOG2  5
#    elif CONFIG_SAMA5_PWM_CHAN1_MCKDIV == 64
#      define SAMA5_PWM_CHAN1_MCKDIV_LOG2  6
#    elif CONFIG_SAMA5_PWM_CHAN1_MCKDIV == 128
#      define SAMA5_PWM_CHAN1_MCKDIV_LOG2  7
#    elif CONFIG_SAMA5_PWM_CHAN1_MCKDIV == 256
#      define SAMA5_PWM_CHAN1_MCKDIV_LOG2  8
#    elif CONFIG_SAMA5_PWM_CHAN1_MCKDIV == 512
#      define SAMA5_PWM_CHAN1_MCKDIV_LOG2  9
#    elif CONFIG_SAMA5_PWM_CHAN1_MCKDIV == 1024
#      define SAMA5_PWM_CHAN1_MCKDIV_LOG2  10
#    else
#      error Unsupported MCK divider value
#    endif

#  elif defined(CONFIG_SAMA5_PWM_CHAN1_CLKA)
#    undef CONFIG_SAMA5_PWM_CHAN1_CLKB

#  elif !defined(CONFIG_SAMA5_PWM_CHAN1_CLKB)
#    error CHAN1 clock source not defined

#  endif
#endif

#ifdef CONFIG_SAMA5_PWM_CHAN2
#  if defined(CONFIG_SAMA5_PWM_CHAN2_MCK)
#    undef CONFIG_SAMA5_PWM_CHAN2_CLKA
#    undef CONFIG_SAMA5_PWM_CHAN2_CLKB
#    if CONFIG_SAMA5_PWM_CHAN2_MCKDIV == 1
#      define SAMA5_PWM_CHAN2_MCKDIV_LOG2  0
#    elif CONFIG_SAMA5_PWM_CHAN2_MCKDIV == 2
#      define SAMA5_PWM_CHAN2_MCKDIV_LOG2  1
#    elif CONFIG_SAMA5_PWM_CHAN2_MCKDIV == 4
#      define SAMA5_PWM_CHAN2_MCKDIV_LOG2  2
#    elif CONFIG_SAMA5_PWM_CHAN2_MCKDIV == 8
#      define SAMA5_PWM_CHAN2_MCKDIV_LOG2  3
#    elif CONFIG_SAMA5_PWM_CHAN2_MCKDIV == 16
#      define SAMA5_PWM_CHAN2_MCKDIV_LOG2  4
#    elif CONFIG_SAMA5_PWM_CHAN2_MCKDIV == 32
#      define SAMA5_PWM_CHAN2_MCKDIV_LOG2  5
#    elif CONFIG_SAMA5_PWM_CHAN2_MCKDIV == 64
#      define SAMA5_PWM_CHAN2_MCKDIV_LOG2  6
#    elif CONFIG_SAMA5_PWM_CHAN2_MCKDIV == 128
#      define SAMA5_PWM_CHAN2_MCKDIV_LOG2  7
#    elif CONFIG_SAMA5_PWM_CHAN2_MCKDIV == 256
#      define SAMA5_PWM_CHAN2_MCKDIV_LOG2  8
#    elif CONFIG_SAMA5_PWM_CHAN2_MCKDIV == 512
#      define SAMA5_PWM_CHAN2_MCKDIV_LOG2  9
#    elif CONFIG_SAMA5_PWM_CHAN2_MCKDIV == 1024
#      define SAMA5_PWM_CHAN2_MCKDIV_LOG2  10
#    else
#      error Unsupported MCK divider value
#    endif

#  elif defined(CONFIG_SAMA5_PWM_CHAN2_CLKA)
#    undef CONFIG_SAMA5_PWM_CHAN2_CLKB

#  elif !defined(CONFIG_SAMA5_PWM_CHAN2_CLKB)
#    error CHAN2 clock source not defined

#  endif
#endif

#ifdef CONFIG_SAMA5_PWM_CHAN3
#  if defined(CONFIG_SAMA5_PWM_CHAN3_MCK)
#    undef CONFIG_SAMA5_PWM_CHAN3_CLKA
#    undef CONFIG_SAMA5_PWM_CHAN3_CLKB
#    if CONFIG_SAMA5_PWM_CHAN3_MCKDIV == 1
#      define SAMA5_PWM_CHAN3_MCKDIV_LOG2  0
#    elif CONFIG_SAMA5_PWM_CHAN3_MCKDIV == 2
#      define SAMA5_PWM_CHAN3_MCKDIV_LOG2  1
#    elif CONFIG_SAMA5_PWM_CHAN3_MCKDIV == 4
#      define SAMA5_PWM_CHAN3_MCKDIV_LOG2  2
#    elif CONFIG_SAMA5_PWM_CHAN3_MCKDIV == 8
#      define SAMA5_PWM_CHAN3_MCKDIV_LOG2  3
#    elif CONFIG_SAMA5_PWM_CHAN3_MCKDIV == 16
#      define SAMA5_PWM_CHAN3_MCKDIV_LOG2  4
#    elif CONFIG_SAMA5_PWM_CHAN3_MCKDIV == 32
#      define SAMA5_PWM_CHAN3_MCKDIV_LOG2  5
#    elif CONFIG_SAMA5_PWM_CHAN3_MCKDIV == 64
#      define SAMA5_PWM_CHAN3_MCKDIV_LOG2  6
#    elif CONFIG_SAMA5_PWM_CHAN3_MCKDIV == 128
#      define SAMA5_PWM_CHAN3_MCKDIV_LOG2  7
#    elif CONFIG_SAMA5_PWM_CHAN3_MCKDIV == 256
#      define SAMA5_PWM_CHAN3_MCKDIV_LOG2  8
#    elif CONFIG_SAMA5_PWM_CHAN3_MCKDIV == 512
#      define SAMA5_PWM_CHAN3_MCKDIV_LOG2  9
#    elif CONFIG_SAMA5_PWM_CHAN3_MCKDIV == 1024
#      define SAMA5_PWM_CHAN3_MCKDIV_LOG2  10
#    else
#      error Unsupported MCK divider value
#    endif

#  elif defined(CONFIG_SAMA5_PWM_CHAN3_CLKA)
#    undef CONFIG_SAMA5_PWM_CHAN3_CLKB

#  elif !defined(CONFIG_SAMA5_PWM_CHAN3_CLKB)
#    error CHAN3 clock source not defined

#  endif
#endif

/* The current design does not use any PWM interrupts */

#undef PWM_INTERRUPTS

/* Pin configuration ********************************************************/

#define PWM_INPUTCFG     (PIO_INPUT | PIO_CFG_DEFAULT | PIO_DRIVE_LOW)
#define PWM_PINMASK      (PIO_PORT_MASK | PIO_PIN_MASK)
#define PWM_MKINPUT(cfg) (((cfg) & PWM_PINMASK) | PWM_INPUTCFG)

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* Channel clock sources */

enum pwm_clksrc_e
{
  PWM_CLKSRC_MCK = 1,  /* Source = Divided MCK */
  PWM_CLKSRC_CLKA,     /* Source = CLKA */
  PWM_CLKSRC_CLKB,     /* Source = CLKB */
};

/* This structure represents the state of one PWM channel */

struct sam_pwm_s;
struct sam_pwm_chan_s
{
  const struct pwm_ops_s *ops;     /* PWM operations */
#ifndef PWM_SINGLE
  struct sam_pwm_s *pwm;           /* Parent PWM peripheral */
#endif
  uintptr_t base;                  /* Base address of channel registers */
  uint8_t channel;                 /* PWM channel: {0,..3} */
  uint8_t clksrc;                  /* 0=MCK; 1=CLKA; 2=CLKB */
  uint8_t divlog2;                 /* Log2 MCK divisor: 0->1, 1->2, 2->4, ... 10->1024 */
  pio_pinset_t ohpincfg;           /* Output high pin configuration */
  pio_pinset_t olpincfg;           /* Output low pin configuration */
  pio_pinset_t fipincfg;           /* Fault input pin configuration */
};

/* This structure represents the overall state of the PWM peripheral */

struct sam_pwm_s
{
  bool initialized;                /* True: one time initialization has been performed */
#ifndef PWM_SINGLE
  uintptr_t base;                  /* Base address of peripheral registers */
#endif
  /* Debug stuff */

#ifdef CONFIG_SAMA5_PWM_REGDEBUG
  bool wr;                         /* Last was a write */
  uint32_t regaddr;                /* Last address */
  uint32_t regval;                 /* Last value */
  int count;                       /* Number of times */
#endif
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/
/* Register access */

#ifdef CONFIG_SAMA5_PWM_REGDEBUG
static bool pwm_checkreg(FAR struct sam_pwm_s *chan, bool wr, uint32_t regval,
                         uintptr_t regaddr);
#else
# define pwm_checkreg(chan,wr,regval,regaddr) (false)
#endif

static uint32_t pwm_getreg(FAR struct sam_pwm_chan_s *chan, int offset);
static void pwm_putreg(FAR struct sam_pwm_chan_s *chan, int offset, uint32_t regval);

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(FAR struct sam_pwm_chan_s *chan, FAR const char *msg);
#else
#  define pwm_dumpregs(chan,msg)
#endif

/* PWM Interrupts */

#ifdef PWM_INTERRUPTS
static int pwm_interrupt(int irq, void *context, FAR void *arg);
#endif

/* PWM driver methods */

static int pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);
static int pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info);
static int pwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                     int cmd, unsigned long arg);

/* Initialization */

static unsigned int pwm_clk_prescaler_log2(uint32_t mck, uint32_t fclk);
static unsigned int pwm_clk_divider(uint32_t mck, uint32_t fclk,
                                    unsigned int prelog2);
static uint32_t pwm_clk_frequency(uint32_t mck, unsigned int prelog2,
                                  unsigned int div);
static void pwm_resetpins(FAR struct sam_pwm_chan_s *chan);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* This is the list of lower half PWM driver methods used by the upper
 * half driver
 */

static const struct pwm_ops_s g_pwmops =
{
  .setup       = pwm_setup,
  .shutdown    = pwm_shutdown,
  .start       = pwm_start,
  .stop        = pwm_stop,
  .ioctl       = pwm_ioctl,
};

/* This is the overall state of the PWM peripheral */

static struct sam_pwm_s g_pwm =
{
  .initialized = false,
#ifndef PWM_SINGLE
  .base        = SAM_PWMC_VBASE,
#endif
};

#ifdef CONFIG_SAMA5_PWM_CHAN0
/* This is the state of the PWM channel 0 */

static struct sam_pwm_chan_s g_pwm_chan0 =
{
  .ops         = &g_pwmops,
#ifndef PWM_SINGLE
  .pwm         = &g_pwm,
#endif
  .channel     = 0,
  .base        = SAM_PWM_CHANA_BASE(0),

#if defined(CONFIG_SAMA5_PWM_CHAN0_MCK)
  .clksrc      = PWM_CLKSRC_MCK,
  .divlog2     = SAMA5_PWM_CHAN0_MCKDIV_LOG2,
#elif defined(CONFIG_SAMA5_PWM_CHAN0_CLKA)
  .clksrc      = PWM_CLKSRC_CLKA,
#elif defined(CONFIG_SAMA5_PWM_CHAN0_CLKB)
  .clksrc      = PWM_CLKSRC_CLKB,
#else
#  error No clock source for channel 0
#endif

#ifdef CONFIG_SAMA5_PWM_CHAN0_OUTPUTH
  .ohpincfg    = PIO_PWM0_H,
#endif
#ifdef CONFIG_SAMA5_PWM_CHAN0_OUTPUTL
  .olpincfg    = PIO_PWM0_L,
#endif
#ifdef CONFIG_SAMA5_PWM_CHAN0_FAULTINPUT
  .fipincfg    = PIO_PWM0_FI,
#endif
};
#endif

#ifdef CONFIG_SAMA5_PWM_CHAN1
/* This is the state of the PWM channel 1 */

static struct sam_pwm_chan_s g_pwm_chan1 =
{
  .ops         = &g_pwmops,
#ifndef PWM_SINGLE
  .pwm         = &g_pwm,
#endif
  .channel     = 1,
  .base        = SAM_PWM_CHANA_BASE(1),

#if defined(CONFIG_SAMA5_PWM_CHAN1_MCK)
  .clksrc      = PWM_CLKSRC_MCK,
  .divlog2     = SAMA5_PWM_CHAN1_MCKDIV_LOG2,
#elif defined(CONFIG_SAMA5_PWM_CHAN1_CLKA)
  .clksrc      = PWM_CLKSRC_CLKA,
#elif defined(CONFIG_SAMA5_PWM_CHAN1_CLKB)
  .clksrc      = PWM_CLKSRC_CLKB,
#else
#  error No clock source for channel 0
#endif

#ifdef CONFIG_SAMA5_PWM_CHAN1_OUTPUTH
  .ohpincfg    = PIO_PWM1_H,
#endif
#ifdef CONFIG_SAMA5_PWM_CHAN1_OUTPUTL
  .olpincfg    = PIO_PWM1_L,
#endif
#ifdef CONFIG_SAMA5_PWM_CHAN1_FAULTINPUT
  .fipincfg    = PIO_PWM1_FI,
#endif
};
#endif

#ifdef CONFIG_SAMA5_PWM_CHAN2
/* This is the state of the PWM channel 2 */

static struct sam_pwm_chan_s g_pwm_chan2 =
{
  .ops         = &g_pwmops,
#ifndef PWM_SINGLE
  .pwm         = &g_pwm,
#endif
  .channel     = 2,
  .base        = SAM_PWM_CHANA_BASE(2),

#if defined(CONFIG_SAMA5_PWM_CHAN2_MCK)
  .clksrc      = PWM_CLKSRC_MCK,
  .divlog2     = SAMA5_PWM_CHAN2_MCKDIV_LOG2,
#elif defined(CONFIG_SAMA5_PWM_CHAN2_CLKA)
  .clksrc      = PWM_CLKSRC_CLKA,
#elif defined(CONFIG_SAMA5_PWM_CHAN2_CLKB)
  .clksrc      = PWM_CLKSRC_CLKB,
#else
#  error No clock source for channel 0
#endif

#ifdef CONFIG_SAMA5_PWM_CHAN2_OUTPUTH
  .ohpincfg    = PIO_PWM2_H,
#endif
#ifdef CONFIG_SAMA5_PWM_CHAN2_OUTPUTL
  .olpincfg    = PIO_PWM2_L,
#endif
#ifdef CONFIG_SAMA5_PWM_CHAN2_FAULTINPUT
  .fipincfg    = PIO_PWM2_FI,
#endif
};
#endif

#ifdef CONFIG_SAMA5_PWM_CHAN3
/* This is the state of the PWM channel 3 */

static struct sam_pwm_chan_s g_pwm_chan3 =
{
  .ops         = &g_pwmops,
#ifndef PWM_SINGLE
  .pwm         = &g_pwm,
#endif
  .channel     = 3,
  .base        = SAM_PWM_CHANA_BASE(3),

#if defined(CONFIG_SAMA5_PWM_CHAN3_MCK)
  .clksrc      = PWM_CLKSRC_MCK,
  .divlog2     = SAMA5_PWM_CHAN3_MCKDIV_LOG2,
#elif defined(CONFIG_SAMA5_PWM_CHAN3_CLKA)
  .clksrc      = PWM_CLKSRC_CLKA,
#elif defined(CONFIG_SAMA5_PWM_CHAN3_CLKB)
  .clksrc      = PWM_CLKSRC_CLKB,
#else
#  error No clock source for channel 0
#endif

#ifdef CONFIG_SAMA5_PWM_CHAN3_OUTPUTH
  .ohpincfg    = PIO_PWM3_H,
#endif
#ifdef CONFIG_SAMA5_PWM_CHAN3_OUTPUTL
  .olpincfg    = PIO_PWM3_L,
#endif
#ifdef CONFIG_SAMA5_PWM_CHAN3_FAULTINPUT
  .fipincfg    = PIO_PWM3_FI,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   regval  - The value to be written
 *   regaddr - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_PWM_REGDEBUG
static bool pwm_checkreg(FAR struct sam_pwm_s *pwm, bool wr, uint32_t regval,
                         uintptr_t regaddr)
{
  if (wr      == pwm->wr &&      /* Same kind of access? */
      regval  == pwm->regval &&  /* Same value? */
      regaddr == pwm->regaddr)   /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      pwm->count++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (pwm->count > 0)
        {
          /* Yes... show how many times we did it */

          pwminfo("...[Repeats %d times]...\n", pwm->count);
        }

      /* Save information about the new access */

      pwm->wr   = wr;
      pwm->regval  = regval;
      pwm->regaddr = regaddr;
      pwm->count   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: pwm_getreg
 *
 * Description:
 *   Read the value of a PWM register.
 *
 * Input Parameters:
 *   chan - A reference to the PWM channel instance
 *   offset - The offset to the PWM register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t pwm_getreg(struct sam_pwm_chan_s *chan, int offset)
{
#ifdef PWM_SINGLE
  uintptr_t regaddr;
  uint32_t  regval;

  regaddr = SAM_PWMC_VBASE + offset;
  regval  = getreg32(regaddr);

#ifdef CONFIG_SAMA5_PWM_REGDEBUG
  if (pwm_checkreg(&g_pwm, false, regval, regaddr))
    {
      pwminfo("%08x->%08x\n", regaddr, regval);
    }
#endif

  return regval;

#else
  struct sam_pwm_chan_s *pwm = chan->pwm;
  uintptr_t regaddr;
  uint32_t  regval;

  regaddr = pwm->base + offset;
  regval  = getreg32(regaddr);

#ifdef CONFIG_SAMA5_PWM_REGDEBUG
  if (pwm_checkreg(pwm, false, regval, regaddr))
    {
      pwminfo("%08x->%08x\n", regaddr, regval);
    }
#endif

  return regval;

#endif
}

/****************************************************************************
 * Name: pwm_chan_getreg
 *
 * Description:
 *   Read the value of a PWM channel register.
 *
 * Input Parameters:
 *   chan - A reference to the PWM channel instance
 *   offset - The offset to the channel register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO /* Currently only used for debug output */
static uint32_t pwm_chan_getreg(struct sam_pwm_chan_s *chan, int offset)
{
  uintptr_t regaddr;
  uint32_t  regval;

  regaddr = chan->base + offset;
  regval  = getreg32(regaddr);

#ifdef CONFIG_SAMA5_PWM_REGDEBUG
#ifdef PWM_SINGLE
  if (pwm_checkreg(&g_pwm, false, regval, regaddr))
#else
  if (pwm_checkreg(chan->pwm, false, regval, regaddr))
#endif
    {
      pwminfo("%08x->%08x\n", regaddr, regval);
    }
#endif

  return regval;
}
#endif

/****************************************************************************
 * Name: pwm_putreg
 *
 * Description:
 *   Write a value to a PWM register.
 *
 * Input Parameters:
 *   chan - A reference to the PWM channel instance
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pwm_putreg(struct sam_pwm_chan_s *chan, int offset,
                       uint32_t regval)
{
#ifdef PWM_SINGLE
  uintptr_t regaddr = SAM_PWMC_VBASE + offset;

#ifdef CONFIG_SAMA5_PWM_REGDEBUG
  if (pwm_checkreg(&g_pwm, true, regval, regaddr))
    {
      pwminfo("%08x<-%08x\n", regaddr, regval);
    }
#endif

  putreg32(regval, regaddr);

#else
  struct sam_pwm_chan_s *pwm = chan->pwm;
  uintptr_t regaddr = pwm->base + offset;

#ifdef CONFIG_SAMA5_PWM_REGDEBUG
  if (pwm_checkreg(pwm, true, regval, regaddr))
    {
      pwminfo("%08x<-%08x\n", regaddr, regval);
    }
#endif

  putreg32(regval, regaddr);

#endif
}

/****************************************************************************
 * Name: pwm_chan_putreg
 *
 * Description:
 *   Read the value of an PWM channel register.
 *
 * Input Parameters:
 *   chan - A reference to the PWM channel instance
 *   offset - The offset to the channel register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pwm_chan_putreg(struct sam_pwm_chan_s *chan, int offset,
                            uint32_t regval)
{
  uintptr_t regaddr = chan->base + offset;

#ifdef CONFIG_SAMA5_PWM_REGDEBUG
#ifdef PWM_SINGLE
  if (pwm_checkreg(&g_pwm, true, regval, regaddr))
#else
  if (pwm_checkreg(chan->pwm, true, regval, regaddr))
#endif
    {
      pwminfo("%08x<-%08x\n", regaddr, regval);
    }
#endif

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: pwm_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   chan - A reference to the PWM channel instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct sam_pwm_chan_s *chan, FAR const char *msg)
{
  pwminfo("PWM: %s\n", msg);
  pwminfo("   CLK: %08x    SR: %08x  IMR1: %08x  ISR1: %08x\n",
          pwm_getreg(chan, SAM_PWM_CLK_OFFSET),
          pwm_getreg(chan, SAM_PWM_SR_OFFSET),
          pwm_getreg(chan, SAM_PWM_IMR1_OFFSET),
          pwm_getreg(chan, SAM_PWM_ISR1_OFFSET));
  pwminfo("   SCM: %08x  SCUC: %08x  SCUP: %08x  IMR2: %08x\n",
          pwm_getreg(chan, SAM_PWM_SCM_OFFSET),
          pwm_getreg(chan, SAM_PWM_SCUC_OFFSET),
          pwm_getreg(chan, SAM_PWM_SCUP_OFFSET),
          pwm_getreg(chan, SAM_PWM_IMR2_OFFSET));
  pwminfo("  ISR2: %08x   OOV: %08x    OS: %08x   FMR: %08x\n",
          pwm_getreg(chan, SAM_PWM_ISR2_OFFSET),
          pwm_getreg(chan, SAM_PWM_OOV_OFFSET),
          pwm_getreg(chan, SAM_PWM_OS_OFFSET),
          pwm_getreg(chan, SAM_PWM_FMR_OFFSET));
  pwminfo("   FSR: %08x   FPV: %08x   FPE: %08x ELMR0: %08x\n",
          pwm_getreg(chan, SAM_PWM_FSR_OFFSET),
          pwm_getreg(chan, SAM_PWM_FPV_OFFSET),
          pwm_getreg(chan, SAM_PWM_FPE_OFFSET),
          pwm_getreg(chan, SAM_PWM_ELMR0_OFFSET));
  pwminfo(" ELMR1: %08x  SMMR: %08x  WPSR: %08x\n",
          pwm_getreg(chan, SAM_PWM_ELMR1_OFFSET),
          pwm_getreg(chan, SAM_PWM_SMMR_OFFSET),
          pwm_getreg(chan, SAM_PWM_WPSR_OFFSET));
  pwminfo(" CMPV0: %08x CMPM0: %08x CMPV1: %08x CMPM1: %08x\n",
          pwm_getreg(chan, SAM_PWM_CMPV0_OFFSET),
          pwm_getreg(chan, SAM_PWM_CMPM0_OFFSET),
          pwm_getreg(chan, SAM_PWM_CMPV1_OFFSET),
          pwm_getreg(chan, SAM_PWM_CMPM1_OFFSET));
  pwminfo(" CMPV2: %08x CMPM2: %08x CMPV3: %08x CMPM3: %08x\n",
          pwm_getreg(chan, SAM_PWM_CMPV2_OFFSET),
          pwm_getreg(chan, SAM_PWM_CMPM2_OFFSET),
          pwm_getreg(chan, SAM_PWM_CMPV3_OFFSET),
          pwm_getreg(chan, SAM_PWM_CMPM3_OFFSET));
  pwminfo(" CMPV4: %08x CMPM4: %08x CMPV5: %08x CMPM5: %08x\n",
          pwm_getreg(chan, SAM_PWM_CMPV4_OFFSET),
          pwm_getreg(chan, SAM_PWM_CMPM4_OFFSET),
          pwm_getreg(chan, SAM_PWM_CMPV5_OFFSET),
          pwm_getreg(chan, SAM_PWM_CMPM5_OFFSET));
  pwminfo(" CMPV6: %08x CMPM6: %08x CMPV7: %08x CMPM7: %08x\n",
          pwm_getreg(chan, SAM_PWM_CMPV6_OFFSET),
          pwm_getreg(chan, SAM_PWM_CMPM6_OFFSET),
          pwm_getreg(chan, SAM_PWM_CMPV7_OFFSET),
          pwm_getreg(chan, SAM_PWM_CMPM7_OFFSET));
  pwminfo("Channel %d: %s\n", chan->channel, msg);
  pwminfo("   CMR: %08x  CDTY: %08x  CPRD: %08x  CCNT: %08x\n",
          pwm_chan_getreg(chan, SAM_PWM_CMR_OFFSET),
          pwm_chan_getreg(chan, SAM_PWM_CDTY_OFFSET),
          pwm_chan_getreg(chan, SAM_PWM_CPRD_OFFSET),
          pwm_chan_getreg(chan, SAM_PWM_CCNT_OFFSET));
  pwminfo("    CT: %08x\n",
          pwm_chan_getreg(chan, SAM_PWM_DT_OFFSET));
}
#endif

/****************************************************************************
 * Name: pwm_interrupt
 *
 * Description:
 *   Handle timer interrupts.
 *
 * Input Parameters:
 *   Standard interrupt handler inputs
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

#ifdef PWM_INTERRUPTS
static int pwm_interrupt(int irq, void *context, FAR void *arg)
{
  /* No PWM interrupts are used in the current design */

#warning Missing logic
  return OK;
}
#endif

/****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   will be configured and initialized the device so that it is ready for
 *   use.  It will not, however, output pulses until the start method is
 *   called.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct sam_pwm_chan_s *chan = (FAR struct sam_pwm_chan_s *)dev;

  pwminfo("Channel %d: H=%08x L=%08x FI=%08x\n",
          chan->channel, chan->ohpincfg, chan->olpincfg, chan->fipincfg);

  /* Configure selected PWM pins */

  if (chan->ohpincfg)
    {
      sam_configpio(chan->ohpincfg);
    }

  if (chan->olpincfg)
    {
      sam_configpio(chan->olpincfg);
    }

  if (chan->fipincfg)
    {
      sam_configpio(chan->fipincfg);
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct sam_pwm_chan_s *chan = (FAR struct sam_pwm_chan_s *)dev;

  pwminfo("Channel %d\n", chan->channel);

  /* Make sure that the output has been stopped */

  pwm_stop(dev);

  /* Then put the GPIO pins back to the default, input state */

  pwm_resetpins(chan);
  return OK;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev  - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info)
{
  FAR struct sam_pwm_chan_s *chan = (FAR struct sam_pwm_chan_s *)dev;
#if defined(CONFIG_SAMA5_PWM_CLKA) || defined(CONFIG_SAMA5_PWM_CLKB)
  unsigned int prelog2;
  unsigned int div;
  uint32_t mck;
#endif
  uint32_t regval;
  uint32_t cprd;
  uint32_t fsrc;

  /* Disable the channel (should already be disabled) */

  pwm_putreg(chan, SAM_PWM_DIS_OFFSET, PWM_CHID(chan->channel));

  /* Determine the clock source */

  switch (chan->clksrc)
    {
    case PWM_CLKSRC_MCK:
      {
        regval = PWM_CMR_CPRE_MCKDIV(chan->divlog2);
        fsrc   = BOARD_MCK_FREQUENCY >> chan->divlog2;
      }
      break;

#ifdef CONFIG_SAMA5_PWM_CLKA
    case PWM_CLKSRC_CLKA:
      {
        regval  = pwm_getreg(chan, SAM_PWM_CLK_OFFSET);
        prelog2 = (unsigned int)((regval & PWM_CLK_PREA_MASK) >> PWM_CLK_PREA_SHIFT);
        div     = (unsigned int)((regval & PWM_CLK_DIVA_MASK) >> PWM_CLK_DIVA_SHIFT);
        mck     = BOARD_MCK_FREQUENCY;
        fsrc    = pwm_clk_frequency(mck, prelog2, div);
        regval  = PWM_CMR_CPRE_CLKA;
      }
      break;
#endif

#ifdef CONFIG_SAMA5_PWM_CLKB
    case PWM_CLKSRC_CLKB:
      {
        regval  = pwm_getreg(chan, SAM_PWM_CLK_OFFSET);
        prelog2 = (unsigned int)((regval & PWM_CLK_PREB_MASK) >> PWM_CLK_PREB_SHIFT);
        div     = (unsigned int)((regval & PWM_CLK_DIVB_MASK) >> PWM_CLK_DIVB_SHIFT);
        mck     = BOARD_MCK_FREQUENCY;
        fsrc    = pwm_clk_frequency(mck, prelog2, div);
        regval  = PWM_CMR_CPRE_CLKB;
      }
      break;
#endif

    default:
      pwmerr("ERROR: Invalid or unsupported clock source value: %d\n", chan->clksrc);
      return -EINVAL;
    }

  /* Configure the channel */

  pwm_chan_putreg(chan, SAM_PWM_CMR_OFFSET, PWM_CMR_CPRE_CLKA);

  /* Set the PWM period.
   *
   * If the waveform is left-aligned, then the output waveform period
   * depends on the channel counter source clock and can be calculated
   * as follows:
   *
   *   Tchan = cprd / Fsrc
   *   cprd  = Fsrc / Fchan
   *
   * If the waveform is center-aligned, then the output waveform period
   * depends on the channel counter source clock and can be calculated:
   *
   *   Tchan = 2 * cprd / Fsrc
   *   cprd  = Fsrc / 2 / Fchan
   *
   * Since the PWM is disabled, we can write directly to the CPRD (vs.
   * the CPRDUPD) register.
   */

  cprd = (fsrc + (info->frequency >> 1)) / info->frequency;
  pwm_chan_putreg(chan, SAM_PWM_CPRD_OFFSET, cprd);

  /* Set the PWM duty.  Since the PWM is disabled, we can write directly
   * to the CTDY (vs. the CTDYUPD) register.
   */

  regval = b16toi(info->duty * cprd + b16HALF);
  if (regval > cprd)
    {
      /* Rounding up could cause the duty value to exceed CPRD (?) */

      regval = cprd;
    }

  pwm_chan_putreg(chan, SAM_PWM_CDTY_OFFSET, regval);
  pwminfo("Fsrc=%d cprd=%d cdty=%d\n", fsrc, cprd, regval);

  /* Enable the channel */

  pwm_putreg(chan, SAM_PWM_ENA_OFFSET, PWM_CHID(chan->channel));
  pwm_dumpregs(chan, "After start");
  return OK;
}

/****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   This function is called to stop the pulsed output at anytime.  This
 *   method is also called from the timer interrupt handler when a repetition
 *   count expires... automatically stopping the timer.
 *
 ****************************************************************************/

static int pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct sam_pwm_chan_s *chan = (FAR struct sam_pwm_chan_s *)dev;

  pwminfo("Channel %d\n", chan->channel);

  /* Disable further PWM interrupts from this channel */

  pwm_putreg(chan, SAM_PWM_IDR1_OFFSET,
             PWM_INT1_CHID(chan->channel) | PWM_INT1_FCHID(chan->channel));

  /* Disable the channel */

  pwm_putreg(chan, SAM_PWM_DIS_OFFSET, PWM_CHID(chan->channel));
  pwm_dumpregs(chan, "After stop");
  return OK;
}

/****************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg)
{
#ifdef CONFIG_DEBUG_PWM_INFO
  FAR struct sam_pwm_chan_s *chan = (FAR struct sam_pwm_chan_s *)dev;

  /* There are no platform-specific ioctl commands */

  pwminfo("Channel %d\n", chan->channel);
#endif
  return -ENOTTY;
}

/****************************************************************************
 * Name: pwm_clk_prescaler_log2
 *
 * Description:
 *   Return log2 of the clock prescaler value.  The PWM clock divisor
 *   register fields use this kind of value.  The return value of this
 *   function can be converted into a PWM clock register value or an absolute
 *   prescaler value by applying the following operations (macros defined in
 *   chip/sam_pwm.h):
 *
 *   This function selects the prescaler value that allows the largest, valid
 *   divider value.  This may not be optimal in all cases, but in general
 *   should provide a reasonable frequency value.  The frequency is given by:
 *
 *     frequency = MCK / prescaler / div
 *
 *   The divider has a range of 1-255.  Pick smallest prescaler such that:
 *
 *     prescaler = MCK / frequency / div < 256
 *
 *   Example usage given:
 *     unsigned int prelog2;
 *     unsigned int prescaler;
 *     uint32_t regbits;
 *
 *   For clock A:
 *     prelog2   = pwm_clk_prescaler_log2(BOARD_MCK_FREQUENCY,
 *                                    CONFIG_SAMA5_PWM_CLKA_FREQUENCY )
 *     regbits   = PWM_CLK_PREA_DIV(prelog2);
 *     prescaler = (1 << prelog2)
 *
 *   For clock B:
 *     prelog2   = pwm_clk_prescaler_log2(BOARD_MCK_FREQUENCY,
 *                                    CONFIG_SAMA5_PWM_CLKB_FREQUENCY )
 *     regbits   = PWM_CLK_PREB_DIV(prelog2);
 *     prescaler = (1 << prelog2)
 *
 * Input Parameters:
 *     mck  - The main clock frequency
 *     fclk - The desired clock A or B frequency
 *
 * Returned Value:
 *   The select value of log2(prescaler) in the range 0-10 corresponding to
 *   the actual prescaler value in the range 1-1024.
 *
 ****************************************************************************/

static unsigned int pwm_clk_prescaler_log2(uint32_t mck, uint32_t fclk)
{
  uint32_t unscaled;
  unsigned int prelog2;

  unscaled = mck / fclk;
  prelog2  = 0;

  /* Loop, incrementing the log2(prescaler) value.  Exit with either:
   *
   *   1) unscaled < 256 and prelog2 <= 10, or with
   *   2) unscaled >= 256 and prelog2 == 10
   */

  while (unscaled >= 256 && prelog2 < 10)
    {
      unscaled >>= 1;
      prelog2++;
    }

  DEBUGASSERT(unscaled < 256);
  return prelog2;
}

/****************************************************************************
 * Name: pwm_clk_divider
 *
 * Description:
 *   Given that we have already selected the prescaler value, select the
 *   divider in the range of 1 through 255.  The CLKA/B frequency is
 *   determined by both the prescaler and divider values:
 *
 *   frequency = MCK / prescaler / div
 *
 * Then:
 *
 *   div = MCK / prescaler / frequency
 *
 * Input Parameters:
 *     mck     - The main clock frequency
 *     fclk    - The desired clock A or B frequency
 *     prelog2 - The log2(prescaler) value previously selected by
 *               pwm_prescale_log2().
 *
 * Returned Value:
 *   The select value of log2(prescaler) in the range 0-10 corresponding to
 *   the actual prescaler value in the range 1-1024.
 *
 ****************************************************************************/

static unsigned int pwm_clk_divider(uint32_t mck, uint32_t fclk,
                                    unsigned int prelog2)
{
  uint32_t div = (mck >> prelog2) / fclk;

  if (div < 1)
    {
      div = 1;
    }
  else if (div > 255)
    {
      div = 255;
    }

  return div;
}

/****************************************************************************
 * Name: pwm_clk_frequency
 *
 * Description:
 *   Given that we have already selected the prescaler value and calculated
 *   the corresponding divider, the result clock frequency is give by:
 *
 *   frequency = MCK / prescaler / div
 *
 * Input Parameters:
 *     mck     - The main clock frequency
 *     prelog2 - The log2(prescaler) value previously selected by
 *               pwm_prescale_log2().
 *     div     - The divider previously calculated from pwm_clk_divider().
 *
 * Returned Value:
 *   The select value of log2(prescaler) in the range 0-10 corresponding to
 *   the actual prescaler value in the range 1-1024.
 *
 ****************************************************************************/

static uint32_t pwm_clk_frequency(uint32_t mck, unsigned int prelog2,
                                  unsigned int div)
{
  return (mck >> prelog2) / div;
}

/****************************************************************************
 * Name: pwm_resetpins
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   chan - A reference to the PWM channel instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pwm_resetpins(FAR struct sam_pwm_chan_s *chan)
{
  if (chan->ohpincfg)
    {
      sam_configpio(PWM_MKINPUT(chan->ohpincfg));
    }

  if (chan->olpincfg)
    {
      sam_configpio(PWM_MKINPUT(chan->olpincfg));
    }

  if (chan->fipincfg)
    {
      sam_configpio(PWM_MKINPUT(chan->fipincfg));
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_pwminitialize
 *
 * Description:
 *   Initialize one PWM channel for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   channel - A number identifying the PWM channel use.
 *
 * Returned Value:
 *   On success, a pointer to the SAMA5 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct pwm_lowerhalf_s *sam_pwminitialize(int channel)
{
  FAR struct sam_pwm_chan_s *chan;
  uint32_t regval;

  pwminfo("Channel %d\n", channel);

  switch (channel)
    {
#ifdef CONFIG_SAMA5_PWM_CHAN0
      case 0:
        /* Select the Channel 0 interface */

        chan = &g_pwm_chan0;
        break;
#endif

#ifdef CONFIG_SAMA5_PWM_CHAN1
      case 1:
        /* Select the Channel 1 interface */

        chan = &g_pwm_chan1;
        break;
#endif

#ifdef CONFIG_SAMA5_PWM_CHAN2
      case 2:
        /* Select the Channel 2 interface */

        chan = &g_pwm_chan2;
        break;
#endif

#ifdef CONFIG_SAMA5_PWM_CHAN3
      case 3:
        /* Select the Channel 3 interface */

        chan = &g_pwm_chan3;
        break;
#endif

      default:
        pwmerr("ERROR: Channel invalid or not configured: %d\n", channel);
        return NULL;
    }

  /* Have we already performed the one time initialization of the overall
   * PWM peripheral?
   */

  if (!g_pwm.initialized)
    {
#if defined(CONFIG_SAMA5_PWM_CLKA) || defined(CONFIG_SAMA5_PWM_CLKB)
      uint32_t mck;
      unsigned int prelog2;
      unsigned int div;
#endif

      /* Enable the PWM peripheral clock */

      sam_pwm_enableclk();

#if defined(CONFIG_SAMA5_PWM_CLKA) || defined(CONFIG_SAMA5_PWM_CLKB)
      mck      = BOARD_MCK_FREQUENCY;
#endif
#ifdef CONFIG_SAMA5_PWM_CLKA
      /* Set clock A configuration */

      prelog2  = pwm_clk_prescaler_log2(mck, CONFIG_SAMA5_PWM_CLKA_FREQUENCY);
      div      = pwm_clk_divider(mck, CONFIG_SAMA5_PWM_CLKA_FREQUENCY, prelog2);
      regval   = (PWM_CLK_DIVA(div) | PWM_CLK_PREA_DIV(prelog2));
#else
      regval   = 0;
#endif

#ifdef CONFIG_SAMA5_PWM_CLKB
      /* Set clock B configuration */

      prelog2  = pwm_clk_prescaler_log2(mck, CONFIG_SAMA5_PWM_CLKB_FREQUENCY);
      div      = pwm_clk_divider(mck, CONFIG_SAMA5_PWM_CLKA_FREQUENCY, prelog2);
      regval  |= (PWM_CLK_DIVB(div) | PWM_CLK_PREB_DIV(prelog2));
#endif

      pwm_putreg(chan, SAM_PWM_CLK_OFFSET, regval);

      /* Disable all PWM interrupts at the PWM peripheral */

      pwm_putreg(chan, SAM_PWM_IDR1_OFFSET, PWM_INT1_ALL);
      pwm_putreg(chan, SAM_PWM_IDR2_OFFSET, PWM_INT2_ALL);

      /* Attach the PWM interrupt handler */

#ifdef PWM_INTERRUPTS
      ret = irq_attach(SAM_IRQ_PWM, pwm_interrupt, NULL);
      if (ret < 0)
        {
          pwmerr("ERROR: Failed to attach IRQ%d\n", channel);
          return NULL;

        }
#endif

      /* Clear any pending PWM interrupts */

      pwm_getreg(chan, SAM_PWM_ISR1_OFFSET);
      pwm_getreg(chan, SAM_PWM_ISR2_OFFSET);

      /* Enable PWM interrupts at the AIC */

#ifdef PWM_INTERRUPTS
      up_enable_irq(SAM_IRQ_PWM);
#endif

      /* Now were are initialized */

      g_pwm.initialized = true;
      pwm_dumpregs(chan, "After Initialization");
    }

  /* Configure all pins for this channel as inputs */

  pwm_resetpins(chan);

  /* Return the lower-half driver instance for this channel */

  return (FAR struct pwm_lowerhalf_s *)chan;
}

#endif /* CONFIG_SAMA5_PWM */

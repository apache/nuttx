/****************************************************************************
 * arch/mips/src/pic32mz/chip/pic32mz_timer.c
 *
 *   Copyright (C) 2019 Abdelatif Guettouche. All rights reserved.
 *   Author: Abdelatif Guettouche <abdelatif.guettouche@gmail.com>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "mips_internal.h"
#include "mips_arch.h"

#include "hardware/pic32mz_timer.h"
#include "pic32mz_timer.h"
#include "pic32mz_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This module is for the type B timers only.
 * Type A (timer1) is used by the system.
 */

#if defined(CONFIG_PIC32MZ_T2) || defined(CONFIG_PIC32MZ_T3) || \
    defined(CONFIG_PIC32MZ_T4) || defined(CONFIG_PIC32MZ_T5) || \
    defined(CONFIG_PIC32MZ_T6) || defined(CONFIG_PIC32MZ_T7) || \
    defined(CONFIG_PIC32MZ_T8) || defined(CONFIG_PIC32MZ_T9)

/* Undef odd timers if mode32 is enabled in even timers.
 * If mode32 is enabled the consecutive odd timers will be automatically
 * enabled through menuconfig.
 * This is an indication for the user that the odd timer is in use
 * and to avoid a separate, incorrect, utilization.
 * Undef it here so its (useless in this case) structures won't be created
 * later.
 */

#ifdef CONFIG_PIC32MZ_T2_MODE32
#  undef CONFIG_PIC32MZ_T3
#endif

#ifdef CONFIG_PIC32MZ_T4_MODE32
#  undef CONFIG_PIC32MZ_T5
#endif

#ifdef CONFIG_PIC32MZ_T6_MODE32
#  undef CONFIG_PIC32MZ_T7
#endif

#ifdef CONFIG_PIC32MZ_T8_MODE32
#  undef CONFIG_PIC32MZ_T9
#endif

/* Prescale values */

#define  PIC32MZ_TIMER_PRESCALE_1_1    0
#define  PIC32MZ_TIMER_PRESCALE_1_2    1
#define  PIC32MZ_TIMER_PRESCALE_1_4    2
#define  PIC32MZ_TIMER_PRESCALE_1_8    3
#define  PIC32MZ_TIMER_PRESCALE_1_16   4
#define  PIC32MZ_TIMER_PRESCALE_1_32   5
#define  PIC32MZ_TIMER_PRESCALE_1_64   6
#define  PIC32MZ_TIMER_PRESCALE_1_256  7

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Timer's hardware configuration */

struct pic32mz_timer_config_s
{
  uint32_t  base;        /* Timer's base address       */
  uint32_t  irq;         /* Timer's event irq          */
  bool      stopinidle;  /* True: stop in idle         */
  bool      gated;       /* True: enable gated mode    */
  uint8_t   prescale;    /* Timer's prescale value     */
  bool      mode32;      /* True: enable mode 32bit    */
  bool      extclock;    /* True: use external clock   */
  uint32_t  tckreg;      /* Timer's clock PPS register */
  uint8_t   tckpps;      /* Timer's PPS value          */
  uint32_t  tckpin;      /* GPIO config for the input  */
};

/* Timer's Device Structure */

struct pic32mz_timer_priv_s
{
  const struct pic32mz_timer_ops_s *ops;
  struct pic32mz_timer_config_s *config;
  bool inuse;
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

/* Helpers */

static inline uint32_t pic32mz_getreg(FAR struct pic32mz_timer_dev_s *dev,
                                      uint16_t offset);
static inline void pic32mz_putreg(FAR struct pic32mz_timer_dev_s *dev,
                                  uint16_t offset, uint32_t value);
static inline bool pic32mz_timer_mode32(FAR struct pic32mz_timer_dev_s *dev);
static inline uint32_t pic32mz_timer_oddoffset(uint32_t evenoffset);
static inline uint32_t
  pic32mz_timer_nextirq(FAR struct pic32mz_timer_dev_s *dev);

static void pic32mz_timer_stopinidle(FAR struct pic32mz_timer_dev_s *dev,
                                    bool stop);
static void pic32mz_timer_enablegate(FAR struct pic32mz_timer_dev_s *dev,
                                     bool enable);
static void pic32mz_timer_setprescale(FAR struct pic32mz_timer_dev_s *dev,
                                      uint8_t prescale);
static void pic32mz_timer_setmode32(FAR struct pic32mz_timer_dev_s *dev,
                                    bool enable);
static void pic32mz_timer_extclocksource(FAR struct pic32mz_timer_dev_s *dev,
                                         bool enable);
static void pic32mz_timer_inithardware(FAR struct pic32mz_timer_dev_s *dev);

/* Timer's methods */

static void pic32mz_timer_start(FAR struct pic32mz_timer_dev_s *dev);
static void pic32mz_timer_stop(FAR struct pic32mz_timer_dev_s *dev);
static void pic32mz_timer_setperiod(FAR struct pic32mz_timer_dev_s *dev,
                                    uint32_t period);
static uint32_t pic32mz_timer_getcounter(FAR struct pic32mz_timer_dev_s *dev);
static void pic32mz_timer_setcounter(FAR struct pic32mz_timer_dev_s *dev,
                                     uint32_t count);
static uint32_t pic32mz_timer_getfreq(FAR struct pic32mz_timer_dev_s *dev);
static bool pic32mz_timer_setfreq(FAR struct pic32mz_timer_dev_s *dev,
                                  uint32_t freq);
static uint8_t pic32mz_timer_getwidth(FAR struct pic32mz_timer_dev_s *dev);

static int  pic32mz_timer_setisr(FAR struct pic32mz_timer_dev_s *dev,
                                 xcpt_t handler, void *arg);
static void pic32mz_timer_ackint(FAR struct pic32mz_timer_dev_s *dev);
static bool  pic32mz_timer_checkint(FAR struct pic32mz_timer_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pic32mz_timer_ops_s pic32mz_timer_ops =
{
  .start       = pic32mz_timer_start,
  .stop        = pic32mz_timer_stop,
  .setperiod   = pic32mz_timer_setperiod,
  .getcounter  = pic32mz_timer_getcounter,
  .setcounter  = pic32mz_timer_setcounter,
  .getfreq     = pic32mz_timer_getfreq,
  .setfreq     = pic32mz_timer_setfreq,
  .getwidth    = pic32mz_timer_getwidth,

  .setisr      = pic32mz_timer_setisr,
  .ackint      = pic32mz_timer_ackint,
  .checkint    = pic32mz_timer_checkint,
};

#ifdef CONFIG_PIC32MZ_T2
static struct pic32mz_timer_config_s pic32mz_timer_t2_config =
{
  .base       = PIC32MZ_TIMER2_K1BASE,
  .irq        = PIC32MZ_IRQ_T2,
#ifdef CONFIG_PIC32MZ_T2_STOPINIDLE
  .stopinidle = true,
#else
  .stopinidle = false,
#endif
#ifdef CONFIG_PIC32MZ_T2_GATED
  .gated     = true,
  .tckreg    = PIC32MZ_T2CKR,
  .tckpps    = BOARD_T2CK_PPS,
  .tckpin    = GPIO_T2CK,
#else
  .gated     = false,
#endif
  .prescale  = CONFIG_PIC32MZ_T2_PRESCALE,
#ifdef CONFIG_PIC32MZ_T2_MODE32
  .mode32    = true,
#else
  .mode32    = false,
#endif
#ifdef CONFIG_PIC32MZ_T2_EXTERNALCLOCK
  .extclock  = true,
  .tckreg    = PIC32MZ_T2CKR,
  .tckpps    = BOARD_T2CK_PPS,
  .tckpin    = GPIO_T2CK,
#else
  .extclock  = false,
#endif
};

static struct pic32mz_timer_priv_s pic32mz_t2_priv =
{
  .ops        = &pic32mz_timer_ops,
  .config     = &pic32mz_timer_t2_config,
  .inuse      = false,
};
#endif

#ifdef CONFIG_PIC32MZ_T3
static struct pic32mz_timer_config_s pic32mz_timer_t3_config =
{
  .base       = PIC32MZ_TIMER3_K1BASE,
  .irq        = PIC32MZ_IRQ_T3,
#ifdef CONFIG_PIC32MZ_T3_STOPINIDLE
  .stopinidle = true,
#else
  .stopinidle = false,
#endif
#ifdef CONFIG_PIC32MZ_T3_GATED
  .gated     = true,
  .tckreg    = PIC32MZ_T3CKR,
  .tckpps    = BOARD_T3CK_PPS,
  .tckpin    = GPIO_T3CK,
#else
  .gated     = false,
#endif
  .prescale  = CONFIG_PIC32MZ_T3_PRESCALE,
  .mode32    = false,
#ifdef CONFIG_PIC32MZ_T3_EXTERNALCLOCK
  .extclock  = true,
  .tckreg    = PIC32MZ_T3CKR,
  .tckpps    = BOARD_T3CK_PPS,
  .tckpin    = GPIO_T3CK,
#else
  .extclock  = false,
#endif
};

static struct pic32mz_timer_priv_s pic32mz_t3_priv =
{
  .ops        = &pic32mz_timer_ops,
  .config     = &pic32mz_timer_t3_config,
  .inuse      = false,
};
#endif

#ifdef CONFIG_PIC32MZ_T4
static struct pic32mz_timer_config_s pic32mz_timer_t4_config =
{
  .base       = PIC32MZ_TIMER4_K1BASE,
  .irq        = PIC32MZ_IRQ_T4,
#ifdef CONFIG_PIC32MZ_T4_STOPINIDLE
  .stopinidle = true,
#else
  .stopinidle = false,
#endif
#ifdef CONFIG_PIC32MZ_T4_GATED
  .gated     = true,
  .tckreg    = PIC32MZ_T4CKR,
  .tckpps    = BOARD_T4CK_PPS,
  .tckpin    = GPIO_T4CK,
#else
  .gated     = false,
#endif
  .prescale  = CONFIG_PIC32MZ_T4_PRESCALE,
#ifdef CONFIG_PIC32MZ_T4_MODE32
  .mode32    = true,
#else
  .mode32    = false,
#endif
#ifdef CONFIG_PIC32MZ_T4_EXTERNALCLOCK
  .extclock  = true,
  .tckreg    = PIC32MZ_T4CKR,
  .tckpps    = BOARD_T4CK_PPS,
  .tckpin    = GPIO_T4CK,
#else
  .extclock  = false,
#endif
};

static struct pic32mz_timer_priv_s pic32mz_t4_priv =
{
  .ops        = &pic32mz_timer_ops,
  .config     = &pic32mz_timer_t4_config,
  .inuse      = false,
};
#endif

#ifdef CONFIG_PIC32MZ_T5
static struct pic32mz_timer_config_s pic32mz_timer_t5_config =
{
  .base       = PIC32MZ_TIMER5_K1BASE,
  .irq        = PIC32MZ_IRQ_T5,
#ifdef CONFIG_PIC32MZ_T5_STOPINIDLE
  .stopinidle = true,
#else
  .stopinidle = false,
#endif
#ifdef CONFIG_PIC32MZ_T5_GATED
  .gated     = true,
  .tckreg    = PIC32MZ_T5CKR,
  .tckpps    = BOARD_T5CK_PPS,
  .tckpin    = GPIO_T5CK,
#else
  .gated     = false,
#endif
  .prescale  = CONFIG_PIC32MZ_T5_PRESCALE,
  .mode32    = false,
#ifdef CONFIG_PIC32MZ_T5_EXTERNALCLOCK
  .extclock  = true,
  .tckreg    = PIC32MZ_T5CKR,
  .tckpps    = BOARD_T5CK_PPS,
  .tckpin    = GPIO_T5CK,
#else
  .extclock  = false,
#endif
};

static struct pic32mz_timer_priv_s pic32mz_t5_priv =
{
  .ops        = &pic32mz_timer_ops,
  .config     = &pic32mz_timer_t5_config,
  .inuse      = false,
};
#endif

#ifdef CONFIG_PIC32MZ_T6
static struct pic32mz_timer_config_s pic32mz_timer_t6_config =
{
  .base       = PIC32MZ_TIMER6_K1BASE,
  .irq        = PIC32MZ_IRQ_T6,
#ifdef CONFIG_PIC32MZ_T6_STOPINIDLE
  .stopinidle = true,
#else
  .stopinidle = false,
#endif
#ifdef CONFIG_PIC32MZ_T6_GATED
  .gated     = true,
  .tckreg    = PIC32MZ_T6CKR,
  .tckpps    = BOARD_T6CK_PPS,
  .tckpin    = GPIO_T6CK,
#else
  .gated     = false,
#endif
  .prescale  = CONFIG_PIC32MZ_T6_PRESCALE,
#ifdef CONFIG_PIC32MZ_T6_MODE32
  .mode32    = true,
#else
  .mode32    = false,
#endif
#ifdef CONFIG_PIC32MZ_T6_EXTERNALCLOCK
  .extclock  = true,
  .tckreg    = PIC32MZ_T6CKR,
  .tckpps    = BOARD_T6CK_PPS,
  .tckpin    = GPIO_T6CK,
#else
  .extclock  = false,
#endif
};

static struct pic32mz_timer_priv_s pic32mz_t6_priv =
{
  .ops        = &pic32mz_timer_ops,
  .config     = &pic32mz_timer_t6_config,
  .inuse      = false,
};
#endif

#ifdef CONFIG_PIC32MZ_T7
static struct pic32mz_timer_config_s pic32mz_timer_t7_config =
{
  .base       = PIC32MZ_TIMER7_K1BASE,
  .irq        = PIC32MZ_IRQ_T7,
#ifdef CONFIG_PIC32MZ_T7_STOPINIDLE
  .stopinidle = true,
#else
  .stopinidle = false,
#endif
#ifdef CONFIG_PIC32MZ_T7_GATED
  .gated     = true,
  .tckreg    = PIC32MZ_T7CKR,
  .tckpps    = BOARD_T7CK_PPS,
  .tckpin    = GPIO_T7CK,
#else
  .gated     = false,
#endif
  .prescale  = CONFIG_PIC32MZ_T7_PRESCALE,
  .mode32    = false,
#ifdef CONFIG_PIC32MZ_T7_EXTERNALCLOCK
  .extclock  = true,
  .tckreg    = PIC32MZ_T7CKR,
  .tckpps    = BOARD_T7CK_PPS,
  .tckpin    = GPIO_T7CK,
#else
  .extclock  = false,
#endif
};

static struct pic32mz_timer_priv_s pic32mz_t7_priv =
{
  .ops        = &pic32mz_timer_ops,
  .config     = &pic32mz_timer_t7_config,
  .inuse      = false,
};
#endif

#ifdef CONFIG_PIC32MZ_T8
static struct pic32mz_timer_config_s pic32mz_timer_t8_config =
{
  .base       = PIC32MZ_TIMER8_K1BASE,
  .irq        = PIC32MZ_IRQ_T8,
#ifdef CONFIG_PIC32MZ_T8_STOPINIDLE
  .stopinidle = true,
#else
  .stopinidle = false,
#endif
#ifdef CONFIG_PIC32MZ_T8_GATED
  .gated     = true,
  .tckreg    = PIC32MZ_T8CKR,
  .tckpps    = BOARD_T8CK_PPS,
  .tckpin    = GPIO_T8CK,
#else
  .gated     = false,
#endif
  .prescale  = CONFIG_PIC32MZ_T8_PRESCALE,
#ifdef CONFIG_PIC32MZ_T8_MODE32
  .mode32    = true,
#else
  .mode32    = false,
#endif
#ifdef CONFIG_PIC32MZ_T8_EXTERNALCLOCK
  .extclock  = true,
  .tckreg    = PIC32MZ_T8CKR,
  .tckpps    = BOARD_T8CK_PPS,
  .tckpin    = GPIO_T8CK,
#else
  .extclock  = false,
#endif
};

static struct pic32mz_timer_priv_s pic32mz_t8_priv =
{
  .ops        = &pic32mz_timer_ops,
  .config     = &pic32mz_timer_t8_config,
  .inuse      = false,
};
#endif

#ifdef CONFIG_PIC32MZ_T9
static struct pic32mz_timer_config_s pic32mz_timer_t9_config =
{
  .base       = PIC32MZ_TIMER9_K1BASE,
  .irq        = PIC32MZ_IRQ_T9,
#ifdef CONFIG_PIC32MZ_T9_STOPINIDLE
  .stopinidle = true,
#else
  .stopinidle = false,
#endif
#ifdef CONFIG_PIC32MZ_T9_GATED
  .gated     = true,
  .tckreg    = PIC32MZ_T9CKR,
  .tckpps    = BOARD_T9CK_PPS,
  .tckpin    = GPIO_T9CK,
#else
  .gated     = false,
#endif
  .prescale  = CONFIG_PIC32MZ_T9_PRESCALE,
  .mode32    = false,
#ifdef CONFIG_PIC32MZ_T9_EXTERNALCLOCK
  .extclock  = true,
  .tckreg    = PIC32MZ_T9CKR,
  .tckpps    = BOARD_T9CK_PPS,
  .tckpin    = GPIO_T9CK,
#else
  .extclock  = false,
#endif
};

static struct pic32mz_timer_priv_s pic32mz_t9_priv =
{
  .ops        = &pic32mz_timer_ops,
  .config     = &pic32mz_timer_t9_config,
  .inuse      = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t pic32mz_getreg(FAR struct pic32mz_timer_dev_s *dev,
                                      uint16_t offset)
{
  FAR struct pic32mz_timer_priv_s *priv =
    (FAR struct pic32mz_timer_priv_s *)dev;

  return getreg32(priv->config->base + offset);
}

/****************************************************************************
 * Name: pic32mz_putreg
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void pic32mz_putreg(FAR struct pic32mz_timer_dev_s *dev,
                                  uint16_t offset, uint32_t value)
{
  FAR struct pic32mz_timer_priv_s *priv =
    (FAR struct pic32mz_timer_priv_s *)dev;

  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: pic32mz_timer_mode32
 *
 * Description:
 *  Returns true if the 32 mode is enabled
 *
 ****************************************************************************/

static inline bool pic32mz_timer_mode32(FAR struct pic32mz_timer_dev_s *dev)
{
  return ((FAR struct pic32mz_timer_priv_s *)dev)->config->mode32;
}

/****************************************************************************
 * Name: pic32mz_timer_oddoffset
 *
 * Description:
 *  Returns the offset of the consecutive odd timer
 *
 ****************************************************************************/

static inline uint32_t pic32mz_timer_oddoffset(uint32_t evenoffset)
{
  /* To access the consecutive odd timer the base needs be changed.
   * PIC32MZ_TIMERn_OFFSET(1) represents the offset between timers' base.
   * An even timer's base + PIC32MZ_TIMERn_OFFSET(1) gives the base of
   * the next odd timer.
   * This will allow the access of the odd timer from the dev of its
   * previous even timer.
   */

  return PIC32MZ_TIMERn_OFFSET(1) + evenoffset;
}

/****************************************************************************
 * Name: pic32mz_timer_nextirq
 *
 * Description:
 *  Returns the irq of the consecutive odd numbered timer
 *
 ****************************************************************************/

static inline uint32_t
  pic32mz_timer_nextirq(FAR struct pic32mz_timer_dev_s *dev)
{
  uint32_t irq;

  irq = ((FAR struct pic32mz_timer_priv_s *)dev)->config->irq;

  /* The irq offsets between odd and even timers
   * are not always the same.
   */

  if (irq == PIC32MZ_IRQ_T2 || irq == PIC32MZ_IRQ_T4)
    {
      return irq + 5;
    }
  else
    {
      return irq + 4;
    }
}

/****************************************************************************
 * Name: pic32mz_timer_start
 *
 * Description:
 *   Start the timer by setting its ON bit
 *
 ****************************************************************************/

static void pic32mz_timer_start(FAR struct pic32mz_timer_dev_s *dev)
{
  pic32mz_putreg(dev, PIC32MZ_TIMER_CONSET_OFFSET, TIMER_CON_ON);
}

/****************************************************************************
 * Name: pic32mz_timer_stop
 *
 * Description:
 *   Stop the timer by clearing the ON bit
 *
 ****************************************************************************/

static void pic32mz_timer_stop(FAR struct pic32mz_timer_dev_s *dev)
{
  pic32mz_putreg(dev, PIC32MZ_TIMER_CONCLR_OFFSET, TIMER_CON_ON);
}

/****************************************************************************
 * Name: pic32mz_timer_stopinidle
 *
 * Description:
 *   Stop the timer in idle mode
 *
 ****************************************************************************/

static void pic32mz_timer_stopinidle(FAR struct pic32mz_timer_dev_s *dev,
                                     bool stop)
{
  if (stop)
    {
      pic32mz_putreg(dev, PIC32MZ_TIMER_CONSET_OFFSET, TIMER_CON_SIDL);
    }
  else
    {
      pic32mz_putreg(dev, PIC32MZ_TIMER_CONCLR_OFFSET, TIMER_CON_SIDL);

      /* In 32bit mode, this bit should be cleared
       * for the odd timer as well.
       */

      if (pic32mz_timer_mode32(dev))
        {
          pic32mz_putreg(dev,
                        pic32mz_timer_oddoffset(PIC32MZ_TIMER_CONCLR_OFFSET),
                        TIMER_CON_SIDL);
        }
    }

  ((FAR struct pic32mz_timer_priv_s *)dev)->config->stopinidle = stop;
}

/****************************************************************************
 * Name: pic32mz_timer_enablegate
 *
 * Description:
 *   Enabled gated time accumulation.
 *   This has no effect when external clock source is enabled
 *
 ****************************************************************************/

static void pic32mz_timer_enablegate(FAR struct pic32mz_timer_dev_s *dev,
                                     bool enable)
{
  FAR struct pic32mz_timer_priv_s *priv =
    (FAR struct pic32mz_timer_priv_s *)dev;

  if (enable)
    {
      pic32mz_putreg(dev, PIC32MZ_TIMER_CONSET_OFFSET, TIMER_CON_TGATE);

      /* Configure the TxCK input pin */

      putreg32(priv->config->tckpps, priv->config->tckreg);
      pic32mz_configgpio(priv->config->tckpin);
    }
  else
    {
      pic32mz_putreg(dev, PIC32MZ_TIMER_CONCLR_OFFSET, TIMER_CON_TGATE);
    }

  priv->config->gated = enable;
}

/****************************************************************************
 * Name: pic32mz_timer_setprescale
 *
 * Description:
 *   Set the timer's prescaler.
 *
 ****************************************************************************/

static void pic32mz_timer_setprescale(FAR struct pic32mz_timer_dev_s *dev,
                                      uint8_t prescale)
{
  pic32mz_putreg(dev, PIC32MZ_TIMER_CONSET_OFFSET,
                 (prescale << TIMER_CON_TCKPS_SHIFT));

  ((FAR struct pic32mz_timer_priv_s *)dev)->config->prescale = prescale;
}

/****************************************************************************
 * Name: pic32mz_timer_setmode32
 *
 * Description:
 *   Set the timer's mode.
 *   Two 16-bit timers can form a 32-bit timer.
 *
 ****************************************************************************/

static void pic32mz_timer_setmode32(FAR struct pic32mz_timer_dev_s *dev,
                                    bool enable)
{
  /* Only even timers have the TIMER_CON_T32 bit. */

  if (enable)
    {
      pic32mz_putreg(dev, PIC32MZ_TIMER_CONSET_OFFSET, TIMER_CON_T32);
    }
  else
    {
      pic32mz_putreg(dev, PIC32MZ_TIMER_CONCLR_OFFSET, TIMER_CON_T32);
    }

  ((FAR struct pic32mz_timer_priv_s *)dev)->config->mode32 = enable;
}

/****************************************************************************
 * Name: pic32mz_timer_extclocksource
 *
 * Description:
 *   Set the timer's clock source
 *
 ****************************************************************************/

static void pic32mz_timer_extclocksource(FAR struct pic32mz_timer_dev_s *dev,
                                         bool enable)
{
  FAR struct pic32mz_timer_priv_s *priv =
    (FAR struct pic32mz_timer_priv_s *)dev;

  if (enable)
    {
      pic32mz_putreg(dev, PIC32MZ_TIMER_CONSET_OFFSET, TIMER_CON_TCS);

      /* Configure the TxCK input pin */

      putreg32(priv->config->tckpps, priv->config->tckreg);
      pic32mz_configgpio(priv->config->tckpin);
    }
  else
    {
      pic32mz_putreg(dev, PIC32MZ_TIMER_CONCLR_OFFSET, TIMER_CON_TCS);
    }

  priv->config->extclock = enable;
}

/****************************************************************************
 * Name: pic32mz_timer_inithardware
 *
 * Description:
 *   Initializes the timer's hardware
 *   This function is only called when the timer is first initialized.
 *   It uses the provided configuration.
 *
 ****************************************************************************/

static void pic32mz_timer_inithardware(FAR struct pic32mz_timer_dev_s *dev)
{
  FAR struct pic32mz_timer_priv_s *priv =
    (FAR struct pic32mz_timer_priv_s *)dev;

  /* Initialize the hardware using the startup configuration.
   *
   * Set idle mode operations.
   */

  pic32mz_timer_stopinidle(dev, priv->config->stopinidle);

  /* Enable gated time accumulation if chosen.
   * This operation is ignored if an external clock is used.
   */

  pic32mz_timer_enablegate(dev, priv->config->gated);

  /* Set the input clock prescale
   */

  pic32mz_timer_setprescale(dev, priv->config->prescale);

  /* Set the timer's mode (16 or 32bits)
   * This bit only exists for type B even numbered timers (Timer 2, 4, 6, 8)
   */

  pic32mz_timer_setmode32(dev, priv->config->mode32);

  /* Set the clock source.
   * If an external clock source is used, the timer is driven from TxCK pin.
   */

  pic32mz_timer_extclocksource(dev, priv->config->extclock);
}

/****************************************************************************
 * Name: pic32mz_timer_setperiod
 *
 * Description:
 *   Set the PRx register
 *
 ****************************************************************************/

static void pic32mz_timer_setperiod(FAR struct pic32mz_timer_dev_s *dev,
                                    uint32_t period)
{
  /* In 32bit mode:
   *  - even timers represent the least significant half words.
   *  - odd timers represent the most significant half words.
   */

  if (pic32mz_timer_mode32(dev))
    {
      pic32mz_putreg(dev, PIC32MZ_TIMER_PR_OFFSET, period & 0x00000ffff);

      pic32mz_putreg(dev, pic32mz_timer_oddoffset(PIC32MZ_TIMER_PR_OFFSET),
                     (period >> 16) & 0x00000ffff);
    }
  else
    {
      pic32mz_putreg(dev, PIC32MZ_TIMER_PR_OFFSET, period);
    }
}

/****************************************************************************
 * Name: pic32mz_timer_getcounter
 *
 * Description:
 *   Return the TMRx register
 *
 ****************************************************************************/

static uint32_t pic32mz_timer_getcounter(FAR struct pic32mz_timer_dev_s *dev)
{
  /* In 32bit mode:
   *  - even timers represent the least significant half words.
   *  - odd timers represent the most significant half words.
   */

  if (pic32mz_timer_mode32(dev))
    {
      uint16_t lsw;
      uint16_t msw;

      lsw = pic32mz_getreg(dev, PIC32MZ_TIMER_CNT_OFFSET);

      msw = pic32mz_getreg(dev,
                          pic32mz_timer_oddoffset(PIC32MZ_TIMER_CNT_OFFSET));

      return lsw | (msw << 16);
    }
  else
    {
      return pic32mz_getreg(dev, PIC32MZ_TIMER_CNT_OFFSET);
    }
}

/****************************************************************************
 * Name: pic32mz_timer_setcounter
 *
 * Description:
 *   Set the TMRx register
 *
 ****************************************************************************/

static void pic32mz_timer_setcounter(FAR struct pic32mz_timer_dev_s *dev,
                                     uint32_t count)
{
  /* In 32bit mode:
   *  - even timers represent the least significant half words.
   *  - odd timers represent the most significant half words.
   */

  if (pic32mz_timer_mode32(dev))
    {
      pic32mz_putreg(dev, PIC32MZ_TIMER_CNT_OFFSET, count & 0x00000ffff);

      pic32mz_putreg(dev, pic32mz_timer_oddoffset(PIC32MZ_TIMER_CNT_OFFSET),
                     (count >> 16) & 0x00000ffff);
    }
  else
    {
      pic32mz_putreg(dev, PIC32MZ_TIMER_CNT_OFFSET, count);
    }
}

/****************************************************************************
 * Name: pic32mz_timer_getfreq
 *
 * Description:
 *   Returns the frequency of the timer in Hz
 *
 ****************************************************************************/

static uint32_t pic32mz_timer_getfreq(FAR struct pic32mz_timer_dev_s *dev)
{
  uint8_t prescale;
  uint32_t freq;

  prescale = ((FAR struct pic32mz_timer_priv_s *)dev)->config->prescale;

  /* The prescale values are not a continuous power of 2.
   * There is a gap between 64 and 256 (the 128 is skipped).
   */

  if (prescale == PIC32MZ_TIMER_PRESCALE_1_256)
    {
      freq = BOARD_PBCLK3 / (1 << (prescale + 1));
    }
  else
    {
      freq = BOARD_PBCLK3 / (1 << prescale);
    }

  return freq;
}

/****************************************************************************
 * Name: pic32mz_timer_setfreq
 *
 * Description:
 *   Sets the frequency of the timer in Hz
 *
 ****************************************************************************/

static bool pic32mz_timer_setfreq(FAR struct pic32mz_timer_dev_s *dev,
                                  uint32_t freq)
{
  uint16_t prescale;

  DEBUGASSERT(dev != NULL);

  /* Disable Timer? */

  if (freq == 0)
    {
      tmrwarn("Frequency=%luHz not valid", freq);
      pic32mz_timer_stop(dev);

      return 0;
    }

  prescale = BOARD_PBCLK3 / freq;

  tmrinfo("Prescale value calculated %d\n", prescale);

  if (prescale >= 1 && prescale < 2)
    {
      pic32mz_timer_setprescale(dev, PIC32MZ_TIMER_PRESCALE_1_1);
      tmrinfo("Prescale value chosen %d\n",
                (1 << (PIC32MZ_TIMER_PRESCALE_1_1)));
    }
  else if (prescale >= 2 && prescale < 4)
    {
      pic32mz_timer_setprescale(dev, PIC32MZ_TIMER_PRESCALE_1_2);
      tmrinfo("Prescale value chosen %d\n",
                (1 << (PIC32MZ_TIMER_PRESCALE_1_2)));
    }
  else if (prescale >= 4 && prescale < 8)
    {
      pic32mz_timer_setprescale(dev, PIC32MZ_TIMER_PRESCALE_1_4);
      tmrinfo("Prescale value chosen %d\n",
                (1 << (PIC32MZ_TIMER_PRESCALE_1_4)));
    }
  else if (prescale >= 8 && prescale < 16)
    {
      pic32mz_timer_setprescale(dev, PIC32MZ_TIMER_PRESCALE_1_8);
      tmrinfo("Prescale value chosen %d\n",
                (1 << (PIC32MZ_TIMER_PRESCALE_1_8)));
    }
  else if (prescale >= 16 && prescale < 32)
    {
      pic32mz_timer_setprescale(dev, PIC32MZ_TIMER_PRESCALE_1_16);
      tmrinfo("Prescale value chosen %d\n",
                (1 << (PIC32MZ_TIMER_PRESCALE_1_16)));
    }
  else if (prescale >= 32 && prescale < 64)
    {
      pic32mz_timer_setprescale(dev, PIC32MZ_TIMER_PRESCALE_1_32);
      tmrinfo("Prescale value chosen %d\n",
                (1 << (PIC32MZ_TIMER_PRESCALE_1_32)));
    }
  else if (prescale >= 64 && prescale < 256)
    {
      pic32mz_timer_setprescale(dev, PIC32MZ_TIMER_PRESCALE_1_64);
      tmrinfo("Prescale value chosen %d\n",
                (1 << (PIC32MZ_TIMER_PRESCALE_1_64)));
    }
  else if (prescale >= 256)
    {
      pic32mz_timer_setprescale(dev, PIC32MZ_TIMER_PRESCALE_1_256);
      tmrinfo("Prescale value chosen %d\n",
                (1 << (PIC32MZ_TIMER_PRESCALE_1_256 + 1)));
    }
  else
    {
      tmrerr("The frequency of %luHz cannot be set.\n", freq);
      return false;
    }

  tmrinfo("Timer's frequency set to %luHz\n", pic32mz_timer_getfreq(dev));

  return true;
}

/****************************************************************************
 * Name: pic32mz_timer_getwidth
 *
 * Description:
 *   Returns the timer's width
 *
 ****************************************************************************/

static uint8_t pic32mz_timer_getwidth(FAR struct pic32mz_timer_dev_s *dev)
{
  return pic32mz_timer_mode32(dev) ? 32 : 16;
}

/****************************************************************************
 * Name: pic32mz_timer_setisr
 *
 * Description:
 *   Set the timer's ISR
 *
 ****************************************************************************/

static int  pic32mz_timer_setisr(FAR struct pic32mz_timer_dev_s *dev,
                                 xcpt_t handler, FAR void *arg)
{
  FAR struct pic32mz_timer_priv_s *priv =
    (FAR struct pic32mz_timer_priv_s *)dev;

  /* Disable interrupt when callback is removed */

  if (!handler)
    {
      if (pic32mz_timer_mode32(dev))
        {
          /* In 32bit mode, the consecutive odd timer controls
           * the interrupt.
           */

          up_disable_irq(pic32mz_timer_nextirq(dev));
          irq_detach(pic32mz_timer_nextirq(dev));
        }
      else
        {
          up_disable_irq(priv->config->irq);
          irq_detach(priv->config->irq);
        }
    }

  /* Otherwise set callback and enable interrupt */

  else
    {
      if (pic32mz_timer_mode32(dev))
        {
          /* In 32bit mode, the consecutive odd timer controls
           * the interrupt.
           */

          irq_attach(pic32mz_timer_nextirq(dev), handler, arg);
          up_enable_irq(pic32mz_timer_nextirq(dev));
        }
      else
        {
          irq_attach(priv->config->irq, handler, arg);
          up_enable_irq(priv->config->irq);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: pic32mz_timer_ackint
 *
 * Description:
 *   Acknowledge the timer's interrupt
 *
 ****************************************************************************/

static void pic32mz_timer_ackint(FAR struct pic32mz_timer_dev_s *dev)
{
  up_clrpend_irq(((FAR struct pic32mz_timer_priv_s *)dev)->config->irq);

  if (pic32mz_timer_mode32(dev))
    {
      /* In 32bit mode, the consecutive odd timer controls the interrupt. */

      up_clrpend_irq(pic32mz_timer_nextirq(dev));
    }
}

/****************************************************************************
 * Name: pic32mz_timer_checkint
 *
 * Description:
 *   Check if the timer's interrupt is pending
 *
 ****************************************************************************/

static bool  pic32mz_timer_checkint(FAR struct pic32mz_timer_dev_s *dev)
{
  if (pic32mz_timer_mode32(dev))
    {
      /* In 32bit mode, the consecutive odd timer controls the interrupt. */

      return up_pending_irq(pic32mz_timer_nextirq(dev));
    }
  else
    {
      FAR struct pic32mz_timer_priv_s *priv =
          (FAR struct pic32mz_timer_priv_s *)dev;

      return up_pending_irq(priv->config->irq);
    }
}

/****************************************************************************
 * Pubic Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_timer_init
 ****************************************************************************/

FAR struct pic32mz_timer_dev_s *pic32mz_timer_init(int timer)
{
  struct pic32mz_timer_dev_s *dev = NULL;

  switch (timer)
    {
#ifdef CONFIG_PIC32MZ_T2
      case 2:
        dev = (FAR struct pic32mz_timer_dev_s *)&pic32mz_t2_priv;
        break;
#endif
#ifdef CONFIG_PIC32MZ_T3
      case 3:
        dev = (FAR struct pic32mz_timer_dev_s *)&pic32mz_t3_priv;
        break;
#endif
#ifdef CONFIG_PIC32MZ_T4
      case 4:
        dev = (FAR struct pic32mz_timer_dev_s *)&pic32mz_t4_priv;
        break;
#endif
#ifdef CONFIG_PIC32MZ_T5
      case 5:
        dev = (FAR struct pic32mz_timer_dev_s *)&pic32mz_t5_priv;
        break;
#endif
#ifdef CONFIG_PIC32MZ_T6
      case 6:
        dev = (FAR struct pic32mz_timer_dev_s *)&pic32mz_t6_priv;
        break;
#endif
#ifdef CONFIG_PIC32MZ_T7
      case 7:
        dev = (FAR struct pic32mz_timer_dev_s *)&pic32mz_t7_priv;
        break;
#endif
#ifdef CONFIG_PIC32MZ_T8
      case 8:
        dev = (FAR struct pic32mz_timer_dev_s *)&pic32mz_t8_priv;
        break;
#endif
#ifdef CONFIG_PIC32MZ_T9
      case 9:
        dev = (FAR struct pic32mz_timer_dev_s *)&pic32mz_t9_priv;
        break;
#endif
      default:
        return NULL;
    }

  if (((FAR struct pic32mz_timer_priv_s *)dev)->inuse)
    {
      return NULL;
    }
  else
    {
      /* Init the timer's hardware (prescale, clock source, ..) */

      pic32mz_timer_inithardware(dev);

      ((FAR struct pic32mz_timer_priv_s *)dev)->inuse = true;

      return dev;
    }
}

int pic32mz_timer_deinit(FAR struct pic32mz_timer_dev_s *dev)
{
  /* Stop the timer in case it was still running
   * and mark it as unused.
   */

  pic32mz_timer_stop(dev);
  ((FAR struct pic32mz_timer_priv_s *)dev)->inuse = false;

  return OK;
}

#endif /* defined(CONFIG_PIC32MZ_T2 || ... || T9) */

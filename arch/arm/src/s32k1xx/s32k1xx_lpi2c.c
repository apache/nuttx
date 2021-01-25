/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_lpi2c.c
 *
 *   Copyright (C) 2018-2019 Gregory Nutt. All rights reserved.
 *   Author: Ivan Ucherdzhiev <ivanucherdjiev@gmail.com>
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
 *    used to endorse or promote products derived from this softwareGPIO
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
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/irq.h>

#include "arm_arch.h"

#include "s32k1xx_pin.h"
#include "hardware/s32k1xx_pinmux.h"
#include "s32k1xx_lpi2c.h"
#include "s32k1xx_periphclocks.h"

#include <arch/board/board.h>

/* At least one I2C peripheral must be enabled */

#ifdef CONFIG_S32K1XX_LPI2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.
 * Instead, CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_S32K1XX_I2CTIMEOSEC) && !defined(CONFIG_S32K1XX_I2CTIMEOMS)
#  define CONFIG_S32K1XX_I2CTIMEOSEC 0
#  define CONFIG_S32K1XX_I2CTIMEOMS  500   /* Default is 500 milliseconds */
#elif !defined(CONFIG_S32K1XX_I2CTIMEOSEC)
#  define CONFIG_S32K1XX_I2CTIMEOSEC 0     /* User provided milliseconds */
#elif !defined(CONFIG_S32K1XX_I2CTIMEOMS)
#  define CONFIG_S32K1XX_I2CTIMEOMS  0     /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_S32K1XX_I2CTIMEOTICKS
#  define CONFIG_S32K1XX_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_S32K1XX_I2CTIMEOSEC) + MSEC2TICK(CONFIG_S32K1XX_I2CTIMEOMS))
#endif

#ifndef CONFIG_S32K1XX_I2C_DYNTIMEO_STARTSTOP
#  define CONFIG_S32K1XX_I2C_DYNTIMEO_STARTSTOP TICK2USEC(CONFIG_S32K1XX_I2CTIMEOTICKS)
#endif

/* Debug ********************************************************************/

/* I2C event trace logic.  NOTE:  trace uses the internal, non-standard,
 * low-level debug interface syslog() but does not require that any other
 * debug is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define s32k1xx_lpi2c_tracereset(p)
#  define s32k1xx_lpi2c_tracenew(p,s)
#  define s32k1xx_lpi2c_traceevent(p,e,a)
#  define s32k1xx_lpi2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

#ifdef CONFIG_I2C_SLAVE
#  error I2C slave logic is not supported yet for S32K1XX
#endif

#define LPI2C_MASTER    1
#define LPI2C_SLAVE     2

#define MKI2C_OUTPUT(p) (((p) & GPIO_PADMUX_MASK) | \
                         IOMUX_OPENDRAIN | IOMUX_DRIVE_33OHM | \
                         IOMUX_SLEW_SLOW | (5 << GPIO_ALT_SHIFT) | \
                         IOMUX_PULL_NONE | GPIO_OUTPUT_ONE)

#define MKI2C_INPUT(p) (((p) & GPIO_PADMUX_MASK) | \
                        IOMUX_DRIVE_HIZ | IOMUX_SLEW_SLOW | \
                        IOMUX_CMOS_INPUT | (5 << GPIO_ALT_SHIFT) | \
                        IOMUX_PULL_NONE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Interrupt state */

enum s32k1xx_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* Trace events */

enum s32k1xx_trace_e
{
  I2CEVENT_NONE = 0,      /* No events have occurred with this status */
  I2CEVENT_SENDADDR,      /* Start/Master bit set and address sent, param = msgc */
  I2CEVENT_SENDBYTE,      /* Send byte, param = dcnt */
  I2CEVENT_RCVBYTE,       /* Read more dta, param = dcnt */
  I2CEVENT_NOSTART,       /* BTF on last byte with no restart, param = msgc */
  I2CEVENT_STARTRESTART,  /* Last byte sent, re-starting, param = msgc */
  I2CEVENT_STOP,          /* Last byte sten, send stop, param = 0 */
  I2CEVENT_ERROR          /* Error occurred, param = 0 */
};

/* Trace data */

struct s32k1xx_trace_s
{
  uint32_t status;               /* I2C 32-bit SR2|SR1 status */
  uint32_t count;                /* Interrupt count when status change */
  enum s32k1xx_intstate_e event; /* Last event that occurred with this status */
  uint32_t parm;                 /* Parameter associated with the event */
  clock_t time;                  /* First of event or first status */
};

/* I2C Device hardware configuration */

struct s32k1xx_lpi2c_config_s
{
  uint32_t base;              /* LPI2C base address */
  uint16_t busy_idle;         /* LPI2C Bus Idle Timeout */
  uint8_t filtscl;            /* Glitch Filter for SCL pin */
  uint8_t filtsda;            /* Glitch Filter for SDA pin */
  uint32_t scl_pin;           /* GPIO configuration for SCL as SCL */
  uint32_t sda_pin;           /* GPIO configuration for SDA as SDA */
  uint8_t mode;               /* Master or Slave mode */
#ifndef CONFIG_I2C_POLLED
  uint32_t irq;               /* Event IRQ */
#endif
};

/* I2C Device Private Data */

struct s32k1xx_lpi2c_priv_s
{
  /* Standard I2C operations */

  const struct i2c_ops_s *ops;

  /* Port configuration */

  const struct s32k1xx_lpi2c_config_s *config;

  int refs;                    /* Reference count */
  sem_t sem_excl;              /* Mutual exclusion semaphore */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif
  volatile uint8_t intstate;   /* Interrupt handshake (see enum s32k1xx_intstate_e) */

  uint8_t msgc;                /* Message count */
  struct i2c_msg_s *msgv;      /* Message list */
  uint8_t *ptr;                /* Current message buffer */
  uint32_t frequency;          /* Current I2C frequency */
  int dcnt;                    /* Current message length */
  uint16_t flags;              /* Current message flags */

  /* I2C trace support */

#ifdef CONFIG_I2C_TRACE
  int tndx;                    /* Trace array index */
  clock_t start_time;          /* Time when the trace was started */

  /* The actual trace data */

  struct s32k1xx_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  uint32_t status;             /* End of transfer SR2|SR1 status */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t
  s32k1xx_lpi2c_getreg(FAR struct s32k1xx_lpi2c_priv_s *priv,
                       uint16_t offset);
static inline void
  s32k1xx_lpi2c_putreg(FAR struct s32k1xx_lpi2c_priv_s *priv,
                       uint16_t offset, uint32_t value);
static inline void
  s32k1xx_lpi2c_modifyreg(FAR struct s32k1xx_lpi2c_priv_s *priv,
                          uint16_t offset, uint32_t clearbits,
                          uint32_t setbits);
static inline int
  s32k1xx_lpi2c_sem_wait(FAR struct s32k1xx_lpi2c_priv_s *priv);

#ifdef CONFIG_S32K1XX_I2C_DYNTIMEO
static useconds_t
  s32k1xx_lpi2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs);
#endif /* CONFIG_S32K1XX_I2C_DYNTIMEO */

static inline int
  s32k1xx_lpi2c_sem_waitdone(FAR struct s32k1xx_lpi2c_priv_s *priv);
static inline void
  s32k1xx_lpi2c_sem_waitstop(FAR struct s32k1xx_lpi2c_priv_s *priv);
static inline void
  s32k1xx_lpi2c_sem_post(FAR struct s32k1xx_lpi2c_priv_s *priv);
static inline void
  s32k1xx_lpi2c_sem_init(FAR struct s32k1xx_lpi2c_priv_s *priv);
static inline void
  s32k1xx_lpi2c_sem_destroy(FAR struct s32k1xx_lpi2c_priv_s *priv);

#ifdef CONFIG_I2C_TRACE
static void s32k1xx_lpi2c_tracereset(FAR struct s32k1xx_lpi2c_priv_s *priv);
static void s32k1xx_lpi2c_tracenew(FAR struct s32k1xx_lpi2c_priv_s *priv,
                                 uint32_t status);
static void s32k1xx_lpi2c_traceevent(FAR struct s32k1xx_lpi2c_priv_s *priv,
                                     enum s32k1xx_trace_e event,
                                     uint32_t parm);
static void s32k1xx_lpi2c_tracedump(FAR struct s32k1xx_lpi2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

static uint32_t s32k1xx_lpi2c_pckfreq(uintptr_t base);
static void s32k1xx_lpi2c_setclock(FAR struct s32k1xx_lpi2c_priv_s *priv,
                               uint32_t frequency);
static inline void
  s32k1xx_lpi2c_sendstart(FAR struct s32k1xx_lpi2c_priv_s *priv,
                          uint8_t address);
static inline void
  s32k1xx_lpi2c_sendstop(FAR struct s32k1xx_lpi2c_priv_s *priv);
static inline uint32_t
  s32k1xx_lpi2c_getstatus(FAR struct s32k1xx_lpi2c_priv_s *priv);

static int s32k1xx_lpi2c_isr_process(struct s32k1xx_lpi2c_priv_s * priv);

#ifndef CONFIG_I2C_POLLED
static int s32k1xx_lpi2c_isr(int irq, void *context, FAR void *arg);
#endif /* !CONFIG_I2C_POLLED */

static int s32k1xx_lpi2c_init(FAR struct s32k1xx_lpi2c_priv_s *priv);
static int s32k1xx_lpi2c_deinit(FAR struct s32k1xx_lpi2c_priv_s *priv);
static int s32k1xx_lpi2c_transfer(FAR struct i2c_master_s *dev,
                                FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int s32k1xx_lpi2c_reset(FAR struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Trace events strings */

#ifdef CONFIG_I2C_TRACE
static const char *g_trace_names[] =
{
  "NONE           ",
  "SENDADDR       ",
  "SENDBYTE       ",
  "RCVBYTE        ",
  "NOSTART        ",
  "START/RESTART  ",
  "STOP           ",
  "ERROR          "
};
#endif

/* I2C interface */

static const struct i2c_ops_s s32k1xx_lpi2c_ops =
{
  .transfer = s32k1xx_lpi2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = s32k1xx_lpi2c_reset
#endif
};

/* I2C device structures */

#ifdef CONFIG_S32K1XX_LPI2C0
static const struct s32k1xx_lpi2c_config_s s32k1xx_lpi2c0_config =
{
  .base       = S32K1XX_LPI2C0_BASE,
  .busy_idle  = CONFIG_LPI2C0_BUSYIDLE,
  .filtscl    = CONFIG_LPI2C0_FILTSCL,
  .filtsda    = CONFIG_LPI2C0_FILTSDA,
  .scl_pin    = PIN_LPI2C0_SCL,
  .sda_pin    = PIN_LPI2C0_SDA,
#ifndef CONFIG_I2C_SLAVE
  .mode       = LPI2C_MASTER,
#else
  .mode       = LPI2C_SLAVE,
#endif
#ifndef CONFIG_I2C_POLLED
  .irq        = S32K1XX_IRQ_LPI2C0M,
#endif
};

static struct s32k1xx_lpi2c_priv_s s32k1xx_lpi2c0_priv =
{
  .ops        = &s32k1xx_lpi2c_ops,
  .config     = &s32k1xx_lpi2c0_config,
  .refs       = 0,
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0
};
#endif

#ifdef CONFIG_S32K1XX_LPI2C1
static const struct s32k1xx_lpi2c_config_s s32k1xx_lpi2c1_config =
{
  .base       = S32K1XX_LPI2C1_BASE,
  .busy_idle  = CONFIG_LPI2C1_BUSYIDLE,
  .filtscl    = CONFIG_LPI2C1_FILTSCL,
  .filtsda    = CONFIG_LPI2C1_FILTSDA,
  .scl_pin    = PIN_LPI2C1_SCL,
  .sda_pin    = PIN_LPI2C1_SDA,
#ifndef CONFIG_I2C_SLAVE
  .mode       = LPI2C_MASTER,
#else
  .mode       = LPI2C_SLAVE,
#endif
#ifndef CONFIG_I2C_POLLED
  .irq        = S32K1XX_IRQ_LPI2C1M,
#endif
};

static struct s32k1xx_lpi2c_priv_s s32k1xx_lpi2c1_priv =
{
  .ops        = &s32k1xx_lpi2c_ops,
  .config     = &s32k1xx_lpi2c1_config,
  .refs       = 0,
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_lpi2c_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t
  s32k1xx_lpi2c_getreg(FAR struct s32k1xx_lpi2c_priv_s *priv,
                       uint16_t offset)
{
  return getreg32(priv->config->base + offset);
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_putreg
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void
  s32k1xx_lpi2c_putreg(FAR struct s32k1xx_lpi2c_priv_s *priv,
                       uint16_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_modifyreg
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void
  s32k1xx_lpi2c_modifyreg(FAR struct s32k1xx_lpi2c_priv_s *priv,
                          uint16_t offset, uint32_t clearbits,
                          uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_sem_wait
 *
 * Description:
 *   Take the exclusive access, waiting as necessary.  May be interrupted by
 *   a signal.
 *
 ****************************************************************************/

static inline int
  s32k1xx_lpi2c_sem_wait(FAR struct s32k1xx_lpi2c_priv_s *priv)
{
  return nxsem_wait(&priv->sem_excl);
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_tousecs
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be
 *   processed.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_I2C_DYNTIMEO
static useconds_t s32k1xx_lpi2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs)
{
  size_t bytecount = 0;
  int i;

  /* Count the number of bytes left to process */

  for (i = 0; i < msgc; i++)
    {
      bytecount += msgs[i].length;
    }

  /* Then return a number of microseconds based on a user provided scaling
   * factor.
   */

  return (useconds_t)(CONFIG_S32K1XX_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/****************************************************************************
 * Name: s32k1xx_lpi2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int
  s32k1xx_lpi2c_sem_waitdone(FAR struct s32k1xx_lpi2c_priv_s *priv)
{
  struct timespec abstime;
  irqstate_t flags;
  uint32_t regval;
  int ret;

  flags = enter_critical_section();

  /* Enable Interrupts when master mode */

  if (priv->config->mode == LPI2C_MASTER)
    {
      if ((priv->flags & I2C_M_READ) != 0)
        {
          regval = LPI2C_MIER_TDIE | LPI2C_MIER_RDIE | LPI2C_MIER_NDIE | \
                   LPI2C_MIER_ALIE | LPI2C_MIER_SDIE;
          s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MIER_OFFSET, regval);
        }
      else
        {
          regval = LPI2C_MIER_TDIE | LPI2C_MIER_NDIE | \
                   LPI2C_MIER_ALIE | LPI2C_MIER_SDIE;
          s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MIER_OFFSET, regval);
        }
    }

  /* Enable Interrupts when slave mode */

  else
    {
#warning Missing logic for I2C Slave mode
    }

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * nxsem_timedwait() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;
  do
    {
      /* Get the current time */

      clock_gettime(CLOCK_REALTIME, &abstime);

      /* Calculate a time in the future */

#if CONFIG_S32K1XX_I2CTIMEOSEC > 0
      abstime.tv_sec += CONFIG_S32K1XX_I2CTIMEOSEC;
#endif

      /* Add a value proportional to the number of bytes in the transfer */

#ifdef CONFIG_S32K1XX_I2C_DYNTIMEO
      abstime.tv_nsec +=
        1000 * s32k1xx_lpi2c_tousecs(priv->msgc, priv->msgv);

      if (abstime.tv_nsec >= 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }

#elif CONFIG_S32K1XX_I2CTIMEOMS > 0
      abstime.tv_nsec += CONFIG_S32K1XX_I2CTIMEOMS * 1000 * 1000;
      if (abstime.tv_nsec >= 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }
#endif

      /* Wait until either the transfer is complete or the timeout expires */

      ret = nxsem_timedwait_uninterruptible(&priv->sem_isr, &abstime);
      if (ret < 0)
        {
          /* Break out of the loop on irrecoverable errors.  This would
           * include timeouts and mystery errors reported by nxsem_timedwait.
           */

          break;
        }
    }

  /* Loop until the interrupt level transfer is complete. */

  while (priv->intstate != INTSTATE_DONE);

  /* Set the interrupt state back to IDLE */

  priv->intstate = INTSTATE_IDLE;

  /* Disable I2C interrupts */

  /* Enable Interrupts when master mode */

  if (priv->config->mode == LPI2C_MASTER)
    {
      s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MIER_OFFSET, 0);
    }

  /* Enable Interrupts when slave mode */

  else
    {
      s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_SIER_OFFSET, 0);
    }

  leave_critical_section(flags);
  return ret;
}
#else
static inline int
  s32k1xx_lpi2c_sem_waitdone(FAR struct s32k1xx_lpi2c_priv_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Get the timeout value */

#ifdef CONFIG_S32K1XX_I2C_DYNTIMEO
  timeout = USEC2TICK(s32k1xx_lpi2c_tousecs(priv->msgc, priv->msgv));
#else
  timeout = CONFIG_S32K1XX_I2CTIMEOTICKS;
#endif

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * nxsem_timedwait() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;
  start = clock_systime_ticks();

  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;

      /* Poll by simply calling the timer interrupt handler until it
       * reports that it is done.
       */

      s32k1xx_lpi2c_isr_process(priv);
    }

  /* Loop until the transfer is complete. */

  while (priv->intstate != INTSTATE_DONE && elapsed < timeout);

  i2cinfo("intstate: %d elapsed: %ld threshold: %ld status: %08x\n",
          priv->intstate, (long)elapsed, (long)timeout, priv->status);

  /* Set the interrupt state back to IDLE */

  ret = priv->intstate == INTSTATE_DONE ? OK : -ETIMEDOUT;
  priv->intstate = INTSTATE_IDLE;
  return ret;
}
#endif

/****************************************************************************
 * Name: s32k1xx_lpi2c_sem_waitstop
 *
 * Description:
 *   Wait for a STOP to complete
 *
 ****************************************************************************/

static inline void
  s32k1xx_lpi2c_sem_waitstop(FAR struct s32k1xx_lpi2c_priv_s *priv)
{
  clock_t start;
  clock_t elapsed;
  clock_t timeout;
  uint32_t regval;

  /* Select a timeout */

#ifdef CONFIG_S32K1XX_I2C_DYNTIMEO
  timeout = USEC2TICK(CONFIG_S32K1XX_I2C_DYNTIMEO_STARTSTOP);
#else
  timeout = CONFIG_S32K1XX_I2CTIMEOTICKS;
#endif

  /* Wait as stop might still be in progress; but stop might also
   * be set because of a timeout error: "The [STOP] bit is set and
   * cleared by software, cleared by hardware when a Stop condition is
   * detected, set by hardware when a timeout error is detected."
   */

  start = clock_systime_ticks();
  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;

      /* Check for STOP condition */

      if (priv->config->mode == LPI2C_MASTER)
        {
          regval = s32k1xx_lpi2c_getreg(priv, S32K1XX_LPI2C_MSR_OFFSET);
          if ((regval & LPI2C_MSR_SDF) == LPI2C_MSR_SDF)
            {
              return;
            }
        }

      /* Enable Interrupts when slave mode */

      else
        {
          regval = s32k1xx_lpi2c_getreg(priv, S32K1XX_LPI2C_SSR_OFFSET);
          if ((regval & LPI2C_SSR_SDF) == LPI2C_SSR_SDF)
            {
              return;
            }
        }

      /* Check for NACK error */

      if (priv->config->mode == LPI2C_MASTER)
        {
          regval = s32k1xx_lpi2c_getreg(priv, S32K1XX_LPI2C_MSR_OFFSET);
          if ((regval & LPI2C_MSR_NDF) == LPI2C_MSR_NDF)
            {
              return;
            }
        }

      /* Enable Interrupts when slave mode */

      else
        {
#warning Missing logic for I2C Slave
        }
    }

  /* Loop until the stop is complete or a timeout occurs. */

  while (elapsed < timeout);

  /* If we get here then a timeout occurred with the STOP condition
   * still pending.
   */

  i2cinfo("Timeout with Status Register: %" PRIx32 "\n", regval);
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ****************************************************************************/

static inline void s32k1xx_lpi2c_sem_post(struct s32k1xx_lpi2c_priv_s *priv)
{
  nxsem_post(&priv->sem_excl);
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_sem_init
 *
 * Description:
 *   Initialize semaphores
 *
 ****************************************************************************/

static inline void
  s32k1xx_lpi2c_sem_init(FAR struct s32k1xx_lpi2c_priv_s *priv)
{
  nxsem_init(&priv->sem_excl, 0, 1);

#ifndef CONFIG_I2C_POLLED
  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&priv->sem_isr, 0, 0);
  nxsem_set_protocol(&priv->sem_isr, SEM_PRIO_NONE);
#endif
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_sem_destroy
 *
 * Description:
 *   Destroy semaphores.
 *
 ****************************************************************************/

static inline void
  s32k1xx_lpi2c_sem_destroy(FAR struct s32k1xx_lpi2c_priv_s *priv)
{
  nxsem_destroy(&priv->sem_excl);
#ifndef CONFIG_I2C_POLLED
  nxsem_destroy(&priv->sem_isr);
#endif
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_trace*
 *
 * Description:
 *   I2C trace instrumentation
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void s32k1xx_lpi2c_traceclear(FAR struct s32k1xx_lpi2c_priv_s *priv)
{
  struct s32k1xx_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;              /* I2C 32-bit SR2|SR1 status */
  trace->count  = 0;              /* Interrupt count when status change */
  trace->event  = I2CEVENT_NONE;  /* Last event that occurred with this status */
  trace->parm   = 0;              /* Parameter associated with the event */
  trace->time   = 0;              /* Time of first status or event */
}

static void s32k1xx_lpi2c_tracereset(FAR struct s32k1xx_lpi2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  s32k1xx_lpi2c_traceclear(priv);
}

static void s32k1xx_lpi2c_tracenew(FAR struct s32k1xx_lpi2c_priv_s *priv,
                                 uint32_t status)
{
  struct s32k1xx_trace_s *trace = &priv->trace[priv->tndx];

  /* Is the current entry uninitialized?  Has the status changed? */

  if (trace->count == 0 || status != trace->status)
    {
      /* Yes.. Was it the status changed?  */

      if (trace->count != 0)
        {
          /* Yes.. bump up the trace index (unless out of trace entries) */

          if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
            {
              i2cerr("ERROR: Trace table overflow\n");
              return;
            }

          priv->tndx++;
          trace = &priv->trace[priv->tndx];
        }

      /* Initialize the new trace entry */

      s32k1xx_lpi2c_traceclear(priv);
      trace->status = status;
      trace->count  = 1;
      trace->time   = clock_systime_ticks();
    }
  else
    {
      /* Just increment the count of times that we have seen this status */

      trace->count++;
    }
}

static void s32k1xx_lpi2c_traceevent(FAR struct s32k1xx_lpi2c_priv_s *priv,
                                 enum s32k1xx_trace_e event, uint32_t parm)
{
  struct s32k1xx_trace_s *trace;

  if (event != I2CEVENT_NONE)
    {
      trace = &priv->trace[priv->tndx];

      /* Initialize the new trace entry */

      trace->event  = event;
      trace->parm   = parm;

      /* Bump up the trace index (unless we are out of trace entries) */

      if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
        {
          i2cerr("ERROR: Trace table overflow\n");
          return;
        }

      priv->tndx++;
      s32k1xx_lpi2c_traceclear(priv);
    }
}

static void s32k1xx_lpi2c_tracedump(FAR struct s32k1xx_lpi2c_priv_s *priv)
{
  struct s32k1xx_trace_s *trace;
  int i;

  syslog(LOG_DEBUG, "Elapsed time: %ld\n",
         (long)(clock_systime_ticks() - priv->start_time));

  for (i = 0; i < priv->tndx; i++)
    {
      trace = &priv->trace[i];
      syslog(LOG_DEBUG,
             "%2d. STATUS: %08x COUNT: %3d EVENT: %s(%2d) PARM: %08x "
             "TIME: %d\n",
             i + 1, trace->status, trace->count,
             g_trace_names[trace->event],
             trace->event, trace->parm, trace->time - priv->start_time);
    }
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: s32k1xx_lpi2c_pckfreq
 *
 * Description:
 *   Get the peripheral clock frequency for the LPSPI peripheral
 *
 * Input Parameters:
 *   base - The base address of the LPI2C peripheral registers
 *
 * Returned Value:
 *   The frequency of the LPI2C functional input frequency (or zero on a
 *   failure)
 *
 ****************************************************************************/

static uint32_t s32k1xx_lpi2c_pckfreq(uintptr_t base)
{
  enum clock_names_e clkname;
  uint32_t pccclk;
  int ret;

  /* Get the PCC source clock */

#ifdef CONFIG_S32K1XX_LPI2C0
  if (base == S32K1XX_LPI2C0_BASE)
    {
      clkname = LPI2C0_CLK;
    }
  else
#endif
#ifdef CONFIG_S32K1XX_LPI2C1
  if (base == S32K1XX_LPI2C1_BASE)
    {
      clkname = LPI2C1_CLK;
    }
  else
#endif
    {
      DEBUGPANIC();
      return -EINVAL;
    }

  ret = s32k1xx_get_pclkfreq(clkname, &pccclk);
  DEBUGASSERT(ret >= 0);
  if (ret < 0)
    {
      return 0;
    }

  return pccclk;
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_setclock
 *
 * Description:
 *   Set the I2C clock
 *
 ****************************************************************************/

static void s32k1xx_lpi2c_setclock(FAR struct s32k1xx_lpi2c_priv_s *priv,
                                 uint32_t frequency)
{
  uint32_t src_freq = 0;
  uint32_t regval;
  uint32_t men;
  uint32_t prescale = 0;
  uint32_t best_prescale = 0;
  uint32_t best_clk_hi = 0;
  uint32_t abs_error = 0;
  uint32_t best_error = 0xffffffff;
  uint32_t clk_hi_cycle;
  uint32_t computed_rate;
  uint32_t count;

  /* Has the I2C bus frequency changed? */

  if (priv->config->mode == LPI2C_MASTER)
    {
      if (frequency != priv->frequency)
        {
          /* Disable the selected LPI2C peripheral to configure the new
           * clock if it is enabled.
           */

          men = s32k1xx_lpi2c_getreg(priv, S32K1XX_LPI2C_MCR_OFFSET) &
                LPI2C_MCR_MEN;
          if (men)
            {
              s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MCR_OFFSET,
                                    LPI2C_MCR_MEN, 0);
            }

          /* Get the LPI2C clock source frequency */

          src_freq =  s32k1xx_lpi2c_pckfreq(priv->config->base);
          DEBUGASSERT(src_freq != 0);

          /* LPI2C output frequency = (Source Clock (Hz)/ 2^prescale) /
           *   (CLKLO + 1 + CLKHI + 1 + ROUNDDOWN((2 + FILTSCL) / 2^prescale)
           *
           * Assume  CLKLO = 2 * CLKHI, SETHOLD = CLKHI, DATAVD = CLKHI / 2
           */

          for (prescale = 1;
               (prescale <= 128) && (best_error != 0);
               prescale *= 2)
            {
              for (clk_hi_cycle = 1; clk_hi_cycle < 32; clk_hi_cycle++)
                {
                  if (clk_hi_cycle == 1)
                    {
                      computed_rate = (src_freq / prescale) /
                                      (6 + (2 / prescale));
                    }
                  else
                    {
                      computed_rate = (src_freq / prescale) /
                                      ((3 * clk_hi_cycle + 2) +
                                      (2 / prescale));
                    }

                  if (frequency > computed_rate)
                    {
                      abs_error = frequency - computed_rate;
                    }
                  else
                    {
                      abs_error = computed_rate - frequency;
                    }

                  if (abs_error < best_error)
                    {
                      best_prescale = prescale;
                      best_clk_hi = clk_hi_cycle;
                      best_error = abs_error;

                      if (abs_error == 0)
                        {
                          break;
                        }
                    }
                }
            }

          regval = LPI2C_MCCR0_CLKHI(best_clk_hi);

          if (best_clk_hi < 2)
            {
              regval |= LPI2C_MCCR0_CLKLO(3) | LPI2C_MCCR0_SETHOLD(2) |
                        LPI2C_MCCR0_DATAVD(1);
            }
          else
            {
              regval |= LPI2C_MCCR0_CLKLO(2 * best_clk_hi) |
                        LPI2C_MCCR0_SETHOLD(best_clk_hi) |
                        LPI2C_MCCR0_DATAVD(best_clk_hi / 2);
            }

          s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MCCR0_OFFSET, regval);

          for (count = 0; count < 8; count++)
            {
              if (best_prescale == (1 << count))
                {
                  best_prescale = count;
                  break;
                }
            }

          s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MCFGR1_OFFSET,
                                LPI2C_MCFGR1_PRESCALE_MASK,
                                LPI2C_MCFGR1_PRESCALE(best_prescale));

          /* Re-enable LPI2C if it was enabled previously */

          if (men)
            {
              s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MCR_OFFSET, 0,
                                    LPI2C_MCR_MEN);
            }

          /* Save the new LPI2C frequency */

          priv->frequency = frequency;
        }
    }
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_sendstart
 *
 * Description:
 *   Send the START conditions/force Master mode
 *
 ****************************************************************************/

static inline void
  s32k1xx_lpi2c_sendstart(FAR struct s32k1xx_lpi2c_priv_s *priv,
                          uint8_t address)
{
  uint32_t txcount = 0;
  uint32_t status = 0;
  uint8_t addr;

  /* Generate START condition and send the address */

  /* Turn off auto_stop option */

  s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MCFGR1_OFFSET,
                        LPI2C_MCFGR1_IGNACK, 0);

  do
    {
      txcount = (s32k1xx_lpi2c_getreg(priv, S32K1XX_LPI2C_MFSR_OFFSET) &
                 LPI2C_MFSR_TXCOUNT_MASK) >> LPI2C_MFSR_TXCOUNT_SHIFT;
      txcount = 4 - txcount;

      status = s32k1xx_lpi2c_getreg(priv, S32K1XX_LPI2C_MSR_OFFSET);

      if (status & LPI2C_MSR_ERROR_MASK)
        {
          s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MSR_OFFSET,
                             status & LPI2C_MSR_ERROR_MASK);
        }
    }
  while (txcount == 0);

  if ((priv->flags & I2C_M_READ) != 0)
    {
      addr = I2C_READADDR8(address);
    }
  else
    {
      addr = I2C_WRITEADDR8(address);
    }

  s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MTDR_OFFSET,
                    (LPI2C_MTDR_CMD_START | LPI2C_MTDR_DATA(addr)));
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_sendstop
 *
 * Description:
 *   Send the STOP conditions
 *
 ****************************************************************************/

static inline void
  s32k1xx_lpi2c_sendstop(FAR struct s32k1xx_lpi2c_priv_s *priv)
{
  s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MTDR_OFFSET, LPI2C_MTDR_CMD_STOP);
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_getstatus
 *
 * Description:
 *   Get 32-bit status
 *
 ****************************************************************************/

static inline uint32_t
  s32k1xx_lpi2c_getstatus(FAR struct s32k1xx_lpi2c_priv_s *priv)
{
  return s32k1xx_lpi2c_getreg(priv, S32K1XX_LPI2C_MSR_OFFSET);
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_isr_process
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ****************************************************************************/

static int s32k1xx_lpi2c_isr_process(struct s32k1xx_lpi2c_priv_s *priv)
{
  uint32_t status = s32k1xx_lpi2c_getstatus(priv);

  /* Check for new trace setup */

  s32k1xx_lpi2c_tracenew(priv, status);

  /* After an error we can get an SDF  */

  if (priv->intstate == INTSTATE_DONE && (status & LPI2C_MSR_SDF) != 0)
    {
      s32k1xx_lpi2c_traceevent(priv, I2CEVENT_STOP, 0);
      s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MSR_OFFSET, LPI2C_MSR_SDF);
    }

  /* Check if there is more bytes to send */

  else if (((priv->flags & I2C_M_READ) == 0) &&
           (status & LPI2C_MSR_TDF) != 0)
    {
      if (priv->dcnt > 0)
        {
          s32k1xx_lpi2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->dcnt);
          s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MTDR_OFFSET,
                             LPI2C_MTDR_CMD_TXD |
                             LPI2C_MTDR_DATA(*priv->ptr++));
          priv->dcnt--;

          if ((priv->msgc <= 0) && (priv->dcnt == 0))
            {
              s32k1xx_lpi2c_sendstop(priv);
            }
        }
    }

  /* Check if there is more bytes to read */

  else if (((priv->flags & I2C_M_READ) != 0) &&
           (status & LPI2C_MSR_RDF) != 0)
    {
      /* Read a byte, if dcnt goes < 0, then read dummy bytes to ack ISRs */

      if (priv->dcnt > 0)
        {
          s32k1xx_lpi2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->dcnt);

          /* No interrupts or contex switches should occur in the following
           * sequence. Otherwise, additional bytes may be sent by the device.
           */

#ifdef CONFIG_I2C_POLLED
          irqstate_t flags = enter_critical_section();
#endif

          /* Receive a byte */

          *priv->ptr++ =
            s32k1xx_lpi2c_getreg(priv, S32K1XX_LPI2C_MRDR_OFFSET) &
            LPI2C_MRDR_DATA_MASK;

          priv->dcnt--;

#ifdef CONFIG_I2C_POLLED
          leave_critical_section(flags);
#endif
          if ((priv->msgc <= 0) && (priv->dcnt == 0))
            {
              s32k1xx_lpi2c_sendstop(priv);
            }
        }
      else
        {
          s32k1xx_lpi2c_getreg(priv, S32K1XX_LPI2C_MRDR_OFFSET);
        }
    }

  if (priv->dcnt <= 0)
    {
      if (priv->msgc > 0 && priv->msgv != NULL)
        {
          priv->ptr        = priv->msgv->buffer;
          priv->dcnt    = priv->msgv->length;
          priv->flags    = priv->msgv->flags;

          if ((priv->msgv->flags & I2C_M_NOSTART) == 0)
            {
              s32k1xx_lpi2c_traceevent(priv, I2CEVENT_STARTRESTART,
                                     priv->msgc);
              s32k1xx_lpi2c_sendstart(priv, priv->msgv->addr);
            }
          else
            {
              s32k1xx_lpi2c_traceevent(priv, I2CEVENT_NOSTART, priv->msgc);
            }

          priv->msgv++;
          priv->msgc--;

          if ((priv->flags & I2C_M_READ) != 0)
            {
#ifndef CONFIG_I2C_POLLED
              /* Stop TX interrupt */

              s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MIER_OFFSET,
                                    LPI2C_MIER_TDIE, LPI2C_MIER_RDIE);
#endif
              /* Set LPI2C in read mode */

              s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MTDR_OFFSET,
                                 LPI2C_MTDR_CMD_RXD |
                                 LPI2C_MTDR_DATA((priv->dcnt - 1)));
            }
          else
            {
              /* Send the first byte from tx buffer */

              s32k1xx_lpi2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->dcnt);
              s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MTDR_OFFSET,
                                 LPI2C_MTDR_CMD_TXD |
                                 LPI2C_MTDR_DATA(*priv->ptr++));
              priv->dcnt--;
              if ((priv->msgc <= 0) && (priv->dcnt == 0))
                {
                  s32k1xx_lpi2c_sendstop(priv);
                }
            }
        }
      else if (priv->msgv && ((status & LPI2C_MSR_SDF) != 0))
        {
          s32k1xx_lpi2c_traceevent(priv, I2CEVENT_STOP, 0);
          s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MSR_OFFSET,
                             LPI2C_MSR_SDF);

          /* Check is there thread waiting for this event (there should be) */

#ifndef CONFIG_I2C_POLLED
          if (priv->intstate == INTSTATE_WAITING)
            {
              /* Update Status once at the end */

              priv->status = status;

              /* inform the thread that transfer is complete
               * and wake it up
               */

              nxsem_post(&priv->sem_isr);
              priv->intstate = INTSTATE_DONE;
            }
#else
          priv->status = status;
          priv->intstate = INTSTATE_DONE;
#endif
          /* Mark that this transaction stopped */

          priv->msgv = NULL;
        }
#ifndef CONFIG_I2C_POLLED
      else
        {
          s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MIER_OFFSET,
                                LPI2C_MIER_TDIE | LPI2C_MIER_RDIE, 0);
        }
#endif
    }

  /* Check for errors */

  if ((status & LPI2C_MSR_EPF) != 0)
    {
      s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MSR_OFFSET, LPI2C_MSR_EPF);
    }

  if ((status & LPI2C_MSR_ERROR_MASK) != 0)
    {
      s32k1xx_lpi2c_traceevent(priv, I2CEVENT_ERROR, 0);

      /* Clear the TX and RX FIFOs */

      s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MCR_OFFSET, 0,
                            LPI2C_MCR_RTF | LPI2C_MCR_RRF);

      /* Clear the error */

      s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MSR_OFFSET,
                         (status & (LPI2C_MSR_NDF | LPI2C_MSR_ALF |
                                    LPI2C_MSR_FEF)));

#ifndef CONFIG_I2C_POLLED
          if (priv->intstate == INTSTATE_WAITING)
            {
              /* Update Status once at the end */

              priv->status = status;

              /* inform the thread that transfer is complete
               * and wake it up
               */

              nxsem_post(&priv->sem_isr);
              priv->intstate = INTSTATE_DONE;
            }
#else
          priv->status = status;
          priv->intstate = INTSTATE_DONE;
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int s32k1xx_lpi2c_isr(int irq, void *context, FAR void *arg)
{
  struct s32k1xx_lpi2c_priv_s *priv = (struct s32k1xx_lpi2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);
  return s32k1xx_lpi2c_isr_process(priv);
}
#endif

/****************************************************************************
 * Name: s32k1xx_lpi2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int s32k1xx_lpi2c_init(FAR struct s32k1xx_lpi2c_priv_s *priv)
{
  /* Power-up and configure GPIOs .
   *
   * NOTE: Clocking to the LPSPI peripheral must be provided by
   * board-specific logic as part of the clock configuration logic.
   */

  /* Configure pins */

  s32k1xx_pinconfig(priv->config->scl_pin);
  s32k1xx_pinconfig(priv->config->sda_pin);

  /* Reset LPI2C before configuring it */

  s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MCR_OFFSET, LPI2C_MCR_RST);
  s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MCR_OFFSET, 0);

  /* Disable doze mode (Set DOZEN bit in 1 to disable) */

  s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MCR_OFFSET, LPI2C_MCR_DOZEN);

  /* Disable host request */

  s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MCFGR0_OFFSET,
                        LPI2C_MCFG0_HREN | LPI2C_MCFG0_HRSEL,
                        LPI2C_MCFG0_HRPOL);

  /* Pin config and ignore NACK disable */

  s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MCFGR1_OFFSET,
                        LPI2C_MCFGR1_IGNACK | LPI2C_MCFGR1_PINCFG_MASK, 0);

  /* Set tx and rx watermarks */

  s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MFCR_OFFSET,
                     LPI2C_MFCR_TXWATER(0) | LPI2C_MFCR_RXWATER(0));

  /* Force a frequency update */

  priv->frequency = 0;
  s32k1xx_lpi2c_setclock(priv, 100000);

  /* Set scl, sda glitch filters and busy idle */

  s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MCFGR2_OFFSET,
                    LPI2C_MCFG2_BUSIDLE(priv->config->busy_idle) |
                    LPI2C_MCFG2_FILTSCL_CYCLES(priv->config->filtscl) |
                    LPI2C_MCFG2_FILTSDA_CYCLES(priv->config->filtsda));

  /* Set pin low cycles to 0 (disable) */

  s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MCFGR3_OFFSET,
                     LPI2C_MCFG3_PINLOW_CYCLES(0));

  /* Attach ISRs */

#ifndef CONFIG_I2C_POLLED
  irq_attach(priv->config->irq, s32k1xx_lpi2c_isr, priv);
  up_enable_irq(priv->config->irq);
#endif

  /* Enable I2C */

  s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MCR_OFFSET, 0, LPI2C_MCR_MEN);
  return OK;
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ****************************************************************************/

static int s32k1xx_lpi2c_deinit(FAR struct s32k1xx_lpi2c_priv_s *priv)
{
  /* Disable I2C */

  s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MCR_OFFSET, LPI2C_MCR_MEN, 0);

  /* Reset LPI2C */

  s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MCR_OFFSET, LPI2C_MCR_RST);
  s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MCR_OFFSET, 0);

  /* Disable and detach interrupts */

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->irq);
  irq_detach(priv->config->irq);
#endif

  /* NOTE that clocking is left enabled. */

  return OK;
}

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_lpi2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int s32k1xx_lpi2c_transfer(FAR struct i2c_master_s *dev,
                                FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct s32k1xx_lpi2c_priv_s *priv = (struct s32k1xx_lpi2c_priv_s *)dev;
  int ret;

  DEBUGASSERT(count > 0);

  /* Ensure that address or flags don't change meanwhile */

  ret = s32k1xx_lpi2c_sem_wait(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear any pending error interrupts */

  s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MSR_OFFSET, 0xffffffff);

  /* Old transfers are done */

  /* Reset ptr and dcnt to ensure an unexpected data interrupt doesn't
   * overwrite stale data.
   */

  priv->dcnt  = 0;
  priv->ptr   = NULL;

  priv->msgv  = msgs;
  priv->msgc  = count;
  priv->flags = msgs->flags;

  i2cinfo("Flags %x, len %d \n", msgs->flags, msgs->length);

  /* Reset I2C trace logic */

  s32k1xx_lpi2c_tracereset(priv);

  /* Set I2C clock frequency */

  s32k1xx_lpi2c_setclock(priv, msgs->frequency);

  priv->status = 0;

  /* Wait for an ISR, if there was a timeout, fetch latest status to get
   * the BUSY flag.
   */

  if (s32k1xx_lpi2c_sem_waitdone(priv) < 0)
    {
      ret = -ETIMEDOUT;

      i2cerr("ERROR: Timed out: MCR: status: 0x%" PRIx32 "\n", priv->status);
    }

  /* Check for error status conditions */

  else if ((priv->status & LPI2C_MSR_ERROR_MASK) != 0)
    {
      /* I2C_SR1_ERRORMASK is the 'OR' of the following individual bits: */

      if (priv->status & LPI2C_MSR_ALF)
        {
          /* Arbitration Lost (master mode) */

          i2cerr("Arbitration lost\n");
          ret = -EAGAIN;
        }
      else if (priv->status & LPI2C_MSR_NDF)
        {
          /* Acknowledge Failure */

          i2cerr("Ack failure\n");
          ret = -ENXIO;
        }
      else
        {
          /* FIFO Error */

          i2cerr("Transfer without start condition\n");
          ret = -EINVAL;
        }
    }

  /* Dump the trace result */

  s32k1xx_lpi2c_tracedump(priv);

  /* Ensure that any ISR happening after we finish can't overwrite any user
   * data.
   */

  priv->dcnt = 0;
  priv->ptr = NULL;

  s32k1xx_lpi2c_sem_post(priv);
  return ret;
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int s32k1xx_lpi2c_reset(FAR struct i2c_master_s *dev)
{
  FAR struct s32k1xx_lpi2c_priv_s *priv =
    (FAR struct s32k1xx_lpi2c_priv_s *)dev;
  unsigned int clock_count;
  unsigned int stretch_count;
  uint32_t scl_gpio;
  uint32_t sda_gpio;
  uint32_t frequency;
  int ret;

  DEBUGASSERT(dev);

  /* Our caller must own a ref */

  DEBUGASSERT(priv->refs > 0);

  /* Lock out other clients */

  ret = s32k1xx_lpi2c_sem_wait(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EIO;

  /* Save the current frequency */

  frequency = priv->frequency;

  /* De-init the port */

  s32k1xx_lpi2c_deinit(priv);

  /* Use GPIO configuration to un-wedge the bus */

  scl_gpio = MKI2C_OUTPUT(priv->config->scl_pin);
  sda_gpio = MKI2C_OUTPUT(priv->config->sda_pin);

  s32k1xx_pinconfig(scl_gpio);
  s32k1xx_pinconfig(sda_gpio);

  /* Let SDA go high */

  s32k1xx_gpio_write(sda_gpio, 1);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!s32k1xx_gpio_read(sda_gpio))
    {
      /* Give up if we have tried too hard */

      if (clock_count++ > 10)
        {
          goto out;
        }

      /* Sniff to make sure that clock stretching has finished.
       *
       * If the bus never relaxes, the reset has failed.
       */

      stretch_count = 0;
      while (!s32k1xx_gpio_read(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      s32k1xx_gpio_write(scl_gpio, 0);
      up_udelay(10);

      /* Drive SCL high again */

      s32k1xx_gpio_write(scl_gpio, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  s32k1xx_gpio_write(sda_gpio, 0);
  up_udelay(10);
  s32k1xx_gpio_write(scl_gpio, 0);
  up_udelay(10);
  s32k1xx_gpio_write(scl_gpio, 1);
  up_udelay(10);
  s32k1xx_gpio_write(sda_gpio, 1);
  up_udelay(10);

  /* Revert the GPIO configuration. */

  sda_gpio = MKI2C_INPUT(sda_gpio);
  scl_gpio = MKI2C_INPUT(scl_gpio);

  s32k1xx_pinconfig(sda_gpio);
  s32k1xx_pinconfig(scl_gpio);

  /* Re-init the port */

  s32k1xx_lpi2c_init(priv);

  /* Restore the frequency */

  s32k1xx_lpi2c_setclock(priv, frequency);
  ret = OK;

out:

  /* Release the port for re-use by other clients */

  s32k1xx_lpi2c_sem_post(priv);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

FAR struct i2c_master_s *s32k1xx_i2cbus_initialize(int port)
{
  struct s32k1xx_lpi2c_priv_s * priv = NULL;
  irqstate_t flags;

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_S32K1XX_LPI2C0
    case 0:
      priv = (struct s32k1xx_lpi2c_priv_s *)&s32k1xx_lpi2c0_priv;
      break;
#endif

#ifdef CONFIG_S32K1XX_LPI2C1
    case 1:
      priv = (struct s32k1xx_lpi2c_priv_s *)&s32k1xx_lpi2c1_priv;
      break;
#endif

    default:
      return NULL;
    }

  /* Initialize private data for the first time, increment reference count,
   * power-up hardware and configure GPIOs.
   */

  flags = enter_critical_section();

  if ((volatile int)priv->refs++ == 0)
    {
      s32k1xx_lpi2c_sem_init(priv);
      s32k1xx_lpi2c_init(priv);
    }

  leave_critical_section(flags);

  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: s32k1xx_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int s32k1xx_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  FAR struct s32k1xx_lpi2c_priv_s *priv = (struct s32k1xx_lpi2c_priv_s *)dev;
  irqstate_t flags;

  DEBUGASSERT(dev);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  flags = enter_critical_section();

  if (--priv->refs > 0)
    {
      leave_critical_section(flags);
      return OK;
    }

  leave_critical_section(flags);

  /* Disable power and other HW resource (GPIO's) */

  s32k1xx_lpi2c_deinit(priv);

  /* Release unused resources */

  s32k1xx_lpi2c_sem_destroy(priv);
  return OK;
}

#endif /* CONFIG_S32K1XX_LPI2C */

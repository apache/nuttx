/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_lpi2c.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/irq.h>

#include "arm_internal.h"
#include "s32k1xx_edma.h"
#include "s32k1xx_pin.h"
#include "hardware/s32k1xx_dmamux.h"
#include "hardware/s32k1xx_pinmux.h"
#include "s32k1xx_lpi2c.h"
#include "s32k1xx_periphclocks.h"
#include "s32k1xx_pin.h"

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

#define LPI2C_MASTER    1
#define LPI2C_SLAVE     2

#define MKI2C_OUTPUT(p) (((p) & (~_PIN_MODE_MASK)) | GPIO_OUTPUT | GPIO_OUTPUT_ONE)
#define MKI2C_INPUT(p)  (((p) & (~_PIN_MODE_MASK)) | GPIO_INPUT)

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
  I2CEVENT_RCVBYTE,       /* Read more data, param = dcnt */
  I2CEVENT_NOSTART,       /* BTF on last byte with no restart, param = msgc */
  I2CEVENT_STARTRESTART,  /* Last byte sent, re-starting, param = msgc */
  I2CEVENT_STOP,          /* Last byte sent, send stop, param = 0 */
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

/* I2C device hardware configuration */

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
#ifdef CONFIG_S32K1XX_LPI2C_DMA
  uint32_t        dma_rxreqsrc;  /* DMA mux rx source */
  uint32_t        dma_txreqsrc;  /* DMA mux tx source */
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
  mutex_t lock;                /* Mutual exclusion mutex */
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

#ifdef CONFIG_S32K1XX_LPI2C_DMA
  DMACH_HANDLE rxdma;                                  /* rx DMA handle */
  DMACH_HANDLE txdma;                                  /* tx DMA handle */
  uint16_t     cmnds[CONFIG_S32K1XX_LPI2C_DMA_MAXMSG]; /* Commands */
#endif
  uint32_t status;             /* End of transfer SR2|SR1 status */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t
s32k1xx_lpi2c_getreg(struct s32k1xx_lpi2c_priv_s *priv,
                     uint16_t offset);
static inline void
s32k1xx_lpi2c_putreg(struct s32k1xx_lpi2c_priv_s *priv,
                     uint16_t offset, uint32_t value);
static inline void
s32k1xx_lpi2c_modifyreg(struct s32k1xx_lpi2c_priv_s *priv,
                        uint16_t offset, uint32_t clearbits,
                        uint32_t setbits);

#ifdef CONFIG_S32K1XX_I2C_DYNTIMEO
static uint32_t
s32k1xx_lpi2c_toticks(int msgc, struct i2c_msg_s *msgs);
#endif /* CONFIG_S32K1XX_I2C_DYNTIMEO */

static inline int
s32k1xx_lpi2c_sem_waitdone(struct s32k1xx_lpi2c_priv_s *priv);

#ifdef CONFIG_I2C_TRACE
static void s32k1xx_lpi2c_tracereset(struct s32k1xx_lpi2c_priv_s *priv);
static void s32k1xx_lpi2c_tracenew(struct s32k1xx_lpi2c_priv_s *priv,
                                   uint32_t status);
static void s32k1xx_lpi2c_traceevent(struct s32k1xx_lpi2c_priv_s *priv,
                                     enum s32k1xx_trace_e event,
                                     uint32_t parm);
static void s32k1xx_lpi2c_tracedump(struct s32k1xx_lpi2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

static uint32_t s32k1xx_lpi2c_pckfreq(uintptr_t base);
static void s32k1xx_lpi2c_setclock(struct s32k1xx_lpi2c_priv_s *priv,
                                   uint32_t frequency);
static inline void
s32k1xx_lpi2c_sendstart(struct s32k1xx_lpi2c_priv_s *priv,
                        uint8_t address);
static inline void
s32k1xx_lpi2c_sendstop(struct s32k1xx_lpi2c_priv_s *priv);
static inline uint32_t
s32k1xx_lpi2c_getstatus(struct s32k1xx_lpi2c_priv_s *priv);

static int s32k1xx_lpi2c_isr_process(struct s32k1xx_lpi2c_priv_s *priv);

#ifndef CONFIG_I2C_POLLED
static int s32k1xx_lpi2c_isr(int irq, void *context, void *arg);
#endif /* !CONFIG_I2C_POLLED */

static int s32k1xx_lpi2c_init(struct s32k1xx_lpi2c_priv_s *priv);
static int s32k1xx_lpi2c_deinit(struct s32k1xx_lpi2c_priv_s *priv);
static int s32k1xx_lpi2c_transfer(struct i2c_master_s *dev,
                                  struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int s32k1xx_lpi2c_reset(struct i2c_master_s *dev);
#endif

#ifdef CONFIG_S32K1XX_LPI2C_DMA
static void s32k1xx_rxdma_callback(DMACH_HANDLE handle, void *arg, bool done,
                                  int result);
static void s32k1xx_txdma_callback(DMACH_HANDLE handle, void *arg, bool done,
                                  int result);
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
  .transfer = s32k1xx_lpi2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset  = s32k1xx_lpi2c_reset,
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
  .mode       = LPI2C_MASTER,
#ifndef CONFIG_I2C_POLLED
  .irq        = S32K1XX_IRQ_LPI2C0M,
#endif
#ifdef CONFIG_LPI2C0_DMA
  .dma_rxreqsrc    = S32K1XX_DMACHAN_LPI2C0_RX,
  .dma_txreqsrc    = S32K1XX_DMACHAN_LPI2C0_TX,
#endif
};

static struct s32k1xx_lpi2c_priv_s s32k1xx_lpi2c0_priv =
{
  .ops        = &s32k1xx_lpi2c_ops,
  .config     = &s32k1xx_lpi2c0_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0,
};
#endif /* CONFIG_S32K1XX_LPI2C0 */

#ifdef CONFIG_S32K1XX_LPI2C1
static const struct s32k1xx_lpi2c_config_s s32k1xx_lpi2c1_config =
{
  .base       = S32K1XX_LPI2C1_BASE,
  .busy_idle  = CONFIG_LPI2C1_BUSYIDLE,
  .filtscl    = CONFIG_LPI2C1_FILTSCL,
  .filtsda    = CONFIG_LPI2C1_FILTSDA,
  .scl_pin    = PIN_LPI2C1_SCL,
  .sda_pin    = PIN_LPI2C1_SDA,
  .mode       = LPI2C_MASTER,
#ifndef CONFIG_I2C_POLLED
  .irq        = S32K1XX_IRQ_LPI2C1M,
#endif
#ifdef CONFIG_LPI2C1_DMA
  .dma_rxreqsrc    = S32K1XX_DMACHAN_LPI2C1_RX,
  .dma_txreqsrc    = S32K1XX_DMACHAN_LPI2C1_TX,
#endif
};

static struct s32k1xx_lpi2c_priv_s s32k1xx_lpi2c1_priv =
{
  .ops        = &s32k1xx_lpi2c_ops,
  .config     = &s32k1xx_lpi2c1_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0,
};
#endif /* CONFIG_S32K1XX_LPI2C1 */

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
s32k1xx_lpi2c_getreg(struct s32k1xx_lpi2c_priv_s *priv,
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
s32k1xx_lpi2c_putreg(struct s32k1xx_lpi2c_priv_s *priv,
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
s32k1xx_lpi2c_modifyreg(struct s32k1xx_lpi2c_priv_s *priv,
                        uint16_t offset, uint32_t clearbits,
                        uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_toticks
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be
 *   processed.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_I2C_DYNTIMEO
static uint32_t s32k1xx_lpi2c_toticks(int msgc, struct i2c_msg_s *msgs)
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

  return USEC2TICK(CONFIG_S32K1XX_I2C_DYNTIMEO_USECPERBYTE * bytecount);
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
s32k1xx_lpi2c_sem_waitdone(struct s32k1xx_lpi2c_priv_s *priv)
{
  irqstate_t flags;
  uint32_t regval;
  int ret;

  flags = enter_critical_section();

#ifdef CONFIG_S32K1XX_LPI2C_DMA
  if (priv->rxdma == NULL && priv->txdma == NULL)
    {
#endif
  /* Enable Interrupts when master mode */

  if (priv->config->mode == LPI2C_MASTER)
    {
      if ((priv->flags & I2C_M_READ) != 0)
        {
          regval = LPI2C_MIER_TDIE | LPI2C_MIER_RDIE | LPI2C_MIER_NDIE |
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
   * nxsem_tickwait_uninterruptible() sleeps.
   */
#ifdef CONFIG_S32K1XX_LPI2C_DMA
    }

#endif
  priv->intstate = INTSTATE_WAITING;
  do
    {
      /* Wait until either the transfer is complete or the timeout expires */

#ifdef CONFIG_S32K1XX_I2C_DYNTIMEO
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                     s32k1xx_lpi2c_toticks(priv->msgc, priv->msgv));
#else
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                           CONFIG_S32K1XX_I2CTIMEOTICKS);
#endif
      if (ret < 0)
        {
          /* Break out of the loop on irrecoverable errors.  This would
           * include timeouts and mystery errors reported by
           * nxsem_tickwait_uninterruptible.
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

#if 0
  /* Enable Interrupts when slave mode */

  else
    {
      s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_SIER_OFFSET, 0);
    }
#endif

  leave_critical_section(flags);
  return ret;
}
#else
static inline int
s32k1xx_lpi2c_sem_waitdone(struct s32k1xx_lpi2c_priv_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Get the timeout value */

#ifdef CONFIG_S32K1XX_I2C_DYNTIMEO
  timeout = s32k1xx_lpi2c_toticks(priv->msgc, priv->msgv);
#else
  timeout = CONFIG_S32K1XX_I2CTIMEOTICKS;
#endif

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * nxsem_tickwait_uninterruptible() sleeps.
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
 * Name: s32k1xx_rxdma_callback
 *
 * Description:
 *   This function performs the next I2C operation
 *
 ****************************************************************************/
#ifdef CONFIG_S32K1XX_LPI2C_DMA
static void s32k1xx_rxdma_callback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct s32k1xx_lpi2c_priv_s *priv = (struct s32k1xx_lpi2c_priv_s *)arg;

  s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MIER_OFFSET, 0,
                              LPI2C_MIER_SDIE);

  if (result != OK)
    {
      priv->status = s32k1xx_lpi2c_getstatus(priv);

      if ((priv->status & LPI2C_MSR_ERROR_MASK) != 0)
        {
          i2cerr("ERROR: MSR: status: 0x0%" PRIx32 "\n", priv->status);

          s32k1xx_lpi2c_traceevent(priv, I2CEVENT_ERROR, 0);

          /* Clear the TX and RX FIFOs */

          s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MCR_OFFSET, 0,
                                LPI2C_MCR_RTF | LPI2C_MCR_RRF);

          /* Clear the error */

          s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MSR_OFFSET,
                             (priv->status & (LPI2C_MSR_NDF |
                                              LPI2C_MSR_ALF |
                                              LPI2C_MSR_FEF |
                                              LPI2C_MSR_PLTF)));

          if (priv->intstate == INTSTATE_WAITING)
            {
              /* inform the thread that transfer is complete
               * and wake it up
               */

              priv->intstate = INTSTATE_DONE;
              nxsem_post(&priv->sem_isr);
            }
        }
    }
}
#endif

/****************************************************************************
 * Name: s32k1xx_txdma_callback
 *
 * Description:
 *   This function performs the next I2C operation
 *
 ****************************************************************************/
#ifdef CONFIG_S32K1XX_LPI2C_DMA
static void s32k1xx_txdma_callback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct s32k1xx_lpi2c_priv_s *priv = (struct s32k1xx_lpi2c_priv_s *)arg;

  s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MIER_OFFSET, 0,
                              LPI2C_MIER_SDIE);

  if (result != OK)
    {
      priv->status = s32k1xx_lpi2c_getstatus(priv);

      if ((priv->status & LPI2C_MSR_ERROR_MASK) != 0)
        {
          i2cerr("ERROR: MSR: status: 0x0%" PRIx32 "\n", priv->status);

          s32k1xx_lpi2c_traceevent(priv, I2CEVENT_ERROR, 0);

          /* Clear the TX and RX FIFOs */

          s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MCR_OFFSET, 0,
                                LPI2C_MCR_RTF | LPI2C_MCR_RRF);

          /* Clear the error */

          s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MSR_OFFSET,
                             (priv->status & (LPI2C_MSR_NDF |
                                              LPI2C_MSR_ALF |
                                              LPI2C_MSR_FEF |
                                              LPI2C_MSR_PLTF)));

          if (priv->intstate == INTSTATE_WAITING)
            {
              /* inform the thread that transfer is complete
               * and wake it up
               */

              priv->intstate = INTSTATE_DONE;
              nxsem_post(&priv->sem_isr);
            }
        }
    }
}
#endif

/****************************************************************************
 * Name: s32k1xx_lpi2c_trace*
 *
 * Description:
 *   I2C trace instrumentation
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void s32k1xx_lpi2c_traceclear(struct s32k1xx_lpi2c_priv_s *priv)
{
  struct s32k1xx_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;              /* I2C 32-bit SR2|SR1 status */
  trace->count  = 0;              /* Interrupt count when status change */
  trace->event  = I2CEVENT_NONE;  /* Last event that occurred with this status */
  trace->parm   = 0;              /* Parameter associated with the event */
  trace->time   = 0;              /* Time of first status or event */
}

static void s32k1xx_lpi2c_tracereset(struct s32k1xx_lpi2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  s32k1xx_lpi2c_traceclear(priv);
}

static void s32k1xx_lpi2c_tracenew(struct s32k1xx_lpi2c_priv_s *priv,
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

static void s32k1xx_lpi2c_traceevent(struct s32k1xx_lpi2c_priv_s *priv,
                                     enum s32k1xx_trace_e event,
                                     uint32_t parm)
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

static void s32k1xx_lpi2c_tracedump(struct s32k1xx_lpi2c_priv_s *priv)
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

static void s32k1xx_lpi2c_setclock(struct s32k1xx_lpi2c_priv_s *priv,
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
s32k1xx_lpi2c_sendstart(struct s32k1xx_lpi2c_priv_s *priv,
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
s32k1xx_lpi2c_sendstop(struct s32k1xx_lpi2c_priv_s *priv)
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
s32k1xx_lpi2c_getstatus(struct s32k1xx_lpi2c_priv_s *priv)
{
  return s32k1xx_lpi2c_getreg(priv, S32K1XX_LPI2C_MSR_OFFSET);
}

/****************************************************************************
 * Name: imxrt_lpi2c_getenabledints
 *
 * Description:
 *   Get 32-bit status
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_LPI2C_DMA
static inline uint32_t
s32k1xx_lpi2c_getenabledints(struct s32k1xx_lpi2c_priv_s *priv)
{
  return s32k1xx_lpi2c_getreg(priv, S32K1XX_LPI2C_MIER_OFFSET);
}
#endif

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

#ifdef CONFIG_S32K1XX_LPI2C_DMA
  uint32_t current_status = status;

  /* Condition the status with only the enabled interrupts */

  status &= s32k1xx_lpi2c_getenabledints(priv);

  if (priv->rxdma != NULL || priv->txdma != NULL)
    {
      /* End of packet or Stop */

      if ((status & (LPI2C_MSR_SDF | LPI2C_MSR_EPF)) != 0)
        {
          s32k1xx_lpi2c_traceevent(priv, I2CEVENT_STOP, 0);

          /* Acknowledge End of packet or Stop */

          s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MSR_OFFSET, status &
                                                           (LPI2C_MSR_SDF |
                                                           LPI2C_MSR_EPF));
        }

      /* Is there an Error condition */

      if (current_status & LPI2C_MSR_ERROR_MASK)
        {
          s32k1xx_lpi2c_traceevent(priv, I2CEVENT_ERROR, 0);

          /* Shutdown DMA */

          if (priv->rxdma != NULL)
            {
              s32k1xx_dmach_stop(priv->rxdma);
            }

          if (priv->txdma != NULL)
            {
               s32k1xx_dmach_stop(priv->txdma);
            }

          /* Clear the TX and RX FIFOs */

          s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MCR_OFFSET, 0,
                                LPI2C_MCR_RTF | LPI2C_MCR_RRF);

          /* Clear the error */

          s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MSR_OFFSET,
                            (current_status & (LPI2C_MSR_NDF |
                                               LPI2C_MSR_ALF |
                                               LPI2C_MSR_FEF)));

          /* Return the full error status */

          status = current_status;
        }

      /* Mark that this transaction stopped */

      priv->msgv = NULL;
      priv->msgc = 0;
      priv->dcnt = -1;

      if (priv->intstate == INTSTATE_WAITING)
        {
          /* Update Status once at the end */

          priv->status = status;

          /* inform the thread that transfer is complete
           * and wake it up
           */

          priv->intstate = INTSTATE_DONE;
          nxsem_post(&priv->sem_isr);
        }

      return OK;
    }

#endif  /* Check for new trace setup */

  s32k1xx_lpi2c_tracenew(priv, status);

  /* After an error we can get a STOP Detect Flag  */

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
          priv->ptr   = priv->msgv->buffer;
          priv->dcnt  = priv->msgv->length;
          priv->flags = priv->msgv->flags;

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

              priv->intstate = INTSTATE_DONE;
              nxsem_post(&priv->sem_isr);
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

              priv->intstate = INTSTATE_DONE;
              nxsem_post(&priv->sem_isr);
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
static int s32k1xx_lpi2c_isr(int irq, void *context, void *arg)
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

static int s32k1xx_lpi2c_init(struct s32k1xx_lpi2c_priv_s *priv)
{
  /* Power-up and configure pins.
   *
   * NOTE: Clocking to the LPI2C peripheral must be provided by
   * board-specific logic as part of the clock configuration logic.
   */

  /* Configure pins */

  s32k1xx_pinconfig(priv->config->scl_pin);
  s32k1xx_pinconfig(priv->config->sda_pin);

  /* Reset LPI2C before configuring it */

  s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MCR_OFFSET, LPI2C_MCR_RST);
  s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MCR_OFFSET, 0);

  /* Disable doze mode (Set DOZEN bit to 1 to disable) */

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

static int s32k1xx_lpi2c_deinit(struct s32k1xx_lpi2c_priv_s *priv)
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
 * Name: s32k1xx_lpi2c_dma_command_configure
 *
 * Description:
 *   Create a command TCD
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_LPI2C_DMA
static int
s32k1xx_lpi2c_dma_command_configure(struct s32k1xx_lpi2c_priv_s
                                    *priv, uint16_t *ccmd,
                                     uint32_t ncmd)
{
  struct s32k1xx_edma_xfrconfig_s config;
  memset(&config, 0, sizeof(config));

  config.saddr  = (uint32_t) ccmd;
  config.daddr  = priv->config->base + S32K1XX_LPI2C_MTDR_OFFSET;
  config.soff   = sizeof(uint16_t);
  config.doff   = 0;
  config.iter   = 1;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = EDMA_16BIT;
  config.dsize  = EDMA_16BIT;
  config.nbytes = sizeof(uint16_t) * ncmd;

  up_clean_dcache((uintptr_t)config.saddr,
                  (uintptr_t)config.saddr + config.nbytes);

  return s32k1xx_dmach_xfrsetup(priv->txdma, &config);
}
#endif

/****************************************************************************
 * Name: s32k1xx_lpi2c_dma_data_configure
 *
 * Description:
 *   Create a data TCD
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_LPI2C_DMA
static int s32k1xx_lpi2c_dma_data_configure(struct s32k1xx_lpi2c_priv_s
                                              *priv,
                                              struct i2c_msg_s *msg)
{
  DMACH_HANDLE dma;
  struct s32k1xx_edma_xfrconfig_s config;
  memset(&config, 0, sizeof(config));

  config.iter   = msg->length;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = EDMA_8BIT;
  config.dsize  = EDMA_8BIT;
  config.nbytes = sizeof(msg->buffer[0]);

  if (msg->flags & I2C_M_READ)
    {
      dma           = priv->rxdma;
      config.saddr  = priv->config->base + S32K1XX_LPI2C_MRDR_OFFSET;
      config.daddr  = (uint32_t) msg->buffer;
      config.soff   = 0;
      config.doff   = sizeof(msg->buffer[0]);
      up_invalidate_dcache((uintptr_t)msg->buffer,
                           (uintptr_t)msg->buffer + msg->length);
    }
  else
    {
      dma           = priv->txdma;
      config.saddr  = (uint32_t) msg->buffer;
      config.daddr  = priv->config->base + S32K1XX_LPI2C_MTDR_OFFSET;
      config.soff   = sizeof(msg->buffer[0]);
      config.doff   = 0;
      up_clean_dcache((uintptr_t)msg->buffer,
                      (uintptr_t)msg->buffer + msg->length);
    }

  return s32k1xx_dmach_xfrsetup(dma, &config) ? 0 : msg->length;
}
#endif

/****************************************************************************
 * Name: s32k1xx_lpi2c_configure_dma_transfer
 *
 * Description:
 *   DMA based I2C transfer function
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_LPI2C_DMA
static int s32k1xx_lpi2c_form_command_list(struct s32k1xx_lpi2c_priv_s
                                              *priv, struct i2c_msg_s *msg,
                                              int ncmds)
{
  ssize_t length = 0;

  if (priv->flags & I2C_M_NOSTART)
    {
      if (priv->flags & I2C_M_READ)
        {
          /* No start read operation */

          priv->cmnds[ncmds++] = LPI2C_MTDR_CMD_RXD |
                                 LPI2C_MTDR_DATA(msg->length - 1);
        }
    }
  else
    {
      /* A start based read or write operation */

      /* Create bus address with R/W */

      uint16_t badd = (priv->flags & I2C_M_READ) ? I2C_READADDR8(msg->addr) :
                                                   I2C_WRITEADDR8(msg->addr);

      priv->cmnds[ncmds++] = LPI2C_MTDR_CMD_START | LPI2C_MTDR_DATA(badd);

      if (badd & I2C_READBIT)
        {
          length =  msg->length;
          while (length)
            {
              if (length > 256u)
                {
                  priv->cmnds[ncmds++] = LPI2C_MTDR_CMD_RXD |
                                         LPI2C_MTDR_DATA(256u - 1);
                  length -= 256u;
                }
              else
                {
                  priv->cmnds[ncmds++] = LPI2C_MTDR_CMD_RXD |
                                         LPI2C_MTDR_DATA(length - 1);
                  length = 0;
                }
            }
        }
    }

  return ncmds;
}
#endif

/****************************************************************************
 * Name: s32k1xx_lpi2c_dma_transfer
 *
 * Description:
 *   DMA based I2C transfer function
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_LPI2C_DMA
static int s32k1xx_lpi2c_dma_transfer(struct s32k1xx_lpi2c_priv_s *priv)
{
  int m;
  int ntotcmds = 0;
  int ncmds = 0;
  uint16_t *ccmnd = NULL;

  /* Disable Interrupts */

  s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MIER_OFFSET,
                            LPI2C_MIER_RDIE | LPI2C_MIER_TDIE, 0);

  /* Disable DMA */

  s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MDER_OFFSET, LPI2C_MDER_TDDE |
                                                       LPI2C_MDER_RDDE, 0);

  /* Turn off auto_stop option */

  s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MCFGR1_OFFSET, 0,
                          LPI2C_MCFGR1_IGNACK | LPI2C_MCFGR1_AUTOSTOP);

  /* Form chains of tcd to process the messages */

  for (m = 0; m < priv->msgc; m++)
    {
      ncmds = 0;
      priv->flags = priv->msgv[m].flags;

      /* Form a command list */

      ccmnd = &priv->cmnds[ntotcmds];

      ncmds = s32k1xx_lpi2c_form_command_list(priv, &priv->msgv[m],
                                              ntotcmds);

      /* Have commands for this message ? */

      if (ncmds != 0)
        {
          /* Build up a TCD with the command from this message */

          s32k1xx_lpi2c_dma_command_configure(priv, ccmnd, ncmds - ntotcmds);

          ntotcmds += ncmds;

          DEBUGASSERT(ntotcmds < CONFIG_S32K1XX_LPI2C_DMA_MAXMSG);

          s32k1xx_lpi2c_dma_data_configure(priv, &priv->msgv[m]);
        }
    }

  s32k1xx_lpi2c_putreg(priv, S32K1XX_LPI2C_MIER_OFFSET,
                     LPI2C_MIER_NDIE | LPI2C_MIER_ALIE |
                     LPI2C_MIER_PLTIE | LPI2C_MIER_FEIE);

  s32k1xx_dmach_start(priv->rxdma, s32k1xx_rxdma_callback, priv);
  s32k1xx_dmach_start(priv->txdma, s32k1xx_txdma_callback, priv);

  s32k1xx_lpi2c_modifyreg(priv, S32K1XX_LPI2C_MDER_OFFSET, 0,
                          LPI2C_MDER_TDDE | LPI2C_MDER_RDDE);
  return OK;
}
#endif

/****************************************************************************
 * Name: s32k1xx_lpi2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int s32k1xx_lpi2c_transfer(struct i2c_master_s *dev,
                                  struct i2c_msg_s *msgs, int count)
{
  struct s32k1xx_lpi2c_priv_s *priv = (struct s32k1xx_lpi2c_priv_s *)dev;
  int ret;

  DEBUGASSERT(count > 0);

  /* Ensure that address or flags don't change meanwhile */

  ret = nxmutex_lock(&priv->lock);
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

  i2cinfo("Flags %x, len %d\n", msgs->flags, msgs->length);

  /* Reset I2C trace logic */

  s32k1xx_lpi2c_tracereset(priv);

  /* Set I2C clock frequency */

  s32k1xx_lpi2c_setclock(priv, msgs->frequency);

  priv->status = 0;

  /* Wait for ISR. If there was a timeout, fetch latest status to get
   * the BUSY flag.
   */

#ifdef CONFIG_S32K1XX_LPI2C_DMA
  if (priv->rxdma || priv->txdma)
    {
      s32k1xx_lpi2c_dma_transfer(priv);
    }
#endif

  if (s32k1xx_lpi2c_sem_waitdone(priv) < 0)
    {
#ifdef CONFIG_S32K1XX_LPI2C_DMA
      if (priv->rxdma != NULL)
        {
          s32k1xx_dmach_stop(priv->rxdma);
        }

      if (priv->txdma != NULL)
        {
          s32k1xx_dmach_stop(priv->txdma);
        }

#endif
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

  nxmutex_unlock(&priv->lock);
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
static int s32k1xx_lpi2c_reset(struct i2c_master_s *dev)
{
  struct s32k1xx_lpi2c_priv_s *priv = (struct s32k1xx_lpi2c_priv_s *)dev;
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

  ret = nxmutex_lock(&priv->lock);
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

  s32k1xx_gpiowrite(sda_gpio, 1);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!s32k1xx_gpioread(sda_gpio))
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
      while (!s32k1xx_gpioread(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      s32k1xx_gpiowrite(scl_gpio, 0);
      up_udelay(10);

      /* Drive SCL high again */

      s32k1xx_gpiowrite(scl_gpio, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  s32k1xx_gpiowrite(sda_gpio, 0);
  up_udelay(10);
  s32k1xx_gpiowrite(scl_gpio, 0);
  up_udelay(10);
  s32k1xx_gpiowrite(scl_gpio, 1);
  up_udelay(10);
  s32k1xx_gpiowrite(sda_gpio, 1);
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

  nxmutex_unlock(&priv->lock);
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

struct i2c_master_s *s32k1xx_i2cbus_initialize(int port)
{
  struct s32k1xx_lpi2c_priv_s *priv = NULL;

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
   * power-up hardware and configure pins.
   */

  nxmutex_lock(&priv->lock);
  if (priv->refs++ == 0)
    {
      s32k1xx_lpi2c_init(priv);

#ifdef CONFIG_S32K1XX_LPI2C_DMA
      if (priv->config->dma_txreqsrc != 0)
        {
          priv->txdma = s32k1xx_dmach_alloc(priv->config->dma_txreqsrc |
                                        DMAMUX_CHCFG_ENBL, 0);
          DEBUGASSERT(priv->txdma != NULL);
        }

      if (priv->config->dma_rxreqsrc != 0)
        {
          priv->rxdma = s32k1xx_dmach_alloc(priv->config->dma_rxreqsrc |
                                        DMAMUX_CHCFG_ENBL, 0);
          DEBUGASSERT(priv->rxdma != NULL);
        }
#endif
    }

  nxmutex_unlock(&priv->lock);
  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: s32k1xx_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int s32k1xx_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct s32k1xx_lpi2c_priv_s *priv = (struct s32k1xx_lpi2c_priv_s *)dev;

  DEBUGASSERT(dev);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  nxmutex_lock(&priv->lock);
  if (--priv->refs > 0)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* Disable power and other HW resource (GPIO's) */

#ifdef CONFIG_S32K1XX_LPI2C_DMA
  if (priv->rxdma != NULL)
    {
      s32k1xx_dmach_stop(priv->rxdma);
      s32k1xx_dmach_free(priv->rxdma);
      priv->rxdma = NULL;
    }

  if (priv->txdma != NULL)
    {
      s32k1xx_dmach_stop(priv->txdma);
      s32k1xx_dmach_free(priv->txdma);
      priv->txdma = NULL;
    }
#endif

  s32k1xx_lpi2c_deinit(priv);
  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif /* CONFIG_S32K1XX_LPI2C */

/****************************************************************************
 * arch/arm64/src/imx9/imx9_lpi2c.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/spinlock.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "arm64_internal.h"
#include "imx9_ccm.h"
#include "imx9_clockconfig.h"
#include "imx9_gpio.h"
#include "imx9_iomuxc.h"
#include "imx9_lpi2c.h"

#include "hardware/imx9_ccm.h"
#include "hardware/imx9_pinmux.h"
#include "hardware/imx9_lpi2c.h"

#ifdef CONFIG_IMX9_LPI2C_DMA
#  include "chip.h"
#  include "imx9_edma.h"
#  include "hardware/imx9_dmamux.h"
#endif

/* At least one I2C peripheral must be enabled */

#ifdef CONFIG_IMX9_LPI2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.
 * Instead, CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_IMX9_LPI2C_TIMEOSEC) && \
    !defined(CONFIG_IMX9_LPI2C_TIMEOMS)
#  define CONFIG_IMX9_LPI2C_TIMEOSEC 0
#  define CONFIG_IMX9_LPI2C_TIMEOMS  500   /* Default is 500 milliseconds */
#elif !defined(CONFIG_IMX9_LPI2C_TIMEOSEC)
#  define CONFIG_IMX9_LPI2C_TIMEOSEC 0     /* User provided milliseconds */
#elif !defined(CONFIG_IMX9_LPI2C_TIMEOMS)
#  define CONFIG_IMX9_LPI2C_TIMEOMS  0     /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_IMX9_LPI2C_TIMEOTICKS
#  define CONFIG_IMX9_LPI2C_TIMEOTICKS \
     (SEC2TICK(CONFIG_IMX9_LPI2C_TIMEOSEC) + \
      MSEC2TICK(CONFIG_IMX9_LPI2C_TIMEOMS))
#endif

#ifndef CONFIG_IMX9_LPI2C_DYNTIMEO_STARTSTOP
#  define CONFIG_IMX9_LPI2C_DYNTIMEO_STARTSTOP \
     TICK2USEC(CONFIG_IMX9_LPI2C_TIMEOTICKS)
#endif

/* Debug ********************************************************************/

/* I2C event trace logic.  NOTE:  trace uses the internal, non-standard,
 * low-level debug interface syslog() but does not require that any other
 * debug is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define imx9_lpi2c_tracereset(p)
#  define imx9_lpi2c_tracenew(p,s)
#  define imx9_lpi2c_traceevent(p,e,a)
#  define imx9_lpi2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

#ifdef CONFIG_I2C_SLAVE
#  error I2C slave logic is not supported yet for IMX9
#endif

#define LPI2C_MASTER    1
#define LPI2C_SLAVE     2

#define LPI2C_MSR_LIMITED_ERROR_MASK (LPI2C_MSR_ERROR_MASK & ~(LPI2C_MSR_FEF))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Interrupt state */

enum imx9_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* Trace events */

enum imx9_trace_e
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

struct imx9_trace_s
{
  uint32_t status;             /* I2C 32-bit SR2|SR1 status */
  uint32_t count;              /* Interrupt count when status change */
  enum imx9_intstate_e event;  /* Last event that occurred with this status */
  uint32_t parm;               /* Parameter associated with the event */
  clock_t time;                /* First of event or first status */
};

/* I2C Device hardware configuration */

struct imx9_lpi2c_config_s
{
  uint32_t base;               /* LPI2C base address */
  uint8_t clk_root;            /* LPI2C clock root */
  uint8_t clk_gate;            /* LPI2C clock gate */
  uint16_t busy_idle;          /* LPI2C Bus Idle Timeout */
  uint8_t filtscl;             /* Glitch Filter for SCL pin */
  uint8_t filtsda;             /* Glitch Filter for SDA pin */
  iomux_cfg_t scl_pin;         /* Peripheral configuration for SCL as SCL */
  iomux_cfg_t sda_pin;         /* Peripheral configuration for SDA as SDA */
#if defined(CONFIG_I2C_RESET)
  gpio_pinset_t reset_scl_pin; /* GPIO configuration for SCL as GPIO */
  gpio_pinset_t reset_sda_pin; /* GPIO configuration for SDA as GPIO */
#endif
  uint8_t mode;                /* Master or Slave mode */
#ifndef CONFIG_I2C_POLLED
  uint32_t irq;                /* Event IRQ */
#endif
#ifdef CONFIG_IMX9_LPI2C_DMA
  uint32_t dma_rxreqsrc;       /* DMA mux rx source */
  uint32_t dma_txreqsrc;       /* DMA mux tx source */
#endif
};

/* I2C Device Private Data */

struct imx9_lpi2c_priv_s
{
  /* Standard I2C operations */

  const struct i2c_ops_s *ops;

  /* Port configuration */

  const struct imx9_lpi2c_config_s *config;

  int refs;                    /* Reference count */
  mutex_t lock;                /* Mutual exclusion mutex */
  spinlock_t spinlock;         /* Spinlock */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif
  volatile uint8_t intstate;   /* Interrupt handshake (see enum imx9_intstate_e) */

  int8_t msgc;                 /* Message count */
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

  struct imx9_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  uint32_t status;             /* End of transfer SR2|SR1 status */

#ifdef CONFIG_IMX9_LPI2C_DMA
  DMACH_HANDLE rxdma;                               /* rx DMA handle */
  DMACH_HANDLE txdma;                               /* tx DMA handle */
  uint16_t     cmnds[CONFIG_IMX9_LPI2C_DMA_MAXMSG]; /* Commands */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t
imx9_lpi2c_getreg(struct imx9_lpi2c_priv_s *priv, uint16_t offset);
static inline void imx9_lpi2c_putreg(struct imx9_lpi2c_priv_s *priv,
                                     uint16_t offset, uint32_t value);
static inline void imx9_lpi2c_modifyreg(struct imx9_lpi2c_priv_s *priv,
                                        uint16_t offset, uint32_t clearbits,
                                        uint32_t setbits);

#ifdef CONFIG_IMX9_LPI2C_DYNTIMEO
static uint32_t imx9_lpi2c_toticks(int msgc, struct i2c_msg_s *msgs);
#endif /* CONFIG_IMX9_LPI2C_DYNTIMEO */

static inline int
imx9_lpi2c_sem_waitdone(struct imx9_lpi2c_priv_s *priv);

#ifdef CONFIG_I2C_TRACE
static void imx9_lpi2c_tracereset(struct imx9_lpi2c_priv_s *priv);
static void imx9_lpi2c_tracenew(struct imx9_lpi2c_priv_s *priv,
                                uint32_t status);
static void imx9_lpi2c_traceevent(struct imx9_lpi2c_priv_s *priv,
                                  enum imx9_trace_e event, uint32_t parm);
static void imx9_lpi2c_tracedump(struct imx9_lpi2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

static void imx9_lpi2c_setclock(struct imx9_lpi2c_priv_s *priv,
                                uint32_t frequency);
static inline void imx9_lpi2c_sendstart(struct imx9_lpi2c_priv_s *priv,
                                        uint8_t address);
static inline void imx9_lpi2c_sendstop(struct imx9_lpi2c_priv_s *priv);
static inline uint32_t
imx9_lpi2c_getstatus(struct imx9_lpi2c_priv_s *priv);

static int imx9_lpi2c_isr_process(struct imx9_lpi2c_priv_s *priv);

#ifndef CONFIG_I2C_POLLED
static int imx9_lpi2c_isr(int irq, void *context, void *arg);
#endif /* !CONFIG_I2C_POLLED */

static void imx9_lpi2c_clock_enable(struct imx9_lpi2c_priv_s *priv);
static void imx9_lpi2c_clock_disable(struct imx9_lpi2c_priv_s *priv);
static int imx9_lpi2c_init(struct imx9_lpi2c_priv_s *priv);
static int imx9_lpi2c_deinit(struct imx9_lpi2c_priv_s *priv);
static int imx9_lpi2c_transfer(struct i2c_master_s *dev,
                               struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int imx9_lpi2c_reset(struct i2c_master_s *dev);
#endif

#ifdef CONFIG_IMX9_LPI2C_DMA
static void imx9_rxdma_callback(DMACH_HANDLE handle, void *arg, bool done,
                                int result);
static void imx9_txdma_callback(DMACH_HANDLE handle, void *arg, bool done,
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

static const struct i2c_ops_s imx9_lpi2c_ops =
{
  .transfer = imx9_lpi2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset    = imx9_lpi2c_reset,
#endif
};

/* I2C device structures */

#ifdef CONFIG_IMX9_LPI2C1
static const struct imx9_lpi2c_config_s imx9_lpi2c1_config =
{
  .base          = IMX9_LPI2C1_BASE,
  .clk_root      = CCM_CR_LPI2C1,
  .clk_gate      = CCM_LPCG_LPI2C1,
  .busy_idle     = CONFIG_IMX9_LPI2C1_BUSYIDLE,
  .filtscl       = CONFIG_IMX9_LPI2C1_FILTSCL,
  .filtsda       = CONFIG_IMX9_LPI2C1_FILTSDA,
  .scl_pin       = MUX_LPI2C1_SCL,
  .sda_pin       = MUX_LPI2C1_SDA,
#if defined(CONFIG_I2C_RESET)
  .reset_scl_pin = GPIO_LPI2C1_SCL_RESET,
  .reset_sda_pin = GPIO_LPI2C1_SDA_RESET,
#endif
#ifndef CONFIG_I2C_SLAVE
  .mode          = LPI2C_MASTER,
#else
  .mode          = LPI2C_SLAVE,
#endif
#ifndef CONFIG_I2C_POLLED
  .irq           = IMX9_IRQ_LPI2C1,
#endif
#ifdef CONFIG_IMX9_LPI2C1_DMA
  .dma_rxreqsrc  = DMA_REQUEST_MUXLPI2C1RX,
  .dma_txreqsrc  = DMA_REQUEST_MUXLPI2C1TX,
#endif
};

static struct imx9_lpi2c_priv_s imx9_lpi2c1_priv =
{
  .ops           = &imx9_lpi2c_ops,
  .config        = &imx9_lpi2c1_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
  .spinlock      = SP_UNLOCKED,
#ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0
};
#endif

#ifdef CONFIG_IMX9_LPI2C2
static const struct imx9_lpi2c_config_s imx9_lpi2c2_config =
{
  .base          = IMX9_LPI2C2_BASE,
  .clk_root      = CCM_CR_LPI2C2,
  .clk_gate      = CCM_LPCG_LPI2C2,
  .busy_idle     = CONFIG_IMX9_LPI2C2_BUSYIDLE,
  .filtscl       = CONFIG_IMX9_LPI2C2_FILTSCL,
  .filtsda       = CONFIG_IMX9_LPI2C2_FILTSDA,
  .scl_pin       = MUX_LPI2C2_SCL,
  .sda_pin       = MUX_LPI2C2_SDA,
#if defined(CONFIG_I2C_RESET)
  .reset_scl_pin = GPIO_LPI2C2_SCL_RESET,
  .reset_sda_pin = GPIO_LPI2C2_SDA_RESET,
#endif
#ifndef CONFIG_I2C_SLAVE
  .mode          = LPI2C_MASTER,
#else
  .mode          = LPI2C_SLAVE,
#endif
#ifndef CONFIG_I2C_POLLED
  .irq           = IMX9_IRQ_LPI2C2,
#endif
#ifdef CONFIG_IMX9_LPI2C2_DMA
  .dma_rxreqsrc  = DMA_REQUEST_MUXLPI2C2RX,
  .dma_txreqsrc  = DMA_REQUEST_MUXLPI2C2TX,
#endif
};

static struct imx9_lpi2c_priv_s imx9_lpi2c2_priv =
{
  .ops           = &imx9_lpi2c_ops,
  .config        = &imx9_lpi2c2_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
  .spinlock      = SP_UNLOCKED,
#ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0
};
#endif

#ifdef CONFIG_IMX9_LPI2C3
static const struct imx9_lpi2c_config_s imx9_lpi2c3_config =
{
  .base          = IMX9_LPI2C3_BASE,
  .clk_root      = CCM_CR_LPI2C3,
  .clk_gate      = CCM_LPCG_LPI2C3,
  .busy_idle     = CONFIG_IMX9_LPI2C3_BUSYIDLE,
  .filtscl       = CONFIG_IMX9_LPI2C3_FILTSCL,
  .filtsda       = CONFIG_IMX9_LPI2C3_FILTSDA,
  .scl_pin       = MUX_LPI2C3_SCL,
  .sda_pin       = MUX_LPI2C3_SDA,
#if defined(CONFIG_I2C_RESET)
  .reset_scl_pin = GPIO_LPI2C3_SCL_RESET,
  .reset_sda_pin = GPIO_LPI2C3_SDA_RESET,
#endif
#ifndef CONFIG_I2C_SLAVE
  .mode          = LPI2C_MASTER,
#else
  .mode          = LPI2C_SLAVE,
#endif
#ifndef CONFIG_I2C_POLLED
  .irq           = IMX9_IRQ_LPI2C3,
#endif
#ifdef CONFIG_IMX9_LPI2C3_DMA
  .dma_rxreqsrc  = DMA_REQUEST_MUXLPI2C3RX,
  .dma_txreqsrc  = DMA_REQUEST_MUXLPI2C3TX,
#endif
};

static struct imx9_lpi2c_priv_s imx9_lpi2c3_priv =
{
  .ops           = &imx9_lpi2c_ops,
  .config        = &imx9_lpi2c3_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
  .spinlock      = SP_UNLOCKED,
#ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0
};
#endif

#ifdef CONFIG_IMX9_LPI2C4
static const struct imx9_lpi2c_config_s imx9_lpi2c4_config =
{
  .base          = IMX9_LPI2C4_BASE,
  .clk_root      = CCM_CR_LPI2C4,
  .clk_gate      = CCM_LPCG_LPI2C4,
  .busy_idle     = CONFIG_IMX9_LPI2C4_BUSYIDLE,
  .filtscl       = CONFIG_IMX9_LPI2C4_FILTSCL,
  .filtsda       = CONFIG_IMX9_LPI2C4_FILTSDA,
  .scl_pin       = MUX_LPI2C4_SCL,
  .sda_pin       = MUX_LPI2C4_SDA,
#if defined(CONFIG_I2C_RESET)
  .reset_scl_pin = GPIO_LPI2C4_SCL_RESET,
  .reset_sda_pin = GPIO_LPI2C4_SDA_RESET,
#endif
#ifndef CONFIG_I2C_SLAVE
  .mode          = LPI2C_MASTER,
#else
  .mode          = LPI2C_SLAVE,
#endif
#ifndef CONFIG_I2C_POLLED
  .irq           = IMX9_IRQ_LPI2C4,
#endif
#ifdef CONFIG_IMX9_LPI2C4_DMA
  .dma_rxreqsrc  = DMA_REQUEST_MUXLPI2C4RX,
  .dma_txreqsrc  = DMA_REQUEST_MUXLPI2C4TX,
#endif
};

static struct imx9_lpi2c_priv_s imx9_lpi2c4_priv =
{
  .ops           = &imx9_lpi2c_ops,
  .config        = &imx9_lpi2c4_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
  .spinlock      = SP_UNLOCKED,
#ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0
};
#endif

#ifdef CONFIG_IMX9_LPI2C5
static const struct imx9_lpi2c_config_s imx9_lpi2c5_config =
{
  .base          = IMX9_LPI2C5_BASE,
  .clk_root      = CCM_CR_LPI2C5,
  .clk_gate      = CCM_LPCG_LPI2C5,
  .busy_idle     = CONFIG_IMX9_LPI2C5_BUSYIDLE,
  .filtscl       = CONFIG_IMX9_LPI2C5_FILTSCL,
  .filtsda       = CONFIG_IMX9_LPI2C5_FILTSDA,
  .scl_pin       = MUX_LPI2C5_SCL,
  .sda_pin       = MUX_LPI2C5_SDA,
#if defined(CONFIG_I2C_RESET)
  .reset_scl_pin = GPIO_LPI2C5_SCL_RESET,
  .reset_sda_pin = GPIO_LPI2C5_SDA_RESET,
#endif
#ifndef CONFIG_I2C_SLAVE
  .mode          = LPI2C_MASTER,
#else
  .mode          = LPI2C_SLAVE,
#endif
#ifndef CONFIG_I2C_POLLED
  .irq           = IMX9_IRQ_LPI2C5,
#endif
#ifdef CONFIG_IMX9_LPI2C5_DMA
  .dma_rxreqsrc  = DMA_REQUEST_MUXLPI2C5RX,
  .dma_txreqsrc  = DMA_REQUEST_MUXLPI2C5TX,
#endif
};

static struct imx9_lpi2c_priv_s imx9_lpi2c5_priv =
{
  .ops           = &imx9_lpi2c_ops,
  .config        = &imx9_lpi2c5_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
  .spinlock      = SP_UNLOCKED,
#ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0
};
#endif

#ifdef CONFIG_IMX9_LPI2C6
static const struct imx9_lpi2c_config_s imx9_lpi2c6_config =
{
  .base          = IMX9_LPI2C6_BASE,
  .clk_root      = CCM_CR_LPI2C6,
  .clk_gate      = CCM_LPCG_LPI2C6,
  .busy_idle     = CONFIG_IMX9_LPI2C6_BUSYIDLE,
  .filtscl       = CONFIG_IMX9_LPI2C6_FILTSCL,
  .filtsda       = CONFIG_IMX9_LPI2C6_FILTSDA,
  .scl_pin       = MUX_LPI2C6_SCL,
  .sda_pin       = MUX_LPI2C6_SDA,
#if defined(CONFIG_I2C_RESET)
  .reset_scl_pin = GPIO_LPI2C6_SCL_RESET,
  .reset_sda_pin = GPIO_LPI2C6_SDA_RESET,
#endif
#ifndef CONFIG_I2C_SLAVE
  .mode          = LPI2C_MASTER,
#else
  .mode          = LPI2C_SLAVE,
#endif
#ifndef CONFIG_I2C_POLLED
  .irq           = IMX9_IRQ_LPI2C6,
#endif
#ifdef CONFIG_IMX9_LPI2C6_DMA
  .dma_rxreqsrc  = DMA_REQUEST_MUXLPI2C6RX,
  .dma_txreqsrc  = DMA_REQUEST_MUXLPI2C6TX,
#endif
};

static struct imx9_lpi2c_priv_s imx9_lpi2c6_priv =
{
  .ops           = &imx9_lpi2c_ops,
  .config        = &imx9_lpi2c6_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
  .spinlock      = SP_UNLOCKED,
#ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0
};
#endif

#ifdef CONFIG_IMX9_LPI2C7
static const struct imx9_lpi2c_config_s imx9_lpi2c7_config =
{
  .base          = IMX9_LPI2C7_BASE,
  .clk_root      = CCM_CR_LPI2C7,
  .clk_gate      = CCM_LPCG_LPI2C7,
  .busy_idle     = CONFIG_IMX9_LPI2C7_BUSYIDLE,
  .filtscl       = CONFIG_IMX9_LPI2C7_FILTSCL,
  .filtsda       = CONFIG_IMX9_LPI2C7_FILTSDA,
  .scl_pin       = MUX_LPI2C7_SCL,
  .sda_pin       = MUX_LPI2C7_SDA,
#if defined(CONFIG_I2C_RESET)
  .reset_scl_pin = GPIO_LPI2C7_SCL_RESET,
  .reset_sda_pin = GPIO_LPI2C7_SDA_RESET,
#endif
#ifndef CONFIG_I2C_SLAVE
  .mode          = LPI2C_MASTER,
#else
  .mode          = LPI2C_SLAVE,
#endif
#ifndef CONFIG_I2C_POLLED
  .irq           = IMX9_IRQ_LPI2C7,
#endif
#ifdef CONFIG_IMX9_LPI2C7_DMA
  .dma_rxreqsrc  = DMA_REQUEST_MUXLPI2C7RX,
  .dma_txreqsrc  = DMA_REQUEST_MUXLPI2C7TX,
#endif
};

static struct imx9_lpi2c_priv_s imx9_lpi2c7_priv =
{
  .ops           = &imx9_lpi2c_ops,
  .config        = &imx9_lpi2c7_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
  .spinlock      = SP_UNLOCKED,
#ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0
};
#endif

#ifdef CONFIG_IMX9_LPI2C8
static const struct imx9_lpi2c_config_s imx9_lpi2c8_config =
{
  .base          = IMX9_LPI2C8_BASE,
  .clk_root      = CCM_CR_LPI2C8,
  .clk_gate      = CCM_LPCG_LPI2C8,
  .busy_idle     = CONFIG_IMX9_LPI2C8_BUSYIDLE,
  .filtscl       = CONFIG_IMX9_LPI2C8_FILTSCL,
  .filtsda       = CONFIG_IMX9_LPI2C8_FILTSDA,
  .scl_pin       = MUX_LPI2C8_SCL,
  .sda_pin       = MUX_LPI2C8_SDA,
#if defined(CONFIG_I2C_RESET)
  .reset_scl_pin = GPIO_LPI2C8_SCL_RESET,
  .reset_sda_pin = GPIO_LPI2C8_SDA_RESET,
#endif
#ifndef CONFIG_I2C_SLAVE
  .mode          = LPI2C_MASTER,
#else
  .mode          = LPI2C_SLAVE,
#endif
#ifndef CONFIG_I2C_POLLED
  .irq           = IMX9_IRQ_LPI2C8,
#endif
#ifdef CONFIG_IMX9_LPI2C8_DMA
  .dma_rxreqsrc  = DMA_REQUEST_MUXLPI2C8RX,
  .dma_txreqsrc  = DMA_REQUEST_MUXLPI2C8TX,
#endif
};

static struct imx9_lpi2c_priv_s imx9_lpi2c8_priv =
{
  .ops           = &imx9_lpi2c_ops,
  .config        = &imx9_lpi2c8_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
  .spinlock      = SP_UNLOCKED,
#ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_lpi2c_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t
imx9_lpi2c_getreg(struct imx9_lpi2c_priv_s *priv, uint16_t offset)
{
  return getreg32(priv->config->base + offset);
}

/****************************************************************************
 * Name: imx9_lpi2c_putreg
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void imx9_lpi2c_putreg(struct imx9_lpi2c_priv_s *priv,
                                     uint16_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: imx9_lpi2c_modifyreg
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void imx9_lpi2c_modifyreg(struct imx9_lpi2c_priv_s *priv,
                                        uint16_t offset, uint32_t clearbits,
                                        uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: imx9_lpi2c_toticks
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be
 *   processed.
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPI2C_DYNTIMEO
static uint32_t imx9_lpi2c_toticks(int msgc, struct i2c_msg_s *msgs)
{
  int i;
  size_t bytecount = 0;
  uint32_t tick    = 0;

  /* Count the number of bytes left to process */

  for (i = 0; i < msgc; i++)
    {
      bytecount += msgs[i].length;
    }

  /* Then return a number of microseconds based on a user provided scaling
   * factor.
   */

  tick = USEC2TICK(CONFIG_IMX9_LPI2C_DYNTIMEO_USECPERBYTE * bytecount);
  if (tick == 0)
    {
      tick = 1;
    }

  return tick;
}
#endif

/****************************************************************************
 * Name: imx9_lpi2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int
imx9_lpi2c_sem_waitdone(struct imx9_lpi2c_priv_s *priv)
{
  int ret;

#ifdef CONFIG_IMX9_LPI2C_DYNTIMEO
  ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                       imx9_lpi2c_toticks(priv->msgc,
                                                          priv->msgv));
#else
  ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                       CONFIG_IMX9_LPI2C_TIMEOTICKS);
#endif

  /* Set the interrupt state back to IDLE */

  priv->intstate = INTSTATE_IDLE;

  return ret;
}
#else
static inline int
imx9_lpi2c_sem_waitdone(struct imx9_lpi2c_priv_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Get the timeout value */

#ifdef CONFIG_IMX9_LPI2C_DYNTIMEO
  timeout = imx9_lpi2c_toticks(priv->msgc, priv->msgv);
#else
  timeout = CONFIG_IMX9_LPI2C_TIMEOTICKS;
#endif
  start = clock_systime_ticks();

  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;

      /* Poll by simply calling the timer interrupt handler until it
       * reports that it is done.
       */

      imx9_lpi2c_isr_process(priv);
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
 * Name: imx9_rxdma_callback
 *
 * Description:
 *   This function performs the next I2C operation
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPI2C_DMA
static void imx9_rxdma_callback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct imx9_lpi2c_priv_s *priv = (struct imx9_lpi2c_priv_s *)arg;

  imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MIER_OFFSET, 0,
                             LPI2C_MIER_SDIE);

  if (result == OK)
    {
      if ((priv->flags & I2C_M_NOSTOP) == 0)
        {
          imx9_lpi2c_traceevent(priv, I2CEVENT_STOP, 0);
          imx9_lpi2c_sendstop(priv);
        }
    }
  else
    {
      uint32_t status = imx9_lpi2c_getstatus(priv);

      if ((status & LPI2C_MSR_ERROR_MASK) != 0)
        {
          i2cerr("ERROR: MSR: status: 0x0%" PRIx32 "\n", status);

          imx9_lpi2c_traceevent(priv, I2CEVENT_ERROR, 0);
        }
    }
}
#endif

/****************************************************************************
 * Name: imx9_txdma_callback
 *
 * Description:
 *   This function performs the next I2C operation
 *
 ****************************************************************************/
#ifdef CONFIG_IMX9_LPI2C_DMA
static void imx9_txdma_callback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct imx9_lpi2c_priv_s *priv = (struct imx9_lpi2c_priv_s *)arg;

  imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MIER_OFFSET, 0,
                             LPI2C_MIER_SDIE);

  if (result == OK)
    {
      if ((priv->flags & I2C_M_NOSTOP) == 0)
        {
          imx9_lpi2c_traceevent(priv, I2CEVENT_STOP, 0);
          imx9_lpi2c_sendstop(priv);
        }
    }
  else
    {
      uint32_t status = imx9_lpi2c_getstatus(priv);

      if ((status & LPI2C_MSR_ERROR_MASK) != 0)
        {
          i2cerr("ERROR: MSR: status: 0x0%" PRIx32 "\n", status);

          imx9_lpi2c_traceevent(priv, I2CEVENT_ERROR, 0);
        }
    }
}
#endif

/****************************************************************************
 * Name: imx9_lpi2c_trace*
 *
 * Description:
 *   I2C trace instrumentation
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void imx9_lpi2c_traceclear(struct imx9_lpi2c_priv_s *priv)
{
  struct imx9_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;              /* I2C 32-bit SR2|SR1 status */
  trace->count  = 0;              /* Interrupt count when status change */
  trace->event  = I2CEVENT_NONE;  /* Last event that occurred with this status */
  trace->parm   = 0;              /* Parameter associated with the event */
  trace->time   = 0;              /* Time of first status or event */
}

static void imx9_lpi2c_tracereset(struct imx9_lpi2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  imx9_lpi2c_traceclear(priv);
}

static void imx9_lpi2c_tracenew(struct imx9_lpi2c_priv_s *priv,
                                uint32_t status)
{
  struct imx9_trace_s *trace = &priv->trace[priv->tndx];

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

      imx9_lpi2c_traceclear(priv);
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

static void imx9_lpi2c_traceevent(struct imx9_lpi2c_priv_s *priv,
                                  enum imx9_trace_e event, uint32_t parm)
{
  struct imx9_trace_s *trace;

  if (event != I2CEVENT_NONE)
    {
      trace = &priv->trace[priv->tndx];

      /* Initialize the new trace entry */

      trace->event = event;
      trace->parm  = parm;

      /* Bump up the trace index (unless we are out of trace entries) */

      if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
        {
          i2cerr("ERROR: Trace table overflow\n");
          return;
        }

      priv->tndx++;
      imx9_lpi2c_traceclear(priv);
    }
}

static void imx9_lpi2c_tracedump(struct imx9_lpi2c_priv_s *priv)
{
  struct imx9_trace_s *trace;
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
 * Name: imx9_lpi2c_setclock
 *
 * Description:
 *   Set the I2C clock
 *
 ****************************************************************************/

static void imx9_lpi2c_setclock(struct imx9_lpi2c_priv_s *priv,
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

          men = imx9_lpi2c_getreg(priv, IMX9_LPI2C_MCR_OFFSET) &
                LPI2C_MCR_MEN;
          if (men)
            {
              imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MCR_OFFSET,
                                    LPI2C_MCR_MEN, 0);
            }

          /* Get the LPI2C clock source frequency */

          imx9_get_rootclock(priv->config->clk_root, &src_freq);

          /* LPI2C output frequency = (Source Clock (Hz)/ 2^prescale) /
           *   (CLKLO + 1 + CLKHI + 1 + ALIGN_DOWN((2 + FILTSCL)/2^prescale)
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

          imx9_lpi2c_putreg(priv, IMX9_LPI2C_MCCR0_OFFSET, regval);

          for (count = 0; count < 8; count++)
            {
              if (best_prescale == (1 << count))
                {
                  best_prescale = count;
                  break;
                }
            }

          imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MCFGR1_OFFSET,
                               LPI2C_MCFGR1_PRESCALE_MASK,
                               LPI2C_MCFGR1_PRESCALE(best_prescale));

          /* Re-enable LPI2C if it was enabled previously */

          if (men)
            {
              imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MCR_OFFSET, 0,
                                   LPI2C_MCR_MEN);
            }

          /* Save the new LPI2C frequency */

          priv->frequency = frequency;
        }
    }
}

/****************************************************************************
 * Name: imx9_lpi2c_sendstart
 *
 * Description:
 *   Send the START conditions/force Master mode
 *
 ****************************************************************************/

static inline void imx9_lpi2c_sendstart(struct imx9_lpi2c_priv_s *priv,
                                        uint8_t address)
{
  uint32_t txcount = 0;
  uint32_t status = 0;
  uint8_t addr;

  /* Disable AUTOSTOP and turn NAK Ignore off */

  imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MCFGR1_OFFSET,
                       LPI2C_MCFGR1_IGNACK | LPI2C_MCFGR1_AUTOSTOP, 0);

  do
    {
      txcount = (imx9_lpi2c_getreg(priv, IMX9_LPI2C_MFSR_OFFSET) &
                 LPI2C_MFSR_TXCOUNT_MASK) >> LPI2C_MFSR_TXCOUNT_SHIFT;
      txcount = 4 - txcount;

      status = imx9_lpi2c_getreg(priv, IMX9_LPI2C_MSR_OFFSET);

      if (status & LPI2C_MSR_ERROR_MASK)
        {
          imx9_lpi2c_putreg(priv, IMX9_LPI2C_MSR_OFFSET,
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

  /* Generate START condition and send the address */

  imx9_lpi2c_putreg(priv, IMX9_LPI2C_MTDR_OFFSET,
                    (LPI2C_MTDR_CMD_START | LPI2C_MTDR_DATA(addr)));
}

/****************************************************************************
 * Name: imx9_lpi2c_sendstop
 *
 * Description:
 *   Send the STOP conditions
 *
 ****************************************************************************/

static inline void imx9_lpi2c_sendstop(struct imx9_lpi2c_priv_s *priv)
{
  imx9_lpi2c_traceevent(priv, I2CEVENT_STOP, 0);
  imx9_lpi2c_putreg(priv, IMX9_LPI2C_MTDR_OFFSET, LPI2C_MTDR_CMD_STOP);
}

/****************************************************************************
 * Name: imx9_lpi2c_getstatus
 *
 * Description:
 *   Get 32-bit status
 *
 ****************************************************************************/

static inline uint32_t
imx9_lpi2c_getstatus(struct imx9_lpi2c_priv_s *priv)
{
  return imx9_lpi2c_getreg(priv, IMX9_LPI2C_MSR_OFFSET);
}

/****************************************************************************
 * Name: imx9_lpi2c_getenabledints
 *
 * Description:
 *   Get 32-bit status
 *
 ****************************************************************************/

static inline uint32_t
imx9_lpi2c_getenabledints(struct imx9_lpi2c_priv_s *priv)
{
  return imx9_lpi2c_getreg(priv, IMX9_LPI2C_MIER_OFFSET);
}

/****************************************************************************
 * Name: imx9_lpi2c_start_message
 *
 * Description:
 *  Start send or receive a new message in interrupt mode
 *
 ****************************************************************************/

static int imx9_lpi2c_start_message(struct imx9_lpi2c_priv_s *priv)
{
  uint32_t irq_config = (LPI2C_MIER_EPIE | LPI2C_MIER_SDIE |
                         LPI2C_MIER_NDIE | LPI2C_MIER_ALIE |
                         LPI2C_MIER_FEIE);

  priv->ptr   = priv->msgv->buffer;
  priv->dcnt  = priv->msgv->length;
  priv->flags = priv->msgv->flags;

  /* Disable ABORT which may be present after errors */

  imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MCFGR0_OFFSET,
                       LPI2C_MCFG0_ABORT, 0);

  /* Send start + address unless M_NOSTART is defined */

  if ((priv->flags & I2C_M_NOSTART) == 0)
    {
      imx9_lpi2c_traceevent(priv, I2CEVENT_STARTRESTART,
                            priv->msgc);
      imx9_lpi2c_sendstart(priv, priv->msgv->addr);
    }
  else
    {
      imx9_lpi2c_traceevent(priv, I2CEVENT_NOSTART, priv->msgc);
    }

  if ((priv->flags & I2C_M_READ) == 0)
    {
      /* Queue the first byte. NB: if start was sent and NACK received,
       * the byte won't be sent out to the bus.
       */

      imx9_lpi2c_putreg(priv, IMX9_LPI2C_MTDR_OFFSET,
                        LPI2C_MTDR_CMD_TXD |
                        LPI2C_MTDR_DATA(*priv->ptr++));
      priv->dcnt--;

      /* Enable TX interrupt */

      irq_config |= LPI2C_MIER_TDIE;
    }
  else
    {
      /* Set LPI2C in read mode */

      imx9_lpi2c_putreg(priv, IMX9_LPI2C_MTDR_OFFSET,
                        LPI2C_MTDR_CMD_RXD |
                        LPI2C_MTDR_DATA((priv->dcnt - 1)));

      /* Enable RX interrupt */

      irq_config |= LPI2C_MIER_RDIE;
    }

  imx9_lpi2c_putreg(priv, IMX9_LPI2C_MIER_OFFSET, irq_config);

  return OK;
}

/****************************************************************************
 * Name: imx9_lpi2c_stop_transfer
 *
 * Description:
 *  Stop an ongoing transfer amd signal the waiting thread
 *
 ****************************************************************************/

static int imx9_lpi2c_stop_transfer(struct imx9_lpi2c_priv_s *priv)
{
  /* Mark that there are mo more messages to transfer */

  priv->ptr = NULL;
  priv->msgc = 0;
  priv->dcnt = 0;

#ifndef CONFIG_I2C_POLLED

  /* Disable interrupts */

  imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MIER_OFFSET,
                       LPI2C_MIER_TDIE | LPI2C_MIER_RDIE |
                       LPI2C_MIER_NDIE | LPI2C_MIER_ALIE |
                       LPI2C_MIER_SDIE | LPI2C_MIER_EPIE, 0);

  /* Inform the thread that transfer is complete
   * and wake it up
   */

  if (priv->intstate == INTSTATE_WAITING)
    {
      nxsem_post(&priv->sem_isr);
    }
#endif

  priv->intstate = INTSTATE_DONE;

  return OK;
}

/****************************************************************************
 * Name: imx9_lpi2c_isr_process
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ****************************************************************************/

static int imx9_lpi2c_isr_process(struct imx9_lpi2c_priv_s *priv)
{
  uint32_t status = imx9_lpi2c_getstatus(priv);

#ifdef CONFIG_IMX9_LPI2C_DMA
  uint32_t current_status = status;

  if (priv->rxdma != NULL || priv->txdma != NULL)
    {
      /* Condition the status with only the enabled interrupts */

      status &= imx9_lpi2c_getenabledints(priv);

      /* Is there an Error condition */

      if (current_status & LPI2C_MSR_LIMITED_ERROR_MASK)
        {
          imx9_lpi2c_traceevent(priv, I2CEVENT_ERROR, 0);

          /* Return the full error status */

          priv->status = current_status;
        }

      /* End of packet or Stop */

      if ((status & (LPI2C_MSR_SDF | LPI2C_MSR_EPF)) != 0)
        {
          imx9_lpi2c_traceevent(priv, I2CEVENT_STOP, 0);

          /* Acknowledge End of packet or Stop */

          imx9_lpi2c_putreg(priv, IMX9_LPI2C_MSR_OFFSET, status &
                                                           (LPI2C_MSR_SDF |
                                                           LPI2C_MSR_EPF));

          /* Mark that this transaction stopped */

          priv->msgv = NULL;
          priv->msgc = 0;
          priv->dcnt = 0;

          if (priv->intstate == INTSTATE_WAITING)
            {
              /* inform the thread that transfer is complete
               * and wake it up
               */

              imx9_dmach_stop(priv->txdma);
              imx9_dmach_stop(priv->rxdma);

              priv->intstate = INTSTATE_DONE;
              nxsem_post(&priv->sem_isr);
            }
        }

      /* Clear the error */

      imx9_lpi2c_putreg(priv, IMX9_LPI2C_MSR_OFFSET,
                        (current_status & (LPI2C_MSR_NDF |
                                           LPI2C_MSR_ALF |
                                           LPI2C_MSR_FEF)));
      return OK;
    }

#endif

  /* Check for new trace setup */

  imx9_lpi2c_tracenew(priv, status);

  /* Check for errors */

  /* Ignore NACK on RX last byte - this is normal */

  if ((status & (LPI2C_MSR_RDF | LPI2C_MSR_NDF)) ==
      (LPI2C_MSR_RDF | LPI2C_MSR_NDF) && priv->dcnt == 1)
    {
      imx9_lpi2c_putreg(priv, IMX9_LPI2C_MSR_OFFSET, LPI2C_MSR_NDF);
      status &= ~LPI2C_MSR_NDF;
    }

  /* Handle rest of the errors */

  if ((status & LPI2C_MSR_ERROR_MASK) != 0)
    {
      imx9_lpi2c_traceevent(priv, I2CEVENT_ERROR, 0);

      /* Clear the TX and RX FIFOs */

      imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MCR_OFFSET, 0,
                           LPI2C_MCR_RTF | LPI2C_MCR_RRF);

      /* Clear the error */

      imx9_lpi2c_putreg(priv, IMX9_LPI2C_MSR_OFFSET,
                        status & LPI2C_MSR_ERROR_MASK);

      /* If there is no stop condition on the bus, abort (send stop).
       * Otherwise stop the transfer now.
       */

      if ((status & LPI2C_MSR_SDF) == 0)
        {
          /* Disable RX and TX interrupts */

          imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MIER_OFFSET,
                               LPI2C_MIER_TDIE | LPI2C_MIER_TDIE, 0);

          /* Abort any ongoing transfer, this also sends stop on the bus */

          imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MCFGR0_OFFSET, 0,
                               LPI2C_MCFG0_ABORT);
        }
      else
        {
          imx9_lpi2c_stop_transfer(priv);
        }

      /* Mark that there are no more messages to process */

      priv->status = status;
      priv->msgc = 0;
      priv->dcnt = 0;

      return OK;
    }

  /* Check for end of packet or stop */

  if (status & (LPI2C_MSR_EPF | LPI2C_MSR_SDF))
    {
      /* Reset them both */

      imx9_lpi2c_putreg(priv, IMX9_LPI2C_MSR_OFFSET,
                        status & (LPI2C_MSR_EPF | LPI2C_MSR_SDF));

      /* Process more messages or signal the thread */

      if (priv->msgc > 1)
        {
          priv->msgc--;
          priv->msgv++;
          imx9_lpi2c_start_message(priv);
        }
      else
        {
          imx9_lpi2c_stop_transfer(priv);
        }

      return OK;
    }

  /* Check if there are bytes to send */

  if ((priv->flags & I2C_M_READ) == 0 && (status & LPI2C_MSR_TDF) != 0)
    {
      if (priv->dcnt > 0)
        {
          imx9_lpi2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->dcnt);
          imx9_lpi2c_putreg(priv, IMX9_LPI2C_MTDR_OFFSET,
                            LPI2C_MTDR_CMD_TXD |
                            LPI2C_MTDR_DATA(*priv->ptr++));
          priv->dcnt--;
        }
      else if ((priv->flags & I2C_M_NOSTOP) == 0)
        {
          imx9_lpi2c_sendstop(priv);
        }
    }

  /* Check if there are received bytes */

  else if ((status & LPI2C_MSR_RDF) != 0 && priv->dcnt > 0)
    {
      imx9_lpi2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->dcnt);

      /* No interrupts or context switches should occur in the
       * following sequence. Otherwise, additional bytes may be
       * sent by the device.
       */

#ifdef CONFIG_I2C_POLLED
      irqstate_t flags = spin_lock_irqsave(&priv->spinlock);
#endif

      /* Receive a byte */

      *priv->ptr++ = imx9_lpi2c_getreg(priv, IMX9_LPI2C_MRDR_OFFSET) &
        LPI2C_MRDR_DATA_MASK;

      priv->dcnt--;
      if (priv->dcnt == 0 && (priv->flags & I2C_M_NOSTOP) == 0)
        {
          imx9_lpi2c_sendstop(priv);
        }

#ifdef CONFIG_I2C_POLLED
      spin_unlock_irqrestore(&priv->spinlock, flags);
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: imx9_lpi2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int imx9_lpi2c_isr(int irq, void *context, void *arg)
{
  struct imx9_lpi2c_priv_s *priv = (struct imx9_lpi2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);
  return imx9_lpi2c_isr_process(priv);
}
#endif

/****************************************************************************
 * Name: imx9_lpi2c_clock_enable
 *
 * Description:
 *   Ungate LPI2C clock
 *
 ****************************************************************************/

static void imx9_lpi2c_clock_enable(struct imx9_lpi2c_priv_s *priv)
{
  imx9_ccm_gate_on(priv->config->clk_gate, true);
}

/****************************************************************************
 * Name: imx9_lpi2c_clock_disable
 *
 * Description:
 *   Gate LPI2C clock
 *
 ****************************************************************************/

void imx9_lpi2c_clock_disable(struct imx9_lpi2c_priv_s *priv)
{
  imx9_ccm_gate_on(priv->config->clk_gate, false);
}

/****************************************************************************
 * Name: imx9_lpi2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int imx9_lpi2c_init(struct imx9_lpi2c_priv_s *priv)
{
  /* Power-up and configure GPIOs */

  /* Configure pins */

  imx9_iomux_configure(priv->config->scl_pin);
  imx9_iomux_configure(priv->config->sda_pin);

  /* Enable power and reset the peripheral */

  imx9_lpi2c_clock_enable(priv);

  /* Reset LPI2C before configuring it */

  imx9_lpi2c_putreg(priv, IMX9_LPI2C_MCR_OFFSET, LPI2C_MCR_RST);
  imx9_lpi2c_putreg(priv, IMX9_LPI2C_MCR_OFFSET, 0);

  /* Disable doze mode (Set DOZEN bit in 1 to disable) */

  imx9_lpi2c_putreg(priv, IMX9_LPI2C_MCR_OFFSET, LPI2C_MCR_DOZEN);

  /* Disable host request */

  imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MCFGR0_OFFSET,
                       LPI2C_MCFG0_HREN | LPI2C_MCFG0_HRSEL,
                       LPI2C_MCFG0_HRPOL);

  /* Disable AUTOSTOP and turn NAK Ignore off */

  imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MCFGR1_OFFSET,
                       LPI2C_MCFGR1_IGNACK | LPI2C_MCFGR1_AUTOSTOP, 0);

  /* Set tx and rx watermarks */

  imx9_lpi2c_putreg(priv, IMX9_LPI2C_MFCR_OFFSET,
                    LPI2C_MFCR_TXWATER(0) | LPI2C_MFCR_RXWATER(0));

  /* Force a frequency update */

  priv->frequency = 0;
  imx9_lpi2c_setclock(priv, 100000);

  /* Set scl, sda glitch filters and busy idle */

  imx9_lpi2c_putreg(priv, IMX9_LPI2C_MCFGR2_OFFSET,
                    LPI2C_MCFG2_BUSIDLE(priv->config->busy_idle) |
                    LPI2C_MCFG2_FILTSCL_CYCLES(priv->config->filtscl) |
                    LPI2C_MCFG2_FILTSDA_CYCLES(priv->config->filtsda));

  /* Set pin low cycles to 0 (disable) */

  imx9_lpi2c_putreg(priv, IMX9_LPI2C_MCFGR3_OFFSET,
                    LPI2C_MCFG3_PINLOW_CYCLES(0));

  /* Attach ISRs */

#ifndef CONFIG_I2C_POLLED
  irq_attach(priv->config->irq, imx9_lpi2c_isr, priv);
  up_enable_irq(priv->config->irq);
#endif

  /* Enable I2C */

  imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MCR_OFFSET, 0, LPI2C_MCR_MEN);
  return OK;
}

/****************************************************************************
 * Name: imx9_lpi2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ****************************************************************************/

static int imx9_lpi2c_deinit(struct imx9_lpi2c_priv_s *priv)
{
  /* Disable I2C */

  imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MCR_OFFSET, LPI2C_MCR_MEN, 0);

  /* Reset LPI2C */

  imx9_lpi2c_putreg(priv, IMX9_LPI2C_MCR_OFFSET, LPI2C_MCR_RST);
  imx9_lpi2c_putreg(priv, IMX9_LPI2C_MCR_OFFSET, 0);

  /* Disable and detach interrupts */

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->irq);
  irq_detach(priv->config->irq);
#endif

  /* Disable clocking */

  imx9_lpi2c_clock_disable(priv);

  return OK;
}

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_lpi2c_dma_command_configure
 *
 * Description:
 *   Create a command TCD
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPI2C_DMA
static int imx9_lpi2c_dma_command_configure(struct imx9_lpi2c_priv_s *priv,
                                            uint16_t *ccmd, uint32_t ncmd)
{
  struct imx9_edma_xfrconfig_s config;
  memset(&config, 0, sizeof(config));

  config.saddr  = (uintptr_t) ccmd;
  config.daddr  = priv->config->base + IMX9_LPI2C_MTDR_OFFSET;
  config.soff   = sizeof(uint16_t);
  config.doff   = 0;
  config.iter   = 1;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = EDMA_16BIT;
  config.dsize  = EDMA_16BIT;
  config.nbytes = sizeof(uint16_t) * ncmd;

  up_clean_dcache((uintptr_t)config.saddr,
                  (uintptr_t)config.saddr + config.nbytes);

  return imx9_dmach_xfrsetup(priv->txdma, &config);
}
#endif

/****************************************************************************
 * Name: imx9_lpi2c_dma_data_configure
 *
 * Description:
 *   Create a data TCD
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPI2C_DMA
static int imx9_lpi2c_dma_data_configure(struct imx9_lpi2c_priv_s *priv,
                                         struct i2c_msg_s *msg)
{
  DMACH_HANDLE dma;
  struct imx9_edma_xfrconfig_s config;
  memset(&config, 0, sizeof(config));

  config.iter   = msg->length;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = EDMA_8BIT;
  config.dsize  = EDMA_8BIT;
  config.nbytes = sizeof(msg->buffer[0]);

  if (msg->flags & I2C_M_READ)
    {
      dma           = priv->rxdma;
      config.saddr  = priv->config->base + IMX9_LPI2C_MRDR_OFFSET;
      config.daddr  = (uintptr_t) msg->buffer;
      config.soff   = 0;
      config.doff   = sizeof(msg->buffer[0]);
      up_invalidate_dcache((uintptr_t)msg->buffer,
                           (uintptr_t)msg->buffer + msg->length);
    }
  else
    {
      dma           = priv->txdma;
      config.saddr  = (uintptr_t) msg->buffer;
      config.daddr  = priv->config->base + IMX9_LPI2C_MTDR_OFFSET;
      config.soff   = sizeof(msg->buffer[0]);
      config.doff   = 0;
      up_clean_dcache((uintptr_t)msg->buffer,
                      (uintptr_t)msg->buffer + msg->length);
    }

  return imx9_dmach_xfrsetup(dma, &config) ? 0 : msg->length;
}
#endif

/****************************************************************************
 * Name: imx9_lpi2c_form_command_list
 *
 * Description:
 *   Form the DMA command list
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPI2C_DMA
static int imx9_lpi2c_form_command_list(struct imx9_lpi2c_priv_s *priv,
                                        struct i2c_msg_s *msg, int ncmds)
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
 * Name: imx9_lpi2c_dma_transfer
 *
 * Description:
 *   DMA based I2C transfer function
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPI2C_DMA
static int imx9_lpi2c_dma_transfer(struct imx9_lpi2c_priv_s *priv)
{
  int m;
  int ntotcmds = 0;
  int ncmds = 0;
  uint16_t *ccmnd = NULL;

  /* Disable Interrupts */

  imx9_lpi2c_putreg(priv, IMX9_LPI2C_MIER_OFFSET, 0);

  /* Disable DMA */

  imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MDER_OFFSET, LPI2C_MDER_TDDE |
                                                     LPI2C_MDER_RDDE, 0);

  /* Enable AUTOSTOP and NAK Ignore */

  imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MCFGR1_OFFSET, 0,
                       LPI2C_MCFGR1_IGNACK | LPI2C_MCFGR1_AUTOSTOP);

  /* Form chains of TCDs to process the messages */

  for (m = 0; m < priv->msgc; m++)
    {
      ncmds = 0;
      priv->flags = priv->msgv[m].flags;

      /* Form a command list */

      ccmnd = &priv->cmnds[ntotcmds];

      ncmds = imx9_lpi2c_form_command_list(priv, &priv->msgv[m], ntotcmds);

      /* Have commands for this message ? */

      if (ncmds != 0)
        {
          /* Build up a TCD with the command from this message */

          imx9_lpi2c_dma_command_configure(priv, ccmnd, ncmds - ntotcmds);

          ntotcmds += ncmds;

          DEBUGASSERT(ntotcmds < CONFIG_IMX9_LPI2C_DMA_MAXMSG);

          imx9_lpi2c_dma_data_configure(priv, &priv->msgv[m]);
        }
    }

  /* Clear the TX and RX FIFOs */

  imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MCR_OFFSET, 0,
                       LPI2C_MCR_RTF | LPI2C_MCR_RRF);

  /* Reset the Error bits */

  imx9_lpi2c_putreg(priv, IMX9_LPI2C_MSR_OFFSET, LPI2C_MSR_NDF |
                                                 LPI2C_MSR_ALF |
                                                 LPI2C_MSR_FEF);

  /* Enable the Iterrupts */

  imx9_lpi2c_putreg(priv, IMX9_LPI2C_MIER_OFFSET,
                    LPI2C_MIER_NDIE | LPI2C_MIER_ALIE |
                    LPI2C_MIER_PLTIE);

  /* Start The DMA */

  imx9_dmach_start(priv->rxdma, imx9_rxdma_callback, (void *)priv);
  imx9_dmach_start(priv->txdma, imx9_txdma_callback, (void *)priv);

  /* Enable the DMA Request */

  imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MDER_OFFSET, 0,
                       LPI2C_MDER_TDDE | LPI2C_MDER_RDDE);
  return OK;
}
#endif

/****************************************************************************
 * Name: imx9_lpi2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int imx9_lpi2c_transfer(struct i2c_master_s *dev,
                               struct i2c_msg_s *msgs, int count)
{
  struct imx9_lpi2c_priv_s *priv = (struct imx9_lpi2c_priv_s *)dev;
  int ret;
#ifdef CONFIG_IMX9_LPI2C_DMA
  int m;
#endif

  DEBUGASSERT(count > 0);

  /* Ensure that address or flags don't change meanwhile */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear any pending error interrupts */

  imx9_lpi2c_putreg(priv, IMX9_LPI2C_MSR_OFFSET, 0xffffffff);

  /* Old transfers are done */

  /* Reset ptr and dcnt to ensure an unexpected data interrupt doesn't
   * overwrite stale data.
   */

  priv->dcnt  = 0;
  priv->ptr   = NULL;

  priv->msgv  = msgs;
  priv->msgc  = count;
  priv->flags = msgs->flags;

  i2cinfo("Flags %x, len %ld\n", msgs->flags, msgs->length);

  /* Reset I2C trace logic */

  imx9_lpi2c_tracereset(priv);

  /* Set I2C clock frequency */

  imx9_lpi2c_setclock(priv, msgs->frequency);

  priv->status = 0;

  priv->intstate = INTSTATE_WAITING;

  /* Wait for an ISR, if there was a timeout, fetch latest status to get
   * the BUSY flag.
   */

#ifdef CONFIG_IMX9_LPI2C_DMA
  if (priv->rxdma || priv->txdma)
    {
      imx9_lpi2c_dma_transfer(priv);
    }
  else
#endif
    {
      imx9_lpi2c_start_message(priv);
    }

  if (imx9_lpi2c_sem_waitdone(priv) < 0)
    {
#ifdef CONFIG_IMX9_LPI2C_DMA
      if (priv->rxdma != NULL)
        {
          imx9_dmach_stop(priv->rxdma);
        }

      if (priv->txdma != NULL)
        {
          imx9_dmach_stop(priv->txdma);
        }

#endif
      ret = -ETIMEDOUT;
      i2cerr("ERROR: Timed out: MSR: status: 0x0%" PRIx32 "\n",
             priv->status);

      /* Stop the ongoing transfer and clear the FIFOs */

      imx9_lpi2c_stop_transfer(priv);

      imx9_lpi2c_modifyreg(priv, IMX9_LPI2C_MCR_OFFSET, 0,
                           LPI2C_MCR_RTF | LPI2C_MCR_RRF);

      /* Clear any errors */

      imx9_lpi2c_putreg(priv, IMX9_LPI2C_MSR_OFFSET, LPI2C_MSR_ERROR_MASK);

      /* Reset the semaphore. There is a race between interrupts and
       * sem_waitdone, and the semaphore is anyhow posted one extra time in
       * imx9_lpi2c_stop_transfer above
       */

      nxsem_reset(&priv->sem_isr, 0);
    }

  /* Check for error status conditions */

  else if ((priv->status & LPI2C_MSR_ERROR_MASK) != 0)
    {
      /* LPI2C_MSR_ERROR_MASK is the 'OR' of the following individual bits: */

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

          i2cerr("FIFO error\n");
          ret = -EINVAL;
        }
    }

  /* Dump the trace result */

  imx9_lpi2c_tracedump(priv);

  /* Ensure that any ISR happening after we finish can't overwrite any user
   * data.
   */

  priv->dcnt = 0;
  priv->ptr = NULL;

#ifdef CONFIG_IMX9_LPI2C_DMA
  if (priv->rxdma)
    {
      for (m = 0; m < count; m++)
        {
          if (msgs[m].flags & I2C_M_READ)
            {
              up_invalidate_dcache((uintptr_t)msgs[m].buffer,
                                   (uintptr_t)msgs[m].buffer +
                                              msgs[m].length);
            }
        }
    }
#endif

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: imx9_lpi2c_reset
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
static int imx9_lpi2c_reset(struct i2c_master_s *dev)
{
  struct imx9_lpi2c_priv_s *priv = (struct imx9_lpi2c_priv_s *)dev;
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

  imx9_lpi2c_deinit(priv);

  /* Use GPIO configuration to un-wedge the bus */

  imx9_iomux_gpio(priv->config->scl_pin, true);
  imx9_iomux_gpio(priv->config->sda_pin, true);

  scl_gpio = priv->config->reset_scl_pin;
  sda_gpio = priv->config->reset_sda_pin;

  imx9_config_gpio(scl_gpio);
  imx9_config_gpio(sda_gpio);

  /* Let SDA go high */

  imx9_gpio_write(sda_gpio, 1);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!imx9_gpio_read(sda_gpio))
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
      while (!imx9_gpio_read(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      imx9_gpio_write(scl_gpio, 0);
      up_udelay(10);

      /* Drive SCL high again */

      imx9_gpio_write(scl_gpio, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  imx9_gpio_write(sda_gpio, 0);
  up_udelay(10);
  imx9_gpio_write(scl_gpio, 0);
  up_udelay(10);
  imx9_gpio_write(scl_gpio, 1);
  up_udelay(10);
  imx9_gpio_write(sda_gpio, 1);
  up_udelay(10);

  /* Re-init the port */

  imx9_lpi2c_init(priv);

  /* Restore the frequency */

  imx9_lpi2c_setclock(priv, frequency);
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
 * Name: imx9_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

struct i2c_master_s *imx9_i2cbus_initialize(int port)
{
  struct imx9_lpi2c_priv_s * priv = NULL;
  irqstate_t flags;

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_IMX9_LPI2C1
    case 1:
      priv = (struct imx9_lpi2c_priv_s *)&imx9_lpi2c1_priv;
      break;
#endif
#ifdef CONFIG_IMX9_LPI2C2
    case 2:
      priv = (struct imx9_lpi2c_priv_s *)&imx9_lpi2c2_priv;
      break;
#endif
#ifdef CONFIG_IMX9_LPI2C3
    case 3:
      priv = (struct imx9_lpi2c_priv_s *)&imx9_lpi2c3_priv;
      break;
#endif
#ifdef CONFIG_IMX9_LPI2C4
    case 4:
      priv = (struct imx9_lpi2c_priv_s *)&imx9_lpi2c4_priv;
      break;
#endif
#ifdef CONFIG_IMX9_LPI2C5
    case 5:
      priv = (struct imx9_lpi2c_priv_s *)&imx9_lpi2c5_priv;
      break;
#endif
#ifdef CONFIG_IMX9_LPI2C6
    case 6:
      priv = (struct imx9_lpi2c_priv_s *)&imx9_lpi2c6_priv;
      break;
#endif
#ifdef CONFIG_IMX9_LPI2C7
    case 7:
      priv = (struct imx9_lpi2c_priv_s *)&imx9_lpi2c7_priv;
      break;
#endif
#ifdef CONFIG_IMX9_LPI2C8
    case 8:
      priv = (struct imx9_lpi2c_priv_s *)&imx9_lpi2c8_priv;
      break;
#endif
    default:
      return NULL;
    }

  /* Initialize private data for the first time, increment reference count,
   * power-up hardware and configure GPIOs.
   */

  flags = spin_lock_irqsave(&priv->spinlock);

  if ((volatile int)priv->refs++ == 0)
    {
      imx9_lpi2c_init(priv);

#ifdef CONFIG_IMX9_LPI2C_DMA
      if (priv->config->dma_txreqsrc != 0)
        {
          priv->txdma = imx9_dmach_alloc(priv->config->dma_txreqsrc, 0);
          DEBUGASSERT(priv->txdma != NULL);
        }

      if (priv->config->dma_rxreqsrc != 0)
        {
          priv->rxdma = imx9_dmach_alloc(priv->config->dma_rxreqsrc, 0);
          DEBUGASSERT(priv->rxdma != NULL);
        }
#endif
    }

  spin_unlock_irqrestore(&priv->spinlock, flags);

  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: imx9_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int imx9_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct imx9_lpi2c_priv_s *priv = (struct imx9_lpi2c_priv_s *)dev;
  irqstate_t flags;

  DEBUGASSERT(dev);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  flags = spin_lock_irqsave(&priv->spinlock);

  if (--priv->refs > 0)
    {
      spin_unlock_irqrestore(&priv->spinlock, flags);
      return OK;
    }

  spin_unlock_irqrestore(&priv->spinlock, flags);

  /* Disable power and other HW resource (GPIO's) */

#ifdef CONFIG_IMX9_LPI2C_DMA
  if (priv->rxdma != NULL)
    {
      imx9_dmach_stop(priv->rxdma);
      imx9_dmach_free(priv->rxdma);
      priv->rxdma = NULL;
    }

  if (priv->txdma != NULL)
    {
      imx9_dmach_stop(priv->txdma);
      imx9_dmach_free(priv->txdma);
      priv->txdma = NULL;
    }
#endif

  imx9_lpi2c_deinit(priv);

  return OK;
}

#endif /* CONFIG_IMX9_LPI2C */

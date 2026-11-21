/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_riic_i2c.c
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
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <math.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include "rzv2h_gpio.h"
#include "hardware/rzv2h_pinmap.h"

#include <arch/board/board.h>

#include "arm_internal.h"

#include "r_riic_master.h"
#include "rzv2h_riic_i2c.h"
#include "rzv2h_irq.h"
#include "rzv2h_fsp_err.h"
#include "r_ioport_api.h"
#include "r_ioport.h"

/* At least one I2C peripheral must be enabled */

#if defined(CONFIG_RZV2H_RIIC0) || defined(CONFIG_RZV2H_RIIC1) || \
  defined(CONFIG_RZV2H_RIIC2) || defined(CONFIG_RZV2H_RIIC3) || \
  defined(CONFIG_RZV2H_RIIC4) || defined(CONFIG_RZV2H_RIIC5) || \
  defined(CONFIG_RZV2H_RIIC6) || defined(CONFIG_RZV2H_RIIC7) || \
  defined(CONFIG_RZV2H_RIIC8)

/* This driver uses the FSP RIIC HAL which is interrupt-driven via callbacks.
 * Polled mode is not supported yet.
 */

#ifdef CONFIG_I2C_POLLED
#  error "CONFIG_I2C_POLLED is not supported yet"
#endif
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define RZV2H_RIIC_I2C_TIMEOTICKS \
  (SEC2TICK(CONFIG_RZV2H_RIIC_I2C_TIMEOSEC) + \
   MSEC2TICK(CONFIG_RZV2H_RIIC_I2C_TIMEOMS))

/* Debug ********************************************************************/

/* I2C event trace logic.  NOTE:  trace uses the internal, non-standard,
 * low-level debug interface syslog() but does not require that any other
 * debug is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define rzv2h_riic_i2c_tracereset(p)
#  define rzv2h_riic_i2c_tracenew(p,s)
#  define rzv2h_riic_i2c_traceevent(p,e,a)
#  define rzv2h_riic_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Interrupt state */

enum rzv2h_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* Trace events */

enum rzv2h_trace_e
{
  I2CEVENT_NONE = 0,      /* No events have occurred with this status */
  I2CEVENT_SENDADDR,      /* Start/Master bit set and address sent, param = msgc */
  I2CEVENT_SENDBYTE,      /* Send byte, param = dcnt */
  I2CEVENT_ITBUFEN,       /* Enable buffer interrupts, param = 0 */
  I2CEVENT_RCVBYTE,       /* Read more dta, param = dcnt */
  I2CEVENT_REITBUFEN,     /* Re-enable buffer interrupts, param = 0 */
  I2CEVENT_DISITBUFEN,    /* Disable buffer interrupts, param = 0 */
  I2CEVENT_BTFNOSTART,    /* BTF on last byte with no restart, param = msgc */
  I2CEVENT_BTFRESTART,    /* Last byte sent, re-starting, param = msgc */
  I2CEVENT_BTFSTOP,       /* Last byte sten, send stop, param = 0 */
  I2CEVENT_ERROR          /* Error occurred, param = 0 */
};

/* Trace data */

struct rzv2h_trace_s
{
  uint32_t status;             /* I2C 32-bit SR2|SR1 status */
  uint32_t count;              /* Interrupt count when status change */
  enum rzv2h_intstate_e event; /* Last event that occurred with this status */
  uint32_t parm;               /* Parameter associated with the event */
  clock_t time;                /* First of event or first status */
};

/* I2C pins configuration */

struct rzv2h_riic_i2c_pin_config_s
{
  gpio_pinset_t scl_pin;      /* GPIO configuration for SCL */
  gpio_pinset_t sda_pin;      /* GPIO configuration for SDA */
};

/* I2C Device Private Data */

struct rzv2h_riic_i2c_priv_s
{
  /* Standard I2C operations */

  const struct i2c_ops_s *ops;

  /* Pin configuration */

  struct rzv2h_riic_i2c_pin_config_s *pin_config;

  /* FSP abstraction */

  i2c_master_instance_t *i2c_master_hal_inst;

  rzv2h_irqsel_t rxi_irq_event;  /* IRQ event selector */
  rzv2h_irqsel_t txi_irq_event;  /* IRQ event selector */

  int refs;                    /* Reference count */
  mutex_t lock;                /* Mutual exclusion lock */
  sem_t sem_isr;               /* Interrupt wait semaphore */
  volatile uint8_t intstate;   /* Interrupt handshake (see enum rzv2h_intstate_e) */

  uint8_t msgc;                /* Message count */
  struct i2c_msg_s *msgv;      /* Message list */
  uint8_t *ptr;                /* Current message buffer */
  int dcnt;                    /* Current message length */
  uint16_t flags;              /* Current message flags */

  /* I2C trace support */

#ifdef CONFIG_I2C_TRACE
  int tndx;                    /* Trace array index */
  clock_t start_time;          /* Time when the trace was started */

  /* The actual trace data */

  struct rzv2h_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  i2c_master_event_t event;
  uint32_t frequency;          /* Current I2C frequency */
  double rise_time_s;
  double fall_time_s;
  double duty_cycle_percent;
  uint32_t noise_filter_stage;
};

struct rz_riic_master_bitrate
{
  uint32_t bitrate;
  uint32_t duty;
  uint32_t divider;
  uint32_t brl;
  uint32_t brh;
  double duty_error_percent;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rzv2h_riic_i2c_init(struct rzv2h_riic_i2c_priv_s *priv);
static int rzv2h_riic_i2c_deinit(struct rzv2h_riic_i2c_priv_s *priv);
static int rzv2h_riic_i2c_irq_attach(struct rzv2h_riic_i2c_priv_s *priv);
static void rzv2h_riic_i2c_irq_detach(struct rzv2h_riic_i2c_priv_s *priv);
static int rzv2h_riic_i2c_transfer(struct i2c_master_s *dev,
                                   struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int rzv2h_riic_i2c_reset(struct i2c_master_s *dev);
#endif

void riic_master_rxi_isr(void);
void riic_master_txi_isr(void);
void riic_master_tei_isr(void);
void riic_master_naki_isr(void);
void riic_master_sti_isr(void);
void riic_master_spi_isr(void);
void riic_master_ali_isr(void);
void riic_master_tmoi_isr(void);

/****************************************************************************
 * Name: rzv2h_riic_i2c_interrupt
 *
 * Description:
 *   Common interrupt handler for all RIIC interrupt sources.
 *   Converts GIC INTID back to FSP IRQn_Type, dispatches to the
 *   appropriate FSP ISR, and manages the FSP context stack.
 *
 * Input Parameters:
 *   irq     - GIC interrupt number
 *   context - CPU context at time of interrupt (unused)
 *   arg     - I2C private data structure
 *
 * Returned Value:
 *   OK on success, -EINVAL if IRQ does not match any RIIC source
 *
 ****************************************************************************/

static int rzv2h_riic_i2c_interrupt(int irq, void *context, void *arg)
{
  struct rzv2h_riic_i2c_priv_s *priv = (struct rzv2h_riic_i2c_priv_s *)arg;
  const i2c_master_cfg_t *cfg = priv->i2c_master_hal_inst->p_cfg;
  riic_master_extended_cfg_t *ext =
    (riic_master_extended_cfg_t *)cfg->p_extend;
  rzv2h_irqn_t fsp_irq;
  void (*fsp_isr)(void);

  UNUSED(context);

  fsp_irq = (rzv2h_irqn_t)(irq - BSP_CORTEX_VECTOR_TABLE_ENTRIES);

  if (fsp_irq == (rzv2h_irqn_t)cfg->rxi_irq)
    {
      fsp_isr = riic_master_rxi_isr;
    }
  else if (fsp_irq == (rzv2h_irqn_t)cfg->txi_irq)
    {
      fsp_isr = riic_master_txi_isr;
    }
  else if (fsp_irq == (rzv2h_irqn_t)cfg->tei_irq)
    {
      fsp_isr = riic_master_tei_isr;
    }
  else if (fsp_irq == (rzv2h_irqn_t)ext->naki_irq)
    {
      fsp_isr = riic_master_naki_isr;
    }
  else if (fsp_irq == (rzv2h_irqn_t)ext->spi_irq)
    {
      fsp_isr = riic_master_spi_isr;
    }
  else if (fsp_irq == (rzv2h_irqn_t)ext->sti_irq)
    {
      fsp_isr = riic_master_sti_isr;
    }
  else if (fsp_irq == (rzv2h_irqn_t)ext->ali_irq)
    {
      fsp_isr = riic_master_ali_isr;
    }
  else if (fsp_irq == (rzv2h_irqn_t)ext->tmoi_irq)
    {
      fsp_isr = riic_master_tmoi_isr;
    }
  else
    {
      return -EINVAL;
    }

  rzv2h_interrupt_common_handler(fsp_irq, fsp_isr);

  return OK;
}
/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C interface */

static const struct i2c_ops_s g_rzv2h_riic_i2c_ops =
{
  .transfer = rzv2h_riic_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = rzv2h_riic_i2c_reset
#endif
};

/****************************************************************************
 * Name: i2c_rz_riic_callback
 *
 * Description:
 *   FSP RIIC master callback invoked from ISR context when an I2C transfer
 *   completes (success or error).  Records the event and posts the
 *   semaphore to wake the waiting transfer function.
 *
 * Input Parameters:
 *   p_args - FSP callback arguments containing event type and context
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2c_rz_riic_callback(i2c_master_callback_args_t *p_args)
{
  struct rzv2h_riic_i2c_priv_s *priv =
    (struct rzv2h_riic_i2c_priv_s *)p_args->p_context;

  priv->event = p_args->event;
  nxsem_post(&priv->sem_isr);
}

#ifdef CONFIG_RZV2H_RIIC0
static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c0_priv;

static struct rzv2h_riic_i2c_pin_config_s g_rzv2h_riic_i2c0_pin_config =
{
  .scl_pin    = BOARD_RIIC0_SCL_GPIO,
  .sda_pin    = BOARD_RIIC0_SDA_GPIO,
};

static iic_master_instance_ctrl_t g_i2c_master0_ctrl;

static riic_master_extended_cfg_t g_i2c_master0_extend =
{
  .timeout_mode = IIC_MASTER_TIMEOUT_MODE_SHORT,
  .timeout_scl_low = IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED,
  .noise_filter_stage = 1,
  .naki_irq = RZV2H_IRQ_RIIC0_NAKI,
  .spi_irq = RZV2H_IRQ_RIIC0_SPI,
  .sti_irq = RZV2H_IRQ_RIIC0_STI,
  .ali_irq = RZV2H_IRQ_RIIC0_ALI,
  .tmoi_irq = RZV2H_IRQ_RIIC0_TMOI,
  .p_reg = (void *)R_RIIC0,
};

static i2c_master_cfg_t g_i2c_master0_cfg =
{
  .channel             = 0,
  .rate                = I2C_MASTER_RATE_STANDARD,
  .slave               = 0x00,
  .addr_mode           = I2C_MASTER_ADDR_MODE_7BIT,
  .p_transfer_tx       = NULL,
  .p_transfer_rx       = NULL,
  .p_callback          = i2c_rz_riic_callback,
  .p_context           = &g_rzv2h_riic_i2c0_priv,
  .rxi_irq             = CONFIG_RZV2H_RIIC0_RXI_INTSEL,
  .txi_irq             = CONFIG_RZV2H_RIIC0_TXI_INTSEL,
  .tei_irq             = RZV2H_IRQ_RIIC0_TEI,
  .ipl                 = CONFIG_RZV2H_RIIC0_IRQ_PRIORITY,
  .p_extend            = &g_i2c_master0_extend,
};

i2c_master_instance_t g_i2c_master0_hal_inst =
{
  .p_ctrl = &g_i2c_master0_ctrl,
  .p_cfg  = &g_i2c_master0_cfg,
  .p_api  = &g_i2c_master_on_iic
};

static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c0_priv =
{
  .ops                 = &g_rzv2h_riic_i2c_ops,
  .pin_config          = &g_rzv2h_riic_i2c0_pin_config,
  .i2c_master_hal_inst = &g_i2c_master0_hal_inst,
  .rxi_irq_event       = RZV2H_IRQSEL_RIIC0_RI,
  .txi_irq_event       = RZV2H_IRQSEL_RIIC0_TI,
  .refs                = 0,
  .lock                = NXMUTEX_INITIALIZER,
  .sem_isr             = SEM_INITIALIZER(0),
  .intstate            = INTSTATE_IDLE,
  .msgc                = 0,
  .msgv                = NULL,
  .ptr                 = NULL,
  .dcnt                = 0,
  .flags               = 0,
  .frequency           = CONFIG_RZV2H_RIIC0_BITRATE,
  .rise_time_s         = CONFIG_RZV2H_RIIC0_SCL_RISE_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .fall_time_s         = CONFIG_RZV2H_RIIC0_SCL_FALL_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .duty_cycle_percent  = CONFIG_RZV2H_RIIC0_SCL_DUTY_CYCLE,
  .noise_filter_stage  = CONFIG_RZV2H_RIIC0_NF_STAGE
};
#endif /* CONFIG_RZV2H_RIIC0 */

#ifdef CONFIG_RZV2H_RIIC1
static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c1_priv;

static struct rzv2h_riic_i2c_pin_config_s g_rzv2h_riic_i2c1_pin_config =
{
  .scl_pin    = BOARD_RIIC1_SCL_GPIO,
  .sda_pin    = BOARD_RIIC1_SDA_GPIO,
};

static iic_master_instance_ctrl_t g_i2c_master1_ctrl;

static riic_master_extended_cfg_t g_i2c_master1_extend =
{
  .timeout_mode = IIC_MASTER_TIMEOUT_MODE_SHORT,
  .timeout_scl_low = IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED,
  .noise_filter_stage = 1,
  .naki_irq = RZV2H_IRQ_RIIC1_NAKI,
  .spi_irq = RZV2H_IRQ_RIIC1_SPI,
  .sti_irq = RZV2H_IRQ_RIIC1_STI,
  .ali_irq = RZV2H_IRQ_RIIC1_ALI,
  .tmoi_irq = RZV2H_IRQ_RIIC1_TMOI,
  .p_reg = (void *)R_RIIC1,
};

static i2c_master_cfg_t g_i2c_master1_cfg =
{
  .channel             = 1,
  .rate                = I2C_MASTER_RATE_STANDARD,
  .slave               = 0x00,
  .addr_mode           = I2C_MASTER_ADDR_MODE_7BIT,
  .p_transfer_tx       = NULL,
  .p_transfer_rx       = NULL,
  .p_callback          = i2c_rz_riic_callback,
  .p_context           = &g_rzv2h_riic_i2c1_priv,
  .rxi_irq             = CONFIG_RZV2H_RIIC1_RXI_INTSEL,
  .txi_irq             = CONFIG_RZV2H_RIIC1_TXI_INTSEL,
  .tei_irq             = RZV2H_IRQ_RIIC1_TEI,
  .ipl                 = CONFIG_RZV2H_RIIC1_IRQ_PRIORITY,
  .p_extend            = &g_i2c_master1_extend,
};

i2c_master_instance_t g_i2c_master1_hal_inst =
{
  .p_ctrl = &g_i2c_master1_ctrl,
  .p_cfg  = &g_i2c_master1_cfg,
  .p_api  = &g_i2c_master_on_iic
};

static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c1_priv =
{
  .ops                 = &g_rzv2h_riic_i2c_ops,
  .pin_config          = &g_rzv2h_riic_i2c1_pin_config,
  .i2c_master_hal_inst = &g_i2c_master1_hal_inst,
  .rxi_irq_event       = RZV2H_IRQSEL_RIIC1_RI,
  .txi_irq_event       = RZV2H_IRQSEL_RIIC1_TI,
  .refs                = 0,
  .lock                = NXMUTEX_INITIALIZER,
  .sem_isr             = SEM_INITIALIZER(0),
  .intstate            = INTSTATE_IDLE,
  .msgc                = 0,
  .msgv                = NULL,
  .ptr                 = NULL,
  .dcnt                = 0,
  .flags               = 0,
  .frequency           = CONFIG_RZV2H_RIIC1_BITRATE,
  .rise_time_s         = CONFIG_RZV2H_RIIC1_SCL_RISE_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .fall_time_s         = CONFIG_RZV2H_RIIC1_SCL_FALL_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .duty_cycle_percent  = CONFIG_RZV2H_RIIC1_SCL_DUTY_CYCLE,
  .noise_filter_stage  = CONFIG_RZV2H_RIIC1_NF_STAGE
};
#endif /* CONFIG_RZV2H_RIIC1 */

#ifdef CONFIG_RZV2H_RIIC2
static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c2_priv;

static struct rzv2h_riic_i2c_pin_config_s g_rzv2h_riic_i2c2_pin_config =
{
  .scl_pin    = BOARD_RIIC2_SCL_GPIO,
  .sda_pin    = BOARD_RIIC2_SDA_GPIO,
};

static iic_master_instance_ctrl_t g_i2c_master2_ctrl;

static riic_master_extended_cfg_t g_i2c_master2_extend =
{
  .timeout_mode = IIC_MASTER_TIMEOUT_MODE_SHORT,
  .timeout_scl_low = IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED,
  .noise_filter_stage = 1,
  .naki_irq = RZV2H_IRQ_RIIC2_NAKI,
  .spi_irq = RZV2H_IRQ_RIIC2_SPI,
  .sti_irq = RZV2H_IRQ_RIIC2_STI,
  .ali_irq = RZV2H_IRQ_RIIC2_ALI,
  .tmoi_irq = RZV2H_IRQ_RIIC2_TMOI,
  .p_reg = (void *)R_RIIC2,
};

static i2c_master_cfg_t g_i2c_master2_cfg =
{
  .channel             = 2,
  .rate                = I2C_MASTER_RATE_STANDARD,
  .slave               = 0x00,
  .addr_mode           = I2C_MASTER_ADDR_MODE_7BIT,
  .p_transfer_tx       = NULL,
  .p_transfer_rx       = NULL,
  .p_callback          = i2c_rz_riic_callback,
  .p_context           = &g_rzv2h_riic_i2c2_priv,
  .rxi_irq             = CONFIG_RZV2H_RIIC2_RXI_INTSEL,
  .txi_irq             = CONFIG_RZV2H_RIIC2_TXI_INTSEL,
  .tei_irq             = RZV2H_IRQ_RIIC2_TEI,
  .ipl                 = CONFIG_RZV2H_RIIC2_IRQ_PRIORITY,
  .p_extend            = &g_i2c_master2_extend,
};

i2c_master_instance_t g_i2c_master2_hal_inst =
{
  .p_ctrl = &g_i2c_master2_ctrl,
  .p_cfg  = &g_i2c_master2_cfg,
  .p_api  = &g_i2c_master_on_iic
};

static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c2_priv =
{
  .ops                 = &g_rzv2h_riic_i2c_ops,
  .pin_config          = &g_rzv2h_riic_i2c2_pin_config,
  .i2c_master_hal_inst = &g_i2c_master2_hal_inst,
  .rxi_irq_event       = RZV2H_IRQSEL_RIIC2_RI,
  .txi_irq_event       = RZV2H_IRQSEL_RIIC2_TI,
  .refs                = 0,
  .lock                = NXMUTEX_INITIALIZER,
  .sem_isr             = SEM_INITIALIZER(0),
  .intstate            = INTSTATE_IDLE,
  .msgc                = 0,
  .msgv                = NULL,
  .ptr                 = NULL,
  .dcnt                = 0,
  .flags               = 0,
  .frequency           = CONFIG_RZV2H_RIIC2_BITRATE,
  .rise_time_s         = CONFIG_RZV2H_RIIC2_SCL_RISE_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .fall_time_s         = CONFIG_RZV2H_RIIC2_SCL_FALL_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .duty_cycle_percent  = CONFIG_RZV2H_RIIC2_SCL_DUTY_CYCLE,
  .noise_filter_stage  = CONFIG_RZV2H_RIIC2_NF_STAGE
};
#endif /* CONFIG_RZV2H_RIIC2 */

#ifdef CONFIG_RZV2H_RIIC3
static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c3_priv;

static struct rzv2h_riic_i2c_pin_config_s g_rzv2h_riic_i2c3_pin_config =
{
  .scl_pin    = BOARD_RIIC3_SCL_GPIO,
  .sda_pin    = BOARD_RIIC3_SDA_GPIO,
};

static iic_master_instance_ctrl_t g_i2c_master3_ctrl;

static riic_master_extended_cfg_t g_i2c_master3_extend =
{
  .timeout_mode = IIC_MASTER_TIMEOUT_MODE_SHORT,
  .timeout_scl_low = IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED,
  .noise_filter_stage = 1,
  .naki_irq = RZV2H_IRQ_RIIC3_NAKI,
  .spi_irq = RZV2H_IRQ_RIIC3_SPI,
  .sti_irq = RZV2H_IRQ_RIIC3_STI,
  .ali_irq = RZV2H_IRQ_RIIC3_ALI,
  .tmoi_irq = RZV2H_IRQ_RIIC3_TMOI,
  .p_reg = (void *)R_RIIC3,
};

static i2c_master_cfg_t g_i2c_master3_cfg =
{
  .channel             = 3,
  .rate                = I2C_MASTER_RATE_STANDARD,
  .slave               = 0x00,
  .addr_mode           = I2C_MASTER_ADDR_MODE_7BIT,
  .p_transfer_tx       = NULL,
  .p_transfer_rx       = NULL,
  .p_callback          = i2c_rz_riic_callback,
  .p_context           = &g_rzv2h_riic_i2c3_priv,
  .rxi_irq             = CONFIG_RZV2H_RIIC3_RXI_INTSEL,
  .txi_irq             = CONFIG_RZV2H_RIIC3_TXI_INTSEL,
  .tei_irq             = RZV2H_IRQ_RIIC3_TEI,
  .ipl                 = CONFIG_RZV2H_RIIC3_IRQ_PRIORITY,
  .p_extend            = &g_i2c_master3_extend,
};

i2c_master_instance_t g_i2c_master3_hal_inst =
{
  .p_ctrl = &g_i2c_master3_ctrl,
  .p_cfg  = &g_i2c_master3_cfg,
  .p_api  = &g_i2c_master_on_iic
};

static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c3_priv =
{
  .ops                 = &g_rzv2h_riic_i2c_ops,
  .pin_config          = &g_rzv2h_riic_i2c3_pin_config,
  .i2c_master_hal_inst = &g_i2c_master3_hal_inst,
  .rxi_irq_event       = RZV2H_IRQSEL_RIIC3_RI,
  .txi_irq_event       = RZV2H_IRQSEL_RIIC3_TI,
  .refs                = 0,
  .lock                = NXMUTEX_INITIALIZER,
  .sem_isr             = SEM_INITIALIZER(0),
  .intstate            = INTSTATE_IDLE,
  .msgc                = 0,
  .msgv                = NULL,
  .ptr                 = NULL,
  .dcnt                = 0,
  .flags               = 0,
  .frequency           = CONFIG_RZV2H_RIIC3_BITRATE,
  .rise_time_s         = CONFIG_RZV2H_RIIC3_SCL_RISE_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .fall_time_s         = CONFIG_RZV2H_RIIC3_SCL_FALL_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .duty_cycle_percent  = CONFIG_RZV2H_RIIC3_SCL_DUTY_CYCLE,
  .noise_filter_stage  = CONFIG_RZV2H_RIIC3_NF_STAGE
};
#endif /* CONFIG_RZV2H_RIIC3 */

#ifdef CONFIG_RZV2H_RIIC4
static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c4_priv;

static struct rzv2h_riic_i2c_pin_config_s g_rzv2h_riic_i2c4_pin_config =
{
  .scl_pin    = BOARD_RIIC4_SCL_GPIO,
  .sda_pin    = BOARD_RIIC4_SDA_GPIO,
};

static iic_master_instance_ctrl_t g_i2c_master4_ctrl;

static riic_master_extended_cfg_t g_i2c_master4_extend =
{
  .timeout_mode = IIC_MASTER_TIMEOUT_MODE_SHORT,
  .timeout_scl_low = IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED,
  .noise_filter_stage = 1,
  .naki_irq = RZV2H_IRQ_RIIC4_NAKI,
  .spi_irq = RZV2H_IRQ_RIIC4_SPI,
  .sti_irq = RZV2H_IRQ_RIIC4_STI,
  .ali_irq = RZV2H_IRQ_RIIC4_ALI,
  .tmoi_irq = RZV2H_IRQ_RIIC4_TMOI,
  .p_reg = (void *)R_RIIC4,
};

static i2c_master_cfg_t g_i2c_master4_cfg =
{
  .channel             = 4,
  .rate                = I2C_MASTER_RATE_STANDARD,
  .slave               = 0x00,
  .addr_mode           = I2C_MASTER_ADDR_MODE_7BIT,
  .p_transfer_tx       = NULL,
  .p_transfer_rx       = NULL,
  .p_callback          = i2c_rz_riic_callback,
  .p_context           = &g_rzv2h_riic_i2c4_priv,
  .rxi_irq             = CONFIG_RZV2H_RIIC4_RXI_INTSEL,
  .txi_irq             = CONFIG_RZV2H_RIIC4_TXI_INTSEL,
  .tei_irq             = RZV2H_IRQ_RIIC4_TEI,
  .ipl                 = CONFIG_RZV2H_RIIC4_IRQ_PRIORITY,
  .p_extend            = &g_i2c_master4_extend,
};

i2c_master_instance_t g_i2c_master4_hal_inst =
{
  .p_ctrl = &g_i2c_master4_ctrl,
  .p_cfg  = &g_i2c_master4_cfg,
  .p_api  = &g_i2c_master_on_iic
};

static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c4_priv =
{
  .ops                 = &g_rzv2h_riic_i2c_ops,
  .pin_config          = &g_rzv2h_riic_i2c4_pin_config,
  .i2c_master_hal_inst = &g_i2c_master4_hal_inst,
  .rxi_irq_event       = RZV2H_IRQSEL_RIIC4_RI,
  .txi_irq_event       = RZV2H_IRQSEL_RIIC4_TI,
  .refs                = 0,
  .lock                = NXMUTEX_INITIALIZER,
  .sem_isr             = SEM_INITIALIZER(0),
  .intstate            = INTSTATE_IDLE,
  .msgc                = 0,
  .msgv                = NULL,
  .ptr                 = NULL,
  .dcnt                = 0,
  .flags               = 0,
  .frequency           = CONFIG_RZV2H_RIIC4_BITRATE,
  .rise_time_s         = CONFIG_RZV2H_RIIC4_SCL_RISE_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .fall_time_s         = CONFIG_RZV2H_RIIC4_SCL_FALL_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .duty_cycle_percent  = CONFIG_RZV2H_RIIC4_SCL_DUTY_CYCLE,
  .noise_filter_stage  = CONFIG_RZV2H_RIIC4_NF_STAGE
};
#endif /* CONFIG_RZV2H_RIIC4 */

#ifdef CONFIG_RZV2H_RIIC5
static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c5_priv;

static struct rzv2h_riic_i2c_pin_config_s g_rzv2h_riic_i2c5_pin_config =
{
  .scl_pin    = BOARD_RIIC5_SCL_GPIO,
  .sda_pin    = BOARD_RIIC5_SDA_GPIO,
};

static iic_master_instance_ctrl_t g_i2c_master5_ctrl;

static riic_master_extended_cfg_t g_i2c_master5_extend =
{
  .timeout_mode = IIC_MASTER_TIMEOUT_MODE_SHORT,
  .timeout_scl_low = IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED,
  .noise_filter_stage = 1,
  .naki_irq = RZV2H_IRQ_RIIC5_NAKI,
  .spi_irq = RZV2H_IRQ_RIIC5_SPI,
  .sti_irq = RZV2H_IRQ_RIIC5_STI,
  .ali_irq = RZV2H_IRQ_RIIC5_ALI,
  .tmoi_irq = RZV2H_IRQ_RIIC5_TMOI,
  .p_reg = (void *)R_RIIC5,
};

static i2c_master_cfg_t g_i2c_master5_cfg =
{
  .channel             = 5,
  .rate                = I2C_MASTER_RATE_STANDARD,
  .slave               = 0x00,
  .addr_mode           = I2C_MASTER_ADDR_MODE_7BIT,
  .p_transfer_tx       = NULL,
  .p_transfer_rx       = NULL,
  .p_callback          = i2c_rz_riic_callback,
  .p_context           = &g_rzv2h_riic_i2c5_priv,
  .rxi_irq             = CONFIG_RZV2H_RIIC5_RXI_INTSEL,
  .txi_irq             = CONFIG_RZV2H_RIIC5_TXI_INTSEL,
  .tei_irq             = RZV2H_IRQ_RIIC5_TEI,
  .ipl                 = CONFIG_RZV2H_RIIC5_IRQ_PRIORITY,
  .p_extend            = &g_i2c_master5_extend,
};

i2c_master_instance_t g_i2c_master5_hal_inst =
{
  .p_ctrl = &g_i2c_master5_ctrl,
  .p_cfg  = &g_i2c_master5_cfg,
  .p_api  = &g_i2c_master_on_iic
};

static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c5_priv =
{
  .ops                 = &g_rzv2h_riic_i2c_ops,
  .pin_config          = &g_rzv2h_riic_i2c5_pin_config,
  .i2c_master_hal_inst = &g_i2c_master5_hal_inst,
  .rxi_irq_event       = RZV2H_IRQSEL_RIIC5_RI,
  .txi_irq_event       = RZV2H_IRQSEL_RIIC5_TI,
  .refs                = 0,
  .lock                = NXMUTEX_INITIALIZER,
  .sem_isr             = SEM_INITIALIZER(0),
  .intstate            = INTSTATE_IDLE,
  .msgc                = 0,
  .msgv                = NULL,
  .ptr                 = NULL,
  .dcnt                = 0,
  .flags               = 0,
  .frequency           = CONFIG_RZV2H_RIIC5_BITRATE,
  .rise_time_s         = CONFIG_RZV2H_RIIC5_SCL_RISE_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .fall_time_s         = CONFIG_RZV2H_RIIC5_SCL_FALL_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .duty_cycle_percent  = CONFIG_RZV2H_RIIC5_SCL_DUTY_CYCLE,
  .noise_filter_stage  = CONFIG_RZV2H_RIIC5_NF_STAGE
};
#endif /* CONFIG_RZV2H_RIIC5 */

#ifdef CONFIG_RZV2H_RIIC6
static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c6_priv;

static struct rzv2h_riic_i2c_pin_config_s g_rzv2h_riic_i2c6_pin_config =
{
  .scl_pin    = BOARD_RIIC6_SCL_GPIO,
  .sda_pin    = BOARD_RIIC6_SDA_GPIO,
};

static iic_master_instance_ctrl_t g_i2c_master6_ctrl;

static riic_master_extended_cfg_t g_i2c_master6_extend =
{
  .timeout_mode = IIC_MASTER_TIMEOUT_MODE_SHORT,
  .timeout_scl_low = IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED,
  .noise_filter_stage = 1,
  .naki_irq = RZV2H_IRQ_RIIC6_NAKI,
  .spi_irq = RZV2H_IRQ_RIIC6_SPI,
  .sti_irq = RZV2H_IRQ_RIIC6_STI,
  .ali_irq = RZV2H_IRQ_RIIC6_ALI,
  .tmoi_irq = RZV2H_IRQ_RIIC6_TMOI,
  .p_reg = (void *)R_RIIC6,
};

static i2c_master_cfg_t g_i2c_master6_cfg =
{
  .channel             = 6,
  .rate                = I2C_MASTER_RATE_STANDARD,
  .slave               = 0x00,
  .addr_mode           = I2C_MASTER_ADDR_MODE_7BIT,
  .p_transfer_tx       = NULL,
  .p_transfer_rx       = NULL,
  .p_callback          = i2c_rz_riic_callback,
  .p_context           = &g_rzv2h_riic_i2c6_priv,
  .rxi_irq             = CONFIG_RZV2H_RIIC6_RXI_INTSEL,
  .txi_irq             = CONFIG_RZV2H_RIIC6_TXI_INTSEL,
  .tei_irq             = RZV2H_IRQ_RIIC6_TEI,
  .ipl                 = CONFIG_RZV2H_RIIC6_IRQ_PRIORITY,
  .p_extend            = &g_i2c_master6_extend,
};

i2c_master_instance_t g_i2c_master6_hal_inst =
{
  .p_ctrl = &g_i2c_master6_ctrl,
  .p_cfg  = &g_i2c_master6_cfg,
  .p_api  = &g_i2c_master_on_iic
};

static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c6_priv =
{
  .ops                 = &g_rzv2h_riic_i2c_ops,
  .pin_config          = &g_rzv2h_riic_i2c6_pin_config,
  .i2c_master_hal_inst = &g_i2c_master6_hal_inst,
  .rxi_irq_event       = RZV2H_IRQSEL_RIIC6_RI,
  .txi_irq_event       = RZV2H_IRQSEL_RIIC6_TI,
  .refs                = 0,
  .lock                = NXMUTEX_INITIALIZER,
  .sem_isr             = SEM_INITIALIZER(0),
  .intstate            = INTSTATE_IDLE,
  .msgc                = 0,
  .msgv                = NULL,
  .ptr                 = NULL,
  .dcnt                = 0,
  .flags               = 0,
  .frequency           = CONFIG_RZV2H_RIIC6_BITRATE,
  .rise_time_s         = CONFIG_RZV2H_RIIC6_SCL_RISE_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .fall_time_s         = CONFIG_RZV2H_RIIC6_SCL_FALL_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .duty_cycle_percent  = CONFIG_RZV2H_RIIC6_SCL_DUTY_CYCLE,
  .noise_filter_stage  = CONFIG_RZV2H_RIIC6_NF_STAGE
};
#endif /* CONFIG_RZV2H_RIIC6 */

#ifdef CONFIG_RZV2H_RIIC7
static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c7_priv;

static struct rzv2h_riic_i2c_pin_config_s g_rzv2h_riic_i2c7_pin_config =
{
  .scl_pin    = BOARD_RIIC7_SCL_GPIO,
  .sda_pin    = BOARD_RIIC7_SDA_GPIO,
};

static iic_master_instance_ctrl_t g_i2c_master7_ctrl;

static riic_master_extended_cfg_t g_i2c_master7_extend =
{
  .timeout_mode = IIC_MASTER_TIMEOUT_MODE_SHORT,
  .timeout_scl_low = IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED,
  .noise_filter_stage = 1,
  .naki_irq = RZV2H_IRQ_RIIC7_NAKI,
  .spi_irq = RZV2H_IRQ_RIIC7_SPI,
  .sti_irq = RZV2H_IRQ_RIIC7_STI,
  .ali_irq = RZV2H_IRQ_RIIC7_ALI,
  .tmoi_irq = RZV2H_IRQ_RIIC7_TMOI,
  .p_reg = (void *)R_RIIC7,
};

static i2c_master_cfg_t g_i2c_master7_cfg =
{
  .channel             = 7,
  .rate                = I2C_MASTER_RATE_STANDARD,
  .slave               = 0x00,
  .addr_mode           = I2C_MASTER_ADDR_MODE_7BIT,
  .p_transfer_tx       = NULL,
  .p_transfer_rx       = NULL,
  .p_callback          = i2c_rz_riic_callback,
  .p_context           = &g_rzv2h_riic_i2c7_priv,
  .rxi_irq             = CONFIG_RZV2H_RIIC7_RXI_INTSEL,
  .txi_irq             = CONFIG_RZV2H_RIIC7_TXI_INTSEL,
  .tei_irq             = RZV2H_IRQ_RIIC7_TEI,
  .ipl                 = CONFIG_RZV2H_RIIC7_IRQ_PRIORITY,
  .p_extend            = &g_i2c_master7_extend,
};

i2c_master_instance_t g_i2c_master7_hal_inst =
{
  .p_ctrl = &g_i2c_master7_ctrl,
  .p_cfg  = &g_i2c_master7_cfg,
  .p_api  = &g_i2c_master_on_iic
};

static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c7_priv =
{
  .ops                 = &g_rzv2h_riic_i2c_ops,
  .pin_config          = &g_rzv2h_riic_i2c7_pin_config,
  .i2c_master_hal_inst = &g_i2c_master7_hal_inst,
  .rxi_irq_event       = RZV2H_IRQSEL_RIIC7_RI,
  .txi_irq_event       = RZV2H_IRQSEL_RIIC7_TI,
  .refs                = 0,
  .lock                = NXMUTEX_INITIALIZER,
  .sem_isr             = SEM_INITIALIZER(0),
  .intstate            = INTSTATE_IDLE,
  .msgc                = 0,
  .msgv                = NULL,
  .ptr                 = NULL,
  .dcnt                = 0,
  .flags               = 0,
  .frequency           = CONFIG_RZV2H_RIIC7_BITRATE,
  .rise_time_s         = CONFIG_RZV2H_RIIC7_SCL_RISE_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .fall_time_s         = CONFIG_RZV2H_RIIC7_SCL_FALL_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .duty_cycle_percent  = CONFIG_RZV2H_RIIC7_SCL_DUTY_CYCLE,
  .noise_filter_stage  = CONFIG_RZV2H_RIIC7_NF_STAGE
};
#endif /* CONFIG_RZV2H_RIIC7 */

#ifdef CONFIG_RZV2H_RIIC8
static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c8_priv;

static struct rzv2h_riic_i2c_pin_config_s g_rzv2h_riic_i2c8_pin_config =
{
  .scl_pin    = BOARD_RIIC8_SCL_GPIO,
  .sda_pin    = BOARD_RIIC8_SDA_GPIO,
};

static iic_master_instance_ctrl_t g_i2c_master8_ctrl;

static riic_master_extended_cfg_t g_i2c_master8_extend =
{
  .timeout_mode = IIC_MASTER_TIMEOUT_MODE_SHORT,
  .timeout_scl_low = IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED,
  .noise_filter_stage = 1,
  .naki_irq = RZV2H_IRQ_RIIC8_NAKI,
  .spi_irq = RZV2H_IRQ_RIIC8_SPI,
  .sti_irq = RZV2H_IRQ_RIIC8_STI,
  .ali_irq = RZV2H_IRQ_RIIC8_ALI,
  .tmoi_irq = RZV2H_IRQ_RIIC8_TMOI,
  .p_reg = (void *)R_RIIC8,
};

static i2c_master_cfg_t g_i2c_master8_cfg =
{
  .channel             = 8,
  .rate                = I2C_MASTER_RATE_STANDARD,
  .slave               = 0x00,
  .addr_mode           = I2C_MASTER_ADDR_MODE_7BIT,
  .p_transfer_tx       = NULL,
  .p_transfer_rx       = NULL,
  .p_callback          = i2c_rz_riic_callback,
  .p_context           = &g_rzv2h_riic_i2c8_priv,
  .rxi_irq             = CONFIG_RZV2H_RIIC8_RXI_INTSEL,
  .txi_irq             = CONFIG_RZV2H_RIIC8_TXI_INTSEL,
  .tei_irq             = RZV2H_IRQ_RIIC8_TEI,
  .ipl                 = CONFIG_RZV2H_RIIC8_IRQ_PRIORITY,
  .p_extend            = &g_i2c_master8_extend,
};

i2c_master_instance_t g_i2c_master8_hal_inst =
{
  .p_ctrl = &g_i2c_master8_ctrl,
  .p_cfg  = &g_i2c_master8_cfg,
  .p_api  = &g_i2c_master_on_iic
};

static struct rzv2h_riic_i2c_priv_s g_rzv2h_riic_i2c8_priv =
{
  .ops                 = &g_rzv2h_riic_i2c_ops,
  .pin_config          = &g_rzv2h_riic_i2c8_pin_config,
  .i2c_master_hal_inst = &g_i2c_master8_hal_inst,
  .rxi_irq_event       = RZV2H_IRQSEL_RIIC8_RI,
  .txi_irq_event       = RZV2H_IRQSEL_RIIC8_TI,
  .refs                = 0,
  .lock                = NXMUTEX_INITIALIZER,
  .sem_isr             = SEM_INITIALIZER(0),
  .intstate            = INTSTATE_IDLE,
  .msgc                = 0,
  .msgv                = NULL,
  .ptr                 = NULL,
  .dcnt                = 0,
  .flags               = 0,
  .frequency           = CONFIG_RZV2H_RIIC8_BITRATE,
  .rise_time_s         = CONFIG_RZV2H_RIIC8_SCL_RISE_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .fall_time_s         = CONFIG_RZV2H_RIIC8_SCL_FALL_TIME /
                          RZ_RIIC_MASTER_DIV_TIME_NS,
  .duty_cycle_percent  = CONFIG_RZV2H_RIIC8_SCL_DUTY_CYCLE,
  .noise_filter_stage  = CONFIG_RZV2H_RIIC8_NF_STAGE
};
#endif /* CONFIG_RZV2H_RIIC8 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: calc_riic_master_bitrate
 *
 * Description:
 *   Calculate the actual bitrate, duty cycle, and error for a given set of
 *   RIIC clock parameters.  This is a helper used by
 *   calc_riic_master_clock_setting() to evaluate candidate divider/BRH
 *   combinations.
 *
 * Input Parameters:
 *   priv           - I2C private data containing timing parameters
 *   total_brl_brh  - Sum of BRL and BRH register values
 *   brh            - BRH register value
 *   divider        - CKS clock divider value (0-7)
 *   result         - Output structure filled with calculated bitrate info
 *
 * Returned Value:
 *   None.  Result is written to *result.
 *
 ****************************************************************************/

static void calc_riic_master_bitrate(struct rzv2h_riic_i2c_priv_s *priv,
                                      uint32_t total_brl_brh, uint32_t brh,
                                      uint32_t divider,
                                      struct rz_riic_master_bitrate *result)
{
  const uint32_t noise_filter_stages = priv->noise_filter_stage;
  const double rise_time_s = priv->rise_time_s;
  const double fall_time_s = priv->fall_time_s;
  const double requested_duty = priv->duty_cycle_percent;
  const uint32_t peripheral_clock =
    R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_P0CLK);
  uint32_t constant_add = 0;
  double divided_p0;

  /* A constant is added to BRL and BRH in all formulas.  This constant
   * is 3 + nf when CKS == 0, or 2 + nf when CKS != 0.
   */

  if (divider == 0)
    {
      constant_add = 3 + noise_filter_stages;
    }
  else
    {
      /* All dividers other than 0 use an addition of
       * 2 + noise_filter_stages.
       */

      constant_add = 2 + noise_filter_stages;
    }

  /* Converts all divided numbers to double to avoid data loss. */

  divided_p0 = (peripheral_clock >> divider);

  result->bitrate =
    1 / ((total_brl_brh + 2 * constant_add) / divided_p0 +
         rise_time_s + fall_time_s);
  result->duty =
    100 *
    ((rise_time_s + ((brh + constant_add) / divided_p0)) /
     (rise_time_s + fall_time_s +
      ((total_brl_brh + 2 * constant_add)) / divided_p0));
  result->divider = divider;
  result->brh = brh;
  result->brl = total_brl_brh - brh;
  result->duty_error_percent =
    (result->duty > requested_duty ?
     (result->duty - requested_duty) :
     (requested_duty - result->duty)) /
    requested_duty;
}

/****************************************************************************
 * Name: calc_riic_master_clock_setting
 *
 * Description:
 *   Calculate the optimal RIIC clock settings (CKS, BRL, BRH) for the
 *   requested I2C frequency.  Iterates over all CKS divider values and
 *   BRH/BRL combinations to find the best match for the requested bitrate
 *   and duty cycle.
 *
 * Input Parameters:
 *   priv      - I2C private data containing timing parameters
 *   frequency - Requested I2C frequency in Hz
 *   clk_cfg   - Output structure filled with calculated clock settings
 *
 * Returned Value:
 *   None.  Clock settings are written to *clk_cfg.
 *
 ****************************************************************************/

static void
calc_riic_master_clock_setting(struct rzv2h_riic_i2c_priv_s *priv,
                               const uint32_t frequency,
                               iic_master_clock_settings_t *clk_cfg)
{
  const uint32_t noise_filter_stages = priv->noise_filter_stage;
  const uint32_t requested_duty = priv->duty_cycle_percent;
  double rise_time_s = priv->rise_time_s;
  double fall_time_s = priv->fall_time_s;
  const uint32_t peripheral_clock =
    R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_P0CLK);
  uint32_t requested_bitrate = 0;
  uint32_t min_brh;
  uint32_t min_brl_brh;
  uint32_t constant_add;
  struct rz_riic_master_bitrate bitrate =
  {
    0
  };

  int temp_divider;

  switch (frequency)
    {
      case I2C_SPEED_STANDARD:
      case I2C_SPEED_FAST:
      case I2C_SPEED_FAST_PLUS:
        requested_bitrate = frequency;
        break;

      default:
        i2cerr("%s: Invalid I2C speed rate: %ld", __func__, frequency);
        return;
    }

  /* Store the calculated rise and fall times in the private structure */

  priv->rise_time_s = rise_time_s;
  priv->fall_time_s = fall_time_s;

  /* Start with maximum possible bitrate. */

  min_brh = noise_filter_stages + 1;
  min_brl_brh = 2 * min_brh;

  calc_riic_master_bitrate(priv, min_brl_brh, min_brh, 0, &bitrate);

  /* Start with the smallest divider because it gives the most
   * resolution.
   */

  constant_add = 3 + noise_filter_stages;

  for (temp_divider = 0; temp_divider <= 7; ++temp_divider)
    {
      double divided_p0;
      uint32_t total_brl_brh;
      uint32_t temp_brh;
      struct rz_riic_master_bitrate temp_bitrate =
      {
        0
      };

      struct rz_riic_master_bitrate test_bitrate;

      if (1 == temp_divider)
        {
          /* All dividers other than 0 use an addition of
           * 2 + noise_filter_stages.
           */

          constant_add = 2 + noise_filter_stages;
        }

      /* If the requested bitrate cannot be achieved with this divider,
       * continue.
       */

      divided_p0 = (peripheral_clock >> temp_divider);
      total_brl_brh =
        ceil(((1 / (double)requested_bitrate) -
              (rise_time_s + fall_time_s)) *
             divided_p0 - (2 * constant_add));

      if ((total_brl_brh > 62) || (total_brl_brh < min_brl_brh))
        {
          continue;
        }

      temp_brh = total_brl_brh * requested_duty / 100;

      if (temp_brh < min_brh)
        {
          temp_brh = min_brh;
        }

      /* Calculate the actual bitrate and duty cycle. */

      calc_riic_master_bitrate(priv, total_brl_brh, temp_brh,
                                temp_divider, &temp_bitrate);

      /* Adjust duty cycle down if it helps. */

      test_bitrate = temp_bitrate;

      while (test_bitrate.duty > requested_duty)
        {
          struct rz_riic_master_bitrate new_bitrate =
          {
            0
          };

          temp_brh -= 1;

          if ((temp_brh < min_brh) ||
              ((total_brl_brh - temp_brh) > 31))
            {
              break;
            }

          calc_riic_master_bitrate(priv, total_brl_brh, temp_brh,
                                    temp_divider, &new_bitrate);

          if (new_bitrate.duty_error_percent <
              temp_bitrate.duty_error_percent)
            {
              temp_bitrate = new_bitrate;
            }
          else
            {
              break;
            }
        }

      /* Adjust duty cycle up if it helps. */

      while (test_bitrate.duty < requested_duty)
        {
          struct rz_riic_master_bitrate new_bitrate =
          {
            0
          };

          ++temp_brh;

          if ((temp_brh > total_brl_brh) || (temp_brh > 31) ||
              ((total_brl_brh - temp_brh) < min_brh))
            {
              break;
            }

          calc_riic_master_bitrate(priv, total_brl_brh, temp_brh,
                                    temp_divider, &new_bitrate);

          if (new_bitrate.duty_error_percent <
              temp_bitrate.duty_error_percent)
            {
              temp_bitrate = new_bitrate;
            }
          else
            {
              break;
            }
        }

      if ((temp_bitrate.brh < 32) && (temp_bitrate.brl < 32))
        {
          /* Valid setting found. */

          bitrate = temp_bitrate;
          break;
        }
    }

  clk_cfg->brl_value = bitrate.brl;
  clk_cfg->brh_value = bitrate.brh;
  clk_cfg->cks_value = bitrate.divider;
}

/****************************************************************************
 * Name: rzv2h_riic_i2c_irq_attach
 *
 * Description:
 *   Attach and configure all RIIC interrupt handlers for this channel.
 *   Converts FSP IRQn_Type to GIC INTIDs, connects RXI/TXI SEL events,
 *   attaches NuttX handlers, and sets interrupt priorities.
 *
 * Input Parameters:
 *   priv - I2C private data structure for the channel
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int rzv2h_riic_i2c_irq_attach(struct rzv2h_riic_i2c_priv_s *priv)
{
  const i2c_master_cfg_t *cfg = priv->i2c_master_hal_inst->p_cfg;
  riic_master_extended_cfg_t *ext =
    (riic_master_extended_cfg_t *)cfg->p_extend;
  rzv2h_irqn_t rxi_sel = (rzv2h_irqn_t)cfg->rxi_irq;
  rzv2h_irqn_t txi_sel = (rzv2h_irqn_t)cfg->txi_irq;

  int rxi_irq  = RZV2H_FSP_TO_GIC_IRQ(rxi_sel);
  int txi_irq  = RZV2H_FSP_TO_GIC_IRQ(txi_sel);
  int tei_irq  = RZV2H_FSP_TO_GIC_IRQ(cfg->tei_irq);
  int naki_irq = RZV2H_FSP_TO_GIC_IRQ(ext->naki_irq);
  int spi_irq  = RZV2H_FSP_TO_GIC_IRQ(ext->spi_irq);
  int sti_irq  = RZV2H_FSP_TO_GIC_IRQ(ext->sti_irq);
  int ali_irq  = RZV2H_FSP_TO_GIC_IRQ(ext->ali_irq);
  int tmoi_irq = RZV2H_FSP_TO_GIC_IRQ(ext->tmoi_irq);
  int ret;

  /* Keep the IRQs disabled until their NuttX handlers are attached and FSP
   * has installed its contexts.
   */

  up_disable_irq(rxi_irq);
  up_disable_irq(txi_irq);
  up_disable_irq(tei_irq);
  up_disable_irq(naki_irq);
  up_disable_irq(spi_irq);
  up_disable_irq(sti_irq);
  up_disable_irq(ali_irq);
  up_disable_irq(tmoi_irq);

  /* Connect the statically-assigned SEL lines (optionally overridden via
   * Kconfig) to their events.  RXI/TXI are both rising-edge triggered.
   */

  if (rzv2h_intsel_connect_event(rxi_sel, priv->rxi_irq_event,
                            BSP_GIC_SPI_DETECT_EDGE) < 0 ||
      rzv2h_intsel_connect_event(txi_sel, priv->txi_irq_event,
                            BSP_GIC_SPI_DETECT_EDGE) < 0)
    {
      rzv2h_intsel_disconnect_event(rxi_sel);
      rzv2h_intsel_disconnect_event(txi_sel);
      return ERROR;
    }

  ret = irq_attach(rxi_irq, rzv2h_riic_i2c_interrupt, priv);
  if (ret < 0)
    {
      goto errout_events;
    }

  ret = irq_attach(txi_irq, rzv2h_riic_i2c_interrupt, priv);
  if (ret < 0)
    {
      goto errout_rxi;
    }

  ret = irq_attach(tei_irq, rzv2h_riic_i2c_interrupt, priv);
  if (ret < 0)
    {
      goto errout_txi;
    }

  ret = irq_attach(naki_irq, rzv2h_riic_i2c_interrupt, priv);
  if (ret < 0)
    {
      goto errout_tei;
    }

  ret = irq_attach(spi_irq, rzv2h_riic_i2c_interrupt, priv);
  if (ret < 0)
    {
      goto errout_naki;
    }

  ret = irq_attach(sti_irq, rzv2h_riic_i2c_interrupt, priv);
  if (ret < 0)
    {
      goto errout_spi;
    }

  ret = irq_attach(ali_irq, rzv2h_riic_i2c_interrupt, priv);
  if (ret < 0)
    {
      goto errout_sti;
    }

  ret = irq_attach(tmoi_irq, rzv2h_riic_i2c_interrupt, priv);
  if (ret < 0)
    {
      goto errout_ali;
    }

  /* Leave the IRQs disabled here.  R_RIIC_MASTER_Open() configures and
   * enables the IRQs via R_BSP_IrqCfgEnable().
   */

  return OK;

errout_ali:
  irq_detach(ali_irq);
errout_sti:
  irq_detach(sti_irq);
errout_spi:
  irq_detach(spi_irq);
errout_naki:
  irq_detach(naki_irq);
errout_tei:
  irq_detach(tei_irq);
errout_txi:
  irq_detach(txi_irq);
errout_rxi:
  irq_detach(rxi_irq);
errout_events:
  rzv2h_intsel_disconnect_event(txi_sel);
  rzv2h_intsel_disconnect_event(rxi_sel);
  return ret;
}

/****************************************************************************
 * Name: rzv2h_riic_i2c_irq_detach
 *
 * Description:
 *   Detach all RIIC interrupt handlers and release SEL event routing.
 *   Disables and detaches all 8 RIIC IRQs (RXI, TXI, TEI, NAKI, SPI,
 *   STI, ALI, TMOI) and disconnects the RXI/TXI SEL event lines.
 *
 * Input Parameters:
 *   priv - I2C private data structure for the channel
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_riic_i2c_irq_detach(struct rzv2h_riic_i2c_priv_s *priv)
{
  const i2c_master_cfg_t *cfg = priv->i2c_master_hal_inst->p_cfg;
  riic_master_extended_cfg_t *ext =
    (riic_master_extended_cfg_t *)cfg->p_extend;

  int rxi_irq  = RZV2H_FSP_TO_GIC_IRQ(cfg->rxi_irq);
  int txi_irq  = RZV2H_FSP_TO_GIC_IRQ(cfg->txi_irq);
  int tei_irq  = RZV2H_FSP_TO_GIC_IRQ(cfg->tei_irq);
  int naki_irq = RZV2H_FSP_TO_GIC_IRQ(ext->naki_irq);
  int spi_irq  = RZV2H_FSP_TO_GIC_IRQ(ext->spi_irq);
  int sti_irq  = RZV2H_FSP_TO_GIC_IRQ(ext->sti_irq);
  int ali_irq  = RZV2H_FSP_TO_GIC_IRQ(ext->ali_irq);
  int tmoi_irq = RZV2H_FSP_TO_GIC_IRQ(ext->tmoi_irq);

  up_disable_irq(rxi_irq);
  up_disable_irq(txi_irq);
  up_disable_irq(tei_irq);
  up_disable_irq(naki_irq);
  up_disable_irq(spi_irq);
  up_disable_irq(sti_irq);
  up_disable_irq(ali_irq);
  up_disable_irq(tmoi_irq);

  irq_detach(rxi_irq);
  irq_detach(txi_irq);
  irq_detach(tei_irq);
  irq_detach(naki_irq);
  irq_detach(spi_irq);
  irq_detach(sti_irq);
  irq_detach(ali_irq);
  irq_detach(tmoi_irq);

  rzv2h_intsel_disconnect_event((rzv2h_irqn_t)cfg->rxi_irq);
  rzv2h_intsel_disconnect_event((rzv2h_irqn_t)cfg->txi_irq);
}

/****************************************************************************
 * Name: rzv2h_riic_i2c_init
 *
 * Description:
 *   Initialize the I2C hardware for operation.  Configures GPIO pins for
 *   I2C peripheral function, attaches all RIIC IRQs, calculates clock
 *   settings for the configured frequency, and opens the FSP RIIC module.
 *
 * Input Parameters:
 *   priv - I2C private data structure for the channel
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int rzv2h_riic_i2c_init(struct rzv2h_riic_i2c_priv_s *priv)
{
  riic_master_extended_cfg_t *hal_extended_cfg =
    (riic_master_extended_cfg_t *)
    priv->i2c_master_hal_inst->p_cfg->p_extend;
  int ret;

  /* Configure I2C pins for peripheral function */

  rzv2h_configgpio(priv->pin_config->scl_pin);
  rzv2h_configgpio(priv->pin_config->sda_pin);

  /* Attach all RIIC IRQs with proper error handling and priority setup */

  ret = rzv2h_riic_i2c_irq_attach(priv);
  if (ret < 0)
    {
      return ret;
    }

  calc_riic_master_clock_setting(priv, priv->frequency,
                                  &hal_extended_cfg->clock_settings);

  /* Open the FSP module.  R_RIIC_MASTER_Open() calls R_BSP_IrqCfgEnable()
   * for each of this channel's 8 IRQs, which sets the GIC detect type
   * from g_gic_detect_type[] (see R_BSP_IrqCfg()).  The fixed
   * TEI/NAKI/SPI/STI/ALI/TMOI vectors already default to the correct
   * type at compile time; RXI/TXI were just recorded as edge
   * type in g_gic_detect_type[] by rzv2h_intsel_connect_event() in
   * rzv2h_riic_i2c_irq_attach(), so Open() applies the correct type for all
   * 8 IRQs on its own -- no separate up_set_irq_type() pass is needed.
   */

  if (R_RIIC_MASTER_Open(priv->i2c_master_hal_inst->p_ctrl,
                        priv->i2c_master_hal_inst->p_cfg) != FSP_SUCCESS)
    {
      R_RIIC_MASTER_Close(priv->i2c_master_hal_inst->p_ctrl);
      rzv2h_riic_i2c_irq_detach(priv);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_riic_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware.  Closes the FSP RIIC module, detaches all
 *   RIIC IRQs, and releases SEL event routing.  After this call, the I2C
 *   peripheral is powered down and cannot be used until re-initialized.
 *
 * Input Parameters:
 *   priv - I2C private data structure for the channel
 *
 * Returned Value:
 *   OK on success, ERROR if FSP close fails
 *
 ****************************************************************************/

static int rzv2h_riic_i2c_deinit(struct rzv2h_riic_i2c_priv_s *priv)
{
  /* Close I2C FSP module */

  if (R_RIIC_MASTER_Close(priv->i2c_master_hal_inst->p_ctrl) != FSP_SUCCESS)
    {
      return ERROR;
    }

  /* Detach all IRQs and release SEL event routing */

  rzv2h_riic_i2c_irq_detach(priv);

  return OK;
}

/****************************************************************************
 * Name: rzv2h_riic_i2c_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to recover a stuck bus.  This
 *   function bit-bangs SCL to clock out any stuck slave, then generates a
 *   START+STOP sequence to reset slave state machines.  The I2C peripheral
 *   is de-initialized before the bit-bang and re-initialized afterwards.
 *
 * Input Parameters:
 *   dev - I2C master device instance
 *
 * Returned Value:
 *   OK on success, -EIO on failure
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int rzv2h_riic_i2c_reset(struct i2c_master_s *dev)
{
  struct rzv2h_riic_i2c_priv_s *priv;
  gpio_pinset_t scl_gpio;
  gpio_pinset_t sda_gpio;
  unsigned int clock_count;
  unsigned int stretch_count;
  int ret;

  DEBUGASSERT(dev);

  /* Get I2C private structure */

  priv = (struct rzv2h_riic_i2c_priv_s *)dev;

  /* Our caller must own a ref */

  DEBUGASSERT(priv->refs > 0);

  /* Lock out other clients */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EIO;

  /* De-init the port (close FSP module, detach IRQs) */

  rzv2h_riic_i2c_deinit(priv);

  /* Use GPIO configuration to recover the bus.  Build GPIO output
   * pinset from the stored I2C pin's port_pin, with output
   * and input enabled (to allow reading SDA/SCL state).
   */

  scl_gpio.port_pin = priv->pin_config->scl_pin.port_pin;
  scl_gpio.cfg = IOPORT_CFG_PORT_DIRECTION_OUTPUT_INPUT |
                 IOPORT_CFG_NOD_ENABLE |
                 IOPORT_CFG_PORT_OUTPUT_HIGH;

  sda_gpio.port_pin = priv->pin_config->sda_pin.port_pin;
  sda_gpio.cfg = IOPORT_CFG_PORT_DIRECTION_OUTPUT_INPUT |
                 IOPORT_CFG_NOD_ENABLE |
                 IOPORT_CFG_PORT_OUTPUT_HIGH;

  rzv2h_configgpio(sda_gpio);
  rzv2h_configgpio(scl_gpio);

  /* Let SDA go high */

  rzv2h_gpiowrite(sda_gpio, true);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!rzv2h_gpioread(sda_gpio))
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
      while (!rzv2h_gpioread(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      rzv2h_gpiowrite(scl_gpio, false);
      up_udelay(10);

      /* Drive SCL high again */

      rzv2h_gpiowrite(scl_gpio, true);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave state machines. */

  rzv2h_gpiowrite(sda_gpio, false);
  up_udelay(10);
  rzv2h_gpiowrite(scl_gpio, false);
  up_udelay(10);
  rzv2h_gpiowrite(scl_gpio, true);
  up_udelay(10);
  rzv2h_gpiowrite(sda_gpio, true);
  up_udelay(10);

  ret = OK;

out:

  if (rzv2h_riic_i2c_init(priv) < 0)
    {
      ret = -EIO;
    }

  /* Release the port for reuse by other clients */

  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_riic_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers.  Merges consecutive same-direction
 *   messages into a single FSP call to avoid spurious STOP conditions.
 *   Handles frequency changes by closing and reopening the FSP module.
 *
 * Input Parameters:
 *   dev   - I2C master device instance
 *   msgs  - Array of I2C messages to transfer
 *   count - Number of messages in the array
 *
 * Returned Value:
 *   OK on success, negative errno on failure (-EIO, -ETIMEDOUT, -ENXIO,
 *   -EAGAIN, -EINVAL, -ENOMEM)
 *
 ****************************************************************************/

static int rzv2h_riic_i2c_transfer(struct i2c_master_s *dev,
                                   struct i2c_msg_s *msgs, int count)
{
  struct rzv2h_riic_i2c_priv_s *priv = (struct rzv2h_riic_i2c_priv_s *)dev;
  riic_master_extended_cfg_t *hal_extended_cfg =
    (riic_master_extended_cfg_t *)
    priv->i2c_master_hal_inst->p_cfg->p_extend;
  int ret;
  fsp_err_t err;
  int i;

  DEBUGASSERT(count > 0);

  /* Ensure that address or flags don't change meanwhile */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < count; i++)
    {
      i2c_master_addr_mode_t addr_mode;
      int merge_count;
      uint8_t *buf;
      uint32_t len;
      uint8_t tmpbuf[CONFIG_RZV2H_RIIC_I2C_MERGE_BUFSIZE];
      int j;
      bool restart;

      /* Reconfigure the clock if the current frequency does not match
       * the frequency specified in the message.  FSP requires a
       * close/reopen cycle for rate changes (matches Zephyr
       * i2c_renesas_rz_riic.c pattern).
       */

      if (priv->frequency != msgs[i].frequency)
        {
          priv->frequency = msgs[i].frequency;
          calc_riic_master_clock_setting(priv, priv->frequency,
                                          &hal_extended_cfg->
                                          clock_settings);

          /* Close before reopen for rate change */

          R_RIIC_MASTER_Close(priv->i2c_master_hal_inst->p_ctrl);

          if (R_RIIC_MASTER_Open(priv->i2c_master_hal_inst->p_ctrl,
                                priv->i2c_master_hal_inst->p_cfg) !=
                                FSP_SUCCESS)
            {
              ret = -EIO;
              goto release_bus;
            }
        }

      /* Set destination address with configured address mode before
       * sending msg.
       */

      if (msgs[i].flags & I2C_M_TEN)
        {
          addr_mode = I2C_MASTER_ADDR_MODE_10BIT;
        }
      else
        {
          addr_mode = I2C_MASTER_ADDR_MODE_7BIT;
        }

      err = R_RIIC_MASTER_SlaveAddressSet(priv->i2c_master_hal_inst->
                                           p_ctrl, msgs[i].addr,
                                           addr_mode);
      if (err != FSP_SUCCESS)
        {
          i2cerr("Failed to set slave address");
          ret = rzv2h_fsp_err_to_errno(err);
          goto release_bus;
        }

      /* Find how many consecutive same-direction messages can be
       * merged.  The RIIC hardware does not generate a proper RESTART
       * between two same-direction messages (e.g. write->write for
       * register + data).  Merge consecutive same-direction messages
       * into a single FSP call so the RIIC sends all bytes without
       * STOP between them.
       */

      merge_count = 1;

      while ((i + merge_count) < count)
        {
          bool cur_is_read =
            (msgs[i + merge_count - 1].flags & I2C_M_READ) != 0;
          bool next_is_read =
            (msgs[i + merge_count].flags & I2C_M_READ) != 0;

          if (cur_is_read != next_is_read)
            {
              break;
            }

          merge_count++;
        }

      /* Build the transfer buffer.  For a single message use the
       * original buffer directly; for merged messages copy into a
       * temp buffer on the stack.  The semaphore wait below guarantees
       * the FSP has finished reading the buffer before it goes out of
       * scope.
       */

      if (merge_count == 1)
        {
          buf = msgs[i].buffer;
          len = msgs[i].length;
        }
      else
        {
          len = 0;

          for (j = 0; j < merge_count; j++)
            {
              if (len + msgs[i + j].length > sizeof(tmpbuf))
                {
                  i2cerr("Merge buffer overflow\n");
                  ret = -ENOMEM;
                  goto release_bus;
                }

              memcpy(&tmpbuf[len], msgs[i + j].buffer,
                     msgs[i + j].length);
              len += msgs[i + j].length;
            }

          buf = tmpbuf;
        }

      /* Determine if a RESTART (no STOP) should be issued after this
       * (possibly merged) message.  RESTART is only needed when a
       * direction change follows (write->read or read->write).
       */

      restart = false;

      if ((i + merge_count) < count)
        {
          bool cur_read = (msgs[i].flags & I2C_M_READ) != 0;
          bool next_read =
            (msgs[i + merge_count].flags & I2C_M_READ) != 0;

          if (cur_read != next_read)
            {
              restart = true;
            }
        }

      if (msgs[i].flags & I2C_M_READ)
        {
          err = R_RIIC_MASTER_Read(priv->i2c_master_hal_inst->p_ctrl,
                                    buf, len, restart);
        }
      else
        {
          err = R_RIIC_MASTER_Write(priv->i2c_master_hal_inst->p_ctrl,
                                     buf, len, restart);
        }

      if (err != FSP_SUCCESS)
        {
          i2cerr("I2C transfer failed: FSP_ERR=%d\n", err);
          ret = rzv2h_fsp_err_to_errno(err);
          goto release_bus;
        }

      /* Wait for callback to return. */

      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                           RZV2H_RIIC_I2C_TIMEOTICKS);
      if (ret < 0)
        {
          i2cerr("%s: semaphore wait timeout.\n", __func__);
          goto release_bus;
        }

      /* Handle event msg from callback. */

      switch (priv->event)
        {
          case I2C_MASTER_EVENT_ABORTED:
            i2cerr("%s: %s failed.", __func__,
                   (msgs[i].flags & I2C_M_READ) ? "Read" : "Write");
            ret = -EIO;
            goto release_bus;

          case I2C_MASTER_EVENT_RX_COMPLETE:
            break;

          case I2C_MASTER_EVENT_TX_COMPLETE:
            break;

          default:
            break;
        }

      /* Skip the messages that were merged */

      i += (merge_count - 1);
    }

release_bus:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_riic_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus.  On first call for a given port, initializes
 *   the hardware via rzv2h_riic_i2c_init().  Increments the reference count
 *   on subsequent calls.
 *
 * Input Parameters:
 *   port - I2C port number (0-8)
 *
 * Returned Value:
 *   I2C master device instance on success, NULL on failure
 *
 ****************************************************************************/

struct i2c_master_s *rzv2h_riic_i2cbus_initialize(int port)
{
  struct rzv2h_riic_i2c_priv_s *priv = NULL;

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_RZV2H_RIIC0
      case 0:
        priv = (struct rzv2h_riic_i2c_priv_s *)&g_rzv2h_riic_i2c0_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_RIIC1
      case 1:
        priv = (struct rzv2h_riic_i2c_priv_s *)&g_rzv2h_riic_i2c1_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_RIIC2
      case 2:
        priv = (struct rzv2h_riic_i2c_priv_s *)&g_rzv2h_riic_i2c2_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_RIIC3
      case 3:
        priv = (struct rzv2h_riic_i2c_priv_s *)&g_rzv2h_riic_i2c3_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_RIIC4
      case 4:
        priv = (struct rzv2h_riic_i2c_priv_s *)&g_rzv2h_riic_i2c4_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_RIIC5
      case 5:
        priv = (struct rzv2h_riic_i2c_priv_s *)&g_rzv2h_riic_i2c5_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_RIIC6
      case 6:
        priv = (struct rzv2h_riic_i2c_priv_s *)&g_rzv2h_riic_i2c6_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_RIIC7
      case 7:
        priv = (struct rzv2h_riic_i2c_priv_s *)&g_rzv2h_riic_i2c7_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_RIIC8
      case 8:
        priv = (struct rzv2h_riic_i2c_priv_s *)&g_rzv2h_riic_i2c8_priv;
        break;
#endif
      default:
        return NULL;
    }

  /* Initialize private data for the first time, increment reference count,
   * power-up hardware and configure GPIOs.
   */

  nxmutex_lock(&priv->lock);
  if (priv->refs++ == 0)
    {
      if (rzv2h_riic_i2c_init(priv) < 0)
        {
          priv->refs--;
          nxmutex_unlock(&priv->lock);
          return NULL;
        }
    }

  nxmutex_unlock(&priv->lock);
  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: rzv2h_riic_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus.  Decrements the reference count and shuts
 *   down the hardware when the last reference is released.
 *
 * Input Parameters:
 *   dev - I2C master device instance
 *
 * Returned Value:
 *   OK on success, ERROR if reference count underflow
 *
 ****************************************************************************/

int rzv2h_riic_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct rzv2h_riic_i2c_priv_s *priv = (struct rzv2h_riic_i2c_priv_s *)dev;

  DEBUGASSERT(dev);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  nxmutex_lock(&priv->lock);
  if (--priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* Shut down the I2C hardware */

  rzv2h_riic_i2c_deinit(priv);
  nxmutex_unlock(&priv->lock);

  return OK;
}
#endif /* CONFIG_RZV2H_RIIC0 || CONFIG_RZV2H_RIIC1 || ... */

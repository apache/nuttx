/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_sci_i2c.c
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include "rzv2h_gpio.h"
#include "hardware/rzv2h_pinmap.h"

#include <arch/board/board.h>

#include "arm_internal.h"

#include "r_sci_b_i2c.h"
#include "rzv2h_sci_i2c.h"
#include "rzv2h_irq.h"
#include "rzv2h_fsp_err.h"
#include "r_ioport_api.h"
#include "r_ioport.h"

/* At least one SCI channel must be used in simple I2C mode */

#if defined(CONFIG_RZV2H_SCI_I2C0) || defined(CONFIG_RZV2H_SCI_I2C1) || \
    defined(CONFIG_RZV2H_SCI_I2C2) || defined(CONFIG_RZV2H_SCI_I2C3) || \
    defined(CONFIG_RZV2H_SCI_I2C4) || defined(CONFIG_RZV2H_SCI_I2C5) || \
    defined(CONFIG_RZV2H_SCI_I2C6) || defined(CONFIG_RZV2H_SCI_I2C7) || \
    defined(CONFIG_RZV2H_SCI_I2C8) || defined(CONFIG_RZV2H_SCI_I2C9)

/* This driver uses the FSP SCI-B I2C HAL which is interrupt-driven via
 * callbacks.  Polled mode is not supported.
 */

#ifdef CONFIG_I2C_POLLED
#  error "CONFIG_I2C_POLLED is not supported by the SCI I2C driver"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define RZV2H_SCI_I2C_TIMEOTICKS \
  (SEC2TICK(CONFIG_RZV2H_SCI_I2C_TIMEOSEC) + \
   MSEC2TICK(CONFIG_RZV2H_SCI_I2C_TIMEOMS))

/* Simple I2C mode bit rate (RSCI chapter of the hardware manual):
 *
 *   B = TCLK / (64 * 2^(2n - 1) * (N + 1)) = TCLK / (32 * 4^n * (N + 1))
 *
 * where n is CCR2.CKS (0-3) and N is CCR2.BRR (0-255).  With bit rate
 * modulation enabled B is additionally scaled by MDDR / 256, where MDDR is
 * 128-255 (256 means modulation disabled).
 */

#define RZV2H_SCI_I2C_CKS_MAX         3
#define RZV2H_SCI_I2C_BRR_MAX         255
#define RZV2H_SCI_I2C_BRR_DIV(cks)    (32u << (2 * (cks)))
#define RZV2H_SCI_I2C_MDDR_MIN        128
#define RZV2H_SCI_I2C_MDDR_DISABLED   256
#define RZV2H_SCI_I2C_MDDR_REG_MAX    255

/* ICR.IICDL must be 1-31 in simple I2C mode (0 is prohibited) */

#define RZV2H_SCI_I2C_IICDL_MIN       1
#define RZV2H_SCI_I2C_IICDL_MAX       31

/* Address limits */

#define RZV2H_SCI_I2C_7BIT_ADDR_MAX   0x7f
#define RZV2H_SCI_I2C_10BIT_ADDR_MAX  0x3ff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C pins configuration */

struct rzv2h_sci_i2c_pin_config_s
{
  gpio_pinset_t scl_pin;              /* GPIO configuration for SCL (RXDn) */
  gpio_pinset_t sda_pin;              /* GPIO configuration for SDA (TXDn) */
};

/* SCI I2C Device Private Data */

struct rzv2h_sci_i2c_priv_s
{
  /* Standard I2C operations (must be first) */

  const struct i2c_ops_s *ops;

  /* Pin configuration */

  const struct rzv2h_sci_i2c_pin_config_s *pin_config;

  /* FSP r_sci_b_i2c instance */

  sci_b_i2c_instance_ctrl_t ctrl;     /* FSP control block */
  sci_b_i2c_extended_cfg_t extend;    /* FSP extended configuration */
  i2c_master_cfg_t cfg;               /* FSP configuration */

  int refs;                           /* Reference count */
  mutex_t lock;                       /* Mutual exclusion lock */
  sem_t sem_isr;                      /* Interrupt wait semaphore */
  volatile i2c_master_event_t event;  /* Last event reported by FSP */

  uint32_t frequency;                 /* Requested rate  */
  uint32_t sda_delay_ns;              /* Requested SDA output delay */
  bool bitrate_modulation;            /* Use MDDR bit rate modulation */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rzv2h_sci_i2c_interrupt(int irq, void *context, void *arg);
static void rzv2h_sci_i2c_callback(i2c_master_callback_args_t *p_args);
static int rzv2h_sci_i2c_calc_clock(uint32_t frequency,
                                      uint32_t sda_delay_ns,
                                      bool modulation,
                                      sci_b_i2c_clock_settings_t *settings,
                                      uint32_t *actual);
static int rzv2h_sci_i2c_configure(struct rzv2h_sci_i2c_priv_s *priv,
                                     uint32_t frequency);
static int rzv2h_sci_i2c_set_frequency(struct rzv2h_sci_i2c_priv_s *priv,
                                         uint32_t frequency);
static int rzv2h_sci_i2c_irq_attach(struct rzv2h_sci_i2c_priv_s *priv);
static void rzv2h_sci_i2c_irq_detach(struct rzv2h_sci_i2c_priv_s *priv);
static int rzv2h_sci_i2c_init(struct rzv2h_sci_i2c_priv_s *priv);
static int rzv2h_sci_i2c_deinit(struct rzv2h_sci_i2c_priv_s *priv);
static bool rzv2h_sci_i2c_can_merge(const struct i2c_msg_s *prev,
                                      const struct i2c_msg_s *next);
static int rzv2h_sci_i2c_merge_count(const struct i2c_msg_s *msgs,
                                       int count, uint32_t *length);
static int rzv2h_sci_i2c_check_msgs(const struct i2c_msg_s *msgs,
                                      int count, uint32_t *max_merged_len);
static int rzv2h_sci_i2c_segment(struct rzv2h_sci_i2c_priv_s *priv,
                                   struct i2c_msg_s *msgs, int nmsgs,
                                   uint32_t length, bool restart,
                                   uint8_t *bounce);
static int rzv2h_sci_i2c_transfer(struct i2c_master_s *dev,
                                    struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int rzv2h_sci_i2c_reset(struct i2c_master_s *dev);
#endif

/* FSP r_sci_b_i2c interrupt service routines (not exported by the FSP
 * header).
 */

void sci_b_i2c_txi_isr(void);
void sci_b_i2c_tei_isr(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C interface */

static const struct i2c_ops_s g_rzv2h_sci_i2c_ops =
{
  .transfer = rzv2h_sci_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = rzv2h_sci_i2c_reset
#endif
};

#ifdef CONFIG_RZV2H_SCI_I2C0
static const struct rzv2h_sci_i2c_pin_config_s
  g_rzv2h_sci_i2c0_pin_config =
{
  .scl_pin            = BOARD_SCI_I2C0_SCL_GPIO,
  .sda_pin            = BOARD_SCI_I2C0_SDA_GPIO,
};

static struct rzv2h_sci_i2c_priv_s g_rzv2h_sci_i2c0_priv =
{
  .ops                = &g_rzv2h_sci_i2c_ops,
  .pin_config         = &g_rzv2h_sci_i2c0_pin_config,
  .extend             =
  {
    .clock_settings   =
    {
      .snfr_value     = CONFIG_RZV2H_SCI_I2C0_NF_CLK,
      .clock_source   = SCI_B_I2C_CLOCK_SOURCE_SCISPICLK,
    },
    .p_reg            = (void *)R_SCI0,
  },
  .cfg                =
  {
    .channel          = 0,
    .rate             = I2C_MASTER_RATE_STANDARD,
    .slave            = 0x00,
    .addr_mode        = I2C_MASTER_ADDR_MODE_7BIT,
    .ipl              = CONFIG_RZV2H_SCI_I2C0_IRQ_PRIORITY,
    .rxi_irq          = RZV2H_IRQ_SCI0_RXI, /* Unused: no DMAC */
    .txi_irq          = RZV2H_IRQ_SCI0_TXI,
    .tei_irq          = RZV2H_IRQ_SCI0_TEI,
    .eri_irq          = RZV2H_IRQ_SCI0_ERI, /* Unused in simple I2C */
    .p_transfer_tx    = NULL,
    .p_transfer_rx    = NULL,
    .p_callback       = rzv2h_sci_i2c_callback,
    .p_context        = &g_rzv2h_sci_i2c0_priv,
    .p_extend         = &g_rzv2h_sci_i2c0_priv.extend,
  },
  .refs               = 0,
  .lock               = NXMUTEX_INITIALIZER,
  .sem_isr            = SEM_INITIALIZER(0),
  .frequency          = CONFIG_RZV2H_SCI_I2C0_BITRATE,
  .sda_delay_ns       = CONFIG_RZV2H_SCI_I2C0_SDA_DELAY,
#ifdef CONFIG_RZV2H_SCI_I2C0_BITRATE_MODULATION
  .bitrate_modulation = true,
#else
  .bitrate_modulation = false,
#endif
};
#endif /* CONFIG_RZV2H_SCI_I2C0 */

#ifdef CONFIG_RZV2H_SCI_I2C1
static const struct rzv2h_sci_i2c_pin_config_s
  g_rzv2h_sci_i2c1_pin_config =
{
  .scl_pin            = BOARD_SCI_I2C1_SCL_GPIO,
  .sda_pin            = BOARD_SCI_I2C1_SDA_GPIO,
};

static struct rzv2h_sci_i2c_priv_s g_rzv2h_sci_i2c1_priv =
{
  .ops                = &g_rzv2h_sci_i2c_ops,
  .pin_config         = &g_rzv2h_sci_i2c1_pin_config,
  .extend             =
  {
    .clock_settings   =
    {
      .snfr_value     = CONFIG_RZV2H_SCI_I2C1_NF_CLK,
      .clock_source   = SCI_B_I2C_CLOCK_SOURCE_SCISPICLK,
    },
    .p_reg            = (void *)R_SCI1,
  },
  .cfg                =
  {
    .channel          = 1,
    .rate             = I2C_MASTER_RATE_STANDARD,
    .slave            = 0x00,
    .addr_mode        = I2C_MASTER_ADDR_MODE_7BIT,
    .ipl              = CONFIG_RZV2H_SCI_I2C1_IRQ_PRIORITY,
    .rxi_irq          = RZV2H_IRQ_SCI1_RXI, /* Unused: no DMAC */
    .txi_irq          = RZV2H_IRQ_SCI1_TXI,
    .tei_irq          = RZV2H_IRQ_SCI1_TEI,
    .eri_irq          = RZV2H_IRQ_SCI1_ERI, /* Unused in simple I2C */
    .p_transfer_tx    = NULL,
    .p_transfer_rx    = NULL,
    .p_callback       = rzv2h_sci_i2c_callback,
    .p_context        = &g_rzv2h_sci_i2c1_priv,
    .p_extend         = &g_rzv2h_sci_i2c1_priv.extend,
  },
  .refs               = 0,
  .lock               = NXMUTEX_INITIALIZER,
  .sem_isr            = SEM_INITIALIZER(0),
  .frequency          = CONFIG_RZV2H_SCI_I2C1_BITRATE,
  .sda_delay_ns       = CONFIG_RZV2H_SCI_I2C1_SDA_DELAY,
#ifdef CONFIG_RZV2H_SCI_I2C1_BITRATE_MODULATION
  .bitrate_modulation = true,
#else
  .bitrate_modulation = false,
#endif
};
#endif /* CONFIG_RZV2H_SCI_I2C1 */

#ifdef CONFIG_RZV2H_SCI_I2C2
static const struct rzv2h_sci_i2c_pin_config_s
  g_rzv2h_sci_i2c2_pin_config =
{
  .scl_pin            = BOARD_SCI_I2C2_SCL_GPIO,
  .sda_pin            = BOARD_SCI_I2C2_SDA_GPIO,
};

static struct rzv2h_sci_i2c_priv_s g_rzv2h_sci_i2c2_priv =
{
  .ops                = &g_rzv2h_sci_i2c_ops,
  .pin_config         = &g_rzv2h_sci_i2c2_pin_config,
  .extend             =
  {
    .clock_settings   =
    {
      .snfr_value     = CONFIG_RZV2H_SCI_I2C2_NF_CLK,
      .clock_source   = SCI_B_I2C_CLOCK_SOURCE_SCISPICLK,
    },
    .p_reg            = (void *)R_SCI2,
  },
  .cfg                =
  {
    .channel          = 2,
    .rate             = I2C_MASTER_RATE_STANDARD,
    .slave            = 0x00,
    .addr_mode        = I2C_MASTER_ADDR_MODE_7BIT,
    .ipl              = CONFIG_RZV2H_SCI_I2C2_IRQ_PRIORITY,
    .rxi_irq          = RZV2H_IRQ_SCI2_RXI, /* Unused: no DMAC */
    .txi_irq          = RZV2H_IRQ_SCI2_TXI,
    .tei_irq          = RZV2H_IRQ_SCI2_TEI,
    .eri_irq          = RZV2H_IRQ_SCI2_ERI, /* Unused in simple I2C */
    .p_transfer_tx    = NULL,
    .p_transfer_rx    = NULL,
    .p_callback       = rzv2h_sci_i2c_callback,
    .p_context        = &g_rzv2h_sci_i2c2_priv,
    .p_extend         = &g_rzv2h_sci_i2c2_priv.extend,
  },
  .refs               = 0,
  .lock               = NXMUTEX_INITIALIZER,
  .sem_isr            = SEM_INITIALIZER(0),
  .frequency          = CONFIG_RZV2H_SCI_I2C2_BITRATE,
  .sda_delay_ns       = CONFIG_RZV2H_SCI_I2C2_SDA_DELAY,
#ifdef CONFIG_RZV2H_SCI_I2C2_BITRATE_MODULATION
  .bitrate_modulation = true,
#else
  .bitrate_modulation = false,
#endif
};
#endif /* CONFIG_RZV2H_SCI_I2C2 */

#ifdef CONFIG_RZV2H_SCI_I2C3
static const struct rzv2h_sci_i2c_pin_config_s
  g_rzv2h_sci_i2c3_pin_config =
{
  .scl_pin            = BOARD_SCI_I2C3_SCL_GPIO,
  .sda_pin            = BOARD_SCI_I2C3_SDA_GPIO,
};

static struct rzv2h_sci_i2c_priv_s g_rzv2h_sci_i2c3_priv =
{
  .ops                = &g_rzv2h_sci_i2c_ops,
  .pin_config         = &g_rzv2h_sci_i2c3_pin_config,
  .extend             =
  {
    .clock_settings   =
    {
      .snfr_value     = CONFIG_RZV2H_SCI_I2C3_NF_CLK,
      .clock_source   = SCI_B_I2C_CLOCK_SOURCE_SCISPICLK,
    },
    .p_reg            = (void *)R_SCI3,
  },
  .cfg                =
  {
    .channel          = 3,
    .rate             = I2C_MASTER_RATE_STANDARD,
    .slave            = 0x00,
    .addr_mode        = I2C_MASTER_ADDR_MODE_7BIT,
    .ipl              = CONFIG_RZV2H_SCI_I2C3_IRQ_PRIORITY,
    .rxi_irq          = RZV2H_IRQ_SCI3_RXI, /* Unused: no DMAC */
    .txi_irq          = RZV2H_IRQ_SCI3_TXI,
    .tei_irq          = RZV2H_IRQ_SCI3_TEI,
    .eri_irq          = RZV2H_IRQ_SCI3_ERI, /* Unused in simple I2C */
    .p_transfer_tx    = NULL,
    .p_transfer_rx    = NULL,
    .p_callback       = rzv2h_sci_i2c_callback,
    .p_context        = &g_rzv2h_sci_i2c3_priv,
    .p_extend         = &g_rzv2h_sci_i2c3_priv.extend,
  },
  .refs               = 0,
  .lock               = NXMUTEX_INITIALIZER,
  .sem_isr            = SEM_INITIALIZER(0),
  .frequency          = CONFIG_RZV2H_SCI_I2C3_BITRATE,
  .sda_delay_ns       = CONFIG_RZV2H_SCI_I2C3_SDA_DELAY,
#ifdef CONFIG_RZV2H_SCI_I2C3_BITRATE_MODULATION
  .bitrate_modulation = true,
#else
  .bitrate_modulation = false,
#endif
};
#endif /* CONFIG_RZV2H_SCI_I2C3 */

#ifdef CONFIG_RZV2H_SCI_I2C4
static const struct rzv2h_sci_i2c_pin_config_s
  g_rzv2h_sci_i2c4_pin_config =
{
  .scl_pin            = BOARD_SCI_I2C4_SCL_GPIO,
  .sda_pin            = BOARD_SCI_I2C4_SDA_GPIO,
};

static struct rzv2h_sci_i2c_priv_s g_rzv2h_sci_i2c4_priv =
{
  .ops                = &g_rzv2h_sci_i2c_ops,
  .pin_config         = &g_rzv2h_sci_i2c4_pin_config,
  .extend             =
  {
    .clock_settings   =
    {
      .snfr_value     = CONFIG_RZV2H_SCI_I2C4_NF_CLK,
      .clock_source   = SCI_B_I2C_CLOCK_SOURCE_SCISPICLK,
    },
    .p_reg            = (void *)R_SCI4,
  },
  .cfg                =
  {
    .channel          = 4,
    .rate             = I2C_MASTER_RATE_STANDARD,
    .slave            = 0x00,
    .addr_mode        = I2C_MASTER_ADDR_MODE_7BIT,
    .ipl              = CONFIG_RZV2H_SCI_I2C4_IRQ_PRIORITY,
    .rxi_irq          = RZV2H_IRQ_SCI4_RXI, /* Unused: no DMAC */
    .txi_irq          = RZV2H_IRQ_SCI4_TXI,
    .tei_irq          = RZV2H_IRQ_SCI4_TEI,
    .eri_irq          = RZV2H_IRQ_SCI4_ERI, /* Unused in simple I2C */
    .p_transfer_tx    = NULL,
    .p_transfer_rx    = NULL,
    .p_callback       = rzv2h_sci_i2c_callback,
    .p_context        = &g_rzv2h_sci_i2c4_priv,
    .p_extend         = &g_rzv2h_sci_i2c4_priv.extend,
  },
  .refs               = 0,
  .lock               = NXMUTEX_INITIALIZER,
  .sem_isr            = SEM_INITIALIZER(0),
  .frequency          = CONFIG_RZV2H_SCI_I2C4_BITRATE,
  .sda_delay_ns       = CONFIG_RZV2H_SCI_I2C4_SDA_DELAY,
#ifdef CONFIG_RZV2H_SCI_I2C4_BITRATE_MODULATION
  .bitrate_modulation = true,
#else
  .bitrate_modulation = false,
#endif
};
#endif /* CONFIG_RZV2H_SCI_I2C4 */

#ifdef CONFIG_RZV2H_SCI_I2C5
static const struct rzv2h_sci_i2c_pin_config_s
  g_rzv2h_sci_i2c5_pin_config =
{
  .scl_pin            = BOARD_SCI_I2C5_SCL_GPIO,
  .sda_pin            = BOARD_SCI_I2C5_SDA_GPIO,
};

static struct rzv2h_sci_i2c_priv_s g_rzv2h_sci_i2c5_priv =
{
  .ops                = &g_rzv2h_sci_i2c_ops,
  .pin_config         = &g_rzv2h_sci_i2c5_pin_config,
  .extend             =
  {
    .clock_settings   =
    {
      .snfr_value     = CONFIG_RZV2H_SCI_I2C5_NF_CLK,
      .clock_source   = SCI_B_I2C_CLOCK_SOURCE_SCISPICLK,
    },
    .p_reg            = (void *)R_SCI5,
  },
  .cfg                =
  {
    .channel          = 5,
    .rate             = I2C_MASTER_RATE_STANDARD,
    .slave            = 0x00,
    .addr_mode        = I2C_MASTER_ADDR_MODE_7BIT,
    .ipl              = CONFIG_RZV2H_SCI_I2C5_IRQ_PRIORITY,
    .rxi_irq          = RZV2H_IRQ_SCI5_RXI, /* Unused: no DMAC */
    .txi_irq          = RZV2H_IRQ_SCI5_TXI,
    .tei_irq          = RZV2H_IRQ_SCI5_TEI,
    .eri_irq          = RZV2H_IRQ_SCI5_ERI, /* Unused in simple I2C */
    .p_transfer_tx    = NULL,
    .p_transfer_rx    = NULL,
    .p_callback       = rzv2h_sci_i2c_callback,
    .p_context        = &g_rzv2h_sci_i2c5_priv,
    .p_extend         = &g_rzv2h_sci_i2c5_priv.extend,
  },
  .refs               = 0,
  .lock               = NXMUTEX_INITIALIZER,
  .sem_isr            = SEM_INITIALIZER(0),
  .frequency          = CONFIG_RZV2H_SCI_I2C5_BITRATE,
  .sda_delay_ns       = CONFIG_RZV2H_SCI_I2C5_SDA_DELAY,
#ifdef CONFIG_RZV2H_SCI_I2C5_BITRATE_MODULATION
  .bitrate_modulation = true,
#else
  .bitrate_modulation = false,
#endif
};
#endif /* CONFIG_RZV2H_SCI_I2C5 */

#ifdef CONFIG_RZV2H_SCI_I2C6
static const struct rzv2h_sci_i2c_pin_config_s
  g_rzv2h_sci_i2c6_pin_config =
{
  .scl_pin            = BOARD_SCI_I2C6_SCL_GPIO,
  .sda_pin            = BOARD_SCI_I2C6_SDA_GPIO,
};

static struct rzv2h_sci_i2c_priv_s g_rzv2h_sci_i2c6_priv =
{
  .ops                = &g_rzv2h_sci_i2c_ops,
  .pin_config         = &g_rzv2h_sci_i2c6_pin_config,
  .extend             =
  {
    .clock_settings   =
    {
      .snfr_value     = CONFIG_RZV2H_SCI_I2C6_NF_CLK,
      .clock_source   = SCI_B_I2C_CLOCK_SOURCE_SCISPICLK,
    },
    .p_reg            = (void *)R_SCI6,
  },
  .cfg                =
  {
    .channel          = 6,
    .rate             = I2C_MASTER_RATE_STANDARD,
    .slave            = 0x00,
    .addr_mode        = I2C_MASTER_ADDR_MODE_7BIT,
    .ipl              = CONFIG_RZV2H_SCI_I2C6_IRQ_PRIORITY,
    .rxi_irq          = RZV2H_IRQ_SCI6_RXI, /* Unused: no DMAC */
    .txi_irq          = RZV2H_IRQ_SCI6_TXI,
    .tei_irq          = RZV2H_IRQ_SCI6_TEI,
    .eri_irq          = RZV2H_IRQ_SCI6_ERI, /* Unused in simple I2C */
    .p_transfer_tx    = NULL,
    .p_transfer_rx    = NULL,
    .p_callback       = rzv2h_sci_i2c_callback,
    .p_context        = &g_rzv2h_sci_i2c6_priv,
    .p_extend         = &g_rzv2h_sci_i2c6_priv.extend,
  },
  .refs               = 0,
  .lock               = NXMUTEX_INITIALIZER,
  .sem_isr            = SEM_INITIALIZER(0),
  .frequency          = CONFIG_RZV2H_SCI_I2C6_BITRATE,
  .sda_delay_ns       = CONFIG_RZV2H_SCI_I2C6_SDA_DELAY,
#ifdef CONFIG_RZV2H_SCI_I2C6_BITRATE_MODULATION
  .bitrate_modulation = true,
#else
  .bitrate_modulation = false,
#endif
};
#endif /* CONFIG_RZV2H_SCI_I2C6 */

#ifdef CONFIG_RZV2H_SCI_I2C7
static const struct rzv2h_sci_i2c_pin_config_s
  g_rzv2h_sci_i2c7_pin_config =
{
  .scl_pin            = BOARD_SCI_I2C7_SCL_GPIO,
  .sda_pin            = BOARD_SCI_I2C7_SDA_GPIO,
};

static struct rzv2h_sci_i2c_priv_s g_rzv2h_sci_i2c7_priv =
{
  .ops                = &g_rzv2h_sci_i2c_ops,
  .pin_config         = &g_rzv2h_sci_i2c7_pin_config,
  .extend             =
  {
    .clock_settings   =
    {
      .snfr_value     = CONFIG_RZV2H_SCI_I2C7_NF_CLK,
      .clock_source   = SCI_B_I2C_CLOCK_SOURCE_SCISPICLK,
    },
    .p_reg            = (void *)R_SCI7,
  },
  .cfg                =
  {
    .channel          = 7,
    .rate             = I2C_MASTER_RATE_STANDARD,
    .slave            = 0x00,
    .addr_mode        = I2C_MASTER_ADDR_MODE_7BIT,
    .ipl              = CONFIG_RZV2H_SCI_I2C7_IRQ_PRIORITY,
    .rxi_irq          = RZV2H_IRQ_SCI7_RXI, /* Unused: no DMAC */
    .txi_irq          = RZV2H_IRQ_SCI7_TXI,
    .tei_irq          = RZV2H_IRQ_SCI7_TEI,
    .eri_irq          = RZV2H_IRQ_SCI7_ERI, /* Unused in simple I2C */
    .p_transfer_tx    = NULL,
    .p_transfer_rx    = NULL,
    .p_callback       = rzv2h_sci_i2c_callback,
    .p_context        = &g_rzv2h_sci_i2c7_priv,
    .p_extend         = &g_rzv2h_sci_i2c7_priv.extend,
  },
  .refs               = 0,
  .lock               = NXMUTEX_INITIALIZER,
  .sem_isr            = SEM_INITIALIZER(0),
  .frequency          = CONFIG_RZV2H_SCI_I2C7_BITRATE,
  .sda_delay_ns       = CONFIG_RZV2H_SCI_I2C7_SDA_DELAY,
#ifdef CONFIG_RZV2H_SCI_I2C7_BITRATE_MODULATION
  .bitrate_modulation = true,
#else
  .bitrate_modulation = false,
#endif
};
#endif /* CONFIG_RZV2H_SCI_I2C7 */

#ifdef CONFIG_RZV2H_SCI_I2C8
static const struct rzv2h_sci_i2c_pin_config_s
  g_rzv2h_sci_i2c8_pin_config =
{
  .scl_pin            = BOARD_SCI_I2C8_SCL_GPIO,
  .sda_pin            = BOARD_SCI_I2C8_SDA_GPIO,
};

static struct rzv2h_sci_i2c_priv_s g_rzv2h_sci_i2c8_priv =
{
  .ops                = &g_rzv2h_sci_i2c_ops,
  .pin_config         = &g_rzv2h_sci_i2c8_pin_config,
  .extend             =
  {
    .clock_settings   =
    {
      .snfr_value     = CONFIG_RZV2H_SCI_I2C8_NF_CLK,
      .clock_source   = SCI_B_I2C_CLOCK_SOURCE_SCISPICLK,
    },
    .p_reg            = (void *)R_SCI8,
  },
  .cfg                =
  {
    .channel          = 8,
    .rate             = I2C_MASTER_RATE_STANDARD,
    .slave            = 0x00,
    .addr_mode        = I2C_MASTER_ADDR_MODE_7BIT,
    .ipl              = CONFIG_RZV2H_SCI_I2C8_IRQ_PRIORITY,
    .rxi_irq          = RZV2H_IRQ_SCI8_RXI, /* Unused: no DMAC */
    .txi_irq          = RZV2H_IRQ_SCI8_TXI,
    .tei_irq          = RZV2H_IRQ_SCI8_TEI,
    .eri_irq          = RZV2H_IRQ_SCI8_ERI, /* Unused in simple I2C */
    .p_transfer_tx    = NULL,
    .p_transfer_rx    = NULL,
    .p_callback       = rzv2h_sci_i2c_callback,
    .p_context        = &g_rzv2h_sci_i2c8_priv,
    .p_extend         = &g_rzv2h_sci_i2c8_priv.extend,
  },
  .refs               = 0,
  .lock               = NXMUTEX_INITIALIZER,
  .sem_isr            = SEM_INITIALIZER(0),
  .frequency          = CONFIG_RZV2H_SCI_I2C8_BITRATE,
  .sda_delay_ns       = CONFIG_RZV2H_SCI_I2C8_SDA_DELAY,
#ifdef CONFIG_RZV2H_SCI_I2C8_BITRATE_MODULATION
  .bitrate_modulation = true,
#else
  .bitrate_modulation = false,
#endif
};
#endif /* CONFIG_RZV2H_SCI_I2C8 */

#ifdef CONFIG_RZV2H_SCI_I2C9
static const struct rzv2h_sci_i2c_pin_config_s
  g_rzv2h_sci_i2c9_pin_config =
{
  .scl_pin            = BOARD_SCI_I2C9_SCL_GPIO,
  .sda_pin            = BOARD_SCI_I2C9_SDA_GPIO,
};

static struct rzv2h_sci_i2c_priv_s g_rzv2h_sci_i2c9_priv =
{
  .ops                = &g_rzv2h_sci_i2c_ops,
  .pin_config         = &g_rzv2h_sci_i2c9_pin_config,
  .extend             =
  {
    .clock_settings   =
    {
      .snfr_value     = CONFIG_RZV2H_SCI_I2C9_NF_CLK,
      .clock_source   = SCI_B_I2C_CLOCK_SOURCE_SCISPICLK,
    },
    .p_reg            = (void *)R_SCI9,
  },
  .cfg                =
  {
    .channel          = 9,
    .rate             = I2C_MASTER_RATE_STANDARD,
    .slave            = 0x00,
    .addr_mode        = I2C_MASTER_ADDR_MODE_7BIT,
    .ipl              = CONFIG_RZV2H_SCI_I2C9_IRQ_PRIORITY,
    .rxi_irq          = RZV2H_IRQ_SCI9_RXI, /* Unused: no DMAC */
    .txi_irq          = RZV2H_IRQ_SCI9_TXI,
    .tei_irq          = RZV2H_IRQ_SCI9_TEI,
    .eri_irq          = RZV2H_IRQ_SCI9_ERI, /* Unused in simple I2C */
    .p_transfer_tx    = NULL,
    .p_transfer_rx    = NULL,
    .p_callback       = rzv2h_sci_i2c_callback,
    .p_context        = &g_rzv2h_sci_i2c9_priv,
    .p_extend         = &g_rzv2h_sci_i2c9_priv.extend,
  },
  .refs               = 0,
  .lock               = NXMUTEX_INITIALIZER,
  .sem_isr            = SEM_INITIALIZER(0),
  .frequency          = CONFIG_RZV2H_SCI_I2C9_BITRATE,
  .sda_delay_ns       = CONFIG_RZV2H_SCI_I2C9_SDA_DELAY,
#ifdef CONFIG_RZV2H_SCI_I2C9_BITRATE_MODULATION
  .bitrate_modulation = true,
#else
  .bitrate_modulation = false,
#endif
};
#endif /* CONFIG_RZV2H_SCI_I2C9 */
/* SCI channel to driver instance lookup */

static struct rzv2h_sci_i2c_priv_s * const
g_rzv2h_sci_i2c_privs[RZV2H_SCI_I2C_NCHANNELS] =
{
#ifdef CONFIG_RZV2H_SCI_I2C0
  [0] = &g_rzv2h_sci_i2c0_priv,
#endif
#ifdef CONFIG_RZV2H_SCI_I2C1
  [1] = &g_rzv2h_sci_i2c1_priv,
#endif
#ifdef CONFIG_RZV2H_SCI_I2C2
  [2] = &g_rzv2h_sci_i2c2_priv,
#endif
#ifdef CONFIG_RZV2H_SCI_I2C3
  [3] = &g_rzv2h_sci_i2c3_priv,
#endif
#ifdef CONFIG_RZV2H_SCI_I2C4
  [4] = &g_rzv2h_sci_i2c4_priv,
#endif
#ifdef CONFIG_RZV2H_SCI_I2C5
  [5] = &g_rzv2h_sci_i2c5_priv,
#endif
#ifdef CONFIG_RZV2H_SCI_I2C6
  [6] = &g_rzv2h_sci_i2c6_priv,
#endif
#ifdef CONFIG_RZV2H_SCI_I2C7
  [7] = &g_rzv2h_sci_i2c7_priv,
#endif
#ifdef CONFIG_RZV2H_SCI_I2C8
  [8] = &g_rzv2h_sci_i2c8_priv,
#endif
#ifdef CONFIG_RZV2H_SCI_I2C9
  [9] = &g_rzv2h_sci_i2c9_priv,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_sci_i2c_interrupt
 *
 * Description:
 *   Common interrupt handler for the SCI simple I2C interrupt sources.
 *
 * Input Parameters:
 *   irq     - GIC interrupt number
 *   context - CPU context at time of interrupt (unused)
 *   arg     - SCI I2C private data structure
 *
 * Returned Value:
 *   OK on success, -EINVAL if IRQ does not match any SCI I2C source
 *
 ****************************************************************************/

static int rzv2h_sci_i2c_interrupt(int irq, void *context, void *arg)
{
  struct rzv2h_sci_i2c_priv_s *priv = (struct rzv2h_sci_i2c_priv_s *)arg;
  rzv2h_irqn_t fsp_irq;
  void (*fsp_isr)(void);

  UNUSED(context);

  fsp_irq = (rzv2h_irqn_t)(irq - BSP_CORTEX_VECTOR_TABLE_ENTRIES);

  if (fsp_irq == (rzv2h_irqn_t)priv->cfg.txi_irq)
    {
      fsp_isr = sci_b_i2c_txi_isr;
    }
  else if (fsp_irq == (rzv2h_irqn_t)priv->cfg.tei_irq)
    {
      fsp_isr = sci_b_i2c_tei_isr;
    }
  else
    {
      return -EINVAL;
    }

  rzv2h_interrupt_common_handler(fsp_irq, fsp_isr);

  return OK;
}

/****************************************************************************
 * Name: rzv2h_sci_i2c_callback
 *
 * Description:
 *   FSP SCI-B I2C callback invoked from ISR context when an I2C transfer
 *   completes (successfully or aborted on NACK).  Records the event and
 *   posts the semaphore to wake the waiting transfer function.
 *
 * Input Parameters:
 *   p_args - FSP callback arguments containing event type and context
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_sci_i2c_callback(i2c_master_callback_args_t *p_args)
{
  struct rzv2h_sci_i2c_priv_s *priv =
    (struct rzv2h_sci_i2c_priv_s *)p_args->p_context;

  priv->event = p_args->event;
  nxsem_post(&priv->sem_isr);
}

/****************************************************************************
 * Name: rzv2h_sci_i2c_calc_clock
 *
 * Description:
 *   Calculate the CCR2.CKS, CCR2.BRR, CCR2.MDDR/BRME and ICR.IICDL
 *   settings for the requested bit rate.  The settings with the largest
 *   bit rate that does not exceed the requested bit rate are selected.
 *
 * Input Parameters:
 *   frequency    - Requested bit rate in Hz
 *   sda_delay_ns - Requested SDA output delay in nanoseconds
 *   modulation   - True to allow bit rate modulation (MDDR)
 *   settings     - FSP clock settings to update.  The noise filter and
 *                  clock source fields are left unchanged.
 *   actual       - Returns the resulting bit rate in Hz
 *
 * Returned Value:
 *   OK on success, -EINVAL if the bit rate cannot be generated
 *
 ****************************************************************************/

static int rzv2h_sci_i2c_calc_clock(uint32_t frequency,
                                      uint32_t sda_delay_ns,
                                      bool modulation,
                                      sci_b_i2c_clock_settings_t *settings,
                                      uint32_t *actual)
{
  const uint32_t tclk = R_FSP_SystemClockHzGet(BSP_FEATURE_SCI_CLOCK);
  uint64_t divisor = 0;
  uint64_t sda_clk;
  uint32_t multiple = 0;
  uint32_t mddr = RZV2H_SCI_I2C_MDDR_DISABLED;
  uint32_t brr = 0;
  uint32_t cks;
  uint32_t rate;
  uint32_t temp_brr;
  uint32_t cycles;

  if (frequency == 0 || tclk == 0)
    {
      return -EINVAL;
    }

  /* Start with the smallest divider because it gives the most resolution.
   * Select the smallest BRR whose bit rate does not exceed the request:
   * BRR = ceil(TCLK / divisor) - 1.
   */

  for (cks = 0; cks <= RZV2H_SCI_I2C_CKS_MAX; cks++)
    {
      multiple = RZV2H_SCI_I2C_BRR_DIV(cks);
      divisor  = (uint64_t)multiple * frequency;
      brr = (uint32_t)((tclk + divisor - 1) / divisor) - 1;
      if (brr <= RZV2H_SCI_I2C_BRR_MAX)
        {
          break;
        }
    }

  if (cks > RZV2H_SCI_I2C_CKS_MAX)
    {
      i2cerr("ERROR: SCI I2C bit rate %" PRIu32 " Hz is too low\n",
             frequency);
      return -EINVAL;
    }

  /* Bit rate modulation: MDDR = floor(divisor * 256 * (BRR + 1) / TCLK)
   * must be between 128 and 256.  Keep the BRR/MDDR pair with the
   * smallest bit rate error.  The resulting bit rate never exceeds the
   * requested bit rate.  For a given CKS the bit rate is proportional to
   * MDDR / (BRR + 1), so candidates are compared exactly by
   * cross-multiplication.
   */

  if (modulation)
    {
      for (temp_brr = brr; temp_brr > 0; temp_brr--)
        {
          uint64_t temp_mddr = (divisor * RZV2H_SCI_I2C_MDDR_DISABLED *
                                (temp_brr + 1)) / tclk;

          if (temp_mddr < RZV2H_SCI_I2C_MDDR_MIN)
            {
              break;
            }

          if (temp_mddr > RZV2H_SCI_I2C_MDDR_DISABLED)
            {
              continue;
            }

          if (temp_mddr * (brr + 1) > (uint64_t)mddr * (temp_brr + 1))
            {
              brr  = temp_brr;
              mddr = (uint32_t)temp_mddr;
            }
        }
    }

  rate = (uint32_t)(((uint64_t)tclk * mddr) /
                    ((uint64_t)multiple * RZV2H_SCI_I2C_MDDR_DISABLED *
                     (brr + 1)));

  /* MDDR == 256 means no modulation; the register only holds 128-255 */

  if (mddr == RZV2H_SCI_I2C_MDDR_DISABLED)
    {
      mddr       = RZV2H_SCI_I2C_MDDR_REG_MAX;
      modulation = false;
    }

  /* SDA output delay in cycles of the baud rate generator clock
   * (TCLK / 4^CKS, Table 7.3-10 "Clock Source Settings"), rounded up.
   */

  sda_clk = tclk >> (2 * cks);
  cycles  = (uint32_t)(((uint64_t)sda_delay_ns * sda_clk +
                        NSEC_PER_SEC - 1) / NSEC_PER_SEC);
  if (cycles < RZV2H_SCI_I2C_IICDL_MIN)
    {
      cycles = RZV2H_SCI_I2C_IICDL_MIN;
    }
  else if (cycles > RZV2H_SCI_I2C_IICDL_MAX)
    {
      cycles = RZV2H_SCI_I2C_IICDL_MAX;
    }

  settings->bitrate_modulation = modulation;
  settings->brr_value          = (uint8_t)brr;
  settings->clk_divisor_value  = (uint8_t)cks;
  settings->mddr_value         = (uint8_t)mddr;
  settings->cycles_value       = (uint8_t)cycles;

  *actual = rate;
  return OK;
}

/****************************************************************************
 * Name: rzv2h_sci_i2c_configure
 *
 * Description:
 *   Update the FSP configuration of a channel for the requested bit rate.
 *   The hardware is not touched; the settings take effect on the next
 *   R_SCI_B_I2C_Open().
 *
 * Input Parameters:
 *   priv      - SCI I2C private data structure for the channel
 *   frequency - Requested bit rate in Hz (at most 400 kHz)
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int rzv2h_sci_i2c_configure(struct rzv2h_sci_i2c_priv_s *priv,
                                     uint32_t frequency)
{
  sci_b_i2c_clock_settings_t settings = priv->extend.clock_settings;
  uint32_t actual;
  int ret;

  DEBUGASSERT(frequency <= RZV2H_SCI_I2C_MAX_FREQUENCY);

  ret = rzv2h_sci_i2c_calc_clock(frequency, priv->sda_delay_ns,
                                   priv->bitrate_modulation, &settings,
                                   &actual);
  if (ret < 0)
    {
      return ret;
    }

  priv->extend.clock_settings = settings;
  priv->cfg.rate = frequency > I2C_SPEED_STANDARD ?
                   I2C_MASTER_RATE_FAST : I2C_MASTER_RATE_STANDARD;
  priv->frequency = frequency;

  i2cinfo("SCI%u I2C: requested %" PRIu32 " Hz, actual %" PRIu32
          " Hz (CKS=%u BRR=%u MDDR=%u BRME=%u IICDL=%u)\n",
          priv->cfg.channel, frequency, actual,
          settings.clk_divisor_value, settings.brr_value,
          settings.mddr_value, settings.bitrate_modulation,
          settings.cycles_value);

  return OK;
}

/****************************************************************************
 * Name: rzv2h_sci_i2c_set_frequency
 *
 * Description:
 *   Change the bit rate of an open channel.  FSP requires a close/reopen
 *   cycle for rate changes, so this must only be called while the bus is
 *   idle (no RESTART pending).
 *
 * Input Parameters:
 *   priv      - SCI I2C private data structure for the channel
 *   frequency - Requested bit rate in Hz (at most 400 kHz)
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int rzv2h_sci_i2c_set_frequency(struct rzv2h_sci_i2c_priv_s *priv,
                                         uint32_t frequency)
{
  uint32_t old_frequency = priv->frequency;
  fsp_err_t err;
  int ret;

  ret = rzv2h_sci_i2c_configure(priv, frequency);
  if (ret < 0)
    {
      return ret;
    }

  R_SCI_B_I2C_Close(&priv->ctrl);

  err = R_SCI_B_I2C_Open(&priv->ctrl, &priv->cfg);
  if (err != FSP_SUCCESS)
    {
      i2cerr("ERROR: SCI%u I2C reopen failed: %d\n",
             priv->cfg.channel, err);

      /* Try to restore the previous bit rate */

      if (rzv2h_sci_i2c_configure(priv, old_frequency) == OK)
        {
          R_SCI_B_I2C_Open(&priv->ctrl, &priv->cfg);
        }

      return rzv2h_fsp_err_to_errno(err);
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_sci_i2c_irq_attach
 *
 * Description:
 *   Attach the TXI and TEI interrupt handlers of the channel.  These are
 *   dedicated GIC SPIs on RZ/V2H, so no INTSEL routing is needed.  The
 *   interrupts are left disabled here; R_SCI_B_I2C_Open() sets their
 *   trigger type (TXI edge, TEI level) and priority and enables them.
 *
 * Input Parameters:
 *   priv - SCI I2C private data structure for the channel
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int rzv2h_sci_i2c_irq_attach(struct rzv2h_sci_i2c_priv_s *priv)
{
  int txi_irq = RZV2H_FSP_TO_GIC_IRQ(priv->cfg.txi_irq);
  int tei_irq = RZV2H_FSP_TO_GIC_IRQ(priv->cfg.tei_irq);
  int ret;

  up_disable_irq(txi_irq);
  up_disable_irq(tei_irq);

  ret = irq_attach(txi_irq, rzv2h_sci_i2c_interrupt, priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = irq_attach(tei_irq, rzv2h_sci_i2c_interrupt, priv);
  if (ret < 0)
    {
      irq_detach(txi_irq);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_sci_i2c_irq_detach
 *
 * Description:
 *   Disable and detach the TXI and TEI interrupt handlers of the channel.
 *
 * Input Parameters:
 *   priv - SCI I2C private data structure for the channel
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_sci_i2c_irq_detach(struct rzv2h_sci_i2c_priv_s *priv)
{
  int txi_irq = RZV2H_FSP_TO_GIC_IRQ(priv->cfg.txi_irq);
  int tei_irq = RZV2H_FSP_TO_GIC_IRQ(priv->cfg.tei_irq);

  up_disable_irq(txi_irq);
  up_disable_irq(tei_irq);

  irq_detach(txi_irq);
  irq_detach(tei_irq);
}

/****************************************************************************
 * Name: rzv2h_sci_i2c_init
 *
 * Description:
 *   Initialize the SCI channel for simple I2C operation.  Calculates the
 *   clock settings, attaches the interrupts, opens the FSP module and
 *   finally routes the pins to the SCI peripheral.
 *
 * Input Parameters:
 *   priv - SCI I2C private data structure for the channel
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int rzv2h_sci_i2c_init(struct rzv2h_sci_i2c_priv_s *priv)
{
  fsp_err_t err;
  int ret;

  ret = rzv2h_sci_i2c_configure(priv, priv->frequency);
  if (ret < 0)
    {
      return ret;
    }

  ret = rzv2h_sci_i2c_irq_attach(priv);
  if (ret < 0)
    {
      return ret;
    }

  err = R_SCI_B_I2C_Open(&priv->ctrl, &priv->cfg);
  if (err != FSP_SUCCESS)
    {
      i2cerr("ERROR: SCI%u I2C open failed: %d\n", priv->cfg.channel, err);
      rzv2h_sci_i2c_irq_detach(priv);
      return rzv2h_fsp_err_to_errno(err);
    }

  /* Configure the pins for the SCI peripheral function */

  rzv2h_configgpio(priv->pin_config->scl_pin);
  rzv2h_configgpio(priv->pin_config->sda_pin);

  return OK;
}

/****************************************************************************
 * Name: rzv2h_sci_i2c_deinit
 *
 * Description:
 *   Shutdown the SCI channel.  Closes the FSP module (aborting any
 *   transfer and stopping the module clock) and detaches the interrupts.
 *
 * Input Parameters:
 *   priv - SCI I2C private data structure for the channel
 *
 * Returned Value:
 *   OK on success, negative errno if FSP close fails
 *
 ****************************************************************************/

static int rzv2h_sci_i2c_deinit(struct rzv2h_sci_i2c_priv_s *priv)
{
  fsp_err_t err;

  err = R_SCI_B_I2C_Close(&priv->ctrl);

  rzv2h_sci_i2c_irq_detach(priv);

  return rzv2h_fsp_err_to_errno(err);
}

/****************************************************************************
 * Name: rzv2h_sci_i2c_can_merge
 *
 * Description:
 *   Check whether message 'next' can be sent in the same FSP transfer as
 *   message 'prev', i.e. without any START/RESTART/STOP condition or
 *   address in between.  This is the case when both messages have the same
 *   direction, address mode and slave address.
 *
 ****************************************************************************/

static bool rzv2h_sci_i2c_can_merge(const struct i2c_msg_s *prev,
                                      const struct i2c_msg_s *next)
{
  bool same_addr = prev->addr == next->addr;
  bool same_dir = (prev->flags & I2C_M_READ) == (next->flags & I2C_M_READ);
  bool same_addr_mode = (prev->flags & I2C_M_TEN) ==
                        (next->flags & I2C_M_TEN);

  return same_addr && same_dir && same_addr_mode;
}

/****************************************************************************
 * Name: rzv2h_sci_i2c_merge_count
 *
 * Description:
 *   Return how many consecutive messages starting at 'msgs' form one bus
 *   segment (one START/RESTART + address), and their total data length.
 *
 *   A message is appended to the segment only when it has I2C_M_NOSTART
 *   set, because every FSP read/write starts with a START/RESTART and the
 *   slave address.  check_msgs() has already rejected I2C_M_NOSTART
 *   messages that cannot be merged.  Every other message starts a new
 *   segment.
 *
 ****************************************************************************/

static int rzv2h_sci_i2c_merge_count(const struct i2c_msg_s *msgs,
                                       int count, uint32_t *length)
{
  uint32_t total = (uint32_t)msgs[0].length;
  int n = 1;

  while (n < count && (msgs[n].flags & I2C_M_NOSTART) != 0)
    {
      total += (uint32_t)msgs[n].length;
      n++;
    }

  *length = total;
  return n;
}

/****************************************************************************
 * Name: rzv2h_sci_i2c_check_msgs
 *
 * Description:
 *   Validate a message list before anything is put on the bus, so that a
 *   transfer can only fail part way through because of a bus error, never
 *   with a RESTART pending.  Also returns the largest merged segment size
 *   so that a bounce buffer can be allocated in advance.
 *
 * Returned Value:
 *   OK on success, -EINVAL if the message list is invalid
 *
 ****************************************************************************/

static int rzv2h_sci_i2c_check_msgs(const struct i2c_msg_s *msgs,
                                      int count, uint32_t *max_merged_len)
{
  uint32_t length;
  int nmsgs;
  int i;

  *max_merged_len = 0;

  for (i = 0; i < count; i++)
    {
      const struct i2c_msg_s *msg = &msgs[i];

      if (msg->length < 0 || (msg->length > 0 && msg->buffer == NULL))
        {
          return -EINVAL;
        }

      if (msg->addr > ((msg->flags & I2C_M_TEN) != 0 ?
                       RZV2H_SCI_I2C_10BIT_ADDR_MAX :
                       RZV2H_SCI_I2C_7BIT_ADDR_MAX))
        {
          i2cerr("ERROR: Invalid slave address 0x%x\n", msg->addr);
          return -EINVAL;
        }

      /* A message with I2C_M_NOSTART continues the previous message, so it
       * must have the same direction, slave address and address mode
       */

      if (i > 0 && (msg->flags & I2C_M_NOSTART) != 0 &&
          !rzv2h_sci_i2c_can_merge(&msgs[i - 1], msg))
        {
          i2cerr("ERROR: I2C_M_NOSTART with direction/address change\n");
          return -EINVAL;
        }
    }

  for (i = 0; i < count; i += nmsgs)
    {
      nmsgs = rzv2h_sci_i2c_merge_count(&msgs[i], count - i, &length);

      /* I2C cannot read zero bytes: the slave drives the first data byte
       * immediately after acknowledging its address.
       */

      if ((msgs[i].flags & I2C_M_READ) != 0 && length == 0)
        {
          i2cerr("ERROR: Zero length read is not supported\n");
          return -EINVAL;
        }

      if (nmsgs > 1 && length > *max_merged_len)
        {
          *max_merged_len = length;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_sci_i2c_segment
 *
 * Description:
 *   Perform one bus segment: START (or RESTART), slave address and the
 *   data of 'nmsgs' merged messages, followed by a RESTART when 'restart'
 *   is true or a STOP otherwise.
 *
 * Input Parameters:
 *   priv    - SCI I2C private data structure for the channel
 *   msgs    - First message of the segment
 *   nmsgs   - Number of merged messages in the segment
 *   length  - Total data length of the segment
 *   restart - True to end the segment with a RESTART instead of a STOP
 *   bounce  - Buffer of at least 'length' bytes used when nmsgs > 1
 *
 * Returned Value:
 *   OK on success, negative errno on failure (-EIO NACK, -ETIMEDOUT no
 *   completion)
 *
 ****************************************************************************/

static int rzv2h_sci_i2c_segment(struct rzv2h_sci_i2c_priv_s *priv,
                                   struct i2c_msg_s *msgs, int nmsgs,
                                   uint32_t length, bool restart,
                                   uint8_t *bounce)
{
  bool read = (msgs[0].flags & I2C_M_READ) != 0;
  i2c_master_addr_mode_t addr_mode;
  uint8_t *buf;
  uint32_t offset;
  fsp_err_t err;
  int ret;
  int j;

  /* Use the message buffer directly for a single message, otherwise
   * stage the merged messages in the bounce buffer.
   */

  if (nmsgs == 1)
    {
      buf = msgs[0].buffer;
    }
  else
    {
      DEBUGASSERT(bounce != NULL);
      buf = bounce;

      if (!read)
        {
          for (j = 0, offset = 0; j < nmsgs; j++)
            {
              if (msgs[j].length > 0)
                {
                  memcpy(&buf[offset], msgs[j].buffer, msgs[j].length);
                  offset += msgs[j].length;
                }
            }
        }
    }

  addr_mode = (msgs[0].flags & I2C_M_TEN) != 0 ?
              I2C_MASTER_ADDR_MODE_10BIT : I2C_MASTER_ADDR_MODE_7BIT;

  err = R_SCI_B_I2C_SlaveAddressSet(&priv->ctrl, msgs[0].addr, addr_mode);
  if (err != FSP_SUCCESS)
    {
      i2cerr("ERROR: Failed to set slave address: %d\n", err);
      return rzv2h_fsp_err_to_errno(err);
    }

  /* Discard any stale completion before starting */

  priv->event = (i2c_master_event_t)0;
  nxsem_reset(&priv->sem_isr, 0);

  if (read)
    {
      err = R_SCI_B_I2C_Read(&priv->ctrl, buf, length, restart);
    }
  else
    {
      err = R_SCI_B_I2C_Write(&priv->ctrl, buf, length, restart);
    }

  if (err != FSP_SUCCESS)
    {
      i2cerr("ERROR: SCI I2C transfer failed: FSP_ERR=%d\n", err);
      return rzv2h_fsp_err_to_errno(err);
    }

  /* Wait for the callback */

  ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                       RZV2H_SCI_I2C_TIMEOTICKS);
  if (ret < 0)
    {
      i2cerr("ERROR: SCI%u I2C %s timeout (addr 0x%x)\n",
             priv->cfg.channel, read ? "read" : "write", msgs[0].addr);

      /* Force the peripheral back to the idle state and release the
       * bus lines.
       */

      R_SCI_B_I2C_Abort(&priv->ctrl);
      return -ETIMEDOUT;
    }

  switch (priv->event)
    {
      case I2C_MASTER_EVENT_RX_COMPLETE:
      case I2C_MASTER_EVENT_TX_COMPLETE:
        break;

      case I2C_MASTER_EVENT_ABORTED:
      default:

        /* FSP SCI-B I2C only aborts a transfer when a NACK is received */

        i2cinfo("SCI%u I2C %s to 0x%x NACKed\n", priv->cfg.channel,
                read ? "read" : "write", msgs[0].addr);
        return -EIO;
    }

  /* Scatter merged read data back to the messages */

  if (read && nmsgs > 1)
    {
      for (j = 0, offset = 0; j < nmsgs; j++)
        {
          if (msgs[j].length > 0)
            {
              memcpy(msgs[j].buffer, &buf[offset], msgs[j].length);
              offset += msgs[j].length;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_sci_i2c_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to recover a stuck bus.  This
 *   function bit-bangs SCL to clock out any stuck slave, then generates a
 *   START+STOP sequence to reset slave state machines.  The SCI channel
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
static int rzv2h_sci_i2c_reset(struct i2c_master_s *dev)
{
  struct rzv2h_sci_i2c_priv_s *priv;
  gpio_pinset_t scl_gpio;
  gpio_pinset_t sda_gpio;
  unsigned int clock_count;
  unsigned int stretch_count;
  int ret;

  DEBUGASSERT(dev);

  /* Get I2C private structure */

  priv = (struct rzv2h_sci_i2c_priv_s *)dev;

  /* Our caller must own a ref */

  DEBUGASSERT(priv->refs > 0);

  /* Lock out other clients */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EIO;

  /* De-init the channel (close FSP module, detach IRQs) */

  rzv2h_sci_i2c_deinit(priv);

  /* Use GPIO configuration to recover the bus.  Build open-drain GPIO
   * output pinsets from the I2C pins, with input enabled to allow reading
   * the SDA/SCL state.
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

  /* Re-init the channel; this also restores the peripheral pin function */

  if (rzv2h_sci_i2c_init(priv) < 0)
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
 * Name: rzv2h_sci_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers.  A message with I2C_M_NOSTART is
 *   merged with the previous message into one FSP transfer; all other
 *   message boundaries are joined with a repeated START, and a STOP is
 *   generated after the last message.
 *
 *   The bit rate is taken from the first message and applied (FSP
 *   close/reopen) before the first START if it differs from the current
 *   one.  SCI simple I2C supports up to 400 kHz; higher requests are
 *   clamped to 400 kHz.
 *
 * Input Parameters:
 *   dev   - I2C master device instance
 *   msgs  - Array of I2C messages to transfer
 *   count - Number of messages in the array
 *
 * Returned Value:
 *   OK on success, negative errno on failure (-EINVAL, -ENOMEM, -EIO,
 *   -ETIMEDOUT)
 *
 ****************************************************************************/

static int rzv2h_sci_i2c_transfer(struct i2c_master_s *dev,
                                    struct i2c_msg_s *msgs, int count)
{
  struct rzv2h_sci_i2c_priv_s *priv = (struct rzv2h_sci_i2c_priv_s *)dev;
  uint8_t stackbuf[CONFIG_RZV2H_SCI_I2C_MERGE_BUFSIZE];
  uint8_t *bounce = stackbuf;
  uint32_t max_merged_len;
  uint32_t frequency;
  uint32_t length;
  int nmsgs;
  int ret;
  int i;

  DEBUGASSERT(dev != NULL && msgs != NULL && count > 0);

  /* Reject invalid message lists before touching the bus */

  ret = rzv2h_sci_i2c_check_msgs(msgs, count, &max_merged_len);
  if (ret < 0)
    {
      return ret;
    }

  if (max_merged_len > sizeof(stackbuf))
    {
      bounce = kmm_malloc(max_merged_len);
      if (bounce == NULL)
        {
          return -ENOMEM;
        }
    }

  /* Ensure that address or flags don't change meanwhile */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      goto errout_free;
    }

  /* The bit rate can only be changed while the bus is idle, so the
   * frequency of the first message applies to the whole transfer.
   * A frequency of zero keeps the current bit rate.
   */

  frequency = msgs[0].frequency;
  if (frequency == 0)
    {
      frequency = priv->frequency;
    }
  else if (frequency > RZV2H_SCI_I2C_MAX_FREQUENCY)
    {
      i2cwarn("WARNING: SCI%u I2C %" PRIu32 " Hz not supported, "
              "using %d Hz\n", priv->cfg.channel, frequency,
              RZV2H_SCI_I2C_MAX_FREQUENCY);
      frequency = RZV2H_SCI_I2C_MAX_FREQUENCY;
    }

  if (frequency != priv->frequency)
    {
      ret = rzv2h_sci_i2c_set_frequency(priv, frequency);
      if (ret < 0)
        {
          goto errout_unlock;
        }
    }

  for (i = 0; i < count; i += nmsgs)
    {
      bool restart;

      nmsgs = rzv2h_sci_i2c_merge_count(&msgs[i], count - i, &length);

      /* End this segment with a RESTART if more segments follow, so the
       * whole message list is one bus transaction.  Only the last segment
       * ends with a STOP.
       */

      restart = (i + nmsgs) < count;

      ret = rzv2h_sci_i2c_segment(priv, &msgs[i], nmsgs, length, restart,
                                    bounce);
      if (ret < 0)
        {
          break;
        }
    }

errout_unlock:
  nxmutex_unlock(&priv->lock);

errout_free:
  if (bounce != stackbuf)
    {
      kmm_free(bounce);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_sci_i2cbus_initialize
 *
 * Description:
 *   Initialize one SCI channel in simple I2C master mode.  On the first
 *   call for a given channel, attaches the interrupts, opens the FSP
 *   r_sci_b_i2c module and then configures the pins.
 *
 * Input Parameters:
 *   channel - SCI channel number (0-9)
 *
 * Returned Value:
 *   I2C master device instance on success, NULL on failure
 *
 ****************************************************************************/

struct i2c_master_s *rzv2h_sci_i2cbus_initialize(int channel)
{
  struct rzv2h_sci_i2c_priv_s *priv;

  if (channel < 0 || channel >= RZV2H_SCI_I2C_NCHANNELS)
    {
      return NULL;
    }

  /* Get I2C private structure */

  priv = g_rzv2h_sci_i2c_privs[channel];
  if (priv == NULL)
    {
      i2cerr("ERROR: SCI%d is not configured for I2C\n", channel);
      return NULL;
    }

  /* Initialize the hardware for the first user, increment reference
   * count otherwise.
   */

  nxmutex_lock(&priv->lock);
  if (priv->refs++ == 0)
    {
      if (rzv2h_sci_i2c_init(priv) < 0)
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
 * Name: rzv2h_sci_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an SCI simple I2C bus.  Decrements the reference count
 *   and shuts down the hardware when the last reference is released.
 *
 * Input Parameters:
 *   dev - I2C master device instance
 *
 * Returned Value:
 *   OK on success, ERROR if reference count underflow
 *
 ****************************************************************************/

int rzv2h_sci_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct rzv2h_sci_i2c_priv_s *priv = (struct rzv2h_sci_i2c_priv_s *)dev;

  DEBUGASSERT(dev);

  nxmutex_lock(&priv->lock);

  /* Check for reference count underflow */

  if (priv->refs == 0)
    {
      nxmutex_unlock(&priv->lock);
      return ERROR;
    }

  if (--priv->refs > 0)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* Shut down the hardware */

  rzv2h_sci_i2c_deinit(priv);
  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif /* CONFIG_RZV2H_SCI_I2C0 || CONFIG_RZV2H_SCI_I2C1 || ... */

/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_spi.c
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

#if defined(CONFIG_RZV2H_SPI_CHANNEL_0_MASTER) || \
    defined(CONFIG_RZV2H_SPI_CHANNEL_1_MASTER) || \
    defined(CONFIG_RZV2H_SPI_CHANNEL_2_MASTER)
#  define HAVE_RZV2H_SPI_MASTER 1
#endif

#if defined(CONFIG_SPI_SLAVE) && \
   (defined(CONFIG_RZV2H_SPI_CHANNEL_0_SLAVE) || \
    defined(CONFIG_RZV2H_SPI_CHANNEL_1_SLAVE) || \
    defined(CONFIG_RZV2H_SPI_CHANNEL_2_SLAVE))
#  define HAVE_RZV2H_SPI_SLAVE 1
#endif

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/spi/spi.h>
#ifdef HAVE_RZV2H_SPI_SLAVE
#  include <nuttx/spi/slave.h>
#endif

#include <arch/board/board.h>

#include "chip.h"
#include "rzv2h_fsp_err.h"
#include "rzv2h_irq.h"
#include "rzv2h_spi.h"

/* FSP headers are included last. */

#include "r_spi_b.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RZV2H_SPI_FSP_PHASE(m)          \
  (((m) & 1) != 0 ? SPI_CLK_PHASE_EDGE_EVEN : SPI_CLK_PHASE_EDGE_ODD)
#define RZV2H_SPI_FSP_POLARITY(m)       \
  (((m) & 2) != 0 ? SPI_CLK_POLARITY_HIGH : SPI_CLK_POLARITY_LOW)

#ifdef CONFIG_RZV2H_SPI_CHANNEL_0_SLAVE
#  define RZV2H_SPI_CHANNEL_0_MODE  SPI_MODE_SLAVE
#else
#  define RZV2H_SPI_CHANNEL_0_MODE  SPI_MODE_MASTER
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_0_4WIRE
#  define RZV2H_SPI_CHANNEL_0_WIRE_MODE  SPI_B_SSL_MODE_SPI
#else
#  define RZV2H_SPI_CHANNEL_0_WIRE_MODE  SPI_B_SSL_MODE_CLK_SYN
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_1_SLAVE
#  define RZV2H_SPI_CHANNEL_1_MODE  SPI_MODE_SLAVE
#else
#  define RZV2H_SPI_CHANNEL_1_MODE  SPI_MODE_MASTER
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_1_4WIRE
#  define RZV2H_SPI_CHANNEL_1_WIRE_MODE  SPI_B_SSL_MODE_SPI
#else
#  define RZV2H_SPI_CHANNEL_1_WIRE_MODE  SPI_B_SSL_MODE_CLK_SYN
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_2_SLAVE
#  define RZV2H_SPI_CHANNEL_2_MODE  SPI_MODE_SLAVE
#else
#  define RZV2H_SPI_CHANNEL_2_MODE  SPI_MODE_MASTER
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_2_4WIRE
#  define RZV2H_SPI_CHANNEL_2_WIRE_MODE  SPI_B_SSL_MODE_SPI
#else
#  define RZV2H_SPI_CHANNEL_2_WIRE_MODE  SPI_B_SSL_MODE_CLK_SYN
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rzv2h_spi_priv_s;

#ifdef HAVE_RZV2H_SPI_SLAVE
struct spi_dev_slave_s
{
  struct spi_slave_ctrlr_s ctrlr; /* NuttX slave controller interface */
  struct rzv2h_spi_priv_s *priv;  /* Parent SPI_B instance */
  struct spi_slave_dev_s *dev;    /* Bound slave device */
  spinlock_t lock;                /* Slave ISR/thread state lock */
  uint32_t txbuffer[CONFIG_RZV2H_SPI_SLAVE_BUFFER_SIZE];
  uint32_t rxbuffer[CONFIG_RZV2H_SPI_SLAVE_BUFFER_SIZE];
  size_t nwords;                  /* Active transfer length */
  size_t rxoffset;                /* Words already accepted by device */
  bool opened;                    /* FSP instance is open */
};
#endif

struct rzv2h_spi_priv_s
{
#ifdef HAVE_RZV2H_SPI_MASTER
  struct spi_dev_s master;        /* Externally visible master interface */
#endif
#ifdef HAVE_RZV2H_SPI_SLAVE
  struct spi_dev_slave_s slave; /* Externally visible slave interface */
#endif
  const spi_instance_t *instance; /* FSP SPI instance */
  spi_cfg_t *cfg;                 /* Runtime FSP common configuration */
  spi_b_extended_cfg_t *extcfg;   /* Runtime FSP extended configuration */
  rzv2h_irqsel_t rxi_irq_event;   /* IRQ event selector */
  rzv2h_irqsel_t txi_irq_event;   /* IRQ event selector */
  mutex_t lock;                   /* NuttX bus and initialization lock */
  sem_t waitsem;                  /* Synchronous transfer completion */
  uint32_t frequency;             /* Last requested frequency */
  uint32_t actual;                /* Actual configured frequency */
  enum spi_mode_e mode;           /* Current CPOL/CPHA mode */
  uint8_t nbits;                  /* Current bits per word */
  volatile spi_event_t event;     /* Event reported by the FSP callback */
  volatile bool transferring;     /* A transfer is in progress */
  bool initialized;               /* Has SPI interface been initialized */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef HAVE_RZV2H_SPI_MASTER
static int rzv2h_spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t rzv2h_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency);
#ifdef CONFIG_SPI_DELAY_CONTROL
static int rzv2h_spi_setdelay(struct spi_dev_s *dev, uint32_t startdelay,
                              uint32_t stopdelay, uint32_t csdelay,
                              uint32_t ifdelay);
#endif
static void rzv2h_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void rzv2h_spi_setbits(struct spi_dev_s *dev, int nbits);
static uint32_t rzv2h_spi_send(struct spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SPI_EXCHANGE
static int rzv2h_spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                              void *rxbuffer, size_t nwords);
#else
static void rzv2h_spi_sndblock(struct spi_dev_s *dev,
                               const void *txbuffer, size_t nwords);
static void rzv2h_spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                                size_t nwords);
#endif
static int rzv2h_spi_transfer(struct rzv2h_spi_priv_s *priv,
                              const void *txbuffer, void *rxbuffer,
                              size_t nwords);
static int rzv2h_spi_calculate_bitrate(struct rzv2h_spi_priv_s *priv,
                                       uint32_t frequency,
                                       rspck_div_setting_t *setting,
                                       uint32_t *actual);
#endif
static void rzv2h_spi_callback(spi_callback_args_t *args);

#ifdef HAVE_RZV2H_SPI_SLAVE
static void rzv2h_spi_slave_bind(struct spi_slave_ctrlr_s *ctrlr,
                                 struct spi_slave_dev_s *dev,
                                 enum spi_slave_mode_e mode, int nbits);
static void rzv2h_spi_slave_unbind(struct spi_slave_ctrlr_s *ctrlr);
static int rzv2h_spi_slave_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                                   const void *data, size_t nwords);
static bool rzv2h_spi_slave_qfull(struct spi_slave_ctrlr_s *ctrlr);
static void rzv2h_spi_slave_qflush(struct spi_slave_ctrlr_s *ctrlr);
static size_t rzv2h_spi_slave_qpoll(struct spi_slave_ctrlr_s *ctrlr);
static int rzv2h_spi_slave_start(struct spi_dev_slave_s *slave,
                                 const void *data, size_t nwords);
static void rzv2h_spi_slave_callback(struct rzv2h_spi_priv_s *priv,
                                     spi_event_t event);
static size_t rzv2h_spi_slave_wordsize(struct rzv2h_spi_priv_s *priv);
#endif

static int rzv2h_spi_interrupt(int irq, void *context, void *arg);
static int rzv2h_spi_irq_attach(struct rzv2h_spi_priv_s *priv);
static int rzv2h_spi_irq_configure(struct rzv2h_spi_priv_s *priv);
static void rzv2h_spi_irq_detach(struct rzv2h_spi_priv_s *priv);
static int rzv2h_spi_open(struct rzv2h_spi_priv_s *priv);
static struct rzv2h_spi_priv_s *rzv2h_spi_get_channel(int channel,
                                                       spi_mode_t role);
static int rzv2h_spi_initialize(struct rzv2h_spi_priv_s *priv);

/* The FSP implementation exports its IRQ entry points from r_spi_b.c. */

void spi_b_rxi_isr(void);
void spi_b_txi_isr(void);
void spi_b_tei_isr(void);
void spi_b_eri_isr(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_RZV2H_SPI_SLAVE
static const struct spi_slave_ctrlrops_s g_spi_slave_ops =
{
  .bind    = rzv2h_spi_slave_bind,
  .unbind  = rzv2h_spi_slave_unbind,
  .enqueue = rzv2h_spi_slave_enqueue,
  .qfull   = rzv2h_spi_slave_qfull,
  .qflush  = rzv2h_spi_slave_qflush,
  .qpoll   = rzv2h_spi_slave_qpoll,
};
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_0

static const spi_instance_t g_spi0_instance;
static spi_cfg_t g_spi0_cfg;
static spi_b_extended_cfg_t g_spi0_ext_cfg;

#ifdef CONFIG_RZV2H_SPI_CHANNEL_0_MASTER
static const struct spi_ops_s g_spi0_ops =
{
  .lock             = rzv2h_spi_lock,
  .select           = rzv2h_spiselect,
  .setfrequency     = rzv2h_spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay         = rzv2h_spi_setdelay,
#endif
  .setmode          = rzv2h_spi_setmode,
  .setbits          = rzv2h_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures       = NULL,
#endif
  .status           = rzv2h_spi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata          = rzv2h_spi0cmddata,
#endif
  .send             = rzv2h_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = rzv2h_spi_exchange,
#else
  .sndblock         = rzv2h_spi_sndblock,
  .recvblock        = rzv2h_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger          = NULL,
#endif
  .registercallback = NULL,
};
#endif

static struct rzv2h_spi_priv_s g_spi0_priv =
{
#ifdef CONFIG_RZV2H_SPI_CHANNEL_0_MASTER
  .master =
  {
    .ops = &g_spi0_ops,
  },
#endif
#ifdef CONFIG_RZV2H_SPI_CHANNEL_0_SLAVE
  .slave =
  {
    .ctrlr =
    {
      .ops = &g_spi_slave_ops,
    },
    .priv = &g_spi0_priv,
    .lock = SP_UNLOCKED,
  },
#endif
  .instance         = &g_spi0_instance,
  .cfg              = &g_spi0_cfg,
  .extcfg           = &g_spi0_ext_cfg,
  .rxi_irq_event    = RZV2H_IRQSEL_SPI0_RXI,
  .txi_irq_event    = RZV2H_IRQSEL_SPI0_TXI,
  .lock             = NXMUTEX_INITIALIZER,
  .waitsem          = SEM_INITIALIZER(0),
  .frequency        = CONFIG_RZV2H_SPI_CHANNEL_0_FREQUENCY,
  .mode             = CONFIG_RZV2H_SPI_CHANNEL_0_INITIAL_MODE,
  .nbits            = CONFIG_RZV2H_SPI_CHANNEL_0_NBITS,
};

static spi_b_instance_ctrl_t g_spi0_ctrl;

static spi_b_extended_cfg_t g_spi0_ext_cfg =
{
  .spi_clksyn       = RZV2H_SPI_CHANNEL_0_WIRE_MODE,
  .spi_comm         = SPI_B_COMMUNICATION_FULL_DUPLEX,
  .ssl_polarity     = SPI_B_SSLP_LOW,
  .ssl_select       = (spi_b_ssl_select_t)BOARD_SPI0_SSL_SELECT,
  .mosi_idle        = SPI_B_MOSI_IDLE_VALUE_FIXING_DISABLE,
  .parity           = SPI_B_PARITY_MODE_DISABLE,
  .byte_swap        = SPI_B_BYTE_SWAP_DISABLE,
  .clock_source     = SPI_B_CLOCK_SOURCE_PCLK,
  .spck_delay       = SPI_B_DELAY_COUNT_1,
  .ssl_negation_delay = SPI_B_DELAY_COUNT_1,
  .next_access_delay = SPI_B_DELAY_COUNT_1,
  .ssl_level_keep   = SPI_B_SSL_LEVEL_KEEP_DISABLE,
  .spck_div =
  {
    .spbr = 6,  /* Actual calculated bitrate: 14285714. */
    .brdv = 0 },
  .transmit_fifo_threshold = 0,
  .receive_fifo_threshold = 0,
  .receive_data_ready_detect_adjustment = 0,
  .p_reg = (void *)R_SPI_B0,
};

static spi_cfg_t g_spi0_cfg =
{
  .channel        = 0,
  .rxi_irq        = CONFIG_RZV2H_SPI0_RXI_INTSEL,
  .txi_irq        = CONFIG_RZV2H_SPI0_TXI_INTSEL,
  .tei_irq        = RZV2H_IRQ_SPI0_CE,
  .eri_irq        = RZV2H_IRQ_SPI0_ERI,
  .rxi_ipl        = CONFIG_RZV2H_SPI_B_IRQ_PRIORITY,
  .txi_ipl        = CONFIG_RZV2H_SPI_B_IRQ_PRIORITY,
  .tei_ipl        = CONFIG_RZV2H_SPI_B_IRQ_PRIORITY,
  .eri_ipl        = CONFIG_RZV2H_SPI_B_IRQ_PRIORITY,
  .operating_mode = RZV2H_SPI_CHANNEL_0_MODE,
  .clk_phase      = RZV2H_SPI_FSP_PHASE(
                      CONFIG_RZV2H_SPI_CHANNEL_0_INITIAL_MODE),
  .clk_polarity   = RZV2H_SPI_FSP_POLARITY(
                      CONFIG_RZV2H_SPI_CHANNEL_0_INITIAL_MODE),
  .mode_fault     = SPI_MODE_FAULT_ERROR_DISABLE,
  .bit_order      = SPI_BIT_ORDER_MSB_FIRST,
  .p_transfer_tx  = NULL,
  .p_transfer_rx  = NULL,
  .p_callback     = rzv2h_spi_callback,
  .p_context      = &g_spi0_priv,
  .p_extend       = &g_spi0_ext_cfg,
};

static const spi_instance_t g_spi0_instance =
{
  .p_ctrl = &g_spi0_ctrl,
  .p_cfg  = &g_spi0_cfg,
  .p_api  = &g_spi_on_spi_b,
};
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_1

static const spi_instance_t g_spi1_instance;
static spi_cfg_t g_spi1_cfg;
static spi_b_extended_cfg_t g_spi1_ext_cfg;

#ifdef CONFIG_RZV2H_SPI_CHANNEL_1_MASTER
static const struct spi_ops_s g_spi1_ops =
{
  .lock             = rzv2h_spi_lock,
  .select           = rzv2h_spiselect,
  .setfrequency     = rzv2h_spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay         = rzv2h_spi_setdelay,
#endif
  .setmode          = rzv2h_spi_setmode,
  .setbits          = rzv2h_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures       = NULL,
#endif
  .status           = rzv2h_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata          = rzv2h_spi1cmddata,
#endif
  .send             = rzv2h_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = rzv2h_spi_exchange,
#else
  .sndblock         = rzv2h_spi_sndblock,
  .recvblock        = rzv2h_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger          = NULL,
#endif
  .registercallback = NULL,
};
#endif

static struct rzv2h_spi_priv_s g_spi1_priv =
{
#ifdef CONFIG_RZV2H_SPI_CHANNEL_1_MASTER
  .master =
  {
    .ops = &g_spi1_ops,
  },
#endif
#ifdef CONFIG_RZV2H_SPI_CHANNEL_1_SLAVE
  .slave =
  {
    .ctrlr =
    {
      .ops = &g_spi_slave_ops,
    },
    .priv = &g_spi1_priv,
    .lock = SP_UNLOCKED,
  },
#endif
  .instance  = &g_spi1_instance,
  .cfg       = &g_spi1_cfg,
  .extcfg    = &g_spi1_ext_cfg,
  .rxi_irq_event    = RZV2H_IRQSEL_SPI1_RXI,
  .txi_irq_event    = RZV2H_IRQSEL_SPI1_TXI,
  .lock      = NXMUTEX_INITIALIZER,
  .waitsem   = SEM_INITIALIZER(0),
  .frequency = CONFIG_RZV2H_SPI_CHANNEL_1_FREQUENCY,
  .mode      = CONFIG_RZV2H_SPI_CHANNEL_1_INITIAL_MODE,
  .nbits     = CONFIG_RZV2H_SPI_CHANNEL_1_NBITS,
};

static spi_b_instance_ctrl_t g_spi1_ctrl;

static spi_b_extended_cfg_t g_spi1_ext_cfg =
{
  .spi_clksyn       = RZV2H_SPI_CHANNEL_1_WIRE_MODE,
  .spi_comm         = SPI_B_COMMUNICATION_FULL_DUPLEX,
  .ssl_polarity     = SPI_B_SSLP_LOW,
  .ssl_select       = (spi_b_ssl_select_t)BOARD_SPI1_SSL_SELECT,
  .mosi_idle        = SPI_B_MOSI_IDLE_VALUE_FIXING_DISABLE,
  .parity           = SPI_B_PARITY_MODE_DISABLE,
  .byte_swap        = SPI_B_BYTE_SWAP_DISABLE,
  .clock_source     = SPI_B_CLOCK_SOURCE_PCLK,
  .spck_delay       = SPI_B_DELAY_COUNT_1,
  .ssl_negation_delay = SPI_B_DELAY_COUNT_1,
  .next_access_delay = SPI_B_DELAY_COUNT_1,
  .ssl_level_keep   = SPI_B_SSL_LEVEL_KEEP_DISABLE,
  .spck_div =
  {
    .spbr = 6,  /* Actual calculated bitrate: 14285714. */
    .brdv = 0 },
  .transmit_fifo_threshold = 0,
  .receive_fifo_threshold = 0,
  .receive_data_ready_detect_adjustment = 0,
  .p_reg = (void *)R_SPI_B1,
};

static spi_cfg_t g_spi1_cfg =
{
  .channel        = 1,
  .rxi_irq        = CONFIG_RZV2H_SPI1_RXI_INTSEL,
  .txi_irq        = CONFIG_RZV2H_SPI1_TXI_INTSEL,
  .tei_irq        = RZV2H_IRQ_SPI1_CE,
  .eri_irq        = RZV2H_IRQ_SPI1_ERI,
  .rxi_ipl        = CONFIG_RZV2H_SPI_B_IRQ_PRIORITY,
  .txi_ipl        = CONFIG_RZV2H_SPI_B_IRQ_PRIORITY,
  .tei_ipl        = CONFIG_RZV2H_SPI_B_IRQ_PRIORITY,
  .eri_ipl        = CONFIG_RZV2H_SPI_B_IRQ_PRIORITY,
  .operating_mode = RZV2H_SPI_CHANNEL_1_MODE,
  .clk_phase      = RZV2H_SPI_FSP_PHASE(
                      CONFIG_RZV2H_SPI_CHANNEL_1_INITIAL_MODE),
  .clk_polarity   = RZV2H_SPI_FSP_POLARITY(
                      CONFIG_RZV2H_SPI_CHANNEL_1_INITIAL_MODE),
  .mode_fault     = SPI_MODE_FAULT_ERROR_DISABLE,
  .bit_order      = SPI_BIT_ORDER_MSB_FIRST,
  .p_transfer_tx  = NULL,
  .p_transfer_rx  = NULL,
  .p_callback     = rzv2h_spi_callback,
  .p_context      = &g_spi1_priv,
  .p_extend       = &g_spi1_ext_cfg,
};

static const spi_instance_t g_spi1_instance =
{
  .p_ctrl = &g_spi1_ctrl,
  .p_cfg  = &g_spi1_cfg,
  .p_api  = &g_spi_on_spi_b,
};
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_2

static const spi_instance_t g_spi2_instance;
static spi_cfg_t g_spi2_cfg;
static spi_b_extended_cfg_t g_spi2_ext_cfg;

#ifdef CONFIG_RZV2H_SPI_CHANNEL_2_MASTER
static const struct spi_ops_s g_spi2_ops =
{
  .lock             = rzv2h_spi_lock,
  .select           = rzv2h_spiselect,
  .setfrequency     = rzv2h_spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay         = rzv2h_spi_setdelay,
#endif
  .setmode          = rzv2h_spi_setmode,
  .setbits          = rzv2h_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures       = NULL,
#endif
  .status           = rzv2h_spi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata          = rzv2h_spi2cmddata,
#endif
  .send             = rzv2h_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = rzv2h_spi_exchange,
#else
  .sndblock         = rzv2h_spi_sndblock,
  .recvblock        = rzv2h_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger          = NULL,
#endif
  .registercallback = NULL,
};
#endif

static struct rzv2h_spi_priv_s g_spi2_priv =
{
#ifdef CONFIG_RZV2H_SPI_CHANNEL_2_MASTER
  .master =
  {
    .ops = &g_spi2_ops,
  },
#endif
#ifdef CONFIG_RZV2H_SPI_CHANNEL_2_SLAVE
  .slave =
  {
    .ctrlr =
    {
      .ops = &g_spi_slave_ops,
    },
    .priv = &g_spi2_priv,
    .lock = SP_UNLOCKED,
  },
#endif
  .instance  = &g_spi2_instance,
  .cfg       = &g_spi2_cfg,
  .extcfg    = &g_spi2_ext_cfg,
  .rxi_irq_event    = RZV2H_IRQSEL_SPI2_RXI,
  .txi_irq_event    = RZV2H_IRQSEL_SPI2_TXI,
  .lock      = NXMUTEX_INITIALIZER,
  .waitsem   = SEM_INITIALIZER(0),
  .frequency = CONFIG_RZV2H_SPI_CHANNEL_2_FREQUENCY,
  .mode      = CONFIG_RZV2H_SPI_CHANNEL_2_INITIAL_MODE,
  .nbits     = CONFIG_RZV2H_SPI_CHANNEL_2_NBITS,
};

static spi_b_instance_ctrl_t g_spi2_ctrl;

static spi_b_extended_cfg_t g_spi2_ext_cfg =
{
  .spi_clksyn       = RZV2H_SPI_CHANNEL_2_WIRE_MODE,
  .spi_comm         = SPI_B_COMMUNICATION_FULL_DUPLEX,
  .ssl_polarity     = SPI_B_SSLP_LOW,
  .ssl_select       = (spi_b_ssl_select_t)BOARD_SPI2_SSL_SELECT,
  .mosi_idle        = SPI_B_MOSI_IDLE_VALUE_FIXING_DISABLE,
  .parity           = SPI_B_PARITY_MODE_DISABLE,
  .byte_swap        = SPI_B_BYTE_SWAP_DISABLE,
  .clock_source     = SPI_B_CLOCK_SOURCE_PCLK,
  .spck_delay       = SPI_B_DELAY_COUNT_1,
  .ssl_negation_delay = SPI_B_DELAY_COUNT_1,
  .next_access_delay = SPI_B_DELAY_COUNT_1,
  .ssl_level_keep   = SPI_B_SSL_LEVEL_KEEP_DISABLE,
  .spck_div =
  {
    .spbr = 6,  /* Actual calculated bitrate: 14285714. */
    .brdv = 0 },
  .transmit_fifo_threshold = 0,
  .receive_fifo_threshold = 0,
  .receive_data_ready_detect_adjustment = 0,
  .p_reg = (void *)R_SPI_B2,
};

static spi_cfg_t g_spi2_cfg =
{
  .channel        = 2,
  .rxi_irq        = CONFIG_RZV2H_SPI2_RXI_INTSEL,
  .txi_irq        = CONFIG_RZV2H_SPI2_TXI_INTSEL,
  .tei_irq        = RZV2H_IRQ_SPI2_CE,
  .eri_irq        = RZV2H_IRQ_SPI2_ERI,
  .rxi_ipl        = CONFIG_RZV2H_SPI_B_IRQ_PRIORITY,
  .txi_ipl        = CONFIG_RZV2H_SPI_B_IRQ_PRIORITY,
  .tei_ipl        = CONFIG_RZV2H_SPI_B_IRQ_PRIORITY,
  .eri_ipl        = CONFIG_RZV2H_SPI_B_IRQ_PRIORITY,
  .operating_mode = RZV2H_SPI_CHANNEL_2_MODE,
  .clk_phase      = RZV2H_SPI_FSP_PHASE(
                      CONFIG_RZV2H_SPI_CHANNEL_2_INITIAL_MODE),
  .clk_polarity   = RZV2H_SPI_FSP_POLARITY(
                      CONFIG_RZV2H_SPI_CHANNEL_2_INITIAL_MODE),
  .mode_fault     = SPI_MODE_FAULT_ERROR_DISABLE,
  .bit_order      = SPI_BIT_ORDER_MSB_FIRST,
  .p_transfer_tx  = NULL,
  .p_transfer_rx  = NULL,
  .p_callback     = rzv2h_spi_callback,
  .p_context      = &g_spi2_priv,
  .p_extend       = &g_spi2_ext_cfg,
};

static const spi_instance_t g_spi2_instance =
{
  .p_ctrl = &g_spi2_ctrl,
  .p_cfg  = &g_spi2_cfg,
  .p_api  = &g_spi_on_spi_b,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef HAVE_RZV2H_SPI_MASTER
/****************************************************************************
 * Name: rzv2h_spi_lock
 *
 * Description:
 *   Acquire or release exclusive access to an SPI_B master bus.
 *
 * Input Parameters:
 *   dev  - SPI lower-half device instance.
 *   lock - True to acquire the bus mutex; false to release it.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int rzv2h_spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct rzv2h_spi_priv_s *priv = (struct rzv2h_spi_priv_s *)dev;

  return lock ? nxmutex_lock(&priv->lock) : nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: rzv2h_spi_calculate_bitrate
 *
 * Description:
 *   Ask FSP to calculate the clock divider and report the resulting rate.
 *
 * Input Parameters:
 *   priv      - RZ/V2H SPI_B channel state.
 *   frequency - Requested SPI clock frequency in hertz.
 *
 * Output Parameters:
 *   setting - Receives the calculated SPI_B divider setting.
 *   actual  - Receives the resulting SPI clock frequency in hertz.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 ****************************************************************************/

static int rzv2h_spi_calculate_bitrate(struct rzv2h_spi_priv_s *priv,
                                       uint32_t frequency,
                                       rspck_div_setting_t *setting,
                                       uint32_t *actual)
{
  uint32_t source;
  uint32_t target;
  uint32_t divisor;
  fsp_err_t err;

  if (frequency == 0 || setting == NULL || actual == NULL)
    {
      return -EINVAL;
    }

  /* All RZ/V2H SPI_B instances currently use PCLK.  Keeping the clock query
   * and divider calculation in FSP avoids duplicating SoC clock knowledge.
   */

  if (priv->extcfg->clock_source != SPI_B_CLOCK_SOURCE_PCLK)
    {
      return -ENOTSUP;
    }

  source = R_FSP_SystemClockHzGet(BSP_FEATURE_SPI_PCLK);
  if (source == 0)
    {
      return -EIO;
    }

  /* Avoid overflow in the FSP ceiling-division expression for requests
   * above SPI_B's maximum rate.  FSP specifies that these requests select
   * PCLK/2.
   */

  target = frequency > source / 2 ? source / 2 : frequency;
  err = R_SPI_B_CalculateBitrate(target, priv->extcfg->clock_source,
                                 setting);
  if (err != FSP_SUCCESS)
    {
      return rzv2h_fsp_err_to_errno(err);
    }

  divisor = 2U * ((uint32_t)setting->spbr + 1U) *
            (1U << setting->brdv);
  *actual = source / divisor;
  return OK;
}

/****************************************************************************
 * Name: rzv2h_spi_setfrequency
 *
 * Description:
 *   Select the closest supported frequency not exceeding the request.
 *   Apply the FSP-calculated divider by opening the FSP instance again.
 *
 * Input Parameters:
 *   dev       - SPI lower-half device instance.
 *   frequency - Requested SPI clock frequency in hertz.
 *
 * Returned Value:
 *   The configured SPI clock frequency in hertz.  If the request cannot be
 *   applied, the previously configured frequency is returned.
 ****************************************************************************/

static uint32_t rzv2h_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  struct rzv2h_spi_priv_s *priv = (struct rzv2h_spi_priv_s *)dev;
  rspck_div_setting_t old_setting;
  rspck_div_setting_t setting;
  uint32_t actual;
  int ret;

  if (frequency == priv->frequency && priv->actual != 0)
    {
      return priv->actual;
    }

  ret = rzv2h_spi_calculate_bitrate(priv, frequency, &setting, &actual);
  if (ret < 0)
    {
      spierr("ERROR: SPI_B channel %u cannot generate %lu Hz: %d\n",
             priv->cfg->channel, (unsigned long)frequency, ret);
      return priv->actual;
    }

  old_setting = priv->extcfg->spck_div;
  if (old_setting.spbr == setting.spbr &&
      old_setting.brdv == setting.brdv)
    {
      priv->frequency = frequency;
      priv->actual = actual;
      return actual;
    }

  priv->extcfg->spck_div = setting;
  ret = rzv2h_spi_open(priv);
  if (ret < 0)
    {
      priv->extcfg->spck_div = old_setting;
      spierr("ERROR: SPI_B channel %u frequency change failed: %d\n",
             priv->cfg->channel, ret);
      return priv->actual;
    }

  priv->frequency = frequency;
  priv->actual = actual;

  spiinfo("SPI_B channel %u frequency %lu -> %lu Hz\n",
          priv->cfg->channel, (unsigned long)frequency,
          (unsigned long)actual);
  return actual;
}

#ifdef CONFIG_SPI_DELAY_CONTROL
/****************************************************************************
 * Name: rzv2h_spi_setdelay
 *
 * Description:
 *   Validate a generic NuttX SPI delay request.  The current SPI_B mapping
 *   supports only requests that add no delay.
 *
 * Input Parameters:
 *   dev        - SPI lower-half device instance.
 *   startdelay - Delay from chip-select assertion to the first clock.
 *   stopdelay  - Delay from the last clock to chip-select deassertion.
 *   csdelay    - Delay between chip-select operations.
 *   ifdelay    - Delay between consecutive words or frames.
 *
 * Returned Value:
 *   Zero when all requested delays are zero; -ENOTSUP otherwise.
 *
 ****************************************************************************/

static int rzv2h_spi_setdelay(struct spi_dev_s *dev, uint32_t startdelay,
                              uint32_t stopdelay, uint32_t csdelay,
                              uint32_t ifdelay)
{
  UNUSED(dev);

  /* The board uses the native FSP SSL output in 4-wire mode and no SSL in
   * 3-wire clock-synchronous mode.  The FSP delay fields cannot provide
   * the generic NuttX delay semantics at run time.  Accept only requests
   * that require no additional delay.
   */

  return startdelay == 0 && stopdelay == 0 && csdelay == 0 && ifdelay == 0 ?
         OK : -ENOTSUP;
}
#endif

/****************************************************************************
 * Name: rzv2h_spi_setmode
 *
 * Description:
 *   Set CPOL and CPHA through the FSP configuration and open the FSP
 *   instance again.
 *
 * Input Parameters:
 *   dev  - SPI lower-half device instance.
 *   mode - Requested NuttX SPI clock mode.
 *
 * Returned Value:
 *   None.
 ****************************************************************************/

static void rzv2h_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct rzv2h_spi_priv_s *priv = (struct rzv2h_spi_priv_s *)dev;
  spi_clk_phase_t old_phase;
  spi_clk_polarity_t old_polarity;
  int ret;

  if (mode == priv->mode)
    {
      return;
    }

  if (mode < SPIDEV_MODE0 || mode > SPIDEV_MODE3)
    {
      spierr("ERROR: SPI_B channel %u does not support mode %u\n",
             priv->cfg->channel, mode);
      return;
    }

  old_phase = priv->cfg->clk_phase;
  old_polarity = priv->cfg->clk_polarity;
  priv->cfg->clk_phase = RZV2H_SPI_FSP_PHASE(mode);
  priv->cfg->clk_polarity = RZV2H_SPI_FSP_POLARITY(mode);

  ret = rzv2h_spi_open(priv);
  if (ret < 0)
    {
      priv->cfg->clk_phase = old_phase;
      priv->cfg->clk_polarity = old_polarity;
      spierr("ERROR: SPI_B channel %u mode change failed: %d\n",
             priv->cfg->channel, ret);
      return;
    }

  priv->mode = mode;

  spiinfo("SPI_B channel %u mode %u\n", priv->cfg->channel, mode);
}

/****************************************************************************
 * Name: rzv2h_spi_setbits
 *
 * Description:
 *   Save the word size passed to the FSP read/write API on each transfer.
 *
 * Input Parameters:
 *   dev   - SPI lower-half device instance.
 *   nbits - Requested number of bits per SPI word.
 *
 * Returned Value:
 *   None.
 ****************************************************************************/

static void rzv2h_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct rzv2h_spi_priv_s *priv = (struct rzv2h_spi_priv_s *)dev;

  if (nbits < 4 || nbits > 32)
    {
      spierr("ERROR: SPI_B channel %u does not support %d-bit words\n",
             priv->cfg->channel, nbits);
      return;
    }

  if (nbits == priv->nbits)
    {
      return;
    }

  priv->nbits = nbits;
  spiinfo("SPI_B channel %u uses %d-bit words\n",
          priv->cfg->channel, nbits);
}

/****************************************************************************
 * Name: rzv2h_spi_send
 *
 * Description:
 *   Exchange one SPI word using the currently configured word width.
 *
 * Input Parameters:
 *   dev - SPI lower-half device instance.
 *   wd  - Word to transmit.
 *
 * Returned Value:
 *   The received word.  An all-ones value of the configured width is
 *   returned if the transfer fails.
 ****************************************************************************/

static uint32_t rzv2h_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct rzv2h_spi_priv_s *priv = (struct rzv2h_spi_priv_s *)dev;
  int ret;

  if (priv->nbits <= 8)
    {
      uint8_t txword = (uint8_t)wd;
      uint8_t rxword = UINT8_MAX;

      ret = rzv2h_spi_transfer(priv, &txword, &rxword, 1);
      return ret < 0 ? UINT8_MAX : rxword;
    }
  else if (priv->nbits <= 16)
    {
      uint16_t txword = (uint16_t)wd;
      uint16_t rxword = UINT16_MAX;

      ret = rzv2h_spi_transfer(priv, &txword, &rxword, 1);
      return ret < 0 ? UINT16_MAX : rxword;
    }
  else
    {
      uint32_t txword = wd;
      uint32_t rxword = UINT32_MAX;

      ret = rzv2h_spi_transfer(priv, &txword, &rxword, 1);
      return ret < 0 ? UINT32_MAX : rxword;
    }
}

/****************************************************************************
 * Name: rzv2h_spi_transfer
 *
 * Description:
 *   Start an asynchronous FSP transfer and wait for the configured callback,
 *   adapting FSP's interface to NuttX's synchronous SPI lower half.
 *
 * Input Parameters:
 *   priv     - RZ/V2H SPI_B channel state.
 *   txbuffer - Buffer containing words to transmit, or NULL for receive-only
 *              operation.
 *   nwords   - Number of words to transfer.
 *
 * Output Parameters:
 *   rxbuffer - Receives transferred words, or NULL for transmit-only
 *              operation.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 ****************************************************************************/

static int rzv2h_spi_transfer(struct rzv2h_spi_priv_s *priv,
                              const void *txbuffer, void *rxbuffer,
                              size_t nwords)
{
  const spi_instance_t *instance = priv->instance;
  spi_bit_width_t bit_width;
  fsp_err_t err;
  int ret;

  if (nwords == 0)
    {
      return OK;
    }

  if (txbuffer == NULL && rxbuffer == NULL)
    {
      return -EINVAL;
    }

  if (nwords > UINT32_MAX)
    {
      return -E2BIG;
    }

  if (priv->transferring)
    {
      return -EBUSY;
    }

  ret = nxsem_reset(&priv->waitsem, 0);
  if (ret < 0)
    {
      return ret;
    }

  bit_width = (spi_bit_width_t)(priv->nbits - 1);
  priv->event = SPI_EVENT_TRANSFER_ABORTED;
  priv->transferring = true;

  if (txbuffer == NULL)
    {
      err = R_SPI_B_Read(instance->p_ctrl, rxbuffer,
                          (uint32_t)nwords, bit_width);
    }
  else if (rxbuffer == NULL)
    {
      err = R_SPI_B_Write(instance->p_ctrl, txbuffer,
                          (uint32_t)nwords, bit_width);
    }
  else
    {
      err = R_SPI_B_WriteRead(instance->p_ctrl, txbuffer, rxbuffer,
                              (uint32_t)nwords, bit_width);
    }

  ret = rzv2h_fsp_err_to_errno(err);
  if (ret < 0)
    {
      priv->transferring = false;
      spierr("ERROR: SPI_B channel %u transfer start failed: %d\n",
             instance->p_cfg->channel, ret);
      return ret;
    }

  ret = nxsem_wait_uninterruptible(&priv->waitsem);

  if (ret < 0)
    {
      /* Do not clear transferring here.  The FSP operation can still
       * complete, and its callback owns the pending-to-complete transition.
       */

      return ret;
    }

  ret = rzv2h_spi_event_to_errno(priv->event);

  if (ret < 0)
    {
      spierr("ERROR: SPI_B channel %u transfer event %u: %d\n",
             instance->p_cfg->channel, priv->event, ret);
    }

  return ret;
}

#ifdef CONFIG_SPI_EXCHANGE
/****************************************************************************
 * Name: rzv2h_spi_exchange
 *
 * Description:
 *   Exchange a block of words through the synchronous NuttX master API.
 *
 * Input Parameters:
 *   dev      - SPI lower-half device instance.
 *   txbuffer - Buffer containing words to transmit, or NULL.
 *   nwords   - Number of words to exchange.
 *
 * Output Parameters:
 *   rxbuffer - Receives the exchanged words, or NULL.
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int rzv2h_spi_exchange(struct spi_dev_s *dev,
                              const void *txbuffer, void *rxbuffer,
                              size_t nwords)
{
  struct rzv2h_spi_priv_s *priv = (struct rzv2h_spi_priv_s *)dev;

  return rzv2h_spi_transfer(priv, txbuffer, rxbuffer, nwords);
}
#else
/****************************************************************************
 * Name: rzv2h_spi_sndblock
 *
 * Description:
 *   Transmit a block of words through the synchronous NuttX master API.
 *
 * Input Parameters:
 *   dev      - SPI lower-half device instance.
 *   txbuffer - Buffer containing words to transmit.
 *   nwords   - Number of words to transmit.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void rzv2h_spi_sndblock(struct spi_dev_s *dev,
                               const void *txbuffer, size_t nwords)
{
  struct rzv2h_spi_priv_s *priv = (struct rzv2h_spi_priv_s *)dev;

  (void)rzv2h_spi_transfer(priv, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: rzv2h_spi_recvblock
 *
 * Description:
 *   Receive a block of words through the synchronous NuttX master API.
 *
 * Input Parameters:
 *   dev    - SPI lower-half device instance.
 *   nwords - Number of words to receive.
 *
 * Output Parameters:
 *   rxbuffer - Buffer that receives the incoming words.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void rzv2h_spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                                size_t nwords)
{
  struct rzv2h_spi_priv_s *priv = (struct rzv2h_spi_priv_s *)dev;

  (void)rzv2h_spi_transfer(priv, NULL, rxbuffer, nwords);
}
#endif

#endif /* HAVE_RZV2H_SPI_MASTER */

#ifdef HAVE_RZV2H_SPI_SLAVE
/****************************************************************************
 * Name: rzv2h_spi_slave_wordsize
 *
 * Description:
 *   Convert the configured SPI word width to its storage size in bytes.
 *
 * Input Parameters:
 *   priv - RZ/V2H SPI_B channel state.
 *
 * Returned Value:
 *   One, two, or four bytes per configured SPI word.
 ****************************************************************************/

static size_t rzv2h_spi_slave_wordsize(struct rzv2h_spi_priv_s *priv)
{
  if (priv->nbits <= 8)
    {
      return 1;
    }
  else if (priv->nbits <= 16)
    {
      return 2;
    }

  return 4;
}

/****************************************************************************
 * Name: rzv2h_spi_slave_start
 *
 * Description:
 *   Copy and commit one slave transaction.  FSP requires the transfer
 *   length before the external master starts clocking, so each NuttX
 *   enqueue operation maps to one FSP full-duplex transfer.
 *
 * Input Parameters:
 *   slave  - RZ/V2H SPI slave controller state.
 *   data   - Words to transmit, or NULL for receive-only operation.
 *   nwords - Number of words in the transaction.
 *
 * Returned Value:
 *   The committed word count on success; a negated errno value on failure.
 ****************************************************************************/

static int rzv2h_spi_slave_start(struct spi_dev_slave_s *slave,
                                 const void *data, size_t nwords)
{
  struct rzv2h_spi_priv_s *priv = slave->priv;
  spi_bit_width_t bit_width;
  irqstate_t flags;
  size_t wordsize;
  fsp_err_t err;
  int ret;

  if (nwords == 0)
    {
      return -EINVAL;
    }

  if (nwords > CONFIG_RZV2H_SPI_SLAVE_BUFFER_SIZE)
    {
      return -E2BIG;
    }

  flags = spin_lock_irqsave(&slave->lock);
  if (slave->dev == NULL || !slave->opened)
    {
      spin_unlock_irqrestore(&slave->lock, flags);
      return -ENODEV;
    }

  if (priv->transferring || slave->nwords != 0)
    {
      spin_unlock_irqrestore(&slave->lock, flags);
      return -ENOSPC;
    }

  wordsize = rzv2h_spi_slave_wordsize(priv);

  if (data != NULL)
    {
      memcpy(slave->txbuffer, data, nwords * wordsize);
    }

  slave->nwords = nwords;
  slave->rxoffset = 0;
  priv->event = SPI_EVENT_TRANSFER_ABORTED;
  priv->transferring = true;
  spin_unlock_irqrestore(&slave->lock, flags);

  bit_width = (spi_bit_width_t)(priv->nbits - 1);
  UP_DSB();
  if (data == NULL)
    {
      err = R_SPI_B_Read(priv->instance->p_ctrl, slave->rxbuffer,
                         (uint32_t)nwords, bit_width);
    }
  else
    {
      err = R_SPI_B_WriteRead(priv->instance->p_ctrl, slave->txbuffer,
                              slave->rxbuffer, (uint32_t)nwords,
                              bit_width);
    }

  ret = rzv2h_fsp_err_to_errno(err);
  if (ret < 0)
    {
      flags = spin_lock_irqsave(&slave->lock);
      priv->transferring = false;
      slave->nwords = 0;
      slave->rxoffset = 0;
      spin_unlock_irqrestore(&slave->lock, flags);
      spierr("ERROR: SPI_B slave channel %u transfer start failed: %d\n",
             priv->cfg->channel, ret);
      return ret;
    }

  return (int)nwords;
}

/****************************************************************************
 * Name: rzv2h_spi_slave_bind
 *
 * Description:
 *   Bind an SPI slave device, apply its mode and word format, open SPI_B,
 *   and arm any data immediately supplied by the device.
 *
 * Input Parameters:
 *   ctrlr - SPI slave controller instance.
 *   dev   - SPI slave device to bind.
 *   mode  - Clock polarity and phase requested by the slave device.
 *   nbits - Bits per word; a negative value requests LSB-first order.
 *
 * Returned Value:
 *   None.  Binding failures are reported through SPIS_DEV_NOTIFY().
 ****************************************************************************/

static void rzv2h_spi_slave_bind(struct spi_slave_ctrlr_s *ctrlr,
                                 struct spi_slave_dev_s *dev,
                                 enum spi_slave_mode_e mode, int nbits)
{
  struct spi_dev_slave_s *slave =
    (struct spi_dev_slave_s *)ctrlr;
  struct rzv2h_spi_priv_s *priv = slave->priv;
  const void *data = NULL;
  irqstate_t flags;
  size_t nwords;
  int ret;

  DEBUGASSERT(slave != NULL && priv != NULL && dev != NULL);
  DEBUGASSERT(slave->dev == NULL);

  if (mode < SPISLAVE_MODE0 || mode > SPISLAVE_MODE3 || nbits == 0 ||
      nbits < -32 || nbits > 32 || (nbits > -4 && nbits < 4))
    {
      SPIS_DEV_NOTIFY(dev, SPISLAVE_TRANSFER_FAILED);
      return;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      SPIS_DEV_NOTIFY(dev, SPISLAVE_TRANSFER_FAILED);
      return;
    }

  flags = spin_lock_irqsave(&slave->lock);
  slave->dev = dev;
  spin_unlock_irqrestore(&slave->lock, flags);

  priv->mode = (enum spi_mode_e)mode;
  priv->nbits = nbits < 0 ? -nbits : nbits;
  priv->cfg->clk_phase = RZV2H_SPI_FSP_PHASE(mode);
  priv->cfg->clk_polarity = RZV2H_SPI_FSP_POLARITY(mode);
  priv->cfg->bit_order = nbits < 0 ? SPI_BIT_ORDER_LSB_FIRST :
                                    SPI_BIT_ORDER_MSB_FIRST;

  ret = rzv2h_spi_open(priv);
  if (ret >= 0)
    {
      flags = spin_lock_irqsave(&slave->lock);
      slave->opened = true;
      spin_unlock_irqrestore(&slave->lock, flags);
    }

  nxmutex_unlock(&priv->lock);

  if (ret < 0)
    {
      spierr("ERROR: SPI_B slave channel %u bind failed: %d\n",
             priv->cfg->channel, ret);
      SPIS_DEV_NOTIFY(dev, SPISLAVE_TRANSFER_FAILED);
      return;
    }

  SPIS_DEV_SELECT(dev, false);
  SPIS_DEV_CMDDATA(dev, false);

  nwords = SPIS_DEV_GETDATA(dev, &data);
  if (data != NULL && nwords > 0)
    {
      ret = rzv2h_spi_slave_start(slave, data, nwords);
      if (ret < 0)
        {
          SPIS_DEV_NOTIFY(dev, SPISLAVE_TRANSFER_FAILED);
        }
    }
}

/****************************************************************************
 * Name: rzv2h_spi_slave_unbind
 *
 * Description:
 *   Close SPI_B and detach the currently bound SPI slave device.
 *
 * Input Parameters:
 *   ctrlr - SPI slave controller instance.
 *
 * Returned Value:
 *   None.
 ****************************************************************************/

static void rzv2h_spi_slave_unbind(struct spi_slave_ctrlr_s *ctrlr)
{
  struct spi_dev_slave_s *slave =
    (struct spi_dev_slave_s *)ctrlr;
  struct rzv2h_spi_priv_s *priv = slave->priv;
  irqstate_t flags;
  fsp_err_t err;
  bool opened;
  int ret;

  DEBUGASSERT(slave != NULL && priv != NULL && slave->dev != NULL);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      spierr("ERROR: SPI_B slave channel %u unbind lock failed: %d\n",
             priv->cfg->channel, ret);
      return;
    }

  flags = spin_lock_irqsave(&slave->lock);
  opened = slave->opened;
  slave->opened = false;
  spin_unlock_irqrestore(&slave->lock, flags);

  if (opened)
    {
      err = R_SPI_B_Close(priv->instance->p_ctrl);
      if (err != FSP_SUCCESS)
        {
          spierr("ERROR: SPI_B slave channel %u close failed: %d\n",
                 priv->cfg->channel, rzv2h_fsp_err_to_errno(err));
        }
    }

  flags = spin_lock_irqsave(&slave->lock);
  priv->transferring = false;
  slave->nwords = 0;
  slave->rxoffset = 0;
  slave->dev = NULL;
  spin_unlock_irqrestore(&slave->lock, flags);

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: rzv2h_spi_slave_enqueue
 *
 * Description:
 *   Queue and immediately commit one slave transaction to SPI_B.
 *
 * Input Parameters:
 *   ctrlr - SPI slave controller instance.
 *   data  - Words to transmit, or NULL for receive-only operation.
 *   nwords - Number of words in the transaction.
 *
 * Returned Value:
 *   The committed word count on success; a negated errno value on failure.
 ****************************************************************************/

static int rzv2h_spi_slave_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                                   const void *data, size_t nwords)
{
  struct spi_dev_slave_s *slave =
    (struct spi_dev_slave_s *)ctrlr;

  DEBUGASSERT(slave != NULL && slave->priv != NULL);
  return rzv2h_spi_slave_start(slave, data, nwords);
}

/****************************************************************************
 * Name: rzv2h_spi_slave_qfull
 *
 * Description:
 *   Report whether the single committed slave transaction slot is busy or
 *   unavailable.
 *
 * Input Parameters:
 *   ctrlr - SPI slave controller instance.
 *
 * Returned Value:
 *   True if another transaction cannot be accepted; false otherwise.
 ****************************************************************************/

static bool rzv2h_spi_slave_qfull(struct spi_slave_ctrlr_s *ctrlr)
{
  struct spi_dev_slave_s *slave =
    (struct spi_dev_slave_s *)ctrlr;
  irqstate_t flags;
  bool full;

  DEBUGASSERT(slave != NULL && slave->priv != NULL);
  flags = spin_lock_irqsave(&slave->lock);
  full = slave->dev == NULL || !slave->opened ||
         slave->priv->transferring || slave->nwords != 0;
  spin_unlock_irqrestore(&slave->lock, flags);
  return full;
}

/****************************************************************************
 * Name: rzv2h_spi_slave_qflush
 *
 * Description:
 *   Flush uncommitted slave output data.  This implementation commits every
 *   accepted block immediately, so there is no pending queue to discard.
 *
 * Input Parameters:
 *   ctrlr - SPI slave controller instance.
 *
 * Returned Value:
 *   None.
 ****************************************************************************/

static void rzv2h_spi_slave_qflush(struct spi_slave_ctrlr_s *ctrlr)
{
  /* Each accepted output block is immediately committed to FSP.  The
   * NuttX contract permits qflush() to leave committed data untouched.
   */

  UNUSED(ctrlr);
}

/****************************************************************************
 * Name: rzv2h_spi_slave_qpoll
 *
 * Description:
 *   Offer completed receive data to the bound slave device or arm the
 *   configured fixed-length receive transaction when no data is pending.
 *
 * Input Parameters:
 *   ctrlr - SPI slave controller instance.
 *
 * Returned Value:
 *   The number of received words not yet accepted by the slave device.
 ****************************************************************************/

static size_t rzv2h_spi_slave_qpoll(struct spi_slave_ctrlr_s *ctrlr)
{
  struct spi_dev_slave_s *slave =
    (struct spi_dev_slave_s *)ctrlr;
  struct spi_slave_dev_s *dev;
  size_t accepted;
  size_t nwords;
  size_t rxoffset;
  size_t remaining;
  size_t wordsize;
  uint8_t *rxdata;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(slave != NULL && slave->priv != NULL);

  flags = spin_lock_irqsave(&slave->lock);
  dev = slave->dev;

  if (dev == NULL || !slave->opened || slave->priv->transferring)
    {
      spin_unlock_irqrestore(&slave->lock, flags);
      return 0;
    }

  if (slave->nwords == 0)
    {
      spin_unlock_irqrestore(&slave->lock, flags);

      /* FSP cannot remain armed for an unknown number of clocks.  Polling
       * for input therefore commits one configured fixed-length receive.
       */

      ret = rzv2h_spi_slave_start(
              slave, NULL, CONFIG_RZV2H_SPI_SLAVE_DEFAULT_NWORDS);
      if (ret < 0 && ret != -ENOSPC)
        {
          spierr("ERROR: SPI_B slave channel %u listen start failed: %d\n",
                 slave->priv->cfg->channel, ret);
          SPIS_DEV_NOTIFY(dev, SPISLAVE_TRANSFER_FAILED);
        }

      return 0;
    }

  if (slave->rxoffset >= slave->nwords)
    {
      slave->nwords = 0;
      slave->rxoffset = 0;
      spin_unlock_irqrestore(&slave->lock, flags);
      return 0;
    }

  nwords = slave->nwords;
  rxoffset = slave->rxoffset;
  remaining = nwords - rxoffset;
  wordsize = rzv2h_spi_slave_wordsize(slave->priv);
  rxdata = (uint8_t *)slave->rxbuffer + rxoffset * wordsize;
  spin_unlock_irqrestore(&slave->lock, flags);

  /* Do not hold the controller spinlock across a device callback. */

  accepted = SPIS_DEV_RECEIVE(dev, rxdata, remaining);
  if (accepted > remaining)
    {
      accepted = remaining;
    }

  flags = spin_lock_irqsave(&slave->lock);
  if (slave->dev != dev || slave->nwords != nwords ||
      slave->rxoffset != rxoffset || slave->priv->transferring)
    {
      remaining = slave->nwords > slave->rxoffset ?
                  slave->nwords - slave->rxoffset : 0;
      spin_unlock_irqrestore(&slave->lock, flags);
      return remaining;
    }

  slave->rxoffset = rxoffset + accepted;
  remaining = slave->nwords - slave->rxoffset;

  if (remaining == 0)
    {
      slave->nwords = 0;
      slave->rxoffset = 0;
    }

  spin_unlock_irqrestore(&slave->lock, flags);
  return remaining;
}

/****************************************************************************
 * Name: rzv2h_spi_slave_callback
 *
 * Description:
 *   Complete a slave transfer, deliver received words to the bound device,
 *   and report the resulting slave transfer state.
 *
 * Input Parameters:
 *   priv  - RZ/V2H SPI_B channel state.
 *   event - Transfer event reported by FSP.
 *
 * Returned Value:
 *   None.
 ****************************************************************************/

static void rzv2h_spi_slave_callback(struct rzv2h_spi_priv_s *priv,
                                     spi_event_t event)
{
  struct spi_dev_slave_s *slave = &priv->slave;
  struct spi_slave_dev_s *dev;
  irqstate_t flags;
  size_t accepted;
  size_t nwords;

  flags = spin_lock_irqsave(&slave->lock);
  dev = slave->dev;
  if (!priv->transferring || dev == NULL)
    {
      priv->transferring = false;
      spin_unlock_irqrestore(&slave->lock, flags);
      return;
    }

  priv->event = event;

  if (event != SPI_EVENT_TRANSFER_COMPLETE)
    {
      priv->transferring = false;
      slave->nwords = 0;
      slave->rxoffset = 0;
      spin_unlock_irqrestore(&slave->lock, flags);
      SPIS_DEV_NOTIFY(dev, SPISLAVE_TRANSFER_FAILED);
      return;
    }

  nwords = slave->nwords;
  priv->transferring = false;
  spin_unlock_irqrestore(&slave->lock, flags);

  accepted = SPIS_DEV_RECEIVE(dev, slave->rxbuffer, nwords);
  if (accepted > nwords)
    {
      accepted = nwords;
    }

  flags = spin_lock_irqsave(&slave->lock);
  if (slave->dev != dev || slave->nwords != nwords || priv->transferring)
    {
      spin_unlock_irqrestore(&slave->lock, flags);
      return;
    }

  slave->rxoffset = accepted;
  if (slave->rxoffset == slave->nwords)
    {
      slave->nwords = 0;
      slave->rxoffset = 0;
    }

  spin_unlock_irqrestore(&slave->lock, flags);

  SPIS_DEV_NOTIFY(dev, SPISLAVE_RX_COMPLETE);
  SPIS_DEV_NOTIFY(dev, SPISLAVE_TX_COMPLETE);
}
#endif

/****************************************************************************
 * Name: rzv2h_spi_callback
 *
 * Description:
 *   Dispatch an FSP SPI_B event to the active master or slave completion
 *   path.
 *
 * Input Parameters:
 *   args - FSP callback arguments containing the channel context and event.
 *
 * Returned Value:
 *   None.
 ****************************************************************************/

static void rzv2h_spi_callback(spi_callback_args_t *args)
{
  struct rzv2h_spi_priv_s *priv = (struct rzv2h_spi_priv_s *)args->p_context;

#ifdef HAVE_RZV2H_SPI_SLAVE
  if (priv != NULL && priv->cfg->operating_mode == SPI_MODE_SLAVE)
    {
      rzv2h_spi_slave_callback(priv, args->event);
      return;
    }
#endif

  if (priv != NULL && priv->transferring)
    {
      priv->event = args->event;
      priv->transferring = false;
      nxsem_post(&priv->waitsem);
    }
}

/****************************************************************************
 * Name: rzv2h_spi_interrupt
 *
 * Description:
 *   Dispatch a NuttX GIC interrupt to the matching FSP SPI_B ISR.
 *
 * Input Parameters:
 *   irq     - NuttX GIC interrupt number.
 *   context - Saved interrupt context; unused by this handler.
 *   arg     - RZ/V2H SPI_B channel state supplied at IRQ attachment.
 *
 * Returned Value:
 *   Zero after dispatching a recognized SPI_B interrupt; -EINVAL if the
 *   interrupt does not match the configured RXI, TXI, TEI, or ERI source.
 *
 ****************************************************************************/

static int rzv2h_spi_interrupt(int irq, void *context, void *arg)
{
  struct rzv2h_spi_priv_s *priv = (struct rzv2h_spi_priv_s *)arg;
  const spi_cfg_t *cfg = priv->instance->p_cfg;
  IRQn_Type fsp_irq;
  void (*fsp_isr)(void);

  UNUSED(context);
  DEBUGASSERT(priv != NULL && cfg != NULL);

  fsp_irq = (IRQn_Type)(irq - BSP_CORTEX_VECTOR_TABLE_ENTRIES);

  if (fsp_irq == cfg->rxi_irq)
    {
      fsp_isr = spi_b_rxi_isr;
    }
  else if (fsp_irq == cfg->txi_irq)
    {
      fsp_isr = spi_b_txi_isr;
    }
  else if (fsp_irq == cfg->tei_irq)
    {
      fsp_isr = spi_b_tei_isr;
    }
  else if (fsp_irq == cfg->eri_irq)
    {
      fsp_isr = spi_b_eri_isr;
    }
  else
    {
      return -EINVAL;
    }

  rzv2h_interrupt_common_handler(fsp_irq, fsp_isr);

  return OK;
}

/****************************************************************************
 * Name: rzv2h_spi_irq_attach
 *
 * Description:
 *   Route the SPI_B interrupt events, attach their NuttX handlers, configure
 *   priorities and trigger types, and leave the IRQs disabled for FSP open.
 *
 * Input Parameters:
 *   priv - RZ/V2H SPI_B channel state.
 *
 * Returned Value:
 *   Zero on success; a negated errno value or ERROR on failure.
 ****************************************************************************/

static int rzv2h_spi_irq_attach(struct rzv2h_spi_priv_s *priv)
{
  const spi_cfg_t *cfg = priv->instance->p_cfg;

  /* Convert FSP IRQn_Type to GIC INTIDs for the NuttX IRQ interfaces. */

  int rxi_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->rxi_irq);
  int txi_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->txi_irq);
  int tei_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->tei_irq);
  int eri_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->eri_irq);
  int ret;

  /* Keep the IRQs disabled until their NuttX handlers are attached and FSP
   * has installed its contexts.  R_SPI_B_Open() enables RXI, TXI and ERI.
   * TEI is enabled by FSP only after the final frame has been received.
   */

  up_disable_irq(rxi_irq);
  up_disable_irq(txi_irq);
  up_disable_irq(tei_irq);
  up_disable_irq(eri_irq);

  /* Connect the statically-assigned SEL lines (optionally overridden via
   * Kconfig) to their events.
   */

  if (rzv2h_intsel_connect_event(cfg->rxi_irq, priv->rxi_irq_event,
                                  BSP_GIC_SPI_DETECT_EDGE) < 0 ||
      rzv2h_intsel_connect_event(cfg->txi_irq, priv->txi_irq_event,
                                  BSP_GIC_SPI_DETECT_EDGE) < 0)
    {
      rzv2h_intsel_disconnect_event(cfg->rxi_irq);
      rzv2h_intsel_disconnect_event(cfg->txi_irq);

      return ERROR;
    }

  ret = irq_attach(rxi_irq, rzv2h_spi_interrupt, priv);
  if (ret < 0)
    {
      goto errout_events;
    }

  ret = irq_attach(txi_irq, rzv2h_spi_interrupt, priv);
  if (ret < 0)
    {
      goto errout_rxi;
    }

  ret = irq_attach(tei_irq, rzv2h_spi_interrupt, priv);
  if (ret < 0)
    {
      goto errout_txi;
    }

  ret = irq_attach(eri_irq, rzv2h_spi_interrupt, priv);
  if (ret < 0)
    {
      goto errout_tei;
    }

  ret = up_prioritize_irq(rxi_irq, cfg->rxi_ipl << 4);
  if (ret < 0)
    {
      goto errout_eri;
    }

  ret = up_prioritize_irq(txi_irq, cfg->txi_ipl << 4);
  if (ret < 0)
    {
      goto errout_eri;
    }

  ret = up_prioritize_irq(tei_irq, cfg->tei_ipl << 4);
  if (ret < 0)
    {
      goto errout_eri;
    }

  ret = up_prioritize_irq(eri_irq, cfg->eri_ipl << 4);
  if (ret < 0)
    {
      goto errout_eri;
    }

  ret = rzv2h_spi_irq_configure(priv);
  if (ret < 0)
    {
      goto errout_eri;
    }

  /* Leave the IRQs disabled here.  R_SPI_B_Open() clears and enables RXI,
   * TXI and ERI.  FSP keeps TEI disabled until the last frame is received.
   */

  return OK;

errout_eri:
  irq_detach(eri_irq);
errout_tei:
  irq_detach(tei_irq);
errout_txi:
  irq_detach(txi_irq);
errout_rxi:
  irq_detach(rxi_irq);
errout_events:
  rzv2h_intsel_disconnect_event(cfg->txi_irq);
  rzv2h_intsel_disconnect_event(cfg->rxi_irq);
  return ret;
}

/****************************************************************************
 * Name: rzv2h_spi_irq_configure
 *
 * Description:
 *   Restore the SPI_B interrupt trigger modes.  FSP configures IRQs during
 *   R_SPI_B_Open(), but its weak vector table does not describe the SEL
 *   event routing installed by NuttX.  FSP therefore restores the default
 *   level trigger for RXI and TXI even though SPI_B produces pulse events.
 *
 * Input Parameters:
 *   priv - RZ/V2H SPI_B channel state.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 ****************************************************************************/

static int rzv2h_spi_irq_configure(struct rzv2h_spi_priv_s *priv)
{
  const spi_cfg_t *cfg = priv->instance->p_cfg;
  int ret;

  ret = up_set_irq_type(RZV2H_FSP_TO_GIC_IRQ(cfg->rxi_irq),
                        IRQ_RISING_EDGE);
  if (ret < 0)
    {
      return ret;
    }

  ret = up_set_irq_type(RZV2H_FSP_TO_GIC_IRQ(cfg->txi_irq),
                        IRQ_RISING_EDGE);
  if (ret < 0)
    {
      return ret;
    }

  ret = up_set_irq_type(RZV2H_FSP_TO_GIC_IRQ(cfg->tei_irq),
                        IRQ_RISING_EDGE);
  if (ret < 0)
    {
      return ret;
    }

  return up_set_irq_type(RZV2H_FSP_TO_GIC_IRQ(cfg->eri_irq),
                         IRQ_HIGH_LEVEL);
}

/****************************************************************************
 * Name: rzv2h_spi_irq_detach
 *
 * Description:
 *   Disable and detach all SPI_B channel interrupts and remove the RXI and
 *   TXI INTSEL event routes.
 *
 * Input Parameters:
 *   priv - RZ/V2H SPI_B channel state.
 *
 * Returned Value:
 *   None.
 ****************************************************************************/

static void rzv2h_spi_irq_detach(struct rzv2h_spi_priv_s *priv)
{
  const spi_cfg_t *cfg = priv->instance->p_cfg;
  int rxi_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->rxi_irq);
  int txi_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->txi_irq);
  int tei_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->tei_irq);
  int eri_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->eri_irq);

  up_disable_irq(rxi_irq);
  up_disable_irq(txi_irq);
  up_disable_irq(tei_irq);
  up_disable_irq(eri_irq);

  irq_detach(rxi_irq);
  irq_detach(txi_irq);
  irq_detach(tei_irq);
  irq_detach(eri_irq);

  rzv2h_intsel_disconnect_event(cfg->rxi_irq);
  rzv2h_intsel_disconnect_event(cfg->txi_irq);
}

/****************************************************************************
 * Name: rzv2h_spi_open
 *
 * Description:
 *   Open one FSP instance and restore the interrupt trigger configuration
 *   that FSP cannot infer from NuttX's INTSEL routing.
 *
 * Input Parameters:
 *   priv - RZ/V2H SPI_B channel state.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 ****************************************************************************/

static int rzv2h_spi_open(struct rzv2h_spi_priv_s *priv)
{
  fsp_err_t err;
  int ret;

  err = R_SPI_B_Open(priv->instance->p_ctrl, priv->instance->p_cfg);
  ret = rzv2h_fsp_err_to_errno(err);
  if (ret < 0)
    {
      spierr("ERROR: SPI_B channel %u open failed: %d\n",
             priv->cfg->channel, ret);
      return ret;
    }

  ret = rzv2h_spi_irq_configure(priv);
  if (ret < 0)
    {
      spierr("ERROR: SPI_B channel %u IRQ setup failed: %d\n",
             priv->cfg->channel, ret);

      err = R_SPI_B_Close(priv->instance->p_ctrl);
      if (err != FSP_SUCCESS)
        {
          spierr("ERROR: SPI_B channel %u close failed: %d\n",
                 priv->cfg->channel, rzv2h_fsp_err_to_errno(err));
        }
    }

  return ret;
}

/****************************************************************************
 * Name: rzv2h_spi_get_channel
 *
 * Description:
 *   Find an enabled SPI_B channel configured for the requested role.
 *
 * Input Parameters:
 *   channel - SPI_B channel number.
 *   role    - Required FSP master or slave operating mode.
 *
 * Returned Value:
 *   A pointer to the channel state on success; NULL if the channel is not
 *   enabled or is configured for a different role.
 ****************************************************************************/

static struct rzv2h_spi_priv_s *rzv2h_spi_get_channel(int channel,
                                                       spi_mode_t role)
{
  struct rzv2h_spi_priv_s *priv;

  switch (channel)
    {
#ifdef CONFIG_RZV2H_SPI_CHANNEL_0
      case RZV2H_SPI_CHANNEL_0:
        priv = &g_spi0_priv;
        break;
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_1
      case RZV2H_SPI_CHANNEL_1:
        priv = &g_spi1_priv;
        break;
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_2
      case RZV2H_SPI_CHANNEL_2:
        priv = &g_spi2_priv;
        break;
#endif

      default:
        spierr("ERROR: SPI_B channel %d is not enabled\n", channel);
        return NULL;
    }

  if (priv->cfg->operating_mode != role)
    {
      spierr("ERROR: SPI_B channel %d is configured as a %s\n",
             channel, priv->cfg->operating_mode == SPI_MODE_MASTER ?
             "master" : "slave");
      return NULL;
    }

  return priv;
}

/****************************************************************************
 * Name: rzv2h_spi_initialize
 *
 * Description:
 *   Perform the common one-time setup for either role.  A master is opened
 *   here after its initial bitrate is calculated.  A slave is opened later
 *   by bind(), when the slave device supplies mode, width, and bit order.
 *
 * Input Parameters:
 *   priv - RZ/V2H SPI_B channel state.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 ****************************************************************************/

static int rzv2h_spi_initialize(struct rzv2h_spi_priv_s *priv)
{
#ifdef HAVE_RZV2H_SPI_MASTER
  rspck_div_setting_t setting;
  uint32_t actual;
#endif
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      spierr("ERROR: SPI_B channel %u initialization lock failed: %d\n",
             priv->cfg->channel, ret);
      return ret;
    }

  if (priv->initialized)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

#ifdef HAVE_RZV2H_SPI_MASTER
  if (priv->cfg->operating_mode == SPI_MODE_MASTER)
    {
      ret = rzv2h_spi_calculate_bitrate(priv, priv->frequency, &setting,
                                        &actual);
      if (ret < 0)
        {
          spierr("ERROR: SPI_B channel %u bitrate setup failed: %d\n",
                 priv->cfg->channel, ret);
          goto errout_unlock;
        }

      priv->extcfg->spck_div = setting;
      priv->actual = actual;
    }
#endif

  ret = rzv2h_spi_irq_attach(priv);
  if (ret < 0)
    {
      spierr("ERROR: SPI_B channel %u IRQ attach failed: %d\n",
             priv->cfg->channel, ret);
      goto errout_unlock;
    }

#ifdef HAVE_RZV2H_SPI_MASTER
  if (priv->cfg->operating_mode == SPI_MODE_MASTER)
    {
      ret = rzv2h_spi_open(priv);
      if (ret < 0)
        {
          rzv2h_spi_irq_detach(priv);
          goto errout_unlock;
        }
    }
#endif

  priv->initialized = true;
  nxmutex_unlock(&priv->lock);
  return OK;

errout_unlock:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_spibus_initialize
 *
 * Description:
 *   Initialize an enabled SPI_B channel configured as a master.
 *
 * Input Parameters:
 *   channel - SPI_B channel number.
 *
 * Returned Value:
 *   The initialized SPI master lower-half instance on success; NULL if the
 *   channel is unavailable, has the wrong role, or initialization fails.
 ****************************************************************************/

#ifdef HAVE_RZV2H_SPI_MASTER
struct spi_dev_s *rzv2h_spibus_initialize(int channel)
{
  struct rzv2h_spi_priv_s *priv;
  int ret;

  priv = rzv2h_spi_get_channel(channel, SPI_MODE_MASTER);
  if (priv == NULL)
    {
      return NULL;
    }

  ret = rzv2h_spi_initialize(priv);
  if (ret < 0)
    {
      return NULL;
    }

  spiinfo("SPI_B master channel %d: %lu Hz, mode %u, %u bits\n",
          channel, (unsigned long)priv->actual, priv->mode, priv->nbits);

  return &priv->master;
}
#endif /* HAVE_RZV2H_SPI_MASTER */

#ifdef HAVE_RZV2H_SPI_SLAVE
/****************************************************************************
 * Name: rzv2h_spi_slave_initialize
 *
 * Description:
 *   Initialize an enabled SPI_B slave controller.  FSP is opened later by
 *   bind(), when the slave device supplies its mode and word width.
 *
 * Input Parameters:
 *   channel - SPI_B channel number.
 *
 * Returned Value:
 *   The initialized SPI slave controller on success; NULL if the channel is
 *   unavailable, has the wrong role, or initialization fails.
 ****************************************************************************/

struct spi_slave_ctrlr_s *rzv2h_spi_slave_initialize(int channel)
{
  struct rzv2h_spi_priv_s *priv;
  int ret;

  priv = rzv2h_spi_get_channel(channel, SPI_MODE_SLAVE);
  if (priv == NULL)
    {
      return NULL;
    }

  ret = rzv2h_spi_initialize(priv);
  if (ret < 0)
    {
      return NULL;
    }

  return &priv->slave.ctrlr;
}
#endif

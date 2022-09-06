/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_rspi.c
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
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "up_internal.h"
#include "chip.h"
#include "rx65n_definitions.h"
#include "rx65n_rspi.h"
#include "rx65n_dtc.h"

#include <arch/board/board.h>
#include <arch/board/rx65n_gpio.h>

#if defined(CONFIG_RX65N_RSPI0) || defined(CONFIG_RX65N_RSPI1) || defined(CONFIG_RX65N_RSPI2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default bit rate */

#define RSPI_BRDV_DEFAULT 0

/* The number of words that will fit in the Tx FIFO */

#define RX65N_TXFIFO_WORDS BUFSIZE_4FRAME

/* Interrupt priority */

#define RX65N_RSPI_INTRRUPT_PRIO 15

/* RSPI power on and off */

#define RSPI_POWER_ON 0
#define RSPI_POWER_OFF 1

/* RSPI Register protection enable and disable */

#define REG_PROTECTION_ON 1
#define REG_PROTECTION_OFF 0

/* Frequency divisor */

enum FREQ_DIVISOR
{
  FREQ_DIVISOR_2 = 2,
  FREQ_DIVISOR_4 = 4,
  FREQ_DIVISOR_6 = 6,
  FREQ_DIVISOR_8 = 8,
  FREQ_DIVISOR_10 = 10,
  FREQ_DIVISOR_12 = 12,
  FREQ_DIVISOR_24 = 24,
  FREQ_DIVISOR_48 = 48,
  FREQ_DIVISOR_96 = 96,
};

/* The number of words that will fit in the buffer */

enum BUF_SIZE
{
  BUFSIZE_1FRAME = 1,
  BUFSIZE_2FRAME = 2,
  BUFSIZE_3FRAME = 3,
  BUFSIZE_4FRAME = 4,
};

/* RSPI events */

typedef enum
{
  RSPI_EVT_TRANSFER_COMPLETE, /* The data transfer completed */
  RSPI_EVT_TRANSFER_ABORTED,  /* The data transfer was aborted */
  RSPI_EVT_ERR_MODE_FAULT,    /* Mode fault error */
  RSPI_EVT_ERR_READ_OVF,      /* Read overflow */
  RSPI_EVT_ERR_PARITY,        /* Parity error */
  RSPI_EVT_ERR_UNDER_RUN,     /* Under run error */
  RSPI_EVT_ERR_UNDEF          /* Undefined/unknown error event */
} rspi_event_t;

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rx65n_rspidev_s
{
  struct spi_dev_s  rspidev;  /* Externally visible part of the RSPI interface */
  uint32_t  rspibase;         /* RSPIn base address */
  uint32_t  rspiclock;        /* Clocking for the RSPI module */

#ifndef CONFIG_SPI_POLLWAIT
  uint8_t rspitxirq;  /* RSPI tx IRQ number */
  uint8_t rspirxirq;  /* RSPI rx IRQ number */
  uint8_t rspierirq;  /* RSPI error IRQ number */
  uint8_t rspiidlirq; /* RSPI idle IRQ number */
  uint32_t rspigrpbase;
  uint32_t rspierimask;
  uint32_t rspiidlimask;
  sem_t waitsem;      /* Wait for transfer to complete */
#endif

#ifdef  CONFIG_RX65N_RSPI_SW_DT_MODE
  /* These following are the source and destination buffers of the transfer.
   * they are retained in this structure so that they will be accessible
   * from an interrupt handler.  The actual type of the buffer is uint8_t is
   * nbits <=8 and uint16_t is nbits >8.
   */

  void *txbuffer;              /* Source buffer */
  void *rxbuffer;              /* Destination buffer */

  /* These are functions pointers that are configured to perform the
   * appropriate transfer for the particular kind of exchange that is
   * occurring.  Different functions may be selected depending on (1)
   * if the tx or txbuffer is NULL and depending on the number of bits
   * per word.
   */

  void (*txword)(struct rx65n_rspidev_s *priv);
  void (*rxword)(struct rx65n_rspidev_s *priv);
#else
  DTC_HANDLE dtchandle;        /* DTC channel handle for data transfers */
  dtc_transfer_data_t *p_txdt; /* Pointer Tx data transfer */
  dtc_transfer_data_t *p_rxdt; /* Pointer Rx data transfer */
  sem_t txsem;
  sem_t rxsem;
  uint8_t txvec; /* DTC activation source of RSPI Tx */
  uint8_t rxvec; /* DTC activation source of RSPI Rx */
#endif

  bool initialized;   /* Has RSPI interface been initialized */
  mutex_t lock;       /* Held while chip is selected for mutual exclusion */
  uint32_t frequency; /* Requested clock frequency */
  uint32_t actual;    /* Actual clock frequency */

  int ntxwords;      /* Number of words left to transfer on the Tx Buffer */
  int nrxwords;      /* Number of words received on the Rx Buffer */
  int nwords;        /* Number of words to be exchanged */

  uint8_t nbits;     /* Width of word in bits (4 through 16) */
  uint8_t mode;      /* Mode 0,1,2,3 */
  uint8_t bufsize;   /* Buf size: 1,2,3 or 4 word */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint32_t rspi_getreg32(FAR struct rx65n_rspidev_s *priv,
                                     uint8_t offset);
static inline uint16_t rspi_getreg16(FAR struct rx65n_rspidev_s *priv,
                                     uint8_t offset);
static inline uint8_t rspi_getreg8(FAR struct rx65n_rspidev_s *priv,
                                   uint8_t offset);
static inline void rspi_putreg32(FAR struct rx65n_rspidev_s *priv,
                                 uint8_t offset, uint32_t value);
static inline void rspi_putreg16(FAR struct rx65n_rspidev_s *priv,
                                 uint8_t offset, uint16_t value);
static inline void rspi_putreg8(FAR struct rx65n_rspidev_s *priv,
                                uint8_t offset, uint8_t value);

/* SPI data transfer */

#if defined(CONFIG_RX65N_RSPI_SW_DT_MODE)
static void rspi_txnull(struct rx65n_rspidev_s *priv);
static void rspi_txuint32(struct rx65n_rspidev_s *priv);
static void rspi_txuint16(struct rx65n_rspidev_s *priv);
static void rspi_txuint8(struct rx65n_rspidev_s *priv);
static void rspi_rxnull(struct rx65n_rspidev_s *priv);
static void rspi_rxuint32(struct rx65n_rspidev_s *priv);
static void rspi_rxuint16(struct rx65n_rspidev_s *priv);
static void rspi_rxuint8(struct rx65n_rspidev_s *priv);
static void rspi_performtx(struct rx65n_rspidev_s *priv);
static inline void rspi_performrx(struct rx65n_rspidev_s *priv);
#endif
static int rspi_transfer(struct rx65n_rspidev_s *priv, const void *txbuffer,
                           void *rxbuffer, unsigned int nwords);

/* Interrupt handling */

#ifndef CONFIG_SPI_POLLWAIT
static inline struct rx65n_rspidev_s *rspi_mapirq(int irq);
static int rspi_idlinterrupt(int irq, void *context, FAR void *arg);
static int rspi_erinterrupt(int irq, void *context, FAR void *arg);
static int rspi_txinterrupt(int irq, void *context, FAR void *arg);
static int rspi_rxinterrupt(int irq, void *context, FAR void *arg);
#endif

/* SPI methods */

static int   rspi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t rspi_setfrequency(FAR struct spi_dev_s *dev,
                                  uint32_t frequency);
static void  rspi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void  rspi_setbits(FAR struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int  rspi_hwfeatures(FAR struct spi_dev_s *dev,
                                  spi_hwfeatures_t features);
#endif
static uint32_t rspi_send(FAR struct spi_dev_s *dev, uint32_t wd);
static void rspi_exchange(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer,
                                FAR void *rxbuffer, size_t nwords);
#ifdef CONFIG_SPI_TRIGGER
static int  rspi_trigger(FAR struct spi_dev_s *dev);
#endif
#ifndef CONFIG_SPI_EXCHANGE
static void rspi_sndblock(FAR struct spi_dev_s *dev,
                            FAR const void *txbuffer,
                                size_t nwords);
static void  rspi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
                                 size_t nwords);
#endif

/* Initialization */

static void  rspi_bus_initialize(FAR struct rx65n_rspidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RX65N_RSPI0
#if defined(CONFIG_RX65N_RSPI_DTC_DT_MODE)
dtc_transfer_data_t g_tx0dt;
dtc_transfer_data_t g_rx0dt;
#endif
static const struct spi_ops_s g_rspi0ops =
{
  .lock = rspi_lock,
  .select = rx65n_rspi0select,
  .setfrequency = rspi_setfrequency,
  .setmode = rspi_setmode,
  .setbits = rspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures = rspi_hwfeatures,
#endif
  .status = rx65n_rspi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata = rx65n_rspi0cmddata,
#endif
  .send = rspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange = rspi_exchange,
#else
  .sndblock = rspi_sndblock,
  .recvblock = rspi_recvblock,
#endif

#ifdef CONFIG_SPI_CALLBACK
  .registercallback = rx65n_rspi0register,  /* Provided externally */
#else
  .registercallback = 0,
#endif
};

static struct rx65n_rspidev_s g_rspi0dev =
{
  .rspidev   =
    {
      &g_rspi0ops
    },
  .rspibase  = RX65N_RSPI0_BASE,
  .rspiclock = RX65N_PCLK_FREQUENCY,
#ifndef CONFIG_SPI_POLLWAIT
  .rspitxirq  = RX65N_SPTI0_IRQ,
  .rspirxirq  = RX65N_SPRI0_IRQ,
  .rspierirq  = RX65N_SPEI0_IRQ,
  .rspiidlirq  = RX65N_SPII0_IRQ,
  .rspigrpbase = RX65N_GRPAL0_ADDR,
  .rspierimask = RX65N_GRPAL0_SPEI0_MASK,
  .rspiidlimask = RX65N_GRPAL0_SPII0_MASK,
  .waitsem = SEM_INITIALIZER(0),
#endif
#if defined(CONFIG_RX65N_RSPI_DTC_DT_MODE)
  .p_txdt  = &g_tx0dt,
  .p_rxdt  = &g_rx0dt,
  .txsem = SEM_INITIALIZER(0),
  .rxsem = SEM_INITIALIZER(0),
  .txvec = RX65N_RSPI0_TXVECT,
  .rxvec = RX65N_RSPI0_RXVECT,
#endif
  .lock = NXMUTEX_INITIALIZER,
};
#endif

#ifdef CONFIG_RX65N_RSPI1
#if defined(CONFIG_RX65N_RSPI_DTC_DT_MODE)
dtc_transfer_data_t g_tx1dt;
dtc_transfer_data_t g_rx1dt;
#endif
static const struct spi_ops_s g_rspi1ops =
{
  .lock = rspi_lock,
  .select = rx65n_rspi1select,
  .setfrequency = rspi_setfrequency,
  .setmode = rspi_setmode,
  .setbits = rspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures = rspi_hwfeatures,
#endif
  .status = rx65n_rspi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata = rx65n_rspi1cmddata,
#endif
  .send = rspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange = rspi_exchange,
#else
  .sndblock = rspi_sndblock,
  .recvblock = rspi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger = rspi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = rx65n_rspi1register,  /* Provided externally */
#else
  .registercallback = 0,
#endif
};

static struct rx65n_rspidev_s g_rspi1dev =
{
  .rspidev   =
    {
      &g_rspi1ops
    },
  .rspibase  = RX65N_RSPI1_BASE,
  .rspiclock = RX65N_PCLK_FREQUENCY,
#ifndef CONFIG_SPI_POLLWAIT
  .rspitxirq  = RX65N_SPTI1_IRQ,
  .rspirxirq  = RX65N_SPRI1_IRQ,
  .rspierirq  = RX65N_SPEI1_IRQ,
  .rspiidlirq  = RX65N_SPII1_IRQ,
  .rspigrpbase = RX65N_GRPAL0_ADDR,
  .rspierimask = RX65N_GRPAL0_SPEI1_MASK,
  .rspiidlimask = RX65N_GRPAL0_SPII1_MASK,
  .waitsem = SEM_INITIALIZER(0),
#endif
#if defined(CONFIG_RX65N_RSPI_DTC_DT_MODE)
  .p_txdt = &g_tx1dt,
  .p_rxdt = &g_rx1dt,
  .txsem = SEM_INITIALIZER(0),
  .rxsem = SEM_INITIALIZER(0),
  .txvec = RX65N_RSPI1_TXVECT,
  .rxvec = RX65N_RSPI1_RXVECT,
#endif
  .lock = NXMUTEX_INITIALIZER,
};
#endif

#ifdef CONFIG_RX65N_RSPI2
#if defined(CONFIG_RX65N_RSPI_DTC_DT_MODE)
dtc_transfer_data_t g_tx2dt;
dtc_transfer_data_t g_rx2dt;
#endif
static const struct spi_ops_s g_rspi2ops =
{
  .lock = rspi_lock,
  .select = rx65n_rspi2select,
  .setfrequency = rspi_setfrequency,
  .setmode = rspi_setmode,
  .setbits = rspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures = rspi_hwfeatures,
#endif
  .status = rx65n_rspi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata = rx65n_rspi2cmddata,
#endif
  .send = rspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange = rspi_exchange,
#else
  .sndblock = rspi_sndblock,
  .recvblock = rspi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger = rspi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = rx65n_rspi2register,  /* Provided externally */
#else
  .registercallback = 0,
#endif
};

static struct rx65n_rspidev_s g_rspi2dev =
{
  .rspidev   =
    {
      &g_rspi2ops
    },
  .rspibase  = RX65N_RSPI2_BASE,
  .rspiclock = RX65N_PCLK_FREQUENCY,
#ifndef CONFIG_SPI_POLLWAIT
  .rspitxirq  = RX65N_SPTI2_IRQ,
  .rspirxirq  = RX65N_SPRI2_IRQ,
  .rspierirq  = RX65N_SPEI2_IRQ,
  .rspiidlirq  = RX65N_SPII2_IRQ,
  .rspigrpbase = RX65N_GRPAL0_ADDR,
  .rspierimask = RX65N_GRPAL0_SPEI2_MASK,
  .rspiidlimask = RX65N_GRPAL0_SPII2_MASK,
  .waitsem = SEM_INITIALIZER(0),
#endif
#if defined(CONFIG_RX65N_RSPI_DTC_DT_MODE)
  .p_txdt = &g_tx2dt,
  .p_rxdt = &g_rx2dt,
  .txsem = SEM_INITIALIZER(0),
  .rxsem = SEM_INITIALIZER(0),
  .txvec = RX65N_RSPI2_TXVECT,
  .rxvec = RX65N_RSPI2_RXVECT,
#endif
  .lock = NXMUTEX_INITIALIZER,
};
#endif

#if defined(CONFIG_RX65N_RSPI_DTC_DT_MODE)
dtc_static_transfer_data_cfg_t tx_cfg =
{
#if CONFIG_RX65N_RSPI_BUF_SIZE > 1
  .transfer_mode = DTC_TRANSFER_MODE_BLOCK,
#else
  .transfer_mode = DTC_TRANSFER_MODE_NORMAL,
#endif
  .data_size = DTC_DATA_SIZE_BYTE,
  .src_addr_mode = DTC_SRC_ADDR_INCR,
  .chain_transfer_enable = DTC_CHAIN_TRANSFER_DISABLE,
  .chain_transfer_mode = DTC_CHAIN_TRANSFER_CONTINUOUSLY,
  .response_interrupt = DTC_INTERRUPT_AFTER_ALL_COMPLETE,
  .repeat_block_side = DTC_REPEAT_BLOCK_DESTINATION,
  .dest_addr_mode = DTC_DES_ADDR_FIXED,
  .source_addr = 0,    /* This will set dynamically */
  .dest_addr = 0,      /* Set data register address */
  .transfer_count = 0, /* This will set dynamically */
#if CONFIG_RX65N_RSPI_BUF_SIZE > 1
  .block_size = CONFIG_RX65N_RSPI_BUF_SIZE, /* Looks like tx fifo size */
#else
  .block_size = 0,
#endif
  .rsv = 0,
  .writeback_disable = DTC_WRITEBACK_ENABLE,
  .sequence_end = DTC_SEQUENCE_TRANSFER_CONTINUE,
  .refer_index_table_enable = DTC_REFER_INDEX_TABLE_DISABLE,
  .disp_add_enable = DTC_SRC_ADDR_DISP_ADD_DISABLE,
};

dtc_static_transfer_data_cfg_t rx_cfg =
{
#if CONFIG_RX65N_RSPI_BUF_SIZE > 1
  .transfer_mode = DTC_TRANSFER_MODE_BLOCK,
#else
  .transfer_mode = DTC_TRANSFER_MODE_NORMAL,
#endif
  .data_size       = DTC_DATA_SIZE_BYTE,
  .src_addr_mode = DTC_SRC_ADDR_FIXED,
  .chain_transfer_enable = DTC_CHAIN_TRANSFER_DISABLE,
  .chain_transfer_mode = DTC_CHAIN_TRANSFER_CONTINUOUSLY,
  .response_interrupt = DTC_INTERRUPT_AFTER_ALL_COMPLETE,
  .repeat_block_side = DTC_REPEAT_BLOCK_SOURCE,
  .dest_addr_mode = DTC_DES_ADDR_INCR,
  .source_addr = 0,            /* Set data register address */
  .dest_addr = 0,              /* This will set dynamically */
  .transfer_count = 0,         /* This will set dynamically */
#if CONFIG_RX65N_RSPI_BUF_SIZE > 1
  .block_size = CONFIG_RX65N_RSPI_BUF_SIZE, /* Looks like tx fifo size */
#else
  .block_size = 0,
#endif
  .rsv = 0,
  .writeback_disable = DTC_WRITEBACK_ENABLE,
  .sequence_end = DTC_SEQUENCE_TRANSFER_CONTINUE,
  .refer_index_table_enable = DTC_REFER_INDEX_TABLE_DISABLE,
  .disp_add_enable = DTC_SRC_ADDR_DISP_ADD_DISABLE,
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  rx65n_rspi0/1/2 select and rx65n_rspi0/1/2 status
 *
 * Description:
 * The external functions, rx65n_rspi0/1/2 select, rx65n_rspi0/1/2 status,and
 * rx65n_rspi0/1/2 cmddata must be provided by board-specific logic. These
 * are implementations of the select, status, and cmddata methods of the SPI
 * interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h). All
 * other methods (including rx65n_rspibus_initialize()) are provided by
 * common R656N logic.  To use thiscommon SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/...select() and stm32_spi1/2/...status()
 *      functions in your board-specific logic.  These functions will
 *      perform chip selection and status operations using GPIOs in the
 *      way your board is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration file,
 *      then provide stm32_spi1/2/...cmddata() functions in your
 *      board-specific logic.These functions will perform cmd/data
 *      selection operations using GPIOs in the way your board is configured.
 *   4. Add a calls to stm32_spibus_initialize() in your low level
 *      application initialization logic
 *   5. The handle returned by stm32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/
#ifdef CONFIG_RX65N_RSPI0
void rx65n_rspi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" :
                                                   "de-assert");
}

uint8_t rx65n_rspi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

int rx65n_rspi0cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_RX65N_RSPI1
void rx65n_rspi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" :
                                                         "de-assert");
}

uint8_t rx65n_rspi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

int rx65n_rspi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_RX65N_RSPI2
void rx65n_rspi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" :
                                                          "de-assert");
}

uint8_t rx65n_rspi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

int rx65n_rspi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

/****************************************************************************
 * Name: rx65n_rspi0/1/2register
 *
 * Description:
 * If the board supports a card detect callback to inform the SPI-based
 * MMC/SD driver when an SD card is inserted or removed, then
 * CONFIG_SPI_CALLBACK should be defined and the following function(s)
 * must be implemented. These functions implements the registercallback
 * method of the SPI interface (see include/nuttx/spi/spi.h for details)
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The function to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/
#ifdef CONFIG_SPI_CALLBACK
#ifdef CONFIG_RX65N_RSPI0
int rx65n_rspi0register(FAR struct spi_dev_s *dev,
                        spi_mediachange_t callback,
                       FAR void *arg)
{
  spiinfo("INFO: Registering rspi0 device\n");
  return OK;
}
#endif

#ifdef CONFIG_RX65N_RSPI1
int rx65n_rspi1register(FAR struct spi_dev_s *dev,
                        spi_mediachange_t callback,
                       FAR void *arg)
{
  spiinfo("INFO: Registering rspi1 device\n");
  return OK;
}
#endif

#ifdef CONFIG_RX65N_RSPI2
int rx65n_rspi2register(FAR struct spi_dev_s *dev,
                         spi_mediachange_t callback,
                       FAR void *arg)
{
  spiinfo("INFO: Registering rspi2 device\n");
  return OK;
}
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rspi_getreg32
 *
 * Description:
 *   Read the RSPI register at this offeset
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   offset - Offset to the SPI register from the register base address
 *
 * Returned Value:
 *   Value of the register at this offset
 *
 ****************************************************************************/

static inline uint32_t rspi_getreg32(struct rx65n_rspidev_s *priv,
                                     uint8_t offset)
{
  return getreg32(priv->rspibase + offset);
}

/****************************************************************************
 * Name: rspi_getreg16
 *
 * Description:
 *   Read the RSPI register at this offeset
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   offset - Offset to the SPI register from the register base address
 *
 * Returned Value:
 *   Value of the register at this offset
 *
 ****************************************************************************/

static inline uint16_t rspi_getreg16(struct rx65n_rspidev_s *priv,
                                     uint8_t offset)
{
  return getreg16(priv->rspibase + offset);
}

/****************************************************************************
 * Name: rspi_getreg8
 *
 * Description:
 *   Read the RSPI register at this offeset
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   offset - Offset to the SPI register from the register base address
 *
 * Returned Value:
 *   Value of the register at this offset
 *
 ****************************************************************************/

static inline uint8_t rspi_getreg8(struct rx65n_rspidev_s *priv,
                                   uint8_t offset)
{
  return getreg8(priv->rspibase + offset);
}

/****************************************************************************
 * Name: rspi_putreg32
 *
 * Description:
 *   Write the value to the RSPI register at this offeset
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   offset - Offset to the SPI register from the register base address
 *   value  - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rspi_putreg32(struct rx65n_rspidev_s *priv,
                              uint8_t offset, uint32_t value)
{
  putreg32(value, priv->rspibase + offset);
}

/****************************************************************************
 * Name: rspi_putreg16
 *
 * Description:
 *   Write the value to the RSPI register at this offeset
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   offset - Offset to the SPI register from the register base address
 *   value  - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rspi_putreg16(struct rx65n_rspidev_s *priv,
                              uint8_t offset, uint16_t value)
{
  putreg16(value, priv->rspibase + offset);
}

/****************************************************************************
 * Name: rspi_putreg8
 *
 * Description:
 *   Write the value to the RSPI register at this offeset
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   offset - Offset to the SPI register from the register base address
 *   value  - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rspi_putreg8(struct rx65n_rspidev_s *priv,
                              uint8_t offset, uint8_t value)
{
  putreg8(value, priv->rspibase + offset);
}

/****************************************************************************
 * Name: rspi_txnull, rspi_txuint16, and rspi_txuint8
 *
 * Description:
 *   Transfer all ones, a uint8_t, or uint16_t to Tx FIFO and update
 *   the txbuffer pointer appropriately.  The selected function depends
 *   on (1) if there is a source txbuffer provided, and (2) if the
 *   number of bits per word is <=8 or >8.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_RX65N_RSPI_SW_DT_MODE)
static void rspi_txnull(struct rx65n_rspidev_s *priv)
{
  rspi_putreg32(priv, RX65N_RSPI_SPDR_OFFSET, 0xffff);
}

static void rspi_txuint32(struct rx65n_rspidev_s *priv)
{
  uint32_t *ptr = (uint32_t *)priv->txbuffer;
  rspi_putreg32(priv, RX65N_RSPI_SPDR_OFFSET, *ptr++);
  priv->txbuffer = (void *)ptr;
}

static void rspi_txuint16(struct rx65n_rspidev_s *priv)
{
  uint16_t *ptr = (uint16_t *)priv->txbuffer;
  rspi_putreg16(priv, RX65N_RSPI_SPDR_OFFSET, *ptr++);
  priv->txbuffer = (void *)ptr;
}

static void rspi_txuint8(struct rx65n_rspidev_s *priv)
{
  uint8_t *ptr = (uint8_t *)priv->txbuffer;
  rspi_putreg8(priv, RX65N_RSPI_SPDR_OFFSET, *ptr++);
  priv->txbuffer = (void *)ptr;
}
#endif

/****************************************************************************
 * Name: rspi_rxnull,rspi_rxuint16, and rspi_rxuint8
 *
 * Description:
 *   Discard input, save a uint8_t, or or save a uint16_t from Tx FIFO in the
 *   user rxvbuffer and update the rxbuffer pointer appropriately.  The
 *   selected function dependes on (1) if there is a destination rxbuffer
 *   provided, and (2) if the number of bits per word is <=8 or >8.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_RX65N_RSPI_SW_DT_MODE)
static void rspi_rxnull(struct rx65n_rspidev_s *priv)
{
  rspi_getreg32(priv, RX65N_RSPI_SPDR_OFFSET);
}

static void rspi_rxuint32(struct rx65n_rspidev_s *priv)
{
  uint32_t *ptr = (uint32_t *)priv->rxbuffer;
  *ptr++ = (uint32_t)rspi_getreg32(priv, RX65N_RSPI_SPDR_OFFSET);
  priv->rxbuffer = (void *)ptr;
}

static void rspi_rxuint16(struct rx65n_rspidev_s *priv)
{
  uint16_t *ptr = (uint16_t *)priv->rxbuffer;
  *ptr++ = (uint16_t)rspi_getreg16(priv, RX65N_RSPI_SPDR_OFFSET);
  priv->rxbuffer = (void *)ptr;
}

static void rspi_rxuint8(struct rx65n_rspidev_s *priv)
{
  uint8_t *ptr = (uint8_t *)priv->rxbuffer;
  *ptr++ = (uint8_t)rspi_getreg8(priv, RX65N_RSPI_SPDR_OFFSET);
  priv->rxbuffer = (void *)ptr;
}
#endif

/****************************************************************************
 * Name: rspi_performtx
 *
 * Description:
 *   If the Tx FIFO is empty, then transfer as many words as we can to
 *   the FIFO.
 *
 * Input Parameters:
 *
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_RX65N_RSPI_SW_DT_MODE)
static void rspi_performtx(struct rx65n_rspidev_s *priv)
{
  uint8_t regval8;
  int ntxd = 0;  /* Number of words written to Tx FIFO */

  /* Check if the Tx FIFO is empty */

  if ((rspi_getreg8(priv, RX65N_RSPI_SPSR_OFFSET) & RSPI_SPSR_SPTEF) != 0)
    {
      /* Check if all of the Tx words have been sent */

      if (priv->ntxwords > 0)
        {
          /* No..
           * Transfer more words until either the TxFIFO is full or
           * until all of the user provided data has been sent.
           */

          for (; ntxd < priv->ntxwords && ntxd < priv->bufsize; ntxd++)
            {
              priv->txword(priv);
            }

          /* Update the count of words to be transferred */

          priv->ntxwords -= ntxd;

          /* Add dummy data in FIFO to make it full */

          if (ntxd < priv->bufsize)
            {
              while (ntxd != priv->bufsize)
                {
                  rspi_txnull(priv);
                  ntxd++;
                }
            }
        }
      else
        {
          /* Yes.. The transfer is complete,
           * disable Tx FIFO empty interrupt
           */

          regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR_OFFSET);
          regval8 &= ~RSPI_SPCR_SPTIE;
          rspi_putreg8(priv, RX65N_RSPI_SPCR_OFFSET, regval8);
        }
    }
}
#endif

/****************************************************************************
 * Name: rspi_performrx
 *
 * Description:
 *   Transfer as many bytes as possible from the Rx FIFO to the user Rx
 *   buffer (if one was provided).
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_RX65N_RSPI_SW_DT_MODE)
static inline void rspi_performrx(struct rx65n_rspidev_s *priv)
{
  /* Loop while data is available in the Rx FIFO (SPRXn) */

  while ((rspi_getreg8(priv, RX65N_RSPI_SPSR_OFFSET) & RSPI_SPSR_SPRF) != 0)
    {
      /* Have all of the requested words been transferred from the Rx FIFO? */

      if (priv->nrxwords < priv->nwords)
        {
          /* No.. Read more data from Rx FIFO */

          priv->rxword(priv);
          priv->nrxwords++;
        }
      else
        {
          rspi_rxnull(priv); /* Reading dummy data send */
        }
    }
}
#endif

/****************************************************************************
 * Name: rspi_startxfr
 *
 * Description:
 *   If data was added to the Tx FIFO, then start the exchange
 *
 * Input Parameters:
 *   priv  - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rspi_startxfr(struct rx65n_rspidev_s *priv)
{
  uint8_t regval8;

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR_OFFSET);
  regval8 |= RSPI_SPCR_SPE;
  rspi_putreg8(priv, RX65N_RSPI_SPCR_OFFSET, regval8);
}

/****************************************************************************
 * Name: spi_dtcrxsetup
 *
 * Description:
 *   Setup to perform RX DTC
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_RSPI_DTC_DT_MODE
static dtc_err_t rspi_dtctxsetup(FAR struct rx65n_rspidev_s *priv,
                           FAR const void *txbuffer, FAR const void *txdummy,
                           size_t nwords)
{
  dtc_err_t  ret = DTC_SUCCESS;
  dtc_dynamic_transfer_data_cfg_t dcfg;

  /* 8- or 16-bit mode? */

  if (priv->nbits >= 4 && priv->nbits <= 8)
    {
      if (txbuffer)
        {
          dcfg.data_size = DTC_DATA_SIZE_BYTE;
        }
      else
        {
          txbuffer    = txdummy;
          dcfg.data_size = DTC_DATA_SIZE_BYTE;
        }
    }
  else if (priv->nbits > 8 && priv->nbits <= 16)
    {
      if (txbuffer)
        {
          dcfg.data_size = DTC_DATA_SIZE_WORD;
        }
      else
        {
          txbuffer    = txdummy;
          dcfg.data_size = DTC_DATA_SIZE_WORD;
        }
    }
  else
    {
      spierr("ERROR: RSPI not supporting: %d bit word\n", priv->nbits);
    }

  dcfg.source_addr = (uint32_t)txbuffer;
  dcfg.dest_addr = priv->rspibase + RX65N_RSPI_SPDR_OFFSET;

  /* Configure block size if buf size is configured as more than 1 */

  if (CONFIG_RX65N_RSPI_BUF_SIZE - 1)
    {
      /* Configure DTC in Block transfer mode */

      dcfg.block_size = CONFIG_RX65N_RSPI_BUF_SIZE;
      dcfg.transfer_count = nwords / CONFIG_RX65N_RSPI_BUF_SIZE;
    }
  else
    {
      dcfg.transfer_count = nwords;
    }

  /* Configure the RX DTC */

  ret = rx65n_dtc_setup_dynamic_transferdata(priv->dtchandle, priv->txvec,
                                          (uint32_t)&dcfg, 0);
  if (ret < 0)
    {
      spierr("ERROR:[%d] Fail to configure DTC information for TX\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: spi_dtcrxsetup
 *
 * Description:
 *   Setup to perform RX DTC
 *
 ****************************************************************************/

static dtc_err_t rspi_dtcrxsetup(FAR struct rx65n_rspidev_s *priv,
                           FAR void *rxbuffer, FAR void *rxdummy,
                           size_t nwords)
{
  dtc_err_t  ret = DTC_SUCCESS;
  dtc_dynamic_transfer_data_cfg_t dcfg;

  /* 8- or 16-bit mode */

  if (priv->nbits >= 4 && priv->nbits <= 8)
    {
      if (rxbuffer)
        {
          dcfg.data_size = DTC_DATA_SIZE_BYTE;
        }
      else
        {
          rxbuffer    = rxdummy;
          dcfg.data_size = DTC_DATA_SIZE_BYTE;
        }
    }
  else if (priv->nbits > 8 && priv->nbits <= 16)
    {
      if (rxbuffer)
        {
          dcfg.data_size = DTC_DATA_SIZE_WORD;
        }
      else
        {
          rxbuffer    = rxdummy;
          dcfg.data_size = DTC_DATA_SIZE_WORD;
        }
    }
  else
    {
      spierr("ERROR: RSPI not supporting: %d bit word\n", nbits);
    }

  dcfg.source_addr = priv->rspibase + RX65N_RSPI_SPDR_OFFSET;
  dcfg.dest_addr = (uint32_t)rxbuffer;

  /* Configure block size if buf size is configured as more than 1 */

  if (CONFIG_RX65N_RSPI_BUF_SIZE - 1)
    {
      /* Configure DTC in Block transfer mode */

      dcfg.block_size = CONFIG_RX65N_RSPI_BUF_SIZE;
      dcfg.transfer_count = nwords / CONFIG_RX65N_RSPI_BUF_SIZE;
    }
  else
    {
      dcfg.transfer_count = nwords;
    }

  /* Configure the RX DTC */

  ret = rx65n_dtc_setup_dynamic_transferdata(priv->dtchandle,
                                  priv->rxvec, (uint32_t)&dcfg, 0);
  if (ret < 0)
    {
      spierr("ERROR:[%d] Fail to configure DTC information for RX\n", ret);
      return ret;
    }

  return ret;
}

#endif

/****************************************************************************
 * Name: rspi_transfer
 *
 * Description:
 *   Exchange a block data with the SPI device
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   txbuffer - The buffer of data to send to the device (may be NULL).
 *   rxbuffer - The buffer to receive data from the device (may be NULL).
 *   nwords   - The total number of words to be exchanged.  If the interface
 *              uses <= 8 bits per word, then this is the number of
 *              uint8_t's; if the interface uses >8 bits per word, then this
 *              is the number of uint16_t's
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 ****************************************************************************/
#ifdef CONFIG_RX65N_RSPI_SW_DT_MODE
static int rspi_transfer(struct rx65n_rspidev_s *priv, const void *txbuffer,
                        void *rxbuffer, unsigned int nwords)
{
  uint8_t regval8;
#ifndef CONFIG_SPI_POLLWAIT
  irqstate_t flags;
#endif

  /* Set up to perform the transfer */

  priv->txbuffer = (uint8_t *)txbuffer; /* Source buffer */
  priv->rxbuffer = (uint8_t *)rxbuffer; /* Destination buffer */
  priv->ntxwords = nwords;              /* Number of words left to send */
  priv->nrxwords = 0;                   /* Number of words received */
  priv->nwords = nwords;                /* Total number of exchanges */

  /* Set up the low-level data transfer function pointers */

  if (priv->nbits > 16)
    {
      priv->txword = rspi_txuint32;
      priv->rxword = rspi_rxuint32;
    }

  if (priv->nbits > 8)
    {
      priv->txword = rspi_txuint16;
      priv->rxword = rspi_rxuint16;
    }
  else
    {
      priv->txword = rspi_txuint8;
      priv->rxword = rspi_rxuint8;
    }

  if (!txbuffer)
    {
      priv->txword = rspi_txnull;
    }

  if (!rxbuffer)
    {
      priv->rxword = rspi_rxnull;
    }

  /* Copied data to start the sequence (saves one interrupt) */

#ifndef CONFIG_SPI_POLLWAIT
  flags = enter_critical_section();

  /* Enable interrupt */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR_OFFSET);
  regval8 |= (RSPI_SPCR_SPEIE | RSPI_SPCR_SPTIE | RSPI_SPCR_SPRIE);
  rspi_putreg8(priv, RX65N_RSPI_SPCR_OFFSET, regval8);

  /* Start transmission */

  rspi_performtx(priv);
  rspi_startxfr(priv);

  /* Enable Idle interrupt */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR2_OFFSET);
  regval8 |= (RSPI_SPCR2_SPIIE); /* RSPI Idle Interrupt Enable */
  rspi_putreg8(priv, RX65N_RSPI_SPCR2_OFFSET, regval8);

  leave_critical_section(flags);

  /* Wait for the transfer to complete.  Since there is no handshake
   * with SPI, the following should complete even if there are problems
   * with the transfer, so it should be safe with no timeout.
   */

  /* Wait to be signaled from the interrupt handler */

  return nxsem_wait_uninterruptible(&priv->waitsem);

#else
  /* Perform the transfer using polling logic.  This will totally
   * dominate the CPU until the transfer is complete.  Only recommended
   * if (1) your SPI is very fast, and (2) if you only use very short
   * transfers.
   */

  do
    {
      /* Handle outgoing Tx FIFO transfers */

      rspi_performtx(priv);

      /* Handle incoming Rx FIFO transfers */

      rspi_performrx(priv);

      /* Resume the transfer */

      rspi_startxfr(priv);

      /* If there are other threads at this same priority level,
       * the following may help:
       */

      sched_yield();
    }
  while (priv->nrxwords < priv->nwords);

  return OK;
#endif
}
#elif defined(CONFIG_RX65N_RSPI_DTC_DT_MODE)
static int rspi_transfer(struct rx65n_rspidev_s *priv, const void *txbuffer,
                        void *rxbuffer, unsigned int nwords)
{
  uint8_t regval8;
  static uint16_t rxdummy = 0xffff;
  static const uint16_t txdummy = 0xffff;
  irqstate_t flags;
  int ret;

  /* Disable RSPI */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR_OFFSET);
  regval8 &= (~RSPI_SPCR_SPE);
  rspi_putreg8(priv, RX65N_RSPI_SPCR_OFFSET, regval8);

  /* Setup DTC */

  ret = rspi_dtctxsetup(priv, txbuffer, &txdummy, nwords);
  if (ret > 1)
    {
      spierr("ERROR:Fail to setup DTC for TX transfer\n");
      return -EINVAL;
    }

  ret = rspi_dtcrxsetup(priv, rxbuffer, &rxdummy, nwords);
  if (ret > 1)
    {
      spierr("ERROR:[%d] Fail to setup DTC for RX transfer\n");
      return -EINVAL;
    }

  flags = enter_critical_section();

  /* Enable RSPI source for DTC trigger */

  rx65n_dtc_srcactivation(priv->dtchandle, priv->txvec);
  rx65n_dtc_srcactivation(priv->dtchandle, priv->rxvec);

  /* Enable TX, RX and Error interrupt */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR_OFFSET);
  regval8 |= (RSPI_SPCR_SPEIE | RSPI_SPCR_SPRIE | RSPI_SPCR_SPTIE);
  rspi_putreg8(priv, RX65N_RSPI_SPCR_OFFSET, regval8);

  /* Enable RSPI */

  rspi_startxfr(priv);

  /* Wait for transfer completion */

  leave_critical_section(flags);

  /* Wait to be signaled from the interrupt handler */

  return nxsem_wait_uninterruptible(&priv->waitsem);
}

#endif

/****************************************************************************
 * Name: rspi_mapirq
 *
 * Description:
 *   Map an IRQ number into the appropriate SPI device
 *
 * Input Parameters:
 *   irq   - The IRQ number to be mapped
 *
 * Returned Value:
 *   On success, a reference to the private data structgure for this IRQ.
 *   NULL on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_POLLWAIT
static inline struct rx65n_rspidev_s *rspi_mapirq(int irq)
{
  switch (irq)
    {
#ifdef CONFIG_RX65N_RSPI0
      case RX65N_SPRI0_IRQ:
      case RX65N_SPTI0_IRQ:
      case RX65N_SPEI0_IRQ:
      case RX65N_SPII0_IRQ:
        return &g_rspi0dev;
#endif
#ifdef CONFIG_RX65N_RSPI1
      case RX65N_SPRI1_IRQ:
      case RX65N_SPTI1_IRQ:
      case RX65N_SPEI1_IRQ:
      case RX65N_SPII1_IRQ:
        return &g_rspi1dev;
#endif
#ifdef CONFIG_RX65N_RSPI2
      case RX65N_SPRI2_IRQ:
      case RX65N_SPTI2_IRQ:
      case RX65N_SPEI2_IRQ:
      case RX65N_SPII2_IRQ:
        return &g_rspi2dev;
#endif
      default:
        return NULL;
    }
}
#endif

/****************************************************************************
 * Name: rspi_errhandle
 *
 * Description:
 *  RSPI common error handler
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   bus   - channel number
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_POLLWAIT
static void rspi_errhandle(struct rx65n_rspidev_s *priv, uint8_t bus)
{
  uint8_t regval8;
  rspi_event_t event = RSPI_EVT_ERR_UNDEF;

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPSR_OFFSET);

  /* Identify and clear error condition . */

  if (regval8 & RSPI_SPSR_OVRF)
    {
      /* Overrun error */

      if (RSPI_EVT_ERR_UNDEF == event)
        {
          event = RSPI_EVT_ERR_READ_OVF;
        }

      /* Clear error source: OVRF flag. */

      regval8 &= (~RSPI_SPSR_OVRF);
      rspi_putreg8(priv, RX65N_RSPI_SPSR_OFFSET, regval8);
    }

  if (regval8 & RSPI_SPSR_MODF)
    {
      if (regval8 & RSPI_SPSR_UDRF)
        {
          if (RSPI_EVT_ERR_UNDEF == event)
            {
                event = RSPI_EVT_ERR_UNDER_RUN;
            }

          /* Clear error source : MODF flag and UDRF */

          regval8 &= RSPI_SPSR_MODF_UDRF_MASK;
          rspi_putreg8(priv, RX65N_RSPI_SPSR_OFFSET, regval8);
        }
      else
        {
          if (RSPI_EVT_ERR_UNDEF == event)
            {
                event = RSPI_EVT_ERR_MODE_FAULT;
            }

          /* Clear error source : MODF flag */

          regval8 &= (~RSPI_SPSR_MODF);
          rspi_putreg8(priv, RX65N_RSPI_SPSR_OFFSET, regval8);
        }
    }

  if (regval8 & RSPI_SPSR_PERF)
    {
      if (RSPI_EVT_ERR_UNDEF == event)
        {
           event = RSPI_EVT_ERR_PARITY;
        }

      /* Clear error source: PERF flag */

      regval8 &= (~RSPI_SPSR_PERF);
      rspi_putreg8(priv, RX65N_RSPI_SPSR_OFFSET, regval8);
    }

  /* Disable the RSPI operation . */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR_OFFSET);
  regval8 &= (~(RSPI_SPCR_SPTIE | RSPI_SPCR_SPRIE | RSPI_SPCR_SPE));
  rspi_putreg8(priv, RX65N_RSPI_SPCR_OFFSET, regval8);

  /* Yes, wake up the waiting thread . */

  nxsem_post(&priv->waitsem);
}
#endif

/****************************************************************************
 * Name: rspi_idlinterrupt
 *
 * Description:
 *  RSPI IDLE interrupt handler invoke on transmission/reception completion
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_POLLWAIT
static int rspi_idlinterrupt(int irq, void *context, FAR void *arg)
{
  uint8_t regval8;

  struct rx65n_rspidev_s *priv = rspi_mapirq(irq);

  DEBUGASSERT(priv != NULL);

  /* Disable Idle interrupt in SPCR2 */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR2_OFFSET);
  regval8 &= ~(RSPI_SPCR2_SPIIE); /* RSPI Idle Interrupt Enable */
  rspi_putreg8(priv, RX65N_RSPI_SPCR2_OFFSET, regval8);

  /* Disable ransmit/receive interrupt */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR_OFFSET);
  regval8 &= ~(RSPI_SPCR_SPRIE | RSPI_SPCR_SPTIE);
  rspi_putreg8(priv, RX65N_RSPI_SPCR_OFFSET, regval8);

  /* Yes, wake up the waiting thread */

  nxsem_post(&priv->waitsem);

  return OK;
}
#endif

/****************************************************************************
 * Name: rspi_erinterrupt
 *
 * Description:
 *   RSPI common error interrupt handler
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_POLLWAIT
static int rspi_erinterrupt(int irq, void *context, FAR void *arg)
{
  struct rx65n_rspidev_s *priv = rspi_mapirq(irq);

  DEBUGASSERT(priv != NULL);

#ifdef CONFIG_RX65N_RSPI0
  rspi_errhandle(priv, RX65N_RSPI_CHANNEL0);
#endif

#ifdef CONFIG_RX65N_RSPI1
  rspi_errhandle(priv, RX65N_RSPI_CHANNEL1);
#endif

#ifdef CONFIG_RX65N_RSPI2
  rspi_errhandle(priv, RX65N_RSPI_CHANNEL2);
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: rspi_rxinterrupt
 *
 * Description:
 *   Receive buffer full interrupt handler to process a block data from SPI
 *   device
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   rxbuffer - The buffer to receive data from the device (may be NULL).
 *   nwords   - The total number of words to be exchanged.  If the interface
 *              uses <= 8 bits per word, then this is the number of
 *              uint8_t's; if the interface uses >8 bits per word, then this
 *              is the number of uint16_t's
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_POLLWAIT
static int rspi_rxinterrupt(int irq, void *context, FAR void *arg)
{
#ifdef CONFIG_RX65N_RSPI_DTC_DT_MODE
  uint8_t regval8;
#endif

  struct rx65n_rspidev_s *priv = rspi_mapirq(irq);

  DEBUGASSERT(priv != NULL);

#ifdef CONFIG_RX65N_RSPI_SW_DT_MODE
  /* Handle incoming Rx FIFO transfers */

  rspi_performrx(priv);
#else

  /* Disable ransmit/receive interrupt */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR_OFFSET);
  regval8 &= ~(RSPI_SPCR_SPRIE | RSPI_SPCR_SPTIE);
  rspi_putreg8(priv, RX65N_RSPI_SPCR_OFFSET, regval8);

  /* Yes, wake up the waiting thread */

  nxsem_post(&priv->waitsem);

#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: rspi_txinterrupt
 *
 * Description:
 *   Transmit buffer empty interrupt handler to process a block data for SPI
 *   device
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   txbuffer - The buffer of data to send to the device (may be NULL).
 *   nwords   - The total number of words to be exchanged.  If the interface
 *              uses <= 8 bits per word, then this is the number of
 *              uint8_t's; if the interface uses >8 bits per word, then this
 *              is the number of uint16_t's
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_POLLWAIT
static int rspi_txinterrupt(int irq, void *context, FAR void *arg)
{
  struct rx65n_rspidev_s *priv = rspi_mapirq(irq);

#ifdef CONFIG_RX65N_RSPI_DTC_DT_MODE
  uint8_t regval8;
#endif

  DEBUGASSERT(priv != NULL);

#ifdef CONFIG_RX65N_RSPI_SW_DT_MODE
  /* Handle outgoing Tx FIFO transfers */

  rspi_performtx(priv);

#else

  /* DTC transfer completion */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR_OFFSET);
  if ((regval8 & RSPI_SPCR_TXMD) == 1)
    {
      regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR_OFFSET);
      regval8 &= ~(RSPI_SPCR_SPRIE | RSPI_SPCR_SPTIE);
      rspi_putreg8(priv, RX65N_RSPI_SPCR_OFFSET, regval8);

      /* Yes, wake up the waiting thread */

      nxsem_post(&priv->waitsem);
    }

#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: rspi_lock
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 ****************************************************************************/

static int rspi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  struct rx65n_rspidev_s *priv = (struct rx65n_rspidev_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxmutex_lock(&priv->lock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: rspi_setfrequency
 *
 * Description:
 *   Set the RSPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t rspi_setfrequency(FAR struct spi_dev_s *dev,
                                 uint32_t frequency)
{
  struct rx65n_rspidev_s *priv = (struct rx65n_rspidev_s *)dev;
  uint32_t actual;

  DEBUGASSERT(priv);
  actual = priv->actual;

  if (frequency != priv->frequency)
    {
      /* Below formula used to calculate bit rate
       * Bit rate f(PCLK)/(2 * (n + 1) * 2^N) .
       * n denotes SPBR setting (0,1,2..255) .
       * N denotes a BRDV[1:0] bit setting (0, 1, 2, 3) .
       */

      uint16_t regval16;
      uint32_t brdv;
      uint8_t spbr;

      /* N=0 as per fit code */

      brdv = RSPI_BRDV_DEFAULT;

      DEBUGASSERT(brdv < 4);

      if (frequency >= priv->rspiclock / FREQ_DIVISOR_2)
        {
          actual   = priv->rspiclock / FREQ_DIVISOR_2;
        }
      else if (frequency >= priv->rspiclock / FREQ_DIVISOR_4)
        {
          actual   = priv->rspiclock /  FREQ_DIVISOR_4;
        }
      else if (frequency >= priv->rspiclock / FREQ_DIVISOR_6)
        {
          actual   = priv->rspiclock /  FREQ_DIVISOR_6;
        }
      else if (frequency >= priv->rspiclock / FREQ_DIVISOR_8)
        {
          actual   = priv->rspiclock /  FREQ_DIVISOR_8;
        }
      else if (frequency >= priv->rspiclock / FREQ_DIVISOR_10)
        {
          actual   = priv->rspiclock /  FREQ_DIVISOR_10;
        }
      else if (frequency >= priv->rspiclock / FREQ_DIVISOR_12)
        {
          actual   = priv->rspiclock /  FREQ_DIVISOR_12;
        }
      else if (frequency >= priv->rspiclock / FREQ_DIVISOR_24)
        {
          actual   = priv->rspiclock /  FREQ_DIVISOR_24;
        }
      else if (frequency >= priv->rspiclock / FREQ_DIVISOR_48)
        {
          actual   = priv->rspiclock /  FREQ_DIVISOR_48;
        }
      else /* (frequency >= priv->rspiclock / FREQ_DIVISOR_96) */
        {
          actual   = priv->rspiclock /  FREQ_DIVISOR_96;
        }

      /* Calculate n and set the n in SPBR */

      spbr =  (priv->rspiclock / (2 * actual * ((1 << brdv)))) - 1;
      rspi_putreg8(priv, RX65N_RSPI_SPBR_OFFSET, spbr);

      /* Set the N as BRDV[1:0] in SPCMD0 b2 and b3 */

      regval16 = rspi_getreg16(priv, RX65N_RSPI_SPCMD0_OFFSET);
      regval16 &= ~(RSPI_SPCMD_BRDV_MASK);
      regval16 |= (brdv << 2);
      rspi_putreg16(priv, RX65N_RSPI_SPCMD0_OFFSET, regval16);

      /* Save the frequency and actual frequency in RSPI device structure */

      priv->frequency = frequency;
      priv->actual    = actual;
    }

  return actual;
}

/****************************************************************************
 * Name: rspi_setmode
 *
 * Description:
 *   Set the SPI mode. Optional.  See enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void rspi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct rx65n_rspidev_s *priv = (struct rx65n_rspidev_s *)dev;

  uint16_t regval16;

  if (priv && mode != priv->mode)
    {
      uint32_t modebits;

      /* Select the CTL register bits based on the selected mode */

      switch (mode)
        {
          case SPIDEV_MODE0: /* CPOL=0 CPHA=0 */
            modebits = 0;
            break;

          case SPIDEV_MODE1: /* CPOL=0 CPHA=1 */
            modebits = RSPI_SPCMD_PHA;
            break;

          case SPIDEV_MODE2: /* CPOL=1 CPHA=0 */
            modebits = RSPI_SPCMD_POL;
            break;

          case SPIDEV_MODE3: /* CPOL=1 CPHA=1 */
            modebits = RSPI_SPCMD_PHA | RSPI_SPCMD_POL;
            break;

          default:
            spiwarn("Warning: Unsupported RSPI mode: %d\n", mode);
            return;
        }

      /* Then set the selected mode */

      regval16 = rspi_getreg16(priv, RX65N_RSPI_SPCMD0_OFFSET);
      regval16 &= ~(RSPI_SPCMD_PHA | RSPI_SPCMD_POL);
      regval16 |= modebits;
      rspi_putreg16(priv, RX65N_RSPI_SPCMD0_OFFSET, regval16);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: rspi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void rspi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  struct rx65n_rspidev_s *priv = (struct rx65n_rspidev_s *)dev;
  uint8_t regval8;
  uint16_t regval16;

  if (priv && nbits != priv->nbits && nbits > 0 && nbits <= 16)
    {
      if (nbits >= 4 && nbits <= 8)
        {
          regval8 = rspi_getreg8(priv, RX65N_RSPI_SPDCR_OFFSET);
          regval8 |= RSPI_SPDCR_SPBYT;
          rspi_putreg8(priv, RX65N_RSPI_SPDCR_OFFSET, regval8);
        }
      else if (nbits > 8 && nbits <= 16)
        {
          regval8 = rspi_getreg8(priv, RX65N_RSPI_SPDCR_OFFSET);
          regval8 &= ~(RSPI_SPDCR_SPLW | RSPI_SPDCR_SPBYT);
          rspi_putreg8(priv, RX65N_RSPI_SPDCR_OFFSET, regval8);
        }
      else
        {
          spierr("ERROR: RSPI not supporting: %d bit word\n", nbits);
        }

      /* Configure command data register for this transfer */

      regval16 = rspi_getreg16(priv, RX65N_RSPI_SPCMD0_OFFSET);
      regval16 |= (RSPI_SPCMD_SPB_MASK & ((nbits - 1) << 8));
      rspi_putreg16(priv, RX65N_RSPI_SPCMD0_OFFSET, regval16);

      priv->nbits   = nbits;
    }
  else
    {
      spiwarn("Warning: Unsupported RSPI Word width: %d\n", nbits);
    }
}

/****************************************************************************
 * Name: rspi_send
 *
 * Description:
 *   Exchange one word on RSPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint32_t rspi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  struct rx65n_rspidev_s *priv = (struct rx65n_rspidev_s *)dev;
  uint32_t response = 0;

  rspi_transfer(priv, &wd, &response, 1);
  return response;
}

/****************************************************************************
 * Name: rspi_exchange
 *
 * Description:
 *   Exahange a block of data from SPI. Required.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   buffer   - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
static void rspi_exchange(FAR struct spi_dev_s *dev,
                            FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  struct rx65n_rspidev_s *priv = (struct rx65n_rspidev_s *)dev;
  rspi_transfer(priv, txbuffer, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: rspi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nwords - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void rspi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer,
                         size_t nwords)
{
  struct rx65n_rspidev_s *priv = (struct rx65n_rspidev_s *)dev;
  rspi_transfer(priv, buffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: rspi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to receive data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of
 *            bits-per-word selected for the SPI interface.  If nbits <= 8,
 *            the data is packed into uint8_t's; if nbits >8, the data is
 *            packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void rspi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                          size_t nwords)
{
  struct rx65n_rspidev_s *priv = (struct rx65n_rspidev_s *)dev;
  rspi_transfer(priv, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Function Name: rspi_interrupt_init
 * Description  : Configure ICU for RSPI interrupt
 * Arguments    : none
 * Return Value : none
 ****************************************************************************/

void rspi_interrupt_init(FAR struct rx65n_rspidev_s *priv, uint8_t bus)
{
  /* Enable error interrupt source bit */

  IEN(ICU, GROUPAL0) = 0;  /* Disable Group AL0 interrupts */
  IR(ICU, GROUPAL0)  = 0;  /* Clear interrupt flag */
  IPR(ICU, GROUPAL0) = RX65N_RSPI_INTRRUPT_PRIO;
  IEN(ICU, GROUPAL0) = 1;  /* Enable Group AL0 interrupt */

#ifdef CONFIG_RX65N_RSPI0
  if (bus == RX65N_RSPI_CHANNEL0)
    {
      /* Configure Transmit empty buffer interrupt source bit */

      IEN(RSPI0, SPTI0) = 0;  /* Disable SPTI0 interrupts */
      IR(RSPI0, SPTI0)  = 0;  /* Clear interrupt flag */
      IPR(RSPI0, SPTI0) = RX65N_RSPI_INTRRUPT_PRIO;
      IEN(RSPI0, SPTI0) = 1;  /* Enable SPTI0 interrupt */
      ICU.IER[0x04].BIT.IEN7 = 1;

      /* Configure Receive buffer full interrupt source bit */

      IEN(RSPI0, SPRI0) = 0; /* Disable SPRI0 interrupts */
      IR(RSPI0, SPRI0) = 0;  /* Clear interrupt flag */
      IPR(RSPI0, SPRI0) = RX65N_RSPI_INTRRUPT_PRIO;
      IEN(RSPI0, SPRI0) = 1; /* Enable SPRI0 interrupt */
      ICU.IER[0x04].BIT.IEN6 = 1;

      /* Enable error interrupt source bit */

      ICU.GENAL0.BIT.EN17 = 1;

      /* Enable Idle interrupt source bit */

      ICU.GENAL0.BIT.EN16 = 1;
    }
#endif

#ifdef CONFIG_RX65N_RSPI1
  if (bus == RX65N_RSPI_CHANNEL1)
    {
      /* Configure Transmit empty buffer interrupt */

      IEN(RSPI1, SPTI1) = 0;  /* Disable SPTI1 interrupts */
      IR(RSPI1, SPTI1)  = 0;  /* Clear interrupt flag */
      IPR(RSPI1, SPTI1) = RX65N_RSPI_INTRRUPT_PRIO;
      IEN(RSPI1, SPTI1) = 1;  /* Enable SPTI1 interrupt */
      ICU.IER[0x05].BIT.IEN1 = 1;

      /* Configure Receive buffer full interrupt */

      IEN(RSPI1, SPRI1) = 0;  /* Disable SPRI1 interrupts */
      IR(RSPI1,  SPRI1) = 0;  /* Clear interrupt flag */
      IPR(RSPI1, SPRI1) = RX65N_RSPI_INTRRUPT_PRIO;
      IEN(RSPI1, SPRI1) = 1;  /* Enable SPRI1 interrupt */
      ICU.IER[0x05].BIT.IEN0 = 1;

      /* Enable error interrupt source bit */

      ICU.GENAL0.BIT.EN19 = 1;

      /* Enable Idle interrupt source bit */

      ICU.GENAL0.BIT.EN18 = 1;
    }
#endif

#ifdef CONFIG_RX65N_RSPI2
  if (bus == RX65N_RSPI_CHANNEL2)
    {
      /* Configure Transmit empty buffer interrupt */

      IEN(RSPI2, SPTI2) = 0;  /* Disable SPTI2 interrupts */
      IR(RSPI2, SPTI2)  = 0;  /* Clear interrupt flag */
      IPR(RSPI2, SPTI2) = RX65N_RSPI_INTRRUPT_PRIO;
      IEN(RSPI2, SPTI2) = 1;  /* Enable SPTI2 interrupt */
      ICU.IER[13].BIT.IEN5 = 1;

      /* Configure Receive buffer full interrupt */

      IEN(RSPI2, SPRI2) = 0;  /* Disable SPRI2 interrupts */
      IR(RSPI2, SPRI2)  = 0;  /* Clear interrupt flag */
      IPR(RSPI2, SPRI2) = RX65N_RSPI_INTRRUPT_PRIO;
      IEN(RSPI2, SPRI2) = 1;  /* Enable SPRI2 interrupt */
      ICU.IER[13].BIT.IEN4 = 1;

      /* Enable error interrupt source bit */

      ICU.GENAL0.BIT.EN21 = 1;

      /* Enable Idle interrupt source bit */

      ICU.GENAL0.BIT.EN20 = 1;
    }
#endif
}

/****************************************************************************
 * Function Name: power_on_off
 * Description : Switches power to an RSPI channel.  Required by FIT spec.
 * Arguments :
 *   channel -Which channel to use.
 *   on_or_off -0 means 1 means off
 * Return Value : none
 ****************************************************************************/

static void rspi_power_on_off (uint8_t channel, uint8_t on_or_off)
{
  SYSTEM.PRCR.WORD = 0xa50bu;

  switch (channel)
    {
      case RX65N_RSPI_CHANNEL0:
        MSTP(RSPI0) = on_or_off;
        break;
      case RX65N_RSPI_CHANNEL1:
        MSTP(RSPI1) = on_or_off;
        break;
      case RX65N_RSPI_CHANNEL2:
        MSTP(RSPI2) = on_or_off;
        break;
    }
}

/****************************************************************************
 * Function Name: rspi_reg_protect
 * Description : Register protection enable/disable.  Required by FIT spec.
 * Arguments :
 *   enable - used to enable and disable protection
 * Return Value : none
 ****************************************************************************/

static void rspi_reg_protect (uint8_t enable)
{
  SYSTEM.PRCR.WORD = 0xa50b;
  MPC.PWPR.BIT.B0WI = 0;
  MPC.PWPR.BIT.PFSWE = 1;
}

/****************************************************************************
 * Name: rspi_bus_initialize
 *
 * Description:
 *   Initialize the selected RSPI bus in its default state (Master, 8-bit,
 *   mode 0, etc.)
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rspi_bus_initialize(FAR struct rx65n_rspidev_s *priv)
{
  uint8_t regval8;
  uint16_t regval16;

#if  defined(CONFIG_RX65N_RSPI_DTC_DT_MODE)
  int ret;

  /* Prepare Transmit and receive parameter */

  priv->dtchandle = rx65n_dtc_gethandle(0);
  ret = rx65n_dtc_setup_static_transferdata(priv->dtchandle,
                                         priv->txvec, (uint32_t)&tx_cfg,
                                        (uint32_t)priv->p_txdt, (uint32_t)0);
  if (ret < 0)
    {
      spierr("ERROR:[%d] Fail to configure DTC information for TX\n", ret);
      return;
    }

  ret = rx65n_dtc_setup_static_transferdata(priv->dtchandle,
                                       priv->rxvec, (uint32_t)&rx_cfg,
                                      (uint32_t)priv->p_rxdt, (uint32_t)0);
  if (ret < 0)
    {
      spierr("ERROR:[%d] Fail to configure DTC information for RX\n", ret);
      return;
    }

#endif

  /* Initialize control register */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR_OFFSET);
  regval8 = regval8 & (~RSPI_SPCR_SMPS); /* RSPI operation (4-wire method) */
  regval8 = regval8 & (~RSPI_SPCR_TXMD); /* Full-duplex synchronous serial communications */
  regval8 = regval8 | (RSPI_SPCR_MSTR);  /* Master mode */
  rspi_putreg8(priv, RX65N_RSPI_SPCR_OFFSET, regval8);

  /* RSPI Pin Control Register */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPPCR_OFFSET);
  regval8 &= (~(RSPI_SPPCR_SPLP      /* Loopback mode */
               | RSPI_SPPCR_SPLP2    /* Loopback mode */
               | RSPI_SPPCR_MOIFV    /* MOSI pin idles high */
               | RSPI_SPPCR_MOIFE)); /* MOSI pin idles at MOIFV */
  rspi_putreg8(priv, RX65N_RSPI_SPPCR_OFFSET, regval8);

  /* Sets polarity of SSL signal */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SSLP_OFFSET);
  regval8 &= (~(RSPI_SSLP_SSL0P
                | RSPI_SSLP_SSL1P
                | RSPI_SSLP_SSL2P
                | RSPI_SSLP_SSL3P)); /* RSPCK is low when idle */
  rspi_putreg8(priv, RX65N_RSPI_SSLP_OFFSET, regval8);

  /* Inititalize frequency, frame size and SPI mode */

  priv->frequency = 0;
  priv->mode      = SPIDEV_MODE0;

  /* Select a default frequency of approx. 400KHz */

  rspi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* Configure data control register SPDCR
   * Four frames can be transmitted or received in one round of transmission
   * or reception activation.
   */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPDCR_OFFSET);
  regval8 &= (~RSPI_SPDCR_MASK);
#ifdef CONFIG_RX65N_RSPI_BUF_SIZE
  regval8 |= (RSPI_SPDCR_SPFC_MASK & (CONFIG_RX65N_RSPI_BUF_SIZE -1));
  priv->bufsize = CONFIG_RX65N_RSPI_BUF_SIZE;
#else
  regval8 |= (RSPI_SPDCR_SPFC0 | RSPI_SPDCR_SPFC1); /* 4 frames */
  priv->bufsize = BUFSIZE_4FRAME;
#endif
  regval8 |= (RSPI_SPDCR_SPBYT);
  priv->nbits = 8;
  rspi_putreg8(priv, RX65N_RSPI_SPDCR_OFFSET, regval8);

  /*  Configure RSPI clock delay registers SPCKD */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCKD_OFFSET);
  regval8 &= (~RSPI_SPCKD_MASK);
#if CONFIG_RX65N_RSPI_SPCKD_DELAY
  regval8 |= (RSPI_SPCKD_MASK & CONFIG_RX65N_RSPI_SPCKD_DELAY);
#else
  regval8 |= (RSPI_SPCKD_SCKDL1); /* 2RSPCK delay */
#endif
  rspi_putreg8(priv, RX65N_RSPI_SPCKD_OFFSET, regval8);

  /*  Configure RSPI slave select negation delay register SSLND */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SSLND_OFFSET);
  regval8 &= (~RSPI_SSLND_MASK);
#if CONFIG_RX65N_RSPI_SSLND_DELAY
  regval8 |= (RSPI_SSLND_MASK & CONFIG_RX65N_RSPI_SSLND_DELAY);
#else
  regval8 |= (RSPI_SSLND_SLNDL1); /* 2RSPCK delay */
#endif
  rspi_putreg8(priv, RX65N_RSPI_SSLND_OFFSET, regval8);

  /* Configure RSPI next-access delay register SPND  */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPND_OFFSET);
  regval8 &= (~RSPI_SPND_MASK);
#if CONFIG_RX65N_RSPI_SPND_DELAY
  regval8 |= (RSPI_SPND_MASK & CONFIG_RX65N_RSPI_SPND_DELAY);
#else
  regval8 |= (RSPI_SPND_SPNDL1); /* 2RSPCK delay */
#endif
  rspi_putreg8(priv, RX65N_RSPI_SPND_OFFSET, regval8);

  /* Configure RSPI Control Register 2 */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR2_OFFSET);
  regval8 &= (~RSPI_SPCR2_MASK); /* Clear all bit */
#ifdef CONFIG_RX65N_RSPI_HIGHSPEED
  regval8 |= (RSPI_SPCR2_SCKASE);
#endif
  rspi_putreg8(priv, RX65N_RSPI_SPCR2_OFFSET, regval8);

  /* Configure RSPI sequence control register SPSCR */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPSCR_OFFSET);
  regval8 &= (~RSPI_SPSCR_MASK);
  rspi_putreg8(priv, RX65N_RSPI_SPSCR_OFFSET, regval8);

  /* Configure the SPCMD0 command register */

  regval16 = rspi_getreg16(priv, RX65N_RSPI_SPCMD0_OFFSET);
  regval16 &= (~RSPI_SPCMD_MASK);
  regval16 |= ((RSPI_SPCMD_SPB0 | RSPI_SPCMD_SPB1 | RSPI_SPCMD_SPB2)
#if CONFIG_RX65N_RSPI_BITORDER
                | (RSPI_SPCMD_LSBF)     /* RSPI LSB First */
#endif
                | (RSPI_SPCMD_SPNDEN)   /* RSPI Next-Access Delay Enable */
                | (RSPI_SPCMD_SLNDEN)   /* SSL Negation Delay Setting Enable */
                | (RSPI_SPCMD_SCKDEN)); /* RSPCK Delay Setting Enable */
  rspi_putreg16(priv, RX65N_RSPI_SPCMD0_OFFSET, regval16);

  /* Configure Set RSPI data control register 2 (SPDCR2) */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPDCR2_OFFSET);
  regval8 &= (~RSPI_SPDCR2_BYSW);
  rspi_putreg8(priv, RX65N_RSPI_SPDCR2_OFFSET, regval8);

#ifndef CONFIG_SPI_POLLWAIT
  /* Disable all interrupt in SPCR */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR_OFFSET);
  regval8 &= ~(RSPI_SPCR_SPEIE     /* RSPI Error Interrupt Enable */
               | RSPI_SPCR_SPTIE   /* Transmit Buffer Empty Interrupt Enable */
               | RSPI_SPCR_SPRIE); /* RSPI Receive Buffer Full Interrupt Enable */
  rspi_putreg8(priv, RX65N_RSPI_SPCR_OFFSET, regval8);

  /* Disable Idle interrupt in SPCR2 */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR2_OFFSET);
  regval8 &= ~(RSPI_SPCR2_SPIIE); /* RSPI Idle Interrupt Enable */
  rspi_putreg8(priv, RX65N_RSPI_SPCR2_OFFSET, regval8);
#endif

  /* Attach the interrupt */

#ifndef CONFIG_SPI_POLLWAIT
  irq_attach(priv->rspitxirq, (xcpt_t)rspi_txinterrupt, NULL);
  irq_attach(priv->rspirxirq, (xcpt_t)rspi_rxinterrupt, NULL);
  irq_attach(priv->rspierirq, (xcpt_t)rspi_erinterrupt, NULL);
  irq_attach(priv->rspiidlirq, (xcpt_t)rspi_idlinterrupt, NULL);
#endif

  /* Enable RSPI functionality */

  regval8 = rspi_getreg8(priv, RX65N_RSPI_SPCR_OFFSET);
  regval8 |= (RSPI_SPCR_SPE); /* RSPI Function Enable */
  rspi_putreg8(priv, RX65N_RSPI_SPCR_OFFSET, regval8);

  /* Enable SPI irq */

#ifndef CONFIG_SPI_POLLWAIT
  up_enable_irq(priv->rspitxirq);
  up_enable_irq(priv->rspirxirq);
  up_enable_irq(priv->rspierirq);
  up_enable_irq(priv->rspiidlirq);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_rspibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *rx65n_rspibus_initialize(int bus)
{
  FAR struct rx65n_rspidev_s *priv = NULL;
  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_RX65N_RSPI0
  if (bus == RX65N_RSPI_CHANNEL0)
    {
      /* Select RSPI0 */

      priv = &g_rspi0dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure RSPI0 pins for 4WIRE mode
           * RSPCKA,MOSIA, MISOA and SSLA0
           */

          rspi_reg_protect(REG_PROTECTION_OFF); /* Disable protection */
          rspi_pinconfig(bus);

          /* Switches power to an RSPI channel */

          rspi_power_on_off(bus, RSPI_POWER_ON);

          /* Configure interrupt controller for RSPI */

          rspi_interrupt_init(priv, bus);

          /* Set up default configuration: Master, 8-bit, etc. */

          rspi_bus_initialize(priv);

          priv->initialized = true;
        }
    }
  else
#endif
#ifdef CONFIG_RX65N_RSPI1
  if (bus == RX65N_RSPI_CHANNEL1)
    {
      /* Select SPI1 */

      priv = &g_rspi1dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI1 pins 4WIRE mode: RSPCKB, MOSIB, MISOB and SSLB0 */

          rspi_reg_protect(REG_PROTECTION_OFF); /* Disable protection */
          rspi_pinconfig(bus);

          /* Switches power to an RSPI channel */

          rspi_power_on_off(bus, RSPI_POWER_ON);

          /* Configure interrupt controller for RSPI */

          rspi_interrupt_init(priv, bus);

          /* Set up default configuration: Master, 8-bit, etc. */

          rspi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
#ifdef CONFIG_RX65N_RSPI2
  if (bus == RX65N_RSPI_CHANNEL2)
    {
      /* Select SPI2 */

      priv = &g_rspi2dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI2 pins 4WIRE mode: RSPCKC, MOSIC, MISOC and SSLC0 */

          rspi_reg_protect(REG_PROTECTION_OFF); /* Disable protection */
          rspi_pinconfig(bus);

          /* Switches power to an RSPI channel */

          rspi_power_on_off(bus, RSPI_POWER_ON);

          /* Configure interrupt controller for RSPI */

          rspi_interrupt_init(priv, bus);

          /* Set up default configuration: Master, 8-bit, etc. */

          rspi_bus_initialize(priv);

          priv->initialized = true;
        }
    }
  else
#endif
    {
      spierr("ERROR: Unsupported RSPI bus: %d\n", bus);
    }

  leave_critical_section(flags);
  return (FAR struct spi_dev_s *)priv;
}

#endif /* CONFIG_RX65N_RSPI0 || CONFIG_RX65N_RSPI1 || CONFIG_RX65N_RSPI2 */

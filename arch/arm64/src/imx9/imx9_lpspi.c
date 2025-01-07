/****************************************************************************
 * arch/arm64/src/imx9/imx9_lpspi.c
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
 * The external functions, imx9_lpspi1/2/3/4select and
 * imx9_lpspi1/2/3/4status must be provided by board-specific logic.
 * They are implementations of the select and status methods of the SPI
 * interface defined by struct imx9_lpspi_ops_s (see
 * include/nuttx/spi/spi.h). All other methods (including
 * imx9_lpspibus_initialize()) are provided by common IMX9 logic.
 * To use this common SPI logic on your board:
 *
 *   1. Provide logic in imx9_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide imx9_lpspi1/2/3/4select() and imx9_lpspi1/2/3/4status()
 *      functions in your board-specific logic.  These functions will
 *      perform chip selection and status operations using GPIOs in the way
 *      your board is configured.
 *   3. Add a calls to imx9_lpspibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by imx9_lpspibus_initialize() may then be
 *      used to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_lpspislotinitialize(), for example, will bind the SPI
 *      driver to the SPI MMC/SD driver).
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
#include <stddef.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "arm64_internal.h"
#include "imx9_ccm.h"
#include "imx9_clockconfig.h"
#include "imx9_dma_alloc.h"
#include "imx9_gpio.h"
#include "imx9_iomuxc.h"
#include "imx9_lpspi.h"

#include "hardware/imx9_ccm.h"
#include "hardware/imx9_lpspi.h"
#include "hardware/imx9_pinmux.h"

#ifdef CONFIG_IMX9_LPSPI_DMA
#  include "chip.h"
#  include "imx9_edma.h"
#  include "hardware/imx9_dmamux.h"
#endif

#ifdef CONFIG_IMX9_LPSPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SPI interrupts */

#ifdef CONFIG_IMX9_LPSPI_INTERRUPTS
#  error "Interrupt driven SPI not yet supported"
#endif

/* Can't have both interrupt driven SPI and SPI DMA */

#if defined(CONFIG_IMX9_LPSPI_INTERRUPTS) && defined(CONFIG_IMX9_LPSPI_DMA)
#  error "Cannot enable both interrupt mode and DMA mode for SPI"
#endif

#define SPI_SR_CLEAR   (LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF  | \
                        LPSPI_SR_TEF | LPSPI_SR_REF | LPSPI_SR_DMF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imx9_lpspidev_s
{
  struct spi_dev_s spidev;    /* Externally visible part of the SPI interface */
  uint32_t spibase;           /* SPIn base address */
  uint8_t clk_root;           /* SPIn clock root */
  uint8_t clk_gate;           /* SPIn clock gate */
#ifdef CONFIG_IMX9_LPSPI_INTERRUPTS
  uint8_t spiirq;             /* SPI IRQ number */
#endif
  mutex_t lock;               /* Held while chip is selected for mutual exclusion */
  uint32_t frequency;         /* Requested clock frequency */
  uint32_t actual;            /* Actual clock frequency */
  int8_t nbits;               /* Width of word in bits */
  uint8_t mode;               /* Mode 0,1,2,3 */
#ifdef CONFIG_IMX9_LPSPI_DMA
  volatile uint32_t rxresult;   /* Result of the RX DMA */
  volatile uint32_t txresult;   /* Result of the TX DMA */
  const uint16_t    rxch;       /* The RX DMA channel number */
  const uint16_t    txch;       /* The TX DMA channel number */
  DMACH_HANDLE      rxdma;      /* DMA channel handle for RX transfers */
  DMACH_HANDLE      txdma;      /* DMA channel handle for TX transfers */
  sem_t             rxsem;      /* Wait for RX DMA to complete */
  sem_t             txsem;      /* Wait for TX DMA to complete */
  void             *txbuf;      /* Driver DMA safe buffer for TX */
  void             *rxbuf;      /* Driver DMA safe buffer for RX */
  int               refcount;   /* SPIn initialization counter */
#endif
};

enum imx9_delay_e
{
  LPSPI_PCS_TO_SCK = 1,       /* PCS-to-SCK delay. */
  LPSPI_LAST_SCK_TO_PCS,      /* Last SCK edge to PCS delay. */
  LPSPI_BETWEEN_TRANSFER      /* Delay between transfers. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint32_t
imx9_lpspi_getreg32(struct imx9_lpspidev_s *priv, uint8_t offset);
static inline void imx9_lpspi_putreg32(struct imx9_lpspidev_s *priv,
                                       uint8_t offset, uint32_t value);
static inline uint32_t imx9_lpspi_readword(
                          struct imx9_lpspidev_s *priv);
static inline void imx9_lpspi_writeword(struct imx9_lpspidev_s *priv,
                                        uint16_t byte);
static inline bool imx9_lpspi_9to16bitmode(
                          struct imx9_lpspidev_s *priv);
static inline void imx9_lpspi_master_set_delays(struct imx9_lpspidev_s
                                                 *priv, uint32_t delay_ns,
                                                 enum imx9_delay_e type);
static inline void imx9_lpspi_master_set_delay_scaler(
                          struct imx9_lpspidev_s *priv,
                          uint32_t scaler,
                          enum imx9_delay_e type);

/* DMA support */

#ifdef CONFIG_IMX9_LPSPI_DMA
static int         spi_dmarxwait(struct imx9_lpspidev_s *priv);
static int         spi_dmatxwait(struct imx9_lpspidev_s *priv);
static inline void spi_dmarxwakeup(struct imx9_lpspidev_s *priv);
static inline void spi_dmatxwakeup(struct imx9_lpspidev_s *priv);
static void        spi_dmarxcallback(DMACH_HANDLE handle, void *arg,
                                     bool done, int result);
static void        spi_dmatxcallback(DMACH_HANDLE handle, void *arg,
                                     bool done, int result);
static inline void spi_dmarxstart(struct imx9_lpspidev_s *priv);
static inline void spi_dmatxstart(struct imx9_lpspidev_s *priv);
#endif

/* SPI methods */

static int imx9_lpspi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t imx9_lpspi_setfrequency(struct spi_dev_s *dev,
                                        uint32_t frequency);
static void imx9_lpspi_setmode(struct spi_dev_s *dev,
                               enum spi_mode_e mode);
static void imx9_lpspi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int imx9_lpspi_hwfeatures(struct spi_dev_s *dev,
                                 imx9_lpspi_hwfeatures_t features);
#endif
static uint32_t imx9_lpspi_send(struct spi_dev_s *dev, uint32_t wd);
static void imx9_lpspi_exchange(struct spi_dev_s *dev,
                                const void *txbuffer,
                                void *rxbuffer,
                                size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void imx9_lpspi_sndblock(struct spi_dev_s *dev,
                                const void *txbuffer, size_t nwords);
static void imx9_lpspi_recvblock(struct spi_dev_s *dev,
                                 void *rxbuffer,
                                 size_t nwords);
#endif

/* Initialization */

static void imx9_lpspi_bus_initialize(struct imx9_lpspidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
  .lock         = imx9_lpspi_lock,
  .select       = imx9_lpspi_select,
  .setfrequency = imx9_lpspi_setfrequency,
  .setmode      = imx9_lpspi_setmode,
  .setbits      = imx9_lpspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = imx9_lpspi_hwfeatures,
#endif
  .status       = imx9_lpspi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = imx9_lpspi_cmddata,
#endif
  .send         = imx9_lpspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = imx9_lpspi_exchange,
#else
  .sndblock     = imx9_lpspi_sndblock,
  .recvblock    = imx9_lpspi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = imx9_lpspi_register,  /* Provided externally */
#else
  .registercallback = 0,                    /* Not implemented */
#endif
};

#ifdef CONFIG_IMX9_LPSPI1
static struct imx9_lpspidev_s g_lpspi1dev =
{
  .spidev       =
  {
    .ops        = &g_spiops,
  },
  .spibase      = IMX9_LPSPI1_BASE,
  .clk_root     = CCM_CR_LPSPI1,
  .clk_gate     = CCM_LPCG_LPSPI1,
#ifdef CONFIG_IMX9_LPSPI_INTERRUPTS
  .spiirq       = IMX9_IRQ_LPSPI1,
#endif
  .lock         = NXMUTEX_INITIALIZER,
#ifdef CONFIG_IMX9_LPSPI1_DMA
  .rxch         = DMA_REQUEST_MUXLPSPI1RX,
  .txch         = DMA_REQUEST_MUXLPSPI1TX,
  .rxsem        = SEM_INITIALIZER(0),
  .txsem        = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_IMX9_LPSPI2
static struct imx9_lpspidev_s g_lpspi2dev =
{
  .spidev       =
  {
    .ops        = &g_spi2ops,
  },
  .spibase      = IMX9_LPSPI2_BASE,
  .clk_root     = CCM_CR_LPSPI2,
  .clk_gate     = CCM_LPCG_LPSPI2,
#ifdef CONFIG_IMX9_LPSPI_INTERRUPTS
  .spiirq       = IMX9_IRQ_LPSPI2,
#endif
  .lock         = NXMUTEX_INITIALIZER,
#ifdef CONFIG_IMX9_LPSPI2_DMA
  .rxch         = DMA_REQUEST_MUXLPSPI2RX,
  .txch         = DMA_REQUEST_MUXLPSPI2TX,
  .rxsem        = SEM_INITIALIZER(0),
  .txsem        = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_IMX9_LPSPI3
static struct imx9_lpspidev_s g_lpspi3dev =
{
  .spidev       =
  {
    .ops        = &g_spi3ops,
  },
  .spibase      = IMX9_LPSPI3_BASE,
  .clk_root     = CCM_CR_LPSPI3,
  .clk_gate     = CCM_LPCG_LPSPI3,
#ifdef CONFIG_IMX9_LPSPI_INTERRUPTS
  .spiirq       = IMX9_IRQ_LPSPI3,
#endif
  .lock         = NXMUTEX_INITIALIZER,
#ifdef CONFIG_IMX9_LPSPI3_DMA
  .rxch         = DMA_REQUEST_MUXLPSPI3RX,
  .txch         = DMA_REQUEST_MUXLPSPI3TX,
  .rxsem        = SEM_INITIALIZER(0),
  .txsem        = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_IMX9_LPSPI4
static struct imx9_lpspidev_s g_lpspi4dev =
{
  .spidev       =
  {
    .ops        = &g_spiops,
  },
  .spibase      = IMX9_LPSPI4_BASE,
  .clk_root     = CCM_CR_LPSPI4,
  .clk_gate     = CCM_LPCG_LPSPI4,
#ifdef CONFIG_IMX9_LPSPI_INTERRUPTS
  .spiirq       = IMX9_IRQ_LPSPI4,
#endif
  .lock         = NXMUTEX_INITIALIZER,
#ifdef CONFIG_IMX9_LPSPI4_DMA
  .rxch         = DMA_REQUEST_MUXLPSPI4RX,
  .txch         = DMA_REQUEST_MUXLPSPI4TX,
  .rxsem        = SEM_INITIALIZER(0),
  .txsem        = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_IMX9_LPSPI5
static struct imx9_lpspidev_s g_lpspi5dev =
{
  .spidev       =
  {
    &g_spiops
  },
  .spibase      = IMX9_LPSPI5_BASE,
  .clk_root     = CCM_CR_LPSPI5,
  .clk_gate     = CCM_LPCG_LPSPI5,
#ifdef CONFIG_IMX9_LPSPI_INTERRUPTS
  .spiirq       = IMX9_IRQ_LPSPI5,
#endif
  .lock         = NXMUTEX_INITIALIZER,
#ifdef CONFIG_IMX9_LPSPI5_DMA
  .rxch         = DMA_REQUEST_MUXLPSPI5RX,
  .txch         = DMA_REQUEST_MUXLPSPI5TX,
  .rxsem        = SEM_INITIALIZER(0),
  .txsem        = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_IMX9_LPSPI6
static struct imx9_lpspidev_s g_lpspi6dev =
{
  .spidev       =
  {
    &g_spiops
  },
  .spibase      = IMX9_LPSPI6_BASE,
  .clk_root     = CCM_CR_LPSPI6,
  .clk_gate     = CCM_LPCG_LPSPI6,
#ifdef CONFIG_IMX9_LPSPI_INTERRUPTS
  .spiirq       = IMX9_IRQ_LPSPI6,
#endif
  .lock         = NXMUTEX_INITIALIZER,
#ifdef CONFIG_IMX9_LPSPI6_DMA
  .rxch         = DMA_REQUEST_MUXLPSPI6RX,
  .txch         = DMA_REQUEST_MUXLPSPI6TX,
  .rxsem        = SEM_INITIALIZER(0),
  .txsem        = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_IMX9_LPSPI7
static struct imx9_lpspidev_s g_lpspi7dev =
{
  .spidev       =
  {
    &g_spiops
  },
  .spibase      = IMX9_LPSPI7_BASE,
  .clk_root     = CCM_CR_LPSPI7,
  .clk_gate     = CCM_LPCG_LPSPI7,
#ifdef CONFIG_IMX9_LPSPI_INTERRUPTS
  .spiirq       = IMX9_IRQ_LPSPI7,
#endif
  .lock         = NXMUTEX_INITIALIZER,
#ifdef CONFIG_IMX9_LPSPI7_DMA
  .rxch         = DMA_REQUEST_MUXLPSPI7RX,
  .txch         = DMA_REQUEST_MUXLPSPI7TX,
  .rxsem        = SEM_INITIALIZER(0),
  .txsem        = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_IMX9_LPSPI8
static struct imx9_lpspidev_s g_lpspi8dev =
{
  .spidev       =
  {
    &g_spiops
  },
  .spibase      = IMX9_LPSPI8_BASE,
  .clk_root     = CCM_CR_LPSPI8,
  .clk_gate     = CCM_LPCG_LPSPI8,
#ifdef CONFIG_IMX9_LPSPI_INTERRUPTS
  .spiirq       = IMX9_IRQ_LPSPI8,
#endif
  .lock         = NXMUTEX_INITIALIZER,
#ifdef CONFIG_IMX9_LPSPI6_DMA
  .rxch         = DMA_REQUEST_MUXLPSPI8RX,
  .txch         = DMA_REQUEST_MUXLPSPI8TX,
  .rxsem        = SEM_INITIALIZER(0),
  .txsem        = SEM_INITIALIZER(0),
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_lpspi_getreg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline uint32_t
imx9_lpspi_getreg32(struct imx9_lpspidev_s *priv, uint8_t offset)
{
  return getreg32(priv->spibase + offset);
}

/****************************************************************************
 * Name: imx9_lpspi_putreg
 *
 * Description:
 *   Write a 16-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 32-bit value to be written
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline void imx9_lpspi_putreg32(struct imx9_lpspidev_s *priv,
                                       uint8_t offset, uint32_t value)
{
  putreg32(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: imx9_lpspi_readword
 *
 * Description:
 *   Read one word from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   word as read
 *
 ****************************************************************************/

static inline uint32_t
imx9_lpspi_readword(struct imx9_lpspidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  while ((imx9_lpspi_getreg32(priv, IMX9_LPSPI_SR_OFFSET)
                      & LPSPI_SR_RDF) == 0);

  /* Then return the received byte */

  return imx9_lpspi_getreg32(priv, IMX9_LPSPI_RDR_OFFSET);
}

/****************************************************************************
 * Name: imx9_lpspi_writeword
 *
 * Description:
 *   Write one word to SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   word - word to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void imx9_lpspi_writeword(struct imx9_lpspidev_s *priv,
                                        uint16_t word)
{
  /* Wait until the transmit buffer is empty */

  while ((imx9_lpspi_getreg32(priv, IMX9_LPSPI_SR_OFFSET)
                       & LPSPI_SR_TDF) == 0);

  /* Then send the word */

  imx9_lpspi_putreg32(priv, IMX9_LPSPI_TDR_OFFSET, word);
}

/****************************************************************************
 * Name: imx9_lpspi_9to16bitmode
 *
 * Description:
 *   Check if the SPI is operating in more then 8 bit mode
 *
 * Input Parameters:
 *   priv     - Device-specific state data
 *
 * Returned Value:
 *   true: >8 bit mode-bit mode, false: <= 8-bit mode
 *
 ****************************************************************************/

static inline bool
imx9_lpspi_9to16bitmode(struct imx9_lpspidev_s *priv)
{
  bool ret;

  if (((imx9_lpspi_getreg32(priv, IMX9_LPSPI_TCR_OFFSET) &
        LPSPI_TCR_FRAMESZ_MASK) + 1) < 9)
    {
      ret = false;
    }
  else
    {
      ret = true;
    }

  return ret;
}

/****************************************************************************
 * Name: imx9_lpspi_modifyreg32
 *
 * Description:
 *   Clear and set bits in register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   offset  - Register offset
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imx9_lpspi_modifyreg32(struct imx9_lpspidev_s *priv,
                                   uint8_t offset, uint32_t clrbits,
                                   uint32_t setbits)
{
  modifyreg32(priv->spibase + offset, clrbits, setbits);
}

/****************************************************************************
 * Name: imx9_lpspi_modifyreg32
 *
 * Description:
 *   Clear and set bits TCR register. Need a safe wrapper as TCR expects
 *   LPSPI is _enabled_ when writing.
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   offset  - Register offset
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imx9_lpspi_modifytcr(struct imx9_lpspidev_s *priv,
                                 uint32_t clrbits, uint32_t setbits)
{
  uint32_t men;

  /* Enable LPSPI if it was disabled previously */

  men = imx9_lpspi_getreg32(priv, IMX9_LPSPI_CR_OFFSET) & LPSPI_CR_MEN;
  if (!men)
    {
      imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CR_OFFSET, 0,
                                   LPSPI_CR_MEN);
    }

  /* Update the register */

  imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_TCR_OFFSET, clrbits, setbits);

  /* Disable LPSPI if it was disabled */

  if (!men)
    {
      imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CR_OFFSET,
                                   LPSPI_CR_MEN, 0);
    }
}

/****************************************************************************
 * Name: imx9_lpspi_master_set_delays
 *
 * Description:
 *   SET LPSPI Delay times
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   scaler - scaler value
 *   type   - delay time type
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void imx9_lpspi_master_set_delay_scaler(
                          struct imx9_lpspidev_s *priv,
                          uint32_t scaler,
                          enum imx9_delay_e type)
{
  uint32_t ccr1;
  uint32_t dbt;
  uint32_t sckdiv;

  /* Read from SCKDIV and DTB will always return 0. In order to preserve the
   * old values we must calculate them here locally from CCR1 values.
   */

  ccr1    = imx9_lpspi_getreg32(priv, IMX9_LPSPI_CCR1_OFFSET);
  dbt     = (ccr1 & LPSPI_CCR1_SCKSCK_MASK) >> LPSPI_CCR1_SCKSCK_SHIFT;
  sckdiv  = (ccr1 & LPSPI_CCR1_SCKHLD_MASK) >> LPSPI_CCR1_SCKHLD_SHIFT;
  sckdiv += (ccr1 & LPSPI_CCR1_SCKSET_MASK) >> LPSPI_CCR1_SCKSET_SHIFT;

  switch (type)
    {
    case LPSPI_PCS_TO_SCK:
      imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CCR_OFFSET,
                                   LPSPI_CCR_PCSSCK_MASK, 0);
      imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CCR_OFFSET, 0,
                                   LPSPI_CCR_DBT(dbt) |
                                   LPSPI_CCR_PCSSCK(scaler) |
                                   LPSPI_CCR_SCKDIV(sckdiv));
      break;

    case LPSPI_LAST_SCK_TO_PCS:
      imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CCR_OFFSET,
                                   LPSPI_CCR_SCKPCS_MASK, 0);
      imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CCR_OFFSET, 0,
                                   LPSPI_CCR_DBT(dbt) |
                                   LPSPI_CCR_PCSSCK(scaler) |
                                   LPSPI_CCR_SCKDIV(sckdiv));
      break;

    case LPSPI_BETWEEN_TRANSFER:
      imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CCR_OFFSET,
                                   LPSPI_CCR_DBT_MASK, 0);
      imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CCR_OFFSET, 0,
                                   LPSPI_CCR_DBT(dbt) |
                                   LPSPI_CCR_PCSSCK(scaler) |
                                   LPSPI_CCR_SCKDIV(sckdiv));
      break;
    }
}

/****************************************************************************
 * Name: imx9_lpspi_master_set_delays
 *
 * Description:
 *   SET LPSPI Delay times
 *
 * Input Parameters:
 *   priv     - Device-specific state data
 *   delay_ns - delay time in nano seconds
 *   type     - delay time type
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void imx9_lpspi_master_set_delays(
                             struct imx9_lpspidev_s *priv,
                             uint32_t delay_ns,
                             enum imx9_delay_e type)
{
  uint32_t src_freq;
  uint64_t real_delay;
  uint32_t scaler;
  uint32_t best_scaler;
  uint32_t diff;
  uint32_t min_diff;
  uint64_t initial_delay_ns;
  uint32_t clock_div_prescaler;
  uint32_t additional_scaler;

  imx9_get_rootclock(priv->clk_root, &src_freq);

  clock_div_prescaler = src_freq /
              (1 << ((imx9_lpspi_getreg32(priv, IMX9_LPSPI_TCR_OFFSET) &
              LPSPI_TCR_PRESCALE_MASK) >> LPSPI_TCR_PRESCALE_SHIFT));

  min_diff = 0xffffffff;

  /* Initialize scaler to max value to generate the max delay */

  best_scaler = 0xff;

  if (type == LPSPI_BETWEEN_TRANSFER)
    {
      /* First calculate the initial, default delay, note min delay is 2
       * clock cycles. Due to large size of * calculated values (uint64_t),
       * we need to break up the calculation into several steps to ensure
       * accurate calculated results
       */

      initial_delay_ns = 1000000000U;
      initial_delay_ns *= 2;
      initial_delay_ns /= clock_div_prescaler;

      additional_scaler = 1U;
    }
  else
    {
      /* First calculate the initial, default delay, min delay is 1 clock
       * cycle. Due to large size of calculated values (uint64_t), we need to
       * break up the calculation into several steps to ensure accurate
       * calculated * results.
       */

      initial_delay_ns = 1000000000U;
      initial_delay_ns /= clock_div_prescaler;

      additional_scaler = 0;
    }

  /* If the initial, default delay is already greater than the desired delay,
   * then * set the delay to their initial value (0) and return the delay. In
   * other words, * there is no way to decrease the delay value further.
   */

  if (initial_delay_ns >= delay_ns)
    {
      imx9_lpspi_master_set_delay_scaler(priv, 0, type);
    }
  else
    {
      /* If min_diff = 0, the exit for loop */

      for (scaler = 0; (scaler < 256) && min_diff; scaler++)
        {
          /* Calculate the real delay value as we cycle through the scaler
           * values. Due to large size of calculated values (uint64_t),
           * we need to break up the calculation into several steps to
           * ensure accurate calculated results
           */

          real_delay  = 1000000000U;
          real_delay *= (scaler + 1 + additional_scaler);
          real_delay /= clock_div_prescaler;

          /* calculate the delay difference based on the conditional
           * statement that states that the calculated delay must not be
           * less then the desired delay
           */

          if (real_delay >= delay_ns)
            {
              diff = real_delay - delay_ns;
              if (min_diff > diff)
                {
                  /* A better match found */

                  min_diff = diff;
                  best_scaler = scaler;
                }
            }
        }

      imx9_lpspi_master_set_delay_scaler(priv, best_scaler, type);
    }
}

/****************************************************************************
 * Name: imx9_lpspi_lock
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
 *   None
 *
 ****************************************************************************/

static int imx9_lpspi_lock(struct spi_dev_s *dev, bool lock)
{
  struct imx9_lpspidev_s *priv = (struct imx9_lpspidev_s *)dev;
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
 * Name: imx9_lpspi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t imx9_lpspi_setfrequency(struct spi_dev_s *dev,
                                        uint32_t frequency)
{
  struct imx9_lpspidev_s *priv = (struct imx9_lpspidev_s *)dev;

  uint32_t men;
  uint32_t src_freq = 0;
  uint32_t prescaler;
  uint32_t best_prescaler;
  uint32_t scaler;
  uint32_t best_scaler;
  uint32_t real_frequency;
  uint32_t best_frequency;
  uint32_t diff;
  uint32_t min_diff;

  /* Has the LPSPI bus frequency changed? */

  if (frequency != priv->frequency)
    {
      /* Disable LPSPI if it is enabled */

      men = imx9_lpspi_getreg32(priv, IMX9_LPSPI_CR_OFFSET) & LPSPI_CR_MEN;
      if (men)
        {
          imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CR_OFFSET,
                                       LPSPI_CR_MEN, 0);
        }

      imx9_get_rootclock(priv->clk_root, &src_freq);

      min_diff       = 0xffffffff;
      best_prescaler = 7;
      best_scaler    = 255;
      best_frequency = 0;

      for (prescaler = 0; (prescaler < 8) && min_diff; prescaler++)
        {
          for (scaler = 0; (scaler < 256) && min_diff; scaler++)
            {
              real_frequency = src_freq / ((1 << prescaler) * (scaler + 2));

              /* Calculate the frequency difference based on conditional
               * statement that states that the calculated frequency must not
               * exceed desired frequency.
               */

              if (frequency >= real_frequency)
                {
                  diff = frequency - real_frequency;
                  if (min_diff > diff)
                    {
                      /* A better match found */

                      min_diff = diff;
                      best_prescaler = prescaler;
                      best_scaler = scaler;
                      best_frequency = real_frequency;
                    }
                }
            }
        }

      /* Write the best values in the CCR register */

      imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CCR_OFFSET,
                             LPSPI_CCR_SCKDIV_MASK,
                             LPSPI_CCR_SCKDIV(best_scaler));

      /* Update TCR */

      imx9_lpspi_modifytcr(priv, LPSPI_TCR_PRESCALE_MASK,
                           LPSPI_TCR_PRESCALE(best_prescaler));

      priv->frequency = frequency;
      priv->actual = best_frequency;
      imx9_lpspi_master_set_delays(priv, 1000000000 / best_frequency,
                                   LPSPI_PCS_TO_SCK);
      imx9_lpspi_master_set_delays(priv, 1000000000 / best_frequency,
                                   LPSPI_LAST_SCK_TO_PCS);
      imx9_lpspi_master_set_delays(priv, 1000000000 / best_frequency,
                                   LPSPI_BETWEEN_TRANSFER);

      /* Re-enable LPSPI if it was enabled previously */

      if (men)
        {
          imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CR_OFFSET, 0,
                                       LPSPI_CR_MEN);
        }
    }

  return priv->actual;
}

/****************************************************************************
 * Name: imx9_lpspi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e mode for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static void imx9_lpspi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct imx9_lpspidev_s *priv = (struct imx9_lpspidev_s *)dev;
  uint32_t setbits;
  uint32_t clrbits;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Disable LPSPI if it is enabled */

      switch (mode)
        {
        case SPIDEV_MODE0:     /* CPOL=0; CPHA=0 */
          setbits = 0;
          clrbits = LPSPI_TCR_CPOL | LPSPI_TCR_CPHA;
          break;

        case SPIDEV_MODE1:     /* CPOL=0; CPHA=1 */
          setbits = LPSPI_TCR_CPHA;
          clrbits = LPSPI_TCR_CPOL;
          break;

        case SPIDEV_MODE2:     /* CPOL=1; CPHA=0 */
          setbits = LPSPI_TCR_CPOL;
          clrbits = LPSPI_TCR_CPHA;
          break;

        case SPIDEV_MODE3:     /* CPOL=1; CPHA=1 */
          setbits = LPSPI_TCR_CPOL | LPSPI_TCR_CPHA;
          clrbits = 0;
          break;

        default:
          return;
        }

      /* Update TCR register */

      imx9_lpspi_modifytcr(priv, clrbits, setbits);

      /* Reset SPI read FIFO */

      imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CR_OFFSET, 0,
                                LPSPI_CR_RRF);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: imx9_lpspi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imx9_lpspi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct imx9_lpspidev_s *priv = (struct imx9_lpspidev_s *)dev;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      if (nbits < 2 || nbits > 4096)
        {
          return;
        }

      /* Update TCR */

      imx9_lpspi_modifytcr(priv, LPSPI_TCR_FRAMESZ_MASK,
                           LPSPI_TCR_FRAMESZ(nbits - 1));

      /* Save the selection so the subsequent re-configurations
       * will be faster.
       */

      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: imx9_lpspi_hwfeatures
 *
 * Description:
 *   Set hardware-specific feature flags.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   features - H/W feature flags
 *
 * Returned Value:
 *   Zero (OK) if the selected H/W features are enabled; A negated errno
 *   value if any H/W feature is not supportable.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int imx9_lpspi_hwfeatures(struct spi_dev_s *dev,
                                 imx9_lpspi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
  struct imx9_lpspidev_s *priv = (struct imx9_lpspidev_s *)dev;
  uint32_t setbits;
  uint32_t clrbits;

  spiinfo("features=%08x\n", features);

  /* Transfer data LSB first? */

  if ((features & HWFEAT_LSBFIRST) != 0)
    {
      setbits = LPSPI_TCR_LSBF;
      clrbits = 0;
    }
  else
    {
      setbits = 0;
      clrbits = LPSPI_TCR_LSBF;
    }

  imx9_lpspi_modifytcr(priv, clrbits, setbits);

  /* Other H/W features are not supported */

  return ((features & ~HWFEAT_LSBFIRST) == 0) ? OK : -ENOSYS;
#else
  return -ENOSYS;
#endif
}
#endif

/****************************************************************************
 * Name: imx9_lpspi_send
 *
 * Description:
 *   Exchange one word on SPI
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

static uint32_t imx9_lpspi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct imx9_lpspidev_s *priv = (struct imx9_lpspidev_s *)dev;
  uint32_t regval;
  uint32_t ret;

  DEBUGASSERT(priv && priv->spibase);

  imx9_lpspi_writeword(priv, wd);

  while ((imx9_lpspi_getreg32(priv, IMX9_LPSPI_SR_OFFSET) &
          LPSPI_SR_RDF) != LPSPI_SR_RDF);

  ret = imx9_lpspi_readword(priv);

  /* Check and clear any error flags (Reading from the SR clears the error
   * flags).
   */

  regval = imx9_lpspi_getreg32(priv, IMX9_LPSPI_SR_OFFSET);

  spiinfo(
         "Sent: %04" PRIx32 " Return: %04" PRIx32 " Status: %02" PRIx32 "\n",
          wd, ret, regval);

  UNUSED(regval);
  return ret;
}

/****************************************************************************
 * Name: imx9_lpspi_exchange (no DMA).  aka imx9_lpspi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed
 *              into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if !defined(CONFIG_IMX9_LPSPI_DMA)
static void imx9_lpspi_exchange(struct spi_dev_s *dev,
                                const void *txbuffer,
                                void *rxbuffer,
                                size_t nwords)
#else
static void imx9_lpspi_exchange_nodma(struct spi_dev_s *dev,
                                      const void *txbuffer,
                                      void *rxbuffer, size_t nwords)
#endif
{
  struct imx9_lpspidev_s *priv = (struct imx9_lpspidev_s *)dev;
  DEBUGASSERT(priv && priv->spibase);

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%lu\n", txbuffer, rxbuffer,
          nwords);

  /* 8- or 16-bit mode? */

  if (imx9_lpspi_9to16bitmode(priv))
    {
      /* 16-bit mode */

      const uint16_t *src = txbuffer;
      uint16_t *dest = rxbuffer;
      uint16_t word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
            }
          else
            {
              word = 0xffff;
            }

          /* Exchange one word */

          word = (uint16_t) imx9_lpspi_send(dev, (uint32_t) word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
  else
    {
      /* 8-bit mode */

      const uint8_t *src = txbuffer;
      uint8_t *dest = rxbuffer;
      uint8_t word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
            }
          else
            {
              word = 0xff;
            }

          /* Exchange one word */

          word = (uint8_t)imx9_lpspi_send(dev, word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
}

/****************************************************************************
 * Name: spi_exchange (with DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits > 8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPSPI_DMA
static void imx9_lpspi_exchange(struct spi_dev_s *dev,
                                const void *txbuffer,
                                void *rxbuffer, size_t nwords)
{
  struct imx9_lpspidev_s  *priv = (struct imx9_lpspidev_s *)dev;
  int                      ret;
  size_t                   adjust;
  ssize_t                  nbytes;
  static uint8_t           rxdummy[4] aligned_data(4);
  static const uint16_t    txdummy = 0xffff;
  uint32_t                 regval;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv && priv->spibase);
  spiinfo("txbuffer=%p rxbuffer=%p nwords=%lu\n", txbuffer, rxbuffer,
          nwords);

  /* Convert the number of word to a number of bytes */

  nbytes = (priv->nbits > 8) ? nwords << 1 : nwords;

  /* Invalid DMA channels fall back to non-DMA method. */

  if (priv->rxdma == NULL || priv->txdma == NULL
#ifdef CONFIG_IMX9_LPSPI_DMATHRESHOLD
      /* If this is a small SPI transfer, then let
       * imx9_lpspi_exchange_nodma() do the work.
       */

      || nbytes <= CONFIG_IMX9_LPSPI_DMATHRESHOLD
#endif
      )
    {
      imx9_lpspi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }

  /* Check if the transfer is too long */

  if (nbytes > CONFIG_IMX9_LPSPI_DMA_BUFFER_SIZE)
    {
      /* Transfer is too long, revert to slow non-DMA method */

      spiwarn("frame %lu too long, fall back to no DMA transfer\n", nbytes);
      imx9_lpspi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }

  /* Disable SPI when we are configuring it */

  imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CR_OFFSET, LPSPI_CR_MEN, 0);

  /* ERR050456 workaround: Reset FIFOs using CR[RST] bit */

  regval = imx9_lpspi_getreg32(priv, IMX9_LPSPI_CFGR1_OFFSET);

  imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CR_OFFSET,
                LPSPI_CR_RTF | LPSPI_CR_RRF,
                LPSPI_CR_RTF | LPSPI_CR_RRF);

  imx9_lpspi_putreg32(priv, IMX9_LPSPI_CFGR1_OFFSET, regval);

  /* Clear all status bits */

  imx9_lpspi_putreg32(priv, IMX9_LPSPI_SR_OFFSET, SPI_SR_CLEAR);

  /* disable DMA */

  imx9_lpspi_putreg32(priv, IMX9_LPSPI_DER_OFFSET, 0);

  if (txbuffer)
    {
      /* Move the user data to device internal buffer */

      memcpy(priv->txbuf, txbuffer, nbytes);

      /* And flush it to RAM */

      up_clean_dcache((uintptr_t)priv->txbuf,
                      (uintptr_t)priv->txbuf + nbytes);
    }

  /* Set up the DMA */

  adjust = (priv->nbits > 8) ? 2 : 1;

  struct imx9_edma_xfrconfig_s config;

  config.saddr  = priv->spibase + IMX9_LPSPI_RDR_OFFSET;
  config.daddr  = (uintptr_t) (rxbuffer ? priv->rxbuf : rxdummy);
  config.soff   = 0;
  config.doff   = rxbuffer ? adjust : 0;
  config.iter   = nwords;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.dsize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.nbytes = adjust;
#ifdef CONFIG_IMX9_EDMA_ELINK
  config.linkch = NULL;
#endif
  imx9_dmach_xfrsetup(priv->rxdma, &config);

  config.saddr  = (uintptr_t) (txbuffer ? priv->txbuf : &txdummy);
  config.daddr  = priv->spibase + IMX9_LPSPI_TDR_OFFSET;
  config.soff   = txbuffer ? adjust : 0;
  config.doff   = 0;
  config.iter   = nwords;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.dsize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.nbytes = adjust;
#ifdef CONFIG_IMX9_EDMA_ELINK
  config.linkch = NULL;
#endif
  imx9_dmach_xfrsetup(priv->txdma, &config);

  /* Start the DMAs */

  spi_dmarxstart(priv);
  spi_dmatxstart(priv);

  /* Enable SPI again */

  imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CR_OFFSET, 0, LPSPI_CR_MEN);

  /* Invoke SPI DMA */

  imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_DER_OFFSET,
                         0, LPSPI_DER_TDDE | LPSPI_DER_RDDE);

  /* Then wait for each to complete */

  ret = spi_dmarxwait(priv);

  if (ret < 0)
    {
      ret = spi_dmatxwait(priv);
    }

  /* Reset any status */

  imx9_lpspi_putreg32(priv, IMX9_LPSPI_SR_OFFSET, SPI_SR_CLEAR);

  /* Disable DMA */

  imx9_lpspi_putreg32(priv, IMX9_LPSPI_DER_OFFSET, 0);

  if (rxbuffer && ret >= 0)
    {
      /* Flush the RX data to ram */

      up_invalidate_dcache((uintptr_t)priv->rxbuf,
                           (uintptr_t)priv->rxbuf + nbytes);

      /* Copy data to user buffer */

      memcpy(rxbuffer, priv->rxbuf, nbytes);
    }
}

#endif /* CONFIG_IMX9_SPI_DMA */

/****************************************************************************
 * Name: imx9_lpspi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of
 *              words. The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.  If nbits <= 8,
 *              the data is packed into uint8_t's; if nbits >8, the data is
 *              packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void imx9_lpspi_sndblock(struct spi_dev_s *dev,
                                const void *txbuffer, size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%lu\n", txbuffer, nwords);
  return imx9_lpspi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: imx9_lpspi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that can be received in the buffer in
 *              number of words.  The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.  If nbits <= 8,
 *              the data is packed into uint8_t's; if nbits >8, the data is
 *              packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void imx9_lpspi_recvblock(struct spi_dev_s *dev,
                                 void *rxbuffer, size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%lu\n", rxbuffer, nwords);
  return imx9_lpspi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: imx9_lpspi_clock_enable
 *
 * Description:
 *   Ungate LPSPI clock
 *
 ****************************************************************************/

static void imx9_lpspi_clock_enable(struct imx9_lpspidev_s *priv)
{
  imx9_ccm_gate_on(priv->clk_gate, true);
}

/****************************************************************************
 * Name: imx9_lpspi_bus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus in its default state
 *   (Master, 8-bit, mode 0, etc.)
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imx9_lpspi_bus_initialize(struct imx9_lpspidev_s *priv)
{
  uint32_t reg = 0;

  /* Make sure bus is disabled */

  imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CR_OFFSET, LPSPI_CR_MEN, 0);

  /* Enable power and reset the peripheral */

  imx9_lpspi_clock_enable(priv);

  /* Reset to known status */

  imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CR_OFFSET, 0, LPSPI_CR_RST);
  imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CR_OFFSET, 0,
                         LPSPI_CR_RTF | LPSPI_CR_RRF);
  imx9_lpspi_putreg32(priv, IMX9_LPSPI_CR_OFFSET, 0x00);

  /* Set LPSPI to master */

  imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CFGR1_OFFSET, 0,
                         LPSPI_CFGR1_MASTER);

  /* Set specific PCS to active high or low
   * TODO: Not needed for now
   */

  /* Set Configuration Register 1 related setting. */

  reg = imx9_lpspi_getreg32(priv, IMX9_LPSPI_CFGR1_OFFSET);
  reg &= ~(LPSPI_CFGR1_OUTCFG | LPSPI_CFGR1_PINCFG_MASK |
           LPSPI_CFGR1_NOSTALL);
  reg |= LPSPI_CFGR1_OUTCFG_RETAIN | LPSPI_CFGR1_PINCFG_SIN_SOUT;
  imx9_lpspi_putreg32(priv, IMX9_LPSPI_CFGR1_OFFSET, reg);

  /* Set frequency and delay times */

  imx9_lpspi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Set default watermarks */

  imx9_lpspi_putreg32(priv, IMX9_LPSPI_FCR_OFFSET,
                      LPSPI_FCR_TXWATER(0) | LPSPI_FCR_RXWATER(0));

  /* Set Transmit Command Register */

  imx9_lpspi_setbits((struct spi_dev_s *)priv, 8);

  imx9_lpspi_setmode((struct spi_dev_s *)priv, SPIDEV_MODE0);

  /* Enable LPSPI */

  imx9_lpspi_modifyreg32(priv, IMX9_LPSPI_CR_OFFSET, 0, LPSPI_CR_MEN);
}

/****************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPSPI_DMA
static int spi_dmarxwait(struct imx9_lpspidev_s *priv)
{
  int ret;

  /* Take the semaphore (perhaps waiting).  If the result is zero, then the
   *  DMA must not really have completed.
   */

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->rxsem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (priv->rxresult == 0 && ret == OK);

  return ret;
}
#endif

/****************************************************************************
 * Name: spi_dmatxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPSPI_DMA
static int spi_dmatxwait(struct imx9_lpspidev_s *priv)
{
  int ret;

  /* Take the semaphore (perhaps waiting).  If the result is zero, then the
   * DMA must not really have completed.
   */

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->txsem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (priv->txresult == 0 && ret == OK);

  return ret;
}
#endif

/****************************************************************************
 * Name: spi_dmarxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPSPI_DMA
static inline void spi_dmarxwakeup(struct imx9_lpspidev_s *priv)
{
  nxsem_post(&priv->rxsem);
}
#endif

/****************************************************************************
 * Name: spi_dmatxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPSPI_DMA
static inline void spi_dmatxwakeup(struct imx9_lpspidev_s *priv)
{
  nxsem_post(&priv->txsem);
}
#endif

/****************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPSPI_DMA
static void spi_dmarxcallback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct imx9_lpspidev_s *priv = (struct imx9_lpspidev_s *)arg;

  priv->rxresult = result | 0x80000000;  /* assure non-zero */
  spi_dmarxwakeup(priv);
}
#endif

/****************************************************************************
 * Name: spi_dmatxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPSPI_DMA
static void spi_dmatxcallback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct imx9_lpspidev_s *priv = (struct imx9_lpspidev_s *)arg;

  /* Wake-up the SPI driver */

  priv->txresult = result | 0x80000000;  /* assure non-zero */
  spi_dmatxwakeup(priv);
}
#endif

/****************************************************************************
 * Name: spi_dmarxstart
 *
 * Description:
 *   Start RX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPSPI_DMA
static inline void spi_dmarxstart(struct imx9_lpspidev_s *priv)
{
  priv->rxresult = 0;
  imx9_dmach_start(priv->rxdma, spi_dmarxcallback, priv);
}
#endif

/****************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPSPI_DMA
static inline void spi_dmatxstart(struct imx9_lpspidev_s *priv)
{
  priv->txresult = 0;
  imx9_dmach_start(priv->txdma, spi_dmatxcallback, priv);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_lpspibus_initialize
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

struct spi_dev_s *imx9_lpspibus_initialize(int bus)
{
  struct imx9_lpspidev_s *priv = NULL;

#ifdef CONFIG_IMX9_LPSPI1
  if (bus == 1)
    {
      /* Select SPI1 */

      priv = &g_lpspi1dev;

      /* Only configure if the bus is not already configured */

      if (priv->refcount == 0)
        {
          /* Configure SPI1 pins: SCK, MISO, and MOSI */

          imx9_iomux_configure(MUX_LPSPI1_SCK);
          imx9_iomux_configure(MUX_LPSPI1_MISO);
          imx9_iomux_configure(MUX_LPSPI1_MOSI);
#if defined(MUX_LPSPI1_CS) && defined(GPIO_LPSPI1_CS)
          imx9_iomux_configure(MUX_LPSPI1_CS);
          imx9_config_gpio(GPIO_LPSPI1_CS);
#endif
#if defined(GPIO_LPSPI1_DC) && defined(CONFIG_SPI_CMDDATA)
          imx9_iomux_configure(GPIO_LPSPI1_DC);
#endif
        }
    }
  else
#endif
#ifdef CONFIG_IMX9_LPSPI2
  if (bus == 2)
    {
      /* Select SPI2 */

      priv = &g_lpspi2dev;

      /* Only configure if the bus is not already configured */

      if (priv->refcount == 0)
        {
          /* Configure SPI2 pins: SCK, MISO, and MOSI */

          imx9_iomux_configure(MUX_LPSPI2_SCK);
          imx9_iomux_configure(MUX_LPSPI2_MISO);
          imx9_iomux_configure(MUX_LPSPI2_MOSI);
#if defined(MUX_LPSPI2_CS) && defined(GPIO_LPSPI2_CS)
          imx9_iomux_configure(MUX_LPSPI2_CS);
          imx9_config_gpio(GPIO_LPSPI2_CS);
#endif
#if defined(GPIO_LPSPI2_DC) && defined(CONFIG_SPI_CMDDATA)
          imx9_iomux_configure(GPIO_LPSPI2_DC);
#endif
        }
    }
  else
#endif
#ifdef CONFIG_IMX9_LPSPI3
  if (bus == 3)
    {
      /* Select SPI3 */

      priv = &g_lpspi3dev;

      /* Only configure if the bus is not already configured */

      if (priv->refcount == 0)
        {
          /* Configure SPI3 pins: SCK, MISO, and MOSI */

          imx9_iomux_configure(MUX_LPSPI3_SCK);
          imx9_iomux_configure(MUX_LPSPI3_MISO);
          imx9_iomux_configure(MUX_LPSPI3_MOSI);
#if defined(MUX_LPSPI3_CS) && defined(GPIO_LPSPI3_CS)
          imx9_iomux_configure(MUX_LPSPI3_CS);
          imx9_config_gpio(GPIO_LPSPI3_CS);
#endif
#if defined(GPIO_LPSPI3_DC) && defined(CONFIG_SPI_CMDDATA)
          imx9_iomux_configure(GPIO_LPSPI3_DC);
#endif
        }
    }
  else
#endif
#ifdef CONFIG_IMX9_LPSPI4
  if (bus == 4)
    {
      /* Select SPI4 */

      priv = &g_lpspi4dev;

      /* Only configure if the bus is not already configured */

      if (priv->refcount == 0)
        {
          /* Configure SPI4 pins: SCK, MISO, and MOSI */

          imx9_iomux_configure(MUX_LPSPI4_SCK);
          imx9_iomux_configure(MUX_LPSPI4_MISO);
          imx9_iomux_configure(MUX_LPSPI4_MOSI);
#if defined(MUX_LPSPI4_CS) && defined(GPIO_LPSPI4_CS)
          imx9_iomux_configure(MUX_LPSPI4_CS);
          imx9_config_gpio(GPIO_LPSPI4_CS);
#endif
#if defined(GPIO_LPSPI4_DC) && defined(CONFIG_SPI_CMDDATA)
          imx9_iomux_configure(GPIO_LPSPI4_DC);
#endif
        }
    }
  else
#endif
#ifdef CONFIG_IMX9_LPSPI5
  if (bus == 5)
    {
      /* Select SPI5 */

      priv = &g_lpspi5dev;

      /* Only configure if the bus is not already configured */

      if (priv->refcount == 0)
        {
          /* Configure SPI5 pins: SCK, MISO, and MOSI */

          imx9_iomux_configure(MUX_LPSPI5_SCK);
          imx9_iomux_configure(MUX_LPSPI5_MISO);
          imx9_iomux_configure(MUX_LPSPI5_MOSI);
#if defined(MUX_LPSPI5_CS) && defined(GPIO_LPSPI5_CS)
          imx9_iomux_configure(MUX_LPSPI5_CS);
          imx9_config_gpio(GPIO_LPSPI5_CS);
#endif
#if defined(GPIO_LPSPI5_DC) && defined(CONFIG_SPI_CMDDATA)
          imx9_iomux_configure(GPIO_LPSPI5_DC);
#endif
        }
    }
  else
#endif
#ifdef CONFIG_IMX9_LPSPI6
  if (bus == 6)
    {
      /* Select SPI6 */

      priv = &g_lpspi6dev;

      /* Only configure if the bus is not already configured */

      if (priv->refcount == 0)
        {
          /* Configure SPI6 pins: SCK, MISO, and MOSI */

          imx9_iomux_configure(MUX_LPSPI6_SCK);
          imx9_iomux_configure(MUX_LPSPI6_MISO);
          imx9_iomux_configure(MUX_LPSPI6_MOSI);
#if defined(MUX_LPSPI6_CS) && defined(GPIO_LPSPI6_CS)
          imx9_iomux_configure(MUX_LPSPI6_CS);
          imx9_config_gpio(GPIO_LPSPI6_CS);
#endif
#if defined(GPIO_LPSPI6_DC) && defined(CONFIG_SPI_CMDDATA)
          imx9_iomux_configure(GPIO_LPSPI6_DC);
#endif
        }
    }
  else
#endif
#ifdef CONFIG_IMX9_LPSPI7
  if (bus == 7)
    {
      /* Select SPI7 */

      priv = &g_lpspi7dev;

      /* Only configure if the bus is not already configured */

      if (priv->refcount == 0)
        {
          /* Configure SPI7 pins: SCK, MISO, and MOSI */

          imx9_iomux_configure(MUX_LPSPI7_SCK);
          imx9_iomux_configure(MUX_LPSPI7_MISO);
          imx9_iomux_configure(MUX_LPSPI7_MOSI);
#if defined(MUX_LPSPI7_CS) && defined(GPIO_LPSPI7_CS)
          imx9_iomux_configure(MUX_LPSPI7_CS);
          imx9_config_gpio(GPIO_LPSPI7_CS);
#endif
#if defined(GPIO_LPSPI7_DC) && defined(CONFIG_SPI_CMDDATA)
          imx9_iomux_configure(GPIO_LPSPI7_DC);
#endif
        }
    }
  else
#endif
#ifdef CONFIG_IMX9_LPSPI8
  if (bus == 8)
    {
      /* Select SPI8 */

      priv = &g_lpspi8dev;

      /* Only configure if the bus is not already configured */

      if (priv->refcount == 0)
        {
          /* Configure SPI6 pins: SCK, MISO, and MOSI */

          imx9_iomux_configure(MUX_LPSPI8_SCK);
          imx9_iomux_configure(MUX_LPSPI8_MISO);
          imx9_iomux_configure(MUX_LPSPI8_MOSI);
#if defined(MUX_LPSPI8_CS) && defined(GPIO_LPSPI8_CS)
          imx9_iomux_configure(MUX_LPSPI8_CS);
          imx9_config_gpio(GPIO_LPSPI8_CS);
#endif
#if defined(GPIO_LPSPI8_DC) && defined(CONFIG_SPI_CMDDATA)
          imx9_iomux_configure(GPIO_LPSPI8_DC);
#endif
        }
    }
  else
#endif
    {
      spierr("ERROR: Unsupported SPI bus: %d\n", bus);
      return NULL;
    }

  /* Set up default configuration: Master, 8-bit, etc. */

  if (priv->refcount == 0)
    {
      imx9_lpspi_bus_initialize(priv);
    }

  priv->refcount++;

#ifdef CONFIG_IMX9_LPSPI_DMA
  if (priv->rxch && priv->txch)
    {
      if (priv->txdma == NULL && priv->rxdma == NULL)
        {
          priv->txdma = imx9_dmach_alloc(priv->txch, 0);
          priv->rxdma = imx9_dmach_alloc(priv->rxch, 0);
          DEBUGASSERT(priv->rxdma && priv->txdma);
        }

      if (priv->txbuf == NULL && priv->rxbuf == NULL)
        {
          priv->txbuf = imx9_dma_alloc(CONFIG_IMX9_LPSPI_DMA_BUFFER_SIZE);
          priv->rxbuf = imx9_dma_alloc(CONFIG_IMX9_LPSPI_DMA_BUFFER_SIZE);
          DEBUGASSERT(priv->txbuf && priv->rxbuf);

          /* Invalidate the RX buffer area initially */

          up_invalidate_dcache((uintptr_t)priv->rxbuf,
                               (uintptr_t)priv->rxbuf +
                               CONFIG_IMX9_LPSPI_DMA_BUFFER_SIZE);
        }
    }
  else
    {
      priv->rxdma = NULL;
      priv->txdma = NULL;
    }
#endif

  return (struct spi_dev_s *)priv;
}

/****************************************************************************
 * Name: imx9_lpspibus_uninitialize
 *
 * Description:
 *   Unitialize the selected SPI bus
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx9_lpspi_uninitialize(struct spi_dev_s *dev)
{
  struct imx9_lpspidev_s  *priv = (struct imx9_lpspidev_s *)dev;

  if (priv->refcount > 0)
    {
      priv->refcount--;
      if (priv->refcount == 0)
        {
          imx9_lpspi_modifyreg32(priv,
                                 IMX9_LPSPI_CR_OFFSET, LPSPI_CR_MEN, 0);
        }
    }
}

#endif /* CONFIG_IMX9_LPSPI */

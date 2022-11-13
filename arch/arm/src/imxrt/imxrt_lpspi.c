/****************************************************************************
 * arch/arm/src/imxrt/imxrt_lpspi.c
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
 * The external functions, imxrt_lpspi1/2/3/4select and
 * imxrt_lpspi1/2/3/4status must be provided by board-specific logic.
 * They are implementations of the select and status methods of the SPI
 * interface defined by struct imxrt_lpspi_ops_s (see
 * include/nuttx/spi/spi.h). All other methods (including
 * imxrt_lpspibus_initialize()) are provided by common IMXRT logic.
 * To use this common SPI logic on your board:
 *
 *   1. Provide logic in imxrt_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide imxrt_lpspi1/2/3/4select() and imxrt_lpspi1/2/3/4status()
 *      functions in your board-specific logic.  These functions will
 *      perform chip selection and status operations using GPIOs in the way
 *      your board is configured.
 *   3. Add a calls to imxrt_lpspibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by imxrt_lpspibus_initialize() may then be
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

#include "arm_internal.h"
#include "chip.h"

#include "imxrt_lpspi.h"
#include "imxrt_gpio.h"
#include "hardware/imxrt_pinmux.h"
#include "hardware/imxrt_lpspi.h"
#include "hardware/imxrt_ccm.h"
#include "imxrt_periphclks.h"

#include "hardware/imxrt_dmamux.h"
#include "imxrt_edma.h"

#if defined(CONFIG_IMXRT_LPSPI1) || defined(CONFIG_IMXRT_LPSPI2) || \
    defined(CONFIG_IMXRT_LPSPI3) || defined(CONFIG_IMXRT_LPSPI4)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SPI interrupts */

#ifdef CONFIG_IMXRT_LPSPI_INTERRUPTS
#  error "Interrupt driven SPI not yet supported"
#endif

/* Can't have both interrupt driven SPI and SPI DMA */

#if defined(CONFIG_IMXRT_LPSPI_INTERRUPTS) && defined(CONFIG_IMXRT_LPSPI_DMA)
#  error "Cannot enable both interrupt mode and DMA mode for SPI"
#endif

#define  SPI_SR_CLEAR   (LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF  | \
                         LPSPI_SR_TEF | LPSPI_SR_REF | LPSPI_SR_DMF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imxrt_lpspidev_s
{
  struct spi_dev_s spidev;    /* Externally visible part of the SPI interface */
  uint32_t spibase;           /* SPIn base address */
#ifdef CONFIG_IMXRT_LPSPI_INTERRUPTS
  uint8_t spiirq;             /* SPI IRQ number */
#endif
  mutex_t lock;               /* Held while chip is selected for mutual exclusion */
  uint32_t frequency;         /* Requested clock frequency */
  uint32_t actual;            /* Actual clock frequency */
  int8_t nbits;               /* Width of word in bits */
  uint8_t mode;               /* Mode 0,1,2,3 */
#ifdef CONFIG_IMXRT_LPSPI_DMA
  volatile uint32_t rxresult;   /* Result of the RX DMA */
  volatile uint32_t txresult;   /* Result of the TX DMA */
  const uint16_t    rxch;       /* The RX DMA channel number */
  const uint16_t    txch;       /* The TX DMA channel number */
  DMACH_HANDLE      rxdma;      /* DMA channel handle for RX transfers */
  DMACH_HANDLE      txdma;      /* DMA channel handle for TX transfers */
  sem_t             rxsem;      /* Wait for RX DMA to complete */
  sem_t             txsem;      /* Wait for TX DMA to complete */
#endif
};

enum imxrt_delay_e
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
imxrt_lpspi_getreg32(struct imxrt_lpspidev_s *priv,
                     uint8_t offset);
static inline void imxrt_lpspi_putreg32(struct imxrt_lpspidev_s *priv,
                                        uint8_t offset, uint32_t value);
static inline uint32_t imxrt_lpspi_readword(
                          struct imxrt_lpspidev_s *priv);
static inline void imxrt_lpspi_writeword(struct imxrt_lpspidev_s *priv,
                                         uint16_t byte);
static inline bool imxrt_lpspi_9to16bitmode(
                          struct imxrt_lpspidev_s *priv);
static inline void imxrt_lpspi_master_set_delays(struct imxrt_lpspidev_s
                                                 *priv, uint32_t delay_ns,
                                                 enum imxrt_delay_e type);
static inline void imxrt_lpspi_master_set_delay_scaler(
                          struct imxrt_lpspidev_s *priv,
                          uint32_t scaler,
                          enum imxrt_delay_e type);

/* DMA support */

#ifdef CONFIG_IMXRT_LPSPI_DMA
static int         spi_dmarxwait(struct imxrt_lpspidev_s *priv);
static int         spi_dmatxwait(struct imxrt_lpspidev_s *priv);
static inline void spi_dmarxwakeup(struct imxrt_lpspidev_s *priv);
static inline void spi_dmatxwakeup(struct imxrt_lpspidev_s *priv);
static void        spi_dmarxcallback(DMACH_HANDLE handle, void *arg,
                                     bool done, int result);
static void        spi_dmatxcallback(DMACH_HANDLE handle, void *arg,
                                     bool done, int result);
static inline void spi_dmarxstart(struct imxrt_lpspidev_s *priv);
static inline void spi_dmatxstart(struct imxrt_lpspidev_s *priv);
#endif

/* SPI methods */

static int imxrt_lpspi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t imxrt_lpspi_setfrequency(struct spi_dev_s *dev,
                                         uint32_t frequency);
static void imxrt_lpspi_setmode(struct spi_dev_s *dev,
                                enum spi_mode_e mode);
static void imxrt_lpspi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int imxrt_lpspi_hwfeatures(struct spi_dev_s *dev,
                                  imxrt_lpspi_hwfeatures_t features);
#endif
static uint32_t imxrt_lpspi_send(struct spi_dev_s *dev, uint32_t wd);
static void imxrt_lpspi_exchange(struct spi_dev_s *dev,
                                 const void *txbuffer,
                                 void *rxbuffer,
                                 size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void imxrt_lpspi_sndblock(struct spi_dev_s *dev,
                                 const void *txbuffer, size_t nwords);
static void imxrt_lpspi_recvblock(struct spi_dev_s *dev,
                                  void *rxbuffer,
                                  size_t nwords);
#endif

/* Initialization */

static void imxrt_lpspi_bus_initialize(struct imxrt_lpspidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_IMXRT_LPSPI1
static const struct spi_ops_s g_spi1ops =
{
  .lock         = imxrt_lpspi_lock,
  .select       = imxrt_lpspi1select,
  .setfrequency = imxrt_lpspi_setfrequency,
  .setmode      = imxrt_lpspi_setmode,
  .setbits      = imxrt_lpspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = imxrt_lpspi_hwfeatures,
#endif
  .status       = imxrt_lpspi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = imxrt_lpspi1cmddata,
#endif
  .send         = imxrt_lpspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = imxrt_lpspi_exchange,
#else
  .sndblock     = imxrt_lpspi_sndblock,
  .recvblock    = imxrt_lpspi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = imxrt_lpspi1register,  /* Provided externally */
#else
  .registercallback = 0,                     /* Not implemented */
#endif
};

static struct imxrt_lpspidev_s g_lpspi1dev =
{
  .spidev       =
  {
    &g_spi1ops
  },
  .spibase      = IMXRT_LPSPI1_BASE,
#ifdef CONFIG_IMXRT_LPSPI_INTERRUPTS
  .spiirq       = IMXRT_IRQ_LPSPI1,
#endif
#ifdef CONFIG_IMXRT_LPSPI1_DMA
  .rxch         = IMXRT_DMACHAN_LPSPI1_RX,
  .txch         = IMXRT_DMACHAN_LPSPI1_TX,
#endif
};
#endif

#ifdef CONFIG_IMXRT_LPSPI2
static const struct spi_ops_s g_spi2ops =
{
  .lock         = imxrt_lpspi_lock,
  .select       = imxrt_lpspi2select,
  .setfrequency = imxrt_lpspi_setfrequency,
  .setmode      = imxrt_lpspi_setmode,
  .setbits      = imxrt_lpspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = imxrt_lpspi_hwfeatures,
#endif
  .status       = imxrt_lpspi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = imxrt_lpspi2cmddata,
#endif
  .send         = imxrt_lpspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = imxrt_lpspi_exchange,
#else
  .sndblock     = imxrt_lpspi_sndblock,
  .recvblock    = imxrt_lpspi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = imxrt_lpspi2register,  /* Provided externally */
#else
  .registercallback = 0,                     /* Not implemented */
#endif
};

static struct imxrt_lpspidev_s g_lpspi2dev =
{
  .spidev       =
  {
    &g_spi2ops
  },
  .spibase      = IMXRT_LPSPI2_BASE,
#ifdef CONFIG_IMXRT_LPSPI_INTERRUPTS
  .spiirq       = IMXRT_IRQ_LPSPI2,
#endif
#ifdef CONFIG_IMXRT_LPSPI2_DMA
  .rxch         = IMXRT_DMACHAN_LPSPI2_RX,
  .txch         = IMXRT_DMACHAN_LPSPI2_TX,
#endif
};
#endif

#ifdef CONFIG_IMXRT_LPSPI3
static const struct spi_ops_s g_spi3ops =
{
  .lock         = imxrt_lpspi_lock,
  .select       = imxrt_lpspi3select,
  .setfrequency = imxrt_lpspi_setfrequency,
  .setmode      = imxrt_lpspi_setmode,
  .setbits      = imxrt_lpspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = imxrt_lpspi_hwfeatures,
#endif
  .status       = imxrt_lpspi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = imxrt_lpspi3cmddata,
#endif
  .send         = imxrt_lpspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = imxrt_lpspi_exchange,
#else
  .sndblock     = imxrt_lpspi_sndblock,
  .recvblock    = imxrt_lpspi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = imxrt_lpspi3register,  /* Provided externally */
#else
  .registercallback = 0,                     /* Not implemented */
#endif
};

static struct imxrt_lpspidev_s g_lpspi3dev =
{
  .spidev       =
  {
    &g_spi3ops
  },
  .spibase      = IMXRT_LPSPI3_BASE,
#ifdef CONFIG_IMXRT_LPSPI_INTERRUPTS
  .spiirq       = IMXRT_IRQ_LPSPI3,
#endif
#ifdef CONFIG_IMXRT_LPSPI3_DMA
  .rxch         = IMXRT_DMACHAN_LPSPI3_RX,
  .txch         = IMXRT_DMACHAN_LPSPI3_TX,
#endif
};
#endif

#ifdef CONFIG_IMXRT_LPSPI4
static const struct spi_ops_s g_spi4ops =
{
  .lock         = imxrt_lpspi_lock,
  .select       = imxrt_lpspi4select,
  .setfrequency = imxrt_lpspi_setfrequency,
  .setmode      = imxrt_lpspi_setmode,
  .setbits      = imxrt_lpspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = imxrt_lpspi_hwfeatures,
#endif
  .status       = imxrt_lpspi4status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = imxrt_lpspi4cmddata,
#endif
  .send         = imxrt_lpspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = imxrt_lpspi_exchange,
#else
  .sndblock     = imxrt_lpspi_sndblock,
  .recvblock    = imxrt_lpspi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = imxrt_lpspi4register,  /* Provided externally */
#else
  .registercallback = 0,                     /* Not implemented */
#endif
};

static struct imxrt_lpspidev_s g_lpspi4dev =
{
  .spidev       =
  {
    &g_spi4ops
  },
  .spibase      = IMXRT_LPSPI4_BASE,
#ifdef CONFIG_IMXRT_LPSPI_INTERRUPTS
  .spiirq       = IMXRT_IRQ_LPSPI4,
#endif
#ifdef CONFIG_IMXRT_LPSPI4_DMA
  .rxch         = IMXRT_DMACHAN_LPSPI4_RX,
  .txch         = IMXRT_DMACHAN_LPSPI4_TX,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_lpspi_getreg8
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 8-bit register
 *
 ****************************************************************************/

static inline uint8_t imxrt_lpspi_getreg8(struct imxrt_lpspidev_s *priv,
                                          uint8_t offset)
{
  return getreg8(priv->spibase + offset);
}

/****************************************************************************
 * Name: imxrt_lpspi_putreg8
 *
 * Description:
 *   Write a 8-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 8-bit value to be written
 *
 ****************************************************************************/

static inline void imxrt_lpspi_putreg8(struct imxrt_lpspidev_s *priv,
                                       uint8_t offset, uint8_t value)
{
  putreg8(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: imxrt_lpspi_getreg
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
imxrt_lpspi_getreg32(struct imxrt_lpspidev_s *priv,
                     uint8_t offset)
{
  return getreg32(priv->spibase + offset);
}

/****************************************************************************
 * Name: imxrt_lpspi_putreg
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

static inline void imxrt_lpspi_putreg32(struct imxrt_lpspidev_s *priv,
                                        uint8_t offset, uint32_t value)
{
  putreg32(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: imxrt_lpspi_readword
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
imxrt_lpspi_readword(struct imxrt_lpspidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  while ((imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_SR_OFFSET)
                      & LPSPI_SR_RDF) == 0);

  /* Then return the received byte */

  return imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_RDR_OFFSET);
}

/****************************************************************************
 * Name: imxrt_lpspi_writeword
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

static inline void imxrt_lpspi_writeword(struct imxrt_lpspidev_s *priv,
                                         uint16_t word)
{
  /* Wait until the transmit buffer is empty */

  while ((imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_SR_OFFSET)
                       & LPSPI_SR_TDF) == 0);

  /* Then send the word */

  imxrt_lpspi_putreg32(priv, IMXRT_LPSPI_TDR_OFFSET, word);
}

/****************************************************************************
 * Name: imxrt_lpspi_readbyte
 *
 * Description:
 *   Read one byte from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Byte as read
 *
 ****************************************************************************/

static inline uint8_t imxrt_lpspi_readbyte(struct imxrt_lpspidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  while ((imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_SR_OFFSET)
                       & LPSPI_SR_RDF) == 0);

  /* Then return the received byte */

  return imxrt_lpspi_getreg8(priv, IMXRT_LPSPI_RDR_OFFSET);
}

/****************************************************************************
 * Name: imxrt_lpspi_writebyte
 *
 * Description:
 *   Write one 8-bit frame to the SPI FIFO
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   byte - Byte to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void imxrt_lpspi_writebyte(struct imxrt_lpspidev_s *priv,
                                         uint8_t byte)
{
  /* Wait until the transmit buffer is empty */

  while ((imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_SR_OFFSET)
                           & LPSPI_SR_TDF) == 0);

  /* Then send the byte */

  imxrt_lpspi_putreg8(priv, IMXRT_LPSPI_TDR_OFFSET, byte);
}

/****************************************************************************
 * Name: imxrt_lpspi_9to16bitmode
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
imxrt_lpspi_9to16bitmode(struct imxrt_lpspidev_s *priv)
{
  bool ret;

  if (((imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_TCR_OFFSET) &
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
 * Name: imxrt_lpspi_modifyreg32
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

static void imxrt_lpspi_modifyreg32(struct imxrt_lpspidev_s *priv,
                                    uint8_t offset, uint32_t clrbits,
                                    uint32_t setbits)
{
  modifyreg32(priv->spibase + offset, clrbits, setbits);
}

/****************************************************************************
 * Name: imxrt_lpspi_master_set_delays
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

static inline void imxrt_lpspi_master_set_delay_scaler(
                          struct imxrt_lpspidev_s *priv,
                          uint32_t scaler,
                          enum imxrt_delay_e type)
{
  switch (type)
    {
    case LPSPI_PCS_TO_SCK:
      imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CCR_OFFSET,
                              LPSPI_CCR_PCSSCK_MASK, 0);
      imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CCR_OFFSET, 0,
                              LPSPI_CCR_PCSSCK(scaler));
      break;

    case LPSPI_LAST_SCK_TO_PCS:
      imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CCR_OFFSET,
                              LPSPI_CCR_SCKPCS_MASK, 0);
      imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CCR_OFFSET, 0,
                              LPSPI_CCR_SCKPCS(scaler));
      break;

    case LPSPI_BETWEEN_TRANSFER:
      imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CCR_OFFSET,
                                    LPSPI_CCR_DBT_MASK, 0);
      imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CCR_OFFSET, 0,
                              LPSPI_CCR_DBT(scaler));
      break;
    }
}

/****************************************************************************
 * Name: imxrt_lpspi_master_set_delays
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

static inline void imxrt_lpspi_master_set_delays(
                             struct imxrt_lpspidev_s *priv,
                             uint32_t delay_ns,
                             enum imxrt_delay_e type)
{
  uint32_t pll3_div;
  uint32_t pll_freq;
  uint32_t src_freq;
  uint64_t real_delay;
  uint32_t scaler;
  uint32_t best_scaler;
  uint32_t diff;
  uint32_t min_diff;
  uint64_t initial_delay_ns;
  uint32_t clock_div_prescaler;
  uint32_t additional_scaler;

  if ((getreg32(IMXRT_CCM_ANALOG_PLL_USB1) &
       CCM_ANALOG_PLL_USB1_DIV_SELECT_MASK) != 0)
    {
      pll3_div = 22;
    }
  else
    {
      pll3_div = 20;
    }

  pll_freq = BOARD_XTAL_FREQUENCY * pll3_div;

  /* Assumption this formula will work only if the LPSPI Clock Source is PLL3
   * PFD0 so check if LPSPI clock source is set to 1 (PLL3 PFD0) in CCM_CBCMR
   * register bits 4-5
   */

  src_freq  = pll_freq /
              ((getreg32(IMXRT_CCM_ANALOG_PFD_480)
                         & CCM_ANALOG_PFD_480_PFD0_FRAC_MASK) >>
                         CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT);
  src_freq *= 18;
  src_freq /= ((getreg32(IMXRT_CCM_CBCMR) & CCM_CBCMR_LPSPI_PODF_MASK) >>
               CCM_CBCMR_LPSPI_PODF_SHIFT) + 1;

  clock_div_prescaler = src_freq /
              (1 << ((imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_TCR_OFFSET) &
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
      imxrt_lpspi_master_set_delay_scaler(priv, 0, type);
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

      imxrt_lpspi_master_set_delay_scaler(priv, best_scaler, type);
    }
}

/****************************************************************************
 * Name: imxrt_lpspi_lock
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

static int imxrt_lpspi_lock(struct spi_dev_s *dev, bool lock)
{
  struct imxrt_lpspidev_s *priv = (struct imxrt_lpspidev_s *)dev;
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
 * Name: imxrt_lpspi_setfrequency
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

static uint32_t imxrt_lpspi_setfrequency(struct spi_dev_s *dev,
                                         uint32_t frequency)
{
  struct imxrt_lpspidev_s *priv = (struct imxrt_lpspidev_s *)dev;

  uint32_t men;
  uint32_t pll_freq;
  uint32_t pll3_div;
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

      men = imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_CR_OFFSET) & LPSPI_CR_MEN;
      if (men)
        {
          imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CR_OFFSET,
                                        LPSPI_CR_MEN, 0);
        }

      if ((getreg32(IMXRT_CCM_ANALOG_PLL_USB1) &
           CCM_ANALOG_PLL_USB1_DIV_SELECT_MASK) != 0)
        {
          pll3_div = 22;
        }
      else
        {
          pll3_div = 20;
        }

      pll_freq = BOARD_XTAL_FREQUENCY * pll3_div;

      /* Assumption this formula will work only if the LPSPI Clock Source is
       * PLL3 PFD0 * so check if LPSPI clock source is set to 1 (PLL3 PFD0)
       * in CCM_CBCMR register bits 4-5
       */

      src_freq  = pll_freq /
                  ((getreg32(IMXRT_CCM_ANALOG_PFD_480)
                    & CCM_ANALOG_PFD_480_PFD0_FRAC_MASK) >>
                      CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT);
      src_freq *= 18;
      src_freq /= ((getreg32(IMXRT_CCM_CBCMR) & CCM_CBCMR_LPSPI_PODF_MASK) >>
                   CCM_CBCMR_LPSPI_PODF_SHIFT) + 1;

      min_diff  = 0xffffffff;

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

      imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CCR_OFFSET,
                              LPSPI_CCR_SCKDIV_MASK,
                              LPSPI_CCR_SCKDIV(best_scaler));

      imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_TCR_OFFSET,
                              LPSPI_TCR_PRESCALE_MASK,
                              LPSPI_TCR_PRESCALE(best_prescaler));

      priv->frequency = frequency;
      priv->actual = best_frequency;

      imxrt_lpspi_master_set_delays(priv, 1000000000 / best_frequency,
                                    LPSPI_PCS_TO_SCK);
      imxrt_lpspi_master_set_delays(priv, 1000000000 / best_frequency,
                                    LPSPI_LAST_SCK_TO_PCS);
      imxrt_lpspi_master_set_delays(priv, 1000000000 / best_frequency,
                                    LPSPI_BETWEEN_TRANSFER);

      /* Re-enable LPSPI if it was enabled previously */

      if (men)
        {
          imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CR_OFFSET, 0,
                                        LPSPI_CR_MEN);
        }
    }

  return priv->actual;
}

/****************************************************************************
 * Name: imxrt_lpspi_setmode
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

static void imxrt_lpspi_setmode(struct spi_dev_s *dev,
                                enum spi_mode_e mode)
{
  struct imxrt_lpspidev_s *priv = (struct imxrt_lpspidev_s *)dev;
  uint32_t setbits;
  uint32_t clrbits;
  uint32_t men;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Disable LPSPI if it is enabled */

      men = imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_CR_OFFSET) & LPSPI_CR_MEN;
      if (men)
        {
          imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CR_OFFSET,
                                        LPSPI_CR_MEN, 0);
        }

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

      imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_TCR_OFFSET,
                              clrbits, setbits);

      while ((imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_RSR_OFFSET) &
              LPSPI_RSR_RXEMPTY) != LPSPI_RSR_RXEMPTY)
        {
          /* Flush SPI read FIFO */

          imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_RSR_OFFSET);
        }

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;

      /* Re-enable LPSPI if it was enabled previously */

      if (men)
        {
          imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CR_OFFSET, 0,
                                        LPSPI_CR_MEN);
        }
    }
}

/****************************************************************************
 * Name: imxrt_lpspi_setbits
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

static void imxrt_lpspi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct imxrt_lpspidev_s *priv = (struct imxrt_lpspidev_s *)dev;
  uint32_t men;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      if (nbits < 2 || nbits > 4096)
        {
          return;
        }

      /* Disable LPSPI if it is enabled */

      men = imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_CR_OFFSET) & LPSPI_CR_MEN;
      if (men)
        {
          imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CR_OFFSET,
                                        LPSPI_CR_MEN, 0);
        }

      imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_TCR_OFFSET,
                              LPSPI_TCR_FRAMESZ_MASK,
                              LPSPI_TCR_FRAMESZ(nbits - 1));

      /* Save the selection so the subsequent re-configurations
       * will be faster.
       */

      priv->nbits = nbits;

      /* Re-enable LPSPI if it was enabled previously */

      if (men)
        {
          imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CR_OFFSET, 0,
                                        LPSPI_CR_MEN);
        }
    }
}

/****************************************************************************
 * Name: imxrt_lpspi_hwfeatures
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
static int imxrt_lpspi_hwfeatures(struct spi_dev_s *dev,
                                  imxrt_lpspi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
  struct imxrt_lpspidev_s *priv = (struct imxrt_lpspidev_s *)dev;
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

  imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_TCR_OFFSET, clrbits, setbits);

  /* Other H/W features are not supported */

  return ((features & ~HWFEAT_LSBFIRST) == 0) ? OK : -ENOSYS;
#else
  return -ENOSYS;
#endif
}
#endif

/****************************************************************************
 * Name: imxrt_lpspi_send
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

static uint32_t imxrt_lpspi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct imxrt_lpspidev_s *priv = (struct imxrt_lpspidev_s *)dev;
  uint32_t regval;
  uint32_t ret;

  DEBUGASSERT(priv && priv->spibase);

  imxrt_lpspi_writeword(priv, wd);

  while ((imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_SR_OFFSET) &
          LPSPI_SR_RDF) != LPSPI_SR_RDF);

  ret = imxrt_lpspi_readword(priv);

  /* Check and clear any error flags (Reading from the SR clears the error
   * flags).
   */

  regval = imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_SR_OFFSET);

  spiinfo(
         "Sent: %04" PRIx32 " Return: %04" PRIx32 " Status: %02" PRIx32 "\n",
          wd, ret, regval);

  UNUSED(regval);
  return ret;
}

/****************************************************************************
 * Name: imxrt_lpspi_exchange (no DMA).  aka imxrt_lpspi_exchange_nodma
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

#if !defined(CONFIG_IMXRT_LPSPI_DMA)
static void imxrt_lpspi_exchange(struct spi_dev_s *dev,
                                 const void *txbuffer,
                                 void *rxbuffer,
                                 size_t nwords)
#else
static void imxrt_lpspi_exchange_nodma(struct spi_dev_s *dev,
                                       const void *txbuffer,
                                       void *rxbuffer, size_t nwords)
#endif
{
  struct imxrt_lpspidev_s *priv = (struct imxrt_lpspidev_s *)dev;
  DEBUGASSERT(priv && priv->spibase);

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* 8- or 16-bit mode? */

  if (imxrt_lpspi_9to16bitmode(priv))
    {
      /* 16-bit mode */

      const uint16_t *src = (const uint16_t *)txbuffer;
      uint16_t *dest = (uint16_t *) rxbuffer;
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

          word = (uint16_t) imxrt_lpspi_send(dev, (uint32_t) word);

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

      const uint8_t *src = (const uint8_t *)txbuffer;
      uint8_t *dest = (uint8_t *) rxbuffer;
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

          word = (uint8_t) imxrt_lpspi_send(dev, (uint32_t) word);

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

#ifdef CONFIG_IMXRT_LPSPI_DMA
static void imxrt_lpspi_exchange(struct spi_dev_s * dev,
                                 const void * txbuffer,
                                 void * rxbuffer, size_t nwords)
{
  int                          ret;
  size_t                       adjust;
  ssize_t                      nbytes;
  static uint8_t               rxdummy[4] aligned_data(4);
  static const uint16_t        txdummy = 0xffff;
  uint32_t                     regval;
  struct imxrt_lpspidev_s *priv = (struct imxrt_lpspidev_s *)dev;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv && priv->spibase);
  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* Convert the number of word to a number of bytes */

  nbytes = (priv->nbits > 8) ? nwords << 2 : nwords;

  /* Invalid DMA channels fall back to non-DMA method. */

  if (priv->rxdma == NULL || priv->txdma == NULL
#ifdef CONFIG_IMXRT_LPSPI_DMATHRESHOLD
      /* If this is a small SPI transfer, then let
       * imxrt_lpspi_exchange_nodma() do the work.
       */

      || nbytes <= CONFIG_IMXRT_LPSPI_DMATHRESHOLD
#endif
      )
    {
      imxrt_lpspi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }

  /* ERR050456 workaround: Reset FIFOs using CR[RST] bit */

  regval = imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_CFGR1_OFFSET);

  imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CR_OFFSET,
                LPSPI_CR_RTF | LPSPI_CR_RRF,
                LPSPI_CR_RTF | LPSPI_CR_RRF);

  imxrt_lpspi_putreg32(priv, IMXRT_LPSPI_CFGR1_OFFSET, regval);

  /* Clear all status bits */

  imxrt_lpspi_putreg32(priv, IMXRT_LPSPI_SR_OFFSET, SPI_SR_CLEAR);

  /* disable DMA */

  imxrt_lpspi_putreg32(priv, IMXRT_LPSPI_DER_OFFSET, 0);

  if (txbuffer)
    {
      up_clean_dcache((uintptr_t)txbuffer, (uintptr_t)txbuffer + nbytes);
    }

  if (rxbuffer)
    {
     up_invalidate_dcache((uintptr_t)rxbuffer,
                          (uintptr_t)rxbuffer + nbytes);
    }

  /* Set up the DMA */

  adjust = (priv->nbits > 8) ? 2 : 1;

  struct imxrt_edma_xfrconfig_s config;

  config.saddr  = priv->spibase + IMXRT_LPSPI_RDR_OFFSET;
  config.daddr  = (uint32_t) (rxbuffer ? rxbuffer : rxdummy);
  config.soff   = 0;
  config.doff   = rxbuffer ? adjust : 0;
  config.iter   = nbytes;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.dsize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.nbytes = adjust;
#ifdef CONFIG_KINETIS_EDMA_ELINK
  config.linkch = NULL;
#endif
  imxrt_dmach_xfrsetup(priv->rxdma, &config);

  config.saddr  = (uint32_t) (txbuffer ? txbuffer : &txdummy);
  config.daddr  = priv->spibase + IMXRT_LPSPI_TDR_OFFSET;
  config.soff   = txbuffer ? adjust : 0;
  config.doff   = 0;
  config.iter   = nbytes;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.dsize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.nbytes = adjust;
#ifdef CONFIG_KINETIS_EDMA_ELINK
  config.linkch = NULL;
#endif
  imxrt_dmach_xfrsetup(priv->txdma, &config);

  /* Start the DMAs */

  spi_dmarxstart(priv);
  spi_dmatxstart(priv);

  /* Invoke SPI DMA */

  imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_DER_OFFSET,
                0, LPSPI_DER_TDDE | LPSPI_DER_RDDE);

  /* Then wait for each to complete */

  ret = spi_dmarxwait(priv);

  if (ret < 0)
    {
      ret = spi_dmatxwait(priv);
    }

  /* Reset any status */

  imxrt_lpspi_putreg32(priv, IMXRT_LPSPI_SR_OFFSET,
                         imxrt_lpspi_getreg32(priv,
                                                IMXRT_LPSPI_SR_OFFSET));

  /* Disable DMA */

  imxrt_lpspi_putreg32(priv, IMXRT_LPSPI_DER_OFFSET, 0);
}

#endif  /* CONFIG_IMXRT_SPI_DMA */

/****************************************************************************
 * Name: imxrt_lpspi_sndblock
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
static void imxrt_lpspi_sndblock(struct spi_dev_s *dev,
                                 const void *txbuffer, size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return imxrt_lpspi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: imxrt_lpspi_recvblock
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
static void imxrt_lpspi_recvblock(struct spi_dev_s *dev,
                                  void *rxbuffer, size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return imxrt_lpspi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: imxrt_lpspi_clock_enable
 *
 * Description:
 *   Ungate LPSPI clock
 *
 ****************************************************************************/

void imxrt_lpspi_clock_enable(uint32_t base)
{
  if (base == IMXRT_LPSPI1_BASE)
    {
      imxrt_clockall_lpspi1();
    }
  else if (base == IMXRT_LPSPI2_BASE)
    {
      imxrt_clockall_lpspi2();
    }
  else if (base == IMXRT_LPSPI3_BASE)
    {
      imxrt_clockall_lpspi3();
    }
  else if (base == IMXRT_LPSPI4_BASE)
    {
      imxrt_clockall_lpspi4();
    }
}

/****************************************************************************
 * Name: imxrt_lpspi_clock_disable
 *
 * Description:
 *   Gate LPSPI clock
 *
 ****************************************************************************/

void imxrt_lpspi_clock_disable(uint32_t base)
{
  if (base == IMXRT_LPSPI1_BASE)
    {
      imxrt_clockoff_lpspi1();
    }
  else if (base == IMXRT_LPSPI2_BASE)
    {
      imxrt_clockoff_lpspi2();
    }
  else if (base == IMXRT_LPSPI3_BASE)
    {
      imxrt_clockoff_lpspi3();
    }
  else if (base == IMXRT_LPSPI4_BASE)
    {
      imxrt_clockoff_lpspi4();
    }
}

/****************************************************************************
 * Name: imxrt_lpspi_bus_initialize
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

static void imxrt_lpspi_bus_initialize(struct imxrt_lpspidev_s *priv)
{
  uint32_t reg = 0;

  /* Enable power and reset the peripheral */

  imxrt_lpspi_clock_enable(priv->spibase);

  /* Reset to known status */

  imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CR_OFFSET, 0, LPSPI_CR_RST);
  imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CR_OFFSET, 0,
                          LPSPI_CR_RTF | LPSPI_CR_RRF);
  imxrt_lpspi_putreg32(priv, IMXRT_LPSPI_CR_OFFSET, 0x00);

  /* Set LPSPI to master */

  imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CFGR1_OFFSET, 0,
                          LPSPI_CFGR1_MASTER);

  /* Set specific PCS to active high or low
   * TODO: Not needed for now
   */

  /* Set Configuration Register 1 related setting. */

  reg = imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_CFGR1_OFFSET);
  reg &= ~(LPSPI_CFGR1_OUTCFG | LPSPI_CFGR1_PINCFG_MASK
           | LPSPI_CFGR1_NOSTALL);
  reg |= LPSPI_CFGR1_OUTCFG_RETAIN | LPSPI_CFGR1_PINCFG_SIN_SOUT;
  imxrt_lpspi_putreg32(priv, IMXRT_LPSPI_CFGR1_OFFSET, reg);

  /* Set frequency and delay times */

  imxrt_lpspi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Set default watermarks */

  imxrt_lpspi_putreg32(priv, IMXRT_LPSPI_FCR_OFFSET,
                       LPSPI_FCR_TXWATER(0) | LPSPI_FCR_RXWATER(0));

  /* Set Transmit Command Register */

  imxrt_lpspi_setbits((struct spi_dev_s *)priv, 8);

  imxrt_lpspi_setmode((struct spi_dev_s *)priv, SPIDEV_MODE0);

  /* Initialize the SPI mutex that enforces mutually exclusive access */

  nxmutex_init(&priv->lock);

  /* Enable LPSPI */

  imxrt_lpspi_modifyreg32(priv, IMXRT_LPSPI_CR_OFFSET, 0, LPSPI_CR_MEN);
}

/****************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_LPSPI_DMA
static int spi_dmarxwait(struct imxrt_lpspidev_s *priv)
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

#ifdef CONFIG_IMXRT_LPSPI_DMA
static int spi_dmatxwait(struct imxrt_lpspidev_s *priv)
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

#ifdef CONFIG_IMXRT_LPSPI_DMA
static inline void spi_dmarxwakeup(struct imxrt_lpspidev_s *priv)
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

#ifdef CONFIG_IMXRT_LPSPI_DMA
static inline void spi_dmatxwakeup(struct imxrt_lpspidev_s *priv)
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

#ifdef CONFIG_IMXRT_LPSPI_DMA
static void spi_dmarxcallback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct imxrt_lpspidev_s *priv = (struct imxrt_lpspidev_s *)arg;

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

#ifdef CONFIG_IMXRT_LPSPI_DMA
static void spi_dmatxcallback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct imxrt_lpspidev_s *priv = (struct imxrt_lpspidev_s *)arg;

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

#ifdef CONFIG_IMXRT_LPSPI_DMA
static inline void spi_dmarxstart(struct imxrt_lpspidev_s *priv)
{
  priv->rxresult = 0;
  imxrt_dmach_start(priv->rxdma, spi_dmarxcallback, priv);
}
#endif

/****************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_LPSPI_DMA
static inline void spi_dmatxstart(struct imxrt_lpspidev_s *priv)
{
  priv->txresult = 0;
  imxrt_dmach_start(priv->txdma, spi_dmatxcallback, priv);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_lpspibus_initialize
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

struct spi_dev_s *imxrt_lpspibus_initialize(int bus)
{
  struct imxrt_lpspidev_s *priv = NULL;

  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_IMXRT_LPSPI1
  if (bus == 1)
    {
      /* Select SPI1 */

      priv = &g_lpspi1dev;

      /* Only configure if the bus is not already configured */

      if ((imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_CR_OFFSET)
           & LPSPI_CR_MEN) == 0)
        {
          /* Configure SPI1 pins: SCK, MISO, and MOSI */

          imxrt_config_gpio(GPIO_LPSPI1_SCK);
          imxrt_config_gpio(GPIO_LPSPI1_MISO);
          imxrt_config_gpio(GPIO_LPSPI1_MOSI);
#ifdef GPIO_LPSPI1_CS
          imxrt_config_gpio(GPIO_LPSPI1_CS);
#endif
#if defined(GPIO_LPSPI1_DC) && defined(CONFIG_SPI_CMDDATA)
          imxrt_config_gpio(GPIO_LPSPI1_DC);
#endif

          /* Set up default configuration: Master, 8-bit, etc. */

          imxrt_lpspi_bus_initialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_IMXRT_LPSPI2
  if (bus == 2)
    {
      /* Select SPI2 */

      priv = &g_lpspi2dev;

      /* Only configure if the bus is not already configured */

      if ((imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_CR_OFFSET)
           & LPSPI_CR_MEN) == 0)
        {
          /* Configure SPI2 pins: SCK, MISO, and MOSI */

          imxrt_config_gpio(GPIO_LPSPI2_SCK);
          imxrt_config_gpio(GPIO_LPSPI2_MISO);
          imxrt_config_gpio(GPIO_LPSPI2_MOSI);
#ifdef GPIO_LPSPI2_CS
          imxrt_config_gpio(GPIO_LPSPI2_CS);
#endif
#if defined(GPIO_LPSPI2_DC) && defined(CONFIG_SPI_CMDDATA)
          imxrt_config_gpio(GPIO_LPSPI2_DC);
#endif

          /* Set up default configuration: Master, 8-bit, etc. */

          imxrt_lpspi_bus_initialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_IMXRT_LPSPI3
  if (bus == 3)
    {
      /* Select SPI3 */

      priv = &g_lpspi3dev;

      /* Only configure if the bus is not already configured */

      if ((imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_CR_OFFSET)
           & LPSPI_CR_MEN) == 0)
        {
          /* Configure SPI3 pins: SCK, MISO, and MOSI */

          imxrt_config_gpio(GPIO_LPSPI3_SCK);
          imxrt_config_gpio(GPIO_LPSPI3_MISO);
          imxrt_config_gpio(GPIO_LPSPI3_MOSI);
#ifdef GPIO_LPSPI3_CS
          imxrt_config_gpio(GPIO_LPSPI3_CS);
#endif
#if defined(GPIO_LPSPI3_DC) && defined(CONFIG_SPI_CMDDATA)
          imxrt_config_gpio(GPIO_LPSPI3_DC);
#endif

          /* Set up default configuration: Master, 8-bit, etc. */

          imxrt_lpspi_bus_initialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_IMXRT_LPSPI4
  if (bus == 4)
    {
      /* Select SPI4 */

      priv = &g_lpspi4dev;

      /* Only configure if the bus is not already configured */

      if ((imxrt_lpspi_getreg32(priv, IMXRT_LPSPI_CR_OFFSET)
           & LPSPI_CR_MEN) == 0)
        {
          /* Configure SPI4 pins: SCK, MISO, and MOSI */

          imxrt_config_gpio(GPIO_LPSPI4_SCK);
          imxrt_config_gpio(GPIO_LPSPI4_MISO);
          imxrt_config_gpio(GPIO_LPSPI4_MOSI);
#ifdef GPIO_LPSPI4_CS
          imxrt_config_gpio(GPIO_LPSPI4_CS);
#endif
#if defined(GPIO_LPSPI4_DC) && defined(CONFIG_SPI_CMDDATA)
          imxrt_config_gpio(GPIO_LPSPI4_DC);
#endif

          /* Set up default configuration: Master, 8-bit, etc. */

          imxrt_lpspi_bus_initialize(priv);
        }
    }
  else
#endif
    {
      spierr("ERROR: Unsupported SPI bus: %d\n", bus);
    }

#ifdef CONFIG_IMXRT_LPSPI_DMA
  /* Initialize the SPI semaphores that is used to wait for DMA completion.
   * This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  if (priv->rxch && priv->txch)
    {
      if (priv->txdma == NULL && priv->rxdma == NULL)
        {
          nxsem_init(&priv->rxsem, 0, 0);
          nxsem_init(&priv->txsem, 0, 0);

          priv->txdma = imxrt_dmach_alloc(priv->txch | DMAMUX_CHCFG_ENBL,
                                            0);
          priv->rxdma = imxrt_dmach_alloc(priv->rxch | DMAMUX_CHCFG_ENBL,
                                            0);
          DEBUGASSERT(priv->rxdma && priv->txdma);
        }
    }
  else
    {
      priv->rxdma = NULL;
      priv->txdma = NULL;
    }
#endif

  leave_critical_section(flags);

  return (struct spi_dev_s *)priv;
}

#endif /* CONFIG_IMXRT_LPSPI1 || CONFIG_IMXRT_LPSPI2 || CONFIG_IMXRT_LPSPI3 || CONFIG_IMXRT_LPSPI4 */

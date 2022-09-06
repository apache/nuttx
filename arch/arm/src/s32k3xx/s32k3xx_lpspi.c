/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_lpspi.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * The external functions, s32k3xx_lpspi0/1/2select and
 * s32k3xx_lpspi0/1/2status must be provided by board-specific logic.  They
 * are implementations of the select and status methods of the SPI interface
 * defined by struct s32k3xx_lpspi_ops_s (see include/nuttx/spi/spi.h).
 * All other methods (including s32k3xx_lpspibus_initialize()) are provided
 * by common S32K3XX logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in s32k3xx_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide s32k3xx_lpspi0/1/2select() and s32k3xx_lpspi0/1/2status()
 *      functions in your board-specific logic.  These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. Add a calls to s32k3xx_lpspibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by s32k3xx_lpspibus_initialize() may then be
 *      used to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_lpspislotinitialize(), for example, will bind the SPI driver
 *      to the SPI MMC/SD driver).
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
#include <nuttx/power/pm.h>

#include "arm_internal.h"

#include "chip.h"

#include "s32k3xx_pin.h"
#include "hardware/s32k3xx_pinmux.h"
#include "hardware/s32k3xx_lpspi.h"
#include "s32k3xx_clockconfig.h"
#include "s32k3xx_lpspi.h"

#ifdef CONFIG_S32K3XX_LPSPI_DMA
#include "hardware/s32k3xx_dmamux.h"
#include "s32k3xx_edma.h"
#endif

#include <arch/board/board.h>

#if defined(CONFIG_S32K3XX_LPSPI0) || defined(CONFIG_S32K3XX_LPSPI1) || \
    defined(CONFIG_S32K3XX_LPSPI2) || defined(CONFIG_S32K3XX_LPSPI3) || \
    defined(CONFIG_S32K3XX_LPSPI4) || defined(CONFIG_S32K3XX_LPSPI5)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SPI interrupts */

#ifdef CONFIG_S32K3XX_LPSPI_INTERRUPTS
#  error "Interrupt driven SPI not yet supported"
#endif

/* Can't have both interrupt driven SPI and SPI DMA */

#if defined(CONFIG_S32K3XX_LPSPI_INTERRUPTS) && defined(CONFIG_S32K3XX_LPSPI_DMA)
#  error "Cannot enable both interrupt mode and DMA mode for SPI"
#endif

#define  SPI_SR_CLEAR   (LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF  | \
                         LPSPI_SR_TEF | LPSPI_SR_REF | LPSPI_SR_DMF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct s32k3xx_lpspidev_s
{
  struct spi_dev_s spidev;    /* Externally visible part of the SPI interface */
  uint32_t spibase;           /* SPIn base address */
#ifdef CONFIG_S32K3XX_LPSPI_INTERRUPTS
  uint8_t spiirq;             /* SPI IRQ number */
#endif
  mutex_t lock;               /* Held while chip is selected for mutual exclusion */
  uint32_t frequency;         /* Requested clock frequency */
  uint32_t actual;            /* Actual clock frequency */
  int8_t nbits;               /* Width of word in bits */
  uint8_t mode;               /* Mode 0,1,2,3 */
  uint32_t pincfg;            /* Input output port configuration */
#ifdef CONFIG_S32K3XX_LPSPI_DMA
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

enum s32k3xx_delay_e
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
  s32k3xx_lpspi_getreg32(struct s32k3xx_lpspidev_s *priv, uint8_t offset);
static inline void s32k3xx_lpspi_putreg32(struct s32k3xx_lpspidev_s *priv,
                                          uint8_t offset, uint32_t value);
static inline uint32_t
  s32k3xx_lpspi_readword(struct s32k3xx_lpspidev_s *priv);
static inline void s32k3xx_lpspi_writeword(struct s32k3xx_lpspidev_s *priv,
                                           uint32_t byte);
static inline uint16_t
  s32k3xx_lpspi_9to16bitmode(struct s32k3xx_lpspidev_s *priv);
static uint32_t s32k3xx_lpspi_pckfreq(uintptr_t base);
static inline uint32_t
  s32k3xx_lpspi_set_delays(struct s32k3xx_lpspidev_s *priv,
                           uint32_t delay_ns, enum s32k3xx_delay_e type);
static inline uint32_t
  s32k3xx_lpspi_set_delay_scaler(struct s32k3xx_lpspidev_s *priv,
                                 uint32_t scaler, enum s32k3xx_delay_e type);

/* DMA support */

#ifdef CONFIG_S32K3XX_LPSPI_DMA
static int         spi_dmarxwait(struct s32k3xx_lpspidev_s *priv);
static int         spi_dmatxwait(struct s32k3xx_lpspidev_s *priv);
static inline void spi_dmarxwakeup(struct s32k3xx_lpspidev_s *priv);
static inline void spi_dmatxwakeup(struct s32k3xx_lpspidev_s *priv);
static void        spi_dmarxcallback(DMACH_HANDLE handle, void *arg,
                                     bool done, int result);
static void        spi_dmatxcallback(DMACH_HANDLE handle, void *arg,
                                     bool done, int result);
static inline void spi_dmarxstart(struct s32k3xx_lpspidev_s *priv);
static inline void spi_dmatxstart(struct s32k3xx_lpspidev_s *priv);
#endif

/* SPI methods */

static int s32k3xx_lpspi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t s32k3xx_lpspi_setfrequency(struct spi_dev_s *dev,
                                           uint32_t frequency);
static void s32k3xx_lpspi_setmode(struct spi_dev_s *dev,
                                  enum spi_mode_e mode);
static void s32k3xx_lpspi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int s32k3xx_lpspi_hwfeatures(struct spi_dev_s *dev,
                                    s32k3xx_lpspi_hwfeatures_t features);
#endif
static uint32_t s32k3xx_lpspi_send(struct spi_dev_s *dev, uint32_t wd);
static void s32k3xx_lpspi_exchange(struct spi_dev_s *dev,
                                   const void *txbuffer, void *rxbuffer,
                                   size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void s32k3xx_lpspi_sndblock(struct spi_dev_s *dev,
                                   const void *txbuffer, size_t nwords);
static void s32k3xx_lpspi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                                    size_t nwords);
#endif

/* Initialization */

static void s32k3xx_lpspi_bus_initialize(struct s32k3xx_lpspidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_LPSPI0
static const struct spi_ops_s g_spi0ops =
{
  .lock         = s32k3xx_lpspi_lock,
  .select       = s32k3xx_lpspi0select,
  .setfrequency = s32k3xx_lpspi_setfrequency,
  .setmode      = s32k3xx_lpspi_setmode,
  .setbits      = s32k3xx_lpspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = s32k3xx_lpspi_hwfeatures,
#endif
  .status       = s32k3xx_lpspi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = s32k3xx_lpspi0cmddata,
#endif
  .send         = s32k3xx_lpspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = s32k3xx_lpspi_exchange,
#else
  .sndblock     = s32k3xx_lpspi_sndblock,
  .recvblock    = s32k3xx_lpspi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = s32k3xx_lpspi0register,  /* Provided externally */
#else
  .registercallback = 0,                       /* Not implemented */
#endif
};

static struct s32k3xx_lpspidev_s g_lpspi0dev =
{
  .spidev       =
  {
    &g_spi0ops
  },
  .spibase      = S32K3XX_LPSPI0_BASE,
#ifdef CONFIG_S32K3XX_LPSPI_INTERRUPTS
  .spiirq       = S32K3XX_IRQ_LPSPI0,
#endif
#ifdef CONFIG_S32K3XX_LPSPI0_DMA
  .rxch         = DMA_REQ_LPSPI0_RX,
  .txch         = DMA_REQ_LPSPI0_TX,
#endif
  .pincfg       = CONFIG_S32K3XX_LPSPI0_PINCFG,
};
#endif

#ifdef CONFIG_S32K3XX_LPSPI1
static const struct spi_ops_s g_spi1ops =
{
  .lock         = s32k3xx_lpspi_lock,
  .select       = s32k3xx_lpspi1select,
  .setfrequency = s32k3xx_lpspi_setfrequency,
  .setmode      = s32k3xx_lpspi_setmode,
  .setbits      = s32k3xx_lpspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = s32k3xx_lpspi_hwfeatures,
#endif
  .status       = s32k3xx_lpspi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = s32k3xx_lpspi1cmddata,
#endif
  .send         = s32k3xx_lpspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = s32k3xx_lpspi_exchange,
#else
  .sndblock     = s32k3xx_lpspi_sndblock,
  .recvblock    = s32k3xx_lpspi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = s32k3xx_lpspi1register,  /* Provided externally */
#else
  .registercallback = 0,                       /* Not implemented */
#endif
};

static struct s32k3xx_lpspidev_s g_lpspi1dev =
{
  .spidev       =
  {
    &g_spi1ops
  },
  .spibase      = S32K3XX_LPSPI1_BASE,
#ifdef CONFIG_S32K3XX_LPSPI_INTERRUPTS
  .spiirq       = S32K3XX_IRQ_LPSPI1,
#endif
#ifdef CONFIG_S32K3XX_LPSPI1_DMA
  .rxch         = DMA_REQ_LPSPI1_RX,
  .txch         = DMA_REQ_LPSPI1_TX,
#endif
  .pincfg       = CONFIG_S32K3XX_LPSPI1_PINCFG,
};
#endif

#ifdef CONFIG_S32K3XX_LPSPI2
static const struct spi_ops_s g_spi2ops =
{
  .lock         = s32k3xx_lpspi_lock,
  .select       = s32k3xx_lpspi2select,
  .setfrequency = s32k3xx_lpspi_setfrequency,
  .setmode      = s32k3xx_lpspi_setmode,
  .setbits      = s32k3xx_lpspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = s32k3xx_lpspi_hwfeatures,
#endif
  .status       = s32k3xx_lpspi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = s32k3xx_lpspi2cmddata,
#endif
  .send         = s32k3xx_lpspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = s32k3xx_lpspi_exchange,
#else
  .sndblock     = s32k3xx_lpspi_sndblock,
  .recvblock    = s32k3xx_lpspi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = s32k3xx_lpspi2register,  /* Provided externally */
#else
  .registercallback = 0,                       /* Not implemented */
#endif
};

static struct s32k3xx_lpspidev_s g_lpspi2dev =
{
  .spidev       =
  {
    &g_spi2ops
  },
  .spibase      = S32K3XX_LPSPI2_BASE,
#ifdef CONFIG_S32K3XX_LPSPI_INTERRUPTS
  .spiirq       = S32K3XX_IRQ_LPSPI2,
#endif
#ifdef CONFIG_S32K3XX_LPSPI2_DMA
  .rxch         = DMA_REQ_LPSPI2_RX,
  .txch         = DMA_REQ_LPSPI2_TX,
#endif
  .pincfg       = CONFIG_S32K3XX_LPSPI2_PINCFG,
};
#endif

#ifdef CONFIG_S32K3XX_LPSPI3
static const struct spi_ops_s g_spi3ops =
{
  .lock         = s32k3xx_lpspi_lock,
  .select       = s32k3xx_lpspi3select,
  .setfrequency = s32k3xx_lpspi_setfrequency,
  .setmode      = s32k3xx_lpspi_setmode,
  .setbits      = s32k3xx_lpspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = s32k3xx_lpspi_hwfeatures,
#endif
  .status       = s32k3xx_lpspi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = s32k3xx_lpspi3cmddata,
#endif
  .send         = s32k3xx_lpspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = s32k3xx_lpspi_exchange,
#else
  .sndblock     = s32k3xx_lpspi_sndblock,
  .recvblock    = s32k3xx_lpspi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = s32k3xx_lpspi3register,  /* Provided externally */
#else
  .registercallback = 0,                       /* Not implemented */
#endif
};

static struct s32k3xx_lpspidev_s g_lpspi3dev =
{
  .spidev       =
  {
    &g_spi3ops
  },
  .spibase      = S32K3XX_LPSPI3_BASE,
#ifdef CONFIG_S32K3XX_LPSPI_INTERRUPTS
  .spiirq       = S32K3XX_IRQ_LPSPI3,
#endif
#ifdef CONFIG_S32K3XX_LPSPI3_DMA
  .rxch         = DMA_REQ_LPSPI3_RX,
  .txch         = DMA_REQ_LPSPI3_TX,
#endif
  .pincfg       = CONFIG_S32K3XX_LPSPI3_PINCFG,
};
#endif

#ifdef CONFIG_S32K3XX_LPSPI4
static const struct spi_ops_s g_spi4ops =
{
  .lock         = s32k3xx_lpspi_lock,
  .select       = s32k3xx_lpspi4select,
  .setfrequency = s32k3xx_lpspi_setfrequency,
  .setmode      = s32k3xx_lpspi_setmode,
  .setbits      = s32k3xx_lpspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = s32k3xx_lpspi_hwfeatures,
#endif
  .status       = s32k3xx_lpspi4status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = s32k3xx_lpspi4cmddata,
#endif
  .send         = s32k3xx_lpspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = s32k3xx_lpspi_exchange,
#else
  .sndblock     = s32k3xx_lpspi_sndblock,
  .recvblock    = s32k3xx_lpspi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = s32k3xx_lpspi4register,  /* Provided externally */
#else
  .registercallback = 0,                       /* Not implemented */
#endif
};

static struct s32k3xx_lpspidev_s g_lpspi4dev =
{
  .spidev       =
  {
    &g_spi4ops
  },
  .spibase      = S32K3XX_LPSPI4_BASE,
#ifdef CONFIG_S32K3XX_LPSPI_INTERRUPTS
  .spiirq       = S32K3XX_IRQ_LPSPI4,
#endif
#ifdef CONFIG_S32K3XX_LPSPI4_DMA
  .rxch         = DMA_REQ_LPSPI4_RX,
  .txch         = DMA_REQ_LPSPI4_TX,
#endif
  .pincfg       = CONFIG_S32K3XX_LPSPI4_PINCFG,
};
#endif

#ifdef CONFIG_S32K3XX_LPSPI5
static const struct spi_ops_s g_spi5ops =
{
  .lock         = s32k3xx_lpspi_lock,
  .select       = s32k3xx_lpspi5select,
  .setfrequency = s32k3xx_lpspi_setfrequency,
  .setmode      = s32k3xx_lpspi_setmode,
  .setbits      = s32k3xx_lpspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = s32k3xx_lpspi_hwfeatures,
#endif
  .status       = s32k3xx_lpspi5status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = s32k3xx_lpspi5cmddata,
#endif
  .send         = s32k3xx_lpspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = s32k3xx_lpspi_exchange,
#else
  .sndblock     = s32k3xx_lpspi_sndblock,
  .recvblock    = s32k3xx_lpspi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = s32k3xx_lpspi5register,  /* Provided externally */
#else
  .registercallback = 0,                       /* Not implemented */
#endif
};

static struct s32k3xx_lpspidev_s g_lpspi5dev =
{
  .spidev       =
  {
    &g_spi5ops
  },
  .spibase      = S32K3XX_LPSPI5_BASE,
#ifdef CONFIG_S32K3XX_LPSPI_INTERRUPTS
  .spiirq       = S32K3XX_IRQ_LPSPI5,
#endif
#ifdef CONFIG_S32K3XX_LPSPI5_DMA
  .rxch         = DMA_REQ_LPSPI5_RX,
  .txch         = DMA_REQ_LPSPI5_TX,
#endif
  .pincfg       = CONFIG_S32K3XX_LPSPI5_PINCFG,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_modifyreg
 *
 * Description:
 *   Atomic modification of the 32-bit contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv      - private SPI device structure
 *   offset    - offset to the register of interest
 *   clearbits - bits to clear
 *   clearbits - bits to set
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_LPSPI_DMA
static inline void spi_modifyreg(struct s32k3xx_lpspidev_s *priv,
                                 uint8_t offset, uint32_t clearbits,
                                 uint32_t setbits)
{
  modifyreg32(priv->spibase + offset, clearbits, setbits);
}
#endif

/****************************************************************************
 * Name: s32k3xx_lpspi_getreg8
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

static inline uint8_t s32k3xx_lpspi_getreg8(struct s32k3xx_lpspidev_s *priv,
                                            uint8_t offset)
{
  return getreg8(priv->spibase + offset);
}

/****************************************************************************
 * Name: s32k3xx_lpspi_putreg8
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

static inline void s32k3xx_lpspi_putreg8(struct s32k3xx_lpspidev_s *priv,
                                         uint8_t offset, uint8_t value)
{
  putreg8(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: s32k3xx_lpspi_getreg
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
  s32k3xx_lpspi_getreg32(struct s32k3xx_lpspidev_s *priv, uint8_t offset)
{
  return getreg32(priv->spibase + offset);
}

/****************************************************************************
 * Name: s32k3xx_lpspi_putreg
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

static inline void s32k3xx_lpspi_putreg32(struct s32k3xx_lpspidev_s *priv,
                                          uint8_t offset, uint32_t value)
{
  putreg32(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: s32k3xx_lpspi_readword
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
  s32k3xx_lpspi_readword(struct s32k3xx_lpspidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  while ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_SR_OFFSET) &
          LPSPI_SR_RDF) == 0)
    {
    }

  /* Then return the received byte */

  return (uint32_t) s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_RDR_OFFSET);
}

/****************************************************************************
 * Name: s32k3xx_lpspi_writeword
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

static inline void s32k3xx_lpspi_writeword(struct s32k3xx_lpspidev_s *priv,
                                           uint32_t word)
{
  /* Wait until the transmit buffer is empty */

  while ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_SR_OFFSET) &
          LPSPI_SR_TDF) == 0)
    {
    }

  /* Then send the word */

  s32k3xx_lpspi_putreg32(priv, S32K3XX_LPSPI_TDR_OFFSET, word);
}

/****************************************************************************
 * Name: s32k3xx_lpspi_write_dword
 *
 * Description:
 *   Write two words to SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   word0, word1 - words to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_LPSPI_DWORD

static inline void s32k3xx_lpspi_write_dword(struct s32k3xx_lpspidev_s *priv,
                                             uint32_t word0, uint32_t word1)
{
  /* Wait until the transmit buffer is empty */

  while ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_SR_OFFSET)
         & LPSPI_SR_TDF) == 0)
    {
    }

  /* Then send the words, use the FIFO */

  s32k3xx_lpspi_putreg32(priv, S32K3XX_LPSPI_TDR_OFFSET, word0);
  s32k3xx_lpspi_putreg32(priv, S32K3XX_LPSPI_TDR_OFFSET, word1);
}

#endif

/****************************************************************************
 * Name: s32k3xx_lpspi_readbyte
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

static inline uint8_t s32k3xx_lpspi_readbyte(struct s32k3xx_lpspidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  while ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_SR_OFFSET) &
          LPSPI_SR_RDF) == 0)
    {
    }

  /* Then return the received byte */

  return s32k3xx_lpspi_getreg8(priv, S32K3XX_LPSPI_RDR_OFFSET);
}

/****************************************************************************
 * Name: s32k3xx_lpspi_writebyte
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

static inline void s32k3xx_lpspi_writebyte(struct s32k3xx_lpspidev_s *priv,
                                           uint8_t byte)
{
  /* Wait until the transmit buffer is empty */

  while ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_SR_OFFSET) &
          LPSPI_SR_TDF) == 0)
    {
    }

  /* Then send the byte */

  s32k3xx_lpspi_putreg8(priv, S32K3XX_LPSPI_TDR_OFFSET, byte);
}

/****************************************************************************
 * Name: s32k3xx_lpspi_9to16bitmode
 *
 * Description:
 *   Check if the SPI is operating in more then 8 bit mode
 *   On the S32K the frame size can grow to 4096 bit/frame
 *
 * Input Parameters:
 *   priv     - Device-specific state data
 *
 * Returned Value:
 *   value: frame size
 *
 ****************************************************************************/

static inline uint16_t
  s32k3xx_lpspi_9to16bitmode(struct s32k3xx_lpspidev_s *priv)
{
  uint16_t ret;

  ret = ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_TCR_OFFSET) &
        LPSPI_TCR_FRAMESZ_MASK) + 1);
  return ret;
}

/****************************************************************************
 * Name: s32k3xx_lpspi_modifyreg
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

static void s32k3xx_lpspi_modifyreg32(struct s32k3xx_lpspidev_s *priv,
                                      uint8_t offset, uint32_t clrbits,
                                      uint32_t setbits)
{
  modifyreg32(priv->spibase + offset, clrbits, setbits);
}

/****************************************************************************
 * Name: s32k3xx_lpspi_pckfreq
 *
 * Description:
 *   Get the peripheral clock frequency for the LPSPI peripheral
 *
 * Input Parameters:
 *   base - The base address of the LPSPI peripheral registers
 *
 * Returned Value:
 *   The frequency of the LPSPI functional input frequency
 *   (or zero on a failure)
 *
 ****************************************************************************/

static uint32_t s32k3xx_lpspi_pckfreq(uintptr_t base)
{
  uint32_t clkfreq;

#ifdef CONFIG_S32K3XX_LPSPI0
  if (base == S32K3XX_LPSPI0_BASE)
    {
      clkfreq = s32k3xx_get_freq(AIPS_PLAT_CLK);
    }
  else
#endif
    {
      clkfreq = s32k3xx_get_freq(AIPS_SLOW_CLK);
    }

  DEBUGASSERT(clkfreq >= 0);

  return clkfreq;
}

/****************************************************************************
 * Name: s32k3xx_lpspi_set_delays
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

static inline uint32_t
  s32k3xx_lpspi_set_delay_scaler(struct s32k3xx_lpspidev_s *priv,
                                 uint32_t scaler, enum s32k3xx_delay_e type)
{
  switch (type)
    {
    case LPSPI_PCS_TO_SCK:
      return LPSPI_CCR_PCSSCK(scaler);
      break;

    case LPSPI_LAST_SCK_TO_PCS:
      return LPSPI_CCR_SCKPCS(scaler);
      break;

    case LPSPI_BETWEEN_TRANSFER:
      return LPSPI_CCR_DBT(scaler);
      break;
    }

  return 0;
}

/****************************************************************************
 * Name: s32k3xx_lpspi_set_delays
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

static inline uint32_t
  s32k3xx_lpspi_set_delays(struct s32k3xx_lpspidev_s *priv,
                           uint32_t delay_ns, enum s32k3xx_delay_e type)
{
  uint32_t inclock;
  uint64_t real_delay;
  uint64_t best_delay;
  uint32_t scaler;
  uint32_t best_scaler;
  uint32_t diff;
  uint32_t min_diff;
  uint64_t initial_delay_ns;
  uint32_t clock_div_prescaler;
  uint32_t additional_scaler;

  /* Get the frequency of the LPSPI functional input clock */

  inclock = s32k3xx_lpspi_pckfreq(priv->spibase);
  DEBUGASSERT(inclock != 0);

  /* Get the pre-scaled input clock */

  clock_div_prescaler = inclock /
              (1 << ((s32k3xx_lpspi_getreg32(priv,
                      S32K3XX_LPSPI_TCR_OFFSET) &
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

      /* Calculate the maximum delay */

      best_delay = 1000000000U;

      /* based on DBT+2, or 255 + 2 */

      best_delay *= 257;
      best_delay /= clock_div_prescaler;

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

      /* Calculate the maximum delay */

      best_delay = 1000000000U;

      /* Based on SCKPCS+1 or PCSSCK+1, or 255 + 1 */

      best_delay *= 256;
      best_delay /= clock_div_prescaler;

      additional_scaler = 0;
    }

  /* If the initial, default delay is already greater than the desired delay,
   * then * set the delay to their initial value (0) and return the delay. In
   * other words, * there is no way to decrease the delay value further.
   */

  if (initial_delay_ns >= delay_ns)
    {
      return s32k3xx_lpspi_set_delay_scaler(priv, 0, type);
    }
  else
    {
      /* If min_diff = 0, the exit for loop */

      for (scaler = 0; (scaler < 256) && min_diff; scaler++)
        {
          /* Calculate the real delay value as we cycle through the scaler
           * values. Due to large size of calculated values (uint64_t),
           * we need to break up the calculation into several steps to ensure
           * accurate calculated results
           */

          real_delay  = 1000000000U;
          real_delay *= (scaler + 1 + additional_scaler);
          real_delay /= clock_div_prescaler;

          /* calculate the delay difference based on the conditional
           * statement that states that the calculated delay must not be less
           * then the desired delay
           */

          if (real_delay >= delay_ns)
            {
              diff = real_delay - delay_ns;
              if (min_diff > diff)
                {
                  /* A better match found */

                  min_diff = diff;
                  best_scaler = scaler;
                  best_delay = real_delay;
                }
            }
        }

      return s32k3xx_lpspi_set_delay_scaler(priv, best_scaler, type);
    }
}

/****************************************************************************
 * Name: s32k3xx_lpspi_lock
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

static int s32k3xx_lpspi_lock(struct spi_dev_s *dev, bool lock)
{
  struct s32k3xx_lpspidev_s *priv = (struct s32k3xx_lpspidev_s *)dev;
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
 * Name: s32k3xx_lpspi_setfrequency
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

static uint32_t s32k3xx_lpspi_setfrequency(struct spi_dev_s *dev,
                                           uint32_t frequency)
{
  struct s32k3xx_lpspidev_s *priv = (struct s32k3xx_lpspidev_s *)dev;

  uint32_t men;
  uint32_t regval;
  uint32_t inclock;
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

      men = s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_CR_OFFSET) &
                                   LPSPI_CR_MEN;
      if (men)
        {
          s32k3xx_lpspi_modifyreg32(priv, S32K3XX_LPSPI_CR_OFFSET,
                                    LPSPI_CR_MEN, 0);
        }

      /* Get the frequency of the LPSPI functional input clock */

      inclock = s32k3xx_lpspi_pckfreq(priv->spibase);
      DEBUGASSERT(inclock != 0);

      min_diff       = 0xffffffff;
      best_prescaler = 7;
      best_scaler    = 255;
      best_frequency = 0;

      for (prescaler = 0; (prescaler < 8) && min_diff; prescaler++)
        {
          for (scaler = 0; (scaler < 256) && min_diff; scaler++)
            {
              real_frequency = inclock / ((1 << prescaler) * (scaler + 2));

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

      s32k3xx_lpspi_modifyreg32(priv, S32K3XX_LPSPI_TCR_OFFSET,
                              LPSPI_TCR_PRESCALE_MASK,
                              LPSPI_TCR_PRESCALE(best_prescaler));

      priv->frequency = frequency;
      priv->actual = best_frequency;

      regval = 0x0;

      regval |= s32k3xx_lpspi_set_delays(priv, 1000000000 / best_frequency,
                                    LPSPI_PCS_TO_SCK);
      regval |= s32k3xx_lpspi_set_delays(priv, 1000000000 / best_frequency,
                                    LPSPI_LAST_SCK_TO_PCS);
      regval |= s32k3xx_lpspi_set_delays(priv, 1000000000 / best_frequency,
                                    LPSPI_BETWEEN_TRANSFER);

      /* Write the best values in the CCR register */

      regval &= ~LPSPI_CCR_SCKDIV_MASK;
      regval |= LPSPI_CCR_SCKDIV(best_scaler);
      s32k3xx_lpspi_putreg32(priv, S32K3XX_LPSPI_CCR_OFFSET, regval);

      /* Re-enable LPSPI if it was enabled previously */

      if (men)
        {
          s32k3xx_lpspi_modifyreg32(priv, S32K3XX_LPSPI_CR_OFFSET, 0,
                                  LPSPI_CR_MEN);
        }
    }

  return priv->actual;
}

/****************************************************************************
 * Name: s32k3xx_lpspi_setmode
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

static void s32k3xx_lpspi_setmode(struct spi_dev_s *dev,
                                  enum spi_mode_e mode)
{
  struct s32k3xx_lpspidev_s *priv = (struct s32k3xx_lpspidev_s *)dev;
  uint32_t setbits;
  uint32_t clrbits;
  uint32_t men;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Disable LPSPI if it is enabled */

      men = s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_CR_OFFSET) &
                                   LPSPI_CR_MEN;
      if (men)
        {
          s32k3xx_lpspi_modifyreg32(priv, S32K3XX_LPSPI_CR_OFFSET,
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

      s32k3xx_lpspi_modifyreg32(priv, S32K3XX_LPSPI_TCR_OFFSET,
                                clrbits, setbits);

      while ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_RSR_OFFSET) &
              LPSPI_RSR_RXEMPTY) != LPSPI_RSR_RXEMPTY)
        {
          /* Flush SPI read FIFO */

          s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_RSR_OFFSET);
        }

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;

      /* Re-enable LPSPI if it was enabled previously */

      if (men)
        {
          s32k3xx_lpspi_modifyreg32(priv, S32K3XX_LPSPI_CR_OFFSET,
                                    0, LPSPI_CR_MEN);
        }
    }
}

/****************************************************************************
 * Name: s32k3xx_lpspi_setbits
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

static void s32k3xx_lpspi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct s32k3xx_lpspidev_s *priv = (struct s32k3xx_lpspidev_s *)dev;
  uint32_t regval;
  uint32_t men;
  int savbits = nbits;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      if (nbits < 2 || nbits > 4096)
        {
          return;
        }

      /* Disable LPSPI if it is enabled */

      men = s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_CR_OFFSET) &
                                   LPSPI_CR_MEN;
      if (men)
        {
          s32k3xx_lpspi_modifyreg32(priv, S32K3XX_LPSPI_CR_OFFSET,
                                    LPSPI_CR_MEN, 0);
        }

      regval = s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_TCR_OFFSET);
      regval &= ~LPSPI_TCR_FRAMESZ_MASK;
      regval |= LPSPI_TCR_FRAMESZ(nbits - 1);

      s32k3xx_lpspi_putreg32(priv, S32K3XX_LPSPI_TCR_OFFSET, regval);

      /* Save the selection so that subsequent re-configurations will
       * be faster.
       */

      priv->nbits = savbits;    /* nbits has been clobbered... save the signed
                                 * value. */

      /* Re-enable LPSPI if it was enabled previously */

      if (men)
        {
          s32k3xx_lpspi_modifyreg32(priv, S32K3XX_LPSPI_CR_OFFSET,
                                    0, LPSPI_CR_MEN);
        }
    }
}

/****************************************************************************
 * Name: s32k3xx_lpspi_hwfeatures
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
static int s32k3xx_lpspi_hwfeatures(struct spi_dev_s *dev,
                                    s32k3xx_lpspi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
  struct s32k3xx_lpspidev_s *priv = (struct s32k3xx_lpspidev_s *)dev;
  uint32_t setbits;
  uint32_t clrbits;
  int savbits = nbits;

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

  s32k3xx_lpspi_modifyreg32(priv, S32K3XX_LPSPI_TCR_OFFSET,
                            clrbits, setbits);

  /* Other H/W features are not supported */

  return ((features & ~HWFEAT_LSBFIRST) == 0) ? OK : -ENOSYS;
#else
  return -ENOSYS;
#endif
}
#endif

/****************************************************************************
 * Name: s32k3xx_lpspi_send
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

static uint32_t s32k3xx_lpspi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct s32k3xx_lpspidev_s *priv = (struct s32k3xx_lpspidev_s *)dev;
  uint32_t regval;
  uint32_t ret;

  DEBUGASSERT(priv && priv->spibase);

  s32k3xx_lpspi_writeword(priv, wd);

  while ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_SR_OFFSET) &
          LPSPI_SR_RDF) != LPSPI_SR_RDF);

  ret = s32k3xx_lpspi_readword(priv);

  /* Check and clear any error flags (Reading from the SR clears the error
   * flags).
   */

  regval = s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_SR_OFFSET);

  spiinfo("Sent: %04" PRIx32 " Return: %04" PRIx32
          " Status: %02" PRIx32 "\n", wd, ret, regval);

  UNUSED(regval);
  return ret;
}

/****************************************************************************
 * Name: s32k3xx_lpspi_send_dword
 *
 * Description:
 *   Exchange two words on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd0, wd1  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_LPSPI_DWORD
static uint32_t s32k3xx_lpspi_send_dword(struct spi_dev_s *dev, uint32_t wd0,
                                         uint32_t wd1, uint32_t *rw1)
{
  struct s32k3xx_lpspidev_s *priv = (struct s32k3xx_lpspidev_s *)dev;
  uint32_t regval;
  uint32_t ret;

  DEBUGASSERT(priv && priv->spibase);

  /* check if the receive buffer is empty, if not clear it */

  while ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_SR_OFFSET)
         & LPSPI_SR_RDF))
    {
      s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_RDR_OFFSET);
    }

  s32k3xx_lpspi_write_dword(priv, wd0, wd1);

  while ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_SR_OFFSET)
         & LPSPI_SR_RDF) != LPSPI_SR_RDF);

  ret  = s32k3xx_lpspi_readword(priv);
  *rw1 = s32k3xx_lpspi_readword(priv);

  /* Check and clear any error flags (Reading from the SR clears the error
   * flags).
   */

  regval = s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_SR_OFFSET);

  spiinfo("Sent: %04" PRIx32 " %04" PRIx32 " Return: %04"
          PRIx32 " %04" PRIx32 " Status: %02" PRIx32 "\n",
          wd0, wd1, ret, *rw1, regval);

  UNUSED(regval);
  return ret;
}

#endif

/****************************************************************************
 * Name: s32k3xx_lpspi_exchange (no DMA).  aka s32k3xx_lpspi_exchange_nodma
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
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if !defined(CONFIG_S32K3XX_LPSPI_DMA)
static void s32k3xx_lpspi_exchange(struct spi_dev_s *dev,
                                   const void *txbuffer, void *rxbuffer,
                                   size_t nwords)
#else
static void s32k3xx_lpspi_exchange_nodma(struct spi_dev_s *dev,
                                         const void *txbuffer,
                                         void *rxbuffer, size_t nwords)
#endif
{
  struct s32k3xx_lpspidev_s *priv = (struct s32k3xx_lpspidev_s *)dev;
  uint16_t framesize;
  DEBUGASSERT(priv && priv->spibase);

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* bit mode? */

  framesize = s32k3xx_lpspi_9to16bitmode(priv);
  if (framesize > 16 && framesize % 32 != 0)
    {
      /* 17-bit or higher, byte transfer due to padding
       * take care of big endian mode of hardware !!
       */

      const uint8_t *src = (const uint8_t *)txbuffer;
      uint8_t *dest = (uint8_t *) rxbuffer;
      uint32_t word = 0x0;
#ifdef CONFIG_S32K3XX_LPSPI_DWORD
      uint32_t word1 = 0x0;
      uint32_t rword1;
      bool     dwords = false;
#endif

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              /* read the required number of bytes */

            switch (framesize)
              {
#ifdef CONFIG_S32K3XX_LPSPI_DWORD
              case 40:
                   word = (src[0] << 24) + (src[1] << 16)
                          + (src[2] << 8) + src[3];
                   word1 = src[4];
                   src += 5;
                   dwords = true;
                   break;
#endif
              default:
                      break;
              }
            }
          else
            {
              word = 0xffffffff;
            }

#ifdef CONFIG_S32K3XX_LPSPI_DWORD
          /* Exchange 2 words */

          if (dwords)
            {
              word = s32k3xx_lpspi_send_dword(dev, word, word1, &rword1);
            }
          else
#endif
            {
              word = s32k3xx_lpspi_send(dev, word);
            }

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
            switch (framesize)
              {
#ifdef CONFIG_S32K3XX_LPSPI_DWORD
              case 40:
                   dest[0] = (word >> 24) & 0xff;
                   dest[1] = (word >> 16) & 0xff;
                   dest[2] = (word >>  8) & 0xff;
                   dest[3] =  word        & 0xff;
                   dest[4] =  rword1      & 0xff;
                   dest += 5;
                   break;
#endif

              default:

                      break;
            }
          }
        }
    }
  else if (framesize > 16)
    {
      /* 32-bit or 64 bit, word size memory transfers */

      const uint32_t *src = (const uint32_t *)txbuffer;
      uint32_t *dest = (uint32_t *) rxbuffer;
      uint32_t word = 0x0;
#ifdef CONFIG_S32K3XX_LPSPI_DWORD
      uint32_t word1 = 0x0;
      uint32_t rword1;
      bool     dwords = false;
#endif

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              /* read the required number of bytes */

            switch (framesize)
              {
              case 32:
                   word = __builtin_bswap32(*src);
                   src += 4;
                   break;
#ifdef CONFIG_S32K3XX_LPSPI_DWORD
              case 64:
                   word  = __builtin_bswap32(src[0]);
                   word1 = __builtin_bswap32(src[1]);
                   src += 8;
                   dwords = true;
#endif
              default:
                      break;
              }
            }
          else
            {
              word = 0xffffffff;
            }

#ifdef CONFIG_S32K3XX_LPSPI_DWORD
          /* Exchange 2 words */

          if (dwords)
            {
              word = s32k3xx_lpspi_send_dword(dev, word, word1, &rword1);
            }
          else
#endif
            {
            word = s32k3xx_lpspi_send(dev, word);
            }

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
            switch (framesize)
              {
              case 32:
                   *dest = __builtin_bswap32(word);
                   dest += 4;
                   break;
#ifdef CONFIG_S32K3XX_LPSPI_DWORD
              case 64:
                   dest[0] = __builtin_bswap32(word);
                   dest[1] = __builtin_bswap32(rword1);
                   dest += 8;
                   break;
#endif

              default:

                      break;
            }
          }
        }
    }
  else if (framesize > 8)
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
              word = __builtin_bswap16(*src++);

              /* read the required number of bytes */
            }
          else
            {
              word = 0xffff;
            }

          /* Exchange one word */

          word = (uint16_t) s32k3xx_lpspi_send(dev, (uint32_t) word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = __builtin_bswap16(word);
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

          word = (uint8_t) s32k3xx_lpspi_send(dev, (uint32_t) word);

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

#ifdef CONFIG_S32K3XX_LPSPI_DMA
static void s32k3xx_lpspi_exchange(struct spi_dev_s *dev,
                                   const void *txbuffer, void *rxbuffer,
                                   size_t nwords)
{
  int                          ret;
  size_t                       adjust;
  ssize_t                      nbytes;
  static uint8_t               rxdummy[4] aligned_data(4);
  static const uint16_t        txdummy = 0xffff;
  uint32_t                     regval;
  struct s32k3xx_lpspidev_s *priv = (struct s32k3xx_lpspidev_s *)dev;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv && priv->spibase);
  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* Convert the number of word to a number of bytes */

  nbytes = (priv->nbits > 8) ? nwords << 2 : nwords;

  /* Invalid DMA channels fall back to non-DMA method. */

  if (priv->rxdma == NULL || priv->txdma == NULL
#ifdef CONFIG_S32K3XX_LPSPI_DMATHRESHOLD
      /* If this is a small SPI transfer, then let
       * s32k3xx_lpspi_exchange_nodma() do the work.
       */

      || nbytes <= CONFIG_S32K3XX_LPSPI_DMATHRESHOLD
#endif
      )
    {
      s32k3xx_lpspi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }

  /* ERR050456 workaround: Reset FIFOs using CR[RST] bit */

  regval = s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_CFGR1_OFFSET);

  spi_modifyreg(priv, S32K3XX_LPSPI_CR_OFFSET,
                LPSPI_CR_RTF | LPSPI_CR_RRF,
                LPSPI_CR_RTF | LPSPI_CR_RRF);

  s32k3xx_lpspi_putreg32(priv, S32K3XX_LPSPI_CFGR1_OFFSET, regval);

  /* Clear all status bits */

  s32k3xx_lpspi_putreg32(priv, S32K3XX_LPSPI_SR_OFFSET, SPI_SR_CLEAR);

  /* disable DMA */

  s32k3xx_lpspi_putreg32(priv, S32K3XX_LPSPI_DER_OFFSET, 0);

  /* Set up the DMA */

  adjust = (priv->nbits > 8) ? 2 : 1;

  struct s32k3xx_edma_xfrconfig_s config;

  config.saddr  = priv->spibase + S32K3XX_LPSPI_RDR_OFFSET;
  config.daddr  = (uint32_t) (rxbuffer ? rxbuffer : rxdummy);
  config.soff   = 0;
  config.doff   = rxbuffer ? adjust : 0;
  config.iter   = nbytes;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.dsize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.ttype  = EDMA_PERIPH2MEM;
  config.nbytes = adjust;
#ifdef CONFIG_KINETIS_EDMA_ELINK
  config.linkch = NULL;
#endif
  s32k3xx_dmach_xfrsetup(priv->rxdma, &config);

  config.saddr  = (uint32_t) (txbuffer ? txbuffer : &txdummy);
  config.daddr  = priv->spibase + S32K3XX_LPSPI_TDR_OFFSET;
  config.soff   = txbuffer ? adjust : 0;
  config.doff   = 0;
  config.iter   = nbytes;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.dsize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.ttype  = EDMA_MEM2PERIPH;
  config.nbytes = adjust;
#ifdef CONFIG_KINETIS_EDMA_ELINK
  config.linkch = NULL;
#endif
  s32k3xx_dmach_xfrsetup(priv->txdma, &config);

  /* Start the DMAs */

  spi_dmarxstart(priv);
  spi_dmatxstart(priv);

  /* Invoke SPI DMA */

  spi_modifyreg(priv, S32K3XX_LPSPI_DER_OFFSET,
                0, LPSPI_DER_TDDE | LPSPI_DER_RDDE);

  /* Then wait for each to complete */

  ret = spi_dmarxwait(priv);

  if (ret < 0)
    {
      ret = spi_dmatxwait(priv);
    }

  /* Reset any status */

  s32k3xx_lpspi_putreg32(priv, S32K3XX_LPSPI_SR_OFFSET,
                         s32k3xx_lpspi_getreg32(priv,
                                                S32K3XX_LPSPI_SR_OFFSET));

  /* Disable DMA */

  s32k3xx_lpspi_putreg32(priv, S32K3XX_LPSPI_DER_OFFSET, 0);

  up_invalidate_dcache((uintptr_t)rxbuffer,
                           (uintptr_t)rxbuffer + nbytes);
}

#endif  /* CONFIG_S32K3XX_SPI_DMA */

/****************************************************************************
 * Name: s32k3xx_lpspi_sndblock
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
static void s32k3xx_lpspi_sndblock(struct spi_dev_s *dev,
                                   const void *txbuffer, size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return s32k3xx_lpspi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: s32k3xx_lpspi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that can be received in the buffer in
 *              number of words.  The wordsize is determined by the number
 *              of bits-per-word selected for the SPI interface.  If
 *              nbits <= 8, the data is packed into uint8_t's; if nbits >8,
 *              the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void s32k3xx_lpspi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                                    size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return s32k3xx_lpspi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: s32k3xx_lpspi_bus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus in its default state (Master, 8-bit,
 *   mode 0, etc.)
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void s32k3xx_lpspi_bus_initialize(struct s32k3xx_lpspidev_s *priv)
{
  uint32_t reg = 0;

  /* NOTE:
   * Clocking to the LPSPI peripheral must be provided by board-specific
   * logic as part of the clock configuration logic.
   */

  /* Reset to known status */

  s32k3xx_lpspi_modifyreg32(priv, S32K3XX_LPSPI_CR_OFFSET, 0, LPSPI_CR_RST);
  s32k3xx_lpspi_modifyreg32(priv, S32K3XX_LPSPI_CR_OFFSET, 0,
                          LPSPI_CR_RTF | LPSPI_CR_RRF);
  s32k3xx_lpspi_putreg32(priv, S32K3XX_LPSPI_CR_OFFSET, 0x00);

  /* Set LPSPI to master */

  s32k3xx_lpspi_modifyreg32(priv, S32K3XX_LPSPI_CFGR1_OFFSET, 0,
                          LPSPI_CFGR1_MASTER);

  /* Set specific PCS to active high or low */

  /* TODO: Not needed for now */

  /* Set Configuration Register 1 related setting. */

  reg = s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_CFGR1_OFFSET);
  reg &= ~(LPSPI_CFGR1_OUTCFG |
           LPSPI_CFGR1_PINCFG_MASK | LPSPI_CFGR1_NOSTALL);
  reg |= LPSPI_CFGR1_OUTCFG_RETAIN;
  reg |= ((priv->pincfg << LPSPI_CFGR1_PINCFG_SHIFT)
          & LPSPI_CFGR1_PINCFG_MASK);

  s32k3xx_lpspi_putreg32(priv, S32K3XX_LPSPI_CFGR1_OFFSET, reg);

  /* Set frequency and delay times */

  s32k3xx_lpspi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Set default watermarks */

  s32k3xx_lpspi_putreg32(priv, S32K3XX_LPSPI_FCR_OFFSET,
                       LPSPI_FCR_TXWATER(0) | LPSPI_FCR_RXWATER(0));

  /* Set Transmit Command Register */

  s32k3xx_lpspi_setbits((struct spi_dev_s *)priv, 8);

  s32k3xx_lpspi_setmode((struct spi_dev_s *)priv, SPIDEV_MODE0);

  /* Initialize the SPI mutex that enforces mutually exclusive access */

  nxmutex_init(&priv->lock);

  /* Enable LPSPI */

  s32k3xx_lpspi_modifyreg32(priv, S32K3XX_LPSPI_CR_OFFSET, 0, LPSPI_CR_MEN);
}

/****************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_LPSPI_DMA
static int spi_dmarxwait(struct s32k3xx_lpspidev_s *priv)
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

#ifdef CONFIG_S32K3XX_LPSPI_DMA
static int spi_dmatxwait(struct s32k3xx_lpspidev_s *priv)
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

#ifdef CONFIG_S32K3XX_LPSPI_DMA
static inline void spi_dmarxwakeup(struct s32k3xx_lpspidev_s *priv)
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

#ifdef CONFIG_S32K3XX_LPSPI_DMA
static inline void spi_dmatxwakeup(struct s32k3xx_lpspidev_s *priv)
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

#ifdef CONFIG_S32K3XX_LPSPI_DMA
static void spi_dmarxcallback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct s32k3xx_lpspidev_s *priv = (struct s32k3xx_lpspidev_s *)arg;

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

#ifdef CONFIG_S32K3XX_LPSPI_DMA
static void spi_dmatxcallback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct s32k3xx_lpspidev_s *priv = (struct s32k3xx_lpspidev_s *)arg;

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

#ifdef CONFIG_S32K3XX_LPSPI_DMA
static inline void spi_dmarxstart(struct s32k3xx_lpspidev_s *priv)
{
  priv->rxresult = 0;
  s32k3xx_dmach_start(priv->rxdma, spi_dmarxcallback, priv);
}
#endif

/****************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_LPSPI_DMA
static inline void spi_dmatxstart(struct s32k3xx_lpspidev_s *priv)
{
  priv->txresult = 0;
  s32k3xx_dmach_start(priv->txdma, spi_dmatxcallback, priv);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_lpspibus_initialize
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

struct spi_dev_s *s32k3xx_lpspibus_initialize(int bus)
{
  struct s32k3xx_lpspidev_s *priv = NULL;

  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_S32K3XX_LPSPI0
  if (bus == 0)
    {
      /* Select SPI0 */

      priv = &g_lpspi0dev;

      /* Only configure if the bus is not already configured */

      if ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_CR_OFFSET) &
          LPSPI_CR_MEN) == 0)
        {
          /* Configure SPI0 pins: SCK, MISO, and MOSI */

          s32k3xx_pinconfig(PIN_LPSPI0_SCK);
          s32k3xx_pinconfig(PIN_LPSPI0_MISO);
          s32k3xx_pinconfig(PIN_LPSPI0_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          s32k3xx_lpspi_bus_initialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_S32K3XX_LPSPI1
  if (bus == 1)
    {
      /* Select SPI1 */

      priv = &g_lpspi1dev;

      /* Only configure if the bus is not already configured */

      if ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_CR_OFFSET) &
          LPSPI_CR_MEN) == 0)
        {
          /* Configure SPI1 pins: SCK, MISO, and MOSI */

          s32k3xx_pinconfig(PIN_LPSPI1_SCK);
          s32k3xx_pinconfig(PIN_LPSPI1_MISO);
          s32k3xx_pinconfig(PIN_LPSPI1_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          s32k3xx_lpspi_bus_initialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_S32K3XX_LPSPI2
  if (bus == 2)
    {
      /* Select SPI2 */

      priv = &g_lpspi2dev;

      /* Only configure if the bus is not already configured */

      if ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_CR_OFFSET) &
          LPSPI_CR_MEN) == 0)
        {
          /* Configure SPI2 pins: SCK, MISO, and MOSI */

          s32k3xx_pinconfig(PIN_LPSPI2_SCK);
          s32k3xx_pinconfig(PIN_LPSPI2_MISO);
          s32k3xx_pinconfig(PIN_LPSPI2_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          s32k3xx_lpspi_bus_initialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_S32K3XX_LPSPI3
  if (bus == 3)
    {
      /* Select SPI3 */

      priv = &g_lpspi3dev;

      /* Only configure if the bus is not already configured */

      if ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_CR_OFFSET) &
          LPSPI_CR_MEN) == 0)
        {
          /* Configure SPI2 pins: SCK, MISO, and MOSI */

          s32k3xx_pinconfig(PIN_LPSPI3_SCK);
          s32k3xx_pinconfig(PIN_LPSPI3_MISO);
          s32k3xx_pinconfig(PIN_LPSPI3_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          s32k3xx_lpspi_bus_initialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_S32K3XX_LPSPI4
  if (bus == 4)
    {
      /* Select SPI3 */

      priv = &g_lpspi4dev;

      /* Only configure if the bus is not already configured */

      if ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_CR_OFFSET) &
          LPSPI_CR_MEN) == 0)
        {
          /* Configure SPI2 pins: SCK, MISO, and MOSI */

          s32k3xx_pinconfig(PIN_LPSPI4_SCK);
          s32k3xx_pinconfig(PIN_LPSPI4_MISO);
          s32k3xx_pinconfig(PIN_LPSPI4_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          s32k3xx_lpspi_bus_initialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_S32K3XX_LPSPI5
  if (bus == 5)
    {
      /* Select SPI3 */

      priv = &g_lpspi5dev;

      /* Only configure if the bus is not already configured */

      if ((s32k3xx_lpspi_getreg32(priv, S32K3XX_LPSPI_CR_OFFSET) &
          LPSPI_CR_MEN) == 0)
        {
          /* Configure SPI2 pins: SCK, MISO, and MOSI */

          s32k3xx_pinconfig(PIN_LPSPI5_SCK);
          s32k3xx_pinconfig(PIN_LPSPI5_MISO);
          s32k3xx_pinconfig(PIN_LPSPI5_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          s32k3xx_lpspi_bus_initialize(priv);
        }
    }
  else
#endif
    {
      spierr("ERROR: Unsupported SPI bus: %d\n", bus);
    }

#ifdef CONFIG_S32K3XX_LPSPI_DMA
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

          nxsem_set_protocol(&priv->rxsem, SEM_PRIO_NONE);
          nxsem_set_protocol(&priv->txsem, SEM_PRIO_NONE);

          priv->txdma = s32k3xx_dmach_alloc(priv->txch | DMAMUX_CHCFG_ENBL,
                                            0);
          priv->rxdma = s32k3xx_dmach_alloc(priv->rxch | DMAMUX_CHCFG_ENBL,
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

#endif /* CONFIG_S32K3XX_LPSPI0 || CONFIG_S32K3XX_LPSPI1 || CONFIG_S32K3XX_LPSPI2 */

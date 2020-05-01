/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_spi_master.c
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 *    used to endorse or promote products derived from this software
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

/* TODO:
 *
 *   - There are no interrupt driven transfers, only polled.  I don't
 *     consider this a significant problem because of the higher rate that
 *     would be necessary for interrupt driven transfers.
 *   - Integrate DMA transfers.  This is fairly important because it can
 *     a) improve the data transfer rates and b) free the CPU when the
 *     SPI driver would otherwise be stuck in a tight polling loop.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "hardware/lpc54_pinmux.h"
#include "hardware/lpc54_syscon.h"
#include "hardware/lpc54_flexcomm.h"
#include "hardware/lpc54_spi.h"
#include "lpc54_config.h"
#include "lpc54_enableclk.h"
#include "lpc54_gpio.h"
#include "lpc54_spi_master.h"

#ifdef HAVE_SPI_MASTER_DEVICE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_DUMMYDATA8   0xff
#define SPI_DUMMYDATA16  0xffff

#define SPI_MINWIDTH     4
#ifdef CONFIG_LPC54_SPI_WIDEDATA
#  define SPI_MAXWIDTH   16
#else
#  define SPI_MAXWIDTH   8
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the SSP driver */

struct lpc54_spidev_s
{
  struct spi_dev_s dev;       /* Externally visible part of the SPI interface */
  uintptr_t base;             /* Base address of Flexcomm registers */
  sem_t exclsem;              /* Held while chip is selected for mutual exclusion */
  uint32_t fclock;            /* Flexcomm function clock frequency */
  uint32_t frequency;         /* Requested clock frequency */
  uint32_t actual;            /* Actual clock frequency */
  uint16_t irq;               /* Flexcomm IRQ number */
  uint8_t nbits;              /* Width of word in bits (SPI_MINWIDTH to SPI_MAXWIDTH) */
  uint8_t mode;               /* Mode 0,1,2,3 */
};

/* These structures describes the Rx side of an 8- or 16-bit SPI data
 * exchange.
 */

struct lpc54_rxtransfer8_s
{
  FAR uint8_t *rxptr;         /* Pointer into receive buffer */
  unsigned int remaining;     /* Bytes remaining in the receive buffer */
  unsigned int expected;      /* Bytes expected to be received */
};

#ifdef CONFIG_LPC54_SPI_WIDEDATA
struct lpc54_rxtransfer16_s
{
  FAR uint16_t *rxptr;        /* Pointer into receive buffer */
  unsigned int remaining;     /* Hwords remaining in the receive buffer */
  unsigned int expected;      /* Hwords expected to be received */
};
#endif

/* These structures describes the Tx side of an 8- or 16-bit SPI data
 * exchange.
 */

struct lpc54_txtransfer8_s
{
  uint32_t txctrl;            /* Tx control bits */
  FAR const uint8_t *txptr;   /* Pointer into transmit buffer */
  unsigned int remaining;     /* Bytes remaining in the transmit buffer */
};

#ifdef CONFIG_LPC54_SPI_WIDEDATA
struct lpc54_txtransfer16_s
{
  uint32_t txctrl;            /* Tx control bits */
  FAR const uint16_t *txptr;  /* Pointer into transmit buffer */
  unsigned int remaining;     /* Hwords remaining in the transmit buffer */
};
#endif

struct lpc54_txdummy_s
{
  uint32_t txctrl;            /* Tx control bits */
  unsigned int remaining;     /* Bytes remaining in the transmit buffer */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Transfer helpers */

static inline size_t lpc54_spi_fifodepth(FAR struct lpc54_spidev_s *priv);
static inline bool lpc54_spi_txavailable(FAR struct lpc54_spidev_s *priv);
static inline bool lpc54_spi_rxavailable(FAR struct lpc54_spidev_s *priv);

static void     lpc54_spi_resetfifos(FAR struct lpc54_spidev_s *priv);
static void     lpc54_spi_rxtransfer8(FAR struct lpc54_spidev_s *priv,
                  FAR struct lpc54_rxtransfer8_s *xfr);
#ifdef CONFIG_LPC54_SPI_WIDEDATA
static void     lpc54_spi_rxtransfer16(FAR struct lpc54_spidev_s *priv,
                  FAR struct lpc54_rxtransfer16_s *xfr);
#endif
static bool     lpc54_spi_txtransfer8(FAR struct lpc54_spidev_s *priv,
                  FAR struct lpc54_txtransfer8_s *xfr);
#ifdef CONFIG_LPC54_SPI_WIDEDATA
static bool     lpc54_spi_txtransfer16(FAR struct lpc54_spidev_s *priv,
                  FAR struct lpc54_txtransfer16_s *xfr);
#endif
static bool     lpc54_spi_txdummy(FAR struct lpc54_spidev_s *priv,
                  FAR struct lpc54_txdummy_s *xfr);
#ifdef CONFIG_SPI_EXCHANGE
static void     lpc54_spi_exchange8(FAR struct lpc54_spidev_s *priv,
                  FAR const void *txbuffer, FAR void *rxbuffer,
                  size_t nwords);
#ifdef CONFIG_LPC54_SPI_WIDEDATA
static void     lpc54_spi_exchange16(FAR struct lpc54_spidev_s *priv,
                  FAR const void *txbuffer, FAR void *rxbuffer,
                  size_t nwords);
#endif
#endif
static void     lpc54_spi_sndblock8(FAR struct lpc54_spidev_s *priv,
                  FAR const void *buffer, size_t nwords);
#ifdef CONFIG_LPC54_SPI_WIDEDATA
static void     lpc54_spi_sndblock16(FAR struct lpc54_spidev_s *priv,
                  FAR const void *buffer, size_t nwords);
#endif
static void     lpc54_spi_recvblock8(FAR struct lpc54_spidev_s *priv,
                  FAR void *buffer, size_t nwords);
#ifdef CONFIG_LPC54_SPI_WIDEDATA
static void     lpc54_spi_recvblock16(FAR struct lpc54_spidev_s *priv,
                  FAR void *buffer, size_t nwords);
#endif

/* SPI methods */

static int      lpc54_spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t lpc54_spi_setfrequency(FAR struct spi_dev_s *dev,
                  uint32_t frequency);
static void     lpc54_spi_setmode(FAR struct spi_dev_s *dev,
                  enum spi_mode_e mode);
static void     lpc54_spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint32_t lpc54_spi_send(FAR struct spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SPI_EXCHANGE
static void     lpc54_spi_exchange(FAR struct spi_dev_s *dev,
                  FAR const void *txbuffer, FAR void *rxbuffer,
                  size_t nwords);
#endif
static void     lpc54_spi_sndblock(FAR struct spi_dev_s *dev,
                  FAR const void *buffer, size_t nwords);
static void     lpc54_spi_recvblock(FAR struct spi_dev_s *dev,
                  FAR void *buffer, size_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LPC54_SPI0_MASTER
static const struct spi_ops_s g_spi0_ops =
{
  .lock              = lpc54_spi_lock,
  .select            = lpc54_spi0_select,   /* Provided externally */
  .setfrequency      = lpc54_spi_setfrequency,
  .setmode           = lpc54_spi_setmode,
  .setbits           = lpc54_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = NULL,                /* Not supported */
#endif
  .status            = lpc54_spi0_status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc54_spi0_cmddata,  /* Provided externally */
#endif
  .send              = lpc54_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = lpc54_spi_exchange,
#else
  .sndblock          = lpc54_spi_sndblock,
  .recvblock         = lpc54_spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc54_spi0_register, /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct lpc54_spidev_s g_spi0_dev;
#endif

#ifdef CONFIG_LPC54_SPI1_MASTER
static const struct spi_ops_s g_spi1_ops =
{
  .lock              = lpc54_spi_lock,
  .select            = lpc54_spi1_select,   /* Provided externally */
  .setfrequency      = lpc54_spi_setfrequency,
  .setmode           = lpc54_spi_setmode,
  .setbits           = lpc54_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = NULL,                /* Not supported */
#endif
  .status            = lpc54_spi1_status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc54_spi1_cmddata,  /* Provided externally */
#endif
  .send              = lpc54_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = lpc54_spi_exchange,
#else
  .sndblock          = lpc54_spi_sndblock,
  .recvblock         = lpc54_spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc54_spi1_register, /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct lpc54_spidev_s g_spi1_dev;
#endif

#ifdef CONFIG_LPC54_SPI2_MASTER
static const struct spi_ops_s g_spi2_ops =
{
  .lock              = lpc54_spi_lock,
  .select            = lpc54_spi2_select,   /* Provided externally */
  .setfrequency      = lpc54_spi_setfrequency,
  .setmode           = lpc54_spi_setmode,
  .setbits           = lpc54_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = NULL,                /* Not supported */
#endif
  .status            = lpc54_spi2_status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc54_spi2_cmddata,  /* Provided externally */
#endif
  .send              = lpc54_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = lpc54_spi_exchange,
#else
  .sndblock          = lpc54_spi_sndblock,
  .recvblock         = lpc54_spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc54_spi2_register, /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct lpc54_spidev_s g_spi2_dev;
#endif

#ifdef CONFIG_LPC54_SPI3_MASTER
static const struct spi_ops_s g_spi3_ops =
{
  .lock              = lpc54_spi_lock,
  .select            = lpc54_spi3_select,   /* Provided externally */
  .setfrequency      = lpc54_spi_setfrequency,
  .setmode           = lpc54_spi_setmode,
  .setbits           = lpc54_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = NULL,                /* Not supported */
#endif
  .status            = lpc54_spi3_status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc54_spi3_cmddata,  /* Provided externally */
#endif
  .send              = lpc54_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = lpc54_spi_exchange,
#else
  .sndblock          = lpc54_spi_sndblock,
  .recvblock         = lpc54_spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc54_spi3_register, /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct lpc54_spidev_s g_spi3_dev;
#endif

#ifdef CONFIG_LPC54_SPI4_MASTER
static const struct spi_ops_s g_spi4_ops =
{
  .lock              = lpc54_spi_lock,
  .select            = lpc54_spi4_select,   /* Provided externally */
  .setfrequency      = lpc54_spi_setfrequency,
  .setmode           = lpc54_spi_setmode,
  .setbits           = lpc54_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = NULL,                /* Not supported */
#endif
  .status            = lpc54_spi4_status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc54_spi4_cmddata,  /* Provided externally */
#endif
  .send              = lpc54_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = lpc54_spi_exchange,
#else
  .sndblock          = lpc54_spi_sndblock,
  .recvblock         = lpc54_spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc54_spi4_register, /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct lpc54_spidev_s g_spi4_dev;
#endif

#ifdef CONFIG_LPC54_SPI5_MASTER
static const struct spi_ops_s g_spi5_ops =
{
  .lock              = lpc54_spi_lock,
  .select            = lpc54_spi5_select,   /* Provided externally */
  .setfrequency      = lpc54_spi_setfrequency,
  .setmode           = lpc54_spi_setmode,
  .setbits           = lpc54_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = NULL,                /* Not supported */
#endif
  .status            = lpc54_spi5_status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc54_spi5_cmddata,  /* Provided externally */
#endif
  .send              = lpc54_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = lpc54_spi_exchange,
#else
  .sndblock          = lpc54_spi_sndblock,
  .recvblock         = lpc54_spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc54_spi5_register, /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct lpc54_spidev_s g_spi5_dev;
#endif

#ifdef CONFIG_LPC54_SPI6_MASTER
static const struct spi_ops_s g_spi6_ops =
{
  .lock              = lpc54_spi_lock,
  .select            = lpc54_spi6_select,   /* Provided externally */
  .setfrequency      = lpc54_spi_setfrequency,
  .setmode           = lpc54_spi_setmode,
  .setbits           = lpc54_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = NULL,                /* Not supported */
#endif
  .status            = lpc54_spi6_status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc54_spi6_cmddata,  /* Provided externally */
#endif
  .send              = lpc54_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = lpc54_spi_exchange,
#else
  .sndblock          = lpc54_spi_sndblock,
  .recvblock         = lpc54_spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc54_spi6_register, /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct lpc54_spidev_s g_spi6_dev;
#endif

#ifdef CONFIG_LPC54_SPI7_MASTER
static const struct spi_ops_s g_spi7_ops =
{
  .lock              = lpc54_spi_lock,
  .select            = lpc54_spi7_select,   /* Provided externally */
  .setfrequency      = lpc54_spi_setfrequency,
  .setmode           = lpc54_spi_setmode,
  .setbits           = lpc54_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = NULL,                /* Not supported */
#endif
  .status            = lpc54_spi7_status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc54_spi7_cmddata,  /* Provided externally */
#endif
  .send              = lpc54_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = lpc54_spi_exchange,
#else
  .sndblock          = lpc54_spi_sndblock,
  .recvblock         = lpc54_spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc54_spi7_register, /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct lpc54_spidev_s g_spi7_dev;
#endif

#ifdef CONFIG_LPC54_SPI8_MASTER
static const struct spi_ops_s g_spi8_ops =
{
  .lock              = lpc54_spi_lock,
  .select            = lpc54_spi8_select,   /* Provided externally */
  .setfrequency      = lpc54_spi_setfrequency,
  .setmode           = lpc54_spi_setmode,
  .setbits           = lpc54_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = NULL,                /* Not supported */
#endif
  .status            = lpc54_spi8_status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc54_spi8_cmddata,  /* Provided externally */
#endif
  .send              = lpc54_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = lpc54_spi_exchange,
#else
  .sndblock          = lpc54_spi_sndblock,
  .recvblock         = lpc54_spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc54_spi8_register, /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct lpc54_spidev_s g_spi8_dev;
#endif

#ifdef CONFIG_LPC54_SPI9_MASTER
static const struct spi_ops_s g_spi9_ops =
{
  .lock              = lpc54_spi_lock,
  .select            = lpc54_spi9_select,   /* Provided externally */
  .setfrequency      = lpc54_spi_setfrequency,
  .setmode           = lpc54_spi_setmode,
  .setbits           = lpc54_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = NULL,                /* Not supported */
#endif
  .status            = lpc54_spi9_status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc54_spi9_cmddata,  /* Provided externally */
#endif
  .send              = lpc54_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = lpc54_spi_exchange,
#else
  .sndblock          = lpc54_spi_sndblock,
  .recvblock         = lpc54_spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc54_spi9_register, /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct lpc54_spidev_s g_spi9_dev;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_spi_putreg
 *
 * Description:
 *   Write a value to a register at the offset from the Flexcomm base.
 *
 ****************************************************************************/

static inline void lpc54_spi_putreg(struct lpc54_spidev_s *priv,
                                    unsigned int regoffset, uint32_t regval)
{
  putreg32(regval, priv->base + regoffset);
}

/****************************************************************************
 * Name: lpc54_spi_getreg
 *
 * Description:
 *   Read the content of a register at the offset from the Flexcomm base.
 *
 ****************************************************************************/

static inline uint32_t lpc54_spi_getreg(struct lpc54_spidev_s *priv,
                                        unsigned int regoffset)
{
  return getreg32(priv->base + regoffset);
}

/****************************************************************************
 * Name: lpc54_spi_fifodepth
 *
 * Description:
 *   Return the depth of the SPI FIFOs.  This is a constant value and could
 *   be hard coded.
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   The FIFO depth in words of the configured bit width.
 *
 ****************************************************************************/

static inline size_t lpc54_spi_fifodepth(FAR struct lpc54_spidev_s *priv)
{
  uint32_t regval = lpc54_spi_getreg(priv, LPC54_SPI_FIFOCFG_OFFSET);
  return ((regval & SPI_FIFOCFG_SIZE_MASK) >> SPI_FIFOCFG_SIZE_SHIFT) << 3;
}

/****************************************************************************
 * Name: lpc54_spi_txavailable
 *
 * Description:
 *   Return true if the Tx FIFO is not full.
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   true: Tx FIFO is not full.
 *
 ****************************************************************************/

static inline bool lpc54_spi_txavailable(FAR struct lpc54_spidev_s *priv)
{
  uint32_t regval = lpc54_spi_getreg(priv, LPC54_SPI_FIFOSTAT_OFFSET);
  return ((regval & SPI_FIFOSTAT_TXNOTFULL) != 0);
}

/****************************************************************************
 * Name: lpc54_spi_rxavailable
 *
 * Description:
 *   Return true if the Rx FIFO is not empty.
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   true: Rx FIFO is not empty.
 *
 ****************************************************************************/

static inline bool lpc54_spi_rxavailable(FAR struct lpc54_spidev_s *priv)
{
  uint32_t regval = lpc54_spi_getreg(priv, LPC54_SPI_FIFOSTAT_OFFSET);
  return ((regval & SPI_FIFOSTAT_RXNOTEMPTY) != 0);
}

/****************************************************************************
 * Name: lpc54_spi_rxdiscard
 *
 * Description:
 *   Read and discard the data until the Rx FIFO is empty.
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc54_spi_rxdiscard(FAR struct lpc54_spidev_s *priv)
{
  while (lpc54_spi_rxavailable(priv))
    {
      lpc54_spi_getreg(priv, LPC54_SPI_FIFORD_OFFSET);
    }
}

/****************************************************************************
 * Name: lpc54_spi_resetfifos
 *
 * Description:
 *   Clear Tx/Rx errors and empty FIFOs.
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc54_spi_resetfifos(FAR struct lpc54_spidev_s *priv)
{
  uint32_t regval;

  /* Clear Tx/Rx errors and empty FIFOs */

  regval  = lpc54_spi_getreg(priv, LPC54_SPI_FIFOCFG_OFFSET);
  regval |= (SPI_FIFOCFG_EMPTYTX | SPI_FIFOCFG_EMPTYRX);
  lpc54_spi_putreg(priv, LPC54_SPI_FIFOCFG_OFFSET, regval);

  regval  = lpc54_spi_getreg(priv, LPC54_SPI_FIFOSTAT_OFFSET);
  regval |= (SPI_FIFOSTAT_TXERR | SPI_FIFOSTAT_RXERR);
  lpc54_spi_putreg(priv, LPC54_SPI_FIFOSTAT_OFFSET, regval);
}

/****************************************************************************
 * Name: lpc54_spi_rxtransfer8 and lpc54_spi_rxtransfer16
 *
 * Description:
 *   Receive one 8-bit or 16-bit value from the selected SPI device.
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   xfr  - Describes the Rx transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc54_spi_rxtransfer8(FAR struct lpc54_spidev_s *priv,
                                  FAR struct lpc54_rxtransfer8_s *xfr)
{
  /* Read one byte if available and expected */

  if (lpc54_spi_rxavailable(priv))
    {
      /* There is something in the Rx FIFO to be read.  Are we expecting
       * data in the Rx FIFO?  Is there space available in the Rx buffer?
       */

      if (xfr->expected == 0 || xfr->remaining == 0)
        {
          /* No.. then just read and discard the data until the Rx FIFO is empty */

          lpc54_spi_rxdiscard(priv);
          xfr->expected = 0;
        }
      else
        {
          /* Read and transfer one byte */

          *xfr->rxptr = lpc54_spi_getreg(priv, LPC54_SPI_FIFORD_OFFSET);

          /* Update pointers and counts */

          xfr->rxptr++;
          xfr->remaining--;
          xfr->expected--;
        }
    }
}

#ifdef CONFIG_LPC54_SPI_WIDEDATA
static void lpc54_spi_rxtransfer16(FAR struct lpc54_spidev_s *priv,
                                   FAR struct lpc54_rxtransfer16_s *xfr)
{
  /* Read one HWord if available and expected */

  if (lpc54_spi_rxavailable(priv))
    {
      /* There is something in the Rx FIFO to be read.  Are we expecting
       * data in the Rx FIFO?  Is there space available in the Rx buffer?
       */

      if (xfr->expected == 0 || xfr->remaining == 0)
        {
          /* No.. then just read and discard the data until the Rx FIFO
           * is empty.
           */

          lpc54_spi_rxdiscard(priv);
          xfr->expected = 0;
        }
      else
        {
          /* Read and transfer HWord */

          *xfr->rxptr = lpc54_spi_getreg(priv, LPC54_SPI_FIFORD_OFFSET);

          /* Update pointers and counts */

          xfr->rxptr++;
          xfr->remaining--;
          xfr->expected--;
        }
    }
}
#endif

/****************************************************************************
 * Name: lpc54_spi_txtransfer8 and lpc54_spi_txtransfer16
 *
 * Description:
 *   Send one 8- or 16-bit value to the selected SPI device.
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   xfr  - Describes the Tx transfer
 *
 * Returned Value:
 *   true:  The value was added to the TxFIFO
 *
 ****************************************************************************/

static bool lpc54_spi_txtransfer8(FAR struct lpc54_spidev_s *priv,
                                  FAR struct lpc54_txtransfer8_s *xfr)
{
  uint32_t regval;

  /* Transmit if txFIFO is not full and there is more Tx data to be sent */

  if (lpc54_spi_txavailable(priv) && xfr->remaining > 0)
    {
      /* Get the next byte to be sent */

      regval = *xfr->txptr;

      /* And send it */

      regval |= xfr->txctrl;
      lpc54_spi_putreg(priv, LPC54_SPI_FIFOWR_OFFSET, regval);

      /* Update pointers and counts */

      xfr->txptr++;
      xfr->remaining--;

      return true;
    }

  return false;
}

#ifdef CONFIG_LPC54_SPI_WIDEDATA
static bool lpc54_spi_txtransfer16(FAR struct lpc54_spidev_s *priv,
                                   FAR struct lpc54_txtransfer16_s *xfr)
{
  uint32_t regval;

  /* Transmit if txFIFO is not full and there is more Tx data to be sent */

  if (lpc54_spi_txavailable(priv) && xfr->remaining > 0)
    {
      /* Get the next byte to be sent */

      regval = *xfr->txptr;

      /* And send it */

      regval |= xfr->txctrl;
      lpc54_spi_putreg(priv, LPC54_SPI_FIFOWR_OFFSET, regval);

      /* Update pointers and counts */

      xfr->txptr++;
      xfr->remaining--;

      return true;
    }

  return false;
}
#endif

/****************************************************************************
 * Name: lpc54_spi_txdummy
 *
 * Description:
 *   Send dummy Tx data when we really only care about the Rx data.
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   xfr  - Describes the Tx transfer
 *
 * Returned Value:
 *   true:  The dummy value was added to the TxFIFO
 *
 ****************************************************************************/

static bool lpc54_spi_txdummy(FAR struct lpc54_spidev_s *priv,
                              FAR struct lpc54_txdummy_s *xfr)
{
  /* Transmit if txFIFO is not full and there is more Tx data to be sent */

  if (lpc54_spi_txavailable(priv) && xfr->remaining > 0)
    {
      /* Send the dummy data */

      lpc54_spi_putreg(priv, LPC54_SPI_FIFOWR_OFFSET, xfr->txctrl);

      /* Update counts */

      xfr->remaining--;
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: lpc54_spi_exchange8 and lpc54_spi_exchange16
 *
 * Description:
 *   Implements the SPI exchange method for the case of 8-bit and 16-bit
 *   transfers.
 *
 * Input Parameters:
 *   priv     - Device-specific state data
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

#ifdef CONFIG_SPI_EXCHANGE
static void lpc54_spi_exchange8(FAR struct lpc54_spidev_s *priv,
                                FAR const void *txbuffer, FAR void *rxbuffer,
                                size_t nwords)
{
  struct lpc54_rxtransfer8_s rxtransfer;
  struct lpc54_txtransfer8_s txtransfer;
  size_t depth;

  DEBUGASSERT(rxbuffer != NULL && txbuffer != NULL);

  /* Get the FIFO depth */

  depth = lpc54_spi_fifodepth(priv);

  /* Set up the transfer data */

  txtransfer.txctrl = SPI_FIFOWR_LEN(priv->nbits) | SPI_FIFOWR_TXSSELN_ALL;
  txtransfer.txptr = (FAR uint8_t *)txbuffer;
  txtransfer.remaining = nwords;
  rxtransfer.rxptr = (FAR uint8_t *)rxbuffer;
  rxtransfer.remaining = nwords;
  rxtransfer.expected = 0;

  /* Clear Tx/Rx errors and empty FIFOs */

  lpc54_spi_resetfifos(priv);

  /* Loop until all Tx data has been sent and until all Rx data has been
   * received.
   */

  while (txtransfer.remaining != 0 || rxtransfer.remaining != 0)
    {
      /* Transfer one byte from the Rx FIFO to the caller's Rx buffer */

      lpc54_spi_rxtransfer8(priv, &rxtransfer);

      /* If sending another byte would exceed the capacity of the Rx FIFO
       * then read-only until there space freed.
       */

      if (rxtransfer.expected < depth)
        {
          /* Attempt to transfer one byte from the caller's Tx buffer to
           * the Tx FIFO.
           */

          if (lpc54_spi_txtransfer8(priv, &txtransfer))
            {
              /* Increment the Rx expected count if successful */

              rxtransfer.expected++;
            }
        }
    }
}
#endif /* CONFIG_SPI_EXCHANGE */

#if defined(CONFIG_SPI_EXCHANGE) && defined(CONFIG_LPC54_SPI_WIDEDATA)
static void lpc54_spi_exchange16(FAR struct lpc54_spidev_s *priv,
                                 FAR const void *txbuffer,
                                 FAR void *rxbuffer,
                                 size_t nwords)
{
  struct lpc54_rxtransfer16_s rxtransfer;
  struct lpc54_txtransfer16_s txtransfer;
  uint32_t regval;
  size_t depth;

  DEBUGASSERT(rxbuffer != NULL && ((uintptr_t)rxbuffer & 1) == 0);
  DEBUGASSERT(txbuffer != NULL && ((uintptr_t)txbuffer & 1) == 0);

  /* Get the FIFO depth */

  depth = lpc54_spi_fifodepth(priv);

  /* Set up the transfer data */

  txtransfer.txctrl = SPI_FIFOWR_LEN(priv->nbits) | SPI_FIFOWR_TXSSELN_ALL;
  txtransfer.txptr = (FAR uint16_t *)txbuffer;
  txtransfer.remaining = nwords;
  rxtransfer.rxptr = (FAR uint16_t *)rxbuffer;
  rxtransfer.remaining = nwords;
  rxtransfer.expected = 0;

  /* Clear Tx/Rx errors and empty FIFOs */

  lpc54_spi_resetfifos(priv);

  /* Loop until all Tx data has been sent and until all Rx data has been
   * received.
   */

  while (txtransfer.remaining || rxtransfer.remaining || rxtransfer.expected)
    {
      /* Transfer one HWord from the Rx FIFO to the caller's Rx buffer */

      lpc54_spi_rxtransfer16(priv, &rxtransfer);

      /* If sending another byte would exceed the capacity of the Rx FIFO
       * then read-only until there space freed.
       */

      if (rxtransfer.expected < depth)
        {
          /* Attempt to send one more byte */

          if (lpc54_spi_txtransfer16(priv, &txtransfer))
            {
              /* Increment the Rx expected count if successful */

              rxtransfer.expected++;
            }
        }
    }
}
#endif /* CONFIG_SPI_EXCHANGE && CONFIG_LPC54_SPI_WIDEDATA */

/****************************************************************************
 * Name: lpc54_spi_sndblock8 and lpc54_spi_sndblock16
 *
 * Description:
 *   Implements the SPI sndblock method for the case of 8- and 16-bit
 *   transfers.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
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

static void lpc54_spi_sndblock8(FAR struct lpc54_spidev_s *priv,
                                FAR const void *buffer, size_t nwords)
{
  struct lpc54_txtransfer8_s txtransfer;

  DEBUGASSERT(buffer != NULL);

  /* Set up the transfer data.  NOTE that we are ignoring returned Rx data */

  txtransfer.txctrl    = SPI_FIFOWR_RXIGNORE | SPI_FIFOWR_LEN(priv->nbits) |
                         SPI_FIFOWR_TXSSELN_ALL;
  txtransfer.txptr     = (FAR uint8_t *)buffer;
  txtransfer.remaining = nwords;

  /* Clear Tx/Rx errors and empty FIFOs */

  lpc54_spi_resetfifos(priv);

  /* Loop until all Tx data has been sent */

  while (txtransfer.remaining != 0)
    {
      /* Attempt to transfer one byte from the caller's Tx buffer to the
       * Tx FIFO.
       */

      lpc54_spi_txtransfer8(priv, &txtransfer);
    }
}

#ifdef CONFIG_LPC54_SPI_WIDEDATA
static void lpc54_spi_sndblock16(FAR struct lpc54_spidev_s *priv,
                                 FAR const void *buffer, size_t nwords)
{
  struct lpc54_txtransfer16_s txtransfer;

  DEBUGASSERT(buffer != NULL);

  /* Set up the transfer data.  NOTE that we are ignoring returned Rx data */

  txtransfer.txctrl    = SPI_FIFOWR_RXIGNORE | SPI_FIFOWR_LEN(priv->nbits) |
                         SPI_FIFOWR_TXSSELN_ALL;
  txtransfer.txptr     = (FAR uint16_t *)buffer;
  txtransfer.remaining = nwords;

  /* Clear Tx/Rx errors and empty FIFOs */

  lpc54_spi_resetfifos(priv);

  /* Loop until all Tx data has been sent and until all Rx data has been
   * received.
   */

  while (txtransfer.remaining != 0)
    {
      /* Attempt to transfer one byte from the caller's Tx buffer to the
       * Tx FIFO.
       */

      lpc54_spi_txtransfer16(priv, &txtransfer);
    }
}
#endif /* CONFIG_LPC54_SPI_WIDEDATA */

/****************************************************************************
 * Name: lpc54_spi_recvblock8 and lpc54_spi_recvblock16
 *
 * Description:
 *   Implements the SPI recvblock method for the case of 8- and 16-bit
 *   transfers.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   buffer - A pointer to the buffer in which to receive data
 *   nwords - the length of data that can be received in the buffer in
 *            number of words.  The wordsize is determined by the number of
 *            bits-per-word selected for the SPI interface.  If nbits <= 8,
 *            the data is packed into uint8_t's; if nbits >8, the data is
 *            packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc54_spi_recvblock8(FAR struct lpc54_spidev_s *priv,
                                 FAR void *buffer, size_t nwords)
{
  struct lpc54_rxtransfer8_s rxtransfer;
  struct lpc54_txdummy_s txtransfer;
  size_t depth;

  DEBUGASSERT(buffer != NULL);

  /* Get the FIFO depth */

  depth = lpc54_spi_fifodepth(priv);

  /* Set up the transfer data */

  txtransfer.txctrl    = SPI_DUMMYDATA8 | SPI_FIFOWR_LEN(priv->nbits) |
                         SPI_FIFOWR_TXSSELN_ALL;
  txtransfer.remaining = nwords;
  rxtransfer.rxptr     = (FAR uint8_t *)buffer;
  rxtransfer.remaining = nwords;
  rxtransfer.expected  = 0;

  /* Clear Tx/Rx errors and empty FIFOs */

  lpc54_spi_resetfifos(priv);

  /* Loop until all Tx data has been sent and until all Rx data has been
   * received.
   */

  while (txtransfer.remaining != 0 || rxtransfer.remaining != 0)
    {
      /* Transfer one byte from the Rx FIFO to the caller's Rx buffer */

      lpc54_spi_rxtransfer8(priv, &rxtransfer);

      /* If sending another byte would exceed the capacity of the Rx FIFO
       * then read-only until there space freed.
       */

      if (rxtransfer.expected < depth)
        {
          /* Attempt to transfer one dummy byte to the Tx FIFO. */

          if (lpc54_spi_txdummy(priv, &txtransfer))
            {
              /* Increment the Rx expected count if successful */

              rxtransfer.expected++;
            }
        }
    }
}

#ifdef CONFIG_LPC54_SPI_WIDEDATA
static void lpc54_spi_recvblock16(FAR struct lpc54_spidev_s *priv,
                                  FAR void *buffer, size_t nwords)
{
  struct lpc54_rxtransfer16_s rxtransfer;
  struct lpc54_txdummy_s txtransfer;
  size_t depth;

  DEBUGASSERT(buffer != NULL);

  /* Get the FIFO depth */

  depth = lpc54_spi_fifodepth(priv);

  /* Set up the transfer data */

  txtransfer.txctrl    = SPI_DUMMYDATA16 | SPI_FIFOWR_LEN(priv->nbits) |
                         SPI_FIFOWR_TXSSELN_ALL;
  txtransfer.remaining = nwords;
  rxtransfer.rxptr     = (FAR uint16_t *)rxbuffer;
  rxtransfer.remaining = nwords;
  rxtransfer.expected  = 0;

  /* Clear Tx/Rx errors and empty FIFOs */

  lpc54_spi_resetfifos(priv);

  /* Loop until all Tx data has been sent and until all Rx data has been
   * received.
   */

  while (txtransfer.remaining || rxtransfer.remaining || rxtransfer.expected)
    {
      /* Transfer one HWord from the Rx FIFO to the caller's Rx buffer */

      lpc54_spi_rxtransfer16(priv, &rxtransfer);

      /* If sending another byte would exceed the capacity of the Rx FIFO
       * then read-only until there space freed.
       */

      if (rxtransfer.expected < depth)
        {
          /* Attempt to transfer one dummy HWord to the Tx FIFO. */

          if (lpc54_spi_txdummy(priv, &txtransfer))
            {
              /* Increment the Rx expected count if successful */

              rxtransfer.expected++;
            }
        }
    }
}
#endif /* CONFIG_LPC54_SPI_WIDEDATA */

/****************************************************************************
 * Name: lpc54_spi_lock
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

static int lpc54_spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct lpc54_spidev_s *priv = (FAR struct lpc54_spidev_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxsem_wait_uninterruptible(&priv->exclsem);
    }
  else
    {
      ret = nxsem_post(&priv->exclsem);
    }

  return ret;
}

/****************************************************************************
 * Name: lpc54_spi_setfrequency
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

static uint32_t lpc54_spi_setfrequency(FAR struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  FAR struct lpc54_spidev_s *priv = (FAR struct lpc54_spidev_s *)dev;
  uint32_t divider;
  uint32_t actual;
  uint32_t regval;

  /* Check if the requested frequency is the same as the current frequency
   * selection.
   */

  DEBUGASSERT(priv != NULL && frequency <= priv->fclock / 2);

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }

  /* Set the new SPI frequency */

  divider = priv->fclock / frequency;
  if (divider > 0x10000)
    {
      divider = 0x10000;
    }

  regval  = lpc54_spi_getreg(priv, LPC54_SPI_DIV_OFFSET);
  regval &= ~SPI_DIV_MASK;
  regval |= SPI_DIV(divider);
  lpc54_spi_putreg(priv, LPC54_SPI_DIV_OFFSET, regval);

  /* Calculate the actual frequency */

  actual  = priv->fclock / divider;

  /* Save the frequency setting */

  priv->frequency = frequency;
  priv->actual    = actual;

  spiinfo("Frequency %d->%d\n", frequency, actual);
  return actual;
}

/****************************************************************************
 * Name: lpc54_spi_setmode
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

static void lpc54_spi_setmode(FAR struct spi_dev_s *dev,
                              enum spi_mode_e mode)
{
  FAR struct lpc54_spidev_s *priv = (FAR struct lpc54_spidev_s *)dev;
  uint32_t regval;

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set the new mode */

      regval  = lpc54_spi_getreg(priv, LPC54_SPI_CFG_OFFSET);
      regval &= ~(SPI_CFG_CPHA | SPI_CFG_CPOL);

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          regval |= SPI_CFG_CPHA;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          regval |= SPI_CFG_CPOL;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          regval |= (SPI_CFG_CPHA | SPI_CFG_CPOL);
          break;

        default:
          DEBUGPANIC();
          return;
        }

      lpc54_spi_putreg(priv, LPC54_SPI_CFG_OFFSET, regval);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: lpc54_spi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev   -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void lpc54_spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct lpc54_spidev_s *priv = (FAR struct lpc54_spidev_s *)dev;

  /* The valid range of bit selections is SPI_MINWIDTH through SPI_MAXWIDTH */

  DEBUGASSERT(priv != NULL && nbits >= SPI_MINWIDTH &&
              nbits <= SPI_MAXWIDTH);

  if (nbits >= SPI_MINWIDTH && nbits <= SPI_MAXWIDTH)
    {
      /* Save the selection.  It will be applied when data is transferred. */

      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: lpc54_spi_send
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

static uint32_t lpc54_spi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  FAR struct lpc54_spidev_s *priv = (FAR struct lpc54_spidev_s *)dev;
  uint32_t regval;

  DEBUGASSERT(priv != NULL);

  /* Clear Tx/Rx errors and empty FIFOs */

  lpc54_spi_resetfifos(priv);

  /* Send the word.  Since we just reset the FIFOs, we assume that the Tx
   * FIFO is not full and that the Rx FIFO is empty.
   */

  DEBUGASSERT(lpc54_spi_txavailable(priv) || !lpc54_spi_rxavailable(priv));

  regval = wd | SPI_FIFOWR_LEN(priv->nbits) | SPI_FIFOWR_TXSSELN_ALL;
  lpc54_spi_putreg(priv, LPC54_SPI_FIFOWR_OFFSET, regval);

  /* Wait for the Rx FIFO to become non-empty. */

  while (!lpc54_spi_rxavailable(priv))
    {
    }

  /* Then read and return the value from the Rx FIFO */

  return lpc54_spi_getreg(priv, LPC54_SPI_FIFORD_OFFSET);
}

/****************************************************************************
 * Name: lpc54_spi_exchange
 *
 * Description:
 *   Exchange a block of data on SPI
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

#ifdef CONFIG_SPI_EXCHANGE
static void lpc54_spi_exchange(FAR struct lpc54_spidev_s *priv,
                               FAR const void *txbuffer, FAR void *rxbuffer,
                               size_t nwords)
{
  FAR struct lpc54_spidev_s *priv = (FAR struct lpc54_spidev_s *)dev;

  DEBUGASSERT(priv != NULL);

  /* If there is no data sink, then handle this transfer with
   * lpc54_spi_sndblock().
   */

  if (rxbuffer == NULL)
    {
      lpc54_spi_sndblock(priv, txbuffer, nwords)
    }

  /* If there is no data source, then handle this transfer with
   * lpc54_spi_recvblock().
   */

  else if (txbuffer == NULL)
    {
      lpc54_spi_recvblock(priv, rxbuffer, nwords)
    }

#ifdef CONFIG_LPC54_SPI_WIDEDATA
  /* If the data with is > 8-bits, then handle this transfer with
   * lpc54_spi_exchange16().
   */

  else if (priv->nbits > 8)
    {
      lpc54_spi_exchange16(priv, txbuffer, rxbuffer, nwords);
    }
#endif

  /* Otherwise, let lpc54_spi_exchange8() do the job */

  else if (priv->nbits > 8)
    {
      lpc54_spi_exchange8(priv, txbuffer, rxbuffer, nwords);
    }
}
#endif /* CONFIG_SPI_EXCHANGE */

/****************************************************************************
 * Name: lpc54_spi_sndblock
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

static void lpc54_spi_sndblock(FAR struct spi_dev_s *dev,
                               FAR const void *buffer, size_t nwords)
{
  FAR struct lpc54_spidev_s *priv = (FAR struct lpc54_spidev_s *)dev;

  spiinfo("buffer=%p nwords=%d\n", buffer, nwords);
  DEBUGASSERT(priv != NULL && buffer != NULL);

#ifdef CONFIG_LPC54_SPI_WIDEDATA
  /* If the data with is > 8-bits, then handle this transfer with
   * lpc54_spi_sndblock16().
   */

  if (priv->nbits > 8)
    {
      lpc54_spi_sndblock16(priv, buffer, nwords);
    }

  /* Otherwise, let lpc54_spi_sndblock8() do the job */

  else if (priv->nbits > 8)
#endif
    {
      lpc54_spi_sndblock8(priv, buffer, nwords);
    }
}

/****************************************************************************
 * Name: lpc54_spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to receive data
 *   nwords - the length of data that can be received in the buffer in
 *            number of words.  The wordsize is determined by the number of
 *            bits-per-word selected for the SPI interface.  If nbits <= 8,
 *            the data is packed into uint8_t's; if nbits >8, the data is
 *            packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc54_spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                                size_t nwords)
{
  FAR struct lpc54_spidev_s *priv = (FAR struct lpc54_spidev_s *)dev;

  spiinfo("buffer=%p nwords=%d\n", buffer, nwords);
  DEBUGASSERT(priv != NULL && buffer != NULL);

#ifdef CONFIG_LPC54_SPI_WIDEDATA
  /* If the data with is > 8-bits, then handle this transfer with
   * lpc54_spi_recvblock16().
   */

  if (priv->nbits > 8)
    {
      lpc54_spi_recvblock16(priv, buffer, nwords);
    }

  /* Otherwise, let lpc54_spi_recvblock8() do the job */

  else if (priv->nbits > 8)
#endif
    {
      lpc54_spi_recvblock8(priv, buffer, nwords);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *   0 - SPI0
 *   1 - SPI1
 *   ...
 *   9 - SSP9
 *
 * Input Parameters:
 *   port - SPI peripheral number.  0..9
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *lpc54_spibus_initialize(int port)
{
  struct lpc54_spidev_s *priv;
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  /* Configure the requested SPI peripheral */

  /* NOTE:  The basic FLEXCOMM initialization was performed in
   * lpc54_lowputc.c.
   */

#ifdef CONFIG_LPC54_SPI0_MASTER
  if (port == 0)
    {
      /* Attach 12 MHz clock to FLEXCOMM0 */

      lpc54_flexcomm0_enableclk();

      /* Set FLEXCOMM0 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM0_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi0_dev;
      priv->base     = LPC54_FLEXCOMM0_BASE;
      priv->irq      = LPC54_IRQ_FLEXCOMM0;
      priv->fclock   = BOARD_FLEXCOMM0_FCLK;
      priv->dev.ops  = &g_spi0_ops;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_SPI0_SCK);
      lpc54_gpio_config(GPIO_SPI0_MOSI);
      lpc54_gpio_config(GPIO_SPI0_MISO);

      /* Set up the FLEXCOMM0 function clock */

      putreg32(BOARD_FLEXCOMM0_CLKSEL, LPC54_SYSCON_FCLKSEL0);
    }
  else
#endif
#ifdef CONFIG_LPC54_SPI1_MASTER
  if (port == 1)
    {
      /* Attach 12 MHz clock to FLEXCOMM1 */

      lpc54_flexcomm1_enableclk();

      /* Set FLEXCOMM1 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM1_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi1_dev;
      priv->base     = LPC54_FLEXCOMM1_BASE;
      priv->irq      = LPC54_IRQ_FLEXCOMM1;
      priv->fclock   = BOARD_FLEXCOMM1_FCLK;
      priv->dev.ops  = &g_spi1_ops;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_SPI1_SCK);
      lpc54_gpio_config(GPIO_SPI1_MOSI);
      lpc54_gpio_config(GPIO_SPI1_MISO);

      /* Set up the FLEXCOMM1 function clock */

      putreg32(BOARD_FLEXCOMM1_CLKSEL, LPC54_SYSCON_FCLKSEL1);
    }
  else
#endif
#ifdef CONFIG_LPC54_SPI2_MASTER
  if (port == 2)
    {
      /* Attach 12 MHz clock to FLEXCOMM2 */

      lpc54_flexcomm2_enableclk();

      /* Set FLEXCOMM2 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM2_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi2_dev;
      priv->base     = LPC54_FLEXCOMM2_BASE;
      priv->irq      = LPC54_IRQ_FLEXCOMM2;
      priv->fclock   = BOARD_FLEXCOMM2_FCLK;
      priv->dev.ops  = &g_spi2_ops;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_SPI2_SCK);
      lpc54_gpio_config(GPIO_SPI2_MOSI);
      lpc54_gpio_config(GPIO_SPI2MISO);

      /* Set up the FLEXCOMM2 function clock */

      putreg32(BOARD_FLEXCOMM2_CLKSEL, LPC54_SYSCON_FCLKSEL2);
    }
  else
#endif
#ifdef CONFIG_LPC54_SPI3_MASTER
  if (port == 3)
    {
      /* Attach 12 MHz clock to FLEXCOMM3 */

      lpc54_flexcomm3_enableclk();

      /* Set FLEXCOMM3 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM3_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi3_dev;
      priv->base     = LPC54_FLEXCOMM3_BASE;
      priv->irq      = LPC54_IRQ_FLEXCOMM3;
      priv->fclock   = BOARD_FLEXCOMM3_FCLK;
      priv->dev.ops  = &g_spi3_ops;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_SPI3_SCK);
      lpc54_gpio_config(GPIO_SPI3_MOSI);
      lpc54_gpio_config(GPIO_SPI3_MISO);

      /* Set up the FLEXCOMM3 function clock */

      putreg32(BOARD_FLEXCOMM3_CLKSEL, LPC54_SYSCON_FCLKSEL3);
    }
  else
#endif
#ifdef CONFIG_LPC54_SPI4_MASTER
  if (port == 4)
    {
      /* Attach 12 MHz clock to FLEXCOMM4 */

      lpc54_flexcomm4_enableclk();

      /* Set FLEXCOMM4 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM4_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi4_dev;
      priv->base     = LPC54_FLEXCOMM4_BASE;
      priv->irq      = LPC54_IRQ_FLEXCOMM4;
      priv->fclock   = BOARD_FLEXCOMM4_FCLK;
      priv->dev.ops  = &g_spi4_ops;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_SPI4_SCK);
      lpc54_gpio_config(GPIO_SPI4_MOSI);
      lpc54_gpio_config(GPIO_SPI4_MISO);

      /* Set up the FLEXCOMM4 function clock */

      putreg32(BOARD_FLEXCOMM4_CLKSEL, LPC54_SYSCON_FCLKSEL4);
    }
  else
#endif
#ifdef CONFIG_LPC54_SPI5_MASTER
  if (port == 5)
    {
      /* Attach 12 MHz clock to FLEXCOMM5 */

      lpc54_flexcomm5_enableclk();

      /* Set FLEXCOMM5 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM5_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi5_dev;
      priv->base     = LPC54_FLEXCOMM5_BASE;
      priv->irq      = LPC54_IRQ_FLEXCOMM5;
      priv->fclock   = BOARD_FLEXCOMM5_FCLK;
      priv->dev.ops  = &g_spi5_ops;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_SPI5_SCK);
      lpc54_gpio_config(GPIO_SPI5_MOSI);
      lpc54_gpio_config(GPIO_SPI5_MISO);

      /* Set up the FLEXCOMM5 function clock */

      putreg32(BOARD_FLEXCOMM5_CLKSEL, LPC54_SYSCON_FCLKSEL5);
    }
  else
#endif
#ifdef CONFIG_LPC54_SPI6_MASTER
  if (port == 6)
    {
      /* Attach 12 MHz clock to FLEXCOMM6 */

      lpc54_flexcomm6_enableclk();

      /* Set FLEXCOMM6 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM6_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi6_dev;
      priv->base     = LPC54_FLEXCOMM6_BASE;
      priv->irq      = LPC54_IRQ_FLEXCOMM6;
      priv->fclock   = BOARD_FLEXCOMM6_FCLK;
      priv->dev.ops  = &g_spi6_ops;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_SPI6_SCK);
      lpc54_gpio_config(GPIO_SPI6_MOSI);
      lpc54_gpio_config(GPIO_SPI6_MISO);

      /* Set up the FLEXCOMM6 function clock */

      putreg32(BOARD_FLEXCOMM6_CLKSEL, LPC54_SYSCON_FCLKSEL6);
    }
  else
#endif
#ifdef CONFIG_LPC54_SPI7_MASTER
  if (port == 7)
    {
      /* Attach 12 MHz clock to FLEXCOMM7 */

      lpc54_flexcomm7_enableclk();

      /* Set FLEXCOMM7 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM7_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi7_dev;
      priv->base     = LPC54_FLEXCOMM7_BASE;
      priv->irq      = LPC54_IRQ_FLEXCOMM7;
      priv->fclock   = BOARD_FLEXCOMM7_FCLK;
      priv->dev.ops  = &g_spi7_ops;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_SPI7_SCK);
      lpc54_gpio_config(GPIO_SPI7_MOSI);
      lpc54_gpio_config(GPIO_SPI7_MISO);

      /* Set up the FLEXCOMM7 function clock */

      putreg32(BOARD_FLEXCOMM7_CLKSEL, LPC54_SYSCON_FCLKSEL7);
    }
  else
#endif
#ifdef CONFIG_LPC54_SPI8_MASTER
  if (port == 8)
    {
      /* Attach 12 MHz clock to FLEXCOMM8 */

      lpc54_flexcomm8_enableclk();

      /* Set FLEXCOMM8 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM8_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi8_dev;
      priv->base     = LPC54_FLEXCOMM8_BASE;
      priv->irq      = LPC54_IRQ_FLEXCOMM8;
      priv->fclock   = BOARD_FLEXCOMM8_FCLK;
      priv->dev.ops  = &g_spi8_ops;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_SPI8_SCK);
      lpc54_gpio_config(GPIO_SPI8_MOSI);
      lpc54_gpio_config(GPIO_SPI8_MISO);

      /* Set up the FLEXCOMM8 function clock */

      putreg32(BOARD_FLEXCOMM8_CLKSEL, LPC54_SYSCON_FCLKSEL8);
    }
  else
#endif
#ifdef CONFIG_LPC54_SPI9_MASTER
  if (port == 9)
    {
      /* Attach 12 MHz clock to FLEXCOMM9 */

      lpc54_flexcomm9_enableclk();

      /* Set FLEXCOMM9 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM9_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi9_dev;
      priv->base     = LPC54_FLEXCOMM9_BASE;
      priv->irq      = LPC54_IRQ_FLEXCOMM9;
      priv->fclock   = BOARD_FLEXCOMM9_FCLK;
      priv->dev.ops  = &g_spi9_ops;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_SPI9_SCK);
      lpc54_gpio_config(GPIO_SPI9_MOSI);
      lpc54_gpio_config(GPIO_SPI9_MISO);

      /* Set up the FLEXCOMM9 function clock */

      putreg32(BOARD_FLEXCOMM9_CLKSEL, LPC54_SYSCON_FCLKSEL9);
    }
  else
#endif
    {
      return NULL;
    }

  leave_critical_section(flags);

  /* Set the initial SPI configuration */

  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  nxsem_init(&priv->exclsem, 0, 1);

  /* Configure master mode in mode 0:
   *
   * ENABLE    - Disabled for now      (0)
   * MASTER    - Master mode           (1)
   * LSBF      - MSB first             (0)
   * CPHA/CPOL - Mode 0                (0,0)
   * LOOP      - Disable loopback mode (0)
   * SPOLn     - Active low            (0,0,0)
   */

  regval  = lpc54_spi_getreg(priv, LPC54_SPI_CFG_OFFSET);
  regval &= ~(SPI_CFG_ENABLE | SPI_CFG_LSBF | SPI_CFG_CPHA | SPI_CFG_CPOL |
              SPI_CFG_LOOP | SPI_CFG_SPOL0 | SPI_CFG_SPOL1 | SPI_CFG_SPOL2 |
              SPI_CFG_SPOL3);
  regval |= SPI_CFG_MASTER;
  lpc54_spi_putreg(priv, LPC54_SPI_CFG_OFFSET, regval);

  /* Enable FIFOs */

  regval  = lpc54_spi_getreg(priv, LPC54_SPI_FIFOCFG_OFFSET);
  regval |= (SPI_FIFOCFG_EMPTYTX | SPI_FIFOCFG_EMPTYRX);
  lpc54_spi_putreg(priv, LPC54_SPI_FIFOCFG_OFFSET, regval);

  regval |= (SPI_FIFOCFG_ENABLETX | SPI_FIFOCFG_ENABLERX);
  lpc54_spi_putreg(priv, LPC54_SPI_FIFOCFG_OFFSET, regval);

  /* Set FIFO trigger levels:  Empty for Tx FIFO; 1 word for RxFIFO */

  regval  = lpc54_spi_getreg(priv, LPC54_SPI_FIFOCFG_OFFSET);
  regval &= ~(SPI_FIFOTRIG_TXLVL_MASK | SPI_FIFOTRIG_RXLVL_MASK);
  regval |= (SPI_FIFOTRIG_TXLVL_EMPTY | SPI_FIFOTRIG_RXLVL_NOTEMPTY);

  /* Enable generation of interrupts for selected FIFO trigger levels */

  regval |= (SPI_FIFOTRIG_TXLVLENA | SPI_FIFOTRIG_RXLVLENA);
  lpc54_spi_putreg(priv, LPC54_SPI_FIFOCFG_OFFSET, regval);

  /* Set the delay configuration (not used) */

  regval = (SPI_DLY_PRE_DELAY(0) | SPI_DLY_POST_DELAY(0) |
            SPI_DLY_FRAME_DELAY(0) | SPI_DLY_TRANSFER_DELAY(0));
  lpc54_spi_putreg(priv, LPC54_SPI_DLY_OFFSET, regval);

  /* Select a default frequency of approx. 400KHz */

  lpc54_spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* Enable the SPI peripheral */

  regval  = lpc54_spi_getreg(priv, LPC54_SPI_CFG_OFFSET);
  regval |= SPI_CFG_ENABLE;
  lpc54_spi_putreg(priv, LPC54_SPI_CFG_OFFSET, regval);

  return &priv->dev;
}

#endif /* HAVE_SPI_MASTER_DEVICE */

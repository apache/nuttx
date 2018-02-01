/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz-spi.c
 *
 *   Copyright (C) 2015-2017 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip/pic32mz-spi.h"
#include "chip/pic32mz-pps.h"
#include "pic32mz-spi.h"

#ifdef CONFIG_PIC32MZ_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* All SPI peripherals are clocked by PBCLK2 */

#define BOARD_PBCLOCK BOARD_PBCLK2

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure describes the fixed (ROM-able) configuration of the SPI
 * peripheral.
 */

struct pic32mz_config_s
{
  uint32_t         base;       /* SPI register base address */
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  uint8_t          firq;       /* SPI fault interrupt number */
  uint8_t          rxirq;      /* SPI receive done interrupt number */
  uint8_t          txirq;      /* SPI transfer done interrupt number */
#endif
  uint8_t          sdipps;     /* SDI peripheral pin selection */
  uint8_t          sdopps;     /* SDO peripheral pin selection */
  uintptr_t        sdoreg;     /* SDO peripheral pin configuration register */
};

/* This structure describes the state of the SPI driver */

struct pic32mz_dev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  FAR const struct pic32mz_config_s *config;
  sem_t            exclsem;    /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          mode;       /* Mode 0,1,2,3 */
  uint8_t          nbits;      /* Width of word in bits (8 to 16) */

#ifdef CONFIG_PIC32MZ_SPI_REGDEBUG
  bool             wrlast;     /* Last was a write */
  uint32_t         addrlast;   /* Last address */
  uint32_t         vallast;    /* Last value */
  int              ntimes;     /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Low-level register access */

#ifdef CONFIG_PIC32MZ_SPI_REGDEBUG
static bool     spi_checkreg(FAR struct pic32mz_dev_s *priv,
                  uintptr_t regaddr, uint32_t regvaql, bool wr);
static uint32_t spi_getreg(FAR struct pic32mz_dev_s *priv,
                  unsigned int offset);
static void     spi_putaddr(FAR struct pic32mz_dev_s *priv,
                  uintptr_t regaddr, uint32_t value);
#else
static inline uint32_t spi_getreg(FAR struct pic32mz_dev_s *priv,
                  unsigned int offset);
static inline void     spi_putaddr(FAR struct pic32mz_dev_s *priv,
                  uintptr_t regaddr, uint32_t value);
#endif
static inline void spi_putreg(FAR struct pic32mz_dev_s *priv,
                  unsigned int offset, uint32_t value);
static void     spi_exchange16(FAR struct pic32mz_dev_s *priv,
                   FAR const uint16_t *txbuffer, FAR uint16_t *rxbuffer,
                   size_t nwords);
static void     spi_exchange8(FAR struct pic32mz_dev_s *priv,
                   FAR const uint8_t *txbuffer, FAR uint8_t *rxbuffer,
                   size_t nbytes);

/* SPI methods */

static int      spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void     spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void     spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t ch);
static void     spi_exchange(FAR struct spi_dev_s *dev,
                   FAR const void *txbuffer, FAR void *rxbuffer,
                   size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void     spi_sndblock(FAR struct spi_dev_s *dev,
                   FAR const void *buffer, size_t nwords);
static void     spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                   size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI1

static const struct spi_ops_s g_spi1ops =
{
  .lock              = spi_lock,
  .select            = pic32mz_spi1select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                    /* Not supported */
#endif
  .status            = pic32mz_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi1cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi1register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static const struct pic32mz_config_s g_spi1config =
{
  .base              = PIC32MZ_SPI1_K1BASE,
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  .firq              = PIC32MZ_IRQ_SPI1F,
  .rxirq             = PIC32MZ_IRQ_SPI1RX,
  .txirq             = PIC32MZ_IRQ_SPI1TX,
#endif
  .sdipps            = BOARD_SDI1_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO1_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO1_PPS),
};

static struct pic32mz_dev_s g_spi1dev =
{
  .spidev            = { &g_spi1ops },
  .config            = &g_spi1config,
};
#endif

#ifdef CONFIG_PIC32MZ_SPI2
static const struct spi_ops_s g_spi2ops =
{
  .lock              = spi_lock,
  .select            = pic32mz_spi2select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi2cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi2register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static const struct pic32mz_config_s g_spi2config =
{
  .base              = PIC32MZ_SPI2_K1BASE,
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  .firq              = PIC32MZ_IRQ_SPI2F,
  .rxirq             = PIC32MZ_IRQ_SPI2RX,
  .txirq             = PIC32MZ_IRQ_SPI2TX,
#endif
  .sdipps            = BOARD_SDI2_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO2_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO2_PPS),
};

static struct pic32mz_dev_s g_spi2dev =
{
  .spidev            = { &g_spi2ops },
  .config            = &g_spi2config,
};
#endif

#ifdef CONFIG_PIC32MZ_SPI3
static const struct spi_ops_s g_spi3ops =
{
  .lock              = spi_lock,
  .select            = pic32mz_spi3select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi3cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi3register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static const struct pic32mz_config_s g_spi3config =
{
  .base              = PIC32MZ_SPI3_K1BASE,
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  .firq              = PIC32MZ_IRQ_SPI3F,
  .rxirq             = PIC32MZ_IRQ_SPI3RX,
  .txirq             = PIC32MZ_IRQ_SPI3TX,
#endif
  .sdipps            = BOARD_SDI3_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO3_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO3_PPS),
};

static struct pic32mz_dev_s g_spi3dev =
{
  .spidev            = { &g_spi3ops },
  .config            = &g_spi3config,
};
#endif

#ifdef CONFIG_PIC32MZ_SPI4
static const struct spi_ops_s g_spi4ops =
{
  .lock              = spi_lock,
  .select            = pic32mz_spi4select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi4status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi4cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi4register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static const struct pic32mz_config_s g_spi4config =
{
  .base              = PIC32MZ_SPI4_K1BASE,
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  .firq              = PIC32MZ_IRQ_SPI4F,
  .rxirq             = PIC32MZ_IRQ_SPI4RX,
  .txirq             = PIC32MZ_IRQ_SPI4TX,
#endif
  .sdipps            = BOARD_SDI4_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO4_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO4_PPS),
};

static struct pic32mz_dev_s g_spi4dev =
{
  .spidev            = { &g_spi4ops },
  .config            = &g_spi4config,
};
#endif

#ifdef CONFIG_PIC32MZ_SPI5
static const struct spi_ops_s g_spi5ops =
{
  .lock              = spi_lock,
  .select            = pic32mz_spi5select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi5status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi5cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi5register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static const struct pic32mz_config_s g_spi5config =
{
  .base              = PIC32MZ_SPI5_K1BASE,
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  .firq              = PIC32MZ_IRQ_SPI5F,
  .rxirq             = PIC32MZ_IRQ_SPI5RX,
  .txirq             = PIC32MZ_IRQ_SPI5TX,
#endif
  .sdipps            = BOARD_SDI5_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO5_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO5_PPS),
};

static struct pic32mz_dev_s g_spi5dev =
{
  .spidev            = { &g_spi5ops },
  .config            = &g_spi5config,
};
#endif

#ifdef CONFIG_PIC32MZ_SPI6
static const struct spi_ops_s g_spi6ops =
{
  .lock              = spi_lock,
  .select            = pic32mz_spi6select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi6status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi6cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi6register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static const struct pic32mz_config_s g_spi6config =
{
  .base              = PIC32MZ_SPI6_K1BASE,
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  .firq              = PIC32MZ_IRQ_SPI6F,
  .rxirq             = PIC32MZ_IRQ_SPI6RX,
  .txirq             = PIC32MZ_IRQ_SPI6TX,
#endif
  .sdipps            = BOARD_SDI6_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO6_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO6_PPS),
};

static struct pic32mz_dev_s g_spi6dev =
{
  .spidev            = { &g_spi6ops },
  .config            = &g_spi6config,
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   priv    - SPI driver instance
 *   regaddr - The address of the register to write to
 *   regval  - The value to be written
 *   wr      - True: write access
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   false: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI_REGDEBUG
static bool spi_checkreg(struct pic32mz_dev_s *priv, uintptr_t regaddr,
                         uint32_t regval, bool wr)
{
  if (wr      == priv->wrlast &&   /* Same kind of access? */
      regval  == priv->vallast &&  /* Same value? */
      regaddr == priv->addrlast)   /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          _info("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast   = wr;
      priv->vallast  = regval;
      priv->addrlast = regaddr;
      priv->ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Return the contents of one, 32-bit SPI register
 *
 * Input Parameters:
 *   priv   - A pointer to a PIC32MZ SPI state structure
 *   offset - Offset from the SPI base address to the register of interest
 *
 * Returned Value:
 *   The current contents of the register
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI_REGDEBUG
static uint32_t spi_getreg(FAR struct pic32mz_dev_s *priv,
                           unsigned int offset)
{
  uintptr_t regaddr;
  uint32_t regval;

  /* Read the register value */

  regaddr = priv->config->base + offset;
  regval  = getreg32(regaddr);

  /* Should we print something? */

  if (spi_checkreg(priv, regaddr, regval, false))
    {
      /* Yes.. */

      _info("%08lx->%08lx\n",
           (unsigned long)regaddr, (unsigned long)regval);
    }

  /* Return the value read */

  return regval;
}
#else
static inline uint32_t spi_getreg(FAR struct pic32mz_dev_s *priv,
                                  unsigned int offset)
{
  return getreg32(priv->config->base + offset);
}
#endif

/****************************************************************************
 * Name: spi_putaddr
 *
 * Description:
 *   Write a 32-bit value value an absolute address
 *
 * Input Parameters:
 *   priv    - A pointer to a PIC32MZ SPI state structure
 *   regaddr - Address to write to
 *   regval  - The value to write to the SPI register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI_REGDEBUG
static void spi_putaddr(FAR struct pic32mz_dev_s *priv, uintptr_t regaddr,
                        uint32_t regval)
{
  /* Should we print something? */

  if (spi_checkreg(priv, regaddr, regval, true))
    {
      /* Yes.. */

      _info("%08lx<-%08lx\n",
           (unsigned long)regaddr, (unsigned long)regval);
    }

  /* Write the value to the register */

  putreg32(regval, regaddr);
}
#else
static inline void spi_putaddr(FAR struct pic32mz_dev_s *priv,
                               uintptr_t regaddr, uint32_t regval)
{
  /* Write the value to the register */

  putreg32(regval, regaddr);
}
#endif

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a value to one, 32-bit SPI register
 *
 * Input Parameters:
 *   priv   - A pointer to a PIC32MZ SPI state structure
 *   offset - Offset from the SPI base address to the register of interest
 *   value  - The value to write to the SPI register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_putreg(FAR struct pic32mz_dev_s *priv,
                              unsigned int offset, uint32_t regval)
{
  spi_putaddr(priv, priv->config->base + offset, regval);
}

/****************************************************************************
 * Name: spi_exchange8
 *
 * Description:
 *   Exchange a block of 8-bit data from SPI.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of byres that to be exchanged
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_exchange8(FAR struct pic32mz_dev_s *priv,
                          FAR const uint8_t *txbuffer, FAR uint8_t *rxbuffer,
                          size_t nbytes)
{
  uint32_t regval;
  uint8_t data;

  spiinfo("nbytes: %d\n", nbytes);
  while (nbytes)
    {
      /* Write the data to transmitted to the SPI Data Register */

      if (txbuffer)
        {
          data = *txbuffer++;
        }
      else
        {
          data = 0xff;
        }

      spi_putreg(priv, PIC32MZ_SPI_BUF_OFFSET, (uint32_t)data);

#ifdef CONFIG_PIC32MZ_SPI_ENHBUF
      /* Wait for the SPIRBE bit in the SPI Status Register to be set to 0. In
       * enhanced buffer mode, the SPIRBE bit will be cleared in  when the
       * receive buffer is not empty.
       */

     while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) & SPI_STAT_SPIRBE) != 0);
#else
      /* Wait for the SPIRBF bit in the SPI Status Register to be set to 1. In
       * normal mode, the SPIRBF bit will be set when receive data is available.
       */

     while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) & SPI_STAT_SPIRBF) == 0);
#endif

     /* Read from the buffer register to clear the status bit */

     regval = spi_getreg(priv, PIC32MZ_SPI_BUF_OFFSET);
     if (rxbuffer)
       {
         *rxbuffer++ = (uint8_t)regval;
       }
     else
       {
         UNUSED(regval);
       }

     nbytes--;
    }
}

/****************************************************************************
 * Name: spi_exchange16
 *
 * Description:
 *   Exchange a block of 16-bit data from SPI.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to recieve data
 *   nwords   - the length of data that to be exchanged in units of words.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_exchange16(FAR struct pic32mz_dev_s *priv,
                           FAR const uint16_t *txbuffer,
                           FAR uint16_t *rxbuffer, size_t nwords)
{
  uint32_t regval;
  uint16_t data;

  spiinfo("nwords: %d\n", nwords);
  while (nwords)
    {
      /* Write the data to transmitted to the SPI Data Register */

      if (txbuffer)
        {
          data = *txbuffer++;
        }
      else
        {
          data = 0xffff;
        }

      spi_putreg(priv, PIC32MZ_SPI_BUF_OFFSET, (uint32_t)data);

#ifdef CONFIG_PIC32MZ_SPI_ENHBUF
      /* Wait for the SPIRBE bit in the SPI Status Register to be set to 0. In
       * enhanced buffer mode, the SPIRBE bit will be cleared in  when the
       * receive buffer is not empty.
       */

     while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) & SPI_STAT_SPIRBE) != 0);
#else
      /* Wait for the SPIRBF bit in the SPI Status Register to be set to 1. In
       * normal mode, the SPIRBF bit will be set when receive data is available.
       */

     while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) & SPI_STAT_SPIRBF) == 0);
#endif

     /* Read from the buffer register to clear the status bit */

     regval = spi_getreg(priv, PIC32MZ_SPI_BUF_OFFSET);
     if (rxbuffer)
       {
         *rxbuffer++ = (uint16_t)regval;
       }
     else
       {
         UNUSED(regval);
       }

     nwords--;
    }
}

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
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

static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct pic32mz_dev_s *priv = (FAR struct pic32mz_dev_s *)dev;
  int ret;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      do
        {
          ret = nxsem_wait(&priv->exclsem);

          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          DEBUGASSERT(ret == OK || ret == -EINTR);
        }
      while (ret == -EINTR);
    }
  else
    {
      (void)nxsem_post(&priv->exclsem);
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: spi_setfrequency
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

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  FAR struct pic32mz_dev_s *priv = (FAR struct pic32mz_dev_s *)dev;
  uint32_t divisor;
  uint32_t actual;
  uint32_t regval;

  spiinfo("Old frequency: %d actual: %d New frequency: %d\n",
          priv->frequency, priv->actual, frequency);

  /* Check if the requested frequency is the same as the frequency selection */

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }

  /* Calculate the divisor
   *
   * frequency = BOARD_PBCLOCK / (2 * divisor), or
   * divisor  = (BOARD_PBCLOCK / 2) / frequency
   */

  divisor = (BOARD_PBCLOCK / 2) / frequency;

  /* The a BRG register value is that divisor minus one
   *
   * frequency = BOARD_PBCLOCK /(2 * (BRG + 1)), or
   * BRG       = (BOARD_PBCLOCK / 2) / frequency - 1
   */

  regval = divisor;
  if (regval > 0)
    {
      regval--;
    }

  /* Save the new BRG value */

  spi_putreg(priv, PIC32MZ_SPI_BRG_OFFSET, regval);
  spiinfo("PBCLOCK: %d frequency: %d divisor: %d BRG: %d\n",
          BOARD_PBCLOCK, frequency, divisor, regval);

  /* Calculate the new actual frequency.
   *
   * frequency = BOARD_PBCLOCK / (2 * divisor)
   */

  actual = (BOARD_PBCLOCK / 2) / divisor;

  /* Save the frequency setting */

  priv->frequency = frequency;
  priv->actual    = actual;

  spiinfo("New frequency: %d Actual: %d\n", frequency, actual);
  return actual;
}

/****************************************************************************
 * Name: spi_setmode
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

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct pic32mz_dev_s *priv = (FAR struct pic32mz_dev_s *)dev;
  uint32_t regval;

  spiinfo("Old mode: %d New mode: %d\n", priv->mode, mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set CON register appropriately.
       *
       * Standard terminology is as follows:
       *
       *   Mode  CPOL  CPHA
       *     0     0     0
       *     1     0     1
       *     2     1     0
       *     3     1     1
       *
       *   CPOL=0: The inactive value of the clock is zero
       *    CPOL=1: The inactive value of the clock is one
       *   CPHA=0: Data is captured on the clock's inactive-to-active edge and
       *           data is propagated on a active-to-inactive edge.
       *   CPHA=1: Data is captured on the clock's active-to-inactive edge and
       *           data is propagated on a active-to-inactive edge.
       *
       * CON Register mapping:
       *   CPOL=0 corresponds to CON:CKP=0; CPOL=1 corresponds to CON:CKP=1
       *   CPHA=0 corresponds to CON:CKE=1; CPHA=1 corresponds to CON:CKE=1
       *
       * In addition, the CON register supports SMP: SPI Data Input Sample
       * Phase bit:
       *
       *    1 = Input data sampled at end of data output time
       *    0 = Input data sampled at middle of data output time
       *
       * Which is hardcoded to 1.
       */

      regval = spi_getreg(priv, PIC32MZ_SPI_CON_OFFSET);
      regval &= ~(SPI_CON_CKP | SPI_CON_CKE);

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          regval |= SPI_CON_CKE;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          regval |= SPI_CON_CKP;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          regval |= (SPI_CON_CKP | SPI_CON_CKE);
          break;

        default:
          DEBUGASSERT(FALSE);
          return;
        }

      spi_putreg(priv, PIC32MZ_SPI_CON_OFFSET, regval);
      spiinfo("CON: %08x\n", regval);

      /* Save the mode so that subsequent re-configuratins will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct pic32mz_dev_s *priv = (FAR struct pic32mz_dev_s *)dev;
  uint32_t setting;
  uint32_t regval;

  spiinfo("Old nbits: %d New nbits: %d\n", priv->nbits, nbits);

  /* Has the number of bits changed? */

  DEBUGASSERT(priv && nbits > 7 && nbits < 17);
  if (nbits != priv->nbits)
    {
      /* Yes... Set the CON register appropriately */

      if (nbits == 8)
        {
          setting = SPI_CON_MODE_8BIT;
        }
      else if (nbits == 16)
        {
          setting = SPI_CON_MODE_8BIT;
        }
      else if (nbits == 32)
        {
          setting = SPI_CON_MODE_8BIT;
        }
      else
        {
          spierr("ERROR: Unsupported nbits: %d\n", nbits);
          return;
        }

      regval = spi_getreg(priv, PIC32MZ_SPI_CON_OFFSET);
      regval &= ~SPI_CON_MODE_MASK;
      regval |= setting;
      regval = spi_getreg(priv, PIC32MZ_SPI_CON_OFFSET);
      spiinfo("CON: %08x\n", regval);

      /* Save the selection so the subsequence re-configurations will be
       * faster
       */

      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: spi_send
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

static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  FAR struct pic32mz_dev_s *priv = (FAR struct pic32mz_dev_s *)dev;

  DEBUGASSERT(priv);
  if (priv->nbits > 8)
    {
      uint16_t txword;
      uint16_t rxword;

      /* spi_exchange16() can do this. */

      txword = wd;
      rxword = 0;
      spi_exchange16(priv, &txword, &rxword, 1);

      spiinfo("Sent %04x received %04x\n", txword, rxword);
      return rxword;
    }
  else
    {
      uint8_t txbyte;
      uint8_t rxbyte;

      /* spi_exchange8() can do this. */

      txbyte = (uint8_t)wd;
      rxbyte = (uint8_t)0;
      spi_exchange8(priv, &txbyte, &rxbyte, 1);

      spiinfo("Sent %02x received %02x\n", txbyte, rxbyte);
      return (uint16_t)rxbyte;
    }
}

/****************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Exchange a block of data from SPI..
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to recieve data
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

static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  FAR struct pic32mz_dev_s *priv = (FAR struct pic32mz_dev_s *)dev;

  DEBUGASSERT(priv);
  if (priv->nbits > 8)
    {
      /* spi_exchange16() can do this. */

      spi_exchange16(priv, (FAR const uint16_t *)txbuffer,
                     (FAR uint16_t *)rxbuffer, nwords);
    }
  else
    {
      /* spi_exchange8() can do this. */

      spi_exchange8(priv, (FAR const uint8_t *)txbuffer,
                    (FAR uint8_t *)rxbuffer, nwords);
    }
}

/****************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nwords - the length of data to send from the buffer in number of
 *            words.  The wordsize is determined by the number of
 *            bits-per-word selected for the SPI interface.  If nbits <= 8,
 *            the data is packed into uint8_t's; if nbits >8, the data is
 *            packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer,
                         size_t nwords)
{
  /* spi_exchange() can do this. */

  spi_exchange(dev, buffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   dev   -  Device-specific state data
 *   buffer - A pointer to the buffer in which to recieve data
 *   nwords - the length of data that can be received in the buffer in
 *            number of words.  The wordsize is determined by the number of
 *            bits-per-word selected for the SPI interface.  If nbits <= 8,
 *            the data is  packed into uint8_t's; if nbits >8, the data is
 *            packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                          size_t nwords)
{
  /* spi_exchange() can do this. */

  spi_exchange(dev, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *pic32mz_spibus_initialize(int port)
{
  FAR struct pic32mz_dev_s *priv;
  uintptr_t regaddr;
  irqstate_t flags;
  uint32_t regval;

  spiinfo("port: %d\n", port);

  /* Select the SPI state structure and SDI PPS register for this port */

#ifdef CONFIG_PIC32MZ_SPI1
  if (port == 1)
    {
      priv    = &g_spi1dev;
      regaddr = PIC32MZ_SDI1R;
    }
  else
#endif
#ifdef CONFIG_PIC32MZ_SPI2
  if (port == 2)
    {
      priv = &g_spi2dev;
      regaddr = PIC32MZ_SDI2R;
    }
  else
#endif
#ifdef CONFIG_PIC32MZ_SPI3
  if (port == 3)
    {
      priv = &g_spi3dev;
      regaddr = PIC32MZ_SDI3R;
    }
  else
#endif
#ifdef CONFIG_PIC32MZ_SPI4
  if (port == 4)
    {
      priv = &g_spi4dev;
      regaddr = PIC32MZ_SDI4R;
    }
  else
#endif
#ifdef CONFIG_PIC32MZ_SPI5
  if (port == 5)
    {
      priv = &g_spi5dev;
      regaddr = PIC32MZ_SDI5R;
    }
  else
#endif
#ifdef CONFIG_PIC32MZ_SPI6
  if (port == 6)
    {
      priv = &g_spi6dev;
      regaddr = PIC32MZ_SDI6R;
    }
  else
#endif
   {
     spierr("ERROR: Unsuppport port: %d\n", port);
     return NULL;
   }

  /* Disable SPI interrupts */

  flags = enter_critical_section();
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  up_disable_irq(priv->config->firq);
  up_disable_irq(priv->config->rxirq);
  up_disable_irq(priv->config->txirq);
#endif

  /* Stop and reset the SPI module by clearing the ON bit in the CON register. */

  spi_putreg(priv, PIC32MZ_SPI_CON_OFFSET, 0);

  /* Clear the receive buffer by reading from the BUF register */

  regval = spi_getreg(priv, PIC32MZ_SPI_BUF_OFFSET);

  /* Configure SPI SDI (input) and SDO (output) pins. SS (output) pins are
   * managed as GPIOs; CLK (output) pins are not selectable.
   */

  spi_putaddr(priv, regaddr, (uint32_t)priv->config->sdipps);
  spi_putaddr(priv, priv->config->sdoreg, (uint32_t)priv->config->sdopps);

#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  /* Attach the interrupt handlers.  We do this early to make sure that the
   * resources are available.
   */

  ret = irq_attach(priv->config->rxirq, spi_interrupt, NULL);
  if (ret < 0)
    {
      spierr("ERROR: Failed to attach RX interrupt: %d port: %d\n",
             priv->config->rxirq, port);
      goto errout;
    }

  ret = irq_attach(priv->config->txirq, spi_interrupt, NULL);
  if (ret < 0)
    {
      spierr("ERROR: Failed to attach TX interrupt: %d port: %d\n",
             priv->tconfig->xirq, port);
      goto errout_with_rxirq;
    }

  ret = irq_attach(priv->config->firq, spi_interrupt, NULL);
  if (ret < 0)
    {
      spierr("ERROR: Failed to attach fault interrupt: %d port: %d\n",
             priv->config->firq, port);
      goto errout_with_txirq;
    }
#endif

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* Clear the SPIROV overflow bit (SPIxSTAT:6). */

  spi_putreg(priv, PIC32MZ_SPI_STATCLR_OFFSET, SPI_STAT_SPIROV);

  /* Initial settings 8 bit +  master mode + mode 0.  NOTE that MSSEN
   * not set:  The slave select pin must be driven manually via the
   * board-specific pic32mz_spiNselect() interface.
   */

  regval = (SPI_CON_MSTEN | SPI_CON_SMP | SPI_CON_MODE_8BIT | SPI_CON_ON);

  /* Set the ENHBUF bit if using Enhanced Buffer mode. */

#ifdef CONFIG_PIC32MZ_SPI_ENHBUF
  regval |= (SPI_CON_ENHBUF | SPI_CON_SRXISEL_HALF | SPI_CON_STXISEL_HALF);
#endif
  spi_putreg(priv, PIC32MZ_SPI_CON_OFFSET, regval);
  spiinfo("CON: %08x\n", regval);

  /* Set the initial SPI configuration */

  priv->nbits = 8;
  priv->mode  = SPIDEV_MODE0;

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  nxsem_init(&priv->exclsem, 0, 1);

#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  /* Enable interrupts at the SPI controller */

  up_enable_irq(priv->config->firq);
  up_enable_irq(priv->config->rxirq);
  up_enable_irq(priv->config->txirq);
#endif

  /* Enable interrupts at the interrupt controller */

  leave_critical_section(flags);
  return &priv->spidev;

#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
errout_with_txirq:
  irq_detatch(priv->config->txirq);
errout_with_rxirq:
  irq_detatch(priv->config->rxirq);
errout:
  leave_critical_section(flags);
  return NULL;
#endif
}

#endif /* CONFIG_PIC32MZ_SPI */

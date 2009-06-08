/****************************************************************************
 * config/olimex-strp711/src/up_spi.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <nuttx/spi.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/spi.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "str71x_internal.h"

#if defined(CONFIG_STR71X_BSPI0) || defined(CONFIG_STR71X_BSPI1)

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_STR714X_BSPI0_TXFIFO_DEPTH
#  define CONFIG_STR714X_BSPI0_TXFIFO_DEPTH 8
#endif

#ifndef CONFIG_STR714X_BSPI0_RXFIFO_DEPTH
#  define CONFIG_STR714X_BSPI0_RXFIFO_DEPTH 8
#endif

#ifndef CONFIG_STR714X_BSPI1_TXFIFO_DEPTH
#  define CONFIG_STR714X_BSPI1_TXFIFO_DEPTH 8
#endif

#ifndef CONFIG_STR714X_BSPI1_RXFIFO_DEPTH
#  define CONFIG_STR714X_BSPI1_RXFIFO_DEPTH 8
#endif

#if defined(CONFIG_STR71X_HDLC) && defined (CONFIG_STR71X_BSPI1)
#  warning "BSPI1 GPIO usage conflicts with HDLC"
#endif

/****************************************************************************
 * On the Olimex-STR-STR-P711, BSPI0 is not connected on board, but is
 * available on a header for use in the prototyping area.  BSPI connects
 * to the MMC/SD card slot.
 *
 * GPIO pin configurations (STR710/STR711,2,5).
 * BSP0:
 *   PIN     NORMAL  ALTERNATE  Olimex-STR-STR-P711 Connection
 *   123/52  P0.0    S0.MISO *  UEXT-3 (Not connected on board)
 *   124/53  P0.1    S0.MOSI *  UEXT-4  " " "       " "" "   "
 *   125/54  P0.2    S0.SCLK ** UEXT-5  " " "       " "" "   "
 *   126/55  P0.3   ~SO.SS   ** UEXT-6  " " "       " "" "   "
 *
 *  * Programming the AF function selects UART3 by default.  BSPI must be
 *    enabled with the SPI_EN bit in the BOOTCR register
 * ** Programming the AF function selects I2C1 by default.  BSPI must be
 *    enabled with the SPI_EN bit in the BOOTCR register
 *
 * BSP1
 *   PIN     NORMAL  ALTERNATE Olimex-STR-STR-P711 Connection
 *   127/56  P0.4    S1.MISO   SD_CARDBOT DAT0/D0
 *   140/60  P0.5    S1.MOSI   SD_CARDBOT CMD/DI
 *   141/61  P0.6    S1.SCLK   SD_CARDBOT CLK/SCLK
 *   142/62  P0.7   ~S1.SS     SD_CARDBOT CD/DAT/CS
 *
 * Two GPIO pins also connect to the MMC/SD slot:
 *
 *   PIN     NORMAL  ALTERNATE Olimex-STR-STR-P711 Connection
 *   106/46  P1.10   USB clock MMC/SD write protect (WP)
 *   111/49  P1.15   HDLC xmit MMC/SD card present (CP)
 *
 ****************************************************************************/

#define BSPI0_GPIO0_MISO (0x0001)
#define BSPI0_GPIO0_MOSI (0x0002)
#define BSPI0_GPIO0_SCLK (0x0004)
#define BSPI0_GPIO0_SS   (0x0008)

#define BSPIO_GPIO0_ALT  (BSPI0_GPIO0_MISO|BSPI0_GPIO0_MOSI|BSPI0_GPIO0_SCLK)
#define BSPIO_GPIO0_OUT   BSPI0_GPIO0_SS
#define BSPIO_GPIO0_ALL  (0x000f)

#define BSPI1_GPIO0_MISO (0x0010)
#define BSPI1_GPIO0_MOSI (0x0020)
#define BSPI1_GPIO0_SCLK (0x0040)
#define BSPI1_GPIO0_SS   (0x0080)

#define BSPI1_GPIO0_ALT  (BSPI1_GPIO0_MISO|BSPI1_GPIO0_MOSI|BSPI1_GPIO0_SCLK)
#define BSPI1_GPIO0_OUT   BSPI1_GPIO0_SS
#define BSPI1_GPIO0_ALL  (0x00f0)

#define MMCSD_GPIO1_WPIN (0x0400)
#define MMCSD_GPIO1_CPIN (0x8000)
#define MMCSD_GPIO1_ALL  (MMCSD_GPIO1_WPIN|MMCSD_GPIO1_CPIN)

/* Configuration register settings ******************************************/

#if CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 1
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE1
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 2
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE12
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 3
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE13
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 4
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE14
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 5
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE15
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 6
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE16
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 7
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE17
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 8
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE18
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 9
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE19
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 10
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE110
#else
#  error "Invaid RX FIFO depth setting"
#endif

#define STR71X_BSPI0_CSR1DISABLE STR71X_BSPI0_CSR1RXFIFODEPTH
#define STR71X_BSPI0_CSR1ENABLE  (STR71X_BSPICSR1_BSPE|STR71X_BSPICSR1_MSTR|STR71X_BSPI0_CSR1RXFIFODEPTH)

#if CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 1
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE1
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 2
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE12
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 3
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE13
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 4
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE14
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 5
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE15
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 6
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE16
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 7
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE17
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 8
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE18
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 9
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE19
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 10
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE110
#else
#  error "Invaid TX FIFO depth setting"
#endif

#define STR71X_BSPI0_CSR2VALUE STR71X_BSPI0_CSR1TXFIFODEPTH

#if CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 1
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE1
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 2
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE12
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 3
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE13
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 4
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE14
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 5
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE15
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 6
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE16
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 7
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE17
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 8
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE18
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 9
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE19
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 10
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE110
#else
#  error "Invaid RX FIFO depth setting"
#endif

#define STR71X_BSPI1_CSR1DISABLE STR71X_BSPI1_CSR1RXFIFODEPTH
#define STR71X_BSPI1_CSR1ENABLE  (STR71X_BSPICSR1_BSPE|STR71X_BSPICSR1_MSTR|STR71X_BSPI1_CSR1RXFIFODEPTH)

#if CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 1
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE1
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 2
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE12
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 3
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE13
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 4
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE14
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 5
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE15
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 6
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE16
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 7
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE17
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 8
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE18
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 9
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE19
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 10
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE110
#else
#  error "Invaid TX FIFO depth setting"
#endif

#define STR71X_BSPI1_CSR2VALUE STR71X_BSPI1_CSR1TXFIFODEPTH

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct str71x_spidev_s
{
  struct spi_dev_s spidev;  /* Externally visible part of the SPI interface */
  uint32           spibase; /* BSPIn base address */
  uint16           csbit;   /* BSPIn SS bit int GPIO0 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint16 spi_getreg(FAR struct str71x_spidev_s *priv, ubyte offset);
static inline void   spi_putreg(FAR struct str71x_spidev_s *priv, ubyte offset, uint16 value);

/* SPI methods */

static void   spi_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, boolean selected);
static uint32 spi_setfrequency(FAR struct spi_dev_s *dev, uint32 frequency);
static ubyte  spi_status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
static uint16 spi_send(FAR struct spi_dev_s *dev, uint16 wd);
static void   spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t buflen);
static void   spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
  .select            = spi_select,
  .setfrequency      = spi_setfrequency,
  .status            = spi_status,
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
};

#ifdef CONFIG_STR71X_BSPI0
static struct str71x_spidev_s g_spidev0 =
{
  .spidev  = { &g_spiops },
  .spibase = STR71X_BSPI0_BASE,
  .csbit   = BSPI0_GPIO0_SS
};
#endif

#ifdef CONFIG_STR71X_BSPI1
static struct str71x_spidev_s g_spidev1 =
{
  .spidev  = { &g_spiops },
  .spibase = STR71X_BSPI1_BASE,
  .csbit   = BSPI1_GPIO0_SS
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ****************************************************************************/

static inline uint16 spi_getreg(FAR struct str71x_spidev_s *priv, ubyte offset)
{
  return getreg16(priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 16-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ****************************************************************************/

static inline void spi_putreg(FAR struct str71x_spidev_s *priv, ubyte offset, uint16 value)
{
  putreg16(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Enable/disable the SPI slave select.  The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   devid -    Identifies the device to select
 *   selected - TRUE: slave selected, FALSE: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, boolean selected)
{
  FAR struct str71x_spidev_s *priv = (FAR struct str71x_spidev_s *)dev;
  uint16 reg16;

  DEBUGASSERT(priv && priv->spibase);

  reg16 = spi_getreg(priv, STR71X_GPIO_PD_OFFSET);
  if (selected)
    {
     /* Enable slave select (low enables) */

      reg16 &= ~priv->csbit;
      spi_putreg(priv, STR71X_GPIO_PD_OFFSET, reg16);
    }
  else
    {
      /* Disable slave select (low enables) */

       reg16 |= priv->csbit;
       spi_putreg(priv, STR71X_GPIO_PD_OFFSET, reg16);

#if CONFIG_STR714X_BSPI0_TXFIFO_DEPTH > 1
       /* Wait while the TX FIFO is full */

       while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_TFF) != 0);
#else
       /* Wait until the TX FIFO is empty */

       while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_TFE) == 0);
#endif
       /* Write 0xff to the TX FIFO */

       spi_putreg(priv, STR71X_BSPI_TXR_OFFSET, 0xff00);

       /* Wait for the TX FIFO empty */

       while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_TFNE) != 0);

       /* Wait for the RX FIFO not empty */

       while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_RFNE) == 0);

       /* Then read and discard bytes until the RX FIFO is empty */

       do
         {
           (void)spi_getreg(priv, STR71X_BSPI_RXR_OFFSET);
         }
       while (spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET & STR71X_BSPICSR2_RFNE) != 0);
    }
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

static uint32 spi_setfrequency(FAR struct spi_dev_s *dev, uint32 frequency)
{
  FAR struct str71x_spidev_s *priv = (FAR struct str71x_spidev_s *)dev;
  uint32 divisor;
  uint32 cr1;

  DEBUGASSERT(priv && priv->spibase);

  /* The BSPI clock is determined by divider the APB1 clock (PCLK1).
   *
   * Eg. PCLK1 = 32MHz, frequency = 20000000:
   *     correct divisor is 2.1, calculated value is 2.
   */

  divisor = (STR71X_PCLK1 + (frequency >> 1)) / frequency;

  /* The divisor must be an even number and contrained to the range of
   * 5 (master mode, or 7 for slave mode) and 255.  These bits must
   * be configured BEFORE  the BSPE or MSTR bits.. i.e., before the SPI
   * is put into master mode.
   */

  divisor <<= 1;   /* The full, even divisor */
  if (divisor < 6)
    {
      divisor = 6;
    }
  else if (divisor > 254)
    {
      divisor = 254;
    }

  /* The BSPI must be disable when the following setting is made. */

  cr1 = spi_getreg(priv, STR71X_BSPI_CSR1_OFFSET);
  cr1 &= ~(STR71X_BSPICSR1_BSPE|STR71X_BSPICSR1_MSTR);
  spi_putreg(priv, STR71X_BSPI_CSR1_OFFSET, cr1);
  spi_putreg(priv, STR71X_BSPI_CLK_OFFSET, (uint16)divisor);

  /* Now we can enable the BSP in master mode */

  cr1 |= (STR71X_BSPICSR1_BSPE|STR71X_BSPICSR1_MSTR);
  spi_putreg(priv, STR71X_BSPI_CSR1_OFFSET, cr1);

  return STR71X_PCLK1 / divisor;
}

/****************************************************************************
 * Name: spi_status
 *
 * Description:
 *   Get SPI/MMC status
 *
 * Input Parameters:
 *   dev -   Device-specific state data
 *   devid - Identifies the device to report status on
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines
 *
 ****************************************************************************/

static ubyte spi_status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  ubyte ret = 0;
  uint16 reg16 = getreg16(STR71X_GPIO1_PD);

  if ((reg16 & MMCSD_GPIO1_WPIN) != 0)
    {
      ret |= SPI_STATUS_WRPROTECTED;
    }

  if ((reg16 & MMCSD_GPIO1_CPIN) != 0)
    {
      ret |= SPI_STATUS_PRESENT;
    }

  return ret;
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

static uint16 spi_send(FAR struct spi_dev_s *dev, uint16 wd)
{
  FAR struct str71x_spidev_s *priv = (FAR struct str71x_spidev_s *)dev;

  DEBUGASSERT(priv && priv->spibase);

#if CONFIG_STR714X_BSPI0_TXFIFO_DEPTH > 1
  /* Wait while the TX FIFO is full */

  while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_TFF) != 0);
#else
  /* Wait until the TX FIFO is empty */

  while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_TFE) == 0);
#endif

  /* Write the byte to the TX FIFO */

  spi_putreg(priv, STR71X_BSPI_TXR_OFFSET, wd << 8);

  /* Wait for the RX FIFO not empty */

  while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_RFNE) == 0);

  /* Get the received value from the RX FIFO and return it */

  return (ubyte)(spi_getreg(priv, STR71X_BSPI_RXR_OFFSET) >> 8);
}

/*************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   buflen - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into ubytes; if nbits >8, the data is packed into uint16's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t buflen)
{
  FAR struct str71x_spidev_s *priv = (FAR struct str71x_spidev_s *)dev;
  FAR const ubyte *ptr = (FAR const ubyte *)buffer;
  uint16 csr2;

  DEBUGASSERT(priv && priv->spibase);

  /* Loop while thre are bytes remaining to be sent */

  while (buflen > 0)
    {
      /* While the TX FIFO is not full and there are bytes left to send */

      while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_TFF) == 0 && buflen > 0)
        {
          /* Send the data */

          spi_putreg(priv, STR71X_BSPI_TXR_OFFSET, ((uint16)*ptr) << 8);
          ptr++;
          buflen--;
        }
    }

  /* Then discard all card responses until the RX & TX FIFOs are emptied. */

  do
    {
      /* Is there anything in the RX fifo? */

      csr2 = spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET);
      if ((csr2 & STR71X_BSPICSR2_RFNE) != 0)
        {
          /* Yes.. Read and discard */

          (void)spi_getreg(priv, STR71X_BSPI_RXR_OFFSET);
        }

      /* There is a race condition where TFNE may go FALSE just before
       * RFNE goes TRUE and this loop terminates prematurely.  The nasty little
       * delay in the following solves that (it could probably be tuned to
       * improve performance).
       */

      else if ((csr2 & STR71X_BSPICSR2_TFNE) != 0)
        {
          up_udelay(100);
          csr2 = spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET);
        }
    }
  while ((csr2 & STR71X_BSPICSR2_RFNE) != 0 || (csr2 & STR71X_BSPICSR2_TFNE) == 0);
}

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to recieve data
 *   buflen - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into ubytes; if nbits >8, the data is packed into uint16's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t buflen)
{
  FAR struct str71x_spidev_s *priv = (FAR struct str71x_spidev_s *)dev;
  FAR ubyte *ptr = (FAR ubyte*)buffer;
  uint32 fifobytes = 0;

  DEBUGASSERT(priv && priv->spibase);

  /* While there is remaining to be sent (and no synchronization error has occurred) */

  while (buflen || fifobytes)
    {
      /* Fill the transmit FIFO with 0xff...
       * Write 0xff to the data register while (1) the TX FIFO is
       * not full, (2) we have not exceeded the depth of the TX FIFO,
       * and (3) there are more bytes to be sent.
       */

      while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_TFF) == 0 &&
             (fifobytes < CONFIG_STR714X_BSPI0_TXFIFO_DEPTH) && buflen > 0)
        {
          spi_putreg(priv, STR71X_BSPI_TXR_OFFSET, 0xff00);
          buflen--;
          fifobytes++;
        }

      /* Now, read the RX data from the RX FIFO while the RX FIFO is not empty */

      while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_RFNE) != 0)
        {
          *ptr++ = (ubyte)(spi_getreg(priv, STR71X_BSPI_RXR_OFFSET) >> 8);
          fifobytes--;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structre reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *up_spiinitialize(int port)
{
  FAR struct spi_dev_s *ret;
  irqstate_t flags;
  uint16 reg16;

  flags = irqsave();
#ifdef CONFIG_STR71X_BSPI0
  if (port == 0)
    {
      /* The default, alternate functionality of the GPIO0 pin selections is
       * UART3/I2C1.  In order to have BSP0 functionality, we also have to
       * set the BSPI0 enable bit in the PCU BOOTCR register.
       */

      reg16 = getreg16(STR71X_PCU_BOOTCR);
      reg16 |= STR71X_PCUBOOTCR_BSPIOEN;
      putreg16(reg16, STR71X_PCU_BOOTCR);

      /* Configure all GPIO pins to their alternate function EXCEPT
       * for the SS pin .. will will configure that as an output
       * and control the chip select as a normal GPIO.
       */

      reg16  = getreg16(STR71X_GPIO0_PC0);
      reg16 |= BSPIO_GPIO0_ALL;
      putreg16(reg16, STR71X_GPIO0_PC0);

      reg16  = getreg16(STR71X_GPIO0_PC1);
      reg16 &= ~BSPIO_GPIO0_ALL;
      reg16 |= BSPIO_GPIO0_ALT;
      putreg16(reg16, STR71X_GPIO0_PC1);

      reg16  = getreg16(STR71X_GPIO0_PC2);
      reg16 |= BSPIO_GPIO0_ALL;
      putreg16(reg16, STR71X_GPIO0_PC2);

      /* Start with chip slave disabled */

      reg16  = getreg16(STR71X_GPIO0_PD);
      reg16 |= BSPI0_GPIO0_SS;
      putreg16(reg16, STR71X_GPIO0_PD);

      /* Set the clock divider to the maximum */

      putreg16(255, STR71X_BSPI0_CLK);

      /* Set FIFO sizes and disable the BSP1.  It won't be enabled
       * until the frequency is set.
       */

      putreg16(STR71X_BSPI0_CSR1DISABLE, STR71X_BSPI0_CSR1);
      putreg16(STR71X_BSPI0_CSR2VALUE, STR71X_BSPI0_CSR2);

      ret = &g_spidev0.spidev;
    }
  else
#endif
#ifdef CONFIG_STR71X_BSPI1
  if (port == 1)
    {
      /* Configure all GPIO pins to their alternate function EXCEPT
       * for the SS pin .. will will configure that as an output
       * and control the chip select as a normal GPIO.
       */

      reg16  = getreg16(STR71X_GPIO0_PC0);
      reg16 |= BSPI1_GPIO0_ALL;
      putreg16(reg16, STR71X_GPIO0_PC0);

      reg16  = getreg16(STR71X_GPIO0_PC1);
      reg16 &= ~BSPI1_GPIO0_ALL;
      reg16 |= BSPI1_GPIO0_ALT;
      putreg16(reg16, STR71X_GPIO0_PC1);

      reg16  = getreg16(STR71X_GPIO0_PC2);
      reg16 |= BSPI1_GPIO0_ALL;
      putreg16(reg16, STR71X_GPIO0_PC2);

      /* Start with chip slave disabled */

      reg16  = getreg16(STR71X_GPIO0_PD);
      reg16 |= BSPI1_GPIO0_SS;
      putreg16(reg16, STR71X_GPIO0_PD);

      /* Set the clock divider to the maximum */

      putreg16(255, STR71X_BSPI1_CLK);

      /* Set FIFO sizes and disable the BSP1.  It won't be enabled
       * until the frequency is set.
       */

      putreg16(STR71X_BSPI1_CSR1DISABLE, STR71X_BSPI1_CSR1);
      putreg16(STR71X_BSPI1_CSR2VALUE, STR71X_BSPI1_CSR2);

      /* Configure GPIO1 pins for WP/CP input */

      reg16  = getreg16(STR71X_GPIO1_PC0);
      reg16 |= MMCSD_GPIO1_ALL;
      putreg16(reg16, STR71X_GPIO1_PC0);

      reg16  = getreg16(STR71X_GPIO1_PC1);
      reg16 &= ~MMCSD_GPIO1_ALL;
      putreg16(reg16, STR71X_GPIO1_PC1);

      reg16  = getreg16(STR71X_GPIO1_PC2);
      reg16 &= ~MMCSD_GPIO1_ALL;
      putreg16(reg16, STR71X_GPIO1_PC2);

      ret = &g_spidev1.spidev;
    }
  else
#endif
    {
      ret = NULL;
    }
  irqrestore(flags);
  return ret;
}

#endif /* CONFIG_STR71X_BSPI0 || CONFIG_STR71X_BSPI1 */

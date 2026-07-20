/****************************************************************************
 * arch/arm/src/common/ameba/ameba_spi.c
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

/* NuttX SPI master lower half for the Realtek Ameba SPI controllers (SPI0
 * and SPI1).  Each controller is registered from board bring-up and appears
 * as /dev/spiN through the stock SPI character driver (spi_register()).
 *
 * The controller is a Synopsys DesignWare SSI block programmed through the
 * SDK fwlib SSI API in polling mode (no interrupts): SSI_Init() programs the
 * role/mode/clock, and each word is moved by spinning on SSI_Writeable() /
 * SSI_Readable() around SSI_WriteData() / SSI_ReadData().  The block runs in
 * transmit-and-receive mode so every word written yields one word read,
 * which is exactly the full-duplex exchange NuttX expects.
 *
 * The SSI fwlib routines resolve to the on-chip ROM symbol table (they are
 * _LONG_CALL_ entries), like the UART fwlib and unlike the I2C fwlib.  The
 * SSI baud divider is derived from the peripheral clock ip_clk =
 * PLL_ClkGet() / (HPERI divider + 1), read once at registration.
 *
 * The chip select is driven as a plain GPIO (software CS) rather than the
 * SSI hardware CS: select() toggles the CS pad through GPIO_WriteBit(), so
 * any pad can serve as CS and the CS timing is not tied to the FIFO state.
 *
 * The chip-specific wiring (controller count, register bases, clock masks
 * and pad-mux codes) lives in the per-chip ameba_spi_chip.h.  To keep the
 * vendor headers out of the NuttX include world, the few fwlib symbols and
 * the SSI_InitTypeDef layout used here are declared locally rather than
 * pulled in from <ameba_spi.h>.
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <debug.h>
#include <inttypes.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include "ameba_spi.h"
#include "ameba_spi_chip.h"
#include "ameba_gpio_chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The register bases, peripheral-clock masks, crossbar pad-mux codes and
 * controller count (AMEBA_NSPI) come from the per-chip ameba_spi_chip.h.
 * Everything below is common to every current Ameba chip.
 */

/* fwlib SSI_InitTypeDef field values (SSI_MASTER, TMOD/FRF, SCPOL/SCPH).
 * These are identical on every Ameba chip audited.
 */

#define AMEBA_SSI_MASTER        0x1   /* SSI_MASTER (SPI_Role)              */
#define AMEBA_SSI_TMOD_TR       0x0   /* Transmit & receive (SPI_TransferMode) */
#define AMEBA_SSI_FRF_SPI       0x0   /* FRF_MOTOROLA_SPI (SPI_DataFrameFormat) */

#define AMEBA_SSI_SCPOL_LOW     0x0   /* SCPOL_INACTIVE_IS_LOW  (CPOL=0)    */
#define AMEBA_SSI_SCPOL_HIGH    0x1   /* SCPOL_INACTIVE_IS_HIGH (CPOL=1)    */
#define AMEBA_SSI_SCPH_MIDDLE   0x0   /* SCPH_TOGGLES_IN_MIDDLE (CPHA=0)    */
#define AMEBA_SSI_SCPH_START    0x1   /* SCPH_TOGGLES_AT_START  (CPHA=1)    */

/* GPIO field values for the software chip select (see ameba_gpio.c). */

#define AMEBA_GPIO_MODE_OUT     0x1   /* GPIO_Mode_OUT                      */
#define AMEBA_GPIO_PUPD_NONE    0x0   /* GPIO_PuPd_NOPULL                   */

/* Second/third argument to fwlib "state" style APIs. */

#define AMEBA_DISABLE           0x0
#define AMEBA_ENABLE            0x1

/* Bus defaults used until the upper half programs its own. */

#define AMEBA_SPI_DEFAULT_FREQ  1000000
#define AMEBA_SPI_DEFAULT_BITS  8

/* Bounded spin guarding each FIFO wait so a wedged bus cannot hang. */

#define AMEBA_SPI_TIMEOUT       1000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Layout-compatible mirror of the fwlib SSI_InitTypeDef (all u32, same
 * order); passed by address to SSI_StructInit()/SSI_Init().
 */

struct ameba_ssi_init_s
{
  uint32_t dmarxlvl;           /* SPI_DmaRxDataLevel   */
  uint32_t dmatxlvl;           /* SPI_DmaTxDataLevel   */
  uint32_t rxthlvl;            /* SPI_RxThresholdLevel */
  uint32_t txthlvl;            /* SPI_TxThresholdLevel */
  uint32_t ssenable;           /* SPI_SlaveSelectEnable */
  uint32_t divider;            /* SPI_ClockDivider     */
  uint32_t framenum;           /* SPI_DataFrameNumber  */
  uint32_t frameformat;        /* SPI_DataFrameFormat  */
  uint32_t framesize;          /* SPI_DataFrameSize    */
  uint32_t intmask;            /* SPI_InterruptMask    */
  uint32_t role;               /* SPI_Role             */
  uint32_t sclkphase;          /* SPI_SclkPhase        */
  uint32_t sclkpolarity;       /* SPI_SclkPolarity     */
  uint32_t transfermode;       /* SPI_TransferMode     */
};

/* Layout-compatible mirror of the fwlib GPIO_InitTypeDef (see ameba_gpio.c),
 * used to configure the software chip-select pad as an output.
 */

struct ameba_gpio_init_s
{
  uint32_t mode;               /* GPIO_Mode        */
  uint32_t pupd;               /* GPIO_PuPd        */
  uint32_t ittrigger;          /* GPIO_ITTrigger   */
  uint32_t itpolarity;         /* GPIO_ITPolarity  */
  uint32_t itdebounce;         /* GPIO_ITDebounce  */
  uint32_t pin;                /* GPIO_Pin         */
};

struct ameba_spi_dev_s
{
  struct spi_dev_s dev;        /* SPI master lower half (must be first) */
  uintptr_t base;              /* Non-secure SPI register base address */
  uint32_t  periph;            /* APBPeriph function mask (RCC arg 1) */
  uint32_t  clk;               /* APBPeriph clock mask (RCC arg 2) */
  uint32_t  ipclk;             /* SSI peripheral clock ip_clk (Hz) */
  uint32_t  frequency;         /* Requested bus frequency (Hz) */
  uint32_t  actual;            /* Achieved bus frequency (Hz) */
  uint32_t  divider;           /* SSI clock divider for that frequency */
  uint8_t   mode;              /* Currently programmed enum spi_mode_e */
  uint8_t   nbits;             /* Currently programmed bits per word */
  bool      dirty;             /* SSI needs an SSI_Init() before use */
  uint8_t   clkpin;            /* SCLK pad (AMEBA_PA()/AMEBA_PB()) */
  uint8_t   mosipin;           /* MOSI pad */
  uint8_t   misopin;           /* MISO pad */
  uint8_t   cspin;             /* Chip-select pad (GPIO output) */
  uint8_t   clkfid;            /* Pin mux code for the SCLK pad */
  uint8_t   mosifid;           /* Pin mux code for the MOSI pad */
  uint8_t   misofid;           /* Pin mux code for the MISO pad */
  mutex_t   lock;              /* Serializes bus access */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SDK fwlib SSI/GPIO/pin/clock API, all resolved from the on-chip ROM. */

extern void RCC_PeriphClockCmd(uint32_t periph, uint32_t clock,
                               uint8_t newstate);
extern void Pinmux_Config(uint8_t pin, uint32_t func);
extern void GPIO_Init(struct ameba_gpio_init_s *init);
extern void GPIO_WriteBit(uint32_t pin, uint32_t state);
extern void SSI_SetRole(void *spidev, uint32_t role);
extern void SSI_StructInit(struct ameba_ssi_init_s *init);
extern void SSI_Init(void *spidev, struct ameba_ssi_init_s *init);
extern void SSI_Cmd(void *spidev, uint32_t newstate);
extern uint32_t SSI_Writeable(void *spidev);
extern uint32_t SSI_Readable(void *spidev);
extern void SSI_WriteData(void *spidev, uint32_t value);
extern uint32_t SSI_ReadData(void *spidev);

/* SPI master lower-half operations. */

static int ameba_spi_lock(struct spi_dev_s *dev, bool lock);
static void ameba_spi_select(struct spi_dev_s *dev, uint32_t devid,
                             bool selected);
static uint32_t ameba_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency);
static void ameba_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void ameba_spi_setbits(struct spi_dev_s *dev, int nbits);
static uint8_t ameba_spi_status(struct spi_dev_s *dev, uint32_t devid);
static uint32_t ameba_spi_send(struct spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SPI_EXCHANGE
static void ameba_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer, void *rxbuffer,
                               size_t nwords);
#else
static void ameba_spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                               size_t nwords);
static void ameba_spi_recvblock(struct spi_dev_s *dev, void *buffer,
                                size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_ameba_spi_ops =
{
  .lock         = ameba_spi_lock,
  .select       = ameba_spi_select,
  .setfrequency = ameba_spi_setfrequency,
  .setmode      = ameba_spi_setmode,
  .setbits      = ameba_spi_setbits,
  .status       = ameba_spi_status,
  .send         = ameba_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = ameba_spi_exchange,
#else
  .sndblock     = ameba_spi_sndblock,
  .recvblock    = ameba_spi_recvblock,
#endif
};

/* Per-controller register base, peripheral function/clock masks and crossbar
 * pad-mux codes, indexed by controller number and supplied by the per-chip
 * ameba_spi_chip.h.
 */

static const uintptr_t g_spi_base[AMEBA_NSPI]   = AMEBA_SPI_BASES;
static const uint32_t  g_spi_periph[AMEBA_NSPI] = AMEBA_SPI_APBPERIPH;
static const uint32_t  g_spi_clk[AMEBA_NSPI]    = AMEBA_SPI_APBPERIPH_CLK;
static const uint8_t   g_spi_clkfid[AMEBA_NSPI]  = AMEBA_SPI_CLKFID;
static const uint8_t   g_spi_mosifid[AMEBA_NSPI] = AMEBA_SPI_MOSIFID;
static const uint8_t   g_spi_misofid[AMEBA_NSPI] = AMEBA_SPI_MISOFID;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_spi_ipclk
 *
 * Description:
 *   Return the SSI peripheral clock ip_clk = PLL_ClkGet() / (HPERI + 1),
 *   the frequency the baud divider divides down to produce SCLK.
 *
 ****************************************************************************/

static uint32_t ameba_spi_ipclk(void)
{
  /* The register address and HPERI field position are chip-specific and are
   * expressed by AMEBA_SPI_IPCLK() in the per-chip ameba_spi_chip.h.
   */

  return AMEBA_SPI_IPCLK();
}

/****************************************************************************
 * Name: ameba_spi_configure
 *
 * Description:
 *   (Re)program the SSI block for the currently requested frequency, mode
 *   and word length, and enable it.  The DesignWare SSI can only be
 *   configured while disabled, so this runs lazily on the first word after
 *   any of those parameters changes (priv->dirty).
 *
 ****************************************************************************/

static void ameba_spi_configure(struct ameba_spi_dev_s *priv)
{
  struct ameba_ssi_init_s init;
  void *spidev = (void *)priv->base;

  if (!priv->dirty)
    {
      return;
    }

  memset(&init, 0, sizeof(init));
  SSI_StructInit(&init);

  init.role         = AMEBA_SSI_MASTER;
  init.transfermode = AMEBA_SSI_TMOD_TR;
  init.frameformat  = AMEBA_SSI_FRF_SPI;
  init.ssenable     = 0;      /* Software CS: do not drive the SSI CS line */
  init.intmask      = 0;      /* Polling: no SSI interrupts */
  init.divider      = priv->divider;
  init.framesize    = priv->nbits - 1;

  /* enum spi_mode_e encodes CPOL in bit1 and CPHA in bit0. */

  init.sclkpolarity = (priv->mode & 0x2) ? AMEBA_SSI_SCPOL_HIGH :
                                           AMEBA_SSI_SCPOL_LOW;
  init.sclkphase    = (priv->mode & 0x1) ? AMEBA_SSI_SCPH_START :
                                           AMEBA_SSI_SCPH_MIDDLE;

  SSI_Cmd(spidev, AMEBA_DISABLE);
  SSI_Init(spidev, &init);
  SSI_Cmd(spidev, AMEBA_ENABLE);

  priv->dirty = false;
}

/****************************************************************************
 * Name: ameba_spi_transfer
 *
 * Description:
 *   Move nwords words full-duplex in polling mode.  Every word written to
 *   the TX FIFO produces one word in the RX FIFO (transmit-and-receive
 *   mode); a NULL txbuffer sends 0xffff and a NULL rxbuffer discards the
 *   received words.
 *
 ****************************************************************************/

static void ameba_spi_transfer(struct ameba_spi_dev_s *priv,
                               const void *txbuffer, void *rxbuffer,
                               size_t nwords)
{
  void *spidev = (void *)priv->base;
  bool wide = priv->nbits > 8;
  size_t i;
  uint32_t to;

  ameba_spi_configure(priv);

  for (i = 0; i < nwords; i++)
    {
      uint32_t word = 0xffff;
      uint32_t rxword;

      if (txbuffer != NULL)
        {
          word = wide ? ((const uint16_t *)txbuffer)[i] :
                        ((const uint8_t *)txbuffer)[i];
        }

      for (to = AMEBA_SPI_TIMEOUT;
           to > 0 && SSI_Writeable(spidev) == 0; to--);

      SSI_WriteData(spidev, word);

      for (to = AMEBA_SPI_TIMEOUT;
           to > 0 && SSI_Readable(spidev) == 0; to--);

      rxword = SSI_ReadData(spidev);

      if (rxbuffer != NULL)
        {
          if (wide)
            {
              ((uint16_t *)rxbuffer)[i] = (uint16_t)rxword;
            }
          else
            {
              ((uint8_t *)rxbuffer)[i] = (uint8_t)rxword;
            }
        }
    }
}

/****************************************************************************
 * Name: ameba_spi_lock
 ****************************************************************************/

static int ameba_spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct ameba_spi_dev_s *priv = (struct ameba_spi_dev_s *)dev;

  return lock ? nxmutex_lock(&priv->lock) : nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: ameba_spi_select
 *
 * Description:
 *   Assert (selected) or de-assert the active-low chip-select GPIO.  A
 *   single CS pad is driven per bus, so the devid is not used.
 *
 ****************************************************************************/

static void ameba_spi_select(struct spi_dev_s *dev, uint32_t devid,
                             bool selected)
{
  struct ameba_spi_dev_s *priv = (struct ameba_spi_dev_s *)dev;

  GPIO_WriteBit(priv->cspin, selected ? AMEBA_DISABLE : AMEBA_ENABLE);
}

/****************************************************************************
 * Name: ameba_spi_setfrequency
 *
 * Description:
 *   Choose the largest even SSI divider whose SCLK does not exceed the
 *   requested frequency (SCLK = ip_clk / divider, even, 2..65534), and
 *   return the frequency actually achieved.
 *
 ****************************************************************************/

static uint32_t ameba_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  struct ameba_spi_dev_s *priv = (struct ameba_spi_dev_s *)dev;
  uint32_t divider;

  if (frequency == 0)
    {
      return priv->actual;
    }

  if (frequency == priv->frequency)
    {
      return priv->actual;
    }

  /* Round the divider down to an even value, then bump it up until SCLK no
   * longer exceeds the request; clamp to the 16-bit even range.
   */

  divider = (priv->ipclk / frequency / 2) * 2;
  if (divider < 2)
    {
      divider = 2;
    }

  if ((priv->ipclk / divider) > frequency)
    {
      divider += 2;
    }

  if (divider > 0xfffe)
    {
      divider = 0xfffe;
    }

  priv->divider   = divider;
  priv->frequency = frequency;
  priv->actual    = priv->ipclk / divider;
  priv->dirty     = true;

  spiinfo("frequency=%" PRIu32 " actual=%" PRIu32 " div=%" PRIu32 "\n",
          frequency, priv->actual, divider);
  return priv->actual;
}

/****************************************************************************
 * Name: ameba_spi_setmode
 ****************************************************************************/

static void ameba_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct ameba_spi_dev_s *priv = (struct ameba_spi_dev_s *)dev;

  if ((uint8_t)mode != priv->mode)
    {
      priv->mode  = (uint8_t)mode;
      priv->dirty = true;
    }
}

/****************************************************************************
 * Name: ameba_spi_setbits
 ****************************************************************************/

static void ameba_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct ameba_spi_dev_s *priv = (struct ameba_spi_dev_s *)dev;

  if (nbits >= 4 && nbits <= 16 && (uint8_t)nbits != priv->nbits)
    {
      priv->nbits = (uint8_t)nbits;
      priv->dirty = true;
    }
}

/****************************************************************************
 * Name: ameba_spi_status
 ****************************************************************************/

static uint8_t ameba_spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  UNUSED(dev);
  UNUSED(devid);
  return 0;
}

/****************************************************************************
 * Name: ameba_spi_send
 ****************************************************************************/

static uint32_t ameba_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct ameba_spi_dev_s *priv = (struct ameba_spi_dev_s *)dev;
  uint16_t rxword = 0;
  uint16_t txword = (uint16_t)wd;

  ameba_spi_transfer(priv, &txword, &rxword, 1);
  return rxword;
}

#ifdef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: ameba_spi_exchange
 ****************************************************************************/

static void ameba_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer, void *rxbuffer,
                               size_t nwords)
{
  struct ameba_spi_dev_s *priv = (struct ameba_spi_dev_s *)dev;

  ameba_spi_transfer(priv, txbuffer, rxbuffer, nwords);
}

#else

/****************************************************************************
 * Name: ameba_spi_sndblock
 ****************************************************************************/

static void ameba_spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                               size_t nwords)
{
  struct ameba_spi_dev_s *priv = (struct ameba_spi_dev_s *)dev;

  ameba_spi_transfer(priv, buffer, NULL, nwords);
}

/****************************************************************************
 * Name: ameba_spi_recvblock
 ****************************************************************************/

static void ameba_spi_recvblock(struct spi_dev_s *dev, void *buffer,
                                size_t nwords)
{
  struct ameba_spi_dev_s *priv = (struct ameba_spi_dev_s *)dev;

  ameba_spi_transfer(priv, NULL, buffer, nwords);
}

#endif /* CONFIG_SPI_EXCHANGE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_spi_register
 *
 * Description:
 *   See ameba_spi.h.
 *
 ****************************************************************************/

struct spi_dev_s *ameba_spi_register(int bus, uint8_t clkpin,
                                     uint8_t mosipin, uint8_t misopin,
                                     uint8_t cspin)
{
  struct ameba_spi_dev_s *priv;
  struct ameba_gpio_init_s gpio;
  int ret;

  if (bus < 0 || bus >= AMEBA_NSPI)
    {
      return NULL;
    }

  priv = kmm_zalloc(sizeof(struct ameba_spi_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->dev.ops   = &g_ameba_spi_ops;
  priv->base      = g_spi_base[bus];
  priv->periph    = g_spi_periph[bus];
  priv->clk       = g_spi_clk[bus];
  priv->clkpin    = clkpin;
  priv->mosipin   = mosipin;
  priv->misopin   = misopin;
  priv->cspin     = cspin;
  priv->clkfid    = g_spi_clkfid[bus];
  priv->mosifid   = g_spi_mosifid[bus];
  priv->misofid   = g_spi_misofid[bus];
  priv->mode      = SPIDEV_MODE0;
  priv->nbits     = AMEBA_SPI_DEFAULT_BITS;
  priv->frequency = 0;
  priv->dirty     = true;
  nxmutex_init(&priv->lock);

  /* Gate the SSI peripheral clock and route the CLK/MOSI/MISO pads. */

  RCC_PeriphClockCmd(priv->periph, priv->clk, AMEBA_ENABLE);

  /* The controller powers up as an SPI slave (the LSYS SPIx_MST bit is clear
   * out of reset), and in slave mode the master-only baud-rate and slave-
   * enable registers are write-ignored so the block never drives SCLK.
   * Switch it to master before any SSI_Init() so that configuration sticks.
   */

  SSI_SetRole((void *)priv->base, AMEBA_SSI_MASTER);

  Pinmux_Config(priv->clkpin, priv->clkfid);
  Pinmux_Config(priv->mosipin, priv->mosifid);
  Pinmux_Config(priv->misopin, priv->misofid);

  /* Configure the chip-select pad as a GPIO output, de-asserted (high). */

  RCC_PeriphClockCmd(AMEBA_APBPERIPH_GPIO, AMEBA_APBPERIPH_GPIO,
                     AMEBA_ENABLE);
  memset(&gpio, 0, sizeof(gpio));
  gpio.mode = AMEBA_GPIO_MODE_OUT;
  gpio.pupd = AMEBA_GPIO_PUPD_NONE;
  gpio.pin  = priv->cspin;
  GPIO_Init(&gpio);
  GPIO_WriteBit(priv->cspin, AMEBA_ENABLE);

  /* Latch the peripheral clock and prime the default bus frequency. */

  priv->ipclk = ameba_spi_ipclk();
  ameba_spi_setfrequency(&priv->dev, AMEBA_SPI_DEFAULT_FREQ);

  ret = spi_register(&priv->dev, bus);
  if (ret < 0)
    {
      _err("ERROR: spi_register(/dev/spi%d) failed: %d\n", bus, ret);
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return NULL;
    }

  return &priv->dev;
}

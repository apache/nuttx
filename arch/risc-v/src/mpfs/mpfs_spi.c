/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_spi.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "mpfs_spi.h"
#include "hardware/mpfs_spi.h"
#include "hardware/mpfs_sysreg.h"
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_SPI_FREQ_DEFAULT         4000000

#define MPFS_SYSREG_SOFT_RESET_CR     (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SOFT_RESET_CR_OFFSET)
#define MPFS_SYSREG_SUBBLK_CLOCK_CR   (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET)

#define MPFS_SPI_CONTROL_OFFSET       0x00
#define MPFS_SPI_FRAMESIZE_OFFSET     0x04
#define MPFS_SPI_STATUS_OFFSET        0x08
#define MPFS_SPI_INT_CLEAR_OFFSET     0x0C
#define MPFS_SPI_RX_DATA_OFFSET       0x10
#define MPFS_SPI_TX_DATA_OFFSET       0x14
#define MPFS_SPI_CLK_GEN_OFFSET       0x18
#define MPFS_SPI_SLAVE_SELECT_OFFSET  0x1C
#define MPFS_SPI_INTMASK_OFFSET       0x20
#define MPFS_SPI_INTRAW_OFFSET        0x24
#define MPFS_SPI_CONTROL2_OFFSET      0x28
#define MPFS_SPI_COMMAND_OFFSET       0x2C
#define MPFS_SPI_PKTSIZE_OFFSET       0x30
#define MPFS_SPI_CMD_SIZE_OFFSET      0x34
#define MPFS_SPI_HWSTATUS_OFFSET      0x38
#define MPFS_SPI_STATS_OFFSET         0x3C
#define MPFS_SPI_CTRL0_OFFSET         0x40
#define MPFS_SPI_CTRL1_OFFSET         0x44
#define MPFS_SPI_CTRL2_OFFSET         0x48
#define MPFS_SPI_CTRL3_OFFSET         0x4C
#define MPFS_SPI_FRAMESUP_OFFSET      0x50

#define MPFS_SPI_CONTROL      (priv->hw_base + MPFS_SPI_CONTROL_OFFSET)
#define MPFS_SPI_FSIZE        (priv->hw_base + MPFS_SPI_FRAMESIZE_OFFSET)
#define MPFS_SPI_STATUS       (priv->hw_base + MPFS_SPI_STATUS_OFFSET)
#define MPFS_SPI_RX_DATA      (priv->hw_base + MPFS_SPI_RX_DATA_OFFSET)
#define MPFS_SPI_TX_DATA      (priv->hw_base + MPFS_SPI_TX_DATA_OFFSET)
#define MPFS_SPI_INT_CLEAR    (priv->hw_base + MPFS_SPI_INT_CLEAR_OFFSET)
#define MPFS_SPI_CLK_GEN      (priv->hw_base + MPFS_SPI_CLK_GEN_OFFSET)
#define MPFS_SPI_SSELECT      (priv->hw_base + MPFS_SPI_SLAVE_SELECT_OFFSET)
#define MPFS_SPI_INTMASK      (priv->hw_base + MPFS_SPI_INTMASK_OFFSET)
#define MPFS_SPI_CONTROL2     (priv->hw_base + MPFS_SPI_CONTROL2_OFFSET)
#define MPFS_SPI_COMMAND      (priv->hw_base + MPFS_SPI_COMMAND_OFFSET)
#define MPFS_SPI_PKTSIZE      (priv->hw_base + MPFS_SPI_PKTSIZE_OFFSET)
#define MPFS_SPI_CMD_SIZE     (priv->hw_base + MPFS_SPI_CMD_SIZE_OFFSET)
#define MPFS_SPI_FRAMESUP     (priv->hw_base + MPFS_SPI_FRAMESUP_OFFSET)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPI Device hardware configuration */

struct mpfs_spi_config_s
{
  uint32_t        clk_freq;  /* SPI clock frequency */
  enum spi_mode_e mode;      /* SPI default mode */
  bool            use_irq;   /* Use DMA */
};

struct mpfs_spi_priv_s
{
  struct spi_dev_s               spi_dev;
  const struct mpfs_spi_config_s *config;   /* Port configuration */

  uintptr_t                      hw_base;   /* Bus base address */
  uint16_t                       plic_irq;  /* Platform IRQ */

  int                            refs;      /* Reference count */
  int                            enabled;   /* Enable flag */
  const int                      id;        /* SPI0 or SPI1 id */
  uint32_t                       devid;     /* SPI CS device 0..7 */

  mutex_t                        lock;      /* Bus usage mutex */
  sem_t                          sem_isr;   /* Interrupt wait semaphore */

  uint32_t                       frequency; /* Requested clock frequency */
  uint32_t                       actual;    /* Actual clock frequency */
  uint32_t                       txwords;   /* Words for TX */
  const void                     *txbuf;    /* TX buffer */
  uint32_t                       tx_pos;    /* TX position */

  uint32_t                       rxwords;   /* Words for RX */
  void                           *rxbuf;    /* RX buffer */
  uint32_t                       rx_pos;    /* RX position */

  uint32_t                       fifosize;  /* Fifo size */
  uint32_t                       fifolevel; /* Fifo IRQ level */
  int                            error;     /* Detected error */

  enum spi_mode_e                mode;      /* Actual SPI hardware mode */

  uint8_t                        nbits;     /* Bits per transaction */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mpfs_spi_lock(struct spi_dev_s *dev, bool lock);

#ifdef CONFIG_SPI_CS_CONTROL
static void mpfs_spi_select(struct spi_dev_s *dev, uint32_t devid,
                            bool selected);
#endif

static uint32_t mpfs_spi_setfrequency(struct spi_dev_s *dev,
                                      uint32_t frequency);
static void mpfs_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void mpfs_spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int mpfs_spi_hwfeatures(struct spi_dev_s *dev,
                               spi_hwfeatures_t features);
#endif
static uint8_t mpfs_spi_status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
static int mpfs_spi_cmddata(struct spi_dev_s *dev,
                            uint32_t devid, bool cmd);
#endif
static uint32_t mpfs_spi_send(struct spi_dev_s *dev, uint32_t wd);
static void mpfs_spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                              void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void mpfs_spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
                              size_t nwords);
static void mpfs_spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                               size_t nwords);
#endif
#ifdef CONFIG_SPI_TRIGGER
static int mpfs_spi_trigger(struct spi_dev_s *dev);
#endif
static void mpfs_spi_enable(struct mpfs_spi_priv_s *priv, uint8_t enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct mpfs_spi_config_s mpfs_spi_config =
{
    .clk_freq = MPFS_SPI_FREQ_DEFAULT,
    .mode     = SPIDEV_MODE0,
    .use_irq  = true,
};

#ifdef CONFIG_MPFS_SPI0
static const struct spi_ops_s mpfs_spi0_ops =
{
    .lock             = mpfs_spi_lock,
    .select           = mpfs_spi0_select,
    .setfrequency     = mpfs_spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
    .setdelay         = mpfs_spi_setdelay,
#endif
    .setmode          = mpfs_spi_setmode,
    .setbits          = mpfs_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
    .hwfeatures       = mpfs_spi_hwfeatures,
#endif
    .status           = mpfs_spi_status,
#ifdef CONFIG_SPI_CMDDATA
    .cmddata          = mpfs_spi_cmddata,
#endif
    .send             = mpfs_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
    .exchange         = mpfs_spi_exchange,
#else
    .sndblock         = mpfs_spi_sndblock,
    .recvblock        = mpfs_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
    .trigger          = mpfs_spi_trigger,
#endif
    .registercallback = NULL,
};

static struct mpfs_spi_priv_s g_mpfs_spi0_priv =
{
  .spi_dev =
  {
    .ops             = &mpfs_spi0_ops
  },
  .config            = &mpfs_spi_config,
  .hw_base           = MPFS_SPI0_LO_BASE,
  .plic_irq          = MPFS_IRQ_SPI0,
  .id                = 0,
  .devid             = 0,
  .lock              = NXMUTEX_INITIALIZER,
  .sem_isr           = SEM_INITIALIZER(0),
};
#endif /* CONFIG_MPFS_SPI0 */

#ifdef CONFIG_MPFS_SPI1
static const struct spi_ops_s mpfs_spi1_ops =
{
    .lock             = mpfs_spi_lock,
    .select           = mpfs_spi1_select,
    .setfrequency     = mpfs_spi_setfrequency,
#ifdef CONFIG_SPI_CS_DELAY_CONTROL
    .setdelay         = mpfs_spi_setdelay,
#endif
    .setmode          = mpfs_spi_setmode,
    .setbits          = mpfs_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
    .hwfeatures       = mpfs_spi_hwfeatures,
#endif
    .status           = mpfs_spi_status,
#ifdef CONFIG_SPI_CMDDATA
    .cmddata          = mpfs_spi_cmddata,
#endif
    .send             = mpfs_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
    .exchange         = mpfs_spi_exchange,
#else
    .sndblock         = mpfs_spi_sndblock,
    .recvblock        = mpfs_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
    .trigger          = mpfs_spi_trigger,
#endif
    .registercallback = NULL,
};
static struct mpfs_spi_priv_s g_mpfs_spi1_priv =
{
  .spi_dev =
  {
    .ops             = &mpfs_spi1_ops
  },
  .config            = &mpfs_spi_config,
  .hw_base           = MPFS_SPI1_LO_BASE,
  .plic_irq          = MPFS_IRQ_SPI1,
  .id                = 1,
  .devid             = 1,
  .lock              = NXMUTEX_INITIALIZER,
  .sem_isr           = SEM_INITIALIZER(0),
};

#endif /* CONFIG_MPFS_SPI1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_spi_rxoverflow_recover
 *
 * Description:
 *   Recover from RX overflow condition. This resets the whole SPI device.
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_spi_rxoverflow_recover(struct mpfs_spi_priv_s *priv)
{
  uint32_t control_reg;
  uint32_t clk_gen;
  uint32_t frame_size;
  uint32_t control2;
  uint32_t packet_size;
  uint32_t cmd_size;
  uint32_t slave_select;

  /* Read current SPI hardware block configuration */

  control_reg  = getreg32(MPFS_SPI_CONTROL);
  clk_gen      = getreg32(MPFS_SPI_CLK_GEN);
  frame_size   = getreg32(MPFS_SPI_FSIZE);
  control2     = getreg32(MPFS_SPI_CONTROL2);
  packet_size  = getreg32(MPFS_SPI_PKTSIZE);
  cmd_size     = getreg32(MPFS_SPI_CMD_SIZE);
  slave_select = getreg32(MPFS_SPI_SSELECT);

  /* Reset the SPI hardware block */

  modifyreg32(MPFS_SPI_CONTROL, 0, MPFS_SPI_RESET);
  modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_RESET, 0);

  mpfs_spi_enable(priv, 0);

  /* Restore SPI hardware block configuration */

  putreg32(control_reg, MPFS_SPI_CONTROL);
  putreg32(clk_gen, MPFS_SPI_CLK_GEN);
  putreg32(frame_size, MPFS_SPI_FSIZE);

  mpfs_spi_enable(priv, 1);

  putreg32(control2, MPFS_SPI_CONTROL2);
  putreg32(packet_size, MPFS_SPI_PKTSIZE);
  putreg32(cmd_size, MPFS_SPI_CMD_SIZE);
  putreg32(slave_select, MPFS_SPI_SSELECT);
}

/****************************************************************************
 * Name: mpfs_spi_lock
 *
 * Description:
 *   Lock or unlock the SPI device
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   lock   - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   The result of lock or unlock the SPI device
 *
 ****************************************************************************/

static int mpfs_spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct mpfs_spi_priv_s *priv = (struct mpfs_spi_priv_s *)dev;
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
 * Name: mpfs_spi_select
 *
 * Description:
 *   Enable/disable the SPI chip select.
 *
 * Input Parameters:
 *   priv     - Private SPI device structure
 *   devid    - Identifies the device to select
 *   selected - true: slave selected, false: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CS_CONTROL
static void mpfs_spi_select(struct spi_dev_s *dev, uint32_t devid,
                            bool selected)
{
  struct mpfs_spi_priv_s *priv = (struct mpfs_spi_priv_s *)dev;

  priv->devid = devid & 0x1f;

  if (selected)
    {
      modifyreg32(MPFS_SPI_SSELECT, 0, 1 << (priv->devid));
    }
  else
    {
      modifyreg32(MPFS_SPI_SSELECT, 1 << (priv->devid), 0);
    }

  spiinfo("devid: %u, CS: %s\n", devid, selected ? "select" : "free");
}
#endif

/****************************************************************************
 * Name:  mpfs_spi0/1_select
 *
 * Description:
 *   The external function, mpfs_spi0/1_select.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   devid - The SPI CS or device number
 *   selected - true: assert CS, false de-assert CS
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MPFS_SPI0
void weak_function mpfs_spi0_select(struct spi_dev_s *dev,
                                    uint32_t devid, bool selected)
{
#ifdef CONFIG_SPI_CS_CONTROL
  mpfs_spi_select(dev, devid, selected);
#else
# warning "Missing logic"
#endif
}
#endif /* CONFIG_MPFS_SPI0 */

#ifdef CONFIG_MPFS_SPI1
void weak_function mpfs_spi1_select(struct spi_dev_s *dev,
                                    uint32_t devid, bool selected)
{
#ifdef CONFIG_SPI_CS_CONTROL
  mpfs_spi_select(dev, devid, selected);
#else
# warning "Missing logic"
#endif
}
#endif /* CONFIG_MPFS_SPI1 */

/****************************************************************************
 * Name: mpfs_spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the current actual frequency selected
 *
 ****************************************************************************/

static uint32_t mpfs_spi_setfrequency(struct spi_dev_s *dev,
                                      uint32_t frequency)
{
  struct mpfs_spi_priv_s *priv = (struct mpfs_spi_priv_s *)dev;
  uint32_t divider;

  DEBUGASSERT(frequency > 0);

  if (priv->enabled)
    {
      modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_ENABLE, 0);
    }

  priv->frequency = frequency;
  divider = (MPFS_MSS_APB_AHB_CLK / frequency) >> 1;
  priv->actual = (uint32_t)frequency * divider;

  DEBUGASSERT(divider >= 2u && divider <= 512u);

  putreg32(divider, MPFS_SPI_CLK_GEN);

  spiinfo("frequency=%u, actual=%u\n", priv->frequency, priv->actual);

  if (priv->enabled)
    {
      modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_ENABLE, MPFS_SPI_ENABLE);
    }

  return priv->actual;
}

/****************************************************************************
 * Name: mpfs_spi_setmode
 *
 * Description:
 *   Set the SPI mode.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void mpfs_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct mpfs_spi_priv_s *priv = (struct mpfs_spi_priv_s *)dev;

  spiinfo("mode=%d\n", mode);

  switch (mode)
    {
      case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
        modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_SPO | MPFS_SPI_SPH, 0);
        break;

      case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
        modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_SPO, MPFS_SPI_SPH);
        break;

      case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
        modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_SPH, MPFS_SPI_SPO);
        break;

      case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
        modifyreg32(MPFS_SPI_CONTROL, 0, MPFS_SPI_SPO | MPFS_SPI_SPH);
        break;

      default:
        return;
    }

    priv->mode = mode;

    /* SPS bit ensures the slave select remains asserted between frames,
     * applicable to SPIDEV_MODEx:s.
     */

    modifyreg32(MPFS_SPI_CONTROL, 0, MPFS_SPI_SPS);
}

/****************************************************************************
 * Name: mpfs_spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits in an SPI word.
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void mpfs_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct mpfs_spi_priv_s *priv = (struct mpfs_spi_priv_s *)dev;

  spiinfo("nbits=%d\n", nbits);

  /* SPI must be disabled for FS change to apply */

  if (priv->enabled)
    {
      modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_ENABLE, 0);
    }

  modifyreg32(MPFS_SPI_FSIZE, MPFS_SPI_FRAMESIZE, nbits);

  if (priv->enabled)
    {
      modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_ENABLE, MPFS_SPI_ENABLE);
    }

  priv->nbits = nbits;
}

/****************************************************************************
 * Name: mpfs_spi_status
 *
 * Description:
 *   Get SPI/MMC status.  Optional.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   devid - Identifies the device to report status on
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines)
 *
 ****************************************************************************/

static uint8_t mpfs_spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  return status;
}

/****************************************************************************
 * Name: mpfs_spi_cmddata
 *
 * Description:
 *   Some devices require an additional out-of-band bit to specify if the
 *   next word sent to the device is a command or data. This feature is not
 *   implemented.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   cmd - TRUE: The following word is a command; FALSE: the following words
 *         are data.
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static int mpfs_spi_cmddata(struct spi_dev_s *dev,
                            uint32_t devid, bool cmd)
{
  spierr("SPI cmddata not supported\n");
  DEBUGPANIC();

  return -1;
}
#endif

/****************************************************************************
 * Name: mpfs_spi_hwfeatures
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
static int mpfs_spi_hwfeatures(struct spi_dev_s *dev,
                               spi_hwfeatures_t features)
{
  /* Other H/W features are not supported */

  spierr("SPI hardware specific feature not supported\n");
  DEBUGPANIC();

  return -1;
}
#endif

/****************************************************************************
 * Name: mpfs_spi_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int mpfs_spi_sem_waitdone(struct mpfs_spi_priv_s *priv)
{
  return nxsem_tickwait_uninterruptible(&priv->sem_isr, SEC2TICK(10));
}

/****************************************************************************
 * Name: mpfs_spi_load_tx_fifo
 *
 * Description:
 *   Fill up the TX fifo with nwords (in 8 or 16-bit).
 *
 * Input Parameters:
 *   priv     - SPI private state data
 *   txbuffer - A pointer to the buffer of data to be sent
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

static void mpfs_spi_load_tx_fifo(struct mpfs_spi_priv_s *priv,
                                  const void *txbuffer,
                                  uint32_t nwords)
{
  uint32_t tx_fifo_full;
  uint16_t *data16;
  uint8_t  *data8;
  int i;

  DEBUGASSERT(nwords > 0);

  data16 = (uint16_t *)txbuffer;
  data8  = (uint8_t *)txbuffer;

  for (i = 0; i < nwords; i++)
    {
      if (txbuffer)
        {
          if (priv->nbits == 8)
            {
              putreg32((uint32_t)data8[priv->tx_pos], MPFS_SPI_TX_DATA);
            }
          else
            {
              putreg32((uint32_t)data16[priv->tx_pos], MPFS_SPI_TX_DATA);
            }
        }
      else
        {
          putreg32(0, MPFS_SPI_TX_DATA);
        }

      priv->tx_pos++;
      tx_fifo_full = getreg32(MPFS_SPI_STATUS) & MPFS_SPI_TXFIFOFUL;

      DEBUGASSERT(tx_fifo_full == 0);
    }
}

/****************************************************************************
 * Name: mpfs_spi_unload_rx_fifo
 *
 * Description:
 *   Unload the RX fifo with nwords -number of elements. That is, read the
 *   specified number of elements from the RX fifo.
 *
 * Input Parameters:
 *   priv     - SPI private state data
 *   txbuffer - A pointer to the buffer of data for receiving data
 *   nwords   - the length of data that to be exchanged in units of words.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_spi_unload_rx_fifo(struct mpfs_spi_priv_s *priv,
                                    void *rxbuffer,
                                    uint32_t nwords)
{
  uint32_t rx_fifo_empty;
  uint16_t *data16;
  uint8_t  *data8;
  int i;

  DEBUGASSERT(nwords > 0);

  data16 = (uint16_t *)rxbuffer;
  data8  = (uint8_t *)rxbuffer;

  for (i = 0; i < nwords; i++)
    {
      rx_fifo_empty = getreg32(MPFS_SPI_STATUS) & MPFS_SPI_RXFIFOEMP;
      DEBUGASSERT(rx_fifo_empty == 0);

      if (rxbuffer)
        {
          if (priv->nbits == 8)
            {
              data8[priv->rx_pos] = getreg32(MPFS_SPI_RX_DATA);
            }
          else
            {
              data16[priv->rx_pos] = getreg32(MPFS_SPI_RX_DATA);
            }
        }
      else
        {
          getreg32(MPFS_SPI_RX_DATA);
        }

      priv->rx_pos++;

      DEBUGASSERT(priv->rx_pos <= priv->rxwords);
    }
}

/****************************************************************************
 * Name: mpfs_spi_irq_exchange
 *
 * Description:
 *   Exchange a block of data from SPI by DMA.
 *
 * Input Parameters:
 *   priv     - SPI private state data
 *   txbuffer - A pointer to the buffer of data to be sent
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

static void mpfs_spi_irq_exchange(struct mpfs_spi_priv_s *priv,
                                  const void *txbuffer,
                                  void *rxbuffer, uint32_t nwords)
{
  uint32_t status;

  DEBUGASSERT(nwords >= 1u && nwords < 0xffffu);

  /* SPI fifo clear  */

  modifyreg32(MPFS_SPI_COMMAND, 0, MPFS_SPI_TXFIFORST | MPFS_SPI_RXFIFORST);

  /* Recover from RX overflow condition if present */

  status = getreg32(MPFS_SPI_STATUS);
  if (status & MPFS_SPI_RXOVERFLOW)
    {
      mpfs_spi_rxoverflow_recover(priv);
    }

  DEBUGASSERT(priv->nbits == 8 || priv->nbits == 16);

  priv->fifosize = (256 / priv->nbits);
  priv->fifolevel = priv->fifosize / 2;

  priv->txwords = nwords;
  priv->txbuf   = txbuffer;
  priv->tx_pos  = 0;

  priv->rxwords = nwords;
  priv->rxbuf   = rxbuffer;
  priv->rx_pos  = 0;
  priv->error   = 0;

  putreg32(0, MPFS_SPI_FRAMESUP);

  /* Clear the interrupts before writing FIFO */

  putreg32(MPFS_SPI_TXCHUNDRUN |
           MPFS_SPI_RXCHOVRFLW |
           MPFS_SPI_RXRDONECLR |
           MPFS_SPI_TXDONECLR, MPFS_SPI_INT_CLEAR);

  if (nwords > priv->fifosize)
    {
      modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_FRAMECNT,
                  MPFS_SPI_FRAMECNT & (priv->fifolevel << 8));
      mpfs_spi_load_tx_fifo(priv, txbuffer, priv->fifolevel);
    }
  else
    {
      modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_FRAMECNT,
                  MPFS_SPI_FRAMECNT & (nwords << 8));
      mpfs_spi_load_tx_fifo(priv, txbuffer, nwords);
    }

  /* Enable TX, RX, underrun and overflow interrupts */

  modifyreg32(MPFS_SPI_CONTROL, 0, MPFS_SPI_INTTXTURUN |
                                   MPFS_SPI_INTRXOVRFLOW |
                                   MPFS_SPI_INTRXDATA |
                                   MPFS_SPI_INTTXDATA);

  if (mpfs_spi_sem_waitdone(priv) < 0)
    {
      spiinfo("Message timed out\n");
      priv->error = -ETIMEDOUT;
    }

  modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_INTTXTURUN |
                                MPFS_SPI_INTRXOVRFLOW |
                                MPFS_SPI_INTRXDATA |
                                MPFS_SPI_INTTXDATA,
                                0);

  putreg32(MPFS_SPI_TXCHUNDRUN |
           MPFS_SPI_RXCHOVRFLW |
           MPFS_SPI_RXRDONECLR |
           MPFS_SPI_TXDONECLR, MPFS_SPI_INT_CLEAR);
}

/****************************************************************************
 * Name: mpfs_spi_poll_send
 *
 * Description:
 *   Exchange one word on SPI by polling mode.
 *
 * Input Parameters:
 *   priv - SPI private state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value
 *
 ****************************************************************************/

static uint32_t mpfs_spi_poll_send(struct mpfs_spi_priv_s *priv,
                                   uint32_t wd)
{
  uint32_t max_tx_cnt = 10000;
  uint32_t max_rx_cnt = 10000;
  uint32_t tx_done = 0;
  uint32_t rx_ready = 0;
  uint32_t rx_fifo_empty;

  uint32_t val = 0;

  /* Write data to tx fifo */

  putreg32(wd, MPFS_SPI_TX_DATA);

  /* Wait until TX data is sent */

  while (!tx_done && max_tx_cnt)
    {
      tx_done = getreg32(MPFS_SPI_STATUS) & MPFS_SPI_TXDATSENT;
      max_tx_cnt--;
    }

  /* Wait for rx data */

  while (!rx_ready && max_rx_cnt)
    {
      rx_ready = getreg32(MPFS_SPI_STATUS) & MPFS_SPI_RXDATRCED;
      max_rx_cnt--;
    }

  val = getreg32(MPFS_SPI_RX_DATA);

  /* Flush the RX fifo just in case */

  rx_fifo_empty = getreg32(MPFS_SPI_STATUS) & MPFS_SPI_RXFIFOEMP;
  while (!rx_fifo_empty)
    {
        getreg32(MPFS_SPI_RX_DATA);
        rx_fifo_empty = getreg32(MPFS_SPI_STATUS) & MPFS_SPI_RXFIFOEMP;
    }

  spiinfo("send=%x and recv=%x\n", wd, val);

  return val;
}

/****************************************************************************
 * Name: mpfs_spi_send
 *
 * Description:
 *   Exchange one word on SPI.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value
 *
 ****************************************************************************/

static uint32_t mpfs_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct mpfs_spi_priv_s *priv = (struct mpfs_spi_priv_s *)dev;
  uint32_t rd;

  rd = mpfs_spi_poll_send(priv, wd);

  return rd;
}

/****************************************************************************
 * Name: mpfs_spi_poll_exchange
 *
 * Description:
 *   Exchange a block of data from SPI.
 *
 * Input Parameters:
 *   priv     - SPI private state data
 *   txbuffer - A pointer to the buffer of data to be sent
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

static void mpfs_spi_poll_exchange(struct mpfs_spi_priv_s *priv,
                                   const void *txbuffer,
                                   void *rxbuffer, size_t nwords)
{
  uint32_t w_wd = 0xffff;
  uint32_t r_wd;
  uint32_t status;
  int i;

  /* SPI fifo clear  */

  modifyreg32(MPFS_SPI_COMMAND, 0, MPFS_SPI_TXFIFORST | MPFS_SPI_RXFIFORST);

  /* Recover from RX overflow condition if present */

  status = getreg32(MPFS_SPI_STATUS);
  if (status & MPFS_SPI_RXOVERFLOW)
    {
      mpfs_spi_rxoverflow_recover(priv);
    }

  putreg32(0, MPFS_SPI_FRAMESUP);
  modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_FRAMECNT,
              MPFS_SPI_FRAMECNT & (nwords << 8));

  for (i = 0; i < nwords; i++)
    {
      if (txbuffer)
        {
          if (priv->nbits == 8)
            {
              w_wd = ((uint8_t *)txbuffer)[i];
            }
          else
            {
              w_wd = ((uint16_t *)txbuffer)[i];
            }
        }

      r_wd = mpfs_spi_poll_send(priv, w_wd);

      if (rxbuffer)
        {
          if (priv->nbits == 8)
            {
              ((uint8_t *)rxbuffer)[i] = r_wd;
            }
          else
            {
              ((uint16_t *)rxbuffer)[i] = r_wd;
            }
        }
    }
}

/****************************************************************************
 * Name: mpfs_spi_exchange
 *
 * Description:
 *   Exchange a block of data from SPI.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
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

static void mpfs_spi_exchange(struct spi_dev_s *dev,
                              const void *txbuffer, void *rxbuffer,
                              size_t nwords)
{
  struct mpfs_spi_priv_s *priv = (struct mpfs_spi_priv_s *)dev;

  if (priv->config->use_irq)
    {
      mpfs_spi_irq_exchange(priv, txbuffer, rxbuffer, nwords);
    }
  else
    {
      mpfs_spi_poll_exchange(priv, txbuffer, rxbuffer, nwords);
    }
}

#ifndef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: mpfs_spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
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

static void mpfs_spi_sndblock(struct spi_dev_s *dev,
                              const void *txbuffer, size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%lu\n", txbuffer, nwords);

  mpfs_spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: mpfs_spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI.
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to receive data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-
 *            per-word selected for the SPI interface.  If nbits <= 8, the
 *            data is packed into uint8_t's; if nbits >8, the data is packed
 *            into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_spi_recvblock(struct spi_dev_s *dev,
                               void *rxbuffer, size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%lu\n", rxbuffer, nwords);

  mpfs_spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: mpfs_spi_trigger
 *
 * Description:
 *   Trigger a previously configured DMA transfer.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   OK       - Trigger was fired
 *   -ENOSYS  - Trigger not fired due to lack of DMA or low level support
 *   -EIO     - Trigger not fired because not previously primed
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_TRIGGER
static int mpfs_spi_trigger(struct spi_dev_s *dev)
{
  spierr("SPI trigger not supported\n");
  DEBUGPANIC();

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: mpfs_spi_set_master_mode
 *
 * Description:
 *   set spi mode
 *
 * Input Parameters:
 *   master   - master or slave
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_spi_set_master_mode(struct mpfs_spi_priv_s *priv,
                                     uint8_t master)
{
  if (priv->enabled)
    {
      modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_ENABLE, 0);
    }

  if (master)
    {
      modifyreg32(MPFS_SPI_CONTROL, 0, MPFS_SPI_MODE);
    }
  else
    {
      modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_MODE, 0);
    }

  modifyreg32(MPFS_SPI_CONTROL, 0, MPFS_SPI_BIGFIFO | MPFS_SPI_CLKMODE);

  if (priv->enabled)
    {
      modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_ENABLE, MPFS_SPI_ENABLE);
    }
}

/****************************************************************************
 * Name: mpfs_spi_irq
 *
 * Description:
 *   This is the common SPI interrupt handler. It will be invoked when an
 *   interrupt is received on the device.
 *
 * Parameters:
 *   cpuint        - CPU interrupt index
 *   context       - Context data from the ISR
 *   arg           - Opaque pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int mpfs_spi_irq(int cpuint, void *context, void *arg)
{
  struct mpfs_spi_priv_s *priv = (struct mpfs_spi_priv_s *)arg;
  uint32_t remaining;
  uint32_t status;

  status = getreg32(MPFS_SPI_INTMASK);

  spiinfo("irq status=%x\n", status);

  if (status & MPFS_SPI_RXRDYMSKINT)
    {
      remaining = priv->rxwords - priv->rx_pos;

      if (remaining <= priv->fifosize)
        {
          mpfs_spi_unload_rx_fifo(priv, priv->rxbuf, remaining);
        }
      else
        {
          mpfs_spi_unload_rx_fifo(priv, priv->rxbuf, priv->fifolevel);
        }

      putreg32(MPFS_SPI_RXRDONECLR, MPFS_SPI_INT_CLEAR);
    }

  if (status & MPFS_SPI_TXDONEMSKINT)
    {
      remaining = priv->txwords - priv->tx_pos;
      putreg32(MPFS_SPI_TXDONECLR, MPFS_SPI_INT_CLEAR);

      if (remaining == 0)
        {
          /* Done sending - finish up */

          nxsem_post(&priv->sem_isr);
        }
      else
        {
          if (remaining <= priv->fifosize)
            {
              modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_FRAMECNT,
                          MPFS_SPI_FRAMECNT & (remaining << 8));
              mpfs_spi_load_tx_fifo(priv, priv->txbuf, remaining);
            }
          else
            {
              modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_FRAMECNT,
                          MPFS_SPI_FRAMECNT & (priv->fifolevel << 8));
              mpfs_spi_load_tx_fifo(priv, priv->txbuf, priv->fifolevel);
            }
        }
    }

  if (status & MPFS_SPI_RXCHOVRFMSKINT)
    {
      /* Handle receive overflow */

      modifyreg32(MPFS_SPI_COMMAND, 0, MPFS_SPI_RXFIFORST);
      mpfs_spi_rxoverflow_recover(priv);

      putreg32(MPFS_SPI_RXCHOVRFLW, MPFS_SPI_INT_CLEAR);
    }

  if (status & MPFS_SPI_TXCHUNDDMSKINT)
    {
      /* Handle transmit underrun */

      modifyreg32(MPFS_SPI_COMMAND, 0, MPFS_SPI_TXFIFORST);
      putreg32(MPFS_SPI_TXCHUNDRUN, MPFS_SPI_INT_CLEAR);
    }

  if (status & MPFS_SPI_CMDMSKINT)
    {
      /* Disable command interrupt to avoid retriggering */

      modifyreg32(MPFS_SPI_CONTROL2, MPFS_SPI_INTEN_CMD, 0);
      putreg32(MPFS_SPI_CMDINT, MPFS_SPI_INT_CLEAR);
    }

  if (status & MPFS_SPI_SSENDMSKINT)
    {
      modifyreg32(MPFS_SPI_COMMAND, 0, MPFS_SPI_TXFIFORST |
                  MPFS_SPI_RXFIFORST);
      putreg32(MPFS_SPI_SSEND, MPFS_SPI_INT_CLEAR);
    }

  return OK;
}

/****************************************************************************
 * Name: mpfs_spi_enable
 *
 * Description:
 *   Enable or disable the SPI bus
 *
 * Input Parameters:
 *   enable   - enable or disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_spi_enable(struct mpfs_spi_priv_s *priv, uint8_t enable)
{
  if (enable)
    {
      modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_ENABLE, MPFS_SPI_ENABLE);
      priv->enabled = 1;
    }
  else
    {
      modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_ENABLE, 0);
      priv->enabled = 0;
    }
}

/****************************************************************************
 * Name: mpfs_spi_init
 *
 * Description:
 *   Initialize mpfs SPI hardware interface
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_spi_init(struct spi_dev_s *dev)
{
  struct mpfs_spi_priv_s *priv = (struct mpfs_spi_priv_s *)dev;
  const struct mpfs_spi_config_s *config = priv->config;

  up_disable_irq(priv->plic_irq);

  /* Perform reset and enable clocks */

  if (priv->id == 0)
    {
      modifyreg32(MPFS_SYSREG_SOFT_RESET_CR, 0, SYSREG_SOFT_RESET_CR_SPI0);

      modifyreg32(MPFS_SYSREG_SOFT_RESET_CR, SYSREG_SOFT_RESET_CR_SPI0, 0);

      modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR, 0,
                  SYSREG_SUBBLK_CLOCK_CR_SPI0);
    }
  else
    {
      modifyreg32(MPFS_SYSREG_SOFT_RESET_CR, 0, SYSREG_SOFT_RESET_CR_SPI1);

      modifyreg32(MPFS_SYSREG_SOFT_RESET_CR, SYSREG_SOFT_RESET_CR_SPI1, 0);

      modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR, 0,
                  SYSREG_SUBBLK_CLOCK_CR_SPI1);
    }

  /* Reset the device */

  modifyreg32(MPFS_SPI_CONTROL, 0, MPFS_SPI_RESET);
  modifyreg32(MPFS_SPI_CONTROL, MPFS_SPI_RESET, 0);

  /* Install some default values */

  mpfs_spi_setfrequency(dev, config->clk_freq);
  mpfs_spi_setbits(dev, 8);
  mpfs_spi_setmode(dev, config->mode);

  /* Set master mode */

  mpfs_spi_set_master_mode(priv, 1);
  mpfs_spi_enable(priv, 1);

  up_enable_irq(priv->plic_irq);
}

/****************************************************************************
 * Name: mpfs_spi_deinit
 *
 * Description:
 *   Deinitialize mpfs SPI hardware interface
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_spi_deinit(struct spi_dev_s *dev)
{
  struct mpfs_spi_priv_s *priv = (struct mpfs_spi_priv_s *)dev;

  up_disable_irq(priv->plic_irq);
  mpfs_spi_enable(priv, 0);
  irq_detach(priv->plic_irq);

  /* Disable clock */

  if (priv->id == 0)
    {
      modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR,
                  SYSREG_SUBBLK_CLOCK_CR_SPI0, 0);
    }
  else
    {
      modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR,
                  SYSREG_SUBBLK_CLOCK_CR_SPI1, 0);
    }

  priv->frequency = 0;
  priv->actual    = 0;
  priv->mode      = SPIDEV_MODE0;
  priv->nbits     = 0;
}

/****************************************************************************
 * Name: mpfs_spibus_initialize
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

struct spi_dev_s *mpfs_spibus_initialize(int port)
{
  struct spi_dev_s *spi_dev;
  struct mpfs_spi_priv_s *priv;
  int ret;

  switch (port)
    {
#ifdef CONFIG_MPFS_SPI0
    case 0:
      priv = &g_mpfs_spi0_priv;
      break;
#endif
#ifdef CONFIG_MPFS_SPI1
    case 1:
      priv = &g_mpfs_spi1_priv;
      break;
#endif
  default:
    return NULL;
    }

  spi_dev = (struct spi_dev_s *)priv;

  nxmutex_lock(&priv->lock);
  if (priv->refs != 0)
    {
      priv->refs++;
      nxmutex_unlock(&priv->lock);

      return spi_dev;
    }

  ret = irq_attach(priv->plic_irq, mpfs_spi_irq, priv);
  if (ret != OK)
    {
      nxmutex_unlock(&priv->lock);
      return NULL;
    }

  mpfs_spi_init(spi_dev);
  priv->refs++;

  nxmutex_unlock(&priv->lock);
  return spi_dev;
}

/****************************************************************************
 * Name: mpfs_spibus_uninitialize
 *
 * Description:
 *   Uninitialize an SPI bus
 *
 ****************************************************************************/

int mpfs_spibus_uninitialize(struct spi_dev_s *dev)
{
  struct mpfs_spi_priv_s *priv = (struct mpfs_spi_priv_s *)dev;

  DEBUGASSERT(dev);

  if (priv->refs == 0)
    {
      return ERROR;
    }

  nxmutex_lock(&priv->lock);
  if (--priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  mpfs_spi_deinit(dev);
  nxmutex_unlock(&priv->lock);

  return OK;
}


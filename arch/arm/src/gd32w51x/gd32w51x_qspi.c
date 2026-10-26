/****************************************************************************
 * arch/arm/src/gd32w51x/gd32w51x_qspi.c
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

#include <sys/param.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/qspi.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "gd32w51x.h"
#include "gd32w51x_clockconfig.h"
#include "gd32w51x_dma.h"
#include "gd32w51x_gpio.h"
#include "gd32w51x_qspi.h"
#include "hardware/gd32w51x_qspi.h"
#include "hardware/gd32w51x_rcu.h"

#ifdef CONFIG_GD32W51X_QSPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The QSPI kernel clock is the AHB clock */

#define QSPI_CLOCK          GD32W51X_AHB_FREQ

/* Size of the attached memory, as the DCFG.FMSZ field: 2^(FMSZ + 1) bytes.
 * The indirect mode does not depend on it, but the address must stay
 * below that size.
 */

#define QSPI_FMSZ           (31)

/* Poll counts for the end of a transfer */

#define QSPI_TIMEOUT        0x00ffffff

/* Longest wait for a DMA read: 64 KB at the slowest useful clock */

#define QSPI_DMA_TIMEOUT    MSEC2TICK(1000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gd32_qspidev_s
{
  struct qspi_dev_s qspi;       /* Externally visible part of the device */
  mutex_t lock;                 /* Assures mutually exclusive access */
  uint32_t frequency;           /* Requested clock frequency */
  uint32_t actual;              /* Actual clock frequency */
  uint8_t mode;                 /* Mode 0 or 3 */
  bool initialized;
  bool memmap;                  /* In memory-mapped mode */
#ifdef CONFIG_GD32W51X_QSPI_DMA
  DMA_HANDLE rxdma;             /* DMA channel of the reads */
  sem_t dmawait;                /* Wait for the end of a DMA read */
  volatile uint16_t dmaresult;  /* DMA status, non-zero when done */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int      qspi_lock(struct qspi_dev_s *dev, bool lock);
static uint32_t qspi_setfrequency(struct qspi_dev_s *dev,
                                  uint32_t frequency);
static void     qspi_setmode(struct qspi_dev_s *dev, enum qspi_mode_e mode);
static void     qspi_setbits(struct qspi_dev_s *dev, int nbits);
static int      qspi_command(struct qspi_dev_s *dev,
                             struct qspi_cmdinfo_s *cmdinfo);
static int      qspi_memory(struct qspi_dev_s *dev,
                            struct qspi_meminfo_s *meminfo);
static void    *qspi_alloc(struct qspi_dev_s *dev, size_t buflen);
static void     qspi_free(struct qspi_dev_s *dev, void *buffer);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qspi_ops_s g_qspi_ops =
{
  .lock         = qspi_lock,
  .setfrequency = qspi_setfrequency,
  .setmode      = qspi_setmode,
  .setbits      = qspi_setbits,
  .command      = qspi_command,
  .memory       = qspi_memory,
  .alloc        = qspi_alloc,
  .free         = qspi_free,
};

static struct gd32_qspidev_s g_qspi0dev =
{
  .qspi =
  {
    .ops = &g_qspi_ops,
  },
  .lock = NXMUTEX_INITIALIZER,
#ifdef CONFIG_GD32W51X_QSPI_DMA
  .dmawait = SEM_INITIALIZER(0),
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qspi_wait_idle
 *
 * Description:
 *   Wait until the controller is not busy any more.
 *
 ****************************************************************************/

static int qspi_wait_idle(void)
{
  uint32_t timeout = QSPI_TIMEOUT;

  while ((getreg32(GD32W51X_QSPI_CTL) & QSPI_CTL_BUSY) != 0)
    {
      if (--timeout == 0)
        {
          return -ETIMEDOUT;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: qspi_wait_status
 *
 * Description:
 *   Wait until any of the given status flags is set.
 *
 ****************************************************************************/

static int qspi_wait_status(uint32_t flags)
{
  uint32_t timeout = QSPI_TIMEOUT;
  uint32_t stat;

  do
    {
      stat = getreg32(GD32W51X_QSPI_STAT);
      if ((stat & QSPI_STAT_TERR) != 0)
        {
          putreg32(QSPI_STATC_TERRC, GD32W51X_QSPI_STATC);
          return -EIO;
        }

      if (--timeout == 0)
        {
          return -ETIMEDOUT;
        }
    }
  while ((stat & flags) == 0);

  return OK;
}

#ifdef CONFIG_GD32W51X_QSPI_DMA
/****************************************************************************
 * Name: qspi_dmacallback
 ****************************************************************************/

static void qspi_dmacallback(DMA_HANDLE handle, uint16_t isr, void *arg)
{
  struct gd32_qspidev_s *priv = (struct gd32_qspidev_s *)arg;

  /* The status is OR'ed with 0x0080 to assure that it is non-zero */

  priv->dmaresult = isr | 0x0080;
  nxsem_post(&priv->dmawait);
}

/****************************************************************************
 * Name: qspi_dmaread
 *
 * Description:
 *   Move the data phase of an indirect read with the DMA.  TCFG has been
 *   written; the transfer starts when the address is written.
 *
 ****************************************************************************/

static int qspi_dmaread(struct gd32_qspidev_s *priv, uint32_t addr,
                        uint8_t *buffer, size_t buflen)
{
  struct gd32_dma_config_s cfg;
  int ret;

  cfg.periph_addr         = GD32W51X_QSPI_DATA;
  cfg.memory0_addr        = (uint32_t)buffer;
  cfg.number              = buflen;
  cfg.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
  cfg.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
  cfg.periph_memory_width = DMA_WIDTH_8BITS_SELECT;
  cfg.circular_mode       = DMA_CIRCULAR_MODE_DISABLE;
  cfg.direction           = DMA_PERIPH_TO_MEMORY;
  cfg.priority            = DMA_PRIO_HIGH_SELECT;

  priv->dmaresult = 0;
  gd32_dma_setup(priv->rxdma, &cfg, GD32_DMA_SINGLE_DATA_MODE);
  gd32_dma_start(priv->rxdma, qspi_dmacallback, priv,
                 DMA_CHXCTL_FTFIE | DMA_CHXCTL_TAEIE);

  modifyreg32(GD32W51X_QSPI_STAT, 0, QSPI_STAT_DMAEN);
  putreg32(addr, GD32W51X_QSPI_ADDR);

  do
    {
      ret = nxsem_tickwait_uninterruptible(&priv->dmawait,
                                           QSPI_DMA_TIMEOUT);
    }
  while (ret == OK && priv->dmaresult == 0);

  modifyreg32(GD32W51X_QSPI_STAT, QSPI_STAT_DMAEN, 0);
  gd32_dma_stop(priv->rxdma);

  if (ret == OK && (priv->dmaresult & DMA_INTF_TAEIF) != 0)
    {
      ret = -EIO;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: qspi_transfer
 *
 * Description:
 *   Run one indirect mode transfer.  'tcfg' holds everything but the
 *   functional mode; the transfer starts when TCFG (or ADDR, when there
 *   is an address phase) is written.
 *
 ****************************************************************************/

static int qspi_transfer(struct gd32_qspidev_s *priv, uint32_t tcfg,
                         bool hasaddr, uint32_t addr, uint8_t *buffer,
                         size_t buflen, bool read)
{
  size_t i;
  int ret;

  ret = qspi_wait_idle();
  if (ret < 0)
    {
      return ret;
    }

  putreg32(QSPI_STATC_TERRC | QSPI_STATC_TCC | QSPI_STATC_SMC |
           QSPI_STATC_TMOUTC | QSPI_STATC_WSC, GD32W51X_QSPI_STATC);

  if (buflen > 0)
    {
      putreg32(buflen - 1, GD32W51X_QSPI_DTLEN);
    }

  tcfg |= (read ? QSPI_FMOD_INDIRECT_READ : QSPI_FMOD_INDIRECT_WRITE) <<
          QSPI_TCFG_FMOD_SHIFT;
  putreg32(tcfg, GD32W51X_QSPI_TCFG);

#ifdef CONFIG_GD32W51X_QSPI_DMA
  if (read && hasaddr && buflen >= CONFIG_GD32W51X_QSPI_DMATHRESHOLD &&
      !up_interrupt_context())
    {
      ret = qspi_dmaread(priv, addr, buffer, buflen);
      if (ret < 0)
        {
          goto errout;
        }

      hasaddr = false;
      buflen  = 0;
    }
#endif

  if (hasaddr)
    {
      putreg32(addr, GD32W51X_QSPI_ADDR);
    }

  for (i = 0; i < buflen; i++)
    {
      ret = qspi_wait_status(read ? (QSPI_STAT_FT | QSPI_STAT_TC) :
                                    QSPI_STAT_FT);
      if (ret < 0)
        {
          goto errout;
        }

      if (read)
        {
          buffer[i] = getreg8(GD32W51X_QSPI_DATA);
        }
      else
        {
          putreg8(buffer[i], GD32W51X_QSPI_DATA);
        }
    }

  ret = qspi_wait_status(QSPI_STAT_TC);
  if (ret < 0)
    {
      goto errout;
    }

  putreg32(QSPI_STATC_TCC, GD32W51X_QSPI_STATC);
  return OK;

errout:
  modifyreg32(GD32W51X_QSPI_CTL, 0, QSPI_CTL_ABORT);
  qspi_wait_idle();
  spierr("ERROR: QSPI transfer failed: %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: qspi_lock
 ****************************************************************************/

static int qspi_lock(struct qspi_dev_s *dev, bool lock)
{
  struct gd32_qspidev_s *priv = (struct gd32_qspidev_s *)dev;

  if (lock)
    {
      return nxmutex_lock(&priv->lock);
    }

  return nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: qspi_setfrequency
 ****************************************************************************/

static uint32_t qspi_setfrequency(struct qspi_dev_s *dev,
                                  uint32_t frequency)
{
  struct gd32_qspidev_s *priv = (struct gd32_qspidev_s *)dev;
  uint32_t psc;

  if (frequency == 0)
    {
      return 0;
    }

  /* The clock cannot change in the memory-mapped mode */

  if (priv->memmap)
    {
      return priv->actual;
    }

  if (frequency == priv->frequency)
    {
      return priv->actual;
    }

  /* SCK = QSPI_CLOCK / (PSC + 1), rounded down to the requested value */

  psc = (QSPI_CLOCK + frequency - 1) / frequency;
  psc = psc > 0 ? psc - 1 : 0;
  if (psc > 255)
    {
      psc = 255;
    }

  if (qspi_wait_idle() == OK)
    {
      modifyreg32(GD32W51X_QSPI_CTL, QSPI_CTL_PSC_MASK,
                  psc << QSPI_CTL_PSC_SHIFT);
    }

  priv->frequency = frequency;
  priv->actual    = QSPI_CLOCK / (psc + 1);

  spiinfo("Frequency %" PRIu32 " -> %" PRIu32 "\n", frequency,
          priv->actual);
  return priv->actual;
}

/****************************************************************************
 * Name: qspi_setmode
 ****************************************************************************/

static void qspi_setmode(struct qspi_dev_s *dev, enum qspi_mode_e mode)
{
  struct gd32_qspidev_s *priv = (struct gd32_qspidev_s *)dev;

  /* Only the modes 0 and 3 are supported by the controller: the clock
   * level while idle is selected, the sampling edge is fixed.
   */

  if (mode != priv->mode && !priv->memmap && qspi_wait_idle() == OK)
    {
      modifyreg32(GD32W51X_QSPI_DCFG, QSPI_DCFG_CKMOD,
                  mode == QSPIDEV_MODE3 ? QSPI_DCFG_CKMOD : 0);
      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: qspi_setbits
 ****************************************************************************/

static void qspi_setbits(struct qspi_dev_s *dev, int nbits)
{
  /* Only 8-bit transfers are supported */

  DEBUGASSERT(nbits == 8);
}

/****************************************************************************
 * Name: qspi_command
 ****************************************************************************/

static int qspi_command(struct qspi_dev_s *dev,
                        struct qspi_cmdinfo_s *cmdinfo)
{
  uint32_t tcfg;
  uint32_t lines;

  /* Reject commands issued while in memory-mapped mode, which they would
   * cancel.  The memory-mapped mode must be left first.
   */

  if (((struct gd32_qspidev_s *)dev)->memmap)
    {
      return -EBUSY;
    }

  lines = QSPICMD_ISIQUAD(cmdinfo->flags) ? QSPI_LINES_4 :
          QSPICMD_ISIDUAL(cmdinfo->flags) ? QSPI_LINES_2 : QSPI_LINES_1;

  tcfg = (cmdinfo->cmd & 0xff) | (lines << QSPI_TCFG_IMOD_SHIFT);

  if (QSPICMD_ISADDRESS(cmdinfo->flags))
    {
      DEBUGASSERT(cmdinfo->addrlen >= 1 && cmdinfo->addrlen <= 4);
      tcfg |= (lines << QSPI_TCFG_ADDRMOD_SHIFT) |
              ((cmdinfo->addrlen - 1) << QSPI_TCFG_ADDRSZ_SHIFT);
    }

  if (QSPICMD_ISDATA(cmdinfo->flags))
    {
      DEBUGASSERT(cmdinfo->buffer != NULL && cmdinfo->buflen > 0);
      tcfg |= lines << QSPI_TCFG_DATAMOD_SHIFT;
    }

  return qspi_transfer((struct gd32_qspidev_s *)dev, tcfg,
                       QSPICMD_ISADDRESS(cmdinfo->flags),
                       cmdinfo->addr, cmdinfo->buffer,
                       QSPICMD_ISDATA(cmdinfo->flags) ? cmdinfo->buflen : 0,
                       !QSPICMD_ISWRITE(cmdinfo->flags));
}

/****************************************************************************
 * Name: qspi_memtcfg
 *
 * Description:
 *   Return the transfer configuration of a memory read or write, without
 *   the functional mode.
 *
 ****************************************************************************/

static uint32_t qspi_memtcfg(const struct qspi_meminfo_s *meminfo)
{
  uint32_t ilines;
  uint32_t alines;
  uint32_t dlines;

  ilines = (meminfo->flags & QSPIMEM_IQUAD) != 0 ? QSPI_LINES_4 :
           (meminfo->flags & QSPIMEM_IDUAL) != 0 ? QSPI_LINES_2 :
                                                   QSPI_LINES_1;

  if ((meminfo->flags & QSPIMEM_QUADIO) != 0)
    {
      alines = QSPI_LINES_4;
      dlines = QSPI_LINES_4;
    }
  else if ((meminfo->flags & QSPIMEM_DUALIO) != 0)
    {
      alines = QSPI_LINES_2;
      dlines = QSPI_LINES_2;
    }
  else
    {
      alines = ilines;
      dlines = (meminfo->flags & QSPIMEM_QUADDATA) != 0 ? QSPI_LINES_4 :
                                                         ilines;
    }

  DEBUGASSERT(meminfo->addrlen >= 1 && meminfo->addrlen <= 4);

  return (meminfo->cmd & 0xff) |
         (ilines << QSPI_TCFG_IMOD_SHIFT) |
         (alines << QSPI_TCFG_ADDRMOD_SHIFT) |
         ((meminfo->addrlen - 1) << QSPI_TCFG_ADDRSZ_SHIFT) |
         ((meminfo->dummies << QSPI_TCFG_DUMYC_SHIFT) &
          QSPI_TCFG_DUMYC_MASK) |
         (dlines << QSPI_TCFG_DATAMOD_SHIFT);
}

/****************************************************************************
 * Name: qspi_memory
 ****************************************************************************/

static int qspi_memory(struct qspi_dev_s *dev,
                       struct qspi_meminfo_s *meminfo)
{
  uint32_t tcfg;

  /* Reject transfers issued while in memory-mapped mode, which they would
   * cancel.  The memory-mapped mode must be left first.
   */

  if (((struct gd32_qspidev_s *)dev)->memmap)
    {
      return -EBUSY;
    }

  DEBUGASSERT(meminfo->buffer != NULL && meminfo->buflen > 0);

  tcfg = qspi_memtcfg(meminfo);

  return qspi_transfer((struct gd32_qspidev_s *)dev, tcfg, true,
                       meminfo->addr, meminfo->buffer, meminfo->buflen,
                       QSPIMEM_ISREAD(meminfo->flags));
}

/****************************************************************************
 * Name: qspi_alloc and qspi_free
 ****************************************************************************/

static void *qspi_alloc(struct qspi_dev_s *dev, size_t buflen)
{
  return kmm_malloc(buflen);
}

static void qspi_free(struct qspi_dev_s *dev, void *buffer)
{
  kmm_free(buffer);
}

/****************************************************************************
 * Name: qspi_pins_initialize
 *
 * Description:
 *   Route the QSPI signals.  The QSPI is a secure peripheral: its
 *   alternate functions only reach pins that are marked secure, while
 *   gd32w51x_tzinit() makes every pin non-secure at boot.  The pins are
 *   configured first, then handed back to the secure world.
 *
 ****************************************************************************/

static void qspi_pins_initialize(void)
{
  static const uint32_t pins[] =
  {
    GPIO_QSPI_SCK, GPIO_QSPI_CS, GPIO_QSPI_IO0, GPIO_QSPI_IO1,
    GPIO_QSPI_IO2, GPIO_QSPI_IO3
  };

  int i;

  for (i = 0; i < nitems(pins); i++)
    {
      gd32_gpio_config(pins[i]);
      gd32w51x_gpio_setsecure(pins[i], true);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_qspi_initialize
 ****************************************************************************/

struct qspi_dev_s *gd32_qspi_initialize(int intf)
{
  struct gd32_qspidev_s *priv = &g_qspi0dev;

  if (intf != 0)
    {
      return NULL;
    }

  if (!priv->initialized)
    {
      qspi_pins_initialize();

      modifyreg32(GD32W51X_RCU_AHB3EN, 0, RCU_AHB3EN_QSPIEN);

      /* Disable while configuring: FIFO threshold of one byte, a slow
       * clock until the upper half asks for more, CS high for 2 cycles
       * between commands and the largest memory size.
       */

      putreg32(0, GD32W51X_QSPI_CTL);
      putreg32(255 << QSPI_CTL_PSC_SHIFT, GD32W51X_QSPI_CTL);
      putreg32((1 << QSPI_DCFG_CSHC_SHIFT) |
               (QSPI_FMSZ << QSPI_DCFG_FMSZ_SHIFT), GD32W51X_QSPI_DCFG);
      modifyreg32(GD32W51X_QSPI_CTL, 0, QSPI_CTL_QSPIEN);

#ifdef CONFIG_GD32W51X_QSPI_DMA
      priv->rxdma = gd32_dma_channel_alloc(DMA_REQ_QSPI);
      DEBUGASSERT(priv->rxdma != NULL);
#endif

      priv->frequency   = 0;
      priv->mode        = QSPIDEV_MODE0;
      priv->initialized = true;
    }

  return &priv->qspi;
}

/****************************************************************************
 * Name: gd32_qspi_enter_memorymapped
 *
 * Description:
 *   Put the QSPI controller into memory-mapped mode: the memory is read in
 *   place through the GD32W51X_QSPIMEM_BASE window.  The QSPI command and
 *   memory operations fail with -EBUSY until gd32_qspi_exit_memorymapped().
 *
 * Input Parameters:
 *   dev     - QSPI device
 *   meminfo - The read transfer used to fetch the memory
 *   lpto    - Number of SCK cycles without access after which CS is
 *             released, or 0 to keep it asserted
 *
 ****************************************************************************/

void gd32_qspi_enter_memorymapped(struct qspi_dev_s *dev,
                                  const struct qspi_meminfo_s *meminfo,
                                  uint32_t lpto)
{
  struct gd32_qspidev_s *priv = (struct gd32_qspidev_s *)dev;

  qspi_lock(dev, true);

  if (!priv->memmap && qspi_wait_idle() == OK)
    {
      putreg32(QSPI_STATC_TERRC | QSPI_STATC_TCC | QSPI_STATC_SMC |
               QSPI_STATC_TMOUTC | QSPI_STATC_WSC, GD32W51X_QSPI_STATC);

      if (lpto > 0)
        {
          putreg32(lpto, GD32W51X_QSPI_TMOUT);
          modifyreg32(GD32W51X_QSPI_CTL, 0, QSPI_CTL_TMOUTEN);
        }
      else
        {
          modifyreg32(GD32W51X_QSPI_CTL, QSPI_CTL_TMOUTEN, 0);
        }

      putreg32(qspi_memtcfg(meminfo) |
               (QSPI_FMOD_MEMORY_MAPPED << QSPI_TCFG_FMOD_SHIFT),
               GD32W51X_QSPI_TCFG);
      priv->memmap = true;
    }

  qspi_lock(dev, false);
}

/****************************************************************************
 * Name: gd32_qspi_exit_memorymapped
 *
 * Description:
 *   Take the QSPI controller out of memory-mapped mode.
 *
 * Input Parameters:
 *   dev - QSPI device
 *
 ****************************************************************************/

void gd32_qspi_exit_memorymapped(struct qspi_dev_s *dev)
{
  struct gd32_qspidev_s *priv = (struct gd32_qspidev_s *)dev;

  qspi_lock(dev, true);

  if (priv->memmap)
    {
      /* The controller keeps CS asserted to prefetch: abort it */

      modifyreg32(GD32W51X_QSPI_CTL, 0, QSPI_CTL_ABORT);
      qspi_wait_idle();
      putreg32(QSPI_STATC_TERRC | QSPI_STATC_TCC | QSPI_STATC_SMC |
               QSPI_STATC_TMOUTC | QSPI_STATC_WSC, GD32W51X_QSPI_STATC);
      priv->memmap = false;
    }

  qspi_lock(dev, false);
}

#endif /* CONFIG_GD32W51X_QSPI */

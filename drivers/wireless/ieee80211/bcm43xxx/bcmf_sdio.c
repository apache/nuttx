/****************************************************************************
 * drivers/wireless/bcm43xxx/ieee80211/bcmf_sdio.c
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
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
#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <queue.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/kthread.h>
#include <nuttx/wdog.h>

#include <nuttx/wireless/ieee80211/mmc_sdio.h>
#include <nuttx/wireless/ieee80211/bcmf_sdio.h>
#include <nuttx/wireless/ieee80211/bcmf_board.h>

#include "bcmf_sdio.h"
#include "bcmf_core.h"
#include "bcmf_sdpcm.h"
#include "bcmf_utils.h"

#include "bcmf_sdio_core.h"
#include "bcmf_sdio_regs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BCMF_DEVICE_RESET_DELAY_MS 10
#define BCMF_DEVICE_START_DELAY_MS 10
#define BCMF_CLOCK_SETUP_DELAY_MS  500

#define BCMF_THREAD_NAME       "bcmf"
#define BCMF_THREAD_STACK_SIZE 2048

#define BCMF_WAITDOG_TIMEOUT_TICK (5*CLOCKS_PER_SEC)

/* Chip-common registers */

#define CHIPCOMMON_GPIO_CONTROL ((uint32_t)(0x18000000 + 0x6c) )
#define CHIPCOMMON_SR_CONTROL0  ((uint32_t)(0x18000000 + 0x504) )
#define CHIPCOMMON_SR_CONTROL1  ((uint32_t)(0x18000000 + 0x508) )

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Supported chip configurations */

#ifdef CONFIG_IEEE80211_BROADCOM_BCM43362
  extern const struct bcmf_sdio_chip bcmf_43362_config_sdio;
#endif
#ifdef CONFIG_IEEE80211_BROADCOM_BCM43438
  extern const struct bcmf_sdio_chip bcmf_43438_config_sdio;
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  bcmf_probe(FAR struct bcmf_sdio_dev_s *sbus);
static int  bcmf_hwinitialize(FAR struct bcmf_sdio_dev_s *sbus);
static void bcmf_hwuninitialize(FAR struct bcmf_sdio_dev_s *sbus);
static int  bcmf_chipinitialize(FAR struct bcmf_sdio_dev_s *sbus);

static int  bcmf_oob_irq(FAR void *arg);

static int  bcmf_sdio_bus_sleep(FAR struct bcmf_sdio_dev_s *sbus,
                                bool sleep);

static void bcmf_sdio_waitdog_timeout(wdparm_t arg);
static int  bcmf_sdio_thread(int argc, char **argv);

static int  bcmf_sdio_find_block_size(unsigned int size);

static int  bcmf_sdio_sr_init(FAR struct bcmf_sdio_dev_s *sbus);
static bool brcm_chip_sr_capable(FAR struct bcmf_sdio_dev_s *sbus);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* FIXME remove */

FAR struct bcmf_dev_s *g_sdio_priv;

/* Buffer pool for SDIO bus interface
 * This pool is shared between all driver devices
 */

static struct bcmf_sdio_frame g_pktframes[BCMF_PKT_POOL_SIZE];

/* TODO free_queue should be static */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int bcmf_oob_irq(FAR void *arg)
{
  FAR struct bcmf_sdio_dev_s *sbus = (FAR struct bcmf_sdio_dev_s *)arg;

  if (sbus->ready)
    {
      /* Signal bmcf thread */

      sbus->irq_pending = true;
      nxsem_post(&sbus->thread_signal);
    }

  return OK;
}

int bcmf_sdio_bus_sleep(FAR struct bcmf_sdio_dev_s *sbus, bool sleep)
{
  int ret;
  int loops;
  uint8_t value;

  if (sbus->sleeping == sleep)
    {
      return OK;
    }

  if (sleep)
    {
      sbus->sleeping = true;
      return bcmf_write_reg(sbus, 1, SBSDIO_FUNC1_CHIPCLKCSR, 0);
    }
  else
    {
      loops = 200;
      while (--loops > 0)
        {
          /* Request HT Avail */

          ret = bcmf_write_reg(sbus, 1, SBSDIO_FUNC1_CHIPCLKCSR,
                               SBSDIO_HT_AVAIL_REQ | SBSDIO_FORCE_HT);
          if (ret != OK)
            {
              wlerr("HT Avail request failed %d\n", ret);
              return ret;
            }

          /* Wait for High Throughput clock */

          up_mdelay(100);
          ret = bcmf_read_reg(sbus, 1, SBSDIO_FUNC1_CHIPCLKCSR, &value);

          if (ret != OK)
            {
              return ret;
            }

          if (value & SBSDIO_HT_AVAIL)
            {
              /* High Throughput clock is ready */

              break;
            }
        }

      if (loops <= 0)
        {
          wlerr("HT clock not ready\n");
          return -ETIMEDOUT;
        }

      sbus->sleeping = false;
    }

  return OK;
}

/****************************************************************************
 * Name: bcmf_probe
 ****************************************************************************/

int bcmf_probe(FAR struct bcmf_sdio_dev_s *sbus)
{
  int ret;

  /* Probe sdio card compatible device */

  ret = sdio_probe(sbus->sdio_dev);
  if (ret != OK)
    {
      goto exit_error;
    }

  /* Set FN0 / FN1 / FN2 default block size */

  ret = sdio_set_blocksize(sbus->sdio_dev, 0, 64);
  if (ret != OK)
    {
      goto exit_error;
    }

  ret = sdio_set_blocksize(sbus->sdio_dev, 1, 64);
  if (ret != OK)
    {
      goto exit_error;
    }

  ret = sdio_set_blocksize(sbus->sdio_dev, 2, 64);
  if (ret != OK)
    {
      goto exit_error;
    }

  /* Enable device interrupts for FN0, FN1 and FN2 */

  ret = bcmf_write_reg(sbus, 0, SDIO_CCCR_INTEN,
                       (1 << 0) | (1 << 1) | (1 << 2));
  if (ret != OK)
    {
      goto exit_error;
    }

  /* Default device clock speed is up to 25 MHz
   * We could set EHS bit to operate at a clock rate up to 50 MHz.
   */

  SDIO_CLOCK(sbus->sdio_dev, CLOCK_SD_TRANSFER_4BIT);
  up_mdelay(BCMF_CLOCK_SETUP_DELAY_MS);

  /* Enable bus FN1 */

  ret = sdio_enable_function(sbus->sdio_dev, 1);
  if (ret != OK)
    {
      goto exit_error;
    }

  return OK;

exit_error:

  wlerr("ERROR: failed to probe device %d\n", sbus->minor);
  return ret;
}

/****************************************************************************
 * Name: bcmf_businitialize
 ****************************************************************************/

int bcmf_businitialize(FAR struct bcmf_sdio_dev_s *sbus)
{
  int ret;
  int loops;
  uint8_t value;

  /* Send Active Low-Power clock request */

  ret = bcmf_write_reg(sbus, 1, SBSDIO_FUNC1_CHIPCLKCSR,
                       SBSDIO_FORCE_HW_CLKREQ_OFF |
                       SBSDIO_ALP_AVAIL_REQ |
                       SBSDIO_FORCE_ALP);

  if (ret != OK)
    {
      return ret;
    }

  loops = 10;
  while (--loops > 0)
    {
      up_mdelay(10);
      ret = bcmf_read_reg(sbus, 1, SBSDIO_FUNC1_CHIPCLKCSR, &value);

      if (ret != OK)
        {
          return ret;
        }

      if (value & SBSDIO_ALP_AVAIL)
        {
          /* Active Low-Power clock is ready */

          break;
        }
    }

  if (loops <= 0)
    {
      wlerr("failed to enable ALP\n");
      return -ETIMEDOUT;
    }

  /* Clear Active Low-Power clock request */

  ret = bcmf_write_reg(sbus, 1, SBSDIO_FUNC1_CHIPCLKCSR, 0);
  if (ret != OK)
    {
      return ret;
    }

  /* Disable pull-ups on SDIO cmd, d0-2 lines */

  ret = bcmf_write_reg(sbus, 1, SBSDIO_FUNC1_SDIOPULLUP, 0);
  if (ret != OK)
    {
      return ret;
    }

  /* Do chip specific initialization */

  ret = bcmf_chipinitialize(sbus);
  if (ret != OK)
    {
      return ret;
    }

  /* Upload firmware */

  ret = bcmf_core_upload_firmware(sbus);
  if (ret != OK)
    {
      return ret;
    }

  /* Enable FN2 (frame transfers) */

  ret = sdio_enable_function(sbus->sdio_dev, 2);
  if (ret != OK)
    {
      return ret;
    }

  return OK;
}

int bcmf_bus_setup_interrupts(FAR struct bcmf_sdio_dev_s *sbus)
{
  int ret;

  /* Configure gpio interrupt pin */

  bcmf_board_setup_oob_irq(sbus->minor, bcmf_oob_irq, (void *)sbus);

  /* Enable function 2 interrupt */

  ret = sdio_enable_interrupt(sbus->sdio_dev, 0);
  if (ret != OK)
    {
      return ret;
    }

  ret = sdio_enable_interrupt(sbus->sdio_dev, 2);
  if (ret != OK)
    {
      return ret;
    }

#ifndef CONFIG_BCMFMAC_NO_OOB
  /* Redirect, configure and enable io for out-of-band interrupt signal */

  ret = bcmf_write_reg(sbus, 0, SDIO_CCCR_BRCM_SEPINT,
                       SDIO_SEPINT_MASK | SDIO_SEPINT_OE |
                       SDIO_SEPINT_ACT_HI);
  if (ret != OK)
    {
      return ret;
    }
#endif

  /* Wake up chip to be sure function 2 is running */

  ret = bcmf_sdio_bus_sleep(sbus, false);
  if (ret != OK)
    {
      return ret;
    }

  /* FN2 successfully enabled, set core and enable interrupts */

  bcmf_write_sbregw(sbus,
                   CORE_BUS_REG(sbus->chip->core_base[SDIOD_CORE_ID],
                   hostintmask), I_HMB_SW_MASK);

  bcmf_write_sbregb(sbus,
                   CORE_BUS_REG(sbus->chip->core_base[SDIOD_CORE_ID],
                   funcintmask), 2);

  /* Lower F2 Watermark to avoid DMA Hang in F2 when SD Clock is stopped. */

  bcmf_write_reg(sbus, 1, SBSDIO_WATERMARK, 8);
  return OK;
}

/****************************************************************************
 * Name: bcmf_hwinitialize
 ****************************************************************************/

int bcmf_hwinitialize(FAR struct bcmf_sdio_dev_s *sbus)
{
  /* Attach and prepare SDIO interrupts */

  SDIO_ATTACH(sbus->sdio_dev);

  /* Set ID mode clocking (<400KHz) */

  SDIO_CLOCK(sbus->sdio_dev, CLOCK_IDMODE);

  /* Configure hardware */

  bcmf_board_initialize(sbus->minor);

  /* Reset and power device */

  bcmf_board_reset(sbus->minor, true);
  bcmf_board_power(sbus->minor, true);
  up_mdelay(BCMF_DEVICE_RESET_DELAY_MS);
  bcmf_board_reset(sbus->minor, false);

  /* Wait for device to start */

  up_mdelay(BCMF_DEVICE_START_DELAY_MS);

  return OK;
}

/****************************************************************************
 * Name: bcmf_hwuninitialize
 ****************************************************************************/

void bcmf_hwuninitialize(FAR struct bcmf_sdio_dev_s *sbus)
{
  /*  Shutdown device */

  bcmf_board_power(sbus->minor, false);
  bcmf_board_reset(sbus->minor, true);
}

int bcmf_sdio_find_block_size(unsigned int size)
{
  int ret = 0;
  int size_copy = size;

  while (size_copy)
    {
      size_copy >>= 1;
      ret++;
    }

  if (size & (size - 1))
    {
      return 1 << ret;
    }

  return 1 << (ret - 1);
}

/* Init save-restore if the firmware support it: */

static int bcmf_sdio_sr_init(FAR struct bcmf_sdio_dev_s *sbus)
{
  uint8_t  data;

  if (brcm_chip_sr_capable(sbus))
    {
      /* Configure WakeupCtrl register to set HtAvail request bit in
       * chipClockCSR register after the sdiod core is powered on.
       */

      bcmf_read_reg(sbus, 1, SBSDIO_FUNC1_WAKEUPCTRL, &data);
      data |= SBSDIO_FUNC1_WCTRL_HTWAIT_MASK;
      bcmf_write_reg(sbus, 1, SBSDIO_FUNC1_WAKEUPCTRL, data);

      /* Set brcmCardCapability to noCmdDecode mode.
       * It makes sdiod_aos to wakeup host for any activity of cmd line,
       * even though module won't decode cmd or respond
       */

      bcmf_write_reg(sbus, 0, SDIO_CCCR_BRCM_CARDCAP,
                     SDIO_CCCR_BRCM_CARDCAP_CMD_NODEC);
      bcmf_write_reg(sbus, 1, SBSDIO_FUNC1_CHIPCLKCSR,
                     SBSDIO_FORCE_HT);

      /* Enable KeepSdioOn (KSO) bit for normal operation */

      bcmf_read_reg(sbus, 1, SBSDIO_FUNC1_SLEEPCSR, &data);
      if ((data & SBSDIO_FUNC1_SLEEPCSR_KSO_MASK) == 0)
        {
          data |= SBSDIO_FUNC1_SLEEPCSR_KSO_MASK;
          bcmf_write_reg(sbus, 1, SBSDIO_FUNC1_SLEEPCSR, data);
        }
    }

  return OK;
}

/* Check if the firmware supports save restore feature.
 * TODO: Add more chip specific logic, and move it to a new bcmf_chip.c file.
 */

static bool brcm_chip_sr_capable(FAR struct bcmf_sdio_dev_s *sbus)
{
  uint32_t srctrl = 0;
  int ret;

  /* Check if fw initialized sr engine */

  ret = bcmf_read_sbregw(sbus, CHIPCOMMON_SR_CONTROL1, &srctrl);
  if (ret != OK)
    {
      return false;
    }
  else
    {
      return (srctrl != 0);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_transfer_bytes
 ****************************************************************************/

int bcmf_transfer_bytes(FAR struct bcmf_sdio_dev_s *sbus, bool write,
                        uint8_t function, uint32_t address,
                        uint8_t *buf, unsigned int len)
{
  unsigned int blocklen;
  unsigned int nblocks;

  /*  Use rw_io_direct method if len is 1 */

  if (len == 0)
    {
      return -EINVAL;
    }

  if (len == 1)
    {
      if (write)
        {
          return sdio_io_rw_direct(sbus->sdio_dev, write,
                                   function, address, *buf, NULL);
        }

      return sdio_io_rw_direct(sbus->sdio_dev, write,
                               function, address, 0, buf);
    }

  /* Find best block settings for transfer */

  if (len >= 64)
    {
      /* Use block mode */

      blocklen = 64;
      nblocks = (len + 63) / 64;
    }
  else
    {
      /* Use byte mode */

      blocklen = bcmf_sdio_find_block_size(len);
      nblocks = 0;
    }

  return sdio_io_rw_extended(sbus->sdio_dev, write, function, address, true,
                             buf, blocklen, nblocks);
}

/****************************************************************************
 * Name: bcmf_read_reg
 ****************************************************************************/

int bcmf_read_reg(FAR struct bcmf_sdio_dev_s *sbus, uint8_t function,
                  uint32_t address, uint8_t *reg)
{
  *reg = 0;
  return bcmf_transfer_bytes(sbus, false, function, address, reg, 1);
}

/****************************************************************************
 * Name: bcmf_write_reg
 ****************************************************************************/

int bcmf_write_reg(FAR struct bcmf_sdio_dev_s *sbus, uint8_t function,
                   uint32_t address, uint8_t reg)
{
  return bcmf_transfer_bytes(sbus, true, function, address, &reg, 1);
}

/****************************************************************************
 * Name: bcmf_bus_sdio_initialize
 ****************************************************************************/

int bcmf_bus_sdio_initialize(FAR struct bcmf_dev_s *priv,
                             int minor, FAR struct sdio_dev_s *dev)
{
  int ret;
  FAR struct bcmf_sdio_dev_s *sbus;

  /* Allocate sdio bus structure */

  sbus = (FAR struct bcmf_sdio_dev_s *)kmm_malloc(sizeof(*sbus));

  if (!sbus)
    {
      return -ENOMEM;
    }

  /* Initialize sdio bus device structure */

  memset(sbus, 0, sizeof(*sbus));
  sbus->sdio_dev           = dev;
  sbus->minor              = minor;
  sbus->ready              = false;
  sbus->sleeping           = true;

  sbus->bus.txframe        = bcmf_sdpcm_queue_frame;
  sbus->bus.rxframe        = bcmf_sdpcm_get_rx_frame;
  sbus->bus.allocate_frame = bcmf_sdpcm_alloc_frame;
  sbus->bus.free_frame     = bcmf_sdpcm_free_frame;
  sbus->bus.stop           = NULL; /* TODO */

  /* Init transmit frames queue */

  if ((ret = nxsem_init(&sbus->queue_mutex, 0, 1)) != OK)
    {
      goto exit_free_bus;
    }

  sq_init(&sbus->tx_queue);
  sq_init(&sbus->rx_queue);
  sq_init(&sbus->free_queue);

  /* Setup free buffer list */

  /* FIXME this should be static to driver */

  for (ret = 0; ret < BCMF_PKT_POOL_SIZE; ret++)
    {
      bcmf_dqueue_push(&sbus->free_queue, &g_pktframes[ret].list_entry);
    }

  /* Init thread semaphore */

  if ((ret = nxsem_init(&sbus->thread_signal, 0, 0)) != OK)
    {
      goto exit_free_bus;
    }

  if ((ret = nxsem_set_protocol(&sbus->thread_signal, SEM_PRIO_NONE)) != OK)
    {
      goto exit_free_bus;
    }

  /* Initialize device hardware */

  ret = bcmf_hwinitialize(sbus);
  if (ret != OK)
    {
      goto exit_free_bus;
    }

  /* Probe device */

  ret = bcmf_probe(sbus);
  if (ret != OK)
    {
      goto exit_uninit_hw;
    }

  /* Initialize device bus */

  ret = bcmf_businitialize(sbus);
  if (ret != OK)
    {
      goto exit_uninit_hw;
    }

  up_mdelay(100);

  sbus->ready = true;

  ret = bcmf_bus_setup_interrupts(sbus);
  if (ret != OK)
    {
      goto exit_uninit_hw;
    }

  ret = bcmf_sdio_sr_init(sbus);
  if (ret != OK)
    {
      goto exit_uninit_hw;
    }

  /* FIXME global variable for now */

  g_sdio_priv = priv;

  /* Register sdio bus */

  priv->bus = &sbus->bus;

  /* Start the waitdog timer */

  wd_start(&sbus->waitdog, BCMF_WAITDOG_TIMEOUT_TICK,
           bcmf_sdio_waitdog_timeout, (wdparm_t)priv);

  /* Spawn bcmf daemon thread */

  ret = kthread_create(BCMF_THREAD_NAME, SCHED_PRIORITY_MAX,
                       BCMF_THREAD_STACK_SIZE, bcmf_sdio_thread,
                       (FAR char * const *)NULL);

  if (ret <= 0)
    {
      wlerr("Cannot spawn bcmf thread\n");
      ret = -EBADE;
      goto exit_uninit_hw;
    }

  sbus->thread_id = ret;

  /* SDIO bus is up and running */

  return OK;

exit_uninit_hw:
  bcmf_hwuninitialize(sbus);

exit_free_bus:
  kmm_free(sbus);
  priv->bus = NULL;
  return ret;
}

int bcmf_chipinitialize(FAR struct bcmf_sdio_dev_s *sbus)
{
  uint32_t value = 0;
  int chipid;
  int ret;

  ret = bcmf_read_sbregw(sbus, SI_ENUM_BASE, &value);
  if (ret != OK)
    {
      return ret;
    }

  chipid            = value & 0xffff;
  sbus->cur_chip_id = chipid;

  switch (chipid)
    {
#ifdef CONFIG_IEEE80211_BROADCOM_BCM43362
      case SDIO_DEVICE_ID_BROADCOM_43362:
        wlinfo("bcm43362 chip detected\n");
        sbus->chip = (struct bcmf_sdio_chip *)&bcmf_43362_config_sdio;
        break;
#endif

#ifdef CONFIG_IEEE80211_BROADCOM_BCM43438
      case SDIO_DEVICE_ID_BROADCOM_43430:
        wlinfo("bcm43438 chip detected\n");
        sbus->chip = (struct bcmf_sdio_chip *)&bcmf_43438_config_sdio;
        break;
#endif

      default:
        wlerr("chip 0x%x is not supported\n", chipid);
        return -ENODEV;
    }

  return OK;
}

void bcmf_sdio_waitdog_timeout(wdparm_t arg)
{
  FAR struct bcmf_dev_s *priv = (FAR struct bcmf_dev_s *)arg;
  FAR struct bcmf_sdio_dev_s *sbus = (FAR struct bcmf_sdio_dev_s *)priv->bus;

  /* Notify bcmf thread */

  wlinfo("Notify bcmf thread\n");
  nxsem_post(&sbus->thread_signal);
}

int bcmf_sdio_thread(int argc, char **argv)
{
  FAR struct bcmf_dev_s *priv = g_sdio_priv;
  FAR struct bcmf_sdio_dev_s *sbus = (FAR struct bcmf_sdio_dev_s *)priv->bus;
  int ret;

  wlinfo(" Enter\n");

  /*  FIXME wait for the chip to be ready to receive commands */

  up_mdelay(50);

  while (sbus->ready)
    {
      /* Wait for event (device interrupt, user request or waitdog timer) */

      ret = nxsem_wait(&sbus->thread_signal);
      if (ret < 0)
        {
          wlerr("Error while waiting for semaphore\n");
          break;
        }

      /* Restart the waitdog timer */

      wd_start(&sbus->waitdog, BCMF_WAITDOG_TIMEOUT_TICK,
               bcmf_sdio_waitdog_timeout, (wdparm_t)priv);

      /* Wake up device */

      bcmf_sdio_bus_sleep(sbus, false);

      if (sbus->irq_pending)
        {
          /* Woken up by interrupt, read device status */

          sbus->irq_pending = false;

          bcmf_read_sbregw(
            sbus,
            CORE_BUS_REG(sbus->chip->core_base[SDIOD_CORE_ID], intstatus),
            &sbus->intstatus);

          /* Clear interrupts */

          bcmf_write_sbregw(
            sbus,
            CORE_BUS_REG(sbus->chip->core_base[SDIOD_CORE_ID], intstatus),
            sbus->intstatus);
        }

      /* On frame indication, read available frames */

      if (sbus->intstatus & I_HMB_FRAME_IND)
        {
          do
            {
              ret = bcmf_sdpcm_readframe(priv);
            }
          while (ret == OK);

          if (ret == -ENODATA)
            {
              /*  All frames processed */

              sbus->intstatus &= ~I_HMB_FRAME_IND;
            }
        }

      /* Send all queued frames */

      do
        {
          ret = bcmf_sdpcm_sendframe(priv);
        }
      while (ret == OK);

      /* Check if RX frames are available */

      if (sbus->intstatus & I_HMB_FRAME_IND)
        {
          /* Try again */

          wlinfo("Try read again\n");
          continue;
        }

      /* Re-configure the board GPIO interrupt pin */

      bcmf_board_setup_oob_irq(sbus->minor, bcmf_oob_irq, (void *)sbus);

      /* If we're done for now, turn off clock request. */

#if 0
      /* TODO add wakelock */

      bcmf_sdio_bus_sleep(sbus, true);
#endif
    }

  wlinfo("Exit\n");
  return 0;
}

struct bcmf_sdio_frame *bcmf_sdio_allocate_frame(FAR struct bcmf_dev_s *priv,
                                                 bool block, bool tx)
{
  FAR struct bcmf_sdio_dev_s *sbus = (FAR struct bcmf_sdio_dev_s *)priv->bus;
  struct bcmf_sdio_frame *sframe;
  dq_entry_t *entry = NULL;

  while (1)
    {
      if (nxsem_wait(&sbus->queue_mutex) < 0)
        {
          DEBUGPANIC();
        }

#if 0
      if (!tx || sbus->tx_queue_count < BCMF_PKT_POOL_SIZE - 1)
#endif
        {
          if ((entry = bcmf_dqueue_pop_tail(&sbus->free_queue)) != NULL)
            {
              if (tx)
                {
                  sbus->tx_queue_count += 1;
                }

              nxsem_post(&sbus->queue_mutex);
              break;
            }
        }

      nxsem_post(&sbus->queue_mutex);

      if (block)
        {
          /* TODO use signaling semaphore */

          wlinfo("alloc failed %d\n", tx);
          up_mdelay(100);
          continue;
        }

      wlinfo("No avail buffer\n");
      return NULL;
    }

  sframe = container_of(entry, struct bcmf_sdio_frame, list_entry);

  sframe->header.len  = HEADER_SIZE + MAX_NETDEV_PKTSIZE;
  sframe->header.base = sframe->data;
  sframe->header.data = sframe->data;
  sframe->tx          = tx;
  return sframe;
}

void bcmf_sdio_free_frame(FAR struct bcmf_dev_s *priv,
                          struct bcmf_sdio_frame *sframe)
{
  FAR struct bcmf_sdio_dev_s *sbus = (FAR struct bcmf_sdio_dev_s *)priv->bus;

  if (nxsem_wait(&sbus->queue_mutex) < 0)
    {
      DEBUGPANIC();
    }

  bcmf_dqueue_push(&sbus->free_queue, &sframe->list_entry);

  if (sframe->tx)
    {
      sbus->tx_queue_count -= 1;
    }

  nxsem_post(&sbus->queue_mutex);
}

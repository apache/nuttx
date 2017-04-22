/****************************************************************************
 * drivers/wireless/ieee80211/bcmf_sdio.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <nuttx/kmalloc.h>
#include <nuttx/sdio.h>
#include <nuttx/arch.h>
#include <nuttx/kthread.h>

#include <nuttx/wireless/ieee80211/mmc_sdio.h>
#include <nuttx/wireless/ieee80211/bcmf_sdio.h>
#include <nuttx/wireless/ieee80211/bcmf_board.h>

#include "bcmf_sdio.h"
#include "bcmf_core.h"
#include "bcmf_sdpcm.h"

// TODO remove
#include "bcmf_ioctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BCMF_DEVICE_RESET_DELAY_MS 10
#define BCMF_DEVICE_START_DELAY_MS 10
#define BCMF_CLOCK_SETUP_DELAY_MS  500

#define BCMF_THREAD_NAME       "bcmf"
#define BCMF_THREAD_STACK_SIZE 2048

#define BCMF_WAITDOG_TIMEOUT_TICK (5*CLOCKS_PER_SEC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  bcmf_probe(FAR struct bcmf_dev_s *priv);
static int  bcmf_hwinitialize(FAR struct bcmf_dev_s *priv);
static void bcmf_hwuninitialize(FAR struct bcmf_dev_s *priv);
static int  bcmf_chipinitialize(FAR struct bcmf_dev_s *priv);

static int  bcmf_oob_irq(int irq, FAR void *context, FAR void *arg);

static int  bcmf_sdio_bus_sleep(FAR struct bcmf_dev_s *priv, bool sleep);

static void bcmf_sdio_waitdog_timeout(int argc, wdparm_t arg1, ...);
static int  bcmf_sdio_thread(int argc, char **argv);

static int  bcmf_sdio_find_block_size(unsigned int size);

/* FIXME remove */
FAR struct bcmf_dev_s *g_sdio_priv;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int bcmf_oob_irq(int irq, FAR void *context, FAR void *arg)
{
  FAR struct bcmf_dev_s *priv = (struct bcmf_dev_s*)arg;

  if (priv->ready)
    {
      /*  Signal bmcf thread */
      priv->irq_pending = true;

      sem_post(&priv->thread_signal);
    }
  return OK;
}

int bcmf_sdio_bus_sleep(FAR struct bcmf_dev_s *priv, bool sleep)
{
  int ret;
  int loops;
  uint8_t value;

  if (priv->sleeping == sleep)
    {
      return OK;
    }

  if (sleep)
    {
      priv->sleeping = true;
      return bcmf_write_reg(priv, 1, SBSDIO_FUNC1_CHIPCLKCSR, 0);
    }
  else
    {
      /* Request HT Avail */
  
      ret = bcmf_write_reg(priv, 1, SBSDIO_FUNC1_CHIPCLKCSR,
                           SBSDIO_HT_AVAIL_REQ | SBSDIO_FORCE_HT);
      if (ret != OK)
        {
          _err("HT Avail request failed %d\n", ret);
          return ret;
        }
  
      /* Wait for High Troughput clock */
  
      loops = 20;
      while (--loops > 0)
        {
          up_mdelay(1);
          ret = bcmf_read_reg(priv, 1, SBSDIO_FUNC1_CHIPCLKCSR, &value);
  
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
          _err("HT clock not ready\n");
          return -ETIMEDOUT;
        }

      priv->sleeping = false;
    }
  
  return OK;
}

/****************************************************************************
 * Name: bcmf_probe
 ****************************************************************************/

int bcmf_probe(FAR struct bcmf_dev_s *priv)
{
  int ret;

  /* Probe sdio card compatible device */

  ret = sdio_probe(priv->sdio_dev);
  if (ret != OK)
    {
      goto exit_error;
    }

  /* Set FN0 / FN1 / FN2 default block size */

  ret = sdio_set_blocksize(priv->sdio_dev, 0, 64);
  if (ret != OK)
    {
      goto exit_error;
    }

  ret = sdio_set_blocksize(priv->sdio_dev, 1, 64);
  if (ret != OK)
    {
      goto exit_error;
    }

  ret = sdio_set_blocksize(priv->sdio_dev, 2, 64);
  if (ret != OK)
    {
      goto exit_error;
    }

  /* Enable device interrupts for FN0, FN1 and FN2 */

  ret = bcmf_write_reg(priv, 0, SDIO_CCCR_INTEN,
                       (1 << 0) | (1 << 1) | (1 << 2));
  if (ret != OK)
    {
      goto exit_error;
    }

  /* Default device clock speed is up to 25 Mhz
   * We could set EHS bit to operate at a clock rate up to 50 Mhz */

  SDIO_CLOCK(priv->sdio_dev, CLOCK_SD_TRANSFER_4BIT);
  up_mdelay(BCMF_CLOCK_SETUP_DELAY_MS);

  /* Enable bus FN1 */

  ret = sdio_enable_function(priv->sdio_dev, 1);
  if (ret != OK)
    {
      goto exit_error;
    }

  return OK;

exit_error:

  _err("ERROR: failed to probe device %d\n", priv->minor);
  return ret;
}

/****************************************************************************
 * Name: bcmf_businitialize
 ****************************************************************************/

int bcmf_businitialize(FAR struct bcmf_dev_s *priv)
{
  int ret;
  int loops;
  uint8_t value;

  /* Send Active Low-Power clock request */

  ret = bcmf_write_reg(priv, 1, SBSDIO_FUNC1_CHIPCLKCSR,
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
      ret = bcmf_read_reg(priv, 1, SBSDIO_FUNC1_CHIPCLKCSR, &value);

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
      _err("failed to enable ALP\n");
      return -ETIMEDOUT;
    }

  /* Clear Active Low-Power clock request */

  ret = bcmf_write_reg(priv, 1, SBSDIO_FUNC1_CHIPCLKCSR, 0);
  if (ret != OK)
    {
      return ret;
    }

  /* Disable pull-ups on SDIO cmd, d0-2 lines */

  ret = bcmf_write_reg(priv, 1, SBSDIO_FUNC1_SDIOPULLUP, 0);
  if (ret != OK)
    {
      return ret;
    }

  /* Do chip specific initialization */

  ret = bcmf_chipinitialize(priv);
  if (ret != OK)
    {
      return ret;
    }

  /* Upload firmware */

  ret = bcmf_core_upload_firmware(priv);
  if (ret != OK)
    {
      return ret;
    }

  /* Enable FN2 (frame transfers) */

  ret = sdio_enable_function(priv->sdio_dev, 2);
  if (ret != OK)
    {
      return ret;
    }

  return OK;
}

int bcmf_bus_setup_interrupts(FAR struct bcmf_dev_s *priv)
{
  int ret;

  /* Configure gpio interrupt pin */

  bcmf_board_setup_oob_irq(priv->minor, bcmf_oob_irq, (void*)priv);

  /* Enable function 2 interrupt */

  ret = sdio_enable_interrupt(priv->sdio_dev, 0);
  if (ret != OK)
    {
      return ret;
    }
  ret = sdio_enable_interrupt(priv->sdio_dev, 2);
  if (ret != OK)
    {
      return ret;
    }

  /* Redirect, configure and enable io for out-of-band interrupt signal */

  ret = bcmf_write_reg(priv, 0, SDIO_CCCR_BRCM_SEPINT,
                       SDIO_SEPINT_MASK | SDIO_SEPINT_OE | SDIO_SEPINT_ACT_HI);
  if (ret != OK)
    {
      return ret;
    }

  /* Wake up chip to be sure function 2 is running */

  ret = bcmf_sdio_bus_sleep(priv, false);
  if (ret != OK)
    {
      return ret;
    }

  /* FN2 successfully enabled, set core and enable interrupts */

  bcmf_write_sbregw(priv,
                   CORE_BUS_REG(priv->get_core_base_address(SDIOD_CORE_ID),
                   hostintmask), I_HMB_SW_MASK);

  bcmf_write_sbregb(priv,
                   CORE_BUS_REG(priv->get_core_base_address(SDIOD_CORE_ID),
                   funcintmask), 2);

  bcmf_write_reg(priv, 1, SBSDIO_WATERMARK, 8);

  return OK;
}

/****************************************************************************
 * Name: bcmf_hwinitialize
 ****************************************************************************/

int bcmf_hwinitialize(FAR struct bcmf_dev_s *priv)
{
  /* Attach and prepare SDIO interrupts */

  SDIO_ATTACH(priv->sdio_dev);

  /* Set ID mode clocking (<400KHz) */

  SDIO_CLOCK(priv->sdio_dev, CLOCK_IDMODE);

  /* Configure hardware */

  bcmf_board_initialize(priv->minor);

  /* Reset and power device */

  bcmf_board_reset(priv->minor, true);
  bcmf_board_power(priv->minor, true);
  up_mdelay(BCMF_DEVICE_RESET_DELAY_MS);
  bcmf_board_reset(priv->minor, false);

  /* Wait for device to start */

  up_mdelay(BCMF_DEVICE_START_DELAY_MS);

  return OK;
}

/****************************************************************************
 * Name: bcmf_hwuninitialize
 ****************************************************************************/

void bcmf_hwuninitialize(FAR struct bcmf_dev_s *priv)
{
  /*  Shutdown device */

  bcmf_board_power(priv->minor, false);
  bcmf_board_reset(priv->minor, true);
}

int bcmf_sdio_find_block_size(unsigned int size)
{
  int ret = 0;
  int size_copy = size;
  while (size_copy) {
    size_copy >>= 1;
    ret++;
  }

  if (size & (size-1))
    {
      return 1<<ret;
    }
  return 1<<(ret-1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_transfer_bytes
 ****************************************************************************/

int bcmf_transfer_bytes(FAR struct bcmf_dev_s *priv, bool write,
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
          return sdio_io_rw_direct(priv->sdio_dev, write,
                                   function, address, *buf, NULL);
        }
      return sdio_io_rw_direct(priv->sdio_dev, write,
                               function, address, 0, buf);
    }

    /* Find best block settings for transfer */

    if (len >= 64)
      {
        /* Use block mode */

        blocklen = 64;
        nblocks = (len+63) / 64;
      }
    else
      {
        /* Use byte mode */

        blocklen = bcmf_sdio_find_block_size(len);
        nblocks = 0;
      }

    return sdio_io_rw_extended(priv->sdio_dev, write,
                               function, address, true, buf, blocklen, nblocks);
}

/****************************************************************************
 * Name: bcmf_read_reg
 ****************************************************************************/

int bcmf_read_reg(FAR struct bcmf_dev_s *priv, uint8_t function,
                  uint32_t address, uint8_t *reg)
{
  *reg = 0;
  return bcmf_transfer_bytes(priv, false, function, address, reg, 1);
}

/****************************************************************************
 * Name: bcmf_write_reg
 ****************************************************************************/

int bcmf_write_reg(FAR struct bcmf_dev_s *priv, uint8_t function,
                   uint32_t address, uint8_t reg)
{
  return bcmf_transfer_bytes(priv, true, function, address, &reg, 1);
}

/****************************************************************************
 * Name: bcmf_sem_wait
 ****************************************************************************/

int bcmf_sem_wait(sem_t *sem, unsigned int timeout_ms)
{
  struct timespec abstime;

  /* Get the current time */

  (void)clock_gettime(CLOCK_REALTIME, &abstime);

  abstime.tv_nsec += 1000 * 1000* timeout_ms;

  if (abstime.tv_nsec >= 1000 * 1000 * 1000)
    {
      abstime.tv_sec++;
      abstime.tv_nsec -= 1000 * 1000 * 1000;
    }

  return sem_timedwait(sem, &abstime);
}

/****************************************************************************
 * Name: bcmf_sdio_initialize
 ****************************************************************************/

int bcmf_sdio_initialize(int minor, FAR struct sdio_dev_s *dev)
{
  FAR struct bcmf_dev_s *priv;
  int ret;

  _info("minor: %d\n", minor);

  /* Allocate a bcmf device structure */

  priv = (FAR struct bcmf_dev_s *)kmm_malloc(sizeof(*priv));

  if (!priv)
    {
      return -ENOMEM;
    }

  /* Initialize bcmf device structure */

  memset(priv, 0, sizeof(*priv));
  priv->sdio_dev = dev;
  priv->minor = minor;
  priv->ready = false;
  priv->sleeping = true;
  
  if ((ret = sem_init(&priv->thread_signal, 0, 0)) != OK)
    {
      goto exit_free_priv;
    }
  if ((ret = sem_setprotocol(&priv->thread_signal, SEM_PRIO_NONE)) != OK)
    {
      goto exit_free_priv;
    }

  priv->waitdog = wd_create();
  if (!priv->waitdog)
    {
      ret = -ENOMEM;
      goto exit_free_priv;
    }

  if ((ret = sem_init(&priv->control_mutex, 0, 1)) != OK)
    {
      goto exit_free_waitdog;
    }

  if ((ret = sem_init(&priv->control_timeout, 0, 0)) != OK)
    {
      goto exit_free_waitdog;
    }
  if ((ret = sem_setprotocol(&priv->control_timeout, SEM_PRIO_NONE)) != OK)
    {
      goto exit_free_waitdog;
    }
  /* Init transmit frames queue */

  if ((ret = sem_init(&priv->tx_queue_mutex, 0, 1)) != OK)
    {
      goto exit_free_waitdog;
    }
  sq_init(&priv->tx_queue);

  /* Initialize device hardware */

  ret = bcmf_hwinitialize(priv);

  if (ret != OK)
    {
      goto exit_free_waitdog;
    }

  /* Probe device */

  ret = bcmf_probe(priv);

  if (ret != OK)
    {
      goto exit_uninit_hw;
    }

  /* Initialize device bus */

  ret = bcmf_businitialize(priv);

  if (ret != OK)
    {
      goto exit_uninit_hw;
    }


  up_mdelay(100);

  priv->ready = true;

  ret = bcmf_bus_setup_interrupts(priv);
  if (ret != OK)
    {
      goto exit_uninit_hw;
    }

  /* FIXME global variable for now */
  g_sdio_priv = priv;

  /* Start the waitdog timer */

  wd_start(priv->waitdog, BCMF_WAITDOG_TIMEOUT_TICK, bcmf_sdio_waitdog_timeout, 0);

  /* Spawn bcmf daemon thread */

  ret = kernel_thread(BCMF_THREAD_NAME, SCHED_PRIORITY_MAX,
                      BCMF_THREAD_STACK_SIZE, bcmf_sdio_thread,
                      (FAR char * const *)NULL);

  if (ret <= 0)
    {
      _err("Cannot spawn bcmf thread\n");
      ret = -EBADE;
      goto exit_uninit_hw;
    }

  /* sdio bus is ready, init driver */

  ret = bcmf_wl_initialize(priv);
  if (ret != OK)
    {
      _err("Cannot init wlan driver %d\n", ret);
      ret = -EIO;
      goto exit_uninit_hw;
    }

  return OK;

exit_uninit_hw:
  bcmf_hwuninitialize(priv);
exit_free_waitdog:
  // TODO
exit_free_priv:
  kmm_free(priv);
  priv->ready = false;
  return ret;
}

int bcmf_chipinitialize(FAR struct bcmf_dev_s *priv)
{
  int ret;
  uint32_t value = 0;

  ret = bcmf_read_sbregw(priv, SI_ENUM_BASE, &value);
  if (ret != OK)
    {
      return ret;
    }
  _info("chip id is 0x%x\n", value);

  int chipid = value & 0xffff;
  switch (chipid)
    {
      case SDIO_DEVICE_ID_BROADCOM_43362:
        _info("bcm43362 chip detected\n");
        priv->get_core_base_address = bcmf_43362_get_core_base_address;
        break;
      default:
        _err("chip 0x%x is not supported\n", chipid);
        return -ENODEV;
   }
  return OK;
}

void bcmf_sdio_waitdog_timeout(int argc, wdparm_t arg1, ...)
{
  FAR struct bcmf_dev_s *priv = g_sdio_priv;

  /* Notify bcmf thread */

  sem_post(&priv->thread_signal);
}

int bcmf_sdio_thread(int argc, char **argv)
{
  int ret;
  FAR struct bcmf_dev_s *priv = g_sdio_priv;
  
  _info("Enter\n");

  /*  FIXME wait for the chip to be ready to receive commands */

  up_mdelay(50);
  
  while (1)
    {
      /* Wait for event (device interrupt, user request or waitdog timer) */

      ret = sem_wait(&priv->thread_signal);
      if (ret != OK)
        {
          _err("Error while waiting for semaphore\n");
          break;
        }

      /* Restart the waitdog timer */

      wd_start(priv->waitdog, BCMF_WAITDOG_TIMEOUT_TICK,
               bcmf_sdio_waitdog_timeout, 0);

      /* Wake up device */

      bcmf_sdio_bus_sleep(priv, false);

      if (priv->irq_pending)
        {
          /* Woken up by interrupt, read device status */

          priv->irq_pending = false;

          bcmf_read_sbregw(priv,
                       CORE_BUS_REG(priv->get_core_base_address(SDIOD_CORE_ID),
                       intstatus), &priv->intstatus);

          /* Clear interrupts */

          bcmf_write_sbregw(priv,
                       CORE_BUS_REG(priv->get_core_base_address(SDIOD_CORE_ID),
                       intstatus), priv->intstatus);
          // _info("intstatus %x\n", priv->intstatus);
        }

      /* On frame indication, read available frames */

      if (priv->intstatus & I_HMB_FRAME_IND)
        {
          // _info("Frames available\n");

          do
            {
              ret = bcmf_sdpcm_readframe(priv);
            } while (ret == OK);
          
          if (ret == -ENODATA)
            {
              /*  All frames processed */

              priv->intstatus &= ~I_HMB_FRAME_IND;
            }
        }

      /* Send all queued frames */

      do
        {
          ret = bcmf_sdpcm_sendframe(priv);
        } while (ret == OK);

      /* If we're done for now, turn off clock request. */

      // TODO add wakelock
      // bcmf_sdio_bus_sleep(priv, true);
    }
    return 0;
}
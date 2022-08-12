/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_gspi.c
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
#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/kthread.h>
#include <nuttx/wdog.h>
#include <nuttx/sdio.h>
#include <nuttx/signal.h>

#include <nuttx/wireless/ieee80211/bcmf_sdio.h>
#include <nuttx/wireless/ieee80211/bcmf_board.h>

#include "bcmf_gspi.h"
#include "bcmf_gspi_f2_frame.h"
#include "bcmf_core.h"
#include "bcmf_sdpcm.h"
#include "bcmf_utils.h"

#include "bcmf_sdio_core.h"
#include "bcmf_sdio_regs.h"
#include "cyw_reg_def.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BCMF_GSPI_READY_TRYS            10
#define BCMF_GSPI_THREAD_NAME           "bcmf-gspi-thread"
#define BCMF_GSPI_THREAD_STACK_SIZE     2048
#define BCMF_GSPI_LOWPOWER_TIMEOUT_TICK SEC2TICK(2)

#ifdef CONFIG_IEEE80211_INFINEON_CYW43439
  extern const struct bcmf_sdio_chip cyw43439_config_sdio;
#endif

#ifdef CONFIG_ARCH_CHIP_RP2040
#  define REV16(x) __asm ("rev16 %0, %0" : "+l" (x) : :)
#else
#  define REV16(x) (((x & 0x000000ff) << 8)   \
                  | ((x & 0x0000ff00) >> 8)   \
                  | ((x & 0x00ff0000) << 8)   \
                  | ((x & 0xff000000) >> 8))
#endif

/* Chip-common registers */

#define CHIPCOMMON_GPIO_CONTROL ((uint32_t)(0x18000000 + 0x06c) )
#define CHIPCOMMON_SR_CONTROL0  ((uint32_t)(0x18000000 + 0x504) )
#define CHIPCOMMON_SR_CONTROL1  ((uint32_t)(0x18000000 + 0x508) )

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_gspi_read_reg_32
 *
 * Description:
 *   Read a 32-bit register
 *
 ****************************************************************************/

static inline uint32_t bcmf_gspi_read_reg_32(FAR gspi_dev_t       *gspi,
                                             enum gspi_cmd_func_e  function,
                                             uint32_t              address)
{
  uint32_t buffer;

  gspi->read(gspi, true, function, address, 4, &buffer);

  return buffer;
}

/****************************************************************************
 * Name: bcmf_gspi_read_reg_16
 *
 * Description:
 *   Read a 32-bit register
 *
 ****************************************************************************/

static inline uint16_t bcmf_gspi_read_reg_16(FAR gspi_dev_t       *gspi,
                                             enum gspi_cmd_func_e  function,
                                             uint32_t              address)
{
  uint32_t buffer;

  gspi->read(gspi, true, function, address, 2, &buffer);

  return buffer;
}

/****************************************************************************
 * Name: bcmf_gspi_read_reg_8
 *
 * Description:
 *   Read a 32-bit register
 *
 ****************************************************************************/

static inline uint8_t bcmf_gspi_read_reg_8(FAR gspi_dev_t       *gspi,
                                           enum gspi_cmd_func_e  function,
                                           uint32_t              address)
{
  uint32_t buffer;

  gspi->read(gspi, true, function, address, 1, &buffer);

  return buffer;
}

/****************************************************************************
 * Name: bcmf_gspi_write_reg_32
 *
 * Description:
 *   Write a 32-bit register
 *
 ****************************************************************************/

static inline void bcmf_gspi_write_reg_32(FAR gspi_dev_t       *gspi,
                                          enum gspi_cmd_func_e  function,
                                          uint32_t              address,
                                          uint32_t              value)
{
  gspi->write(gspi, true, function, address, 4, &value);
}

/****************************************************************************
 * Name: bcmf_gspi_write_reg_16
 *
 * Description:
 *   Read a 32-bit register
 *
 ****************************************************************************/

static inline void bcmf_gspi_write_reg_16(FAR gspi_dev_t       *gspi,
                                          enum gspi_cmd_func_e  function,
                                          uint32_t              address,
                                          uint32_t              value)
{
  gspi->write(gspi, true, function, address, 2, &value);
}

/****************************************************************************
 * Name: bcmf_gspi_write_reg_8
 *
 * Description:
 *   Write a 8-bit register
 *
 ****************************************************************************/

static inline void bcmf_gspi_write_reg_8(FAR gspi_dev_t       *gspi,
                                         enum gspi_cmd_func_e  function,
                                         uint32_t              address,
                                         uint32_t              value)
{
  gspi->write(gspi, true, function, address, 1, &value);
}

/****************************************************************************
 * Name: bcmf_gspi_kso_enable
 ****************************************************************************/

static int bcmf_gspi_kso_enable(FAR bcmf_gspi_dev_t *gbus, bool enable)
{
  FAR gspi_dev_t *gspi = gbus->gspi;
  uint8_t         value;
  int             loops;

  if (!gbus->ready)
    {
      return -EPERM;
    }

  if (gbus->kso_enable == enable)
    {
      return OK;
    }

  if (enable)
    {
      wlinfo("enable\n");

      loops = 200;
      while (--loops > 0)
        {
          bcmf_gspi_write_reg_8(gspi,
                                gspi_f1_backplane,
                                SBSDIO_FUNC1_SLEEPCSR,
                                  SBSDIO_FUNC1_SLEEPCSR_KSO_MASK
                                | SBSDIO_FUNC1_SLEEPCSR_DEVON_MASK);

          nxsig_usleep(100 * 1000);

          value = bcmf_gspi_read_reg_8(gspi, 1, SBSDIO_FUNC1_SLEEPCSR);

          if ((value & (SBSDIO_FUNC1_SLEEPCSR_KSO_MASK |
                        SBSDIO_FUNC1_SLEEPCSR_DEVON_MASK)) != 0)
            {
              break;
            }
        }

      if (loops <= 0)
        {
          return -ETIMEDOUT;
        }
    }
  else
    {
      wlinfo("disable\n");

      bcmf_gspi_write_reg_8(gspi, 1, SBSDIO_FUNC1_SLEEPCSR, 0);
    }

  gbus->kso_enable = enable;

  return OK;
}

/****************************************************************************
 * Name: bcmf_gspi_bus_sleep
 ****************************************************************************/

static int bcmf_gspi_bus_sleep(FAR bcmf_gspi_dev_t *gbus, bool sleep)
{
  FAR gspi_dev_t *gspi = gbus->gspi;
  uint8_t         value;
  int             loops;

  if (!gbus->ready)
    {
      return -EPERM;
    }

  if (gbus->sleeping == sleep)
    {
      return OK;
    }

  if (sleep)
    {
      wlinfo("enable\n");

      gbus->sleeping = true;
      bcmf_gspi_write_reg_8(gspi,
                            gspi_f1_backplane,
                            SBSDIO_FUNC1_CHIPCLKCSR,
                            0);

      wlinfo("exit\n");

      return OK;
    }
  else
    {
      wlinfo("disable\n");

      loops = 200;
      while (--loops > 0)
        {
          /* Request HT Avail */

          bcmf_gspi_write_reg_8(gspi,
                                gspi_f1_backplane,
                                SBSDIO_FUNC1_CHIPCLKCSR,
                                  SBSDIO_HT_AVAIL_REQ
                                | SBSDIO_FORCE_HT);

          /* Wait for High Throughput clock */

          nxsig_usleep(100 * 1000);
          value = bcmf_gspi_read_reg_8(gspi, 1, SBSDIO_FUNC1_CHIPCLKCSR);

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

      gbus->sleeping = false;
    }

  return OK;
}

/****************************************************************************
 * Name: bcmf_gspi_bus_lowpower
 ****************************************************************************/

static int bcmf_gspi_bus_lowpower(FAR bcmf_gspi_dev_t *gbus, bool enable)
{
  return gbus->support_sr ? bcmf_gspi_kso_enable(gbus, !enable)
                          : bcmf_gspi_bus_sleep(gbus, enable);
}

/****************************************************************************
 * Name: bcmf_gspi_thread_isr
 ****************************************************************************/

static int bcmf_gspi_thread_isr(int isr, void *context, void *arg)
{
  FAR bcmf_gspi_dev_t *gbus = (FAR bcmf_gspi_dev_t  *) arg;
  FAR gspi_dev_t      *gspi = gbus->gspi;
  int                 semcount;

  gbus->irq_pending = true;

  nxsem_get_value(&gbus->thread_signal, &semcount);

  if (semcount < 1)
    {
      nxsem_post(&gbus->thread_signal);
    }

  /* Disable interrupt until bcmf_gspi_thread runs */

  gspi->interrupt_enable(gspi, false);

  return OK;
}

/****************************************************************************
 * Name: bcmf_gspi_thread
 ****************************************************************************/

static int bcmf_gspi_thread(int argc, char **argv)
{
  FAR struct bcmf_dev_s *priv;
  FAR bcmf_gspi_dev_t   *gbus;
  FAR gspi_dev_t        *gspi;
  uint32_t               status;
  uint16_t               intr_flags;
  int                    ret;
  int                    length;
  int                    wait_count      = 0;
  bool                   wait_for_event;
  bool                   enter_low_power = false;

  priv = (FAR struct bcmf_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  gbus = (FAR bcmf_gspi_dev_t *)priv->bus;
  gspi = gbus->gspi;

  wlinfo(">>>> entered\n");

  nxsig_usleep(50 * 1000);

  gbus->thread_run = true;

  bcmf_gspi_bus_lowpower(gbus, false);

  while (gbus->thread_run)
    {
      /* Preset the wait for event flag in case we have nothing to do */

      wait_for_event = true;

      /* Get the device status */

      status = bcmf_gspi_read_reg_32(gspi, gspi_f0_bus, CYW_REG_STATUS);

#ifdef CONFIG_DEBUG_WIRELESS_ERROR
      if (status & (1 << 2))
        {
          wlerr("CYW_REG_STATUS_FIFO_OVERFLOW\n");
        }

      if (status & (1 << 7))
        {
          wlerr("CYW_REG_STATUS_CMD_DATA_ERROR\n");
        }
#endif

      /* wlinfo(">>>> Status = 0x%08lX\n", status); */

      /* If we were woken by an device interrupt clear the interrupt bits. */

      if (gbus->irq_pending)
        {
          gbus->irq_pending = false;

          /* These call also update the status in gspi->status */

          intr_flags = bcmf_gspi_read_reg_16(gspi,
                                             gspi_f0_bus,
                                             CYW_REG_INTERRUPT);

          if (intr_flags != 0)
            {
              bcmf_gspi_write_reg_16(gspi,
                                    gspi_f0_bus,
                                    CYW_REG_INTERRUPT,
                                    intr_flags);
            }
        }

      /* If we have a packet available to read -- read it */

      if (status & CYW_REG_STATUS_F2_PKT_AVAIL)
        {
          length   = status & CYW_REG_STATUS_F2_PKT_LEN_MASK;
          length >>= CYW_REG_STATUS_F2_PKT_LEN_SHIFT;

          /* wlinfo(">>>> Packet available len: %d\n", length); */

          /* If we don't have a frame leave the loop */

          if (length == 0) break;

          /* Read and process frame. This updates gspi->status */

          ret = bcmf_gspi_read_f2_frame(priv, length);

          if (ret == OK)
            {
              wait_for_event = false;
            }
          else
            {
              wlerr("error reading f2 frame: %d\n", ret);
            }
        }
      else
        {
          /* If we don't have anything to read, try sending a packet */

          while ((status & CYW_REG_STATUS_F2_RECEIVE_RDY) == 0)
            {
              /* Oops! no room for a packet.  We'll wait a bit to see
               * if room shows up.
               */

              wlinfo(">>>> not ready to receive\n");

              if (++wait_count > 100)
                {
                  wlerr("Chip cannot receive F2 frame\n");
                  break;
                }

              /* No room at the inn for an f2 frame -- wait a bit */

              usleep(10000);

              status = bcmf_gspi_read_reg_32(gspi,
                                             gspi_f0_bus,
                                             CYW_REG_STATUS);
            }

          /* reset the count for next time  */

          wait_count = 0;

          /* We have space, send the frame */

          ret = bcmf_gspi_send_f2_frame(priv);

          if (ret == OK)
            {
              /* wlinfo(">>>> frame sent\n"); */

              wait_for_event = false;
            }
          else
            {
#ifdef CONFIG_DEBUG_WIRELESS_ERROR
              if (ret != -ENODATA)
                {
                  wlerr("error sending f2 frame: %d\n", ret);
                }
#endif
            }
        }

      /* No more transfer requests.  Wait for something to happen. */

      if (wait_for_event)
        {
          /* Wait for event (device interrupt or user request) */

          gspi->interrupt_enable(gspi, true);

          if (enter_low_power)
            {
              enter_low_power = false;

              wlinfo(">>>> sleep until interrupt\n");

              bcmf_gspi_bus_lowpower(gbus, true);
              nxsem_wait_uninterruptible(&gbus->thread_signal);
              bcmf_gspi_bus_lowpower(gbus, false);
            }
          else
            {
              /* wlinfo(">>>> waiting for interrupt\n"); */

              ret = nxsem_tickwait_uninterruptible(
                            &gbus->thread_signal,
                            BCMF_GSPI_LOWPOWER_TIMEOUT_TICK);

              if (ret == -ETIMEDOUT) enter_low_power = true;
            }
        }
    }

  wlinfo(">>>> exit\n");

  return 0;
}

/****************************************************************************
 * Name: bcmf_gspi_init_device
 *
 * Description:
 *   Keeps checking the 43439's test register looking for the test pattern.
 *   When found puts the chip in 32-bit mode.
 *
 * Note:
 *   Some calls to the 43439 in this function uses the "magic" rev16 mode
 *   to account for the 16-bit mode's default byte ordering.
 ****************************************************************************/

static int bcmf_gspi_init_device(FAR bcmf_gspi_dev_t *gbus)
{
  FAR gspi_dev_t *gspi = gbus->gspi;
  uint32_t        buffer[2];
  uint32_t        data;
  int             ret;
  int             i;

  wlinfo("entered.\n");

  /* Look for the chip ready pattern. */

  for (i = 0; i < BCMF_GSPI_READY_TRYS; ++i)
    {
      ret = gspi->read(gspi,
                       true,
                       gspi_f0_bus_rev16,
                       CYW_REG_TEST_RO,
                       4,
                       buffer);

      if (ret != 0)
        {
          wlerr("Error looking for \"ready\" pattern: %d\n", ret);
          return ret;
        }

      REV16(buffer[0]);

      if (buffer[0] == CYW_REG_TEST_RO_PATTERN) break;
    }

  if (i == BCMF_GSPI_READY_TRYS)
    {
      wlerr("Could not find cyw43439 \"ready\" pattern\n");
      return -ENODEV;
    }

  data =   CYW_REG_SETUP_WORD_LEN_32
         | CYW_REG_SETUP_BIG_ENDIAN
         | CYW_REG_SETUP_HIGH_SPEED
         | CYW_REG_SETUP_INT_POLARITY
         | CYW_REG_SETUP_WAKE_UP
         | CYW_REG_STAT_ENA_INTR_STAT;

  REV16(data);

  gspi->write(gspi, true, gspi_f0_bus_rev16, CYW_REG_SETUP, 4, &data);

  /* We are now in 32-bit bigendian mode -- we no longer do REV16s. */

  /* Set a 4-byte response delay for F1 packets */

  bcmf_gspi_write_reg_8(gspi, gspi_f0_bus, CYW_REG_RESP_DELAY_F1, 4);

  wlinfo("complete\n");

  return OK;
}

/****************************************************************************
 * Name: bcmf_gspi_probe_chip
 ****************************************************************************/

static int bcmf_gspi_probe_chip(FAR bcmf_gspi_dev_t *gbus)
{
  uint32_t        value;
  int             chipid;
  int             ret;

  wlinfo("entered\n");

  ret = bcmf_read_sbregw(gbus, SI_ENUM_BASE, &value);
  if (ret != OK)
    {
      wlerr("bcmf_read_sbregw failed\n");
      return ret;
    }

  chipid            = value & 0xffff;
  gbus->cur_chip_id = chipid;

  switch (chipid)
    {
#ifdef CONFIG_IEEE80211_INFINEON_CYW43439
      case SDIO_DEVICE_ID_INFINEON_CYW43439:
        wlinfo("cyw%d chip detected\n", chipid);
        gbus->chip = (struct bcmf_sdio_chip *)&cyw43439_config_sdio;
        break;
#endif

      default:
        wlerr("chip 0x%08X is not supported\n", chipid);
        return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: bcmf_gspi_setup_interrupts
 ****************************************************************************/

static int bcmf_gspi_setup_interrupts(FAR bcmf_gspi_dev_t *gbus)
{
  FAR gspi_dev_t *gspi = gbus->gspi;

  wlinfo("entered\n");

  /* Set up device interrupt preferences */

  bcmf_gspi_write_reg_16(gspi,
                         gspi_f0_bus,
                         CYW_REG_INTERRUPT,
                         CYW_REG_INTERRUPT_DATA_NOT_AVAIL
                       | CYW_REG_INTERRUPT_COMMAND_ERROR
                       | CYW_REG_INTERRUPT_DATA_ERROR
                       | CYW_REG_INTERRUPT_F1_OVERFLOW);

  /* We only want an interrupt if an F2 packet is available */

  bcmf_gspi_write_reg_16(gspi,
                         gspi_f0_bus,
                         CYW_REG_INTR_ENA,
                         CYW_REG_INTR_ENA_F2_PKT_AVAIL);

  wlinfo("interrupt set up complete\n");

  return OK;
}

/****************************************************************************
 * Name: bcmf_gspi_init_alp_clock
 ****************************************************************************/

static int bcmf_gspi_init_alp_clock(FAR bcmf_gspi_dev_t *gbus)
{
  FAR gspi_dev_t *gspi = gbus->gspi;
  int             loops;
  uint8_t         value;

  wlinfo("entered\n");

  /* Send Active Low-Power clock request */

  bcmf_gspi_write_reg_8(gspi,
                        gspi_f1_backplane,
                        SBSDIO_FUNC1_CHIPCLKCSR,
                          SBSDIO_FORCE_HW_CLKREQ_OFF
                        | SBSDIO_ALP_AVAIL_REQ
                        | SBSDIO_FORCE_ALP);

  loops = 10;
  while (--loops > 0)
    {
      nxsig_usleep(10 * 1000);

      value = bcmf_gspi_read_reg_8(gspi,
                                   gspi_f1_backplane,
                                   SBSDIO_FUNC1_CHIPCLKCSR);

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

  bcmf_gspi_write_reg_8(gspi,
                        gspi_f1_backplane,
                        SBSDIO_FUNC1_CHIPCLKCSR,
                        0);

  wlinfo("ALP initialization complete\n");

  nxsig_usleep(100 * 1000);

  return OK;
}

/****************************************************************************
 * Name: bcmf_gspi_init_save_restore
 ****************************************************************************/

static int bcmf_gspi_init_save_restore(FAR bcmf_gspi_dev_t *gbus)
{
  uint8_t  data;
  uint32_t srctrl = 0;
  int      ret;

  wlinfo("entered\n");

  ret = bcmf_read_sbregw(gbus, CHIPCOMMON_SR_CONTROL1, &srctrl);
  if (ret != OK)
    {
      wlinfo("exit -- not SR capable.\n");

      return OK;  /* chip not sr capable */
    }

  if (srctrl != 0)
    {
      /* Configure WakeupCtrl register to set HtAvail request bit in
       * chipClockCSR register after the sdiod core is powered on.
       */

      bcmf_read_reg(gbus, 1, SBSDIO_FUNC1_WAKEUPCTRL, &data);
      data |= SBSDIO_FUNC1_WCTRL_HTWAIT_MASK;
      bcmf_write_reg(gbus, 1, SBSDIO_FUNC1_WAKEUPCTRL, data);

      /* Set brcmCardCapability to noCmdDecode mode.
       * It makes sdiod_aos to wakeup host for any activity of cmd line,
       * even though module won't decode cmd or respond
       */

      bcmf_write_reg(gbus, 0, SDIO_CCCR_BRCM_CARDCAP,
                     SDIO_CCCR_BRCM_CARDCAP_CMD_NODEC);
      bcmf_write_reg(gbus, 1, SBSDIO_FUNC1_CHIPCLKCSR,
                     SBSDIO_FORCE_HT);

      /* Enable KeepSdioOn (KSO) bit for normal operation */

      bcmf_gspi_kso_enable(gbus, true);

      gbus->support_sr = true;
    }

  wlinfo("exit\n");

  return OK;
}

/****************************************************************************
 * Name: bcmf_gspi_hw_uninitialize
 ****************************************************************************/

static int bcmf_gspi_hw_uninitialize(FAR bcmf_gspi_dev_t *gbus)
{
  FAR gspi_dev_t *gspi = gbus->gspi;

  wlinfo("entered\n");

  if (gbus->thread_id != 0)
    {
      gbus->thread_run = false;
      nxsem_post(&gbus->thread_signal);
    }

  gspi->deinit(gspi);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_bus_gspi_initialize
 *
 * Description:
 *   Initialize a bcmf device connected via gSPI interface.
 *
 ****************************************************************************/

int bcmf_bus_gspi_initialize(FAR struct bcmf_dev_s  *priv,
                             FAR struct gspi_dev_s  *gspi)
{
  FAR bcmf_gspi_dev_t *gbus;
  int                  ret;

  wlinfo("entered.\n");

  /* Allocate gSPI bus structure */

  gbus = (FAR bcmf_gspi_dev_t *)kmm_zalloc(sizeof(*gbus));

  if (!gspi)
    {
      return -ENOMEM;
    }

  /* Initialize sdio bus device structure */

  gbus->gspi               = gspi;
  gbus->ready              = false;
  gbus->sleeping           = true;

  /* FIX ME -- The interface needs some room to send F2 packets
   *           to the device before we've read the first F2
   *           packet from the device, so we set the max_seq
   *           to something small;
   */

  gbus->max_seq            = 4;

  gbus->bus.txframe        = bcmf_sdpcm_queue_frame;
  gbus->bus.rxframe        = bcmf_sdpcm_get_rx_frame;
  gbus->bus.allocate_frame = bcmf_sdpcm_alloc_frame;
  gbus->bus.free_frame     = bcmf_sdpcm_free_frame;
  gbus->bus.stop           = NULL; /* TODO */

  /* Init transmit frames queue */

  if ((ret = nxsem_init(&gbus->queue_mutex, 0, 1)) != OK)
    {
      goto exit_free_bus;
    }

  list_initialize(&gbus->tx_queue);
  list_initialize(&gbus->rx_queue);

  /* Setup free buffer list */

  bcmf_initialize_interface_frames();

  /* Init thread semaphore */

  if ((ret = nxsem_init(&gbus->thread_signal, 0, 0)) != OK)
    {
      goto exit_free_bus;
    }

  if ((ret = nxsem_set_protocol(&gbus->thread_signal, SEM_PRIO_NONE)) != OK)
    {
      goto exit_free_bus;
    }

  /* Register sdio bus */

  priv->bus = &gbus->bus;

  wlinfo("complete.\n");

  return OK;

exit_free_bus:

  wlinfo("failed.\n");

  kmm_free(gbus);
  priv->bus = NULL;
  return ret;
}

/****************************************************************************
 * Name: bcmf_bus_gspi_active
 *
 * Description:
 *   Activate (or deactivate) a bcmf device connected via gSPI interface.
 ****************************************************************************/

int bcmf_bus_gspi_active(FAR struct bcmf_dev_s *priv,
                         bool                   active)
{
  FAR bcmf_gspi_dev_t  *gbus = (FAR bcmf_gspi_dev_t *)priv->bus;
  FAR gspi_dev_t       *gspi = gbus->gspi;
  int                   ret  = OK;
  FAR char             *argv[2];
  char                  arg1[32];

  wlinfo("entered.  active = %d\n", active);

  if (!active)
    {
      goto exit_uninit_hw;
    }

  /* Initialize device hardware */

  ret = gspi->init(gspi);

  if (ret != OK)
    {
      return ret;
    }

  gbus->ready = active;

  /* Probe device */

  ret = bcmf_gspi_init_device(gbus);
  if (ret != OK)
    {
      goto exit_uninit_hw;
    }

  /* Detect and configure for specific chip */

  ret = bcmf_gspi_probe_chip(gbus);
  if (ret != OK)
    {
      wlerr("bcmf_gspi_probe_chip failed\n");
      goto exit_uninit_hw;
    }

  ret = bcmf_gspi_setup_interrupts(gbus);
  if (ret != OK)
    {
      goto exit_uninit_hw;
    }

  /* Active the low power clock */

  ret = bcmf_gspi_init_alp_clock(gbus);
  if (ret != OK)
    {
      goto exit_uninit_hw;
    }

  /* Upload firmware */

  ret = bcmf_core_upload_firmware(gbus);
  if (ret != OK)
    {
      wlerr("bcmf_core_upload_firmware failed\n");
      goto exit_uninit_hw;
    }

  /* Spawn bcmf daemon thread */

  snprintf(arg1, sizeof(arg1), "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create(BCMF_GSPI_THREAD_NAME,
                       CONFIG_IEEE80211_BROADCOM_SCHED_PRIORITY,
                       BCMF_GSPI_THREAD_STACK_SIZE,
                       bcmf_gspi_thread,
                       argv);
  if (ret <= 0)
    {
      wlerr("Cannot spawn daemon thread\n");
      ret = -EBADE;
      goto exit_uninit_hw;
    }

  gbus->thread_id = (pid_t)ret;

  ret = gspi->set_isr(gspi, bcmf_gspi_thread_isr, gbus);

  if (ret != OK)
    {
      wlerr("set_isr failed\n");
      goto exit_uninit_hw;
    }

  ret = bcmf_gspi_init_save_restore(gbus);
  if (ret != OK)
    {
      goto exit_uninit_hw;
    }

  return OK;

exit_uninit_hw:
  gbus->ready = false;
  bcmf_gspi_hw_uninitialize(gbus);

  return ret;
}

/****************************************************************************
 * Name: bcmf_transfer_bytes
 ****************************************************************************/

/* FIXME: Low level bus data transfer function
 * To avoid bus error, len will be aligned to:
 * - upper power of 2 iflen is lesser than 64
 * - upper 64 bytes block if len is greater than 64
 */

int bcmf_transfer_bytes(FAR bcmf_gspi_dev_t *gbus,
                        bool                 write,
                        uint8_t              function,
                        uint32_t             address,
                        uint8_t             *buf,
                        unsigned int         len)
{
  FAR gspi_dev_t *gspi = gbus->gspi;
  int             ret;

  DEBUGASSERT((((uintptr_t) buf) & 0x03) == 0); /* make sure buf is word aligned */

  if (write)
    {
      ret = gspi->write(gspi,
                        true,
                        function,
                        address,
                        len,
                        (FAR uint32_t *) buf);

      return ret;
    }

  /* -- read btytes -- */

  ret = gspi->read(gspi,
                   true,
                   function,
                   address,
                   len,
                   (FAR uint32_t *) buf);

  return ret;
}

/****************************************************************************
 * Name: bcmf_read_reg
 ****************************************************************************/

int bcmf_read_reg(FAR bcmf_gspi_dev_t *gbus,
                  uint8_t              function,
                  uint32_t             address,
                  uint8_t             *reg)
{
  *reg = bcmf_gspi_read_reg_8(gbus->gspi, function, address);

  return OK;
}

/****************************************************************************
 * Name: bcmf_write_reg
 ****************************************************************************/

int bcmf_write_reg(FAR bcmf_gspi_dev_t *gbus,
                   uint8_t              function,
                   uint32_t             address,
                   uint8_t              reg)
{
  bcmf_gspi_write_reg_8(gbus->gspi, function, address, reg);

  return OK;
}


/****************************************************************************
 * drivers/wireless/ieee80211/morse/morse_mmhal.c
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

/* Implementation of the Morse Micro Hardware Abstraction Layer (mmhal) used
 * by libmorse: SPI transport, control GPIOs, interrupts and access to the
 * chip firmware and board configuration file (BCF).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/irq.h>
#include <nuttx/macro.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wireless/morse.h>

#include "mmhal_core.h"
#include "mmhal_wlan.h"
#include "mmosal.h"
#include "morse.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of 0xff bytes clocked with CS high to put the chip SPI interface
 * in a known state (>= 74 clocks, SD specification 6.4.1.1).
 */

#define MORSE_TRAINING_BYTES  16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct morse_hal_s
{
  FAR struct spi_dev_s *spi;
  FAR const struct morse_lower_s *lower;
  uint32_t frequency;
  mmhal_irq_handler_t spi_handler;
  mmhal_irq_handler_t busy_handler;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct morse_hal_s g_morse_hal;

/* Chip firmware and BCF embedded in the image (mbin format) */

extern const uint8_t g_morse_fw[];
extern const uint8_t g_morse_fw_end[];
extern const uint8_t g_morse_bcf[];
extern const uint8_t g_morse_bcf_end[];

__asm__(".section .rodata.morse_fw, \"a\", %progbits\n"
        ".balign 4\n"
        ".globl g_morse_fw\n"
        "g_morse_fw:\n"
        ".incbin \"" STRINGIFY(MORSE_FW_FILE) "\"\n"
        ".globl g_morse_fw_end\n"
        "g_morse_fw_end:\n"
        ".balign 4\n"
        ".globl g_morse_bcf\n"
        "g_morse_bcf:\n"
        ".incbin \"" STRINGIFY(MORSE_BCF_FILE) "\"\n"
        ".globl g_morse_bcf_end\n"
        "g_morse_bcf_end:\n"
        ".previous\n");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int morse_spi_isr(int irq, FAR void *context, FAR void *arg)
{
  if (g_morse_hal.spi_handler != NULL)
    {
      g_morse_hal.spi_handler();
    }

  return OK;
}

static int morse_busy_isr(int irq, FAR void *context, FAR void *arg)
{
  if (g_morse_hal.busy_handler != NULL)
    {
      g_morse_hal.busy_handler();
    }

  return OK;
}

static void morse_read_blob(FAR const uint8_t *start,
                            FAR const uint8_t *end, uint32_t offset,
                            uint32_t requested_len,
                            FAR struct mmhal_robuf *robuf)
{
  size_t size = end - start;

  robuf->buf      = NULL;
  robuf->len      = 0;
  robuf->free_cb  = NULL;
  robuf->free_arg = NULL;

  if (offset > size)
    {
      wlerr("ERROR: read beyond the end of the file (%" PRIu32 ")\n",
            offset);
      return;
    }

  robuf->buf = start + offset;
  robuf->len = MIN(size - offset, requested_len);
}

static void morse_spi_configure(FAR struct spi_dev_s *spi)
{
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, g_morse_hal.frequency);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int morse_hal_initialize(FAR struct spi_dev_s *spi,
                         FAR const struct morse_lower_s *lower)
{
  int ret;

  DEBUGASSERT(spi != NULL && lower != NULL);
  DEBUGASSERT(lower->reset != NULL && lower->irq_attach != NULL &&
              lower->irq_enable != NULL && lower->irq_asserted != NULL);

  g_morse_hal.spi       = spi;
  g_morse_hal.lower     = lower;
  g_morse_hal.frequency = lower->frequency != 0 ? lower->frequency :
                          CONFIG_IEEE80211_MORSE_SPI_FREQUENCY;

  wlinfo("FW %zu bytes, BCF %zu bytes, SPI %" PRIu32 " Hz\n",
         (size_t)(g_morse_fw_end - g_morse_fw),
         (size_t)(g_morse_bcf_end - g_morse_bcf), g_morse_hal.frequency);

  lower->irq_enable(lower, false);
  ret = lower->irq_attach(lower, morse_spi_isr, NULL);
  if (ret < 0)
    {
      return ret;
    }

  if (lower->busy_attach != NULL && lower->busy_enable != NULL)
    {
      lower->busy_enable(lower, false);
      ret = lower->busy_attach(lower, morse_busy_isr, NULL);
    }

  return ret;
}

/****************************************************************************
 * Name: morse_hal_has_powersave
 *
 * Description:
 *   Return true if the board connects both the WAKE and the BUSY lines,
 *   which are required to put the chip in power save.
 *
 ****************************************************************************/

bool morse_hal_has_powersave(void)
{
  return g_morse_hal.lower->wake != NULL && g_morse_hal.lower->busy != NULL;
}

/* Core *********************************************************************/

uint32_t mmhal_random_u32(uint32_t min, uint32_t max)
{
  uint32_t value;

  arc4random_buf(&value, sizeof(value));

  if (min == 0 && max == UINT32_MAX)
    {
      return value;
    }

  return min + value % (max - min + 1);
}

void mmhal_set_deep_sleep_veto(uint8_t veto_id)
{
  UNUSED(veto_id);
}

void mmhal_clear_deep_sleep_veto(uint8_t veto_id)
{
  UNUSED(veto_id);
}

/* WLAN *********************************************************************/

void mmhal_wlan_init(void)
{
  mmhal_wlan_assert_reset(false);
}

void mmhal_wlan_deinit(void)
{
  mmhal_wlan_assert_reset(true);
}

const struct mmhal_chip *mmhal_get_chip(void)
{
#ifdef CONFIG_IEEE80211_MORSE_MM8108
  return &mmhal_mm8108;
#else
  return &mmhal_mm6108;
#endif
}

void mmhal_read_mac_addr(FAR uint8_t *mac_addr)
{
  FAR const struct morse_lower_s *lower = g_morse_hal.lower;
  uint8_t mac[MMWLAN_MAC_ADDR_LEN];

  /* A board provided address takes precedence over the chip OTP one.  If
   * neither is valid the address is left zeroed and libmorse generates a
   * random one.
   */

  if (lower->getmac != NULL && lower->getmac(lower, mac))
    {
      memcpy(mac_addr, mac, sizeof(mac));
    }
}

void mmhal_wlan_hard_reset(void)
{
  mmhal_wlan_assert_reset(true);
  mmosal_task_sleep(5);
  mmhal_wlan_assert_reset(false);
  mmosal_task_sleep(20);
}

void mmhal_wlan_assert_reset(bool assert_reset)
{
  g_morse_hal.lower->reset(g_morse_hal.lower, assert_reset);
}

void mmhal_wlan_wake_assert(void)
{
  if (g_morse_hal.lower->wake != NULL)
    {
      g_morse_hal.lower->wake(g_morse_hal.lower, true);
    }
}

void mmhal_wlan_wake_deassert(void)
{
  if (g_morse_hal.lower->wake != NULL)
    {
      g_morse_hal.lower->wake(g_morse_hal.lower, false);
    }
}

bool mmhal_wlan_busy_is_asserted(void)
{
  /* Without a BUSY line the chip cannot tell when it wants to be woken
   * up.  Reporting it as always busy prevents libmorse from suspending the
   * host interface (see morse_hal_has_powersave()).
   */

  if (g_morse_hal.lower->busy == NULL)
    {
      return true;
    }

  return g_morse_hal.lower->busy(g_morse_hal.lower);
}

void mmhal_wlan_register_busy_irq_handler(mmhal_irq_handler_t handler)
{
  g_morse_hal.busy_handler = handler;
}

void mmhal_wlan_set_busy_irq_enabled(bool enabled)
{
  if (g_morse_hal.lower->busy_enable != NULL)
    {
      g_morse_hal.lower->busy_enable(g_morse_hal.lower, enabled);
    }
}

void mmhal_wlan_register_spi_irq_handler(mmhal_irq_handler_t handler)
{
  g_morse_hal.spi_handler = handler;
}

bool mmhal_wlan_spi_irq_is_asserted(void)
{
  return g_morse_hal.lower->irq_asserted(g_morse_hal.lower);
}

void mmhal_wlan_set_spi_irq_enabled(bool enabled)
{
  FAR const struct morse_lower_s *lower = g_morse_hal.lower;

  if (enabled)
    {
      /* The chip holds the IRQ line low while it has pending events.  The
       * interrupt is edge triggered, so check the line level to not miss
       * an event that was signaled while the interrupt was disabled.
       */

      if (lower->irq_asserted(lower) && g_morse_hal.spi_handler != NULL)
        {
          g_morse_hal.spi_handler();
        }
      else
        {
          lower->irq_enable(lower, true);
        }
    }
  else
    {
      lower->irq_enable(lower, false);
    }
}

void mmhal_wlan_read_bcf_file(uint32_t offset, uint32_t requested_len,
                              FAR struct mmhal_robuf *robuf)
{
  morse_read_blob(g_morse_bcf, g_morse_bcf_end, offset, requested_len,
                  robuf);
}

void mmhal_wlan_read_fw_file(uint32_t offset, uint32_t requested_len,
                             FAR struct mmhal_robuf *robuf)
{
  morse_read_blob(g_morse_fw, g_morse_fw_end, offset, requested_len,
                  robuf);
}

/* SPI **********************************************************************/

void mmhal_wlan_spi_cs_assert(void)
{
  FAR struct spi_dev_s *spi = g_morse_hal.spi;

  SPI_LOCK(spi, true);
  morse_spi_configure(spi);
  SPI_SELECT(spi, g_morse_hal.lower->devid, true);
}

void mmhal_wlan_spi_cs_deassert(void)
{
  FAR struct spi_dev_s *spi = g_morse_hal.spi;

  SPI_SELECT(spi, g_morse_hal.lower->devid, false);
  SPI_LOCK(spi, false);
}

uint8_t mmhal_wlan_spi_rw(uint8_t data)
{
  return (uint8_t)SPI_SEND(g_morse_hal.spi, data);
}

void mmhal_wlan_spi_read_buf(FAR uint8_t *buf, unsigned len)
{
  /* The SPI driver clocks out 0xff when receiving */

  SPI_RECVBLOCK(g_morse_hal.spi, buf, len);
}

void mmhal_wlan_spi_write_buf(FAR const uint8_t *buf, unsigned len)
{
  SPI_SNDBLOCK(g_morse_hal.spi, buf, len);
}

void mmhal_wlan_send_training_seq(void)
{
  FAR struct spi_dev_s *spi = g_morse_hal.spi;
  int i;

  /* Clock the bus with CS de-asserted and MOSI high */

  SPI_LOCK(spi, true);
  morse_spi_configure(spi);
  SPI_SELECT(spi, g_morse_hal.lower->devid, false);

  for (i = 0; i < MORSE_TRAINING_BYTES; i++)
    {
      SPI_SEND(spi, 0xff);
    }

  SPI_LOCK(spi, false);
}

/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_qspi.c
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

#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>

#include <arch/barriers.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/qspi.h>

#include "chip.h"
#include "bsp_api.h"
#include "r_xspi_qspi.h"

#include "rzv2h_qspi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* R_XSPI_QSPI_DirectTransfer() moves at most 4 bytes of data per
 * transaction.  The CDT buffer's data field is architecturally 8 bytes
 * (CDD0+CDD1), but this FSP driver's r_xspi_qspi_direct_transfer()
 * (external/hal_renesas/drivers/rz/fsp/src/rzv/r_xspi_qspi/r_xspi_qspi.c)
 * only ever reads/writes CDD0 -- CDD1 is never touched.  A data_length of
 * 5-8 silently drops the top bytes instead of erroring, so the limit has
 * to be enforced here.  Matches refer/RZV2HCR8_0/src/xspi_init.c's own
 * xspi_write_1s1s1s()/xspi_read_1s1s1s(), which document the same "max 4
 * bytes - limited by the driver's single CDD0 register".
 */

#define XSPI_MAX_XFRBYTES          4

/* Polling controls.  Page program normally completes much sooner than this.
 * The timeout prevents a bad status response or hardware problem from
 * blocking the caller forever.
 */

#define XSPI_READY_POLL_DELAY_US   10
#define XSPI_PROGRAM_TIMEOUT_US    1000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rzv2h_qspidev_s
{
  struct qspi_dev_s         qspi;      /* Externally visible QSPI iface */
  mutex_t                   lock;      /* Mutual exclusion */
  uint32_t                  frequency; /* Requested frequency (advisory
                                         * only, see rzv2h_qspi_setfreq) */
  enum qspi_mode_e          mode;
  int                       nbits;

  /* Board-supplied flash description */

  const struct rzv2h_qspi_flash_s *flash;

  /* Renesas FSP xSPI-QSPI HAL instance */

  xspi_qspi_instance_ctrl_t ctrl;
  spi_flash_cfg_t           cfg;
  xspi_qspi_extended_cfg_t  extend;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rzv2h_qspi_lock(struct qspi_dev_s *dev, bool lock);
static uint32_t rzv2h_qspi_setfrequency(struct qspi_dev_s *dev,
                                         uint32_t frequency);
static void rzv2h_qspi_setmode(struct qspi_dev_s *dev,
                                enum qspi_mode_e mode);
static void rzv2h_qspi_setbits(struct qspi_dev_s *dev, int nbits);
static int rzv2h_qspi_command(struct qspi_dev_s *dev,
                               struct qspi_cmdinfo_s *cmdinfo);
static int rzv2h_qspi_memory(struct qspi_dev_s *dev,
                              struct qspi_meminfo_s *meminfo);
static void *rzv2h_qspi_alloc(struct qspi_dev_s *dev, size_t buflen);
static void rzv2h_qspi_free(struct qspi_dev_s *dev, void *buffer);
static int rzv2h_qspi_page_program(struct rzv2h_qspidev_s *priv,
                                    uint16_t cmd, uint8_t addrlen,
                                    uint32_t addr, const uint8_t *buffer,
                                    size_t buflen);
static int rzv2h_qspi_xlate_read(struct rzv2h_qspidev_s *priv,
                                  uint32_t addr, uint8_t *buffer,
                                  uint32_t remaining, uint8_t dummies);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qspi_ops_s g_qspi_ops =
{
  .lock         = rzv2h_qspi_lock,
  .setfrequency = rzv2h_qspi_setfrequency,
  .setmode      = rzv2h_qspi_setmode,
  .setbits      = rzv2h_qspi_setbits,
#ifdef CONFIG_QSPI_HWFEATURES
  .hwfeatures   = NULL,
#endif
  .command      = rzv2h_qspi_command,
  .memory       = rzv2h_qspi_memory,
  .alloc        = rzv2h_qspi_alloc,
  .free         = rzv2h_qspi_free,
};

/* Generic JEDEC-standard SPI NOR flash geometry, used as the initial
 * xSPI configuration.  These values (page size, erase sizes/opcodes,
 * status/write-enable opcodes) are common defaults shared by most
 * standard SPI NOR parts, but should be reviewed against the actual
 * flash device fitted on the target board.
 */

static const spi_flash_erase_command_t g_erase_commands[] =
{
  {
    .command = 0x20, .size = 4096
  },              /* Sector erase (4 KiB) */
  {
    .command = 0x52, .size = 32768
  },              /* Block erase (32 KiB) */
  {
    .command = 0xd8, .size = 65536
  },              /* Block erase (64 KiB) */
  {
    .command = 0xc7, .size = SPI_FLASH_ERASE_SIZE_CHIP_ERASE
  },
};

static const xspi_qspi_timing_setting_t g_timing_settings =
{
  .command_to_command_interval = XSPI_QSPI_COMMAND_INTERVAL_CLOCKS_7,
  .cs_pullup_lag                = XSPI_QSPI_CS_PULLUP_CLOCKS_NO_EXTENSION,
  .cs_pulldown_lead             = XSPI_QSPI_CS_PULLDOWN_CLOCKS_NO_EXTENSION,
};

static struct rzv2h_qspidev_s g_qspidev0;
static bool g_qspidev0_initialized;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_qspi_lock
 ****************************************************************************/

static int rzv2h_qspi_lock(struct qspi_dev_s *dev, bool lock)
{
  struct rzv2h_qspidev_s *priv = (struct rzv2h_qspidev_s *)dev;

  return lock ? nxmutex_lock(&priv->lock) : nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: rzv2h_qspi_setfrequency
 *
 * Description:
 *   The xSPI serial clock (SPI0CLK) is generated by a dedicated CPG
 *   divider (CPG_CSDIV0.DIVCTL3, selector FSP_PRIV_CLOCK_DIVIDER_
 *   PLLCM33_XSPI) that only feeds this peripheral, so it is safe to
 *   change here.  Only four ratios are available (source clock divided
 *   by 2, 4, 8, or 16); the closest ratio that does not exceed the
 *   requested frequency is selected.
 *
 ****************************************************************************/

static uint32_t rzv2h_qspi_setfrequency(struct qspi_dev_s *dev,
                                         uint32_t frequency)
{
  static const uint32_t g_xspi_dividers[4] =
  {
    2, 4, 8, 16
  };

  struct rzv2h_qspidev_s *priv = (struct rzv2h_qspidev_s *)dev;
  uint32_t divctl3 = R_CPG->CPG_CSDIV0_b.DIVCTL3;
  uint32_t source;
  uint32_t best = 3;
  int i;

  if (frequency == 0)
    {
      return priv->frequency;
    }

  if (divctl3 >= 4)
    {
      divctl3 = 3;
    }

  source = R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_SPI0CLK) *
           g_xspi_dividers[divctl3];

  for (i = 0; i < 4; i++)
    {
      if ((source / g_xspi_dividers[i]) <= frequency)
        {
          best = i;
          break;
        }
    }

  R_BSP_ClockDividerSet(FSP_PRIV_CLOCK_DIVIDER_PLLCM33_XSPI, best);

  priv->frequency = R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_SPI0CLK);
  return priv->frequency;
}

/****************************************************************************
 * Name: rzv2h_qspi_setmode
 ****************************************************************************/

static void rzv2h_qspi_setmode(struct qspi_dev_s *dev,
                                enum qspi_mode_e mode)
{
  struct rzv2h_qspidev_s *priv = (struct rzv2h_qspidev_s *)dev;

  priv->mode = mode;
}

/****************************************************************************
 * Name: rzv2h_qspi_setbits
 ****************************************************************************/

static void rzv2h_qspi_setbits(struct qspi_dev_s *dev, int nbits)
{
  struct rzv2h_qspidev_s *priv = (struct rzv2h_qspidev_s *)dev;

  priv->nbits = nbits;
}

/****************************************************************************
 * Name: rzv2h_qspi_protocol
 *
 * Description:
 *   Map the generic QSPIMEM_xxx width flags onto one of the SPI
 *   protocol settings the xSPI-QSPI FSP driver understands.  Combinations
 *   that are not one of the standard flash protocols fall back to plain
 *   single-line SPI.
 *
 ****************************************************************************/

static spi_flash_protocol_t rzv2h_qspi_protocol(bool dual, bool quad,
                                                 bool idual, bool iquad)
{
  if (quad)
    {
      return iquad ? SPI_FLASH_PROTOCOL_4S_4S_4S :
                      SPI_FLASH_PROTOCOL_1S_4S_4S;
    }
  else if (dual)
    {
      return idual ? SPI_FLASH_PROTOCOL_2S_2S_2S :
                      SPI_FLASH_PROTOCOL_1S_2S_2S;
    }

  return SPI_FLASH_PROTOCOL_1S_1S_1S;
}

/****************************************************************************
 * Name: rzv2h_qspi_setprotocol
 ****************************************************************************/

static int rzv2h_qspi_setprotocol(struct rzv2h_qspidev_s *priv,
                                   spi_flash_protocol_t protocol)
{
  fsp_err_t err;

  if (priv->ctrl.spi_protocol == protocol)
    {
      return OK;
    }

  err = R_XSPI_QSPI_SpiProtocolSet(&priv->ctrl, protocol);
  if (err != FSP_SUCCESS)
    {
      syslog(LOG_ERR, "ERROR: QSPI protocol change failed: %d\n", err);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_qspi_wait_ready
 ****************************************************************************/

static int rzv2h_qspi_wait_ready(struct rzv2h_qspidev_s *priv,
                                 uint32_t timeout_us)
{
  spi_flash_status_t status;
  fsp_err_t err;
  uint32_t elapsed = 0;

  do
    {
      err = R_XSPI_QSPI_StatusGet(&priv->ctrl, &status);
      if (err != FSP_SUCCESS)
        {
          syslog(LOG_ERR, "ERROR: QSPI StatusGet failed: %d\n", err);
          return -EIO;
        }

      if (!status.write_in_progress)
        {
          return OK;
        }

      up_udelay(XSPI_READY_POLL_DELAY_US);
      elapsed += XSPI_READY_POLL_DELAY_US;
    }
  while (elapsed < timeout_us);

  syslog(LOG_ERR, "ERROR: QSPI ready timeout after %" PRIu32 " us\n",
         timeout_us);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rzv2h_qspi_transfer
 *
 * Description:
 *   Perform a single xSPI manual-command transaction via the FSP
 *   DirectTransfer API: opcode, optional address, and up to
 *   XSPI_MAX_XFRBYTES bytes of data in either direction.
 *
 ****************************************************************************/

static int rzv2h_qspi_transfer(struct rzv2h_qspidev_s *priv, uint16_t cmd,
                                uint32_t addr, uint8_t addrlen,
                                void *buffer, size_t buflen,
                                uint8_t dummies, bool iswrite)
{
  spi_flash_direct_transfer_t transfer;
  fsp_err_t err;

  DEBUGASSERT(buflen <= XSPI_MAX_XFRBYTES);

  if (buflen > XSPI_MAX_XFRBYTES || (buflen > 0 && buffer == NULL))
    {
      return -EINVAL;
    }

  memset(&transfer, 0, sizeof(transfer));
  transfer.command        = cmd;
  transfer.command_length = 1;
  transfer.address        = addr;
  transfer.address_length = addrlen;
  transfer.data_length    = buflen;
  transfer.dummy_cycles   = dummies;

  if (iswrite && buflen > 0)
    {
      memcpy(&transfer.data_u64, buffer, buflen);
    }

  err = R_XSPI_QSPI_DirectTransfer(&priv->ctrl, &transfer,
                                    iswrite ?
                                    SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE :
                                    SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
  if (err != FSP_SUCCESS)
    {
      syslog(LOG_ERR,
             "ERROR: QSPI transfer failed: err=%d cmd=0x%02x "
             "addr=0x%08" PRIx32 " len=%zu write=%d\n",
             err, cmd, addr, buflen, iswrite);
      return err == FSP_ERR_DEVICE_BUSY ? -EBUSY : -EIO;
    }

  if (!iswrite && buflen > 0)
    {
      memcpy(buffer, &transfer.data_u64, buflen);
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_qspi_page_program
 *
 * Description:
 *   Program flash via manual DirectTransfer, matching the CR8 reference
 *   (xspi_init.c's xspi_write_1s1s1s()). The memory-mapped write path
 *   (R_XSPI_QSPI_Write()) was tried and dropped: it can stall the CPU on
 *   BMCTL1.MWRPUSH with no recoverable timeout.
 *
 *   Each DirectTransfer chunk deasserts CS, clearing the write-enable
 *   latch, so WREN is reissued before every chunk.
 *
 ****************************************************************************/

static int rzv2h_qspi_page_program(struct rzv2h_qspidev_s *priv,
                                    uint16_t cmd, uint8_t addrlen,
                                    uint32_t addr, const uint8_t *buffer,
                                    size_t buflen)
{
  uint32_t page_size = priv->cfg.page_size_bytes;
  int ret;

  if (page_size == 0 || (buflen > 0 && buffer == NULL))
    {
      return -EINVAL;
    }

  while (buflen > 0)
    {
      uint32_t page_left = page_size - (addr % page_size);
      size_t chunk = buflen;

      if (chunk > XSPI_MAX_XFRBYTES)
        {
          chunk = XSPI_MAX_XFRBYTES;
        }

      if (chunk > page_left)
        {
          chunk = page_left;
        }

      /* The previous chunk's CS deassertion cleared the write-enable
       * latch, so every chunk needs its own WREN.
       */

      ret = rzv2h_qspi_transfer(priv, priv->cfg.write_enable_command,
                                0, 0, NULL, 0, 0, true);
      if (ret < 0)
        {
          return ret;
        }

      ret = rzv2h_qspi_transfer(priv, cmd, addr, addrlen,
                                (void *)buffer, chunk, 0, true);
      if (ret < 0)
        {
          return ret;
        }

      ret = rzv2h_qspi_wait_ready(priv, XSPI_PROGRAM_TIMEOUT_US);
      if (ret < 0)
        {
          return ret;
        }

      buffer += chunk;
      addr   += chunk;
      buflen -= chunk;
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_qspi_xlate_read
 *
 * Description:
 *   Re-encode the read opcode requested by the MTD driver (e.g. the
 *   n25qxxx driver's Quad I/O Fast Read, 1S-4S-4S) onto the opcode and
 *   protocol the board selected in struct rzv2h_qspi_flash_s, since
 *   the MTD driver always asks for the same opcode regardless of wiring.
 *
 *   4S-4S-4S and 4S-4D-4D both enter the flash's persistent Quad SPI
 *   mode (qpi_enter_cmd) before the read and always exit it
 *   (qpi_exit_cmd) afterward -- even on failure -- since every other
 *   command on this port assumes 1S-1S-1S.
 *
 ****************************************************************************/

static int rzv2h_qspi_xlate_read(struct rzv2h_qspidev_s *priv,
                                  uint32_t addr, uint8_t *buffer,
                                  uint32_t remaining, uint8_t dummies)
{
  const struct rzv2h_qspi_flash_s *flash = priv->flash;
  spi_flash_protocol_t protocol;
  bool persistent = false;
  int exit_ret;
  int ret;

  switch (flash->read_protocol)
    {
      case RZV2H_QSPI_PROTOCOL_1S1S1S:
        protocol = SPI_FLASH_PROTOCOL_1S_1S_1S;
        break;

      case RZV2H_QSPI_PROTOCOL_1S2S2S:
        protocol = SPI_FLASH_PROTOCOL_1S_2S_2S;
        break;

      case RZV2H_QSPI_PROTOCOL_4S4S4S:
        protocol   = SPI_FLASH_PROTOCOL_4S_4S_4S;
        persistent = true;
        break;

      case RZV2H_QSPI_PROTOCOL_4S4D4D:
        protocol   = SPI_FLASH_PROTOCOL_4S_4D_4D;
        persistent = true;
        break;

      default:
        return -EINVAL;
    }

  if (flash->read_dummies >= 0)
    {
      dummies = flash->read_dummies;
    }

  if (persistent)
    {
      ret = rzv2h_qspi_setprotocol(priv, SPI_FLASH_PROTOCOL_1S_1S_1S);
      if (ret < 0)
        {
          return ret;
        }

      ret = rzv2h_qspi_transfer(priv, flash->qpi_enter_cmd, 0, 0, NULL, 0,
                                0, true);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: ENTER QPI failed: %d\n", ret);
          return ret;
        }
    }

  ret = rzv2h_qspi_setprotocol(priv, protocol);
  if (ret == OK)
    {
      while (remaining > 0)
        {
          size_t chunk = remaining > XSPI_MAX_XFRBYTES ?
                         XSPI_MAX_XFRBYTES : remaining;

          ret = rzv2h_qspi_transfer(priv, flash->read_xlate_to, addr, 3,
                                    buffer, chunk, dummies, false);
          if (ret < 0)
            {
              break;
            }

          buffer    += chunk;
          addr      += chunk;
          remaining -= chunk;
        }
    }

  if (persistent)
    {
      exit_ret = rzv2h_qspi_transfer(priv, flash->qpi_exit_cmd, 0, 0, NULL,
                                     0, 0, true);
      if (exit_ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: RESET QPI failed: %d -- flash may be stuck in QPI "
                 "mode\n",
                 exit_ret);
          ret = ret < 0 ? ret : exit_ret;
        }

      exit_ret = rzv2h_qspi_setprotocol(priv, SPI_FLASH_PROTOCOL_1S_1S_1S);
      if (exit_ret < 0 && ret >= 0)
        {
          ret = exit_ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: rzv2h_qspi_command
 ****************************************************************************/

static int rzv2h_qspi_command(struct qspi_dev_s *dev,
                               struct qspi_cmdinfo_s *cmdinfo)
{
  struct rzv2h_qspidev_s *priv = (struct rzv2h_qspidev_s *)dev;
  spi_flash_protocol_t protocol;
  uint8_t addrlen;
  bool iswrite;
  int ret;

  if (cmdinfo == NULL)
    {
      return -EINVAL;
    }

  if (QSPICMD_ISDATA(cmdinfo->flags) &&
      cmdinfo->buflen > XSPI_MAX_XFRBYTES)
    {
      return -EINVAL;
    }

  /* Honor QSPICMD_IDUAL/IQUAD so chip-mode enter/exit commands can use
   * the chip's current instruction width; ordinary commands default to
   * 1S-1S-1S as before.
   */

  if (QSPICMD_ISIQUAD(cmdinfo->flags))
    {
      protocol = SPI_FLASH_PROTOCOL_4S_4S_4S;
    }
  else if (QSPICMD_ISIDUAL(cmdinfo->flags))
    {
      protocol = SPI_FLASH_PROTOCOL_2S_2S_2S;
    }
  else
    {
      protocol = SPI_FLASH_PROTOCOL_1S_1S_1S;
    }

  ret = rzv2h_qspi_setprotocol(priv, protocol);
  if (ret < 0)
    {
      return ret;
    }

  addrlen = QSPICMD_ISADDRESS(cmdinfo->flags) ? cmdinfo->addrlen : 0;
  iswrite = QSPICMD_ISWRITE(cmdinfo->flags);

  return rzv2h_qspi_transfer(priv, cmdinfo->cmd, cmdinfo->addr, addrlen,
                              cmdinfo->buffer,
                              QSPICMD_ISDATA(cmdinfo->flags) ?
                              cmdinfo->buflen : 0,
                              0, iswrite);
}

/****************************************************************************
 * Name: rzv2h_qspi_memory
 *
 * Description:
 *   Perform a memory read/write.  Each xSPI manual-command transaction
 *   can only move XSPI_MAX_XFRBYTES, so larger transfers are split into
 *   a sequence of transactions with increasing addresses.
 *
 ****************************************************************************/

static int rzv2h_qspi_memory(struct qspi_dev_s *dev,
                              struct qspi_meminfo_s *meminfo)
{
  struct rzv2h_qspidev_s *priv = (struct rzv2h_qspidev_s *)dev;
  uint8_t *buffer = (uint8_t *)meminfo->buffer;
  uint32_t addr = meminfo->addr;
  uint32_t remaining = meminfo->buflen;
  bool iswrite = QSPIMEM_ISWRITE(meminfo->flags);
  spi_flash_protocol_t protocol;
  int ret;

  if (meminfo == NULL ||
      (meminfo->buflen > 0 && meminfo->buffer == NULL))
    {
      return -EINVAL;
    }

  buffer = (uint8_t *)meminfo->buffer;
  addr = meminfo->addr;
  remaining = meminfo->buflen;
  iswrite = QSPIMEM_ISWRITE(meminfo->flags);

  if (!iswrite && priv->flash->read_xlate_from != 0 &&
      meminfo->cmd == priv->flash->read_xlate_from)
    {
      return rzv2h_qspi_xlate_read(priv, addr, buffer, remaining,
                                   meminfo->dummies);
    }

  protocol = rzv2h_qspi_protocol(QSPIMEM_ISDUALIO(meminfo->flags),
                                 QSPIMEM_ISQUADIO(meminfo->flags),
                                 QSPIMEM_ISIDUAL(meminfo->flags),
                                 QSPIMEM_ISIQUAD(meminfo->flags));

  ret = rzv2h_qspi_setprotocol(priv, protocol);
  if (ret < 0)
    {
      return ret;
    }

  if (iswrite && meminfo->cmd == priv->cfg.page_program_command)
    {
      return rzv2h_qspi_page_program(priv, meminfo->cmd, meminfo->addrlen,
                                     addr, buffer, remaining);
    }

  /* Splitting a non-page-program write across DirectTransfer calls is
   * unsafe -- each call deasserts CS, clearing the write-enable latch.
   * Larger register writes go through command() instead.
   */

  if (iswrite && remaining > XSPI_MAX_XFRBYTES)
    {
      syslog(LOG_ERR,
             "ERROR: Unsupported QSPI direct write: cmd=0x%02x len=%" PRIu32
             "\n",
             meminfo->cmd, remaining);
      return -ENOTSUP;
    }

  while (remaining > 0)
    {
      size_t chunk = remaining > XSPI_MAX_XFRBYTES ?
                     XSPI_MAX_XFRBYTES : remaining;

      ret = rzv2h_qspi_transfer(priv, meminfo->cmd, addr,
                                meminfo->addrlen, buffer, chunk,
                                meminfo->dummies, iswrite);
      if (ret < 0)
        {
          return ret;
        }

      buffer    += chunk;
      addr      += chunk;
      remaining -= chunk;
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_qspi_pre_init
 *
 * Description:
 *   Pulse the flash reset line (LIOCTL.RSTCS0/RSTCS1, matching whichever
 *   chip select the board wires the flash to)
 *   before the first command; R_XSPI_QSPI_Open() never touches this
 *   register, so without it the flash can start in a stale state
 *   (writes hang, reads look fine).
 *
 *   Matches CR8 reference xspi_init.c's xspi_pre_init(), minus the
 *   Dual/Quad enhanced-volatile-config branch (this port only opens in
 *   1S-1S-1S).
 *
 ****************************************************************************/

static fsp_err_t rzv2h_qspi_pre_init(spi_flash_api_t const *p_api,
                                      xspi_qspi_instance_ctrl_t *p_ctrl)
{
  UNUSED(p_api);

  if (g_qspidev0.flash->chip_select == 1)
    {
      p_ctrl->p_reg->LIOCTL_b.RSTCS1 = 0;
      up_udelay(1);
      p_ctrl->p_reg->LIOCTL_b.RSTCS1 = 1;
    }
  else
    {
      p_ctrl->p_reg->LIOCTL_b.RSTCS0 = 0;
      up_udelay(1);
      p_ctrl->p_reg->LIOCTL_b.RSTCS0 = 1;
    }

  return FSP_SUCCESS;
}

/****************************************************************************
 * Name: rzv2h_qspi_memsize
 *
 * Description:
 *   Map a flash capacity in MiB onto the xSPI memory-size setting.
 *
 ****************************************************************************/

static xspi_qspi_memory_size_t rzv2h_qspi_memsize(uint8_t memory_mb)
{
  switch (memory_mb)
    {
      case 1:
        return XSPI_QSPI_MEMORY_SIZE_1MB;
      case 2:
        return XSPI_QSPI_MEMORY_SIZE_2MB;
      case 4:
        return XSPI_QSPI_MEMORY_SIZE_4MB;
      case 8:
        return XSPI_QSPI_MEMORY_SIZE_8MB;
      case 16:
        return XSPI_QSPI_MEMORY_SIZE_16MB;
      case 32:
        return XSPI_QSPI_MEMORY_SIZE_32MB;
      default:
        return XSPI_QSPI_MEMORY_SIZE_64MB;
    }
}

/****************************************************************************
 * Name: rzv2h_qspi_alloc
 ****************************************************************************/

static void *rzv2h_qspi_alloc(struct qspi_dev_s *dev, size_t buflen)
{
  return kmm_malloc(buflen);
}

/****************************************************************************
 * Name: rzv2h_qspi_free
 ****************************************************************************/

static void rzv2h_qspi_free(struct qspi_dev_s *dev, void *buffer)
{
  kmm_free(buffer);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_qspibus_initialize
 ****************************************************************************/

struct qspi_dev_s *rzv2h_qspibus_initialize(int intf,
                                const struct rzv2h_qspi_flash_s *flash)
{
  struct rzv2h_qspidev_s *priv;
  fsp_err_t err;

  if (intf != 0 || flash == NULL)
    {
      return NULL;
    }

  if (flash->read_xlate_from != 0 &&
      (flash->read_protocol == RZV2H_QSPI_PROTOCOL_4S4S4S ||
       flash->read_protocol == RZV2H_QSPI_PROTOCOL_4S4D4D) &&
      (flash->qpi_enter_cmd == 0 || flash->qpi_exit_cmd == 0))
    {
      return NULL;
    }

  priv = &g_qspidev0;

  if (!g_qspidev0_initialized)
    {
      memset(priv, 0, sizeof(*priv));
      priv->qspi.ops = &g_qspi_ops;
      priv->flash    = flash;
      nxmutex_init(&priv->lock);

      priv->extend.unit              = 0;
      priv->extend.chip_select       = flash->chip_select == 1 ?
                                       XSPI_QSPI_CHIP_SELECT_1 :
                                       XSPI_QSPI_CHIP_SELECT_0;
      priv->extend.memory_size       = rzv2h_qspi_memsize(flash->memory_mb);
      priv->extend.p_timing_settings = &g_timing_settings;
      priv->extend.prefetch_en       = XSPI_QSPI_PREFETCH_FUNCTION_DISABLE;
      priv->extend.pre_init          = rzv2h_qspi_pre_init;
      priv->extend.post_init         = NULL;
      priv->extend.stop_on_close     = false;
      priv->extend.p_reg             = (void *)R_XSPI0;

      priv->cfg.spi_protocol   = SPI_FLASH_PROTOCOL_1S_1S_1S;
      priv->cfg.read_mode      = SPI_FLASH_READ_MODE_STANDARD;
      priv->cfg.address_bytes  = SPI_FLASH_ADDRESS_BYTES_3;
      priv->cfg.dummy_clocks   = SPI_FLASH_DUMMY_CLOCKS_0;
      priv->cfg.page_program_address_lines = SPI_FLASH_DATA_LINES_1;
      priv->cfg.write_status_bit    = flash->write_status_bit;
      priv->cfg.write_enable_bit    = flash->write_enable_bit;
      priv->cfg.page_size_bytes     = flash->page_size;
      priv->cfg.page_program_command = flash->page_program_cmd;
      priv->cfg.write_enable_command = flash->write_enable_cmd;
      priv->cfg.status_command        = flash->status_cmd;
      priv->cfg.read_command           = flash->read_cmd;
      priv->cfg.erase_command_list_length =
        sizeof(g_erase_commands) / sizeof(g_erase_commands[0]);
      priv->cfg.p_erase_command_list      = g_erase_commands;
      priv->cfg.p_extend                  = &priv->extend;

      /* Select PLLCM33 as the XSPI_CLK0 mux source (CPG_SSEL1.SELCTL2).
       * Nothing else programs this mux, so on a NuttX-only boot it would
       * stay at its reset state otherwise -- and
       * rzv2h_qspi_setfrequency()'s divider has no effect without it.
       */

      err = R_BSP_ClockSelectorSet(FSP_PRIV_CLOCK_SELECTOR_SMUX2_XSPI_CLK0,
                                    BSP_CFG_SSEL1_SELCTL2_SOURCE);
      UP_DSB();
      if (err != FSP_SUCCESS)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to select xSPI clock source: %d\n", err);
          nxmutex_destroy(&priv->lock);
          return NULL;
        }

      err = R_XSPI_QSPI_Open(&priv->ctrl, &priv->cfg);
      UP_DSB();
      if (err != FSP_SUCCESS)
        {
          syslog(LOG_ERR, "ERROR: R_XSPI_QSPI_Open failed: %d\n", err);
          nxmutex_destroy(&priv->lock);
          return NULL;
        }

      g_qspidev0_initialized = true;
    }

  return &priv->qspi;
}

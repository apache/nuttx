/****************************************************************************
 * drivers/mmcsd/sdio.c
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

#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <string.h>

#include <nuttx/compiler.h>
#include <nuttx/arch.h>
#include <nuttx/sdio.h>
#include <nuttx/signal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SDIO_CMD53_TIMEOUT_MS 100
#define SDIO_IDLE_DELAY_MS    50

#ifdef CONFIG_SDIO_MUXBUS
#  define SDIO_TAKELOCK(dev)  SDIO_LOCK(dev, true)
#  define SDIO_GIVELOCK(dev)  SDIO_LOCK(dev, false)
#else
#  define SDIO_TAKELOCK(dev)
#  define SDIO_GIVELOCK(dev)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct sdio_cmd52
{
  uint32_t write_data       : 8;
  uint32_t reserved_8       : 1;
  uint32_t register_address : 17;
  uint32_t reserved_26      : 1;
  uint32_t raw_flag         : 1;
  uint32_t function_number  : 3;
  uint32_t rw_flag          : 1;
} end_packed_struct;

begin_packed_struct struct sdio_cmd53
{
  uint32_t byte_block_count : 9;
  uint32_t register_address : 17;
  uint32_t op_code          : 1;
  uint32_t block_mode       : 1;
  uint32_t function_number  : 3;
  uint32_t rw_flag          : 1;
} end_packed_struct;

begin_packed_struct struct sdio_resp_r5
{
  uint32_t data             : 8;
  begin_packed_struct
  struct
  {
    uint32_t out_of_range     : 1;
    uint32_t function_number  : 1;
    uint32_t rfu              : 1;
    uint32_t error            : 1;
    uint32_t io_current_state : 2;
    uint32_t illegal_command  : 1;
    uint32_t com_crc_error    : 1;
  }
  end_packed_struct flags;
  uint32_t reserved_16      : 16;
} end_packed_struct;

union sdio_cmd5x
{
  uint32_t value;
  struct sdio_cmd52 cmd52;
  struct sdio_cmd53 cmd53;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sdio_sendcmdpoll(FAR struct sdio_dev_s *dev,
                            uint32_t cmd, uint32_t arg)
{
  int ret;

  /* Send the command */

  ret = SDIO_SENDCMD(dev, cmd, arg);
  if (ret == OK)
    {
      /* Then poll-wait until the response is available */

      ret = SDIO_WAITRESPONSE(dev, cmd);
      if (ret != OK)
        {
          wlerr("ERROR: Wait for response to cmd: %08" PRIx32
                " failed: %d\n",
                cmd, ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sdio_io_rw_direct(FAR struct sdio_dev_s *dev, bool write,
                      uint8_t function, uint32_t address,
                      uint8_t inb, FAR uint8_t *outb)
{
  union sdio_cmd5x arg;
  struct sdio_resp_r5 resp;
  uint32_t data;
  int ret;

  /* Setup CMD52 argument */

  arg.value = 0;

  if (write)
    {
      arg.cmd52.write_data   = inb;
    }
  else
    {
      arg.cmd52.write_data   = 0;
    }

  arg.cmd52.register_address = address & 0x1ffff;
  arg.cmd52.raw_flag         = (write && outb);
  arg.cmd52.function_number  = function & 7;
  arg.cmd52.rw_flag          = write;

  /* Send CMD52 command */

  SDIO_TAKELOCK(dev);
  sdio_sendcmdpoll(dev, SD_ACMD52, arg.value);
  ret = SDIO_RECVR5(dev, SD_ACMD52, &data);
  SDIO_GIVELOCK(dev);

  if (ret != OK)
    {
      wlerr("ERROR: SDIO_RECVR5 failed %d\n", ret);
      return ret;
    }

  memcpy(&resp, &data, sizeof(resp));

  /* Check for errors */

  if (resp.flags.error)
    {
      return -EIO;
    }

  if (resp.flags.function_number || resp.flags.out_of_range)
    {
      return -EINVAL;
    }

  /* Write output byte */

  if (outb)
    {
      *outb = resp.data & 0xff;
    }

  return OK;
}

int sdio_io_rw_extended(FAR struct sdio_dev_s *dev, bool write,
                        uint8_t function, uint32_t address,
                        bool inc_addr, FAR uint8_t *buf,
                        unsigned int blocklen, unsigned int nblocks)
{
  union sdio_cmd5x arg;
  struct sdio_resp_r5 resp;
  uint32_t data;
  int ret;
  sdio_eventset_t wkupevent;

  /* Setup CMD53 argument */

  arg.value = 0;
  arg.cmd53.register_address = address & 0x1ffff;
  arg.cmd53.op_code          = inc_addr;
  arg.cmd53.function_number  = function & 7;
  arg.cmd53.rw_flag          = write;

  if (nblocks == 0 && blocklen < 512)
    {
      /* Use byte mode */

      arg.cmd53.block_mode = 0;
      arg.cmd53.byte_block_count = blocklen;
      nblocks = 1;
    }
  else
    {
      /* Use block mode */

      arg.cmd53.block_mode = 1;
      arg.cmd53.byte_block_count = nblocks;
    }

  SDIO_TAKELOCK(dev);

  /* Send CMD53 command */

  SDIO_BLOCKSETUP(dev, blocklen, nblocks);
  SDIO_WAITENABLE(dev,
                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR,
                  SDIO_CMD53_TIMEOUT_MS);

  if (write)
    {
      wlinfo("prep write %d %d\n", blocklen, nblocks);

      /* Get the capabilities of the SDIO hardware */

      if ((SDIO_CAPABILITIES(dev) & SDIO_CAPS_DMABEFOREWRITE) != 0)
        {
          SDIO_DMASENDSETUP(dev, buf, blocklen * nblocks);
          SDIO_SENDCMD(dev, SD_ACMD53WR, arg.value);

          wkupevent = SDIO_EVENTWAIT(dev);
          ret = SDIO_RECVR5(dev, SD_ACMD53WR, &data);
        }
      else
        {
          sdio_sendcmdpoll(dev, SD_ACMD53WR, arg.value);
          ret = SDIO_RECVR5(dev, SD_ACMD53WR, &data);

          SDIO_DMASENDSETUP(dev, buf, blocklen * nblocks);
          wkupevent = SDIO_EVENTWAIT(dev);
        }
    }
  else
    {
      wlinfo("prep read %d\n", blocklen * nblocks);
      SDIO_DMARECVSETUP(dev, buf, blocklen * nblocks);
      SDIO_SENDCMD(dev, SD_ACMD53RD, arg.value);

      wkupevent = SDIO_EVENTWAIT(dev);
      ret = SDIO_RECVR5(dev, SD_ACMD53RD, &data);
    }

  wlinfo("Transaction ends\n");
  sdio_sendcmdpoll(dev, SD_ACMD52ABRT, 0);

  /* There may not be a response to this, so don't look for one */

  SDIO_RECVR1(dev, SD_ACMD52ABRT, &data);
  SDIO_GIVELOCK(dev);

  if (ret != OK)
    {
      wlerr("ERROR: SDIO_RECVR5 failed %d\n", ret);
      return ret;
    }

  memcpy(&resp, &data, sizeof(resp));

  /* Check for errors */

  if (wkupevent & SDIOWAIT_TIMEOUT)
    {
      wlerr("timeout\n");
      return -ETIMEDOUT;
    }

  if (resp.flags.error || (wkupevent & SDIOWAIT_ERROR))
    {
      wlerr("error 1\n");
      return -EIO;
    }

  if (resp.flags.function_number || resp.flags.out_of_range)
    {
      wlerr("error 2\n");
      return -EINVAL;
    }

  return OK;
}

int sdio_set_wide_bus(FAR struct sdio_dev_s *dev)
{
  int ret;
  uint8_t value;

  /* Read Bus Interface Control register */

  ret = sdio_io_rw_direct(dev, false, 0, SDIO_CCCR_BUS_IF, 0, &value);
  if (ret != OK)
    {
      return ret;
    }

  /* Set 4 bits bus width setting */

  value &= ~SDIO_CCCR_BUS_IF_WIDTH_MASK;
  value |= SDIO_CCCR_BUS_IF_4_BITS;

  ret = sdio_io_rw_direct(dev, true, 0, SDIO_CCCR_BUS_IF, value, NULL);
  if (ret != OK)
    {
      return ret;
    }

  SDIO_WIDEBUS(dev, true);
  return OK;
}

int sdio_probe(FAR struct sdio_dev_s *dev)
{
  int ret;
  uint32_t data = 0;

  SDIO_TAKELOCK(dev);

  /* Set device state from reset to idle */

  ret = sdio_sendcmdpoll(dev, MMCSD_CMD0, 0);
  if (ret != OK)
    {
      goto err;
    }

  up_mdelay(SDIO_IDLE_DELAY_MS);

  /* Device is SDIO card compatible so we can send CMD5 instead of ACMD41 */

  ret = sdio_sendcmdpoll(dev, SDIO_CMD5, 0);
  if (ret != OK)
    {
      goto err;
    }

  /* Receive R4 response */

  ret = SDIO_RECVR4(dev, SDIO_CMD5, &data);
  if (ret != OK)
    {
      goto err;
    }

  return OK;

  /* Device is in Card Identification Mode, request device RCA */

  ret = sdio_sendcmdpoll(dev, SD_CMD3, 0);
  if (ret != OK)
    {
      goto err;
    }

  ret = SDIO_RECVR6(dev, SD_CMD3, &data);
  if (ret != OK)
    {
      wlerr("ERROR: RCA request failed: %d\n", ret);
      goto err;
    }

  wlinfo("rca is %" PRIx32 "\n", data >> 16);

  /* Send CMD7 with the argument == RCA in order to select the card
   * and put it in Transfer State.
   */

  ret = sdio_sendcmdpoll(dev, MMCSD_CMD7S, data & 0xffff0000);
  if (ret != OK)
    {
      wlerr("ERROR: CMD7 request failed: %d\n", ret);
      goto err;
    }

  ret = SDIO_RECVR1(dev, MMCSD_CMD7S, &data);
  if (ret != OK)
    {
      wlerr("ERROR: card selection failed: %d\n", ret);
      goto err;
    }

  /* Configure 4 bits bus width */

  SDIO_GIVELOCK(dev);
  return sdio_set_wide_bus(dev);

err:
  SDIO_GIVELOCK(dev);
  return OK;
}

int sdio_set_blocksize(FAR struct sdio_dev_s *dev, uint8_t function,
                       uint16_t blocksize)
{
  int ret;

  ret = sdio_io_rw_direct(dev, true, 0,
                  (function << SDIO_FBR_SHIFT) + SDIO_CCCR_FN0_BLKSIZE_0,
                  blocksize & 0xff, NULL);
  if (ret != OK)
    {
      return ret;
    }

  ret = sdio_io_rw_direct(dev, true, 0,
                  (function << SDIO_FBR_SHIFT) + SDIO_CCCR_FN0_BLKSIZE_1,
                  (blocksize >> 8), NULL);

  if (ret != OK)
    {
      return ret;
    }

  return OK;
}

int sdio_enable_function(FAR struct sdio_dev_s *dev, uint8_t function)
{
  int ret;
  uint8_t value;

  /* Read current I/O Enable register */

  ret = sdio_io_rw_direct(dev, false, 0, SDIO_CCCR_IOEN, 0, &value);
  if (ret != OK)
    {
      return ret;
    }

  ret = sdio_io_rw_direct(dev, true, 0,
                          SDIO_CCCR_IOEN, value | (1 << function), NULL);

  if (ret != OK)
    {
      return ret;
    }

  /* Wait 1s for function to be enabled */

  int loops = 100;

  while (loops-- > 0)
    {
      nxsig_usleep(10 * 1000);

      ret = sdio_io_rw_direct(dev, false, 0, SDIO_CCCR_IORDY, 0, &value);
      if (ret != OK)
        {
          return ret;
        }

      if (value & (1 << function))
        {
          /* Function enabled */

          wlinfo("Function %d enabled\n", function);
          return OK;
        }
    }

  return -ETIMEDOUT;
}

int sdio_enable_interrupt(FAR struct sdio_dev_s *dev, uint8_t function)
{
  int ret;
  uint8_t value;

  /* Read current Int Enable register */

  ret = sdio_io_rw_direct(dev, false, 0, SDIO_CCCR_INTEN, 0, &value);
  if (ret != OK)
    {
      return ret;
    }

  return sdio_io_rw_direct(dev, true, 0,
                           SDIO_CCCR_INTEN, value | (1 << function), NULL);
}

/****************************************************************************
 * include/nuttx/mmcsd.h
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

#ifndef __INCLUDE_NUTTX_MMCSD_H
#define __INCLUDE_NUTTX_MMCSD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* mmcsd ioctl */

#define MMC_IOC_CMD             _MMCSDIOC(0x0000)
#define MMC_IOC_MULTI_CMD       _MMCSDIOC(0x0001)

#define MMC_IOC_MAX_BYTES       (512L * 1024)
#define MMC_IOC_MAX_CMDS        255
#define mmc_ioc_cmd_set_data(ic, ptr) ic.data_ptr = (uint64_t)(unsigned long)ptr

/* rpmb request */

#define MMC_RPMB_WRITE_KEY      0x01
#define MMC_RPMB_READ_CNT       0x02
#define MMC_RPMB_WRITE          0x03
#define MMC_RPMB_READ           0x04

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mmc_ioc_cmd
{
  /* Direction of data: nonzero = write, zero = read.
   * Bit 31 selects 'Reliable Write' for RPMB.
   */

  int write_flag;

  /* Application-specific command.  true = precede with CMD55 */

  int is_acmd;

  uint32_t opcode;
  uint32_t arg;
  uint32_t response[4];  /* CMD response */
  unsigned int flags;
  unsigned int blksz;
  unsigned int blocks;

  /* For 64-bit machines, the next member, ``uint64_t data_ptr``, wants to
   * be 8-byte aligned.  Make sure this struct is the same size when
   * built for 32-bit.
   */

  uint32_t pad;

  /* DAT buffer */

  uint64_t data_ptr;
};

/* struct mmc_ioc_multi_cmd - multi command information
 * @num_of_cmds: Number of commands to send. Must be equal to or less than
 * MMC_IOC_MAX_CMDS.
 * @cmds: Array of commands with length equal to 'num_of_cmds'
 */

struct mmc_ioc_multi_cmd
{
  uint64_t num_of_cmds;
  struct mmc_ioc_cmd cmds[0];
};

struct mmc_rpmb_frame_s
{
  uint8_t  stuff[196];
  uint8_t  key_mac[32];
  uint8_t  data[256];
  uint8_t  nonce[16];
  uint32_t write_counter;
  uint16_t addr;
  uint16_t block_count;
  uint16_t result;
  uint16_t req_resp;
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mmcsd_slotinitialize
 *
 * Description:
 *   Initialize one slot for operation using the MMC/SD interface
 *
 * Input Parameters:
 *   minor - The MMC/SD minor device number.  The MMC/SD device will be
 *     registered as /dev/mmcsdN where N is the minor number
 *   dev - And instance of an MMC/SD interface.  The MMC/SD hardware should
 *     be initialized and ready to use.
 *
 ****************************************************************************/

struct sdio_dev_s; /* See nuttx/sdio.h */
int mmcsd_slotinitialize(int minor, FAR struct sdio_dev_s *dev);

/****************************************************************************
 * Name: mmcsd_spislotinitialize
 *
 * Description:
 *   Initialize one slot for operation using the SPI MMC/SD interface
 *
 * Input Parameters:
 *   minor - The MMC/SD minor device number.  The MMC/SD device will be
 *     registered as /dev/mmcsdN where N is the minor number
 *   slotno - The slot number to use.  This is only meaningful for
 *     architectures that support multiple MMC/SD slots. This value must be
 *     in the range {0, ..., CONFIG_MMCSD_NSLOTS}.
 *   spi - And instance of an SPI interface obtained by called the
 *     appropriate xyz_spibus_initialize() function for the MCU "xyz" with
 *     the appropriate port number.
 *
 ****************************************************************************/

struct spi_dev_s; /* See nuttx/spi/spi.h */
int mmcsd_spislotinitialize(int minor, int slotno,
                            FAR struct spi_dev_s *spi);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_MMCSD_H */

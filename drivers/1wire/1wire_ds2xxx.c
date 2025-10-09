/****************************************************************************
 * drivers/1wire/1wire_ds2xxx.c
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
 * Author: Stepan Pressl <pressl.stepan@gmail.com>
 *                       <pressste@fel.cvut.cz>
 *
 ****************************************************************************/

/* This is a driver for 1Wire Maxim Integrated DS2xxx EEPROMs.
 * Currently, the driver was tested against the DS2431 EEPROM but other
 * EEPROMs listed in 1wire_ds2xxx.h
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <inttypes.h>
#include <debug.h>
#include <nuttx/fs/fs.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/1wire/1wire.h>
#include <nuttx/1wire/1wire_crc.h>
#include <nuttx/1wire/1wire_ds2xxx.h>
#include <nuttx/1wire/1wire_master.h>
#include <nuttx/arch.h>

#include "1wire_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ENDIAN_BIG
#  define onewire_leuint64(x) (x)
#  define onewire_leuint32(x) (x)
#endif

#define WS_HEADER_OFFSET   ((uint8_t)3)
#define RS_HEADER_SIZE     ((uint8_t)3)
#define CS_HEADER_SIZE     ((uint8_t)4)
#define HELPBUF_MARGIN     ((uint8_t)5)
#define MATCHROM_BUF_SIZE  ((uint8_t)9)
#define DS2XXX_TPROG_US    ((useconds_t)12000)

/****************************************************************************
 * Priavate Types
 ****************************************************************************/

struct ds2xxx_dev_s
{
  FAR struct onewire_master_s *master;

  uint64_t romcode;              /* Romcode to be addressed on the bus */
  int devtype;                   /* See enum ds2xxx_eeproms_e */
  uint8_t *helpbuf;              /* Helpful memory to avoid alloc on stack */
  uint8_t refs;                  /* Opened references to ds2xxx */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     ds2xxx_open(FAR struct file *filep);
static int     ds2xxx_close(FAR struct file *filep);
static int     ds2xxx_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);
static ssize_t ds2xxx_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t ds2xxx_read_internal(FAR struct ds2xxx_dev_s *priv,
                                    off_t eeprom_addr,
                                    FAR uint8_t *buffer, size_t buflen);
static ssize_t ds2xxx_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static ssize_t ds2xxx_write_internal(FAR struct ds2xxx_dev_s *priv,
                                     off_t eeprom_addr,
                                     FAR uint8_t *buffer, size_t buflen);
static off_t   ds2xxx_seek(FAR struct file *filep, off_t offset, int whence);
static int     ds2xxx_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/* Mapping from ds2xxx_types_e to eeprom sizes, pages and family codes.
 * The driver is fully generic for all supported DS2XXX memories with
 * a write scratchpad.
 */

static size_t g_eeprom_sizes[EEPROM_DS_COUNT] =
{
  32,
  128,
  128,
  512,
  512,
  128,
  2560
};

static size_t g_eeprom_scratchpad_sizes[EEPROM_DS_COUNT] =
{
  8,
  8,
  8,
  32,
  32,
  8,
  32
};

static int g_eeprom_familycodes[EEPROM_DS_COUNT] =
{
  0x14,
  0x2d,
  0xb3,
  0x23,
  0x1c,
  0x2d,
  0x43
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ds2xxx_fops =
{
  .open = ds2xxx_open,
  .close = ds2xxx_close,
  .read = ds2xxx_read,
  .write = ds2xxx_write,
  .seek = ds2xxx_seek,
  .ioctl = ds2xxx_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ENDIAN_BIG
static inline uint64_t onewire_leuint64(uint64_t x)
{
  return (((x & 0xff00000000000000ull) >> 56) |
          ((x & 0x00ff000000000000ull) >> 40) |
          ((x & 0x0000ff0000000000ull) >> 24) |
          ((x & 0x000000ff00000000ull) >> 8)  |
          ((x & 0x00000000ff000000ull) << 8)  |
          ((x & 0x0000000000ff0000ull) << 24) |
          ((x & 0x000000000000ff00ull) << 40) |
          ((x & 0x00000000000000ffull) << 56));
}
#endif

#ifdef CONFIG_ENDIAN_BIG
static inline uint32_t onewire_leuint32(uint32_t x)
{
  return (((x & 0xff000000) >> 24) |
          ((x & 0x00ff0000) >> 8)  |
          ((x & 0x0000ff00) << 8)  |
          ((x & 0x000000ff) << 24));
}
#endif

/****************************************************************************
 * Name: ds2xxx_startcomm
 *
 * Description:
 *   Performs the Match Rom command on the bus.
 *
 ****************************************************************************/

static int ds2xxx_startcomm(struct ds2xxx_dev_s *priv, uint8_t *wrbuf)
{
  int ret;
  uint64_t tmp;

  ret = ONEWIRE_RESET(priv->master->dev);
  if (ret < 0)
    {
      return ret;
    }

  wrbuf[0] = DS2XXX_MATCH_ROM;
  tmp = onewire_leuint64(priv->master->selected_rom);
  memcpy(&wrbuf[1], &tmp, MATCHROM_BUF_SIZE - 1);

  return ONEWIRE_WRITE(priv->master->dev, wrbuf, MATCHROM_BUF_SIZE);
}

/****************************************************************************
 * Name: ds2xxx_read_internal
 *
 * Description:
 *   Begins communication and reads data from the EEPROM. The read procedure
 *   is much simpler as data can be read from any address.
 *
 ****************************************************************************/

static ssize_t ds2xxx_read_internal(FAR struct ds2xxx_dev_s *priv,
                                    off_t eeprom_addr,
                                    FAR uint8_t *buffer, size_t buflen)
{
  int ret;

  /* The process of reading is much simpler.
   * The master must issue the Read Memory command followed with 2 bytes
   * of address to be read from. The address is sent in little endian order.
   */

  /* Initialize the communication */

  ret = ds2xxx_startcomm(priv, buffer);
  if (ret < 0)
    {
      return ret;
    }

  priv->helpbuf[0] = DS2XXX_READ_MEMORY;
  priv->helpbuf[1] = eeprom_addr & 0xff;
  priv->helpbuf[2] = eeprom_addr >> 8;
  ret = ONEWIRE_WRITE(priv->master->dev, priv->helpbuf, 3);
  if (ret < 0)
    {
      return ret;
    }

  /* Now read real data. */

  ret = ONEWIRE_READ(priv->master->dev, (uint8_t *)buffer, buflen);
  return ret;
}

/****************************************************************************
 * Name: ds2xxx_write_internal
 *
 * Description:
 *   Begins communication and reads data from the EEPROM. The write procedure
 *   is not that straightforward. It is assumed the writes by this function
 *   are aligned. It is assumed buflen is at most the size of the scratchpad.
 *
 ****************************************************************************/

static ssize_t ds2xxx_write_internal(FAR struct ds2xxx_dev_s *priv,
                                     off_t eeprom_addr,
                                     FAR uint8_t *buffer, size_t buflen)
{
  ssize_t ret;
  uint8_t buf[32 + HELPBUF_MARGIN];
  uint16_t crc;
  uint16_t reccrc;

  i2cinfo("writing %zu bytes to %ld", buflen, eeprom_addr);

  ret = ds2xxx_startcomm(priv, buf);
  if (ret < 0)
    {
      return ret;
    }

  /* Now issue the write scratchpad command. Also calculate
   * CRC16 which must be compared with the received one.
   */

  buf[0] = DS2XXX_WRITE_SCRATCHPAD;
  buf[1] = eeprom_addr & 0xff;
  buf[2] = eeprom_addr >> 8;
  crc = onewire_crc16(buf, 3, 0);

  ret = ONEWIRE_WRITE(priv->master->dev, buf, 3);
  if (ret < 0)
    {
      return ret;
    }

  crc = onewire_crc16(buffer, buflen, crc);
  ret = ONEWIRE_WRITE(priv->master->dev, buffer, buflen);
  if (ret < 0)
    {
      return ret;
    }

  /* We now need to receive the CRC. The order is little endian. */

  ret = ONEWIRE_READ(priv->master->dev, buf, 2);
  if (ret < 0)
    {
      return ret;
    }

  reccrc = buf[0] | (buf[1] << 8);
  if (reccrc != (uint16_t) ~crc)
    {
      return -EPROTO;
    }

  /* We have successfully performed the Write Scratchpad command.
   * Now issue read scratchpad.
   */

  ret = ds2xxx_startcomm(priv, buf);
  if (ret < 0)
    {
      return ret;
    }

  buf[0] = DS2XXX_READ_SCRATCHPAD;
  ret = ONEWIRE_WRITE(priv->master->dev, buf, 1);
  if (ret < 0)
    {
      return ret;
    }

  crc = onewire_crc16(buf, 1, 0);

  /* TA-E/S + buflen + ~CRC16 => buflen + 5 */

  ret = ONEWIRE_READ(priv->master->dev, buf,
                     buflen + HELPBUF_MARGIN);
  if (ret < 0)
    {
      return ret;
    }

  crc = onewire_crc16(buf, buflen + RS_HEADER_SIZE, crc);
  reccrc = buf[RS_HEADER_SIZE + buflen] |
           buf[RS_HEADER_SIZE + buflen + 1] << 8;
  if (reccrc != (uint16_t) ~crc)
    {
      return -EPROTO;
    }

  /* Memcheck should be performed, but imho CRC16 is already a good check.
   * Commit the scratchpad into the eeprom.
   */

  priv->helpbuf[0] = DS2XXX_COPY_SCRATCHPAD;
  priv->helpbuf[1] = buf[0];
  priv->helpbuf[2] = buf[1];
  priv->helpbuf[3] = buf[2];
  ret = ds2xxx_startcomm(priv, buf);
  if (ret < 0)
    {
      return ret;
    }

  ret = ONEWIRE_WRITE(priv->master->dev, priv->helpbuf, CS_HEADER_SIZE);
  if (ret < 0)
    {
      return ret;
    }

  /* Impose a delay. Across all supported devices, the maximum delay is 12 ms
   * for a page.
   */

  up_udelay(DS2XXX_TPROG_US);

  i2cinfo("aligned write of %zu bytes successful", buflen);
  return OK;
}

/****************************************************************************
 * Name: ds2xxx_open
 *
 * Description:
 *   The open callback function.
 *
 ****************************************************************************/

static int ds2xxx_open(FAR struct file *filep)
{
  FAR struct ds2xxx_dev_s *priv;
  FAR struct inode *inode = filep->f_inode;
  int ret = OK;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  ret = nxrmutex_lock(&priv->master->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if ((priv->refs + 1) == 0)
    {
      ret = -EMFILE;
    }
  else
    {
      priv->refs++;
    }

  nxrmutex_unlock(&priv->master->devlock);
  return OK;
}

/****************************************************************************
 * Name: ds2xxx_write
 *
 * Description:
 *   The write callback function. As writes into the EEPROM must be aligned
 *   with the scratchpad, all nonaligned writes must first read the aligned
 *   part, change some bytes and then commit the aligned part to the
 *   scratchpad. This function takes care of this.
 *
 ****************************************************************************/

static int ds2xxx_write(FAR struct file *filep, FAR const char *buffer,
                        size_t buflen)
{
  FAR struct ds2xxx_dev_s *priv;
  FAR struct inode *inode = filep->f_inode;
  off_t addr;
  off_t misalign;
  size_t to_write;
  size_t scratchpad_size;
  size_t buflen_save = buflen;
  int ret = OK;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;
  scratchpad_size = g_eeprom_scratchpad_sizes[priv->devtype];

  i2cwarn("writing %zu bytes to ptr %" PRIdOFF, buflen, filep->f_pos);

  if (filep->f_pos >= g_eeprom_sizes[priv->devtype])
    {
      return -EFBIG;
    }

  if ((filep->f_pos + buflen) >= g_eeprom_sizes[priv->devtype])
    {
      buflen = g_eeprom_sizes[priv->devtype] - filep->f_pos;
      buflen_save = buflen;
    }

  ret = nxrmutex_lock(&priv->master->devlock);
  if (ret < 0)
    {
      goto done;
    }

  /* The process of writing to 1WIRE EEPROMs involves the usage
   * of the scratchpad. Firstly, the Write Scratchpad command
   * must be issued, the payload is as follows:
   * 2 bytes ADDR (Little Endian) | DATA,
   * where ADDR must be aligned to the scratchpad size
   * (DS2431's scratchpad is 8 bytes, DS2433's scratchpad is 32 bytes
   * and the DATA is maximally 8 bytes or 32 bytes, depending on the EEPROM.
   *
   * Since writes are not generally row aligned, we must read
   * the corresponding unaligned ends beforehand, modifying respective parts
   * and writing this back.
   *
   * To check the transaction has completed successfully, Read Scratchpad
   * command is issued, returning the 3-byte authorization pattern, also.
   *
   * The writing procedure is ended with issuing the Copy Scratchpad
   * command.
   */

  /* Calculate the misalignment in the beginning. And the starting address. */

  misalign = (filep->f_pos) % scratchpad_size;
  addr = (filep->f_pos / scratchpad_size) * scratchpad_size;

  /* The pointer points at the beginning of a page, decide if we skip. */

  if (misalign == 0)
    {
      goto no_misalign_begin;
    }

  ret = ds2xxx_read_internal(priv, addr, priv->helpbuf, scratchpad_size);
  if (ret < 0)
    {
      goto done;
    }

  /* Fill in the misaligned data. But decide on the length of the write. */

  if (buflen >= scratchpad_size - misalign)
    {
      to_write = scratchpad_size - misalign;
    }
  else
    {
      to_write = buflen;
    }

  memcpy(priv->helpbuf + misalign, buffer, to_write);
  ret = ds2xxx_write_internal(priv, addr, priv->helpbuf, scratchpad_size);
  if (ret < 0)
    {
      goto done;
    }

  /* Move all the pointers */

  buffer += to_write;
  buflen -= to_write;
  addr += scratchpad_size;

no_misalign_begin:

  /* Now loop over all aligned writes. If everything was already
   * written in the misaligned write or the next write is also misaligned,
   * this loop won't run.
   */

  while (buflen >= scratchpad_size)
    {
      ret = ds2xxx_write_internal(priv, addr, (uint8_t *)buffer,
                                  scratchpad_size);
      if (ret < 0)
        {
          goto done;
        }

      buffer += scratchpad_size;
      buflen -= scratchpad_size;
      addr += scratchpad_size;
    }

  /* There's no rest. */

  if (buflen == 0)
    {
      goto success;
    }

  /* Finish this with the last read. The read is aligned but needs to be
   * of the same size as the scratchpad.
   */

  ret = ds2xxx_read_internal(priv, addr, priv->helpbuf, scratchpad_size);
  if (ret < 0)
    {
      goto done;
    }

  memcpy(priv->helpbuf, buffer, buflen);
  ret = ds2xxx_write_internal(priv, addr, priv->helpbuf, scratchpad_size);
  if (ret < 0)
    {
      goto done;
    }

  /* Everything went well. Move the pointer */

success:
  i2cwarn("success writing %d bytes", buflen_save);
  filep->f_pos += buflen_save;
  ret = buflen_save;
done:
  nxrmutex_unlock(&priv->master->devlock);
  return ret;
}

/****************************************************************************
 * Name: ds2xxx_read
 *
 * Description:
 *   The read callback function. First, it checks the boundaries and then
 *   calls the ds2xxx_read_internal function.
 *
 ****************************************************************************/

static ssize_t ds2xxx_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  FAR struct ds2xxx_dev_s *priv;
  FAR struct inode *inode = filep->f_inode;
  int ret = OK;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  ret = nxrmutex_lock(&priv->master->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* The pointer is outside the "file" */

  if (filep->f_pos >= g_eeprom_sizes[priv->devtype])
    {
      goto done;
    }

  /* Trim the length */

  if ((filep->f_pos + len) > g_eeprom_sizes[priv->devtype])
    {
      len = g_eeprom_sizes[priv->devtype] - filep->f_pos;
    }

  ret = ds2xxx_read_internal(priv, filep->f_pos, (uint8_t *)buffer, len);
  if (ret < 0)
    {
      goto done;
    }

  ret = len;
  filep->f_pos += len;

  i2cinfo("DS2XXX read OK, curpos=%" PRIdOFF, filep->f_pos);

done:
  nxrmutex_unlock(&priv->master->devlock);
  return ret;
}

/****************************************************************************
 * Name: ds2xxx_close
 *
 * Description:
 *   The close callback function.
 *
 ****************************************************************************/

static int ds2xxx_close(FAR struct file *filep)
{
  FAR struct ds2xxx_dev_s *priv;
  FAR struct inode *inode = filep->f_inode;
  int ret = OK;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  ret = nxrmutex_lock(&priv->master->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->refs == 0)
    {
      ret = -EIO;
    }
  else
    {
      priv->refs--;
    }

  nxrmutex_unlock(&priv->master->devlock);
  return ret;
}

/****************************************************************************
 * Name: ds2xxx_seek
 *
 * Description:
 *   The seek callback function. Practically the same as in i2c_xx24xx.c.
 *
 ****************************************************************************/

static off_t ds2xxx_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct ds2xxx_dev_s *priv;
  FAR struct inode *inode = filep->f_inode;
  int ret;
  off_t newpos;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  ret = nxrmutex_lock(&priv->master->devlock);
  if (ret < 0)
    {
      return ret;
    }

  switch (whence)
    {
    case SEEK_CUR:
      newpos = filep->f_pos + offset;
      break;
    case SEEK_SET:
      newpos = offset;
      break;
    case SEEK_END:
      newpos = g_eeprom_sizes[priv->devtype] + offset;
      break;
    default:
      nxrmutex_unlock(&priv->master->devlock);
      return -EINVAL;
    }

  /* Opengroup.org:
   *
   *  "The lseek() function shall allow the file offset to be set beyond the
   *   end of the existing data in the file. If data is later written at
   *   this point, subsequent reads of data in the gap shall return bytes
   *   with the value 0 until data is actually written into the gap."
   *
   * We can conform to the first part, but not the second. But return -EINVAL
   * if
   *
   *  "...the resulting file offset would be negative for a regular file,
   *   block special file, or directory."
   */

  if (newpos >= 0)
    {
      filep->f_pos = newpos;
      ret = newpos;
      i2cinfo("ds2xxx_seek: newpos %" PRIdOFF, newpos);
    }
  else
    {
      ret = -EINVAL;
    }

  nxrmutex_unlock(&priv->master->devlock);
  return ret;
}

/****************************************************************************
 * Name: ds2xxx_ioctl
 *
 * Description:
 *   The ioctl callback function. The target romcode is specified here.
 *
 ****************************************************************************/

static int ds2xxx_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct ds2xxx_dev_s *priv;
  FAR struct inode *inode = filep->f_inode;
  int ret = OK;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  nxrmutex_lock(&priv->master->devlock);
  switch (cmd)
    {
      case ONEWIREIOC_SETROM:
        /* The issue is that unsigned long is 32bits on most architectures.
         * That means we should not pass the romcode as a value.
         */

        i2cinfo("Rom to be set is 0x%" PRIx64, *((uint64_t *)arg));
        ret = onewire_ioctl_setrom(priv->master, *((uint64_t *)arg));
        break;
      case ONEWIREIOC_GETFAMILYROMS:
        ret = onewire_ioctl_getfamilyroms(priv->master,
              (FAR struct onewire_availroms_s *)arg,
              g_eeprom_familycodes[priv->devtype]);
        break;
      default:
        ret = -ENOTTY;
    }

  nxrmutex_unlock(&priv->master->devlock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ds2xxx_initialize(FAR struct onewire_dev_s *dev,
                      enum ds2xxx_eeproms_e devtype, FAR char *devname)
{
  FAR struct ds2xxx_dev_s *priv;

  if (devtype < 0 || devtype > EEPROM_DS_COUNT)
    {
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct ds2xxx_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->master = onewire_initialize(dev, CONFIG_1WIRE_EE_DS2XXX_MEMSONBUS);
  if (priv->master == NULL)
    {
      kmm_free(priv);
      return -ENOMEM;
    }

  /* We need extra 5 bytes for the Copy Scratchpad command. */

  priv->helpbuf = kmm_zalloc(HELPBUF_MARGIN +
                             g_eeprom_scratchpad_sizes[devtype]);
  if (priv->helpbuf == NULL)
    {
      kmm_free(priv->master);
      kmm_free(priv);
      return -ENOMEM;
    }

  nxrmutex_init(&priv->master->devlock);
  priv->master->dev = dev;
  priv->devtype     = devtype;

  return register_driver(devname, &g_ds2xxx_fops, 0666, priv);
}

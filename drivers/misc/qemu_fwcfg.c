/****************************************************************************
 * drivers/misc/qemu_fwcfg.c
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

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <sys/endian.h>
#include <sys/types.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/mtd/configdata.h>

#include <nuttx/drivers/qemu_fwcfg.h>

#include "qemu_fwcfg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* QEMU fw_cfg MMIO register offsets (see "qemu,fw-cfg-mmio").  On MMIO the
 * data register occupies the low 8 bytes and the selector the next 2.
 */

#define FW_CFG_REG_DATA        0x00  /* Data register (sequential read)     */
#define FW_CFG_REG_SELECTOR    0x08  /* Selector register (write, BE16)     */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Device state: the MMIO base plus a lock serialising the (inherently
 * stateful) select-then-read sequence.
 */

struct fwcfg_dev_s
{
  uintptr_t base;                    /* MMIO base of the register block     */
  mutex_t   lock;                    /* Serialise select + read sequences   */
};

/* On-wire file directory entry: all integer fields are big-endian */

begin_packed_struct struct fwcfg_wire_file_s
{
  uint32_t size;                        /* File size (big-endian)           */
  uint16_t select;                      /* Selector key (big-endian)        */
  uint16_t reserved;
  char     name[FW_CFG_MAX_FILE_PATH];  /* NUL-terminated name              */
} end_packed_struct;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     fwcfg_open(FAR struct file *filep);
static int     fwcfg_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fwcfg_fops =
{
  fwcfg_open,        /* open  */
  NULL,              /* close */
  NULL,              /* read  */
  NULL,              /* write */
  NULL,              /* seek  */
  fwcfg_ioctl,       /* ioctl */
  NULL,              /* truncate */
  NULL               /* poll  */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fwcfg_select
 *
 * Description:
 *   Point the fw_cfg data register at the given selector key.  The selector
 *   register is big-endian regardless of CPU endianness, and selecting an
 *   item resets its read position to the start.
 *
 ****************************************************************************/

static void fwcfg_select(uintptr_t base, uint16_t key)
{
  *(FAR volatile uint16_t *)(base + FW_CFG_REG_SELECTOR) = htobe16(key);
}

/****************************************************************************
 * Name: fwcfg_read_bytes
 *
 * Description:
 *   Read len bytes from the currently-selected fw_cfg item.  Reading the
 *   data register returns the next byte and auto-advances the position.
 *
 ****************************************************************************/

static void fwcfg_read_bytes(uintptr_t base, FAR void *buf, size_t len)
{
  FAR uint8_t *p = buf;
  size_t i;

  for (i = 0; i < len; i++)
    {
      p[i] = *(FAR volatile uint8_t *)(base + FW_CFG_REG_DATA);
    }
}

/****************************************************************************
 * Name: fwcfg_dir_count
 *
 * Description:
 *   Select the file directory and return the number of entries it holds.
 *   On return the data register is positioned right after the count, ready
 *   to stream the entry array sequentially.
 *
 ****************************************************************************/

static uint32_t fwcfg_dir_count(uintptr_t base)
{
  uint32_t count;

  fwcfg_select(base, FW_CFG_FILE_DIR);
  fwcfg_read_bytes(base, &count, sizeof(count));
  return be32toh(count);
}

/****************************************************************************
 * Name: fwcfg_find
 *
 * Description:
 *   Walk the file directory looking for an entry whose name matches, and
 *   return its selector key and size (both in host byte order).
 *
 ****************************************************************************/

static int fwcfg_find(FAR struct fwcfg_dev_s *dev, FAR const char *name,
                      FAR uint16_t *select, FAR uint32_t *size)
{
  struct fwcfg_wire_file_s wire;
  uint32_t count;
  uint32_t i;

  count = fwcfg_dir_count(dev->base);
  for (i = 0; i < count; i++)
    {
      fwcfg_read_bytes(dev->base, &wire, sizeof(wire));
      if (strncmp(wire.name, name, sizeof(wire.name)) == 0)
        {
          *select = be16toh(wire.select);
          *size   = be32toh(wire.size);
          return OK;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: fwcfg_getconfig
 *
 * Description:
 *   Implement CFGDIOC_GETCONFIG: look up the named item in the file
 *   directory, then read its content into the caller's buffer, truncating
 *   to pdata->len when necessary.
 *
 ****************************************************************************/

static int fwcfg_getconfig(FAR struct fwcfg_dev_s *dev,
                           FAR struct config_data_s *pdata)
{
  uint16_t select;
  uint32_t size;
  int ret;

  ret = fwcfg_find(dev, pdata->name, &select, &size);
  if (ret < 0)
    {
      return ret;
    }

  if (size > pdata->len)
    {
      size = pdata->len;
    }

  /* Selecting the item resets its read position to the start */

  fwcfg_select(dev->base, select);
  fwcfg_read_bytes(dev->base, pdata->configdata, size);
  pdata->len = size;
  return OK;
}

/****************************************************************************
 * Name: fwcfg_findconfig
 *
 * Description:
 *   Implement CFGDIOC_FINDCONFIG: report whether the named item exists.
 *
 ****************************************************************************/

static int fwcfg_findconfig(FAR struct fwcfg_dev_s *dev,
                            FAR struct config_data_s *pdata)
{
  uint16_t select;
  uint32_t size;

  return fwcfg_find(dev, pdata->name, &select, &size);
}

/****************************************************************************
 * Name: fwcfg_enum
 *
 * Description:
 *   Implement CFGDIOC_FIRSTCONFIG (first == true) and CFGDIOC_NEXTCONFIG
 *   (first == false).  The name of the current entry is returned in pdata
 *   and the per-open cursor (stored in filep->f_priv) is advanced.
 *
 ****************************************************************************/

static int fwcfg_enum(FAR struct fwcfg_dev_s *dev, FAR struct file *filep,
                      FAR struct config_data_s *pdata, bool first)
{
  struct fwcfg_wire_file_s wire;
  uint32_t index;
  uint32_t count;
  uint32_t size;
  uint32_t i;

  index = first ? 0 : (uint32_t)(uintptr_t)filep->f_priv;

  count = fwcfg_dir_count(dev->base);
  if (index >= count)
    {
      return -ENOENT;
    }

  /* The directory can only be streamed sequentially, so skip preceding
   * entries before returning the requested one.
   */

  for (i = 0; i <= index; i++)
    {
      fwcfg_read_bytes(dev->base, &wire, sizeof(wire));
    }

  /* Return the entry name together with its content, so the caller can
   * enumerate names and read values in a single ioctl (like NVS).
   */

  strlcpy(pdata->name, wire.name, sizeof(pdata->name));

  size = be32toh(wire.size);
  if (size > pdata->len)
    {
      size = pdata->len;
    }

  fwcfg_select(dev->base, be16toh(wire.select));
  fwcfg_read_bytes(dev->base, pdata->configdata, size);
  pdata->len = size;

  filep->f_priv = (FAR void *)(uintptr_t)(index + 1);
  return OK;
}

/****************************************************************************
 * Name: fwcfg_open
 ****************************************************************************/

static int fwcfg_open(FAR struct file *filep)
{
  /* Reset the per-open enumeration cursor */

  filep->f_priv = (FAR void *)0;
  return OK;
}

/****************************************************************************
 * Name: fwcfg_ioctl
 *
 * Description:
 *   configdata ioctl interface.  The device is read-only, so SETCONFIG and
 *   DELCONFIG report -ENOSYS.  Item identification uses the NAMED variant
 *   of struct config_data_s (CONFIG_MTD_CONFIG_NAMED).
 *
 ****************************************************************************/

static int fwcfg_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct fwcfg_dev_s *dev = filep->f_inode->i_private;
  FAR struct config_data_s *pdata = (FAR struct config_data_s *)arg;
  int ret;

  if (pdata == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case CFGDIOC_GETCONFIG:
        ret = fwcfg_getconfig(dev, pdata);
        break;

      case CFGDIOC_FINDCONFIG:
        ret = fwcfg_findconfig(dev, pdata);
        break;

      case CFGDIOC_FIRSTCONFIG:
        ret = fwcfg_enum(dev, filep, pdata, true);
        break;

      case CFGDIOC_NEXTCONFIG:
        ret = fwcfg_enum(dev, filep, pdata, false);
        break;

      case CFGDIOC_SETCONFIG:
      case CFGDIOC_DELCONFIG:
        ret = -ENOSYS;
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fw_cfg_register
 *
 * Description:
 *   Bind a discovered QEMU fw_cfg MMIO region to a character device node.
 *   See include/nuttx/drivers/qemu_fwcfg.h for the full contract.
 *
 ****************************************************************************/

int fw_cfg_register(uintptr_t base, FAR const char *path)
{
  FAR struct fwcfg_dev_s *dev;
  char sig[FW_CFG_SIG_SIZE];
  int ret;

  if (base == 0 || path == NULL)
    {
      return -EINVAL;
    }

  /* Verify the fw_cfg signature before creating the node */

  fwcfg_select(base, FW_CFG_SIGNATURE);
  fwcfg_read_bytes(base, sig, sizeof(sig));
  if (memcmp(sig, "QEMU", FW_CFG_SIG_SIZE) != 0)
    {
      return -ENODEV;
    }

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->base = base;
  nxmutex_init(&dev->lock);

  ret = register_driver(path, &g_fwcfg_fops, 0444, dev);
  if (ret < 0)
    {
      nxmutex_destroy(&dev->lock);
      kmm_free(dev);
    }

  return ret;
}

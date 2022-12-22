/****************************************************************************
 * drivers/lcd/tda19988.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/video/edid.h>
#include <nuttx/lcd/tda19988.h>

#include "tda19988.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Returned values from tda19988_connected() */

#define DISPLAY_CONNECTED 0
#define DISPLAY_DETACHED  1

/* Number of times to try reading EDID */

#define MAX_READ_ATTEMPTS  100

#define HDMI_CTRL_CEC_ENAMODS            0xff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one TDA19988 driver instance */

struct tda1988_dev_s
{
  /* The contained lower half driver instance */

  FAR const struct tda19988_lower_s *lower;

  /* Upper half driver state */

  mutex_t lock;               /* Assures exclusive access to the driver */
  uint8_t page;               /* Currently selected page */
  uint8_t crefs;              /* Number of open references */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked;              /* True, driver has been unlinked */
#endif
  uint16_t version;           /* TDA19988 version */
  FAR uint8_t *edid;          /* Extended Display Identification Data */
  uint32_t edid_len;          /* Size of EDID */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* General I2C Helpers */

static int     tda19988_getregs(FAR const struct tda19988_i2c_s *dev,
                 uint8_t regaddr, FAR uint8_t *regval, int nregs);
static int     tda19988_putreg(FAR const struct tda19988_i2c_s *dev,
                 uint8_t regaddr, uint8_t regval);
static int     tda19988_putreg16(FAR const struct tda19988_i2c_s *dev,
                 uint8_t regaddr, uint16_t regval);
static int     tda19988_modifyreg(FAR const struct tda19988_i2c_s *dev,
                 uint8_t regaddr, uint8_t clrbits, uint8_t setbits);

/* CEC I2C Helpers */

static inline int tda19988_cec_getregs(FAR struct tda1988_dev_s *priv,
                 uint8_t regaddr, FAR uint8_t *regval, int nregs);
static inline int tda19988_cec_putreg(FAR struct tda1988_dev_s *priv,
                 uint8_t regaddr, uint8_t regval);
static inline int tda19988_cec_modifyreg(FAR struct tda1988_dev_s *priv,
                 uint8_t regaddr, uint8_t clrbits, uint8_t setbits);

/* HDMI I2C Helpers */

static int     tda19988_select_page(FAR struct tda1988_dev_s *priv,
                 uint8_t page);
static int     tda19988_hdmi_getregs(FAR struct tda1988_dev_s *priv,
                 uint16_t reginfo, FAR uint8_t *regval, int nregs);
static int     tda19988_hdmi_putreg(FAR struct tda1988_dev_s *priv,
                 uint16_t reginfo, uint8_t regval);
static int     tda19988_hdmi_putreg16(FAR struct tda1988_dev_s *priv,
                 uint16_t reginfo, uint16_t regval);
static int     tda19988_hdmi_modifyreg(FAR struct tda1988_dev_s *priv,
                 uint16_t reginfo, uint8_t clrbits, uint8_t setbits);

/* CEC Module Helpers */

#if 0 /* Not used */
static int     tda19988_connected(FAR struct tda1988_dev_s *priv);
#endif

/* HDMI Module Helpers */

static int     tda19988_fetch_edid_block(FAR struct tda1988_dev_s *priv,
                 FAR uint8_t *buf, int block);
static int     tda19988_fetch_edid(struct tda1988_dev_s *priv);
static ssize_t tda19988_read_internal(FAR struct tda1988_dev_s *priv,
                 off_t offset, FAR uint8_t *buffer, size_t buflen);

/* Character driver methods */

static int     tda19988_open(FAR struct file *filep);
static int     tda19988_close(FAR struct file *filep);
static ssize_t tda19988_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t tda19988_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static off_t   tda19988_seek(FAR struct file *filep, off_t offset,
                 int whence);
static int     tda19988_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);
static int     tda19988_poll(FAR struct file *filep, FAR struct pollfd *fds,
                 bool setup);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     tda19988_unlink(FAR struct inode *inode);
#endif

/* Initialization */

static int     tda19988_hwinitialize(FAR struct tda1988_dev_s *priv);
static int     tda19988_videomode_internal(FAR struct tda1988_dev_s *priv,
                 FAR const struct videomode_s *mode);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static void    tda19988_shutdown(FAR struct tda1988_dev_s *priv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations tda19988_fops =
{
  tda19988_open,     /* open */
  tda19988_close,    /* close */
  tda19988_read,     /* read */
  tda19988_write,    /* write */
  tda19988_seek,     /* seek */
  tda19988_ioctl,    /* ioctl */
  NULL,              /* truncate */
  tda19988_poll      /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , tda19988_unlink  /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tda19988_getregs
 *
 * Description:
 *   Read the value from one or more TDA19988 CEC or HDMI registers
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int tda19988_getregs(FAR const struct tda19988_i2c_s *dev,
                            uint8_t regaddr, FAR uint8_t *regval, int nregs)
{
  uint8_t buffer[1];
  int ret;

  DEBUGASSERT(dev != NULL && regval != NULL && nregs > 0);

  /* Write the register address and read the register value */

  buffer[0] = regaddr;
  ret = i2c_writeread(dev->i2c, &dev->config, buffer, 1, regval, nregs);
  if (ret < 0)
    {
      lcderr("ERROR: i2c_writeread() failed: %d\n", ret);
      return -1;
    }

  lcdinfo("Write: %02x<-%02x\n", regaddr, *regval);
  lcderrdumpbuffer("Read:", regval, nregs);
  return OK;
}

/****************************************************************************
 * Name: tda19988_putreg
 *
 * Description:
 *   Write an 8-bit value to one TDA19988 CEC or HDMI register
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int tda19988_putreg(FAR const struct tda19988_i2c_s *dev,
                           uint8_t regaddr, uint8_t regval)
{
  uint8_t buffer[2];
  int ret;

  /* Write the register address and the register value */

  buffer[0] = regaddr;
  buffer[1] = regval;

  ret = i2c_write(dev->i2c, &dev->config, buffer, 2);
  if (ret < 0)
    {
      lcderr("ERROR: i2c_write() failed: %d\n", ret);
      return ret;
    }

  lcdinfo("Wrote: %02x<-%02x\n", regaddr, regval);
  return OK;
}

/****************************************************************************
 * Name: tda19988_putreg16
 *
 * Description:
 *   Write a 16-bit value to one TDA19988 CEC or HDMI register
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int tda19988_putreg16(FAR const struct tda19988_i2c_s *dev,
                             uint8_t regaddr, uint16_t regval)
{
  uint8_t buffer[3];
  int ret;

  /* Write the register address and the register value */

  buffer[0] = regaddr;
  buffer[1] = (regval >> 8);
  buffer[2] = (regval & 0xff);

  ret = i2c_write(dev->i2c, &dev->config, buffer, 3);
  if (ret < 0)
    {
      lcderr("ERROR: i2c_write() failed: %d\n", ret);
      return ret;
    }

  lcdinfo("Wrote: 02x<-%04x\n", regaddr, regval);
  return OK;
}

/****************************************************************************
 * Name: tda19988_modifyreg
 *
 * Description:
 *   Modify bits in one TDA19988 CEC or HDMI register
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int tda19988_modifyreg(FAR const struct tda19988_i2c_s *dev,
                              uint8_t regaddr, uint8_t clrbits,
                              uint8_t setbits)
{
  uint8_t regval;
  int ret;

  /* Read the register contents */

  ret = tda19988_getregs(dev, regaddr, &regval, 1);
  if (ret < 0)
    {
      lcderr("ERROR: tda19988_getregs failed: %d\n", ret);
      return ret;
    }

  /* Modify the register content */

  regval &= ~clrbits;
  regval |= setbits;

  /* Write the modified register content */

  ret = tda19988_putreg(dev, regaddr, regval);
  if (ret < 0)
    {
      lcderr("ERROR: tda19988_putreg failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: tda19988_cec_getregs
 *
 * Description:
 *   Read the value from one or more TDA19988 CEC registers
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static inline int tda19988_cec_getregs(FAR struct tda1988_dev_s *priv,
                                       uint8_t regaddr, FAR uint8_t *regval,
                                       int nregs)
{
  return tda19988_getregs(&priv->lower->cec, regaddr, regval, nregs);
}

/****************************************************************************
 * Name: tda19988_cec_putreg
 *
 * Description:
 *   Write a value to one TDA19988 CEC register
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static inline int tda19988_cec_putreg(FAR struct tda1988_dev_s *priv,
                                      uint8_t regaddr, uint8_t regval)
{
  return tda19988_putreg(&priv->lower->cec, regaddr, regval);
}

/****************************************************************************
 * Name: tda19988_cec_modifyreg
 *
 * Description:
 *   Modify bits in one TDA19988 CEC register
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static inline int tda19988_cec_modifyreg(FAR struct tda1988_dev_s *priv,
                                         uint8_t regaddr, uint8_t clrbits,
                                         uint8_t setbits)
{
  return tda19988_modifyreg(&priv->lower->cec, regaddr, clrbits, setbits);
}

/****************************************************************************
 * Name: tda19988_select_page
 *
 * Description:
 *   Select the HDMI page (if not already selected)
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int tda19988_select_page(FAR struct tda1988_dev_s *priv, uint8_t page)
{
  int ret = OK;

  /* Check if we need to select a new page for this transfer */

  if (page != HDMI_NO_PAGE && page != priv->page)
    {
      ret = tda19988_putreg(&priv->lower->hdmi,
                            REGADDR(HDMI_PAGE_SELECT_REG), page);
    }

  return ret;
}

/****************************************************************************
 * Name: tda19988_hdmi_getregs
 *
 * Description:
 *   Read the value from one or more TDA19988 HDMI registers
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int tda19988_hdmi_getregs(FAR struct tda1988_dev_s *priv,
                                 uint16_t reginfo, FAR uint8_t *regval,
                                 int nregs)
{
  uint8_t page    = REGPAGE(reginfo);
  uint8_t regaddr = REGADDR(reginfo);
  int ret;

  DEBUGASSERT(priv != NULL && regval != NULL && nregs > 0);

  /* Select the HDMI page */

  ret = tda19988_select_page(priv, page);
  if (ret < 0)
    {
      lcderr("ERROR: Failed to select page %02x: %d\n", page, ret);
      return ret;
    }

  /* Write the register address and read the register value */

  ret = tda19988_getregs(&priv->lower->hdmi, regaddr, regval, nregs);
  if (ret < 0)
    {
      lcderr("ERROR: tda19988_getregs() failed: %d\n", ret);
      return -1;
    }

  lcdinfo("Read: %02x:%02x->%02x\n", page, regaddr, *regval);
  return OK;
}

/****************************************************************************
 * Name: tda19988_hdmi_putreg
 *
 * Description:
 *   Write an 8-bit value to one TDA19988 HDMI register
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int tda19988_hdmi_putreg(FAR struct tda1988_dev_s *priv,
                                uint16_t reginfo, uint8_t regval)
{
  uint8_t page    = REGPAGE(reginfo);
  uint8_t regaddr = REGADDR(reginfo);
  int ret;

  /* Select the HDMI page */

  ret = tda19988_select_page(priv, page);
  if (ret < 0)
    {
      lcderr("ERROR: tda19988_select_page failed page %02x: %d\n",
             page, ret);
      return ret;
    }

  /* Write the register address and the register value */

  ret = tda19988_putreg(&priv->lower->hdmi, regaddr, regval);
  if (ret < 0)
    {
      lcderr("ERROR: tda19988_putreg() failed: %d\n", ret);
      return ret;
    }

  lcdinfo("Read: %02x:%02x<-%02x\n", page, regaddr, regval);
  return OK;
}

/****************************************************************************
 * Name: tda19988_hdmi_putreg16
 *
 * Description:
 *   Write a 16-bit value to one TDA19988 HDMI register
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int tda19988_hdmi_putreg16(FAR struct tda1988_dev_s *priv,
                                  uint16_t reginfo, uint16_t regval)
{
  uint8_t page    = REGPAGE(reginfo);
  uint8_t regaddr = REGADDR(reginfo);
  int ret;

  /* Select the HDMI page */

  ret = tda19988_select_page(priv, page);
  if (ret < 0)
    {
      lcderr("ERROR: tda19988_select_page failed page %02x: %d\n",
             page, ret);
      return ret;
    }

  /* Write the register address and the register value */

  ret = tda19988_putreg16(&priv->lower->hdmi, regaddr, regval);
  if (ret < 0)
    {
      lcderr("ERROR: tda19988_putreg16() failed: %d\n", ret);
      return ret;
    }

  lcdinfo("Read: %02x:%02x<-%04x\n", page, regaddr, regval);
  return OK;
}

/****************************************************************************
 * Name: tda19988_hdmi_modifyreg
 *
 * Description:
 *   Modify bits in one TDA19988 HDMI register
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int tda19988_hdmi_modifyreg(FAR struct tda1988_dev_s *priv,
                                   uint16_t reginfo, uint8_t clrbits,
                                   uint8_t setbits)
{
  uint8_t page    = REGPAGE(reginfo);
  uint8_t regaddr = REGADDR(reginfo);
  int ret;

  /* Select the HDMI page */

  ret = tda19988_select_page(priv, page);
  if (ret < 0)
    {
      lcderr("ERROR: Failed to select page %02x: %d\n", page, ret);
      return ret;
    }

  /* Read-modify-write the register contents */

  ret = tda19988_modifyreg(&priv->lower->hdmi, regaddr, clrbits, setbits);
  if (ret < 0)
    {
      lcderr("ERROR: tda19988_modifyreg failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: tda19988_connected
 *
 * Description:
 *   Check if a display is connected.
 *
 * Returned Values:
 *   DISPLAY_CONNECTED - A display is connected
 *   DISPLAY_DETACHED  - No display is connected
 *   A negated errno value is returned on any failure.
 *
 ****************************************************************************/

#if 0 /* Not used */
static int tda19988_connected(FAR struct tda1988_dev_s *priv)
{
  uint8_t regval;
  int ret;

  ret = tda19988_cec_getregs(priv, CEC_STATUS_REG, &regval, 1);
  if (ret < 0)
    {
      lcderr("ERROR: tda19988_cec_getregs failed: %d\n", ret);
      return ret;
    }

  if ((regval & CEC_STATUS_CONNECTED) == 0)
    {
      lcdwarn("WARNING:  Display not connected\n");
      return DISPLAY_DETACHED;
    }
  else
    {
      lcdinfo("Display connect\n");
      return DISPLAY_CONNECTED;
    }
}
#endif

/****************************************************************************
 * Name: tda19988_fetch_edid_block
 *
 * Description:
 *   Fetch one EDID block from the DSD.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int tda19988_fetch_edid_block(FAR struct tda1988_dev_s *priv,
                                     FAR uint8_t *buf, int block)
{
  uint8_t data;
  int attempt;
  int ret;

  ret = OK;

  tda19988_hdmi_modifyreg(priv, HDMI_CTRL_INT_REG, 0, HDMI_CTRL_INT_EDID);

  /* Block 0 */

  tda19988_hdmi_putreg(priv, HDMI_EDID_DEV_ADDR_REG, 0xa0);
  tda19988_hdmi_putreg(priv, HDMI_EDID_OFFSET_REG,
                       (block & 1) != 0 ? 128 : 0);
  tda19988_hdmi_putreg(priv, HDMI_EDID_SEGM_ADDR_REG, 0x60);
  tda19988_hdmi_putreg(priv, HDMI_EDID_DDC_SEGM_REG, block >> 1);

  tda19988_hdmi_putreg(priv, HDMI_EDID_REQ_REG, HDMI_EDID_REQ_READ);
  tda19988_hdmi_putreg(priv, HDMI_EDID_REQ_REG, 0);

  data = 0;
  for (attempt = 0; attempt < MAX_READ_ATTEMPTS; attempt++)
    {
      tda19988_hdmi_getregs(priv, HDMI_CTRL_INT_REG, &data, 1);
      if ((data & HDMI_CTRL_INT_EDID) != 0)
        {
          break;
        }
    }

  if (attempt == MAX_READ_ATTEMPTS)
    {
      ret = -ETIMEDOUT;
      goto done;
    }

  if (tda19988_hdmi_getregs(priv, HDMI_EDID_DATA_REG, buf, EDID_LENGTH) != 0)
    {
      ret = -EIO;
      goto done;
    }

done:
  tda19988_hdmi_modifyreg(priv, HDMI_CTRL_INT_REG, HDMI_CTRL_INT_EDID, 0);
  return ret;
}

/****************************************************************************
 * Name: tda19988_fetch_edid
 *
 * Description:
 *   Fetch the EDID block from the DSD.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int tda19988_fetch_edid(struct tda1988_dev_s *priv)
{
  int blocks;
  int ret;

  ret = 0;

  if (priv->version == HDMI_CTRL_REV_TDA19988)
    {
      tda19988_hdmi_modifyreg(priv, HDMI_HDCPOTP_TX4_REG,
                              HDMI_HDCPOTP_TX4_PDRAM, 0);
    }

  ret = tda19988_fetch_edid_block(priv, priv->edid, 0);
  if (ret < 0)
    {
      goto done;
    }

  blocks = priv->edid[EDID_TRAILER_NEXTENSIONS_OFFSET];
  if (blocks > 0)
    {
      FAR uint8_t *edid;
      unsigned int edid_len;
      int i;

      edid_len =  EDID_LENGTH * (blocks + 1);
      edid     = (FAR void *)kmm_realloc(priv->edid, edid_len);

      if (edid == NULL)
        {
          lcderr("ERROR:  Failed to kmm_realloc EDID\n");
          ret = -ENOMEM;
          goto done;
        }

      priv->edid     = edid;
      priv->edid_len = edid_len;

      for (i = 0; i < blocks; i++)
        {
          FAR uint8_t *buf;

          /* TODO: check validity */

          buf = priv->edid + EDID_LENGTH * (i + 1);
          ret = tda19988_fetch_edid_block(priv, buf, i);
          if (ret < 0)
            {
              lcderr("ERROR: tda19988_fetch_edid_block failed: %d\n", ret);
              goto done;
            }
        }
    }

done:
  if (priv->version == HDMI_CTRL_REV_TDA19988)
    {
      tda19988_hdmi_modifyreg(priv, HDMI_HDCPOTP_TX4_REG,
                              0, HDMI_HDCPOTP_TX4_PDRAM);
    }

  return ret;
}

/****************************************************************************
 * Name: tda19988_read_internal
 *
 * Description:
 *   Return the previously read EDID data.
 *
 * Returned Value:
 *   The number of bytes actually read is returned on success; A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

static ssize_t tda19988_read_internal(FAR struct tda1988_dev_s *priv,
                                      off_t offset, FAR uint8_t *buffer,
                                      size_t buflen)
{
  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);

  /* Check if the offset lies outside of the EDID buffer */

  DEBUGASSERT(priv->edid != NULL && priv->edid_len > 0);
  if (offset < 0)
    {
      offset = 0;
    }
  else if (offset >= priv->edid_len)
    {
      return 0;  /* End-of-file */
    }

  /* Clip the number of bytes so that the read region is wholly
   * within the EDID buffer.
   */

  if (offset + buflen > priv->edid_len)
    {
      buflen = priv->edid_len - offset;
    }

  memcpy(buffer, &priv->edid[offset], buflen);
  return (ssize_t)buflen;
}

/****************************************************************************
 * Name: tda19988_open
 *
 * Description:
 *   Standard character driver open method.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int tda19988_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct tda1988_dev_s *priv;
  int ret;

  /* Get the private driver state instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct tda1988_dev_s *)inode->i_private;
  DEBUGASSERT(priv != NULL);

  /* Get exclusive access to the driver instance */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the reference count on the driver instance */

  DEBUGASSERT(priv->crefs != UINT8_MAX);
  priv->crefs++;

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: tda19988_close
 *
 * Description:
 *   Standard character driver cl;ose method.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int tda19988_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct tda1988_dev_s *priv;
  int ret;

  /* Get the private driver state instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct tda1988_dev_s *)inode->i_private;
  DEBUGASSERT(priv != NULL);

  /* Get exclusive access to the driver */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  /* If the count has decremented to zero and the driver has been unlinked,
   * then self-destruct now.
   */

  if (priv->crefs == 0 && priv->unlinked)
    {
      tda19988_shutdown(priv);
      return OK;
    }
#endif

  nxmutex_unlock(&priv->lock);
  return -ENOSYS;
}

/****************************************************************************
 * Name: tda19988_read
 *
 * Description:
 *   Standard character driver read method.
 *
 * Returned Value:
 *   The number of bytes read is returned on success; A negated errno value
 *   is returned on any failure.  End-of-file (zero) is never returned.
 *
 ****************************************************************************/

static ssize_t tda19988_read(FAR struct file *filep, FAR char *buffer,
                             size_t len)
{
  FAR struct inode *inode;
  FAR struct tda1988_dev_s *priv;
  ssize_t nread;
  int ret;

  /* Get the private driver state instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct tda1988_dev_s *)inode->i_private;
  DEBUGASSERT(priv != NULL);

  /* Get exclusive access to the driver */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Return the previously read EDID data */

  nread = tda19988_read_internal(priv, filep->f_pos, (FAR uint8_t *)buffer,
                                 len);
  if (nread > 0)
    {
      filep->f_pos += nread;
    }

  nxmutex_unlock(&priv->lock);
  return nread;
}

/****************************************************************************
 * Name: tda19988_write
 *
 * Description:
 *   Standard character driver write method.
 *
 * Returned Value:
 *   The number of bytes written is returned on success; A negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static ssize_t tda19988_write(FAR struct file *filep, FAR const char *buffer,
                              size_t len)
{
  /* Driver may be opened for write access.  Writing, however, is not
   * supported.
   */

  return -ENOSYS;
}

/****************************************************************************
 * Name: tda19988_seek
 *
 * Description:
 *   Standard character driver poll method.
 *
 * Returned Value:
 *   The current file position is returned on success; A negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

static off_t tda19988_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode;
  FAR struct tda1988_dev_s *priv;
  off_t pos;
  int ret;

  /* Get the private driver state instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct tda1988_dev_s *)inode->i_private;
  DEBUGASSERT(priv != NULL);

  /* Get exclusive access to the driver */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Perform the seek operation */

  pos = filep->f_pos;

  switch (whence)
    {
      case SEEK_CUR:
        pos += offset;
        if (pos > EDID_LENGTH)
          {
            pos = EDID_LENGTH;
          }
        else if (pos < 0)
          {
            pos = 0;
          }

        filep->f_pos = pos;
        break;

      case SEEK_SET:
        pos = offset;
        if (pos > EDID_LENGTH)
          {
            pos = EDID_LENGTH;
          }
        else if (pos < 0)
          {
            pos = 0;
          }

        filep->f_pos = pos;
        break;

      case SEEK_END:
        pos = EDID_LENGTH + offset;
        if (pos > EDID_LENGTH)
          {
            pos = EDID_LENGTH;
          }
        else if (pos < 0)
          {
            pos = 0;
          }

        filep->f_pos = pos;
        break;

      default:

        /* Return EINVAL if the whence argument is invalid */

        pos = (off_t)-EINVAL;
        break;
    }

  nxmutex_unlock(&priv->lock);
  return pos;
}

/****************************************************************************
 * Name: tda19988_ioctl
 *
 * Description:
 *   Standard character driver poll method.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int tda19988_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct tda1988_dev_s *priv;
  int ret;

  /* Get the private driver state instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct tda1988_dev_s *)inode->i_private;
  DEBUGASSERT(priv != NULL);

  /* Get exclusive access to the driver */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle the IOCTL command */

  switch (cmd)
    {
      /* TDA19988_IOC_VIDEOMODE:
       *   Description:  Select the video mode.  This must be done as part
       *                 of the initialization of the driver.  This is
       *                 equivalent to calling tda18899_videomode() within
       *                 the OS.
       *   Argument:     A reference to a videomode_s structure
       *                 instance.
       *   Returns:      None
       */

      case TDA19988_IOC_VIDEOMODE:
        {
          FAR const struct videomode_s *mode =
            (FAR const struct videomode_s *)((uintptr_t)arg);

          if (mode == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              ret = tda19988_videomode_internal(priv, mode);
              if (ret < 0)
                {
                  lcderr("ERROR: tda19988_videomode_internal failed: %d\n",
                         ret);
                }
            }
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: tda19988_poll
 *
 * Description:
 *   Standard character driver poll method.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int tda19988_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup)
{
  FAR struct inode *inode;
  FAR struct tda1988_dev_s *priv;
  int ret;

  /* Get the private driver state instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct tda1988_dev_s *)inode->i_private;
  DEBUGASSERT(priv != NULL);

  /* Get exclusive access to the driver */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      poll_notify(&fds, 1, POLLIN | POLLOUT);
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: tda19988_unlink
 *
 * Description:
 *   Standard character driver unlink method.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int tda19988_unlink(FAR struct inode *inode)
{
  FAR struct tda1988_dev_s *priv;
  int ret;

  /* Get the private driver state instance */

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv = (FAR struct tda1988_dev_s *)inode->i_private;

  /* Get exclusive access to the driver */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Are there open references to the driver data structure? */

  if (priv->crefs <= 0)
    {
      tda19988_shutdown(priv);
      return OK;
    }

  /* No... just mark the driver as unlinked and free the resources when the
   * last client closes their reference to the driver.
   */

  priv->unlinked = true;
  nxmutex_unlock(&priv->lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: tda19988_hwinitialize
 *
 * Description:
 *   Initialize the TDA19988 hardware.
 *
 * Input Parameters:
 *   priv - TDA19988 driver state
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int tda19988_hwinitialize(FAR struct tda1988_dev_s *priv)
{
  uint16_t version;
  uint8_t data;
  int ret;

  tda19988_cec_putreg(priv, CEC_ENAMODS_REG,
                      CEC_ENAMODS_RXSENS | CEC_ENAMODS_HDMI);
  up_udelay(1000);
  tda19988_cec_getregs(priv, CEC_STATUS_REG, &data, 1);

  /* Reset core */

  tda19988_hdmi_modifyreg(priv, HDMI_CTRL_RESET_REG, 0, 3);
  up_udelay(100);
  tda19988_hdmi_modifyreg(priv, HDMI_CTRL_RESET_REG, 3, 0);
  up_udelay(100);

  /* Reset transmitter: */

  tda19988_hdmi_modifyreg(priv, HDMI_CTRL_MAIN_CNTRL0_REG, 0,
                          HDMI_CTRL_MAIN_CNTRL0_SR);
  tda19988_hdmi_modifyreg(priv, HDMI_CTRL_MAIN_CNTRL0_REG,
                          HDMI_CTRL_MAIN_CNTRL0_SR, 0);

  /* PLL registers common configuration */

  tda19988_hdmi_putreg(priv, HDMI_PLL_SERIAL_1_REG, 0x00);
  tda19988_hdmi_putreg(priv, HDMI_PLL_SERIAL_2_REG,
                       HDMI_PLL_SERIAL_2_SRL_NOSC(1));
  tda19988_hdmi_putreg(priv, HDMI_PLL_SERIAL_3_REG, 0x00);
  tda19988_hdmi_putreg(priv, HDMI_PLL_SERIALIZER_REG, 0x00);
  tda19988_hdmi_putreg(priv, HDMI_PLL_BUFFER_OUT_REG, 0x00);
  tda19988_hdmi_putreg(priv, HDMI_PLL_SCG1_REG, 0x00);
  tda19988_hdmi_putreg(priv, HDMI_PLL_SEL_CLK_REG,
                       HDMI_PLL_SEL_CLK_SEL_CLK1 |
                       HDMI_PLL_SEL_CLK_ENA_SC_CLK);
  tda19988_hdmi_putreg(priv, HDMI_PLL_SCGN1_REG, 0xfa);
  tda19988_hdmi_putreg(priv, HDMI_PLL_SCGN2_REG, 0x00);
  tda19988_hdmi_putreg(priv, HDMI_PLL_SCGR1_REG, 0x5b);
  tda19988_hdmi_putreg(priv, HDMI_PLL_SCGR2_REG, 0x00);
  tda19988_hdmi_putreg(priv, HDMI_PLL_SCG2_REG, 0x10);

  /* Write the default value MUX register */

  tda19988_hdmi_putreg(priv, HDMI_CTRL_MUX_VP_VIP_OUT_REG, 0x24);

  tda19988_hdmi_getregs(priv, HDMI_CTRL_REV_LO_REG, &data, 1);
  version  = (uint16_t)data;
  tda19988_hdmi_getregs(priv, HDMI_CTRL_REV_HI_REG, &data, 1);
  version |= ((uint16_t)data << 8);

  /* Clear feature bits */

  priv->version = version & ~0x30;
  switch (priv->version)
    {
    case HDMI_CTRL_REV_TDA19988:
      lcdinfo("TDA19988\n");
      break;

    default:
      lcderr("ERROR: Unknown device: %04x\n", priv->version);
      ret = -ENODEV;
      goto done;
    }

  tda19988_hdmi_putreg(priv, HDMI_CTRL_DDC_CTRL_REG, HDMI_CTRL_DDC_EN);
  tda19988_hdmi_putreg(priv, HDMI_HDCPOTP_TX3_REG, 39);

  tda19988_cec_putreg(priv, CEC_FRO_IM_CLK_CTRL_REG,
                      CEC_FRO_IM_CLK_CTRL_GHOST_DIS |
                      CEC_FRO_IM_CLK_CTRL_IMCLK_SEL);

  ret = tda19988_fetch_edid(priv);
  if (ret < 0)
    {
      lcderr("ERROR:  tda19988_fetch_edid failed: %d\n", ret);
      goto done;
    }

  /* Default values for RGB 4:4:4 mapping */

  tda19988_hdmi_putreg(priv, HDMI_CTRL_VIPCTRL_0_REG, 0x23);
  tda19988_hdmi_putreg(priv, HDMI_CTRL_VIPCTRL_1_REG, 0x01);
  tda19988_hdmi_putreg(priv, HDMI_CTRL_VIPCTRL_2_REG, 0x45);

  ret = OK;

done:
  return ret;
}

/****************************************************************************
 * Name: tda19988_videomode_internal
 *
 * Description:
 *   Initialize the TDA19988 driver to a specified video mode.  This is a
 *   necessary part of the TDA19988 initialization:  A video mode  must be
 *   configured before the driver is usable.
 *
 * Input Parameters:
 *   priv - TDA19988 driver state
 *   mode - The new video mode.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int
  tda19988_videomode_internal(FAR struct tda1988_dev_s *priv,
                              FAR const struct videomode_s *mode)
{
  uint16_t ref_pix;
  uint16_t ref_line;
  uint16_t n_pix;
  uint16_t n_line;
  uint16_t hs_pix_start;
  uint16_t hs_pix_stop;
  uint16_t vs1_pix_start;
  uint16_t vs1_pix_stop;
  uint16_t vs1_line_start;
  uint16_t vs1_line_end;
  uint16_t vs2_pix_start;
  uint16_t vs2_pix_stop;
  uint16_t vs2_line_start;
  uint16_t vs2_line_end;
  uint16_t vwin1_line_start;
  uint16_t vwin1_line_end;
  uint16_t vwin2_line_start;
  uint16_t vwin2_line_end;
  uint16_t de_start;
  uint16_t de_stop;
  uint8_t regval;
  uint8_t div;

  DEBUGASSERT(priv != NULL && mode != NULL);

  n_pix        = mode->htotal;
  n_line       = mode->vtotal;

  hs_pix_stop  = mode->hsync_end - mode->hdisplay;
  hs_pix_start = mode->hsync_start - mode->hdisplay;

  de_stop      = mode->htotal;
  de_start     = mode->htotal - mode->hdisplay;
  ref_pix      = hs_pix_start + 3;

  if (mode->flags & VID_HSKEW)
    {
      ref_pix += mode->hskew;
    }

  if ((mode->flags & VID_INTERLACE) == 0)
    {
      ref_line         = 1 + mode->vsync_start - mode->vdisplay;
      vwin1_line_start = mode->vtotal - mode->vdisplay - 1;
      vwin1_line_end   = vwin1_line_start + mode->vdisplay;

      vs1_pix_start    = vs1_pix_stop = hs_pix_start;
      vs1_line_start   = mode->vsync_start - mode->vdisplay;
      vs1_line_end     = vs1_line_start + mode->vsync_end -
                         mode->vsync_start;

      vwin2_line_start = vwin2_line_end = 0;
      vs2_pix_start    = vs2_pix_stop = 0;
      vs2_line_start   = vs2_line_end = 0;
    }
  else
    {
      ref_line         = 1 + (mode->vsync_start - mode->vdisplay) / 2;
      vwin1_line_start = (mode->vtotal - mode->vdisplay) / 2;
      vwin1_line_end   = vwin1_line_start + mode->vdisplay / 2;

      vs1_pix_start    = vs1_pix_stop = hs_pix_start;
      vs1_line_start   = (mode->vsync_start - mode->vdisplay) / 2;
      vs1_line_end     = vs1_line_start +
                         (mode->vsync_end - mode->vsync_start) / 2;

      vwin2_line_start = vwin1_line_start + mode->vtotal / 2;
      vwin2_line_end   = vwin2_line_start + mode->vdisplay / 2;

      vs2_pix_start    = vs2_pix_stop = hs_pix_start + mode->htotal / 2;
      vs2_line_start   = vs1_line_start + mode->vtotal / 2;
      vs2_line_end     = vs2_line_start +
                         (mode->vsync_end - mode->vsync_start) / 2;
    }

  div = 148500 / mode->dotclock;
  if (div != 0)
    {
      if (--div > 3)
        {
          div = 3;
        }
    }

  /* Set HDMI HDCP mode off */

  tda19988_hdmi_modifyreg(priv, HDMI_CTRL_TBG_CNTRL_1_REG, 0,
                          HDMI_CTRL_TBG_CNTRL_1_DWIN_DIS);
  tda19988_hdmi_modifyreg(priv, HDMI_HDCPOTP_TX33_REG,
                          HDMI_HDCPOTP_TX33_HDMI, 0);
  tda19988_hdmi_putreg(priv, HDMI_AUDIO_ENC_CTRL_REG,
                       HDMI_AUDIO_ENC_CNTRL_DVI_MODE);

  /* No pre-filter or interpreter */

  tda19988_hdmi_putreg(priv, HDMI_CTRL_HVF_CNTRL_0_REG,
                       HDMI_CTRL_HVF_CNTRL_0_INTPOL_BYPASS |
                       HDMI_CTRL_HVF_CNTRL_0_PREFIL_NONE);
  tda19988_hdmi_putreg(priv, HDMI_CTRL_VIPCTRL_5_REG,
                       HDMI_CTRL_VIPCTRL_5_SP_CNT(0));
  tda19988_hdmi_putreg(priv, HDMI_CTRL_VIPCTRL_4_REG,
                       HDMI_CTRL_VIPCTRL_4_BLANKIT_NDE |
                       HDMI_CTRL_VIPCTRL_4_BLC_NONE);

  tda19988_hdmi_modifyreg(priv, HDMI_PLL_SERIAL_3_REG,
                          HDMI_PLL_SERIAL_3_SRL_CCIR, 0);
  tda19988_hdmi_modifyreg(priv, HDMI_PLL_SERIAL_1_REG,
                          HDMI_PLL_SERIAL_1_SRL_MAN_IP, 0);
  tda19988_hdmi_modifyreg(priv, HDMI_PLL_SERIAL_3_REG,
                          HDMI_PLL_SERIAL_3_SRL_DE, 0);
  tda19988_hdmi_putreg(priv, HDMI_PLL_SERIALIZER_REG, 0);
  tda19988_hdmi_putreg(priv, HDMI_CTRL_HVF_CNTRL_1_REG,
                       HDMI_CTRL_HVF_CNTRL_1_VQR_FULL);

  tda19988_hdmi_putreg(priv, HDMI_CTRL_RPT_CNTRL_REG, 0);
  tda19988_hdmi_putreg(priv, HDMI_PLL_SEL_CLK_REG,
                       HDMI_PLL_SEL_CLK_SEL_VRF_CLK(0) |
                       HDMI_PLL_SEL_CLK_SEL_CLK1 |
                       HDMI_PLL_SEL_CLK_ENA_SC_CLK);

  tda19988_hdmi_putreg(priv, HDMI_PLL_SERIAL_2_REG,
                       HDMI_PLL_SERIAL_2_SRL_NOSC(div) |
                       HDMI_PLL_SERIAL_2_SRL_PR(0));

  tda19988_hdmi_modifyreg(priv, HDMI_CTRL_MATCTRL_REG, 0,
                          HDMI_CTRL_MAT_CONTRL_MAT_BP);

  tda19988_hdmi_putreg(priv, HDMI_PLL_ANA_GENERAL_REG, 0x09);

  tda19988_hdmi_modifyreg(priv, HDMI_CTRL_TBG_CNTRL_0_REG,
                          HDMI_CTRL_TBG_CNTRL_0_SYNC_MTHD, 0);

  /* Sync on rising HSYNC/VSYNC */

  regval = HDMI_CTRL_VIPCTRL_3_SYNC_HS;
  if (mode->flags & VID_NHSYNC)
    {
      regval |= HDMI_CTRL_VIPCTRL_3_H_TGL;
    }

  if (mode->flags & VID_NVSYNC)
    {
      regval |= HDMI_CTRL_VIPCTRL_3_V_TGL;
    }

  tda19988_hdmi_putreg(priv, HDMI_CTRL_VIPCTRL_3_REG, regval);

  regval = HDMI_CTRL_TBG_CNTRL_1_TGL_EN;
  if (mode->flags & VID_NHSYNC)
    {
      regval |= HDMI_CTRL_TBG_CNTRL_1_H_TGL;
    }

  if (mode->flags & VID_NVSYNC)
    {
      regval |= HDMI_CTRL_TBG_CNTRL_1_V_TGL;
    }

  tda19988_hdmi_putreg(priv, HDMI_CTRL_TBG_CNTRL_1_REG, regval);

  /* Program timing */

  tda19988_hdmi_putreg(priv, HDMI_CTRL_MUX_VIDFORMAT_REG, 0x00);

  tda19988_hdmi_putreg16(priv, HDMI_CTRL_MUX_REFPIX_MSB_REG, ref_pix);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_MUX_REFLINE_MSB_REG, ref_line);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_MUX_NPIX_MSB_REG, n_pix);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_MUX_NLINE_MSB_REG, n_line);

  tda19988_hdmi_putreg16(priv, HDMI_CTRL_MUX_VS_LINE_STRT_1_MSB_REG,
                         vs1_line_start);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_MUX_VS_PIX_STRT_1_MSB_REG,
                         vs1_pix_start);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_VS_LINE_END_1_MSB_REG,
                         vs1_line_end);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_VS_PIX_END_1_MSB_REG, vs1_pix_stop);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_VS_LINE_STRT_2_MSB_REG,
                         vs2_line_start);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_VS_PIX_STRT_2_MSB_REG,
                         vs2_pix_start);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_VS_LINE_END_2_MSB_REG,
                         vs2_line_end);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_VS_PIX_END_2_MSB_REG, vs2_pix_stop);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_HS_PIX_START_MSB_REG, hs_pix_start);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_HS_PIX_STOP_MSB_REG, hs_pix_stop);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_VWIN_START_1_MSB_REG,
                         vwin1_line_start);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_VWIN_END_1_MSB_REG, vwin1_line_end);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_VWIN_START_2_MSB_REG,
                         vwin2_line_start);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_VWIN_END_2_MSB_REG, vwin2_line_end);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_DE_START_MSB_REG, de_start);
  tda19988_hdmi_putreg16(priv, HDMI_CTRL_DE_STOP_MSB_REG, de_stop);

  if (priv->version == HDMI_CTRL_REV_TDA19988)
    {
      tda19988_hdmi_putreg(priv, HDMI_CTRL_ENABLE_SPACE_REG, 0x00);
    }

  /* Must be last register set */

  tda19988_hdmi_modifyreg(priv, HDMI_CTRL_TBG_CNTRL_0_REG,
                          HDMI_CTRL_TBG_CNTRL_0_SYNC_ONCE, 0);
  return OK;
}

/****************************************************************************
 * Name: tda19988_shutdown
 *
 * Description:
 *   Free resources used by the driver when it has been unlinked.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static void tda19988_shutdown(FAR struct tda1988_dev_s *priv)
{
  /* Detach and disable interrupts */

  if (priv->lower != NULL)  /* May be called before fully initialized */
    {
      DEBUGASSERT(priv->lower->attach != NULL &&
                  priv->lower->enable != NULL);

      priv->lower->attach(priv->lower, NULL, NULL);
      priv->lower->enable(priv->lower, false);
    }

  /* Release resources */

  nxmutex_destroy(&priv->lock);

  /* Free memory */

  if (priv->edid != NULL)
    {
      kmm_free(priv->edid);
    }

  kmm_free(priv);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tda19988_register
 *
 * Description:
 *   Create and register the the TDA19988 driver at 'devpath'
 *
 * Input Parameters:
 *   devpath - The location to register the TDA19988 driver instance.  The
 *             standard location would be a path like /dev/hdmi0.
 *   lower   - The interface to the the TDA19988 lower half driver.
 *
 * Returned Value:
 *   On success, non-NULL handle is returned that may be subsequently used
 *   with tda19988_videomode().  NULL is returned on failure.
 *
 ****************************************************************************/

TDA19988_HANDLE tda19988_register(FAR const char *devpath,
                                  FAR const struct tda19988_lower_s *lower)
{
  FAR struct tda1988_dev_s *priv;
  int ret;

  DEBUGASSERT(devpath != NULL && lower != NULL);

  /* Allocate an instance of the TDA19988 driver */

  priv = (FAR struct tda1988_dev_s *)
    kmm_zalloc(sizeof(struct tda1988_dev_s));
  if (priv == NULL)
    {
      lcderr("ERROR: Failed to allocate device structure\n");
      return NULL;
    }

  /* Assume a single block in EDID */

  priv->edid = (FAR uint8_t *)kmm_malloc(EDID_LENGTH);
  if (priv->edid == NULL)
    {
      lcderr("ERROR: Failed to allocate EDID\n");
      tda19988_shutdown(priv);
      return NULL;
    }

  priv->edid_len = EDID_LENGTH;

  /* Initialize the driver structure */

  priv->lower = lower;
  priv->page  = HDMI_NO_PAGE;

  nxmutex_init(&priv->lock);

  /* Initialize the TDA19988 */

  ret = tda19988_hwinitialize(priv);
  if (ret < 0)
    {
      lcderr("ERROR: tda19988_hwinitialize failed: %d\n", ret);
      tda19988_shutdown(priv);
      return NULL;
    }

  /* Register the driver */

  ret = register_driver(devpath, &tda19988_fops, 0666, NULL);
  if (ret < 0)
    {
      lcderr("ERROR: register_driver() failed: %d\n", ret);
      tda19988_shutdown(priv);
      return NULL;
    }

  return (TDA19988_HANDLE)priv;
}

/****************************************************************************
 * Name: tda19988_videomode
 *
 * Description:
 *   Initialize the TDA19988 driver to a specified video mode.  This is a
 *   necessary part of the TDA19988 initialization:  A video mode  must be
 *   configured before the driver is usable.
 *
 *   NOTE:  This may be done in two ways:  (1) via a call to
 *   tda19988_videomode() from board-specific logic within the OS, or
 *   equivalently (2) using the TDA19988_IOC_VIDEOMODE from application
 *   logic outside of the OS.
 *
 * Input Parameters:
 *   handle - The handle previously returned by tda19988_register().
 *   mode   - The new video mode.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int tda19988_videomode(TDA19988_HANDLE handle,
                       FAR const struct videomode_s *mode)
{
  FAR struct tda1988_dev_s *priv = (FAR struct tda1988_dev_s *)handle;
  int ret;

  DEBUGASSERT(priv != NULL && mode != NULL);

  /* Get exclusive access to the driver */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Defer the heavy lifting to tda19988_videomode_internal() */

  ret = tda19988_videomode_internal(priv, mode);
  if (ret < 0)
    {
      lcderr("ERROR: tda19988_videomode_internal failed: %d\n", ret);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: tda19988_read_edid
 *
 * Description:
 *   Read the EDID (Extended Display Identification Data).
 *
 *   NOTE:  This may be done in two ways:  (1) via a call to
 *   tda19988_read_edid() from board-specific logic within the OS, or
 *   equivalently (2) using a standard read() to read the EDID from
 *   application logic outside of the OS.
 *
 * Input Parameters:
 *   handle - The handle previously returned by tda19988_register().
 *   offset - The offset into the EDID to begin reading (0..127)
 *   buffer - Location in which to return the EDID data
 *   buflen - Size of buffer in bytes
 *
 * Returned Value:
 *   On success, the number of bytes read is returned; a negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

ssize_t tda19988_read_edid(TDA19988_HANDLE handle, off_t offset,
                           FAR uint8_t *buffer, size_t buflen)
{
  FAR struct tda1988_dev_s *priv = (FAR struct tda1988_dev_s *)handle;
  size_t nread;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Get exclusive access to the driver */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Defer the heavy lifting to tda19988_read_internal() */

  nread = tda19988_read_internal(priv, offset, buffer, buflen);
  if (nread < 0)
    {
      lcderr("ERROR: tda19988_read_internal failed: %d\n",
             (int)nread);
    }

  nxmutex_unlock(&priv->lock);
  return nread;
}

/************************************************************************************
 * drivers/mtd/at24xx.c
 * Driver for I2C-based at24cxx EEPROM(at24c32,at24c64,at24c128,at24c256)
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *   History: 0.1 2011-08-20 initial version
 *
 * Derived from drivers/mtd/m25px.c
 *
 *   Copyright (C) 2009-2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/ioctl.h>
#include <nuttx/i2c.h>
#include <nuttx/mtd.h>

#ifdef CONFIG_MTD_AT24XX

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
#ifndef CONFIG_AT24XX_SIZE
#define CONFIG_AT24XX_SIZE     64
#endif
#ifndef CONFIG_AT24XX_ADDR
#define CONFIG_AT24XX_ADDR     0x50
#endif

#if CONFIG_AT24XX_SIZE == 32
#define AT24XX_NPAGES     128
#define AT24XX_PAGESIZE   32
#elif CONFIG_AT24XX_SIZE == 64
#define AT24XX_NPAGES     256
#define AT24XX_PAGESIZE   32
#elif CONFIG_AT24XX_SIZE == 128
#define AT24XX_NPAGES     256
#define AT24XX_PAGESIZE   64
#elif CONFIG_AT24XX_SIZE == 256
#define AT24XX_NPAGES     512
#define AT24XX_PAGESIZE   64
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct at24c_dev_s.
 */

struct at24c_dev_s
{
    struct mtd_dev_s mtd;      /* MTD interface */
    FAR struct i2c_dev_s *dev; /* Saved I2C interface instance */
    uint8_t addr;
    uint16_t pagesize;         /* 32,63 */
    uint16_t npages;           /* 128,256,512,1024 */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* MTD driver methods */

static int at24c_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t at24c_bread(FAR struct mtd_dev_s *dev, off_t startblock,size_t nblocks, FAR uint8_t *buf);
static ssize_t at24c_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,size_t nblocks, FAR const uint8_t *buf);
static ssize_t at24c_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,FAR uint8_t *buffer);
static int at24c_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/
static struct at24c_dev_s g_at24c =
{
    .addr  = CONFIG_AT24XX_ADDR,
    .pagesize = AT24XX_PAGESIZE,
    .npages = AT24XX_NPAGES,
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static int at24c_eraseall(FAR struct at24c_dev_s *priv)
{
    int blocksleft = priv->pagesize;
    int startblock = 0;
    uint8_t buf[66];
    memset(&buf[2],0xff,priv->pagesize);
    I2C_SETADDRESS(priv->dev,priv->addr,7);
    I2C_SETFREQUENCY(priv->dev,100000);
    while (blocksleft-- > 0)
    {
        uint16_t offset=startblock*priv->pagesize;
        buf[1]=offset&0xff;
        buf[0]=(offset>>8)&0xff;

        while (I2C_WRITE(priv->dev,buf,2)<0)
        {
            usleep(1000);
        }
        I2C_WRITE(priv->dev, buf, priv->pagesize+2);
        startblock++;
    }
    return OK;
}

/************************************************************************************
 * Name: at24c_erase
 ************************************************************************************/

static int at24c_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
    /*
     * EEprom need not erase
     */
    return (int)nblocks;
}

/************************************************************************************
 * Name: at24c_bread
 ************************************************************************************/

static ssize_t at24c_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                           FAR uint8_t *buffer)
{
    FAR struct at24c_dev_s *priv = (FAR struct at24c_dev_s *)dev;
    size_t blocksleft = nblocks;

    fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

    if (startblock >= priv->npages)
    {
        return 0;
    }

    if (startblock + nblocks > priv->npages)
    {
        nblocks = priv->npages - startblock;
    }

    I2C_SETADDRESS(priv->dev,priv->addr,7);
    I2C_SETFREQUENCY(priv->dev,100000);
    int tail;
    while (blocksleft-- > 0)
    {
        uint16_t offset=startblock*priv->pagesize;
        uint8_t buf[2];
        buf[1]=offset&0xff;
        buf[0]=(offset>>8)&0xff;
        while (I2C_WRITE(priv->dev,buf,2)<0)
        {
            fvdbg("wait\n");
            usleep(1000);
        }
        I2C_READ(priv->dev, buffer,priv->pagesize);
        startblock++;
        buffer+=priv->pagesize;
    }
    return nblocks;
}

/************************************************************************************
 * Name: at24c_bwrite
 ************************************************************************************/

static ssize_t at24c_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                            FAR const uint8_t *buffer)
{
    FAR struct at24c_dev_s *priv = (FAR struct at24c_dev_s *)dev;
    size_t blocksleft = nblocks;
    uint8_t buf[66];

    if (startblock >= priv->npages)
    {
        return 0;
    }

    if (startblock + nblocks > priv->npages)
    {
        nblocks = priv->npages - startblock;
    }


    fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);
    I2C_SETADDRESS(priv->dev,priv->addr,7);
    I2C_SETFREQUENCY(priv->dev,100000);
    while (blocksleft-- > 0)
    {
        uint16_t offset=startblock*priv->pagesize;
        while (I2C_WRITE(priv->dev,(uint8_t *)&offset,2)<0)
        {
            fvdbg("wait\n");
            usleep(1000);
        }
        buf[1]=offset&0xff;
        buf[0]=(offset>>8)&0xff;
        memcpy(&buf[2],buffer,priv->pagesize);

        I2C_WRITE(priv->dev, buf, priv->pagesize+2);
        startblock++;
        buffer+=priv->pagesize;
    }
    return nblocks;
}

/************************************************************************************
 * Name: at24c_ioctl
 ************************************************************************************/

static int at24c_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
    FAR struct at24c_dev_s *priv = (FAR struct at24c_dev_s *)dev;
    int ret = -EINVAL; /* Assume good command with bad parameters */

    fvdbg("cmd: %d \n", cmd);

    switch (cmd)
    {
    case MTDIOC_GEOMETRY:
    {
        FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)((uintptr_t)arg);
        if (geo)
        {
            /* Populate the geometry structure with information need to know
             * the capacity and how to access the device.
             *
             * NOTE: that the device is treated as though it where just an array
             * of fixed size blocks.  That is most likely not true, but the client
             * will expect the device logic to do whatever is necessary to make it
             * appear so.
             */

            geo->blocksize    = priv->pagesize;
            geo->erasesize    = priv->pagesize;
            geo->neraseblocks = priv->npages;
            ret               = OK;

            fvdbg("blocksize: %d erasesize: %d neraseblocks: %d\n",
                  geo->blocksize, geo->erasesize, geo->neraseblocks);
        }
    }
    break;

    case MTDIOC_BULKERASE:
        ret=at24c_eraseall(priv);
        break;

    case MTDIOC_XIPBASE:
    default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

    return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: at24c_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ************************************************************************************/

FAR struct mtd_dev_s *at24c_initialize(FAR struct i2c_dev_s *dev)
{
    FAR struct at24c_dev_s *priv;
    int ret;

    fvdbg("dev: %p\n", dev);

    /* Allocate a state structure (we allocate the structure instead of using
     * a fixed, static allocation so that we can handle multiple FLASH devices.
     * The current implementation would handle only one FLASH part per SPI
     * device (only because of the SPIDEV_FLASH definition) and so would have
     * to be extended to handle multiple FLASH parts on the same SPI bus.
     */

    priv = &g_at24c;
    if (priv)
    {
        /* Initialize the allocated structure */

        priv->mtd.erase  = at24c_erase;
        priv->mtd.bread  = at24c_bread;
        priv->mtd.bwrite = at24c_bwrite;
        priv->mtd.ioctl  = at24c_ioctl;
        priv->dev        = dev;
    }
    /* Return the implementation-specific state structure as the MTD device */
    fvdbg("Return %p\n", priv);
    return (FAR struct mtd_dev_s *)priv;
}

#endif

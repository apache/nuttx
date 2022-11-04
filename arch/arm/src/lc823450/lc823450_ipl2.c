/****************************************************************************
 * arch/arm/src/lc823450/lc823450_ipl2.c
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
#include <sys/stat.h>
#include <stdio.h>
#include <debug.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <nvic.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_FS_EVFAT
#  include <nuttx/fs/mkevfatfs.h>
#endif

#include <nuttx/usb/usbmsc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/drivers/drivers.h>

#ifdef CONFIG_I2C
#  include <nuttx/i2c.h>
#endif

#ifdef CONFIG_LASTKMSG
#  include <nuttx/lastkmsg.h>
#endif /* CONFIG_LASTKMSG */

#include <nuttx/usb/usbdev.h>

#include <libgen.h>

#include "arm_internal.h"

#ifdef CONFIG_ADC
#  include "lc823450_adc.h"
#endif

#include "lc823450_syscontrol.h"
#include "lc823450_mtd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPL2_DEVPATH    "/dev/mtdblock0"
#define IPL2_IMAGE      "LC8234xx_17S_start_data.boot_bin"

#define LASTMSG_LOGPATH "/log/lastkmsg"

#define R2A20056BM_ADDR       0x1B
#define R2A20056BM_SCL        375000

#ifdef CONFIG_CHARGER
#  define FORCE_USBBOOT_CHARGER
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

static struct
{
  uint32_t sig;
  uint32_t chunknum;
  struct
  {
    char fname[32];
    char csum[32];
    uint32_t size;
    uint32_t enc;
    uint32_t offset;
  }
  chunk[10];
} upg_image;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char copybuf[512];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_USBMSC
static void sysreset(void);
static int set_config(int num, char *buf);
#endif

/****************************************************************************
 * Name: blk_read()
 ****************************************************************************/

static int blk_read(void *buf, int len, const char *path, int offset)
{
  void *handle;
  int ret;

  ret = bchlib_setup(path, true, &handle);

  if (ret)
    {
      return ret;
    }

  ret = bchlib_read(handle, buf, offset, len);

  bchlib_teardown(handle);

  return ret;
}

/****************************************************************************
 * Name: blk_write()
 ****************************************************************************/

#ifdef CONFIG_USBMSC
static int blk_write(const void *buf, int len, const char *path, int offset)
{
  void *handle;
  int ret;

  ret = bchlib_setup(path, true, &handle);

  if (ret)
    {
      return ret;
    }

  ret = bchlib_write(handle, buf, offset, len);

  bchlib_teardown(handle);

  return ret;
}
#endif

/****************************************************************************
 * Name: install_recovery()
 ****************************************************************************/

static int install_recovery(const char *srcpath)
{
  struct file rfile;
  int i;
  int len;
  int rem;
  int ret = 0;
  void *handle = NULL;

  if (bchlib_setup(CONFIG_MTD_RECOVERY_DEVPATH, false, &handle))
    {
      return -1;
    }

  ret = file_open(&rfile, srcpath, O_RDONLY, 0444);

  if (file_read(&rfile, &upg_image, sizeof(upg_image)) != sizeof(upg_image))
    {
      _info("read head");
      ret = -EIO;
      goto err;
    }

#ifdef IMG_SIGNATURE
  if (upg_image.sig != IMG_SIGNATURE)
    {
      _info("image signature mismatch. IPL2=%u, UPG=%u\n",
            IMG_SIGNATURE, upg_image.sig);
      _info("go normal boot\n");

      memset(copybuf, 0, sizeof(copybuf));
      snprintf(copybuf, sizeof(copybuf), "normal");
      set_config(1, copybuf);
      sysreset();

      /* NOT REACHED */
    }
#endif

  for (i = 0; i < upg_image.chunknum; i++)
    {
      if (!strcmp(basename(upg_image.chunk[i].fname), "recovery"))
        {
          break;
        }
    }

  if (i == upg_image.chunknum)
    {
      _info("recovery not found\n");
      ret = -ENOENT;
      goto err;
    }

  file_seek(&rfile, upg_image.chunk[i].offset +
        ((void *)&upg_image.chunk[upg_image.chunknum] - (void *)&upg_image),
        SEEK_SET);

  rem = upg_image.chunk[i].size;

  while (rem > 0)
    {
      len = file_read(&rfile, copybuf, rem > 512 ? 512 : rem);

      if (len < 0)
        {
          _info("read image");
          ret = -EIO;
          goto err;
        }

      bchlib_write(handle, copybuf, upg_image.chunk[i].size - rem, len);
      rem -= len;
    }

err:
  if (handle)
    {
      bchlib_teardown(handle);
    }

  file_close(&rfile);
  _info("DONE\n");
  return ret;
}

/****************************************************************************
 * Name: load_kernel()
 ****************************************************************************/

static void load_kernel(const char *name, const char *devname)
{
  int i;
  void *tmp = (void *)0x02040000;

  blk_read(tmp, 512 * 1024, devname, 0);

  /* disable all IRQ */

  for (i = LC823450_IRQ_NMI + 1; i < NR_IRQS; i++)
    {
      up_disable_irq(i);
    }

  /* clear pending IRQ */

  putreg32(0xffffffff, NVIC_IRQ0_31_CLRPEND);
  putreg32(0xffffffff, NVIC_IRQ32_63_CLRPEND);
  putreg32(0xffffffff, NVIC_IRQ64_95_CLRPEND);

  _info("start %s\n", name);

  __asm__ __volatile__
    (
     "ldr sp, [%0, #0]\n" /* set sp */
     "ldr pc, [%0, #4]"   /* set pc, start nuttx */
     : : "r"(tmp)
     );
}

/****************************************************************************
 * Name: check_diskformat()
 ****************************************************************************/

#ifdef CONFIG_USBMSC
static int check_diskformat(void)
{
  int ret;

#ifdef CONFIG_FS_EVFAT
  struct evfat_format_s fmt = EVFAT_FORMAT_INITIALIZER;

  /* load MBR */

  ret = blk_read(copybuf, sizeof(copybuf), "/dev/mtdblock0p2", 0);

  if (ret < 0)
    {
      return 0;
    }

  /* If part2 has MBR signature, this eMMC was formatted by PC.
   * This means the set is just after writing IPL2.
   */

  if (copybuf[510] != 0x55 || copybuf[511] != 0xaa)
    {
      return 0;
    }

  ret = mkevfatfs(CONFIG_MTD_CP_DEVPATH, &fmt);
#endif

  _info("FORMAT content partition : %d\n", ret);

  memset(copybuf, 0, sizeof(copybuf));
  ret = blk_write(copybuf, 512, CONFIG_MTD_ETC_DEVPATH, 0);
  _info("clear /etc : %d\n", ret);
  ret = blk_write(copybuf, 512, CONFIG_MTD_SYSTEM_DEVPATH, 0);
  _info("clear /system : %d\n", ret);
  ret = blk_write(copybuf, 512, CONFIG_MTD_CACHE_DEVPATH, 0);
  _info("clear /cache : %d\n", ret);

  return 1;
}
#endif

/****************************************************************************
 * Name: check_forceusbboot()
 ****************************************************************************/

#ifdef CONFIG_ADC
static int check_forceusbboot(void)
{
  uint32_t val;
  uint32_t val1;

  /* enable clock & unreset */

  modifyreg32(MCLKCNTAPB, 0, MCLKCNTAPB_ADC_CLKEN);
  modifyreg32(MRSTCNTAPB, 0, MRSTCNTAPB_ADC_RSTB);

  nxsig_usleep(10000);

  /* start ADC0,1 */

  putreg32(ADCCTL_ADCNVCK_DIV32 | ADCCTL_ADACT | ADCCTL_ADCHSCN |
           1 /* 0,1 ch */, ADCCTL);

  putreg32(53, ADCSMPL);

  /* wait for adc done */

  while ((getreg32(ADCSTS) & ADCSTS_ADCMPL) == 0)
    ;

  val = getreg32(ADC0DT);
  val1 = getreg32(ADC1DT);

  _info("val = %d, val1 = %d\n", val, val1);

  /* disable clock & reset */

  modifyreg32(MCLKCNTAPB, MCLKCNTAPB_ADC_CLKEN, 0);
  modifyreg32(MRSTCNTAPB, MRSTCNTAPB_ADC_RSTB, 0);

  /* check KEY0_AD_D key pressed */

  if (val >= (0x3a << 2) && val < (0x57 << 2))
    {
      return 1;
    }

  /* check KEY0_AD_B key pressed */

  if (val >= (0x0b << 2) && val < (0x20 << 2))
    {
      return 1;
    }

  /* check KEY1_AD_B key pressed */

  if (val1 >= (0x0b << 2) && val1 < (0x20 << 2))
    {
      return 1;
    }

  return 0;
}
#endif

#ifdef CONFIG_USBMSC

/****************************************************************************
 * Name: sysreset()
 ****************************************************************************/

static void sysreset(void)
{
  /* workaround to flush eMMC cache */

  nxsig_usleep(100000);

  up_systemreset();
}

/****************************************************************************
 * Name: get_config()
 ****************************************************************************/

static int get_config(int num, char *buf)
{
  int ret;
  ret = blk_read(buf, 512, CONFIG_MTD_CONFIG_DEVPATH, num * 512);
  return ret;
}

/****************************************************************************
 * Name: set_config()
 ****************************************************************************/

static int set_config(int num, char *buf)
{
  int ret;
  ret = blk_write(buf, 512, CONFIG_MTD_CONFIG_DEVPATH, num * 512);
  return ret;
}

#endif /* CONFIG_USBMSC */

extern volatile int g_update_flag;

/****************************************************************************
 * Name: chg_disable()
 ****************************************************************************/

#ifdef CONFIG_CHARGER
static void chg_disable(void)
{
  struct i2c_dev_s *i2c;
  int ret;
  uint32_t freq;

  const uint8_t addr = 0x01;
  const uint8_t data = 0x83;

  struct i2c_msg_s msg[2] =
  {
    {
      .addr   = R2A20056BM_ADDR,
      .flags  = 0,
      .buffer = (uint8_t *)&addr,
      .length = 1,
    },
    {
      .addr   = R2A20056BM_ADDR,
      .flags  = I2C_M_NOSTART,
      .buffer = (uint8_t *)&data,
      .length = 1,
    }
  };

  /* I2C pinmux */

  modifyreg32(PMDCNT0, 0x0003c000, 0x00014000);

  /* I2C drv : 4mA */

  modifyreg32(PTDRVCNT0, 0x0003c000, 0x0003c000);

  /* Enable I2C controller */

  modifyreg32(MCLKCNTAPB, 0, MCLKCNTAPB_I2C0_CLKEN);
  modifyreg32(MRSTCNTAPB, 0, MRSTCNTAPB_I2C0_RSTB);

  /* I2C SCL: push pull */

  modifyreg32(I2CMODE, 0, I2CMODE0);

  /* Disable charge */

  i2c = up_i2cinitialize(1);

  if (i2c)
    {
      /* Set slave address */

      ret = I2C_SETADDRESS(i2c, R2A20056BM_ADDR, 7);

      /* Set frequency  */

      freq = I2C_SETFREQUENCY(i2c, R2A20056BM_SCL);

      /* Charge disable */

      if (ret == OK && freq == R2A20056BM_SCL)
        {
          ret = I2C_TRANSFER(i2c, msg, sizeof(msg) / sizeof(msg[0]));

          if (ret != OK)
            {
              _info("no vbus (%d)\n", ret);
            }
          else
            {
              nxsig_usleep(20);
            }
        }

      up_i2cuninitialize(i2c);
    }
  else
    {
      _info("Failed to i2c initialize\n");
    }
}
#endif

/****************************************************************************
 * Name: msc_enable()
 ****************************************************************************/

#ifdef CONFIG_USBMSC
static int msc_enable(int forced)
{
  int ret;
  void *handle;

  usbmsc_configure(1, &handle);
  usbmsc_bindlun(handle, CONFIG_MTD_CP_DEVPATH, 0, 0, 0, false);
  usbmsc_exportluns(handle);

#ifdef FORCE_USBBOOT_CHARGER
  if (!forced && !usbdev_is_usbcharger())
    {
      /* If not USBCharger, go normal boot */

      usbmsc_uninitialize(handle);
      return 0;
    }

  /* wait for SCSI command */

  while (g_update_flag == 0)
    {
      /* If key released, go normal boot */

      if (!forced && !check_forceusbboot())
        {
          usbmsc_uninitialize(handle);
          return 0;
        }

      nxsig_usleep(10000);
    }

#else
  /* wait for SCSI command */

  while (g_update_flag == 0)
    {
      nxsig_usleep(10000);
    }
#endif

  usbmsc_uninitialize(handle);

  /* check recovery kernel update */

  nx_mount(CONFIG_MTD_CP_DEVPATH, "/mnt/sd0", "evfat", 0, NULL);
  nxsig_usleep(10000);

  /* recovery kernel install from UPG.img */

  ret = install_recovery("/mnt/sd0/UPG.IMG");

  if (ret == 0)
    {
      _info("Install recovery\n");

      /* clear old MBR */

      memset(copybuf, 0, sizeof(copybuf));
      set_config(0, copybuf);
    }

  /* set bootmode to recovery */

  memset(copybuf, 0, sizeof(copybuf));
  snprintf(copybuf, sizeof(copybuf), "recovery");
  set_config(1, copybuf);

  sysreset();

  /* not reached */

  return 0;
}
#endif

/****************************************************************************
 * Name: ipl2_main()
 ****************************************************************************/

int ipl2_main(int argc, char *argv[])
{
  int ret;

  UNUSED(ret); /* Not used in all configurations */

#ifdef CONFIG_CHARGER
  /* NOTE:
   * chg_disable() must be done before CMIC_FWAKE L->H.
   * Please refer to PDFW15IS-2494 for more information
   */

  chg_disable();
#endif

  lc823450_mtd_initialize(0);

#ifdef CONFIG_ADC
  ret = check_forceusbboot();
#endif

#ifdef CONFIG_USBMSC
  if (ret)
    {
      msc_enable(0);
    }

  ret = check_diskformat();

  if (ret)
    {
      msc_enable(1);
    }

  memset(copybuf, 0, sizeof(copybuf));
  get_config(1, copybuf);

  /* for "reboot usb" */

  if (!strncmp("usb", copybuf, 3))
    {
      /* remove boot flag for next boot */

      memset(copybuf, 0, sizeof(copybuf));
      set_config(1, copybuf);
      msc_enable(1);
    }
#endif

#ifdef CONFIG_LASTKMSG
  check_lastkmsg();
#endif /* CONFIG_LASTKMSG */

  if (!strncmp("recovery", copybuf, 8))
    {
      /* check recovery kernel update */

      nx_mount(CONFIG_MTD_CP_DEVPATH, "/mnt/sd0", "evfat", 0, NULL);
      nxsig_usleep(10000);

      /* recovery kernel install from UPG.img */

      install_recovery("/mnt/sd0/UPG.IMG");
      load_kernel("recovery", CONFIG_MTD_RECOVERY_DEVPATH);
    }
  else
    {
      load_kernel("nuttx", CONFIG_MTD_KERNEL_DEVPATH);
    }

  /* not reached */

  return -1;
}

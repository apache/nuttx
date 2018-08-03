/****************************************************************************
 * arch/arm/src/lc823450/lc823450_ipl2.c
 *
 *   Copyright 2015,2016,2017 Sony Video & Sound Products Inc.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
 *   Author: Yasuhiro Osaki <Yasuhiro.Osaki@jp.sony.com>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#ifdef CONFIG_FS_EVFAT
#  include <nuttx/fs/mkevfatfs.h>
#endif

#include <nuttx/usb/usbmsc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/drivers/drivers.h>

#ifdef CONFIG_I2C
#  include <nuttx/i2c.h>
#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <stdio.h>
#include <debug.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <nvic.h>
#include <unistd.h>
#include <errno.h>

#ifdef CONFIG_LASTKMSG
#  include <nuttx/lastkmsg.h>
#endif /* CONFIG_LASTKMSG */

#include <nuttx/usb/usbdev.h>

#include <libgen.h>

#include "up_internal.h"
#include "up_arch.h"

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

#pragma GCC optimize ("O0")

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
  } chunk[10];
} upg_image;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char copybuf[512];
static void *tmp;

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
  int rfd, i, len, rem;
  int ret = 0;
  void *handle = NULL;

  if (bchlib_setup(CONFIG_MTD_RECOVERY_DEVPATH, false, &handle))
    {
      return -1;
    }

  rfd = open(srcpath, O_RDONLY, 0444);

  if (read(rfd, &upg_image, sizeof(upg_image)) != sizeof(upg_image))
    {
      _info("read head");
      ret = -EIO;
      goto err;
    }

#ifdef IMG_SIGNATURE
  if (upg_image.sig != IMG_SIGNATURE)
    {
      _info("image signature missmatch. IPL2=%u, UPG=%u\n",
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

  lseek(rfd, upg_image.chunk[i].offset +
        ((void *)&upg_image.chunk[upg_image.chunknum] - (void *)&upg_image),
        SEEK_SET);

  rem = upg_image.chunk[i].size;

  while (rem > 0)
    {
      len = read(rfd, copybuf, rem > 512 ? 512 : rem);

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

  close(rfd);
  _info("DONE\n");
  return ret;
}

/****************************************************************************
 * Name: load_kernel()
 ****************************************************************************/

static void load_kernel(const char *name, const char *devname)
{
  int i;

  tmp = (void *)0x02040000;

  (void)blk_read(tmp, 512 * 1024, devname, 0);

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
     "ldr r0, =tmp\n"
     "ldr r1, [r0, #0]\n" /* r1 = 0x02040000 */
     "ldr sp, [r1, #0]\n" /* set sp */
     "ldr pc, [r1, #4]"   /* set pc, start nuttx */
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

  /* If part2 has MBR signature, this eMMC was formated by PC.
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

  usleep(10000);

  /* start ADC0,1 */

  putreg32(rADCCTL_fADCNVCK_DIV32 | rADCCTL_fADACT | rADCCTL_fADCHSCN |
           1 /* 0,1 ch */, rADCCTL);

  putreg32(53, rADCSMPL);

  /* wait for adc done */

  while ((getreg32(rADCSTS) & rADCSTS_fADCMPL) == 0)
    ;

  val = getreg32(rADC0DT);
  val1 = getreg32(rADC1DT);

  _info("val = %d, val1 = %d\n", val, val1);

  /* disable clock & reset */

  modifyreg32(MCLKCNTAPB, MCLKCNTAPB_ADC_CLKEN, 0);
  modifyreg32(MRSTCNTAPB, MRSTCNTAPB_ADC_RSTB, 0);

  /* check KEY0_AD_D key pressed */

  if (val >= (0x3A << 2) && val < (0x57 << 2))
    {
      return 1;
    }

  /* check KEY0_AD_B key pressed */

  if (val >= (0x0B << 2) && val < (0x20 << 2))
    {
      return 1;
    }

  /* check KEY1_AD_B key pressed */

  if (val1 >= (0x0B << 2) && val1 < (0x20 << 2))
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

  usleep(100000);

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
  FAR struct i2c_dev_s *i2c;
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

  modifyreg32(PMDCNT0, 0x0003C000, 0x00014000);

  /* I2C drv : 4mA */

  modifyreg32(PTDRVCNT0, 0x0003C000, 0x0003C000);

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
              usleep(20);
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
      usleep(10000);
    }

#else
  /* wait for SCSI command */

  while (g_update_flag == 0)
    {
      usleep(10000);
    }
#endif

  usbmsc_uninitialize(handle);

  /* check recovery kernel update */

  mount(CONFIG_MTD_CP_DEVPATH, "/mnt/sd0", "evfat", 0, NULL);
  usleep(10000);

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

#ifdef CONFIG_LASTKMSG

/****************************************************************************
 * Name: check_lastkmsg()
 ****************************************************************************/

void check_lastkmsg(void)
{
  int ret;
  FILE *fp;

  if (g_lastksg_buf.sig != LASTKMSG_SIG)
    {
      return;
    }

  ret = mount(CONFIG_MTD_LOG_DEVPATH, "/log", "vfat", 0, NULL);

  if (ret)
    {
      _info("mount: ret = %d\n", ret);
      return;
    }

  /* log rotate */

  (void)unlink(LASTMSG_LOGPATH ".4");
  (void)rename(LASTMSG_LOGPATH ".3", LASTMSG_LOGPATH ".4");
  (void)rename(LASTMSG_LOGPATH ".2", LASTMSG_LOGPATH ".3");
  (void)rename(LASTMSG_LOGPATH ".1", LASTMSG_LOGPATH ".2");
  (void)rename(LASTMSG_LOGPATH ".0", LASTMSG_LOGPATH ".1");

  fp = fopen(LASTMSG_LOGPATH ".0", "w");

  if (fp)
    {
      lastkmsg_output(fp);
      fflush(fp);
      fclose(fp);
    }

  umount("/log");

  /* XXX: workaround for logfile size = 0 */

  usleep(100000);
}
#endif /* CONFIG_LASTKMSG */

/****************************************************************************
 * Name: ipl2_main()
 ****************************************************************************/

int ipl2_main(int argc, char *argv[])
{
  int ret;

  UNUSED(ret); /* Not used in all configurations */

  _info("start: %s\n", CONFIG_CURRENT_REVISION);
  _info("imgsig: %u\n", IMG_SIGNATURE);

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

      mount(CONFIG_MTD_CP_DEVPATH, "/mnt/sd0", "evfat", 0, NULL);
      usleep(10000);

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


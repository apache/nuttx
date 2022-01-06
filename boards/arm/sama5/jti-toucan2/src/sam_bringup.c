/****************************************************************************
 * boards/arm/sama5/jti-toucan2/src/sam_bringup.c
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>
#include <stdlib.h>
#include <syslog.h>
#include <debug.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>
#include "arm_arch.h"
#include "hardware/sam_sfr.h"
#include <nuttx/spi/spi.h>
#include "jti-toucan2.h"

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#ifdef CONFIG_NET_CDCECM
#  include <nuttx/usb/cdcecm.h>
#  include <net/if.h>
#endif

#ifdef CONFIG_FUSB302
# include <nuttx/usb/fusb302.h>
#endif

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#endif

#ifdef CONFIG_MMCSD
#  include <nuttx/mmcsd.h>
#  include "sam_sdmmc.h"
#endif

#if defined (CONFIG_SAMA5_MCAN0) || (CONFIG_SAMA5_MCAN1)
# include "sam_mcan.h"
#endif

#ifdef CONFIG_SAMA5D2_CLASSD
# include "sam_classd.h"
#endif

#if defined(CONFIG_MTD_M25P)
# include <nuttx/fs/fs.h>
# include <nuttx/mtd/mtd.h>
# include <nuttx/fs/nxffs.h>
#endif

#if defined(HAVE_AT25)
//# include <nuttx/eeprom/spi_xx25xx.h>
# include <fcntl.h>
#endif

#if defined (HAVE_EGT)
# include <nuttx/sensors/max31855.h>
#endif

#if defined (HAVE_FUSB302) && !defined (CONFIG_SAMA5_TWI0)
#undef HAVE_FUSB302
#warning HAVE_FUSB302 has been undefined as CONFIG_SAMA5_TWI0 not defined
#else
# include <nuttx/usb/fusb302.h>
#endif


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NSECTORS(n) \
  (((n)+CONFIG_SAMA5D4EK_ROMFS_ROMDISK_SECTSIZE-1) / \
   CONFIG_SAMA5D4EK_ROMFS_ROMDISK_SECTSIZE)


/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Public Data
 ****************************************************************************/
#if 0
#ifdef CONFIG_SAMA5_TWI0
FAR struct i2c_master_s *g_i2c0_dev;
#endif
#ifdef CONFIG_SAMA5_TWI1
FAR struct i2c_master_s *g_i2c1_dev;
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/
#if 0
/****************************************************************************
 * Name: sam_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined (CONFIG_SAMA5_TWI0) || defined (CONFIG_SAMA5_TWI1)
static void sam_i2c_register(FAR struct i2c_master_s *i2c, int bus)
{
  ;
  int ret;
  i2c = sam_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      _err("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          _err("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          //sam_i2cbus_uninitialize(i2c);
        }
    }
}
#endif
#endif


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int sam_bringup(void)
{
  int ret;

  /*
  There are issues with the 24MHz crystal as u-boot is not setting 
  the SFR register to indicate the correct crystal frequency
  */
  uint32_t regval;
  /* get UTMI timing register */
  regval = getreg32(SAM_SFR_VBASE + SAM_SFR_UTMICKTRIM_OFFSET);
  regval &= 0xFFFFFFFC;
  regval |= BOARD_CRYSTAL_FREQUENCY;
  putreg32(regval, (SAM_SFR_VBASE + SAM_SFR_UTMICKTRIM_OFFSET));
  _info("INFO: UTMI CLK TRIM register is: %08x\n", regval);

 /* Register I2C drivers */
#if 0
#if defined (CONFIG_SAMA5_TWI0)
  sam_i2c_register(g_i2c0_dev, 0);
#endif

#if defined (CONFIG_SAMA5_TWI1)
#if !defined (CONFIG_SAMA5_TWI0)
    sam_i2c_register(g_i2c0_dev, 0);
#else
    sam_i2c_register(g_i2c1_dev, 1);
#endif
#endif
#endif

#ifdef HAVE_AUTOMOUNTER
  #undef HAVE_AUTOMOUNTER
#endif

#ifdef HAVE_ROMFS
  /* Create a ROM disk for the /etc filesystem */

  ret = romdisk_register(CONFIG_SAMA5D4EK_ROMFS_ROMDISK_MINOR, romfs_img,
                         NSECTORS(romfs_img_len),
                         CONFIG_SAMA5D4EK_ROMFS_ROMDISK_SECTSIZE);
  if (ret < 0)
    {
      _err("ERROR: romdisk_register failed: %d\n", -ret);
    }
  else
    {
      /* Mount the file system */

      ret = nx_mount(CONFIG_SAMA5D4EK_ROMFS_ROMDISK_DEVNAME,
                     CONFIG_SAMA5D4EK_ROMFS_MOUNT_MOUNTPOINT,
                     "romfs", MS_RDONLY, NULL);
      if (ret < 0)
        {
          _err("ERROR: nx_mount(%s,%s,romfs) failed: %d\n",
               CONFIG_SAMA5D4EK_ROMFS_ROMDISK_DEVNAME,
               CONFIG_SAMA5D4EK_ROMFS_MOUNT_MOUNTPOINT, ret);
        }
    }
#endif

#if defined (HAVE_EGT) && defined(CONFIG_SAMA5_FLEXCOM4_SPI)
FAR struct flexcom_spi_dev_s *flexcom_spi_egt;

flexcom_spi_egt = sam_flexcom_spibus_initialize(EGT_PORT);

if (!flexcom_spi_egt)
  {
    syslog(LOG_ERR, "ERROR: Failed to initialize EGT Flexcom SPI port\n");
  }
else
  {
    syslog(LOG_INFO, "Successfully initialized EGT Flexcom SPI port\n");
    /* register the sensor interface */
    if (max31855_register("/dev/temp0", flexcom_spi_egt, 0) < 0)
    {
      syslog(LOG_ERR, "ERROR: Error registering MAX31855\n");
    }
    else
    {
      syslog(LOG_INFO, "Successfully registered MAX31855\n");
    }
  }
#endif  

#if defined(HAVE_AT25) && defined(CONFIG_SAMA5_SPI0)
sam_at25_automount(AT25_PORT);
#if 0
FAR struct spi_dev_s *spi_at25;
spi_at25 = sam_spibus_initialize(AT25_PORT);
if (!spi_at25)
  {
    syslog(LOG_ERR, "ERROR: Failed to initialize AT25 SPI port 1\n");
  }
else
  {
    syslog(LOG_INFO, "Successfully initialized AT25 SPI port\n");
  }
/* Now bind the SPI interface to the AT25  SPI FLASH driver */

ret = ee25xx_initialize(spi_at25, "/dev/AT25", EEPROM_25XX128, O_RDWR);
if (ret<0)
  {
    syslog(LOG_ERR,
            "ERROR: Failed to initialise the AT25 driver\n");

  }
else
  {
    syslog(LOG_INFO, "Successfully initialised the AT25 driver\n");
       
  }
#endif
#endif

#if 0
#if defined(CONFIG_MTD_M25P) && defined(CONFIG_SAMA5_SPI0)
FAR struct spi_dev_s *spi_m25p;
FAR struct mtd_dev_s *mtd_m25p;
spi_m25p = sam_spibus_initialize(M25P_PORT);
if (!spi_m25p)
  {
    syslog(LOG_ERR, "ERROR: Failed to initialize M25P SPI port 0\n");
  }
else
  {
    syslog(LOG_INFO, "Successfully initialized M25P SPI\n");
  /* Now bind the SPI interface to the MT25QL256 SPI FLASH driver */

  mtd_m25p = m25p_initialize(spi_m25p);

  if (!mtd_m25p)
    {
      syslog(LOG_ERR,
            "ERROR: Failed to bind SPI0 CS1 to the SPI FLASH driver\n");
    }
  else
   {
      syslog(LOG_INFO, "Successfully bound SPI0 CS1 to the SPI"
                      " FLASH driver\n");
      if (ret < 0)
        {
          syslog(LOG_ERR, "Failed to initialise the FTL layer: %d\n", ret);
        }
        else
          {
            //syslog(LOG_INFO, "FTL layer successfully initialised\n");    
            //ret = smart_initialize(M25P_MINOR, mtd_m25p, NULL);
            ret = ftl_initialize(0, mtd_m25p);
            if (ret < 0)
              {
                syslog(LOG_ERR, "Failed to initialise the ftl layer %d\n", ret);
              }
            else
              {
                syslog(LOG_INFO, "Successfully initialised ftl layer\n");
                syslog(LOG_INFO, "initializing  file system...be patient\n");
                ret = nxffs_initialize(mtd_m25p);

                //smart_initialize(M25P_MINOR, mtd_m25p, NULL);

                if (ret < 0)
                  {
                    syslog(LOG_ERR, "Failed to initialise file system %d\n", ret);
                  }
                else      
                  {
                    syslog(LOG_INFO, "Successfully initialised file system\n");
                
                    //ret = nx_mount("/dev/smart0", "/mnt", "smartfs",0, NULL);
                    
                    ret = nx_mount(NULL, "/mnt/flash", "nxffs",0, NULL);
                    
                    if (ret < 0)
                      {
                        syslog(LOG_ERR, "Failed to mount the file system %d\n", ret);
                      }
                    else      
                      {
                        syslog(LOG_INFO, "Successfully mounted the file system\n");

                      }
                  }
              }
          }
      }
  }
#endif
#endif

#ifdef HAVE_FUSB302
  //ret = sam_fusb302init(g_i2c0_dev, FUSB302_I2C_ADDR);
  ret = sam_fusb302init(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialise FUSB302 device, %d\n", ret);
    }

#endif



#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  sam_usbhost_initialize() starts a thread
   * will monitor for USB connection and disconnection events.
   */

  ret = sam_usbhost_initialize();
  if (ret != OK)
    {
        _err("ERROR: Failed to initialize USB host: %d\n", ret);
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
        _err("ERROR: Failed to start the USB monitor: %d\n", ret);
    }
  else
  {
    syslog(LOG_INFO, "INFO: started the USB monitor\n");
  }
    
#endif

#ifdef HAVE_MAXTOUCH
  /* Initialize the touchscreen */

  ret = sam_tsc_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_tsc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */
#	 ifdef CONFIG_SAMA5_PWM_CHAN0
	 ret = sam_pwm_setup(0);
   if (ret < 0)
	   {
	     syslog(LOG_ERR, "ERROR: sam_pwm_setup() for PWM0 failed: %d\n", ret);
	   }
#  endif
#	 ifdef CONFIG_SAMA5_PWM_CHAN1
	 /* PWM channel1 not available on the JTi Toucan2 board */
   syslog(LOG_ERR, "ERROR: sam_pwm_setup for PWM1 not allowed on this board\n");
   #if 0   
      ret = sam_pwm_setup(1);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: sam_pwm_setup() for PWM1 failed: %d\n", ret);
        }
   #endif
#  endif
#	 ifdef CONFIG_SAMA5_PWM_CHAN2
	 ret = sam_pwm_setup(2);
   if (ret < 0)
	   {
	     syslog(LOG_ERR, "ERROR: sam_pwm_setup() for PWM2 failed: %d\n", ret);
	   }
#  endif
#	 ifdef CONFIG_SAMA5_PWM_CHAN3
	 ret = sam_pwm_setup(3);
   if (ret < 0)
	   {
	     syslog(LOG_ERR, "ERROR: sam_pwm_setup() for PWM3 failed: %d\n", ret);
	   }
#  endif
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = sam_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef HAVE_AUDIO_NULL
  /* Configure the NULL audio device */

  ret = sam_audio_null_initialize(0);
  if (ret != OK)
    {
      _err("ERROR: Failed to initialize the NULL audio device: %d\n", ret);
    }
#endif

#ifdef HAVE_AUDIO_CLASSD
  /*  configure the ClassD audio device  */
#ifndef HAVE_AUDIO_NULL
  ret = sam_audio_classd_initialize(0);
#else
  ret = sam_audio_classd_initialize(1);
#endif
  if (ret != OK)
    {
      _err("ERROR: Failed to initialize the CLASSD audio device: %d\n", ret);
    }
#endif



#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, SAMA5_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      _err("ERROR: Failed to mount procfs at %s: %d\n",
           SAMA5_PROCFS_MOUNTPOINT, ret);
    }
#endif

#if defined(CONFIG_SAMA5_MCAN0) || defined(CONFIG_SAMA5_MCAN1)
  /* Initialize CAN and register the  driver(s). */
  //sam_configpio(PIO_MCAN0_SILENT_MODE);
  //sam_piowrite(PIO_MCAN0_SILENT_MODE, false);
  ret = sam_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_can_setup failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_USBMSC)
  board_usbmsc_initialize(AT25_PORT);
#endif

#if defined(CONFIG_RNDIS)
  /* Set up a MAC address for the RNDIS device. */

  uint8_t mac[6];
  mac[0] = 0xa0; /* TODO */
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;
  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;
  usbdev_rndis_initialize(mac);
#endif

#ifdef CONFIG_NET_CDCECM
    ret = cdcecm_initialize(0, NULL);
  if (ret < 0)
    {
      _err("ERROR: cdcecm_initialize() failed: %d\n", ret);
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */
   

  UNUSED(ret);
  return OK;
}

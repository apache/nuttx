/*****************************************************************************
 * include/nuttx/sensors/lsm330.h
 *
 *   Copyright (C) 2017-2018 RAF Research LLC. All rights reserved.
 *   Author: Bob Feretich <bob.feretich@rafresearch.com>
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
 *****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_LSM330_H
#define __INCLUDE_NUTTX_SENSORS_LSM330_H

/*****************************************************************************
 * Driver usage notes:
 *
 * This driver is a "kernel sensor leaf driver" that may be used directly
 * from user applications via the file_operations interface or have selected
 * entry points called directly from a "kernel sensor cluster driver".
 *
 * To use this driver via the file_operations interface, the board
 * initialization function should call this driver's registration function.
 * The driver will register itself with Nuttx under the /dev path that is
 * provided by the config structure.  Then user applications may access the
 * driver via the "file descriptor handle" returned by the file_operations
 * open() function.
 *
 * By default the  accelerometer's open() function configures the sensor
 * for:
 *
 *   Output Data Rate (ODR) = 1600 Hz.
 *   Bandwidth (BW) = 800 Hz.
 *   Range = 16g.
 *
 * By default the  gyroscope's open() function configures the sensor for:
 *
 *   Output Data Rate (ODR) = 760 Hz.
 *   Bandwidth (BW) = 100 Hz.
 *   Range = 500 dps.
 *   Low Pass Filter #1 selected.
 *
 * If the user desires different configuration settings, then the user may
 * either provide a pointer to an array of "struct lsm330_reg_pair_s" that
 * will be applied to the sensor upon open(); or dynamically use the lseek()
 * and write() file_operations functions to set the sensor configuration
 * as desired.
 *
 * When using the sensor from the file_operations interface, the sensor is
 * accessed in Programmed I/O (PIO) mode. (i.e. When the read() function is
 * executed, the sensor is read on that thread.) PIO reads and writes block
 * the calling thread until data is available. Since the sensor is on an SPI
 * bus running at near 10 MHz, the read or write operations should only take
 * a few microseconds (about a microsecond per byte of data), so for
 * individual sensor reads and writes, the overhead of using interrupts or
 * DMA is not worthwhile.
 *
 * Use the Cluster Driver Interface to perform DMA block transfers.
 *
 * This driver supports the Common Sensor Register Interface.
 * See drivers/sensors/README.txt for details.
 *
 * This driver supports the Sensor Cluster Driver Interface.
 * See drivers/sensors/README.txt for details.
 *
 * It also extends the interface by permitting cluster driver calls to
 * a function that is intended to perform high performance DMA SPI exchange
 * operations. See the usage note on the exchange operation below.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/cluster_driver.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LSM330 common definitions */

#define LSM330_READ                 0x80
#define LSM330_WRITE                0x00
#define LSM330_GYRO_AUTO            0x40

/****************************************************************************
 * LSM330 Accelerometer Register definitions
 ****************************************************************************/

#define LSM330_ACL_IDREG            0x0f
#define LSM330_ACL_IDREG_VALUE      0x40

#define LSM330_ACL_CTRL_REG2        0x21
#define LSM330_ACR2_HYST1_MASK      0xe0  /* Hysteresis for SM1 bit mask */
#define LSM330_ACR2_HYST1_SHIFT     5     /* Hysteresis for SM1 shift amount */
#define LSM330_ACR2_SM1_PIN         0x08  /* 1= SM1 int on INT2_A; 0= on INT1_A */
#define LSM330_ACR2_SM1_EN          0x01  /* 1= SM1 enabled; 0= disabled */

#define LSM330_ACL_CTRL_REG3        0x22
#define LSM330_ACR3_HYST2_MASK      0xe0  /* Hysteresis for SM2 bit mask */
#define LSM330_ACR3_HYST2_SHIFT     5     /* Hysteresis for SM2 shift amount */
#define LSM330_ACR3_SM2_PIN         0x08  /* 1= SM2 int on INT1_A; 0= on INT1_A */
#define LSM330_ACR3_SM2_EN          0x01  /* 1= SM2 enabled; 0= disabled */

#define LSM330_ACL_CTRL_REG4        0x23
#define LSM330_ACR4_DREN            0x80  /* 1= Data Ready enabled on INT1_A; 0= disabled */
#define LSM330_ACR4_IEA             0x40  /* 1= INT1 polarity active high; 0= low */
#define LSM330_ACR4_IEL             0x20  /* 1= INT1 pulsed; 0= latched */
#define LSM330_ACR4_INT2_EN         0x10  /* 1= INT2_A enabled; 0= disabled */
#define LSM330_ACR4_INT1_EN         0x08  /* 1= INT1_A enabled; 0= disabled */
#define LSM330_ACR4_VFILT           0x40  /* 1= Vector filter enabled; 0= disabled */
#define LSM330_ACR4_STRT            0x01  /* 1= Soft reset. Cleared by HW when done */

#define LSM330_ACL_CTRL_REG5        0x20
#define LSM330_ACR5_ODR_MASK        0xf0  /* ODR bit mask */
#define LSM330_ACR5_ODR_OFF         0x00  /* ODR = powered down */
#define LSM330_ACR5_ODR_3           0x10  /* ODR = 3.125 Hz */
#define LSM330_ACR5_ODR_6           0x20  /* ODR = 6.25 Hz */
#define LSM330_ACR5_ODR_12          0x30  /* ODR = 12.5 Hz */
#define LSM330_ACR5_ODR_25          0x40  /* ODR = 25  Hz  */
#define LSM330_ACR5_ODR_50          0x50  /* ODR = 50  Hz  */
#define LSM330_ACR5_ODR_100         0x60  /* ODR = 100 Hz  */
#define LSM330_ACR5_ODR_400         0x70  /* ODR = 400 Hz  */
#define LSM330_ACR5_ODR_800         0x80  /* ODR = 800 Hz  */
#define LSM330_ACR5_ODR_1600        0x90  /* ODR = 1600 Hz */
#define LSM330_ACR5_BDU             0x08  /* 1= MSB not updated until LSB is read */
#define LSM330_ACR5_ZEN             0x04  /* 1= Z-axis sensor enabled */
#define LSM330_ACR5_YEN             0x02  /* 1= Y-axis sensor enabled  */
#define LSM330_ACR5_XEN             0x01  /* 1= X-axis sensor enabled  */

#define LSM330_ACL_CTRL_REG6        0x24
#define LSM330_ACR6_BW_MASK         0xc0  /* Bandwidth bit mask */
#define LSM330_ACR6_BW_800          0x00  /* Bandwidth = 800 Hz */
#define LSM330_ACR6_BW_200          0x40  /* Bandwidth = 200 Hz */
#define LSM330_ACR6_BW_400          0x80  /* Bandwidth = 400 Hz */
#define LSM330_ACR6_BW_50           0xc0  /* Bandwidth = 50 Hz  */
#define LSM330_ACR6_FS_MASK         0x38  /* Full Scale bit mask */
#define LSM330_ACR6_FS_2            0x00  /* FScale = 2g  */
#define LSM330_ACR6_FS_4            0x08  /* FScale = 4g  */
#define LSM330_ACR6_FS_6            0x10  /* FScale = 6g  */
#define LSM330_ACR6_FS_8            0x18  /* FScale = 8g  */
#define LSM330_ACR6_FS_16           0x20  /* FScale = 16g */
#define LSM330_ACR6_SIM             0x01  /* 1= 3-wire SPI; 0= 4-wire SPI */

#define LSM330_ACL_CTRL_REG7        0x25
#define LSM330_ACR7_BOOT            0x80  /* Force reboot, cleared by HW when done */
#define LSM330_ACR7_FIFO_EN         0x40  /* 1= FIFO enabled; 0= disabled */
#define LSM330_ACR7_WTM_EN          0x20  /* 1= FIFO watermark enabled; 0= disabled */
#define LSM330_ACR7_ADD_INC         0x10  /* 1= auto post-increment ACL addresses */
#define LSM330_ACR7_P1_EMPTY        0x08  /* 1= enable FIFO empty on INT1_A */
#define LSM330_ACR7_P1_WTM          0x04  /* 1= enable FIFO watermark on INT1_A */
#define LSM330_ACR7_P1_OVERRUN      0x02  /* 1= enable FIFO overrun on INT1_A */
#define LSM330_ACR7_P2_BOOT         0x01  /* 1= enable BOOT interrupt on INT2_A */

#define LSM330_ACL_STATUS           0x27
#define LSM330_ASR_ZYXOR            0x80  /* 1= At least one of the axes has overrun */
#define LSM330_ASR_ZOR              0x40  /* 1= Z-axis has overrun and lost data */
#define LSM330_ASR_YOR              0x20  /* 1= Y-axis has overrun and lost data */
#define LSM330_ASR_XOR              0x10  /* 1= X-axis has overrun and lost data */
#define LSM330_ASR_ZYXDA            0x08  /* 1= All of the axes have new data */
#define LSM330_ASR_ZDA              0x04  /* 1= New Z-axis data is available */
#define LSM330_ASR_YDA              0x02  /* 1= New Y-axis data is available */
#define LSM330_ASR_XDA              0x01  /* 1= New X-axis data is available */

#define LSM330_ACL_OUT_X_L          0x28
#define LSM330_ACL_OUT_X_H          0x29
#define LSM330_ACL_OUT_Y_L          0x2a
#define LSM330_ACL_OUT_Y_H          0x2b
#define LSM330_ACL_OUT_Z_L          0x2c
#define LSM330_ACL_OUT_Z_H          0x2d
#define LSM330_ACL_THRS1_1          0x57
#define LSM330_ACL_LAST             0x7f
#define LSM330_ACL_SCRATCH          LSM330_ACL_THRS1_1

/* LSM330 Gyroscope Register definitions */

#define LSM330_GYRO_IDREG           0x0f
#   define LSM330_GYRO_IDREG_VALUE  0xd4
#define LSM330_GYRO_CTRL_REG1       0x20
#define LSM330_GYRO_CTRL_REG2       0x21
#define LSM330_GYRO_CTRL_REG3       0x22
#define LSM330_GYRO_CTRL_REG4       0x23
#define LSM330_GYRO_CTRL_REG5       0x24
#   define LSM_GYRO_BOOT_MASK       0x80
#define LSM330_GYRO_OUT_TEMP        0x26
#define LSM330_GYRO_STATUS_REG      0x27
#define LSM330_GYRO_OUT_X_L         0x28
#define LSM330_GYRO_OUT_X_H         0x29
#define LSM330_GYRO_OUT_Y_L         0x2a
#define LSM330_GYRO_OUT_Y_H         0x2b
#define LSM330_GYRO_OUT_Z_L         0x2c
#define LSM330_GYRO_OUT_Z_H         0x2d
#define LSM330_GYRO_INT1_THS_ZL     0x37
#define LSM330_GYRO_LAST            0x3f
#define LSM330_GYRO_SCRATCH         LSM330_GYRO_INT1_THS_ZL

/* SPI Bus Parameters */

#define LSM330_SPI_FREQUENCY       (10000000)     /* 10 MHz */
#define LSM330_SPI_MODE            (SPIDEV_MODE3) /* SPI Mode 3: CPOL=1,CPHA=1 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the LSM330
 * driver. This structure provides information about the configuration
 * of the sensor and provides some board-specific hooks.
 *
 * This sensor driver presents two interfaces, the POSIX character driver
 * interface (fops) that is intended for use from a user application, and
 * a set of direct call entry points that are intended to be used by
 * a sensor cluster driver that is running as a kernel task (a driver to
 * driver interface).  Application tasks should not attempt to call sensor
 * cluster driver entry points.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct lsm330_reg_pair_s  /* Utility struct for the below... */
{
  uint8_t addr;           /* SPI register address */
  uint8_t value;          /* Value to be stored in the above reg on open() */
};

struct lsm330spi_dvr_entry_vector_s
{
  struct sensor_cluster_operations_s c;

  /* Extend the sensor cluster driver interface with a SPI DMA exchange
   * transfer.  The standard driver_read and driver_write perform PIO
   * transfers.  The will loop waiting on the SPI hardware and are only
   * appropriate for short data transfers.
   *
   * Note that the first byte in the tx buffer must be a command/address
   * byte. The exchange function does not provide one. Also note that
   * the first byte stored in the rxbuffer is a garbage byte, which
   * is natural for a SPI exchange transfer. Plan your buffer accordingly.
   */

  CODE void (*driver_spiexc)(FAR void *instance_handle,
                             FAR const void *txbuffer,
                             FAR void *rxbuffer, size_t nwords);
};

struct lsm330_config_s
{
  /* Since multiple LSM330 can be connected to the same SPI bus we need
   * to use multiple spi device ids which are employed by NuttX to select/
   * deselect the desired LSM330 chip via their chip select inputs.
   */

  int spi_devid;

  /* Initial control register configuration values. */

   uint16_t initial_cr_values_size;     /* size of the below array.
                                         * 0 = use default values. */

  /* The initial value store operations will occur in the order they appear
   * in the array.
   */

   struct lsm330_reg_pair_s *initial_cr_values;

  /* The below fields are intended for the sensor cluster driver interface
   * and may be ignored when the sensor cluster driver is not being used.
   * The leaf driver's registration function fills in the below fields.
   */

  /* Leaf sensor handle (opaque) for sensor cluster kernel driver */

  FAR void *leaf_handle;

  /* Pointer to the leaf driver's sensor_cluster_operations_s structure */

  FAR const struct lsm330spi_dvr_entry_vector_s *sc_ops;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/*******************************************************************************
 * Name: lsm330_register
 *
 * Description:
 *   Register the LSM330 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath_acl  - The full path to the driver to register. E.g., "/dev/acl0"
 *   devpath_gyro - The full path to the driver to register. E.g., "/dev/gyr0"
 *   spi - An instance of the SPI interface to use to communicate with LSM330
 *   config_acl   - configuration for the LSM330 accelerometer driver.
 *                  For details see description above.
 *   config_gyro  - configuration for the LSM330 gyroscope driver.
 *                  For details see description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 *******************************************************************************
 */

int lsm330_register(FAR const char *devpath_acl,
                    FAR const char *devpath_gyro,
                    FAR struct spi_dev_s *spi,
                    FAR struct lsm330_config_s *config_acl,
                    FAR struct lsm330_config_s *config_gyro);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_LSM330_H */

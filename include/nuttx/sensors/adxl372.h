/******************************************************************************
 * include/nuttx/sensors/adxl372.h
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
 ******************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_ADXL372_H
#define __INCLUDE_NUTTX_SENSORS_ADXL372_H

/******************************************************************************
 * Driver usage notes:
 *
 * This driver is a "kernel sensor leaf driver" that may be used directly from
 * user applications via the file_operations interface or have selected entry
 * points called directly from a "kernel sensor cluster driver".
 *
 * To use this driver via the file_operations interface, the board
 * initialization function should call this driver's registration function.
 * The driver will register itself with Nuttx under the /dev path that is
 * provided by the config structure.  Then user applications may access the
 * driver via the "file descriptor handle" returned by the file_operations
 * open() function.
 *
 * By default the open() function configures the sensor for:
 *
 *   Output Data Rate (ODR) = 1600 Hz.
 *   Bandwidth (BW) = 800 Hz.
 *   Normal mode sampling (as opposed to low power mode sampling).
 *   The Low Pass Filter is enabled and the High Pass Filter is disabled.
 *   A filter settling time of 370ms is selected.
 *
 * If the user desires a different configuration settings, the the user may
 * either provide a pointer to an array of "struct adxl372_reg_pair_s" that
 * will be applied to to the sensor upon open(); or dynamically use
 * the lseek() and write() file_operations functions to set the
 * sensor configuration as desired.
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

/*******************************************************************************
 * Included Files
 *******************************************************************************
 */

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/cluster_driver.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_ADXL372) \
    && defined(CONFIG_SPI_EXCHANGE)

/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************
 */

/* ADXL372 common definitions */

#define ADXL372_READ                    0x01
#define ADXL372_WRITE                   0x00
#define ADXL372_ADDR_MASK               0x0fe

/* ADXL372 Accelerometer Register definitions */

#define ADXL372_DEVID_AD                (0x00 << 1)
#   define ADXL372_DEVID_AD_VALUE       0xad
#define ADXL372_DEVID_MST               (0x01 << 1)
#   define ADXL372_DEVID_MST_VALUE      0x1D
#define ADXL372_PARTID                  (0x02 << 1)
#   define ADXL372_PARTID_VALUE         0xfa
#define ADXL372_REVID                   (0x03 << 1)
#define ADXL372_STATUS                  (0x04 << 1)
#define ADXL372_STATUS2                 (0x05 << 1)
#define ADXL372_FIFO_ENTRIES2           (0x06 << 1)
#define ADXL372_FIFO_ENTRIES            (0x07 << 1)
#define ADXL372_XDATA_H                 (0x08 << 1)
#define ADXL372_XDATA_L                 (0x09 << 1)
#define ADXL372_YDATA_H                 (0x0a << 1)
#define ADXL372_YDATA_L                 (0x0b << 1)
#define ADXL372_ZDATA_H                 (0x0c << 1)
#define ADXL372_ZDATA_L                 (0x0d << 1)
#define ADXL372_THRESH_ACT_X_H          (0x23 << 1)
#define ADXL372_FIFO_CTL                (0x3a << 1)
#   define ADXL372_FIFO_BYPASSED        0x00
#   define ADXL372_FIFO_STREAMED        0x02
#define ADXL372_INT1_MAP                (0x3b << 1)
#   define ADXL372_INT1_MAP_DR          0x01
#   define ADXL372_INT1_MAP_FRDY        0x02
#   define ADXL372_INT1_MAP_FFULL       0x04
#define ADXL372_TIMING                  (0x3d << 1)
#   define ADXL372_TIMING_ODR400        (0x0 << 5)      /* 400 Hz ODR */
#   define ADXL372_TIMING_ODR800        (0x1 << 5)      /* 800 Hz ODR */
#   define ADXL372_TIMING_ODR1600       (0x2 << 5)      /* 1600 Hz ODR */
#   define ADXL372_TIMING_ODR3200       (0x3 << 5)      /* 3200 Hz ODR */
#   define ADXL372_TIMING_ODR6400       (0x4 << 5)      /* 6400 Hz ODR */
#define ADXL372_MEASURE                 (0x3e << 1)
#   define ADXL372_MEAS_BW200           0x0     /* 200 Hz Bandwidth */
#   define ADXL372_MEAS_BW400           0x1     /* 400 Hz Bandwidth */
#   define ADXL372_MEAS_BW800           0x2     /* 800 Hz Bandwidth */
#   define ADXL372_MEAS_BW1600          0x3     /* 1600 Hz Bandwidth */
#   define ADXL372_MEAS_BW3200          0x4     /* 3200 Hz Bandwidth */
#define ADXL372_POWER_CTL               (0x3f << 1)
#   define ADXL372_POWER_LPF_DISABLE    (1 << 3)
#   define ADXL372_POWER_HPF_DISABLE    (1 << 2)
#   define ADXL372_POWER_MODE_STANDBY   0x0
#   define ADXL372_POWER_MODE_WAKEUP    0x1
#   define ADXL372_POWER_MODE_INSTON    0x2
#   define ADXL372_POWER_MODE_MEASURE   0x3
#define ADXL372_RESET                   (0x41 << 1)
#   define ADXL372_RESET_VALUE          0x52
#define ADXL372_FIFO_DATA               (0x42 << 1)
#define ADXL372_LAST                    (0x42 << 1)
#define ADXL372_SCRATCH                 ADXL372_THRESH_ACT_X_H

/* SPI Bus Parameters */

#define ADXL372_SPI_FREQUENCY           (10000000)     /* 10 MHz */
#define ADXL372_SPI_MODE                (SPIDEV_MODE0) /* SPI Mode 0: CPOL=0,CPHA=0 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the ADXL372
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

struct adxl372_reg_pair_s  /* Utility struct for the below... */
{
  uint8_t addr;            /* SPI register address */
  uint8_t value;           /* Value to be stored in the above reg on open() */
};

struct adxl372_dvr_entry_vector_s
{
  struct sensor_cluster_operations_s c;

  /* Extend the sensor cluster driver interface with a SPI DMA exchange transfer.
   * The standard driver_read and driver_write perform PIO transfers.
   * The will loop waiting on the SPI hardware and are only appropriate for
   * short data transfers.
   * Note that the first byte in the tx buffer must be a command/address
   * byte. The exchange function does not provide one. Also note that
   * the first byte stored in the rxbuffer is a garbage byte, which
   * is natural for a SPI exchange transfer. Plan your buffer accordingly.
   */

  CODE void (*driver_spiexc)(FAR void *instance_handle,
                             FAR const void *txbuffer,
                             FAR void *rxbuffer, size_t nwords);
};

struct adxl372_config_s
{
  /* Since multiple ADXL372 can be connected to the same SPI bus we need
   * to use multiple SPI device ids which are employed by NuttX to select/
   * deselect the desired ADXL372 chip via their chip select inputs.
   */

  int spi_devid;

  /* Initial control register configuration values. */

  uint16_t initial_cr_values_size;  /* size of the below array.
                                     * 0 = use default values. */

  /* The initial value store operations will occur in the order they
   * appear in the array.
   */

  struct adxl372_reg_pair_s *initial_cr_values;

  /* The below fields are intended for the sensor cluster driver interface
   * and may be ignored when the sensor cluster driver is not being used.
   * The leaf driver's registration function fills in the below fields.
   */

  /* Leaf sensor handle for sensor cluster kernel driver */

  FAR struct spi_dev_s *leaf_handle;

  /* Pointer to the leaf driver's sensor_cluster_operations_s structure */

  FAR const struct adxl372_dvr_entry_vector_s *sc_ops;
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

/****************************************************************************
 * Name: adxl372_register
 *
 * Description:
 *   Register the ADXL372 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/acl0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             ADXL372
 *   config  - configuration for the ADXL372 accelerometer driver.  For
 *             details see description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int adxl372_register(FAR const char *devpath,
                     FAR struct spi_dev_s *spi,
                     FAR struct adxl372_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_SENSORS_ADXL372 && CONFIG_SPI_EXCHANGE */
#endif /* __INCLUDE_NUTTX_SENSORS_ADXL372_H */

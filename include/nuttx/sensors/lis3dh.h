/****************************************************************************
 * include/nuttx/sensors/lis3dh.h
 *
 *   Copyright (C) 2018 Extent3D. All rights reserved.
 *   Author: Matt Thompson <matt@extent3d.com>
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

#ifndef __INCLUDE_NUTTX_SENSORS_LIS3DH_H
#define __INCLUDE_NUTTX_SENSORS_LIS3DH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>

#if defined(CONFIG_SPI) && defined(CONFIG_LIS3DH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LIS3DH Device Identification *********************************************/

#define LIS3DH_DEVICE_ID      (0x33)

/* LIS3DH Register Definitions **********************************************/

#define LIS3DH_STATUS_REG_AUX (0x07)
#define LIS3DH_OUT_ADC1_L     (0x08)
#define LIS3DH_OUT_ADC1_H     (0x09)
#define LIS3DH_OUT_ADC2_L     (0x0A)
#define LIS3DH_OUT_ADC2_H     (0x0B)
#define LIS3DH_OUT_ADC3_L     (0x0C)
#define LIS3DH_OUT_ADC3_H     (0x0D)
#define LIS3DH_WHO_AM_I       (0x0F)
#define LIS3DH_CTRL_REG0      (0x1E)
#define LIS3DH_TEMP_CFG_REG   (0x1F)
#define LIS3DH_CTRL_REG1      (0x20)
#define LIS3DH_CTRL_REG2      (0x21)
#define LIS3DH_CTRL_REG3      (0x22)
#define LIS3DH_CTRL_REG4      (0x23)
#define LIS3DH_CTRL_REG5      (0x24)
#define LIS3DH_CTRL_REG6      (0x25)
#define LIS3DH_REFERENCE      (0x26)
#define LIS3DH_STATUS_REG     (0x27)
#define LIS3DH_OUT_X_L        (0x28)
#define LIS3DH_OUT_X_H        (0x29)
#define LIS3DH_OUT_Y_L        (0x2A)
#define LIS3DH_OUT_Y_H        (0x2B)
#define LIS3DH_OUT_Z_L        (0x2C)
#define LIS3DH_OUT_Z_H        (0x2D)
#define LIS3DH_FIFO_CTRL_REG  (0x2E)
#define LIS3DH_FIFO_SRC_REG   (0x2F)
#define LIS3DH_INT1_CFG       (0x30)
#define LIS3DH_INT1_SRC       (0x31)
#define LIS3DH_INT1_THS       (0x32)
#define LIS3DH_INT1_DURATION  (0x33)
#define LIS3DH_INT2_CFG       (0x34)
#define LIS3DH_INT2_SRC       (0x35)
#define LIS3DH_INT2_THS       (0x36)
#define LIS3DH_INT2_DURATION  (0x37)
#define LIS3DH_CLICK_CFG      (0x38)
#define LIS3DH_CLICK_SRC      (0x39)
#define LIS3DH_CLICK_THS      (0x3A)
#define LIS3DH_TIME_LIMIT     (0x3B)
#define LIS3DH_TIME_LATENCY   (0x3C)
#define LIS3DH_TIME_WINDOW    (0x3D)
#define LIS3DH_ACT_THS        (0x3E)
#define LIS3DH_ACT_DUR        (0x3F)

/* LIS3DH STATUS_REG_AUX Definitions **********************************************/

#define LIS3DH_STATUS_REG_AUX_321OR       (1 << 7)
#define LIS3DH_STATUS_REG_AUX_3OR         (1 << 6)
#define LIS3DH_STATUS_REG_AUX_2OR         (1 << 5)
#define LIS3DH_STATUS_REG_AUX_1OR         (1 << 4)
#define LIS3DH_STATUS_REG_AUX_321DA       (1 << 3)
#define LIS3DH_STATUS_REG_AUX_3DA         (1 << 2)
#define LIS3DH_STATUS_REG_AUX_2DA         (1 << 1)
#define LIS3DH_STATUS_REG_AUX_1DA         (1 << 0)

/* LIS3DH CTRL_REG0 Definitions **********************************************/

#define LIS3DH_CTRL_REG0_SDO_PU_DISC      (1 << 7)  /* Disconnect SDO/SA0 pull-up */

/* LIS3DH TEMP_CFG_REG Definitions **********************************************/

#define LIS3DH_TEMP_CFG_REG_ADC_EN        (1 << 7)  /* ADC enable */
#define LIS3DH_TEMP_CFG_REG_TEMP_EN       (1 << 6)  /* Temperator sensor enable */

/* LIS3DH CTRL_REG1 Definitions **********************************************/

#define LIS3DH_CTRL_REG1_ODR_SHIFT        (4)
#define LIS3DH_CTRL_REG1_ODR_MASK         (0xf << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR(n)           ((n) << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_POWER_DOWN   (0)
#define LIS3DH_CTRL_REG1_ODR_1HZ          (0x1 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_10HZ         (0x2 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_25HZ         (0x3 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_50HZ         (0x4 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_100HZ        (0x5 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_200HZ        (0x6 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_400HZ        (0x7 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_LP_1600HZ    (0x8 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_1344HZ       (0x9 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_LP_5376HZ    (0x9 << LIS3DH_CTRL_REG1_ODR_SHIFT)

#define LIS3DH_CTRL_REG1_LPEN             (1 << 3)  /* Low-power mode enable */
#define LIS3DH_CTRL_REG1_ZEN              (1 << 2)  /* Z axis enable */
#define LIS3DH_CTRL_REG1_YEN              (1 << 1)  /* Y axis enable */
#define LIS3DH_CTRL_REG1_XEN              (1 << 0)  /* X axis enable */

/* LIS3DH CTRL_REG2 Definitions **********************************************/

/* LIS3DH CTRL_REG3 Definitions **********************************************/

#define LIS3DH_CTRL_REG3_I1_CLICK         (1 << 6) /* Click interrupt on INT1 */
#define LIS3DH_CTRL_REG3_I1_IA1           (1 << 6) /* IA1 interrupt on INT1 */
#define LIS3DH_CTRL_REG3_I1_IA2           (1 << 5) /* IA2 interrupt on INT1 */
#define LIS3DH_CTRL_REG3_I1_ZYXDA         (1 << 4) /* ZYX data available interrupt on INT1 */
#define LIS3DH_CTRL_REG3_I1_321DA         (1 << 3) /* 321 data available interrupt on INT1 */
#define LIS3DH_CTRL_REG3_I1_WTM           (1 << 2) /* FIFO watermark interrupt on INT1 */
#define LIS3DH_CTRL_REG3_I1_OVERRUN       (1 << 1) /* FIFO overrun interrupt on INT1 */

/* LIS3DH CTRL_REG4 Definitions **********************************************/

#define LIS3DH_CTRL_REG4_BDU              (1 << 7) /* Block data update */
#define LIS3DH_CTRL_REG4_BLE              (1 << 6) /* Endian selection. 0: LSB first, 1: MSB first */
#define LIS3DH_CTRL_REG4_FS_16G           (3 << 4) /* 16g full scale range */
#define LIS3DH_CTRL_REG4_FS_8G            (2 << 4) /* 8g full scale range */
#define LIS3DH_CTRL_REG4_FS_4G            (1 << 4) /* 4g full scale range */
#define LIS3DH_CTRL_REG4_FS_2G            (0 << 4) /* 2g full scale range */
#define LIS3DH_CTRL_REG4_HR               (1 << 3) /* High resolution output enable */
#define LIS3DH_CTRL_REG4_ST1              (2 << 1) /* Self test 1 */
#define LIS3DH_CTRL_REG4_ST0              (1 << 1) /* Self test 0 */
#define LIS3DH_CTRL_REG4_SIM              (1 << 0) /* SPI serial interface mode selection (0: 4-wire, 1: 3-wire) */

/* LIS3DH CTRL_REG5 Definitions **********************************************/

#define LIS3DH_CTRL_REG5_BOOT             (1 << 7) /* Reboot memory content */
#define LIS3DH_CTRL_REG5_FIFO_EN          (1 << 6) /* FIFO enable */
#define LIS3DH_CTRL_REG5_LIR_INT1         (1 << 3) /* Latch interrupt request on INT1_SRC register */
#define LIS3DH_CTRL_REG5_D4D_INT1         (1 << 2) /* 4D detection enable on INT1 */
#define LIS3DH_CTRL_REG5_LIR_INT2         (1 << 1) /* Latch interrupt request on INT2_SRC register */
#define LIS3DH_CTRL_REG5_D4D_INT2         (1 << 0) /* 4D detection enable on INT2 */

/* LIS3DH CTRL_REG6 Definitions **********************************************/

#define LIS3DH_CTRL_REG6_I2_CLICK         (1 << 6) /* Click interrupt on INT2 */
#define LIS3DH_CTRL_REG6_I2_IA1           (1 << 6) /* IA1 interrupt on INT2 */
#define LIS3DH_CTRL_REG6_I2_IA2           (1 << 5) /* IA2 interrupt on INT2 */
#define LIS3DH_CTRL_REG6_I2_BOOT          (1 << 4) /* Enable boot on INT2 */
#define LIS3DH_CTRL_REG6_I2_ACT           (1 << 3) /* Enable activity interrupt on INT2 */
#define LIS3DH_CTRL_REG6_INT_POLARITY     (1 << 1) /* INT1 and INT2 pin polarity */

/* LIS3DH STATUS_REG Definitions **********************************************/

#define LIS3DH_STATUS_ZYXOR               (1 << 7) /* X,Y,Z axis data overrun */
#define LIS3DH_STATUS_ZOR                 (1 << 6) /* Z axis data overrun */
#define LIS3DH_STATUS_YOR                 (1 << 5) /* Y axis data overrun */
#define LIS3DH_STATUS_XOR                 (1 << 4) /* X axis data overrun */
#define LIS3DH_STATUS_REG_ZYXDA           (1 << 3) /* X,Y,Z axis data available */
#define LIS3DH_STATUS_REG_ZDA             (1 << 2) /* Z axis data available */
#define LIS3DH_STATUS_REG_YDA             (1 << 1) /* Y axis data available */
#define LIS3DH_STATUS_REG_XDA             (1 << 0) /* X axis data available */

/* LIS3DH FIFO_CTRL_REG Definitions **********************************************/

#define LIS3DH_FIFO_CTRL_REG_MODE_STREAM2 (3 << 6)
#define LIS3DH_FIFO_CTRL_REG_MODE_STREAM  (2 << 6)
#define LIS3DH_FIFO_CTRL_REG_MODE_FIFO    (1 << 6)
#define LIS3DH_FIFO_CTRL_REG_MODE_BYPASS  (0 << 6)
#define LIS3DH_FIFO_CTRL_REG_TR           (1 << 5)

/* LIS3DH FIFO_SRC_REG Definitions **********************************************/

#define LIS3DH_FIFO_SRC_REG_WTM           (1 << 7)
#define LIS3DH_FIFO_SRC_REG_OVRN_FIFO     (1 << 6)
#define LIS3DH_FIFO_SRC_REG_EMPTY         (1 << 5)

/* SPI parameters ***************************************************************/

#define LIS3DH_SPI_FREQUENCY    (9600000)        /* SPI Clock Frequency */
#define LIS3DH_SPI_MODE         (SPIDEV_MODE3)   /* Device uses SPI Mode 3: CPOL=1, CPHA=1 */

/* Power Modes ******************************************************************/

#define LIS3DH_POWER_LOW        (0x0) /* Lower power 8bit output */
#define LIS3DH_POWER_NORMAL     (0x1) /* Normal 10bit */
#define LIS3DH_POWER_HIGH       (0x2) /* HR 12bit mode */

/* Output Data Rates  ***********************************************************/

#define LIS3DH_ODR_POWER_DOWN   (0)   /* Disable output */
#define LIS3DH_ODR_1HZ          (0x1) /* 1Hz in all power modes */
#define LIS3DH_ODR_10HZ         (0x2) /* 10Hz in all power modes */
#define LIS3DH_ODR_25HZ         (0x3) /* 25Hz in all power modes */
#define LIS3DH_ODR_50HZ         (0x4) /* 50Hz in all power modes */
#define LIS3DH_ODR_100HZ        (0x5) /* 100Hz in all power modes */
#define LIS3DH_ODR_200HZ        (0x6) /* 200Hz in all power modes */
#define LIS3DH_ODR_400HZ        (0x7) /* 400Hz in all power modes */
#define LIS3DH_ODR_LP_1600HZ    (0x8) /* 1.6kHz in low power mode only */
#define LIS3DH_ODR_1344HZ       (0x9) /* 1.344kHz in normal and high power modes only */
#define LIS3DH_ODR_LP_5376HZ    (0x9) /* 5.376kHz in low power mode only */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct lis3dh_sensor_data_s
{
  float x_acc;                       /* X axis acceleration */
  float y_acc;                       /* Y axis acceleration */
  float z_acc;                       /* Z axis acceleration */
};

/* Configuration structure used to register the driver */

struct lis3dh_config_s
{
  /* SPI device ID used to select the CS line of the sensor */

  int spi_devid;

  /* IRQ number associated with this driver instance */

  int irq;

  /* Attach callback used to configure the interrupt line */

  int (*irq_attach)(FAR struct lis3dh_config_s *, xcpt_t, void *arg);
  int (*irq_detach)(FAR struct lis3dh_config_s *);
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
 * Name: lis3dh_register
 *
 * Description:
 *   Register the LIS3DH character device at the specified device path
 *
 * Input Parameters:
 *   devpath - Full path of device node to register ie "/dev/accel0"
 *   spi     - SPI bus device instance
 *   config  - Driver instance configuration structure
 *
 * Returned Value:
 *   OK on success or a negative errno value on failure.
 *
 ****************************************************************************/

int lis3dh_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                    FAR struct lis3dh_config_s *);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_LIS3DH */
#endif /* __INCLUDE_NUTTX_SENSORS_LIS3DH_H */

/********************************************************************************************
 * include/nuttx/input/adxl345.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_INPUT_ADXL345_H
#define __INCLUDE_NUTTX_INPUT_ADXL345_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <nuttx/i2c.h>
#include <nuttx/spi/spi.h>

#include <nuttx/irq.h>

#if defined(CONFIG_SENSORS_ADXL345)

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* Configuration ****************************************************************************/
/* Prerequisites:
 *
 * CONFIG_SCHED_WORKQUEUE - Work queue support is required
 *
 * Settings that effect the driver: CONFIG_DISABLE_POLL
 *
 * CONFIG_SENSORS_ADXL345
 *   Enables support for the ADXL345 driver
 * CONFIG_ADXL345_SPI
 *   Enables support for the SPI interface (not currenly supported)
 * CONFIG_ADXL345_I2C
 *   Enables support for the I2C interface
 * CONFIG_ADXL345_ACTIVELOW
 *    The ADXL345 interrupt will be inverted. Instead starting low and
 *    going high, it will start high and will go low when an interrupt
 *    is fired. Default:  Active high/rising edge.
 * CONFIG_ADXL345_REGDEBUG
 *   Enable very low register-level debug output.  Requires CONFIG_DEBUG.
 */

#ifdef CONFIG_DISABLE_SIGNALS
#  error "Signals are required.  CONFIG_DISABLE_SIGNALS must not be selected."
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Work queue support required.  CONFIG_SCHED_WORKQUEUE must be selected."
#endif

/* The ADXL345 interfaces with the target CPU via a I2C or SPI interface. The pin IN_1
 * allows the selection of interface protocol at reset state.
 */

#if !defined(CONFIG_ADXL345_SPI) && !defined(CONFIG_ADXL345_I2C)
#  error "One of CONFIG_ADXL345_SPI or CONFIG_ADXL345_I2C must be defined"
#endif

#if defined(CONFIG_ADXL345_SPI) && defined(CONFIG_ADXL345_I2C)
#  error "Only one of CONFIG_ADXL345_SPI or CONFIG_ADXL345_I2C can be defined"
#endif

/* Check for some required settings.  This can save the user a lot of time
 * in getting the right configuration.
 */

#ifdef CONFIG_ADXL345_I2C
#  ifndef CONFIG_I2C
#    error "CONFIG_I2C is required in the I2C support"
#  endif
#  ifndef CONFIG_I2C_TRANSFER
#    error "CONFIG_I2C_TRANSFER is required in the I2C configuration"
#  endif
#endif

/* I2C **************************************************************************************/
/* ADXL345 Address:  The ADXL345 may have 7-bit address 0x41 or 0x44, depending upon the
 * state of the ADDR0 pin.
 */

#define ADXL345_I2C_ADDRESS_MASK    (0x78)       /* Bits 3-7: Invariant part of ADXL345 address */
#define ADXL345_I2C_ADDRESS         (0x40)       /* Bits 3-7: Always set at '0100 0xxR' */
#define ADXL345_I2C_A1              (1 << 2)     /* Bit 2: A1 */
#define ADXL345_I2C_A0              (1 << 1)     /* Bit 1: A0 */
#define ADXL345_I2C_READ            (1 << 0)     /* Bit 0=1: Selects read operation */
#define ADXL345_I2C_WRITE           (0)          /* Bit 0=0: Selects write operation */

/* I2C frequency */

#define ADXL345_I2C_MAXFREQUENCY    400000       /* 400KHz */

/* SPI **************************************************************************************/
/* The device always operates in mode 0 */

#define ADXL345_SPI_MODE            SPIDEV_MODE0 /* Mode 0 */

/* SPI frequency */

#define ADXL345_SPI_MAXFREQUENCY    500000       /* 5MHz */

/* ADXL345 Registers ************************************************************************/
/* Register Addresses */

#define ADXL345_DEVID               0x00  /* Device identification (8-bit) */
                                          /* 0x01 to 0x1C Reserved*/
#define ADXL345_THRESH_TAP          0x1D  /* Tap threshold */
#define ADXL345_OFSX                0x1E  /* X-axis offset */
#define ADXL345_OFSY                0x1F  /* Y-axis offset */
#define ADXL345_OFSZ                0x20  /* Z-axis offset */
#define ADXL345_DUR                 0x21  /* Tap duration */
#define ADXL345_LATENT              0x22  /* Tap latency */
#define ADXL345_WINDOW              0x23  /* Tap window */
#define ADXL345_THRESH_ACT          0x24  /* Activity threshold */
#define ADXL345_THRESH_INACT        0x25  /* Inactivity threshold */
#define ADXL345_TIME_INACT          0x26  /* Inactivity time */
#define ADXL345_ATC_INACT_CTL       0x27  /* Axis enable control for activity and inactivity detection */
#define ADXL345_THRESH_FF           0x28  /* Free-fall threshold */
#define ADXL345_TIME_FF             0x29  /* Free-fall fime */
#define ADXL345_TAP_AXES            0x2A  /* Axis control for tap/double tap */
#define ADXL345_ACT_TAP_STATUS      0x2B  /* Source of tap/double tap */
#define ADXL345_BW_RATE             0x2C  /* Data rate and power mode control */
#define ADXL345_POWER_CTL           0x2D  /* Power-saving features control */
#define ADXL345_INT_ENABLE          0x2E  /* Interrupt enable control */
#define ADXL345_INT_MAP             0x2F  /* Interrupt mapping control */
#define ADXL345_INT_SOURCE          0x30  /* Source of interrupts */
#define ADXL345_DATA_FORMAT         0x31  /* Data format control */
#define ADXL345_DATAX0              0x32  /* X-axis Data 0 */
#define ADXL345_DATAX1              0x33  /* X-axis Data 1 */
#define ADXL345_DATAY0              0x34  /* Y-axis Data 0 */
#define ADXL345_DATAY1              0x35  /* Y-axis Data 1 */
#define ADXL345_DATAZ0              0x36  /* Z-axis Data 0 */
#define ADXL345_DATAZ1              0x37  /* Z-axis Data 1 */
#define ADXL345_FIFO_CTL            0x38  /* FIFO Control */
#define ADXL345_FIFO_STATUS         0x39  /* FIFO Status */

/* Register bit definitions */

/* Device identification (8-bit) */

#define DEVID                      0xE5

/* Register 0x27 - ACT_INACT_CTL */

#define INACT_Z_ENABLE               (1 << 0)  /* Bit 0: Include/Exclude Z-axis in detecting inactivity */
#define INACT_Y_ENABLE               (1 << 1)  /* Bit 1: Include/Exclude Y-axis in detecting inactivity */
#define INACT_X_ENABLE               (1 << 2)  /* Bit 2: Include/Exclude X-axis in detecting inactivity */
#define INACT_AC_DC                  (1 << 3)  /* Bit 3: 0 = DC-coupled operation / 1 = AC-coupled operation */
#define ACT_Z_ENABLE                 (1 << 4)  /* Bit 4: Include/Exclude Z-axis in detecting activity */
#define ACT_Y_ENABLE                 (1 << 5)  /* Bit 5: Include/Exclude Z-axis in detecting activity */
#define ACT_X_ENABLE                 (1 << 6)  /* Bit 6: Include/Exclude Z-axis in detecting activity */
#define ACT_AC_DC                    (1 << 7)  /* Bit 7: 0 = DC-coupled operation / 1 = AC-coupled operation */

/* Register 0x2A - TAP AXES */

#define TAP_Z_ENABLE                 (1 << 0)  /* Bit 0: Enable/disable Z-axis in tap detection */
#define TAP_Y_ENABLE                 (1 << 1)  /* Bit 1: Enable/disable Y-axis in tap detection */
#define TAP_X_ENABLE                 (1 << 2)  /* Bit 2: Enable/disable X-axis in tap detection */
#define TAP_SUPRESS                  (1 << 3)  /* Bit 3: Supress double tap detection */

/* Register 0x2B - ACT_TAP_STATUS */

#define TAP_Z_SOURCE                 (1 << 0)  /* Bit 0: Indicates Z-axis is involved in a tap event */
#define TAP_Y_SOURCE                 (1 << 1)  /* Bit 1: Indicates Y-axis is involved in a tap event */
#define TAP_X_SOURCE                 (1 << 2)  /* Bit 2: Indicates X-axis is involved in a tap event */
#define ASLEEP_STATUS                (1 << 3)  /* Bit 3: Indicates if device is asleep */
#define ACT_Z_SOURCE                 (1 << 4)  /* Bit 4: Indicates Z-axis is involved in an activity event */
#define ACT_Y_SOURCE                 (1 << 5)  /* Bit 5: Indicates Y-axis is involved in an activity event */
#define ACT_X_SOURCE                 (1 << 6)  /* Bit 6: Indicates X-axis is involved in an activity event */

/* Register 0x2C - BW_RATE */

#define BWR_RATE_SHIFT               0  /* Bit 0-3: Rate bits: up to 3200Hz output data rate */
#define BWR_RATE_MASK                (15 << RATE_SHIFT)  /* Bit 0: Master interrupt enable */
#define BWR_LOW_POWER                (1 << 4)  /* Bit 4: Set low power operation */

/* Register 0x2D - POWER_CTL */

#define POWER_CTL_WAKEUP_SHIFT       0  /* Bit 0-1: Controls frequency of reading in sleep mode*/
#define POWER_CTL_WAKEUP_MASK        (3 << POWER_CTL_WAKEUP_SHIFT)
#define POWER_CTL_WAKEUP_8HZ         (0 << POWER_CTL_WAKEUP_SHIFT)
#define POWER_CTL_WAKEUP_4HZ         (1 << POWER_CTL_WAKEUP_SHIFT)
#define POWER_CTL_WAKEUP_2HZ         (2 << POWER_CTL_WAKEUP_SHIFT)
#define POWER_CTL_WAKEUP_1HZ         (3 << POWER_CTL_WAKEUP_SHIFT)
#define POWER_CTL_SLEEP              (1 << 2)  /* Bit 2: Sleep mode, only activity function can be used */
#define POWER_CTL_MEASURE            (1 << 3)  /* Bit 3: Writing 0 put part in standy mode, 1 measurement mode */
#define POWER_CTL_AUTO_SLEEP         (1 << 4)  /* Bit 4: If set and link bit is set then device sleep if no activity */
#define POWER_CTL_LINK               (1 << 5)  /* Bit 5: Wait an inactivity before detecting an activity */

/* Register 0x2E - INT_ENABLE */

#define INT_OVERRUN                  (1 << 0)  /* Bit 0: Enable Overrun detection */
#define INT_WATERMARK                (1 << 1)  /* Bit 1: Enable Watermark detection */
#define INT_FREE_FALL                (1 << 2)  /* Bit 2: Enable Free-fall detection */
#define INT_INACTIVITY               (1 << 3)  /* Bit 3: Enable Inactivity detection*/
#define INT_ACTIVITY                 (1 << 4)  /* Bit 4: Enable Activity detection */
#define INT_DOUBLE_TAP               (1 << 5)  /* Bit 5: Enable Double tap detection */
#define INT_SINGLE_TAP               (1 << 6)  /* Bit 6: Enable Single tap detection */
#define INT_DATA_READY               (1 << 7)  /* Bit 7: Enable Data Ready detection */

/* Register 0x2F - INT_MAP */

#define INT_MAP_OVERRUN              (1 << 0)  /* Bit 0: Map Overrun interrupt 0 = INT1 / 1 = INT2 */
#define INT_MAP_WATERMARK            (1 << 1)  /* Bit 1: Map Watermark interrupt 0 = INT1 / 1 = INT2 */
#define INT_MAP_FREE_FALL            (1 << 2)  /* Bit 2: Map Free-fall interrupt 0 = INT1 / 1 = INT2 */
#define INT_MAP_INACTIVITY           (1 << 3)  /* Bit 3: Map Inactivity interrupt 0 = INT1 / 1 = INT2 */
#define INT_MAP_ACTIVITY             (1 << 4)  /* Bit 4: Map Activity interrupt 0 = INT1 / 1 = INT2 */
#define INT_MAP_DOUBLE_TAP           (1 << 5)  /* Bit 5: Map Double tap interrupt 0 = INT1 / 1 = INT2 */
#define INT_MAP_SINGLE_TAP           (1 << 6)  /* Bit 6: Map Single tap interrupt 0 = INT1 / 1 = INT2 */
#define INT_MAP_DATA_READY           (1 << 7)  /* Bit 7: Map Data Ready interrupt 0 = INT1 / 1 = INT2 */

/* Register 0x30 - INT_SOURCE */

#define INT_SRC_OVERRUN              (1 << 0)  /* Bit 0: Overrun happened, even if INT_ENABLE is disabled */
#define INT_SRC_WATERMARK            (1 << 1)  /* Bit 1: Watermark happened, even if INT_ENABLE is disabled*/
#define INT_SRC_FREE_FALL            (1 << 2)  /* Bit 2: Free-fall detected */
#define INT_SRC_INACTIVITY           (1 << 3)  /* Bit 3: Inactivity detected */
#define INT_SRC_ACTIVITY             (1 << 4)  /* Bit 4: Activity detected */
#define INT_SRC_DOUBLE_TAP           (1 << 5)  /* Bit 5: Double tap detected */
#define INT_SRC_SINGLE_TAP           (1 << 6)  /* Bit 6: Single tap detected */
#define INT_SRC_DATA_READY           (1 << 7)  /* Bit 7: Data Ready happened, even if INT_ENABLE is disabled */

/* Register 0x31 - DATA_FORMAT */

#define DATA_FMT_RANGE_SHIFT         0 /* Bits 0-1: G-force Range */
#define DATA_FMT_RANGE_MASK          (3 << DATA_FMT_RANGE_SHIFT)
#define DATA_FMT_RANGE_2G            (0 << DATA_FMT_RANGE_SHIFT)
#define DATA_FMT_RANGE_4G            (1 << DATA_FMT_RANGE_SHIFT)
#define DATA_FMT_RANGE_8G            (2 << DATA_FMT_RANGE_SHIFT)
#define DATA_FMT_RANGE_16G           (3 << DATA_FMT_RANGE_SHIFT)
#define DATA_FMT_JUSTIFY             (1 << 2)  /* Bit 2: Value 1 selects left justified mode, 0 selects right just. */
#define DATA_FMT_FULL_RES            (1 << 3)  /* Bit 3: Value 0 sets 10-bit mode, value 1 sets 13-bit mode */
#define DATA_FMT_INT_INVERT          (1 << 5)  /* Bit 5: Value 0 interrupt active high, 1 sets interrupt active low */
#define DATA_FMT_SPI                 (1 << 6)  /* Bit 6: Value 1 set 3-Wire SPI mode, 0 set 4-wire SPI mode */
#define DATA_FMT_SELF_TEST           (1 << 7)  /* Bit 7: Apply a SELF_TEST force to the sensor */

/* Register 0x38 - FIFO_CTL */

#define FIFO_CTL_SAMPLES_SHIFT       0         /* Bit 0-4: Numbers of samples needed to trigger a watermask event */
#define FIFO_CTL_SAMPLES_MASK        (31 << FIFO_CTL_SAMPLES_SHIFT)
#define FIFO_CTL_TRIGGER             (1 << 5)  /* Bit 5: Value 0 links trigger event to INT1, value 1 to INT2 */
#define FIFO_CTL_MODE_SHIFT          6         /* Bit 6-7: FIFO Mode */
#define FIFO_CTL_MODE_MASK           (3 << FIFO_CTL_MODE_SHIFT)
#define FIFO_CTL_MODE_BYPASS         (0 << FIFO_CTL_MODE_SHIFT)
#define FIFO_CTL_MODE_FIFO           (1 << FIFO_CTL_MODE_SHIFT)
#define FIFO_CTL_MODE_STREAM         (2 << FIFO_CTL_MODE_SHIFT)
#define FIFO_CTL_MODE_TRIGGER        (3 << FIFO_CTL_MODE_SHIFT)

/* Register 0x39 - FIFO_STATUS */

#define FIFO_STATUS_ENTRIES_SHIFT    0          /* Bit 0-5: Reports how many samples are stored in the FIFO */
#define FIFO_STATUS_ENTRIES_MASK     (63 << FIFO_STATUS_ENTRIES_SHIFT)
#define FIFO_STATUS_TRIG             (1 << 7)   /* Bit 7: A 1 reports a trigger event occurred, 0 means no event */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/* Form of the GPIO "interrupt handler" callback. Callbacks do not occur from an interrupt
 * handler but rather from the context of the worker thread with interrupts enabled.
 */

typedef void (*adxl345_handler_t)(FAR struct adxl345_config_s *config, FAR void *arg);

/* A reference to a structure of this type must be passed to the ADXL345 driver when the
 * driver is instantiated. This structure provides information about the configuration of the
 * ADXL345 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by the driver
 * and is presumed to persist while the driver is active. The memory must be writable
 * because, under certain circumstances, the driver may modify the frequency.
 */

struct adxl345_config_s
{
  /* Device characterization */

#ifdef CONFIG_ADXL345_I2C
  uint8_t address;     /* 7-bit I2C address (only bits 0-6 used) */
#endif
  uint32_t frequency;  /* I2C or SPI frequency */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the ADXL345 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach the ADXL345 interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   * clear   - Acknowledge/clear any pending GPIO interrupt
   */

  int  (*attach)(FAR struct adxl345_config_s *state, adxl345_handler_t handler,
                 FAR void *arg);
  void (*enable)(FAR struct adxl345_config_s *state, bool enable);
  void (*clear)(FAR struct adxl345_config_s *state);
};

typedef FAR void *ADXL345_HANDLE;

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/********************************************************************************************
 * Name: adxl345_instantiate
 *
 * Description:
 *   Instantiate and configure the ADXL345 device driver to use the provided I2C or SPI
 *   device instance.
 *
 * Input Parameters:
 *   dev     - An I2C or SPI driver instance
 *   config  - Persistant board configuration data
 *
 * Returned Value:
 *   A non-zero handle is returned on success.  This handle may then be used to configure
 *   the ADXL345 driver as necessary.  A NULL handle value is returned on failure.
 *
 ********************************************************************************************/

#ifdef CONFIG_ADXL345_SPI
EXTERN ADXL345_HANDLE adxl345_instantiate(FAR struct spi_dev_s *dev,
                                            FAR struct adxl345_config_s *config);
#else
EXTERN ADXL345_HANDLE adxl345_instantiate(FAR struct i2c_dev_s *dev,
                                            FAR struct adxl345_config_s *config);
#endif

/********************************************************************************************
 * Name: adxl345_register
 *
 * Description:
 *  This function will register the accelerometer driver as /dev/accelN
 *  where N is the minor device number
 *
 * Input Parameters:
 *   handle    - The handle previously returned by adxl345_instantiate
 *   minor     - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is returned to indicate
 *   the nature of the failure.
 *
 ********************************************************************************************/

EXTERN int adxl345_register(ADXL345_HANDLE handle, int minor);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_ADXL345 */
#endif /* __INCLUDE_NUTTX_INPUT_ADXL345_H */

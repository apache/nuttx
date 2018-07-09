/********************************************************************************************
 * include/nuttx/sensors/bmg160.h
 *
 *   Copyright (C) 2016 DS-Automotion GmbH. All rights reserved.
 *   Author: Alexander Entinger <a.entinger@ds-automotion.com>
 *           Thomas Ilk
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

#ifndef __INCLUDE_NUTTX_SENSORS_BMG160_H
#define __INCLUDE_NUTTX_SENSORS_BMG160_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_BMG160)

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* BMG160 Register Definitions **************************************************************/

/* Chip ID */

#define BMG160_CHIP_ID_REG             (0x00)  /* Contains the chip identification code */

/* Data Register */

#define BMG160_RATE_X_LSB_REG          (0x02)
#define BMG160_RATE_X_MSB_REG          (0x03)
#define BMG160_RATE_Y_LSB_REG          (0x04)
#define BMG160_RATE_Y_MSB_REG          (0x05)
#define BMG160_RATE_Z_LSB_REG          (0x06)
#define BMG160_RATE_Z_MSB_REG          (0x07)
#define BMG160_TEMP_REG                (0x08)

/* Status Register */

#define BMG160_INT_STATUS_0_REG        (0x09)  /* Contains interrupt status bits */
#define BMG160_INT_STATUS_1_REG        (0x0A)  /* Contains interrupt status bits */
#define BMG160_INT_STATUS_2_REG        (0x0B)  /* Contains any motion interrupt status bits */
#define BMG160_INT_STATUS_3_REG        (0x0C)  /* Contains high rate interrupt status bits */
#define BMG160_FIFO_STATUS_REG         (0x0E)  /* Contains FIFO status flags */

/* Control Register */

#define BMG160_RANGE_REG               (0x0F)  /* enables to select FSR */
#define BMG160_BW_REG                  (0x10)  /* enables to select ODR */
#define BMG160_LPM1_REG                (0x11)  /* Selection of the main power modes */
#define BMG160_LPM2_REG                (0x12)  /* Configuration settings for fast power-up and external trigger */
#define BMG160_RATE_HBW_REG            (0x13)  /* Angular rate data acquisition and data output format */
#define BMG160_BGW_SOFTRESET_REG       (0x14)  /* Controls user triggered reset of the sensor */

/* Interrupt Status Register */

#define BMG160_INT_EN_0_REG            (0x15)  /* Controls which interrupts are enabled */
#define BMG160_INT_EN_1_REG            (0x16)  /* Contains interrupt pin configuration */
#define BMG160_INT_MAP_0_REG           (0x17)  /* Controls which interrupt signals are mapped to the INT1 pin */
#define BMG160_INT_MAP_1_REG           (0x18)  /* Controls which interrupt signals are mapped to the INT1 pin and INT2 pin */
#define BMG160_INT_MAP_2_REG           (0x19)  /* Controls which interrupt signals are mapped to the INT2 pin */
#define BMG160_INT_ZERO_REG            (0x1A)  /* Contains the data source definition of those interrupts with selectable data source */
#define BMG160_INT_ONE_REG             (0x1B)  /* Contains the data source definition of fast offset compensation and the any motion threshold */
#define BMG160_INT_TWO_REG             (0x1C)  /* Contains the any motion configuration for x-, y- and z-axis */
#define BMG160_INT_FOUR_REG            (0x1E)
#define BMG160_INT_RST_LATCH_REG       (0x21)  /* Contains the interrupt reset bit and the interrupt mode selection */

/* Interrupt High Rate Configuration Register */

#define BMG160_HIGH_TH_X_REG           (0x22)  /* Contains the high rate threshold and high rate hysteresis setting for the x-axis */
#define BMG160_HIGH_DUR_X_REG          (0x23)  /* Contains high rate duration setting for the x-axis */
#define BMG160_HIGH_TH_Y_REG           (0x24)  /* Contains the high rate threshold and high rate hysteresis setting for the y-axis */
#define BMG160_HIGH_DUR_Y_REG          (0x25)  /* Contains high rate duration setting for the y-axis */
#define BMG160_HIGH_TH_Z_REG           (0x26)  /* Contains the high rate threshold and high rate hysteresis setting for the z-axis */
#define BMG160_HIGH_DUR_Z_REG          (0x27)  /* Contains high rate duration setting for the z-axis */

/* Offset Register */

#define BMG160_SOC_REG                 (0x31)  /* Contains the slow offset cancellation setting */
#define BMG160_FOC_REG                 (0x32)  /* Contains the fast offset cancellation setting */

/* NVM Control Register */

#define BMG160_TRIM_NVM_CTRL_REG       (0x33)  /* Contains the control settings for the few-time programmable non-volatile memory (NVM) */

/* Digital Interface Register */

#define BMG160_BGW_SPI3_WDT_REG        (0x34)  /* Contains settings for the digital interfaces */

/* Offset Configuration Register */

#define BMG160_OFC1_REG                (0x36)  /* Contains offset compensation values */
#define BMG160_OFC2_REG                (0x37)  /* Contains offset compensation values for X-channel */
#define BMG160_OFC3_REG                (0x38)  /* Contains offset compensation values for Y-channel */
#define BMG160_OFC4_REG                (0x39)  /* Contains offset compensation values for Z-channel */
#define BMG160_TRIM_GP0_REG            (0x3A)  /* Contains general purpose data register with NVM back-up */
#define BMG160_TRIM_GP1_REG            (0x3B)  /* Contains general purpose data register with NVM back-up */

/* Self-test Register */

#define BMG160_BIST_REG                (0x3C)  /* Contains Built in Self-Test possibilities */

/* FIFO Register */

#define BMG160_FIFO_CONFIG_0_REG       (0x3D)  /* Contains the FIFO watermark level */
#define BMG160_FIFO_CONFIG_1_REG       (0x3E)  /* Contains FIFO configuration settings. The FIFO buffer memory is cleared and
                                                * the FIFO-full flag cleared when writing to FIFO_CONFIG_1 register */
#define BMG160_FIFO_DATA_REG           (0x3F)  /* FIFO data readout register */

/* Control Register Definitions *************************************************************/

/* BMG160 RANGE_REG Definitions */

#define BMG160_RANGE_REG_FSR_0_bm      (1 << 0)  /* Full scale selection bit 0 */
#define BMG160_RANGE_REG_FSR_1_bm      (1 << 1)  /* Full scale selection bit 1 */
#define BMG160_RANGE_REG_FSR_2_bm      (1 << 2)  /* Full scale selection bit 2 */
#define BMG160_RANGE_REG_FIX_VAL_bm    (1 << 7)  /* write 1 to 7th bit of Range Register */

/* BMG160 BW_REG Definitions */

#define BMG160_BW_REG_ODR_0_bm         (1 << 0)  /* Output data rate selection bit 0 */
#define BMG160_BW_REG_ODR_1_bm         (1 << 1)  /* Output data rate selection bit 1 */
#define BMG160_BW_REG_ODR_2_bm         (1 << 2)  /* Output data rate selection bit 2 */

/* BMG160 LPM1_REG Definitions */

#define BMG160_LPM1_REG_SP_bm          (1 << 7)  /* active suspend mode */
#define BMG160_LPM1_REG_D_SP_bm        (1 << 5)  /* active deep suspend mode */
#define BMG160_LPM1_REG_S_DUR_0_bm     (1 << 1)  /* Sleep duration selection bit 0 */
#define BMG160_LPM1_REG_S_DUR_1_bm     (1 << 2)  /* Sleep duration selection bit 1 */
#define BMG160_LPM1_REG_S_DUR_2_bm     (1 << 3)  /* Sleep duration selection bit 2 */

/* BMG160 LPM1_REG Definitions */

#define BMG160_LPM1_REG_AS_DUR_0_bm    (1 << 0)  /* Auto sleep duration selection bit 0 */
#define BMG160_LPM1_REG_AS_DUR_1_bm    (1 << 1)  /* Auto sleep duration selection bit 1 */
#define BMG160_LPM1_REG_AS_DUR_2_bm    (1 << 2)  /* Auto sleep duration selection bit 2 */
#define BMG160_LPM1_REG_E_T_S_0_bm     (1 << 4)  /* External trigger selection bit 0 */
#define BMG160_LPM1_REG_E_T_S_1_bm     (1 << 5)  /* External trigger selection bit 1 */
#define BMG160_LPM1_REG_P_S_M_bm       (1 << 6)  /* Power save mode */
#define BMG160_LPM1_REG_FAST_PU_bm     (1 << 7)  /* Fast power-up mode */

/* BMG160 RATE_HBW_REG Definitions */

#define BMG160_HBW_REG_DATA_HIGH_BW_bm (1 << 7)  /* Enable unfiltered data reading */
#define BMG160_HBW_REG_SHW_DIS_bm      (1 << 6)  /* Disable shadow mechanism for the rate data output register */


/* Interrupt Status Register Definitions ****************************************************/

/* BMG160 INT_EN_0_REG Definitions */

#define BMG160_INT_EN_0_REG_DATA_EN_bm        (1 << 7)   /* Enable new data interrupt */
#define BMG160_INT_EN_0_REG_FIFO_EN_bm        (1 << 6)   /* Enable FIFO interrupt */
#define BMG160_INT_EN_0_REG_AUTO_OFF_EN_bm    (1 << 1)   /* Enable auto-offset compensation */

/* BMG160 INT_EN_1_REG Definitions */

#define BMG160_INT_EN_1_REG_INT2_OD_bm        (1 << 3)   /* Select open drive for INT2 */
#define BMG160_INT_EN_1_REG_INT2_LVL_bm       (1 << 2)   /* Select active level '1' for INT2 */
#define BMG160_INT_EN_1_REG_INT1_OD_bm        (1 << 1)   /* Select open drive for INT1 */
#define BMG160_INT_EN_1_REG_INT1_LVL_bm       (1 << 0)   /* Select active level '1' for INT1 */

/* BMG160 INT_MAP_0_REG Definitions */

#define BMG160_INT_MAP_0_REG_INT1_HIGH_bm     (1 << 3)   /* Map high rate interrupt to INT1 pin */
#define BMG160_INT_MAP_0_REG_INT1_ANY_bm      (1 << 1)   /* Map Any-Motion to INT1 pin */

/* BMG160 INT_MAP_1_REG Definitions */

#define BMG160_INT_MAP_1_REG_INT2_DATA_bm     (1 << 7)   /* Map new data interrupt to INT2 pin */
#define BMG160_INT_MAP_1_REG_INT2_Fast_OFF_bm (1 << 6)   /* Map Fast Offset interrupt to INT2 pin */
#define BMG160_INT_MAP_1_REG_INT2_FIFO_bm     (1 << 5)   /* Map FIFO interrupt to INT2 pin */
#define BMG160_INT_MAP_1_REG_INT2_AUTO_OFF_bm (1 << 4)   /* Map Auto Offset tap interrupt to INT2 pin */
#define BMG160_INT_MAP_1_REG_INT1_AUTO_OFF_bm (1 << 3)   /* Map Auto Offset tap interrupt to INT1 pin */
#define BMG160_INT_MAP_1_REG_INT1_FIFO_bm     (1 << 2)   /* Map FIFO interrupt to INT1 pin */
#define BMG160_INT_MAP_1_REG_INT1_Fast_OFF_bm (1 << 1)   /* Map Fast Offset interrupt to INT1 pin */
#define BMG160_INT_MAP_1_REG_INT1_DATA_bm     (1 << 0)   /* Map new data interrupt to INT1 pin */

/* BMG160 INT_MAP_2_REG Definitions */

#define BMG160_INT_MAP_0_REG_INT2_HIGH_bm     (1 << 3)   /* Map high rate interrupt to INT2 pin */
#define BMG160_INT_MAP_0_REG_INT2_ANY_bm      (1 << 1)   /* Map Any-Motion to INT2 pin */

/* BMG160 INT_ZERO_REG Definitions */

#define BMG160_INT_ZERO_REG_SLOW_OFF_UN_bm    (1 << 5)   /* Selects unfiltered data for slow offset compensation */
#define BMG160_INT_ZERO_REG_HIGH_UN_D_bm      (1 << 3)   /* Selects unfiltered data for high rate interrupt */
#define BMG160_INT_ZERO_REG_ANY_UN_D_bm       (1 << 1)   /* Selects unfiltered data for any motion interrupt

/* BMG160 INT_ONE_REG Definitions */

#define BMG160_INT_ONE_REG_FAST_OFF_UN_bm     (1 << 7)   /* Selects unfiltered data for fast offset compensation */

/* BMG160 INT_TWO_REG Definitions */

#define BMG160_INT_TWO_REG_ANY_EN_Z_bm        (1 << 2)   /* Enables any motion interrupt for z-axis */
#define BMG160_INT_TWO_REG_ANY_EN_Y_bm        (1 << 1)   /* Enables any motion interrupt for y-axis */
#define BMG160_INT_TWO_REG_ANY_EN_X_bm        (1 << 0)   /* Enables any motion interrupt for x-axis */

/* BMG160 INT_FOUR_REG Definitions */

#define BMG160_INT_FOUR_REG_FIFO_WM_EN_bm     (1 << 2)   /* Enables fifo water mark level interrupt

/* BMG160 INT_RST_LATCH_REG Definitions */

#define BMG160_INT_RST_LATCH_REG_RST_INT_bm     (1 << 7) /* Clears any latched interrupts */
#define BMG160_INT_RST_LATCH_REG_OFF_RST_bm     (1 << 6) /* Resets the Offset value calculated with Fast-, Slow- and AutoOffset */
#define BMG160_INT_RST_LATCH_REG_LATCH_STAT_bm  (1 << 4)
#define BMG160_INT_RST_LATCH_REG_LATCH_INT_3_bm (1 << 3) /* Latch mode selection bit 3 */
#define BMG160_INT_RST_LATCH_REG_LATCH_INT_2_bm (1 << 2) /* Latch mode selection bit 2 */
#define BMG160_INT_RST_LATCH_REG_LATCH_INT_1_bm (1 << 1) /* Latch mode selection bit 1 */
#define BMG160_INT_RST_LATCH_REG_LATCH_INT_0_bm (1 << 0) /* Latch mode selection bit 0 */

/* Interrupt High Rate Configuration Register Definitions ************************************/

/* BMG160 HIGH_TH_X_REG Definitions */

#define BMG160_HIGH_TH_X_REG_HY_X_1_bm   (1 << 7)
#define BMG160_HIGH_TH_X_REG_HY_X_0_bm   (1 << 6)
#define BMG160_HIGH_TH_X_REG_TH_X_4_bm   (1 << 5)
#define BMG160_HIGH_TH_X_REG_TH_X_3_bm   (1 << 4)
#define BMG160_HIGH_TH_X_REG_TH_X_2_bm   (1 << 3)
#define BMG160_HIGH_TH_X_REG_TH_X_1_bm   (1 << 2)
#define BMG160_HIGH_TH_X_REG_TH_X_0_bm   (1 << 1)
#define BMG160_HIGH_TH_X_REG_EN_X_1_bm   (1 << 0)  /* Enables high rate interrupt for x-axis */

/* BMG160 HIGH_DUR_X_REG Definitions */

#define BMG160_HIGH_DUR_X_REG_7_bm       (1 << 7)
#define BMG160_HIGH_DUR_X_REG_6_bm       (1 << 6)
#define BMG160_HIGH_DUR_X_REG_5_bm       (1 << 5)
#define BMG160_HIGH_DUR_X_REG_4_bm       (1 << 4)
#define BMG160_HIGH_DUR_X_REG_3_bm       (1 << 3)
#define BMG160_HIGH_DUR_X_REG_2_bm       (1 << 2)
#define BMG160_HIGH_DUR_X_REG_1_bm       (1 << 1)
#define BMG160_HIGH_DUR_X_REG_0_bm       (1 << 0)

/* BMG160 HIGH_TH_Y_REG Definitions */

#define BMG160_HIGH_TH_Y_REG_HY_Y_1_bm   (1 << 7)
#define BMG160_HIGH_TH_Y_REG_HY_Y_0_bm   (1 << 6)
#define BMG160_HIGH_TH_Y_REG_TH_Y_4_bm   (1 << 5)
#define BMG160_HIGH_TH_Y_REG_TH_Y_3_bm   (1 << 4)
#define BMG160_HIGH_TH_Y_REG_TH_Y_2_bm   (1 << 3)
#define BMG160_HIGH_TH_Y_REG_TH_Y_1_bm   (1 << 2)
#define BMG160_HIGH_TH_Y_REG_TH_Y_0_bm   (1 << 1)
#define BMG160_HIGH_TH_Y_REG_EN_Y_1_bm   (1 << 0)  /* Enables high rate interrupt for Y-axis */

/* BMG160 HIGH_DUR_Y_REG Definitions */

#define BMG160_HIGH_DUR_Y_REG_7_bm       (1 << 7)
#define BMG160_HIGH_DUR_Y_REG_6_bm       (1 << 6)
#define BMG160_HIGH_DUR_Y_REG_5_bm       (1 << 5)
#define BMG160_HIGH_DUR_Y_REG_4_bm       (1 << 4)
#define BMG160_HIGH_DUR_Y_REG_3_bm       (1 << 3)
#define BMG160_HIGH_DUR_Y_REG_2_bm       (1 << 2)
#define BMG160_HIGH_DUR_Y_REG_1_bm       (1 << 1)
#define BMG160_HIGH_DUR_Y_REG_0_bm       (1 << 0)

/* BMG160 HIGH_TH_Z_REG Definitions */

#define BMG160_HIGH_TH_Z_REG_HY_Z_1_bm   (1 << 7)
#define BMG160_HIGH_TH_Z_REG_HY_Z_0_bm   (1 << 6)
#define BMG160_HIGH_TH_Z_REG_TH_Z_4_bm   (1 << 5)
#define BMG160_HIGH_TH_Z_REG_TH_Z_3_bm   (1 << 4)
#define BMG160_HIGH_TH_Z_REG_TH_Z_2_bm   (1 << 3)
#define BMG160_HIGH_TH_Z_REG_TH_Z_1_bm   (1 << 2)
#define BMG160_HIGH_TH_Z_REG_TH_Z_0_bm   (1 << 1)
#define BMG160_HIGH_TH_Z_REG_EN_Z_1_bm   (1 << 0)  /* Enables high rate interrupt for Z-axis */

/* BMG160 HIGH_DUR_Z_REG Definitions */

#define BMG160_HIGH_DUR_Z_REG_7_bm       (1 << 7)
#define BMG160_HIGH_DUR_Z_REG_6_bm       (1 << 6)
#define BMG160_HIGH_DUR_Z_REG_5_bm       (1 << 5)
#define BMG160_HIGH_DUR_Z_REG_4_bm       (1 << 4)
#define BMG160_HIGH_DUR_Z_REG_3_bm       (1 << 3)
#define BMG160_HIGH_DUR_Z_REG_2_bm       (1 << 2)
#define BMG160_HIGH_DUR_Z_REG_1_bm       (1 << 1)
#define BMG160_HIGH_DUR_Z_REG_0_bm       (1 << 0)

/* Offset Register Definitions **************************************************************/

/* BMG160 SOC_REG */

#define BMG160_SOC_REG_SLOW_OFF_EN_Z_bm  (1 << 2)  /* Enables slow offset compensation for z-axis */
#define BMG160_SOC_REG_SLOW_OFF_EN_Y_bm  (1 << 1)  /* Enables slow offset compensation for y-axis */
#define BMG160_SOC_REG_SLOW_OFF_EN_X_bm  (1 << 0)  /* Enables slow offset compensation for x-axis */

/* BMG160 FOC_REG */

#define BMG160_FOC_REG_FAST_OFF_EN_bm    (1 << 2)  /* Triggers the fast offset compensation for the enabled axes */
#define BMG160_FOC_REG_FAST_OFF_EN_Z_bm  (1 << 2)  /* Enables fast offset compensation for z-axis */
#define BMG160_FOC_REG_FAST_OFF_EN_Y_bm  (1 << 1)  /* Enables fast offset compensation for y-axis */
#define BMG160_FOC_REG_FAST_OFF_EN_X_bm  (1 << 0)  /* Enables fast offset compensation for x-axis */

/* NVM Control Register Definitions *********************************************************/

/* BMG160 TRIM_NVM_CTRL_REG */

#define BMG160_TRIM_NVM_CTRL_REG_NVM_LOAD_bm      (1 << 3) /* Triggers an update of all config registers form NVM,
                                                            * the NVM_RDY flag must be '1' prior to triggering the update */
#define BMG160_TRIM_NVM_CTRL_REG_NVM_PROG_TRIG_bm (1 << 1) /* Triggers an NVM write operation; (see page 59, data sheet)
                                                            * the NVM_RDY flag must be '1' prior to triggering the update */
#define BMG160_TRIM_NVM_CTRL_REG_NVM_PROG_MODE_bm (1 << 0) /* unlock NVM write operation */

/* Digital Interface Register Definitions ***************************************************/

/* BMG160 BGW_SPI3_WDT_REG */

#define BMG160_BGW_SPI3_WDT_REG_I2C_WDT_EN_bm  (1 << 2)  /* Enables watchdog at the SDA pin if I2C mode is selected */
#define BMG160_BGW_SPI3_WDT_REG_I2C_WDT_SEL_bm (1 << 1)  /* Select an I2C watchdog timer period of 50ms */
#define BMG160_BGW_SPI3_WDT_REG_SPI3_bm        (1 << 0)  /* Enable 3-wire SPI mode */

/* Offset Configuration Register Definitions ************************************************/

/* FIFO Register Definitions ****************************************************************/

/* BMG160 FIFO_CONFIG_0_REG */

#define BMG160_FIFO_CONFIG_0_REG_TAG_bm        (1 << 7)  /* Enables FIFO tag (interrupt) */

/* BMG160 FIFO_CONFIG_1_REG */

#define BMG160_FIFO_CONFIG_1_REG_MODE_1_bm     (1 << 7)  /* FIFO mode selection bit 1 */
#define BMG160_FIFO_CONFIG_1_REG_MODE_0_bm     (1 << 6)  /* FIFO mode selection bit 0 */
#define BMG160_FIFO_CONFIG_1_REG_DATA_SEL_1_bm (1 << 1)  /* FIFO data selection bit 1 */
#define BMG160_FIFO_CONFIG_1_REG_DATA_SEL_0_bm (1 << 0)  /* FIFO data selection bit 0 */

/* SPI BUS PARAMETERS ***********************************************************************/

#define BMG160_SPI_FREQUENCY  (4000000)        /* 4 MHz */
#define BMG160_SPI_MODE       (SPIDEV_MODE3)   /* Device uses SPI Mode 3: CPOL=1, CPHA=1 */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/* A reference to a structure of this type must be passed to the BMG160
 * driver. This structure provides information about the configuration
 * of the sensor and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct bmg160_config_s
{
  /* Since multiple BMG160 can be connected to the same SPI bus we need
   * to use multiple spi device ids which are employed by NuttX to select/
   * deselect the desired BMG160 chip via their chip select inputs.
   */

  int spi_devid;

  /* The IRQ number must be provided for each BMG160 device so that
   * their interrupts can be distinguished.
   */

  int irq;

  /* Attach the BMG160 interrupt handler to the GPIO interrupt of the
   * concrete BMG160 instance.
   */

  int (*attach)(FAR struct bmg160_config_s *, xcpt_t);
};

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************************
 * Name: bmg160_register
 *
 * Description:
 *   Register the BMG160 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gyr0"
 *   spi - An instance of the SPI interface to use to communicate with BMG160
 *   config - configuration for the BMG160 driver. For details see description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ********************************************************************************************/

int bmg160_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                    FAR struct bmg160_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_SENSORS_BMG160 */
#endif /* __INCLUDE_NUTTX_SENSORS_BMG160_H */

/****************************************************************************
 * include/nuttx/sensors/lis3dsh.h
 *
 *   Copyright (C) 2016 DS-Automotion GmbH. All rights reserved.
 *   Author:  Alexander Entinger <a.entinger@ds-automotion.com>
 *            Thomas Ilk
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

#ifndef __INCLUDE_NUTTX_SENSORS_LIS3DSH_H
#define __INCLUDE_NUTTX_SENSORS_LIS3DSH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>

#if defined(CONFIG_SPI) && defined(CONFIG_LIS3DSH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LIS3DSH Register Definitions *********************************************/

#define LIS3DSH_INFO_REG_1    (0x0D)
#define LIS3DSH_INFO_REG_2    (0x0E)
#define LIS3DSH_WHO_AM_I_REG  (0x0F)
#define LIS3DSH_CTRL_REG_1    (0x21)
#define LIS3DSH_CTRL_REG_2    (0x22)
#define LIS3DSH_CTRL_REG_3    (0x23)
#define LIS3DSH_CTRL_REG_4    (0x20)
#define LIS3DSH_CTRL_REG_5    (0x24)
#define LIS3DSH_CTRL_REG_6    (0x25)
#define LIS3DSH_STATUS_REG    (0x27)
#define LIS3DSH_OUT_T_REG     (0x0C)
#define LIS3DSH_OFF_X_REG     (0x10)
#define LIS3DSH_OFF_Y_REG     (0x11)
#define LIS3DSH_OFF_Z_REG     (0x12)
#define LIS3DSH_CS_X_REG      (0x13)
#define LIS3DSH_CS_Y_REG      (0x14)
#define LIS3DSH_CS_Z_REG      (0x15)
#define LIS3DSH_LC_L_REG      (0x16)
#define LIS3DSH_LC_H_REG      (0x17)
#define LIS3DSH_STAT_REG      (0x18)
#define LIS3DSH_PEAK1_REG     (0x19)
#define LIS3DSH_PEAK2_REG     (0x1A)
#define LIS3DSH_VFC_REG_1     (0x1B)
#define LIS3DSH_VFC_REG_2     (0x1C)
#define LIS3DSH_VFC_REG_3     (0x1D)
#define LIS3DSH_VFC_REG_4     (0x1E)
#define LIS3DSH_THRS3_REG     (0x1F)
#define LIS3DSH_OUT_X_L_REG   (0x28)
#define LIS3DSH_OUT_X_H_REG   (0x29)
#define LIS3DSH_OUT_Y_L_REG   (0x2A)
#define LIS3DSH_OUT_Y_H_REG   (0x2B)
#define LIS3DSH_OUT_Z_L_REG   (0x2C)
#define LIS3DSH_OUT_Z_H_REG   (0x2D)
#define LIS3DSH_FIFO_CTRL_REG (0x2E)
#define LIS3DSH_FIFO_SRC_REG  (0x2F)
#define LIS3DSH_FIFO_CTRL_REG (0x2E)
#define LIS3DSH_ST1_REG_1     (0x40)
#define LIS3DSH_ST1_REG_2     (0x41)
#define LIS3DSH_ST1_REG_3     (0x42)
#define LIS3DSH_ST1_REG_4     (0x43)
#define LIS3DSH_ST1_REG_5     (0x44)
#define LIS3DSH_ST1_REG_6     (0x45)
#define LIS3DSH_ST1_REG_7     (0x46)
#define LIS3DSH_ST1_REG_8     (0x47)
#define LIS3DSH_ST1_REG_9     (0x48)
#define LIS3DSH_ST1_REG_10    (0x49)
#define LIS3DSH_ST1_REG_11    (0x4A)
#define LIS3DSH_ST1_REG_12    (0x4B)
#define LIS3DSH_ST1_REG_13    (0x4C)
#define LIS3DSH_ST1_REG_14    (0x4D)
#define LIS3DSH_ST1_REG_15    (0x4E)
#define LIS3DSH_ST1_REG_16    (0x4F)
#define LIS3DSH_TIM4_1_REG    (0x50)
#define LIS3DSH_TIM3_1_REG    (0x51)
#define LIS3DSH_TIM2_1_REG_1  (0x52)
#define LIS3DSH_TIM2_1_REG_2  (0x53)
#define LIS3DSH_TIM1_1_REG_1  (0x54)
#define LIS3DSH_TIM1_1_REG_5  (0x55)
#define LIS3DSH_THRS2_1_REG   (0x56)
#define LIS3DSH_THRS1_1_REG   (0x57)
#define LIS3DSH_MASK1_B_REG   (0x59)
#define LIS3DSH_MASK1_A_REG   (0x5A)
#define LIS3DSH_SETT1_REG     (0x5B)
#define LIS3DSH_PR1_REG       (0x5C)
#define LIS3DSH_TC1_REG_1     (0x5D)
#define LIS3DSH_TC1_REG_2     (0x5E)
#define LIS3DSH_OUTS1_REG     (0x5F)
#define LIS3DSH_ST2_REG_1     (0x60)
#define LIS3DSH_ST2_REG_2     (0x61)
#define LIS3DSH_ST2_REG_3     (0x62)
#define LIS3DSH_ST2_REG_4     (0x63)
#define LIS3DSH_ST2_REG_5     (0x64)
#define LIS3DSH_ST2_REG_6     (0x65)
#define LIS3DSH_ST2_REG_7     (0x66)
#define LIS3DSH_ST2_REG_8     (0x67)
#define LIS3DSH_ST2_REG_9     (0x68)
#define LIS3DSH_ST2_REG_10    (0x69)
#define LIS3DSH_ST2_REG_11    (0x6A)
#define LIS3DSH_ST2_REG_12    (0x6B)
#define LIS3DSH_ST2_REG_13    (0x6C)
#define LIS3DSH_ST2_REG_14    (0x6D)
#define LIS3DSH_ST2_REG_15    (0x6E)
#define LIS3DSH_ST2_REG_16    (0x6F)
#define LIS3DSH_TIM4_2_REG    (0x70)
#define LIS3DSH_TIM3_2_REG    (0x71)
#define LIS3DSH_TIM2_2_REG_1  (0x72)
#define LIS3DSH_TIM2_2_REG_2  (0x73)
#define LIS3DSH_TIM1_2_REG_1  (0x74)
#define LIS3DSH_TIM1_2_REG_5  (0x75)
#define LIS3DSH_THRS2_2_REG   (0x76)
#define LIS3DSH_THRS1_2_REG   (0x77)
#define LIS3DSH_DES2_REG      (0x78)
#define LIS3DSH_MASK2_B_REG   (0x79)
#define LIS3DSH_MASK2_A_REG   (0x7A)
#define LIS3DSH_SETT2_REG     (0x7B)
#define LIS3DSH_PR2_REG       (0x7C)
#define LIS3DSH_TC2_REG_1     (0x7D)
#define LIS3DSH_TC2_REG_2     (0x7E)

/* LIS3DSH CTRL_REG_3 Definitions *******************************************/

#define LIS3DSH_CTRL_REG_3_DR_EN_BM       (1<<7)  /* DRDY signal enable to INT 1 */
#define LIS3DSH_CTRL_REG_3_IEA_BM         (1<<6)  /* Interrupt signal polarity */
#define LIS3DSH_CTRL_REG_3_IEL_BM         (1<<5)  /* Interrupt signal latching */
#define LIS3DSH_CTRL_REG_3_INT2_EN_BM     (1<<4)  /* Interrupt 2 enable / disable */
#define LIS3DSH_CTRL_REG_3_INT1_EN_BM     (1<<3)  /* Interrupt 1 enable / disable */
#define LIS3DSH_CTRL_REG_3_VFILT_BM       (1<<2)  /* Vector filter enable / disable */
#define LIS3DSH_CTRL_REG_3_STRT_BM        (1<<0)  /* Enable soft reset */

/* LIS3DSH CTRL_REG_4 Definitions *******************************************/

#define LIS3DSH_CTRL_REG_4_ODR_3_BM       (1<<7)  /* Output data rate and power mode selection bit 3 */
#define LIS3DSH_CTRL_REG_4_ODR_2_BM       (1<<6)  /* Output data rate and power mode selection bit 2 */
#define LIS3DSH_CTRL_REG_4_ODR_1_BM       (1<<5)  /* Output data rate and power mode selection bit 1 */
#define LIS3DSH_CTRL_REG_4_ODR_0_BM       (1<<4)  /* Output data rate and power mode selection bit 0 */
#define LIS3DSH_CTRL_REG_4_BDU_BM         (1<<3)  /* Enable block data update for accelerating data */
#define LIS3DSH_CTRL_REG_4_ZEN_BM         (1<<2)  /* Enable Z-axis */
#define LIS3DSH_CTRL_REG_4_YEN_BM         (1<<1)  /* Enable Y-axis */
#define LIS3DSH_CTRL_REG_4_XEN_BM         (1<<0)  /* Enable X-axis */

/* LIS3DSH CTRL_REG_5 Definitions *******************************************/

#define LIS3DSH_CTRL_REG_5_BW_2_BM        (1<<7)  /* Anti-aliasing filter bandwidth bit 2 */
#define LIS3DSH_CTRL_REG_5_BW_1_BM        (1<<6)  /* Anti-aliasing filter bandwidth bit 1 */
#define LIS3DSH_CTRL_REG_5_FSCALE_2_BM    (1<<5)  /* Full-scale selection bit 2 */
#define LIS3DSH_CTRL_REG_5_FSCALE_1_BM    (1<<4)  /* Full-scale selection bit 1 */
#define LIS3DSH_CTRL_REG_5_FSCALE_0_BM    (1<<3)  /* Full-scale selection bit 0 */
#define LIS3DSH_CTRL_REG_5_ST_2_BM        (1<<2)  /* Enable self-test bit 2 */
#define LIS3DSH_CTRL_REG_5_ST_1_BM        (1<<1)  /* Enable self-test bit 1 */
#define LIS3DSH_CTRL_REG_5_SIM_BM         (1<<0)  /* Enable SPI 4-wire interface */

/* LIS3DSH CTRL_REG_6 Definitions *******************************************/

#define LIS3DSH_CTRL_REG_6_BOOT_BM        (1<<7)  /* Force reboot, cleared as soon as the reboot is finished. Active high */
#define LIS3DSH_CTRL_REG_6_FIFO_EN_BM     (1<<6)  /* Enable FIFO */
#define LIS3DSH_CTRL_REG_6_WTM_EN_BM      (1<<5)  /* Enable FIFO watermark level use */
#define LIS3DSH_CTRL_REG_6_ADD_INC_BM     (1<<4)  /* Register address automatically incremented during a multiple byte access with a serial interface */
#define LIS3DSH_CTRL_REG_6_P1_EMPTY_BM    (1<<3)  /* Enable FIFO empty indication on Int1 */
#define LIS3DSH_CTRL_REG_6_P1_WTM_BM      (1<<2)  /* FIFO watermark interrupt Int1 */
#define LIS3DSH_CTRL_REG_6_P1_OVERRUN_BM  (1<<1)  /* FIFO overrun interrupt on Int1 */
#define LIS3DSH_CTRL_REG_6_P2_BOOT_BM     (1<<0)  /* BOOT interrupt on Int2 */

/* SPI BUS PARAMETERS *******************************************************/

#define LIS3DSH_SPI_FREQUENCY    (5000000)        /* 5 MHz */
#define LIS3DSH_SPI_MODE         (SPIDEV_MODE3)   /* Device uses SPI Mode 3: CPOL=1, CPHA=1 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the LIS3DSH
 * driver. This structure provides information about the configuration
 * of the sensor and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct lis3dsh_config_s
{
  /* Since multiple sensors can be connected to the same SPI bus we need
   * to use multiple spi device ids which are employed by NuttX to select/
   * deselect the desired LIS3DSH chip via their chip select inputs.
   */

  int spi_devid;

  /* The IRQ number must be provided for each LIS3DSH device so that
   * their interrupts can be distinguished.
   */

  int irq;

  /* Attach the LIS3DSH interrupt handler to the GPIO interrupt of the
   * concrete LIS3DSH instance.
   */

  int (*attach)(FAR struct lis3dsh_config_s *, xcpt_t);
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
 * Name: lis3dsh_register
 *
 * Description:
 *   Register the LIS3DSH character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/acc0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             LIS3DSH
 *   config  - configuration for the LIS3DSH driver. For details see
 *             description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lis3dsh_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                     FAR struct lis3dsh_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_LIS3DSH */
#endif /* __INCLUDE_NUTTX_SENSORS_LIS3DSH_H */

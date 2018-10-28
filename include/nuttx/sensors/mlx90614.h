/****************************************************************************
 * include/nuttx/sensors/mlx90614.h
 *
 *   Copyright (C) 2018 Alan Carvalho de Assis. All rights reserved.
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_MLX90614_H
#define __INCLUDE_NUTTX_SENSORS_MLX90614_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MLX90614)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MLX90614 Commands */

#define MLX90614_CMD_RAM_ACCESS      (0x00)          /* RAM Access Command */
#define MLX90614_CMD_EEPROM_ACCESS   (0x20)          /* EEPROM Access Command */
#define MLX90614_CMD_READ_FLAGS      (0xf0)          /* Read Flags Command */
#define MLX90614_CMD_ENTER_SLEEP     (0xff)          /* Enter Sleep Mode Command */

/* MLX90614 EEPROM Addresses ************************************************/

#define MLX90614_TO_MAX              (0x00)          /* To_max = Max Temperature of the Object */
#define MLX90614_TO_MIN              (0x01)          /* To_min = Min Temperature of the Object */
#define MLX90614_PWMCTRL             (0x02)          /* PWM Control */
#define MLX90614_TA_RANGE            (0x03)          /* Range of Ambient Temperature */
#define MLX90614_KE                  (0x04)          /* Ke */
#define MLX90614_CONFIG_REG1         (0x05)          /* Config Register1 */
                                                     /* 0x06-0x0D - Reserved */
#define MLX90614_SMBUS_ADDR          (0x0e)          /* SMBus address */
                                                     /* 0x0f-0x1b - Reserved */
#define MLX90614_ID_NUM1             (0x1c)          /* ID number */
#define MLX90614_ID_NUM2             (0x1d)          /* ID number */
#define MLX90614_ID_NUM3             (0x1e)          /* ID number */
#define MLX90614_ID_NUM4             (0x1f)          /* ID number */

/* Register bits definitions */

/* PWMCTRL */

#define PWMCTRL_MODE_SINGLE          (1 << 0)        /* Single PWM, 0 = Extended PWM */
#define PWMCTRL_PWM_ENABLE           (1 << 1)        /* Enable PWM/disable SMBus, 0 = Disable PWM/enable SMBus */
#define PWMCTRL_PWM_PUSHPULL         (1 << 2)        /* Push-Pull mode, 0 = OpenDrain */
#define PWMCTRL_THERMORELAY          (1 << 3)        /* ThermoRelay mode, 0 = PWM mode */
#define PWMCTRL_EXT_PWM_SHIFT        (4)             /* Number of repetitions */
#define PWMCTRL_EXT_PWM_MASK         (0x1f << PWMCTRL_EXT_PWM_SHIFT)
#define PWMCTRL_EXT_PWM_REP(n)       (n << PWMCTRL_EXT_PWM_SHIFT)
#define PWMCTRL_PWM_CLK_CFG_SHIFT    (9)             /* PWM clock configuration */
#define PWMCTRL_PWM_CLK_CFG_MASK     (0x7f << PWMCTRL_PWM_CLK_CFG_SHIFT)
#define PWMCTRL_PWM_CLK_CFG_DIV(n)   (n << PWMCTRL_PWM_CLK_CFG_SHIFT)

/* CONFIG_REG1 */

#define CONFIG_REG1_IIR_SHIFT        (0) /* Bits 2:0 - Configure IIR Coeficients */
#define CONFIG_REG1_IIR_MASK         (7 << CONFIG_REG1_IIR_SHIFT)
#define CONFIG_REG1_IIR_0p5_0p5      (0 << CONFIG_REG1_IIR_SHIFT) /* a1 = 0.5 and b1 = 0.5 */
#define CONFIG_REG1_IIR_0p57_0p42    (7 << CONFIG_REG1_IIR_SHIFT) /* a1 = 0.571428571 and b1 = 0.428571428 */
#define CONFIG_REG1_IIR_0p66_0p33    (6 << CONFIG_REG1_IIR_SHIFT) /* a1 = 0.666... and b1 = 0.333... */
#define CONFIG_REG1_IIR_0p8_0p2      (5 << CONFIG_REG1_IIR_SHIFT) /* a1 = 0.8 and b1 = 0.2 */
#define CONFIG_REG1_IIR_BYPASS       (4 << CONFIG_REG1_IIR_SHIFT) /* a1 = 1 and b1 = 0 => IIR bypassed */
#define CONFIG_REG1_AMB_SENSOR_PTC   (1 << 3) /* Ambient temperature sensor: 1 = PTC, 0 = PTAT */
#define CONFIG_REG1_DATA_PWM_SHIFT   (4) /* Bits 5:4 - Data transmitted through PWM */
#define CONFIG_REG1_DATA_PWM_MASK    (3 << CONFIG_REG1_DATA_PWM_SHIFT)
#define CONFIG_REG1_DATA_PWM_TA_IR1  (0 << CONFIG_REG1_DATA_PWM_SHIFT) /* Data1 = Ta and Data2 = IR1 */
#define CONFIG_REG1_DATA_PWM_TA_IR2  (1 << CONFIG_REG1_DATA_PWM_SHIFT) /* Data1 = Ta and Data2 = IR2 */
#define CONFIG_REG1_DATA_PWM_IR1_IR2 (2 << CONFIG_REG1_DATA_PWM_SHIFT) /* Data1 = IR1 and Data2 = IR2 */
#define CONFIG_REG1_DATA_PWM_IR1_UND (3 << CONFIG_REG1_DATA_PWM_SHIFT) /* Data1 = IR2 and Data2 = Undefined */
#define CONFIG_REG1_NUM_SENSORS      (1 << 6) /* Number of sensors: 1 = 2 sensors, 0 = 1 sensor */
#define CONFIG_REG1_KS               (1 << 7) /* Define the sign Ks. Factory calibration, do not alter */
#define CONFIG_REG1_FIR_SHIFT        (8) /* Bits 10:8 - Configure coefficient N of FIR digital filter */
#define CONFIG_REG1_FIR_MASK         (7 << CONFIG_REG1_FIR_SHIFT)
#define CONFIG_REG1_FIR_N(n)         (((n >> 3) - 1) << CONFIG_REG1_FIR_SHIFT) /* n = 8, 16, 32, 64 ... 1024 */
#define CONFIG_REG1_AMP_GAIN_SHIFT   (11) /* Bits 13:11 - Configure the gain of amplifier */
#define CONFIG_REG1_AMP_GAIN_MASK    (7 << CONFIG_REG1_AMP_GAIN_SHIFT)
#define CONFIG_REG1_AMP_GAIN_1       (0 << CONFIG_REG1_AMP_GAIN_SHIFT) /* Gain = 1 => preamplifier bypassed */
#define CONFIG_REG1_AMP_GAIN_3       (1 << CONFIG_REG1_AMP_GAIN_SHIFT) /* Gain = 3 */
#define CONFIG_REG1_AMP_GAIN_6       (2 << CONFIG_REG1_AMP_GAIN_SHIFT) /* Gain = 6 */
#define CONFIG_REG1_AMP_GAIN_12p5    (3 << CONFIG_REG1_AMP_GAIN_SHIFT) /* Gain = 12.5 */
#define CONFIG_REG1_AMP_GAIN_25      (4 << CONFIG_REG1_AMP_GAIN_SHIFT) /* Gain = 25 */
#define CONFIG_REG1_AMP_GAIN_50      (5 << CONFIG_REG1_AMP_GAIN_SHIFT) /* Gain = 50 */
#define CONFIG_REG1_AMP_GAIN_100     (6 << CONFIG_REG1_AMP_GAIN_SHIFT) /* Gain = 100 */
#define CONFIG_REG1_AMP_GAIN_200     (7 << CONFIG_REG1_AMP_GAIN_SHIFT) /* Gain = 200 */
#define CONFIG_REG1_THERMOSHOCK_NEG  (1 << 15) /* Define the sign of thermosock: 1 - negative, 0 - positive */

/* MLX90614 RAM Register ****************************************************/

#define MLX90614_TA                  (0x06) /* Ta = Temperature Ambient     */
#define MLX90614_TOBJ1               (0x07) /* Tobj1 = Temperature Object 1 */
#define MLX90614_TOBJ2               (0x08) /* Tobj2 = Temperature Object 2 */

/* MLX90614 Read Flags */

#define MLX90614_EEBUSY              (1 << 15) /* Previos write/erase EEPROM access is still in progress */
#define MLX90614_EEDEAD              (1 << 13) /* EEPROM double error has occurred */
#define MLX90614_INIT                (1 << 12) /* POR initialization routine is still ongoing */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mlx90614_temp_s
{
  uint16_t ta;
  uint16_t tobj1;
  uint16_t tobj2;
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
 * Name: mlx90614_register
 *
 * Description:
 *   Register the MLX90614 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mag0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             MLX90614
 *   addr    - The I2C address used by the MLX90614.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mlx90614_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_MLX90614 */

#endif /* __INCLUDE_NUTTX_SENSORS_MLX90614_H */


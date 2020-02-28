/****************************************************************************
 * include/nuttx/input/as726x.h
 *
 *   Copyright (C) 2019 Fabian Justi. All rights reserved.
 *   Author: Fabian Justi <Fabian.Justi@gmx.de> and
 *           Andreas Kurz <andreas.kurz@methodpark.de>
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

#ifndef __INCLUDE_NUTTX_SENSORS_AS726X_H
#define __INCLUDE_NUTTX_SENSORS_AS726X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_SENSORS_AS726X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AS726X_I2C_PORTNO 1   /* On I2C1 */

/* Device I2C Address */

#if 0
#define AS726X_I2C_DATA_LSB_CMD_ADDR  0x38
#define AS726X_I2C_DATA_MSB_ADDR      0x39
#endif

#define AS726X_I2C_ADDR         0x49 /* 7-bit unshifted default I2C Address */
#define SENSORTYPE_AS7262       0x3e
#define SENSORTYPE_AS7263       0x3f

/* Register addresses */

#define AS726x_DEVICE_TYPE      0x00
#define AS726x_HW_VERSION       0x01
#define AS726x_CONTROL_SETUP    0x04
#define AS726x_INT_T            0x05
#define AS726x_DEVICE_TEMP      0x06
#define AS726x_LED_CONTROL      0x07

#define AS72XX_SLAVE_STATUS_REG 0x00
#define AS72XX_SLAVE_WRITE_REG  0x01
#define AS72XX_SLAVE_READ_REG   0x02

/* The same register locations are shared between the AS7262 (V,B,G,Y,O,R)
 * and AS7263 (R,S,T,U,V,W) AS7262 and AS7263 registers
 */

#define AS726X_V_R              0x08
#define AS726X_B_S              0x0a
#define AS726X_G_T              0x0c
#define AS726X_Y_U              0x0e
#define AS726X_O_V              0x10
#define AS726X_R_W              0x12
#define AS726X_V_R_CAL          0x14
#define AS726X_B_S_CAL          0x18
#define AS726X_G_T_CAL          0x1c
#define AS726X_Y_U_CAL          0x20
#define AS726X_O_V_CAL          0x24
#define AS726X_R_W_CAL          0x28

#define AS72XX_SLAVE_TX_VALID   0x02
#define AS72XX_SLAVE_RX_VALID   0x01

#define SENSORTYPE_AS7262       0x3e
#define SENSORTYPE_AS7263       0x3f

/* Amount of ms to wait between checking for virtual register changes */

#define AS726X_POLLING_DELAY    5000

#if 0
#define AS726X_CMD_SD           0x01 /* Shutdown command */
#define AS726X_CMD_RSV          0x02
#define AS726X_CMD_IT_0_5T      0x00 /* IT1=0 : IT0=0 */
#define AS726X_CMD_IT_1T        0x04 /* IT1=0 : IT0=1 */
#define AS726X_CMD_IT_2T        0x08 /* IT1=1 : IT0=0 */
#define AS726X_CMD_IT_4T        0x0c /* IT1=1 : IT0=1 */
#define AS726X_CMD_ACK_THD      0x10 /* Acknowledge threshold:
                                      *   0 = 102 steps
                                      *   1 = 145 steps */
#define AS726X_CMD_ACK          0x20 /* Acknowledge activity */
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;
struct as726x_sensor_data_s
{
  uint16_t v_r_value;
  uint16_t b_s_value;
  uint16_t g_t_value;
  uint16_t y_u_value;
  uint16_t o_v_value;
  uint16_t r_w_value;
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
 * Name: as726x_register
 *
 * Description:
 *   Register the AS726X character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/spectr0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *              AS726X
 *   addr    - The I2C address of the AS726X.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int as726x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_AS726X */
#endif /* __INCLUDE_NUTTX_SENSORS_AS726X_H */

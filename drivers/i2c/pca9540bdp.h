/****************************************************************************
 * /nuttx/drivers/i2c/pca9540bdp.h
 *
 *   Copyright (C) 2018 Giorgio Groß. All rights reserved.
 *   Author: Giorgio Groß <giorgio.gross@robodev.eu>
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

#ifndef __DRIVERS_I2C_PCA9540BDP_H
#define __DRIVERS_I2C_PCA9540BDP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_I2CMULTIPLEXER_PCA9540BDP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_PCA9549BDP_I2C_FREQUENCY
#  define CONFIG_PCA9549BDP_I2C_FREQUENCY   400000  /* 400 khz */
#endif

/* Bit Definitions to be passed to write calls */

#define PCA9540BDP_ENABLE                   0x4
#define PCA9540BDP_DISABLE                  0x0

/* Bit masks */

#define PCA9540BDP_CH_BITMASK               0x03
#define PCA9540BDP_ENABLE_BITMASK           0x04

#define PCA9540BDP_CH_BIT                   0
#define PCA9540BDP_CH_NONE_BIT              1
#define PCA9540BDP_ENABLE_BIT               2

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct pca9540bdp_dev_s
{
  FAR struct i2c_master_s *i2c;      /* I2C interface */
  uint16_t addr;
  uint8_t state;                     /* Control register state */
};

struct i2c_port_dev_s
{
  FAR struct i2c_master_s vi2c;      /* Nested structure to allow casting as
                                      * public i2c master */
  uint8_t port;                      /* Associated port on the mux */
  FAR struct pca9540bdp_dev_s* dev;  /* Associated device */
};

#endif /* CONFIG_I2CMULTIPLEXER_PCA9540BDP */
#endif /* __DRIVERS_I2CMULTIPLEXER_PCA9540BDP_H */

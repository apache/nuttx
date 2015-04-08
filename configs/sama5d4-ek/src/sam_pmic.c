/************************************************************************************
 * configs/sama5d4-ek/src/sam_pmic.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c.h>

#include "sam_twi.h"

#include "sama5d4-ek.h"

#ifdef HAVE_PMIC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_pmic_initialize
 *
 * Description:
 *   Currently, this function only disables the PMIC.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_pmic_initialize(void)
{
  FAR struct i2c_dev_s *i2c;
  uint8_t buffer[2];

  /* Get an instance of the I2C interface for the PMIC */

  i2c = up_i2cinitialize(PMIC_TWI_BUS);
  if (!i2c)
    {
      dbg("ERROR: Failed to initialize TWI%d\n", PMIC_TWI_BUS);
    }
  else
    {
      /* Configure the I2C instance */

      (void)I2C_SETADDRESS(i2c, PMIC_I2C_ADDRESS, 7);
      (void)I2C_SETFREQUENCY(i2c, PMIC_I2C_FREQUENCY);

      /* Send the disable sequence */

      buffer[0] = 0x0b;
      buffer[1] = 0xee;
      (void)I2C_WRITE(i2c, buffer, 2);

      buffer[0] = 0x02;
      buffer[1] = 0x0f;
      (void)I2C_WRITE(i2c, buffer, 2);

      buffer[0] = 0x03;
      buffer[1] = 0x0f;
      (void)I2C_WRITE(i2c, buffer, 2);
   }
}

#endif /* HAVE_PMIC */

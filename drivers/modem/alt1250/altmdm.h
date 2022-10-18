/****************************************************************************
 * drivers/modem/alt1250/altmdm.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __DRIVERS_MODEM_ALT1250_ALTMDM_H
#define __DRIVERS_MODEM_ALT1250_ALTMDM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/modem/alt1250.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALTMDM_RETURN_RESET_V1  (-1)
#define ALTMDM_RETURN_NOTREADY  (-2)
#define ALTMDM_RETURN_CANCELED  (-3)
#define ALTMDM_RETURN_RESET_V4  (-4)
#define ALTMDM_RETURN_RESET_PKT (-5)
#define ALTMDM_RETURN_EXIT      (-6)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altmdm_init
 *
 * Description:
 *   Initialize the ALTMDM driver.
 *
 * Input Parameters:
 *   spidev   - An SPI instance that communicates with the ALT1250.
 *   lower    - An instance of the lower interface.
 *
 * Returned Value:
 *   Returns 0 on success.
 *   When an error occurs, a negative value is returned.
 *
 ****************************************************************************/

int altmdm_init(FAR struct spi_dev_s *spidev,
  FAR const struct alt1250_lower_s *lower);

/****************************************************************************
 * Name: altmdm_fin
 *
 * Description:
 *   Finalize the ALTMDM driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns 0 on success.
 *   When an error occurs, a negative value is returned.
 *
 ****************************************************************************/

int altmdm_fin(void);

/****************************************************************************
 * Name: altmdm_read
 *
 * Description:
 *   Read an SPI packet sent from the ALT1250.
 *
 * Input Parameters:
 *   buff     - A buffer for reading SPI packets.
 *   sz       - Buffer size.
 *
 * Returned Value:
 *   Returns the size of the read SPI packet. Otherwise, it returns a
 *   negative value. See the macro prefixed with ALTMDM_RETURN_ as defined
 *   in this file.
 *
 ****************************************************************************/

int altmdm_read(FAR uint8_t *buff, int sz);

/****************************************************************************
 * Name: altmdm_write
 *
 * Description:
 *   Write an SPI packet to the ALT1250.
 *
 * Input Parameters:
 *   buff     - A buffer for sending SPI packets.
 *   sz       - Buffer size.
 *
 * Returned Value:
 *   Returns the size of the write SPI packet. Otherwise, it returns a
 *   negative value. See the macro prefixed with ALTMDM_RETURN_ as defined
 *   in this file.
 *
 ****************************************************************************/

int altmdm_write(FAR uint8_t *buff, int sz);

/****************************************************************************
 * Name: altmdm_take_wlock
 *
 * Description:
 *   Take a wakelock. The ALT1250 will not be able to sleep until
 *   it is released with altmdm_take_wloc().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns the count of wakelocks currently acquired.
 *
 ****************************************************************************/

int altmdm_take_wlock(void);

/****************************************************************************
 * Name: altmdm_give_wlock
 *
 * Description:
 *   Give a wakelock. If the wakelock count is 0, the ALT1250 can sleep.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns the count of wakelocks currently acquired.
 *
 ****************************************************************************/

int altmdm_give_wlock(void);

/****************************************************************************
 * Name: altmdm_poweron
 *
 * Description:
 *   Turn on the power of the ALT1250.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns 0 on success.
 *   When an error occurs, a negative value is returned.
 *
 ****************************************************************************/

int altmdm_poweron(void);

/****************************************************************************
 * Name: altmdm_poweroff
 *
 * Description:
 *   Turn off the power of the ALT1250.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns 0 on success.
 *   When an error occurs, a negative value is returned.
 *
 ****************************************************************************/

int altmdm_poweroff(void);

/****************************************************************************
 * Name: altmdm_reset
 *
 * Description:
 *   Reset the ALT1250.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns 0 on success.
 *   When an error occurs, a negative value is returned.
 *
 ****************************************************************************/

int altmdm_reset(void);

/****************************************************************************
 * Name: altmdm_get_reset_reason
 *
 * Description:
 *   Returns whether the reset was initiated by the user or ALT1250.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns LTE_RESTART_USER_INITIATED if it was reset by the user
 *   or LTE_RESTART_MODEM_INITIATED by the ALT1250.
 *
 ****************************************************************************/

uint32_t altmdm_get_reset_reason(void);

/****************************************************************************
 * Name: altmdm_get_protoversion
 *
 * Description:
 *   Returns the protocol version of the ALTCOM command.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns the protocol version of the ALTCOM command. ALTCOM_VERX means
 *   that the ALTCOM version check sequence has not been completed or is not
 *   supported.
 *
 ****************************************************************************/

uint8_t altmdm_get_protoversion(void);

#endif  /* __DRIVERS_MODEM_ALT1250_ALTMDM_H */

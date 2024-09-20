/****************************************************************************
 * include/nuttx/net/lan9250.h
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

#ifndef __INCLUDE_NUTTX_NET_LAN9250_H
#define __INCLUDE_NUTTX_NET_LAN9250_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The LAN9250 device typically provides interrupts to the MCU through a GPIO
 * pin. The structure below provides an MCU-independent mechanism for
 * controlling this LAN9250 GPIO interrupt.
 *
 * "When an enabled interrupt occurs, the interrupt pin will remain low until
 * all flags which are causing the interrupt are cleared or masked off
 * (enable bit is cleared) by the host controller."  However, the interrupt
 * will behave like a falling edge interrupt because "After an interrupt
 * occurs, the host controller [clears] the global enable bit for the
 * interrupt pin before servicing the interrupt. Clearing the enable bit
 * will cause the interrupt pin to return to the non-asserted state (high).
 * Doing so will prevent the host controller from missing a falling edge
 * should another interrupt occur while the immediate interrupt is being
 * serviced."
 */

struct lan9250_lower_s
{
  CODE int  (*attach)(FAR const struct lan9250_lower_s *lower,
                      xcpt_t handler, FAR void *arg);
  CODE void (*enable)(FAR const struct lan9250_lower_s *lower);
  CODE void (*disable)(FAR const struct lan9250_lower_s *lower);

  /* This function is optional and used to get a specific MAC address from
   * a MCU-specific implementation. If this function is NULL, the LAN9250
   * driver will not read the MAC from the CPU.
   */

  CODE int (*getmac)(FAR const struct lan9250_lower_s *lower,
                      FAR uint8_t *mac);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lan9250_initialize
 *
 * Description:
 *   Initialize the LAN9250 Ethernet driver.
 *
 * Input Parameters:
 *   spi   - A reference to the platform's SPI driver for the LAN9250 when
 *           enable CONFIG_LAN9250_SPI
 *   qspi  - A reference to the platform's SQI driver for the LAN9250 when
 *           enable CONFIG_LAN9250_SQI
 *   lower - The MCU-specific interrupt used to control low-level MCU
 *           functions (i.e., LAN9250 GPIO interrupts).
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_LAN9250_SPI
struct spi_dev_s;   /* see nuttx/spi/spi.h */
#else
struct qspi_dev_s;  /* see nuttx/spi/qspi.h */
#endif
int lan9250_initialize(
#ifdef CONFIG_LAN9250_SPI
                       FAR struct spi_dev_s *spi,
#else
                       FAR struct qspi_dev_s *qspi,
#endif
                       FAR const struct lan9250_lower_s *lower);

/****************************************************************************
 * Function: lan9250_uninitialize
 *
 * Description:
 *   Un-initialize the Ethernet driver
 *
 * Input Parameters:
 *   lower - The MCU-specific interrupt used to control low-level MCU
 *           functions (i.e., LAN9250 GPIO interrupts).
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int lan9250_uninitialize(FAR const struct lan9250_lower_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_LAN9250_H */

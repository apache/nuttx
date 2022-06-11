/****************************************************************************
 * include/nuttx/net/w5500.h
 * WIZnet W5500 Ethernet Controller
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

#ifndef __INCLUDE_NUTTX_NET_W5500_H
#define __INCLUDE_NUTTX_NET_W5500_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_NET_W5500

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes the lower-half interface provided by board-
 * specific logic.
 */

struct w5500_lower_s
{
  uint32_t frequency;   /* Frequency to use with SPI_SETFREQUENCY() */
  uint16_t spidevid;    /* Index used with SPIDEV_ETHERNET() macro */
  enum spi_mode_e mode; /* SPI more for use with SPI_SETMODE() */

  /* Lower-half callbacks:
   *
   * attach() - Attach the W5500 interrupt to the driver interrupt handler.
   * enable() - Enable or disable the W5500 interrupt.
   * reset()  - Set the RSTn pin to the provided state.
   */

  int  (*attach)(FAR const struct w5500_lower_s *lower, xcpt_t handler,
                 FAR void *arg);
  void (*enable)(FAR const struct w5500_lower_s *lower, bool enable);
  void (*reset)(FAR const struct w5500_lower_s *lower, bool reset);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: w5500_initialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver
 *
 * Parameters:
 *   spi   - A reference to the platform's SPI driver for the W5500.
 *   lower - The lower half driver instance for this W5500 chip.
 *   devno - If more than one W5500 is supported, then this is the
 *           zero based number that identifies the W5500.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int w5500_initialize(FAR struct spi_dev_s *spi_dev,
                     FAR const struct w5500_lower_s *lower,
                     unsigned int devno);

#endif /* CONFIG_NET_W5500 */
#endif /* __INCLUDE_NUTTX_NET_W5500_H */

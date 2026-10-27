/****************************************************************************
 * include/nuttx/wireless/morse.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_WIRELESS_MORSE_H
#define __INCLUDE_NUTTX_WIRELESS_MORSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/irq.h>
#include <nuttx/spi/spi.h>

#ifdef CONFIG_IEEE80211_MORSE

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The Morse Micro MM6108/MM8108 Wi-Fi HaLow (IEEE 802.11ah) transceiver is
 * connected to the host through an SPI bus plus a few GPIOs:
 *
 *   RESET_N  - output, active low, the chip is held in reset at boot
 *   WAKE     - output, active high, wakes the chip from deep sleep
 *   SPI_IRQ  - input, active low, the chip has pending events
 *   BUSY     - input, active high, used by the chip power save logic
 *
 * The board logic provides an instance of this structure describing how to
 * drive these lines.  The chip select must be driven by the board's
 * <spi>select() function for the device ID 'devid'.
 *
 * The lower half callbacks may be called from interrupt context unless
 * noted otherwise.
 */

struct morse_lower_s
{
  uint32_t devid;      /* SPI device ID used for chip select */
  uint32_t frequency;  /* SPI clock frequency (Hz), 0 means driver default */

  /* Control the RESET_N line: true = hold the chip in reset */

  CODE void (*reset)(FAR const struct morse_lower_s *lower, bool assert);

  /* Control the WAKE line: true = wake the chip.
   *
   * Read the BUSY line: true = BUSY asserted.
   *
   * WAKE and BUSY are needed for the chip power save.  They are optional:
   * if any of them is NULL (line not connected) the chip is kept always
   * awake.
   */

  CODE void (*wake)(FAR const struct morse_lower_s *lower, bool assert);
  CODE bool (*busy)(FAR const struct morse_lower_s *lower);

  /* Read the SPI_IRQ line: true = IRQ asserted (line low) */

  CODE bool (*irq_asserted)(FAR const struct morse_lower_s *lower);

  /* Attach the SPI_IRQ handler (falling edge interrupt) and enable or
   * disable that interrupt.
   */

  CODE int  (*irq_attach)(FAR const struct morse_lower_s *lower,
                          xcpt_t handler, FAR void *arg);
  CODE void (*irq_enable)(FAR const struct morse_lower_s *lower,
                          bool enable);

  /* Attach the BUSY handler (rising edge interrupt) and enable or disable
   * that interrupt.  These are optional and may be NULL.
   */

  CODE int  (*busy_attach)(FAR const struct morse_lower_s *lower,
                           xcpt_t handler, FAR void *arg);
  CODE void (*busy_enable)(FAR const struct morse_lower_s *lower,
                           bool enable);

  /* Optional: provide a board specific MAC address.  Return true if
   * 'mac' was filled with a valid address.  May be NULL, in which case the
   * address programmed in the chip OTP is used.  Not called from interrupt
   * context.
   */

  CODE bool (*getmac)(FAR const struct morse_lower_s *lower,
                      FAR uint8_t *mac);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: morse_register
 *
 * Description:
 *   Initialize the Morse Micro HaLow driver, boot the transceiver and
 *   register the network interface (wlanN).  The interface is then
 *   controlled with the standard wireless IOCTLs (for example with the
 *   wapi tool).
 *
 * Input Parameters:
 *   spi   - SPI bus the transceiver is connected to
 *   lower - Board specific lower half.  It must persist for the lifetime
 *           of the driver.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int morse_register(FAR struct spi_dev_s *spi,
                   FAR const struct morse_lower_s *lower);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_IEEE80211_MORSE */
#endif /* __INCLUDE_NUTTX_WIRELESS_MORSE_H */

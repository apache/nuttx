/****************************************************************************
 * arch/arm64/src/am62x/am62x_tisci.h
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

/* AM62x TISCI system-controller client.
 *
 * On K3 SoCs the A53 cluster does not directly own device power, clocks,
 * resets, or interrupt routing.  Those are managed by the DM/TIFS firmware
 * on the R5 and requested over the TISCI protocol via the secure-proxy
 * mailbox.  This client provides the minimal request helpers other AM62x
 * drivers use to power on a device, enable/set a functional clock, release
 * resets, and route an interrupt through the interrupt router.
 */

#ifndef __ARCH_ARM64_SRC_AM62X_AM62X_TISCI_H
#define __ARCH_ARM64_SRC_AM62X_AM62X_TISCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Selected TISCI device IDs (ti,sci-dev-id).
 * Source: Linux arch/arm64/boot/dts/ti/k3-am62-main.dtsi.  Extend as drivers
 * are added; the full list is in the TISCI device documentation for AM62x.
 */

#define AM62X_DEV_GPIOMUX_INTROUTER0  3    /* main_gpiomux_introuter0 */
#define AM62X_DEV_GPIO0               77   /* main_gpio0 @ 0x00600000 */
#define AM62X_DEV_GPIO1               78   /* main_gpio1 @ 0x00601000 */
#define AM62X_DEV_I2C0                102  /* main_i2c0 @ 0x20000000 */
#define AM62X_DEV_I2C1                103  /* main_i2c1 @ 0x20010000 */
#define AM62X_DEV_I2C2                104  /* main_i2c2 @ 0x20020000 */
#define AM62X_DEV_I2C3                105  /* main_i2c3 @ 0x20030000 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct am62x_tisci_version_s
{
  uint16_t firmware_revision;
  uint8_t  abi_major;
  uint8_t  abi_minor;
  char     firmware_description[33];   /* 32 chars + NUL terminator        */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: am62x_tisci_initialize
 *
 * Description:
 *   Initialise the TISCI client and verify the secure-proxy link by
 *   issuing a version handshake with the DM/TIFS firmware.  Must be called
 *   once, before any other am62x_tisci_* helper, early in board bring-up.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int am62x_tisci_initialize(void);

/****************************************************************************
 * Name: am62x_tisci_get_version
 *
 * Description:
 *   Query the running system firmware ABI version and description.
 *
 ****************************************************************************/

int am62x_tisci_get_version(struct am62x_tisci_version_s *ver);

/****************************************************************************
 * Name: am62x_tisci_set_device_state / am62x_tisci_get_device_state
 *
 * Description:
 *   Set or query the software power state of a device (one of
 *   MSG_DEVICE_SW_STATE_*).  get returns the current hardware state in
 *   *current_state (one of MSG_DEVICE_HW_STATE_*).
 *
 ****************************************************************************/

int am62x_tisci_set_device_state(uint32_t devid, uint8_t state);
int am62x_tisci_get_device_state(uint32_t devid, uint8_t *current_state);

/****************************************************************************
 * Name: am62x_tisci_module_enable
 *
 * Description:
 *   Power on a device, release all device-local resets, and request its
 *   functional clock.  This is the normal sequence before a driver touches
 *   AM62x/K3 peripheral registers.
 *
 ****************************************************************************/

int am62x_tisci_module_enable(uint32_t devid, uint32_t clkid);

/****************************************************************************
 * Name: am62x_tisci_set_device_resets
 *
 * Description:
 *   Program the device-local reset lines (bitmask; 0 releases all resets).
 *
 ****************************************************************************/

int am62x_tisci_set_device_resets(uint32_t devid, uint32_t resets);

/****************************************************************************
 * Name: am62x_tisci_set_clock_state / am62x_tisci_clk_enable
 *
 * Description:
 *   Set a clock's software state (one of MSG_CLOCK_SW_STATE_*).
 *   am62x_tisci_clk_enable is a wrapper that requests the clock on.
 *
 ****************************************************************************/

int am62x_tisci_set_clock_state(uint32_t devid, uint32_t clkid,
                                uint8_t state);
int am62x_tisci_clk_enable(uint32_t devid, uint32_t clkid);

/****************************************************************************
 * Name: am62x_tisci_set_clock_freq / am62x_tisci_get_clock_freq
 *
 * Description:
 *   Request a target frequency for a clock, or read its current frequency.
 *   set accepts the target in Hz and asks the firmware to honour it exactly.
 *
 ****************************************************************************/

int am62x_tisci_set_clock_freq(uint32_t devid, uint32_t clkid,
                               uint64_t freq_hz);
int am62x_tisci_get_clock_freq(uint32_t devid, uint32_t clkid,
                               uint64_t *freq_hz);

/****************************************************************************
 * Name: am62x_tisci_irq_set / am62x_tisci_irq_release
 *
 * Description:
 *   Program (or release) a route through an interrupt router so a peripheral
 *   output interrupt reaches the A53 GIC.  src_id is the router's device id,
 *   src_index its input line, dst_id the destination IRQ-controller device
 *   id (the GIC), and dst_host_irq the destination GIC interrupt input.
 *
 ****************************************************************************/

int am62x_tisci_irq_set(uint16_t src_id, uint16_t src_index,
                        uint16_t dst_id, uint16_t dst_host_irq);
int am62x_tisci_irq_release(uint16_t src_id, uint16_t src_index,
                            uint16_t dst_id, uint16_t dst_host_irq);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM64_SRC_AM62X_AM62X_TISCI_H */

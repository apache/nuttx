/****************************************************************************
 * arch/risc-v/src/esp32c6/hardware/esp32c6_rom_layout.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C6_HARDWARE_ESP32C6_ROM_LAYOUT_H
#define __ARCH_RISCV_SRC_ESP32C6_HARDWARE_ESP32C6_ROM_LAYOUT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Structure and functions for returning ROM global layout
 *
 * This is for address symbols defined in the linker script,
 * which may change during ECOs.
 */

struct esp32c6_rom_layout_s
{
  uintptr_t dram0_stack_shared_mem_start;
  uintptr_t dram0_rtos_reserved_start;
  uintptr_t stack_sentry;
  uintptr_t stack;

  /* BTDM data */

  uintptr_t data_start_btdm;
  uintptr_t data_end_btdm;
  uintptr_t bss_start_btdm;
  uintptr_t bss_end_btdm;
  uintptr_t data_start_btdm_rom;
  uintptr_t data_start_interface_btdm;
  uintptr_t data_end_interface_btdm;
  uintptr_t bss_start_interface_btdm;
  uintptr_t bss_end_interface_btdm;

  /* BTBB data */

  uintptr_t dram_start_btbbrom;
  uintptr_t dram_end_btbbrom;

  /* PHY data */

  uintptr_t dram_start_phyrom;
  uintptr_t dram_end_phyrom;

  /* Wi-Fi data */

  uintptr_t dram_start_net80211;
  uintptr_t dram_end_net80211;
  uintptr_t data_start_interface_net80211;
  uintptr_t data_end_interface_net80211;
  uintptr_t bss_start_interface_net80211;
  uintptr_t bss_end_interface_net80211;
  uintptr_t dram_start_pp;
  uintptr_t dram_end_pp;
  uintptr_t data_start_interface_pp;
  uintptr_t data_end_interface_pp;
  uintptr_t bss_start_interface_pp;
  uintptr_t bss_end_interface_pp;

  /* Coexist data */

  uintptr_t dram_start_coexist;
  uintptr_t dram_end_coexist;
  uintptr_t data_start_interface_coexist;
  uintptr_t data_end_interface_coexist;
  uintptr_t bss_start_interface_coexist;
  uintptr_t bss_end_interface_coexist;

  /* USB device data */

  uintptr_t dram_start_usbdev_rom;
  uintptr_t dram_end_usbdev_rom;
  uintptr_t dram_start_uart_rom;
  uintptr_t dram_end_uart_rom;
};

#endif /* __ARCH_RISCV_SRC_ESP32C6_HARDWARE_ESP32C6_ROM_LAYOUT_H */


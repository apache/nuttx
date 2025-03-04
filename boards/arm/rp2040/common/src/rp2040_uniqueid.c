/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_uniqueid.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "rp2040_uniqueid.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ROM_FUNC_CIF                      ROM_TABLE_CODE('I', 'F')
#define ROM_FUNC_FEX                      ROM_TABLE_CODE('E', 'X')
#define ROM_FUNC_FFC                      ROM_TABLE_CODE('F', 'C')

#define QSPI_SS_CTRL_OUTOVER_LSB          8
#define QSPI_SS_CTRL_OUTOVER_VALUE_LOW    0x2
#define QSPI_SS_CTRL_OUTOVER_VALUE_HIGH   0x3
#define QSPI_SS_CTRL_OUTOVER_BITS         0x00000300
#define QSPI_SS_CTRL                      0x4001800c
#define XIP_BASE                          0x10000000
#define XIP_SSI_SR                        0x18000028
#define XIP_SSI_DR0                       0x18000060
#define SSI_SR_TFNF_BITS                  0x00000002
#define SSI_SR_RFNE_BITS                  0x00000008
#define BOOT2_SIZE_WORDS                  64
#define REG_ALIAS_XOR_BITS                (0x1u << 12u)

#define ROM_TABLE_CODE(c1, c2)     ((c1) | ((c2) << 8))
#define hw_alias_check_addr(addr)  ((uintptr_t)(addr))
#define hw_xor_alias_untyped(addr) ((void *)(REG_ALIAS_XOR_BITS | hw_alias_check_addr(addr)))

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef volatile uint32_t io_rw_32;
typedef void (*rom_cif_fn)(void);
typedef void (*rom_fex_fn)(void);
typedef void (*rom_ffc_fn)(void);
typedef void (*rom_flash_enter_cmd_xip_fn)(void);
typedef void *(*rom_table_lookup_fn)(uint16_t *table, uint32_t code);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void __compiler_memory_barrier(void);
static inline void *rom_hword_as_ptr(uint16_t rom_address);
static inline uint32_t rom_table_code(uint8_t c1, uint8_t c2);
static void *rf_lookup(uint32_t code);
static void hw_xor_bits(io_rw_32 *addr, uint32_t mask);
static void hw_write_masked(io_rw_32 *addr,
                            uint32_t values,
                            uint32_t write_mask);
static void flash_cs_force (bool high);
void rp2040_flash_cmd(const uint8_t *txbuf, uint8_t *rxbuf, size_t count);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_uniqueid[CONFIG_BOARDCTL_UNIQUEID_SIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __compiler_memory_barrier
 *
 * Description:
 *   Prevent compiler from moving memory access across this barrier.
 *
 ****************************************************************************/

static inline void __compiler_memory_barrier(void)
{
}

/****************************************************************************
 * Name: rom_hword_as_ptr
 *
 * Description:
 *   Converts a (well known) address value into a pointer to that address.
 *
 ****************************************************************************/

static inline void *rom_hword_as_ptr(uint16_t rom_address)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
  return (void *)(uintptr_t)*(uint16_t *)(uintptr_t)rom_address;
#pragma GCC diagnostic pop
}

/****************************************************************************
 * Name: <Static function name>
 *
 * Description:
 *   Return a bootrom lookup code based on two ASCII characters.
 *
 * Input Parameters:
 *   uint8_t c1: first character
 *   uint8_t c2: second character
 *
 * Returned Value:
 *   A code to use with rf_lookup
 *
 ****************************************************************************/

static inline uint32_t rom_table_code(uint8_t c1, uint8_t c2)
{
  return ROM_TABLE_CODE((uint32_t) c1, (uint32_t) c2);
}

/****************************************************************************
 * Name: rf_lookup
 *
 * Description:
 *   Lookup a bootrom function by code.
 *
 * Input Parameters:
 *   uint32_t code: A code from rom_table_code()
 *
 * Returned Value:
 *   a pointer to the function, or NULL if the code does not match any
 *   bootrom function
 *
 ****************************************************************************/

always_inline_function static void *rf_lookup(uint32_t code)
{
  rom_table_lookup_fn rom_table_lookup;
  rom_table_lookup = (rom_table_lookup_fn) rom_hword_as_ptr(0x18);
  uint16_t *func_table = (uint16_t *) rom_hword_as_ptr(0x14);
  return rom_table_lookup(func_table, code);
}

/****************************************************************************
 * Name: hw_xor_bits
 *
 * Description:
 *   Helper function for flash_cs_force.
 *
 ****************************************************************************/

always_inline_function static void hw_xor_bits(io_rw_32 *addr, uint32_t mask)
{
  *(io_rw_32 *) hw_xor_alias_untyped((volatile void *) addr) = mask;
}

/****************************************************************************
 * Name: hw_write_masked
 *
 * Description:
 *   Helper function for flash_cs_force.
 *
 ****************************************************************************/

always_inline_function static void hw_write_masked(io_rw_32 *addr,
                                           uint32_t values,
                                           uint32_t write_mask)
{
  hw_xor_bits(addr, (*addr ^ values) & write_mask);
}

/****************************************************************************
 * Name: flash_cs_force
 *
 * Description:
 *   Override the chip select line to flash chip.
 *
 * Input Parameters:
 *   bool high: true to force CS high, false to force low
 *
 ****************************************************************************/

noinline_function locate_code(".ram_code.flash_cs_force")
static void flash_cs_force (bool high)
{
  uint32_t field_val = high ?
    QSPI_SS_CTRL_OUTOVER_VALUE_HIGH :
    QSPI_SS_CTRL_OUTOVER_VALUE_LOW;
  hw_write_masked((io_rw_32 *)QSPI_SS_CTRL,
    field_val << QSPI_SS_CTRL_OUTOVER_LSB,
    QSPI_SS_CTRL_OUTOVER_BITS
  );
}

/****************************************************************************
 * Name: rp2040_flash_cmd
 *
 * Description:
 *   Send a command to flash chip and receive the result.
 *
 * Input Parameters:
 *   uint8_t* txbuf: Pointer to buffer to send
 *   uint8_t* rxbuf: Pointer to buffer to hold received value
 *   size_t   count: Number of bytes to send / receive
 *
 ****************************************************************************/

noinline_function locate_code(".ram_code.rp2040_flash_cmd")
void rp2040_flash_cmd(const uint8_t *txbuf, uint8_t *rxbuf, size_t count)
{
  rom_cif_fn connect_internal_flash = (rom_cif_fn)rf_lookup(ROM_FUNC_CIF);
  rom_fex_fn flash_exit_xip = (rom_fex_fn)rf_lookup(ROM_FUNC_FEX);
  rom_ffc_fn flash_flush_cache = (rom_ffc_fn)rf_lookup(ROM_FUNC_FFC);

  uint32_t boot2_copyout[BOOT2_SIZE_WORDS];
  for (int i = 0; i < BOOT2_SIZE_WORDS; ++i)
    boot2_copyout[i] = ((uint32_t *)XIP_BASE)[i];
  __compiler_memory_barrier();
  connect_internal_flash();
  flash_exit_xip();

  flash_cs_force(0);
  size_t tx_cnt = count;
  size_t rx_cnt = count;

  const size_t max_in_flight = 16 - 2;
  while (tx_cnt || rx_cnt)
    {
      uint32_t flags = *((uint32_t *)XIP_SSI_SR);

      bool can_put = !!(flags & SSI_SR_TFNF_BITS);
      bool can_get = !!(flags & SSI_SR_RFNE_BITS);

      if (can_put && tx_cnt && rx_cnt - tx_cnt < max_in_flight)
        {
          *((uint8_t *)XIP_SSI_DR0) = *txbuf++;
          --tx_cnt;
        }

      if (can_get && rx_cnt)
        {
          *rxbuf++ = *((uint8_t *)XIP_SSI_DR0);
          --rx_cnt;
        }
    }

  flash_cs_force(1);
  flash_flush_cache();
  ((void (*)(void))((intptr_t)boot2_copyout + 1))(); /* re-enable xip */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_uniqueid_initialize
 *
 * Description:
 *   The RP2040 doesn't have a unique ID, so we load the ID from the
 *   connected flash chip.  We use the flash ID to seed a simple xorshift
 *   PRNG.  The PRNG then generates CONFIG_BOARDCTL_UNIQUEID_SIZE bytes,
 *   which we will use as the board's unique ID.
 *
 *   Retrieving the flash id is somewhat slow and complex, so we only do
 *   this during initialization and store the result for later use.
 *
 * Assumptions/Limitations:
 *   This uniqueid implementation requires a flash chip.  It should not be
 *   used on boards without flash.
 *
 ****************************************************************************/

void rp2040_uniqueid_initialize(void)
{
  uint64_t x;
  uint8_t  txbuf[RP2040_FLASH_ID_BUFFER_SIZE];
  uint8_t  rxbuf[RP2040_FLASH_ID_BUFFER_SIZE];

  memset(g_uniqueid, 0xac, CONFIG_BOARDCTL_UNIQUEID_SIZE);
  memset(txbuf, 0, RP2040_FLASH_ID_BUFFER_SIZE);
  memset(rxbuf, 0, RP2040_FLASH_ID_BUFFER_SIZE);
  txbuf[0] = RP2040_FLASH_RUID_CMD;

  rp2040_flash_cmd(txbuf, rxbuf, RP2040_FLASH_ID_BUFFER_SIZE);

  /* xorshift PRNG: */

  x = *(uint64_t *)(rxbuf + RP2040_FLASH_ID_BUFFER_OFFSET);
  for (int i = 0; i < CONFIG_BOARDCTL_UNIQUEID_SIZE; i++)
    {
      x ^= x >> 12;
      x ^= x << 25;
      x ^= x >> 27;
      g_uniqueid[i] = (uint8_t)((x * 0x2545f4914f6cdd1dull) >> 32);
    }
}

/****************************************************************************
 * Name: board_uniqueid
 *
 * Description:
 *   Return a unique ID associated with the board.
 *
 * Input Parameters:
 *   uniqueid - A reference to a writable memory location provided by the
 *     caller to receive the board unique ID.  The memory memory referenced
 *     by this pointer must be at least CONFIG_BOARDCTL_UNIQUEID_SIZE in
 *     length.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwize a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ****************************************************************************/

int board_uniqueid(uint8_t *uniqueid)
{
  memcpy(uniqueid, g_uniqueid, CONFIG_BOARDCTL_UNIQUEID_SIZE);
  return OK;
}

/****************************************************************************
 * arch/risc-v/src/esp32c3/hardware/esp32c3_soc.h
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ESP32C3_HARDWARE_ESP32C3_SOC_H
#define __ARCH_RISCV_SRC_ESP32C3_HARDWARE_ESP32C3_SOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "esp32c3_attr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DR_REG_SYSTEM_BASE                      0x600c0000
#define DR_REG_SENSITIVE_BASE                   0x600c1000
#define DR_REG_INTERRUPT_BASE                   0x600c2000
#define DR_REG_DMA_COPY_BASE                    0x600c3000
#define DR_REG_EXTMEM_BASE                      0x600c4000
#define DR_REG_MMU_TABLE                        0x600c5000
#define DR_REG_AES_BASE                         0x6003a000
#define DR_REG_SHA_BASE                         0x6003b000
#define DR_REG_RSA_BASE                         0x6003c000
#define DR_REG_HMAC_BASE                        0x6003e000
#define DR_REG_DIGITAL_SIGNATURE_BASE           0x6003d000
#define DR_REG_GDMA_BASE                        0x6003f000
#define DR_REG_ASSIST_DEBUG_BASE                0x600ce000
#define DR_REG_DEDICATED_GPIO_BASE              0x600cf000
#define DR_REG_WORLD_CNTL_BASE                  0x600d0000
#define DR_REG_DPORT_END                        0x600d3FFC
#define DR_REG_UART_BASE                        0x60000000
#define DR_REG_SPI1_BASE                        0x60002000
#define DR_REG_SPI0_BASE                        0x60003000
#define DR_REG_GPIO_BASE                        0x60004000
#define DR_REG_FE2_BASE                         0x60005000
#define DR_REG_FE_BASE                          0x60006000
#define DR_REG_RTCCNTL_BASE                     0x60008000
#define DR_REG_IO_MUX_BASE                      0x60009000
#define DR_REG_RTC_I2C_BASE                     0x6000e000
#define DR_REG_UART1_BASE                       0x60010000
#define DR_REG_I2C_EXT_BASE                     0x60013000
#define DR_REG_UHCI0_BASE                       0x60014000
#define DR_REG_RMT_BASE                         0x60016000
#define DR_REG_LEDC_BASE                        0x60019000
#define DR_REG_EFUSE_BASE                       0x60008800
#define DR_REG_NRX_BASE                         0x6001CC00
#define DR_REG_BB_BASE                          0x6001D000
#define DR_REG_TIMERGROUP0_BASE                 0x6001F000
#define DR_REG_TIMERGROUP1_BASE                 0x60020000
#define DR_REG_SYS_TIMER_BASE                   0x60023000
#define DR_REG_SPI2_BASE                        0x60024000
#define DR_REG_SYSCON_BASE                      0x60026000
#define DR_REG_APB_CTRL_BASE                    0x60026000 /* Old name for SYSCON */
#define DR_REG_TWAI_BASE                        0x6002B000
#define DR_REG_I2S0_BASE                        0x6002D000
#define DR_REG_APB_SARADC_BASE                  0x60040000
#define DR_REG_AES_XTS_BASE                     0x600CC000

/* Registers Operation */

#define REG_UHCI_BASE(i)      (DR_REG_UHCI0_BASE - (i) * 0x8000)
#define REG_UART_BASE(i)      (DR_REG_UART_BASE + (i) * 0x10000 + \
                               ( (i) > 1 ? 0xe000 : 0 ) )
#define REG_UART_AHB_BASE(i)  (0x60000000 + (i) * 0x10000 + \
                               ( (i) > 1 ? 0xe000 : 0 ) )
#define UART_FIFO_AHB_REG(i)  (REG_UART_AHB_BASE(i) + 0x0)
#define REG_I2S_BASE(i)       (DR_REG_I2S_BASE + (i) * 0x1E000)
#define REG_TIMG_BASE(i)      (DR_REG_TIMERGROUP0_BASE + (i)*0x1000)
#define REG_SPI_MEM_BASE(i)   (DR_REG_SPI0_BASE - (i) * 0x1000)
#define REG_SPI_BASE(i)       (DR_REG_SPI2_BASE)
#define REG_I2C_BASE(i)       (DR_REG_I2C_EXT_BASE + (i) * 0x14000)

/* Peripheral Clock */

#define APB_CLK_FREQ_ROM        (40 * 1000000)
#define CPU_CLK_FREQ_ROM        APB_CLK_FREQ_ROM
#define UART_CLK_FREQ_ROM       (40 * 1000000)
#define EFUSE_CLK_FREQ_ROM      (20 * 1000000)
#define CPU_CLK_FREQ            APB_CLK_FREQ
#define APB_CLK_FREQ            (80 * 1000000)
#define REF_CLK_FREQ            (1000000)
#define RTC_CLK_FREQ            (20 * 1000000)
#define XTAL_CLK_FREQ           (40 * 1000000)
#define UART_CLK_FREQ           APB_CLK_FREQ
#define WDT_CLK_FREQ            APB_CLK_FREQ
#define TIMER_CLK_FREQ          (80000000 >> 4) /* 80MHz divided by 16 */
#define SPI_CLK_DIV             4
#define TICKS_PER_US_ROM        40    /* CPU is 80MHz */
#define GPIO_MATRIX_DELAY_NS    0

/* Overall memory map */

#define SOC_DROM_LOW            0x3c000000
#define SOC_DROM_HIGH           0x3c800000
#define SOC_IROM_LOW            0x42000000
#define SOC_IROM_HIGH           0x42800000
#define SOC_IROM_MASK_LOW       0x40000000
#define SOC_IROM_MASK_HIGH      0x40060000
#define SOC_DROM_MASK_LOW       0x3ff00000
#define SOC_DROM_MASK_HIGH      0x3ff20000
#define SOC_IRAM_LOW            0x4037c000
#define SOC_IRAM_HIGH           0x403e0000
#define SOC_DRAM_LOW            0x3fc80000
#define SOC_DRAM_HIGH           0x3fce0000
#define SOC_RTC_RAM_LOW         0x50000000 /* ESP32-C3 only has RTC fast memory */
#define SOC_RTC_RAM_HIGH        0x50002000

/* First and last words of the D/IRAM region, for both the DRAM address as
 * well as the IRAM alias.
 */

#define SOC_DIRAM_IRAM_LOW      0x40380000
#define SOC_DIRAM_IRAM_HIGH     0x403e0000
#define SOC_DIRAM_DRAM_LOW      0x3fc80000
#define SOC_DIRAM_DRAM_HIGH     0x3fce0000

/*  Region of memory accessible via DMA. See esp_ptr_dma_capable(). */

#define SOC_DMA_LOW             0x3fc88000
#define SOC_DMA_HIGH            0x3fd00000

/* Region of RAM that is byte-accessible. See esp_ptr_byte_accessible(). */

#define SOC_BYTE_ACCESSIBLE_LOW   0x3fc88000
#define SOC_BYTE_ACCESSIBLE_HIGH  0x3fd00000

/* Region of memory that is internal, as in on the same silicon die as the
 * ESP32-C3 CPU (excluding RTC data region, that's checked separately.)
 * See esp_ptr_internal().
 */

#define SOC_MEM_INTERNAL_LOW    0x3fc80000
#define SOC_MEM_INTERNAL_HIGH   0x3fce0000
#define SOC_MEM_INTERNAL_LOW1   0x40370000
#define SOC_MEM_INTERNAL_HIGH1  0x403e0000
#define SOC_MEM_INTERNAL_LOW2   0x600fe000
#define SOC_MEM_INTERNAL_HIGH2  0x60100000

/* Largest span of contiguous memory (DRAM or IRAM) in the address space */

#define SOC_MAX_CONTIGUOUS_RAM_SIZE (SOC_IRAM_HIGH - SOC_IRAM_LOW)

/* Region of address space that holds peripherals */

#define SOC_PERIPHERAL_LOW      0x60000000
#define SOC_PERIPHERAL_HIGH     0x60100000

/* Debug region, not used by software */

#define SOC_DEBUG_LOW           0x20000000
#define SOC_DEBUG_HIGH          0x28000000

/* Start (highest address) of ROM boot stack, only relevant during
 * early boot
 */

#define SOC_ROM_STACK_START     0x3fcebf10

/* Interrupt cpu using table */

/****************************************************************************
 *Intr num  Level     Type          PRO CPU usage
 *   0        1   extern level      WMAC
 *   1        1   extern level      BT/BLE Host VHCI
 *   2        1   extern level
 *   3        1   extern level
 *   4        1   extern level      WBB
 *   5        1   extern level      BT/BLE Controller
 *   6        1   timer             RTOS Tick
 *   7        1   software          BT/BLE VHCI
 *   8        1   extern level      BT/BLE BB(RX/TX)
 *   9        1   extern level
 *   10       1   extern edge
 *   11       3   profiling
 *   12       1   extern level
 *   13       1   extern level
 *   14       7   nmi               Reserved
 *   15       3   timer             RTOS Tick(L3)
 *   16       5   timer
 *   17       1   extern level
 *   18       1   extern level
 *   19       2   extern level
 *   20       2   extern level
 *   21       2   extern level
 *   22       3   extern edge
 *   23       3   extern level
 *   24       4   extern level      TG1_WDT
 *   25       4   extern level      CACHEERR
 *   26       5   extern level
 *   27       3   extern level      Reserved
 *   28       4   extern edge       DPORT ACCESS
 *   29       3   software          Reserved
 *   30       4   extern edge       Reserved
 *   31       5   extern level
 ****************************************************************************/

/* CPU Interrupt number reserved, do not touch this. */

#define ETS_WMAC_INUM               1
/* #define ETS_BT_HOST_INUM         1 */
#define ETS_WBB_INUM                4
#define ETS_SYSTICK_INUM            9
#define ETS_TG0_T1_INUM             10 /* use edge interrupt */
#define ETS_CPU_INTR0_INUM          12 /* used as freertos soft intr */
#define ETS_FRC1_INUM               22
#define ETS_T1_WDT_INUM             24
#define ETS_CACHEERR_INUM           25
#define ETS_DPORT_INUM              28

/* CPU Max valid interrupt number */

#define ETS_MAX_INUM                31

/* CPU Interrupt number used in ROM, should be cancelled in SDK */

#define ETS_SLC_INUM                1
#define ETS_UART0_INUM              5
#define ETS_UART1_INUM              5
#define ETS_SPI2_INUM               1

/* CPU Interrupt number used in ROM code only when module init function
 * called, should pay attention here.
 */

#define ETS_GPIO_INUM               4

/* Other interrupt number should be managed by the user */

/* Invalid interrupt for number interrupt matrix */

#define ETS_INVALID_INUM            0

/* Interrupt medium level, used for INT WDT for example */

#define SOC_INTERRUPT_LEVEL_MEDIUM  4

#define BIT(nr)                     (1UL << (nr))

/* Extract the field from the register and shift it to avoid wrong reading */

#define REG_MASK(_reg, _field) (((_reg) & (_field##_M)) >> (_field##_S))

/* Helper to place a value in a field */

#define VALUE_TO_FIELD(_value, _field) (((_value) << (_field##_S)) & (_field##_M))
#define DPORT_CPUPERIOD_SEL_80      0
#define DPORT_CPUPERIOD_SEL_160     1

#define DPORT_SOC_CLK_SEL_XTAL    0
#define DPORT_SOC_CLK_SEL_PLL    1
#define DPORT_SOC_CLK_SEL_8M     2

/* Write value to register */

#define REG_WRITE(_r, _v)    (*(volatile uint32_t *)(_r)) = (_v)

/* Read value from register */

#define REG_READ(_r) (*(volatile uint32_t *)(_r))

/* Get bit or get bits from register */

#define REG_GET_BIT(_r, _b)  (*(volatile uint32_t*)(_r) & (_b))

/* Set bit or set bits to register */

#define REG_SET_BIT(_r, _b)  (*(volatile uint32_t*)(_r) |= (_b))

/* Clear bit or clear bits of register */

#define REG_CLR_BIT(_r, _b)  (*(volatile uint32_t*)(_r) &= ~(_b))

/* Get field from register,
 * used when _f is not left shifted by _f##_S
 */

#define REG_GET_FIELD(_r, _f) ((REG_READ(_r) >> (_f##_S)) & (_f##_V))

/* Set field to register,
 * used when _f is not left shifted by _f##_S
 */

#define REG_SET_FIELD(_r, _f, _v) (REG_WRITE((_r),((REG_READ(_r) & ~((_f##_V) << (_f##_S)))|(((_v) & (_f##_V))<<(_f##_S)))))

#define SOC_SYSTIMER_BIT_WIDTH_LO (32) /* Bit width of systimer low part */
#define SOC_SYSTIMER_BIT_WIDTH_HI (20) /* Bit width of systimer high part */

/*  phy registers and memory size */

#define SOC_PHY_DIG_REGS_MEM_SIZE (21*4)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_ptr_rtc
 *
 * Description:
 *   Check if the buffer comes from the RTC RAM.
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32c3_ptr_rtc(const void *p)
{
  return ((intptr_t)p >= SOC_RTC_RAM_LOW &&
          (intptr_t)p < SOC_RTC_RAM_HIGH);
}

#endif /* __ARCH_RISCV_SRC_ESP32C3_HARDWARE_ESP32C3_SOC_H */

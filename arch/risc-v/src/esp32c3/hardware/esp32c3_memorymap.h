/****************************************************************************
 * arch/risc-v/src/esp32c3/hardware/esp32c3_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_HARDWARE_ESP32C3_MEMORYMAP_H
#define __ARCH_RISCV_SRC_ESP32C3_HARDWARE_ESP32C3_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32C3_SYSTEM_BASE                      0x600c0000
#define ESP32C3_SENSITIVE_BASE                   0x600c1000
#define ESP32C3_INTERRUPT_BASE                   0x600c2000
#define ESP32C3_DMA_COPY_BASE                    0x600c3000
#define ESP32C3_EXTMEM_BASE                      0x600c4000
#define ESP32C3_MMU_TABLE                        0x600c5000
#define ESP32C3_AES_BASE                         0x6003a000
#define ESP32C3_SHA_BASE                         0x6003b000
#define ESP32C3_RSA_BASE                         0x6003c000
#define ESP32C3_HMAC_BASE                        0x6003e000
#define ESP32C3_DIGITAL_SIGNATURE_BASE           0x6003d000
#define ESP32C3_GDMA_BASE                        0x6003f000
#define ESP32C3_ASSIST_DEBUG_BASE                0x600ce000
#define ESP32C3_DEDICATED_GPIO_BASE              0x600cf000
#define ESP32C3_WORLD_CNTL_BASE                  0x600d0000
#define ESP32C3_DPORT_END                        0x600d3ffc
#define ESP32C3_UART_BASE                        0x60000000
#define ESP32C3_SPI1_BASE                        0x60002000
#define ESP32C3_SPI0_BASE                        0x60003000
#define ESP32C3_GPIO_BASE                        0x60004000
#define ESP32C3_FE2_BASE                         0x60005000
#define ESP32C3_FE_BASE                          0x60006000
#define ESP32C3_RTCCNTL_BASE                     0x60008000
#define ESP32C3_IO_MUX_BASE                      0x60009000
#define ESP32C3_RTC_I2C_BASE                     0x6000e000
#define ESP32C3_UART1_BASE                       0x60010000
#define ESP32C3_I2C_EXT_BASE                     0x60013000
#define ESP32C3_UHCI0_BASE                       0x60014000
#define ESP32C3_RMT_BASE                         0x60016000
#define ESP32C3_LEDC_BASE                        0x60019000
#define ESP32C3_EFUSE_BASE                       0x60008800
#define ESP32C3_NRX_BASE                         0x6001cc00
#define ESP32C3_BB_BASE                          0x6001d000
#define ESP32C3_TIMERGROUP0_BASE                 0x6001f000
#define ESP32C3_TIMERGROUP1_BASE                 0x60020000
#define ESP32C3_SYS_TIMER_BASE                   0x60023000
#define ESP32C3_SPI2_BASE                        0x60024000
#define ESP32C3_SYSCON_BASE                      0x60026000
#define ESP32C3_APB_CTRL_BASE                    0x60026000    /* Old name for SYSCON */
#define ESP32C3_TWAI_BASE                        0x6002b000
#define ESP32C3_I2S0_BASE                        0x6002d000
#define ESP32C3_APB_SARADC_BASE                  0x60040000
#define ESP32C3_AES_XTS_BASE                     0x600cc000

#endif /* __ARCH_RISCV_SRC_ESP32C3_HARDWARE_ESP32C3_MEMORYMAP_H */

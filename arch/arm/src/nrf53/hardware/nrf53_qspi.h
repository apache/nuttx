/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_qspi.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_QSPI_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_QSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF53_QSPI_TASKS_ACTIVATE_OFFSET    0x0000 /* Activate QSPI interface */
#define NRF53_QSPI_TASKS_READSTART_OFFSET   0x0004 /* Start transfer from external flash memory to internal RAM */
#define NRF53_QSPI_TASKS_WRITESTART_OFFSET  0x0008 /* Start transfer from internal RAM to external flash memory */
#define NRF53_QSPI_TASKS_ERASESTART_OFFSET  0x000c /* Start external flash memory erase operation */
#define NRF53_QSPI_TASKS_DEACTIVATE_OFFSET  0x0010 /* Deactivate QSPI interface */
#define NRF53_QSPI_EVENTS_READY_OFFSET      0x0100 /* QSPI peripheral is ready */
#define NRF53_QSPI_INTEN_OFFSET             0x0300 /* Enable or disable interrupt */
#define NRF53_QSPI_INTENSET_OFFSET          0x0304 /* Enable interrupt */
#define NRF53_QSPI_INTENCLR_OFFSET          0x0308 /* Disable interrupt */
#define NRF53_QSPI_ENABLE_OFFSET            0x0500 /* Enable QSPI peripheral */
#define NRF53_QSPI_READ_SRC_OFFSET          0x0504 /* Flash memory source address */
#define NRF53_QSPI_READ_DST_OFFSET          0x0508 /* RAM destination address */
#define NRF53_QSPI_READ_CNT_OFFSET          0x050c /* Read transfer length */
#define NRF53_QSPI_WRITE_SRC_OFFSET         0x0510 /* Flash destination address */
#define NRF53_QSPI_WRITE_DST_OFFSET         0x0514 /* RAM source address */
#define NRF53_QSPI_WRITE_CNT_OFFSET         0x0518 /* Write transfer length */
#define NRF53_QSPI_ERASE_PTR_OFFSET         0x051c /* Start address of flash block to be erased */
#define NRF53_QSPI_ERASE_LEN_OFFSET         0x0520 /* Size of block to be erased */
#define NRF53_QSPI_PSEL_SCK_OFFSET          0x0524 /* Pin select for serial clock SCK */
#define NRF53_QSPI_PSEL_CSN_OFFSET          0x0528 /* Pin select for chip select signal CSN */
#define NRF53_QSPI_PSEL_IO0_OFFSET          0x0530 /* Pin select for serial data MOSI/IO0 */
#define NRF53_QSPI_PSEL_IO1_OFFSET          0x0534 /* Pin select for serial data MISO/IO1 */
#define NRF53_QSPI_PSEL_IO2_OFFSET          0x0538 /* Pin select for serial data IO2 */
#define NRF53_QSPI_PSEL_IO3_OFFSET          0x053c /* Pin select for serial data IO3 */
#define NRF53_QSPI_XIPOFFSET_OFFSET         0x0540 /* Address offset into the external memory for XIP */
#define NRF53_QSPI_IFCONFIG0_OFFSET         0x0544 /* Interface configuration */
#define NRF53_QSPI_XIPEN_OFFSET             0x054c /* Enable Execute in Place operation. */
#define NRF53_QSPI_XIPENC_KEY0_OFFSET       0x0560 /* Bits 31:0 of XIP AES KEY */
#define NRF53_QSPI_XIPENC_KEY1_OFFSET       0x0564 /* Bits 63:32 of XIP AES KEY */
#define NRF53_QSPI_XIPENC_KEY2_OFFSET       0x0568 /* Bits 95:64 of XIP AES KEY */
#define NRF53_QSPI_XIPENC_KEY3_OFFSET       0x056c /* Bits 95:64 of XIP AES KEY */
#define NRF53_QSPI_XIPENC_NONCE0_OFFSET     0x0570 /* Bits 127:96 of XIP AES KEY */
#define NRF53_QSPI_XIPENC_NONCE1_OFFSET     0x0574 /* Bits 31:0 of XIP NONCE */
#define NRF53_QSPI_XIPENC_NONCE2_OFFSET     0x0578 /* Bits 63:32 of XIP NONCE */
#define NRF53_QSPI_XIPENC_ENABLE_OFFSET     0x057c /* Enable stream cipher for XIP */
#define NRF53_QSPI_DMAENC_KEY0_OFFSET       0x0580 /* Bits 31:0 of DMA AES KEY */
#define NRF53_QSPI_DMAENC_KEY1_OFFSET       0x0584 /* Bits 63:32 of DMA AES KEY */
#define NRF53_QSPI_DMAENC_KEY2_OFFSET       0x0588 /* Bits 95:64 of DMA AES KEY */
#define NRF53_QSPI_DMAENC_KEY3_OFFSET       0x058c /* Bits 127:96 of DMA AES KEY */
#define NRF53_QSPI_DMAENC_NONCE0_OFFSET     0x0590 /* Bits 31:0 of DMA NONCE */
#define NRF53_QSPI_DMAENC_NONCE1_OFFSET     0x0594 /* Bits 63:32 of DMA NONCE */
#define NRF53_QSPI_DMAENC_NONCE2_OFFSET     0x0598 /* Bits 95:64 of DMA NONCE */
#define NRF53_QSPI_DMAENC_ENABLE_OFFSET     0x059c /* Enable stream cipher for EasyDMA */
#define NRF53_QSPI_IFCONFIG1_OFFSET         0x0600 /* Interface configuration */
#define NRF53_QSPI_STATUS_OFFSET            0x0604 /* Status register */
#define NRF53_QSPI_DPMDUR_OFFSET            0x0614 /* DPM duration */
#define NRF53_QSPI_ADDRCONF_OFFSET          0x0624 /* Extended address configuration. */
#define NRF53_QSPI_CINSTRCONF_OFFSET        0x0634 /* Custom instruction configuration register. */
#define NRF53_QSPI_CINSTRDAT0_OFFSET        0x0638 /* Custom instruction data register 0. */
#define NRF53_QSPI_CINSTRDAT1_OFFSET        0x063c /* Custom instruction data register 1. */
#define NRF53_QSPI_IFTIMING_OFFSET          0x0640 /* SPI interface timing. */

/* Register Bitfield Definitions ********************************************/

/* INTEN/INTENSET/INTENCLR */

#define QSPI_INT_READY                   (1 << 0)

/* ENABLE */

#define QSPI_ENABLE_EN                   (1)
#define QSPI_ENABLE_DIS                  (0)

/* ERASE.LEN */

#define QSPI_ERASE_SECTOR                (0)
#define QSPI_ERASE_PAGE                  (1)
#define QSPI_ERASE_ALL                   (2)

/* PSEL */

#define QSPI_PSEL_PIN_SHIFT              (0)
#define QSPI_PSEL_PORT_SHIFT             (5)
#define QSPI_PSEL_CONNECT                (1 << 31)

/* IFCONFIG0 */

#define QSPI_IFCONFIG0_READOC_SHIFT      (0)
#define QSPI_IFCONFIG0_READOC_MASK       (0x7 << QSPI_IFCONFIG0_READOC_SHIFT)
#  define QSPI_IFCONFIG0_READOC_FASTREAD (0 << QSPI_IFCONFIG0_READOC_SHIFT)
#  define QSPI_IFCONFIG0_READOC_READ2O   (1 << QSPI_IFCONFIG0_READOC_SHIFT)
#  define QSPI_IFCONFIG0_READOC_READ2IO  (2 << QSPI_IFCONFIG0_READOC_SHIFT)
#  define QSPI_IFCONFIG0_READOC_READ4O   (3 << QSPI_IFCONFIG0_READOC_SHIFT)
#  define QSPI_IFCONFIG0_READOC_READ4IO  (4 << QSPI_IFCONFIG0_READOC_SHIFT)
#define QSPI_IFCONFIG0_WRITEOC_SHIFT     (3)
#define QSPI_IFCONFIG0_WRITEOC_MASK      (0x7 << QSPI_IFCONFIG0_WRITEOC_SHIFT)
#  define QSPI_IFCONFIG0_WRITEOC_PP      (0 << QSPI_IFCONFIG0_WRITEOC_SHIFT)
#  define QSPI_IFCONFIG0_WRITEOC_PP2O    (1 << QSPI_IFCONFIG0_WRITEOC_SHIFT)
#  define QSPI_IFCONFIG0_WRITEOC_PP4O    (2 << QSPI_IFCONFIG0_WRITEOC_SHIFT)
#  define QSPI_IFCONFIG0_WRITEOC_PP4IO   (3 << QSPI_IFCONFIG0_WRITEOC_SHIFT)
#define QSPI_IFCONFIG0_ADDRMODE_24       (0 << 6)
#define QSPI_IFCONFIG0_ADDRMODE_32       (1 << 6)
#define QSPI_IFCONFIG0_DPMENABLE         (1 << 7)
#define QSPI_IFCONFIG0_PPSIZE_512        (1 << 12)

/* IFCONFIG1 */

#define QSPI_IFCONFIG1_SCKDELAY_SHIFT    (0)
#define QSPI_IFCONFIG1_SCKDELAY_MASK     (0xff << QSPI_IFCONFIG1_SCKDELAY_SHIFT)
#define QSPI_IFCONFIG1_DPMEN_EXIT        (0 << 24)
#define QSPI_IFCONFIG1_DPMEN_ENTER       (1 << 24)
#define QSPI_IFCONFIG1_SPIMODE_0         (0 << 25)
#define QSPI_IFCONFIG1_SPIMODE_3         (1 << 25)
#define QSPI_IFCONFIG1_SCKFREQ_SHIFT     (28)
#define QSPI_IFCONFIG1_SCKFREQ_MASK      (0xf << QSPI_IFCONFIG1_SCKFREQ_SHIFT)

/* STATUS */

#define QSPI_STATUS_DPM                  (1 << 2)
#define QSPI_STATUS_READY                (1 << 3)
#define QSPI_STATUS_SREG_SHIFT           (24)
#define QSPI_STATUS_SREG_MASK            (0xff << QSPI_STATUS_SREG_SHIFT)

/* DPMDUR */

#define QSPI_DPMDUR_ENTER_SHIFT          (0)
#define QSPI_DPMDUR_ENTER_MASK           (0xffff << QSPI_DPMDUR_ENTER_SHIFT)
#define QSPI_DPMDUR_EXIT_SHIFT           (16)
#define QSPI_DPMDUR_EXIT_MASK            (0xffff << QSPI_DPMDUR_EXIT_SHIFT)

/* ADDRCONF */

#define QSPI_ADDRCONF_OPCODE_SHIFT       (0)
#define QSPI_ADDRCONF_OPCODE_MASK        (0xff << QSPI_ADDRCONF_OPCODE_SHIFT)
#define QSPI_ADDRCONF_BYTE0_SHIFT        (8)
#define QSPI_ADDRCONF_BYTE0_MASK         (0xff << QSPI_ADDRCONF_BYTE0_SHIFT)
#define QSPI_ADDRCONF_BYTE1_SHIFT        (16)
#define QSPI_ADDRCONF_BYTE1_MASK         (0xff << QSPI_ADDRCONF_BYTE1_SHIFT)
#define QSPI_ADDRCONF_MODE_SHIFT         (24)
#define QSPI_ADDRCONF_MODE_MASK          (0x3 << QSPI_ADDRCONF_MODE_SHIFT)
#  define QSPI_ADDRCONF_MODE_NOINSTR     (0 << QSPI_ADDRCONF_MODE_SHIFT)
#  define QSPI_ADDRCONF_MODE_OPCODE      (1 << QSPI_ADDRCONF_MODE_SHIFT)
#  define QSPI_ADDRCONF_MODE_OPBYTE0     (2 << QSPI_ADDRCONF_MODE_SHIFT)
#  define QSPI_ADDRCONF_MODE_ALL         (3 << QSPI_ADDRCONF_MODE_SHIFT)
#define QSPI_ADDRCONF_WIPWAIT            (1 << 26)
#define QSPI_ADDRCONF_WREN               (1 << 27)

/* CINSTRCONF */

#define QSPI_CINSTRCONF_OPCODE_SHIFT     (0)
#define QSPI_CINSTRCONF_OPCODE_MASK      (0xff << QSPI_CINSTRCONF_OPCODE_SHIFT)
#define QSPI_CINSTRCONF_LENGTH_SHIFT     (8)
#define QSPI_CINSTRCONF_LENGTH_MASK      (0xf << QSPI_CINSTRCONF_LENGTH_SHIFT)
#define QSPI_CINSTRCONF_LENGTH(n)        ((n) << QSPI_CINSTRCONF_LENGTH_SHIFT)
#define QSPI_CINSTRCONF_LIO2             (1 << 12)
#define QSPI_CINSTRCONF_LIO3             (1 << 13)
#define QSPI_CINSTRCONF_WIPWAIT          (1 << 14)
#define QSPI_CINSTRCONF_WREN             (1 << 15)
#define QSPI_CINSTRCONF_LFEN             (1 << 16)
#define QSPI_CINSTRCONF_LFSTOP_STOP      (1 << 17)

/* CINSTRDAT0 */

#define QSPI_CINSTRDAT0_BYTE0_SHIFT      (0)
#define QSPI_CINSTRDAT0_BYTE0_MASK       (0xff << QSPI_CINSTRDAT0_BYTE0_SHIFT)
#define QSPI_CINSTRDAT0_BYTE0(n)         ((n) << QSPI_CINSTRDAT0_BYTE0_SHIFT)
#define QSPI_CINSTRDAT0_BYTE1_SHIFT      (8)
#define QSPI_CINSTRDAT0_BYTE1_MASK       (0xff << QSPI_CINSTRDAT0_BYTE1_SHIFT)
#define QSPI_CINSTRDAT0_BYTE1(n)         ((n) << QSPI_CINSTRDAT0_BYTE1_SHIFT)
#define QSPI_CINSTRDAT0_BYTE2_SHIFT      (16)
#define QSPI_CINSTRDAT0_BYTE2_MASK       (0xff << QSPI_CINSTRDAT0_BYTE2_SHIFT)
#define QSPI_CINSTRDAT0_BYTE2(n)         ((n) << QSPI_CINSTRDAT0_BYTE2_SHIFT)
#define QSPI_CINSTRDAT0_BYTE3_SHIFT      (24)
#define QSPI_CINSTRDAT0_BYTE3_MASK       (0xff << QSPI_CINSTRDAT0_BYTE3_SHIFT)
#define QSPI_CINSTRDAT0_BYTE3(n)         ((n) << QSPI_CINSTRDAT0_BYTE3_SHIFT)

/* CINSTRDAT1 */

#define QSPI_CINSTRDAT1_BYTE0_SHIFT      (0)
#define QSPI_CINSTRDAT1_BYTE0_MASK       (0xff << QSPI_CINSTRDAT1_BYTE0_SHIFT)
#define QSPI_CINSTRDAT1_BYTE0(n)         ((n) << QSPI_CINSTRDAT1_BYTE0_SHIFT)
#define QSPI_CINSTRDAT1_BYTE1_SHIFT      (8)
#define QSPI_CINSTRDAT1_BYTE1_MASK       (0xff << QSPI_CINSTRDAT1_BYTE1_SHIFT)
#define QSPI_CINSTRDAT1_BYTE1(n)         ((n) << QSPI_CINSTRDAT1_BYTE1_SHIFT)
#define QSPI_CINSTRDAT1_BYTE2_SHIFT      (16)
#define QSPI_CINSTRDAT1_BYTE2_MASK       (0xff << QSPI_CINSTRDAT1_BYTE2_SHIFT)
#define QSPI_CINSTRDAT1_BYTE2(n)         ((n) << QSPI_CINSTRDAT1_BYTE2_SHIFT)
#define QSPI_CINSTRDAT1_BYTE3_SHIFT      (24)
#define QSPI_CINSTRDAT1_BYTE3_MASK       (0xff << QSPI_CINSTRDAT1_BYTE3_SHIFT)
#define QSPI_CINSTRDAT1_BYTE3(n)         ((n) << QSPI_CINSTRDAT1_BYTE3_SHIFT)

/* IFTIMING */

#define QSPI_IFTIMING_RXDELAY_SHIFT      (8)
#define QSPI_IFTIMING_RXDELAY_MASK       (0xff << QSPI_IFTIMING_RXDELAY_SHIFT)
#define QSPI_IFTIMING_RXDELAY(n)         ((n) << QSPI_IFTIMING_RXDELAY_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_QSPI_H */

/****************************************************************************
 * boards/risc-v/ab32vg1/ab32vg1-generic/tools/target/target.h
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

/* Definitions shared by the flash programming stub and the stage-1 boot
 * loader.  These programs run bare metal on the AB32VG1 (not under NuttX),
 * so they define the few registers they need here.
 */

#ifndef __BOARDS_RISCV_AB32VG1_AB32VG1_GENERIC_TOOLS_TARGET_TARGET_H
#define __BOARDS_RISCV_AB32VG1_AB32VG1_GENERIC_TOOLS_TARGET_TARGET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SFR(a)          (*(volatile uint32_t *)(uintptr_t)(a))

#define FUNCMCON0       SFR(0x01c)
#define UART0CON        SFR(0x040)
#define UART0CPND       SFR(0x044)
#define UART0BAUD       SFR(0x048)
#define UART0DATA       SFR(0x04c)
#define TMR0CON         SFR(0x050)
#define TMR0CNT         SFR(0x058)
#define TMR0PR          SFR(0x05c)
#define WDTCON          SFR(0x068)
#define CLKCON1         SFR(0x074)
#define SPI0CON         SFR(0x0a8)
#define SPI0BUF         SFR(0x0ac)
#define SPI0BAUD        SFR(0x0b0)
#define SPI0CPND        SFR(0x0b4)
#define CRCCON          SFR(0x34c)
#define CLKGAT0         SFR(0x3b0)
#define CLKGAT1         SFR(0x3fc)
#define PICCONCLR       SFR(0x530)
#define PICCON          SFR(0x540)
#define PICEN           SFR(0x544)
#define GPIOAFEN        SFR(0x614)
#define GPIOADE         SFR(0x610)
#define GPIOADIR        SFR(0x60c)
#define GPIOAPU         SFR(0x61c)
#define GPIOAPD         SFR(0x620)
#define GPIOGSET        SFR(0x700)
#define GPIOGCLR        SFR(0x704)
#define GPIOGDIR        SFR(0x70c)
#define GPIOGFEN        SFR(0x714)
#define GPIOGPU         SFR(0x71c)

#define BIT(n)          (1u << (n))
#define UNUSED(x)       ((void)(x))

#define WDTCON_DISABLE  0x00000aa0
#define WDTCON_RESET    0x0a000110      /* Reset after 1 ms */
#define WDTCON_EN4S     0x0a500110      /* Enable, reset after 4096 ms */
#define WDTCON_KICK     0x0000000a

/* Image table written after the boot sector of a NuttX flash image */

#define IMG_MAGIC       0x42415858      /* "XXAB" */
#define IMG_TABLE       0x00001000      /* Flash offset of the table */
#define IMG_MAXSEG      4

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct img_seg_s
{
  uint32_t flash;                       /* Offset of the data in flash */
  uint32_t addr;                        /* Load address */
  uint32_t size;                        /* Size in bytes */
  uint32_t crc;                         /* CRC-32 of the data */
};

struct img_table_s
{
  uint32_t magic;                       /* IMG_MAGIC */
  uint32_t entry;                       /* Entry point */
  uint32_t nseg;                        /* Number of segments */
  struct img_seg_s seg[IMG_MAXSEG];
  uint32_t crc;                         /* CRC-32 of the fields above */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* spiflash.c */

void sf_init(uint32_t baud);
uint32_t sf_readid(void);
uint8_t sf_status(void);
void sf_read(uint32_t addr, uint8_t *buf, uint32_t len);
void sf_erase(uint32_t addr, bool block64k);
void sf_program(uint32_t addr, const uint8_t *buf, uint32_t len);

/* util.c */

void uart_init(uint32_t baud);
void uart_putc(int ch);
void uart_puts(const char *s);
int  uart_getc(void);
int  uart_trygetc(void);
uint32_t crc32_update(uint32_t crc, const uint8_t *buf, uint32_t len);
uint16_t crc16_update(uint16_t crc, const uint8_t *buf, uint32_t len);

#endif /* __BOARDS_RISCV_AB32VG1_AB32VG1_GENERIC_TOOLS_TARGET_TARGET_H */

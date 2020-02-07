/*******************************************************************************************
 * drivers/lcd/ft80x.h
 * Definitions for the FTDI FT80x GUI
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *  - Document No.: FT_000792, "FT800 Embedded Video Engine", Datasheet Version 1.1,
 *    Clearance No.: FTDI# 334, Future Technology Devices International Ltd.
 *  - Document No.: FT_000986, "FT801 Embedded Video Engine Datasheet", Version 1.0,
 *    Clearance No.: FTDI#376, Future Technology Devices International Ltd.
 *  - Some definitions derive from FTDI sample code.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************************/

#ifndef __DRIVERS_LCD_FT80X_H
#define __DRIVERS_LCD_FT80X_H

/*******************************************************************************************
 * Included Files
 *******************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>

/*******************************************************************************************
 * Public Types
 *******************************************************************************************/

/* Host write command
 *
 * For a SPI write command write transaction, the host writes a zero bit followed by a one
 * bit, followed by the 5-bit command, followed by two bytes of zero. All data is streamed
 * with a single chip select.
 *
 * I2C data format is equivalent (with obvious differences in bus protocol)
 */

struct ft80x_hostwrite_s
{
  uint8_t cmd;   /* Bits 6-7: 01, Bits 0-5: command */
  uint8_t pad1;  /* Zero */
  uint8_t pad2;  /* Zero */
};

/* For SPI memory read transaction, the host sends two zero bits, followed by the 22-bit
 * address. This is followed by a dummy byte. After the dummy byte, the FT80x responds to
 * each host byte with read data bytes.
 *
 * For I2C memory read transaction, bytes are packed in the I2C protocol as follow:
 *
 *   [start] <DEVICE ADDRESS + write bit>
 *   <00b+Address[21:16]>
 *   <Address[15:8]>
 *   <Address[7:0]>
 *   [restart] <DEVICE ADDRESS + read bit>
 *   <Read data byte 0>
 *   ....
 *   <Read data byte n> [stop]
 */

struct ft80x_spiread_s
{
  uint8_t addrh;   /* Bits 6-7: 00, Bits 0-5: Address[21:16] */
  uint8_t addrm;   /* Address[15:8] */
  uint8_t addrl;   /* Address[7:0] */
  uint8_t dummy;   /* Dummy byte */
};

struct ft80x_i2cread_s
{
  uint8_t addrh;   /* Bits 6-7: 00, Bits 0-5: Address[21:16] */
  uint8_t addrm;   /* Address[15:8] */
  uint8_t addrl;   /* Address[7:0] */
};

/* For SPI memory write transaction, the host sends a '1' bit and '0' bit, followed by the
 * 22-bit address. This is followed by the write data.
 *
 * For I2C memory write transaction, bytes are packed in the I2C protocol as follow:
 *
 *  [start] <DEVICE ADDRESS + write bit>
 *  <10b,Address[21:16]>
 *  <Address[15:8]>
 *  <Address[7:0]>
 *  <Write data byte 0>
 *  ....
 *  <Write data byte n> [stop]
 */

struct ft80x_spiwrite_s
{
  uint8_t addrh;   /* Bits 6-7: 10, Bits 0-5: Address[21:16] */
  uint8_t addrm;   /* Address[15:8] */
  uint8_t addrl;   /* Address[7:0] */
                   /* Write data follows */
};

struct ft80x_spiwrite8_s
{
  uint8_t addrh;   /* Bits 6-7: 10, Bits 0-5: Address[21:16] */
  uint8_t addrm;   /* Address[15:8] */
  uint8_t addrl;   /* Address[7:0] */
  uint8_t data;    /* 8-bit data follows */
};
struct ft80x_spiwrite16_s
{
  uint8_t addrh;   /* Bits 6-7: 10, Bits 0-5: Address[21:16] */
  uint8_t addrm;   /* Address[15:8] */
  uint8_t addrl;   /* Address[7:0] */
  uint8_t data[2]; /* 16-bit data follows */
};
struct ft80x_spiwrite32_s
{
  uint8_t addrh;   /* Bits 6-7: 10, Bits 0-5: Address[21:16] */
  uint8_t addrm;   /* Address[15:8] */
  uint8_t addrl;   /* Address[7:0] */
  uint8_t data[4]; /* 32-bit data follows */
};
struct ft80x_i2cwrite_s
{
  uint8_t addrh;   /* Bits 6-7: 10, Bits 0-5: Address[21:16] */
  uint8_t addrm;   /* Address[15:8] */
  uint8_t addrl;   /* Address[7:0] */
                   /* Write data follows */
};

/* This structure describes one signal notification */

struct ft80x_eventinfo_s
{
  struct sigevent event;                  /* Describe the way a task is to be notified */
  struct sigwork_s work;                  /* Work for SIGEV_THREAD */
  bool enable;                            /* True: enable notification; false: disable */
  int16_t pid;                            /* Send the notification to this task */
};

/* This structure describes the overall state of the FT80x driver */

struct spi_dev_s;    /* Forward reference */
struct i2c_master_s; /* Forward reference */

struct ft80x_dev_s
{
  /* Cached interface instances */

#ifdef CONFIG_LCD_FT80X_SPI
  FAR struct spi_dev_s  *spi;             /* Cached SPI device reference */
#else
  FAR struct i2c_master_s *i2c;           /* Cached SPI device reference */
#endif
  FAR const struct ft80x_config_s *lower; /* Cached lower half instance */

  /* Internal driver logic */

  struct work_s intwork;                  /* Support back end interrupt processing */
  uint32_t frequency;                     /* Effective frequency */
  sem_t exclsem;                          /* Mutual exclusion semaphore */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  uint8_t crefs;                          /* Number of open references */
  bool unlinked;                          /* True if the driver has been unlinked */
#endif

  /* Event notification support */

  struct ft80x_eventinfo_s notify[FT80X_INT_NEVENTS];
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ft80x_host_command
 *
 * Description:
 *   Send a host command to the FT80x
 *
 *   FFor a SPI write command write transaction, the host writes a zero bit
 *   followed by a one bit, followed by the 5-bit command, followed by two
 *   bytes of zero. All data is streamed with a single chip select.
 *
 ****************************************************************************/

void ft80x_host_command(FAR struct ft80x_dev_s *priv, uint8_t cmd);

/****************************************************************************
 * Name: ft80x_read_memory
 *
 * Description:
 *   Read from FT80X memory
 *
 *   For SPI memory read transaction, the host sends two zero bits, followed
 *   by the 22-bit address. This is followed by a dummy byte. After the dummy
 *   byte, the FT80x responds to each host byte with read data bytes.
 *
 ****************************************************************************/

void ft80x_read_memory(FAR struct ft80x_dev_s *priv, uint32_t addr,
                       FAR void *buffer, size_t buflen);

/****************************************************************************
 * Name: ft80x_read_byte, ft80x_read_hword, ft80x_read_word
 *
 * Description:
 *   Read an 8-, 16-, or 32-bt bit value from FT80X memory
 *
 *   For SPI memory read transaction, the host sends two zero bits, followed
 *   by the 22-bit address. This is followed by a dummy byte. After the dummy
 *   byte, the FT80x responds to each host byte with read data bytes.
 *
 ****************************************************************************/

uint8_t  ft80x_read_byte(FAR struct ft80x_dev_s *priv, uint32_t addr);
uint16_t ft80x_read_hword(FAR struct ft80x_dev_s *priv, uint32_t addr);
uint32_t ft80x_read_word(FAR struct ft80x_dev_s *priv, uint32_t addr);

/****************************************************************************
 * Name: ft80x_write_memory
 *
 * Description:
 *   Write to FT80X memory
 *
 *   For SPI memory write transaction, the host sends a '1' bit and '0' bit,
 *   followed by the 22-bit address. This is followed by the write data.
 *
 ****************************************************************************/

void ft80x_write_memory(FAR struct ft80x_dev_s *priv, uint32_t addr,
                        FAR const void *buffer, size_t buflen);

/****************************************************************************
 * Name: ft80x_write_byte, ft80x_write_hword, ft80x_write_word
 *
 * Description:
 *   Write an 8-, 16-, or 32-bt bit value to FT80X memory
 *
 *   For SPI memory write transaction, the host sends a '1' bit and '0' bit,
 *   followed by the 22-bit address. This is followed by the write data.
 *
 ****************************************************************************/

void ft80x_write_byte(FAR struct ft80x_dev_s *priv, uint32_t addr,
                      uint8_t data);
void ft80x_write_hword(FAR struct ft80x_dev_s *priv, uint32_t addr,
                       uint16_t data);
void ft80x_write_word(FAR struct ft80x_dev_s *priv, uint32_t addr,
                      uint32_t data);

#endif /* __DRIVERS_LCD_FT80X_H */

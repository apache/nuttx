/****************************************************************************
 * include/nuttx/rpmsg/rpmsg_port.h
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

#ifndef __INCLUDE_NUTTX_RPMSG_RPMSG_PORT_H
#define __INCLUDE_NUTTX_RPMSG_RPMSG_PORT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/slave.h>

#ifdef CONFIG_RPMSG_PORT

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct rpmsg_port_config_s
{
  FAR const char *remotecpu;
  uint16_t       txnum;         /* Number of tx buffer. */
  uint16_t       rxnum;         /* Number of rx buffer. */
  uint16_t       txlen;         /* Length of a single tx buffer. */
  uint16_t       rxlen;         /* Length of a single rx buffer. */

  /* Pointer to whole tx/rx buffer, if it was null, transport layer will
   * alloc internal.
   */

  FAR void       *txbuf;
  FAR void       *rxbuf;
};

#if defined(CONFIG_RPMSG_PORT_SPI) || defined(CONFIG_RPMSG_PORT_SPI_SLAVE)

/* There are two gpios used for communication between two chips. At the SPI
 * master side, mreq is an output gpio pin which is used to notify the
 * slave side there is a data packet to be sent. it actually transfers the
 * data only when it receives an interrupt from sreq pin. and at the SPI
 * slave side, it prepares the data to be sent, and activates the sreq to
 * the master side, master will initiate a transfer immediately when it
 * receives an interrupt from sreq pin to receive the data.
 *
 * If IOEXPANDER_OPTION_INVERT option of pin is set to be 0, then it will
 * be triggered an interrupt at the rising edge. or it will be triggered
 * at the falling edge.
 */

struct rpmsg_port_spi_config_s
{
  /* GPIO configurations of pins used for communication between two chips. */

  uint8_t         mreq_pin;
  uint8_t         sreq_pin;
  int             mreq_invert;
  int             sreq_invert; /* Pin options described in ioexpander.h */

  int             mode;        /* Mode of enum spi_mode_e */
  int             nbits;
  uint32_t        devid;       /* Device ID of enum spi_devtype_e */
  uint32_t        freq;        /* SPI frequency (Hz) */
};

#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_RPMSG_PORT_SPI

/****************************************************************************
 * Name: rpmsg_port_spi_initialize
 *
 * Description:
 *   Initialize a rpmsg_port_spi device to communicate between two chips.
 *
 * Input Parameters:
 *   cfg     - Configuration of buffers needed for communication.
 *   spicfg  - SPI device's configuration.
 *   spi     - SPI device used for transfer data between two chips.
 *   ioe     - ioexpander used to config gpios.
 *
 * Returned Value:
 *   Zero on success or an negative value on failure.
 *
 ****************************************************************************/

int
rpmsg_port_spi_initialize(FAR const struct rpmsg_port_config_s *cfg,
                          FAR const struct rpmsg_port_spi_config_s *spicfg,
                          FAR struct spi_dev_s *spi,
                          FAR struct ioexpander_dev_s *ioe);
#endif

#ifdef CONFIG_RPMSG_PORT_SPI_SLAVE

/****************************************************************************
 * Name: rpmsg_port_spi_slave_initialize
 *
 * Description:
 *   Initialize a rpmsg_port_spi_slave device to communicate between two
 *   chips.
 *
 * Input Parameters:
 *   cfg      - Configuration of buffers needed for communication.
 *   spicfg   - SPI device's configuration.
 *   spictrlr - SPI slave controller used for transfer data between two
 *              chips.
 *   ioe      - ioexpander used to config gpios.
 *
 * Returned Value:
 *   Zero on success or an negative value on failure.
 *
 ****************************************************************************/

int
rpmsg_port_spi_slave_initialize(FAR const struct rpmsg_port_config_s *cfg,
  FAR const struct rpmsg_port_spi_config_s *spicfg,
  FAR struct spi_slave_ctrlr_s *spictrlr, FAR struct ioexpander_dev_s *ioe);

#endif

#ifdef CONFIG_RPMSG_PORT_UART

/****************************************************************************
 * Name: rpmsg_port_uart_initialize
 *
 * Description:
 *   Initialze a rpmsg_port_uart device to communicate between two chips.
 *
 * Input Parameters:
 *   cfg      - Configuration of buffers needed for communication.
 *   uartpath - Uart device path.
 *   localcpu - Local cpu name
 *
 ****************************************************************************/

int rpmsg_port_uart_initialize(FAR const struct rpmsg_port_config_s *cfg,
                               FAR const char *uartpath,
                               FAR const char *localcpu);
#endif

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_RPMSG_PORT */
#endif /* __INCLUDE_NUTTX_RPMSG_RPMSG_PORT_H */

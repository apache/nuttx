/****************************************************************************
 * include/nuttx/can/mcp2515.h
 *
 *   Copyright (C) 2017 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_CAN_MCP2515_H
#define __INCLUDE_NUTTX_CAN_MCP2515_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/can/can.h>

#if defined(CONFIG_CAN) && defined(CONFIG_CAN_MCP2515)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI BUS PARAMETERS *******************************************************/

#define MCP2515_SPI_FREQUENCY    (1000000)        /* 1 MHz */
#define MCP2515_SPI_MODE         (SPIDEV_MODE0)   /* Device uses SPI Mode 0: CPOL=0, CPHA=0 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Type of the MCP2515 interrupt handling callback */

struct mcp2515_config_s; /* Forward reference */
typedef void (*mcp2515_handler_t)(FAR struct mcp2515_config_s *config,
                                  FAR void *arg);

/* A reference to a structure of this type must be passed to the MCP2515
 * driver when the driver is instantiated. This structure provides
 * information about the configuration of the MCP2515 and provides some
 * board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by
 * the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify the frequency.
 */

struct mcp2515_config_s
{
  struct spi_dev_s *spi;    /* SPI used for MCP2515 communication */
  uint32_t baud;            /* Configured baud */
  uint32_t btp;             /* Bit timing/prescaler register setting */
  uint8_t devid;            /* MCP2515 device ID */
  uint8_t mode;             /* See enum mcp2515_canmod_e */
  uint8_t nfilters;         /* Number of standard/extended filters */
  uint8_t ntxbuffers;       /* Number of TX Buffer available */
  uint8_t txbuf0[10];       /* Transmit Buffer 0 */
  uint8_t txbuf1[10];       /* Transmit Buffer 1 */
  uint8_t txbuf2[10];       /* Transmit Buffer 2 */
#ifdef MCP2515_LOOPBACK
  bool loopback;            /* True: Loopback mode */
#endif

  /* Device characterization */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the ADXL345 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach the ADXL345 interrupt handler to the GPIO interrupt
   */

  CODE int (*attach)(FAR struct mcp2515_config_s *state,
                     mcp2515_handler_t handler, FAR void *arg);
};

/* Internal representation of the MCP2515 state data */

struct mcp2515_can_s;

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

/****************************************************************************
 * Name: mcp2515_instantiate
 *
 * Description:
 *   Initialize a CAN Driver for MCP2515.
 *
 * Input Parameters:
 *   spi     - An instance of the SPI interface to use to communicate with
 *             MCP2515
 *   config  - Describes the configuration of the MCP2515 part.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

FAR struct mcp2515_can_s *mcp2515_instantiate(FAR struct mcp2515_config_s *config);

/****************************************************************************
 * Name: mcp2515_initialize
 *
 * Description:
 *   Initialize a CAN Driver for MCP2515.
 *
 * Input Parameters:
 *   spi     - An instance of the SPI interface to use to communicate with
 *             MCP2515
 *   config  - Describes the configuration of the MCP2515 part.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

FAR struct can_dev_s *mcp2515_initialize(FAR struct mcp2515_can_s *mcp2515can);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_CAN && CONFIG_CAN_MCP2515 */

#endif /* __INCLUDE_NUTTX_CAN_MCP2515_H */

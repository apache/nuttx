/****************************************************************************
 * include/nuttx/ioexpander/icjx.h
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

#ifndef __INCLUDE_NUTTX_IOEXPANDER_ICJX_H
#define __INCLUDE_NUTTX_IOEXPANDER_ICJX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ICJX_CTRL_WORD_CURR_SRC_DIS   0   /* Pin current source disabled */
#define ICJX_CTRL_WORD_PULLDOWN_200U  1   /* 200 uA pull down */
#define ICJX_CTRL_WORD_PULLDOWN_600U  2   /* 600 uA pull down */
#define ICJX_CTRL_WORD_PULLDOWN_2M    3   /* 2 mA pull down */
#define ICJX_CTRL_WORD_PULLUP_200U    5   /* 200 uA pull up */
#define ICJX_CTRL_WORD_PULLUP_600U    6   /* 600 uA pull up */
#define ICJX_CTRL_WORD_PULLUP_2M      7   /* 2 mA pull up*/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the iC-JX
 * driver when the driver is instantiated. This structure provides
 * information about the configuration of the iC-JX and provides some
 * board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by
 * the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify the frequency.
 */

struct icjx_config_s
{
  /* Device characterization */

  uint8_t id;           /* Device ID (if more expanders are used) */
  uint8_t verification; /* True if data verification on MISO line is used */
  uint8_t current_src;  /* Current sources for pin nibbles (pull up,
                         * pull down) - see Control Word 2 register
                         */
  uint8_t addr;         /* Device address (set by A(1:0) pins) */
  uint8_t mode;         /* SPI mode */
  uint32_t frequency;   /* SPI frequency */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the iC-JX driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach the iC-JX interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   */

  CODE int  (*attach)(FAR struct icjx_config_s *config, xcpt_t handler,
                      FAR void *arg);
  CODE void (*enable)(FAR struct icjx_config_s *config, bool enable);
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: icjx_initialize
 *
 * Description:
 *   Instantiate and configure the iC-JX device driver to use the
 *   provided SPI device instance.
 *
 * Input Parameters:
 *   spi     - A SPI driver instance
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   an ioexpander_dev_s instance on success, NULL on failure.
 *
 ****************************************************************************/

struct spi_dev_s;
FAR struct ioexpander_dev_s *icjx_initialize(FAR struct spi_dev_s *spi,
                                      FAR struct icjx_config_s *config);

#endif /* __INCLUDE_NUTTX_IOEXPANDER_ICJX_H */

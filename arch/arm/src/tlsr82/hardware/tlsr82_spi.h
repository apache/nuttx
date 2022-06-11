/****************************************************************************
 * arch/arm/src/tlsr82/hardware/tlsr82_spi.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_SPI_H
#define __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/tlsr82/chip.h>

#include "hardware/tlsr82_register.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI and I2C Group select register */

#define SPI_I2C_GROUP_REG       REG_ADDR8(0x5b6)

#define SPI_I2C_GROUPA_EN       BIT_RNG(4,5)
#define SPI_I2C_GROUPB_EN       BIT(6)
#define SPI_I2C_GROUPD_EN       BIT(7)

/* SPI and I2C Pin select register */

#define SPI_I2C_PIN_REG         REG_ADDR8(0x5b7)

#define SPI_PA3_SPI_EN          BIT(0)
#define SPI_PA4_SPI_EN          BIT(1)
#define SPI_PB6_SPI_EN          BIT(2)
#define SPI_PD7_SPI_EN          BIT(3)
#define I2C_PA3_I2C_EN          BIT(4)
#define I2C_PA4_I2C_EN          BIT(5)
#define I2C_PB6_I2C_EN          BIT(6)
#define I2C_PD7_I2C_EN          BIT(7)

/* SPI Data register */

#define SPI_DATA_REG            REG_ADDR8(0x08)

/* SPI Control register */

#define SPI_CTRL_REG            REG_ADDR8(0x09)

#define SPI_CTRL_CS             BIT(0)
#define SPI_CTRL_MASTER_EN      BIT(1)
#define SPI_CTRL_OUT_OFF        BIT(2)
#define SPI_CTRL_RW_STA         BIT(3)
#define SPI_CTRL_ADDR_ADD       BIT(4)
#define SPI_CTRL_SHARE_MODE     BIT(5)
#define SPI_CTRL_BUSY           BIT(6)

#define SPI_CS_HIGH             BM_SET(SPI_CTRL_REG, SPI_CTRL_CS)
#define SPI_CS_LOW              BM_CLR(SPI_CTRL_REG, SPI_CTRL_CS)

/* SPI Clock register */

#define SPI_CLK_REG             REG_ADDR8(0x0a)

#define SPI_CLK_DIV             BIT_RNG(0,6)
#define SPI_CLK_EN              BIT(7)

/* SPI Mode select register, CPOL, CPHA */

#define SPI_MODE_REG            REG_ADDR8(0x0b)

#define SPI_MODE_CPOL           BIT(0)
#define SPI_MODE_CPHA           BIT(1)

#endif /* __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_SPI_H */

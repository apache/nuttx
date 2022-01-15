/****************************************************************************
 * arch/arm/src/imx1/imx_i2c.h
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

#ifndef __ARCH_ARM_SRC_IMX1_IMX_I2C_H
#define __ARCH_ARM_SRC_IMX1_IMX_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C Register Offsets *****************************************************/

#define I2C_IADR_OFFSET             0x0000
#define I2C_IFDR_OFFSET             0x0004
#define I2C_I2CR_OFFSET             0x0008
#define I2C_I2SR_OFFSET             0x000c
#define I2C_I2DR_OFFSET             0x0010

/* I2C Register Addresses ***************************************************/

#define IMX_I2C_IADR                (IMX_I2C_VBASE + I2C_IADR_OFFSET)
#define IMX_I2C_IFDR                (IMX_I2C_VBASE + I2C_IFDR_OFFSET)
#define IMX_I2C_I2CR                (IMX_I2C_VBASE + I2C_I2CR_OFFSET)
#define IMX_I2C_I2SR                (IMX_I2C_VBASE + I2C_I2SR_OFFSET)
#define IMX_I2C_I2DR                (IMX_I2C_VBASE + I2C_I2DR_OFFSET)

/* I2C Register Bit Definitions *********************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_IMX_I2C_H */

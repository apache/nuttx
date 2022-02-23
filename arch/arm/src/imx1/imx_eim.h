/****************************************************************************
 * arch/arm/src/imx1/imx_eim.h
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

#ifndef __ARCH_ARM_SRC_IMX1_IMX_IEM_H
#define __ARCH_ARM_SRC_IMX1_IMX_IEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EIM Register Offsets *****************************************************/

#define EIM_CS0H_OFFSET             0x00
#define EIM_CS0L_OFFSET             0x04
#define EIM_CS1H_OFFSET             0x08
#define EIM_CS1L_OFFSET             0x0c
#define EIM_CS2H_OFFSET             0x10
#define EIM_CS2L_OFFSET             0x14
#define EIM_CS3H_OFFSET             0x18
#define EIM_CS3L_OFFSET             0x1c
#define EIM_CS4H_OFFSET             0x20
#define EIM_CS4L_OFFSET             0x24
#define EIM_CS5H_OFFSET             0x28
#define EIM_CS5L_OFFSET             0x2c
#define EIM_WEIM_OFFSET             0x30

/* EIM Register Addresses ***************************************************/

#define IMX_EIM_CS0H                (EIM_BASE_ADDR + EIM_CS0H_OFFSET)
#define IMX_EIM_CS0L                (EIM_BASE_ADDR + EIM_CS0L_OFFSET)
#define IMX_EIM_CS1H                (EIM_BASE_ADDR + EIM_CS1H_OFFSET)
#define IMX_EIM_CS1L                (EIM_BASE_ADDR + EIM_CS1L_OFFSET)
#define IMX_EIM_CS2H                (EIM_BASE_ADDR + EIM_CS2H_OFFSET)
#define IMX_EIM_CS2L                (EIM_BASE_ADDR + EIM_CS2L_OFFSET)
#define IMX_EIM_CS3H                (EIM_BASE_ADDR + EIM_CS3H_OFFSET)
#define IMX_EIM_CS3L                (EIM_BASE_ADDR + EIM_CS3L_OFFSET)
#define IMX_EIM_CS4H                (EIM_BASE_ADDR + EIM_CS4H_OFFSET)
#define IMX_EIM_CS4L                (EIM_BASE_ADDR + EIM_CS4L_OFFSET)
#define IMX_EIM_CS5H                (EIM_BASE_ADDR + EIM_CS5H_OFFSET)
#define IMX_EIM_CS5L                (EIM_BASE_ADDR + EIM_CS5L_OFFSET)
#define IMX_EIM_WEIM                (EIM_BASE_ADDR + EIM_WEIM_OFFSET)

/* EIM Register Bit Definitions *********************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_IMX1_IMX_IEM_H */

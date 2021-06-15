/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_usbctrl_dpsram.h
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

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_USBCTRL_DPSRAM_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_USBCTRL_DPSRAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp2040_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP2040_USBCTRL_DPSRAM_SETUP_PACKET_OFFSET       0x000000
#define RP2040_USBCTRL_DPSRAM_EP_IN_CTRL_OFFSET(n)      (0x000008 + ((n) - 1) * 8)
#define RP2040_USBCTRL_DPSRAM_EP_OUT_CTRL_OFFSET(n)     (0x00000c + ((n) - 1) * 8)
#define RP2040_USBCTRL_DPSRAM_EP_CTRL_OFFSET(n)         (0x000008 + ((n) - 2) * 4)
#define RP2040_USBCTRL_DPSRAM_EP_IN_BUF_CTRL_OFFSET(n)  (0x000080 + (n) * 8)
#define RP2040_USBCTRL_DPSRAM_EP_OUT_BUF_CTRL_OFFSET(n) (0x000084 + (n) * 8)
#define RP2040_USBCTRL_DPSRAM_EP_BUF_CTRL_OFFSET(n)     (0x000080 + (n) * 4)
#define RP2040_USBCTRL_DPSRAM_EP0_BUF_0_OFFSET          0x000100
#define RP2040_USBCTRL_DPSRAM_EP0_BUF_1_OFFSET          0x000140
#define RP2040_USBCTRL_DPSRAM_DATA_BUF_OFFSET           0x000180

/* Register definitions *****************************************************/

#define RP2040_USBCTRL_DPSRAM_SETUP_PACKET          (RP2040_USBCTRL_DPSRAM_BASE + RP2040_USBCTRL_DPSRAM_SETUP_PACKET_OFFSET)
#define RP2040_USBCTRL_DPSRAM_EP_IN_CTRL(n)         (RP2040_USBCTRL_DPSRAM_BASE + RP2040_USBCTRL_DPSRAM_EP_IN_CTRL_OFFSET(n))
#define RP2040_USBCTRL_DPSRAM_EP_OUT_CTRL(n)        (RP2040_USBCTRL_DPSRAM_BASE + RP2040_USBCTRL_DPSRAM_EP_OUT_CTRL_OFFSET(n))
#define RP2040_USBCTRL_DPSRAM_EP_CTRL(n)            (RP2040_USBCTRL_DPSRAM_BASE + RP2040_USBCTRL_DPSRAM_EP_CTRL_OFFSET(n))
#define RP2040_USBCTRL_DPSRAM_EP_IN_BUF_CTRL(n)     (RP2040_USBCTRL_DPSRAM_BASE + RP2040_USBCTRL_DPSRAM_EP_IN_BUF_CTRL_OFFSET(n))
#define RP2040_USBCTRL_DPSRAM_EP_OUT_BUF_CTRL(n)    (RP2040_USBCTRL_DPSRAM_BASE + RP2040_USBCTRL_DPSRAM_EP_OUT_BUF_CTRL_OFFSET(n))
#define RP2040_USBCTRL_DPSRAM_EP_BUF_CTRL(n)        (RP2040_USBCTRL_DPSRAM_BASE + RP2040_USBCTRL_DPSRAM_EP_BUF_CTRL_OFFSET(n))
#define RP2040_USBCTRL_DPSRAM_EP0_BUF_0             (RP2040_USBCTRL_DPSRAM_BASE + RP2040_USBCTRL_DPSRAM_EP0_BUF_0_OFFSET)
#define RP2040_USBCTRL_DPSRAM_EP0_BUF_1             (RP2040_USBCTRL_DPSRAM_BASE + RP2040_USBCTRL_DPSRAM_EP0_BUF_1_OFFSET)
#define RP2040_USBCTRL_DPSRAM_DATA_BUF              (RP2040_USBCTRL_DPSRAM_BASE + RP2040_USBCTRL_DPSRAM_DATA_BUF_OFFSET)

/* Register bit definitions *************************************************/

#define RP2040_USBCTRL_DPSRAM_EP_CTRL_ENABLE            (1 << 31)
#define RP2040_USBCTRL_DPSRAM_EP_CTRL_DOUBLE_BUF        (1 << 30)
#define RP2040_USBCTRL_DPSRAM_EP_CTRL_INT_1BUF          (1 << 29)
#define RP2040_USBCTRL_DPSRAM_EP_CTRL_INT_2BUF          (1 << 28)
#define RP2040_USBCTRL_DPSRAM_EP_CTRL_EP_TYPE_SHIFT     (26)
#define RP2040_USBCTRL_DPSRAM_EP_CTRL_EP_TYPE_MASK      (0x3 << RP2040_USBCTRL_DPSRAM_EP_CTRL_EP_TYPE_SHIFT)
#define RP2040_USBCTRL_DPSRAM_EP_CTRL_EP_TYPE_CTRL      (0 << RP2040_USBCTRL_DPSRAM_EP_CTRL_EP_TYPE_SHIFT)
#define RP2040_USBCTRL_DPSRAM_EP_CTRL_EP_TYPE_ISO       (1 << RP2040_USBCTRL_DPSRAM_EP_CTRL_EP_TYPE_SHIFT)
#define RP2040_USBCTRL_DPSRAM_EP_CTRL_EP_TYPE_BULK      (2 << RP2040_USBCTRL_DPSRAM_EP_CTRL_EP_TYPE_SHIFT)
#define RP2040_USBCTRL_DPSRAM_EP_CTRL_EP_TYPE_INTR      (3 << RP2040_USBCTRL_DPSRAM_EP_CTRL_EP_TYPE_SHIFT)
#define RP2040_USBCTRL_DPSRAM_EP_CTRL_INT_STALL         (1 << 17)
#define RP2040_USBCTRL_DPSRAM_EP_CTRL_INT_NAK           (1 << 16)
#define RP2040_USBCTRL_DPSRAM_EP_CTRL_EP_ADDR_SHIFT     (6)
#define RP2040_USBCTRL_DPSRAM_EP_CTRL_EP_ADDR_MASK      (0xffc0)

#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_FULL1            (1 << 31)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_LAST1            (1 << 30)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA_PID1_SHIFT  (29)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA_PID1_MASK   (1 << RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA_PID1_SHIFT)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA0_PID1       (0 << RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA_PID1_SHIFT)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA1_PID1       (1 << RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA_PID1_SHIFT)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DBUF_OFF_128     (0 << 27)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DBUF_OFF_256     (1 << 27)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DBUF_OFF_512     (2 << 27)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DBUF_OFF_1024    (3 << 27)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_AVAIL1           (1 << 26)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_LEN1_SHIFT       (16)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_LEN1_MASK        (0x3ff << RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_LEN1_SHIFT)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_FULL             (1 << 15)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_LAST             (1 << 14)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA_PID_SHIFT   (13)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA_PID_MASK    (1 << RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA_PID_SHIFT)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA0_PID        (0 << RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA_PID_SHIFT)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA1_PID        (1 << RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA_PID_SHIFT)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_SEL              (1 << 12)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_STALL            (1 << 11)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_AVAIL            (1 << 10)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_LEN_SHIFT        (0)
#define RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_LEN_MASK         (0x3ff << RP2040_USBCTRL_DPSRAM_EP_BUFF_CTRL_LEN_SHIFT)

#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_USBCTRL_DPSRAM_H */

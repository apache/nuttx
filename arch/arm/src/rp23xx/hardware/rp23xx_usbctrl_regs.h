/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_usbctrl_regs.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_USBCTRL_REGS_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_USBCTRL_REGS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_USBCTRL_REGS_ADDR_ENDP_OFFSET               0x000000  /* Device address and endpoint control */
#define RP23XX_USBCTRL_REGS_ADDR_ENDPN_OFFSET(n)           (0x000004 + ((n) - 1) * 4)
                                                                     /* Interrupt endpoint 1. Only valid for HOST mode. */
#define RP23XX_USBCTRL_REGS_MAIN_CTRL_OFFSET               0x000040  /* Main control register */
#define RP23XX_USBCTRL_REGS_SOF_WR_OFFSET                  0x000044  /* Set the SOF (Start of Frame) frame number in the host controller. The SOF packet is sent every 1ms and the host will increment the frame number by 1 each time. */
#define RP23XX_USBCTRL_REGS_SOF_RD_OFFSET                  0x000048  /* Read the last SOF (Start of Frame) frame number seen. In device mode the last SOF received from the host. In host mode the last SOF sent by the host. */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_OFFSET                0x00004c  /* SIE control register */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_OFFSET              0x000050  /* SIE status register */
#define RP23XX_USBCTRL_REGS_INT_EP_CTRL_OFFSET             0x000054  /* interrupt endpoint control register */
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_OFFSET             0x000058  /* Buffer status register. A bit set here indicates that a buffer has completed on the endpoint (if the buffer interrupt is enabled). It is possible for 2 buffers to be completed, so clearing the buffer status bit may instantly re set it on the next clock cycle. */
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_OFFSET  0x00005c  /* Which of the double buffers should be handled. Only valid if using an interrupt per buffer (i.e. not per 2 buffers). Not valid for host interrupt endpoint polling because they are only single buffered. */
#define RP23XX_USBCTRL_REGS_EP_ABORT_OFFSET                0x000060  /* Device only: Can be set to ignore the buffer control register for this endpoint in case you would like to revoke a buffer. A NAK will be sent for every access to the endpoint until this bit is cleared. A corresponding bit in `EP_ABORT_DONE` is set when it is safe to modify the buffer control register. */
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_OFFSET           0x000064  /* Device only: Used in conjunction with `EP_ABORT`. Set once an endpoint is idle so the programmer knows it is safe to modify the buffer control register. */
#define RP23XX_USBCTRL_REGS_EP_STALL_ARM_OFFSET            0x000068  /* Device: this bit must be set in conjunction with the `STALL` bit in the buffer control register to send a STALL on EP0. The device controller clears these bits when a SETUP packet is received because the USB spec requires that a STALL condition is cleared when a SETUP packet is received. */
#define RP23XX_USBCTRL_REGS_NAK_POLL_OFFSET                0x00006c  /* Used by the host controller. Sets the wait time in microseconds before trying again if the device replies with a NAK. */
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_OFFSET     0x000070  /* Device: bits are set when the `IRQ_ON_NAK` or `IRQ_ON_STALL` bits are set. For EP0 this comes from `SIE_CTRL`. For all other endpoints it comes from the endpoint control register. */
#define RP23XX_USBCTRL_REGS_USB_MUXING_OFFSET              0x000074  /* Where to connect the USB controller. Should be to_phy by default. */
#define RP23XX_USBCTRL_REGS_USB_PWR_OFFSET                 0x000078  /* Overrides for the power signals in the event that the VBUS signals are not hooked up to GPIO. Set the value of the override and then the override enable to switch over to the override value. */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OFFSET           0x00007c  /* This register allows for direct control of the USB phy. Use in conjunction with usbphy_direct_override register to enable each override bit. */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_OFFSET  0x000080  /* Override enable for each control in usbphy_direct */
#define RP23XX_USBCTRL_REGS_USBPHY_TRIM_OFFSET             0x000084  /* Used to adjust trim values of USB phy pull down resistors. */
#define RP23XX_USBCTRL_REGS_LINESTATE_TUNING_OFFSET        0x000088  /* Used for debug only */
#define RP23XX_USBCTRL_REGS_INTR_OFFSET                    0x00008c  /* Raw Interrupts */
#define RP23XX_USBCTRL_REGS_INTE_OFFSET                    0x000090  /* Interrupt Enable */
#define RP23XX_USBCTRL_REGS_INTF_OFFSET                    0x000094  /* Interrupt Force */
#define RP23XX_USBCTRL_REGS_INTS_OFFSET                    0x000098  /* Interrupt status after masking & forcing */
#define RP23XX_USBCTRL_REGS_SOF_TIMESTAMP_RAW_OFFSET       0x000100  /* Device only. Raw value of free-running PHY clock counter @48MHz. Used to calculate time between SOF events */
#define RP23XX_USBCTRL_REGS_SOF_TIMESTAMP_LAST_OFFSET      0x000104  /* Device only. Value of free-running PHY clock counter @48MHz when last SOF event occured */
#define RP23XX_USBCTRL_REGS_SM_STATE_OFFSET                0x000108
#define RP23XX_USBCTRL_REGS_EP_TX_ERROR_OFFSET             0x00010c  /* TX error count for each endpoint. Write to each field to reset the counter to 0 */
#define RP23XX_USBCTRL_REGS_EP_RX_ERROR_OFFSET             0x000110  /* RX error count for each endpoint. Write to each field to reset the counter to 0 */
#define RP23XX_USBCTRL_REGS_DEV_SM_WATCHDOG_OFFSET         0x000114  /* Watchdog that forces the device state machine to idle and raises an interrupt if the device stays in a state that isn’t idle for the configured limit. The counter is reset on every state transition. Set limit while enable is low and then set the enable */

/* Register definitions *****************************************************/

#define RP23XX_USBCTRL_REGS_ADDR_ENDP               (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_ADDR_ENDP_OFFSET)
#define RP23XX_USBCTRL_REGS_ADDR_ENDPN(n)           (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_ADDR_ENDPN_OFFSET(n))
#define RP23XX_USBCTRL_REGS_MAIN_CTRL               (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_MAIN_CTRL_OFFSET)
#define RP23XX_USBCTRL_REGS_SOF_WR                  (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_SOF_WR_OFFSET)
#define RP23XX_USBCTRL_REGS_SOF_RD                  (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_SOF_RD_OFFSET)
#define RP23XX_USBCTRL_REGS_SIE_CTRL                (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_SIE_CTRL_OFFSET)
#define RP23XX_USBCTRL_REGS_SIE_STATUS              (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_SIE_STATUS_OFFSET)
#define RP23XX_USBCTRL_REGS_INT_EP_CTRL             (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_INT_EP_CTRL_OFFSET)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS             (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_BUFF_STATUS_OFFSET)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE  (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_OFFSET)
#define RP23XX_USBCTRL_REGS_EP_ABORT                (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_EP_ABORT_OFFSET)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE           (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_EP_ABORT_DONE_OFFSET)
#define RP23XX_USBCTRL_REGS_EP_STALL_ARM            (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_EP_STALL_ARM_OFFSET)
#define RP23XX_USBCTRL_REGS_NAK_POLL                (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_NAK_POLL_OFFSET)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK     (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_OFFSET)
#define RP23XX_USBCTRL_REGS_USB_MUXING              (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_USB_MUXING_OFFSET)
#define RP23XX_USBCTRL_REGS_USB_PWR                 (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_USB_PWR_OFFSET)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT           (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OFFSET)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE  (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_OFFSET)
#define RP23XX_USBCTRL_REGS_USBPHY_TRIM             (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_USBPHY_TRIM_OFFSET)
#define RP23XX_USBCTRL_REGS_LINESTATE_TUNING        (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_LINESTATE_TUNING_OFFSET)
#define RP23XX_USBCTRL_REGS_INTR                    (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_INTR_OFFSET)
#define RP23XX_USBCTRL_REGS_INTE                    (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_INTE_OFFSET)
#define RP23XX_USBCTRL_REGS_INTF                    (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_INTF_OFFSET)
#define RP23XX_USBCTRL_REGS_INTS                    (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_INTS_OFFSET)

#define RP23XX_USBCTRL_REGS_SOF_TIMESTAMP_RAW       (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_SOF_TIMESTAMP_RAW_OFFSET)
#define RP23XX_USBCTRL_REGS_SOF_TIMESTAMP_LAST      (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_SOF_TIMESTAMP_LAST_OFFSET)
#define RP23XX_USBCTRL_REGS_SM_STATE                (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_SM_STATE_OFFSET)
#define RP23XX_USBCTRL_REGS_EP_TX_ERROR             (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_EP_TX_ERROR_OFFSET)
#define RP23XX_USBCTRL_REGS_EP_RX_ERROR             (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_EP_RX_ERROR_OFFSET)
#define RP23XX_USBCTRL_REGS_DEV_SM_WATCHDOG         (RP23XX_USBCTRL_REGS_BASE + RP23XX_USBCTRL_REGS_DEV_SM_WATCHDOG_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_USBCTRL_REGS_ADDR_ENDP_ENDPOINT_SHIFT           (16)       /* Device endpoint to send data to. Only valid for HOST mode. */
#define RP23XX_USBCTRL_REGS_ADDR_ENDP_ENDPOINT_MASK            (0x0f << RP23XX_USBCTRL_REGS_ADDR_ENDP_ENDPOINT_SHIFT)
#define RP23XX_USBCTRL_REGS_ADDR_ENDP_ADDRESS_MASK             (0x7f)     /* In device mode, the address that the device should respond to. Set in response to a SET_ADDR setup packet from the host. In host mode set to the address of the device to communicate with. */

#define RP23XX_USBCTRL_REGS_ADDR_ENDPN_INTEP_PREAMBLE          (1 << 26)  /* Interrupt EP requires preamble (is a low speed device on a full speed hub) */
#define RP23XX_USBCTRL_REGS_ADDR_ENDPN_INTEP_DIR               (1 << 25)  /* Direction of the interrupt endpoint. In=0, Out=1 */
#define RP23XX_USBCTRL_REGS_ADDR_ENDPN_ENDPOINT_SHIFT          (16)       /* Endpoint number of the interrupt endpoint */
#define RP23XX_USBCTRL_REGS_ADDR_ENDPN_ENDPOINT_MASK           (0x0f << RP23XX_USBCTRL_REGS_ADDR_ENDP1_ENDPOINT_SHIFT)
#define RP23XX_USBCTRL_REGS_ADDR_ENDPN_ADDRESS_MASK            (0x7f)     /* Device address */

#define RP23XX_USBCTRL_REGS_MAIN_CTRL_SIM_TIMING               (1 << 31)  /* Reduced timings for simulation */
#define RP23XX_USBCTRL_REGS_MAIN_CTRL_PHY_ISO                  (1 << 2)   /* Isolates USB phy after controller power-up Remove isolation once software has configured the controller Not isolated = 0, Isolated = 1 */
#define RP23XX_USBCTRL_REGS_MAIN_CTRL_HOST_NDEVICE             (1 << 1)   /* Device mode = 0, Host mode = 1 */
#define RP23XX_USBCTRL_REGS_MAIN_CTRL_CONTROLLER_EN            (1 << 0)   /* Enable controller */

#define RP23XX_USBCTRL_REGS_SOF_WR_COUNT_MASK                  (0x7ff)

#define RP23XX_USBCTRL_REGS_SOF_RD_COUNT_MASK                  (0x7ff)

#define RP23XX_USBCTRL_REGS_SIE_CTRL_EP0_INT_STALL             (1 << 31)  /* Device: Set bit in EP_STATUS_STALL_NAK when EP0 sends a STALL */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_EP0_DOUBLE_BUF            (1 << 30)  /* Device: EP0 single buffered = 0, double buffered = 1 */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_EP0_INT_1BUF              (1 << 29)  /* Device: Set bit in BUFF_STATUS for every buffer completed on EP0 */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_EP0_INT_2BUF              (1 << 28)  /* Device: Set bit in BUFF_STATUS for every 2 buffers completed on EP0 */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_EP0_INT_NAK               (1 << 27)  /* Device: Set bit in EP_STATUS_STALL_NAK when EP0 sends a NAK */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_DIRECT_EN                 (1 << 26)  /* Direct bus drive enable */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_DIRECT_DP                 (1 << 25)  /* Direct control of DP */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_DIRECT_DM                 (1 << 24)  /* Direct control of DM */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_EP0_STOP_ON_SHORT_PACKET  (1 << 19)  /* Device: Stop EP0 on a short packet */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_TRANSCEIVER_PD            (1 << 18)  /* Power down bus transceiver */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_RPU_OPT                   (1 << 17)  /* Device: Pull-up strength (0=1K2, 1=2k3) */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_PULLUP_EN                 (1 << 16)  /* Device: Enable pull up resistor */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_PULLDOWN_EN               (1 << 15)  /* Host: Enable pull down resistors */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_RESET_BUS                 (1 << 13)  /* Host: Reset bus */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_RESUME                    (1 << 12)  /* Device: Remote wakeup. Device can initiate its own resume after suspend. */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_VBUS_EN                   (1 << 11)  /* Host: Enable VBUS */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_KEEP_ALIVE_EN             (1 << 10)  /* Host: Enable keep alive packet (for low speed bus) */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_SOF_EN                    (1 << 9)   /* Host: Enable SOF generation (for full speed bus) */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_SOF_SYNC                  (1 << 8)   /* Host: Delay packet(s) until after SOF */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_PREAMBLE_EN               (1 << 6)   /* Host: Preable enable for LS device on FS hub */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_STOP_TRANS                (1 << 4)   /* Host: Stop transaction */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_RECEIVE_DATA              (1 << 3)   /* Host: Receive transaction (IN to host) */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_SEND_DATA                 (1 << 2)   /* Host: Send transaction (OUT from host) */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_SEND_SETUP                (1 << 1)   /* Host: Send Setup packet */
#define RP23XX_USBCTRL_REGS_SIE_CTRL_START_TRANS               (1 << 0)   /* Host: Start transaction */

#define RP23XX_USBCTRL_REGS_SIE_STATUS_DATA_SEQ_ERROR          (1 << 31)  /* Data Sequence Error. */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_ACK_REC                 (1 << 30)  /* ACK received. Raised by both host and device. */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_STALL_REC               (1 << 29)  /* Host: STALL received */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_NAK_REC                 (1 << 28)  /* Host: NAK received */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_RX_TIMEOUT              (1 << 27)  /* RX timeout is raised by both the host and device if an ACK is not received in the maximum time specified by the USB spec. */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_RX_OVERFLOW             (1 << 26)  /* RX overflow is raised by the Serial RX engine if the incoming data is too fast. */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_BIT_STUFF_ERROR         (1 << 25)  /* Bit Stuff Error. Raised by the Serial RX engine. */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_CRC_ERROR               (1 << 24)  /* CRC Error. Raised by the Serial RX engine. */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_ENDPOINT_ERROR          (1 << 23)  /* An endpoint has encounted an error. Read the ep_rx_error and ep_tx_error registers to find out which endpoint had an error */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_BUS_RESET               (1 << 19)  /* Device: bus reset received */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_TRANS_COMPLETE          (1 << 18)  /* Transaction complete. */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_SETUP_REC               (1 << 17)  /* Device: Setup packet received */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_CONNECTED               (1 << 16)  /* Device: connected */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_RX_SHORT_PACKET         (1 << 12)  /* Device or Host has received a short packet. This is when the data recieved is less than configured in the buffer control register. Device: If using double buffered mode on device the buffer select will not be toggled after writing status back to the buffer control register. This is to prevent any further transactions on that endpoint until the user has reset the buffer control registers. Host: the current transfer will be stopped early */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_RESUME                  (1 << 11)  /* Host: Device has initiated a remote resume. Device: host has initiated a resume. */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_VBUS_OVER_CURR          (1 << 10)  /* VBUS over current detected */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_SPEED_SHIFT             (8)        /* Host: device speed. Disconnected = 00, LS = 01, FS = 10 */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_SPEED_MASK              (0x03 << RP23XX_USBCTRL_REGS_SIE_STATUS_SPEED_SHIFT)
#define RP23XX_USBCTRL_REGS_SIE_STATUS_SUSPENDED               (1 << 4)   /* Bus in suspended state. Valid for device and host. Host and device will go into suspend if neither Keep Alive / SOF frames are enabled. */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_LINE_STATE_SHIFT        (2)        /* USB bus line state */
#define RP23XX_USBCTRL_REGS_SIE_STATUS_LINE_STATE_MASK         (0x03 << RP23XX_USBCTRL_REGS_SIE_STATUS_LINE_STATE_SHIFT)
#define RP23XX_USBCTRL_REGS_SIE_STATUS_VBUS_DETECTED           (1 << 0)   /* Device: VBUS Detected */

#define RP23XX_USBCTRL_REGS_INT_EP_CTRL_INT_EP_ACTIVE_SHIFT    (1)        /* Host: Enable interrupt endpoint 1 -> 15 */
#define RP23XX_USBCTRL_REGS_INT_EP_CTRL_INT_EP_ACTIVE_MASK     (0x7fff << RP23XX_USBCTRL_REGS_INT_EP_CTRL_INT_EP_ACTIVE_SHIFT)

#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP15_OUT               (1 << 31)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP15_IN                (1 << 30)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP14_OUT               (1 << 29)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP14_IN                (1 << 28)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP13_OUT               (1 << 27)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP13_IN                (1 << 26)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP12_OUT               (1 << 25)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP12_IN                (1 << 24)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP11_OUT               (1 << 23)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP11_IN                (1 << 22)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP10_OUT               (1 << 21)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP10_IN                (1 << 20)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP9_OUT                (1 << 19)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP9_IN                 (1 << 18)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP8_OUT                (1 << 17)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP8_IN                 (1 << 16)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP7_OUT                (1 << 15)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP7_IN                 (1 << 14)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP6_OUT                (1 << 13)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP6_IN                 (1 << 12)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP5_OUT                (1 << 11)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP5_IN                 (1 << 10)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP4_OUT                (1 << 9)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP4_IN                 (1 << 8)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP3_OUT                (1 << 7)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP3_IN                 (1 << 6)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP2_OUT                (1 << 5)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP2_IN                 (1 << 4)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP1_OUT                (1 << 3)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP1_IN                 (1 << 2)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP0_OUT                (1 << 1)
#define RP23XX_USBCTRL_REGS_BUFF_STATUS_EP0_IN                 (1 << 0)

#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP15_OUT    (1 << 31)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP15_IN     (1 << 30)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP14_OUT    (1 << 29)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP14_IN     (1 << 28)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP13_OUT    (1 << 27)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP13_IN     (1 << 26)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP12_OUT    (1 << 25)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP12_IN     (1 << 24)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP11_OUT    (1 << 23)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP11_IN     (1 << 22)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP10_OUT    (1 << 21)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP10_IN     (1 << 20)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP9_OUT     (1 << 19)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP9_IN      (1 << 18)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP8_OUT     (1 << 17)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP8_IN      (1 << 16)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP7_OUT     (1 << 15)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP7_IN      (1 << 14)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP6_OUT     (1 << 13)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP6_IN      (1 << 12)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP5_OUT     (1 << 11)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP5_IN      (1 << 10)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP4_OUT     (1 << 9)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP4_IN      (1 << 8)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP3_OUT     (1 << 7)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP3_IN      (1 << 6)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP2_OUT     (1 << 5)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP2_IN      (1 << 4)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP1_OUT     (1 << 3)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP1_IN      (1 << 2)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP0_OUT     (1 << 1)
#define RP23XX_USBCTRL_REGS_BUFF_CPU_SHOULD_HANDLE_EP0_IN      (1 << 0)

#define RP23XX_USBCTRL_REGS_EP_ABORT_EP15_OUT                  (1 << 31)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP15_IN                   (1 << 30)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP14_OUT                  (1 << 29)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP14_IN                   (1 << 28)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP13_OUT                  (1 << 27)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP13_IN                   (1 << 26)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP12_OUT                  (1 << 25)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP12_IN                   (1 << 24)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP11_OUT                  (1 << 23)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP11_IN                   (1 << 22)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP10_OUT                  (1 << 21)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP10_IN                   (1 << 20)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP9_OUT                   (1 << 19)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP9_IN                    (1 << 18)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP8_OUT                   (1 << 17)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP8_IN                    (1 << 16)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP7_OUT                   (1 << 15)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP7_IN                    (1 << 14)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP6_OUT                   (1 << 13)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP6_IN                    (1 << 12)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP5_OUT                   (1 << 11)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP5_IN                    (1 << 10)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP4_OUT                   (1 << 9)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP4_IN                    (1 << 8)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP3_OUT                   (1 << 7)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP3_IN                    (1 << 6)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP2_OUT                   (1 << 5)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP2_IN                    (1 << 4)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP1_OUT                   (1 << 3)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP1_IN                    (1 << 2)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP0_OUT                   (1 << 1)
#define RP23XX_USBCTRL_REGS_EP_ABORT_EP0_IN                    (1 << 0)

#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP15_OUT             (1 << 31)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP15_IN              (1 << 30)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP14_OUT             (1 << 29)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP14_IN              (1 << 28)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP13_OUT             (1 << 27)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP13_IN              (1 << 26)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP12_OUT             (1 << 25)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP12_IN              (1 << 24)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP11_OUT             (1 << 23)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP11_IN              (1 << 22)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP10_OUT             (1 << 21)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP10_IN              (1 << 20)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP9_OUT              (1 << 19)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP9_IN               (1 << 18)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP8_OUT              (1 << 17)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP8_IN               (1 << 16)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP7_OUT              (1 << 15)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP7_IN               (1 << 14)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP6_OUT              (1 << 13)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP6_IN               (1 << 12)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP5_OUT              (1 << 11)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP5_IN               (1 << 10)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP4_OUT              (1 << 9)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP4_IN               (1 << 8)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP3_OUT              (1 << 7)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP3_IN               (1 << 6)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP2_OUT              (1 << 5)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP2_IN               (1 << 4)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP1_OUT              (1 << 3)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP1_IN               (1 << 2)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP0_OUT              (1 << 1)
#define RP23XX_USBCTRL_REGS_EP_ABORT_DONE_EP0_IN               (1 << 0)

#define RP23XX_USBCTRL_REGS_EP_STALL_ARM_EP0_OUT               (1 << 1)
#define RP23XX_USBCTRL_REGS_EP_STALL_ARM_EP0_IN                (1 << 0)

#define RP23XX_USBCTRL_REGS_NAK_POLL_RETRY_COUNT_HI_SHIFT      (28)       /* Bits 9:6 of nak_retry count */
#define RP23XX_USBCTRL_REGS_NAK_POLL_RETRY_COUNT_HI_MASK       (0xf << RP23XX_USBCTRL_REGS_NAK_POLL_RETRY_COUNT_HI_SHIFT)
#define RP23XX_USBCTRL_REGS_NAK_POLL_DELAY_FS_SHIFT            (16)       /* NAK polling interval for a full speed device */
#define RP23XX_USBCTRL_REGS_NAK_POLL_DELAY_FS_MASK             (0x3ff << RP23XX_USBCTRL_REGS_NAK_POLL_DELAY_FS_SHIFT)
#define RP23XX_USBCTRL_REGS_NAK_POLL_EPX_STOPPED_ON_NAK        (1 << 27)  /* EPX polling has stopped because a nak was received */
#define RP23XX_USBCTRL_REGS_NAK_POLL_STOP_EPX_ON_NAK           (1 << 26)  /* Stop polling epx when a nak is received */
#define RP23XX_USBCTRL_REGS_NAK_POLL_RETRY_COUNT_LO_SHIFT      (10)       /* Bits 5:0 of nak_retry count */
#define RP23XX_USBCTRL_REGS_NAK_POLL_RETRY_COUNT_LO_MASK       (0x3f << RP23XX_USBCTRL_REGS_NAK_POLL_RETRY_COUNT_LO_SHIFT)
#define RP23XX_USBCTRL_REGS_NAK_POLL_DELAY_LS_MASK             (0x3ff)    /* NAK polling interval for a low speed device */

#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP15_OUT       (1 << 31)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP15_IN        (1 << 30)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP14_OUT       (1 << 29)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP14_IN        (1 << 28)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP13_OUT       (1 << 27)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP13_IN        (1 << 26)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP12_OUT       (1 << 25)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP12_IN        (1 << 24)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP11_OUT       (1 << 23)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP11_IN        (1 << 22)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP10_OUT       (1 << 21)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP10_IN        (1 << 20)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP9_OUT        (1 << 19)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP9_IN         (1 << 18)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP8_OUT        (1 << 17)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP8_IN         (1 << 16)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP7_OUT        (1 << 15)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP7_IN         (1 << 14)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP6_OUT        (1 << 13)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP6_IN         (1 << 12)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP5_OUT        (1 << 11)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP5_IN         (1 << 10)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP4_OUT        (1 << 9)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP4_IN         (1 << 8)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP3_OUT        (1 << 7)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP3_IN         (1 << 6)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP2_OUT        (1 << 5)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP2_IN         (1 << 4)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP1_OUT        (1 << 3)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP1_IN         (1 << 2)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP0_OUT        (1 << 1)
#define RP23XX_USBCTRL_REGS_EP_STATUS_STALL_NAK_EP0_IN         (1 << 0)

#define RP23XX_USBCTRL_REGS_USB_MUXING_SWAP_DPDM               (1 << 31)  /* Swap the USB PHY DP and DM pins and all related controls and flip receive differential data. Can be used to switch USB DP/DP on the PCB. This is done at a low level so overrides all other controls */
#define RP23XX_USBCTRL_REGS_USB_MUXING_USBPHY_AS_GPIO          (1 << 4)   /* Use the usb DP and DM pins as GPIO pins instead of connecting them to the USB controller */
#define RP23XX_USBCTRL_REGS_USB_MUXING_SOFTCON                 (1 << 3)
#define RP23XX_USBCTRL_REGS_USB_MUXING_TO_DIGITAL_PAD          (1 << 2)
#define RP23XX_USBCTRL_REGS_USB_MUXING_TO_EXTPHY               (1 << 1)
#define RP23XX_USBCTRL_REGS_USB_MUXING_TO_PHY                  (1 << 0)

#define RP23XX_USBCTRL_REGS_USB_PWR_OVERCURR_DETECT_EN         (1 << 5)
#define RP23XX_USBCTRL_REGS_USB_PWR_OVERCURR_DETECT            (1 << 4)
#define RP23XX_USBCTRL_REGS_USB_PWR_VBUS_DETECT_OVERRIDE_EN    (1 << 3)
#define RP23XX_USBCTRL_REGS_USB_PWR_VBUS_DETECT                (1 << 2)
#define RP23XX_USBCTRL_REGS_USB_PWR_VBUS_EN_OVERRIDE_EN        (1 << 1)
#define RP23XX_USBCTRL_REGS_USB_PWR_VBUS_EN                    (1 << 0)

#define RP23XX_USBCTRL_REGS_USBPHY_RX_DM_OVERRIDE              (1 << 25)  /* Override rx_dm value into controller */
#define RP23XX_USBCTRL_REGS_USBPHY_RX_DP_OVERRIDE              (1 << 24)  /* Override rx_dp value into controller */
#define RP23XX_USBCTRL_REGS_USBPHY_RX_DD_OVERRIDE              (1 << 23)  /* Override rx_dd value into controller */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_DM_OVV               (1 << 22)  /* DM over voltage */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_DP_OVV               (1 << 21)  /* DP over voltage */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_DM_OVCN              (1 << 20)  /* DM overcurrent */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_DP_OVCN              (1 << 19)  /* DP overcurrent */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_RX_DM                (1 << 18)  /* DPM pin state */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_RX_DP                (1 << 17)  /* DPP pin state */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_RX_DD                (1 << 16)  /* Differential RX */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_TX_DIFFMODE          (1 << 15)  /* TX_DIFFMODE=0: Single ended mode TX_DIFFMODE=1: Differential drive mode (TX_DM, TX_DM_OE ignored) */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_TX_FSSLEW            (1 << 14)  /* TX_FSSLEW=0: Low speed slew rate TX_FSSLEW=1: Full speed slew rate */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_TX_PD                (1 << 13)  /* TX power down override (if override enable is set). 1 = powered down. */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_RX_PD                (1 << 12)  /* RX power down override (if override enable is set). 1 = powered down. */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_TX_DM                (1 << 11)  /* Output data. TX_DIFFMODE=1, Ignored TX_DIFFMODE=0, Drives DPM only. TX_DM_OE=1 to enable drive. DPM=TX_DM */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_TX_DP                (1 << 10)  /* Output data. If TX_DIFFMODE=1, Drives DPP/DPM diff pair. TX_DP_OE=1 to enable drive. DPP=TX_DP, DPM=~TX_DP If TX_DIFFMODE=0, Drives DPP only. TX_DP_OE=1 to enable drive. DPP=TX_DP */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_TX_DM_OE             (1 << 9)   /* Output enable. If TX_DIFFMODE=1, Ignored. If TX_DIFFMODE=0, OE for DPM only. 0 - DPM in Hi-Z state; 1 - DPM driving */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_TX_DP_OE             (1 << 8)   /* Output enable. If TX_DIFFMODE=1, OE for DPP/DPM diff pair. 0 - DPP/DPM in Hi-Z state; 1 - DPP/DPM driving If TX_DIFFMODE=0, OE for DPP only. 0 - DPP in Hi-Z state; 1 - DPP driving */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_DM_PULLDN_EN         (1 << 6)   /* DM pull down enable */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_DM_PULLUP_EN         (1 << 5)   /* DM pull up enable */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_DM_PULLUP_HISEL      (1 << 4)   /* Enable the second DM pull up resistor. 0 - Pull = Rpu2; 1 - Pull = Rpu1 + Rpu2 */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_DP_PULLDN_EN         (1 << 2)   /* DP pull down enable */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_DP_PULLUP_EN         (1 << 1)   /* DP pull up enable */
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_DP_PULLUP_HISEL      (1 << 0)   /* Enable the second DP pull up resistor. 0 - Pull = Rpu2; 1 - Pull = Rpu1 + Rpu2 */

#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_RX_DM_OVERRIDE_EN            (1 << 18)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_RX_DP_OVERRIDE_EN            (1 << 17)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_RX_DD_OVERRIDE_EN            (1 << 16)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_TX_DIFFMODE_OVERRIDE_EN      (1 << 15)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_DM_PULLUP_OVERRIDE_EN        (1 << 12)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_TX_FSSLEW_OVERRIDE_EN        (1 << 11)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_TX_PD_OVERRIDE_EN            (1 << 10)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_RX_PD_OVERRIDE_EN            (1 << 9)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_TX_DM_OVERRIDE_EN            (1 << 8)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_TX_DP_OVERRIDE_EN            (1 << 7)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_TX_DM_OE_OVERRIDE_EN         (1 << 6)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_TX_DP_OE_OVERRIDE_EN         (1 << 5)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_DM_PULLDN_EN_OVERRIDE_EN     (1 << 4)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_DP_PULLDN_EN_OVERRIDE_EN     (1 << 3)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_DP_PULLUP_EN_OVERRIDE_EN     (1 << 2)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_DM_PULLUP_HISEL_OVERRIDE_EN  (1 << 1)
#define RP23XX_USBCTRL_REGS_USBPHY_DIRECT_OVERRIDE_DP_PULLUP_HISEL_OVERRIDE_EN  (1 << 0)

#define RP23XX_USBCTRL_REGS_USBPHY_TRIM_DM_PULLDN_TRIM_SHIFT   (8)        /* Value to drive to USB PHY DM pulldown resistor trim control Experimental data suggests that the reset value will work, but this register allows adjustment if required */
#define RP23XX_USBCTRL_REGS_USBPHY_TRIM_DM_PULLDN_TRIM_MASK    (0x1f << RP23XX_USBCTRL_REGS_USBPHY_TRIM_DM_PULLDN_TRIM_SHIFT)
#define RP23XX_USBCTRL_REGS_USBPHY_TRIM_DP_PULLDN_TRIM_MASK    (0x1f)     /* Value to drive to USB PHY DP pulldown resistor trim control Experimental data suggests that the reset value will work, but this register allows adjustment if required */

#define RP23XX_USBCTRL_REGS_LINESTATE_TUNING_SPARE_FIX_SHIFT                    (8)
#define RP23XX_USBCTRL_REGS_LINESTATE_TUNING_SPARE_FIX_MASK                     (0xf << RP23XX_USBCTRL_REGS_LINESTATE_TUNING_SPARE_FIX_SHIFT)
#define RP23XX_USBCTRL_REGS_LINESTATE_TUNING_DEV_LS_WAKE_FIX                    (1 << 7)  /* Device - exit suspend on any non-idle signalling, not qualified with a 1ms time */
#define RP23XX_USBCTRL_REGS_LINESTATE_TUNING_DEV_RX_ERR_QUIESCE                 (1 << 6)  /* Device - suppress repeated errors until the device FSM is next in the process of decoding an inbound packet */
#define RP23XX_USBCTRL_REGS_LINESTATE_TUNING_SIE_RX_CHATTER_SE0_FIX             (1 << 5)  /* RX - when recovering from line chatter or bitstuff errors, treat SE0 as the end of chatter as well as 8 consecutive idle bits */
#define RP23XX_USBCTRL_REGS_LINESTATE_TUNING_SIE_RX_BITSTUFF_FIX                (1 << 4)  /* RX - when a bitstuff error is signalled by rx_dasm, unconditionally terminate RX decode to avoid a hang during certain packet phases */
#define RP23XX_USBCTRL_REGS_LINESTATE_TUNING_DEV_BUFF_CONTROL_DOUBLE_READ_FIX   (1 << 3)  /* Device - the controller FSM performs two reads of the buffer status memory address to avoid sampling metastable data. An enabled buffer is only used if both reads match */
#define RP23XX_USBCTRL_REGS_LINESTATE_TUNING_MULTI_HUB_FIX                      (1 << 2)  /* Host - increase inter-packet and turnaround timeouts to accommodate worst-case hub delays */
#define RP23XX_USBCTRL_REGS_LINESTATE_TUNING_LINESTATE_DELAY                    (1 << 1)  /* Device/Host - add an extra 1-bit debounce of linestate sampling */
#define RP23XX_USBCTRL_REGS_LINESTATE_TUNING_RCV_DELAY                          (1 << 0)  /* Device - register the received data to account for hub bit dribble before EOP. Only affects certain hubs */

#define RP23XX_USBCTRL_REGS_INTR_EP_STALL_NAK                  (1 << 19)  /* Raised when any bit in EP_STATUS_STALL_NAK is set. Clear by clearing all bits in EP_STATUS_STALL_NAK. */
#define RP23XX_USBCTRL_REGS_INTR_ABORT_DONE                    (1 << 18)  /* Raised when any bit in ABORT_DONE is set. Clear by clearing all bits in ABORT_DONE. */
#define RP23XX_USBCTRL_REGS_INTR_DEV_SOF                       (1 << 17)  /* Set every time the device receives a SOF (Start of Frame) packet. Cleared by reading SOF_RD */
#define RP23XX_USBCTRL_REGS_INTR_SETUP_REQ                     (1 << 16)  /* Device. Source: SIE_STATUS.SETUP_REC */
#define RP23XX_USBCTRL_REGS_INTR_DEV_RESUME_FROM_HOST          (1 << 15)  /* Set when the device receives a resume from the host. Cleared by writing to SIE_STATUS.RESUME */
#define RP23XX_USBCTRL_REGS_INTR_DEV_SUSPEND                   (1 << 14)  /* Set when the device suspend state changes. Cleared by writing to SIE_STATUS.SUSPENDED */
#define RP23XX_USBCTRL_REGS_INTR_DEV_CONN_DIS                  (1 << 13)  /* Set when the device connection state changes. Cleared by writing to SIE_STATUS.CONNECTED */
#define RP23XX_USBCTRL_REGS_INTR_BUS_RESET                     (1 << 12)  /* Source: SIE_STATUS.BUS_RESET */
#define RP23XX_USBCTRL_REGS_INTR_VBUS_DETECT                   (1 << 11)  /* Source: SIE_STATUS.VBUS_DETECT */
#define RP23XX_USBCTRL_REGS_INTR_STALL                         (1 << 10)  /* Source: SIE_STATUS.STALL_REC */
#define RP23XX_USBCTRL_REGS_INTR_ERROR_CRC                     (1 << 9)   /* Source: SIE_STATUS.CRC_ERROR */
#define RP23XX_USBCTRL_REGS_INTR_ERROR_BIT_STUFF               (1 << 8)   /* Source: SIE_STATUS.BIT_STUFF_ERROR */
#define RP23XX_USBCTRL_REGS_INTR_ERROR_RX_OVERFLOW             (1 << 7)   /* Source: SIE_STATUS.RX_OVERFLOW */
#define RP23XX_USBCTRL_REGS_INTR_ERROR_RX_TIMEOUT              (1 << 6)   /* Source: SIE_STATUS.RX_TIMEOUT */
#define RP23XX_USBCTRL_REGS_INTR_ERROR_DATA_SEQ                (1 << 5)   /* Source: SIE_STATUS.DATA_SEQ_ERROR */
#define RP23XX_USBCTRL_REGS_INTR_BUFF_STATUS                   (1 << 4)   /* Raised when any bit in BUFF_STATUS is set. Clear by clearing all bits in BUFF_STATUS. */
#define RP23XX_USBCTRL_REGS_INTR_TRANS_COMPLETE                (1 << 3)   /* Raised every time SIE_STATUS.TRANS_COMPLETE is set. Clear by writing to this bit. */
#define RP23XX_USBCTRL_REGS_INTR_HOST_SOF                      (1 << 2)   /* Host: raised every time the host sends a SOF (Start of Frame). Cleared by reading SOF_RD */
#define RP23XX_USBCTRL_REGS_INTR_HOST_RESUME                   (1 << 1)   /* Host: raised when a device wakes up the host. Cleared by writing to SIE_STATUS.RESUME */
#define RP23XX_USBCTRL_REGS_INTR_HOST_CONN_DIS                 (1 << 0)   /* Host: raised when a device is connected or disconnected (i.e. when SIE_STATUS.SPEED changes). Cleared by writing to SIE_STATUS.SPEED */

#define RP23XX_USBCTRL_REGS_INTE_EP_STALL_NAK                  (1 << 19)  /* Raised when any bit in EP_STATUS_STALL_NAK is set. Clear by clearing all bits in EP_STATUS_STALL_NAK. */
#define RP23XX_USBCTRL_REGS_INTE_ABORT_DONE                    (1 << 18)  /* Raised when any bit in ABORT_DONE is set. Clear by clearing all bits in ABORT_DONE. */
#define RP23XX_USBCTRL_REGS_INTE_DEV_SOF                       (1 << 17)  /* Set every time the device receives a SOF (Start of Frame) packet. Cleared by reading SOF_RD */
#define RP23XX_USBCTRL_REGS_INTE_SETUP_REQ                     (1 << 16)  /* Device. Source: SIE_STATUS.SETUP_REC */
#define RP23XX_USBCTRL_REGS_INTE_DEV_RESUME_FROM_HOST          (1 << 15)  /* Set when the device receives a resume from the host. Cleared by writing to SIE_STATUS.RESUME */
#define RP23XX_USBCTRL_REGS_INTE_DEV_SUSPEND                   (1 << 14)  /* Set when the device suspend state changes. Cleared by writing to SIE_STATUS.SUSPENDED */
#define RP23XX_USBCTRL_REGS_INTE_DEV_CONN_DIS                  (1 << 13)  /* Set when the device connection state changes. Cleared by writing to SIE_STATUS.CONNECTED */
#define RP23XX_USBCTRL_REGS_INTE_BUS_RESET                     (1 << 12)  /* Source: SIE_STATUS.BUS_RESET */
#define RP23XX_USBCTRL_REGS_INTE_VBUS_DETECT                   (1 << 11)  /* Source: SIE_STATUS.VBUS_DETECT */
#define RP23XX_USBCTRL_REGS_INTE_STALL                         (1 << 10)  /* Source: SIE_STATUS.STALL_REC */
#define RP23XX_USBCTRL_REGS_INTE_ERROR_CRC                     (1 << 9)   /* Source: SIE_STATUS.CRC_ERROR */
#define RP23XX_USBCTRL_REGS_INTE_ERROR_BIT_STUFF               (1 << 8)   /* Source: SIE_STATUS.BIT_STUFF_ERROR */
#define RP23XX_USBCTRL_REGS_INTE_ERROR_RX_OVERFLOW             (1 << 7)   /* Source: SIE_STATUS.RX_OVERFLOW */
#define RP23XX_USBCTRL_REGS_INTE_ERROR_RX_TIMEOUT              (1 << 6)   /* Source: SIE_STATUS.RX_TIMEOUT */
#define RP23XX_USBCTRL_REGS_INTE_ERROR_DATA_SEQ                (1 << 5)   /* Source: SIE_STATUS.DATA_SEQ_ERROR */
#define RP23XX_USBCTRL_REGS_INTE_BUFF_STATUS                   (1 << 4)   /* Raised when any bit in BUFF_STATUS is set. Clear by clearing all bits in BUFF_STATUS. */
#define RP23XX_USBCTRL_REGS_INTE_TRANS_COMPLETE                (1 << 3)   /* Raised every time SIE_STATUS.TRANS_COMPLETE is set. Clear by writing to this bit. */
#define RP23XX_USBCTRL_REGS_INTE_HOST_SOF                      (1 << 2)   /* Host: raised every time the host sends a SOF (Start of Frame). Cleared by reading SOF_RD */
#define RP23XX_USBCTRL_REGS_INTE_HOST_RESUME                   (1 << 1)   /* Host: raised when a device wakes up the host. Cleared by writing to SIE_STATUS.RESUME */
#define RP23XX_USBCTRL_REGS_INTE_HOST_CONN_DIS                 (1 << 0)   /* Host: raised when a device is connected or disconnected (i.e. when SIE_STATUS.SPEED changes). Cleared by writing to SIE_STATUS.SPEED */

#define RP23XX_USBCTRL_REGS_INTF_EP_STALL_NAK                  (1 << 19)  /* Raised when any bit in EP_STATUS_STALL_NAK is set. Clear by clearing all bits in EP_STATUS_STALL_NAK. */
#define RP23XX_USBCTRL_REGS_INTF_ABORT_DONE                    (1 << 18)  /* Raised when any bit in ABORT_DONE is set. Clear by clearing all bits in ABORT_DONE. */
#define RP23XX_USBCTRL_REGS_INTF_DEV_SOF                       (1 << 17)  /* Set every time the device receives a SOF (Start of Frame) packet. Cleared by reading SOF_RD */
#define RP23XX_USBCTRL_REGS_INTF_SETUP_REQ                     (1 << 16)  /* Device. Source: SIE_STATUS.SETUP_REC */
#define RP23XX_USBCTRL_REGS_INTF_DEV_RESUME_FROM_HOST          (1 << 15)  /* Set when the device receives a resume from the host. Cleared by writing to SIE_STATUS.RESUME */
#define RP23XX_USBCTRL_REGS_INTF_DEV_SUSPEND                   (1 << 14)  /* Set when the device suspend state changes. Cleared by writing to SIE_STATUS.SUSPENDED */
#define RP23XX_USBCTRL_REGS_INTF_DEV_CONN_DIS                  (1 << 13)  /* Set when the device connection state changes. Cleared by writing to SIE_STATUS.CONNECTED */
#define RP23XX_USBCTRL_REGS_INTF_BUS_RESET                     (1 << 12)  /* Source: SIE_STATUS.BUS_RESET */
#define RP23XX_USBCTRL_REGS_INTF_VBUS_DETECT                   (1 << 11)  /* Source: SIE_STATUS.VBUS_DETECT */
#define RP23XX_USBCTRL_REGS_INTF_STALL                         (1 << 10)  /* Source: SIE_STATUS.STALL_REC */
#define RP23XX_USBCTRL_REGS_INTF_ERROR_CRC                     (1 << 9)   /* Source: SIE_STATUS.CRC_ERROR */
#define RP23XX_USBCTRL_REGS_INTF_ERROR_BIT_STUFF               (1 << 8)   /* Source: SIE_STATUS.BIT_STUFF_ERROR */
#define RP23XX_USBCTRL_REGS_INTF_ERROR_RX_OVERFLOW             (1 << 7)   /* Source: SIE_STATUS.RX_OVERFLOW */
#define RP23XX_USBCTRL_REGS_INTF_ERROR_RX_TIMEOUT              (1 << 6)   /* Source: SIE_STATUS.RX_TIMEOUT */
#define RP23XX_USBCTRL_REGS_INTF_ERROR_DATA_SEQ                (1 << 5)   /* Source: SIE_STATUS.DATA_SEQ_ERROR */
#define RP23XX_USBCTRL_REGS_INTF_BUFF_STATUS                   (1 << 4)   /* Raised when any bit in BUFF_STATUS is set. Clear by clearing all bits in BUFF_STATUS. */
#define RP23XX_USBCTRL_REGS_INTF_TRANS_COMPLETE                (1 << 3)   /* Raised every time SIE_STATUS.TRANS_COMPLETE is set. Clear by writing to this bit. */
#define RP23XX_USBCTRL_REGS_INTF_HOST_SOF                      (1 << 2)   /* Host: raised every time the host sends a SOF (Start of Frame). Cleared by reading SOF_RD */
#define RP23XX_USBCTRL_REGS_INTF_HOST_RESUME                   (1 << 1)   /* Host: raised when a device wakes up the host. Cleared by writing to SIE_STATUS.RESUME */
#define RP23XX_USBCTRL_REGS_INTF_HOST_CONN_DIS                 (1 << 0)   /* Host: raised when a device is connected or disconnected (i.e. when SIE_STATUS.SPEED changes). Cleared by writing to SIE_STATUS.SPEED */

#define RP23XX_USBCTRL_REGS_INTS_EP_STALL_NAK                  (1 << 19)  /* Raised when any bit in EP_STATUS_STALL_NAK is set. Clear by clearing all bits in EP_STATUS_STALL_NAK. */
#define RP23XX_USBCTRL_REGS_INTS_ABORT_DONE                    (1 << 18)  /* Raised when any bit in ABORT_DONE is set. Clear by clearing all bits in ABORT_DONE. */
#define RP23XX_USBCTRL_REGS_INTS_DEV_SOF                       (1 << 17)  /* Set every time the device receives a SOF (Start of Frame) packet. Cleared by reading SOF_RD */
#define RP23XX_USBCTRL_REGS_INTS_SETUP_REQ                     (1 << 16)  /* Device. Source: SIE_STATUS.SETUP_REC */
#define RP23XX_USBCTRL_REGS_INTS_DEV_RESUME_FROM_HOST          (1 << 15)  /* Set when the device receives a resume from the host. Cleared by writing to SIE_STATUS.RESUME */
#define RP23XX_USBCTRL_REGS_INTS_DEV_SUSPEND                   (1 << 14)  /* Set when the device suspend state changes. Cleared by writing to SIE_STATUS.SUSPENDED */
#define RP23XX_USBCTRL_REGS_INTS_DEV_CONN_DIS                  (1 << 13)  /* Set when the device connection state changes. Cleared by writing to SIE_STATUS.CONNECTED */
#define RP23XX_USBCTRL_REGS_INTS_BUS_RESET                     (1 << 12)  /* Source: SIE_STATUS.BUS_RESET */
#define RP23XX_USBCTRL_REGS_INTS_VBUS_DETECT                   (1 << 11)  /* Source: SIE_STATUS.VBUS_DETECT */
#define RP23XX_USBCTRL_REGS_INTS_STALL                         (1 << 10)  /* Source: SIE_STATUS.STALL_REC */
#define RP23XX_USBCTRL_REGS_INTS_ERROR_CRC                     (1 << 9)   /* Source: SIE_STATUS.CRC_ERROR */
#define RP23XX_USBCTRL_REGS_INTS_ERROR_BIT_STUFF               (1 << 8)   /* Source: SIE_STATUS.BIT_STUFF_ERROR */
#define RP23XX_USBCTRL_REGS_INTS_ERROR_RX_OVERFLOW             (1 << 7)   /* Source: SIE_STATUS.RX_OVERFLOW */
#define RP23XX_USBCTRL_REGS_INTS_ERROR_RX_TIMEOUT              (1 << 6)   /* Source: SIE_STATUS.RX_TIMEOUT */
#define RP23XX_USBCTRL_REGS_INTS_ERROR_DATA_SEQ                (1 << 5)   /* Source: SIE_STATUS.DATA_SEQ_ERROR */
#define RP23XX_USBCTRL_REGS_INTS_BUFF_STATUS                   (1 << 4)   /* Raised when any bit in BUFF_STATUS is set. Clear by clearing all bits in BUFF_STATUS. */
#define RP23XX_USBCTRL_REGS_INTS_TRANS_COMPLETE                (1 << 3)   /* Raised every time SIE_STATUS.TRANS_COMPLETE is set. Clear by writing to this bit. */
#define RP23XX_USBCTRL_REGS_INTS_HOST_SOF                      (1 << 2)   /* Host: raised every time the host sends a SOF (Start of Frame). Cleared by reading SOF_RD */
#define RP23XX_USBCTRL_REGS_INTS_HOST_RESUME                   (1 << 1)   /* Host: raised when a device wakes up the host. Cleared by writing to SIE_STATUS.RESUME */
#define RP23XX_USBCTRL_REGS_INTS_HOST_CONN_DIS                 (1 << 0)   /* Host: raised when a device is connected or disconnected (i.e. when SIE_STATUS.SPEED changes). Cleared by writing to SIE_STATUS.SPEED */

#define RP23XX_USBCTRL_REGS_SOF_TIMESTAMP_RAW_MASK             (0x1fffff) /* Device only. Raw value of free-running PHY clock counter @48MHz. Used to calculate time between SOF events */
#define RP23XX_USBCTRL_REGS_SOF_TIMESTAMP_LAST_MASK            (0x1fffff) /* Device only. Value of free-running PHY clock counter @48MHz when last SOF event occured */
#define RP23XX_USBCTRL_REGS_SM_STATE_RX_DASM_SHIFT             (8)
#define RP23XX_USBCTRL_REGS_SM_STATE_RX_DASM_MASK              (0xf << RP23XX_USBCTRL_REGS_SM_STATE_RX_DASM_SHIFT)
#define RP23XX_USBCTRL_REGS_SM_STATE_BC_STATE_SHIFT            (5)
#define RP23XX_USBCTRL_REGS_SM_STATE_BC_STATE_MASK             (0x7 << RP23XX_USBCTRL_REGS_SM_STATE_BC_STATE_SHIFT)
#define RP23XX_USBCTRL_REGS_SM_STATE_STATE_MASK                (0x00001f)

#define RP23XX_USBCTRL_REGS_DEV_SM_WATCHDOG_FIRED              (1 << 20)
#define RP23XX_USBCTRL_REGS_DEV_SM_WATCHDOG_RESET              (1 << 19)  /* Set to 1 to forcibly reset the device state machine on watchdog expiry */
#define RP23XX_USBCTRL_REGS_DEV_SM_WATCHDOG_ENABLE             (1 << 18)
#define RP23XX_USBCTRL_REGS_DEV_SM_WATCHDOG_LIMIT_MASK         (0x03ffff)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_USBCTRL_REGS_H */

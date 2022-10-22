/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_usb.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_USB_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_USB_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SOFT_RESET_REG_MASK               0x03u

#define MPFS_USB_NENDPOINTS               9  /* EP0 + 4x IN and 4x OUT EPs */
#define MPFS_USB_MAXPACKETSIZE(ep)        64
#define MPFS_USB_MAXPACKETSIZE_HS(ep)     512
#define MPFS_EP0_MAXPACKET                64
#define EP0                               0

#define MPFS_USB_FADDR_OFFSET             0x000
#define MPFS_USB_POWER_OFFSET             0x001
#define MPFS_USB_TX_IRQ_OFFSET            0x002
#define MPFS_USB_RX_IRQ_OFFSET            0x004
#define MPFS_USB_TX_IRQ_ENABLE_OFFSET     0x006
#define MPFS_USB_RX_IRQ_ENABLE_OFFSET     0x008
#define MPFS_USB_IRQ_OFFSET               0x00a
#define MPFS_USB_ENABLE_OFFSET            0x00b
#define MPFS_USB_FRAME_OFFSET             0x00c
#define MPFS_USB_INDEX_OFFSET             0x00e
#define MPFS_USB_TEST_MODE_OFFSET         0x00f

#define MPFS_USB_INDEXED_CSR_OFFSET       0x010
#define MPFS_USB_INDEXED_CSR_SIZE         0x10

#define MPFS_USB_INDEXED_CSR (MPFS_USB_BASE + MPFS_USB_INDEXED_CSR_OFFSET)

#define MPFS_USB_INDEXED_CSR_EP0_TX_MAP_P (MPFS_USB_INDEXED_CSR + 0x00)
#define MPFS_USB_INDEXED_CSR_EP0_CSR0     (MPFS_USB_INDEXED_CSR + 0x02)
#define MPFS_USB_INDEXED_CSR_EP0_RX_MAP_P (MPFS_USB_INDEXED_CSR + 0x04)
#define MPFS_USB_INDEXED_CSR_EP0_RX_CSR   (MPFS_USB_INDEXED_CSR + 0x06)
#define MPFS_USB_INDEXED_CSR_EP0_COUNT0   (MPFS_USB_INDEXED_CSR + 0x08)

#define MPFS_USB_INDEXED_CSR_TX_MAP_P     (MPFS_USB_INDEXED_CSR + 0x00)
#define MPFS_USB_INDEXED_CSR_TX_CSR       (MPFS_USB_INDEXED_CSR + 0x02)
#define MPFS_USB_INDEXED_CSR_RX_MAP_P     (MPFS_USB_INDEXED_CSR + 0x04)
#define MPFS_USB_INDEXED_CSR_RX_CSR       (MPFS_USB_INDEXED_CSR + 0x06)
#define MPFS_USB_INDEXED_CSR_RX_COUNT     (MPFS_USB_INDEXED_CSR + 0x08)

#define MPFS_USB_FIFO_OFFSET              0x020
#define MPFS_USB_FIFO_REG_SIZE            0x004

#define MPFS_USB_FIFO_MAX                 16
#define MPFS_USB_FIFO_SIZE                0x04

#define MPFS_USB_DEV_CTRL_OFFSET          0x060
#define MPFS_USB_MISC_OFFSET              0x061
#define MPFS_USB_TX_FIFO_SIZE_OFFSET      0x062
#define MPFS_USB_RX_FIFO_SIZE_OFFSET      0x063
#define MPFS_USB_TX_FIFO_ADDR_OFFSET      0x064
#define MPFS_USB_RX_FIFO_ADDR_OFFSET      0x066
#define MPFS_USB_VBUS_CSR_OFFSET          0x068
#define MPFS_USB_HW_VERSION_OFFSET        0x06c
#define MPFS_USB_RESERVED_OFFSET          0x06e

#define MPFS_USB_ULPI_VBUS_CTRL_OFFSET    0x070
#define MPFS_USB_ULPI_CARKIT_CTRL_OFFSET  0x071
#define MPFS_USB_ULPI_IRQ_MASK_OFFSET     0x072
#define MPFS_USB_ULPI_IRQ_SRC_OFFSET      0x073
#define MPFS_USB_ULPI_DATA_REG_OFFSET     0x074
#define MPFS_USB_ULPI_ADDR_REG_OFFSET     0x075
#define MPFS_USB_ULPI_CTRL_REG_OFFSET     0x076
#define MPFS_USB_ULPI_RAW_DATA_OFFSET     0x077
#define MPFS_USB_EP_INFO_OFFSET           0x078
#define MPFS_USB_RAM_INFO_OFFSET          0x079
#define MPFS_USB_LINK_INFO_OFFSET         0x07a
#define MPFS_USB_VP_LEN_OFFSET            0x07b
#define MPFS_USB_HS_EOF1_OFFSET           0x07c
#define MPFS_USB_FS_EOF1_OFFSET           0x07d
#define MPFS_USB_LS_EOF1_OFFSET           0x07e
#define MPFS_USB_SOFT_RST_OFFSET          0x07f

#define MPFS_USB_TAR_OFFSET               0x080
#define MFFS_USB_TAR_MAX                  16
#define MPFS_USB_TAR_SIZE                 0x08
#define MPFS_USB_TAR_TX_FUNC_ADDR_OFFSET  0x00
#define MPFS_USB_TAR_UNUSED0_OFFSET       0x01
#define MPFS_USB_TAR_TX_HUB_ADDR_OFFSET   0x02
#define MPFS_USB_TAR_TX_HUB_PORT_OFFSET   0x03
#define MPFS_USB_TAR_RX_FUNC_ADDR         0x04
#define MPFS_USB_TAR_UNUSED1              0x05
#define MPFS_USB_TAR_RX_HUB_ADDR          0x06
#define MPFS_USB_TAR_RX_HUB_PORT          0x07

#define MPFS_USB_ENDPOINT_OFFSET          0x100
#define MPFS_USB_ENDPOINT_MAX             16
#define MPFS_USB_ENDPOINT_SIZE            0x10

#define MPFS_USB_ENDPOINT_TX_MAX_P_OFFSET    0x00
#define MPFS_USB_ENDPOINT_TX_CSR_OFFSET      0x02
#define MPFS_USB_ENDPOINT_RX_MAX_P_OFFSET    0x04
#define MPFS_USB_ENDPOINT_RX_CSR_OFFSET      0x06
#define MPFS_USB_ENDPOINT_RX_COUNT_OFFSET    0x08
#define MPFS_USB_ENDPOINT_TX_TYPE_OFFSET     0x0a
#define MPFS_USB_ENDPOINT_TX_INTERVAL_OFFSET 0x0b
#define MPFS_USB_ENDPOINT_RX_TYPE_OFFSET     0x0c
#define MPFS_USB_ENDPOINT_RX_INTERVAL_OFFSET 0x0d
#define MPFS_USB_ENDPOINT_RESERVED_OFFSET    0x0e
#define MPFS_USB_ENDPOINT_FIFO_SIZE_OFFSET   0x0f

#define MPFS_USB_DMA_CHANNEL_OFFSET       0x200
#define MPFS_USB_DMA_CHANNEL_MAX          8
#define MPFS_USB_DMA_CHANNEL_SIZE         0x10
#define MPFS_USB_DMA_IRQ_OFFSET           0x00
#define MPFS_USB_DMA_CNTL_OFFSET          0x04
#define MPFS_USB_DMA_ADDR_OFFSET          0x08
#define MPFS_USB_DMA_COUNT_OFFSET         0x0c

#define MPFS_USB_RESERVED_EXT_OFFSET      0x280
#define MPFS_USB_RQ_PKT_CNT_OFFSET        0x300
#define MPFS_USB_RQ_PKT_CNT_MAX_OFFSET    16
#define MPFS_USB_RQ_PKT_CNT_SIZE_OFFSET   0x04

#define MPFS_USB_RX_DPBUF_DIS_OFFSET      0x340
#define MPFS_USB_TX_DPBUF_DIS_OFFSET      0x342
#define MPFS_USB_C_T_UCH_OFFSET           0x344
#define MPFS_USB_C_T_HHSRTN_OFFSET        0x346
#define MPFS_USB_C_T_HSBT_OFFSET          0x348

#define MPFS_USB_POWER                    (MPFS_USB_BASE + MPFS_USB_POWER_OFFSET)
#define MPFS_USB_POWER_ENABLE_SUSPENDM    (1 << 0)
#define MPFS_USB_POWER_SUSPEND_MODE       (1 << 1)
#define MPFS_USB_POWER_RESUME_SIGNAL      (1 << 2)
#define MPFS_USB_POWER_BUS_RESET_SIGNAL   (1 << 3)
#define MPFS_USB_POWER_HS_MODE            (1 << 4)
#define MPFS_USB_POWER_ENABLE_HS          (1 << 5)
#define MPFS_USB_POWER_SOFT_CONN          (1 << 6)
#define MPFS_USB_POWER_ISO_UPDATE         (1 << 7)

#define MPFS_USB_FADDR         (MPFS_USB_BASE + MPFS_USB_FADDR_OFFSET)
#define MPFS_USB_TX_IRQ        (MPFS_USB_BASE + MPFS_USB_TX_IRQ_OFFSET)
#define MPFS_USB_RX_IRQ        (MPFS_USB_BASE + MPFS_USB_RX_IRQ_OFFSET)
#define MPFS_USB_TX_IRQ_ENABLE (MPFS_USB_BASE + MPFS_USB_TX_IRQ_ENABLE_OFFSET)
#define MPFS_USB_RX_IRQ_ENABLE (MPFS_USB_BASE + MPFS_USB_RX_IRQ_ENABLE_OFFSET)
#define MPFS_USB_IRQ           (MPFS_USB_BASE + MPFS_USB_IRQ_OFFSET)
#define MPFS_USB_ENABLE        (MPFS_USB_BASE + MPFS_USB_ENABLE_OFFSET)
#define MPFS_USB_FRAME         (MPFS_USB_BASE + MPFS_USB_FRAME_OFFSET)
#define MPFS_USB_INDEX         (MPFS_USB_BASE + MPFS_USB_INDEX_OFFSET)
#define MPFS_USB_TEST_MODE     (MPFS_USB_BASE + MPFS_USB_TEST_MODE_OFFSET)
#define MPFS_USB_DEV_CTRL      (MPFS_USB_BASE + MPFS_USB_DEV_CTRL_OFFSET)
#define MPFS_USB_TAR(n)        (MPFS_USB_BASE + MPFS_USB_TAR_OFFSET + MPFS_USB_TAR_SIZE * n)
#define MPFS_USB_TX_FIFO_SIZE  (MPFS_USB_BASE + MPFS_USB_TX_FIFO_SIZE_OFFSET)
#define MPFS_USB_RX_FIFO_SIZE  (MPFS_USB_BASE + MPFS_USB_RX_FIFO_SIZE_OFFSET)
#define MPFS_USB_FIFO(n)       (MPFS_USB_BASE + MPFS_USB_FIFO_OFFSET + MPFS_USB_FIFO_SIZE * n)
#define MPFS_USB_TX_FIFO_ADDR  (MPFS_USB_BASE + MPFS_USB_TX_FIFO_ADDR_OFFSET)
#define MPFS_USB_RX_FIFO_ADDR  (MPFS_USB_BASE + MPFS_USB_RX_FIFO_ADDR_OFFSET)
#define MPFS_USB_SOFT_RST      (MPFS_USB_BASE + MPFS_USB_SOFT_RST_OFFSET)
#define MPFS_USB_C_T_HSBT      (MPFS_USB_BASE + MPFS_USB_C_T_HSBT_OFFSET)
#define MPFS_USB_ENDPOINT(n)   (MPFS_USB_BASE + MPFS_USB_ENDPOINT_OFFSET + MPFS_USB_ENDPOINT_SIZE * n)
#define MPFS_USB_RX_DPBUF_DIS  (MPFS_USB_BASE + MPFS_USB_RX_DPBUF_DIS_OFFSET)
#define MPFS_USB_TX_DPBUF_DIS  (MPFS_USB_BASE + MPFS_USB_TX_DPBUF_DIS_OFFSET)

#define MPFS_USB_DMA_CHANNEL(n) (MPFS_USB_BASE + MPFS_USB_DMA_CHANNEL_OFFSET + MPFS_USB_DMA_CHANNEL_SIZE * n)

/****************************************************************************
 * CSR0H bit masks (peripheral mode)
 ****************************************************************************/

#define CSR0H_DEV_FLUSH_FIFO_MASK                       0x0100u

/****************************************************************************
 * TX_IRQ_ENABLE register masks
 ****************************************************************************/

#define TX_IRQ_ENABLE_REG_CEP_MASK                      0x0001u

/****************************************************************************
 * CSR0L bit masks (peripheral mode)
 ****************************************************************************/

#define CSR0L_DEV_RX_PKT_RDY_MASK                       0x0001u
#define CSR0L_DEV_TX_PKT_RDY_MASK                       0x0002u
#define CSR0L_DEV_STALL_SENT_MASK                       0x0004u
#define CSR0L_DEV_DATA_END_MASK                         0x0008u
#define CSR0L_DEV_SETUP_END_MASK                        0x0010u
#define CSR0L_DEV_SEND_STALL_MASK                       0x0020u
#define CSR0L_DEV_SERVICED_RX_PKT_RDY_MASK              0x0040u
#define CSR0L_DEV_SERVICED_SETUP_END_MASK               0x0080u

/****************************************************************************
 * Endpoint TXMAXP register bit masks
 ****************************************************************************/

#define TX_MAX_P_REG_NUM_USB_PKT_SHIFT                  11u

/****************************************************************************
 * Endpoint TXCSRL register bit masks
 ****************************************************************************/

#define TXCSRL_REG_EPN_TX_PKT_RDY_MASK                  0x0001u
#define TXCSRL_REG_EPN_TX_FIFO_NE_MASK                  0x0002u
#define TXCSRL_REG_EPN_UNDERRUN_MASK                    0x0004u
#define TXCSRL_REG_EPN_FLUSH_FIFO_MASK                  0x0008u
#define TXCSRL_REG_EPN_SEND_STALL_MASK                  0x0010u
#define TXCSRL_REG_EPN_STALL_SENT_MASK                  0x0020u
#define TXCSRL_REG_EPN_CLR_DATA_TOG_MASK                0x0040u
#define TXCSRL_REG_EPN_ISO_INCOMP_TX_MASK               0x0080u

/****************************************************************************
 * Endpoint TXCSRH register bit masks
 ****************************************************************************/

#define TXCSRH_REG_EPN_DMA_MODE_MASK                    0x0400u
#define TXCSRH_REG_EPN_FRC_DAT_TOG_MASK                 0x0800u
#define TXCSRH_REG_EPN_ENABLE_DMA_MASK                  0x1000u
#define TXCSRH_REG_EPN_TXRX_MODE_MASK                   0x2000u
#define TXCSRH_REG_EPN_ENABLE_ISO_MASK                  0x4000u
#define TXCSRH_REG_EPN_ENABLE_AUTOSET_MASK              0x8000u

/****************************************************************************
 * Endpoint DMA_CNTL register bit masks
 ****************************************************************************/

#define DMA_CNTL_REG_START_XFR_MASK                     0x00000001u
#define DMA_CNTL_REG_DMA_DIR_MASK                       0x00000002u
#define DMA_CNTL_REG_DMA_MODE_MASK                      0x00000004u
#define DMA_CNTL_REG_ENABLE_DMA_IRQ_MASK                0x00000008u
#define DMA_CNTL_REG_DMA_EP_NUM_MASK                    0x000000F0u
#define DMA_CNTL_REG_DMA_BUS_ERR_MASK                   0x00000100u
#define DMA_CNTL_REG_DMA_BURST_MODE_MASK                0x00000600u

#define DMA_CNTL_REG_DMA_BURST_MODE_SHIFT               9u
#define DMA_CNTL_REG_DMA_EP_NUM_SHIFT                   4u
#define DMA_CNTL_REG_DMA_DIR_SHIFT                      1u
#define DMA_CNTL_REG_DMA_MODE_SHIFT                     2u

/****************************************************************************
 * Endpoint RXCSRL register bit masks
 ****************************************************************************/

#define RXCSRL_REG_EPN_RX_PKT_RDY_MASK                  0x0001u
#define RXCSRL_REG_EPN_RX_FIFO_FULL_MASK                0x0002u
#define RXCSRL_REG_EPN_OVERRUN_MASK                     0x0004u
#define RXCSRL_REG_EPN_DATA_ERR_MASK                    0x0008u
#define RXCSRL_REG_EPN_FLUSH_FIFO_MASK                  0x0010u
#define RXCSRL_REG_EPN_SEND_STALL_MASK                  0x0020u
#define RXCSRL_REG_EPN_STALL_SENT_MASK                  0x0040u
#define RXCSRL_REG_EPN_CLR_DAT_TOG_MASK                 0x0080u

/****************************************************************************
 * Endpoint RXCSRH register bit masks
 ****************************************************************************/

#define RXCSRL_REG_EPN_RX_ISO_INCOMP                    0x0100u
#define RXCSRL_REG_EPN_DMA_MODE_MASK                    0x0800u
#define RXCSRL_REG_EPN_ISO_PID_ERR_MASK                 0x1000u
#define RXCSRL_REG_EPN_BI_DIS_NYET_MASK                 0x1000u
#define RXCSRL_REG_EPN_ENABLE_DMA_MASK                  0x2000u
#define RXCSRL_REG_EPN_ENABLE_ISO_MASK                  0x4000u
#define RXCSRL_REG_EPN_ENABLE_AUTOCLR_MASK              0x8000u

/****************************************************************************
 * Power register
 ****************************************************************************/

#define POWER_REG_ENABLE_SUSPENDM_MASK                  0x01u
#define POWER_REG_SUSPEND_MODE_MASK                     0x02u
#define POWER_REG_RESUME_SIGNAL_MASK                    0x04u
#define POWER_REG_BUS_RESET_SIGNAL_MASK                 0x08u
#define POWER_REG_HS_MODE_MASK                          0x10u
#define POWER_REG_ENABLE_HS_MASK                        0x20u
#define POWER_REG_SOFT_CONN_MASK                        0x40u
#define POWER_REG_ISO_UPDATE_MASK                       0x80u

/****************************************************************************
 * TXType register bit masks
 ****************************************************************************/

#define TXTYPE_HOST_TARGET_EP_NUM_MASK                  0x0fu
#define TXTYPE_HOST_TARGET_EP_PROTOCOL_MASK             0x30u
#define TXTYPE_HOST_TARGET_EP_SPEED_MASK                0xc0u

#define TXTYPE_HOST_TARGET_EP_NUM_SHIFT                 0u
#define TXTYPE_HOST_TARGET_EP_PROTOCOL_SHIFT            4u
#define TXTYPE_HOST_TARGET_EP_SPEED_SHIFT               6u

/****************************************************************************
 * TXINTERVAL register bit masks
 ****************************************************************************/

#define TXINTERVAL_HOST_REG_MASK                        0x00

/****************************************************************************
 * RXType register bit masks
 ****************************************************************************/

#define RXTYPE_HOST_TARGET_EP_NUM_MASK                  0x0fu
#define RXTYPE_HOST_TARGET_EP_PROTOCOL_MASK             0x30u
#define RXTYPE_HOST_TARGET_EP_SPEED_MASK                0xc0u

#define RXTYPE_HOST_TARGET_EP_NUM_SHIFT                 0u
#define RXTYPE_HOST_TARGET_EP_PROTOCOL_SHIFT            4u
#define RXTYPE_HOST_TARGET_EP_SPEED_SHIFT               6u

/****************************************************************************
 * Endpoint RXCSRL register bit masks
 ****************************************************************************/

#define RXCSRL_HOST_EPN_RX_PKT_RDY_MASK                 0x0001u
#define RXCSRL_HOST_EPN_RX_FIFO_FULL_MASK               0x0002u
#define RXCSRL_HOST_EPN_RESPONSE_ERR_MASK               0x0004u
#define RXCSRL_HOST_EPN_NAK_TIMEOUT_ERR_MASK            0x0008u
#define RXCSRL_HOST_EPN_FLUSH_FIFO_MASK                 0x0010u
#define RXCSRL_HOST_EPN_IN_PKT_REQ_MASK                 0x0020u
#define RXCSRL_HOST_EPN_STALL_RCVD_MASK                 0x0040u
#define RXCSRL_HOST_EPN_CLR_DATA_TOG_MASK               0x0080u

/****************************************************************************
 * CSR0L bit masks
 ****************************************************************************/

#define CSR0L_HOST_RX_PKT_RDY_MASK                      0x0001u
#define CSR0L_HOST_TX_PKT_RDY_MASK                      0x0002u
#define CSR0L_HOST_STALL_RCVD_MASK                      0x0004u
#define CSR0L_HOST_SETUP_PKT_MASK                       0x0008u
#define CSR0L_HOST_RETRY_ERR_MASK                       0x0010u
#define CSR0L_HOST_IN_PKT_REQ_MASK                      0x0020u
#define CSR0L_HOST_STATUS_PKT_MASK                      0x0040u
#define CSR0L_HOST_NAK_TIMEOUT_MASK                     0x0080u

/****************************************************************************
 * CSR0H bit masks
 ****************************************************************************/

#define CSR0H_HOST_FLUSH_FIFO_MASK                      0x0100u /* Self Clearing */
#define CSR0H_HOST_DATA_TOG_MASK                        0x0200u
#define CSR0H_HOST_DATA_TOG_WE_MASK                     0x0400u /* Self Clearing */
#define CSR0H_HOST_DISABLE_PING_MASK                    0x0800u

/****************************************************************************
 * INTRUSBE register - USB interrupts masks
 ****************************************************************************/

#define SUSPEND_IRQ_MASK                                0x01u
#define RESUME_IRQ_MASK                                 0x02u
#define RESET_IRQ_MASK                                  0x04u   /* Device mode */
#define BABBLE_IRQ_MASK                                 0x04u   /* Host mode */
#define SOF_IRQ_MASK                                    0x08u
#define CONNECT_IRQ_MASK                                0x10u
#define DISCONNECT_IRQ_MASK                             0x20u
#define SESSION_REQUEST_IRQ_MASK                        0x40u
#define VBUS_ERROR_IRQ_MASK                             0x80u

/****************************************************************************
 * DevCTL register bit masks
 ****************************************************************************/

#define DEV_CTRL_SESSION_MASK                           0x01u
#define DEV_CTRL_HOST_REQ_MASK                          0x02u
#define DEV_CTRL_HOST_MODE_MASK                         0x04u
#define DEV_CTRL_VBUS_MASK                              0x18u
#define DEV_CTRL_LS_DEV_MASK                            0x20u
#define DEV_CTRL_FS_DEV_MASK                            0x40u
#define DEV_CTRL_B_DEVICE_MASK                          0x80u

#define VBUS_BELOW_SESSION_END                          0x00u
#define VBUS_ABOVE_SESSION_END                          0x08u
#define VBUS_ABOVE_AVALID                               0x10u
#define VBUS_ABOVE_VBUS_VALID                           0x18u

/****************************************************************************
 * Endpoint TXMAXP register bit masks
 ****************************************************************************/

#define RX_MAX_P_REG_NUM_USB_PKT_SHIFT                  11u

struct mpfs_req_s
{
  struct usbdev_req_s   req;          /* Standard USB request */
  struct mpfs_req_s    *flink;        /* Supports a singly linked list */
  uint16_t              inflight;     /* Number of TX bytes transmitting or
                                       * number of RX bytes we are waiting */
};

union wb_u
{
  uint16_t w;
  uint8_t  b[2];
};

/* The head of a queue of requests */

struct mpfs_rqhead_s
{
  struct mpfs_req_s     *head;         /* Requests are added to the head of the list */
  struct mpfs_req_s     *tail;         /* Requests are removed from the tail of the list */
};

struct mpfs_ep_s
{
  struct usbdev_ep_s    ep;           /* Standard endpoint structure */

  struct mpfs_usbdev_s  *dev;         /* Reference to private driver data */
  struct mpfs_rqhead_s  reqq;         /* Read/write request queue */
  struct mpfs_rqhead_s  pendq;        /* Write requests pending stall sent */
  struct usbdev_epdesc_s *descb[2];   /* Pointers to this endpoint descriptors */
  volatile uint8_t      epstate;      /* State of the endpoint (see enum mpfs_epstate_e) */
  uint8_t               stalled:1;    /* true: Endpoint is stalled */
  uint8_t               pending:1;    /* true: IN Endpoint stall is pending */
  uint8_t               halted:1;     /* true: Endpoint feature halted */
  uint8_t               zlpsent:1;    /* Zero length packet has been sent */
  uint8_t               txbusy:1;     /* Write request queue is busy */
  uint8_t               rxactive:1;   /* read request is active (for top of queue) */
};

/* Device Endpoint Descriptor.  See USBDEV_* bit definitions above. */

struct usbdev_epdesc_s
{
  uintptr_t addr;        /* Address of Data buffer (Both banks) */
  uint32_t  pktsize;     /* Packet Size */
};

struct mpfs_usbdev_s
{
  struct usbdev_s usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* USB-specific fields */

  struct usb_ctrlreq_s ctrl;          /* Last EP0 request */
  uint8_t              devstate;      /* State of the device (see enum mpfs_devstate_e) */
  uint8_t              prevstate;     /* Previous state of the device before SUSPEND */
  uint8_t              devaddr;       /* Assigned device address */
  uint8_t              selfpowered:1; /* 1: Device is self powered */
  uint16_t             epavail;       /* Bitset of available endpoints */

  /* The endpoint list */

  aligned_data(4) struct mpfs_ep_s eplist[MPFS_USB_NENDPOINTS];

  /* Endpoint descriptors 2 banks for each endpoint */

  aligned_data(4)
  struct usbdev_epdesc_s ep_descriptors[MPFS_USB_NENDPOINTS * 2];

  /* EP0 data buffer.  For data that is included in an EP0 SETUP OUT
   * transaction.  In this case, no request is in place from the class
   * driver and the incoming data is caught in this buffer.  The size
   * of valid data in the buffer is given by ctrlreg.len[].  For the
   * case of EP0 SETUP IN transaction, the normal request mechanism is
   * used and the class driver provides the buffering.
   */

  aligned_data(4) uint8_t ep0out[MPFS_EP0_MAXPACKET];
};

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_USB_H */

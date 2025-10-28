/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_can.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/time.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/can.h>
#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/can.h>
#include <nuttx/can/can.h>

#include <arch/board/board.h>

#include "mpfs_can.h"
#include "riscv_internal.h"
#include "mpfs_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_MPFS_MSS_CAN
#  error This should not be compiled if MSS CAN block is not enabled
#endif

/* CAN 0 and 1 register base definition */

#define CAN0_BASE       MPFS_CAN0_LO_BASE
#define CAN1_BASE       MPFS_CAN1_LO_BASE

/* High level driver operational configuration */

#define CANWORK         HPWORK

/* For allocating the tx and rx CAN frame buffer */

#define POOL_SIZE       1
#define TIMESTAMP_SIZE  sizeof(struct timeval) /* support timestamping frame */

/* MSS CAN TX/RX buffer configuration */

#define CAN_RX_BUFFER               32
#define CAN_TX_BUFFER               32
#define CAN_RX_BUFFER_CTRL_DEFAULT  MPFS_CAN_RX_MSG_CTRL_CMD_WPNH \
                                    | MPFS_CAN_RX_MSG_CTRL_CMD_WPNL \
                                    | MPFS_CAN_RX_MSG_CTRL_CMD_RX_BUFFER_EBL \
                                    | MPFS_CAN_RX_MSG_CTRL_CMD_RX_INT_ENABLE

/* MSS CAN Configuration and Speed definitions */

#define CAN_SAMPLE_BOTH_EDGES  MPFS_CAN_CAN_CONFIG_EDGE_MODE
#define CAN_THREE_SAMPLES      MPFS_CAN_CAN_CONFIG_SAMPLING_MODE
#define CAN_SET_SJW(_sjw)      (_sjw << MPFS_CAN_CAN_CONFIG_CFG_SJW_SHIFT)
#define CAN_AUTO_RESTART       MPFS_CAN_CAN_CONFIG_AUTO_RESTART
#define CAN_SET_TSEG2(_tseg2)  (_tseg2 << MPFS_CAN_CAN_CONFIG_CFG_TSEG2_SHIFT)
#define CAN_SET_TSEG1(_tseg1)  (_tseg1 << MPFS_CAN_CAN_CONFIG_CFG_TSEG1_SHIFT)
#define CAN_SET_BITRATE(_br)   (_br << MPFS_CAN_CAN_CONFIG_CFG_BITRATE_SHIFT)
#define CAN_ARB_FIXED_PRIO     MPFS_CAN_CAN_CONFIG_CFG_ARBITER
#define CAN_LITTLE_ENDIAN      MPFS_CAN_CAN_CONFIG_SWAP_ENDIAN

/* The following constants are used in the PolarFire SoC MSS CAN driver for
 * bitrate definitions:
 *
 * | Constants          |  Description                                      |
 * |--------------------|---------------------------------------------------|
 * | CAN_SPEED_8M_5K    | Indicates CAN controller shall be configured with |
 * |                    | 5Kbps baud rate if the input clock is 8MHz.       |
 * | CAN_SPEED_16M_5K   | Indicates CAN controller shall be configured with |
 * |                    | 5Kbps baud rate if the input clock is 16MHz.      |
 * | CAN_SPEED_32M_5K   | Indicates CAN controller shall be configured with |
 * |                    | 5Kbps baud rate if the input clock is 32MHz.      |
 * | CAN_SPEED_8M_10K   | Indicates CAN controller shall be configured with |
 * |                    | 10Kbps baud rate if the input clock is 8MHz.      |
 * | CAN_SPEED_16M_10K  | Indicates CAN controller shall be configured with |
 * |                    | 10Kbps baud rate if the input clock is 16MHz.     |
 * | CAN_SPEED_32M_10K  | Indicates CAN controller shall be configured with |
 * |                    | 10Kbps baud rate if the input clock is 32MHz.     |
 * | CAN_SPEED_8M_20K   | Indicates CAN controller shall be configured with |
 * |                    | 20Kbps baud rate if the input clock is 8MHz.      |
 * | CAN_SPEED_16M_20K  | Indicates CAN controller shall be configured with |
 * |                    | 20Kbps baud rate if the input clock is 16MHz.     |
 * | CAN_SPEED_32M_20K  | Indicates CAN controller shall be configured with |
 * |                    | 20Kbps baud rate if the input clock is 32MHz.     |
 * | CAN_SPEED_8M_50K   | Indicates CAN controller shall be configured with |
 * |                    | 50Kbps baud rate if the input clock is 8MHz.      |
 * | CAN_SPEED_16M_50K  | Indicates CAN controller shall be configured with |
 * |                    | 50Kbps baud rate if the input clock is 16MHz.     |
 * | CAN_SPEED_32M_50K  | Indicates CAN controller shall be configured with |
 * |                    | 50Kbps baud rate if the input clock is 32MHz.     |
 * | CAN_SPEED_8M_100K  | Indicates CAN controller shall be configured with |
 * |                    | 100Kbps baud rate if the input clock is 8MHz.     |
 * | CAN_SPEED_16M_100K | Indicates CAN controller shall be configured with |
 * |                    | 100Kbps baud rate if the input clock is 16MHz.    |
 * | CAN_SPEED_32M_100K | Indicates CAN controller shall be configured with |
 * |                    | 100Kbps baud rate if the input clock is 32MHz.    |
 * | CAN_SPEED_8M_125K  | Indicates CAN controller shall be configured with |
 * |                    | 125Kbps baud rate if the input clock is 8MHz.     |
 * | CAN_SPEED_16M_125K | Indicates CAN controller shall be configured with |
 * |                    | 125Kbps baud rate if the input clock is 16MHz.    |
 * | CAN_SPEED_32M_125K | Indicates CAN controller shall be configured with |
 * |                    | 125Kbps baud rate if the input clock is 32MHz.    |
 * | CAN_SPEED_8M_250K  | Indicates CAN controller shall be configured with |
 * |                    | 250Kbps baud rate if the input clock is 8MHz.     |
 * | CAN_SPEED_16M_250K | Indicates CAN controller shall be configured with |
 * |                    | 250Kbps baud rate if the input clock is 16MHz.    |
 * | CAN_SPEED_32M_250K | Indicates CAN controller shall be configured with |
 * |                    | 250Kbps baud rate if the input clock is 32MHz.    |
 * | CAN_SPEED_8M_500K  | Indicates CAN controller shall be configured with |
 * |                    | 500Kbps baud rate if the input clock is 8MHz.     |
 * | CAN_SPEED_16M_500K | Indicates CAN controller shall be configured with |
 * |                    | 500Kbps baud rate if the input clock is 16MHz.    |
 * | CAN_SPEED_32M_500K | Indicates CAN controller shall be configured with |
 * |                    | 500Kbps baud rate if the input clock is 32MHz.    |
 * | CAN_SPEED_8M_1M    | Indicates CAN controller shall be configured with |
 * |                    | 1MBPS baud rate if the input clock is 8MHz.       |
 * | CAN_SPEED_16M_1M   | Indicates CAN controller shall be configured with |
 * |                    | 1MBPS baud rate if the input clock is 16MHz.      |
 * | CAN_SPEED_32M_1M   | Indicates CAN controller shall be configured with |
 * |                    | 1MBPS baud rate if the input clock is 32MHz.      |
 */

/* 5000m       81%  Sample bit three times  */

#define CAN_SPEED_8M_5K      CAN_SET_BITRATE(99)|CAN_SET_TSEG1(11) \
                              |CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES
#define CAN_SPEED_16M_5K     CAN_SET_BITRATE(199)|CAN_SET_TSEG1(11) \
                              |CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES
#define CAN_SPEED_32M_5K     CAN_SET_BITRATE(399)|CAN_SET_TSEG1(11) \
                              |CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES

/* 5000m       81%  Sample bit three times */

#define CAN_SPEED_8M_10K     CAN_SET_BITRATE(49)|CAN_SET_TSEG1(11) \
                              |CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES
#define CAN_SPEED_16M_10K    CAN_SET_BITRATE(99)|CAN_SET_TSEG1(11) \
                              |CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES
#define CAN_SPEED_32M_10K    CAN_SET_BITRATE(199)|CAN_SET_TSEG1(11) \
                              |CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES

/* 2500m       81%  Sample bit three times */

#define CAN_SPEED_8M_20K     CAN_SET_BITRATE(24)|CAN_SET_TSEG1(11) \
                              |CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES
#define CAN_SPEED_16M_20K    CAN_SET_BITRATE(49)|CAN_SET_TSEG1(11) \
                              |CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES
#define CAN_SPEED_32M_20K    CAN_SET_BITRATE(99)|CAN_SET_TSEG1(11) \
                              |CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES

/* 1000m       87% */

#define CAN_SPEED_8M_50K     CAN_SET_BITRATE(9)|CAN_SET_TSEG1(12) \
                              |CAN_SET_TSEG2(1)
#define CAN_SPEED_16M_50K    CAN_SET_BITRATE(19)|CAN_SET_TSEG1(12) \
                              |CAN_SET_TSEG2(1)
#define CAN_SPEED_32M_50K    CAN_SET_BITRATE(39)|CAN_SET_TSEG1(12) \
                              |CAN_SET_TSEG2(1)

/* 600m        87% */

#define CAN_SPEED_8M_100K    CAN_SET_BITRATE(4)|CAN_SET_TSEG1(12) \
                              |CAN_SET_TSEG2(1)
#define CAN_SPEED_16M_100K   CAN_SET_BITRATE(9)|CAN_SET_TSEG1(12) \
                              |CAN_SET_TSEG2(1)
#define CAN_SPEED_32M_100K   CAN_SET_BITRATE(19)|CAN_SET_TSEG1(12) \
                              |CAN_SET_TSEG2(1)

/*  500m        87% */

#define CAN_SPEED_8M_125K    CAN_SET_BITRATE(3)|CAN_SET_TSEG1(12) \
                              |CAN_SET_TSEG2(1)
#define CAN_SPEED_16M_125K   CAN_SET_BITRATE(7)|CAN_SET_TSEG1(12) \
                              |CAN_SET_TSEG2(1)
#define CAN_SPEED_32M_125K   CAN_SET_BITRATE(15)|CAN_SET_TSEG1(12) \
                              |CAN_SET_TSEG2(1)

/* 250m        87% */

#define CAN_SPEED_8M_250K    CAN_SET_BITRATE(1)|CAN_SET_TSEG1(12) \
                              |CAN_SET_TSEG2(1)
#define CAN_SPEED_16M_250K   CAN_SET_BITRATE(3)|CAN_SET_TSEG1(12) \
                              |CAN_SET_TSEG2(1)
#define CAN_SPEED_32M_250K   CAN_SET_BITRATE(7)|CAN_SET_TSEG1(12) \
                              |CAN_SET_TSEG2(1)

/* 100m        75% @ 8M, 87% @ 16M */

#define CAN_SPEED_8M_500K    CAN_SET_BITRATE(1)|CAN_SET_TSEG1(4) \
                              |CAN_SET_TSEG2(1)
#define CAN_SPEED_16M_500K   CAN_SET_BITRATE(1)|CAN_SET_TSEG1(12) \
                              |CAN_SET_TSEG2(1)
#define CAN_SPEED_32M_500K   CAN_SET_BITRATE(3)|CAN_SET_TSEG1(12) \
                              |CAN_SET_TSEG2(1)

/* 25m         75% */
#define CAN_SPEED_8M_1M      CAN_SET_BITRATE(0)|CAN_SET_TSEG1(4) \
                              |CAN_SET_TSEG2(1)
#define CAN_SPEED_16M_1M     CAN_SET_BITRATE(1)|CAN_SET_TSEG1(4) \
                              |CAN_SET_TSEG2(1)
#define CAN_SPEED_32M_1M     CAN_SET_BITRATE(3)|CAN_SET_TSEG1(4) \
                              |CAN_SET_TSEG2(1)

/* The following constants are used for error codes:
 *
 * |  Constants            |  Description                                |
 * |-----------------------|---------------------------------------------|
 * | CAN_OK                | Indicates there is no error                 |
 * | CAN_ERR               | Indicates error condition                   |
 * | CAN_TSEG1_TOO_SMALL   | Value provided to configure TSEG1 is too    |
 * |                       | small                                       |
 * | CAN_TSEG2_TOO_SMALL   | Value provided to configure TSEG2 is too    |
 * |                       | small                                       |
 * | CAN_SJW_TOO_BIG       | Value provided to configure synchronous jump|
 * |                       | width (SJW) is too big.                     |
 * | CAN_BASIC_CAN_BUFFER  | Indicates that buffer is configured for     |
 * |                       | Basic CAN operation                         |
 * | CAN_NO_RTR_BUFFER     | Indicates that there is no buffer for       |
 * |                       | remote transmit request (RTR) frame         |
 * | CAN_INVALID_BUFFER    | Indicates invalid buffer number             |
 * | CAN_NO_MSG            | Indicates no message available              |
 * | CAN_VALID_MSG         | Indicates message is valid                  |
 */

#define CAN_OK                  0
#define CAN_ERR                 1
#define CAN_TSEG1_TOO_SMALL     2
#define CAN_TSEG2_TOO_SMALL     3
#define CAN_SJW_TOO_BIG         4
#define CAN_BASIC_CAN_BUFFER    5
#define CAN_NO_RTR_BUFFER       6
#define CAN_INVALID_BUFFER      7
#define CAN_NO_MSG              8
#define CAN_VALID_MSG           0

/****************************************************************************
 * Utility definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_CAN_INFO
static inline void print_uint32_t(const char *prefix, uint32_t val)
{
  /* prefix + " 0b" + 32 bits + null terminator */

  char binary_str[strlen(prefix) + 2 + 32 + 1];

  sprintf(binary_str, "%s 0b", prefix);
  for (int i = 31; i >= 0; i--)
    {
      sprintf(binary_str + strlen(binary_str), "%d", (val >> i) & 1);
    }
  caninfo("%s", binary_str);
}
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The mpfs_can_mode_t enumeration specifies the possible operating modes of
 * CAN controller. The meaning of the constants is as described below
 *
 * |  Modes                     |  Description                             |
 * |----------------------------|------------------------------------------|
 * | CANOP_MODE_NORMAL          | Indicates CAN controller is in normal    |
 * |                            | operational mode.                        |
 * | CANOP_MODE_LISTEN_ONLY     | Indicates CAN controller is in listen    |
 * |                            | only mode.                               |
 * | CANOP_MODE_EXT_LOOPBACK    | Indicates CAN controller is in external  |
 * |                            | loop back mode.                          |
 * | CANOP_MODE_INT_LOOPBACK    | Indicates CAN controller is in internal  |
 * |                            | loop back mode.                          |
 * | CANOP_SRAM_TEST_MODE       | Indicates CAN controller is in test mode.|
 */

enum mpfs_can_mode_e
{
  CANOP_MODE_NORMAL       = MPFS_CAN_CAN_COMMAND_RUN_STOP_MODE,
  CANOP_MODE_LISTEN_ONLY  = MPFS_CAN_CAN_COMMAND_RUN_STOP_MODE |
                            MPFS_CAN_CAN_COMMAND_LISTEN_ONLY_MODE,
  CANOP_MODE_EXT_LOOPBACK = MPFS_CAN_CAN_COMMAND_RUN_STOP_MODE |
                            MPFS_CAN_CAN_COMMAND_LOOPBACK_TEST_MODE,
  CANOP_MODE_INT_LOOPBACK = MPFS_CAN_CAN_COMMAND_RUN_STOP_MODE |
                            MPFS_CAN_CAN_COMMAND_LISTEN_ONLY_MODE |
                            MPFS_CAN_CAN_COMMAND_LOOPBACK_TEST_MODE,
  CANOP_SRAM_TEST_MODE    = MPFS_CAN_CAN_COMMAND_SRAM_TEST_MODE
};

typedef enum mpfs_can_mode_e mpfs_can_mode_t;

/* CAN message object */

struct mpfs_can_msgobject_s
{
  /* CAN Message flags */

  uint32_t msg_ctrl;

  /* CAN Message ID. */

  uint32_t id;

  /* CAN Message Data organized as two 32 bit words */

  uint32_t data_high;
  uint32_t data_low;
};

typedef struct mpfs_can_msgobject_s mpfs_can_msgobject_t;

/* CAN RX message object */

struct mpfs_can_rxmsgobject_s
{
  /* CAN Message flags */

  uint32_t rx_msg_ctrl;

  /* CAN Message ID */

  uint32_t id;

  /* CAN Message Data organized as two 32 bit words */

  uint32_t data_high;
  uint32_t data_low;

  /* CAN Message Filter: acceptance mask and code */

  uint32_t amr;
  uint32_t acr;

  /* CAN Message Filter: Acceptance mask and code data bits */

  uint32_t amr_data;
  uint32_t acr_data;
};

typedef struct mpfs_can_rxmsgobject_s mpfs_can_rxmsgobject_t;

/* CAN filter object */

struct mpfs_can_filterobject_s
{
  /* Use sw mask filter */

  bool use_mask_filter;

  /* CAN Message Filter: acceptance mask and code */

  uint32_t amr;
  uint32_t acr;

  /* CAN Message Filter: Acceptance mask and code data bits */

  uint32_t amr_data;
  uint32_t acr_data;
};

typedef struct mpfs_can_filterobject_s mpfs_can_filterobject_t;

/* CAN device statistics */

struct mpfs_can_device_stats_s
{
  volatile uint32_t error_passive;     /* Changes to error passive count */
  volatile uint32_t bus_off;           /* Changes to bus off count */
  volatile uint32_t arbitration_loss;  /* Arbitration loss errors count */
  volatile uint32_t rx_overload;       /* Rx overload errors count */
  volatile uint32_t bit_errors;        /* Bit errors count */
  volatile uint32_t stuff_errors;      /* Stuffing errors count */
  volatile uint32_t ack_errors;        /* Ack errors count */
  volatile uint32_t form_errors;       /* Form errors count */
  volatile uint32_t crc_errors;        /* CRC errors count */
  volatile uint32_t stuck_at_0;        /* Stuck at 0 errors count */
  volatile uint32_t restarts;          /* CAN controller re-starts count */
  volatile uint32_t txb_sent;          /* Tx messages sent count */
};

typedef struct mpfs_can_device_stats_s mpfs_can_device_stats_t;

/* The structure mpfs_can_instance_t is used by the driver to manage the
 * configuration and operation of each MSS CAN peripheral. The instance
 * content should only be accessed by using the respective API functions.
 *
 * Each API function has a pointer to this instance as first argument.
 */

struct mpfs_can_instance_s
{
  uintptr_t reg_base;       /* Pointer to CAN base register address */

  uint32_t bitrate_value;    /* The numerical bitrate value in bit/s */

  bool bifup;               /* Indicates if the CAN is up or down. */

  /* Interrupt handling */

  uint8_t irqn;             /* IRQ number */
  uint32_t isr;             /* Interrupt status register */

  /* Error status and Stats */

  uint8_t error_status;                 /* Error status */
  uint32_t tx_err_count;                /* Tx error count */
  uint32_t rx_err_count;                /* Rx error count */
  mpfs_can_device_stats_t stats;        /* Device statistics */

  /* buffer count */

  uint8_t  basic_can_rxb_count; /* number of rx buffers */
  uint8_t  basic_can_txb_count; /* number of tx buffers */

  /* Frame descriptors */

  struct can_frame *txdesc; /* Pointer to the transmit frame descriptor. */
  struct can_frame *rxdesc; /* Pointer to the receive frame descriptor. */

  /* MSS CAN composite message objects */

  mpfs_can_msgobject_t *tx_msg; /* Pointer to the transmit message object */
  mpfs_can_msgobject_t *rx_msg; /* Pointer to the receive message object */

  /* Work queue entries */

  struct work_s rxwork;   /* for deferring rx interrupt work to the wq */
  struct work_s txdwork;  /* For deferring tx done interrupt work to the wq */
  struct work_s pollwork; /* For deferring poll work to the wq */

  mpfs_can_filterobject_t filter;  /* hardware and software filters */

  struct net_driver_s dev;  /* Interface understood by the Nuttx network */
};

typedef struct mpfs_can_instance_s mpfs_can_instance_t;

/* Driver memory pool */

#ifdef CONFIG_MPFS_MSS_CAN0
static mpfs_can_instance_t g_can0;

static uint8_t g_tx_pool0[(sizeof(struct can_frame) + TIMESTAMP_SIZE) *
                          POOL_SIZE] aligned_data(sizeof(uint32_t));
static uint8_t g_rx_pool0[(sizeof(struct can_frame) + TIMESTAMP_SIZE) *
                          POOL_SIZE] aligned_data(sizeof(uint32_t));

static mpfs_can_msgobject_t g_tx_msg0;
static mpfs_can_msgobject_t g_rx_msg0;
#endif

#ifdef CONFIG_MPFS_MSS_CAN1
static mpfs_can_instance_t g_can1;

static uint8_t g_tx_pool1[(sizeof(struct can_frame) + TIMESTAMP_SIZE) *
                         POOL_SIZE] aligned_data(sizeof(uint32_t));
static uint8_t g_rx_pool1[(sizeof(struct can_frame) + TIMESTAMP_SIZE) *
                         POOL_SIZE] aligned_data(sizeof(uint32_t));

static mpfs_can_msgobject_t g_tx_msg1;
static mpfs_can_msgobject_t g_rx_msg1;
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* (from interrupt) High-level RX related functions */

static bool mpfs_can_retrieve_rx_frame(mpfs_can_instance_t *priv,
                                       struct can_frame *cf);
static void mpfs_receive_work(void *arg);

/* (from interrupt) High-level TX related functions */

static void mpfs_txdone_work(void *arg);

/* High-level periodical TX related functions */

static int mpfs_transmit(mpfs_can_instance_t *priv);
static int mpfs_txpoll(struct net_driver_s *dev);
static void mpfs_txavail_work(void *arg);
static int mpfs_txavail(struct net_driver_s *dev);

/* (from interrupt) High-level error handling related functions */

static void mpfs_err_interrupt(mpfs_can_instance_t *priv, uint32_t isr);

/* Interrupt service routine */

static int mpfs_interrupt(int irq, void *context, void *arg);

/* RX SW/HW filter related functions */

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
static uint8_t mpfs_can_add_filter(mpfs_can_instance_t *priv,
                                   uint8_t filter_type,
                                   uint32_t filter_id1,
                                   uint32_t filter_id2);
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

static uint8_t mpfs_can_reset_filter(mpfs_can_instance_t *priv);

/* CAN controller configuration setter and status getter helper functions */

static void mpfs_can_reset(mpfs_can_instance_t *priv);

static void mpfs_can_set_mode(mpfs_can_instance_t *priv,
                              mpfs_can_mode_t mode);

#ifdef CONFIG_DEBUG_CAN_INFO
static inline uint32_t
mpfs_can_get_can_command_reg(mpfs_can_instance_t *priv);

static inline uint32_t
mpfs_can_get_can_config_reg(mpfs_can_instance_t *priv);
#endif

static void mpfs_can_set_int_ebl(mpfs_can_instance_t *priv, uint32_t flag);
static void mpfs_can_clear_int_ebl(mpfs_can_instance_t *priv, uint32_t flag);
static uint32_t mpfs_can_get_int_ebl(mpfs_can_instance_t *priv);

static void mpfs_can_clear_int_status(mpfs_can_instance_t *priv,
                                      uint32_t flag);
static uint32_t mpfs_can_get_int_status(mpfs_can_instance_t *priv);

static uint8_t mpfs_can_get_error_status(mpfs_can_instance_t *priv);

#ifdef CONFIG_DEBUG_CAN_INFO
static void mpfs_can_print_status (mpfs_can_instance_t *priv);
#endif

/* CAN controller life cycle functions */

static void mpfs_can_start(mpfs_can_instance_t *priv);
static void mpfs_can_stop(mpfs_can_instance_t *priv);

/* CAN message helper functions */

static uint32_t mpfs_can_canid_to_msgid(uint32_t canid);
static uint32_t mpfs_can_msgid_to_canid(uint32_t id, bool ide, bool rtr);
static uint8_t mpfs_can_set_bitrate(mpfs_can_instance_t *priv,
                                    uint32_t bitrate);
#if defined(CONFIG_DEBUG_CAN_INFO) || defined(CONFIG_NETDEV_CAN_BITRATE_IOCTL)
static uint32_t mpfs_can_get_sample_point(mpfs_can_instance_t *priv);
#endif

/* CAN message RX buffer setter/getter functions */

static uint8_t mpfs_can_config_buffer(mpfs_can_instance_t *priv);

static uint8_t mpfs_can_config_buffer_n(mpfs_can_instance_t *priv,
                                        uint8_t buffer_number,
                                        mpfs_can_rxmsgobject_t *pmsg);

static uint8_t mpfs_can_get_message(mpfs_can_instance_t *priv);

#ifdef CONFIG_DEBUG_CAN_INFO
static inline uint32_t
mpfs_can_get_rx_buffer_status(mpfs_can_instance_t *priv);
#endif

static uint32_t mpfs_can_get_rx_error_count(mpfs_can_instance_t *priv);

#ifdef CONFIG_DEBUG_CAN_INFO
static inline bool mpfs_can_get_rx_gte96(mpfs_can_instance_t *priv);
#endif

/* CAN message TX buffer setter/getter functions */

static uint8_t mpfs_can_send_message_ready(mpfs_can_instance_t *priv);
static uint8_t mpfs_can_send_message(mpfs_can_instance_t *priv);
static uint8_t mpfs_can_send_message_abort(mpfs_can_instance_t *priv);
static uint32_t mpfs_can_get_tx_buffer_status(mpfs_can_instance_t *priv);

static uint32_t mpfs_can_get_tx_error_count(mpfs_can_instance_t *priv);

#ifdef CONFIG_DEBUG_CAN_INFO
static inline bool mpfs_can_get_tx_gte96(mpfs_can_instance_t *priv);
#endif

/* Driver interface to Nuttx network callbacks */

static int mpfs_ifup(struct net_driver_s *dev);
static int mpfs_ifdown(struct net_driver_s *dev);
#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
static int mpfs_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_can_retrieve_rx_frame
 *
 * Description:
 *  Retrieve CAN 2.0B frame from RX Buffer
 *
 * Input Parameters:
 *  priv    - Pointer to the private CAN driver state structure
 *  cf      - Pointer to CAN frame structure
 *
 * Returned Value:
 *  This function returns CAN_OK on successful retrieval of CAN frame else
 *  it returns CAN_ERR
 *
 * Assumptions:
 *  Frame format word is already parsed in advance and provided as 'ffw' arg
 *
 ****************************************************************************/

static bool mpfs_can_retrieve_rx_frame(mpfs_can_instance_t *priv,
                                       struct can_frame *cf)
{
  uint8_t dlc;
  bool ide;
  bool rtr;
  mpfs_can_msgobject_t *pmsg = priv->rx_msg;

  /* CAN ID & EFF & RTR Flags */

  ide = (bool)(pmsg->msg_ctrl & MPFS_CAN_RX_MSG_CTRL_CMD_IDE);
  rtr = (bool)(pmsg->msg_ctrl & MPFS_CAN_RX_MSG_CTRL_CMD_RTR);
  cf->can_id = mpfs_can_msgid_to_canid(pmsg->id, ide, rtr);

  /* DLC */

  dlc = (pmsg->msg_ctrl & MPFS_CAN_RX_MSG_CTRL_CMD_DLC)
         >> MPFS_CAN_RX_MSG_CTRL_CMD_DLC_SHIFT;
  if (dlc <= 8)
    {
     cf->can_dlc = dlc;
    }
  else
    {
      canerr("DLC = %d is out of range\n", dlc);
      return CAN_ERR;
    }

  /* Data (big endian) */

  *(uint32_t *)&cf->data[0] = __builtin_bswap32(pmsg->data_high);
  *(uint32_t *)&cf->data[4] = __builtin_bswap32(pmsg->data_low);

  return CAN_OK;
}

/****************************************************************************
 * Name: mpfs_receive_work
 *
 * Description:
 *  An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void mpfs_receive_work(void *arg)
{
  mpfs_can_instance_t *priv = (mpfs_can_instance_t *)arg;

  caninfo("CAN RX interrupt received\n");

  while (CAN_VALID_MSG == mpfs_can_get_message(priv))
    {
      struct can_frame *cf = (struct can_frame *)priv->rxdesc;

      /* Retrieve the CAN 2.0B frame */

      if (CAN_OK != mpfs_can_retrieve_rx_frame(priv, cf))
        {
          /* Didn't receive full frame or message got filtered out */

          continue;
        }

      /* Lock the network; we have to protect the dev.d_len, dev.d_buf
       * and dev.d_iob from the devif_poll path
       */

      net_lock();

      /* Copy the buffer pointer to priv->dev.d_buf  Set amount of data
       * in priv->dev.d_len
       */

      priv->dev.d_len = sizeof(struct can_frame);
      priv->dev.d_buf = (uint8_t *)cf;

      /* Send to socket interface */

      NETDEV_RXPACKETS(&priv->dev);
      can_input(&priv->dev);

      net_unlock();

      /* Point the packet buffer back to the next Tx buffer that will be
       * used during the next write.  If the write queue is full, then
       * this will point at an active buffer, which must not be written
       * to.  This is OK because devif_poll won't be called unless the
       * queue is not full.
       */

      priv->dev.d_buf = (uint8_t *)priv->txdesc;
    }

  /* Check for RX FIFO Overflow */

  if (MPFS_CAN_INT_STATUS_OVR_LOAD & priv->isr)
    {
      /* Re-enable RX overload err int as all the RX buffers are handled */

      mpfs_can_set_int_ebl(priv, MPFS_CAN_INT_ENABLE_OVR_LOAD_INT_ENBL);
    }

  /* Re-enable RX msg receive interrupt */

  mpfs_can_set_int_ebl(priv, MPFS_CAN_INT_ENABLE_RX_MSG_INT_ENBL);
}

/****************************************************************************
 * Name: mpfs_txdone_work
 *
 * Description:
 *  An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  Global interrupts are disabled by the watchdog logic.
 *  We are not in an interrupt context so that we can lock the network.
 *
 ****************************************************************************/

static void mpfs_txdone_work(void *arg)
{
  mpfs_can_instance_t *priv = (mpfs_can_instance_t *)arg;

  caninfo("TX done interrupt received\n");

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  net_lock();
  devif_poll(&priv->dev, mpfs_txpoll);
  net_unlock();
}

/****************************************************************************
 * Name: mpfs_transmit
 *
 * Description:
 *  Start hardware transmission.  Called either from the txdone interrupt
 *  handling or from watchdog based polling.
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  Zero (CAN_OK) on success; a negated errno on failure
 *
 * Assumptions:
 *  May or may not be called from an interrupt handler.  In either case,
 *  global interrupts are disabled, either explicitly or indirectly through
 *  interrupt handling logic.
 *
 ****************************************************************************/

static int mpfs_transmit(mpfs_can_instance_t *priv)
{
  uint8_t ret;

  /* Retrieve the CAN 2.0B frame from network device buffer */

  struct can_frame *cf = (struct can_frame *)priv->dev.d_buf;

  /* Fill the CAN msg object to be sent */

  mpfs_can_msgobject_t *pmsg = priv->tx_msg;

  /* CAN TX msg control command */

  pmsg->msg_ctrl = ((cf->can_id & CAN_EFF_FLAG) ?
                   MPFS_CAN_TX_MSG_CTRL_CMD_IDE : 0)
                   | ((cf->can_dlc << MPFS_CAN_TX_MSG_CTRL_CMD_DLC_SHIFT)
                   & MPFS_CAN_TX_MSG_CTRL_CMD_DLC)
                   | ((cf->can_id & CAN_RTR_FLAG) ?
                   MPFS_CAN_TX_MSG_CTRL_CMD_RTR : 0);

  /* CAN ID */

  pmsg->id = mpfs_can_canid_to_msgid(cf->can_id);

  /* CAN data */

  if (!(pmsg->msg_ctrl & MPFS_CAN_TX_MSG_CTRL_CMD_RTR))
    {
      pmsg->data_high = __builtin_bswap32(*(uint32_t *)&cf->data[0]);
      pmsg->data_low = __builtin_bswap32(*(uint32_t *)&cf->data[4]);
    }

  /* Insert CAN msg object into available TX bf */

  if (CAN_VALID_MSG != (ret = mpfs_can_send_message(priv)))
    {
      canerr("Failed to send CAN frame due to %s\n",
             ret == CAN_INVALID_BUFFER ? "invalid buffer" :
             ret == CAN_NO_MSG ? "no TX buffer available" : "unknown err");
      return CAN_ERR;
    }

  /* Increment statistics */

  priv->stats.txb_sent++;
  NETDEV_TXPACKETS(&priv->dev);

  caninfo("Single CAN message transmit done\n");

  return CAN_OK;
}

/****************************************************************************
 * Name: mpfs_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. During normal TX polling
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   Zero (CAN_OK) on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int mpfs_txpoll(struct net_driver_s *dev)
{
  mpfs_can_instance_t *priv =
    (mpfs_can_instance_t *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Send the packet. If failure, return a non-zero value to terminate
       * the poll.
       */

      if (CAN_OK != mpfs_transmit(priv))
        {
          return -EBUSY;
        }

      /* Check if there is room in the device to hold another packet. If
       * not, return a non-zero value to terminate the poll.
       */

      if (CAN_OK != mpfs_can_send_message_ready(priv))
        {
          return -EBUSY;
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return CAN_OK;
}

/****************************************************************************
 * Name: mpfs_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void mpfs_txavail_work(void *arg)
{
  mpfs_can_instance_t *priv = (mpfs_can_instance_t *)arg;

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->bifup)
    {
      /* Check if there is room in the controller to hold another outgoing
       * packet.
       */

      if (CAN_OK == mpfs_can_send_message_ready(priv))
        {
          /* Yes, there is, poll the network for new TXT transmit */

          devif_poll(&priv->dev, mpfs_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Name: mpfs_txavail
 *
 * Description:
 *  Driver callback invoked when new TX data is available.  This is a
 *  stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *  latency.
 *
 * Input Parameters:
 *  dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *  0 - CAN_OK
 *
 * Assumptions:
 *  Called in normal user mode
 *
 ****************************************************************************/

static int mpfs_txavail(struct net_driver_s *dev)
{
  mpfs_can_instance_t *priv =
    (mpfs_can_instance_t *)dev->d_private;

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      mpfs_txavail_work(priv);
    }

  return CAN_OK;
}

/****************************************************************************
 * Name: mpfs_err_interrupt
 *
 * Description:
 *    Error frame ISR
 *
 * Input Parameters:
 *    priv  - Pointer to the private CAN driver state structure
 *    isr   - Interrupt status register value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_err_interrupt(mpfs_can_instance_t *priv, uint32_t isr)
{
  uint8_t state;

  if (MPFS_CAN_INT_STATUS_ARB_LOSS & isr)
    {
      canwarn("Arbitration loss error interrupt\n");
      priv->stats.arbitration_loss++;
    }

  if (MPFS_CAN_INT_STATUS_OVR_LOAD & isr)
    {
      canwarn("RX overload error interrupt\n");
      priv->stats.rx_overload++;

      /* Mask the interrupt until it is handled in worker */

      mpfs_can_clear_int_ebl(priv, MPFS_CAN_INT_ENABLE_OVR_LOAD_INT_ENBL);

      /* Notify to socket interface */

      NETDEV_RXERRORS(&priv->dev);
    }

  if (MPFS_CAN_INT_STATUS_BIT_ERR & isr)
    {
      canwarn("Bit error interrupt\n");
      priv->stats.bit_errors++;
    }

  if (MPFS_CAN_INT_STATUS_STUFF_ERR & isr)
    {
      canwarn("Stuffing error interrupt\n");
      priv->stats.stuff_errors++;
    }

  if (MPFS_CAN_INT_STATUS_ACK_ERR & isr)
    {
      canwarn("Ack error interrupt\n");
      priv->stats.ack_errors++;
    }

  if (MPFS_CAN_INT_STATUS_FORM_ERR & isr)
    {
      canwarn("Form error interrupt\n");
      priv->stats.form_errors++;
    }

  if (MPFS_CAN_INT_STATUS_CRC_ERR & isr)
    {
      canwarn("CRC error interrupt\n");
      priv->stats.crc_errors++;
    }

  if (MPFS_CAN_INT_STATUS_STUCK_AT_0 & isr)
    {
      canwarn("Stuck at 0 error interrupt\n");
      priv->stats.stuck_at_0++;
    }

  /* Set error status */

  state = mpfs_can_get_error_status(priv);
  priv->tx_err_count = mpfs_can_get_tx_error_count(priv);
  priv->rx_err_count = mpfs_can_get_rx_error_count(priv);

  /* Check for state change */

  if (priv->error_status == state)
    {
      canwarn("No state change! Missed interrupt?\n");
    }

  priv->error_status = state;

  if (state == 0)
    {
      caninfo("Change to ERROR_ACTIVE error state\n");
      return;
    }
  else if (state == 1)
    {
      priv->stats.error_passive++;
      canwarn("Change to ERROR_PASSIVE error state\n");
    }
  else if (state > 1)
    {
      priv->stats.bus_off++;
      canwarn("Change to BUS_OFF error state\n");
    }
  else
    {
      canwarn("Unhandled error state %d\n", state);
      return;
    }

  /* Handle bus-off and error passive state */

  canwarn("Bus-off and Error passive handling: reset CAN controller..\n");

  /* Bring down the CAN interface */

  mpfs_ifdown(&priv->dev);

  /* Reinitialize CAN controller */

  mpfs_can_send_message_abort(priv);
  mpfs_can_reset(priv);

  /* Initialize the CAN bitrate */

  if (CAN_OK != mpfs_can_set_bitrate(priv, priv->bitrate_value))
    {
      canerr("Failed to set bitrate\n");
    }

  /* Configure CAN modes */

  mpfs_can_set_mode(priv, CANOP_MODE_NORMAL);

  /* Initialize the rx buffer again */

  if (CAN_OK != mpfs_can_config_buffer(priv))
    {
      canerr("CAN RX buffer re-initialization failed\n");
    }

  /* Bring up the CAN interface again */

  mpfs_ifup(&priv->dev);

  /* Notify to socket interface. */

  NETDEV_ERRORS(&priv->dev);
}

/****************************************************************************
 * Name: mpfs_interrupt
 *
 * Description:
 *   Three interrupt sources will vector to this function:
 *   1. CAN frame transmit interrupt
 *   2. CAN frame receive interrupt
 *   3. Error interrupt
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   CAN_OK on success
 *
 ****************************************************************************/

static int mpfs_interrupt(int irq, void *context, void *arg)
{
  mpfs_can_instance_t *priv = (mpfs_can_instance_t *)arg;

  /* Get the interrupt status */

  priv->isr = mpfs_can_get_int_status(priv);

  /* RX available interrupt */

  if (priv->isr & MPFS_CAN_INT_STATUS_RX_MSG)
    {
      /* Mask INT_RX_MSG until received message is handled be the worker */

      mpfs_can_clear_int_ebl(priv, MPFS_CAN_INT_ENABLE_RX_MSG_INT_ENBL);

      work_queue(CANWORK, &priv->rxwork, mpfs_receive_work, priv, 0);
    }

  /* TX done interrupt */

  if (priv->isr & MPFS_CAN_INT_STATUS_TX_MSG)
    {
      /* Schedule work to poll for next available tx frame from the network */

      work_queue(CANWORK, &priv->txdwork, mpfs_txdone_work, priv, 0);
    }

  /* Error interrupts */

  if ((priv->isr & (MPFS_CAN_INT_STATUS_ARB_LOSS
                  | MPFS_CAN_INT_STATUS_OVR_LOAD
                  | MPFS_CAN_INT_STATUS_BIT_ERR
                  | MPFS_CAN_INT_STATUS_STUFF_ERR
                  | MPFS_CAN_INT_STATUS_ACK_ERR
                  | MPFS_CAN_INT_STATUS_FORM_ERR
                  | MPFS_CAN_INT_STATUS_CRC_ERR
                  | MPFS_CAN_INT_STATUS_BUS_OFF
                  | MPFS_CAN_INT_STATUS_STUCK_AT_0)) != 0)
    {
      canerr("Some error interrupts...");

      mpfs_err_interrupt(priv, priv->isr);
    }

  /* All interrupts are now handled, clear them */

  mpfs_can_clear_int_status(priv, priv->isr);

  caninfo("Interrupt received\n");
#ifdef CONFIG_DEBUG_CAN_INFO
  mpfs_can_print_status(priv);
#endif

  return CAN_OK;
}

/****************************************************************************
 * Name: mpfs_can_add_filter
 *
 * Description:
 *  Add new HW filter to CAN Controller. Currently only support ID filter
 *
 * Input Parameters:
 *  priv          - Pointer to the private CAN driver state structure
 *  filter_type   - The type of the filter: mask filter or range filter
 *  filter_id1    - filter id 1 (can be filter value for mask filter or range
 *                  low for range filter)
 *  filter_id2    - filter id 2 (can be filter mask for mask filter or range
 *                  high for range filter)
 *
 * Returned Value:
 *  This function returns CAN_OK on successful bitrate set, otherwise it will
 *  returns CAN_ERR
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
static uint8_t mpfs_can_add_filter(mpfs_can_instance_t *priv,
                                      uint8_t filter_type,
                                      uint32_t filter_id1,
                                      uint32_t filter_id2)
{
  uint8_t ret;

  if (filter_type == CAN_FILTER_MASK)
    {
      if (priv->filter.use_mask_filter)
        {
          canwarn("Mask filter is already in use. Overwrite now\n");
        }

      if (filter_id2 == CAN_SFF_MASK)
        {
          priv->filter.acr = (filter_id1 << MPFS_CAN_MSG_ID_SHIFT
                              << MPFS_CAN_MSG_IDE_SHIFT);
          priv->filter.amr = ~(filter_id2 << MPFS_CAN_MSG_ID_SHIFT
                               << MPFS_CAN_MSG_IDE_SHIFT);
        }
      else if (filter_id2 == CAN_EFF_MASK)
        {
          priv->filter.acr = (filter_id1 << MPFS_CAN_MSG_ID_SHIFT) | 0x04;
          priv->filter.amr = ~(filter_id2 << MPFS_CAN_MSG_ID_SHIFT) | 0x04;
        }
      else
        {
          canerr("Invalid filter mask\n");
          return CAN_ERR;
        }

      /* Configure RX buffer */

      if (CAN_OK != (ret = mpfs_can_config_buffer(priv)))
        {
          canerr("Failed to configure RX buffer:%d\n", ret);
          return CAN_ERR;
        }

      priv->filter.use_mask_filter = true;
    }
  else if (filter_type == CAN_FILTER_RANGE)
    {
      canerr("Range filter type not supported\n");
      return CAN_ERR;
    }
  else
    {
      canerr("Invalid filter type\n");
      return CAN_ERR;
    }

  return CAN_OK;
}
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

/****************************************************************************
 * Name: mpfs_can_reset_filter
 *
 * Description:
 *  Reset both sw and hw filters to default settings
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns CAN_OK on successful bitrate set, otherwise it will
 *  returns CAN_ERR
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_reset_filter(mpfs_can_instance_t *priv)
{
  uint8_t ret;

  priv->filter.use_mask_filter = false;

  priv->filter.amr = 0xffffffff;  /* Mask bit == 1 => bits are not checked */
  priv->filter.acr = 0x00000000;  /* Code bit == 0 => no code to check */
  priv->filter.amr_data = 0xffff; /* Mask bit == 1 => bits are not checked */
  priv->filter.acr_data = 0x0000; /* Code bit == 0 => no code to check */

  /* Reset hw filter and configure RX buffer */

  if (CAN_OK != (ret = mpfs_can_config_buffer(priv)))
    {
      canerr("Failed to configure RX buffer:%d\n", ret);
      return CAN_ERR;
    }

  return CAN_OK;
}

/****************************************************************************
 * Name: The mpfs_can_reset
 *
 * Description:
 *  The mpfs_can_reset() function  sets the configuration register and
 *  starts the CAN controller for normal mode operation. This function is
 *  used when one needs to change the configuration settings while the CAN
 *  controller was already initialized using mpfs_can_init() function  and
 *  is running. mpfs_can_reset() function should not be used when the CAN
 *  controller wasn't initialized yet
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_reset(mpfs_can_instance_t *priv)
{
  if (priv->reg_base == CAN0_BASE)
    {
      /* Reset CAN controller 0 */

      caninfo("Resetting CAN controller 0\n");

      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET,
                  0, SYSREG_SUBBLK_CLOCK_CR_CAN0);
      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET, 0,
                  SYSREG_SOFT_RESET_CR_CAN0);
      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
                  SYSREG_SOFT_RESET_CR_CAN0, 0);
    }
  else if (priv->reg_base == CAN1_BASE)
    {
      /* Reset CAN controller 1 */

      caninfo("Resetting CAN controller 1\n");

      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET,
                  0, SYSREG_SUBBLK_CLOCK_CR_CAN1);
      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET, 0,
                  SYSREG_SOFT_RESET_CR_CAN1);
      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
                  SYSREG_SOFT_RESET_CR_CAN1, 0);
    }

  priv->stats.restarts++;
}

/****************************************************************************
 * Name: The mpfs_can_set_mode
 *
 * Description:
 *  The mpfs_can_set_mode() function sets the CAN controller operating mode
 *  based on the mode parameter. After this operation CAN controller is not
 *  in operational, to do that invoke mpfs_can_start() function
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  mode  - The mode parameter tells about desired operating mode of CAN
 *          controller. Possible operating modes are as mentioned below:
 *    |  Mode                    | Description                             |
 *    |--------------------------|-----------------------------------------|
 *    | CANOP_MODE_NORMAL        | Sets normal operating mode              |
 *    | CANOP_MODE_LISTEN_ONLY   | In listen-only mode, the CAN controller |
 *    |                          | does not send any messages. Normally    |
 *    |                          | used for automatic bitrate detection    |
 *    | CANOP_MODE_INT_LOOPBACK  | Selects internal loopback mode. This is |
 *    |                          | used for self-test                      |
 *    | CANOP_MODE_EXT_LOOPBACK  | Selects external loopback. The CAN      |
 *    |                          | controller will receive a copy of each  |
 *    |                          | message sent.                           |
 *    | CANOP_SRAM_TEST_MODE     | Sets SRAM test mode                     |
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_set_mode(mpfs_can_instance_t *priv,
                       mpfs_can_mode_t mode)
{
  uint32_t reg;

  /* Disable CAN Device */

  reg = getreg32(priv->reg_base + MPFS_CAN_CAN_COMMAND_OFFSET);
  putreg32(reg & ~MPFS_CAN_CAN_COMMAND_RUN_STOP_MODE,
    priv->reg_base + MPFS_CAN_CAN_COMMAND_OFFSET);

  /* Set the mode */

  putreg32((uint32_t)mode, priv->reg_base + MPFS_CAN_CAN_COMMAND_OFFSET);
}

#ifdef CONFIG_DEBUG_CAN_INFO
/****************************************************************************
 * Name: The mpfs_can_get_can_command_reg
 *
 * Description:
 *  The mpfs_can_get_can_command_reg() function returns the status of can
 *  command flags
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns can command flag status
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static inline uint32_t
mpfs_can_get_can_command_reg(mpfs_can_instance_t *priv)
{
  return getreg32(priv->reg_base + MPFS_CAN_CAN_COMMAND_OFFSET);
}

/****************************************************************************
 * Name: The mpfs_can_get_can_config_reg
 *
 * Description:
 *  The mpfs_can_get_can_config_reg() function returns the status of can
 *  config flags
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns can config flag status
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static inline uint32_t mpfs_can_get_can_config_reg(mpfs_can_instance_t *priv)
{
  return getreg32(priv->reg_base + MPFS_CAN_CAN_CONFIG_OFFSET);
}
#endif

/****************************************************************************
 * Name: The mpfs_can_set_int_ebl
 *
 * Description:
 *  The mpfs_can_set_int_ebl() function enable specific interrupt based on
 *  irq_flag parameter
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  irq_flag  - The irq_flag parameter is a 4 byte variable indicates
 *    Interrupt type. Possible values are:
 *  |  Constant            |  Description                                   |
 *  |----------------------|------------------------------------------------|
 *  | CAN_INT_GLOBAL       | Indicates to enable global interrupts          |
 *  | CAN_INT_ARB_LOSS     | Indicates arbitration loss interrupt           |
 *  | CAN_INT_OVR_LOAD     | Indicates overload message detected interrupt  |
 *  | CAN_INT_BIT_ERR      | Indicates bit error interrupt                  |
 *  | CAN_INT_STUFF_ERR    | Indicates bit stuffing error interrupt         |
 *  | CAN_INT_ACK_ERR      | Indicates acknowledge error interrupt          |
 *  | CAN_INT_FORM_ERR     | Indicates format error interrupt               |
 *  | CAN_INT_CRC_ERR      | Indicates CRC error interrupt                  |
 *  | CAN_INT_BUS_OFF      | Indicates bus off interrupt                    |
 *  | CAN_INT_RX_MSG_LOSS  | Indicates received message loss interrupt      |
 *  | CAN_INT_TX_MSG       | Indicates message transmit interrupt           |
 *  | CAN_INT_RX_MSG       | Indicates receive message available interrupt  |
 *  | CAN_INT_RTR_MSG      | Indicates RTR auto-reply message sent interrupt|
 *  | CAN_INT_STUCK_AT_0   | Indicates stuck at dominant error interrupt    |
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_set_int_ebl(mpfs_can_instance_t *priv,
                         uint32_t flag)
{
  uint32_t reg = mpfs_can_get_int_ebl(priv);
  putreg32(reg | flag, priv->reg_base + MPFS_CAN_INT_ENABLE_OFFSET);
}

/****************************************************************************
 * Name: The mpfs_can_clear_int_ebl
 *
 * Description:
 *  The mpfs_can_clear_int_ebl() function disable specific interrupt based on
 *  irq_flag parameter
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  irq_flag  - The irq_flag parameter is a 4 byte variable indicates
 *    Interrupt type. Possible values are:
 *  |  Constant            |  Description                                   |
 *  |----------------------|------------------------------------------------|
 *  | CAN_INT_INT_ENBL     | Indicates to enable global interrupts          |
 *  | CAN_INT_ARB_LOSS     | Indicates arbitration loss interrupt           |
 *  | CAN_INT_OVR_LOAD     | Indicates overload message detected interrupt  |
 *  | CAN_INT_BIT_ERR      | Indicates bit error interrupt                  |
 *  | CAN_INT_STUFF_ERR    | Indicates bit stuffing error interrupt         |
 *  | CAN_INT_ACK_ERR      | Indicates acknowledge error interrupt          |
 *  | CAN_INT_FORM_ERR     | Indicates format error interrupt               |
 *  | CAN_INT_CRC_ERR      | Indicates CRC error interrupt                  |
 *  | CAN_INT_BUS_OFF      | Indicates bus off interrupt                    |
 *  | CAN_INT_RX_MSG_LOSS  | Indicates received message loss interrupt      |
 *  | CAN_INT_TX_MSG       | Indicates message transmit interrupt           |
 *  | CAN_INT_RX_MSG       | Indicates receive message available interrupt  |
 *  | CAN_INT_RTR_MSG      | Indicates RTR auto-reply message sent interrupt|
 *  | CAN_INT_STUCK_AT_0   | Indicates stuck at dominant error interrupt    |
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_clear_int_ebl(mpfs_can_instance_t *priv,
                                   uint32_t flag)
{
  uint32_t reg = mpfs_can_get_int_ebl(priv);
  putreg32(reg & ~flag, priv->reg_base + MPFS_CAN_INT_ENABLE_OFFSET);
}

/****************************************************************************
 * Name: The mpfs_can_get_int_ebl
 *
 * Description:
 *  The mpfs_can_get_int_ebl() function returns the status of interrupt
 *  enable flags
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns interrupt enable flag status
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_get_int_ebl(mpfs_can_instance_t *priv)
{
  return getreg32(priv->reg_base + MPFS_CAN_INT_ENABLE_OFFSET);
}

/****************************************************************************
 * Name: mpfs_can_clear_int_status
 *
 * Description:
 *  The mpfs_can_clear_int_status() function  clears the selected interrupt
 *  flags
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  irq_flag  - The irq_flag parameter is a 4 byte variable indicates
 *  Interrupt type. Possible values are:
 *  |  Constants           |  Description                                   |
 *  |----------------------|------------------------------------------------|
 *  | CAN_INT_INT_ENBL     | Indicates to enable global interrupts          |
 *  | CAN_INT_ARB_LOSS     | Indicates arbitration loss interrupt           |
 *  | CAN_INT_OVR_LOAD     | Indicates overload message detected interrupt  |
 *  | CAN_INT_BIT_ERR      | Indicates bit error interrupt                  |
 *  | CAN_INT_STUFF_ERR    | Indicates bit stuffing error interrupt         |
 *  | CAN_INT_ACK_ERR      | Indicates acknowledge error interrupt          |
 *  | CAN_INT_FORM_ERR     | Indicates format error interrupt               |
 *  | CAN_INT_CRC_ERR      | Indicates CRC error interrupt                  |
 *  | CAN_INT_BUS_OFF      | Indicates bus off interrupt                    |
 *  | CAN_INT_RX_MSG_LOSS  | Indicates received message loss interrupt      |
 *  | CAN_INT_TX_MSG       | Indicates message transmit interrupt           |
 *  | CAN_INT_RX_MSG       | Indicates receive message available interrupt  |
 *  | CAN_INT_RTR_MSG      | Indicates RTR auto-reply message sent interrupt|
 *  | CAN_INT_STUCK_AT_0   | Indicates stuck at dominant error interrupt    |
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_clear_int_status(mpfs_can_instance_t *priv,
                                      uint32_t flag)
{
  putreg32(flag, priv->reg_base + MPFS_CAN_INT_STATUS_OFFSET);
}

/****************************************************************************
 * Name: mpfs_can_get_int_status
 *
 * Description:
 *  The mpfs_can_get_int_status() function returns the status of interrupts
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns status of existed interrupts
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_get_int_status(mpfs_can_instance_t *priv)
{
  return getreg32(priv->reg_base + MPFS_CAN_INT_STATUS_OFFSET);
}

/****************************************************************************
 * Name: mpfs_can_get_error_status
 *
 * Description:
 *  The mpfs_can_get_error_status() function returns the present error state
 *  of the CAN controller. Error state might be error active or error passive
 *  or bus-off
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  | Codes  |  Descriptions                 |
 *  |--------|-------------------------------|
 *  |  0     | error active                  |
 *  |  1     | error passive                 |
 *  |  2     | bus-off                       |
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_get_error_status(mpfs_can_instance_t *priv)
{
  uint32_t reg = getreg32(priv->reg_base + MPFS_CAN_ERROR_STATUS_OFFSET);
  reg = ((reg & MPFS_CAN_ERROR_STATUS_ERROR_STATE)
        >> MPFS_CAN_ERROR_STATUS_ERROR_STATE_SHIFT);
  return (uint8_t)reg;
}

#ifdef CONFIG_DEBUG_CAN_INFO
/****************************************************************************
 * Name: mpfs_can_print_status
 *
 * Description:
 *  The mpfs_can_print_status() function prints the CAN controller status
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_print_status(mpfs_can_instance_t *priv)
{
  caninfo("========================================================\n");
  caninfo(">> CAN Settings:\n");
  caninfo("  Bitrate: %u\n", priv->bitrate_value);
  caninfo("  Sample point: %u\n", mpfs_can_get_sample_point(priv));
  print_uint32_t("  Cfg: ", mpfs_can_get_can_config_reg(priv));
  print_uint32_t("  Cmd: ", mpfs_can_get_can_command_reg(priv));
  print_uint32_t("  Interrupt Enabled: ", mpfs_can_get_int_ebl(priv));
  caninfo(">> CAN TX/RX buffer, Interrupt status:\n");
  print_uint32_t("  TX Buffer Status: ",
                 mpfs_can_get_tx_buffer_status(priv));
  print_uint32_t("  RX Buffer Status: ",
                 mpfs_can_get_rx_buffer_status(priv));
  print_uint32_t("  Interrupt status: ", priv->isr);
  caninfo(">> CAN RX/TX Error Status:\n");
  caninfo("  Error State: %u\n", mpfs_can_get_error_status(priv));
  caninfo("  TX Error Count: %d\n", priv->tx_err_count);
  caninfo("  RX Error Count: %d\n", priv->rx_err_count);
  caninfo("  TX GTE 96: %s\n", mpfs_can_get_tx_gte96(priv) ? "Yes" : "No");
  caninfo("  RX GTE 96: %s\n", mpfs_can_get_rx_gte96(priv) ? "Yes" : "No");
  caninfo(">> CAN Stats:\n");
  caninfo("  Error Passive: %u\n", priv->stats.error_passive);
  caninfo("  Bus Off: %u\n", priv->stats.bus_off);
  caninfo("  Arbitration Loss: %u\n", priv->stats.arbitration_loss);
  caninfo("  Rx Overload: %u\n", priv->stats.rx_overload);
  caninfo("  Bit Errors: %u\n", priv->stats.bit_errors);
  caninfo("  Stuff Errors: %u\n", priv->stats.stuff_errors);
  caninfo("  Ack Errors: %u\n", priv->stats.ack_errors);
  caninfo("  Form Errors: %u\n", priv->stats.form_errors);
  caninfo("  CRC Errors: %u\n", priv->stats.crc_errors);
  caninfo("  Stuck at 0: %u\n", priv->stats.stuck_at_0);
  caninfo("  Restarts: %u\n", priv->stats.restarts);
  caninfo("  TX message sent: %u\n", priv->stats.txb_sent);
  caninfo("========================================================\n");
}
#endif

/****************************************************************************
 * Name: mpfs_can_canid_to_msgid
 *
 * Description:
 *  The mpfs_can_canid_to_msgid() function returns ID bits left justified
 *  based on IDE type. IDE type might be either standard or extended
 *
 * Input Parameters:
 *  pmsg  - The pmsg parameter is a pointer to the message object
 *
 * Returned Value:
 *  This function returns message identifier
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_canid_to_msgid(uint32_t canid)
{
  if (canid & CAN_EFF_FLAG)
    {
      return (canid & CAN_EFF_MASK);
    }
  else
    {
      return ((canid & CAN_SFF_MASK) << MPFS_CAN_MSG_IDE_SHIFT);
    }
}

/****************************************************************************
 * Name: mpfs_can_msgid_to_canid
 *
 * Description:
 *  The mpfs_can_msgid_to_canid() function  packs the ID, IDE, and RTR
 *  bits together as they are used in the message filter mask and returns
 *  packed identifier
 *
 * Input Parameters:
 *  id  - The id parameter is a 4 byte variable to hold message identifier
 *  ide  - The ide parameter is a boolean variable to indicate IDE type.
 *         Acceptable values are as mentioned below:
 *  |  Value     |   Description                 |
 *  |------------|-------------------------------|
 *  |  0         | Standard format               |
 *  |  1         | Extended format               |
 *
 *  rtr  - The rtr parameter is a boolean variable to indicate message type.
 *         Acceptable values are as mentioned below:
 *
 *  |  Value     |   Description                 |
 *  |------------|-------------------------------|
 *  |   0        | Regular message (data frame)  |
 *  |   1        | RTR message (remote frame)    |
 *
 * Returned Value:
 *  This function returns packed id
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_msgid_to_canid(uint32_t id,
                                        bool ide,
                                        bool rtr)
{
  id = (ide ? id : (id >> MPFS_CAN_MSG_IDE_SHIFT))
    | ((uint32_t)ide << 31) | ((uint32_t)rtr << 30);

  return id;
}

/****************************************************************************
 * Name: mpfs_can_set_bitrate
 *
 * Description:
 *  The mpfs_can_set_bitrate() function returns the current set bitrate
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  bitrate  - The bitrate value in bit/s
 *
 * Returned Value:
 *  This function returns CAN_OK on successful bitrate set, otherwise it will
 *  returns CAN_ERR
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_set_bitrate(mpfs_can_instance_t *priv,
                                    uint32_t bitrate)
{
  uint32_t bitrate_constant;
  switch (bitrate)
    {
      case 5000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_32M_5K;
        break;
      case 10000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_32M_10K;
        break;
      case 20000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_32M_20K;
        break;
      case 50000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_32M_50K;
        break;
      case 100000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_32M_100K;
        break;
      case 125000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_32M_125K;
        break;
      case 250000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_32M_250K;
        break;
      case 500000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_32M_500K;
        break;
      case 1000000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_32M_1M;
        break;
      default:
        canerr("Invalid bitrate %u\n", bitrate);
        return CAN_ERR;
    }

  bitrate_constant |= MPFS_CAN_CAN_CONFIG_AUTO_RESTART;
  putreg32(bitrate_constant , priv->reg_base + MPFS_CAN_CAN_CONFIG_OFFSET);

  return CAN_OK;
}

#if defined(CONFIG_DEBUG_CAN_INFO) || defined(CONFIG_NETDEV_CAN_BITRATE_IOCTL)
/****************************************************************************
 * Name: mpfs_can_get_sample_point
 *
 * Description:
 *  The mpfs_can_get_sample_point() function returns the current set sample
 *  point. The sample point % can be calculated as follows:
 *  tseg1 = (CFG_TSEG1 + 1) tq = (prop_seg + phase_seg1) tq
 *  tseg2 = (CFG_TSEG2 + 1) tq
 *  sync_seg = 1 tq
 *  sp = [sync_seg + tseg1]/[sync_seg + tseg1 + tseg2]
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns the current set sample point %
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_get_sample_point(mpfs_can_instance_t *priv)
{
  uint32_t reg;
  reg = getreg32(priv->reg_base + MPFS_CAN_CAN_CONFIG_OFFSET);
  uint32_t tseg1 = ((reg & MPFS_CAN_CAN_CONFIG_CFG_TSEG1) >>
                   MPFS_CAN_CAN_CONFIG_CFG_TSEG1_SHIFT) + 1;
  uint32_t tseg2 = ((reg & MPFS_CAN_CAN_CONFIG_CFG_TSEG2) >>
                   MPFS_CAN_CAN_CONFIG_CFG_TSEG2_SHIFT) + 1;
  const uint32_t sync_seg = 1;
  return (sync_seg + tseg1) * 100 / (sync_seg + tseg1 + tseg2);
}
#endif

/****************************************************************************
 * Name: mpfs_can_config_buffer
 *
 * Description:
 *  The mpfs_can_config_buffer() function configures receive buffers
 *  initialized for Basic CAN operation
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns CAN_OK on successful execution, otherwise it will
 *  returns following error codes:
 *
 *  |  Constants            |  Description                                |
 *  |-----------------------|---------------------------------------------|
 *  | CAN_NO_MSG            | Indicates that there is no message received |
 *  | CAN_INVALID_BUFFER    | Indicates invalid buffer number            |
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_config_buffer(mpfs_can_instance_t *priv)
{
  uint8_t success = CAN_NO_MSG;
  uint8_t buffer_number;
  mpfs_can_rxmsgobject_t canrxobj;

  /* Is a buffer configured for Basic CAN? */

  if (priv->basic_can_rxb_count == 0)
    {
      return CAN_INVALID_BUFFER;
    }

  canrxobj.rx_msg_ctrl = CAN_RX_BUFFER_CTRL_DEFAULT;
  canrxobj.id = 0;
  canrxobj.data_high = 0;
  canrxobj.data_low = 0;
  canrxobj.amr = priv->filter.amr;
  canrxobj.acr = priv->filter.acr;
  canrxobj.amr_data = priv->filter.amr_data;
  canrxobj.acr_data = priv->filter.acr_data;

  /* Find next BASIC CAN buffer that has a message available */

  for (buffer_number = CAN_RX_BUFFER - priv->basic_can_rxb_count;
       buffer_number < CAN_RX_BUFFER; buffer_number++)
    {
      /* Configure buffer */

      if (buffer_number < (CAN_RX_BUFFER - 1))
        {
          /* set link flag, if not last buffer */

          canrxobj.rx_msg_ctrl |= MPFS_CAN_RX_MSG_CTRL_CMD_LF;
          mpfs_can_config_buffer_n(priv, buffer_number, &canrxobj);
        }
      else
        {
          /* clear link flag, if last buffer */

          canrxobj.rx_msg_ctrl &= ~MPFS_CAN_RX_MSG_CTRL_CMD_LF;
          mpfs_can_config_buffer_n(priv, buffer_number, &canrxobj);
        }

        success = CAN_OK;
    }

  return success;
}

/****************************************************************************
 * Name: mpfs_can_config_buffer_n
 *
 * Description:
 *  The mpfs_can_config_buffer_n() function configures n receive buffer
 *  initialized for Basic CAN operation
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  buffer_number  - The buffer_number parameter is a 1 byte variable to
 *                 indicate buffer number
 * pmsg  - The pmsg parameter is a pointer to the message object
 *
 * Returned Value:
 *  This function returns CAN_OK on successful execution, otherwise it will
 *  returns following error codes:
 *
 *  |  Constants            |  Description                                |
 *  |-----------------------|---------------------------------------------|
 *  | CAN_NO_MSG            | Indicates that there is no message received |
 *  | CAN_INVALID_BUFFER    | Indicates invalid buffer number            |
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_config_buffer_n (mpfs_can_instance_t *priv,
                                         uint8_t buffer_number,
                                         mpfs_can_rxmsgobject_t *pmsg)
{
  uintptr_t addr;

  if (priv->basic_can_rxb_count == 0)
    {
      return CAN_INVALID_BUFFER;
    }

  /* Configure buffer */

  addr = priv->reg_base + MPFS_CAN_RX_MSG_OFFSET
    + MPFS_CAN_RX_MSG_TOTAL_SIZE * buffer_number;
  putreg32(pmsg->rx_msg_ctrl, addr + MPFS_CAN_RX_MSG_CTRL_CMD_SHIFT);
  putreg32(pmsg->id, addr + MPFS_CAN_RX_MSG_ID_SHIFT);
  putreg32(pmsg->data_high, addr + MPFS_CAN_RX_MSG_DATA_HIGH_SHIFT);
  putreg32(pmsg->data_low, addr + MPFS_CAN_RX_MSG_DATA_LOW_SHIFT);
  putreg32(pmsg->amr, addr + MPFS_CAN_RX_MSG_AMR_SHIFT);
  putreg32(pmsg->acr, addr + MPFS_CAN_RX_MSG_ACR_SHIFT);
  putreg32(pmsg->amr_data, addr + MPFS_CAN_RX_MSG_AMR_DATA_SHIFT);
  putreg32(pmsg->acr_data, addr + MPFS_CAN_RX_MSG_ACR_DATA_SHIFT);

  return CAN_OK;
}

/****************************************************************************
 * Name: mpfs_can_get_message
 *
 * Description:
 *  The mpfs_can_get_message() function read message from the first buffer
 *  set for Basic CAN  operation that contains a message. Once the message
 *  has been read from the buffer, the message receipt is acknowledged.
 *  Note: Since neither a hardware nor a software FIFO exists, message
 *    inversion
 *    might happen (example, a newer message might be read from the receive
 *    buffer prior to an older message)
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns CAN_VALID_MSG on successful execution, otherwise it
 *  will returns following error codes:
 *
 *  |  Constants            |  Description                                |
 *  |-----------------------|---------------------------------------------|
 *  | CAN_NO_MSG            | Indicates that there is no message received |
 *  | CAN_INVALID_BUFFER    | Indicates invalid buffer number             |
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_get_message(mpfs_can_instance_t *priv)
{
  uint8_t success = CAN_NO_MSG;
  uint8_t buffer_number;
  mpfs_can_msgobject_t *pmsg = priv->rx_msg;
  uintptr_t addr;
  uint32_t reg;

  /* Is a buffer configured for Basic CAN? */

  if (priv->basic_can_rxb_count == 0)
    {
      return CAN_INVALID_BUFFER;
    }

  /* Find next BASIC CAN buffer that has a message available */

  for (buffer_number = CAN_RX_BUFFER - priv->basic_can_rxb_count;
       buffer_number < CAN_RX_BUFFER; buffer_number++)
    {
      addr = priv->reg_base + MPFS_CAN_RX_MSG_OFFSET
        + MPFS_CAN_RX_MSG_TOTAL_SIZE * buffer_number;

      /* Check that if there is a valid message */

      reg = getreg32(addr + MPFS_CAN_RX_MSG_CTRL_CMD_SHIFT);

      if (reg & MPFS_CAN_RX_MSG_CTRL_CMD_MSGAV_RTRS)
        {
          /* Copy ID */

          pmsg->id = getreg32(addr + MPFS_CAN_RX_MSG_ID_SHIFT)
            >> MPFS_CAN_MSG_ID_SHIFT;

          /* Copy 4 of the data bytes */

          pmsg->data_low = getreg32(addr + MPFS_CAN_RX_MSG_DATA_LOW_SHIFT);

          /* Copy the other 4 data bytes */

          pmsg->data_high = getreg32(addr + MPFS_CAN_RX_MSG_DATA_HIGH_SHIFT);

          /* Get DLC, IDE and RTR and time stamp */

          pmsg->msg_ctrl = getreg32(addr + MPFS_CAN_RX_MSG_CTRL_CMD_SHIFT);

          /* Ack that it's been removed from the FIFO */

          putreg32(pmsg->msg_ctrl | MPFS_CAN_RX_MSG_CTRL_CMD_MSGAV_RTRS,
                  addr + MPFS_CAN_RX_MSG_CTRL_CMD_SHIFT);

          success = CAN_VALID_MSG;
          break;
        }
    }

  return success;
}

#ifdef CONFIG_DEBUG_CAN_INFO
/****************************************************************************
 * Name: mpfs_can_get_rx_buffer_status
 *
 * Description:
 *  The mpfs_can_get_rx_buffer_status() function returns the buffer status
 *  of all receive(32) buffers
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value: This function returns status of receive buffers
 *   (32 buffers)
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static inline uint32_t
mpfs_can_get_rx_buffer_status(mpfs_can_instance_t *priv)
{
  return getreg32(priv->reg_base + MPFS_CAN_RX_BUF_STATUS_OFFSET);
}
#endif

/****************************************************************************
 * Name: mpfs_can_get_rx_error_count
 *
 * Description:
 *  The mpfs_can_get_rx_error_count() function returns the current receive
 *  error counter value. Counter value ranges from 0x00 - 0xFF
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns the receive error counter value
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_get_rx_error_count(mpfs_can_instance_t *priv)
{
  uint32_t reg = getreg32(priv->reg_base + MPFS_CAN_ERROR_STATUS_OFFSET);
  reg = ((reg & MPFS_CAN_ERROR_STATUS_RX_ERR_CNT)
        >> MPFS_CAN_ERROR_STATUS_RX_ERR_CNT_SHIFT);
  return reg;
}

#ifdef CONFIG_DEBUG_CAN_INFO
/****************************************************************************
 * Name: mpfs_can_get_rx_gte96
 *
 * Description:
 *  The mpfs_can_get_rx_gte96() function provides information about receive
 *  error count. It identifies that receive error count is greater than or
 *  equal to 96, and reports 1 if count exceeds 96
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  | Value |  Description                                         |
 *  |-------|------------------------------------------------------|
 *  |  0    | if receive error count less than 96.                 |
 *  |  1    | if receive error count greater than or equals to 96. |
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static inline bool mpfs_can_get_rx_gte96(mpfs_can_instance_t *priv)
{
  uint32_t reg = getreg32(priv->reg_base + MPFS_CAN_ERROR_STATUS_OFFSET);
  reg &= MPFS_CAN_ERROR_STATUS_RXGTE96;
  return (bool)reg;
}
#endif

/****************************************************************************
 * Name: mpfs_can_send_message_ready
 *
 * Description:
 *  The mpfs_can_send_message_ready() function will identify the availability
 *  of buffer to fill with new message in basic CAN operation
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns CAN_OK on successful identification of free buffer,
 *  otherwise it will returns following error codes:
 *  |  Constants            |  Description                                |
 *  |-----------------------|---------------------------------------------|
 *  | CAN_ERR               | Indicates error condition                   |
 *  | CAN_INVALID_BUFFER    | Indicates invalid buffer number             |
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_send_message_ready(mpfs_can_instance_t *priv)
{
  uint8_t success = CAN_ERR;
  uint8_t buffer_number;
  uintptr_t addr;

  if (priv->basic_can_txb_count == 0)
    {
      return CAN_INVALID_BUFFER;
    }

  /* Find next BASIC CAN buffer that is available */

  for (buffer_number = CAN_TX_BUFFER - priv->basic_can_txb_count;
       buffer_number < CAN_TX_BUFFER; buffer_number++)
    {
      addr = priv->reg_base + MPFS_CAN_TX_MSG_OFFSET
        + MPFS_CAN_TX_MSG_TOTAL_SIZE * buffer_number;

      if ((getreg32(addr + MPFS_CAN_TX_MSG_CTRL_CMD_SHIFT) &
          MPFS_CAN_TX_MSG_CTRL_CMD_TX_REQ) == 0)
        {
          /* At least one Tx buffer isn't busy */

          success = CAN_OK;
          break;
        }
    }

  return success;
}

/****************************************************************************
 * Name: mpfs_can_send_message
 *
 * Description:
 *  The mpfs_can_send_message() function will copy the data to the first
 *  available buffer set for Basic CAN operation and send data on to the bus.
 *  Note: Since neither a hardware nor a software FIFO exists, message
 *  inversion might happen (example, a newer message might be send from the
 *  transmit buffer prior to an older message)
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns CAN_VALID_MSG on successful identification of free
 *  buffer, otherwise it will returns following error codes:
 *  |  Constants            |  Description                                |
 *  |-----------------------|---------------------------------------------|
 *  | CAN_NO_MSG            | No buffer available for tx                  |
 *  | CAN_INVALID_BUFFER    | Indicates invalid buffer number             |
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_send_message(mpfs_can_instance_t *priv)
{
  uint8_t success = CAN_NO_MSG;
  uint8_t buffer_number;
  mpfs_can_msgobject_t *pmsg = priv->tx_msg;
  uintptr_t addr;

  if (priv->basic_can_txb_count == 0)
    {
      return CAN_INVALID_BUFFER;
    }

  /* Find next BASIC CAN buffer that is available */

  for (buffer_number = CAN_TX_BUFFER - priv->basic_can_txb_count;
       buffer_number < CAN_TX_BUFFER; buffer_number++)
    {
      /* Check which transmit buffer is not busy and use it. */

      if ((mpfs_can_get_tx_buffer_status(priv) & (1 << buffer_number)) == 0)
        {
          /* If at least one Tx buffer isn't busy.... */

          addr = priv->reg_base + MPFS_CAN_TX_MSG_OFFSET
            + MPFS_CAN_TX_MSG_TOTAL_SIZE * buffer_number;

          /* CAN ID */

          putreg32(pmsg->id << MPFS_CAN_MSG_ID_SHIFT,
                  addr + MPFS_CAN_TX_MSG_ID_SHIFT);

          /* CAN data */

          putreg32(pmsg->data_low, addr + MPFS_CAN_TX_MSG_DATA_LOW_SHIFT);
          putreg32(pmsg->data_high, addr + MPFS_CAN_TX_MSG_DATA_HIGH_SHIFT);

          /* CAN TX msg control command */

          putreg32(pmsg->msg_ctrl
                  | MPFS_CAN_TX_MSG_CTRL_CMD_WPN_B
                  | MPFS_CAN_TX_MSG_CTRL_CMD_WPN_A
                  | MPFS_CAN_TX_MSG_CTRL_CMD_TX_INT_EBL
                  | MPFS_CAN_TX_MSG_CTRL_CMD_TX_REQ,
                  addr + MPFS_CAN_TX_MSG_CTRL_CMD_SHIFT);

          success = CAN_VALID_MSG;
          break;
        }
    }

  return success;
}

/****************************************************************************
 * Name: mpfs_can_send_message_abort
 *
 * Description:
 *  The mpfs_can_send_message_abort() function returns the buffer status
 *  of all transmit(32) buffers
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  buffer_number  - The buffer index to abort the message
 *
 * Returned Value: This function returns status of transmit buffers
 *   (32 buffers)
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_send_message_abort(mpfs_can_instance_t *priv)
{
  uint8_t buffer_number;
  uintptr_t addr;
  uint32_t reg;

  if (priv->basic_can_txb_count == 0)
    {
      return CAN_INVALID_BUFFER;
    }

  /* Find next BASIC CAN buffer that is available */

  for (buffer_number = CAN_TX_BUFFER - priv->basic_can_txb_count;
       buffer_number < CAN_TX_BUFFER; buffer_number++)
    {
      /* Check which transmit buffer is busy and abort it. */

      if ((mpfs_can_get_tx_buffer_status(priv) & (1 << buffer_number)) == 1)
        {
          addr = priv->reg_base + MPFS_CAN_TX_MSG_OFFSET
            + MPFS_CAN_TX_MSG_TOTAL_SIZE * buffer_number;

          /* Set abort request */

          reg = getreg32(addr + MPFS_CAN_TX_MSG_CTRL_CMD_SHIFT);
          putreg32((reg & ~MPFS_CAN_TX_MSG_CTRL_CMD_TX_REQ)
                  | MPFS_CAN_TX_MSG_CTRL_CMD_TX_ABORT,
                  addr + MPFS_CAN_TX_MSG_CTRL_CMD_SHIFT);

          /* Check if the abort request is granted */

          reg = getreg32(addr + MPFS_CAN_TX_MSG_CTRL_CMD_SHIFT);
          if ((reg & MPFS_CAN_TX_MSG_CTRL_CMD_TX_ABORT) != 0)
            {
              /* Message not aborted. */

              return CAN_ERR;
            }
        }
    }

  return CAN_OK;
}

/****************************************************************************
 * Name: mpfs_can_get_tx_buffer_status
 *
 * Description:
 *  The mpfs_can_get_tx_buffer_status() function returns the buffer status
 *  of all transmit(32) buffers
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value: This function returns status of transmit buffers
 *   (32 buffers)
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_get_tx_buffer_status(mpfs_can_instance_t *priv)
{
  return getreg32(priv->reg_base + MPFS_CAN_TX_BUF_STATUS_OFFSET);
}

/****************************************************************************
 * Name: mpfs_can_get_tx_error_count
 *
 * Description:
 *  The mpfs_can_get_tx_error_count() function returns the current transmit
 *  error counter value. Counter value ranges from 0x00 - 0xFF
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value: This function returns the transmit error counter value
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_get_tx_error_count(mpfs_can_instance_t *priv)
{
  uint32_t reg = getreg32(priv->reg_base + MPFS_CAN_ERROR_STATUS_OFFSET);
  reg = ((reg & MPFS_CAN_ERROR_STATUS_TX_ERR_CNT)
        >> MPFS_CAN_ERROR_STATUS_TX_ERR_CNT_SHIFT);
  return reg;
}

#ifdef CONFIG_DEBUG_CAN_INFO
/****************************************************************************
 * Name: mpfs_can_get_tx_gte96
 *
 * Description:
 *  The mpfs_can_get_tx_gte96() function provides information about transmit
 *  error count. It identifies that transmit error count is greater than or
 *  equals to 96, and reports 1 if count exceeds 96
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  | Value |  Description                                          |
 *  |-------|-------------------------------------------------------|
 *  |  0    | if transmit error count less than 96.                 |
 *  |  1    | if transmit error count greater than or equals to 96. |
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static inline bool mpfs_can_get_tx_gte96(mpfs_can_instance_t *priv)
{
  uint32_t reg = getreg32(priv->reg_base + MPFS_CAN_ERROR_STATUS_OFFSET);
  reg &= MPFS_CAN_ERROR_STATUS_TXGTE96;
  return (bool)reg;
}
#endif

/****************************************************************************
 * Name: mpfs_can_start
 *
 * Description:
 *  This routine starts the driver. It clears all pending interrupts and
 *  enable CAN controller to perform normal operation. It enables receive
 *  interrupts also
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_start(mpfs_can_instance_t *priv)
{
  uint32_t reg;

  /* Clear all pending interrupts */

  mpfs_can_clear_int_status(priv, 0xffffffff);

  /* Enable CAN Device */

  reg = getreg32(priv->reg_base + MPFS_CAN_CAN_COMMAND_OFFSET);
  putreg32(reg | MPFS_CAN_CAN_COMMAND_RUN_STOP_MODE,
    priv->reg_base + MPFS_CAN_CAN_COMMAND_OFFSET);

  /* Configure interrupts */

  mpfs_can_set_int_ebl(priv, MPFS_CAN_INT_ENABLE_INT_ENBL
                           | MPFS_CAN_INT_ENABLE_TX_MSG_INT_ENBL
                           | MPFS_CAN_INT_ENABLE_RX_MSG_INT_ENBL
                           | MPFS_CAN_INT_ENABLE_ARB_LOSS_INT_ENBL
                           | MPFS_CAN_INT_ENABLE_OVR_LOAD_INT_ENBL
                           | MPFS_CAN_INT_ENABLE_BIT_ERR_INT_ENBL
                           | MPFS_CAN_INT_ENABLE_STUFF_ERR_INT_ENBL
                           | MPFS_CAN_INT_ENABLE_CRC_ERR_INT_ENBL
                           | MPFS_CAN_INT_ENABLE_BUS_OFF_INT_ENBL
                           | MPFS_CAN_INT_STATUS_STUCK_AT_0);

  /* Initialize error status and counts */

  priv->error_status = mpfs_can_get_error_status(priv);
  priv->tx_err_count = mpfs_can_get_tx_error_count(priv);
  priv->rx_err_count = mpfs_can_get_rx_error_count(priv);
}

/****************************************************************************
 * Name: mpfs_can_stop
 *
 * Description:
 *  This routine stops the driver. This is the drivers stop routine. It will
 * disable the CAN controller
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_stop(mpfs_can_instance_t *priv)
{
  uint32_t reg;

  /* Clear all pending interrupts */

  mpfs_can_clear_int_status(priv, 0xffffffff);

  /* Disable CAN Device */

  reg = getreg32(priv->reg_base + MPFS_CAN_CAN_COMMAND_OFFSET);
  putreg32(reg & ~MPFS_CAN_CAN_COMMAND_RUN_STOP_MODE,
    priv->reg_base + MPFS_CAN_CAN_COMMAND_OFFSET);

  /* Disable interrupts from CAN device. */

  mpfs_can_clear_int_ebl(priv, MPFS_CAN_INT_ENABLE_INT_ENBL);
}

/****************************************************************************
 * Name: mpfs_ifup
 *
 * Description:
 *  NuttX Callback: Start the CAN interface
 *
 * Input Parameters:
 *  dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *  Zero (CAN_OK) on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_ifup(struct net_driver_s *dev)
{
  mpfs_can_instance_t *priv =
    (mpfs_can_instance_t *)dev->d_private;

  mpfs_can_start(priv);

  priv->bifup = true;

  priv->dev.d_buf = (uint8_t *)priv->txdesc;

  /* Enable interrupts */

  up_enable_irq(priv->irqn);

  return CAN_OK;
}

/****************************************************************************
 * Name: mpfs_ifdown
 *
 * Description:
 *  NuttX Callback: Stop the CAN interface
 *
 * Input Parameters:
 *  dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *  Zero (CAN_OK) on success; a negated errno value on failure
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int mpfs_ifdown(struct net_driver_s *dev)
{
  mpfs_can_instance_t *priv =
    (mpfs_can_instance_t *)dev->d_private;

  mpfs_can_stop(priv);

  priv->bifup = false;
  return CAN_OK;
}

/****************************************************************************
 * Name: mpfs_ioctl
 *
 * Description:
 *  PHY ioctl command handler
 *
 * Input Parameters:
 *  dev  - Reference to the NuttX driver state structure
 *  cmd  - ioctl command
 *  arg  - Argument accompanying the command
 *
 * Returned Value:
 *  Zero (CAN_OK) on success; a negated errno value on failure
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int mpfs_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
#ifdef CONFIG_DEBUG_CAN_INFO
  caninfo("IOCTL received | cmd: %d arg: %ld\n", cmd, arg);
#endif

#if defined(CONFIG_NETDEV_CAN_BITRATE_IOCTL) || \
defined(CONFIG_NETDEV_CAN_FILTER_IOCTL)
  mpfs_can_instance_t *priv =
    (mpfs_can_instance_t *)dev->d_private;
#endif
  int ret;

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
    case SIOCGCANBITRATE:

      /* Get bitrate from the CAN controller */

      {
        struct can_ioctl_data_s *req =
          (struct can_ioctl_data_s *)((uintptr_t)arg);
        req->arbi_bitrate = priv->bitrate_value;
        req->arbi_samplep = mpfs_can_get_sample_point(priv);
        ret = CAN_OK;
      }
      break;

    case SIOCSCANBITRATE:

      /* Set bitrate of the CAN controller */

      {
        struct can_ioctl_data_s *req =
          (struct can_ioctl_data_s *)((uintptr_t)arg);

        /* Stop CAN controller */

        mpfs_can_stop(priv);

        if (CAN_OK != mpfs_can_set_bitrate(priv, req->arbi_bitrate))
          {
            canerr("CAN controller bitrate set failed");
            ret = -EAGAIN;
            break;
          }

        /* Start CAN controller again */

        mpfs_can_start(priv);

        ret = CAN_OK;
      }
      break;
#endif /* CONFIG_NETDEV_CAN_BITRATE_IOCTL */

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
    case SIOCACANSTDFILTER:
    case SIOCACANEXTFILTER:

      {
        struct can_ioctl_filter_s *req =
          (struct can_ioctl_filter_s *)((uintptr_t)arg);

        if (CAN_OK != mpfs_can_add_filter(priv, req->ftype,
                                          req->fid1, req->fid2))
          {
            canerr("CAN filter add failed");
            ret = -EINVAL;
            break;
          }

        ret = CAN_OK;
      }
      break;

    case SIOCDCANSTDFILTER:
    case SIOCDCANEXTFILTER:

      {
        mpfs_can_reset_filter(priv);
        ret = CAN_OK;
      }
      break;
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_can_init
 *
 * Description:
 *  Initialize the CAN controller and driver
 *
 * Input Parameters:
 *  ncan    - CAN controller number
 *  bitrate - CAN controller bitrate
 *
 * Returned Value:
 *  On success, a pointer to the MPFS CAN driver is
 *  returned. NULL is returned on any failure
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

int mpfs_can_init(int ncan, uint32_t bitrate)
{
  mpfs_can_instance_t *priv;

  switch (ncan)
    {
# ifdef CONFIG_MPFS_MSS_CAN0
      case 0:
        priv = &g_can0;
        memset(priv, 0, sizeof(mpfs_can_instance_t));

        priv->reg_base = CAN0_BASE;
        priv->irqn = MPFS_IRQ_CAN0;

        /* Initialize TX/RX descriptor structure */

        priv->txdesc = (struct can_frame *)&g_tx_pool0;
        priv->rxdesc = (struct can_frame *)&g_rx_pool0;

        /* Initialize TX/RX message object structure */

        priv->tx_msg = &g_tx_msg0;
        priv->rx_msg = &g_rx_msg0;

        break;
# endif
# ifdef CONFIG_MPFS_MSS_CAN1
      case 1:
        priv = &g_can1;
        memset(priv, 0, sizeof(mpfs_can_instance_t));

        priv->reg_base = CAN1_BASE;
        priv->irqn = MPFS_IRQ_CAN1;

        /* Initialize TX/RX descriptor structure */

        priv->txdesc = (struct can_frame *)&g_tx_pool1;
        priv->rxdesc = (struct can_frame *)&g_rx_pool1;

        /* Initialize TX/RX message object structure */

        priv->tx_msg = &g_tx_msg1;
        priv->rx_msg = &g_rx_msg1;

        break;
# endif
      default:
        canerr("No such CAN%d available\n", ncan);
        return -ENOTTY;
    }

  /* Get the controller out of Reset mode */

  mpfs_can_reset(priv);

  /* Initialize the CAN bitrate */

  if (CAN_OK != mpfs_can_set_bitrate(priv, bitrate))
    {
      return -EAGAIN;
    }

  /* Initialize the number of buffers */

  priv->basic_can_rxb_count = CAN_RX_BUFFER;
  priv->basic_can_txb_count = CAN_TX_BUFFER;

  /* Initialize filter */

  if (CAN_OK != mpfs_can_reset_filter(priv))
    {
      canerr("CAN filter reset and RX buffer initialization failed\n");
      return -EAGAIN;
    }

  /* Disable interrupts */

  mpfs_can_clear_int_ebl(priv, MPFS_CAN_INT_ENABLE_INT_ENBL);

  /* Configure CAN modes */

  mpfs_can_set_mode(priv, CANOP_MODE_NORMAL);

  /* Attach the interrupt handler */

  if (irq_attach(priv->irqn, mpfs_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      canerr("Failed to attach to CAN%d IRQ\n", ncan);
      return -EAGAIN;
    }

  /* Initialize the driver network device structure */

  priv->dev.d_ifup    = mpfs_ifup;    /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = mpfs_ifdown;  /* I/F down callback */
  priv->dev.d_txavail = mpfs_txavail; /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = mpfs_ioctl;   /* Support CAN ioctl() calls */
#endif
  priv->dev.d_private = priv;     /* Used to recover private state from dev */

  caninfo("CAN driver init done for CAN controller %d\n", ncan);

  /* Put the interface in the down state */

  mpfs_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_CAN);

  return CAN_OK;
}

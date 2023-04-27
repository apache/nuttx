/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_fpga_canfd.c
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

#include "hardware/mpfs_fpga_canfd.h"
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef OK
#  define OK 0
#endif

/* This module only compiles if the CAN-FD IP core instance
 * is configured to the FPGA
 */

#ifndef CONFIG_MPFS_CANFD
#  error This should not be compiled as CAN-FD FPGA block is not defined
#endif

/* This module only compiles if Nuttx socketCAN interface supports CANFD */

#ifndef CONFIG_NET_CAN_CANFD
#  error This should not be compiled as CAN-FD driver relies on socket CAN
#endif

/* Clock reset and enabling */

#define MPFS_SYSREG_SOFT_RESET_CR     (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SOFT_RESET_CR_OFFSET)
#define MPFS_SYSREG_SUBBLK_CLOCK_CR   (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET)

#define CANWORK                 HPWORK

#define MPFS_CANFD_ID           0xCAFD

/* For allocating the tx and rx CAN-FD frame buffer */

#define POOL_SIZE               1
#define TIMESTAMP_SIZE          sizeof(struct timeval)  /* To support
                                                         * timestamping frame */

/* For bit timing calculation */

#define BT_COMPUTE_MAX_ERROR      50 /* 1/10th of % */
#define BT_COMPUTE_SYNC_SEG       1

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
/* CAN hw filter support */

#define HW_FILTER_A             0
#define HW_FILTER_B             1
#define HW_FILTER_C             2
#define HW_FILTER_RANGE         3

#define CAN_STD_ID              0
#define CAN_EXT_ID              1
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

/* Special address description flags for the CAN_ID */

#define CAN_EFF_FLAG            0x80000000  /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG            0x40000000  /* remote transmission request */
#define CAN_ERR_FLAG            0x20000000  /* error message frame */

/* Valid bits in CAN ID for frame formats */

#define CAN_SFF_MASK            0x000007FF /* standard frame format (SFF) */
#define CAN_EFF_MASK            0x1FFFFFFF /* extended frame format (EFF) */
#define CAN_ERR_MASK            0x1FFFFFFF /* omit EFF, RTR, ERR flags */

/* CAN control mode */

#define CAN_CTRLMODE_LOOPBACK       0x01  /* Loopback mode */
#define CAN_CTRLMODE_LISTENONLY     0x02  /* Listen-only mode */
#define CAN_CTRLMODE_3_SAMPLES      0x04  /* Triple sampling mode */
#define CAN_CTRLMODE_ONE_SHOT       0x08  /* One-Shot mode */
#define CAN_CTRLMODE_BERR_REPORTING 0x10  /* Bus-error reporting */
#define CAN_CTRLMODE_FD             0x20  /* CAN FD mode */
#define CAN_CTRLMODE_PRESUME_ACK    0x40  /* Ignore missing CAN ACKs */
#define CAN_CTRLMODE_FD_NON_ISO     0x80  /* CAN FD in non-ISO mode */
#define CAN_CTRLMODE_CC_LEN8_DLC    0x100 /* Classic CAN DLC option */
#define CAN_CTRLMODE_TDC_AUTO       0x200 /* CAN transiver automatically
                                           * calculates TDCV */
#define CAN_CTRLMODE_TDC_MANUAL     0x400 /* TDCV is manually set up by user */

/* TXT buffer */

enum mpfs_can_txb_status
{
  TXT_NOT_EXIST       = 0x0,
  TXT_RDY             = 0x1,
  TXT_TRAN            = 0x2,
  TXT_ABTP            = 0x3,
  TXT_TOK             = 0x4,
  TXT_ERR             = 0x6,
  TXT_ABT             = 0x7,
  TXT_ETY             = 0x8,
};

enum mpfs_can_txb_command
{
  TXT_CMD_SET_EMPTY   = 0x01,
  TXT_CMD_SET_READY   = 0x02,
  TXT_CMD_SET_ABORT   = 0x04
};

/****************************************************************************
 * Utility definitions
 ****************************************************************************/

#ifndef min
#define min(a, b)  ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b)  ((a) > (b) ? (a) : (b))
#endif

#define clamp(val, lo, hi)  min(max(val, lo), hi)

#define MPFS_CAN_FD_TXNF(priv) \
  (getreg32(priv->base + MPFS_CANFD_STATUS_OFFSET) & MPFS_CANFD_STATUS_TXNF)
#define MPFS_CAN_FD_ENABLED(priv) \
  (getreg32(priv->base + MPFS_CANFD_MODE_OFFSET) & MPFS_CANFD_MODE_ENA)

#if __GNUC__ >= 3
#  define expect(expr,value) __builtin_expect((expr),(value))
#else
#  define expect(expr,value) (expr)
#endif

#define expect_false(expr)   expect((expr) != 0, 0)
#define expect_true(expr)    expect((expr) != 0, 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CAN operational and error states */

/* CAN bit timing parameters */

struct mpfs_can_bittiming_s
{
  uint32_t bitrate;       /* Bit-rate in bits/second */
  uint32_t sample_point;  /* Sample point in one-tenth of a percent */
  uint32_t tq;            /* Time quanta (TQ) in nanoseconds */
  uint32_t prop_seg;      /* Propagation segment in TQs */
  uint32_t phase_seg1;    /* Phase buffer segment 1 in TQs */
  uint32_t phase_seg2;    /* Phase buffer segment 2 in TQs */
  uint32_t sjw;           /* Synchronisation jump width in TQs */
  uint32_t brp;           /* Bitrate prescaler */
};

/* CAN harware dependent bit timing constants
 * Used for calculating and checking bit timing parameters
 */

struct mpfs_can_bittiming_const_s
{
  uint32_t tseg_min;
  uint32_t tseg_max;
  uint32_t brp_min;
  uint32_t brp_max;
};

static const struct mpfs_can_bittiming_const_s mpfs_can_bit_timing_range =
{
  .tseg_min = 3,
  .tseg_max = 253,
  .brp_min = 1,
  .brp_max = 8,
};

static const struct mpfs_can_bittiming_const_s
  mpfs_can_bit_timing_data_range =
{
  .tseg_min = 3,
  .tseg_max = 125,
  .brp_min = 1,
  .brp_max = 2,
};

struct mpfs_can_clock_s
{
  uint32_t freq;     /* CAN system clock frequency in Hz */
};

enum mpfs_can_state_e
{
  CAN_STATE_ERROR_ACTIVE = 0, /* RX/TX error count < 96 */
  CAN_STATE_ERROR_WARNING,    /* RX/TX error count < 128 */
  CAN_STATE_ERROR_PASSIVE,    /* RX/TX error count < 256 */
  CAN_STATE_BUS_OFF,          /* RX/TX error count >= 256 */
  CAN_STATE_STOPPED,          /* Device is stopped */
  CAN_STATE_SLEEPING,         /* Device is sleeping */
  CAN_STATE_MAX
};

struct mpfs_can_ctrlmode_s
{
  uint32_t mask;
  uint32_t flags;
};

struct mpfs_can_berr_counter_s
{
  uint16_t txerr;
  uint16_t rxerr;
};

struct mpfs_can_device_stats_s
{
  uint32_t bus_error;         /* Bus errors */
  uint32_t error_warning;     /* Changes to error warning state */
  uint32_t error_passive;     /* Changes to error passive state */
  uint32_t bus_off;           /* Changes to bus off state */
  uint32_t arbitration_lost;  /* Arbitration lost errors */
  uint32_t restarts;          /* CAN controller re-starts */
};

/* CAN common private data */

struct mpfs_can_priv_s
{
  struct mpfs_can_device_stats_s can_stats;
  struct mpfs_can_bittiming_s bittiming;
  struct mpfs_can_bittiming_s data_bittiming;
  const struct mpfs_can_bittiming_const_s *bittiming_const;
  const struct mpfs_can_bittiming_const_s *data_bittiming_const;
  struct mpfs_can_clock_s clock;

  enum mpfs_can_state_e state;
  uint32_t ctrlmode;
};

/****************************************************************************
 * CANFD Frame Format
 ****************************************************************************/

/* CAN frame format memory map */

enum mpfs_canfd_can_frame_format
{
  MPFS_CANFD_FRAME_FORMAT_W_OFFSET       = 0x0,
  MPFS_CANFD_IDENTIFIER_W_OFFSET         = 0x4,
  MPFS_CANFD_TIMESTAMP_L_W_OFFSET        = 0x8,
  MPFS_CANFD_TIMESTAMP_U_W_OFFSET        = 0xc,
  MPFS_CANFD_DATA_1_4_W_OFFSET           = 0x10,
  MPFS_CANFD_DATA_5_8_W_OFFSET           = 0x14,
  MPFS_CANFD_DATA_61_64_W_OFFSET         = 0x4c,
};

/* CANFD_Frame_format memory region */

/*  FRAME_FORMAT_W registers */

#define MPFS_CANFD_FRAME_FORMAT_W_DLC_SHIFT           (0)
#define MPFS_CANFD_FRAME_FORMAT_W_DLC                 (0x0F << \
  MPFS_CANFD_FRAME_FORMAT_W_DLC_SHIFT)
#define MPFS_CANFD_FRAME_FORMAT_W_RTR                 (1 << 5)
#define MPFS_CANFD_FRAME_FORMAT_W_IDE                 (1 << 6)
#define MPFS_CANFD_FRAME_FORMAT_W_FDF                 (1 << 7)
#define MPFS_CANFD_FRAME_FORMAT_W_BRS                 (1 << 9)
#define MPFS_CANFD_FRAME_FORMAT_W_ESI_RSV             (1 << 10)
#define MPFS_CANFD_FRAME_FORMAT_W_RWCNT_SHIFT         (11)
#define MPFS_CANFD_FRAME_FORMAT_W_RWCNT               (0x1F << \
  MPFS_CANFD_FRAME_FORMAT_W_RWCNT_SHIFT)

/*  IDENTIFIER_W registers */

#define MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_EXT_SHIFT  (0)
#define MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_EXT        (0x03FFFF << \
  MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_EXT_SHIFT)
#define MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE_SHIFT (18)
#define MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE       (0x07FF << \
  MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE_SHIFT)

/****************************************************************************
 * CAN controller hardware configuration
 ****************************************************************************/

struct mpfs_config_s
{
  uint32_t canfd_fpga_irq;           /* the only CAN-FD FPGA IRQ */
};

static const struct mpfs_config_s mpfs_fpga_canfd_config =
{
  .canfd_fpga_irq = MPFS_IRQ_FABRIC_F2H_0,
};

/****************************************************************************
 * The mpfs_driver_s encapsulates all state information for a single
 * hw interface
 ****************************************************************************/

struct mpfs_driver_s
{
  struct mpfs_can_priv_s can;

  const struct mpfs_config_s *config;

  uintptr_t base;             /* CANFD FPGA base address */
  bool bifup;                 /* true:ifup false:ifdown */

  struct work_s rxwork;       /* for deferring rx interrupt work to the wq */
  struct work_s txdwork;      /* For deferring tx done interrupt work to the
                               * wq */
  struct work_s pollwork;     /* For deferring poll work to the wq */

  struct canfd_frame *txdesc; /* A pointer to the list of TX descriptor */
  struct canfd_frame *rxdesc; /* A pointer to the list of RX descriptors */

  /* rx */

  uint32_t drv_flags;         /* driver flag */
  uint32_t rxfrm_first_word;  /* rx frame first word (usually a FFW) */

  /* tx */

  unsigned int txb_sent;
  unsigned int txb_processed;
  uint32_t txb_prio;
  unsigned int ntxbufs;

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
  /* hw filter */

  uint8_t used_bit_filter_number;
  bool used_range_filter;
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;    /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mpfs_driver_s g_canfd;

static uint8_t g_tx_pool[(sizeof(struct canfd_frame) + TIMESTAMP_SIZE) *
                         POOL_SIZE];
static uint8_t g_rx_pool[(sizeof(struct canfd_frame) + TIMESTAMP_SIZE) *
                         POOL_SIZE];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* (from interrupt) RX related functions */

static void mpfs_can_retrieve_rx_frame(struct mpfs_driver_s *priv,
                                       struct canfd_frame *cf,
                                       uint32_t ffw);
static void mpfs_receive_work(void *arg);

/* (from interrupt) TX related functions */

static void mpfs_can_update_txb_prio(struct mpfs_driver_s *priv);
static void mpfs_can_send_txb_cmd(struct mpfs_driver_s *priv,
                                  enum mpfs_can_txb_command cmd,
                                  uint8_t buf);
static void mpfs_txdone(struct mpfs_driver_s *priv);
static void mpfs_txdone_work(void *arg);

/* (from interrupt) Error handling related functions */

static enum mpfs_can_state_e
  mpfs_can_get_err_state(struct mpfs_driver_s *priv);
static void mpfs_can_get_err_count(struct mpfs_driver_s *priv,
                                   struct mpfs_can_berr_counter_s *bec);
static void mpfs_err_interrupt(struct mpfs_driver_s *priv, uint32_t isr);

/* Interrupt service routine */

static int mpfs_interrupt(int irq, void *context, void *arg);

/* (Nuttx network driver interface callback when TX packet available) Tx
 * related functions
 */

static enum mpfs_can_txb_status
  mpfs_can_get_txb_status(struct mpfs_driver_s *priv,
                          uint8_t buf);
static bool mpfs_can_is_txb_writable(struct mpfs_driver_s *priv,
                                     uint8_t buf);
static bool mpfs_can_write_txb(struct mpfs_driver_s *priv,
                               const struct canfd_frame *cf,
                               uint8_t buf,
                               bool is_ccf);
static int mpfs_transmit(struct mpfs_driver_s *priv);
static int mpfs_txpoll(struct net_driver_s *dev);
static void mpfs_txavail_work(void *arg);
static int mpfs_txavail(struct net_driver_s *dev);

/* Bit timing related functions */

static int
  mpfs_can_btr_compute(struct mpfs_driver_s *priv,
                       struct mpfs_can_bittiming_s *bt,
                       const struct mpfs_can_bittiming_const_s *btc);

static int mpfs_can_config_bit_timing(struct mpfs_driver_s *priv,
                                      struct mpfs_can_bittiming_s *bt,
                                      bool arbi);
static int mpfs_can_config_arbi_bit_timing(struct mpfs_driver_s *priv);
static int mpfs_can_config_data_bit_timing(struct mpfs_driver_s *priv);

/*  Miscellaneous CAN controller interface functions */

static int mpfs_can_config_ssp(struct mpfs_driver_s *priv);
static void
  mpfs_can_config_controller_mode(struct mpfs_driver_s *priv,
                                  const struct mpfs_can_ctrlmode_s *mode);

/* HW filter related functions */

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
static void mpfs_can_add_hw_filter(struct mpfs_driver_s *priv,
                                   uint8_t filter_type,
                                   uint8_t can_id_type,
                                   uint8_t can_type,
                                   uint32_t fid1,
                                   uint32_t fid2);
static void mpfs_can_reset_hw_filter(struct mpfs_driver_s *priv);
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

/* CAN controller life cycle routines */

static int mpfs_can_controller_start(struct mpfs_driver_s *priv);
static void mpfs_can_controller_stop(struct mpfs_driver_s *priv);
static int mpfs_reset(struct mpfs_driver_s *priv);

/* Driver interface to Nuttx network callbacks */

static int mpfs_ifup(struct net_driver_s *dev);
static int mpfs_ifdown(struct net_driver_s *dev);
#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
static int mpfs_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg);
#endif

/****************************************************************************
 * Private Function
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_can_retrieve_rx_frame
 *
 * Description:
 *  Retrieve CAN/CANFD frame from RX Buffer
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  cf    - Pointer to CANFD frame structure
 *  ffw   - Previously read frame format word
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  Frame format word is already parsed in advance and provided as 'ffw' arg
 *
 ****************************************************************************/

static void mpfs_can_retrieve_rx_frame(struct mpfs_driver_s *priv,
                                   struct canfd_frame *cf,
                                   uint32_t ffw)
{
  uint32_t idw;
  unsigned int i;
  unsigned int data_wc; /* data word count */
  unsigned int data_bc; /* data byte count */
  unsigned int dlc;
  unsigned int len;

  /* CAN ID */

  idw = getreg32(priv->base + MPFS_CANFD_RX_DATA_OFFSET);
  if (MPFS_CANFD_FRAME_FORMAT_W_IDE & ffw)
    {
      cf->can_id = (idw & CAN_EFF_MASK) | CAN_EFF_FLAG;
    }
  else
    {
      cf->can_id =
        (idw >> MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE_SHIFT) &
        CAN_SFF_MASK;
    }

  /* BRS, ESI, RTR Flags */

  cf->flags = 0;
  if (MPFS_CANFD_FRAME_FORMAT_W_FDF & ffw)
    {
      /* Enable bitrate switch by default if frame is CANFD */

      cf->flags |= CANFD_BRS;

      if (MPFS_CANFD_FRAME_FORMAT_W_ESI_RSV & ffw)
        {
          cf->flags |= CANFD_ESI;
        }
    }
  else if (MPFS_CANFD_FRAME_FORMAT_W_RTR & ffw)
    {
      cf->can_id |= CAN_RTR_FLAG;
    }

  /* RWCNT : RX Count of Words without FRAME_FORMAT WORD. Minus the 3 words
   * for 1 IDW, 2 timestamp words
   */

  data_wc = ((MPFS_CANFD_FRAME_FORMAT_W_RWCNT & ffw) >>
    MPFS_CANFD_FRAME_FORMAT_W_RWCNT_SHIFT) - 3;
  data_bc = data_wc * 4;

  /* DLC */

  dlc = (MPFS_CANFD_FRAME_FORMAT_W_DLC & ffw) >>
    MPFS_CANFD_FRAME_FORMAT_W_DLC_SHIFT;
  if (dlc <= 8)
    {
     len = dlc;
    }
  else
    {
      if (MPFS_CANFD_FRAME_FORMAT_W_FDF & ffw)
        {
          len = data_wc << 2;
        }
      else
        {
          len = 8;
        }
    }
  cf->len = len;
  if (expect_false(len > data_bc))
    {
      len = data_bc;
    }

  /* Timestamp - Read and throw away */

  getreg32(priv->base + MPFS_CANFD_RX_DATA_OFFSET);
  getreg32(priv->base + MPFS_CANFD_RX_DATA_OFFSET);

  /* Data */

  for (i = 0; i < len; i += 4)
    {
      uint32_t data = getreg32(priv->base + MPFS_CANFD_RX_DATA_OFFSET);

      *(uint32_t *)(cf->data + i) = data;
    }

  /* Read and discard exceeding data that does not fit any frame. read
   * pointer is automatically increased if RX buffer is not empty.
   */

  while (expect_false(i < data_bc))
    {
      getreg32(priv->base + MPFS_CANFD_RX_DATA_OFFSET);
      i += 4;
    }
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
  struct mpfs_driver_s *priv = (struct mpfs_driver_s *)arg;

  uint32_t status;
  uint32_t frame_count;
  bool is_classical_can_frame = false;

  frame_count = (getreg32(priv->base + MPFS_CANFD_RX_STATUS_OFFSET) &
    MPFS_CANFD_RX_STATUS_RXFRC) >> MPFS_CANFD_RX_STATUS_RXFRC_SHIFT;
  while (frame_count)
    {
      struct canfd_frame *cf = (struct canfd_frame *)priv->rxdesc;
      uint32_t ffw;

      ffw = getreg32(priv->base + MPFS_CANFD_RX_DATA_OFFSET);

      if (!(MPFS_CANFD_FRAME_FORMAT_W_RWCNT & ffw))
        {
          break;
        }

      if (!(MPFS_CANFD_FRAME_FORMAT_W_FDF & ffw))
        {
          if (MPFS_CANFD_FRAME_FORMAT_W_RTR & ffw)
            {
              caninfo("Remote Frame received\n");
            }
          else
            {
              caninfo("Classical CAN Frame received\n");
            }

          is_classical_can_frame = true;
        }
      else
        {
          caninfo("CANFD Frame received\n");
        }

      /* Retrieve the classical or CANFD or remote frame */

      mpfs_can_retrieve_rx_frame(priv, cf, ffw);

      /* Copy the buffer pointer to priv->dev..  Set amount of data
       * in priv->dev.d_len
       */

      priv->dev.d_len = is_classical_can_frame ?
        sizeof(struct can_frame) : sizeof(struct canfd_frame);
      priv->dev.d_buf = (uint8_t *)cf;

      /* Send to socket interface */

      NETDEV_RXPACKETS(&priv->dev);
      can_input(&priv->dev);

      /* Point the packet buffer back to the next Tx buffer that will be
       * used during the next write.  If the write queue is full, then
       * this will point at an active buffer, which must not be written
       * to.  This is OK because devif_poll won't be called unless the
       * queue is not full.
       */

      priv->dev.d_buf = (uint8_t *)priv->txdesc;

      frame_count = (getreg32(priv->base + MPFS_CANFD_RX_STATUS_OFFSET) &
        MPFS_CANFD_RX_STATUS_RXFRC) >> MPFS_CANFD_RX_STATUS_RXFRC_SHIFT;
    }

  /* Check for RX FIFO Overflow */

  status = getreg32(priv->base + MPFS_CANFD_STATUS_OFFSET);
  if (MPFS_CANFD_STATUS_DOR & status)
    {
      /* Notify to socket interface */

      NETDEV_RXERRORS(&priv->dev);

      /* Clear Data Overrun */

      putreg32(MPFS_CANFD_COMMAND_CDO,
               priv->base + MPFS_CANFD_COMMAND_OFFSET);
    }

  /* Clear and re-enable RBNEI */

  putreg32(MPFS_CANFD_INT_STAT_RBNEI,
           priv->base + MPFS_CANFD_INT_STAT_OFFSET);
  putreg32(MPFS_CANFD_INT_STAT_RBNEI,
           priv->base + MPFS_CANFD_INT_MASK_CLR_OFFSET);
}

/****************************************************************************
 * Name: mpfs_can_update_txb_prio
 *
 * Description:
 *  Rotates priorities of TXT buffers
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

static void mpfs_can_update_txb_prio(struct mpfs_driver_s *priv)
{
  uint32_t prio = priv->txb_prio;

  /* Rotate TX_PRIORITY register states one step left */

  prio = (prio << 4) | ((prio >> ((priv->ntxbufs - 1) * 4)) & 0xf);
  priv->txb_prio = prio;
  putreg32(prio, priv->base + MPFS_CANFD_TX_PRIORITY_OFFSET);
}

/****************************************************************************
 * Name: mpfs_can_send_txb_cmd
 *
 * Description:
 *  Execute a TXT buffer command
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  cmd   - Cmd to give
 *  buf   - The TXT buffer index (0-based)
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_send_txb_cmd(struct mpfs_driver_s *priv,
                                  enum mpfs_can_txb_command cmd,
                                  uint8_t buf)
{
  uint32_t txb_cmd = cmd;

  txb_cmd |= 1 << (buf + 8);
  putreg32(txb_cmd, priv->base + MPFS_CANFD_TX_COMMAND_OFFSET);
}

/****************************************************************************
 * Name: mpfs_txdone
 *
 * Description:
 *  Tx done interrupt service rountine
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

static void mpfs_txdone(struct mpfs_driver_s *priv)
{
  bool first_buffer = true;
  bool buffer_processed;
  enum mpfs_can_txb_status txb_status;
  uint8_t txb_id;

  do
    {
      buffer_processed = false;
      while ((int)(priv->txb_sent - priv->txb_processed) > 0)
        {
          txb_id = priv->txb_processed % priv->ntxbufs;
          txb_status = mpfs_can_get_txb_status(priv, txb_id);
          bool other_status = false;

          switch (txb_status)
            {
            case TXT_TOK:
              break;
            case TXT_ERR:
              canwarn("TXB in Error state\n");
              break;
            case TXT_ABT:
              canwarn("TXB in Aborted state\n");
              break;
            default:
              other_status = true;

              if (first_buffer)
                {
                  canerr("TXB#%u not in a finished state (0x%x)!\n",
                         txb_id, txb_status);

                  priv->txb_processed++;

                  /* Rotate TXT buffer priority */

                  mpfs_can_update_txb_prio(priv);

                  /* Mark current unfinished state TXT buffer as empty */

                  mpfs_can_send_txb_cmd(priv, TXT_CMD_SET_EMPTY, txb_id);

                  /* Something is fishy. CLear txb status change interrupt */

                  putreg32(MPFS_CANFD_INT_STAT_TXBHCI,
                           priv->base + MPFS_CANFD_INT_STAT_OFFSET);

                  return;
                }
              break;
            }

          if (other_status)
            {
              break;
            }
          else
            {
              priv->txb_processed++;
              first_buffer = false;
              buffer_processed = true;

              /* Rotate TXT buffer priority */

              mpfs_can_update_txb_prio(priv);

              /* Mark current finished state TXT buffer as empty */

              mpfs_can_send_txb_cmd(priv, TXT_CMD_SET_EMPTY, txb_id);
            }
        }

      if (buffer_processed)
        {
          /* Since there are some buffers processed and the number of TXT
           * used now matched the number of processed buffer, we can clear
           * the interrupt in order to avoid any erroneous interrupt after
           * the last true TXT buffer interrupt is processed.
           */

          putreg32(MPFS_CANFD_INT_STAT_TXBHCI,
                   priv->base + MPFS_CANFD_INT_STAT_OFFSET);
        }
    }
  while (buffer_processed);
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
  struct mpfs_driver_s *priv = (struct mpfs_driver_s *)arg;

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  net_lock();
  devif_poll(&priv->dev, mpfs_txpoll);
  net_unlock();
}

/****************************************************************************
 * Name: mpfs_can_get_err_state
 *
 * Description:
 *    Reads CAN fault confinement state
 *
 * Input Parameters:
 *    priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *    Fault confinement state of controller
 *
 ****************************************************************************/

static enum mpfs_can_state_e
  mpfs_can_get_err_state(struct mpfs_driver_s *priv)
{
  u_int32_t ewl_erp_fs_reg, rec_tec_reg, ew_limit, rec_val, tec_val;

  ewl_erp_fs_reg = getreg32(priv->base + MPFS_CANFD_EWL_OFFSET);
  rec_tec_reg = getreg32(priv->base + MPFS_CANFD_REC_OFFSET);

  ew_limit = ((ewl_erp_fs_reg & MPFS_CANFD_EWL_EW_LIMIT) >>
    MPFS_CANFD_EWL_EW_LIMIT_SHIFT);
  rec_val = ((rec_tec_reg & MPFS_CANFD_REC_REC_VAL) >>
    MPFS_CANFD_REC_REC_VAL_SHIFT);
  tec_val = ((rec_tec_reg & MPFS_CANFD_REC_TEC_VAL) >>
    MPFS_CANFD_REC_TEC_VAL_SHIFT);

  if (ewl_erp_fs_reg & MPFS_CANFD_EWL_ERA)
    {
      if (rec_val < ew_limit && tec_val < ew_limit)
        {
          return CAN_STATE_ERROR_ACTIVE;
        }
      else
        {
          return CAN_STATE_ERROR_WARNING;
        }
    }
  else if (ewl_erp_fs_reg & MPFS_CANFD_EWL_ERP)
    {
     return CAN_STATE_ERROR_PASSIVE;
    }
  else if (ewl_erp_fs_reg & MPFS_CANFD_EWL_BOF)
    {
     return CAN_STATE_BUS_OFF;
    }

  canwarn("Invalid FPGA CAN-FD error state\n");
  return CAN_STATE_ERROR_PASSIVE;
}

/****************************************************************************
 * Name: mpfs_can_get_err_count
 *
 * Description:
 *    Reads CAN RX/TX error counter
 *
 * Input Parameters:
 *    priv  - Pointer to the private CAN driver state structure
 *    bec   - Pointer to Error counter structure
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

static void mpfs_can_get_err_count(struct mpfs_driver_s *priv,
                                   struct mpfs_can_berr_counter_s *bec)
{
  uint32_t rec_tec_reg = getreg32(priv->base + MPFS_CANFD_REC_OFFSET);

  bec->rxerr = ((rec_tec_reg & MPFS_CANFD_REC_REC_VAL) >>
    MPFS_CANFD_REC_REC_VAL_SHIFT);
  bec->txerr = ((rec_tec_reg & MPFS_CANFD_REC_TEC_VAL) >>
    MPFS_CANFD_REC_TEC_VAL_SHIFT);
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

static void mpfs_err_interrupt(struct mpfs_driver_s *priv, uint32_t isr)
{
  enum mpfs_can_state_e state;
  struct mpfs_can_berr_counter_s bec;
  uint32_t err_capt_retr_ctr_alc_reg;
  uint32_t err_type;
  uint32_t err_pos;
  uint32_t alc_id_field;
  uint32_t alc_bit;

  mpfs_can_get_err_count(priv, &bec);
  state = mpfs_can_get_err_state(priv);
  err_capt_retr_ctr_alc_reg =
    getreg32(priv->base + MPFS_CANFD_ERR_CAPT_OFFSET);

  err_type = ((err_capt_retr_ctr_alc_reg & MPFS_CANFD_ERR_CAPT_ERR_TYPE) >>
    MPFS_CANFD_ERR_CAPT_ERR_TYPE_SHIFT);
  err_pos = ((err_capt_retr_ctr_alc_reg & MPFS_CANFD_ERR_CAPT_ERR_POS) >>
    MPFS_CANFD_ERR_CAPT_ERR_POS_SHIFT);
  alc_id_field =
    ((err_capt_retr_ctr_alc_reg & MPFS_CANFD_ERR_CAPT_ALC_ID_FIELD) >>
      MPFS_CANFD_ERR_CAPT_ALC_ID_FIELD_SHIFT);
  alc_bit = ((err_capt_retr_ctr_alc_reg & MPFS_CANFD_ERR_CAPT_ALC_BIT) >>
    MPFS_CANFD_ERR_CAPT_ALC_BIT_SHIFT);

  caninfo("ISR = 0x%08x, rxerr %d, txerr %d, error type %u, pos %u, ALC "
          "id_field %u, bit %u\n", isr, bec.rxerr, bec.txerr, err_type,
          err_pos, alc_id_field, alc_bit);

  /* Check for error warning limit and fault confinement state change */

  if (MPFS_CANFD_INT_STAT_FCSI & isr || MPFS_CANFD_INT_STAT_EWLI & isr)
    {
      if (priv->can.state == state)
        {
          canwarn("No state change! Missed interrupt?\n");
        }

      priv->can.state = state;

      switch (state)
        {
        case CAN_STATE_BUS_OFF:
          canwarn("Change to BUS_OFF error state\n");
          break;
        case CAN_STATE_ERROR_PASSIVE:
          priv->can.can_stats.error_passive++;
          canwarn("Change to ERROR_PASSIVE error state\n");
          break;
        case CAN_STATE_ERROR_WARNING:
          priv->can.can_stats.error_warning++;
          canwarn("Change to ERROR_WARNING error state\n");
          break;
        case CAN_STATE_ERROR_ACTIVE:
          caninfo("Change to ERROR_ACTIVE error state\n");
          return;
        default:
          canwarn("Unhandled error state %d\n", state);
          break;
        }
    }

  if (MPFS_CANFD_INT_STAT_ALI & isr)
    {
      canerr("Arbitration lost\n");
      priv->can.can_stats.arbitration_lost++;
    }

  if (MPFS_CANFD_INT_STAT_BEI & isr)
    {
      canerr("Bus error\n");
      priv->can.can_stats.bus_error++;
    }

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
 *   OK on success
 *
 ****************************************************************************/

static int mpfs_interrupt(int irq, void *context, void *arg)
{
  struct mpfs_driver_s *priv = (struct mpfs_driver_s *)arg;

  uint32_t isr;
  uint32_t icr;
  uint32_t imask;
  int irq_loops;

  for (irq_loops = 0; irq_loops < 10000; irq_loops++)
    {
      /* Get the interrupt status */

      isr  = getreg32(priv->base + MPFS_CANFD_INT_STAT_OFFSET);

      /* Check and exit interrupt service routine if there is no int flag in
       * INT_STAT reg.
       */

      if (!isr)
        {
          return irq_loops ? OK : -1;
        }

      /* Receive Buffer Not Empty Interrupt */

      if (isr & MPFS_CANFD_INT_STAT_RBNEI)
        {
          /* Mask RBNEI first, then clear interrupt. Even if
          * another IRQ fires, RBNEI will always be 0 (masked).
          */

          icr = MPFS_CANFD_INT_STAT_RBNEI;
          putreg32(icr, priv->base + MPFS_CANFD_INT_MASK_SET_OFFSET);
          putreg32(icr, priv->base + MPFS_CANFD_INT_STAT_OFFSET);

          /* Schedule work to process RX frame from CAN controller */

          work_queue(CANWORK, &priv->rxwork, mpfs_receive_work, priv, 0);
        }

      /* TXT Buffer HW Command Interrupt */

      if (isr & MPFS_CANFD_INT_STAT_TXBHCI)
        {
          /* Clear TX interrupt flags */

          mpfs_txdone(priv);

          /* Schedule work to poll for next available tx frame from the
           * network.
           */

          work_queue(CANWORK, &priv->txdwork, mpfs_txdone_work, priv, 0);
        }

      /* Error Interrupts */

      if (isr & MPFS_CANFD_INT_STAT_EWLI ||
          isr & MPFS_CANFD_INT_STAT_FCSI ||
          isr & MPFS_CANFD_INT_STAT_ALI ||
          isr & MPFS_CANFD_INT_STAT_BEI)
        {
          icr = isr & (MPFS_CANFD_INT_STAT_EWLI |
                      MPFS_CANFD_INT_STAT_FCSI |
                      MPFS_CANFD_INT_STAT_ALI |
                      MPFS_CANFD_INT_STAT_BEI);
          canerr("Some error interrupts. Clearing 0x%08x\n", icr);
          putreg32(icr, priv->base + MPFS_CANFD_INT_STAT_OFFSET);
          mpfs_err_interrupt(priv, isr);
        }
    }

  /* Now, it seems that there are still some interrupt flags that remain
   * stuck in INT_STAT reg.
   */

  canerr("Stuck interrupt (isr=%08x)\n", isr);

  /* Check if any of the stuck one belongs to txb status. */

  if (isr & MPFS_CANFD_INT_STAT_TXBHCI)
    {
#ifdef CONFIG_DEBUG_CAN_INFO
      caninfo("txb_sent=0x%08x txb_processed=0x%08x\n", priv->txb_sent,
              priv->txb_processed);
      for (int i = 0; i < priv->ntxbufs; i++)
        {
          uint32_t status = mpfs_can_get_txb_status(priv, i);
          caninfo("txb[%d] txb status=0x%08x\n", i, status);
        }
#endif
      /* Notify to socket interface */

      NETDEV_TXERRORS(&priv->dev);

      /* Clear txb status change interrupt */

      putreg32(MPFS_CANFD_INT_STAT_TXBHCI,
               priv->base + MPFS_CANFD_INT_STAT_OFFSET);
    }

  /* Check if any of the stuck one belongs to RX buffer data overrun */

  if (isr & MPFS_CANFD_INT_STAT_DOI)
    {
      /* Notify to socket interface */

      NETDEV_RXERRORS(&priv->dev);

      /* Clear Data Overrun interrupt */

      putreg32(MPFS_CANFD_INT_STAT_DOI,
               priv->base + MPFS_CANFD_INT_STAT_OFFSET);
    }

  /* Clear and reset all interrupt. */

  canwarn("Reset all interrupts...\n");
  imask = 0xffffffff;
  putreg32(imask, priv->base + MPFS_CANFD_INT_ENA_CLR_OFFSET);
  putreg32(imask, priv->base + MPFS_CANFD_INT_ENA_SET_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: mpfs_can_get_tx_state
 *
 * Description:
 *  Get status of txt buffer
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  buf   - txt buffer index to get status of (0-based)
 *
 * Returned Value:
 *  Status of txt buffer
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static enum mpfs_can_txb_status
  mpfs_can_get_txb_status(struct mpfs_driver_s *priv, uint8_t buf)
{
  uint32_t tx_status = getreg32(priv->base + MPFS_CANFD_TX_STATUS_OFFSET);
  enum mpfs_can_txb_status status = (tx_status >> (buf * 4)) & 0xf;

  return status;
}

/****************************************************************************
 * Name: mpfs_can_is_txb_writable
 *
 * Description:
 *  Precheck txb state if a new frame can be written
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  buf   - txt buffer index to get status of (0-based)
 *
 * Returned Value:
 *  True  - Frame can be inserted to txt buffer
 *  False - If attempted, frame will not be inserted to txt buffer
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static bool mpfs_can_is_txb_writable(struct mpfs_driver_s *priv,
                                     uint8_t buf)
{
  enum mpfs_can_txb_status buf_status;

  buf_status = mpfs_can_get_txb_status(priv, buf);
  if (buf_status == TXT_RDY || buf_status == TXT_TRAN ||
      buf_status == TXT_ABTP)
    {
      canwarn("TXT buffer status %d\n", buf_status);
     return false;
    }

  return true;
}

/****************************************************************************
 * Name: mpfs_can_write_txb
 *
 * Description:
 *  Load CAN frame onto txt buffer on the CAN controller
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  cf    - Pointer to the CANFD frame to be inserted
 *  buf   - txt buffer index to which the cf frame is inserted (0-based)
 *  is_ccf- is classical can frame (bool)
 *
 * Returned Value:
 *  True  - TXT buffer written successfully
 *  False - Frame was not written to TXT buffer due to:
 *            1. TXT buffer is not writable
 *            2. Invalid TXT buffer index
 *            3. Invalid frame length
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static bool mpfs_can_write_txb(struct mpfs_driver_s *priv,
                                  const struct canfd_frame *cf,
                                  uint8_t buf,
                                  bool is_ccf)
{
  uint32_t buf_base;
  uint32_t ffw = 0;
  uint32_t idw = 0;
  unsigned int i;

  /* Check for invalid txt buffer index */

  if (buf >= priv->ntxbufs)
    {
      canerr("Invalid txt buffer index...\n");
      return false;
    }

  /* Check if it is possible to insert frame to txt buffer */

  if (!mpfs_can_is_txb_writable(priv, buf))
    {
      canwarn("Not possible to insert frame to txt buffer...\n");
      return false;
    }

  /* Check for invalid classical CAN / CANFD frame length */

  if (cf->len > CANFD_MAX_DLEN || (cf->len > CAN_MAX_DLEN && is_ccf))
    {
      canerr("Invalid classical / CANFD CAN frame length...\n");
      return false;
    }

  /* Populate frame format word */

  if (cf->can_id & CAN_RTR_FLAG)  /* remote transmission request */
    {
      ffw |= MPFS_CANFD_FRAME_FORMAT_W_RTR;
    }

  if (cf->can_id & CAN_EFF_FLAG)  /* extended frame format (29 bit long id) */
    {
      ffw |= MPFS_CANFD_FRAME_FORMAT_W_IDE;
    }

  if (!is_ccf)
    {
      ffw |= MPFS_CANFD_FRAME_FORMAT_W_FDF; /* FD Frame */
      if (cf->flags & CANFD_BRS)
        {
          ffw |= MPFS_CANFD_FRAME_FORMAT_W_BRS; /* Bit rate switch */
        }
    }

  ffw |= MPFS_CANFD_FRAME_FORMAT_W_DLC & (len_to_can_dlc[cf->len] <<
    MPFS_CANFD_FRAME_FORMAT_W_DLC_SHIFT);

  /* Populate CAN frame id word */

  if (cf->can_id & CAN_EFF_FLAG)
    {
     idw = cf->can_id & CAN_EFF_MASK;
    }
  else
    {
      idw = MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE &
        ((cf->can_id & CAN_SFF_MASK) <<
          MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE_SHIFT);
    }

  /* Write frame id and frame format word */

  buf_base = (buf + 1) * 0x100;
  putreg32(ffw, priv->base + buf_base + MPFS_CANFD_FRAME_FORMAT_W_OFFSET);
  putreg32(idw, priv->base + buf_base + MPFS_CANFD_IDENTIFIER_W_OFFSET);

  /* Write CAN data payload */

  if (!(cf->can_id & CAN_RTR_FLAG))
    {
      for (i = 0; i < cf->len; i += 4)
        {
          uint32_t data = *(uint32_t *)(cf->data + i);
          putreg32(data,
                   priv->base + buf_base + MPFS_CANFD_DATA_1_4_W_OFFSET + i);
        }
    }

  return true;
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
 *  Zero (OK) on success; a negated errno on failure
 *
 * Assumptions:
 *  May or may not be called from an interrupt handler.  In either case,
 *  global interrupts are disabled, either explicitly or indirectly through
 *  interrupt handling logic.
 *
 ****************************************************************************/

static int mpfs_transmit(struct mpfs_driver_s *priv)
{
  uint32_t txb_id;
  bool ok;
  bool is_classical_can_frame;

  /* Retrieve the classical CAN / CANFD frame from network device buffer */

  is_classical_can_frame =
    priv->dev.d_len <= sizeof(struct can_frame) ? true : false;
  struct canfd_frame *cf = (struct canfd_frame *)priv->dev.d_buf;

  /* Get the current txt buffer ID */

  txb_id = priv->txb_sent % priv->ntxbufs;

  /* Insert classical CAN/CANFD frame into controller txt bf at txb_id */

  ok = mpfs_can_write_txb(priv, cf, txb_id, is_classical_can_frame);
  if (!ok)
    {
      canwarn("TXNF set but cannot insert frame into TXT buffer!\n");
      NETDEV_TXERRORS(&priv->dev);
      return OK;
    }

  /* Now, write to txt buffer seems ok, use txt command to set buffer state
   * to READY for xmit.
   */

  mpfs_can_send_txb_cmd(priv, TXT_CMD_SET_READY, txb_id);
  priv->txb_sent++;

  /* Increment statistics */

  NETDEV_TXPACKETS(&priv->dev);

  return OK;
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
 *   Zero (OK) on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int mpfs_txpoll(struct net_driver_s *dev)
{
  struct mpfs_driver_s *priv =
    (struct mpfs_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Send the packet */

      mpfs_transmit(priv);

      /* Check if there is room in the device to hold another packet. If
        * not, return a non-zero value to terminate the poll.
        */

      if (!MPFS_CAN_FD_TXNF(priv))
        {
          return -EBUSY;
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return OK;
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
  struct mpfs_driver_s *priv = (struct mpfs_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  if (priv->bifup)
    {
      /* Check if there is room in the controller to hold another outgoing
       * packet.
       */

      if (MPFS_CAN_FD_TXNF(priv))
        {
          /* Yes, there is, poll the network for new TXT transmit */

          net_lock();
          devif_poll(&priv->dev, mpfs_txpoll);
          net_unlock();
        }
    }
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
 *  0 - OK
 *
 * Assumptions:
 *  Called in normal user mode
 *
 ****************************************************************************/

static int mpfs_txavail(struct net_driver_s *dev)
{
  struct mpfs_driver_s *priv =
    (struct mpfs_driver_s *)dev->d_private;

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      mpfs_txavail_work(priv);
    }

  return OK;
}

/****************************************************************************
 * Name: mpfs_can_btr_compute
 *
 * Description:
 *  Calculate bit timing values to be written to bit timing register
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  bt    - Pointer to the bit timing structure to be set
 *  btc   - Pointer to the constant bit timing structure to be used to set
 *          the bit timing params
 *
 * Returned Value:
 *  Zero (OK) on success; a negated errno on failure
 *
 * Assumptions:
 *  Bit timing constants needed to be set in advance on which calculation
 *  in this function is based
 *
 ****************************************************************************/

static int
  mpfs_can_btr_compute(struct mpfs_driver_s *priv,
                       struct mpfs_can_bittiming_s *bt,
                       const struct mpfs_can_bittiming_const_s *btc)
{
  /* All measured in number of time quanta (Tq = brp / fsys)
   * brp : baud rate prescaler as number of system clock periods
   * prop_seg : propagation segment (1..8 Tq)
   * phase_seg1 : phase segment 1 (1..8 Tq)
   * phase_seg2 : phase segment 2 (1..8 Tq)
   *
   * tseg1 = prop_seg + phase_seg1
   * tseg2 = phase_seg2
   * tseg = tseg1 + tseg2
   * tsegall = BT_COMPUTE_SYNC_SEG + tseg / 2
   *
   * sample point =  (BT_COMPUTE_SYNC_SEG + tseg1) /
   * (BT_COMPUTE_SYNC_SEG + tseg1 + tseg2) * 1000
   *
   * CAN bitrate = 1 / (number_of_Tq * Tq)
   */

  unsigned int calc_br;                   /* current calculated bitrate */
  unsigned int br_err = 0;                /* diff between user set bitrate
                                           * and calculated bitrate */
  unsigned int best_br_err = UINT_MAX;    /* best bitrate error */

  unsigned int nominal_sp;                /* nominal sample point either set
                                           * by user or inferred from bitrate
                                           * (CiA recommendation) */
  unsigned int calc_sp;                   /* calculated sample point */
  unsigned int sp_err;                    /* diff between and nominal sample
                                           * point and currently calculated
                                           * sample point */
  unsigned int best_sp_err = UINT_MAX;    /* best sample point error */

  unsigned int calc_brp;                  /* currently calculated bitrate
                                           * prescaler */
  unsigned int best_brp = 0;              /* currently calculated bitrate
                                           * prescaler */
  unsigned int tseg_double;
  unsigned int tseg_sync;
  unsigned int tseg1 = 0;
  unsigned int tseg2 = 0;

  /* Get sample point nominal */

  if (bt->sample_point)
    {
      nominal_sp = bt->sample_point;
    }
  else
    {
      if (bt->bitrate <= 500000)
        {
          nominal_sp = 875;
        }
      else if (bt->bitrate <= 800000)
        {
          nominal_sp = 800;
        }
      else
        {
          nominal_sp = 750;
        }
    }
  calc_sp = nominal_sp;

  /* Iterate tseg in possible range to find best bit timing values */

  for (tseg_double = btc->tseg_max * 2 + 1;
        tseg_double >= btc->tseg_min * 2; tseg_double--)
    {
      tseg_sync = tseg_double / 2 + BT_COMPUTE_SYNC_SEG;

      /* Recalculate bitrate prescaler */

      calc_brp = priv->can.clock.freq / (tseg_sync * bt->bitrate) +
                 tseg_double % 2;

      if (calc_brp < btc->brp_min || calc_brp > btc->brp_max)
        {
          continue;
        }

      /* Recalculate bitrate and bitrate error */

      calc_br = priv->can.clock.freq / (calc_brp * tseg_sync);

      br_err = bt->bitrate - calc_br;
      if (br_err > best_br_err)
        {
          continue;
        }
      else
        {
          best_sp_err = UINT_MAX;
        }

      /* Now, it seems that we have a better bitrate, recalculate sample
       * point, tseg1, tseg2 and sample point error
       */

      tseg2 = clamp(tseg_sync - nominal_sp * tseg_sync / 1000, 1,
                    btc->tseg_max * 249 / 1000);
      tseg1 = tseg_double / 2 - tseg2;
      calc_sp = 1000 * (tseg_sync - tseg2) / tseg_sync;

      sp_err = nominal_sp - calc_sp;
      if (calc_sp > nominal_sp || sp_err > best_sp_err)
        {
          continue;
        }

      /* Update best values and end condition check */

      best_brp = calc_brp;
      best_br_err = br_err;
      best_sp_err = sp_err;

      if (best_br_err == 0 && best_sp_err == 0)
        {
          break;
        }
    }

  /* Check bitrate error against limit */

  if ((uint32_t)best_br_err * 1000 > BT_COMPUTE_MAX_ERROR * bt->bitrate)
    {
      canerr("Bitrate error %d.%d%% is too high\n", br_err / 10,
             br_err % 10);
      return -EDOM;
    }

  /* Retrieve the best calculated sample point */

  bt->sample_point = calc_sp;

  /* Retrieve bit timing register components */

  bt->brp = best_brp;
  bt->prop_seg = tseg1 / 2;
  bt->phase_seg1 = tseg1 - bt->prop_seg;
  bt->phase_seg2 = tseg2;

  return OK;
}

/****************************************************************************
 * Name: mpfs_can_config_bit_timing
 *
 * Description:
 *  Set CAN controller arbitration or data bitrate bit timing
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  bt      - Pointer to Bit timing structure
 *  arbi - True - Arbitration bit timing, False - Data bit timing
 *
 * Returned Value:
 *  Zero (OK) on success, -EPERM if CAN controller is not disabled
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int mpfs_can_config_bit_timing(struct mpfs_driver_s *priv,
                                  struct mpfs_can_bittiming_s *bt,
                                  bool arbi)
{
  int max_ph1_len = 31;
  uint32_t btr = 0;

  if (MPFS_CAN_FD_ENABLED(priv))
    {
      canerr("CAN controller should be disabled to set bit timing\n");
      return -EPERM;
    }

  if (arbi)
    {
     max_ph1_len = 63;
    }

  if (bt->phase_seg1 > max_ph1_len)
    {
      bt->prop_seg += bt->phase_seg1 - max_ph1_len;
      bt->phase_seg1 = max_ph1_len;
    }

  if (arbi)
    {
      btr = bt->prop_seg << MPFS_CANFD_BTR_PROP_SHIFT;
      btr |= bt->phase_seg1 << MPFS_CANFD_BTR_PH1_SHIFT;
      btr |= bt->phase_seg2 << MPFS_CANFD_BTR_PH2_SHIFT;
      btr |= bt->brp << MPFS_CANFD_BTR_BRP_SHIFT;
      btr |= bt->sjw << MPFS_CANFD_BTR_SJW_SHIFT;
      caninfo("Arbitration bitrate: %u, Prop_seg: %u, phase_seg1: %u, "
              "phase_seg2: %u, brp: %u, sjw: %u \n", bt->bitrate,
              bt->prop_seg, bt->phase_seg1, bt->phase_seg2, bt->brp,
              bt->sjw);
      putreg32(btr, priv->base + MPFS_CANFD_BTR_OFFSET);
    }
  else
    {
      btr = bt->prop_seg << MPFS_CANFD_BTR_FD_PROP_FD_SHIFT;
      btr |= bt->phase_seg1 << MPFS_CANFD_BTR_FD_PH1_FD_SHIFT;
      btr |= bt->phase_seg2 << MPFS_CANFD_BTR_FD_PH2_FD_SHIFT;
      btr |= bt->brp << MPFS_CANFD_BTR_FD_BRP_FD_SHIFT;
      btr |= bt->sjw << MPFS_CANFD_BTR_FD_SJW_FD_SHIFT;
      caninfo("Data bitrate: %u, Prop_seg: %u, phase_seg1: %u, "
              "phase_seg2: %u, brp: %u, sjw: %u \n", bt->bitrate,
              bt->prop_seg, bt->phase_seg1, bt->phase_seg2, bt->brp,
              bt->sjw);
      putreg32(btr, priv->base + MPFS_CANFD_BTR_FD_OFFSET);
    }

  return OK;
}

/****************************************************************************
 * Name: mpfs_can_config_arbi_bit_timing
 *
 * Description:
 *  Set CAN controller arbitration bit timing
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  Zero (OK) on success, -%EPERM if controller is enabled
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int mpfs_can_config_arbi_bit_timing(struct mpfs_driver_s *priv)
{
  struct mpfs_can_bittiming_s *arbi_bt = &priv->can.bittiming;

  /* Set bit timing for arbitration bit rate */

  return mpfs_can_config_bit_timing(priv, arbi_bt, true);
}

/****************************************************************************
 * Name: mpfs_can_config_data_bit_timing
 *
 * Description:
 *  Set CAN controller data bit timing
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  Zero (OK) on success, -%EPERM if controller is enabled
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int mpfs_can_config_data_bit_timing(struct mpfs_driver_s *priv)
{
  struct mpfs_can_bittiming_s *data_bt = &priv->can.data_bittiming;

  /* Set bit timing for data bit rate */

  return mpfs_can_config_bit_timing(priv, data_bt, false);
}

/****************************************************************************
 * Name: mpfs_can_config_ssp
 *
 * Description:
 *  Set CAN controller secondary sample point.
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  Zero (OK) on success, -EPERM if CAN controller is not disabled
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int mpfs_can_config_ssp(struct mpfs_driver_s *priv)
{
  struct mpfs_can_bittiming_s *dbt = &(priv->can.data_bittiming);
  int ssp_offset = 0;
  uint32_t ssp_cfg = 0;

  if (MPFS_CAN_FD_ENABLED(priv))
    {
      canerr("CAN controller should be disabled to set secondary sample "
             "point\n");
      return -EPERM;
    }

  /* secondary sample point is only used for bitrates > 1 Mbits/s */

  if (dbt->bitrate > 1000000)
    {
      ssp_offset = (priv->can.clock.freq / 1000) *
                   dbt->sample_point / dbt->bitrate;

      if (ssp_offset > 127)
        {
          canwarn("Secondary sample point offset exceeds 127\n");
          ssp_offset = 127;
        }

      ssp_cfg = ssp_offset << MPFS_CANFD_TRV_DELAY_SSP_OFFSET_SHIFT;
      ssp_cfg |= 0x1 << MPFS_CANFD_TRV_DELAY_SSP_SRC_SHIFT;
    }

  putreg32(ssp_cfg, priv->base + MPFS_CANFD_TRV_DELAY_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: mpfs_can_config_controller_mode
 *
 * Description:
 *  Configure CAN controller mode
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  mode  - Pointer to controller modes to be set
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void
  mpfs_can_config_controller_mode(struct mpfs_driver_s *priv,
                                  const struct mpfs_can_ctrlmode_s *mode)
{
  uint32_t mode_reg = getreg32(priv->base + MPFS_CANFD_MODE_OFFSET);

  mode_reg = (mode->flags & CAN_CTRLMODE_LOOPBACK) ?
    (mode_reg | MPFS_CANFD_MODE_ILBP) :
    (mode_reg & ~MPFS_CANFD_MODE_ILBP);

  mode_reg = (mode->flags & CAN_CTRLMODE_LISTENONLY) ?
    (mode_reg | MPFS_CANFD_MODE_BMM) :
    (mode_reg & ~MPFS_CANFD_MODE_BMM);

  mode_reg = (mode->flags & CAN_CTRLMODE_FD) ?
    (mode_reg | MPFS_CANFD_MODE_FDE) :
    (mode_reg & ~MPFS_CANFD_MODE_FDE);

  mode_reg = (mode->flags & CAN_CTRLMODE_PRESUME_ACK) ?
    (mode_reg | MPFS_CANFD_MODE_ACF) :
    (mode_reg & ~MPFS_CANFD_MODE_ACF);

  mode_reg = (mode->flags & CAN_CTRLMODE_FD_NON_ISO) ?
    (mode_reg | MPFS_CANFD_MODE_NISOFD) :
    (mode_reg & ~MPFS_CANFD_MODE_NISOFD);

  mode_reg &= ~MPFS_CANFD_MODE_RTRTH;
  mode_reg = (mode->flags & CAN_CTRLMODE_ONE_SHOT) ?
    (mode_reg | MPFS_CANFD_MODE_RTRLE) :
    (mode_reg & ~MPFS_CANFD_MODE_RTRLE);

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
  mode_reg |= MPFS_CANFD_MODE_AFM;
#endif

  /* Disable test mode */

  mode_reg &= ~MPFS_CANFD_MODE_TSTM;

  putreg32(mode_reg, priv->base + MPFS_CANFD_MODE_OFFSET);
}

/****************************************************************************
 * Name: mpfs_can_add_hw_filter
 *
 * Description:
 *  Add new hw filter to CAN Controller
 *
 * Input Parameters:
 *  priv          - Pointer to the private CAN driver state structure
 *  filter_type   - Filter A, B, C or Range
 *  can_id_type   - std CAN ID / ext CAN ID
 *  can_type      - classical CAN / CANFD
 *  filter_id1    - filter id 1 (can be filter value for bit filter or range
 * low for range filter)
 *  filter_id2    - filter id 2 (can be filter mask for bit filter or range
 * high for range filter)
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
static void mpfs_can_add_hw_filter(struct mpfs_driver_s *priv,
                                   uint8_t filter_type,
                                   uint8_t can_id_type,
                                   uint8_t can_type,
                                   uint32_t fid1,
                                   uint32_t fid2)
{
  uint32_t fc_reg;

  fc_reg = getreg32(priv->base + MPFS_CANFD_FILTER_CONTROL_OFFSET);

  /* Clean up filter control reg */

  fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FAFE;
  fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FAFB;
  fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FANE;
  fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FANB;

  /* Transform fid1, fid2 */

  uint32_t fid1_trans = (can_id_type == CAN_EXT_ID) ?
    (MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_EXT &
            (fid1 << MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_EXT_SHIFT)) :
    (MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE &
            (fid1 << MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE_SHIFT));

  uint32_t fid2_trans = (can_id_type == CAN_EXT_ID) ?
    (MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_EXT &
            (fid2 << MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_EXT_SHIFT)) :
    (MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE &
            (fid2 << MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE_SHIFT));

  switch (filter_type)
    {
      case HW_FILTER_A:

        /* Set filter control reg for filter A */

        if (can_type == CAN_MSGPRIO_LOW)
          {
            /* Classical CAN frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FANE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FANB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FAFE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FAFB;
          }
        else if (can_type == CAN_MSGPRIO_HIGH)
          {
            /* CANFD frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FAFE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FAFB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FANE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FANB;
          }
        putreg32(fc_reg, priv->base + MPFS_CANFD_FILTER_CONTROL_OFFSET);

        /* Set bit filter value / mask */

        putreg32(fid1_trans, priv->base + MPFS_CANFD_FILTER_A_VAL_OFFSET);
        putreg32(fid2_trans, priv->base + MPFS_CANFD_FILTER_A_MASK_OFFSET);
        break;

      case HW_FILTER_B:

        /* Set filter control reg for filter B */

        if (can_type == CAN_MSGPRIO_LOW)
          {
            /* Classical CAN frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FBNE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FBNB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FBFE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FBFB;
          }
        else if (can_type == CAN_MSGPRIO_HIGH)
          {
            /* CANFD frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FBFE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FBFB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FBNE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FBNB;
          }
        putreg32(fc_reg, priv->base + MPFS_CANFD_FILTER_CONTROL_OFFSET);

        /* Set bit filter value / mask */

        putreg32(fid1_trans, priv->base + MPFS_CANFD_FILTER_B_VAL_OFFSET);
        putreg32(fid2_trans, priv->base + MPFS_CANFD_FILTER_B_MASK_OFFSET);
        break;

      case HW_FILTER_C:

        /* Set filter control reg for filter C */

        if (can_type == CAN_MSGPRIO_LOW)
          {
            /* Classical CAN frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FCNE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FCNB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FCFE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FCFB;
          }
        else if (can_type == CAN_MSGPRIO_HIGH)
          {
            /* CANFD frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FCFE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FCFB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FCNE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FCNB;
          }
        putreg32(fc_reg, priv->base + MPFS_CANFD_FILTER_CONTROL_OFFSET);

        /* Set bit filter value / mask */

        putreg32(fid1_trans, priv->base + MPFS_CANFD_FILTER_C_VAL_OFFSET);
        putreg32(fid2_trans, priv->base + MPFS_CANFD_FILTER_C_MASK_OFFSET);
        break;

      case HW_FILTER_RANGE:

        /* Set filter control reg for filter range */

        if (can_type == CAN_MSGPRIO_LOW)
          {
            /* Classical CAN frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FRNE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FRNB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FRFE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FRFB;
          }
        else if (can_type == CAN_MSGPRIO_HIGH)
          {
            /* CANFD frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FRFE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FRFB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FRNE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FRNB;
          }
        putreg32(fc_reg, priv->base + MPFS_CANFD_FILTER_CONTROL_OFFSET);

        /* Set range filter low / high */

        putreg32(fid1_trans, priv->base + MPFS_CANFD_FILTER_RAN_LOW_OFFSET);
        putreg32(fid2_trans, priv->base + MPFS_CANFD_FILTER_RAN_HIGH_OFFSET);
        break;

      default:

        /* Unsupported filter type */

        break;
    }
}
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

/****************************************************************************
 * Name: mpfs_can_reset_hw_filter
 *
 * Description:
 *  Reset all hw filters (both bit and range filter) to default settings
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
#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
static void mpfs_can_reset_hw_filter(struct mpfs_driver_s *priv)
{
  uint32_t reg;

  /* Reset filter control */

  reg = 0;
  reg |= MPFS_CANFD_FILTER_CONTROL_FANB;
  reg |= MPFS_CANFD_FILTER_CONTROL_FANE;
  reg |= MPFS_CANFD_FILTER_CONTROL_FAFB;
  reg |= MPFS_CANFD_FILTER_CONTROL_FAFE;
  putreg32(reg, priv->base + MPFS_CANFD_FILTER_CONTROL_OFFSET);

  /* Reset bit filter A */

  putreg32(0, priv->base + MPFS_CANFD_FILTER_A_VAL_OFFSET);
  putreg32(0, priv->base + MPFS_CANFD_FILTER_A_MASK_OFFSET);

  /* Reset bit filter B */

  putreg32(0, priv->base + MPFS_CANFD_FILTER_B_VAL_OFFSET);
  putreg32(0, priv->base + MPFS_CANFD_FILTER_B_MASK_OFFSET);

  /* Reset bit filter C */

  putreg32(0, priv->base + MPFS_CANFD_FILTER_C_VAL_OFFSET);
  putreg32(0, priv->base + MPFS_CANFD_FILTER_C_MASK_OFFSET);

  /* Reset range filter */

  putreg32(0, priv->base + MPFS_CANFD_FILTER_RAN_LOW_OFFSET);
  putreg32(0, priv->base + MPFS_CANFD_FILTER_RAN_HIGH_OFFSET);

  priv->used_bit_filter_number = 0;
  priv->used_range_filter = false;
}
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

/****************************************************************************
 * Name: mpfs_can_controller_start
 *
 * Description:
 *  This routine starts the driver. Routine expects that controller is in
 *  reset state. It setups initial Tx buffers for FIFO priorities, sets
 *  bittiming, enables interrupts, switches core to operational mode and
 *  changes controller state to %CAN_STATE_STOPPED.
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  Zero (OK) on success and failure value on error
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int mpfs_can_controller_start(struct mpfs_driver_s *priv)
{
  uint32_t int_ena;
  uint32_t int_msk;
  uint32_t mode_reg;
  int err;
  struct mpfs_can_ctrlmode_s mode;

  /* Initialize TXT buffer sent / processed counter values */

  priv->txb_sent = 0;
  priv->txb_processed = 0;

  /* Configure TXT buffers priority */

  priv->txb_prio = 0x01234567;
  putreg32(priv->base, priv->base + MPFS_CANFD_TX_PRIORITY_OFFSET);

  /* Configure bit-rates and ssp */

  err = mpfs_can_config_arbi_bit_timing(priv);
  if (err < 0)
    {
     return err;
    }

  err = mpfs_can_config_data_bit_timing(priv);
  if (err < 0)
    {
      return err;
    }

  err = mpfs_can_config_ssp(priv);
  if (err < 0)
    {
      return err;
    }

  /* Configure modes */

  mode.flags = priv->can.ctrlmode;
  mode.mask = 0xffffffff;
  mpfs_can_config_controller_mode(priv, &mode);

  /* Configure interrupts */

  int_ena = MPFS_CANFD_INT_STAT_RBNEI |
            MPFS_CANFD_INT_STAT_TXBHCI |
            MPFS_CANFD_INT_STAT_EWLI |
            MPFS_CANFD_INT_STAT_FCSI |
            MPFS_CANFD_INT_STAT_DOI;

  /* Bus error reporting -> Allow Error/Arb.lost interrupts */

  if (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING)
    {
      int_ena |= MPFS_CANFD_INT_STAT_ALI | MPFS_CANFD_INT_STAT_BEI;
    }

  int_msk = ~int_ena; /* Mask all disabled interrupts */

  /* It's after reset, so there is no need to clear anything */

  uint32_t mask = 0xffffffff;
  putreg32(mask, priv->base + MPFS_CANFD_INT_MASK_CLR_OFFSET);
  putreg32(int_msk, priv->base + MPFS_CANFD_INT_MASK_SET_OFFSET);
  putreg32(int_ena, priv->base + MPFS_CANFD_INT_ENA_SET_OFFSET);

  /* Put CAN driver to STOPPED state first, CAN controller will enters
   * ERROR_ACTIVE on initial FCSI
   */

  priv->can.state = CAN_STATE_STOPPED;

  /* Enable the CAN controller */

  mode_reg = getreg32(priv->base + MPFS_CANFD_MODE_OFFSET);
  mode_reg |= MPFS_CANFD_MODE_ENA;
  putreg32(mode_reg, priv->base + MPFS_CANFD_MODE_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: mpfs_can_controller_stop
 *
 * Description:
 *  This routine stops the driver. This is the drivers stop routine. It will
 * disable the interrupts and disable the CAN controller
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

static void mpfs_can_controller_stop(struct mpfs_driver_s *priv)
{
  uint32_t mask = 0xffffffff;
  uint32_t mode;

  /* Disable interrupts */

  putreg32(mask, priv->base + MPFS_CANFD_INT_ENA_CLR_OFFSET);
  putreg32(mask, priv->base + MPFS_CANFD_INT_MASK_SET_OFFSET);

  /* Disable the CAN controller */

  mode = getreg32(priv->base + MPFS_CANFD_MODE_OFFSET);
  mode &= ~MPFS_CANFD_MODE_ENA;
  putreg32(mode, priv->base + MPFS_CANFD_MODE_OFFSET);

  /* Set CAN driver state to STOPPED */

  priv->can.state = CAN_STATE_STOPPED;
}

/****************************************************************************
 * Name: mpfs_reset
 *
 * Description:
 *  Put the EMAC in the non-operational, reset state
 *
 * Input Parameters:
 *  priv - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  OK for success and -ETIMEDOUT for failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_reset(struct mpfs_driver_s *priv)
{
  uint32_t i = 100;
  uint32_t device_id;

  /* Reset FPGA and FIC3, enable clock for FIC3 before RD WR */

  modifyreg32(MPFS_SYSREG_SOFT_RESET_CR,
              SYSREG_SOFT_RESET_CR_FPGA | SYSREG_SOFT_RESET_CR_FIC3,
              0);
  modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR, 0, SYSREG_SUBBLK_CLOCK_CR_FIC3);

  /* Reset CAN controller */

  putreg32(MPFS_CANFD_MODE_RST, priv->base + MPFS_CANFD_MODE_OFFSET);

  /* Check if the device is up again */

  do
    {
      device_id = (getreg32(priv->base + MPFS_CANFD_DEVICE_ID_OFFSET) &
        MPFS_CANFD_DEVICE_ID_DEVICE_ID) >>
        MPFS_CANFD_DEVICE_ID_DEVICE_ID_SHIFT;

      if (device_id == MPFS_CANFD_ID)
        {
          return OK;
        }

      if (!i--)
        {
          canwarn("Device did not leave reset\n");
          return -ETIMEDOUT;
        }

      nxsig_usleep(200);
    }
  while (1);
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
 *  Zero (OK) on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_ifup(struct net_driver_s *dev)
{
  struct mpfs_driver_s *priv =
    (struct mpfs_driver_s *)dev->d_private;

  if (mpfs_can_controller_start(priv) < 0)
    {
      canerr("CAN controller start failed\n");
      return -1;
    }

  priv->bifup = true;

  priv->txdesc = (struct canfd_frame *)&g_tx_pool;
  priv->rxdesc = (struct canfd_frame *)&g_rx_pool;

  priv->dev.d_buf = (uint8_t *)priv->txdesc;

  /* Set interrupts */

  up_enable_irq(priv->config->canfd_fpga_irq);

  return OK;
}

/****************************************************************************
 * Name: mpfs_ifdown
 *
 * Description:
 *  NuttX Callback: Stop the CAN interface.
 *
 * Input Parameters:
 *  dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *  Zero (OK) on success; a negated errno value on failure
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int mpfs_ifdown(struct net_driver_s *dev)
{
  struct mpfs_driver_s *priv =
    (struct mpfs_driver_s *)dev->d_private;

  mpfs_can_controller_stop(priv);

  priv->bifup = false;
  return OK;
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
 *  Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int mpfs_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
#if defined(CONFIG_NETDEV_CAN_BITRATE_IOCTL) || \
defined(CONFIG_NETDEV_CAN_FILTER_IOCTL)
  struct mpfs_driver_s *priv =
    (struct mpfs_driver_s *)dev->d_private;
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
        req->arbi_bitrate = priv->can.bittiming.bitrate / 1000; /* kbit/s */
        req->arbi_samplep = priv->can.bittiming.sample_point / 10;
        req->data_bitrate = priv->can.data_bittiming.bitrate / 1000; /* kbit/s */
        req->data_samplep = priv->can.data_bittiming.sample_point / 10;
        ret = OK;
      }
      break;

    case SIOCSCANBITRATE:

      /* Set bitrate of the CAN controller */

      {
        struct can_ioctl_data_s *req =
          (struct can_ioctl_data_s *)((uintptr_t)arg);

        if (req->arbi_bitrate > 1000)
          {
            canerr("Arbitration bitrate > 1Mbps is not supported.");
            ret = -EINVAL;
            break;
          }
        priv->can.bittiming.bitrate = req->arbi_bitrate * 1000; /* bit/s */

        if (req->arbi_samplep > 100 || req->arbi_samplep <= 0)
          {
            canerr("Invalid arbitration sample point. "
                   "Range should be (0,100]%%.");
            ret = -EINVAL;
            break;
          }
        priv->can.bittiming.sample_point =
          req->arbi_samplep * 10; /* In one-tenth of a percent */

        if (req->data_bitrate > 4000 ||
            req->data_bitrate < req->arbi_bitrate)
          {
            canerr("Data bitrate higher than 4Mbps is yet to be supported. "
                   "Data br cannot be smaller than arbitration br.");
            ret = -EINVAL;
            break;
          }
        priv->can.data_bittiming.bitrate = req->data_bitrate * 1000; /* bit/s */

        if (req->data_samplep > 100 || req->data_samplep <= 0)
          {
            canerr("Invalid data sample point. Range should be (0,100]%%.");
            ret = -EINVAL;
            break;
          }
        priv->can.data_bittiming.sample_point =
          req->data_samplep * 10; /* In one-tenth of a percent */

        /* Calculate arbitration and data bit timing */

        mpfs_can_btr_compute(priv,
                             &priv->can.bittiming,
                             priv->can.bittiming_const);
        mpfs_can_btr_compute(priv,
                             &priv->can.data_bittiming,
                             priv->can.data_bittiming_const);

        /* CAN controller reset to write bit timing register */

        mpfs_can_controller_stop(priv);
        if (mpfs_can_controller_start(priv) < 0)
          {
            canerr("CAN controller start failed.");
            ret = -1;
            break;
          }

        ret = OK;
      }
      break;
#endif /* CONFIG_NETDEV_CAN_BITRATE_IOCTL */

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
    case SIOCACANSTDFILTER:

      {
        uint8_t filter;
        struct can_ioctl_filter_s *req =
          (struct can_ioctl_filter_s *)((uintptr_t)arg);

        if (req->ftype == CAN_FILTER_MASK)
          {
              if (priv->used_bit_filter_number == 0)
            {
              /* Use bit filter A */

              filter = HW_FILTER_A;
            }
          else if (priv->used_bit_filter_number == 1)
            {
              /* Use bit filter B */

              filter = HW_FILTER_B;
            }
          else if (priv->used_bit_filter_number == 2)
            {
              /* Use bit filter C */

              filter = HW_FILTER_C;
            }
          else
            {
              /* All bit filters used. Return with error */

              canerr("All bit filters used. Cannot add more. Delete all and "
                     "add again.");
              ret = -1;
              break;
            }
          priv->used_bit_filter_number++;
          }
        else if (req->ftype == CAN_FILTER_RANGE)
          {
            /* Use range filter */

            filter = HW_FILTER_RANGE;
            priv->used_range_filter = true;
          }
        else
          {
            /* Dual address and other filter types not yet support */

            canerr("Dual address and other filter types not yet support");
            ret = -1;
            break;
          }

        /* Add hw filter */

        mpfs_can_add_hw_filter(priv,
                              filter,
                              CAN_STD_ID,
                              req->fprio, /* LOW for CAN, HIGH for FDCAN */
                              req->fid1,
                              req->fid2);

        ret = OK;
      }
      break;

    case SIOCACANEXTFILTER:

      /* Add CAN EXTENDED ID HW FILTER */

      {
        uint8_t filter;
        struct can_ioctl_filter_s *req =
          (struct can_ioctl_filter_s *)((uintptr_t)arg);

        if (req->ftype == CAN_FILTER_MASK)
          {
              if (priv->used_bit_filter_number == 0)
            {
              /* Use bit filter A */

              filter = HW_FILTER_A;
            }
          else if (priv->used_bit_filter_number == 1)
            {
              /* Use bit filter B */

              filter = HW_FILTER_B;
            }
          else if (priv->used_bit_filter_number == 2)
            {
              /* Use bit filter C */

              filter = HW_FILTER_C;
            }
          else
            {
              /* All bit filters used. Return with error */

              canerr("All bit filters used. Cannot add more. Delete all and "
                     "add again.");
              ret = -1;
              break;
            }
          priv->used_bit_filter_number++;
          }
        else if (req->ftype == CAN_FILTER_RANGE)
          {
            /* Use range filter */

            if (priv->used_range_filter)
              {
                /* The range filter is already used. Return with error */

                canerr("Range filter used. Cannot add more. Delete all and "
                       "add again.");
                ret = -1;
                break;
              }
            filter = HW_FILTER_RANGE;
            priv->used_range_filter = true;
          }
        else
          {
            /* Dual address and other filter types not yet support */

            canerr("Dual address and other filter types not yet support");
            ret = -1;
            break;
          }

        /* Add hw filter */

        mpfs_can_add_hw_filter(priv,
                              filter,
                              CAN_EXT_ID,
                              req->fprio, /* CAN Type: LOW for CAN, HIGH for FDCAN */
                              req->fid1,
                              req->fid2);

        ret = OK;
      }
      break;

    case SIOCDCANSTDFILTER:
    case SIOCDCANEXTFILTER:

      /* Reset all HW FILTERs */

      {
        mpfs_can_reset_hw_filter(priv);
        ret = OK;
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
 * Name: mpfs_fpga_canfd_init
 *
 * Description:
 *  Initialize the CAN controller and driver
 *
 * Returned Value:
 *  On success, a pointer to the MPFS CAN-FD driver is
 *  returned. NULL is returned on any failure.
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

int mpfs_fpga_canfd_init(void)
{
  caninfo("Initialize CAN-FD driver...\n");
  struct mpfs_driver_s *priv;
  priv         = &g_canfd;
  memset(priv, 0, sizeof(struct mpfs_driver_s));

  priv->base   = CONFIG_MPFS_CANFD_BASE;
  priv->config = &mpfs_fpga_canfd_config;

  /* Initialize the CAN common private data structure */

  priv->can.state = CAN_STATE_ERROR_ACTIVE;
  priv->ntxbufs = 2;
  priv->can.bittiming_const = &mpfs_can_bit_timing_range;
  priv->can.data_bittiming_const = &mpfs_can_bit_timing_data_range;

  /* Get the can_clk info */

  priv->can.clock.freq = CONFIG_MPFS_CANFD_CLK;

  /* Needed for timing adjustment to be performed as soon as possible */

  priv->can.bittiming.bitrate = CONFIG_MPFS_CANFD_ARBI_BITRATE;
  priv->can.data_bittiming.bitrate = CONFIG_MPFS_CANFD_DATA_BITRATE;
  priv->can.bittiming.sjw = 5;
  priv->can.data_bittiming.sjw = 5;

  /* Calculate nominal and data bit timing */

  mpfs_can_btr_compute(priv,
                          &priv->can.bittiming,
                          priv->can.bittiming_const);
  mpfs_can_btr_compute(priv,
                          &priv->can.data_bittiming,
                          priv->can.data_bittiming_const);

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
  /* Init hw filter runtime var */

  priv->used_bit_filter_number = 0;
  priv->used_range_filter = false;
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

  /* Set CAN control modes */

  priv->can.ctrlmode = CAN_CTRLMODE_FD
                      | CAN_CTRLMODE_BERR_REPORTING;

  /* Attach the interrupt handler */

  if (irq_attach(priv->config->canfd_fpga_irq, mpfs_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      canerr("ERROR: Failed to attach to CAN IRQ\n");
      return -EAGAIN;
    }

  /* Initialize the driver structure */

  priv->dev.d_ifup    = mpfs_ifup;    /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = mpfs_ifdown;  /* I/F down callback */
  priv->dev.d_txavail = mpfs_txavail; /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = mpfs_ioctl;   /* Support CAN ioctl() calls */
#endif
  priv->dev.d_private = priv;         /* Used to recover private state from dev */

  /* Reset controller */

  if (mpfs_reset(priv) < 0)
    {
      return -1;
    }

  caninfo("CAN-FD driver init done\n");

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling mpfs_ifdown().
   */

  mpfs_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_CAN);

  return OK;
}

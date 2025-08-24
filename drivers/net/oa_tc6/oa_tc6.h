/****************************************************************************
 * drivers/net/oa_tc6/oa_tc6.h
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

#ifndef __DRIVERS_NET_OA_TC6_H
#define __DRIVERS_NET_OA_TC6_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/spi/spi.h>

#include <nuttx/wqueue.h>
#include <nuttx/mutex.h>

#include <net/if.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/net/oa_tc6.h>

#include <nuttx/bits.h>

#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device-specific driver ioctl return value if the cmd is not implemented */

#define OA_TC6_IOCTL_CMD_NOT_IMPLEMENTED -ENOSYS

/* NuttX SPI mode number for SPI config as defined in OPEN Alliance TC6 */

#define OA_TC6_SPI_MODE 0

/* Number of bits in a SPI word */

#define OA_TC6_SPI_NBITS 8

#define OA_TC6_CHUNK_SIZE(dev) ((dev)->config->chunk_payload_size + 4)

typedef uint32_t oa_tc6_regid_t;

#define OA_TC6_MAKE_REGID(mms, addr) \
    (((uint32_t)(mms) << 16) | ((uint32_t)(addr) & 0xFFFF))

#define OA_TC6_REGID_GET_MMS(regid) ((uint8_t)((regid >> 16) & 0xF))
#define OA_TC6_REGID_GET_ADDR(regid) ((uint16_t)(regid & 0xFFFF))

#define OA_TC6_IDVER_MMS              0
#define OA_TC6_IDVER_ADDR             0x0000U
#define OA_TC6_IDVER_REGID            OA_TC6_MAKE_REGID(OA_TC6_IDVER_MMS, OA_TC6_IDVER_ADDR)

#define OA_TC6_PHYID_MMS              0
#define OA_TC6_PHYID_ADDR             0x0001U
#define OA_TC6_PHYID_REGID            OA_TC6_MAKE_REGID(OA_TC6_PHYID_MMS, OA_TC6_PHYID_ADDR)
#define OA_TC6_PHYID_OUI_MASK         GENMASK(31, 10)
#define OA_TC6_PHYID_OUI_POS          10
#define OA_TC6_PHYID_MODEL_MASK       GENMASK(9, 4)
#define OA_TC6_PHYID_MODEL_POS        4
#define OA_TC6_PHYID_REV_MASK         GENMASK(3, 0)
#define OA_TC6_PHYID_REV_POS          0

#define OA_TC6_STDCAP_MMS             0
#define OA_TC6_STDCAP_ADDR            0x0002U
#define OA_TC6_STDCAP_REGID           OA_TC6_MAKE_REGID(OA_TC6_STDCAP_MMS, OA_TC6_STDCAP_ADDR)
#define OA_TC6_STDCAP_DPRAC_MASK      BIT(8)
#define OA_TC6_STDCAP_DPRAC_POS       8
#define OA_TC6_STDCAP_CTC_MASK        BIT(7)
#define OA_TC6_STDCAP_CTC_POS         7
#define OA_TC6_STDCAP_MINCPS_MASK     GENMASK(2, 0)
#define OA_TC6_STDCAP_MINCPS_POS      0

#define OA_TC6_RESET_MMS              0
#define OA_TC6_RESET_ADDR             0x0003U
#define OA_TC6_RESET_REGID            OA_TC6_MAKE_REGID(OA_TC6_RESET_MMS, OA_TC6_RESET_ADDR)
#define OA_TC6_RESET_SWRESET_MASK     BIT(0)
#define OA_TC6_RESET_SWRESET_POS      0

#define OA_TC6_CONFIG0_MMS            0
#define OA_TC6_CONFIG0_ADDR           0x0004U
#define OA_TC6_CONFIG0_REGID          OA_TC6_MAKE_REGID(OA_TC6_CONFIG0_MMS, OA_TC6_CONFIG0_ADDR)
#define OA_TC6_CONFIG0_SYNC_MASK      BIT(15)
#define OA_TC6_CONFIG0_SYNC_POS       15
#define OA_TC6_CONFIG0_TXFCSVE_MASK   BIT(14)
#define OA_TC6_CONFIG0_TXFCSVE_POS    14
#define OA_TC6_CONFIG0_CSARFE_MASK    BIT(13)
#define OA_TC6_CONFIG0_CSARFE_POS     13
#define OA_TC6_CONFIG0_ZARFE_MASK     BIT(12)
#define OA_TC6_CONFIG0_ZARFE_POS      12
#define OA_TC6_CONFIG0_TXCTHRESH_MASK GENMASK(11, 10)
#define OA_TC6_CONFIG0_TXCTHRESH_POS  10
#define OA_TC6_CONFIG0_TXCTE_MASK     BIT(9)
#define OA_TC6_CONFIG0_TXCTE_POS      9
#define OA_TC6_CONFIG0_RXCTE_MASK     BIT(8)
#define OA_TC6_CONFIG0_RXCTE_POS      8
#define OA_TC6_CONFIG0_FTSE_MASK      BIT(7)
#define OA_TC6_CONFIG0_FTSE_POS       7
#define OA_TC6_CONFIG0_FTSS_MASK      BIT(6)
#define OA_TC6_CONFIG0_FTSS_POS       6
#define OA_TC6_CONFIG0_PROTE_MASK     BIT(5)
#define OA_TC6_CONFIG0_PROTE_POS      5
#define OA_TC6_CONFIG0_SEQE_MASK      BIT(4)
#define OA_TC6_CONFIG0_SEQE_POS       4
#define OA_TC6_CONFIG0_CPS_MASK       GENMASK(2, 0)
#define OA_TC6_CONFIG0_CPS_POS        0
#define OA_TC6_CONFIG0_CPS_64         6
#define OA_TC6_CONFIG0_CPS_32         5
#define OA_TC6_CONFIG0_CPS_16         4
#define OA_TC6_CONFIG0_CPS_8          3

#define OA_TC6_STATUS0_MMS            0
#define OA_TC6_STATUS0_ADDR           0x0008U
#define OA_TC6_STATUS0_REGID          OA_TC6_MAKE_REGID(OA_TC6_STATUS0_MMS, OA_TC6_STATUS0_ADDR)
#define OA_TC6_STATUS0_RESETC_MASK    BIT(6)
#define OA_TC6_STATUS0_RESETC_POS     6
#define OA_TC6_STATUS0_HDRE_MASK      BIT(5)
#define OA_TC6_STATUS0_HDRE_POS       5

#define OA_TC6_BUFSTS_MMS             0
#define OA_TC6_BUFSTS_ADDR            0x000BU
#define OA_TC6_BUFSTS_REGID           OA_TC6_MAKE_REGID(OA_TC6_BUFSTS_MMS, OA_TC6_BUFSTS_ADDR)

#define OA_TC6_IMSK0_MMS              0
#define OA_TC6_IMSK0_ADDR             0x000CU
#define OA_TC6_IMSK0_REGID            OA_TC6_MAKE_REGID(OA_TC6_IMSK0_MMS, OA_TC6_IMSK0_ADDR)
#define OA_TC6_IMSK0_DEF              0x1FBFU
#define OA_TC6_IMSK0_PHYINTM_MASK     BIT(7)
#define OA_TC6_IMSK0_PHYINTM_POS      7
#define OA_TC6_IMSK0_RXBOEM_MASK      BIT(3)
#define OA_TC6_IMSK0_RXBOEM_POS       3

#define OA_TC6_PHY_CONTROL_MMS        0
#define OA_TC6_PHY_CONTROL_ADDR       0xFF00U
#define OA_TC6_PHY_CONTROL_REGID      OA_TC6_MAKE_REGID(OA_TC6_PHY_CONTROL_MMS, OA_TC6_PHY_CONTROL_ADDR)

#define OA_TC6_PHY_STATUS_MMS         0
#define OA_TC6_PHY_STATUS_ADDR        0xFF01U
#define OA_TC6_PHY_STATUS_REGID       OA_TC6_MAKE_REGID(OA_TC6_PHY_STATUS_MMS, OA_TC6_PHY_STATUS_ADDR)

/* MDIO */

#define OA_TC6_MII_BASE_MMS           0
#define OA_TC6_MII_BASE_ADDR          0xFF00U

#define OA_TC6_MMD_29_BASE_MMS        0
#define OA_TC6_MMD_29_BASE_ADDR       0xFF20U

#define OA_TC6_MMD_3_BASE_MMS         2
#define OA_TC6_MMD_3_BASE_ADDR        0x0000U

#define OA_TC6_MMD_1_BASE_MMS         3
#define OA_TC6_MMD_1_BASE_ADDR        0x0000U

#define OA_TC6_MMD_31_BASE_MMS        4
#define OA_TC6_MMD_31_BASE_ADDR       0x0000U

#define OA_TC6_MMD_7_BASE_MMS         5
#define OA_TC6_MMD_7_BASE_ADDR        0x0000U

#define OA_TC6_MMD_13_BASE_MMS        6
#define OA_TC6_MMD_13_BASE_ADDR       0x0000U

/* OA Data Transaction and Control Transaction protocols bitfields */

/* Common bitfields */

#define OA_TC6_DNC_MASK  BIT(31)
#define OA_TC6_DNC_POS   31

#define OA_TC6_HDRB_MASK BIT(30)
#define OA_TC6_HDRB_POS  30

#define OA_TC6_VS_MASK   GENMASK(23, 22)
#define OA_TC6_VS_POS    22

#define OA_TC6_DV_MASK   BIT(21)
#define OA_TC6_DV_POS    21

#define OA_TC6_SV_MASK   BIT(20)
#define OA_TC6_SV_POS    20

#define OA_TC6_SWO_MASK  GENMASK(19, 16)
#define OA_TC6_SWO_POS   16

#define OA_TC6_EV_MASK   BIT(14)
#define OA_TC6_EV_POS    14

#define OA_TC6_EBO_MASK  GENMASK(13, 8)
#define OA_TC6_EBO_POS   8

#define OA_TC6_P_MASK    BIT(0)
#define OA_TC6_P_POS     0

/* Control Transaction Protocol header bitfields */

#define OA_TC6_WNR_MASK  BIT(29)
#define OA_TC6_WNR_POS   29

#define OA_TC6_AID_MASK  BIT(28)
#define OA_TC6_AID_POS   28

#define OA_TC6_MMS_MASK  GENMASK(27, 24)
#define OA_TC6_MMS_POS   24

#define OA_TC6_ADDR_MASK GENMASK(23, 8)
#define OA_TC6_ADDR_POS  8

#define OA_TC6_LEN_MASK  GENMASK(7, 1)
#define OA_TC6_LEN_POS   1

/* Transmit data header bitfields */

#define OA_TC6_SEQ_MASK  BIT(30)
#define OA_TC6_SEQ_POS   30

#define OA_TC6_NORX_MASK BIT(29)
#define OA_TC6_NORX_POS  29

#define OA_TC6_TSC_MASK  GENMASK(7, 6)
#define OA_TC6_TSC_POS   6

/* Receive data footer bitfields */

#define OA_TC6_EXST_MASK BIT(31)
#define OA_TC6_EXST_POS  31

#define OA_TC6_SYNC_MASK BIT(29)
#define OA_TC6_SYNC_POS  29

#define OA_TC6_RCA_MASK  GENMASK(28, 24)
#define OA_TC6_RCA_POS   24

#define OA_TC6_FD_MASK   BIT(15)
#define OA_TC6_FD_POS    15

#define OA_TC6_RTSA_MASK BIT(7)
#define OA_TC6_RTSA_POS  7

#define OA_TC6_RTSP_MASK BIT(6)
#define OA_TC6_RTSP_POS  6

#define OA_TC6_TXC_MASK  GENMASK(5, 1)
#define OA_TC6_TXC_POS   1

/* General macro for extracting fields from OA registers */

#define oa_tc6_get_field(r, fieldname) \
    (((r) & OA_TC6_##fieldname##_MASK) >> OA_TC6_##fieldname##_POS)

/* Helper macros for extracting control fields from footers/headers */

#define oa_tc6_tx_credits(f)                oa_tc6_get_field(f, TXC)
#define oa_tc6_rx_available(f)              oa_tc6_get_field(f, RCA)
#define oa_tc6_header_bad(f)                oa_tc6_get_field(f, HDRB)
#define oa_tc6_ext_status(f)                oa_tc6_get_field(f, EXST)
#define oa_tc6_data_valid(f)                oa_tc6_get_field(f, DV)
#define oa_tc6_start_valid(f)               oa_tc6_get_field(f, SV)
#define oa_tc6_start_word_offset(f)         oa_tc6_get_field(f, SWO)
#define oa_tc6_end_valid(f)                 oa_tc6_get_field(f, EV)
#define oa_tc6_end_byte_offset(f)           oa_tc6_get_field(f, EBO)
#define oa_tc6_frame_drop(f)                oa_tc6_get_field(f, FD)
#define oa_tc6_rx_frame_timestamp_added(f)  oa_tc6_get_field(f, RTSA)
#define oa_tc6_rx_frame_timestamp_parity(f) oa_tc6_get_field(f, RTSP)
#define oa_tc6_mac_phy_sync(f)              oa_tc6_get_field(f, SYNC)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum oa_tc6_ifstate_e
{
  OA_TC6_IFSTATE_RESET,        /* The i/f is not configured after reset    */
  OA_TC6_IFSTATE_DOWN,         /* The i/f is configured, but disabled      */
  OA_TC6_IFSTATE_DOWN_UNKNOWN, /* The i/f is down after config is lost     */
  OA_TC6_IFSTATE_UP,           /* The i/f is configured and enabled        */
  OA_TC6_IFSTATE_UP_RECOVERY,  /* The i/f is enabled, but SPI lost contact */
  OA_TC6_IFSTATE_UP_UNKNOWN,   /* The i/f is still up after config is lost */
};

enum oa_tc6_action_e
{
  OA_TC6_ACTION_CONFIG,      /* Called before OA generic config           */
  OA_TC6_ACTION_ENABLE,      /* Called to perform device-specific enable  */
  OA_TC6_ACTION_DISABLE,     /* Called to perform device-specific disable */
  OA_TC6_ACTION_EXST,        /* Called when EXST is detected in footer    */
  OA_TC6_ACTION_N            /* Number of elements in this enum           */
};

struct oa_tc6_driver_s;
struct oa_tc6_ops_s
{
  /* action - perform one of oa_tc6_action_e device-specific procedures */

  CODE int (*action)(FAR struct oa_tc6_driver_s *, enum oa_tc6_action_e);

  /* addmac - set the MAC address filter so that the provided MAC will pass
   *   - shall return OK even if the provided MAC is already in the filter
   * rmmac  - remove the provided MAC from the MAC address filter
   *   - shall return OK even if the MAC address was not in the filter before
   */

  CODE int (*addmac)(FAR struct oa_tc6_driver_s *, FAR const uint8_t *mac);
  CODE int (*rmmac)(FAR struct oa_tc6_driver_s *, FAR const uint8_t *mac);
#ifdef CONFIG_NETDEV_IOCTL
  CODE int (*ioctl)(FAR struct oa_tc6_driver_s *,
                    int cmd, unsigned long arg);
#endif
};

struct oa_tc6_driver_s
{
  struct netdev_lowerhalf_s dev;        /* Driver data visible by the net
                                         * stack (must be placed first)    */

  uint8_t *txbuf;                       /* SPI transfer buffers            */
  uint8_t *rxbuf;

  mutex_t lock;                         /* Lock for data race prevention   */
  FAR struct spi_dev_s *spi;            /* The SPI device instance         */
  const struct oa_tc6_config_s *config; /* Driver configuration            */
  enum oa_tc6_ifstate_e ifstate;        /* Driver state                    */
  uint8_t mac_addr[IFHWADDRLEN];        /* MAC address of the interface    */

  struct work_s interrupt_work;         /* interrupt work wq handle        */
  struct work_s recovery_work;          /* recovery work wq handle         */
  struct work_s io_work;                /* io work wq handle               */

  int txc;                              /* TX credits                      */
  int rca;                              /* RX chunks available             */

  FAR netpkt_t *tx_pkt;                 /* Pointer to the TX netpacket     */
  FAR netpkt_t *rx_pkt;                 /* Pointer to the RX netpacket     */
  int tx_pkt_idx;                       /* Position in the TX netpacket    */
  int rx_pkt_idx;                       /* Position in the RX netpacket    */
  int tx_pkt_len;                       /* Length of the TX packet         */
  bool rx_pkt_ready;                    /* RX packet ready to be received  */

  const struct oa_tc6_ops_s *ops;       /* MAC-PHY device-specific hooks   */
};

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: oa_tc6_write_reg
 *
 * Description:
 *   Write to a MAC-PHY register.
 *
 * Input Parameters:
 *   priv  - pointer to the driver-specific state structure
 *   regid - Register id encapsulating MMS and ADDR
 *   word  - 32-bit word to be written to the register
 *
 * Returned Value:
 *   On a successful transaction OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

int oa_tc6_write_reg(FAR struct oa_tc6_driver_s *priv,
                     oa_tc6_regid_t regid, uint32_t word);

/****************************************************************************
 * Name: oa_tc6_read_reg
 *
 * Description:
 *   Read a MAC-PHY register.
 *
 * Input Parameters:
 *   priv  - pointer to the driver-specific state structure
 *   regid - register id encapsulating MMS and ADDR
 *   word  - pointer to a 32-bit destination variable
 *
 * Returned Value:
 *   On successful transaction OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

int oa_tc6_read_reg(FAR struct oa_tc6_driver_s *priv,
                    oa_tc6_regid_t regid, FAR uint32_t *word);

/****************************************************************************
 * Name: oa_tc6_set_clear_bits
 *
 * Description:
 *   Perform a read-modify-write operation on a given register
 *   while setting bits from the setbits argument and clearing bits from
 *   the clearbits argument.
 *
 * Input Parameters:
 *   priv      - pointer to the driver-specific state structure
 *   regid     - register id of the register to be modified
 *   setbits   - bits set to one will be set in the register
 *   clearbits - bits set to one will be cleared in the register
 *
 * Returned Value:
 *   On a successful transaction OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

int oa_tc6_set_clear_bits(FAR struct oa_tc6_driver_s *priv,
                          oa_tc6_regid_t regid,
                          uint32_t setbits, uint32_t clearbits);

/****************************************************************************
 * Name: oa_tc6_store_mac_addr
 *
 * Description:
 *   Store the given MAC address into the net driver structure.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *   mac  - pointer to an array containing the MAC address
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void oa_tc6_store_mac_addr(FAR struct oa_tc6_driver_s *priv,
                           FAR const uint8_t *mac);

/****************************************************************************
 * Name: oa_tc6_bitrev8
 *
 * Description:
 *   Perform a bit reverse of a byte.
 *
 * Input Parameters:
 *   byte - byte to be reversed
 *
 * Returned Value:
 *   Byte with reversed bits is returned.
 *
 ****************************************************************************/

uint8_t oa_tc6_bitrev8(uint8_t byte);

/****************************************************************************
 * Name: oa_tc6_common_init
 *
 * Description:
 *   Initialize the upper-half part of the device structure and reset
 *   the MAC-PHY.
 *
 * Input Parameters:
 *   priv  - pointer to the driver-specific state structure
 *   spi    - reference to the SPI driver state data
 *   config - reference to the predefined configuration of the driver
 *
 * Returned Value:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

int oa_tc6_common_init(FAR struct oa_tc6_driver_s *priv,
                       FAR struct spi_dev_s *spi,
                       FAR const struct oa_tc6_config_s *config);

/****************************************************************************
 * Name: oa_tc6_register
 *
 * Description:
 *   Register the OA-TC6 lower-half driver.
 *
 * Input Parameters:
 *   oa_tc6_dev - reference to the initialized oa_tc6_driver_s structure
 *
 * Returned Value:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

int oa_tc6_register(FAR struct oa_tc6_driver_s *oa_tc6_dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif/* __DRIVERS_NET_OA_TC6_H */

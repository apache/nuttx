/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_mbox_shci.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_STM32WB_MBOX_SHCI_H
#define __ARCH_ARM_SRC_STM32WB_STM32WB_MBOX_SHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/* SHCI event types *********************************************************/

#define STM32WB_SHCI_ASYNC_EVT              0xff

/* SHCI async event subtypes */

#define STM32WB_SHCI_ASYNC_EVT_C2RDY        0x9200

/* SHCI system command acknowledgement events */

#define STM32WB_SHCI_ACK_EVT_C2RDY          0x05

/* SHCI command opcodes *****************************************************/

#define STM32WB_SHCI_OGF                    0x3f
#define STM32WB_SHCI_OP(ogf, ocf)           (((ogf) << 10) | (ocf))

#define STM32WB_SHCI_FUS_GET_STATE          STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x52)
#define STM32WB_SHCI_FUS_FW_UPGRADE         STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x54)
#define STM32WB_SHCI_FUS_FW_DELETE          STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x55)
#define STM32WB_SHCI_FUS_UPDATE_AUTH_KEY    STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x56)
#define STM32WB_SHCI_FUS_LOCK_AUTH_KEY      STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x57)
#define STM32WB_SHCI_FUS_STORE_USR_KEY      STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x58)
#define STM32WB_SHCI_FUS_LOAD_USR_KEY       STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x59)
#define STM32WB_SHCI_FUS_START_WS           STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x5a)
#define STM32WB_SHCI_FUS_LOCK_USR_KEY       STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x5d)
#define STM32WB_SHCI_FUS_UNLOAD_USR_KEY     STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x5e)
#define STM32WB_SHCI_FUS_ANTIROLLBACK       STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x5f)
#define STM32WB_SHCI_BLE_INIT               STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x66)
#define STM32WB_SHCI_THREAD_INIT            STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x67)
#define STM32WB_SHCI_DEBUG_INIT             STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x68)
#define STM32WB_SHCI_FLASH_ERASE_ACTIVITY   STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x69)
#define STM32WB_SHCI_CONCURRENT_SET_MODE    STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x6a)
#define STM32WB_SHCI_FLASH_STORE_DATA       STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x6b)
#define STM32WB_SHCI_FLASH_ERASE_DATA       STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x6c)
#define STM32WB_SHCI_RADIO_ALLOW_LOW_POWER  STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x6d)
#define STM32WB_SHCI_MAC_802154_INIT        STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x6e)
#define STM32WB_SHCI_REINIT                 STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x6f)
#define STM32WB_SHCI_ZIGBEE_INIT            STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x70)
#define STM32WB_SHCI_LLD_TESTS_INIT         STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x71)
#define STM32WB_SHCI_EXTPA_CONFIG           STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x72)
#define STM32WB_SHCI_SET_FLASH_CONTROL      STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x73)
#define STM32WB_SHCI_BLE_LLD_INIT           STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x74)
#define STM32WB_SHCI_CONFIG                 STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x75)
#define STM32WB_SHCI_GET_NEXT_BLE_EVT_TIME  STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x76)
#define STM32WB_SHCI_ENABLE_NEXT_802154_NF  STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x77)
#define STM32WB_SHCI_802_15_4_DEINIT        STM32WB_SHCI_OP(STM32WB_SHCI_OGF, 0x78)

/* Command params bitfield definitions **************************************/

/* BLE init command option flags */

#define STM32WB_SHCI_BLE_INIT_OPT_STACK_MASK              (1 << 0) /* Bit 0: BLE stack select */
#  define STM32WB_SHCI_BLE_INIT_OPT_STACK_LL_HOST         (0 << 0) /* 0x0: Link Layer and Host */
#  define STM32WB_SHCI_BLE_INIT_OPT_STACK_LL              (1 << 0) /* 0x1: Link Layer only */

#define STM32WB_SHCI_BLE_INIT_OPT_SVC_CHCHAR_MASK         (1 << 1) /* Bit 1: Service Changed characteristic */
#  define STM32WB_SHCI_BLE_INIT_OPT_SVC_CHCHAR_ENABLED    (0 << 1) /* 0x0: Characteristic enabled */
#  define STM32WB_SHCI_BLE_INIT_OPT_SVC_CHCHAR_DISABLED   (1 << 1) /* 0x1: Characteristic disabled */

#define STM32WB_SHCI_BLE_INIT_OPT_DEVICE_NAME_MODE_MASK   (1 << 2) /* Bit 2: Device Name mode */
#  define STM32WB_SHCI_BLE_INIT_OPT_DEVICE_NAME_MODE_RW   (0 << 2) /* 0x0: Read-Write mode */
#  define STM32WB_SHCI_BLE_INIT_OPT_DEVICE_NAME_MODE_RO   (1 << 2) /* 0x1: Read-Only mode */

#define STM32WB_SHCI_BLE_INIT_OPT_CS_ALG2_MASK            (1 << 4) /* Bit 4: Channel selection algorithm 2 enabled */
#  define STM32WB_SHCI_BLE_INIT_OPT_CS_ALG2_DISABLED      (0 << 4) /* 0x0: Algorithm 2 disabled */
#  define STM32WB_SHCI_BLE_INIT_OPT_CS_ALG2_ENABLED       (1 << 4) /* 0x1: Algorithm 2 enabled */

#define STM32WB_SHCI_BLE_INIT_OPT_POWER_CLASS_MASK        (1 << 7) /* Bit 7: Power class */
#  define STM32WB_SHCI_BLE_INIT_OPT_POWER_CLASS_2_3       (0 << 7) /* 0x0: Power Class 2-3 */
#  define STM32WB_SHCI_BLE_INIT_OPT_POWER_CLASS_1         (1 << 7) /* 0x1: Power Class 1 */

/* BLE init command rx_model_config flags */

#define STM32WB_SHCI_BLE_INIT_RXMOD_AGC_RSSI_MASK         (1 << 0) /* Bit 0: AGC RSSI model */
#  define STM32WB_SHCI_BLE_INIT_RXMOD_AGC_RSSI_LEGACY     (0 << 0) /* 0x0: AGC RSSI Legacy */
#  define STM32WB_SHCI_BLE_INIT_RXMOD_AGC_RSSI_IMPROVED   (1 << 0) /* 0x1: AGC RSSI Improved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* STM32WB_SHCI_BLE_INIT command params */

begin_packed_struct struct stm32wb_shci_ble_init_cfg_s
{
  void     *ble_buf;              /* Not used, must be NULL. */
  uint32_t ble_buf_size;          /* Not used, must be 0. */

  uint16_t gatt_attr_num;         /* Maximum number of GATT attributes. */
  uint16_t gatt_srv_num;          /* Maximum number of GATT services. */
  uint16_t gatt_attr_buf_size;    /* GATT attributes storage buf size. */

  uint8_t  max_conn;              /* Maximum number of simultaneous
                                   * connections, up to 8 is supported. */

  uint8_t  dle_enable;            /* Data Length Extension enable. */

  uint8_t  prep_write_op_num;     /* Maximum number of Prepare Write
                                   * operations. */

  uint8_t  mem_block_num;         /* Number of allocated memory blocks,
                                   * throughput performance / memory usage
                                   * tuning. */

  uint16_t att_max_mtu_size;      /* Maximum attribute MTU size. */

  uint16_t slave_sca;             /* Sleep clock accuracy (ppm value) in
                                   * slave mode. */

  uint8_t  master_sca_range;      /* Sleep clock accuracy in master mode:
                                   * 0x00: 251 ppm to 500 ppm
                                   * 0x01: 151 ppm to 250 ppm
                                   * 0x02: 101 ppm to 150 ppm
                                   * 0x03: 76 ppm to 100 ppm
                                   * 0x04: 51 ppm to 75 ppm
                                   * 0x05: 31 ppm to 50 ppm
                                   * 0x06: 21 ppm to 30 ppm
                                   * 0x07: 0 ppm to 20 ppm */

  uint8_t  ls_clock_source;       /* Low speed 32 kHz clock source:
                                   * 0x00: use LSE
                                   * 0x01: use HSE */

  uint32_t conn_event_length;     /* Maximum duration of a slave connection
                                   * event in units of 625/256us
                                   * (~2.44us). */

  uint16_t hse_startup;           /* HSE startup time in units of 625/256us
                                   * (~2.44us). */

  uint8_t  viterbi_enable;        /* Enable Viterbi algorithm
                                   * implementation. */

  uint8_t  options;               /* BLE init option flags. */

  uint8_t  hw_version;            /* Not used, must be 0. */

  uint8_t  max_initor_coc_num;    /* Maximum number of connection-oriented
                                   * channels in initiator mode. */

  int8_t   tx_power_min;          /* Minimum transmit power in dBm.
                                   * Range: -127 .. 20 */

  int8_t   tx_power_max;          /* Maximum transmit power in dBm.
                                   * Range: -127 .. 20 */

  uint8_t  rx_model_config;       /* RX model config flags */
} end_packed_struct;

#endif /* __ARCH_ARM_SRC_STM32WB_STM32WB_MBOX_SHCI_H */

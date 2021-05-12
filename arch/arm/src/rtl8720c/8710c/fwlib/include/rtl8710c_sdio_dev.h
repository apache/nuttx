/**************************************************************************//**
 * @file     rtl8710c_sdio_dev.h
 * @brief    The HAL related definition and macros for the SDIO device HAL driver.
 * @version  V1.00
 * @date     2017-09-07
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2016 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/


#ifndef _RTL8710C_SDIOD_H_
#define _RTL8710C_SDIOD_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
* @addtogroup hs_hal_sdio_dev SDIO Device
* @ingroup 8710c_hal
* @{
* @brief The SDIO Device HAL module of the AmebaZ2 platform.
*/

#include "rtl8710c_sdio_dev_type.h"
#include "dlist.h"

#if !defined(container_of)
#define container_of(ptr, type, member) \
			((type *)((char *)(ptr)-(SIZE_T)(&((type *)0)->member)))
#endif

/// the Maximum size for a RX_BD can point to, it should be 4-bytes aligned
#define MAX_RX_BD_BUF_SIZE              16380

/// the minimum RX_BD is needed to send a Packet to Host, 2: one for RX_Desc, the other one for payload.
#define MIN_RX_BD_SEND_PKT              2

/**
 * @brief  Defines the bitwise flag over the RPWM2 register.
 */
enum SDIO_RPWM2_BITS {
	RPWM2_ACT_BIT           = BIT0,     ///< Active
    RPWM2_SLEEP_BIT         = 0,        ///< Sleep
    RPWM2_DSTANDBY_BIT      = BIT1,     ///< Deep Standby
    RPWM2_PG_BIT            = 0,        ///< Power Gated
    RPWM2_FBOOT_BIT         = BIT2,     ///< fast reboot
    RPWM2_NBOOT_BIT         = 0,        ///< normal reboot
    RPWM2_WKPIN_A5_BIT      = BIT3,     ///< enable GPIO A5 wakeup
    RPWM2_WKPIN_C7_BIT      = BIT4,     ///< enable GPIO C7 wakeup
    RPWM2_WKPIN_D5_BIT      = BIT5,     ///< enable GPIO D5 wakeup
    RPWM2_WKPIN_E3_BIT      = BIT6,     ///< enable GPIO E3 wakeup
    RPWM2_PIN_A5_LV_BIT     = BIT7,     ///< GPIO A5 wakeup level
    RPWM2_PIN_C7_LV_BIT     = BIT8,     ///< GPIO C7 wakeup level
    RPWM2_PIN_D5_LV_BIT     = BIT9,     ///< GPIO D5 wakeup level
    RPWM2_PIN_E3_LV_BIT     = BIT10,    ///< GPIO E3 wakeup level
    RPWM2_CG_BIT            = BIT11,    ///< Clock Gated
    RPWM2_ACK_BIT           = BIT14,    ///< Acknowledge
    RPWM2_TOGGLE_BIT        = BIT15,    ///< Toggle bit
};

/**
 * @brief  Defines the bitwise flag over the CPWM2 register.
 */
enum SDIO_CPWM2_BITS {
	CPWM2_ACT_BIT           = BIT0,     ///< Active
    CPWM2_DSTANDBY_BIT      = BIT1,     ///< Deep Standby
    CPWM2_FBOOT_BIT         = BIT2,     ///< fast reboot
    CPWM2_INIC_FW_RDY_BIT   = BIT3,     ///< is the iNIC FW(1) or Boot FW(0)

    CPWM2_TOGGLE_BIT        = BIT15,    ///< Toggle bit
};

/**
 * @brief  Defines the SDIO device HAL events.
 */
enum SDIO_EVENTS {
    SDIO_EVENT_IRQ          = BIT0,     ///< Interrupt triggered
    SDIO_EVENT_RX_PKT_RDY   = BIT1,     ///< A new SDIO packet ready
    SDIO_EVENT_C2H_DMA_DONE = BIT2,     ///< Interrupt of C2H DMA done triggered
    SDIO_EVENT_DUMP         = BIT3,     ///< SDIO status dump periodically Enable
    SDIO_EVENT_TXBD_REFILL  = BIT4,     ///< To refill TX BD buffer
    SDIO_EVENT_EXIT         = BIT28,    ///< Request to exit the SDIO task
    SDIO_EVENT_MP_STOPPED   = BIT29,    ///< The SDIO task is stopped
    SDIO_EVENT_TX_STOPPED   = BIT30,    ///< The SDIO task is stopped
    SDIO_EVENT_RX_STOPPED   = BIT31,    ///< The SDIO task is stopped
};

/**
 * @brief  Defines the packet type of a SDIO device's packet
 */
enum SDIOD_PKT_TYPE_E {
    SDIO_CMD_TX_ETH         = 0x83,    ///< request to TX a 802.3 packet
    SDIO_CMD_TX_WLN         = 0x81,    ///< request to TX a 802.11 packet
    SDIO_CMD_H2C            = 0x11,    ///< H2C(host to device) command packet
    
    SDIO_CMD_RX_ETH         = 0x82,    ///< indicate a RX 802.3 packet
    SDIO_CMD_RX_WLN         = 0x80,    ///< indicate a RX 802.11 packet
    SDIO_CMD_C2H            = 0x10     ///< C2H(device to host) command packet
};

/**
 * @brief  Defines the message type for the SDIO device HAL driver. 
 */
enum SDIOD_MSG_TYPE_E {
	MSG_SDIO_RX_PKT         = 1,		///< request to send a SDIO RX packet to the host side
	MSG_SDIO_C2H            = 2,		///< request to send a C2H message
	MSG_SDIO_RPWM           = 3,		///< request to set the RPWM
    MSG_SDIO_MP_LOOP_TXPKT  = 4,        ///< request to loopback this TX packet

	MSG_SDIO_MAX=0xff
};

/**
 * @brief  The data type of the TX BD(Buffer Descriptor).
 */
typedef struct sdiod_txbd_s {
    uint32_t    addr;               /*!< The TX buffer physical address, it must be 4-bytes aligned */
} sdiod_txbd_t, *psdiod_txbd_t;

/* The RX Buffer Descriptor format */

#if (SYSTEM_ENDIAN==PLATFORM_LITTLE_ENDIAN)
/**
 * @brief  The data type of the RX BD(Buffer Descriptor).
 */
typedef struct sdiod_rxbd_s {
    uint32_t buf_size:14;           /*!< bit[13:0], RX Buffer Size, Maximum 16384-1 */
    uint32_t ls:1;                  /*!< bit[14], is the Last Segment ? */
    uint32_t fs:1;                  /*!< bit[15], is the First Segment ? */
    uint32_t seq:16;                /*!< bit[31:16], The sequence number, it's no use for now */
    uint32_t phy_addr;              /*!< The RX buffer physical address, it must be 4-bytes aligned */
} sdiod_rxbd_t, *psdiod_rxbd_t;

/**
 * @brief  The data type of a TX packet descriptor.
 */
typedef struct sdiod_tx_desc_s {
    // DW0
    uint32_t txpktsize:16;          /*!< bit[15:0], packet size */
    uint32_t offset:8;              /*!< bit[23:16], payload start offset, normally it is sizeof(SDIO_TX_DESC) */
    uint32_t bus_agg_num:8;         /*!< bit[31:24], the bus aggregation number */

    // DW1
    uint32_t type:8;                /*!< bit[7:0], the packet type */
    uint32_t rsvd0:24;

    uint32_t rsvd1;
    uint32_t rsvd2;
    uint32_t rsvd3;
    uint32_t rsvd4;
} sdiod_tx_desc_t, *psdiod_tx_desc_t;

/**
 * @brief  The data type of a RX packet descriptor.
 */
typedef struct sdiod_rx_desc_s {
	// DW 0
	uint32_t pkt_len:16;        /*!< bit[15:0], the packet size */
	uint32_t offset:8;          /*!< bit[23:16], the offset from the packet start to the payload start,
	                                 normally it's the size of a RX Desc */
	uint32_t rsvd0:6;           /*!< bit[29:24], reserved */
	uint32_t icv:1;             // bit[30], ICV error
	uint32_t crc:1;             // bit[31], CRC error

	// DW 1
	uint32_t type:8;            // bit[7:0], the type of this packet
	uint32_t rsvd1:24;          // bit[31:8]

	// DW 2
	uint32_t rsvd2;
	uint32_t rsvd3;
	uint32_t rsvd4;
	uint32_t rsvd5;
} sdiod_rx_desc_t, *psdiod_rx_desc_t;

#else   // else of "#if (SYSTEM_ENDIAN==PLATFORM_LITTLE_ENDIAN)"
/* Structure definition with Big Endian */
/**
 * @brief  The data type of the RX BD(Buffer Descriptor).
 */
typedef struct sdiod_rxbd_s {
    uint32_t phy_addr;              /*!< The RX buffer physical address, it must be 4-bytes aligned */
    uint32_t seq:16;                /*!< bit[31:16], The sequence number, it's no use for now */
    uint32_t fs:1;                  /*!< bit[15], is the First Segment ? */
    uint32_t ls:1;                  /*!< bit[14], is the Last Segment ? */
    uint32_t buf_size:14;           /*!< bit[13:0], RX Buffer Size, Maximum 16384-1 */

} sdiod_rxbd_t, *sdiod_rxbd_t;

/**
 * @brief  The data type of a TX packet descriptor.
 */
typedef struct sdiod_tx_desc_s {
    // DW0
    uint32_t bus_agg_num:8;         /*!< bit[31:24], the bus aggregation number */
    uint32_t offset:8;              /*!< bit[23:16], payload start offset, normally it is sizeof(SDIO_TX_DESC) */
    uint32_t txpktsize:16;          /*!< bit[15:0], packet size */

    // DW1
    uint32_t rsvd0:24;
    uint32_t type:8;                /*!< bit[7:0], the packet type */


    uint32_t rsvd1;
    uint32_t rsvd2;
    uint32_t rsvd3;
    uint32_t rsvd4;
} sdiod_tx_desc_t, *psdiod_tx_desc_t;

/**
 * @brief  The data type of a RX packet descriptor.
 */
typedef struct sdiod_rx_desc_s {
	// DW 0
	uint32_t crc:1;             // bit[31], CRC error
	uint32_t icv:1;             // bit[30], ICV error
	uint32_t rsvd0:6;           /*!< bit[29:24], reserved */
	uint32_t offset:8;          /*!< bit[23:16], the offset from the packet start to the payload start,
                                     normally it's the size of a RX Desc */
	uint32_t pkt_len:16;        /*!< bit[15:0], the packet size */

	// DW 1
	uint32_t rsvd1:24;          // bit[31:8]
	uint32_t type:8;            // bit[7:0], the type of this packet

	// DW 2
	uint32_t rsvd2;
	uint32_t rsvd3;
	uint32_t rsvd4;
	uint32_t rsvd5;
} sdiod_rx_desc_t, *psdiod_rx_desc_t;

#endif  // end of else of "#if (SYSTEM_ENDIAN==PLATFORM_LITTLE_ENDIAN)"

/**
 * @brief  The data type of a packet which will be forwarded to the WLan driver to transmit it.
 */
typedef struct sdiod_tx_packet_s {
	uint8_t *pheader;           /*!< Point to the 1st byte of the packets */
	uint16_t pkt_size;          /*!< the size (bytes) of this packet */
	struct list_head list;                 /*!< the link list to chain packets */
	uint8_t is_malloc;          /*!< is this packet buffer from memory allocate */
} sdiod_tx_packet_t, *psdiod_tx_packet_t;


#if defined(CONFIG_INIC_EN) && (CONFIG_INIC_EN != 0)
/**
 * @brief  The type of WLan driver socket buffer for SDIO device.
 */
typedef struct sk_buff sdiod_sk_buf_t;
#else
typedef void sdiod_sk_buf_t;
#endif

/**
 * @brief  The data type for the binding of a TX BD with a TX packet.
 */
typedef struct sdiod_txbd_hdl_s {
	sdiod_txbd_t *ptxbd;        /*!< Point to the TX_BD buffer */
    union {
        void *priv;                 /*!< for SDPIO API: application priviate data */
        sdiod_sk_buf_t *skb;        /*!< for INIC: Socket buffer from IP layer */
    };
	sdiod_tx_packet_t *ppkt;    /*!< point to the Tx Packet */
	uint8_t is_pkt_end;         /*!< For the case of a packet over 1 BD, this flag is used to indicates whether this BD
	                                 contains a packet end or not */
	uint8_t is_free;            /*!< is this TX BD free */
} sdiod_txbd_hdl_t, *psdiod_txbd_hdl_t;

/**
 * @brief  The data type of a RX (SDIO device to SDIO host) packet.
 */
typedef struct sdiod_rx_packet_s {
    sdiod_rx_desc_t *prx_desc;  /*!< The RX Descriptor for this packet, to be send to Host ahead this packet */
    union {
        void *priv;                 /*!< for SDPIO API: application priviate data */
        sdiod_sk_buf_t *skb;        /*!< for INIC: the WLan SKB buffer */
    };
    uint8_t *pdata;             /*!< point to the head of payload of this packet */
    uint16_t offset;            /*!< the offset from the pData to the payload buffer */
    struct list_head list;                 /*!< the link list to chain packets */
	uint8_t is_malloc;          /*!< is this packet buffer from memory allocate */
} sdiod_rx_packet_t, *psdiod_rx_packet_t;

/**
 * @brief  The data type for the binding of a RX BD with a RX packet.
 */
typedef struct sdiod_rxbd_hdl_s {
    sdiod_rxbd_t *prxbd;        /*!< the RX_BD buffer */
    sdiod_rx_packet_t *ppkt;    /*!< the Rx Packet */
    uint8_t is_pkt_end;         /*!< For the case of a packet with over 1 BD , this flag is used to indicate that
                                     whether this BD contains a packet end or not. */
    uint8_t is_free;            /*!< is this RX BD free (DMA done and its RX packet has been freed) */
} sdiod_rxbd_hdl_t, *psdiod_rxbd_hdl_t;


#define SDIO_TX_BUF_SZ_UNIT          64

/* Be carefull!! the setting of hardware's TX BD buffer size may exceed the real size of
   the TX BD buffer size, this case will cause the hardware DMA write the buffer overflow */
/// Defines the size of a single TX buffer which addressed by TX BD, in unite of 64 byets
#define SDIO_TX_BD_BUF_USIZE        ((((SDIO_TX_BD_BUF_SIZE+sizeof(sdiod_tx_desc_t)-1)/SDIO_TX_BUF_SZ_UNIT)+1)&0xff)

/**
 * @brief  The data type of the SDIO device message entity
 */
typedef struct sdiod_msg_blk_s {
    uint8_t     msg_type;		/*!< the message type */
    uint8_t     para0;          /*!< the optional parameters associated with this message type */
    uint16_t    msg_len;        /*!< the vaild data length of the pBuf */
    uint32_t    para1;          /*!< the optional parameters associated with this message type */
    uint8_t     *pmsg_buf;      /*!< point to a data buffer associated with this message type */
} sdiod_msg_blk_t, *psdiod_msg_blk_t;

/**
 * @brief  Call back function for a TX packet from the SDIO host is received.
 */
typedef int8_t (*sdiod_tx_callback_t)(void *cb_arg, uint8_t *ppkt, uint16_t offset, uint16_t pkt_size,
                                            uint8_t type);

/**
 * @brief  Call back function for a RX packet to the SDIO host transfer is done.
 */
typedef int8_t (*sdiod_rx_done_callback_t)(void *cb_arg, void *priv, uint16_t offset,
                                                  uint16_t pkt_size, uint8_t type);

/**
 * @brief  Call back function for a message from the SDIO host is received.
 */
typedef void (*sdiod_h2c_msg_callback_t) (void *para, uint32_t h2c_msg);

/**
 * @brief  Call back function for a reset command from the SDIO host is received.
 */
typedef void (*sdiod_rst_cmd_callback_t) (void *para);

/**
 * @brief  Call back function for RPWM command from the SDIO host is received.
 */
typedef uint8_t (*sdiod_rpwm_callback_t) (void *para);

/**
 * @brief  Call back function for OS wait function.
 */
typedef void (*sdiod_os_wait_t) (int ms);

/**
 * @brief  The data type of the SDIO device HAL driver management entity.
 */
typedef struct hal_sdiod_adapter_s  hal_sdio_dev_adapter_t, *phal_sdio_dev_adapter_t;

struct hal_sdiod_adapter_s {
    volatile uint32_t critical_lv;          /*!< to record SDIO device HAL enter critical section level */
    uint16_t            tx_bd_num;          /*!< the number of TX descriptor to allocate at driver initialization */
    uint16_t            rx_bd_num;          /*!< the number of RX descriptor to allocate at driver initialization */
    uint32_t            tx_bd_buf_size;     /*!< buffer size of a TX BD, which must be 64*N */
    uint32_t            rx_bd_buf_size;     /*!< buffer size of a RX BD, which must be 64*N */
    // Offset 0x10    
    uint8_t             *ptxbd_addr;        /*!< The buffer allocated for TX BD */
    sdiod_txbd_t        *ptxbd_addr_align;  /*!< The TX_BD start address, it must be 4-bytes aligned */
    sdiod_txbd_hdl_t    *ptxbd_hdl;         /*!< point to the allocated memory for TX_BD Handle array */
    uint16_t            txbd_wptr;          /*!< The SDIO TX(Host->Device) BD local write index, it's different with
                                                 HW maintained write Index. */
    uint16_t            txbd_rptr;          /*!< The SDIO TX(Host->Device) BD read index */
    // Offset 0x20
    uint16_t            txbd_rptr_reg;      /*!< The SDIO TX(Host->Device) BD read index has been write to
                                                 HW register */
    uint16_t            free_rx_bd_cnt;     /*!< the threshold of free RX BD count to trigger interrupt */
    uint8_t             *prxbd_addr;        /*!< The buffer allocated for RX BD */
    sdiod_rxbd_t        *prxbd_addr_align;  /*!< The RX_BD start address, it must be 8-bytes aligned */
    sdiod_rxbd_hdl_t    *prxbd_hdl;         /*!< point to the allocated memory for RX_BD Handle array */
    // Offset 0x30
    uint16_t            rxbd_wptr;		    /*!< The SDIO RX(Device->Host) BD write index */
    uint16_t            rxbd_rptr;		    /*!< The SDIO RX(Device->Host) BD local read index, different with
                                                 HW maintained Read Index. */
    sdio_dev_int_mask_t int_mask;			/*!< The Interrupt Mask */
    sdio_dev_int_sts_t  int_status;			/*!< The Interrupt Status */
    uint32_t            events;				/*!< The Event to the SDIO Task */

    sdio_dev_ccpwm_t    ccpwm;              /*!< the value write to register CCPWM, which will sync to Host HCPWM */
    uint8_t             reserve1;
    sdio_dev_ccpwm2_t   ccpwm2;             /*!< the value write to register CCPWM2, which will sync to Host HCPWM2 */
    // Offset 0x40
    sdio_dev_crpwm_t    crpwm;              /*!< sync from Host HRPWM */
    sdio_dev_crpwm2_t   crpwm2;             /*!< sync from Host HRPWM2 */

    sdiod_tx_callback_t tx_callback;        /*!< to hook the WLan driver TX callback function to handle a Packet TX */
    void                *ptxcb_para;		/*!< the application priviate data for the TX Callback function,
    										   which is from the TX CallBack function register */
    sdiod_rst_cmd_callback_t rst_cmd_callback;  /*!< callback for a reset command is received */
    // Offset 0x50
    sdiod_rpwm_callback_t rpwm1_cmd_callback;   /*!< callback for a RPWM1 command is received */
    sdiod_rpwm_callback_t rpwm2_cmd_callback;   /*!< callback for a RPWM2 command is received */
    sdiod_h2c_msg_callback_t h2c_msg_callback;  /*!< callback for the H2C message received */
    void                *ph2c_msg_cb_para;  /*!< the application priviate data for the H2C message Callback function */
    // Offset 0x60
    sdiod_rx_done_callback_t rx_done_callback;  /*!< to hook RX done callback function to release packet */
    void			    *prx_done_cb_para;   /*!< a pointer will be used to call the RX Done Callback function,
    										      which is from the TX CallBack function register */
    int8_t (*msg_handler) (hal_sdio_dev_adapter_t *psdio_adp, sdiod_msg_blk_t *pmblk);  /*!< callback for a message block is received */
    void (*dcache_invalidate_by_addr)(uint32_t *addr, int32_t dsize);   /*!< callback function to do the D-cache invalidate  */
    // Offset 0x70
    void (*dcache_clean_by_addr) (uint32_t *addr, int32_t dsize);       /*!< callback function to do the D-cache clean  */
    void (*dcache_clean_bd) (uint32_t *addr, int32_t dsize);       /*!< callback function to do the D-cache clean for BD  */

    uint8_t             *prx_desc_buf;      /*!< The buffer allocated for RX descripter */
    sdiod_rx_desc_t     *prx_desc_addr;     /*!< The RX descripters start address, it should aligned to 32-bytes for
                                                 D-cache sync. consideration */
    // Offset 0x80
    struct list_head    pend_rx_pkt_list;   /*!< The list to queue RX packets */
    struct list_head    free_rx_pkt_list;   /*!< The list to queue free Rx packets handler */
    // Offset 0x90
    sdiod_rx_packet_t   *prx_pkt_handler;   /*!< to store allocated RX Packet handler memory address */
    uint32_t            rx_in_q_cnt;        /*!< The packet count for Rx In Queue */
    uint8_t             rx_fifo_busy;       /*!< is the RX BD fetch hardware busy */
    uint8_t             rx_pkt_16k;         /*!< flag to control the support of RX packet size over 16K */
    uint8_t             rx_bd_fetch_barrier; /*!< is limit SW to update RX BD write pointer while HW fetching RX BD  */

    // Offset 0xA0
    hal_status_t (*txbd_hdl_init) (sdiod_txbd_hdl_t *ptxbd_hdl, uint16_t txbd_idx);    /*!< the callback function for an TX BD handle initialization */
    void (*txbd_hdl_deinit) (sdiod_txbd_hdl_t *ptxbd_hdl, uint16_t txbd_idx);          /*!< the callback function for an TX BD handle de-initialization */
    void (*txbd_buf_refill) (sdiod_txbd_hdl_t *ptxbd_hdl);  /*!< the callback function for an TX BD buffer refill */
    void (*rxbd_tr_done_callback) (hal_sdio_dev_adapter_t *psdio_adp, sdiod_rxbd_hdl_t *prxbd_hdl); /*!< the callback function for an RX BD data transfer is done */
    // Offset 0xB0
    int8_t (*txbd_rdy_callback)(hal_sdio_dev_adapter_t *psdio_adp, sdiod_txbd_hdl_t *ptxbd_hdl,\
                                sdiod_tx_desc_t *ptx_desc); /*!< the callback function for an TX packet is ready */
    void (*free_rx_pkt)(hal_sdio_dev_adapter_t *psdio_adp, sdiod_rx_packet_t *ppkt);    /*!< the callback function to free an RX packet */
    void (*cmd11_callback)(hal_sdio_dev_adapter_t *psdio_adp);    /*!< the callback function for CMD11 received indication */
    hal_status_t (*os_init) (hal_sdio_dev_adapter_t *psdio_adp, void *tx_task, void *rx_task);
    // Offset 0xC0
    hal_status_t (*os_deinit) (hal_sdio_dev_adapter_t *psdio_adp);
    void (*rx_lock) (void);
    void (*rx_unlock) (void);
    void (*os_wait_tx_bd) (void);
    // Offset 0xD0
    void (*os_wait_rx_bd) (void);
    sdiod_os_wait_t os_wait;

    uint32_t (*tx_task_down) (void *arg);
    int32_t (*tx_task_up) (void *arg);
    // Offset 0xE0
    uint32_t (*rx_task_down) (void *arg);
    int32_t (*rx_task_up) (void *arg);
    void (*task_exit) (void);
    int (*pop_msg_queue) ( void *arg, void* message);
    
    // Offset 0xF0
    volatile uint32_t   event_sema;         /*!< Semaphore for SDIO events, use to wakeup the SDIO task */

    // members for SPDIO API
    void                *spdio_priv;        /*!< priviate data from user application */

} ;

/**
 * @addtogroup hs_hal_sdio_dev_ll_func
 * @{
 */
void hal_sdio_dev_enter_critical_rtl8710c (hal_sdio_dev_adapter_t *psdio_adp);
void hal_sdio_dev_exit_critical_rtl8710c (hal_sdio_dev_adapter_t *psdio_adp);
void hal_sdio_dev_reg_irq_rtl8710c (irq_handler_t handler);
void hal_sdio_dev_int_def_enable (hal_sdio_dev_adapter_t *psdio_adp);
void hal_sdio_dev_syson_ctrl_rtl8710c(BOOL en);
hal_status_t hal_sdio_dev_init_rtl8710c (hal_sdio_dev_adapter_t *psdio_adp);
void hal_sdio_dev_deinit_rtl8710c (hal_sdio_dev_adapter_t *psdio_adp);
void hal_sdio_dev_task_up_rtl8710c (hal_sdio_dev_adapter_t *psdio_adp);
void hal_sdio_dev_tx_task_rtl8710c (hal_sdio_dev_adapter_t *psdio_adp);
void hal_sdio_dev_rx_task_rtl8710c (hal_sdio_dev_adapter_t *psdio_adp);
void hal_sdio_dev_irq_handler_bh_rtl8710c (hal_sdio_dev_adapter_t *psdio_adp);
void hal_sdio_dev_rx_irq_handler_bh_rtl8710c (hal_sdio_dev_adapter_t *psdio_adp);
void hal_sdio_dev_txbd_buf_refill_rtl8710c (hal_sdio_dev_adapter_t *psdio_adp);
void hal_sdio_dev_tx_fifo_data_ready_rtl8710c (hal_sdio_dev_adapter_t *psdio_adp);
void hal_sdio_dev_recycle_rxbd_rtl8710c (hal_sdio_dev_adapter_t *psdio_adp);
void hal_sdio_dev_return_rx_data_rtl8710c (hal_sdio_dev_adapter_t *psdio_adp);
hal_status_t hal_sdio_dev_rx_pkt_enqueue_rtl8710c (hal_sdio_dev_adapter_t *psdio_adp, sdiod_rx_packet_t *ppkt);

/**
  \brief  The data structure of the stubs functions of the SDIO Device HAL functions in ROM.
*/
typedef struct hal_sdiod_func_stubs_s {
    phal_sdio_dev_adapter_t *ppsdio_dev_adp;
    void (*enter_critical) (hal_sdio_dev_adapter_t *psdio_adp);
    void (*exit_critical) (hal_sdio_dev_adapter_t *psdio_adp);
    void (*set_event) (hal_sdio_dev_adapter_t *psdio_adp, uint32_t event);
    void (*clear_event) (hal_sdio_dev_adapter_t *psdio_adp, uint32_t event);
    void (*irq_handler)(void);
    BOOL (*event_pending) (hal_sdio_dev_adapter_t *psdio_adp, uint32_t event);
    void (*reg_irq) (irq_handler_t handler);
    void (*syson_ctrl)(BOOL en);
    hal_status_t (*init) (hal_sdio_dev_adapter_t *psdio_adp);
    void (*deinit) (hal_sdio_dev_adapter_t *psdio_adp);
    void (*task_up) (hal_sdio_dev_adapter_t *psdio_adp);
    void (*tx_task) (hal_sdio_dev_adapter_t *psdio_adp);
    void (*rx_task) (hal_sdio_dev_adapter_t *psdio_adp);
    void (*irq_handler_bh) (hal_sdio_dev_adapter_t *psdio_adp);
    void (*rx_irq_handler_bh) (hal_sdio_dev_adapter_t *psdio_adp);
    void (*txbd_buf_refill) (hal_sdio_dev_adapter_t *psdio_adp);
    void (*tx_fifo_data_ready) (hal_sdio_dev_adapter_t *psdio_adp);
    void (*recycle_rxbd) (hal_sdio_dev_adapter_t *psdio_adp);
    void (*return_rx_data) (hal_sdio_dev_adapter_t *psdio_adp);
    hal_status_t (*rx_pkt_enqueue) (hal_sdio_dev_adapter_t *psdio_adp, sdiod_rx_packet_t *ppkt);

    uint32_t reserved[11];  // reserved space for next ROM code version function table extending.
} hal_sdiod_func_stubs_t;

/** @} */ /* End of group hs_hal_sdio_dev_ll_func */

/** @} */ /* End of group hs_hal_sdio_dev */

#ifdef __cplusplus
}
#endif

#endif  // #ifndef _RTL8710C_SDIOD_H_


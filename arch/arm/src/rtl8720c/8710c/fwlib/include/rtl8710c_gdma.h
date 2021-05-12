/**************************************************************************//**
 * @file     rtl8710c_gdma.h
 * @brief    The header file of rtl8710c_gdma.c.
 * @version  1.00
 * @date     2017-08-22
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
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

#ifndef RTL8710C_GDMA_H
#define RTL8710C_GDMA_H

// TODO: modification for 8710C: GDMA handshake ID
/**
        \addtogroup hs_hal_gdma GDMA
        \ingroup 8710c_hal
        \brief The GDMA module of the AmebaZ2 platform.
        @{
*/

#define MAX_GDMA_INDX       0           //!< Define max GDMA index, We have GDMA0 only
#define MAX_GDMA_CHNL       2           //!< Define the total channel number
#define MAX_DMA_BLOCK_SIZE  4095        //!< Define max GDMA block size, the maximum size for a single block transfer

/*Define GDMA Handshake interface with various peripheral*/
#define GDMA_HANDSHAKE_FLASH_TX             0   //!< Define GDMA hardware handshake number for FLASH TX
#define GDMA_HANDSHAKE_FLASH_RX             1   //!< Define GDMA hardware handshake number for FLASH RX
#define GDMA_HANDSHAKE_UART2_TX             2   //!< Define GDMA hardware handshake number for UART2 TX
#define GDMA_HANDSHAKE_UART2_RX             3   //!< Define GDMA hardware handshake number for UART2 RX
#define GDMA_HANDSHAKE_UART1_TX             4   //!< Define GDMA hardware handshake number for UART1 TX
#define GDMA_HANDSHAKE_UART1_RX             5   //!< Define GDMA hardware handshake number for UART1 RX
#define GDMA_HANDSHAKE_UART0_TX             6   //!< Define GDMA hardware handshake number for UART0 TX
#define GDMA_HANDSHAKE_UART0_RX             7   //!< Define GDMA hardware handshake number for UART0 RX
#define GDMA_HANDSHAKE_UART3_TX             8   //!< Define GDMA hardware handshake number for UART3 TX
#define GDMA_HANDSHAKE_UART3_RX             9   //!< Define GDMA hardware handshake number for UART3 RX
#define GDMA_HANDSHAKE_SSI0_TX              10  //!< Define GDMA hardware handshake number for SSI0 TX
#define GDMA_HANDSHAKE_SSI0_RX              11  //!< Define GDMA hardware handshake number for SSI0 RX
#define GDMA_HANDSHAKE_I2C0_TX              12  //!< Define GDMA hardware handshake number for I2C0 TX
#define GDMA_HANDSHAKE_I2C0_RX              13  //!< Define GDMA hardware handshake number for I2C0 RX

typedef void (*gdma_callback_t)(void *data);

/**
  \brief  Enumeration to define all available channels
*/
enum gdma_chnl_num_e {
      GdmaNoCh    = 0x0000,     //!<  No channel
      GdmaCh0     = 0x0101,     //!<  channel 0
      GdmaCh1     = 0x0202,     //!<  channel 1
      GdmaAllCh   = 0xffff      //!<  all channels
};
typedef uint32_t gdma_chnl_num_t;

/**
  \brief  Enumeration to define transfer type
*/
enum gdma_tt_fc_type_e {
      TTFCMemToMem    = 0x00,     //!<  Memory to Memory
      TTFCMemToPeri   = 0x01,     //!<  Memory to Peripheral FIFO
      TTFCPeriToMem   = 0x02      //!<  Peripheral FIFO to Memory
};
typedef uint8_t gdma_tt_fc_type_t;


/**
  \brief  Enumeration to define data width per data item
*/
enum gdma_ctl_tr_width_e {
      TrWidthOneByte    = 0x00,   //!<  One byte per data item
      TrWidthTwoBytes   = 0x01,   //!<  Two bytes per data item
      TrWidthFourBytes  = 0x02    //!<  Four bytes per data item
};
typedef uint8_t gdma_ctl_tr_width_t;

/**
  \brief  Enumeration to define burst size (the number of data items) for a burst transaction, is no use for memory side.
*/
enum gdma_ctl_msize_e {
      MsizeOne        = 0x00,     //!<  One data item
      MsizeFour       = 0x01,     //!<  Four data items
      MsizeEight      = 0x02,     //!<  Eight data items
      MsizeSixTeen    = 0x03      //!<  Sixteen data items
};
typedef uint8_t gdma_ctl_msize_t;

/**
  \brief  Enumeration to indicate whether to increment the address, decrement the address or keep the same address value on every transfer
*/
enum gdma_inc_type_e {
      IncType         = 0x00,     //!<  In increasing order
      DecType         = 0x01,     //!<  In decreasing order, not support
      NoChange        = 0x02      //!<  The address point is always fiexed, i.e. FIFO address
};
typedef uint8_t gdma_inc_type_t;

/**
  \brief  Enumeration to define interrupt type
*/
enum gdma_isr_type_e {
  TransferType        = 0x1,     //!<  Trigger interrupt when the transfer is complete
  BlockType           = 0x2,     //!<  Trigger interrupt when a block transfer is done
  SrcTransferType     = 0x4,     //!<  Trigger interrupt when the transfer on the source is complete
  DstTransferType     = 0x8,     //!<  Trigger interrupt when the transfer on the destination is complete
  ErrType             = 0x10     //!<  Trigger interrupt when any error occurs
};
typedef uint8_t gdma_isr_type_t;

/**
  \brief  Enumeration to define secure state of the GDMA transfer
*/
enum gdma_secure_type_e {
  SecureType          = 0x0,     //!<  This transfer is under secure mode
  NonSecureType       = 0x1      //!<  This transfer is under non-secure mode
};
typedef uint8_t gdma_secure_type_t;

/**
  \brief  The data struct of the control register
*/
typedef struct _gdma_ctl_reg_s {
    gdma_tt_fc_type_t       tt_fc;                  //!< The flow control type
    gdma_ctl_tr_width_t     dst_tr_width;           //!< The data width of the destination side
    gdma_ctl_tr_width_t     src_tr_width;           //!< The data width of the source side
    gdma_inc_type_t         dinc;                   //!< The destination address increment setting
    gdma_inc_type_t         sinc;                   //!< The source address increment setting
    gdma_ctl_msize_t        dest_msize;             //!< The transaction burst size of the detination side
    gdma_ctl_msize_t        src_msize;              //!< The transaction burst size of the source side
    u32                     block_size;             //!< The size of the block(transfer length)
    u8                      int_en           :1;    //!< bit 0 : Interrupt enable control
    u8                      llp_dst_en       :1;    //!< bit 1 : linked list enable control on the destination side
    u8                      llp_src_en       :1;    //!< bit 2 : linked list enable control on the source side
    u8                      rsvd             :1;    //!< bit 3 : Reserve
}gdma_ctl_reg_t, *pgdma_ctl_reg_t;

/**
  \brief  The data struct of the configuration register
*/
typedef struct _gdma_cfg_reg_s {
    u8                     src_per;                 //!<  The hardware handshake number of the source peripheral
    u8                     dest_per;                //!<  The hardware handshake number of the destination peripheral
    u8                     rsvd;                    //!<  Reserve
    u8                     ch_susp      :1;         //!<  bit 0 : Channel suspend control
    u8                     fifo_empty   :1;         //!<  bit 1 : Indicate fifo is empty or not
    u8                     hs_sel_dst   :1;         //!<  bit 2 : Destination software or hardware handshaking select. Don't have to configure.
    u8                     hs_sel_src   :1;         //!<  bit 3 : Source software or hardware handshaking select. Don't have to configure.
    u8                     dst_hs_pol   :1;         //!<  bit 4 : Destination handshaking interface polarity. Don't have to configure.
    u8                     src_hs_pol   :1;         //!<  bit 5 : Source handshaking interface polarity. Don't have to configure.
    u8                     reload_src   :1;         //!<  bit 6 : Source auto reload enbale control.
    u8                     reload_dst   :1;         //!<  bit 7 : Destination auto reload enable control.
}gdma_cfg_reg_t, *pgdma_cfg_reg_t;

/**
  \brief  The data struct of the gdma adaptor
*/
typedef struct _hal_gdma_adaptor_s {
    GDMA0_Type                  *gdma_dev;                                                      //!<  The base address of GDMA common register
    GDMA0_CH0_Type              *chnl_dev;                                                      //!<  The base address of GDMA channel register
    gdma_chnl_num_t             ch_en;                                                          //!<  Channel enable control
    gdma_ctl_reg_t              gdma_ctl;                                                       //!<  The data struct of control register
    gdma_cfg_reg_t              gdma_cfg;                                                       //!<  The data struct of configuration register
    gdma_callback_t             gdma_cb_func;                                                   //!<  The pointer of gdma callback function
    void                        *gdma_cb_para;                                                  //!<  The parameter of gdma callback function
    irq_handler_t               gdma_irq_func;                                                  //!<  The pointer of gdma irq function
    void                        *gdma_irq_para;                                                 //!<  The parameter of gdma callback function
    void                        (*dcache_invalidate_by_addr)(uint32_t *addr, int32_t dsize);    //!<  callback function to do the D-cache invalidate
    void                        (*dcache_clean_by_addr) (uint32_t *addr, int32_t dsize);        //!<  callback function to do the D-cache clean
    u32                         rsvd;                                                           //!<  Reserve
    u32                         ch_sar;                                                         //!<  The source address of the current transfer
    u32                         ch_dar;                                                         //!<  The destination address of the current transfer
    u32                         gdma_irq_num;                                                   //!<  The IRQ number
    u32                         abort_recv_byte;
    volatile u8                 busy;                                                           //!<  Indicate the busy status of this GDMA channel
    u8                          ch_num;                                                         //!<  The channel number being allocated to this adaptor
    u8                          gdma_index;                                                     //!<  The GDMA index being allocated to this adaptor
    u8                          gdma_isr_type;                                                  //!<  The interrupt mask setting for the transfer
    u8                          have_chnl;                                                      //!<  Indicate the adaptor owns a channel or not
}hal_gdma_adaptor_t, *phal_gdma_adaptor_t;


/**
  \brief  The data struct that lists available GDMA and channels
*/
typedef struct _hal_gdma_chnl_s {
    u8 gdma_indx;               //!<  Available GDMA index that can be allocated
    u8 gdma_chnl;               //!<  Available GDMA channels that can be allocated
}hal_gdma_chnl_t, *phal_gdma_chnl_t;

/**
  \brief  The data struct of GDMA group adaptor
*/
typedef struct _hal_gdma_group_s {
    phal_gdma_adaptor_t phal_gdma_adaptor[MAX_GDMA_CHNL];    //!<  The pointers of GDMA adaptor for all available GDMA sets and channels
    u8 chnl_in_use;                                            //!<  Record which channel is used, it is stored separately under secure and non-secure mode
#if !defined (CONFIG_BUILD_NONSECURE)
    u8 hal_gdma_reg;                                           //!<  Record which channel is used and registered. Globally managed under secure mode.
#endif
} hal_gdma_group_t, *phal_gdma_group_t;

/**
  \brief  The data struct of GDMA stub functions. ROM code functions are accessed in RAM code through stub functions.
*/
typedef struct hal_gdma_func_stubs_s {
    phal_gdma_group_t *pphal_gdma_group;
    void (*hal_gdma_on) (phal_gdma_adaptor_t phal_gdma_adaptor);
    void (*hal_gdma_off) (phal_gdma_adaptor_t phal_gdma_adaptor);
    void (*hal_gdma_chnl_en) (phal_gdma_adaptor_t phal_gdma_adaptor);
    void (*hal_gdma_chnl_dis) (phal_gdma_adaptor_t phal_gdma_adaptor);
    void (*hal_gdma_isr_en) (phal_gdma_adaptor_t phal_gdma_adaptor);
    void (*hal_gdma_isr_dis) (phal_gdma_adaptor_t phal_gdma_adaptor);
    void (*hal_gdma_clean_pending_isr) (phal_gdma_adaptor_t phal_gdma_adaptor);
    void (*hal_gdma_clean_chnl_isr) (phal_gdma_adaptor_t phal_gdma_adaptor);
    void (*hal_gdma_chnl_clean_auto_src) (phal_gdma_adaptor_t phal_gdma_adaptor);
    void (*hal_gdma_chnl_clean_auto_dst) (phal_gdma_adaptor_t phal_gdma_adaptor);
    hal_status_t (*hal_gdma_chnl_setting) (phal_gdma_adaptor_t phal_gdma_adaptor);
    u32 (*hal_gdma_query_dar) (phal_gdma_adaptor_t phal_gdma_adaptor);
    u32 (*hal_gdma_query_sar) (phal_gdma_adaptor_t phal_gdma_adaptor);
    BOOL (*hal_gdma_query_chnl_en) (phal_gdma_adaptor_t phal_gdma_adaptor);
    u32 (*hal_gdma_query_transfer_bytes) (phal_gdma_adaptor_t phal_gdma_adaptor);
#if !defined (CONFIG_BUILD_NONSECURE)
    hal_status_t (*hal_gdma_chnl_register) (u8 gdma_index, u8 chnl_num);
    hal_status_t (*hal_gdma_chnl_unregister) (phal_gdma_adaptor_t phal_gdma_adaptor);
#endif
    hal_status_t (*hal_gdma_chnl_init) (phal_gdma_adaptor_t phal_gdma_adaptor);
    void (*hal_gdma_chnl_irq_free) (phal_gdma_adaptor_t phal_gdma_adaptor);
    void (*hal_gdma_memcpy_irq_hook) (phal_gdma_adaptor_t phal_gdma_adaptor, gdma_callback_t gdma_cb_func, void *gdma_cb_data);
    void (*hal_gdma_memcpy_irq_handler) (phal_gdma_adaptor_t phal_gdma_adaptor);
    void (*hal_gdma0_irq_handler) (void);
    void (*hal_gdma1_irq_handler) (void);
    void (*hal_gdma_irq_set_priority) (phal_gdma_adaptor_t phal_gdma_adaptor, u32 irq_priority);
    void (*hal_gdma_irq_reg) (phal_gdma_adaptor_t phal_gdma_adaptor, irq_handler_t irq_handler, void *irq_data);
    void (*hal_gdma_transfer_start) (phal_gdma_adaptor_t phal_gdma_adaptor);
    void (*hal_gdma_group_init) (phal_gdma_group_t pgdma_group);
    hal_status_t (*hal_gdma_memcpy_config) (phal_gdma_adaptor_t phal_gdma_adaptor, void *pdest, void *psrc, u32 len);
    void (*hal_gdma_abort) (phal_gdma_adaptor_t phal_gdma_adaptor);
    void (*hal_gdma_chnl_reset) (phal_gdma_adaptor_t phal_gdma_adaptor);
    uint32_t reserved[12];  // reserved space for next ROM code version function table extending.
} hal_gdma_func_stubs_t;


/// @cond DOXYGEN_ROM_HAL_API

/**

        \addtogroup hs_hal_gdma_rom_func GDMA HAL ROM APIs
        @{
*/


void hal_gdma_on_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
void hal_gdma_off_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
void hal_gdma_chnl_en_rtl8710c (phal_gdma_adaptor_t phal_gdma_adaptor);
void hal_gdma_chnl_dis_rtl8710c (phal_gdma_adaptor_t phal_gdma_adaptor);
void hal_gdma_isr_en_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
void hal_gdma_isr_dis_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
void hal_gdma_clean_pending_isr_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
void hal_gdma_clean_chnl_isr_rtl8710c (phal_gdma_adaptor_t phal_gdma_adaptor);
void hal_gdma_chnl_clean_auto_src_rtl8710c (phal_gdma_adaptor_t phal_gdma_adaptor);
void hal_gdma_chnl_clean_auto_dst_rtl8710c (phal_gdma_adaptor_t phal_gdma_adaptor);
hal_status_t hal_gdma_chnl_setting_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
u32 hal_gdma_query_dar_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
u32 hal_gdma_query_sar_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
BOOL hal_gdma_query_chnl_en_rtl8710c (phal_gdma_adaptor_t phal_gdma_adaptor);
u32 hal_gdma_query_transfer_bytes_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
hal_status_t hal_gdma_chnl_register_rtl8710c(u8 gdma_index, u8 chnl_num);
hal_status_t hal_gdma_chnl_unregister_rtl8710c (phal_gdma_adaptor_t phal_gdma_adaptor);
hal_status_t hal_gdma_chnl_init_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
void hal_gdma_chnl_irq_free_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
void hal_gdma_memcpy_irq_hook_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor, gdma_callback_t gdma_cb_func, void *gdma_cb_data);
void hal_gdma_memcpy_irq_handler_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
void GDMA0_IRQHandler(void);
void hal_gdma_irq_set_priority_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor, u32 irq_priority);
void hal_gdma_irq_reg_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor, irq_handler_t irq_handler, void *irq_data);
void hal_gdma_transfer_start_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
void hal_gdma_group_init_rtl8710c(phal_gdma_group_t pgdma_group);
hal_status_t hal_gdma_memcpy_config_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor, void *pdest, void *psrc, u32 len);
void hal_gdma_abort_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
void hal_gdma_chnl_reset_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);

#if !defined (CONFIG_BUILD_NONSECURE)
void hal_gdma_chnl_free_rtl8710c (phal_gdma_adaptor_t phal_gdma_adaptor);
hal_status_t hal_gdma_chnl_alloc_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
BOOL hal_gdma_memcpy_init_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
void hal_gdma_memcpy_deinit_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor);
hal_status_t hal_gdma_memcpy_rtl8710c(phal_gdma_adaptor_t phal_gdma_adaptor, void *pdest, void *psrc, u32 len);

#endif

/** *@} */ /* End of group hs_hal_gdma_rom_func */

/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */


/** *@} */ /* End of group hs_hal_gdma */


#endif /* RTL8710C_GDMA_H */


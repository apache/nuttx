/**************************************************************************************************

    Phyplus Microelectronics Limited confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Phyplus Microelectronics
    Limited ("Phyplus"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of Phyplus. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/

#ifndef _LL_HW_DRV_H_
#define _LL_HW_DRV_H_

#include "types.h"
#include "ll_def.h"
#include "bus_dev.h"
#include "rf_phy_driver.h"

// LL_HW_REGISTER_ADDRESS
#define BB_HW_BASE                              0x40030000          //BB_HW Base address
#define LL_HW_BASE                              0x40031000          //LL_HW Base address
#define LL_HW_TFIFO                             (LL_HW_BASE+0x400)
#define LL_HW_RFIFO                             (LL_HW_BASE+0xC00)

#define LL_HW_WRT_EMPTY_PKT                     *(volatile uint32_t *)(LL_HW_TFIFO) = 0x00000001

//LL_HW_MODE
#define LL_HW_STX                               0x0000
#define LL_HW_SRX                               0x0001

#define LL_HW_TRX                               0x0010
#define LL_HW_RTX                               0x0012

#define LL_HW_TRLP                              0x0020
#define LL_HW_TRLP_EMPT                         0x0022

#define LL_HW_RTLP                              0x0030
#define LL_HW_RTLP_1ST                          0x0031
#define LL_HW_RTLP_EMPT                         0x0032

#define HCLK16M

//#define LL_HW_CYCLES_PER_US               CYCLES_PER_US
#ifdef HCLK16M
    #define LL_HW_HCLK_PER_US                           16              //
    #define LL_HW_HCLK_PER_US_BITS                      4
#else
    #define LL_HW_HCLK_PER_US                           32              //
    #define LL_HW_HCLK_PER_US_BITS                      5
#endif

#define LL_HW_FIFO_MARGIN                           70              //


// LL_HW_IRQ_STATUS
#define LIRQ_MD                                 0x0001              //bit00
#define LIRQ_CERR                               0x0002              //bit01
#define LIRQ_RTO                                0x0004              //bit02
#define LIRQ_RFULL                              0x0008              //bit03
#define LIRQ_RHALF                              0x0010              //bit04
#define LIRQ_BIT5                               0x0020              //bit05
#define LIRQ_BIT6                               0x0040              //bit06
#define LIRQ_BIT7                               0x0080              //bit07
#define LIRQ_TD                                 0x0100              //bit08
#define LIRQ_RD                                 0x0200              //bit09
#define LIRQ_COK                                0x0400              //bit10
#define LIRQ_CERR2                              0x0800              //bit11
#define LIRQ_LTO                                0x1000              //bit12
#define LIRQ_NACK                               0x2000              //bit13
#define LIRQ_BIT14                              0x4000              //bit14
#define LIRQ_BIT15                              0x8000              //bit15

#define LL_HW_IRQ_MASK                          0x3FFF              //total 14bit

// LL_HW_RFIFO_CTRL
#define LL_HW_IGN_EMP                           0x0001              //bit0
#define LL_HW_IGN_CRC                           0x0002              //bit1
#define LL_HW_IGN_SSN                           0x0004              //bit2
#define LL_HW_IGN_ALL                           0x0007              //bit2
#define LL_HW_IGN_NONE                          0x0000

// LL_MD_RX_INI
#define LL_HW_MD_RX_SET0                        0x4000              //set md_rx ini=0 and md_rx_soft=0
#define LL_HW_MD_RX_SET1                        0x4440              //set md_rx ini=1 and md_rx_soft=1

//LL FIFO DEPTH CONFIG
#define LL_HW_FIFO_TX_2K_RX_2K                  0x0200              //TX FIFO 512 Word 
#define LL_HW_FIFO_TX_3K_RX_1K                  0x0300              //TX FIFO 768 Word 
#define LL_HW_FIFO_TX_1K_RX_3K                  0x0100              //TX FIFO 256 Word 

//BB CRC Format Setting
#define LL_HW_CRC_BLE_FMT                       0x02
#define LL_HW_CRC_ZB_FMT                        0x03
#define LL_HW_CRC_16_FMT                        0x04
#define LL_HW_CRC_NULL                          0x00


//ANT SWITCH SLOT
#define LL_HW_ANT_WIN_1us                       4
#define LL_HW_ANT_SW_CTE_OFF                    0x00
#define LL_HW_ANT_SW_CTE_AUTO                   0x01
#define LL_HW_ANT_SW_TX_MANU                    0x02
#define LL_HW_ANT_SW_RX_MANU                    0x04

//CTE Supplement Config
#define CTE_SUPP_AUTO                           0xC0
#define CTE_SUPP_LEN_SET                        0x00
#define CTE_SUPP_NULL                           0x00

#define CONNLESS_CTE_TYPE_AOA                   0x00
#define CONNLESS_CTE_TYPE_AOD_1us               0x01
#define CONNLESS_CTE_TYPE_AOD_2us               0x02
#define CONN_CTE_TYPE_AOA                       0x01
#define CONN_CTE_TYPE_AOD_1us                   0x02
#define CONN_CTE_TYPE_AOD_2us                   0x04


// 2020-01-21 add for CONN CTE REQ TYPE
#define CTE_REQ_TYPE_AOA                        0x00
#define CTE_REQ_TYPE_AOD_1US                    0x01
#define CTE_REQ_TYPE_AOD_2US                    0x02

#define BLE_HEAD_WITH_CTE(x)                   (((x & 0x20)==0x00) ? 0:1)



void     ll_hw_set_stx(void);
void     ll_hw_set_srx(void);
void     ll_hw_set_trx(void);
void     ll_hw_set_rtx(void);
void     ll_hw_set_trlp(uint8_t snNesn,uint8_t txPktNum,uint8_t rxPktNum,uint8_t mdRx);
void     ll_hw_set_rtlp(uint8_t snNesn,uint8_t txPktNum,uint8_t rxPktNum,uint8_t mdRx,uint32_t rdCntIni);
void     ll_hw_set_rtlp_1st(uint8_t snNesn,uint8_t txPktNum,uint8_t rxPktNum,uint8_t mdRx);
void     ll_hw_config(uint8_t ll_mode,uint8_t snNesn,uint8_t txPktNum,uint8_t rxPktNum,uint8_t mdRx,uint32_t rdCntIni);



void     ll_hw_go(void);
void     ll_hw_trigger(void);
void     ll_hw_clr_irq(void);
void     ll_hw_set_irq(uint32_t mask);
void     ll_hw_set_empty_head(uint16_t txHeader);
void     ll_hw_set_rx_timeout_1st(uint32_t rxTimeOut);
void     ll_hw_set_rx_timeout(uint32_t rxTimeOut);
void     ll_hw_set_tx_rx_release(uint16_t txTime,uint16_t rxTime);
void     ll_hw_set_rx_tx_interval(uint32_t intvTime);
void     ll_hw_set_tx_rx_interval(uint32_t intvTime);
void     ll_hw_set_trx_settle(uint8_t tmBb,uint8_t tmAfe,uint8_t tmPll);
void     ll_hw_set_loop_timeout(uint32_t loopTimeOut);
void     ll_hw_set_loop_nack_num(uint8_t nAckNum);
void     ll_hw_set_timing(uint8_t pktFmt);

void     ll_hw_set_tfifo_space(uint16 space);

void     ll_hw_set_ant_switch_mode(uint8_t mode);
void     ll_hw_set_ant_switch_timing(uint8_t antWin,uint8_t antDly);
void     ll_hw_set_ant_pattern(uint32_t ant1, uint32_t ant0);

void     ll_hw_set_cte_rxSupp(uint8_t rxSupp);
void     ll_hw_set_cte_txSupp(uint8_t txSupp);
uint8_t  ll_hw_get_iq_RawSample(uint16_t* p_iSample, uint16_t* p_qSample);


void     ll_hw_rst_rfifo(void);
void     ll_hw_rst_tfifo(void);

void     ll_hw_ign_rfifo(uint8_t ignCtrl);

void     ll_hw_get_tfifo_info(int* rdPtr,int* wrPtr,int* wrDepth);
void     ll_hw_get_rfifo_info(int* rdPtr,int* wrPtr,int* rdDepth);
void     ll_hw_get_rxPkt_stats(uint8_t* crcErrNum,uint8_t* rxTotalNum,uint8_t* rxPktNum);

uint8_t  ll_hw_read_rfifo(uint8_t* rxPkt, uint16_t* pktLen, uint32_t* pktFoot0, uint32_t* pktFoot1);
uint8_t  ll_hw_read_rfifo_zb(uint8_t* rxPkt, uint16_t* pktLen, uint32_t* pktFoot0, uint32_t* pktFoot1);
uint8_t  ll_hw_read_rfifo_pplus(uint8_t* rxPkt, uint16_t* pktLen, uint32_t* pktFoot0, uint32_t* pktFoot1);

uint8_t  ll_hw_write_tfifo(uint8_t* rxPkt, uint16_t pktLen);

void     ll_hw_set_crc_fmt(uint8_t txCrc,uint8_t rxCrc);
void     ll_hw_set_pplus_pktfmt(uint8_t plen);

uint8_t  ll_hw_get_snNesn(void);
uint8_t  ll_hw_get_txAck(void);
uint8_t  ll_hw_get_nAck(void);
uint8_t  ll_hw_get_rxPkt_num(void);
uint32_t ll_hw_get_anchor(void);
uint32_t ll_hw_get_irq_status(void);
uint8_t  ll_hw_get_fsm_status(void);
uint8_t  ll_hw_get_last_ack(void);
uint32_t ll_hw_get_loop_cycle(void);

uint8_t  ll_hw_get_rxPkt_Total_num(void);
uint8_t  ll_hw_get_rxPkt_CrcErr_num(void);
uint8_t  ll_hw_get_rxPkt_CrcOk_num(void);

uint8_t  ll_hw_get_iq_RawSample(uint16_t* p_iSample, uint16_t* p_qSample);

uint8_t  ll_hw_update_rtlp_mode(uint8_t llMode);
uint8_t  ll_hw_update_trlp_mode(uint8_t llMode);
uint8_t  ll_hw_update(uint8_t llMode,uint8_t* txAck,uint8_t* rxRec,uint8_t* snNesn);


void     byte_to_bit(uint8_t byteIn,uint8_t* bitOut);
void     bit_to_byte(uint8_t* bitIn,uint8_t* byteOut);
void     zigbee_crc16_gen(uint8_t* dataIn,int length,uint8_t* seed,uint8_t* crcCode);


//  copy from rf.h by Zeng jiaping
void set_tx_rx_mode(uint8_t mode);

void set_channel(uint32_t  channel);

void set_access_address( uint32_t  access);
void set_crc_seed(uint32_t  seed);
void set_whiten_seed(uint32_t channel);

void set_max_length(uint32_t length);

void calculate_whiten_seed(void);


#endif

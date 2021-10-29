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

/*******************************************************************************
    @file     rf_phy_driver.h
    @brief    Contains all functions support for PHYPLUS RF_PHY_DRIVER
    @version  1.0
    @date     24. Aug. 2017
    @author   Zhongqi Yang



*******************************************************************************/
#ifndef __RF_PHY_DRIVER_H_
#define __RF_PHY_DRIVER_H_




/*******************************************************************************
    INCLUDES
*/


#include "rom_sym_def.h"
#include "clock.h"
#include "ll_hw_drv.h"
#include "jump_function.h"
#include "version.h"


typedef enum  _RF_PHY_CLK_SEL
{
    RF_PHY_CLK_SEL_16M_XTAL     = 0,
    RF_PHY_CLK_SEL_32M_DBL_B    = 1,
    RF_PHY_CLK_SEL_32M_DBL      = 2,
    RF_PHY_CLK_SEL_32M_DLL      = 3
} rfphy_clk_t;

typedef enum  _RX_ADC_CLK_SEL
{
    RX_ADC_CLK_SEL_16M_XTAL     = 0,
    RX_ADC_CLK_SEL_32M_DBL_B    = 1,
    RX_ADC_CLK_SEL_32M_DBL      = 2,
    RX_ADC_CLK_SEL_32M_DLL      = 3
} rxadc_clk_t;




/*******************************************************************************
    Global Var
*/
extern volatile uint8_t g_rfPhyTpCal0;            //** two point calibraion result0            **//
extern volatile uint8_t g_rfPhyTpCal1;            //** two point calibraion result1            **//
extern volatile uint8_t g_rfPhyTpCal0_2Mbps;            //** two point calibraion result0            **//
extern volatile uint8_t g_rfPhyTpCal1_2Mbps;            //** two point calibraion result1            **//
extern volatile uint8_t g_rfPhyTxPower;           //** rf pa output power setting [0x00 0x1f]  **//
extern volatile uint8_t g_rfPhyPktFmt;            //** rf_phy pkt format config                **//
extern volatile uint32  g_rfPhyRxDcIQ;            //** rx dc offset cal result                 **//
extern volatile int8_t  g_rfPhyFreqOffSet;

extern volatile sysclk_t g_system_clk;
extern volatile rfphy_clk_t g_rfPhyClkSel;
extern volatile rxadc_clk_t g_rxAdcClkSel;

extern volatile uint8_t   g_rfPhyDtmCmd[];
extern volatile uint8_t   g_rfPhyDtmEvt[];
extern volatile uint8_t   g_dtmModeType ;
extern volatile uint8_t   g_dtmCmd      ;
extern volatile uint8_t   g_dtmFreq     ;
extern volatile uint8_t   g_dtmLength   ;
extern volatile uint8_t   g_dtmExtLen   ;
extern volatile uint16_t  g_dtmPktIntv  ;
extern volatile uint8_t   g_dtmPKT      ;
extern volatile uint8_t   g_dtmCtrl     ;
extern volatile uint8_t   g_dtmPara     ;
extern volatile uint8_t   g_dtmEvt      ;
extern volatile uint8_t   g_dtmStatus   ;
extern volatile uint16_t  g_dtmPktCount ;
extern volatile uint16_t  g_dtmRxCrcNum ;
extern volatile uint16_t  g_dtmRxTONum  ;
extern volatile uint16_t  g_dtmRsp      ;
extern volatile uint8_t   g_dtmTxPower  ;//RF_PHY_TX_POWER_EXTRA_MAX;//according to the rfdrv
extern volatile uint16_t  g_dtmFoff     ;
extern volatile uint8_t   g_dtmRssi     ;
extern volatile uint8_t   g_dtmCarrSens ;
extern volatile uint8_t   g_dtmTpCalEnable  ;  //default enable tpcal
extern volatile uint32_t  g_dtmTick         ;
extern volatile uint32_t  g_dtmPerAutoIntv  ;
extern volatile uint32_t  g_dtmAccessCode   ;

extern volatile uint8_t   g_rc32kCalRes   ;
/*******************************************************************************
    MACRO
*/
#define RF_PHY_EXT_PREAMBLE_US                      (8)             // ext ble preamble length

#define PHY_REG_RD(x)                               *(volatile uint32_t *)(x)
#define PHY_REG_WT(x,y)                             *(volatile uint32_t *)(x) = (y)
#define RF_CHN_TO_FREQ(x)

#define DCDC_REF_CLK_SETTING(x)                     subWriteReg(0x4000f014,25,25, (0x01&(x)))
#define DCDC_CONFIG_SETTING(x)                      subWriteReg(0x4000f014,18,15, (0x0f&(x)))

/*
    crystal 16M matching cap control for ana.
    5'b0 means 5pF,5'b11110 means 17pF.step size is 2.step value is 0.8pF.
*/
#define XTAL16M_CAP_SETTING(x)                      subWriteReg(0x4000f0bc, 4, 0, (0x1f&(x)))

#define XTAL16M_CURRENT_SETTING(x)                  subWriteReg(0x4000f0bc, 6, 5, (0x03&(x)))
#define DIG_LDO_CURRENT_SETTING(x)                  subWriteReg(0x4000f014,22,21, (0x03&(x)))

#define RF_PHY_LO_LDO_SETTING(x)                    subWriteReg(0x400300cc,11,10, (0x03&(x)))
#define RF_PHY_PA_VTRIM_SETTING(x)                  subWriteReg(0x400300dc, 9, 7, (0x03&(x)))
#define RF_PHY_LNA_LDO_SETTING(x)                   subWriteReg(0x400300dc, 6, 5, (0x03&(x)))


#define RF_PHY_TPCAL_CALC(tp0,tp1,chn)              ((tp0)>(tp1) ?(((tp0<<5)-(tp0-tp1)*(chn)+16)>>5) : tp0 )
//DTM STATE
#define RF_PHY_DTM_IDL                              0
#define RF_PHY_DTM_CMD                              1
#define RF_PHY_DTM_EVT                              2
#define RF_PHY_DTM_TEST                             3

#define RF_PHY_DTM_SYNC_WORD                        0x71764129
#define RF_PHY_DTM_PRBS9_SEED                       0xffffffff
#define RF_PHY_DTM_CRC_WT                           0x00555555

//DTM MODE TYPE
#define RF_PHY_DTM_MODE_RESET                       0
#define RF_PHY_DTM_MODE_TX_BURST                    2
#define RF_PHY_DTM_MODE_TX_CTMOD                    4
#define RF_PHY_DTM_MODE_TX_SINGLE                   6
#define RF_PHY_DTM_MODE_RX_PER                      8
#define RF_PHY_DTM_MODE_TEST_END                    10
#define RF_PHY_DTM_MODE_SET_LENGTH_UP2BIT           12

#define RF_PHY_DTM_MODE_SET_PHY_1M                  16
#define RF_PHY_DTM_MODE_SET_PHY_2M                  18
#define RF_PHY_DTM_MODE_SET_PHY_500K                20
#define RF_PHY_DTM_MODE_SET_PHY_125K                22
#define RF_PHY_DTM_MODE_SET_PHY_ZB                  24

#define RF_PHY_DTM_MODE_ASSUME_TX_MOD_INDX_STANDARD 32
#define RF_PHY_DTM_MODE_ASSUME_TX_MOD_INDX_STABLE   34
#define RF_PHY_DTM_MODE_READ_SUPPORTED_TEST_CASE    36
#define RF_PHY_DTM_MODE_READ_MAX_TX_OCTETS          38
#define RF_PHY_DTM_MODE_READ_MAX_TX_TIME            40
#define RF_PHY_DTM_MODE_READ_MAX_RX_OCTETS          42
#define RF_PHY_DTM_MODE_READ_MAX_RX_TIME            44

#define RF_PHY_DTM_MODE_SET_ACCCODE_0               114
#define RF_PHY_DTM_MODE_SET_ACCCODE_1               116
#define RF_PHY_DTM_MODE_SET_ACCCODE_2               118
#define RF_PHY_DTM_MODE_SET_ACCCODE_3               120

#define RF_PHY_DTM_MODE_SET_FREQ_FOFF               122
#define RF_PHY_DTM_MODE_SET_TPCAL_MANUAL            124
#define RF_PHY_DTM_MODE_SET_XTAL_CAP                126
#define RF_PHY_DTM_MODE_SET_TX_POWER                128
#define RF_PHY_DTM_MODE_GET_FOFF                    130
#define RF_PHY_DTM_MODE_GET_TPCAL                   132
#define RF_PHY_DTM_MODE_GET_RSSI                    134
#define RF_PHY_DTM_MODE_GET_CARR_SENS               136
#define RF_PHY_DTM_MODE_GET_PER_AUTO                138

#define RF_PHY_DTM_MODE_ATE_SET_PKTFMT              0xd0 //208
#define RF_PHY_DTM_MODE_ATE_SET_TXPOWER             0xd1

#define RF_PHY_DTM_MODE_ATE_TX_BURST                0xe0 //224
#define RF_PHY_DTM_MODE_ATE_TX_MOD                  0xe1
#define RF_PHY_DTM_MODE_ATE_TX_CARR                 0xe2
#define RF_PHY_DTM_MODE_ATE_RX_AUTOGAIN             0xe3
#define RF_PHY_DTM_MODE_ATE_RX_FIXGAIN              0xe4
#define RF_PHY_DTM_MODE_ATE_RX_DEMOD                0xe5
#define RF_PHY_DTM_MODE_ATE_RX2TX                   0xe6
#define RF_PHY_DTM_MODE_ATE_TX2RX                   0xe7

#define RF_PHY_DTM_MODE_ATE_RESET                   0xef

#define RF_PHY_DTM_MODE_ERROR                       254


/*******************************************************************************
    CONSTANTS
*/
#define PKT_FMT_ZIGBEE                              0
#define PKT_FMT_BLE1M                               1
#define PKT_FMT_BLE2M                               2
#define PKT_FMT_BLR500K                             3
#define PKT_FMT_BLR125K                             4


#if (SDK_VER_CHIP==__DEF_CHIP_QFN32__)
    #define RF_PHY_TX_POWER_EXTRA_MAX                   0x3f
    #define RF_PHY_TX_POWER_MAX                         0x1f
    #define RF_PHY_TX_POWER_MIN                         0x00

    #define RF_PHY_TX_POWER_5DBM                        0x3f
    #define RF_PHY_TX_POWER_0DBM                        0x1f
    #define RF_PHY_TX_POWER_N2DBM                       0x0f
    #define RF_PHY_TX_POWER_N5DBM                       0x0a
    #define RF_PHY_TX_POWER_N20DBM                      0x01

    #elif(SDK_VER_CHIP==__DEF_CHIP_TSOP16__)
    #define RF_PHY_TX_POWER_EXTRA_MAX                   0x3f
    #define RF_PHY_TX_POWER_MAX                         0x1f
    #define RF_PHY_TX_POWER_MIN                         0x00

    #define RF_PHY_TX_POWER_5DBM                        0x1d
    #define RF_PHY_TX_POWER_4DBM                        0x17
    #define RF_PHY_TX_POWER_3DBM                        0x15
    #define RF_PHY_TX_POWER_0DBM                        0x0d

    #define RF_PHY_TX_POWER_N2DBM                       0x0a
    #define RF_PHY_TX_POWER_N5DBM                       0x06
    #define RF_PHY_TX_POWER_N6DBM                       0x05
    #define RF_PHY_TX_POWER_N10DBM                      0x03
    #define RF_PHY_TX_POWER_N15DBM                      0x02
    #define RF_PHY_TX_POWER_N20DBM                      0x01
#else
    #warning" CHECK Chip Version "
#endif

#define RF_PHY_FREQ_FOFF_00KHZ                      0
#define RF_PHY_FREQ_FOFF_20KHZ                      5
#define RF_PHY_FREQ_FOFF_40KHZ                      10
#define RF_PHY_FREQ_FOFF_60KHZ                      15
#define RF_PHY_FREQ_FOFF_80KHZ                      20
#define RF_PHY_FREQ_FOFF_100KHZ                     25
#define RF_PHY_FREQ_FOFF_120KHZ                     30
#define RF_PHY_FREQ_FOFF_140KHZ                     35
#define RF_PHY_FREQ_FOFF_160KHZ                     40
#define RF_PHY_FREQ_FOFF_180KHZ                     45
#define RF_PHY_FREQ_FOFF_200KHZ                     50
#define RF_PHY_FREQ_FOFF_N20KHZ                     -5
#define RF_PHY_FREQ_FOFF_N40KHZ                     -10
#define RF_PHY_FREQ_FOFF_N60KHZ                     -15
#define RF_PHY_FREQ_FOFF_N80KHZ                     -20
#define RF_PHY_FREQ_FOFF_N100KHZ                    -25
#define RF_PHY_FREQ_FOFF_N120KHZ                    -30
#define RF_PHY_FREQ_FOFF_N140KHZ                    -35
#define RF_PHY_FREQ_FOFF_N160KHZ                    -40
#define RF_PHY_FREQ_FOFF_N180KHZ                    -45
#define RF_PHY_FREQ_FOFF_N200KHZ                    -50


#define RF_PHY_DTM_MANUL_NULL                        0x00
#define RF_PHY_DTM_MANUL_FOFF                        0x01
#define RF_PHY_DTM_MANUL_TXPOWER                     0x02
#define RF_PHY_DTM_MANUL_XTAL_CAP                    0x04
#define RF_PHY_DTM_MANUL_MAX_GAIN                    0x08

#define RF_PHY_DTM_MANUL_ALL                         0xFF
/*******************************************************************************
    FUNCION DEFINE
*/
/**************************************************************************************
    @fn          rf_phy_ini

    @brief       This function process for rf phy ini call api

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      None.
*/
void        rf_phy_ini      (void);

/**************************************************************************************
    @fn          rf_phy_ana_cfg

    @brief       This function process for rf phy analog block config,
                include PLL, RX_FRONT_END,PA Power.

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      None.
*/
void        rf_phy_ana_cfg  (void);

/**************************************************************************************
    @fn          rf_phy_bb_cfg

    @brief       This function process for rf phy baseband tx and rx config.

    input parameters

    @param       pktMod:0 for Zigbee, 1 for BLE1M, 2 for BLE2M, 3or4 for BLELR.

    output parameters

    @param       None.

    @return      None.
*/
void        rf_phy_bb_cfg   (uint8_t pktFmt);


void        rf_phy_change_cfg0(uint8_t pktFmt);
/**************************************************************************************
    @fn          rf_tpCal_cfg

    @brief       This function process for rf tpCal config

    input parameters

    @param       rfChn:  two point calibration rf channel setting(0-80)->2400-2480MHz.

    output parameters

    @param       None.

    @return      None.
*/
void        rf_tpCal_cfg    (uint8_t rfChn);

/**************************************************************************************
    @fn          rf_tp_cal

    @brief       This function process for tx tp calibration.

    input parameters

    @param       rfChn   : rfFreq=2400+rfChn
                fDev    : used to config the tpCal fDelt, 0 for 0.5M, 1 for 1M


    output parameters

    @param       none

    @return      kCal    : cal result for rfChn.
*/
uint8_t     rf_tp_cal       (uint8_t rfChn,uint8_t fDev);

/**************************************************************************************
    @fn          rf_rxDcoc_cfg

    @brief       This function process for rx dc offset calibration and canncellation config.

    input parameters

    @param       rfChn   : rfFreq=2400+rfChn
                bwSet   : used to config rx complex filter bandwitdh. 1 for 1MHz, other for 2MHz


    output parameters

    @param       dcCal   : cal result for rxdc, dcQ[13:8],dcI[5:0]

    @return      none
*/
void        rf_rxDcoc_cfg   (uint8_t rfChn,uint8_t bwSet,volatile uint32* dcCal);

/**************************************************************************************
    @fn          rf_tpCal_gen_cap_arrary

    @brief       This function process for tx tp calibration,genearte the tpCal cap arrary.

    input parameters

    @param

    output parameters

    @param       none

    @return      kCal    : cal result for rfChn.
*/
void        rf_tpCal_gen_cap_arrary(void);

/**************************************************************************************
    @fn          rf_phy_direct_test

    @brief       This function process for rf phy direct test.

    input parameters

    @param       none

    output parameters

    @param       none

    @return      none
*/
void        rf_phy_direct_test  (void);

/**************************************************************************************
    @fn          rf_phy_dtm_cmd_parse

    @brief       This function process for rf phy direct test,cmd parse

    input parameters

    @param       none

    output parameters

    @param       none

    @return      none
*/
void        rf_phy_dtm_cmd_parse(void);

/**************************************************************************************
    @fn          rf_phy_dtm_evt_send

    @brief       This function process for rf phy direct test, test mode trigged

    input parameters

    @param       none

    output parameters

    @param       none

    @return      none
*/
void        rf_phy_dtm_evt_send (uint8_t dtmType);

/**************************************************************************************
    @fn          rf_phy_dtm_trigged

    @brief       This function process for rf phy direct test, test mode trigged

    input parameters

    @param       none

    output parameters

    @param       none

    @return      none
*/
void        rf_phy_dtm_trigged  (void);

/**************************************************************************************
    @fn          rf_phy_get_pktFoot

    @brief       This function process to get pkt foot

    input parameters

    @param       none

    output parameters

    @param       rssi    : recv signal strength indicator(-dBm)
                foff    : estimated freq offset by rx BB ,foff-512-->[-512 511]KHz
                carrSens: sync qualitiy indicator, estimated by rx BB.

    @return      none
*/
void        rf_phy_get_pktFoot  (uint8* rssi, uint16* foff,uint8* carrSens);
void        rf_phy_get_pktFoot_fromPkt(uint32 pktFoot0, uint32 pktFoot1,uint8* rssi, uint16* foff,uint8* carrSens);

/**************************************************************************************
    @fn          rf_phy_set_txPower

    @brief       This function process for rf phy tx power config

    input parameters

    @param       txPower  : tx pa power setting (0~0x1f)

    output parameters

    @param       none

    @return      none
*/
void        rf_phy_set_txPower  (uint8 txPower);

uint8_t     rf_phy_direct_test_ate(uint32_t cmdWord,uint8_t regPatchNum,uint32_t* regPatchAddr,uint32_t* regPatchVal,uint8_t* dOut);

void        rf_phy_dtm_zigbee_pkt_gen(void);

void TRNG_INIT(void);

uint8_t TRNG_Rand(uint8_t* buf,uint8_t len);
#endif

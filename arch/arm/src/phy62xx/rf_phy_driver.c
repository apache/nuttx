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
    @file     rf_phy_driver.c
    @brief    Contains all functions support for PHYPLUS RF_PHY_DRIVER
    @version  1.0
    @date     24. Aug. 2017
    @author   Zhongqi Yang



*******************************************************************************/


/*******************************************************************************
    INCLUDES
*/

#include "rf_phy_driver.h"
#include "mcu.h"
#include "clock.h"
#include "timer.h"
#include "ll_hw_drv.h"

/*******************************************************************************
    BUILD CONFIG
*/

#define RF_PHY_DTM_CTRL_NONE                            0x00
#define RF_PHY_DTM_CTRL_UART                            0x01
#define RF_PHY_DTM_CTRL_HCI                             0x02

#define RF_PHY_DTM_BB_SUPPORT_BLE1M                     0x01
#define RF_PHY_DTM_BB_SUPPORT_BLE2M                     0x02
#define RF_PHY_DTM_BB_SUPPORT_BLR500K                   0x04
#define RF_PHY_DTM_BB_SUPPORT_BLR125K                   0x08
#define RF_PHY_DTM_BB_SUPPORT_ZIGBEE                    0x10

#define RF_PHY_DTM_BB_SUPPORT_BLR_CODED                 (RF_PHY_DTM_BB_SUPPORT_BLR500K|RF_PHY_DTM_BB_SUPPORT_BLR125K)
#define RF_PHY_DTM_BB_SUPPORT_BLE_5                     (RF_PHY_DTM_BB_SUPPORT_BLE1M|RF_PHY_DTM_BB_SUPPORT_BLE2M|RF_PHY_DTM_BB_SUPPORT_BLR_CODED)
#define RF_PHY_DTM_BB_SUPPORT_FULL                      (RF_PHY_DTM_BB_SUPPORT_ZIGBEE|RF_PHY_DTM_BB_SUPPORT_BLE_5)


// TODO: move to phypuls_build_cfg.h
#define RF_PHY_DTM_CTRL_MOD                             RF_PHY_DTM_CTRL_UART
#define RF_PHY_DTM_BB_SUPPORT_MOD                       RF_PHY_DTM_BB_SUPPORT_BLE_5


#define RF_PHY_CT_MONITER                               0 // VCO corase tuning moniter counter
// 0 : disable moniter
// other: enable
#define RF_PHY_TIME_BASE                                TIME_BASE
#define RF_PHY_TIME_DELTA(x,y)                          TIME_DELTA(x,y)

/*******************************************************************************
    Global Var
*/
//volatile uint8_t        g_rfPhyTpCal0       =   0x2d;
//volatile uint8_t        g_rfPhyTpCal1       =   0x23;
//volatile uint8_t        g_rfPhyTpCal0_2Mbps =   0x47;
//volatile uint8_t        g_rfPhyTpCal1_2Mbps =   0x45;
//volatile uint8_t        g_rfPhyTxPower      =   0x0f;
//volatile uint8_t        g_rfPhyPktFmt       =   PKT_FMT_BLE1M;
//volatile uint32         g_rfPhyRxDcIQ       =   0x20200000;
//volatile int8_t         g_rfPhyFreqOffSet   =   RF_PHY_FREQ_FOFF_00KHZ;
//volatile sysclk_t       g_system_clk        =   SYS_CLK_XTAL_16M;
//volatile rfphy_clk_t    g_rfPhyClkSel       =   RF_PHY_CLK_SEL_16M_XTAL;
//volatile rxadc_clk_t    g_rxAdcClkSel       =   RX_ADC_CLK_SEL_32M_DBL;
// volatile uint8_t        g_rfPhyDtmCmd[2]    =   {0};
// volatile uint8_t        g_rfPhyDtmEvt[2]    =   {0};

// volatile uint8_t        g_dtmModeType       =   0;

// volatile uint8_t        g_dtmCmd            =   0;
// volatile uint8_t        g_dtmFreq           =   0;
// volatile uint8_t        g_dtmLength         =   0;
// volatile uint8_t        g_dtmExtLen         =   0;

// volatile uint16_t       g_dtmPktIntv        =   0;


// volatile uint8_t        g_dtmPKT            =   0;
// volatile uint8_t        g_dtmCtrl           =   0;
// volatile uint8_t        g_dtmPara           =   0;
// volatile uint8_t        g_dtmEvt            =   0;
// volatile uint8_t        g_dtmStatus         =   0;
// volatile uint16_t       g_dtmPktCount       =   0;
// volatile uint16_t       g_dtmRxCrcNum       =   0;
// volatile uint16_t       g_dtmRxTONum        =   0;
// volatile uint16_t       g_dtmRsp            =   0;

// volatile uint8_t        g_dtmTxPower        =   RF_PHY_TX_POWER_0DBM;//RF_PHY_TX_POWER_EXTRA_MAX;//according to the rfdrv
// volatile uint16_t       g_dtmFoff           =   0;
// volatile uint8_t        g_dtmRssi           =   0;
// volatile uint8_t        g_dtmCarrSens       =   0;
// volatile uint8_t        g_dtmTpCalEnable    =   1;  //default enable tpcal

// volatile uint32_t       g_dtmTick           =   0;
// volatile uint32_t       g_dtmPerAutoIntv    =   0;

// volatile uint32_t       g_dtmAccessCode     =   RF_PHY_DTM_SYNC_WORD;
volatile uint8_t        g_dtmManualConfig   =   RF_PHY_DTM_MANUL_ALL;
volatile uint8_t        g_rc32kCalRes       =   0xff;
// extern volatile int uart_rx_wIdx;                     //
// extern volatile char *urx_buf;

#if(RF_PHY_DTM_CTRL_MOD == RF_PHY_DTM_CTRL_UART)
    #include "log.h"
    #define MAX_UART_BUF_SIZE       32
    #define MAX_UART_BUF_ID         (MAX_UART_BUF_SIZE-1)
    #define _DTM_UART_              UART0
    #define DTM_OUT(x)              hal_uart_send_byte(_DTM_UART_,x)
    #define DTM_LOG_INIT(...)       dbg_printf_init()
    #define DTM_LOG(...)            dbg_printf_(__VA_ARGS__)
    #define CLR_UART_WIDX           {uart_rx_wIdx=0;}
    #define GET_UART_WIDX           (uart_rx_wIdx)
    #define DTM_ADD_IDX(a,b)        {(a==b)? a=0:a++;}
    static unsigned char            urx_buf[MAX_UART_BUF_SIZE];
    static volatile uint32_t        uart_rx_wIdx=0,uart_rx_rIdx=0;
#endif

#if(RF_PHY_CT_MONITER)
#define CT_MONT_BUFF_SIZE 128
volatile uint32_t g_rfPhy_ct_moniter_word_cnt = 0;
volatile uint32_t g_rfPhy_ct_moniter_word_target = 0;
volatile uint16_t g_rfPhy_ct_moniter_word_arry[CT_MONT_BUFF_SIZE] = {0};
#endif


void rf_tpCal_cfg_avg(uint8 rfChn,uint8 cnt);
/**************************************************************************************
    @fn          rf_phy_ini

    @brief       This function process for rf phy ini call api

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      None.
*/
void rf_phy_ini(void)
{
    g_rfPhyClkSel  = RF_PHY_CLK_SEL_16M_XTAL;
    g_rxAdcClkSel  = RX_ADC_CLK_SEL_32M_DBL;
    rf_phy_ana_cfg();
    rf_phy_set_txPower(g_rfPhyTxPower);//set to max power
    rf_phy_bb_cfg(g_rfPhyPktFmt);
    extern void ll_hw_tx2rx_timing_config(uint8 pkt);
    ll_hw_tx2rx_timing_config(g_rfPhyPktFmt);
}

/**************************************************************************************
    @fn          rf_tpCal_cfg

    @brief       This function process for rf tpCal config

    input parameters

    @param       rfChn:  two point calibration rf channel setting(0-80)->2400-2480MHz.

    output parameters

    @param       None.

    @return      None.
*/
void rf_tpCal_cfg(uint8 rfChn)
{
    if(     g_rfPhyPktFmt==PKT_FMT_BLE1M
            ||  g_rfPhyPktFmt==PKT_FMT_BLR500K
            ||  g_rfPhyPktFmt==PKT_FMT_BLR125K)
    {
        //g_rfPhyTpCal0=rf_tp_cal(rfChn,0)+5;
        //ZQ:debug 20180427
        g_rfPhyTpCal0=rf_tp_cal(rfChn,0)+8;
    }
    else if(   g_rfPhyPktFmt==PKT_FMT_BLE2M )
    {
        g_rfPhyTpCal0=rf_tp_cal(rfChn,1)+4;
    }
    else
    {
        //for ZIGBEE
        g_rfPhyTpCal0=rf_tp_cal(rfChn,1)+4;
    }
}


/**************************************************************************************
    @fn          rf_tpCal_cfg_avg

    @brief       This function process for rf tpCal config

    input parameters

    @param       rfChn:  two point calibration rf channel setting(0-80)->2400-2480MHz.

    output parameters

    @param       None.

    @return      None.
*/
void rf_tpCal_cfg_avg(uint8 rfChn,uint8 avgNum)
{
    volatile uint8_t i = 0;
    volatile uint16_t tmp=0;

    if(     g_rfPhyPktFmt==PKT_FMT_BLE1M
            ||  g_rfPhyPktFmt==PKT_FMT_BLR500K
            ||  g_rfPhyPktFmt==PKT_FMT_BLR125K)
    {
        //g_rfPhyTpCal0=rf_tp_cal(rfChn,0)+5;
        //ZQ:debug 20180427
        for ( i=0; i<(1<<avgNum); i++)
            tmp+=rf_tp_cal(rfChn,0);

        g_rfPhyTpCal0=(tmp>>avgNum)+8;
    }
    else if(   g_rfPhyPktFmt==PKT_FMT_BLE2M )
    {
        for ( i=0; i<(1<<avgNum); i++)
            tmp+=rf_tp_cal(rfChn,1);

        g_rfPhyTpCal0=(tmp>>avgNum)+4;
    }
    else
    {
        //for ZIGBEE
        for ( i=0; i<(1<<avgNum); i++)
            tmp+=rf_tp_cal(rfChn,1);

        g_rfPhyTpCal0=(tmp>>avgNum)+4;
    }
}


/**************************************************************************************
    @fn          rf_tpCal_gen_cap_arrary

    @brief       This function process for tx tp calibration,genearte the tpCal cap arrary.

    input parameters

    @param

    output parameters

    @param       none

    @return      kCal    : cal result for rfChn.
*/

void rf_tpCal_gen_cap_arrary(void)
{
    g_rfPhyTpCal0=rf_tp_cal(/*/rfChn*/2,0)+2;
    g_rfPhyTpCal1=rf_tp_cal(/*/rfChn*/66,0)+2;
    g_rfPhyTpCal0_2Mbps=rf_tp_cal(/*/rfChn*/2,1)+2;
    g_rfPhyTpCal1_2Mbps=rf_tp_cal(/*/rfChn*/66,1)+2;
}

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
void rf_phy_ana_cfg(void)
{
    //-------------------------------------------------------------------
    //               RF_PHY RX ADC CLOCK Config
    subWriteReg(0x4000f040,18,18, 0x01);  // xtal output to digital enable : ALWAYS Set 1
    subWriteReg(0x4000f044,25,24, g_rxAdcClkSel);
    subWriteReg(0x4000f044,23,22, g_rfPhyClkSel);
    subWriteReg(0x4000f044, 6,   5, 0x03);    // trim dll/dbl ldo vout

    if(     (g_rxAdcClkSel == RX_ADC_CLK_SEL_32M_DBL)
            ||  (g_rxAdcClkSel == RX_ADC_CLK_SEL_32M_DBL_B)
            ||  (g_rfPhyClkSel == RF_PHY_CLK_SEL_32M_DBL)
            ||  (g_rfPhyClkSel == RF_PHY_CLK_SEL_32M_DBL_B) )
    {
        // enable dbl for rf
        subWriteReg(0x4000f044, 8, 8, 0x01);  // DBL EN,DLL EN,DLL LDO EN
    }

    if(     (g_rxAdcClkSel == RX_ADC_CLK_SEL_32M_DLL)
            ||  (g_rfPhyClkSel == RF_PHY_CLK_SEL_32M_DLL)
      )
    {
        // enable dll for rf
        subWriteReg(0x4000f044, 7, 7, 0x01);  // DLL ensable
    }

    subWriteReg(0x4000f044, 19, 18, 0x03);    // Rx adc clk en, rf phy clk en
    #if 0

    //Reserved?????
    //20190111 ZQ
    // for 48M case should set dbl clk polarity
    //config sel_rxadc_dbl_clk_32M_polarity;
    if((g_system_clk==SYS_CLK_DLL_48M) )
    {
        subWriteReg(0x4000f044,26,25, 0x03);
    }
    else
    {
        subWriteReg(0x4000f044,26,25, 0x00);
    }

    #endif

    if(g_rfPhyClkSel==RF_PHY_CLK_SEL_16M_XTAL && g_system_clk == SYS_CLK_DLL_48M)
        subWriteReg( 0x4003008c,23,23,0x01);
    else
        subWriteReg( 0x4003008c,23,23,0x00);

    //-------------------------------------------------------------------
    //               PLL
    PHY_REG_WT(0x400300cc,0x20000bc0);  // i_pll_ctrl0 :
    //PHY_REG_WT(0x400300cc,0x20000fc0);  // i_pll_ctrl0 :
    //-------------------------------------------------------------------
    //TX PLL BW
    PHY_REG_WT(0x400300d0,0x00000180);  // i_pll_ctrl1
    PHY_REG_WT(0x400300d4,0x076a3e7a);  // i_pll_ctrl2 pll lpf, boost vco current[7:4]
    PHY_REG_WT(0x400300d8,0x04890000);  // i_pll_ctrl3 vco/tp varactor
    //-------------------------------------------------------------------
    //RX PLL BW active when rx_en
    PHY_REG_WT(0x40030104,0x00000180);  // i_pll_ctrl5
    PHY_REG_WT(0x40030108,0x076a3e7a);  // i_pll_ctrl6 pll lpf, boost vco current[7:4]
    PHY_REG_WT(0x4003010c,0x04898000);  // i_pll_ctrl7 vco/tp varactor
    //-------------------------------------------------------------------
    //VCO Coarse Tuning Setting
    PHY_REG_WT(0x40030080,0x000024cc);  //[11:10] vco coarse tune slot time
    //[9:7] delay from pll reset ends to start vco coarse tune

    //-------------------------------------------------------------------
    //PLL config for rfPhyClk=16M
    if(g_rfPhyClkSel==RF_PHY_CLK_SEL_16M_XTAL)
    {
        subWriteReg(0x40030080, 0, 0, 1);//indicate 16M reference clk to rfpll
        //PHY_REG_WT(0x400300d0,0x00000140);  // i_pll_ctrl1 //double cp current to compensate for clock;
        //PHY_REG_WT(0x40030104,0x00000140);  // i_pll_ctrl5 //double cp current to compensate for clock;
    }

    //-------------------------------------------------------------------
    //               Tx PA
    //PHY_REG_WT(0x400300b8,0x0000f825);  // pa ramp reg, txPower for 0dBm
    PHY_REG_WT(0x400300b8,0x0825|(((g_rfPhyTxPower)&0x1f)<<12));  // pa ramp reg, txPower for g_rfPhyTxPower
    //-------------------------------------------------------------------
    //               TP Modulation
    //PHY_REG_WT(0x40030090,0x00020000);  // reg dc 40000 for 2M
    //PHY_REG_WT(0x40030094,0x00001025);  // tp_cal val
    //PHY_REG_WT(0x400300b0,0x01000003);  // dac dly
    //-------------------------------------------------------------------
    //               Rx FrontEnd
    //PHY_REG_WT(0x400300dc,0x01be7f2f);      //
    PHY_REG_WT(0x400300dc,0x01a6fc2f);      // boost TIA current
    //-------------------------------------------------------------------
    //PA override
    //subWriteReg(0x400300a0,0,0,1);       //pa override
    //subWriteReg(0x400300a4,0,0,1);       //pa on
    //subWriteReg(0x400300a4,8,8,0);       //pa set b
}




/**************************************************************************************
    @fn          rf_phy_bb_cfg

    @brief       This function process for rf phy baseband tx and rx config.

    input parameters

    @param       pktMod:0 for Zigbee, 1 for BLE1M, 2 for BLE2M, 3or4 for BLELR.

    output parameters

    @param       None.

    @return      None.
*/
void rf_phy_bb_cfg(uint8_t pktFmt)
{
    //BW Sel and Gauss sel
    if(pktFmt==0 || pktFmt==2)
    {
        PHY_REG_WT( 0x400300e0,0x00000080);   // set pga bw use small bw for zigbee and BLE2M
        subWriteReg( 0x400300d8,20,18,0x02);   // tpm dac var

        if(g_rfPhyClkSel == RF_PHY_CLK_SEL_16M_XTAL)
            PHY_REG_WT( 0x40030090,0x00080000);   // set reg_dc
        else
            PHY_REG_WT( 0x40030090,0x00040000);   // set reg_dc

        //PHY_REG_WT( 0x40030094,0x00001048);   // tp_cal val
    }
    else if(pktFmt<5)
    {
        PHY_REG_WT( 0x400300e0,0x00000100);   // set pga bw
        subWriteReg( 0x400300d8,20,18,0x01);   // tpm dac var

        //subWriteReg( 0x400300d8,20,18,0x02);   // tpm dac var
        if(g_rfPhyClkSel == RF_PHY_CLK_SEL_16M_XTAL)
            PHY_REG_WT( 0x40030090,0x00040000);   // set reg_dc
        else
            PHY_REG_WT( 0x40030090,0x00020000);   // set reg_dc

        //PHY_REG_WT( 0x40030094,0x00001025);   // tp_cal val
    }
    else
    {
        PHY_REG_WT( 0x400300e0,0x00000100);   // set pga bw
        subWriteReg( 0x400300d8,20,18,0x01);   // tpm dac var

        if(g_rfPhyClkSel == RF_PHY_CLK_SEL_16M_XTAL)
            PHY_REG_WT( 0x40030090,0x00040000);   // set reg_dc
        else
            PHY_REG_WT( 0x40030090,0x00020000);   // set reg_dc
    }

    PHY_REG_WT( 0x400300b0,0x01000003);                 // dac dly
    PHY_REG_WT( 0x40030094,0x00001000+g_rfPhyTpCal0);   // tp_cal val

    //pktFmt Setting and syncThd
    if(pktFmt==0)
    {
        PHY_REG_WT( 0x40030000,0x78068000);
        PHY_REG_WT( 0x40030048,0x00000000);   //clr crc and wtSeed
        PHY_REG_WT( 0x40030040,0x000b2800);   // disable gauss
        PHY_REG_WT( 0x4003004c,0x3675ee07);
        ll_hw_set_crc_fmt (LL_HW_CRC_ZB_FMT, LL_HW_CRC_ZB_FMT);
    }
    else if(pktFmt==1)
    {
        PHY_REG_WT( 0x40030000,0x3d068001);
        PHY_REG_WT( 0x40030048,0x37555555);
        PHY_REG_WT( 0x40030040,0x00032800);   // enable gauss
        PHY_REG_WT( 0x4003004c,0x8e89bed6);
        ll_hw_set_crc_fmt (LL_HW_CRC_BLE_FMT, LL_HW_CRC_BLE_FMT);
    }
    else if(pktFmt==2)
    {
        PHY_REG_WT( 0x40030000,0x3d068002);
        PHY_REG_WT( 0x40030048,0x37555555);
        PHY_REG_WT( 0x40030040,0x00032800);   // enable gauss
        PHY_REG_WT( 0x4003004c,0x8e89bed6);
        ll_hw_set_crc_fmt (LL_HW_CRC_BLE_FMT, LL_HW_CRC_BLE_FMT);
    }
    else if(pktFmt==3 || pktFmt==4)
    {
        //pktFmt=3 or pktFmt=4
        PHY_REG_WT( 0x40030000,0x98068000|pktFmt);  //for tx set differnt phy
        PHY_REG_WT( 0x40030004,0x50985a54);         //for RSSI >-90 set higher syncThd=0x98
        PHY_REG_WT( 0x40030040,0x00032800);   // enable gauss
        PHY_REG_WT( 0x40030048,0x37555555);
        PHY_REG_WT( 0x4003004c,0x8e89bed6);
        ll_hw_set_crc_fmt (LL_HW_CRC_BLE_FMT, LL_HW_CRC_BLE_FMT);
    }
    else
    {
        PHY_REG_WT( 0x40030000,0x42068000|pktFmt);
        PHY_REG_WT( 0x40030048,0x00555555);
        PHY_REG_WT( 0x40030040,0x000b2800);   // enable gauss
        PHY_REG_WT( 0x4003004c,0x8e89bed6);
        ll_hw_set_crc_fmt (LL_HW_CRC_BLE_FMT, LL_HW_CRC_BLE_FMT);
    }

    //Agc Control Setting
    if(pktFmt==0)
    {
        PHY_REG_WT( 0x40030050,0x22086680);
    }
    else if(pktFmt==2)
    {
        PHY_REG_WT( 0x40030050,0x22084580);
    }
    else
    {
        PHY_REG_WT( 0x40030050,0x22085580);
    }

    // add by ZQ 20181030 for DLE feature
    // set to 255
    // need considering the ADV PDU in BLE 5.0
    subWriteReg(0x4003000c,7,0, 0xff);
    //AGC TAB with LNA two gain step,no bypass lna mode
    //20200721 for tsop 6252
    PHY_REG_WT(0x40030054, 0x545c9ca4);
    PHY_REG_WT(0x40030058, 0x4243444c);
    PHY_REG_WT(0x4003005c, 0x464c5241);
    PHY_REG_WT(0x40030060, 0x2e343a40);
    PHY_REG_WT(0x40030064, 0x557f0028);
    PHY_REG_WT(0x40030068, 0x3d43494f);
    PHY_REG_WT(0x4003006c, 0x4c2b3137);
    PHY_REG_WT(0x40030070, 0x343a4046);
    PHY_REG_WT(0x40030074, 0x1c22282e);
    #if 0
    PHY_REG_WT( 0x40030054,0x545c9ca4 );
    PHY_REG_WT( 0x40030058,0x03040c4c );
    PHY_REG_WT( 0x4003005c,0x464c5202 );
    PHY_REG_WT( 0x40030060,0x262e3a40 );
    PHY_REG_WT( 0x40030064,0x557f0020 );
    PHY_REG_WT( 0x40030068,0x3b43494f );
    PHY_REG_WT( 0x4003006c,0x4c23292f );
    PHY_REG_WT( 0x40030070,0x343a4046 );
    PHY_REG_WT( 0x40030074,0x191f252b );
    #endif
    #if(RF_PHY_EXT_PREAMBLE_US)

    //ext preamble for BLE 1M/2M, nByte
    if(pktFmt==PKT_FMT_BLE1M)
    {
        subWriteReg(0x40030040, 7, 5, (RF_PHY_EXT_PREAMBLE_US>>3) ); // 1byte -> 8us
    }
    else if(pktFmt == PKT_FMT_BLE2M)
    {
        subWriteReg(0x40030040, 7, 5, (RF_PHY_EXT_PREAMBLE_US>>2) );//2 byte -> 8us
    }

    #endif
}


void rf_phy_change_cfg0(uint8_t pktFmt)
{
    //BW Sel and Gauss sel
    if(pktFmt==PKT_FMT_BLE2M)
    {
        PHY_REG_WT( 0x400300e0,0x00000080);   // set pga bw use small bw for zigbee and BLE2M
        subWriteReg( 0x400300d8,20,18,0x02);   // tpm dac var

        if(g_rfPhyClkSel == RF_PHY_CLK_SEL_16M_XTAL)
            PHY_REG_WT( 0x40030090,0x00080000);   // set reg_dc
        else
            PHY_REG_WT( 0x40030090,0x00040000);   // set reg_dc

        //PHY_REG_WT( 0x40030094,0x00001048);   // tp_cal val
    }
    else
    {
        PHY_REG_WT( 0x400300e0,0x00000080);   // set pga bw
        subWriteReg( 0x400300d8,20,18,0x01);   // tpm dac var

        if(g_rfPhyClkSel == RF_PHY_CLK_SEL_16M_XTAL)
            PHY_REG_WT( 0x40030090,0x00040000);   // set reg_dc
        else
            PHY_REG_WT( 0x40030090,0x00020000);   // set reg_dc

        //PHY_REG_WT( 0x40030094,0x00001025);   // tp_cal val
    }

    //pktFmt Setting and syncThd
    if(pktFmt==PKT_FMT_BLE1M)
    {
        PHY_REG_WT( 0x40030000,0x3d068001);
    }
    else if(pktFmt==PKT_FMT_BLE2M)
    {
        PHY_REG_WT( 0x40030000,0x3d068002);
    }
    else
    {
        //pktFmt=3 or pktFmt=4
        PHY_REG_WT( 0x40030000,0x98068000|pktFmt);  //for tx set differnt phy
    }

    //Agc Control Setting
    if(pktFmt==PKT_FMT_BLE1M)
    {
        PHY_REG_WT( 0x40030050,0x22086680);
    }
    else if(pktFmt==PKT_FMT_BLE2M)
    {
        PHY_REG_WT( 0x40030050,0x22084580);
    }
    else
    {
        PHY_REG_WT( 0x40030050,0x22085580);
    }
}
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
uint8_t rf_tp_cal(uint8_t rfChn, uint8_t fDev)
{
    PHY_REG_WT( 0x40030040,0x00030010);         // enable test mode not to generate txdone

    if(fDev==1)
    {
        subWriteReg( 0x400300d8,20,18,0x02);   // tpm dac var
        PHY_REG_WT( 0x4003008c,0x0053407f);
    }
    else
    {
        subWriteReg( 0x400300d8,20,18,0x01);   // tpm dac var
        PHY_REG_WT( 0x4003008c,0x0073407f);
    }

    if(g_rfPhyClkSel==RF_PHY_CLK_SEL_16M_XTAL && g_system_clk == SYS_CLK_DLL_48M)
        subWriteReg( 0x4003008c,23,23,0x01);
    else
        subWriteReg( 0x4003008c,23,23,0x00);

    PHY_REG_WT( 0x400300b4,0xff&rfChn);     // set rfFreq=2400+rfChn
    PHY_REG_WT( 0x400300a0,0x0000000e);     // set pll_auto override
    //-------------------------------------------------------------------------
    // Cal Trig
    //
    PHY_REG_WT( 0x400300a4,0x00000000);     // clr tx_auto
    PHY_REG_WT( 0x400300a4,0x00000114);     // set tx_auto
    //-------------------------------------------------------------------------
    // Wait to Read Reslut
    // When HCLK 16M --> 10000*3/16 around 2ms
    volatile int timeOut = 10000;

    switch (g_system_clk)
    {
    case SYS_CLK_XTAL_16M:
        timeOut=timeOut;
        break;

    case SYS_CLK_RC_32M:
    case SYS_CLK_DLL_32M:
        timeOut=timeOut*2;
        break;
        #if (PHY_MCU_TYPE == MCU_BUMBEE_M0 || PHY_MCU_TYPE == MCU_BUMBEE_CK802)

    case SYS_CLK_4M:
        break;

    case SYS_CLK_8M:
        break;
        #elif ((PHY_MCU_TYPE == MCU_PRIME_A1) ||(PHY_MCU_TYPE == MCU_PRIME_A2))

    case SYS_CLK_DBL_32M:
        timeOut=timeOut*2;
        break;
        #endif

    case SYS_CLK_DLL_48M:
        timeOut=timeOut*3;
        break;

    case SYS_CLK_DLL_64M:
        timeOut=timeOut*4;
        break;

    default:
        timeOut=timeOut;
        break;
    }

    while(timeOut--) {};

    uint8_t kCal = (0xff0000 & PHY_REG_RD(0x400300f4))>>16;

    PHY_REG_WT( 0x400300a4,0x00000000);     // clr tx_auto

    PHY_REG_WT( 0x400300a0,0x00000000);     // clr pll_auto override

    PHY_REG_WT( 0x4003008c,0x00104040);     // clr tp_cal_en

    PHY_REG_WT( 0x400300a4,0x00000140);     // clr tx_auto

    PHY_REG_WT( 0x40030040,0x00032800);

    #if(RF_PHY_EXT_PREAMBLE_US)

    //ext preamble for BLE 1M/2M, nByte
    if(g_rfPhyPktFmt==PKT_FMT_BLE1M)
    {
        subWriteReg(0x40030040, 7, 5, (RF_PHY_EXT_PREAMBLE_US>>3) ); // 1byte -> 8us
    }
    else if(g_rfPhyPktFmt == PKT_FMT_BLE2M)
    {
        subWriteReg(0x40030040, 7, 5, (RF_PHY_EXT_PREAMBLE_US>>2) );//2 byte -> 8us
    }

    #endif

    if(g_rfPhyClkSel==RF_PHY_CLK_SEL_16M_XTAL && g_system_clk == SYS_CLK_DLL_48M)
        subWriteReg( 0x4003008c,23,23,0x01);
    else
        subWriteReg( 0x4003008c,23,23,0x00);

    return kCal;
}




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
void rf_rxDcoc_cfg(uint8_t rfChn, uint8_t bwSet, volatile uint32* dcCal)
{
    //--------------------------------------------------------------
    //rf_ana_cfg should be called before doing dcoc calibration
    //--------------------------------------------------------------
    //  restore the rxTimeOut setting, and set rxto to zero, not to generate rx_done
    //
    //
    int rxTimeOut1st = PHY_REG_RD(0x40031024);
    int rxTimeOut    = PHY_REG_RD(0x40031028);
    PHY_REG_WT(0x40031024,0x00000000);
    PHY_REG_WT(0x40031028,0x00000000);
    //--------------------------------------------------------------
    //rx close
    //
    PHY_REG_WT( 0x400300b4,0xff&rfChn);            // set rfFreq=2400+rfChn
    PHY_REG_WT( 0x400300a0,0x0000000e);            // set pll_auto override
    PHY_REG_WT( 0x400300a4,0x00000100);            // clr trx_auto
    PHY_REG_WT( 0x400300a8,0x00000040);            // set lo override
    PHY_REG_WT( 0x400300ac,0x00000050);            // set rx lo and pll buff
    PHY_REG_WT( 0x400300a4,0x0000012a);            // rx close
    //--------------------------------------------------------------
    //set filter bw and rx gain,dcoc cal control
    //
    uint16 fltPhy = 0;

    if(bwSet==1)
    {
        PHY_REG_WT( 0x400300e0,0x00000100);         // set pga bw(0100:1M,1100:2M)
        fltPhy  =   0x034d;
    }
    else
    {
        PHY_REG_WT( 0x400300e0,0x00000080);         // set pga bw(0100:1M,1100:2M)
        fltPhy  =   0x02ca;
    }

    PHY_REG_WT( 0x400300c8,0x000001a4);             // set to rx max gain
    PHY_REG_WT( 0x400300c4,0x00002020);             // dcoc dac controled by mix_sig_top for cal
    PHY_REG_WT( 0x40030050,0x200c5680);             // enlarge dcoc cal average time
    // enlarge pga settle time
    //--------------------------------------------------------------
    //wait for rf settle
    //
    volatile int cnt=1000;

    while(cnt--) {};

    PHY_REG_WT( 0x40030078,(fltPhy<<22)|0x216564);  // clr dcoc_en[0] , set to phase search mode[1]

    PHY_REG_WT( 0x40030078,(fltPhy<<22)|0x216565);  // set dcoc_en[0], dcoc start

    //--------------------------------------------------------------
    //wait to read the cal result
    //
    cnt=10000;

    switch (g_system_clk)
    {
    case SYS_CLK_XTAL_16M:
        cnt=cnt;
        break;

    case SYS_CLK_RC_32M:
    case SYS_CLK_DLL_32M:
        cnt=cnt*2;
        break;
        #if (PHY_MCU_TYPE == MCU_BUMBEE_M0 || PHY_MCU_TYPE == MCU_BUMBEE_CK802)

    case SYS_CLK_4M:
        break;

    case SYS_CLK_8M:
        break;
        #elif ((PHY_MCU_TYPE == MCU_PRIME_A1) ||(PHY_MCU_TYPE == MCU_PRIME_A2))

    case SYS_CLK_DBL_32M:
        cnt=cnt*2;
        break;
        #endif

    case SYS_CLK_DLL_48M:
        cnt=cnt*3;
        break;

    case SYS_CLK_DLL_64M:
        cnt=cnt*4;
        break;

    default:
        cnt=cnt;
        break;
    }

    *dcCal = 0x20200000;        // set the dc cal to default val

    while(cnt--)
    {
        if(3==(0x03 & (PHY_REG_RD(0x400300ec)>>30)))
        {
            *dcCal = PHY_REG_RD(0x400300ec)&0x3f3fffff;                 // get the dc cal result
            PHY_REG_WT( 0x400300c4,0x00010000 | ((*dcCal)>>16));         // set to dcoc dac code
            break;
        }
    }

    //--------------------------------------------------------------
    //end of cfg
    //
    PHY_REG_WT( 0x40030078,(fltPhy<<22)|0x216564);  // clr dcoc_en[0] , set to phase search mode[1]
    PHY_REG_WT( 0x400300a8,0x00000000);             // clr lo override
    PHY_REG_WT( 0x400300ac,0x00000000);             // clr rx lo and pll buff
    PHY_REG_WT( 0x400300c8,0x00000000);             // set to rx max gain
    PHY_REG_WT( 0x400300a4,0x00000100);             // clr tx_auto
    PHY_REG_WT( 0x400300a0,0x00000000);             // clr pll_auto override
    PHY_REG_WT( 0x400300a4,0x00000140);             // clr tx_auto
    PHY_REG_WT(0x40031024,rxTimeOut1st);
    PHY_REG_WT(0x40031028,rxTimeOut);
}

#if(RF_PHY_DTM_CTRL_MOD == RF_PHY_DTM_CTRL_UART)
/**************************************************************************************
    @fn          rf_phy_dtm_uart_irq

    @brief       This function process for rf phy direct test uart irq.

    input parameters

    @param       none

    output parameters

    @param       none

    @return      none
*/
void DTM_UART_IRQHandler(void)
{
    uint8_t IRQ_ID= (AP_UART0->IIR & 0x0f);

    switch (IRQ_ID)
    {
    case RDA_IRQ:
    case TIMEOUT_IRQ:
        while(AP_UART0 ->LSR & 0x1)
        {
            urx_buf[uart_rx_wIdx]=(AP_UART0->RBR & 0xff);
            DTM_ADD_IDX(uart_rx_wIdx, MAX_UART_BUF_ID);
        }

        break;

    case BUSY_IRQ:
        AP_UART0 -> USR;
        break;
    }
}
// static uint32_t dtm_read_current_time(void)
// {
//   // return ((4000000-get_timer3_count())/4+2);
//      return (RF_PHY_TIME_BASE - ((AP_TIM3->CurrentCount) >> 2) ) ;
// }

#endif

#if(RF_PHY_DTM_CTRL_MOD == RF_PHY_DTM_CTRL_UART)
/**************************************************************************************
    @fn          rf_phy_direct_test

    @brief       This function process for rf phy direct test.

    input parameters

    @param       none

    output parameters

    @param       none

    @return      none
*/
void rf_phy_direct_test(void)
{
    int dtmState = 0;
    uint32_t deltTick = 0;
    uint32_t currTick = 0;
    //enable received data available interrupt
    DTM_LOG_INIT();
    DTM_LOG("\n===RF_PHY_DTM V1.1.2===\n");
    //clr UART IRQ, switch to ROM_UART_IRQ
    JUMP_FUNCTION(UART0_IRQ_HANDLER) = (uint32_t)&DTM_UART_IRQHandler;

    if(RF_PHY_DTM_BB_SUPPORT_MOD&RF_PHY_DTM_BB_SUPPORT_BLE1M)
        DTM_LOG("=== SUPPORT BLE 1M  ===\n");

    if(RF_PHY_DTM_BB_SUPPORT_MOD&RF_PHY_DTM_BB_SUPPORT_BLE2M)
        DTM_LOG("=== SUPPORT BLE 2M  ===\n");

    if(RF_PHY_DTM_BB_SUPPORT_MOD&RF_PHY_DTM_BB_SUPPORT_BLR500K)
        DTM_LOG("=== SUPPORT BLR 500K===\n");

    if(RF_PHY_DTM_BB_SUPPORT_MOD&RF_PHY_DTM_BB_SUPPORT_BLR125K)
        DTM_LOG("=== SUPPORT BLR 125K===\n");

    if(RF_PHY_DTM_BB_SUPPORT_MOD&RF_PHY_DTM_BB_SUPPORT_ZIGBEE)
        DTM_LOG("=== SUPPORT ZIGBEE  ===\n");

    //*(volatile int *) 0xe000e100 |= 0x800;
    *(volatile int*) 0xe000e100 = 0x800; //only use uart irq
    *(volatile unsigned int*) 0x40004004 |= 0x01; //ENABLE_ERBFI;
    //set timer3 free run
    //AP_TIM3->ControlReg = 0x05;
    //24bit count down mode no IRQ
    set_timer(AP_TIM3,RF_PHY_TIME_BASE);
    NVIC_DisableIRQ(TIM3_IRQn);
    //clear widx
    CLR_UART_WIDX;

    while(1)
    {
        if(      dtmState == RF_PHY_DTM_IDL)
        {
            if(GET_UART_WIDX>=2)
            {
                g_rfPhyDtmCmd[0]=urx_buf[0];
                g_rfPhyDtmCmd[1]=urx_buf[1];
                dtmState = RF_PHY_DTM_CMD;
                CLR_UART_WIDX;
            }

            //=================== cmd parsing  =====================
        }
        else if(dtmState == RF_PHY_DTM_CMD)
        {
            rf_phy_dtm_cmd_parse();
            dtmState = RF_PHY_DTM_EVT;
            //=================== send event   =====================
        }
        else if(dtmState == RF_PHY_DTM_EVT)
        {
            rf_phy_dtm_evt_send(g_dtmModeType);
            dtmState = RF_PHY_DTM_TEST;
            //=================== TEST Start    =====================
        }
        else if(dtmState == RF_PHY_DTM_TEST)
        {
            rf_phy_dtm_trigged();
            g_dtmPktCount = 0;
            uint8_t rssi;
            uint8_t carrSens;
            uint16_t foff;
            uint8_t zgb_pkt_flg=0;

            //when new cmd arrived, state change
            while(GET_UART_WIDX<2)
            {
                //TX BURST re-trigger
                if(g_dtmModeType == RF_PHY_DTM_MODE_TX_BURST)
                {
                    //============================================================
                    //20180424-ZQ
                    //option 1
                    ll_hw_set_trx_settle    (100, 8, 80);       //TxBB,RxAFE,PLL
                    //============================================================
                    currTick = read_current_fine_time();
                    deltTick = RF_PHY_TIME_DELTA(currTick,g_dtmTick);

                    if(     (ll_hw_get_irq_status() & LIRQ_MD)
                            &&  g_rfPhyPktFmt == PKT_FMT_ZIGBEE
                            &&  zgb_pkt_flg==0)
                    {
                        //ll_hw_rst_tfifo();
                        //rf_phy_dtm_zigbee_pkt_gen();
                        ll_hw_clr_irq();
                        zgb_pkt_flg=1;
                        PHY_REG_WT( 0x4003105c,0x00000000);     // clr rd_ini and rd_last
                    }

                    if(deltTick>=g_dtmPktIntv)
                    {
                        if(!(g_rfPhyPktFmt==PKT_FMT_ZIGBEE))
                        {
                            ll_hw_rst_tfifo();
                            ll_hw_trigger();
//                            if(g_rfPhyPktFmt==PKT_FMT_BLE2M)
//                            {
//                                //set tx_auto_ctrl0 first
//                                PHY_REG_WT( 0x400300a4,0x00000140);     // clr tx_auto
//                                PHY_REG_WT( 0x400300a4,0x00000150);     // clr tx_auto
//                                //wait a while then set tx_bb_en
//                                int delay=100;while(delay--){};
//                                PHY_REG_WT( 0x400300a4,0x00000154);     // clr tx_auto
//                            }
//                            else
//                            {
//                                ll_hw_trigger();
//                                //PHY_REG_WT( 0x400300a0,0x0000000e);     // clr pll_auto override
//                                //PHY_REG_WT( 0x400300a4,0x00000140);     // clr tx_auto
//                                //PHY_REG_WT( 0x400300a4,0x00000154);     // clr tx_auto
//
//                            }
                        }
                        else
                        {
                            ll_hw_trigger();
                            //PHY_REG_WT( 0x400300a4,0x00000140);     // clr tx_auto
                            //PHY_REG_WT( 0x400300a4,0x00000154);     // clr tx_auto
                            zgb_pkt_flg=0;
                        }

                        #if(RF_PHY_CT_MONITER)
                        WaitUs(350);
                        int32_t ct_word= (PHY_REG_RD(0x400300f4)&0x3ff);

                        if(ct_word>0)
                        {
                            if(g_rfPhy_ct_moniter_word_cnt==0)
                                g_rfPhy_ct_moniter_word_target = ct_word;

                            int32_t idx= (ct_word)-g_rfPhy_ct_moniter_word_target+(CT_MONT_BUFF_SIZE>>1);
                            g_rfPhy_ct_moniter_word_cnt++;

                            if(idx<0)
                            {
                                LOG("e %d %d\n",ct_word,PHY_REG_RD(0x400300f8)&0xfffff);
                                g_rfPhy_ct_moniter_word_arry[0]++;
                            }
                            else if(idx>CT_MONT_BUFF_SIZE-1)
                            {
                                g_rfPhy_ct_moniter_word_arry[CT_MONT_BUFF_SIZE-1]++;
                            }
                            else
                            {
                                g_rfPhy_ct_moniter_word_arry[idx]++;
                            }

                            if(g_rfPhy_ct_moniter_word_cnt>RF_PHY_CT_MONITER)
                            {
                                for (int i=0; i<CT_MONT_BUFF_SIZE; i++)
                                {
                                    if(g_rfPhy_ct_moniter_word_arry[i]>0)
                                        LOG("%d %d\n",g_rfPhy_ct_moniter_word_target-(CT_MONT_BUFF_SIZE>>1)+i,g_rfPhy_ct_moniter_word_arry[i]);

                                    g_rfPhy_ct_moniter_word_arry[i]=0;
                                }

                                g_rfPhy_ct_moniter_word_cnt=0;
                            }
                        }

                        #endif
                        //g_dtmTick = read_current_time();
                        g_dtmTick = g_dtmTick+g_dtmPktIntv;

                        if(g_dtmTick>RF_PHY_TIME_BASE)
                            g_dtmTick = g_dtmTick-RF_PHY_TIME_BASE;
                    }
                }
                else if(   g_dtmModeType == RF_PHY_DTM_MODE_RX_PER
                           ||  g_dtmModeType == RF_PHY_DTM_MODE_GET_PER_AUTO)
                {
                    //fix max gain can imporve 2480 sensitivity
                    if(g_dtmManualConfig&RF_PHY_DTM_MANUL_MAX_GAIN)
                    {
                        subWriteReg(0x40030050, 4,0,0x10);      //fix max gain
                    }

                    subWriteReg(0x4000f018,14,9,0x3f);//rc32M clk slow

                    if(ll_hw_get_irq_status()&LIRQ_MD)
                    {
                        if(ll_hw_get_irq_status()&LIRQ_COK)
                        {
                            g_dtmPktCount++;
                            rf_phy_get_pktFoot(&rssi,&foff,&carrSens);
                        }
                        else if(ll_hw_get_irq_status()&LIRQ_CERR)
                        {
                            g_dtmRxCrcNum++;
                        }
                        else if(ll_hw_get_irq_status()&LIRQ_RTO)
                        {
                            g_dtmRxTONum++;
                        }
                        else
                        {
                            //wrap the pktCount
                            g_dtmPktCount= (g_dtmPktCount==65535) ? 0 : g_dtmPktCount;
                        }

                        if( g_dtmModeType == RF_PHY_DTM_MODE_GET_PER_AUTO)
                        {
                            currTick = read_current_fine_time();
                            deltTick = RF_PHY_TIME_DELTA(currTick,g_dtmTick);

                            if(deltTick>g_dtmPerAutoIntv)
                            {
                                rf_phy_dtm_evt_send(RF_PHY_DTM_MODE_TEST_END);//send pktCount
                                WaitUs(500);
                                rf_phy_dtm_evt_send(RF_PHY_DTM_MODE_GET_FOFF);//send FOFF
                                WaitUs(500);
                                rf_phy_dtm_evt_send(RF_PHY_DTM_MODE_GET_RSSI);//send RSSI
                                WaitUs(500);
                                rf_phy_dtm_evt_send(RF_PHY_DTM_MODE_GET_CARR_SENS);//send CARR_SENS
                                WaitUs(500);
                                g_dtmPktCount = 0;
                                g_dtmRxCrcNum = 0;
                                g_dtmRxTONum = 0;
                                g_dtmTick = read_current_fine_time();
                            }
                        }//EndOfIF PER_AUTO

                        ll_hw_clr_irq();
                        ll_hw_trigger();

                        if(g_dtmPktCount>0)
                        {
                            g_dtmRssi       = (g_dtmPktCount==1) ? rssi     :((g_dtmRssi+rssi)          >>1);
                            g_dtmFoff       = (g_dtmPktCount==1) ? foff     :((g_dtmFoff+foff)          >>1);
                            g_dtmCarrSens   = (g_dtmPktCount==1) ? carrSens :((g_dtmCarrSens+carrSens)  >>1);
                        }
                    }
                }//end of RX_PER
            }//end of GET_UART_WIDX

            dtmState = RF_PHY_DTM_IDL;
        }
    }//end of while(1)
}


#endif

#if(RF_PHY_DTM_CTRL_MOD == RF_PHY_DTM_CTRL_UART)
/**************************************************************************************
    @fn          rf_phy_dtm_cmd_parse

    @brief       This function process for rf phy direct test,cmd parse

    input parameters

    @param       none

    output parameters

    @param       none

    @return      none
*/
void rf_phy_dtm_cmd_parse(void)
{
    g_dtmCmd        = ((g_rfPhyDtmCmd[0] & 0xc0)>>6);      // bit 15 14

    //test setup and test end
    if(g_dtmCmd == 0 ||  g_dtmCmd==3 )
    {
        g_dtmCtrl       = ((g_rfPhyDtmCmd[0] & 0x3f)   );      // bit 13  8
        g_dtmPara       = ((g_rfPhyDtmCmd[1] & 0xfc)>>2);      // bit  7  2

        //TEST SETUP
        if(g_dtmCmd==0)
        {
            if(g_dtmCtrl==0 && g_dtmPara==0)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_RESET;
            }
            else if(g_dtmCtrl==1)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_SET_LENGTH_UP2BIT;
                g_dtmExtLen   = (g_dtmPara&0x3);
            }
            else if(g_dtmCtrl==2 && g_dtmPara ==0x01)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_SET_PHY_1M;
                g_rfPhyPktFmt = PKT_FMT_BLE1M;
            }
            else if(g_dtmCtrl==2 && g_dtmPara ==0x02)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_SET_PHY_2M;
                g_rfPhyPktFmt = (RF_PHY_DTM_BB_SUPPORT_BLE2M & RF_PHY_DTM_BB_SUPPORT_MOD)
                                ?PKT_FMT_BLE2M:PKT_FMT_BLE1M;
            }
            else if(g_dtmCtrl==2 && g_dtmPara ==0x03)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_SET_PHY_125K;
                g_rfPhyPktFmt = (RF_PHY_DTM_BB_SUPPORT_BLR125K & RF_PHY_DTM_BB_SUPPORT_MOD)
                                ?PKT_FMT_BLR125K:PKT_FMT_BLE1M;
            }
            else if(g_dtmCtrl==2 && g_dtmPara ==0x04)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_SET_PHY_500K;
                g_rfPhyPktFmt = (RF_PHY_DTM_BB_SUPPORT_BLR500K & RF_PHY_DTM_BB_SUPPORT_MOD)
                                ?PKT_FMT_BLR500K:PKT_FMT_BLE1M;
            }
            else if(g_dtmCtrl==2 && g_dtmPara ==0x20)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_SET_PHY_ZB;
                g_rfPhyPktFmt = (RF_PHY_DTM_BB_SUPPORT_ZIGBEE & RF_PHY_DTM_BB_SUPPORT_MOD)
                                ?PKT_FMT_ZIGBEE:PKT_FMT_BLE1M;
            }
            else if(g_dtmCtrl==3 && g_dtmPara ==0x00)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_ASSUME_TX_MOD_INDX_STANDARD;
            }
            else if(g_dtmCtrl==3 && g_dtmPara ==0x01)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_ASSUME_TX_MOD_INDX_STABLE;
            }
            else if(g_dtmCtrl==4 && g_dtmPara ==0x00)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_READ_SUPPORTED_TEST_CASE;
            }
            else if(g_dtmCtrl==5 && g_dtmPara ==0x00)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_READ_MAX_TX_OCTETS;
            }
            else if(g_dtmCtrl==5 && g_dtmPara ==0x01)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_READ_MAX_TX_TIME;
            }
            else if(g_dtmCtrl==5 && g_dtmPara ==0x02)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_READ_MAX_RX_OCTETS;
            }
            else if(g_dtmCtrl==5 && g_dtmPara ==0x03)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_READ_MAX_RX_TIME;
            }
            else if(g_dtmCtrl==0x34)
            {
                uint8_t ret=g_dtmPara & 0x01;

                for(uint8_t pin=0; pin<23; pin++)
                {
                    if((pin == P2) || (pin == P3))
                    {
                        hal_gpio_pin2pin3_control((gpio_pin_e)pin,ret);
                        gpio_write((gpio_pin_e)pin,ret);
                    }
                    else if((pin == P9) || (pin == P10) || (pin == P16) || (pin == P17))
                    {
                        continue;
                    }
                    else
                        gpio_write((gpio_pin_e)pin,ret);
                }
            }
            else if(g_dtmCtrl==0x35)
            {
                uint8_t ret=g_dtmPara>>5;
                uint8_t pin=g_dtmPara & 0x1f;

                if((pin == P2) || (pin == P3))
                    hal_gpio_pin2pin3_control((gpio_pin_e)pin,ret);

                gpio_write((gpio_pin_e)pin,ret);
                // PHYPLUSE DEFINED:  RF_PHY_DTM_MODE_SET_ACCCODE_0
            }
            else if(g_dtmCtrl==0x36)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_ACCCODE_0;
                g_dtmAccessCode   = (g_dtmAccessCode&(0xffffff00))|g_rfPhyDtmCmd[1];
                // PHYPLUSE DEFINED:  RF_PHY_DTM_MODE_SET_ACCCODE_1
            }
            else if(g_dtmCtrl==0x37)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_ACCCODE_1;
                g_dtmAccessCode   = (g_dtmAccessCode&(0xffff00ff))|(g_rfPhyDtmCmd[1]<<8);
                // PHYPLUSE DEFINED:  RF_PHY_DTM_MODE_SET_ACCCODE_2
            }
            else if(g_dtmCtrl==0x38)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_ACCCODE_2;
                g_dtmAccessCode   = (g_dtmAccessCode&(0xff00ffff))|(g_rfPhyDtmCmd[1]<<16);
                // PHYPLUSE DEFINED:  RF_PHY_DTM_MODE_SET_ACCCODE_3
            }
            else if(g_dtmCtrl==0x39)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_ACCCODE_3;
                g_dtmAccessCode   = (g_dtmAccessCode&(0x00ffffff))|(g_rfPhyDtmCmd[1]<<24);
                // PHYPLUSE DEFINED:  TX TEST MODE Set FREQ FOFF
            }
            else if(g_dtmCtrl==0x3a)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_FREQ_FOFF;

                if(g_dtmManualConfig & RF_PHY_DTM_MANUL_FOFF)
                {
                    g_rfPhyFreqOffSet    = (g_dtmPara-10)*5;//[0:1:20]-->[-50:5:50]-->[-200:20:200]KHz
                }

                // PHYPLUSE DEFINED:  TX TEST MODE Set TP_CAL MANUAL
            }
            else if(g_dtmCtrl==0x3b)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_TPCAL_MANUAL;

                if(g_dtmPara==0)
                {
                    g_dtmTpCalEnable  = 1;
                }
                else
                {
                    g_dtmTpCalEnable  = 0;
                    g_rfPhyTpCal0     = (g_dtmPara)<<1;
                    PHY_REG_WT( 0x40030094,0x00001000+g_rfPhyTpCal0);   // tp_cal val
                }

                // PHYPLUSE DEFINED:  TX TEST MODE Set XTAL CAP
            }
            else if(g_dtmCtrl==0x3c)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_XTAL_CAP;

                if(g_dtmManualConfig & RF_PHY_DTM_MANUL_XTAL_CAP)
                {
                    XTAL16M_CAP_SETTING(g_dtmPara);
                }

                // PHYPLUSE DEFINED:  TX TEST MODE Set TX POWER
            }
            else if(g_dtmCtrl==0x3d)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_TX_POWER;

                if(g_dtmManualConfig & RF_PHY_DTM_MANUL_TXPOWER)
                {
                    g_dtmTxPower      = g_dtmPara;
                }

                // PHYPLUSE DEFINED:  TX TEST MODE Continous Modulation
            }
            else if(g_dtmCtrl==0x3e)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_TX_CTMOD;
                g_dtmFreq         = g_dtmPara;
                // PHYPLUSE DEFINED:  TX TEST MODE Single Tone
            }
            else if(g_dtmCtrl==0x3f)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_TX_SINGLE;
                g_dtmFreq         = g_dtmPara;
            }
            else
            {
                g_dtmModeType = RF_PHY_DTM_MODE_ERROR;
            }

            //TEST END
        }
        else
        {
            if(g_dtmCtrl==0 && g_dtmPara==0)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_TEST_END;
                // PHYPLUSE DEFINED:  get foff
            }
            else if(g_dtmCtrl==0x3f && g_dtmPara==0)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_GET_FOFF;
                // PHYPLUSE DEFINED:  get tpCal
            }
            else if(g_dtmCtrl==0x3f && g_dtmPara==1)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_GET_TPCAL;
                // PHYPLUSE DEFINED:  get RSSI
            }
            else if(g_dtmCtrl==0x3f && g_dtmPara==2)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_GET_RSSI;
                // PHYPLUSE DEFINED:  get CARR_SENS
            }
            else if(g_dtmCtrl==0x3f && g_dtmPara==3)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_GET_CARR_SENS;
                // PHYPLUSE DEFINED:  get PER report Auto
            }
            else if(g_dtmCtrl==0x3f && g_dtmPara==4)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_GET_PER_AUTO;
            }
            else
            {
                g_dtmModeType = RF_PHY_DTM_MODE_ERROR;
            }
        }

        //tx-rx test
    }
    else if(   g_dtmCmd == 1 || g_dtmCmd == 2)
    {
        g_dtmFreq       = ((g_rfPhyDtmCmd[0] & 0x3f)   );      // bit 13  8
        g_dtmLength     = ((g_rfPhyDtmCmd[1] & 0xfc)>>2);      // bit  7  2
        g_dtmPKT        = ((g_rfPhyDtmCmd[1] & 0x03)   );      // bit  1  0

        if(g_dtmPktCount==3 && (g_rfPhyPktFmt==PKT_FMT_BLR125K || g_rfPhyPktFmt==PKT_FMT_BLR500K ))
        {
            g_dtmPKT = 4;//all ones 'b 11111111
        }

        g_dtmModeType   = (g_dtmCmd ==1 )? RF_PHY_DTM_MODE_RX_PER : RF_PHY_DTM_MODE_TX_BURST;
    }
}

#endif
//-------------------------------------------------------------------------------------


#if(RF_PHY_DTM_CTRL_MOD == RF_PHY_DTM_CTRL_UART)
/**************************************************************************************
    @fn          rf_phy_dtm_evt_send

    @brief       This function process for rf phy direct test, test mode trigged

    input parameters

    @param       none

    output parameters

    @param       none

    @return      none
*/
void rf_phy_dtm_evt_send(uint8_t dtmType )
{
    //clear Event Buf
    g_rfPhyDtmEvt[0] = 0;
    g_rfPhyDtmEvt[1] = 0;

    if(dtmType==RF_PHY_DTM_MODE_ERROR)
    {
        g_dtmEvt    = 0;    //status
        g_dtmStatus = 1;    //Error
    }
    else
    {
        if(dtmType == RF_PHY_DTM_MODE_TEST_END)
        {
            g_dtmEvt    = 0x80;    //report
            g_dtmStatus = 0;
            g_dtmRsp    = g_dtmPktCount;
        }
        else if(dtmType == RF_PHY_DTM_MODE_GET_FOFF)
        {
            g_dtmEvt    = 0x80;    //report
            g_dtmStatus = 0;
            g_dtmRsp    = g_dtmFoff;
        }
        else if(dtmType == RF_PHY_DTM_MODE_GET_TPCAL)
        {
            g_dtmEvt    = 0x80;    //report
            g_dtmStatus = 0;
            g_dtmRsp    = g_rfPhyTpCal0;
        }
        else if(dtmType == RF_PHY_DTM_MODE_GET_RSSI )
        {
            g_dtmEvt    = 0x80;    //report
            g_dtmStatus = 0;
            g_dtmRsp    = g_dtmRssi;
        }
        else if(dtmType == RF_PHY_DTM_MODE_GET_CARR_SENS)
        {
            g_dtmEvt    = 0x80;    //report
            g_dtmStatus = 0;
            g_dtmRsp    = g_dtmCarrSens;
        }
        else if(dtmType == RF_PHY_DTM_MODE_READ_SUPPORTED_TEST_CASE)
        {
            g_dtmEvt    = 0x00;    //status
            g_dtmStatus = 0;
            g_dtmRsp    = 0x0c;    //bit3,support stable modulation index
            //bit2, support LE 2M PHY
            //bit1, support LE PACKET LENGTH EXTENTION
        }
        else if(dtmType == RF_PHY_DTM_MODE_READ_MAX_TX_OCTETS)
        {
            g_dtmEvt    = 0x00;    //status
            g_dtmStatus = 0;
            g_dtmRsp    = 27<<1;
        }
        else if(dtmType == RF_PHY_DTM_MODE_READ_MAX_TX_TIME)
        {
            g_dtmEvt    = 0x00;    //status
            g_dtmStatus = 0;
            g_dtmRsp    = 1352<<1;
        }
        else if(dtmType == RF_PHY_DTM_MODE_READ_MAX_RX_OCTETS)
        {
            g_dtmEvt    = 0x00;    //status
            g_dtmStatus = 0;
            g_dtmRsp    = 27<<1;
        }
        else if(dtmType == RF_PHY_DTM_MODE_READ_MAX_RX_TIME)
        {
            g_dtmEvt    = 0x00;    //status
            g_dtmStatus = 0;
            g_dtmRsp    = 1352<<1;
        }
        else
        {
            g_dtmEvt    = 0;        //status
            g_dtmStatus = 0;        //Sucess
            g_dtmRsp    = g_dtmModeType;
        }
    }

    g_rfPhyDtmEvt[0] = g_rfPhyDtmEvt[0] | g_dtmEvt    | (g_dtmRsp >>   8);
    g_rfPhyDtmEvt[1] = g_rfPhyDtmEvt[1] | g_dtmStatus | (g_dtmRsp & 0xff);
    DTM_OUT(g_rfPhyDtmEvt[0]);
    DTM_OUT(g_rfPhyDtmEvt[1]);
}


#endif
//-------------------------------------------------------------------------------------


#if(RF_PHY_DTM_CTRL_MOD == RF_PHY_DTM_CTRL_UART)
/**************************************************************************************
    @fn          rf_phy_dtm_trigged

    @brief       This function process for rf phy direct test, test mode trigged

    input parameters

    @param       none

    output parameters

    @param       none

    @return      none
*/
void rf_phy_dtm_trigged(void)
{
    //cal the pkt length and pkt interval
    g_dtmLength = ((0x03&g_dtmExtLen)<<6)|(0x3f&g_dtmLength);
    int pktLenUs =0;                        //(g_dtmLength+10)<<3;
    uint8_t rfChn = 2+(g_dtmFreq<<1);
    uint8_t preambleLen = 1;

//    if(         pktLenUs <= 376  )  {   g_dtmPktIntv = 625;
//    }else if(   pktLenUs <= 1000 )  {   g_dtmPktIntv = 1250;
//    }else if(   pktLenUs <= 1624 )  {   g_dtmPktIntv = 1875;
//    }else if(   pktLenUs <= 2120 )  {   g_dtmPktIntv = 2500;
//    }else                           {   g_dtmPktIntv = 2500;
//    }
//
    if(         g_rfPhyPktFmt==PKT_FMT_BLE1M)
    {
        pktLenUs = (g_dtmLength+1+4+2+3)<<3;
    }
    else if(   g_rfPhyPktFmt==PKT_FMT_BLE2M)
    {
        pktLenUs = (g_dtmLength+2+4+2+3)<<2;
    }
    else if(   g_rfPhyPktFmt==PKT_FMT_BLR500K)
    {
        pktLenUs = 80+296+((g_dtmLength+2+3)<<4)+6;
    }
    else if(   g_rfPhyPktFmt==PKT_FMT_BLR125K)
    {
        pktLenUs = 80+296+((g_dtmLength+2+3)<<6)+24;
    }
    else
    {
        pktLenUs = (g_dtmLength+4+1+1)<<5;//per byte -> 2symbol ->32us
        rfChn  = 5+g_dtmFreq*5;//for zigbee
    }

    g_dtmPktIntv = (((pktLenUs+249)+624)/625)*625;//  ceil((L+249)/625) * 625

    if(g_dtmModeType == RF_PHY_DTM_MODE_GET_PER_AUTO)
    {
        g_dtmPerAutoIntv = g_dtmPktIntv*1000;
        g_dtmTick = read_current_fine_time();
    }

    PHY_REG_WT( 0x40030040,0x00030000);     // close tx_bb test mode
    PHY_REG_WT( 0x400300a0,0x0000000e);     // clr pll_auto override
    PHY_REG_WT( 0x400300a4,0x00000100);     // clr tx_auto
    PHY_REG_WT( 0x400300a0,0x00000000);     // clr pll_auto override
    PHY_REG_WT( 0x4003008c,0x00104040);     // clr tp_cal_en

    if(g_dtmModeType == RF_PHY_DTM_MODE_RESET)
    {
        g_dtmPktCount       =   0;
        g_dtmRsp            =   0;
        g_dtmExtLen         =   0;
        g_dtmLength         =   0;
        g_dtmPktIntv        =   0;
        g_dtmTxPower        =   g_rfPhyTxPower;
        g_dtmFoff           =   0;
        g_dtmRssi           =   0;
        g_dtmCarrSens       =   0;
        g_rfPhyPktFmt       =   PKT_FMT_BLE1M;
        g_dtmPKT            =   0;
        DCDC_CONFIG_SETTING(0x0a);
    }
    else if(   g_dtmModeType == RF_PHY_DTM_MODE_TX_BURST
               || g_dtmModeType == RF_PHY_DTM_MODE_TX_CTMOD
               || g_dtmModeType == RF_PHY_DTM_MODE_TX_SINGLE   )
    {
        //====== tp cal
        if(g_dtmTpCalEnable)
        {
            rf_phy_ana_cfg();
            //rf_tpCal_cfg(rfChn);
            rf_tpCal_cfg_avg(rfChn,4);
        }

        //====== rf initial
        rf_phy_ini();
        rf_phy_set_txPower(g_dtmTxPower);
        ll_hw_set_timing(g_rfPhyPktFmt);

        //add some tx_foff
        if(g_rfPhyFreqOffSet>=0)
            PHY_REG_WT(0x400300b4,rfChn+(g_rfPhyFreqOffSet<<8));
        else
            PHY_REG_WT(0x400300b4,rfChn-1+((255+g_rfPhyFreqOffSet)<<8));

        if(!(g_rfPhyPktFmt==PKT_FMT_ZIGBEE))
        {
            PHY_REG_WT(0x40030048,RF_PHY_DTM_CRC_WT     );
            PHY_REG_WT(0x4003004c,g_dtmAccessCode  );
            PHY_REG_WT(0x40030040,0x00030010);
        }

        //----------------------------------
        //PRBS SEED should be configed before tx_mod en
        PHY_REG_WT(0x40030044,RF_PHY_DTM_PRBS9_SEED );

        if(g_dtmModeType == RF_PHY_DTM_MODE_TX_SINGLE)
        {
            PHY_REG_WT(0x400300a4,0x00000100);      //tp_mod en and pa_ramp_en
        }
        else
        {
            PHY_REG_WT(0x400300a4,0x00000140);      //tp_mod en and pa_ramp_en
        }

        //PHY_REG_WT(0x40030040,0x000300f0);//need to extend the preamble length

        if(g_dtmModeType == RF_PHY_DTM_MODE_TX_BURST)
        {
            //DCDC_CONFIG_SETTING(0x08);
            if(!(g_rfPhyPktFmt==PKT_FMT_ZIGBEE))
            {
                #if 0

                //[15:8] payload Len [7:5] preamble len, [4] tx mode, [3:0] payload type
                //PHY_REG_WT(0x40030040,(0x00030010|(g_dtmLength<<8)|(preambleLen<<5)|(0x0f & g_dtmPKT)) );
                if((g_rfPhyPktFmt == PKT_FMT_BLR125K || g_rfPhyPktFmt == PKT_FMT_BLR500K)
                        && g_dtmPKT == 0x03 )
                {
                    //for PKT_FMT all ones 11111111
                    //PHY_REG_WT(0x40030040,(0x00030010|((g_dtmLength+2)<<8)|(preambleLen<<5)|0x04) );
                    PHY_REG_WT(0x40030040,(0x00030010|((g_dtmLength)<<8)|(preambleLen<<5)|0x04) );
                }
                else
                {
                    //PHY_REG_WT(0x40030040,(0x00030010|((g_dtmLength+2)<<8)|(preambleLen<<5)|(0x0f & g_dtmPKT)) );
                    PHY_REG_WT(0x40030040,(0x00030010|((g_dtmLength)<<8)|(preambleLen<<5)|(0x0f & g_dtmPKT)) );
                }

                #else
                ll_hw_rst_tfifo();
                extern void rf_phy_dtm_ble_pkt_gen(void);
                rf_phy_dtm_ble_pkt_gen();
                #endif
            }
            else
            {
                ll_hw_rst_tfifo();
                rf_phy_dtm_zigbee_pkt_gen();
            }
        }
        else
        {
            if(!(g_rfPhyPktFmt==PKT_FMT_ZIGBEE))
            {
                //[15:8] payload Len [7:5] preamble len, [4] tx mode, [3:0] payload type
                PHY_REG_WT(0x40030040,0x00030010|(preambleLen<<5));
            }
            else
            {
                //[15:8] payload Len [7:5] preamble len, [4] tx mode, [3:0] payload type
                PHY_REG_WT(0x40030040,0x000b0013|(preambleLen<<5));
                PHY_REG_WT(0x40030000,0x78068002);
            }
        }

        //close ll hw irg
        ll_hw_set_irq(0);
        ll_hw_set_stx();
        ll_hw_clr_irq();

        if(g_dtmModeType == RF_PHY_DTM_MODE_TX_BURST)
        {
            if(!(g_rfPhyPktFmt==PKT_FMT_ZIGBEE))
            {
                //PHY_REG_WT( 0x400300a0,0x0000000e);     // clr pll_auto override
                //PHY_REG_WT( 0x400300a4,0x00000154);     // clr tx_auto
                ll_hw_trigger();
            }
            else
            {
                ll_hw_trigger();
                //PHY_REG_WT( 0x400300a0,0x0000000e);     // clr pll_auto override
                //PHY_REG_WT( 0x400300a4,0x00000154);     // clr tx_auto
            }

            g_dtmTick = read_current_fine_time();
        }
        else
        {
            ll_hw_trigger();
        }
    }
    else if(   g_dtmModeType == RF_PHY_DTM_MODE_RX_PER
               || g_dtmModeType == RF_PHY_DTM_MODE_GET_PER_AUTO )
    {
        rf_phy_ana_cfg();
        rf_rxDcoc_cfg(/*rfChn*/2,/*bwSet*/1,&g_rfPhyRxDcIQ);        //set the rfChn as 2402 for BW=1MHz
        //====== rf initial
        rf_phy_ini();
        ll_hw_set_timing(g_rfPhyPktFmt);
        PHY_REG_WT(0x40031024,g_dtmPktIntv-100);//timeout set
        PHY_REG_WT(0x40031028,g_dtmPktIntv-100);//timeout set

        if((rfChn&0x0f)==0)
        {
            PHY_REG_WT(0x400300b4,rfChn);//freqOffset = 0
            subWriteReg(0x4003011c,11,8,3);//enable spur notch filter setting
        }
        else
        {
            subWriteReg(0x4003011c,11,8,0);//disable spur notch filter setting

            if(g_rfPhyFreqOffSet>=0)
                PHY_REG_WT(0x400300b4,rfChn+(g_rfPhyFreqOffSet<<16));
            else
                PHY_REG_WT(0x400300b4,rfChn-1+((255+g_rfPhyFreqOffSet)<<16));
        }

        if(!(g_rfPhyPktFmt==PKT_FMT_ZIGBEE))
        {
            PHY_REG_WT(0x40030044,RF_PHY_DTM_PRBS9_SEED );
            PHY_REG_WT(0x40030048,RF_PHY_DTM_CRC_WT     );
            PHY_REG_WT(0x4003004c,g_dtmAccessCode  );
        }

        //set rx no timeout[31:16] + max packet len
        //PHY_REG_WT(0x4003000c,(g_dtmLength+3)>255 ? 255: g_dtmLength+3 );
        PHY_REG_WT(0x4003000c,(g_dtmLength+3)>255 ? 255:( g_dtmLength<40 ? 40 : g_dtmLength+3) );
        //close ll hw irg
        ll_hw_set_irq(0);
        ll_hw_set_srx();
        ll_hw_clr_irq();
        ll_hw_trigger();
    }
}
void gen_pn_prbs9(uint16_t seed, int length,uint8_t* pnOut)
{
    uint8_t bitOut[8] = {0};
    uint8_t reg[9];
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t feedback = 0;

    for (i = 0; i < 9; i++)
    {
        reg[i] = (seed >> i) & 0x01;
    }

    for (i = 0; i < length; i++)
    {
        for (j = 0; j < 8; j++)
        {
            feedback = reg[5] ^ reg[0];
            bitOut[j] = reg[0];
            reg[0] = reg[1];
            reg[1] = reg[2];
            reg[2] = reg[3];
            reg[3] = reg[4];
            reg[4] = reg[5];
            reg[5] = reg[6];
            reg[6] = reg[7];
            reg[7] = reg[8];
            reg[8] = feedback;
        }

        bit_to_byte(bitOut, pnOut + i);
    }
}
void rf_phy_dtm_ble_pkt_gen(void)
{
    uint8_t tmp[256];
    tmp[1] = g_dtmLength;
    tmp[0] = g_dtmPKT;
    uint8_t pld=0x00;

    if(g_dtmPKT==1)
    {
        pld = 0x0f;
    }
    else if(g_dtmPKT==2)
    {
        pld=0x55;
    }
    else if(g_dtmPKT==3)
    {
        pld=0xff;
        tmp[0]=4;
    }

    if(g_dtmPKT==0)
    {
        gen_pn_prbs9(0x01ff,g_dtmLength,&(tmp[2]));
    }
    else
    {
        for(uint8_t i=0; i<g_dtmLength; i++)
            tmp[2+i] =pld;
    }

    PHY_REG_WT(0x40030040,0x00030000);
    ll_hw_write_tfifo(tmp,g_dtmLength+2);//include the crc16
}

void rf_phy_dtm_zigbee_pkt_gen(void)
{
    uint8_t seed[16]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    uint8_t crcCode[2]= {0xff,0xff};
    uint8_t i=0;
    uint8_t tmp[128];
    tmp[0] = g_dtmLength;

    for(i=0; i<g_dtmLength-1; i++)
    {
        tmp[i+1] = ( (((tmp[i]&0x01)<<7) | (tmp[i]>>1) ) ^ 0x61);
    }

    zigbee_crc16_gen(tmp+1,g_dtmLength-2,seed,crcCode);
    tmp[g_dtmLength-1] = crcCode[0];
    tmp[g_dtmLength  ] = crcCode[1];
    ll_hw_write_tfifo(tmp,g_dtmLength+1);//include the crc16
}
#endif
//-------------------------------------------------------------------------------------



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
void rf_phy_get_pktFoot(uint8* rssi, uint16* foff,uint8* carrSens)
{
    uint32_t pktFoot0;
    uint32_t pktFoot1;
    uint16_t tmpFoff;
    pktFoot0        = (*(volatile uint32_t*) 0x400300e4);
    pktFoot1        = (*(volatile uint32_t*) 0x400300e8);
    tmpFoff         =   pktFoot0 & 0x3ff;
    *foff           =   (tmpFoff>512) ? tmpFoff-512 : tmpFoff+512;
    *rssi           =   (pktFoot1>>24)     ;
    *carrSens       =   (pktFoot0>>24)     ;
}
void rf_phy_get_pktFoot_fromPkt(uint32 pktFoot0, uint32 pktFoot1,
                                uint8* rssi, uint16* foff,uint8* carrSens)
{
    uint16_t tmpFoff;
//    pktFoot0        = (*(volatile uint32_t *) 0x400300e4);
//    pktFoot1        = (*(volatile uint32_t *) 0x400300e8);
    tmpFoff         =   pktFoot0 & 0x3ff;
    *foff           =   (tmpFoff>512) ? tmpFoff-512 : tmpFoff+512;
    *rssi           =   (pktFoot1>>24)     ;
    *carrSens       =   (pktFoot0>>24)     ;
}
/**************************************************************************************
    @fn          rf_phy_set_txPower

    @brief       This function process for rf phy tx power config

    input parameters

    @param       txPower  : tx pa power setting (0~0x1f)

    output parameters

    @param       none

    @return      none
*/
void rf_phy_set_txPower  (uint8 txPower)
{
    if(RF_PHY_TX_POWER_EXTRA_MAX==txPower)
    {
        DCDC_CONFIG_SETTING(0x08);//set dcdc to highest
        RF_PHY_LO_LDO_SETTING(0);
        RF_PHY_LNA_LDO_SETTING(0);
        RF_PHY_PA_VTRIM_SETTING(1);
    }
    else
    {
        DCDC_CONFIG_SETTING(0x0a);
        RF_PHY_LO_LDO_SETTING(2);
        RF_PHY_LNA_LDO_SETTING(1);
        RF_PHY_PA_VTRIM_SETTING(0);
    }

    PHY_REG_WT(0x400300b8,(PHY_REG_RD(0x400300b8)&0x0fff) | ((txPower&0x1f)<<12));
}



/*******************************************************************************
    @fn          rc32k calibration

    @brief       RF calibration function



    input parameters
    @param       None.


    output parameters

    @param       None.

    @return      rcCalCode

*/
uint8 rc32k_calibration(void)
{
    uint8 delay = 10;
    uint32_t  temp=0;
    *(volatile uint32_t*) 0x4000f05c &= 0xfffffffe;                   // disable RC32K calibration
    WaitRTCCount(6);
    // calibrate RC32K clock
    *(volatile uint32_t*) 0x4000f018 |= 0x80;                   // set capbank controlled by calibration
    *(volatile uint32_t*) 0x4000f05c |= 0x01;                   // enable RC32K calibration

    while (!(*(volatile uint32_t*) 0x4000f068 & 0x200)          // check RC32K calibration OK flag, normally need >200us
            && delay -- > 0)
    {
        WaitRTCCount(8);//30.125*8 us each loop
    }

    if (delay > 0)
    {
        temp = (*(volatile uint32_t*) 0x4000f060 & 0x3f0000) >> 15;         // read 6bit calibration result
        *(volatile uint32_t*)0x4000f018 = (*(volatile uint32_t*) 0x4000f018 & 0xffffff81) | temp;     // write the result
    }

    *(volatile uint32_t*) 0x4000f018 &= 0xffffff7f;             // set capbank controlled by AON
    return (uint8)(0x7f&(temp>>1));
}
/*******************************************************************************
    @fn          rf_calibrate API

    @brief       RF calibration function



    input parameters
    @param       None.


    output parameters

    @param       None.

    @return      None

*/
void rf_calibrate1(void)
{
    /*  rf_phy_ana_cfg will overwrite txpower setting configed in rf_phy_ini.
        in ble_main, rf_calibration is called after rf_phy_ini,so comment the rf_phy_ana_cfg
    */
    //rf_phy_ana_cfg();
    //========== do rf tp cal for tx and rx dcoffset cal
    rf_tpCal_gen_cap_arrary();                                  //generate the tpCal cap arrary
    //rf_tpCal_cfg(/*rfChn*/2);
    rf_rxDcoc_cfg(/*rfChn*/88,/*bwSet*/1,&g_rfPhyRxDcIQ);       //set the rfChn as 2488 for BW=1MHz
    g_rc32kCalRes = rc32k_calibration();
}


static void rf_phy_dtm_init(void)
{
    //*(volatile int *) 0xe000e100 |= 0x800;
    *(volatile int*) 0xe000e100 = 0x800; //only use uart irq
    *(volatile unsigned int*) 0x40004004 |= 0x01; //ENABLE_ERBFI;
    //set timer3 free run
    //AP_TIM3->ControlReg = 0x05;
    //24bit count down mode no IRQ
    set_timer(AP_TIM3, RF_PHY_TIME_BASE);
    NVIC_DisableIRQ(TIM3_IRQn);
}


static void rf_phy_dtm_stop(void)
{
    //======================================================================
    //stop RF_PHY_DTM_MODE
    //PHY_REG_WT( 0x400300a4,0x00000100);     // clr tx_auto
    //PHY_REG_WT( 0x400300a0,0x0000000e);     // pll_auto override
    subWriteReg(AP_PCR_BASE+0x0c,10,10,0);//reset bbll
    PHY_REG_WT( 0x400300a4,0x00000140);     // clr tx_auto
    PHY_REG_WT( 0x400300a0,0x0000000e);     // clr pll_auto override
    PHY_REG_WT( 0x400300a0,0x00000000);     // clr pll_auto override
    subWriteReg(AP_PCR_BASE+0x0c,10,10,1);//release bbll reset
}

/**************************************************************************************
    @fn          rf_phy_dtm_ext_tx_singleTone

    @brief       This function process for rf phy direct test, test mode interup

    input parameters

    @param       txPower     :   rf tx power
                rfChnIdx    :   rf channel = 2402+(rfChnIdx<<1)
                rfFoff      :   rf freq offset = rfFoff*4KHz
                testTimeUs  :   test loop active time(ms)

    output parameters

    @param       none

    @return      none
*/
void    rf_phy_dtm_ext_tx_singleTone(uint8_t txPower, uint8_t rfChnIdx,uint8_t xtal_cap,int8_t rfFoff,uint32 testTimeUs)
{
    rf_phy_dtm_init();
    //======================================================================
    //rest RF_PHY_DTM_MODE
    g_dtmModeType   =   RF_PHY_DTM_MODE_RESET;
    rf_phy_dtm_trigged();
    //======================================================================
    //config dtm mode
    g_dtmModeType       =   RF_PHY_DTM_MODE_TX_SINGLE;
    g_dtmTxPower        =   txPower;
    g_dtmFreq           =   rfChnIdx;
    g_rfPhyPktFmt       =   PKT_FMT_BLE1M;
    g_rfPhyFreqOffSet   =   rfFoff;
    XTAL16M_CAP_SETTING(xtal_cap);
    rf_phy_dtm_trigged();
    WaitUs(testTimeUs);
    //======================================================================
    //stop RF_PHY_DTM_MODE
    rf_phy_dtm_stop();
}


/**************************************************************************************
    @fn          rf_phy_dtm_ext_tx_modulation

    @brief       This function process for rf phy direct test, test mode interup

    input parameters

    @param       txPower     :   rf tx power
                rfChnIdx    :   rf channel = 2402+(rfChnIdx<<1)
                rfFoff      :   rf freq offset = rfFoff*4KHz
                pktType     :   modulaiton data type, 0: prbs9, 1: 1111000: 2 10101010
                testTimeUs  :   test loop active time(ms)

    output parameters

    @param       none

    @return      none
*/
void    rf_phy_dtm_ext_tx_modulation(uint8_t txPower, uint8_t rfChnIdx,uint8_t xtal_cap,int8_t rfFoff,uint8_t pktType,uint32 testTimeUs)
{
    rf_phy_dtm_init();
    //======================================================================
    //rest RF_PHY_DTM_MODE
    g_dtmModeType   =   RF_PHY_DTM_MODE_RESET;
    rf_phy_dtm_trigged();
    //======================================================================
    //config dtm mode
    g_dtmModeType       =   RF_PHY_DTM_MODE_TX_CTMOD;
    g_dtmTxPower        =   txPower;
    g_dtmFreq           =   rfChnIdx;
    g_rfPhyPktFmt       =   PKT_FMT_BLE1M;
    g_dtmPKT            =   pktType;
    g_rfPhyFreqOffSet   =   rfFoff;
    XTAL16M_CAP_SETTING(xtal_cap);
    rf_phy_dtm_trigged();
    WaitUs(testTimeUs);
    //======================================================================
    //stop RF_PHY_DTM_MODE
    rf_phy_dtm_stop();
}

/**************************************************************************************
    @fn          rf_phy_dtm_ext_tx_mt_burst

    @brief       This function process for rf phy direct test, test mode interup

    input parameters

    @param       txPower     :   rf tx power
                rfChnIdx    :   rf channel = 2402+(rfChnIdx<<1)
                rfFoff      :   rf freq offset = rfFoff*4KHz
                pktType     :   modulaiton data type, 0: prbs9, 1: 1111000: 2 10101010
                pktLength   :   pkt length(Byte)

                txPktNum    :   burst pkt tx number
                txPktIntv   :   txPkt intv,0 txPkt intv is pkt interval =  ceil((L+249)/625) * 625

    output parameters

    @param       none

    @return      none
*/
void    rf_phy_dtm_ext_tx_mod_burst(uint8_t txPower, uint8_t rfChnIdx,uint8_t xtal_cap,int8_t rfFoff,
                                    uint8_t pktType, uint8_t pktLength,uint32 txPktNum,uint32 txPktIntv)
{
    rf_phy_dtm_init();
    //======================================================================
    //rest RF_PHY_DTM_MODE
    g_dtmModeType   =   RF_PHY_DTM_MODE_RESET;
    rf_phy_dtm_trigged();
    //======================================================================
    //config dtm mode
    g_dtmModeType       =   RF_PHY_DTM_MODE_TX_BURST;
    g_dtmTxPower        =   txPower;
    g_dtmFreq           =   rfChnIdx;
    g_rfPhyPktFmt       =   PKT_FMT_BLE1M;
    g_dtmPKT            =   pktType;
    g_dtmLength         =   pktLength;
    g_rfPhyFreqOffSet   =   rfFoff;
    XTAL16M_CAP_SETTING(xtal_cap);
    rf_phy_dtm_trigged();
    uint32_t currTick=0;
    uint32_t deltTick=0;
    uint32_t tIntv = 0;

    if(txPktIntv==0)
        tIntv = g_dtmPktIntv;
    else
        tIntv  = txPktIntv;

    while(txPktNum>0)
    {
        ll_hw_set_trx_settle    (100, 8, 90);       //TxBB,RxAFE,PLL
        //subWriteReg(0x400300d4,7,4,0x0f);           //boost vco current[7:4] to 0xf to reduce dcdc spur
        currTick = read_current_fine_time();
        deltTick = RF_PHY_TIME_DELTA(currTick,g_dtmTick);

        if(deltTick>=tIntv)
        {
            ll_hw_rst_tfifo();
            ll_hw_trigger();
            txPktNum--;
            g_dtmTick = g_dtmTick+tIntv;

            if(g_dtmTick>RF_PHY_TIME_BASE)
                g_dtmTick = g_dtmTick-RF_PHY_TIME_BASE;
        }
    }

    //======================================================================
    //stop RF_PHY_DTM_MODE
    rf_phy_dtm_stop();
}


/**************************************************************************************
    @fn          rf_phy_dtm_ext_rx_demod_burst

    @brief       This function process for rf phy direct test, test mode interup

    input parameters

    @param       rfChnIdx        :   rf channel = 2402+(rfChnIdx<<1)
                rfFoff          :   rf freq offset = rfFoff*4KHz
                pktLength       :   pkt length(Byte)
                rxWindow        :   rx demod window length(us)
                rxTimeOut       :   rx on time (ms)

    output parameters

    @param       rxEstFoff       :   rx demod estimated frequency offset
                rxEstRssi       :   rx demod estimated rssi
                rxEstCarrSens   :   rx demod estimated carrier sense
                rxPktNum        :   rx demod received pkt number

    @return      none
*/
void    rf_phy_dtm_ext_rx_demod_burst(uint8_t rfChnIdx,int8_t rfFoff,uint8_t xtal_cap,uint8_t pktLength,uint32 rxTimeOut,uint32 rxWindow,
                                      uint16_t* rxEstFoff,uint8_t* rxEstRssi,uint8_t* rxEstCarrSens,uint16_t* rxPktNum)
{
    rf_phy_dtm_init();
    //======================================================================
    //rest RF_PHY_DTM_MODE
    g_dtmModeType   =   RF_PHY_DTM_MODE_RESET;
    rf_phy_dtm_trigged();
    //======================================================================
    //config dtm mode
    g_dtmModeType       =   RF_PHY_DTM_MODE_RX_PER;
    g_dtmFreq           =   rfChnIdx;
    g_rfPhyPktFmt       =   PKT_FMT_BLE1M;
    g_dtmLength         =   pktLength;
    g_rfPhyFreqOffSet   =   rfFoff;
    rf_phy_dtm_trigged();
    g_dtmPktCount = 0;
    uint8_t rssi;
    uint16_t foff;
    uint8_t carrSens;
    uint32 t0 = hal_systick();
    XTAL16M_CAP_SETTING(xtal_cap);

    while(hal_ms_intv(t0)<rxTimeOut)
    {
        //====================================
        //by ZQ 2018-09-04
        //config rf phy clk
        subWriteReg(0x4000f018, 14, 9, 0x3f);//rc32M clk slow

        if(rxWindow>0)
        {
            PHY_REG_WT(0x40031024,0xffff&rxWindow);
            PHY_REG_WT(0x40031028,0xffff&rxWindow);
        }

        if(ll_hw_get_irq_status()&LIRQ_MD)
        {
            if(ll_hw_get_irq_status()&LIRQ_COK)
            {
                g_dtmPktCount++;
                rf_phy_get_pktFoot(&rssi,&foff,&carrSens);
            }
            else if(ll_hw_get_irq_status()&LIRQ_CERR)
            {
                g_dtmRxCrcNum++;
            }
            else if(ll_hw_get_irq_status()&LIRQ_RTO)
            {
                g_dtmRxTONum++;
            }
            else
            {
                //wrap the pktCount
                g_dtmPktCount= (g_dtmPktCount==65535) ? 0 : g_dtmPktCount;
            }

            ll_hw_clr_irq();
            ll_hw_trigger();

            if(g_dtmPktCount>0)
            {
                g_dtmRssi       = (g_dtmPktCount==1) ? rssi     :((g_dtmRssi+rssi)          >>1);
                g_dtmFoff       = (g_dtmPktCount==1) ? foff     :((g_dtmFoff+foff)          >>1);
                g_dtmCarrSens   = (g_dtmPktCount==1) ? carrSens :((g_dtmCarrSens+carrSens)  >>1);
            }
        }
    }

    *rxEstFoff      = g_dtmFoff;
    *rxEstRssi      = g_dtmRssi;
    *rxPktNum       = g_dtmPktCount;
    *rxEstCarrSens  = g_dtmCarrSens;
    //======================================================================
    //stop RF_PHY_DTM_MODE
    rf_phy_dtm_stop();
}

/**************************************************************************************
    @fn          rf_phy_dtm_ext_acc_code_set

    @brief       config the acc_code in rf phy dtm

    input parameters

    @param       acc_code        :   sync word

    output parameters
    @return      none
*/
void    rf_phy_dtm_ext_acc_code_set(uint32 accCode)
{
    g_dtmAccessCode = accCode;
}







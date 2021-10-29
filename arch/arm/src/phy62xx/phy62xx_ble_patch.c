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

#include <stdlib.h>
#include <string.h>

//#include "common.h"
//#include "uart.h"
//#include "dma.h"
//#include "flash.h"
//#include "gpio_rom.h"
//#include "i2c.h"
//#include "i2s.h"
//#include "spi.h"
//#include "timer.h"
#include "rom_sym_def.h"

#include "types.h"
#include "ll.h"
#include "rf_phy_driver.h"
#include "global_config.h"
#include "jump_function.h"
#include "uart.h"
#include "ll_sleep.h"
#include "ll_debug.h"
#include "ll.h"
#include "bus_dev.h"
#include "ll_hw_drv.h"
#include "gpio.h"
#include "ll_enc.h"
#include "OSAL_Clock.h"
#include "osal_bufmgr.h"
#include "OSAL_Memory.h"
#include "log.h"
#include "hci.h"
#include "hci_tl.h"
#include "version.h"
#include "flash.h"
#include "gatt.h"
#include "att.h"
#include "error.h"
#include "clock.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
//========================================================
// build config
//#define __BUILD_RF_LIB_SLA__            (0x1)
//#define __BUILD_RF_LIB_MST__            (0x2)
//#define __BUILD_RF_LIB_MULTI__            ( __BUILD_RF_LIB_MST__ | __BUILD_RF_LIB_SLA__ )
//
//#ifndef __BUILD_PATCH_CFG__
//    #define __BUILD_PATCH_CFG__             __BUILD_RF_LIB_MST__
//#endif


#define DBG_BUILD_LL_TIMING             0               //0x01 for enable LL timing debug

int hal_uart_send_buff(UART_INDEX_e uart_index,uint8_t* buff,uint16_t len);

#define logx(...) {char tmp_str[128]; sprintf(tmp_str, __VA_ARGS__); hal_uart_send_buff(0, tmp_str , strlen(tmp_str)+1);}

// ======================
#define DBG_GPIO_WRITE(a,b)     gpio_write((a),(b))
//#define DBG_GPIO_WRITE(a,b)
#define DBGIO_LL_TRIG       P23
#define DBGIO_LL_IRQ        P24
#define DBGIO_APP_WAKEUP    P18
#define DBGIO_APP_SLEEP     P20
#define DBGIO_DIS_IRQ       P11
#define DBGIO_EN_IRQ        P34

#define LL_HW_MODE_STX           0x00
#define LL_HW_MODE_SRX           0x01
#define LL_HW_MODE_TRX           0x02
#define LL_HW_MODE_RTX           0x03
#define LL_HW_MODE_TRLP          0x04
#define LL_HW_MODE_RTLP          0x05

//  =============== add in A2 for simultaneous slave and adv/scan
#define LL_SEC_STATE_IDLE             0x00
#define LL_SEC_STATE_SCAN             0x01
#define LL_SEC_STATE_ADV              0x02
#define LL_SEC_STATE_SCAN_PENDING     0x03
#define LL_SEC_STATE_ADV_PENDING      0x04
#define LL_SEC_STATE_IDLE_PENDING     0x05
#define LL_SEC_STATE_INIT             0x06
#define LL_SEC_STATE_INIT_PENDING     0x07

#define WFI()   __WFI()

#define LL_MODE_INVALID       0xFF
#define LL_MODE_LEGACY        0x00
#define LL_MODE_EXTENDED      0x01

#define LL_COPY_DEV_ADDR_LE( dstPtr, srcPtr )        {                          \
        (dstPtr)[0] = (srcPtr)[0];                                                   \
        (dstPtr)[1] = (srcPtr)[1];                                                   \
        (dstPtr)[2] = (srcPtr)[2];                                                   \
        (dstPtr)[3] = (srcPtr)[3];                                                   \
        (dstPtr)[4] = (srcPtr)[4];                                                   \
        (dstPtr)[5] = (srcPtr)[5];}

#define LL_WINDOW_SIZE                2         // 2.5ms in 1.25ms ticks

#define LL_CALC_NEXT_SCAN_CHN(chan)     { chan ++; \
        chan = (chan > LL_SCAN_ADV_CHAN_39) ? LL_SCAN_ADV_CHAN_37 : chan;}

#define   CONN_CSA2_ALLOW                  0x00000080

//------------------------------------------------------------------------------------
//extern rom function
//
//extern int gpio_write(gpio_pin_e pin, bit_action_e en);
extern uint8   ll_processExtAdvIRQ(uint32_t      irq_status);
extern uint8   ll_processPrdAdvIRQ(uint32_t      irq_status);
extern uint8   ll_processExtScanIRQ(uint32_t      irq_status);
extern uint8   ll_processExtInitIRQ(uint32_t      irq_status);
extern uint8   ll_processPrdScanIRQ(uint32_t      irq_status);
extern uint8   ll_processBasicIRQ(uint32_t      irq_status);
extern int clear_timer_int(AP_TIM_TypeDef* TIMx);
extern uint8 isTimer1Running(void);
extern uint8 isTimer4Running(void);
extern void clear_timer(AP_TIM_TypeDef* TIMx);

extern uint8 ll_processMissMasterEvt(uint8 connId);
extern uint8 ll_processMissSlaveEvt(uint8 connId);
extern int gpio_write(GPIO_Pin_e pin, uint8_t en);
extern void ll_hw_tx2rx_timing_config(uint8 pkt);
extern void wakeup_init0(void);
extern void enter_sleep_off_mode0(Sleep_Mode mode);
extern void spif_release_deep_sleep(void);
extern void spif_set_deep_sleep(void);

extern uint8 ll_hw_get_tr_mode(void);
extern int ll_hw_get_rfifo_depth(void);
extern void move_to_master_function(void);

extern struct buf_tx_desc g_tx_adv_buf;
extern struct buf_tx_desc g_tx_ext_adv_buf;
extern struct buf_tx_desc tx_scanRsp_desc;

extern struct buf_rx_desc g_rx_adv_buf;

extern chipMAddr_t g_chipMAddr;

extern uint8  g_llAdvMode;
extern uint32_t       g_llHdcDirAdvTime;
//-----------------------------------------------------------------------------------
//extern rom  variable
//
uint32* pGlobal_config = NULL;

const  unsigned char libRevisionDate[]=__DATE__;
const  unsigned char libRevisionTime[]=__TIME__;

uint8 CreateConn_Flag = FALSE;
uint32_t g_t_llhwgo = 0;

extern uint32 hclk_per_us;
extern uint32 hclk_per_us_shift;
extern volatile uint8 g_clk32K_config;
/////////////////////////

extern uint32 sleep_flag;
extern uint32 osal_sys_tick;
extern uint32 ll_remain_time;

extern uint32 llWaitingIrq;
extern uint32 ISR_entry_time;

extern uint32 counter_tracking;

extern uint32_t  __initial_sp;
extern uint32_t  g_smartWindowSize;
extern volatile uint8_t g_same_rf_channel_flag;
extern uint32_t  g_TIM2_IRQ_TIM3_CurrCount;
extern uint32_t  g_TIM2_IRQ_to_Sleep_DeltTick;
extern uint32_t  g_TIM2_IRQ_PendingTick;
extern uint32_t  g_osal_tick_trim;
extern uint32_t  g_osalTickTrim_mod;
extern uint32_t  g_TIM2_wakeup_delay;
extern uint32_t  rtc_mod_value;
extern uint32_t  g_counter_traking_cnt;
extern uint32_t  sleep_tick;
extern uint32_t  g_wakeup_rtc_tick;
extern int slave_conn_event_recv_delay;
extern uint8  g_llScanMode;
extern uint8 g_currentPeerAddrType;
extern uint8 g_currentPeerRpa[LL_DEVICE_ADDR_LEN];
extern uint8 ownRandomAddr[];
extern uint16_t ll_hw_get_tfifo_wrptr(void);
extern uint32_t llCurrentScanChn;
extern uint8 ownPublicAddr[];
extern uint32_t llScanTime;
extern uint32_t llScanT1;
extern uint8    isPeerRpaStore;
extern uint8    currentPeerRpa[LL_DEVICE_ADDR_LEN];
extern uint8    storeRpaListIndex;
extern uint8    g_currentLocalAddrType;
extern uint8    g_currentLocalRpa[LL_DEVICE_ADDR_LEN];
//extern llPduLenManagment_t g_llPduLen;
extern uint8_t             llSecondaryState;            // secondary state of LL

uint8 ll_processBasicIRQ_SRX(uint32_t            irq_status)
{
    uint8 ret=0;
    typedef uint8 (*my_function)(uint32_t );
    my_function pFunc = NULL;
    pFunc = (my_function)(JUMP_FUNCTION(LL_PROCESSBASICIRQ_SRX));

    if (pFunc != NULL)
        ret = pFunc(irq_status);
    else
        ret = ll_processBasicIRQ(irq_status);

    return ret;
}

uint8 ll_processBasicIRQ_secondaryAdvTRX(uint32_t                  irq_status)
{
    uint8 ret=0;
    typedef uint8 (*my_function)(uint32_t );
    my_function pFunc = NULL;
    pFunc = (my_function)(JUMP_FUNCTION(LL_PROCESSBASICIRQ_SECADVTRX));

    if (pFunc != NULL)
        ret = pFunc(irq_status);
    else
        ret = ll_processBasicIRQ(irq_status);

    return ret;
}

uint8 ll_processBasicIRQ_ScanTRX(uint32_t            irq_status)
{
    uint8 ret=0;
    typedef uint8 (*my_function)(uint32_t );
    my_function pFunc = NULL;
    pFunc = (my_function)(JUMP_FUNCTION(LL_PROCESSBASICIRQ_SCANTRX));

    if (pFunc != NULL)
        ret = pFunc(irq_status);
    else
        ret = ll_processBasicIRQ(irq_status);

    return ret;
}

//----------------------------------------------------------------------------------------------
//patch

void ll_hw_go1(void)
{
    //*(volatile uint32_t *)0x4000f0b8 = 0;  // pclk_clk_gate_en
    //20190115 ZQ recorded ll re-trigger
    //DBG_GPIO_WRITE(DBGIO_LL_TRIG,1);
    //DBG_GPIO_WRITE(DBGIO_LL_TRIG,0);
    if(llWaitingIrq==TRUE)
    {
        g_pmCounters.ll_trigger_err++;
        //llWaitingIrq = FALSE;
        //return;
    }

    g_t_llhwgo = read_current_fine_time();
    *(volatile uint32_t*)(LL_HW_BASE+ 0x14) = LL_HW_IRQ_MASK;   //clr  irq status
    *(volatile uint32_t*)(LL_HW_BASE+ 0x0c) = 0x0001;           //mask irq :only use mode done
    *(volatile uint32_t*)(LL_HW_BASE+ 0x00) = 0x0001;           //trig

    if(CreateConn_Flag)
    {
        osal_memcpy((uint8*)&g_tx_adv_buf.data[0], &initInfo.ownAddr[0], 6);
        CreateConn_Flag= FALSE;
    }

    //2018-05-23 ZQ
    //fix negative rfPhyFreqOff bug, when in scan_rsq case, ll_hw_go will be  excuted before set_channel()
    //so do not change the tx_rx_foff
    //next metal change could modified the set_channel() to deal with the tx_rx_foff
    uint8_t rfChnIdx = PHY_REG_RD(0x400300b4)&0xff;

    if(!g_same_rf_channel_flag)
    {
        if(g_rfPhyFreqOffSet>=0)
            PHY_REG_WT(0x400300b4, (g_rfPhyFreqOffSet<<16)+(g_rfPhyFreqOffSet<<8)+rfChnIdx);
        else
            PHY_REG_WT(0x400300b4, ((255+g_rfPhyFreqOffSet)<<16)+((255+g_rfPhyFreqOffSet)<<8)+(rfChnIdx-1) );
    }

    //2018-02-09 ZQ
    //considering the ll_trigger timing, Trigger first, then set the tp_cal cap

    if(rfChnIdx<2)
    {
        rfChnIdx=2;
    }
    else if(rfChnIdx>80)
    {
        rfChnIdx=80;
    }

    if(g_rfPhyPktFmt==PKT_FMT_BLE2M)
        subWriteReg(0x40030094,7,0,RF_PHY_TPCAL_CALC(g_rfPhyTpCal0_2Mbps,g_rfPhyTpCal1_2Mbps,(rfChnIdx-2)>>1));
    else
        subWriteReg(0x40030094,7,0,RF_PHY_TPCAL_CALC(g_rfPhyTpCal0,g_rfPhyTpCal1,(rfChnIdx-2)>>1));

    int llModeLast;
    llModeLast = ll_hw_get_tr_mode();
    extern uint8_t rxFifoFlowCtrl;
    extern uint8  ctrlToHostEnable;

    if (llModeLast == LL_HW_MODE_RTLP || llModeLast == LL_HW_MODE_TRLP)
    {
        if (ctrlToHostEnable && rxFifoFlowCtrl)
        {
            set_max_length(0);
        }

        //for codedphy rxtimeout
        llConnState_t* connPtr;
        connPtr = &conn_param[g_ll_conn_ctx.currentConn];

        if (connPtr->llRfPhyPktFmt == PKT_FMT_BLR125K || connPtr->llRfPhyPktFmt == PKT_FMT_BLR500K)
        {
            ll_hw_set_rx_timeout(350);
        }
    }

    if((llModeLast == LL_HW_MODE_TRX)&&((llState == LL_STATE_ADV_UNDIRECTED ||llState == LL_STATE_ADV_SCAN ||llState == LL_STATE_ADV_DIRECTED)|| llSecondaryState == LL_SEC_STATE_ADV))
    {
        ll_hw_set_rx_timeout(108);
    }

    // fix slave scan rsp addr type bug
    // if (llModeLast == LL_HW_MODE_STX  &&
    //         (llState == LL_STATE_ADV_UNDIRECTED ||
    //          llState == LL_STATE_ADV_SCAN       )
    //    )
    // {
    //     if(adv_param.ownAddrType == LL_DEV_ADDR_TYPE_PUBLIC)
    //     {
    //         SET_BITS(tx_scanRsp_desc.txheader, LL_DEV_ADDR_TYPE_PUBLIC, TX_ADD_SHIFT, TX_ADD_MASK);
    //     }
    //     else if(adv_param.ownAddrType == LL_DEV_ADDR_TYPE_RANDOM)
    //     {
    //         SET_BITS(tx_scanRsp_desc.txheader, LL_DEV_ADDR_TYPE_RANDOM, TX_ADD_SHIFT, TX_ADD_MASK);
    //     }
         DBG_GPIO_WRITE(DBGIO_LL_TRIG,1);
         DBG_GPIO_WRITE(DBGIO_LL_TRIG,0);
    // }
    //
    //disable scan backoff
    scanInfo.currentBackoff=1;
}

extern int slave_conn_event_recv_delay;
void ll_adptive_adj_next_time1(uint32_t next_time)
{
    (void)(next_time);
    uint32_t loop_time,anchor_point;

    // read loop timeout counter, system clock may be 16MHz, 32MHz, 64MHz and 48MHz, 96MHz
    if (hclk_per_us_shift != 0)
    {
        loop_time = ll_hw_get_loop_cycle() >> hclk_per_us_shift;      // convert to us
    }
    else
    {
        loop_time = ll_hw_get_loop_cycle() / hclk_per_us;             // convert to us
    }

    if (hclk_per_us_shift != 0)
    {
        anchor_point = ll_hw_get_anchor() >> hclk_per_us_shift;      // convert to us
    }
    else
    {
        anchor_point = ll_hw_get_anchor() / hclk_per_us;      // convert to us
    }

    //==================================================
    //DO NOT ADD LOG PRINTF In this FUNCTION
    //==================================================
    llConnState_t* connPtr;
    // get connection information
    connPtr = &conn_param[g_ll_conn_ctx.currentConn];

    //no anche point
    if (connPtr->rx_timeout)
    {
        connPtr->pmCounter.ll_tbd_cnt1++;
        slave_conn_event_recv_delay = LL_TIME_DELTA(g_t_llhwgo, ISR_entry_time)-370+160;//160:timer1 irq->hwgo trigger
    }
    else
    {
        connPtr->pmCounter.ll_tbd_cnt1 = 0;
//        slave_conn_event_recv_delay = loop_time - anchor_point+pGlobal_config[SLAVE_CONN_DELAY];
		slave_conn_event_recv_delay = LL_TIME_DELTA(g_t_llhwgo, ISR_entry_time)  - anchor_point + pGlobal_config[SLAVE_CONN_DELAY];

    }

    // slave_conn_event_recv_delay -= 370;
    //slave_conn_event_recv_delay += (connPtr->curParam.connInterval >> 2);
    // slave_conn_event_recv_delay += pGlobal_config[SLAVE_CONN_DELAY];

    // if( connPtr->firstPacket )
    // {
    //  slave_conn_event_recv_delay+=500;
    // }

    //only adj for the 1st rxtimeout
    if (1 == connPtr->pmCounter.ll_tbd_cnt1)
    {
        slave_conn_event_recv_delay += 500;
		slave_conn_event_recv_delay += 1500;
    }
	else if( connPtr->pmCounter.ll_tbd_cnt1 > 0 )
	{
		slave_conn_event_recv_delay += 1000;
    }

    //adj for ntrm pkt, each pkt cost 50us in wt tfifo
    //if(connPtr->rx_timeout)
    //slave_conn_event_recv_delay += ((connPtr->ll_buf.ntrm_cnt) * 50);
}

void llConnTerminate1( llConnState_t* connPtr,
                       uint8          reason )
{
    /*
        ZQ:20210622
        process chanmp update passed instant(core 4.2 should term link, since core 5.0 just update the )
        just update chanmap do not trigger ll conn termination
    */
    if(     reason == LL_CTRL_PKT_INSTANT_PASSED_PEER_TERM
            && ((uint16)(connPtr->chanMapUpdateEvent - connPtr->currentEvent) >= LL_MAX_UPDATE_COUNT_RANGE )
            &&((!osal_memcmp(connPtr->chanMap,connPtr->chanMapUpdate.chanMap,5))))
    {
        llProcessChanMap(connPtr, connPtr->chanMapUpdate.chanMap);
    }
    else
    {
        llConnTerminate0(connPtr,reason);
    }
}

/*
    fix secAdv evt rfphyPkt error issue
*/
//extern uint8 llSetupSecAdvEvt0( void );
uint8 llSetupSecAdvEvt1( void )
{
    uint8 ret = FALSE;

    if (llState == LL_STATE_IDLE)
    {
        if (adv_param.advEvtType == LL_ADV_CONNECTABLE_UNDIRECTED_EVT)
            llState = LL_STATE_ADV_UNDIRECTED;
        else if (adv_param.advEvtType == LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT)
            llState = LL_STATE_ADV_NONCONN;
        else if (adv_param.advEvtType == LL_ADV_SCANNABLE_UNDIRECTED_EVT)
            llState = LL_STATE_ADV_SCAN;

        llSetupAdv();
        llSecondaryState = LL_SEC_STATE_IDLE;
        return TRUE;
    }
    else
    {
        llConnState_t* connPtr;
        connPtr = &conn_param[g_ll_conn_ctx.currentConn];
        g_rfPhyPktFmt = LE_1M_PHY;
        //support rf phy change
        rf_phy_change_cfg0(g_rfPhyPktFmt);

        if (adv_param.advEvtType == LL_ADV_CONNECTABLE_UNDIRECTED_EVT)
            ret = llSetupSecConnectableAdvEvt();
        else if (adv_param.advEvtType == LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT)
            ret = llSetupSecNonConnectableAdvEvt();
        else if (adv_param.advEvtType == LL_ADV_SCANNABLE_UNDIRECTED_EVT)
            ret = llSetupSecScannableAdvEvt();
        else
            return FALSE;          // other type adv should not here

        g_rfPhyPktFmt = connPtr->llRfPhyPktFmt;
    }

    return ret;
}

//fix sec_scan rfphy issue
void llSetupSecScan1( uint8 chan )
{
    uint32 scanTime;
    // Hold off interrupts.
    _HAL_CS_ALLOC_();
    HAL_ENTER_CRITICAL_SECTION();
    scanTime = scanInfo.scanWindow * 625;

//  if(llWaitingIrq)
//  {
//      LOG("==== error, mode: %d\n", scanInfo.scanMode);
//  }

    if (llState == LL_STATE_IDLE)
    {
        llState = LL_STATE_SCAN;
        llSecondaryState = LL_SEC_STATE_IDLE;
    }
    else
    {
        // calculate scan time
        scanTime = llCalcMaxScanTime();

        if (scanTime)       // trigger scan
        {
            llSecondaryState = LL_SEC_STATE_SCAN;
        }
        else                // no enough time to scan, pending
        {
            llSecondaryState = LL_SEC_STATE_SCAN_PENDING;
            g_pmCounters.ll_conn_scan_pending_cnt ++;
            HAL_EXIT_CRITICAL_SECTION(  );
            return;
        }
    }

    if (scanTime > scanInfo.scanWindow * 625)
        scanTime = scanInfo.scanWindow * 625;

    llConnState_t* connPtr;
    connPtr = &conn_param[g_ll_conn_ctx.currentConn];
    g_rfPhyPktFmt = LE_1M_PHY;
    //support rf phy change
    rf_phy_change_cfg0(g_rfPhyPktFmt);
    // reset all FIFOs; all data is forfeit
    ll_hw_rst_tfifo();
    ll_hw_rst_rfifo();
    set_crc_seed(ADV_CRC_INIT_VALUE); // crc seed for adv is same for all channels
    set_access_address(ADV_SYNCH_WORD);
    set_channel(chan);
    set_whiten_seed(chan);
    set_max_length(0xff);
    ll_hw_set_rx_timeout(scanTime);   // maximum scan time, note that actual scan time may exceed the limit if timer expiry when LL engine receiving a report
    ll_hw_set_srx();
    ll_hw_ign_rfifo(LL_HW_IGN_CRC|LL_HW_IGN_EMP);
    ll_hw_go();
    llScanT1 = read_current_fine_time();
    g_rfPhyPktFmt = connPtr->llRfPhyPktFmt;
    llWaitingIrq = TRUE;
    HAL_EXIT_CRITICAL_SECTION();
//    uint32 remainTime = read_LL_remainder_time();
//  LOG("<%d %d>", scanTime, remainTime);
    return;
}

extern int32   connUpdateTimer;
/*******************************************************************************
    GLOBAL VARIABLES
*/

extern perStatsByChan_t* p_perStatsByChan;
extern  uint8    g_conn_taskID;
extern  uint16   g_conn_taskEvent;


extern volatile sysclk_t g_system_clk;

uint32_t  sysclk_get_clk(void)
{
  switch(g_system_clk){
  case SYS_CLK_RC_32M:
  case SYS_CLK_DLL_32M:
    return 32000000;
  case SYS_CLK_XTAL_16M:
    return 16000000;
  case SYS_CLK_DLL_48M:
    return 48000000;
  case SYS_CLK_DLL_64M:
    return 64000000;
  default:
    break;

  }
  return 16000000;
}


/*******************************************************************************
    Prototypes
*/
extern uint8 llProcessMasterControlProcedures( llConnState_t* connPtr );
extern uint8 llSetupNextMasterEvent( void );
/*******************************************************************************
    @fn          llMasterEvt_TaskEndOk

    @brief       This function is used to handle the PHY task done end cause
                TASK_ENDOK that can result from one of three causes. First, a
                a packet was successfully received with MD=0 (i.e. no more Slave
                data) after having transmitted a packet with MD=0. Second, a
                received packet did not fit in the RX FIFO after transmitting
                a packet with MD=0. Third, a packet was received from the Slave
                while BLE_L_CONF.ENDC is true or after Timer 2 Event 2 occurs.

                Note: The TASK_ENDOK end cause will also handle the TASK_NOSYNC,
                      TASK_RXERR, and TASK_MAXNACK end causes as well.

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      None.
*/
void llMasterEvt_TaskEndOk1( void )
{
    llConnState_t* connPtr;
    uint16         numPkts;
    int        i;
    uint32_t   T2, schedule_time;
    // get connection information
    connPtr = &conn_param[g_ll_conn_ctx.currentConn];
    // advance the connection event count
    connPtr->currentEvent = connPtr->nextEvent;
    // get the total number of received packets
    // Note: Since Auto-Flush is enabled, numRxFifoFull is incremented instead of
    //       numRxOk when there's no room in the FIFO. When Auto-Flush is
    //       disabled and there's no room in the FIFO, only numRxFifoFull is
    //       incremented for any kind of received packet.
    numPkts = ( rfCounters.numRxOk       +
                rfCounters.numRxNotOk    +
                rfCounters.numRxEmpty    +
                rfCounters.numRxIgnored  +
                rfCounters.numRxFifoFull );
    // collect packet error information
    connPtr->perInfo.numPkts   += numPkts;
    connPtr->perInfo.numCrcErr += rfCounters.numRxNotOk;
    //
    connPtr->perInfo.numEvents++;

//  // check if PER by Channel is enabled
//  if ( connPtr->perInfoByChan != NULL )
//  {
//     connPtr->perInfoByChan->numPkts[ PHY_GET_DATA_CHAN() ]   += numPkts;
//     connPtr->perInfoByChan->numCrcErr[ PHY_GET_DATA_CHAN() ] += rfCounters.numRxNotOk;
//  }

    // check if any data has been received
    // Note: numRxOk includes numRxCtrl
    // Note: numRxNotOk removed as 4.5.2 of spec says the timer is reset upon
    //       receipt of a "valid packet", which is taken to mean no CRC error.
    if ( rfCounters.numRxOk    || rfCounters.numRxIgnored ||
            rfCounters.numRxEmpty || rfCounters.numRxFifoFull
            || connPtr->rx_crcok != 0)     // ever Rx CRC OK packet
    {
        // yes, so update the supervision expiration count
        connPtr->expirationEvent = connPtr->currentEvent + connPtr->expirationValue;
        // clear flag that indicates we received first packet
        // Note: The first packet only really needs to be signalled when a new
        //       connection is formed. However, there's no harm in resetting it
        //       every time in order to simplify the control logic.
        // Note: True-Low logic is used here to be consistent with nR's language.
        connPtr->firstPacket = 0;

        //20181206 ZQ add phy change nofity
        //receiver ack notifty the host
        if(connPtr->llPhyModeCtrl.isChanged==TRUE)
        {
            connPtr->llPhyModeCtrl.isChanged = FALSE;
            llPhyModeCtrlUpdateNotify(connPtr,LL_STATUS_SUCCESS);
        }
    }
    else // no data received, or packet received with CRC error
    {
        // check if we received any packets with a CRC error
        if ( rfCounters.numRxNotOk )
        {
            // clear flag that indicates we received first packet
            // Note: The first packet only really needs to be signalled when a new
            //       connection is formed. However, there's no harm in resetting it
            //       every time in order to simplify the control logic.
            // Note: True-Low logic is used here to be consistent with nR's language.
            connPtr->firstPacket = 0;
        }
        else // no packet was received
        {
            // collect packet error information, TI use HCI ext to get this information. No used by PHY+ now
            connPtr->perInfo.numMissedEvts++;
        }

        // check if we have a Supervision Timeout
        if ( connPtr->expirationEvent == connPtr->currentEvent )     // 20201011�� should be "=="
        {
            // check if the connection has already been established
            if ( connPtr->firstPacket == 0 )
            {
                // yes, so terminate with LSTO
                llConnTerminate( connPtr, LL_SUPERVISION_TIMEOUT_TERM );
            }
            else // no, so this is a failure to establish the connection
            {
                // so terminate immediately with failure to establish connection
                llConnTerminate( connPtr, LL_CONN_ESTABLISHMENT_FAILED_TERM );
            }

//#ifdef MULTI_ROLE
            ll_scheduler(LL_INVALID_TIME);           // link is terminated, update scheduler info
//#endif
            return;
        }
    }

    /*
    ** Process RX Data Packets
    */
    // check if there is any data in the Rx FIFO
    uint8_t  buffer_size;
    buffer_size = getRxBufferSize(connPtr);

    for ( i = 0; i < buffer_size; i ++)     // note: i < getRxBufferSize()  will fail the loop
    {
        // there is, so process it; check if data was processed
        if ( llProcessRxData() == FALSE )
        {
            // it wasn't, so we're done
//        ll_scheduler(LL_INVALID_TIME);
            break;
        }
    }

    // check if this connection was terminated
    if ( !connPtr->active )
    {
//#ifdef MULTI_ROLE
        ll_scheduler(LL_INVALID_TIME);
//#endif
        return;
    }

    /*
    ** Check Control Procedure Processing
    */
    if ( llProcessMasterControlProcedures( connPtr ) == LL_CTRL_PROC_STATUS_TERMINATE )
    {
//#ifdef MULTI_ROLE
        ll_scheduler(LL_INVALID_TIME);           // link is termainte, update schedle info
//#endif
        return;
    }
    else if(connPtr->ctrlDataIsPending  == 1)
    {
        uint8 pktLenctrl;
        uint8* pBufctrl = connPtr->ctrlData.data;
        pktLenctrl = LL_REJECT_EXT_IND_PAYLOAD_LEN;

        if((connPtr->ctrlData .header == (pktLenctrl << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT))&&(*pBufctrl == LL_CTRL_REJECT_EXT_IND))
        {
            uint8 ctrlerrorcode = *(pBufctrl + 1);
            *(pBufctrl + 1) = connPtr->rejectOpCode;
            *(pBufctrl + 2) = ctrlerrorcode;
        }
    }

    /*
    ** Process TX Data Packets
    */
    // copy any pending data to the TX FIFO
    llProcessTxData( connPtr, LL_TX_DATA_CONTEXT_POST_PROCESSING );

    // if any fragment l2cap pkt, copy to TX FIFO
    //l2capPocessFragmentTxData((uint16)connPtr->connId);

    /*
    ** Setup Next Slave Event Timing
    */

    // update next event, calculate time to next event, calculate timer drift,
    // update anchor points, setup NR T2E1 and T2E2 events
    if ( llSetupNextMasterEvent() == LL_SETUP_NEXT_LINK_STATUS_TERMINATE )            // PHY+ always return success here
    {
        // this connection is terminated, so nothing to schedule
//#ifdef MULTI_ROLE
        ll_scheduler(LL_INVALID_TIME);
//#endif
        return;
    }

    /*
    ** Schedule Next Task
    */
//#ifdef MULTI_ROLE
//  schedule_time = ll_get_next_timer(g_ll_conn_ctx.currentConn);
    schedule_time = (connPtr->curParam.connInterval + connUpdateTimer) * 625;
    T2 = read_current_fine_time();
    // TODO: don't know the cause, here need add 32us to gain accurate timing
    //2020.11.11,Jie,master conInterval-5us
    ll_scheduler(schedule_time - 10 - LL_TIME_DELTA(g_ll_conn_ctx.timerExpiryTick, T2) );    // 10us: rough delay from timer expire to timer ISR
//#endif
    return;
}

uint8_t  ll_hw_read_rfifo1(uint8_t* rxPkt, uint16_t* pktLen, uint32_t* pktFoot0, uint32_t* pktFoot1)
{
    int       rdPtr, wrPtr, rdDepth, blen, wlen;
    uint32_t* p_rxPkt = (uint32_t*)rxPkt;
    ll_hw_get_rfifo_info(&rdPtr, &wrPtr, &rdDepth);

    if(rdDepth > 0)
    {
        *p_rxPkt++ = *(volatile uint32_t*)(LL_HW_RFIFO);
        uint8_t sp =0;//BLE_HEAD_WITH_CTE(rxPkt[0]);
        blen    = rxPkt[1]+sp;                      //get the byte length for header
        wlen    = 1+ ( (blen+2+3-1) >>2 );      //+2 for Header, +3 for crc

        //compared the wlen and HW_WTR
        //20190115 ZQ
        if( (wlen+2) >rdDepth)
        {
            g_pmCounters.ll_rfifo_read_err++;
            rxPkt[0]  = 0;
            *pktFoot0 = 0;
            *pktFoot1 = 0;
            *pktLen   = 0;
            return 0;
        }

        while(p_rxPkt < (uint32_t*)rxPkt + wlen)
        {
            *p_rxPkt++ = *(volatile uint32_t*)(LL_HW_RFIFO);
        }

        *pktFoot0   = *(volatile uint32_t*)(LL_HW_RFIFO);
        *pktFoot1   = *(volatile uint32_t*)(LL_HW_RFIFO);
        *pktLen     = blen + 2;
        return wlen;
    }
    else
    {
        rxPkt[0]  = 0;
        *pktFoot0 = 0;
        *pktFoot1 = 0;
        *pktLen   = 0;
        return 0;
    }
}

/*******************************************************************************
    @fn          ll_processBasicIRQ_SRX

    @brief      Interrupt Request Handler for Link Layer

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      None
*/
uint8 ll_processBasicIRQ_SRX0(uint32_t      irq_status)
{
    uint8         mode;
    uint32_t      T2, delay;
    llConnState_t* connPtr;
    connPtr = &conn_param[0];        // To update
    _HAL_CS_ALLOC_();
    HAL_ENTER_CRITICAL_SECTION();
    mode = ll_hw_get_tr_mode();

    if (mode == LL_HW_MODE_SRX
            && (llState == LL_STATE_SCAN || llState == LL_STATE_INIT))
    {
        ll_debug_output(DEBUG_LL_HW_SRX);
        uint8_t  rpaListIndex = LL_RESOLVINGLIST_ENTRY_NUM;
        uint8_t  bWlRlCheckOk = TRUE;
        uint8_t*  peerAddr;

        // ============= scan case
        if (llState == LL_STATE_SCAN)
        {
            uint8   bSendingScanReq = FALSE;

            // check status
            if ((irq_status & LIRQ_RD) && (irq_status & LIRQ_COK))       // bug correct 2018-10-15
            {
                // rx done
                uint8_t  packet_len, pdu_type;
                uint16_t pktLen;
                uint32_t pktFoot0, pktFoot1;
                // read packet
                // cost 21-26us(measure with GPIO), depneds on the length of ADV
                packet_len = ll_hw_read_rfifo1((uint8_t*)(&(g_rx_adv_buf.rxheader)),
                                               &pktLen,
                                               &pktFoot0,
                                               &pktFoot1);
                // check receive pdu type
                pdu_type = g_rx_adv_buf.rxheader & 0x0f;

                if(ll_hw_get_rfifo_depth()>0)
                {
                    g_pmCounters.ll_rfifo_read_err++;
                    packet_len=0;
                    pktLen=0;
                }

                if (packet_len   != 0
                        && ((pdu_type == ADV_IND)
                            || (pdu_type  == ADV_NONCONN_IND)
                            || (pdu_type  == ADV_SCAN_IND)
                            || (pdu_type == ADV_DIRECT_IND)))
                {
                    uint8   addrType;                  // peer address type
                    uint8_t txAdd = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random
                    peerAddr = &g_rx_adv_buf.data[0];        // AdvA
                    addrType = txAdd;

                    // Resolving list checking
                    // case 1: receive ScanA using RPA
                    if (txAdd == LL_DEV_ADDR_TYPE_RANDOM  &&
                            (g_rx_adv_buf.data[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR)
                    {
                        bWlRlCheckOk = TRUE;

                        if (g_llRlEnable == TRUE)
                        {
                            rpaListIndex = ll_getRPAListEntry(&g_rx_adv_buf.data[0]);

                            if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM)
                            {
                                peerAddr = &g_llResolvinglist[rpaListIndex].peerAddr[0];
                                // refer to HCI LE Advertising Report Event, RPA address type should be
                                // 0x02: Public Identity Address (Corresponds to Resolved Private Address)
                                // 0x03: Random (static) Identity Address (Corresponds to Resolved Private Address)
                                addrType = g_llResolvinglist[rpaListIndex].peerAddrType + 2;
                                bWlRlCheckOk = TRUE;
                            }
                            else
                            {
                                bWlRlCheckOk = FALSE;
                            }
                        }
                    }
                    else     // case 2: receive ScanA using device ID, or scan device not using RPA
                    {
                        bWlRlCheckOk = TRUE;

                        for (int i = 0; i < LL_RESOLVINGLIST_ENTRY_NUM; i++)
                        {
                            if ( g_llResolvinglist[i].peerAddr[0] == g_rx_adv_buf.data[0]
                                    && g_llResolvinglist[i].peerAddr[1] == g_rx_adv_buf.data[1]
                                    && g_llResolvinglist[i].peerAddr[2] == g_rx_adv_buf.data[2]
                                    && g_llResolvinglist[i].peerAddr[3] == g_rx_adv_buf.data[3]
                                    && g_llResolvinglist[i].peerAddr[4] == g_rx_adv_buf.data[4]
                                    && g_llResolvinglist[i].peerAddr[5] == g_rx_adv_buf.data[5])
                            {
                                // the device ID in the RPA list
                                if (g_llResolvinglist[i].privacyMode == DEVICE_PRIVACY_MODE ||
                                        ll_isIrkAllZero(g_llResolvinglist[i].peerIrk))
                                    rpaListIndex = i;
                                else
                                    bWlRlCheckOk = FALSE;      // the device in the RPA list but not using RPA, reject it

                                break;
                            }
                        }
                    }

                    // check white list
                    if ((pGlobal_config[LL_SWITCH] & LL_WHITELIST_ALLOW)
                            && (scanInfo.wlPolicy  == LL_SCAN_WL_POLICY_USE_WHITE_LIST)
                            && (bWlRlCheckOk == TRUE))
                    {
                        // check white list
                        bWlRlCheckOk = ll_isAddrInWhiteList(txAdd, peerAddr);
                    }

                    /*  20201218 Jie,direct adv report when no whitelist filter
                        else if(pdu_type == ADV_DIRECT_IND)    // direct adv only report addr & addr type match the whitelist
                        bWlRlCheckOk = FALSE;
                    */
                    // if valid, trigger osal event to report adv
                    if (bWlRlCheckOk == TRUE)
                    {
                        uint8  advEventType;
                        int8   rssi;
                        llCurrentScanChn = scanInfo.nextScanChan;

                        // active scan scenario, send scan req
                        if (scanInfo.scanType == LL_SCAN_ACTIVE
                                && (pdu_type== ADV_IND
                                    || pdu_type == ADV_SCAN_IND ))
                        {
                            // back off process
                            scanInfo.currentBackoff = (scanInfo.currentBackoff > 0) ? (scanInfo.currentBackoff - 1) : 0;

                            if (scanInfo.currentBackoff == 0)      // back off value = 0, send scan req
                            {
                                g_tx_adv_buf.txheader = 0xC03;
                                //ZQ 20181012: add AdvFilterCB
                                uint8_t retAdvFilter = 1;

                                if(LL_PLUS_AdvDataFilterCBack)
                                {
                                    //!!!CATION!!!
                                    //timing critical
                                    //txbuf will be changed
                                    retAdvFilter = LL_PLUS_AdvDataFilterCBack();
                                }

                                if(retAdvFilter)
                                {
                                    g_same_rf_channel_flag = TRUE;
                                    ll_hw_set_tx_rx_interval(10);
                                    ll_hw_set_rx_timeout(158);
                                    set_max_length(0xFF);                    // add 2020-03-10
                                    T2 = read_current_fine_time();
                                    delay = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);
                                    delay = 118 - delay - pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY];
                                    ll_hw_set_trx();             // set LL HW as single TRx mode
                                    ll_hw_set_trx_settle(delay,                               // set BB delay, about 80us in 16MHz HCLK
                                                         pGlobal_config[LL_HW_AFE_DELAY],
                                                         pGlobal_config[LL_HW_PLL_DELAY]);        //RxAFE,PLL
                                    ll_hw_go();
                                    g_pmCounters.ll_send_scan_req_cnt++;
                                    llWaitingIrq = TRUE;
                                    // reset Rx/Tx FIFO
                                    ll_hw_rst_rfifo();
                                    ll_hw_rst_tfifo();
                                    ll_hw_ign_rfifo(LL_HW_IGN_CRC | LL_HW_IGN_EMP);

                                    // construct SCAN REQ packet
                                    //g_tx_adv_buf.txheader = 0xCC3;

//                                    //20181012 ZQ: change the txheader according to the adtype
//                                    g_tx_adv_buf.txheader |=(((g_rx_adv_buf.rxheader&0x40)<<1)
//                                                             | (scanInfo.ownAddrType<< TX_ADD_SHIFT & TX_ADD_MASK));

                                    // fill scanA, using RPA or device ID address   // TODO: move below code before ll_hw_go?
                                    if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM &&
                                            !ll_isIrkAllZero(g_llResolvinglist[rpaListIndex].localIrk)
                                            && (scanInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_PUBLIC
                                                || scanInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM))
                                    {
                                        // for resolving private address case, calculate the scanA with Local IRK
                                        ll_CalcRandomAddr(g_llResolvinglist[rpaListIndex].localIrk, &g_tx_adv_buf.data[0]);
                                        SET_BITS(g_tx_adv_buf.txheader, LL_DEV_ADDR_TYPE_RANDOM, TX_ADD_SHIFT, TX_ADD_MASK);
                                    }
                                    else
                                    {
                                        //2020.10.26 Jie,TX_ADD update
                                        if (scanInfo.ownAddrType == LL_DEV_ADDR_TYPE_PUBLIC || scanInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_PUBLIC)
                                        {
                                            osal_memcpy((uint8*)&g_tx_adv_buf.data[0], &ownPublicAddr[0], 6);
                                            SET_BITS(g_tx_adv_buf.txheader, LL_DEV_ADDR_TYPE_PUBLIC, TX_ADD_SHIFT, TX_ADD_MASK);
                                        }
                                        else
                                        {
                                            osal_memcpy((uint8*)&g_tx_adv_buf.data[0], &ownRandomAddr[0], 6);
                                            SET_BITS(g_tx_adv_buf.txheader, LL_DEV_ADDR_TYPE_RANDOM, TX_ADD_SHIFT, TX_ADD_MASK);
                                        }
                                    }

                                    g_tx_adv_buf.txheader |= (txAdd << RX_ADD_SHIFT & RX_ADD_MASK);
                                    // AdvA, for SCAN REQ, it should identical to the ADV_IND/ADV_SCAN_IND
                                    g_tx_adv_buf.data[6]  = g_rx_adv_buf.data[0];
                                    g_tx_adv_buf.data[7]  = g_rx_adv_buf.data[1];
                                    g_tx_adv_buf.data[8]  = g_rx_adv_buf.data[2];
                                    g_tx_adv_buf.data[9]  = g_rx_adv_buf.data[3];
                                    g_tx_adv_buf.data[10] = g_rx_adv_buf.data[4];
                                    g_tx_adv_buf.data[11] = g_rx_adv_buf.data[5];
                                    //write Tx FIFO
                                    ll_hw_write_tfifo((uint8*)&(g_tx_adv_buf.txheader),
                                                      ((g_tx_adv_buf.txheader & 0xff00) >> 8) + 2);   // payload length + header length(2)
                                    bSendingScanReq = TRUE;
                                    g_same_rf_channel_flag = FALSE;
                                }
                            }
                        }

                        // convert pdu type to GAP enum
                        switch (pdu_type)
                        {
                        case ADV_IND:
                            advEventType = LL_ADV_RPT_ADV_IND;
                            break;

                        case ADV_SCAN_IND:
                            advEventType = LL_ADV_RPT_ADV_SCANNABLE_IND;
                            break;

                        case ADV_DIRECT_IND:
                            advEventType = LL_ADV_RPT_ADV_DIRECT_IND;
                            break;

                        case ADV_NONCONN_IND:
                            advEventType = LL_ADV_RPT_ADV_NONCONN_IND;
                            break;

                        case ADV_SCAN_RSP:
                            advEventType = LL_ADV_RPT_INVALID;
                            break;

                        default:
                            advEventType = LL_ADV_RPT_ADV_IND;
                            break;
                        }

                        rssi  =  -(pktFoot1 >> 24);
                        // below function cost 51us/66us(measure with GPIO)
                        LL_AdvReportCback( advEventType,                         // event type
                                           addrType,                                // Adv address type (TxAdd)
                                           &peerAddr[0],                         // Adv address (AdvA)
                                           pktLen - 8,                           // length of rest of the payload, 2 - header, 6 - advA
                                           &g_rx_adv_buf.data[6],                // rest of payload
                                           rssi );                               // RSSI
                        g_pmCounters.ll_recv_adv_pkt_cnt ++;
                    }
                }
                else
                {
                    // invalid ADV PDU type
//                    llSetupScan();
                }
            }

            // if not waiting for scan rsp, schedule next scan
            if (!bSendingScanReq)
            {
                // not sending SCAN REQ, update scan time
                llScanTime += ((ISR_entry_time > llScanT1) ? (ISR_entry_time - llScanT1) : (BASE_TIME_UNITS - llScanT1 + ISR_entry_time));

                if (llScanTime >= scanInfo.scanWindow * 625)
                {
                    // calculate next scan channel
                    LL_CALC_NEXT_SCAN_CHN(scanInfo.nextScanChan);

                    // schedule next scan event
                    if (scanInfo.scanWindow == scanInfo.scanInterval)      // scanWindow == scanInterval, trigger immediately
                        LL_evt_schedule();
                    else
//                        set_timer4((scanInfo.scanInterval - scanInfo.scanWindow) * 625);
                        ll_schedule_next_event((scanInfo.scanInterval - scanInfo.scanWindow) * 625);

                    // reset scan total time
                    llScanTime = 0;
                }
                else
                {
//                    AT_LOG("%03x %x %d %d %d %d\n",irq_status,*(volatile uint32_t *)(0x40031054),ll_hw_get_anchor(),
//                                                        g_rfifo_rst_cnt,(uint32_t)ISR_entry_time,read_current_fine_time());
                    llSetupScan(scanInfo.nextScanChan);
                }
            }
        }
        // ===========  initiator case
        else if (llState == LL_STATE_INIT)
        {
            uint8 bConnecting = FALSE;
            uint8 bMatchAdv = FALSE;     // RPA checking OK in previous adv event, and new adv event identical to the old one
            connPtr = &conn_param[initInfo.connId];           // connId is allocated when create conn

            // check status
            if ((irq_status & LIRQ_RD) && (irq_status & LIRQ_COK))       // bug correct 2018-10-15
            {
                // rx done
                uint8_t packet_len, pdu_type;
                uint16_t pktLen;
                uint32_t pktFoot0, pktFoot1;
                // read packet
                // cost 21-26us(measure with GPIO), depneds on the length of ADV
                packet_len = ll_hw_read_rfifo((uint8_t*)(&(g_rx_adv_buf.rxheader)),
                                              &pktLen,
                                              &pktFoot0,
                                              &pktFoot1);
                // check receive pdu type
                pdu_type = g_rx_adv_buf.rxheader & 0x0f;

                if(ll_hw_get_rfifo_depth() > 0)
                {
                    g_pmCounters.ll_rfifo_read_err++;
                    packet_len=0;
                    pktLen=0;
                }

                if (packet_len    != 0
                        && ((pdu_type == ADV_IND) || pdu_type == ADV_DIRECT_IND))
                {
                    uint8_t txAdd = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random
                    uint8_t chSel = (g_rx_adv_buf.rxheader & CHSEL_MASK) >> CHSEL_SHIFT;
                    rpaListIndex = LL_RESOLVINGLIST_ENTRY_NUM;
                    peerAddr = &g_rx_adv_buf.data[0];        // AdvA
                    g_currentPeerAddrType = txAdd;

                    // ================= Resolving list checking
                    // case 1: receive InitA using RPA
                    if (txAdd == LL_DEV_ADDR_TYPE_RANDOM  &&
                            ((g_rx_adv_buf.data[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR))
                    {
                        bWlRlCheckOk = TRUE;

                        if (g_llRlEnable == TRUE)
                        {
                            // if the RPA checking is done in previous scan, compare
                            if (isPeerRpaStore  == TRUE  &&
                                    currentPeerRpa[0] == g_rx_adv_buf.data[0]
                                    && currentPeerRpa[1] == g_rx_adv_buf.data[1]
                                    && currentPeerRpa[2] == g_rx_adv_buf.data[2]
                                    && currentPeerRpa[3] == g_rx_adv_buf.data[3]
                                    && currentPeerRpa[4] == g_rx_adv_buf.data[4]
                                    && currentPeerRpa[5] == g_rx_adv_buf.data[5])
                            {
                                rpaListIndex = storeRpaListIndex;
                                peerAddr = &g_llResolvinglist[rpaListIndex].peerAddr[0];
                                g_currentPeerAddrType = g_llResolvinglist[rpaListIndex].peerAddrType + 2;
                                bWlRlCheckOk = TRUE;
                                bMatchAdv = TRUE;
                            }
                            else   // resolve the address
                            {
                                rpaListIndex = ll_getRPAListEntry(&g_rx_adv_buf.data[0]);    // spend 30us(48MHz) when the 1st item match

                                if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM)
                                {
                                    peerAddr = &g_llResolvinglist[rpaListIndex].peerAddr[0];
                                    g_currentPeerAddrType = g_llResolvinglist[rpaListIndex].peerAddrType + 2;
                                    bWlRlCheckOk = TRUE;
                                }
                                else
                                {
                                    bWlRlCheckOk = FALSE;
                                }
                            }
                        }
                    }
                    // case 2: receive InitA using device ID, or init device not using RPA
                    else
                    {
                        for (int i = 0; i < LL_RESOLVINGLIST_ENTRY_NUM; i++)
                        {
                            if ( g_llResolvinglist[i].peerAddr[0] == g_rx_adv_buf.data[0]
                                    && g_llResolvinglist[i].peerAddr[1] == g_rx_adv_buf.data[1]
                                    && g_llResolvinglist[i].peerAddr[2] == g_rx_adv_buf.data[2]
                                    && g_llResolvinglist[i].peerAddr[3] == g_rx_adv_buf.data[3]
                                    && g_llResolvinglist[i].peerAddr[4] == g_rx_adv_buf.data[4]
                                    && g_llResolvinglist[i].peerAddr[5] == g_rx_adv_buf.data[5])
                            {
                                // the device ID in the RPA list
                                if (g_llResolvinglist[i].privacyMode == NETWORK_PRIVACY_MODE &&
                                        !ll_isIrkAllZero(g_llResolvinglist[i].peerIrk))
                                    bWlRlCheckOk = FALSE;
                                else
                                    rpaListIndex = i;
                            }
                        }
                    }

                    // ====== for direct adv, also check initA == own addr
                    if (pdu_type == ADV_DIRECT_IND && bWlRlCheckOk == TRUE && bMatchAdv != TRUE)
                    {
                        //20201228,Jie,add RXADD check for direct IND
                        uint8_t rxAdd = (g_rx_adv_buf.rxheader & RX_ADD_MASK) >> RX_ADD_SHIFT;

                        // initA is resolvable address case
                        if (rxAdd == LL_DEV_ADDR_TYPE_RANDOM  &&((g_rx_adv_buf.data[11] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR))
                        {
                            // should not use RPA case
                            if (initInfo.ownAddrType != LL_DEV_ADDR_TYPE_RPA_PUBLIC && initInfo.ownAddrType != LL_DEV_ADDR_TYPE_RPA_RANDOM)
                                bWlRlCheckOk = FALSE;

                            if (rpaListIndex >= LL_RESOLVINGLIST_ENTRY_NUM
                                    || (ll_isIrkAllZero(g_llResolvinglist[rpaListIndex].localIrk))    // all-0 local IRK
                                    || (ll_ResolveRandomAddrs(g_llResolvinglist[rpaListIndex].localIrk, &g_rx_adv_buf.data[6]) != SUCCESS))   // resolve failed
                                bWlRlCheckOk = FALSE;
                        }
                        else
                        {
                            uint8* localAddr;

                            // should not use device ID case
                            if ((initInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_PUBLIC || initInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM )
                                    && (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM
                                        && !ll_isIrkAllZero(g_llResolvinglist[rpaListIndex].localIrk)))
                            {
                                bWlRlCheckOk = FALSE;
                            }

                            if (rxAdd == LL_DEV_ADDR_TYPE_RANDOM)
                                localAddr = ownRandomAddr;
                            else
                                localAddr = ownPublicAddr;

                            if (g_rx_adv_buf.data[6]  != localAddr[0]
                                    || g_rx_adv_buf.data[7]  != localAddr[1]
                                    || g_rx_adv_buf.data[8]  != localAddr[2]
                                    || g_rx_adv_buf.data[9]  != localAddr[3]
                                    || g_rx_adv_buf.data[10] != localAddr[4]
                                    || g_rx_adv_buf.data[11] != localAddr[5])
                            {
                                bWlRlCheckOk = FALSE;
                            }
                        }
                    }

                    // initiator, 2 types of filter process: 1. connect to peer address set by host   2. connect to  address in whitelist only
                    // 1. connect to peer address set by host
                    if (initInfo.wlPolicy == LL_INIT_WL_POLICY_USE_PEER_ADDR
                            && bWlRlCheckOk == TRUE)
                    {
                        if (peerAddr[0]  != peerInfo.peerAddr[0]
                                || peerAddr[1]  != peerInfo.peerAddr[1]
                                || peerAddr[2]  != peerInfo.peerAddr[2]
                                || peerAddr[3]  != peerInfo.peerAddr[3]
                                || peerAddr[4]  != peerInfo.peerAddr[4]
                                || peerAddr[5]  != peerInfo.peerAddr[5])
                        {
                            // not match, not init connect
                            bWlRlCheckOk = FALSE;
                        }
                    }
                    // 2. connect to  address in whitelist only
                    else if (initInfo.wlPolicy == LL_INIT_WL_POLICY_USE_WHITE_LIST &&
                             bWlRlCheckOk == TRUE)
                    {
                        // if advA in whitelist list, connect
                        // check white list
                        bWlRlCheckOk = ll_isAddrInWhiteList(txAdd, peerAddr);

                        //2020.10.26,Jie,update peer addr
                        if (bWlRlCheckOk == TRUE)
                        {
                            peerInfo.peerAddrType = txAdd;
                            peerInfo.peerAddr[0] = peerAddr[0];
                            peerInfo.peerAddr[1] = peerAddr[1];
                            peerInfo.peerAddr[2] = peerAddr[2];
                            peerInfo.peerAddr[3] = peerAddr[3];
                            peerInfo.peerAddr[4] = peerAddr[4];
                            peerInfo.peerAddr[5] = peerAddr[5];
                        }
                    }

                    if (bWlRlCheckOk == TRUE)
                    {
                        g_same_rf_channel_flag = TRUE;

                        // channel selection algorithm decision
                        if ((pGlobal_config[LL_SWITCH] & CONN_CSA2_ALLOW)
                                && chSel == LL_CHN_SEL_ALGORITHM_2)
                        {
                            conn_param[initInfo.connId].channel_selection = LL_CHN_SEL_ALGORITHM_2;
                            SET_BITS(g_tx_adv_buf.txheader, LL_CHN_SEL_ALGORITHM_2, CHSEL_SHIFT, CHSEL_MASK);
                        }
                        else
                        {
                            conn_param[initInfo.connId].channel_selection = LL_CHN_SEL_ALGORITHM_1;
                            SET_BITS(g_tx_adv_buf.txheader, LL_CHN_SEL_ALGORITHM_1, CHSEL_SHIFT, CHSEL_MASK);
                        }

                        // calculate initA if using RPA list, otherwise copy the address stored in initInfo
                        if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM &&
                                !ll_isIrkAllZero(g_llResolvinglist[rpaListIndex].localIrk) &&
                                (initInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_PUBLIC || initInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM))
                        {
                            // for resolving private address case, calculate the scanA with Local IRK
                            ll_CalcRandomAddr(g_llResolvinglist[rpaListIndex].localIrk, &g_tx_adv_buf.data[0]);
                            SET_BITS(g_tx_adv_buf.txheader, LL_DEV_ADDR_TYPE_RANDOM, TX_ADD_SHIFT, TX_ADD_MASK);
//                           osal_memcpy( &g_currentLocalRpa[0],  &g_tx_adv_buf.data[0], 6);
                            g_currentLocalAddrType = LL_DEV_ADDR_TYPE_RPA_RANDOM;
                        }
                        else
                        {
                            if (initInfo.ownAddrType == LL_DEV_ADDR_TYPE_PUBLIC || initInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_PUBLIC)
                            {
                                osal_memcpy((uint8*)&g_tx_adv_buf.data[0], &ownPublicAddr[0], 6);
                                SET_BITS(g_tx_adv_buf.txheader, LL_DEV_ADDR_TYPE_PUBLIC, TX_ADD_SHIFT, TX_ADD_MASK);
                            }
                            else
                            {
                                osal_memcpy((uint8*)&g_tx_adv_buf.data[0], &ownRandomAddr[0], 6);
                                SET_BITS(g_tx_adv_buf.txheader, LL_DEV_ADDR_TYPE_RANDOM, TX_ADD_SHIFT, TX_ADD_MASK);
                            }

                            g_currentLocalAddrType = LL_DEV_ADDR_TYPE_RANDOM;             // not accute local type, for branch selection in enh conn complete event
                        }

                        // send conn req
                        T2 = read_current_fine_time();
                        delay = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);

                        if (delay > 118 - pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY] - pGlobal_config[LL_HW_PLL_DELAY])   // not enough time
                        {
                            // not enough time to send conn req, store the RPA
                            isPeerRpaStore = TRUE;
                            storeRpaListIndex = rpaListIndex;
                            osal_memcpy(&currentPeerRpa[0], &g_rx_adv_buf.data[0], 6);
//                          LOG("store %d\n", storeRpaListIndex);
                            g_same_rf_channel_flag = FALSE;
                            //LOG("<%d>", delay);
                        }
                        else
                        {
                            delay = 118 - delay - pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY];
                            ll_hw_set_trx_settle(delay,                               // set BB delay, about 80us in 16MHz HCLK
                                                 pGlobal_config[LL_HW_AFE_DELAY],
                                                 pGlobal_config[LL_HW_PLL_DELAY]);        //RxAFE,PLL
                            // reset Rx/Tx FIFO
                            ll_hw_rst_rfifo();
                            ll_hw_rst_tfifo();
                            // send conn req
                            ll_hw_set_stx();             // set LL HW as single Tx mode
                            ll_hw_go();
                            llWaitingIrq = TRUE;
                            // AdvA, offset 6
                            osal_memcpy((uint8*)&g_tx_adv_buf.data[6], &g_rx_adv_buf.data[0], 6);
                            //2020.8.11 Jie:add init req header for RxAdd
                            SET_BITS(g_tx_adv_buf.txheader, txAdd, RX_ADD_SHIFT, RX_ADD_MASK);
                            //write Tx FIFO
                            ll_hw_write_tfifo((uint8*)&(g_tx_adv_buf.txheader),
                                              ((g_tx_adv_buf.txheader & 0xff00) >> 8) + 2);   // payload length + header length(2)

                            if (g_currentPeerAddrType >= 0x02)
                                osal_memcpy(&g_currentPeerRpa[0], &g_rx_adv_buf.data[0], 6);

                            if (g_currentLocalAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM)
                                osal_memcpy( &g_currentLocalRpa[0],  &g_tx_adv_buf.data[0], 6);

                            move_to_master_function();
                            isPeerRpaStore = FALSE;
                            bConnecting = TRUE;
                            g_same_rf_channel_flag = FALSE;
                        }
                    }
                }
                else if (packet_len   != 0
                         && (pdu_type == ADV_DIRECT_IND))     // TODO: add process of direct ADV
                {
                }
            }

            // scan again if not start connect
            if (!bConnecting)           // if not waiting for scan rsp, schedule next scan
            {
                if (initInfo.scanMode == LL_SCAN_STOP)
                {
                    // scan has been stopped
                    llState = LL_STATE_IDLE;                                                 // for single connection case, set the LL state idle
                    //  release the associated allocated connection
                    llReleaseConnId(connPtr);                                                // new for multi-connection
                    g_ll_conn_ctx.numLLMasterConns --;
                    (void)osal_set_event( LL_TaskID, LL_EVT_MASTER_CONN_CANCELLED );         // inform high layer
                }
                else
                {
                    // not sending SCAN REQ, update scan time
                    llScanTime += ((ISR_entry_time > llScanT1) ? (ISR_entry_time - llScanT1) : (BASE_TIME_UNITS - llScanT1 + ISR_entry_time));

                    if (llScanTime >= initInfo.scanWindow * 625)
                    {
                        // calculate next scan channel
                        LL_CALC_NEXT_SCAN_CHN(initInfo.nextScanChan);

                        // schedule next scan event
                        if (initInfo.scanWindow == initInfo.scanInterval)      // scanWindow == scanInterval, trigger immediately
                            LL_evt_schedule();
                        else
//                            set_timer4((initInfo.scanInterval - initInfo.scanWindow) * 625);
                            ll_schedule_next_event((initInfo.scanInterval - initInfo.scanWindow) * 625);

                        // reset scan total time
                        llScanTime = 0;
                    }
                    else
                        llSetupScan(initInfo.nextScanChan);
                }
            }
        }
    }

    // post ISR process
    if (!llWaitingIrq)                      // bug fixed 2018-05-04, only clear IRQ status when no config new one
        ll_hw_clr_irq();

    HAL_EXIT_CRITICAL_SECTION();
    return TRUE;
}

uint8 llSetupStartEncRsp( llConnState_t* connPtr )
{
    uint8 pktLen;
    uint8* pBuf = connPtr->ctrlData.data;
    // Note: No need to check if there's enough room in the TX FIFO since it was
    //       forced to empty prior to beginning encryption control procedure.
    // write control type as payload
    *pBuf = LL_CTRL_START_ENC_RSP;
    // encrypt PDU with authentication check
    LL_ENC_Encrypt( connPtr,
                    LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
                    LL_START_ENC_RSP_PAYLOAD_LEN,
                    pBuf );   // input no-encrypt data pBuf, output in the same buffer
    pktLen = LL_START_ENC_RSP_PAYLOAD_LEN + LL_ENC_MIC_LEN;
    connPtr->ctrlDataIsPending  = 1;
    connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;

    // control procedure timeout value only needed for Master after Start Enc Response
//    if ( llState == LL_STATE_CONN_MASTER )
    if( connPtr->llTbd1 != LL_LINK_CONNECT_COMPLETE_MASTER )
    {
        // set the control packet timeout for 40s relative to our present time
        // Note: This is done in terms of connection events.
        // Note: Core Spec V4.0 now indicates that each LL control PDU that is queued
        //       for transmission resets the procedure response timeout timer.
        connPtr->ctrlPktInfo.ctrlTimeout = connPtr->ctrlPktInfo.ctrlTimeoutVal;
    }

    return( TRUE );
}

uint8 llProcessSlaveControlProcedures1( llConnState_t* connPtr )
{
    // check if there are any control packets ready for processing
    while ( connPtr->ctrlPktInfo.ctrlPktCount > 0 )
    {
        // processing based on control packet type at the head of the queue
        switch( connPtr->ctrlPktInfo.ctrlPkts[ 0 ] )
        {
        case LL_CTRL_TERMINATE_IND:

            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // we have already place packet on TX FIFO, so check if its been ACK'ed
                if ( rfCounters.numTxCtrlAck )
                {
                    //  yes, so process the termination
                    // Note: No need to cleanup control packet info as we are done.
                    llConnTerminate( connPtr, LL_HOST_REQUESTED_TERM );
                    return( LL_CTRL_PROC_STATUS_TERMINATE );
                }
                else // no done yet
                {
                    // check if a termination control procedure timeout has occurred
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_HOST_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else // no control procedure timeout yet
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupTermInd( connPtr );
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

        // Note: Unreachable statement generates compiler warning!
        //break;

        case LL_CTRL_ENC_RSP:

            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    // done with this control packet, so remove from the processing queue
                    // Note: By dequeueing here, it is possible to get another control
                    //       packet at the head of the queue. This is techincally not
                    //       supposed to happen if the spec is followed.
                    // ALT: COULD MAKE MORE BULLET PROOF. SINCE THE REPLACE ROUTINE
                    //      CAN'T BE USED UNTIL THE LTK IS RECEIVED BY THE HOST, A
                    //      DUMMY CONTROL PACKET THAT SITS AT THE HEAD UNTIL IT IS
                    //      REPLACE COULD BE USED INSTEAD.
                    //llReplaceCtrlPkt( connPtr, LL_CTRL_DUMMY_PLACE_HOLDER );
                    llDequeueCtrlPkt( connPtr );
                    // notify the Host with RAND and EDIV after sending the RSP
                    // Note: Need to wait for the Host reply to determine if the LTK
                    //       is available or not.
                    LL_EncLtkReqCback( connPtr->connId,
                                       connPtr->encInfo.RAND,
                                       connPtr->encInfo.EDIV );
                }
                else // not done yet
                {
                    // check if a update param req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupEncRsp( connPtr );
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        case LL_CTRL_START_ENC_REQ:

            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This only means the packet has been transmitted, not that it
                //       has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    // enable encryption once start encryption request is sent
                    // Note: We can not receive data once the encryption control
                    //       procedure has begun, so there is no risk of a race
                    //       condition here.
                    connPtr->encEnabled = TRUE;
                    // clear packet counters
                    connPtr->encInfo.txPktCount = 0;
                    connPtr->encInfo.rxPktCount = 0;
                }

                // not done until the LL_CTRL_START_ENC_RSP is received, so check it
                // Note: The following code can not be in the previous "if" statement
                //       since it is possible that numTxCtrl could be true, yet the
                //       flag startEncRspRcved isn't. Then on the next event,
                //       numTxCtrl wouldn't be true, and we would never check the
                //       startEncRspRcved flag again. Since we can't get the
                //       LL_START_ENC_RSP until we send the LL_CTRL_START_ENC_REQ,
                //       this isn't an issue.
                if ( connPtr->encInfo.startEncRspRcved == TRUE )
                {
                    // replace control procedure at head of queue to prevent interleaving
                    llReplaceCtrlPkt( connPtr, LL_CTRL_START_ENC_RSP );
                }
                else // not done yet
                {
                    // check if a start enc req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // first, check if the SK has been calculated
                if ( connPtr->encInfo.SKValid == TRUE )
                {
                    // so try to begin the last step of the encryption procedure
                    if ( llSetupStartEncReq( connPtr ) == TRUE )
                    {
                        // ready the flag that indicates that we've received the response
                        connPtr->encInfo.startEncRspRcved = FALSE;
                        // the control packet is now active
                        connPtr->ctrlPktInfo.ctrlPktActive = TRUE;
                    }

                    // Note: Two cases are possible:
                    //       a) We successfully placed the packet in the TX FIFO.
                    //       b) We did not.
                    //
                    //       In case (a), it may be possible that a previously just
                    //       completed control packet happened to complete based on
                    //       rfCounters.numTxCtrl. Since the current control
                    //       procedure is now active, it could falsely detect
                    //       rfCounters.numTxCtrl, when in fact this was from the
                    //       previous control procedure. Consequently, return.
                    //
                    //       In case (b), the control packet stays at the head of the
                    //       queue, and there's nothing more to do. Consequently, return.
                    //
                    //       So, in either case, return.
                    return( LL_CTRL_PROC_STATUS_SUCCESS );
                }
                else // SK isn't valid yet, so see if we've received the LTK yet
                {
                    if ( connPtr->encInfo.LTKValid )
                    {
                        // generate the Session Key (i.e. SK = AES128(LTK, SKD))
                        LL_ENC_GenerateSK( connPtr->encInfo.LTK,
                                           connPtr->encInfo.SKD,
                                           connPtr->encInfo.SK );
                        // indicate the SK is valid, and drop through
                        connPtr->encInfo.SKValid = TRUE;
                    }
                    else // not done yet
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }

            break;

        case LL_CTRL_START_ENC_RSP:

            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This only means the packet has been transmitted, not that it
                //       has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    // packet TX'ed, so we are done with the encryption procedure
                    // re-activate slave latency
                    connPtr->slaveLatency = connPtr->slaveLatencyValue;
                    // remove control packet from processing queue and drop through
                    llDequeueCtrlPkt( connPtr );
                    // set flag to allow outgoing data transmissions
                    connPtr->txDataEnabled = TRUE;
                    // okay to receive data again
                    connPtr->rxDataEnabled = TRUE;

                    // notify the Host
                    if ( connPtr->encInfo.encRestart == TRUE )
                    {
                        // a key change was requested
                        LL_EncKeyRefreshCback( connPtr->connId,
                                               LL_ENC_KEY_REQ_ACCEPTED );
                    }
                    else
                    {
                        // a new encryption was requested
                        LL_EncChangeCback( connPtr->connId,
                                           LL_ENC_KEY_REQ_ACCEPTED,
                                           LL_ENCRYPTION_ON );
                    }

                    // clear the restart flag in case of another key change request,
                    // and all other encryption flags
                    // Note: But in reality, there isn't a disable encryption in BLE,
                    //       so once encryption is enabled, any call to LL_StartEncrypt
                    //       will result in an encryption key change callback.
                    connPtr->encInfo.encRestart       = FALSE;
                    connPtr->encInfo.encReqRcved      = FALSE;
                    connPtr->encInfo.pauseEncRspRcved = FALSE;
                    connPtr->encInfo.startEncRspRcved = FALSE;
                }
                else // not done yet
                {
                    // check if a update param req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupStartEncRsp( connPtr );
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        case LL_CTRL_PAUSE_ENC_RSP:

            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // not done until the LL_CTRL_PAUSE_ENC_RSP is received, so check it
                if ( connPtr->encInfo.pauseEncRspRcved == TRUE )
                {
                    // done with this control packet, so remove from the processing
                    // queue and drop through (so the encrypton response can be
                    // processed)
                    // ALT: COULD REPLACE HEAD OF QUEUE WITH DUMMY SO NO OTHER CONTROL
                    //      PROCEDURE CAN INTERLEAVE BEFORE THE ENC_REQ IS RECEIVED.
                    llDequeueCtrlPkt( connPtr );
                }
                else // not received yet, so decrement and check control procedure timeout
                {
                    // check if a start enc req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there
                // Note: All pending transmissions must also be finished before this
                //       packet is placed in the TX FIFO.
                if ( llSetupPauseEncRsp( connPtr ) == TRUE )
                {
                    // clear the flag that indicates an Encryption Request has been
                    // received, which is used by this control procedure to restart the
                    // control procedure timeout
                    connPtr->encInfo.pauseEncRspRcved = FALSE;
                    // disable encryption
                    // Note: Not really necessary as no data is supposed to be sent
                    //       or received.
                    connPtr->encEnabled = FALSE;
                    // the control packet is now active; drop through
                    connPtr->ctrlPktInfo.ctrlPktActive = TRUE;
                }
                else // not done yet
                {
                    //  control packet stays at head of queue, so exit here
                    return( LL_CTRL_PROC_STATUS_SUCCESS );
                }
            }

            break;

        case LL_CTRL_REJECT_IND:

            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This only means the packet has been transmitted, not that it
                //       has been ACK'ed or NACK'ed.
                // Note: The control procedure does not end until the Reject is ACKed.
                //       However, if the ACK is a data packet, it will be tossed
                //       unless data is allowed hereafter. So to avoid this, only
                //       the confirmed transmission of this will be used to qualify
                //       the related flags, but a new procedure will not be able to
                //       begin until this procedure completes, per the spec.
                if ( rfCounters.numTxCtrl )
                {
                    // disable encryption
                    // Note: Never really enabled so this isn't necessary.
                    connPtr->encEnabled = FALSE;
                    // set flag to allow outgoing data transmissions
                    connPtr->txDataEnabled = TRUE;
                    // okay to receive data again
                    connPtr->rxDataEnabled = TRUE;
                }

                // we have already place packet on TX FIFO, so check if its been ACK'ed
                if ( rfCounters.numTxCtrlAck )
                {
                    // done with this control packet, so remove from the processing
                    // queue and drop through
                    llDequeueCtrlPkt( connPtr );
                }
                else // not ack'ed yet
                {
                    // check if a control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupRejectInd( connPtr,connPtr->encInfo.encRejectErrCode);
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        // should be LL_CTRL_SLAVE_FEATURE_REQ
//      case LL_CTRL_FEATURE_REQ:    // for v4.2, slave may send LL_CTRL_FEATURE_REQ msg. to be test later.........  HZF
//        // check if the control packet procedure is active
//        if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
//        {
//          // we have already placed a packet on TX FIFO, so wait now until we
//          // get the slave's LL_CTRL_FEATURE_RSP
//          if ( connPtr->featureSetInfo.featureRspRcved == TRUE )
//          {
//            // notify the Host
//            LL_ReadRemoteUsedFeaturesCompleteCback( LL_STATUS_SUCCESS,
//                                                    connPtr->connId,
//                                                    connPtr->featureSetInfo.featureSet );

//            // done with this control packet, so remove from the processing queue
//            llDequeueCtrlPkt( connPtr );
//          }
//          else // no done yet
//          {
//            // check if a update param req control procedure timeout has occurred
//            // Note: No need to cleanup control packet info as we are done.
//            if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
//            {
//              // indicate a control procedure timeout on this request
//              // Note: The parameters are not valid.
//              LL_ReadRemoteUsedFeaturesCompleteCback( LL_CTRL_PKT_TIMEOUT_TERM,
//                                                      connPtr->connId,
//                                                      connPtr->featureSetInfo.featureSet );
//              // we're done waiting, so end it all
//              // Note: No need to cleanup control packet info as we are done.
//              llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_HOST_TERM );

//              return( LL_CTRL_PROC_STATUS_TERMINATE );
//            }
//            else
//            {
//              //  control packet stays at head of queue, so exit here
//              return( LL_CTRL_PROC_STATUS_SUCCESS );
//            }
//          }
//        }
//        else // control packet has not been put on the TX FIFO yet
//        {
//          // so try to put it there; being active depends on a success
//          connPtr->ctrlPktInfo.ctrlPktActive = llSetupFeatureSetReq( connPtr );

//          // set flag while we wait for response
//          // Note: It is okay to repeatedly set this flag in the event the
//          //       setup routine hasn't completed yet (e.g. if the TX FIFO
//          //       has not yet become empty).
//          connPtr->featureSetInfo.featureRspRcved = FALSE;

//          // Note: Two cases are possible:
//          //       a) We successfully placed the packet in the TX FIFO.
//          //       b) We did not.
//          //
//          //       In case (a), it may be possible that a previously just
//          //       completed control packet happened to complete based on
//          //       rfCounters.numTxCtrlAck. Since the current control
//          //       procedure is now active, it could falsely detect
//          //       rfCounters.numTxCtrlAck, when in fact this was from the
//          //       previous control procedure. Consequently, return.
//          //
//          //       In case (b), the control packet stays at the head of the
//          //       queue, and there's nothing more to do. Consequently, return.
//          //
//          //       So, in either case, return.
//          return( LL_CTRL_PROC_STATUS_SUCCESS );
//        }

//        break;

        case LL_CTRL_FEATURE_RSP:

            // check if the control packet procedure is is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    // packet TX'ed, so use this flag on the Slave to indicate that
                    // the feature response procedure has already taken place on this
                    // connection
                    // Note: This is being done to support the HCI extension command
                    //       LL_EXT_SetLocalSupportedFeatures so that the user can
                    //       update the local supported features even after a connection
                    //       is formed. This update will be used as long as a feature
                    //       response feature has not been performed by the Master. Once
                    //       performed, the connection feature set is fixed!
                    connPtr->featureSetInfo.featureRspRcved = TRUE;
                    // ALT: COULD RE-ACTIVATE SL (IF ENABLED) RIGHT HERE.
                    connPtr->slaveLatency = connPtr->slaveLatencyValue;
                    // remove control packet from processing queue and drop through
                    llDequeueCtrlPkt( connPtr );
                }
                else // not done yet
                {
                    // check if a start enc req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                // Note: There is no control procedure timeout associated with this
                //       control packet.
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupFeatureSetRsp( connPtr );
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        // Version Information Indication
        case LL_CTRL_VERSION_IND:

            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if the peer's version information is valid
                if ( connPtr->verExchange.peerInfoValid == TRUE )
                {
                    // yes, so check if the host has requested this information
                    if ( connPtr->verExchange.hostRequest == TRUE )
                    {
                        // yes, so provide it
                        LL_ReadRemoteVersionInfoCback( LL_STATUS_SUCCESS,
                                                       connPtr->connId,
                                                       connPtr->verInfo.verNum,
                                                       connPtr->verInfo.comId,
                                                       connPtr->verInfo.subverNum );
                    }

                    // in any case, dequeue this control procedure
                    llDequeueCtrlPkt( connPtr );
                }
                else // no done yet
                {
                    // check if a update param req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so complete the callback with error
                        LL_ReadRemoteVersionInfoCback( LL_CTRL_PKT_TIMEOUT_TERM,
                                                       connPtr->connId,
                                                       connPtr->verInfo.verNum,
                                                       connPtr->verInfo.comId,
                                                       connPtr->verInfo.subverNum );
                        // and end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_HOST_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // since we are in the process of sending the version indication,
                // it is okay to set this flag here even if it is set repeatedly
                // in the of llSetupVersionIndReq failures
                connPtr->verExchange.verInfoSent = TRUE;
                // so try to put it there; being active depends on a success
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupVersionIndReq( connPtr );
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        case LL_CTRL_LENGTH_REQ:

            // check if the control packet procedure is is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    connPtr->llPduLen.isWatingRsp=TRUE;
                    // remove control packet from processing queue and drop through
                    llDequeueCtrlPkt( connPtr );
                }
                else // not done yet
                {
                    // check if a start enc req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                // Note: There is no control procedure timeout associated with this
                //       control packet.
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupDataLenghtReq( connPtr );
                connPtr->llPduLen.isWatingRsp=FALSE;
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        case LL_CTRL_LENGTH_RSP:

            // check if the control packet procedure is is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    connPtr->llPduLen.isProcessingReq=FALSE;
                    llPduLengthUpdate((uint16)connPtr->connId);
                    // remove control packet from processing queue and drop through
                    llDequeueCtrlPkt( connPtr );
                }
                else // not done yet
                {
                    // check if a start enc req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                // Note: There is no control procedure timeout associated with this
                //       control packet.
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupDataLenghtRsp( connPtr );
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        case LL_CTRL_PHY_REQ:

            // check if the control packet procedure is is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    connPtr->llPhyModeCtrl.isWatingRsp=TRUE;
                    // remove control packet from processing queue and drop through
                    llDequeueCtrlPkt( connPtr );
                }
                else // not done yet
                {
                    // check if a start enc req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                // Note: There is no control procedure timeout associated with this
                //       control packet.
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupPhyReq( connPtr );
                connPtr->llPhyModeCtrl.isWatingRsp=FALSE;
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        case LL_CTRL_PHY_RSP:

            // check if the control packet procedure is is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    connPtr->llPhyModeCtrl.isProcessingReq=FALSE;
                    connPtr->llPhyModeCtrl.isWatingRsp=TRUE;
                    // remove control packet from processing queue and drop through
                    llDequeueCtrlPkt( connPtr );
                }
                else // not done yet
                {
                    // check if a start enc req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                // Note: There is no control procedure timeout associated with this
                //       control packet.
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupPhyRsp( connPtr );
                connPtr->llPhyModeCtrl.isWatingRsp=FALSE;
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        case LL_CTRL_CTE_REQ:

            // check if the control packet procedure is is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    //                 connPtr->llPhyModeCtrl.isWatingRsp=TRUE;
                    // remove control packet from processing queue and drop through
                    llDequeueCtrlPkt( connPtr );
                }
                else // not done yet
                {
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        osal_memset( &(connPtr->llCTEModeCtrl), 0, sizeof( connPtr->llCTEModeCtrl ));
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupCTEReq( connPtr );
                connPtr->llCTEModeCtrl.isWatingRsp = TRUE;
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        case LL_CTRL_CTE_RSP:

            // check if the control packet procedure is is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    connPtr->llCTEModeCtrl.isWatingRsp = FALSE;
                    connPtr->llCTEModeCtrl.isProcessingReq = FALSE;
                    // remove control packet from processing queue and drop through
                    // 2020-02-12 comment:after send CONN CTE RSP , then clear txSupp
                    ll_hw_set_cte_txSupp( CTE_SUPP_NULL);
                    llDequeueCtrlPkt( connPtr );
                }
                else // not done yet
                {
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupCTERsp( connPtr );
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        case LL_CTRL_UNKNOWN_RSP:

            // try to place control packet in the TX FIFO
            // Note: Since there are no dependencies for this control packet, we
            //       do not have to bother with the active flag.
            if ( llSetupUnknownRsp( connPtr ) == TRUE )
            {
                // all we have to do is put this control packet on the TX FIFO, so
                // remove control packet from the processing queue and drop through
                llDequeueCtrlPkt( connPtr );
            }
            else // not done yet
            {
                // control packet stays at head of queue, so exit here
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        // Dummy Place Holder
        //case LL_CTRL_DUMMY_PLACE_HOLDER:
        //  //  dummy packet stays at head of queue, so exit here
        // Note: Unreachable statement generates compiler warning!
        //break;
        //  return( LL_CTRL_PROC_STATUS_SUCCESS );

        default:
            break;
        }
    }

    return( LL_CTRL_PROC_STATUS_SUCCESS );
}

uint8 llProcessMasterControlProcedures1( llConnState_t* connPtr )
{
    // check if there are any control packets ready for processing
    while ( connPtr->ctrlPktInfo.ctrlPktCount > 0 )
    {
        // processing based on control packet type at the head of the queue
        switch( connPtr->ctrlPktInfo.ctrlPkts[ 0 ] )
        {
        case LL_CTRL_TERMINATE_IND:

            // check if the control packet procedure is is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // we have already place packet on TX FIFO, so check if its been ACK'ed
                if ( rfCounters.numTxCtrlAck )
                {
                    // done with this control packet, so remove from the processing queue
                    llDequeueCtrlPkt( connPtr );
                    // yes, so process the termination
                    // Note: No need to cleanup control packet info as we are done.
                    llConnTerminate( connPtr, LL_HOST_REQUESTED_TERM );
                    return( LL_CTRL_PROC_STATUS_TERMINATE );
                }
                else // no done yet
                {
                    // check if a termination control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_HOST_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupTermInd( connPtr );
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

        // Note: Unreachable statement generates compiler warning!
        //break;

        /*
        ** Connection Update Request
        */
        case LL_CTRL_CONNECTION_UPDATE_REQ:

//      LOG("CONN UPD");
            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // we have already placed a packet on TX FIFO, so check if its been ACK'ed
                if ( rfCounters.numTxCtrlAck )
                {
                    // yes, so adjust all time values to units of 625us
                    connPtr->paramUpdate.winSize      <<= 1;
                    connPtr->paramUpdate.winOffset    <<= 1;
                    connPtr->paramUpdate.connInterval <<= 1;
                    connPtr->paramUpdate.connTimeout  <<= 4;
                    // and activate the update
                    connPtr->pendingParamUpdate = TRUE;
                    // done with this control packet, so remove from the processing queue
                    llDequeueCtrlPkt( connPtr );
                }
                else // no done yet
                {
                    // Core Spec V4.0 now indicates there is no control procedure
                    // timeout. However, it still seems prudent to monitor for the
                    // instant while waiting for the slave's ACK.
                    if ( connPtr->nextEvent == connPtr->paramUpdateEvent )
                    {
                        // this event is the instant, and the control procedure still
                        // has not been ACK'ed, we the instant has passed
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_INSTANT_PASSED_HOST_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else // continue waiting for the slave's ACK
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupUpdateParamReq( connPtr );
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        /*
        ** Channel Map Update Request
        */
        case LL_CTRL_CHANNEL_MAP_REQ:

            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // we have already placed a packet on TX FIFO, so check if its been ACK'ed
                if ( rfCounters.numTxCtrlAck )
                {
                    // yes, so activate the update
                    connPtr->pendingChanUpdate = TRUE;
                    // done with this control packet, so remove from the processing queue
                    llDequeueCtrlPkt( connPtr );
                }
                else // no done yet
                {
                    // Core Spec V4.0 now indicates there is no control procedure
                    // timeout. However, it still seems prudent to monitor for the
                    // instant while waiting for the slave's ACK.
                    if ( connPtr->nextEvent == connPtr->chanMapUpdateEvent )
                    {
                        // this event is the instant, and the control procedure still
                        // has not been ACK'ed, we the instant has passed
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_INSTANT_PASSED_HOST_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else // continue waiting for the slave's ACK
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupUpdateChanReq( connPtr );
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        /*
        ** Encryption Request
        */
        case LL_CTRL_ENC_REQ:

//          LOG("1 ENC_REQ->");
            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    // set flag to discard all incoming data transmissions
                    connPtr->rxDataEnabled = FALSE;
                }

                // we have already placed a packet on TX FIFO, so wait now until we
                // get the slave's LL_START_ENC_REQ
                if ( connPtr->encInfo.startEncReqRcved == TRUE )
                {
                    // clear packet counters
                    connPtr->encInfo.txPktCount = 0;
                    connPtr->encInfo.rxPktCount = 0;
                    // enable encryption
                    connPtr->encEnabled = TRUE;
                    // replace control procedure at head of queue to prevent interleaving
                    llReplaceCtrlPkt( connPtr, LL_CTRL_START_ENC_RSP );
                }
                else if ( connPtr->encInfo.rejectIndRcved  == TRUE )
                {
                    // the slave's Host has failed to provide an LTK, so the encryption
                    // setup has been rejected; end the start encryption procedure
                    // done with this control packet, so remove from the processing queue
                    llDequeueCtrlPkt( connPtr );
                    // disable encryption
                    // Note: Not really necessary as no data is supposed to be sent
                    //       or received.
                    connPtr->encEnabled = FALSE;
                    // set flag to allow outgoing transmissions again
                    connPtr->txDataEnabled = TRUE;
                    // set flag to allow all incoming data transmissions
                    connPtr->rxDataEnabled = TRUE;

                    // check the rejection indication error code
                    if ( connPtr->encInfo.encRejectErrCode == LL_STATUS_ERROR_PIN_OR_KEY_MISSING )
                    {
                        // notify the Host
                        LL_EncChangeCback( connPtr->connId,
                                           LL_ENC_KEY_REQ_REJECTED,
                                           LL_ENCRYPTION_OFF );
                    }
                    else // LL_STATUS_ERROR_UNSUPPORTED_REMOTE_FEATURE
                    {
                        // notify the Host
                        LL_EncChangeCback( connPtr->connId,
                                           LL_ENC_KEY_REQ_UNSUPPORTED_FEATURE,
                                           LL_ENCRYPTION_OFF );
                    }
                }
                else if ( connPtr->termInfo.termIndRcvd == TRUE )
                {
                    // the slave's Host has failed to provide an LTK, so the encryption
                    // setup has been rejected; end the start encryption procedure
                    // done with this control packet, so remove from the processing queue
                    llDequeueCtrlPkt( connPtr );
                }
                else // no done yet
                {
                    // check if a update param req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // notify the Host
                        if ( connPtr->encInfo.encRestart == TRUE )
                        {
                            // a key change was requested
                            LL_EncKeyRefreshCback( connPtr->connId,
                                                   LL_CTRL_PKT_TIMEOUT_TERM );
                        }
                        else
                        {
                            // a new encryption was requested
                            LL_EncChangeCback( connPtr->connId,
                                               LL_CTRL_PKT_TIMEOUT_TERM,
                                               LL_ENCRYPTION_OFF );
                        }

                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_HOST_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupEncReq( connPtr );
                // set a flag to indicate we have received LL_START_ENC_REQ
                // Note: The LL_ENC_RSP will be received first, which will result in
                //       the master calculating its IVm and SKDm, concatenating it
                //       with the slave's IVs and SKDs, and calculating the SK from
                //       the LTK and SKD. After that, we will receive the
                //       LL_START_ENC_REQ from the slave. So, it is okay to stay in
                //       this control procedure until LL_START_ENC_REQ is received.
                // Note: It is okay to repeatedly set this flag in the event the
                //       setup routine hasn't completed yet (e.g. if the TX FIFO
                //       has not yet become empty).
                connPtr->encInfo.startEncReqRcved = FALSE;
                connPtr->encInfo.rejectIndRcved   = FALSE;
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        /*
        ** Encryption Start Response
        */
        case LL_CTRL_START_ENC_RSP:

//          LOG("1 START_ENC_RSP->");
            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // we have already placed a packet on TX FIFO, so wait now until we
                // get the slave's LL_START_ENC_RSP
                if ( connPtr->encInfo.startEncRspRcved == TRUE )
                {
                    // done with this control packet, so remove from the processing queue
                    llDequeueCtrlPkt( connPtr );
                    // we're done with encryption procedure, so clear flags
                    connPtr->encInfo.encReqRcved      = FALSE;
                    connPtr->encInfo.pauseEncRspRcved = FALSE;
                    connPtr->encInfo.startEncReqRcved = FALSE;
                    connPtr->encInfo.startEncRspRcved = FALSE;
                    connPtr->encInfo.rejectIndRcved   = FALSE;
                }
                else // no done yet
                {
                    // check if a update param req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // notify the Host
                        if ( connPtr->encInfo.encRestart == TRUE )
                        {
                            // a key change was requested
                            LL_EncKeyRefreshCback( connPtr->connId,
                                                   LL_CTRL_PKT_TIMEOUT_TERM );
                        }
                        else
                        {
                            // a new encryption was requested
                            LL_EncChangeCback( connPtr->connId,
                                               LL_CTRL_PKT_TIMEOUT_TERM,
                                               LL_ENCRYPTION_OFF );
                        }

                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_HOST_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                // Note: The llSetupStartEncRsp routine will *not* reset the control
                //       timeout value since the entire encryption procedure starts
                //       with the master sending the LL_ENC_REQ, and ends when the
                //       master receives the LL_START_ENC_RSP from the slave.
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupStartEncRsp( connPtr );
                // set a flag to indicate we have received LL_START_ENC_RSP
                // Note: It is okay to repeatedly set this flag in the event the
                //       setup routine hasn't completed yet (e.g. if the TX FIFO
                //       has not yet become empty).
                connPtr->encInfo.startEncRspRcved = FALSE;
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        /*
        ** Encryption Pause Request
        */
        case LL_CTRL_PAUSE_ENC_REQ:

            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // we have already placed a packet on TX FIFO, so wait now until we
                // get the slave's LL_PAUSE_ENC_RSP
                if ( connPtr->encInfo.pauseEncRspRcved == TRUE )
                {
                    // disable encryption
                    connPtr->encEnabled = FALSE;
                    // replace control procedure at head of queue to prevent interleaving
                    llReplaceCtrlPkt( connPtr, LL_CTRL_PAUSE_ENC_RSP );
                }
                else // no done yet
                {
                    // check if a update param req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // notify the Host
                        if ( connPtr->encInfo.encRestart == TRUE )
                        {
                            // a key change was requested
                            LL_EncKeyRefreshCback( connPtr->connId,
                                                   LL_CTRL_PKT_TIMEOUT_TERM );
                        }
                        else
                        {
                            // a new encryption was requested
                            LL_EncChangeCback( connPtr->connId,
                                               LL_CTRL_PKT_TIMEOUT_TERM,
                                               LL_ENCRYPTION_OFF );
                        }

                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_HOST_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                // Note: The llSetupStartEncRsp routine will *not* reset the control
                //       timeout value since the entire encryption procedure starts
                //       with the master sending the LL_ENC_REQ, and ends when the
                //       master receives the LL_START_ENC_RSP from the slave.
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupPauseEncReq( connPtr );
                // set a flag to indicate we have received LL_START_ENC_RSP
                // Note: It is okay to repeatedly set this flag in the event the
                //       setup routine hasn't completed yet (e.g. if the TX FIFO
                //       has not yet become empty).
                connPtr->encInfo.pauseEncRspRcved = FALSE;
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        /*
        ** Encryption Pause Response
        */
        case LL_CTRL_PAUSE_ENC_RSP:

            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This only means the packet has been transmitted, not that it
                //       has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    // replace control procedure at head of queue to prevent interleaving
                    llReplaceCtrlPkt( connPtr, LL_CTRL_ENC_REQ );
                }
                else // no done yet
                {
                    // check if a update param req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // notify the Host
                        if ( connPtr->encInfo.encRestart == TRUE )
                        {
                            // a key change was requested
                            LL_EncKeyRefreshCback( connPtr->connId,
                                                   LL_CTRL_PKT_TIMEOUT_TERM );
                        }
                        else
                        {
                            // a new encryption was requested
                            LL_EncChangeCback( connPtr->connId,
                                               LL_CTRL_PKT_TIMEOUT_TERM,
                                               LL_ENCRYPTION_OFF );
                        }

                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_HOST_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupPauseEncRsp( connPtr );
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        /*
        ** Feature Set Request
        */
        case LL_CTRL_FEATURE_REQ:

            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // we have already placed a packet on TX FIFO, so wait now until we
                // get the slave's LL_CTRL_FEATURE_RSP
                if ( connPtr->featureSetInfo.featureRspRcved == TRUE )
                {
                    // notify the Host
                    LL_ReadRemoteUsedFeaturesCompleteCback( LL_STATUS_SUCCESS,
                                                            connPtr->connId,
                                                            connPtr->featureSetInfo.featureSet );
                    // done with this control packet, so remove from the processing queue
                    llDequeueCtrlPkt( connPtr );
                }
                else // no done yet
                {
                    // check if a update param req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // indicate a control procedure timeout on this request
                        // Note: The parameters are not valid.
                        LL_ReadRemoteUsedFeaturesCompleteCback( LL_CTRL_PKT_TIMEOUT_TERM,
                                                                connPtr->connId,
                                                                connPtr->featureSetInfo.featureSet );
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_HOST_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // add by HZF, read device feature set
                for (int i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
                {
                    connPtr->featureSetInfo.featureSet[i] = deviceFeatureSet.featureSet[i];
                }

                // so try to put it there; being active depends on a success
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupFeatureSetReq( connPtr );
                // set flag while we wait for response
                // Note: It is okay to repeatedly set this flag in the event the
                //       setup routine hasn't completed yet (e.g. if the TX FIFO
                //       has not yet become empty).
                connPtr->featureSetInfo.featureRspRcved = FALSE;
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        case LL_CTRL_FEATURE_RSP:            // new for BLE4.2, feature req could be init by slave

            // check if the control packet procedure is is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    // packet TX'ed, so use this flag on the Slave to indicate that
                    // the feature response procedure has already taken place on this
                    // connection
                    // Note: This is being done to support the HCI extension command
                    //       LL_EXT_SetLocalSupportedFeatures so that the user can
                    //       update the local supported features even after a connection
                    //       is formed. This update will be used as long as a feature
                    //       response feature has not been performed by the Master. Once
                    //       performed, the connection feature set is fixed!
                    connPtr->featureSetInfo.featureRspRcved = TRUE;
                    // ALT: COULD RE-ACTIVATE SL (IF ENABLED) RIGHT HERE.
//            connPtr->slaveLatency = connPtr->slaveLatencyValue;
                    // remove control packet from processing queue and drop through
                    llDequeueCtrlPkt( connPtr );
                }
                else // not done yet
                {
                    // check if a start enc req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                // Note: There is no control procedure timeout associated with this
                //       control packet.
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupFeatureSetRsp( connPtr );
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        /*
        ** Vendor Information Exchange (Request or Reply)
        */
        case LL_CTRL_VERSION_IND:

            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if the peer's version information is valid
                if ( connPtr->verExchange.peerInfoValid == TRUE )
                {
                    // yes, so check if the host has requested this information
                    if ( connPtr->verExchange.hostRequest == TRUE )
                    {
                        // yes, so provide it
                        LL_ReadRemoteVersionInfoCback( LL_STATUS_SUCCESS,
                                                       connPtr->connId,
                                                       connPtr->verInfo.verNum,
                                                       connPtr->verInfo.comId,
                                                       connPtr->verInfo.subverNum );
                    }

                    // in any case, dequeue this control procedure
                    llDequeueCtrlPkt( connPtr );
                }
                else // no done yet
                {
                    // check if a update param req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so complete the callback with error
                        LL_ReadRemoteVersionInfoCback( LL_CTRL_PKT_TIMEOUT_TERM,
                                                       connPtr->connId,
                                                       connPtr->verInfo.verNum,
                                                       connPtr->verInfo.comId,
                                                       connPtr->verInfo.subverNum );
                        // and end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_HOST_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // since we are in the process of sending the version indication,
                // it is okay to set this flag here even if it is set repeatedly
                // in the of llSetupVersionIndReq failures
                connPtr->verExchange.verInfoSent = TRUE;
//          // so try to put it there; being active depends on a success
//          connPtr->ctrlPktInfo.ctrlPktActive = llSetupPingReq(connPtr);// llSetupVersionIndReq( connPtr );
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupVersionIndReq( connPtr );
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        case LL_CTRL_LENGTH_REQ:

            // check if the control packet procedure is is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    connPtr->llPduLen.isWatingRsp=TRUE;
                    // remove control packet from processing queue and drop through
                    llDequeueCtrlPkt( connPtr );
                }
                else // not done yet
                {
                    // check if a start enc req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                // Note: There is no control procedure timeout associated with this
                //       control packet.
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupDataLenghtReq( connPtr );
                connPtr->llPduLen.isWatingRsp=FALSE;
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        case LL_CTRL_LENGTH_RSP:

            // check if the control packet procedure is is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    connPtr->llPduLen.isProcessingReq=FALSE;
                    llPduLengthUpdate((uint16)connPtr->connId);
                    // remove control packet from processing queue and drop through
                    llDequeueCtrlPkt( connPtr );
                }
                else // not done yet
                {
                    // check if a start enc req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                // Note: There is no control procedure timeout associated with this
                //       control packet.
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupDataLenghtRsp( connPtr );
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        // LL PHY UPDATE REQ
        case LL_CTRL_PHY_REQ:

            // check if the control packet procedure is is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    connPtr->llPhyModeCtrl.isWatingRsp=TRUE;
                    // remove control packet from processing queue and drop through
                    llDequeueCtrlPkt( connPtr );
                }
                else // not done yet
                {
                    // check if a start enc req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                // Note: There is no control procedure timeout associated with this
                //       control packet.
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupPhyReq( connPtr );
                connPtr->llPhyModeCtrl.isWatingRsp=FALSE;
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        case LL_CTRL_PHY_UPDATE_IND:

            // check if the control packet procedure is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // we have already placed a packet on TX FIFO, so check if its been ACK'ed
                if ( rfCounters.numTxCtrlAck )
                {
                    //20181206 ZQ phy update no change case
                    if(     connPtr->phyUpdateInfo.m2sPhy== 0
                            &&  connPtr->phyUpdateInfo.s2mPhy== 0)
                    {
                        connPtr->phyUpdateInfo.m2sPhy=connPtr->llPhyModeCtrl.local.txPhy;
                        connPtr->phyUpdateInfo.s2mPhy=connPtr->llPhyModeCtrl.local.rxPhy;
                        llPhyModeCtrlUpdateNotify(connPtr,LL_STATUS_SUCCESS);
                    }
                    else
                    {
                        // yes, so activate the update
                        connPtr->pendingPhyModeUpdate = TRUE;
                    }

                    connPtr->llPhyModeCtrl.isWatingRsp=FALSE;
                    connPtr->llPhyModeCtrl.isProcessingReq=FALSE;
                    // done with this control packet, so remove from the processing queue
                    llDequeueCtrlPkt( connPtr );
                }
                else // no done yet
                {
                    // Core Spec V4.0 now indicates there is no control procedure
                    // timeout. However, it still seems prudent to monitor for the
                    // instant while waiting for the slave's ACK.
                    if ( connPtr->nextEvent == connPtr->phyModeUpdateEvent )
                    {
                        // this event is the instant, and the control procedure still
                        // has not been ACK'ed, we the instant has passed
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_INSTANT_PASSED_HOST_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else // continue waiting for the slave's ACK
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                // so try to put it there; being active depends on a success
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupPhyUpdateInd( connPtr );
                // Note: Two cases are possible:
                //       a) We successfully placed the packet in the TX FIFO.
                //       b) We did not.
                //
                //       In case (a), it may be possible that a previously just
                //       completed control packet happened to complete based on
                //       rfCounters.numTxCtrlAck. Since the current control
                //       procedure is now active, it could falsely detect
                //       rfCounters.numTxCtrlAck, when in fact this was from the
                //       previous control procedure. Consequently, return.
                //
                //       In case (b), the control packet stays at the head of the
                //       queue, and there's nothing more to do. Consequently, return.
                //
                //       So, in either case, return.
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        // REJECT EXT IND --> PHY UPDATE COLLSION
        case LL_CTRL_REJECT_EXT_IND:

            // check if the control packet procedure is is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    connPtr->isCollision=TRUE;
                    // remove control packet from processing queue and drop through
                    llDequeueCtrlPkt( connPtr );
                }
                else // not done yet
                {
                    // check if a start enc req control procedure timeout has occurred
                    // Note: No need to cleanup control packet info as we are done.
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                if(connPtr->llPhyModeCtrl.isWatingRsp==TRUE)
                {
                    connPtr->ctrlPktInfo.ctrlPktActive = llSetupRejectExtInd( connPtr,LL_STATUS_ERROR_LL_PROCEDURE_COLLISION);
                }
                else if(connPtr->pendingChanUpdate==TRUE ||
                        connPtr->pendingParamUpdate==TRUE )
                {
                    connPtr->ctrlPktInfo.ctrlPktActive = llSetupRejectExtInd( connPtr,LL_STATUS_ERROR_DIFF_TRANSACTION_COLLISION);
                }
                else if( connPtr->llCTEModeCtrl.isWatingRsp == TRUE)
                {
                    // 2020-01-23 add for CTE
                    connPtr->ctrlPktInfo.ctrlPktActive = llSetupRejectExtInd( connPtr,connPtr->llCTEModeCtrl.errorCode );
                    connPtr->llCTEModeCtrl.errorCode = LL_STATUS_SUCCESS;
                }
                else
                {
                    //should not be here
                }

                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        case LL_CTRL_CTE_REQ:

            // check if the control packet procedure is is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    //  connPtr->llCTEModeCtrl.isWatingRsp = TRUE;
                    // remove control packet from processing queue and drop through
                    llDequeueCtrlPkt( connPtr );
                }
                else // not done yet
                {
                    //
                    if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
                    {
                        osal_memset( &(connPtr->llCTEModeCtrl), 0, sizeof( connPtr->llCTEModeCtrl ));
                        // we're done waiting, so end it all
                        // Note: No need to cleanup control packet info as we are done.
                        llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_PEER_TERM );
                        return( LL_CTRL_PROC_STATUS_TERMINATE );
                    }
                    else
                    {
                        //  control packet stays at head of queue, so exit here
                        return( LL_CTRL_PROC_STATUS_SUCCESS );
                    }
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupCTEReq( connPtr );
                connPtr->llCTEModeCtrl.isWatingRsp = TRUE;
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        case LL_CTRL_CTE_RSP:

            // check if the control packet procedure is is active
            if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
            {
                // yes, so check if it has been transmitted yet
                // Note: This does not mean this packet has been ACK'ed or NACK'ed.
                if ( rfCounters.numTxCtrl )
                {
                    connPtr->llCTEModeCtrl.isProcessingReq = FALSE;
                    // remove control packet from processing queue and drop through
                    llDequeueCtrlPkt( connPtr );
                }
                else // not done yet
                {
                    return( LL_CTRL_PROC_STATUS_TERMINATE );
                }
            }
            else // control packet has not been put on the TX FIFO yet
            {
                connPtr->ctrlPktInfo.ctrlPktActive = llSetupCTERsp( connPtr );
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        /*
        ** Unknown Control Type Response
        */
        case LL_CTRL_UNKNOWN_RSP:

            // try to place control packet in the TX FIFO
            // Note: Since there are no dependencies for this control packet, we
            //       do not have to bother with the active flag.
            if ( llSetupUnknownRsp( connPtr ) == TRUE )
            {
                // all we have to do is put this control packet on the TX FIFO, so
                // remove control packet from the processing queue and drop through
                llDequeueCtrlPkt( connPtr );
            }
            else // not done yet
            {
                // control packet stays at head of queue, so exit here
                return( LL_CTRL_PROC_STATUS_SUCCESS );
            }

            break;

        /*
        ** Control Internal - Wait for Control ACK
        */
        case LL_CTRL_TERMINATE_RX_WAIT_FOR_TX_ACK:

            // check if the control packet has been ACK'ed (i.e. is not pending)
            // Note: Normally this routine is used for control procedures where
            //       control packets are sent by this role. This is a special case
            //       where a terminate indication was received, but we must as a
            //       master wait for our ACK to be sent before terminating.
            if ( rfCounters.numTxCtrlAck == 1)              // ctrl packet has been acked
            {
                // yes, so terminate
                // Note: No need to cleanup control packet info as we are done.
                llConnTerminate( connPtr, connPtr->termInfo.reason );
                return( LL_CTRL_PROC_STATUS_TERMINATE );
            }

            // control packet stays at head of queue, so exit here
            return( LL_CTRL_PROC_STATUS_SUCCESS );

        // Note: Unreachable statement generates compiler warning!
        //break;

        // Dummy Place Holder
        //case LL_CTRL_DUMMY_PLACE_HOLDER:
        //  //  dummy packet stays at head of queue, so exit here
        //  return( LL_CTRL_PROC_STATUS_SUCCESS );
        // Note: Unreachable statement generates compiler warning!
        //break;
        default:
            #ifdef DEBUG
            // fatal error - a unknown control procedure value was used
            LL_ASSERT( FALSE );
            #endif // DEBUG
            break;
        }
    }

    return( LL_CTRL_PROC_STATUS_SUCCESS );
}

static void llAdjBoffUpperLimitFailure1( void )
{
    // first, since this was a failure, clear the number of consecutive successes
    scanInfo.numSuccess = 0;

    // check if we received two failures in a row
    if ( ++scanInfo.numFailure == 2 )
    {
        // yes, so double backoff upper limit
        scanInfo.scanBackoffUL <<= 1;

        // maximum is 256
        if ( scanInfo.scanBackoffUL > 256 )
        {
            scanInfo.scanBackoffUL = 256;
        }

        // reset consecutive count
        scanInfo.numFailure = 0;
    }

    g_pmCounters.ll_tbd_cnt4++;
    return;
}

static void llAdjBoffUpperLimitSuccess1( void )
{
    // first, since this is a success, clear the number of consecutive failures
    scanInfo.numFailure = 0;

    // check if we received two successful in a row
    if ( ++scanInfo.numSuccess == 2 )
    {
        // yes, so half backoff upper limit
        scanInfo.scanBackoffUL >>= 1;

        // however, the minimum is 1
        if ( scanInfo.scanBackoffUL == 0 )
        {
            scanInfo.scanBackoffUL = 1;
        }

        // reset consecutive count
        scanInfo.numSuccess = 0;
    }

    return;
}

static void llGenerateNextBackoffCount1( void )
{
    // determine the new backoff count constrained by upper limit
    // Note: Backoff and Upper Limit can be 1..256.
    if ( scanInfo.scanBackoffUL == 1 )
    {
        scanInfo.currentBackoff = 1;
    }
    else // backoff count is a random number from 1..UL
    {
        scanInfo.currentBackoff = ((uint16)LL_ENC_GeneratePseudoRandNum() % scanInfo.scanBackoffUL) + 1;
    }

//    hal_uart_tx("scanBackoffUL = ");
//    hal_uart_send_int(scanInfo.scanBackoffUL);
//    hal_uart_tx(",currentBackoff = ");
//    hal_uart_send_int(scanInfo.currentBackoff);
//    hal_uart_tx("\r\n");
    return;
}

uint8 ll_processBasicIRQ_ScanTRX0(uint32_t              irq_status )
{
  _HAL_CS_ALLOC_();
  HAL_ENTER_CRITICAL_SECTION();
    ll_debug_output(DEBUG_LL_HW_TRX);
    llScanTime += ((ISR_entry_time > llScanT1) ? (ISR_entry_time - llScanT1) : (BASE_TIME_UNITS - llScanT1 + ISR_entry_time));

    // check whether receives SCAN RSP
    if (irq_status & LIRQ_COK)                        // bug correct 2018-10-15
    {
        // rx done
        uint8_t packet_len, pdu_type;
        uint16_t pktLen;
        uint32_t pktFoot0, pktFoot1;
        // read packet
        packet_len = ll_hw_read_rfifo1((uint8_t*)(&(g_rx_adv_buf.rxheader)),
                                       &pktLen,
                                       &pktFoot0,
                                       &pktFoot1);
        // check receive pdu type
        pdu_type = g_rx_adv_buf.rxheader & 0x0f;

        if(ll_hw_get_rfifo_depth()>0)
        {
            g_pmCounters.ll_rfifo_read_err++;
            packet_len=0;
            pktLen=0;
        }

        if (packet_len > 0 && pdu_type == ADV_SCAN_RSP)
        {
            // receives SCAN_RSP
            uint8  advEventType;
            uint8  rpaListIndex;
            uint8* peerAddr;
            uint8  addrType = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;
            uint8  dataLen  = pktLen - 8;
            int8   rssi     =  -(pktFoot1 >> 24);
            uint8  bCheckOk = TRUE;
            peerAddr = &g_rx_adv_buf.data[0];

            //===
            // AdvA of SCAN_RSP should also be checked here. Refer to 4.4.3.2 Active Scanning
            // After sending a scan request PDU the Link Layer listens for a scan response
            //PDU from that advertiser. If the scan response PDU was not received from that
            //advertiser, it is considered a failure; otherwise it is considered a success.

            // check AdvA in Scan Rsp is identical to Scan Req
            if (g_rx_adv_buf.data[0] != g_tx_adv_buf.data[6]  ||
                    g_rx_adv_buf.data[1] != g_tx_adv_buf.data[7]  ||
                    g_rx_adv_buf.data[2] != g_tx_adv_buf.data[8]  ||
                    g_rx_adv_buf.data[3] != g_tx_adv_buf.data[9]  ||
                    g_rx_adv_buf.data[4] != g_tx_adv_buf.data[10] ||
                    g_rx_adv_buf.data[5] != g_tx_adv_buf.data[11]
               )
                bCheckOk = FALSE;

            // RPA checking. Note that we do not check whether it is the same RPA index
            if (addrType == LL_DEV_ADDR_TYPE_RANDOM  &&
                    (g_rx_adv_buf.data[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR)
            {
                if (g_llRlEnable == TRUE)
                {
                    rpaListIndex = ll_getRPAListEntry(&g_rx_adv_buf.data[0]);

                    if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM)
                    {
                        peerAddr = &g_llResolvinglist[rpaListIndex].peerAddr[0];
                        // refer to HCI LE Advertising Report Event, RPA address type should be
                        // 0x02: Public Identity Address (Corresponds to Resolved Private Address)
                        // 0x03: Random (static) Identity Address (Corresponds to Resolved Private Address)
                        addrType = g_llResolvinglist[rpaListIndex].peerAddrType + 2;
                        bCheckOk = TRUE;
                    }
                    else
                        bCheckOk = FALSE;
                }
            }

            //===

            if (bCheckOk == TRUE)
            {
                advEventType = LL_ADV_RPT_SCAN_RSP;
                // below function cost 51us/66us(measure with GPIO)
                LL_AdvReportCback( advEventType,                   // event type
                                   addrType,                             // Adv address type (TxAdd)
                                   peerAddr,                             // Adv address (AdvA)
                                   dataLen,                              // length of rest of the payload
                                   &g_rx_adv_buf.data[6],                // rest of payload
                                   rssi );                               // RSSI
                g_pmCounters.ll_recv_scan_rsp_cnt ++;
                llAdjBoffUpperLimitSuccess1();
            }
        }
        else
            llAdjBoffUpperLimitFailure1();
    }
    else
        llAdjBoffUpperLimitFailure1();

    // update back off value according to new backoff upperLimit
    llGenerateNextBackoffCount1();

    if (llScanTime >= scanInfo.scanWindow * 625)
    {
        // calculate next scan channel
        LL_CALC_NEXT_SCAN_CHN(scanInfo.nextScanChan);

        // schedule next scan event
        if (scanInfo.scanWindow == scanInfo.scanInterval)      // scanWindow == scanInterval, trigger immediately
            LL_evt_schedule();
        else
//                set_timer4((scanInfo.scanInterval - scanInfo.scanWindow) * 625);
            ll_schedule_next_event((scanInfo.scanInterval - scanInfo.scanWindow) * 625);

        // reset scan total time
        llScanTime = 0;
    }
    else
        llSetupScan(scanInfo.nextScanChan);

    // post ISR process
    if (!llWaitingIrq)                      // bug fixed 2018-05-04, only clear IRQ status when no config new one
        ll_hw_clr_irq();

    HAL_EXIT_CRITICAL_SECTION();
    return TRUE;
}

uint8 ll_processBasicIRQ_secondaryAdvTRX0(uint32_t              irq_status )
{
  _HAL_CS_ALLOC_();
  HAL_ENTER_CRITICAL_SECTION();
    uint32_t      T2, delay;
// secondary adv state, connectable adv or scannable adv
    uint8_t  packet_len, pdu_type, txAdd;
    uint16_t pktLen;
    uint32_t pktFoot0, pktFoot1;
    int      calibra_time;                 // this parameter will be provided by global_config
    //int      i;
    // 2021-02-23
    // bugfix for multi-role secondary advertising
    // bug-case : a device in advertising and receive another device's scan request
    uint8 adv_sch_flag = TRUE;
    // read packet
    packet_len = ll_hw_read_rfifo((uint8_t*)(&(g_rx_adv_buf.rxheader)),
                                  &pktLen,
                                  &pktFoot0,
                                  &pktFoot1);

    if(ll_hw_get_rfifo_depth() > 0)
    {
        g_pmCounters.ll_rfifo_read_err++;
        packet_len=0;
        pktLen=0;
    }

    // check receive pdu type
    pdu_type = g_rx_adv_buf.rxheader & PDU_TYPE_MASK;
    txAdd    = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random

    if (packet_len > 0                       // any better checking rule for rx anything?
            && (irq_status & LIRQ_COK)
            && pdu_type == ADV_SCAN_REQ)
//          && (llState == LL_STATE_ADV_UNDIRECTED
//              || llState == LL_STATE_ADV_SCAN))
    {
        // 1. scan req
        g_pmCounters.ll_recv_scan_req_cnt ++;

        // check AdvA
        if (g_rx_adv_buf.data[6]  != adv_param.ownAddr[0]
                || g_rx_adv_buf.data[7]  != adv_param.ownAddr[1]
                || g_rx_adv_buf.data[8]  != adv_param.ownAddr[2]
                || g_rx_adv_buf.data[9]  != adv_param.ownAddr[3]
                || g_rx_adv_buf.data[10] != adv_param.ownAddr[4]
                || g_rx_adv_buf.data[11] != adv_param.ownAddr[5])
        {
        }
        else
        {
//===
            uint8_t  rpaListIndex, bWlRlCheckOk;
            uint8_t* peerAddr = &g_rx_adv_buf.data[0];      // ScanA
            adv_sch_flag = FALSE;

            // === Resolving list checking
            if (txAdd == LL_DEV_ADDR_TYPE_RANDOM
                    && (g_rx_adv_buf.data[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR)
            {
                bWlRlCheckOk = TRUE;

                // if ScanA is resolvable private address
                if (g_llRlEnable == TRUE)
                {
                    bWlRlCheckOk = FALSE;
                    rpaListIndex = ll_getRPAListEntry(&g_rx_adv_buf.data[0]);

                    if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM)
                    {
                        peerAddr = &g_llResolvinglist[rpaListIndex].peerAddr[0];
                        bWlRlCheckOk = TRUE;
                    }
                }
            }
            else   // ScanA is device Identity, if the device ID in the RPA list, check whether RPA should be used
            {
                bWlRlCheckOk = TRUE;

                for (int i = 0; i < LL_RESOLVINGLIST_ENTRY_NUM; i++)
                {
                    if (g_llResolvinglist[i].peerAddr[0] == g_rx_adv_buf.data[0]
                            && g_llResolvinglist[i].peerAddr[1] == g_rx_adv_buf.data[1]
                            && g_llResolvinglist[i].peerAddr[2] == g_rx_adv_buf.data[2]
                            && g_llResolvinglist[i].peerAddr[3] == g_rx_adv_buf.data[3]
                            && g_llResolvinglist[i].peerAddr[4] == g_rx_adv_buf.data[4]
                            && g_llResolvinglist[i].peerAddr[5] == g_rx_adv_buf.data[5]
                            && g_llResolvinglist[i].peerAddrType == txAdd)
                    {
                        if (g_llResolvinglist[i].privacyMode == NETWORK_PRIVACY_MODE &&
                                !ll_isIrkAllZero(g_llResolvinglist[i].peerIrk))
                            bWlRlCheckOk = FALSE;

                        break;
                    }
                }
            }

            // === check white list
            if ((pGlobal_config[LL_SWITCH] & LL_WHITELIST_ALLOW)
                    && (adv_param.wlPolicy  == LL_ADV_WL_POLICY_WL_SCAN_REQ
                        || adv_param.wlPolicy  == LL_ADV_WL_POLICY_WL_ALL_REQ)
                    && (bWlRlCheckOk == TRUE))
            {
                // check white list
                bWlRlCheckOk = ll_isAddrInWhiteList(txAdd, peerAddr);
            }

            if (bWlRlCheckOk == FALSE)   // if not in white list, do nothing
            {
                g_pmCounters.ll_filter_scan_req_cnt ++;
            }
            else
            {
                g_pmCounters.ll_rx_peer_cnt++;
                uint8 retScanRspFilter=1;

                if(LL_PLUS_ScanRequestFilterCBack)
                {
                    retScanRspFilter = LL_PLUS_ScanRequestFilterCBack();
                }

                if(retScanRspFilter)
                {
                    // send scan rsp
                    ll_hw_set_stx();             // set LL HW as single Tx mode
                    g_same_rf_channel_flag = TRUE;
                    // calculate the delay
                    T2 = read_current_fine_time();
                    delay = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);
                    calibra_time = pGlobal_config[SCAN_RSP_DELAY];            // consider rx_done to ISR time, SW delay after read_current_fine_time(), func read_current_fine_time() delay ...
                    delay = 118 - delay - calibra_time;                       // IFS = 150us, Tx tail -> Rx done time: about 32us
                    ll_hw_set_trx_settle(delay,                               // set BB delay, about 80us in 16MHz HCLK
                                         pGlobal_config[LL_HW_AFE_DELAY],
                                         pGlobal_config[LL_HW_PLL_DELAY]);        //RxAFE,PLL
                    ll_hw_go();
                    llWaitingIrq = TRUE;
                    g_same_rf_channel_flag = FALSE;
                    // reset Rx/Tx FIFO
                    ll_hw_rst_rfifo();
                    ll_hw_rst_tfifo();
                    //write Tx FIFO
                    ll_hw_write_tfifo((uint8*)&(tx_scanRsp_desc.txheader),
                                      ((tx_scanRsp_desc.txheader & 0xff00) >> 8) + 2);   // payload length + header length(2)
                    ll_debug_output(DEBUG_LL_HW_SET_STX);
                    g_pmCounters.ll_send_scan_rsp_cnt ++;
                }
            }
        }
    }
    else if (pdu_type == ADV_CONN_REQ
             && (irq_status & LIRQ_COK) )
//                && (llState == LL_STATE_ADV_UNDIRECTED
//                   || llState == LL_STATE_ADV_DIRECTED))
    {
        uint8_t*  peerAddr;
        uint8_t  bWlRlCheckOk = TRUE;
        // 2. connect req
        g_pmCounters.ll_recv_conn_req_cnt ++;

        // check AdvA
        if (g_rx_adv_buf.data[6]  != adv_param.ownAddr[0]
                || g_rx_adv_buf.data[7]  != adv_param.ownAddr[1]
                || g_rx_adv_buf.data[8]  != adv_param.ownAddr[2]
                || g_rx_adv_buf.data[9]  != adv_param.ownAddr[3]
                || g_rx_adv_buf.data[10] != adv_param.ownAddr[4]
                || g_rx_adv_buf.data[11] != adv_param.ownAddr[5])
        {
            // nothing to do
        }
        else
        {
            uint8_t  rpaListIndex = LL_RESOLVINGLIST_ENTRY_NUM;
            peerAddr = &g_rx_adv_buf.data[0];        // initA
            adv_sch_flag = FALSE;

            // ====== check Resolving list
            if (txAdd == LL_DEV_ADDR_TYPE_RANDOM   &&
                    (g_rx_adv_buf.data[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR)
            {
                bWlRlCheckOk = TRUE;

                if (g_llRlEnable == TRUE)
                {
                    bWlRlCheckOk = FALSE;
                    rpaListIndex = ll_getRPAListEntry(&g_rx_adv_buf.data[0]);

                    if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM)
                    {
                        // save resolved peer address
                        peerAddr = &g_llResolvinglist[rpaListIndex].peerAddr[0];
                        // if resolved address success, map the peer address type to 0x02 or 0x03
                        g_currentPeerAddrType = g_llResolvinglist[rpaListIndex].peerAddrType + 2;
                        osal_memcpy( &g_currentPeerRpa[0],  &g_rx_adv_buf.data[0], 6);   // save latest peer RPA
                        bWlRlCheckOk = TRUE;
                    }
                }
            }
            else   // InitA is device Identity, check whether the device Addr in the RPA list, if it is
            {
                // in the RPA list and network privacy mode is selected and non all-0 IRK, check failed
                bWlRlCheckOk = TRUE;

                for (int i = 0; i < LL_RESOLVINGLIST_ENTRY_NUM; i++)
                {
                    if (g_llResolvinglist[i].peerAddr[0] == g_rx_adv_buf.data[0]
                            && g_llResolvinglist[i].peerAddr[1] == g_rx_adv_buf.data[1]
                            && g_llResolvinglist[i].peerAddr[2] == g_rx_adv_buf.data[2]
                            && g_llResolvinglist[i].peerAddr[3] == g_rx_adv_buf.data[3]
                            && g_llResolvinglist[i].peerAddr[4] == g_rx_adv_buf.data[4]
                            && g_llResolvinglist[i].peerAddr[5] == g_rx_adv_buf.data[5]
                            && g_llResolvinglist[i].peerAddrType == txAdd)
                    {
                        if (g_llResolvinglist[i].privacyMode == NETWORK_PRIVACY_MODE &&
                                !ll_isIrkAllZero(g_llResolvinglist[i].peerIrk))
                            bWlRlCheckOk = FALSE;

                        break;
                    }
                }
            }

            // ====== check white list
            if ((pGlobal_config[LL_SWITCH] & LL_WHITELIST_ALLOW)
                    && (llState == LL_STATE_ADV_UNDIRECTED)
                    && (adv_param.wlPolicy   == LL_ADV_WL_POLICY_WL_CONNECT_REQ
                        || adv_param.wlPolicy  == LL_ADV_WL_POLICY_WL_ALL_REQ)
                    && (bWlRlCheckOk == TRUE))
            {
                // check white list
                bWlRlCheckOk = ll_isAddrInWhiteList(txAdd, peerAddr);
            }

            // fixed bug 2018-09-25, LL/CON/ADV/BV-04-C, for direct adv, initA should equal peer Addr
            if (llState == LL_STATE_ADV_DIRECTED)
            {
                if (//txAdd         != peerInfo.peerAddrType    // for (extended) set adv param, peer addr type could only be 0x0 or 0x01
                    peerAddr[0]  != peerInfo.peerAddr[0]
                    || peerAddr[1]  != peerInfo.peerAddr[1]
                    || peerAddr[2]  != peerInfo.peerAddr[2]
                    || peerAddr[3]  != peerInfo.peerAddr[3]
                    || peerAddr[4]  != peerInfo.peerAddr[4]
                    || peerAddr[5]  != peerInfo.peerAddr[5])
                {
                    bWlRlCheckOk = FALSE;
                }
            }

            if (bWlRlCheckOk == FALSE)   // if not in white list, do nothing
            {
                g_pmCounters.ll_filter_conn_req_cnt ++;
            }
            else
            {
                // increment statistics counter
                g_pmCounters.ll_rx_peer_cnt++;
                // bug fixed 2018-01-23, peerAddrType should read TxAdd
                peerInfo.peerAddrType = txAdd;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random
                osal_memcpy(peerInfo.peerAddr, &peerAddr[0], 6);
                move_to_slave_function();    // move to slave role for connection state
            }
        }
    }

    //test for fast adv
//      else //if(llState == LL_STATE_ADV_UNDIRECTED)
    if( adv_sch_flag )
    {
        // adv in next channel, or schedule next adv event
        uint8 i = 0;

        while (!(adv_param.advChanMap & (1 << i)))   i ++;    // get the 1st adv channel

        // adv_param.advNextChan stores the next adv channel, when adv the last adv channel, advNextChan should equal 1st adv channel
        if (adv_param.advNextChan != (LL_ADV_CHAN_FIRST + i))           // not finish adv the last channel, continue adv
        {
            llSetupSecAdvEvt();
        }
        else
        {
            if (llSecondaryState == LL_SEC_STATE_IDLE_PENDING)         // advertise last channel and transiting to IDLE
                llSecondaryState = LL_SEC_STATE_IDLE;
            else                                                       // otherwise, schedule next adv
                osal_start_timerEx(LL_TaskID, LL_EVT_SECONDARY_ADV, (adv_param.advInterval * 5) >> 3);   // * 625 / 1000
        }
    }

    // post ISR process
    if (!llWaitingIrq)                      // bug fixed 2018-05-04, only clear IRQ status when no config new one
        ll_hw_clr_irq();

    HAL_EXIT_CRITICAL_SECTION();
    return TRUE;
}

void LL_IRQHandler1(void)
{
    uint32         irq_status;
    int8 ret;
    ISR_entry_time = read_current_fine_time();
    //*(volatile uint32_t *)0x4000f0b8 = 1;  // pclk_clk_gate_en
    ll_debug_output(DEBUG_ISR_ENTRY);
    DBG_GPIO_WRITE(DBGIO_LL_IRQ,1);
    irq_status = ll_hw_get_irq_status();


    if (!(irq_status & LIRQ_MD))          // only process IRQ of MODE DONE
    {
        ll_hw_clr_irq();                  // clear irq status
        return;
    }

    llWaitingIrq = FALSE;

    if (llTaskState == LL_TASK_EXTENDED_ADV)
    {
        ret = ll_processExtAdvIRQ(irq_status);

        // TODO: consider whether need process secondary adv/scan here
        if (ret == TRUE)
            return;
    }
    else if (llTaskState == LL_TASK_EXTENDED_SCAN)
    {
        ret = ll_processExtScanIRQ(irq_status);

        // TODO: consider whether need process secondary adv/scan here
        if (ret == TRUE)
            return;
    }
    else if (llTaskState == LL_TASK_EXTENDED_INIT)
    {
        ret = ll_processExtInitIRQ(irq_status);

        // TODO: consider whether need process secondary adv/scan here
        if (ret == TRUE)
            return;
    }
    else if (llTaskState == LL_TASK_PERIODIC_ADV)
    {
        ret = ll_processPrdAdvIRQ(irq_status);

        // TODO: consider whether need process secondary adv/scan here
        if (ret == TRUE)
            return;
    }
    else if (llTaskState == LL_TASK_PERIODIC_SCAN)
    {
        ret = ll_processPrdScanIRQ(irq_status);

        // TODO: consider whether need process secondary adv/scan here
        if (ret == TRUE)
            return;
    }
    else
    {
        uint8         mode;
        mode = ll_hw_get_tr_mode();

        if(mode == LL_HW_MODE_SRX && (llState == LL_STATE_SCAN || llState == LL_STATE_INIT))
        {
            ret = ll_processBasicIRQ_SRX(irq_status);
        }
        else if((llSecondaryState == LL_SEC_STATE_ADV || llSecondaryState == LL_SEC_STATE_IDLE_PENDING)
                && (mode == LL_HW_MODE_TRX )
                && (adv_param.advEvtType == LL_ADV_CONNECTABLE_UNDIRECTED_EVT || adv_param.advEvtType == LL_ADV_SCANNABLE_UNDIRECTED_EVT))
        {
            // JIRA bugfix : BBBSDKREL-294
            ret = ll_processBasicIRQ_secondaryAdvTRX(irq_status);
        }
        else if (mode == LL_HW_MODE_TRX  &&
                 (llState == LL_STATE_SCAN))
        {
            ret = ll_processBasicIRQ_ScanTRX(irq_status);
        }
        else
        {
            ret = ll_processBasicIRQ(irq_status);
        }

        //test for fast adv
        if(     mode == LL_HW_MODE_TRX
                &&  llState == LL_STATE_ADV_UNDIRECTED
                && 0==(irq_status&LIRQ_COK) )
        {
            uint8_t firstAdvChan = (adv_param.advChanMap&LL_ADV_CHAN_37) !=0 ? 37 :
                                   (adv_param.advChanMap&LL_ADV_CHAN_38) !=0 ? 38 : 39;

            if(adv_param.advNextChan>firstAdvChan)
            {
                ll_schedule_next_event(50);         //20180623 modified by ZQ
            }
        }
    }

    // ================ Post ISR process: secondary pending state process
    // conn-adv case 2: other ISR, there is pending secondary advertise event, make it happen
    if (llSecondaryState == LL_SEC_STATE_ADV_PENDING)
    {
        if (llSecAdvAllow())    // for multi-connection case, it is possible still no enough time for adv
        {
            llSetupSecAdvEvt();
            ll_hw_set_rx_timeout(88);
            llSecondaryState = LL_SEC_STATE_ADV;
        }
    }
    // there is pending scan event, make it happen, note that it may stay pending if there is no enough idle time
    else if (llSecondaryState == LL_SEC_STATE_SCAN_PENDING)
    {
        // trigger scan
        llSetupSecScan(scanInfo.nextScanChan);
    }
    // there is pending init event, make it happen, note that it may stay pending if there is no enough idle time
    else if (llSecondaryState == LL_SEC_STATE_INIT_PENDING)
    {
        // trigger scan
        llSetupSecInit(initInfo.nextScanChan);
    }

    DBG_GPIO_WRITE(DBGIO_LL_IRQ,0);
    ll_debug_output(DEBUG_ISR_EXIT);
}

//--------------------------------------
extern uint32 llWaitingIrq;
extern uint32_t  g_wakeup_rtc_tick;

extern uint32 counter_tracking;
extern uint32_t g_counter_traking_avg;
extern uint32_t g_counter_traking_cnt;
extern uint32_t  g_TIM2_IRQ_TIM3_CurrCount;
extern uint32_t  g_TIM2_IRQ_to_Sleep_DeltTick;
extern uint32  read_ll_adv_remainder_time(void);
#define ROM_SLEEP_TICK   *(volatile uint32_t *)(0x1fff0a14)

__attribute__((weak)) void l2capPocessFragmentTxData(uint16 connHandle)
{
    //do nothing
}

#if 0
extern int m_in_critical_region;
int drv_disable_irq1(void)
{
    __disable_irq();
    DBG_GPIO_WRITE(DBGIO_DIS_IRQ,1);
    DBG_GPIO_WRITE(DBGIO_DIS_IRQ,0);
    m_in_critical_region++;
    return m_in_critical_region;
}

int drv_enable_irq1(void)
{
    m_in_critical_region--;

    if (m_in_critical_region == 0)
    {
        __enable_irq();
        DBG_GPIO_WRITE(DBGIO_EN_IRQ,1);
        DBG_GPIO_WRITE(DBGIO_EN_IRQ,0);
    }

    return m_in_critical_region;
}
extern void TIM1_IRQHandler(void);
void TIM1_IRQHandler1(void)
{
    gpio_write(P20,1);
    TIM1_IRQHandler();
    gpio_write(P20,0);
}
#endif

/*******************************************************************************
    @fn          ll_scheduler

    @brief       schedule next task, if current connection will be free, input
                parameter should be LL_INVALID_TIME. The function is invoked
                after old connection task end, it will not add new task but may
                delete exist task

    input parameters

    @param       time - schedule time for current connection

    output parameters

    @param       None.

    @return      None.
*/


void ll_scheduler1(uint32 time)
{
    uint32  T1, T2, delta, min, prio_adj;
    uint8   i, next, temp,conn_temp;
    T1 = read_current_fine_time();

    // timer1 is running, normally it should not occur
    if (isTimer1Running())
    {
        LOG("=== ASSERT FAIL, timer1 running when invoke ll_scheduler ===\n");
        g_pmCounters.ll_evt_shc_err++;
        return;
    }

    // if timer1 is not running, calculate the time elapse since last timer expiry
    delta = g_ll_conn_ctx.current_timer + LL_TIME_DELTA(g_ll_conn_ctx.timerExpiryTick, T1) + pGlobal_config[TIMER_ISR_ENTRY_TIME];
    // update current context
    g_ll_conn_ctx.scheduleInfo[g_ll_conn_ctx.currentConn].remainder = time;      // if current conn terminal, the parameter "time" shall be LL_INVALID_TIME
    min = time;

    if (time == LL_INVALID_TIME)
    {
        ll_deleteTask(g_ll_conn_ctx.currentConn);
        g_ll_conn_ctx.currentConn = LL_INVALID_CONNECTION_ID;
    }

    conn_temp = next = g_ll_conn_ctx.currentConn;

    if (next != LL_INVALID_CONNECTION_ID)
    {
        // if we want master or slave connection has higher schedule priority, set LL_MASTER_PREEMPHASIS/LL_SLAVE_PREEMPHASIS
        if (g_ll_conn_ctx.scheduleInfo[next].linkRole == LL_ROLE_MASTER)
            min = (time > pGlobal_config[LL_MULTICONN_MASTER_PREEMP]) ? (time - pGlobal_config[LL_MULTICONN_MASTER_PREEMP]) : 0;

        if (g_ll_conn_ctx.scheduleInfo[next].linkRole == LL_ROLE_SLAVE)
            min = (time > pGlobal_config[LL_MULTICONN_SLAVE_PREEMP]) ? (time - pGlobal_config[LL_MULTICONN_SLAVE_PREEMP]) : 0;
    }

    // update schedule task list and get the earliest task
    for (i = 0; i < g_maxConnNum; i++)
    {
        if ((i != g_ll_conn_ctx.currentConn) && conn_param[i].active)
        {
            // task conflict process
            // if there is no enough time for new task, invoke relate slave/master conn event process function
//              if (g_ll_conn_ctx.scheduleInfo[i].remainder < delta + g_ll_conn_ctx.scheduleInfo[i].task_duration)
            if (g_ll_conn_ctx.scheduleInfo[i].remainder < delta + 40)     // 40 : margin for process delay, unit: us
            {
                // no enough time to process the event, regard the event as missed and update the conn context and timer
                uint8  ret = LL_PROC_LINK_KEEP;

                if (g_ll_conn_ctx.scheduleInfo[i].linkRole == LL_ROLE_MASTER)
                {
                    // temporary update g_ll_conn_ctx.currentConn to current connection ID because
                    // ll_processMissMasterEvt will invoke function using global variable g_ll_conn_ctx.currentConn
                    temp = g_ll_conn_ctx.currentConn;
                    g_ll_conn_ctx.currentConn = i;
                    ret = ll_processMissMasterEvt(i);
//                  if( delta > g_ll_conn_ctx.scheduleInfo[i].remainder)
//                  {
//                      llConnState_t *connPtr = &conn_param[i];
//                      uint8 missCE = (( delta - g_ll_conn_ctx.scheduleInfo[i].remainder) / ( connPtr->curParam.connInterval*625 )) + 1;
//                      for(uint8 misI = 0;misI<missCE;misI++)
//                      {
//                          ret = ll_processMissMasterEvt(i);
////                            if( LL_PROC_LINK_TERMINATE == ret )
////                                break;
//                      }
//
//                  }
//                  else
//                      ret = ll_processMissMasterEvt(i);
                    g_ll_conn_ctx.currentConn = temp;
                }
                else if (g_ll_conn_ctx.scheduleInfo[i].linkRole == LL_ROLE_SLAVE)
                {
                    // temporary update g_ll_conn_ctx.currentConn to current connection ID because
                    // ll_processMissSlaveEvt will invoke function using global variable g_ll_conn_ctx.currentConn
                    temp = g_ll_conn_ctx.currentConn;
                    g_ll_conn_ctx.currentConn = i;

//                  ret = ll_processMissSlaveEvt(i);
                    if( delta > g_ll_conn_ctx.scheduleInfo[i].remainder)
                    {
                        llConnState_t* connPtr = &conn_param[i];
                        uint8 missCE = (( delta - g_ll_conn_ctx.scheduleInfo[i].remainder) / ( connPtr->curParam.connInterval*625 )) + 1;

                        for(uint8 misI = 0; misI<missCE; misI++)
                        {
                            ret = ll_processMissSlaveEvt(i);

                            if( LL_PROC_LINK_TERMINATE == ret )
                                break;
                        }
                    }
                    else
                        ret = ll_processMissSlaveEvt(i);

                    g_ll_conn_ctx.currentConn = temp;
                }

                if (ret == LL_PROC_LINK_TERMINATE)    // the connection is terminated, update shcedule information.
                {
                    ll_deleteTask(i);
                    continue;                  // continue next link
                }

                // increase the task priority
                g_ll_conn_ctx.scheduleInfo[i].priority ++;

                if (g_ll_conn_ctx.scheduleInfo[i].priority == LL_SCH_PRIO_LAST)
                    g_ll_conn_ctx.scheduleInfo[i].priority = LL_SCH_PRIO_IMMED;
            }

            prio_adj =0;

            if (min != LL_INVALID_TIME)
            {
                // consider the task prioriy
                switch (g_ll_conn_ctx.scheduleInfo[i].priority)
                {
                case LL_SCH_PRIO_LOW:
                    prio_adj = 0;
                    break;

                case LL_SCH_PRIO_MED:
                    prio_adj = MAX(LL_TASK_MASTER_DURATION, LL_TASK_SLAVE_DURATION) + 2000;
                    break;

                case LL_SCH_PRIO_HIGH:
                    prio_adj = MAX(LL_TASK_MASTER_DURATION, LL_TASK_SLAVE_DURATION) + 10 + 2000;
                    break;

                case LL_SCH_PRIO_IMMED:
                    prio_adj = MAX(LL_TASK_MASTER_DURATION, LL_TASK_SLAVE_DURATION) + 20 + 2000;
                    break;

                default:
                    prio_adj = 0;
                    break;
                }

                if (g_ll_conn_ctx.scheduleInfo[i].linkRole == LL_ROLE_MASTER)
                    prio_adj += pGlobal_config[LL_MULTICONN_MASTER_PREEMP];

                if (g_ll_conn_ctx.scheduleInfo[i].linkRole == LL_ROLE_SLAVE)
                    prio_adj += pGlobal_config[LL_MULTICONN_SLAVE_PREEMP];
            }

            // update remainder time
            g_ll_conn_ctx.scheduleInfo[i].remainder -= delta;

            if (g_ll_conn_ctx.scheduleInfo[i].remainder < min + prio_adj)
            {
                next = i;
                min = (g_ll_conn_ctx.scheduleInfo[i].remainder > prio_adj) ? (g_ll_conn_ctx.scheduleInfo[i].remainder - prio_adj) : 0;
            }
        }
    }

    if (min == LL_INVALID_TIME)     // all task may be delete, not start timer
    {
        return;
    }

    T2 = read_current_fine_time();
    // calculate the time elapse since enter this function.
    delta = LL_TIME_DELTA(T1, T2);
    _HAL_CS_ALLOC_();
    HAL_ENTER_CRITICAL_SECTION();
    uint8 rem_l_delta_flag = FALSE;
    uint8 rem_l_delta_value = 0;

    if (g_ll_conn_ctx.scheduleInfo[next].remainder <= delta)          // TODO: should not go here, if this issue detected, root cause should be invest
    {
//      set_timer1(20);
        set_timer(AP_TIM1,20);
        g_ll_conn_ctx.current_timer = 20;
        rem_l_delta_flag = TRUE;
        rem_l_delta_value = next;
//      LOG("-T %d:20,",next);
    }
    else
    {
//      set_timer1(g_ll_conn_ctx.scheduleInfo[next].remainder - delta);
        set_timer(AP_TIM1,g_ll_conn_ctx.scheduleInfo[next].remainder - delta);
        //logx("-S%d,%d\n",next,g_ll_conn_ctx.scheduleInfo[next].remainder - delta);
        
        // update connection context & schedule info
        g_ll_conn_ctx.current_timer = g_ll_conn_ctx.scheduleInfo[next].remainder - delta;
    }

    g_ll_conn_ctx.currentConn = next;

    // set ll state according to current connection LL state
    if (g_ll_conn_ctx.scheduleInfo[g_ll_conn_ctx.currentConn].linkRole == LL_ROLE_SLAVE)
        llState = LL_STATE_CONN_SLAVE;
    else if (g_ll_conn_ctx.scheduleInfo[g_ll_conn_ctx.currentConn].linkRole == LL_ROLE_MASTER)
        llState = LL_STATE_CONN_MASTER;

    // the task is scheduled, set the priority as low
    g_ll_conn_ctx.scheduleInfo[g_ll_conn_ctx.currentConn].priority = LL_SCH_PRIO_LOW;

    // take into account the time between start timer1 and T1
    for (i = 0; i < g_maxConnNum; i++)
    {
        if (conn_param[i].active)
        {
//          if( g_ll_conn_ctx.scheduleInfo[i].remainder >= delta )
//              g_ll_conn_ctx.scheduleInfo[i].remainder -= delta;
            if( ( g_ll_conn_ctx.scheduleInfo[i].remainder < delta ) && ( rem_l_delta_flag == FALSE))
            {
                if (g_ll_conn_ctx.scheduleInfo[i].linkRole == LL_ROLE_MASTER)
                    ll_processMissMasterEvt(i);
                else
                    ll_processMissSlaveEvt(i);
            }

            if( ( rem_l_delta_value == i ) && ( rem_l_delta_flag == TRUE) )
                g_ll_conn_ctx.scheduleInfo[i].remainder = 0;
            else
                g_ll_conn_ctx.scheduleInfo[i].remainder -= delta;

            conn_param[i].llTbd2 = g_ll_conn_ctx.scheduleInfo[i].remainder;
            /*record if error scheduler time*/
            // if( g_ll_conn_ctx.scheduleInfo[i].remainder > 500000)
            //     llConnTerminate(&conn_param[i],LL_SUPERVISION_TIMEOUT_TERM);
        }
    }

    // add for co-master intv bug fix
    if( g_ll_conn_ctx.scheduleInfo[conn_temp].linkRole != LL_ROLE_MASTER )
    {
        HAL_EXIT_CRITICAL_SECTION();
        return;
    }

    int8 k=0;

    for (k = g_maxConnNum-1; k >= 0; k--)
    {
        if ((conn_param[k].active) && (g_ll_conn_ctx.scheduleInfo[k].linkRole == LL_ROLE_MASTER ))
        {
            break;
        }
    }

    i=k;

    if( conn_temp == i  )
    {
        uint8 jm=i;
        uint8 fist_m=0;
        // current master --> first master true value
        uint32 tv_Masters = 0,tv_diff = 0,first_reminder = 0;

        for (i = 0; i < g_maxConnNum; i++)
        {
            if ((conn_param[i].active) && (g_ll_conn_ctx.scheduleInfo[i].linkRole == LL_ROLE_MASTER ))
                break;
        }

        first_reminder = g_ll_conn_ctx.scheduleInfo[i].remainder;
        fist_m = i;

        for (i=fist_m+1; i < jm+1 ; i++)
        {
            if ((conn_param[i].active) && (g_ll_conn_ctx.scheduleInfo[i].linkRole == LL_ROLE_MASTER ))
            {
                tv_Masters =  first_reminder + g_ll_conn_ctx.per_slot_time * 625 * (i - fist_m);

                if( tv_Masters > g_ll_conn_ctx.scheduleInfo[i].remainder)
                    tv_diff = tv_Masters - g_ll_conn_ctx.scheduleInfo[i].remainder;
                else
                    tv_diff = g_ll_conn_ctx.scheduleInfo[i].remainder - tv_Masters;

                // < 1000 : filter scecondary first create master connection & miss process master event
                if(tv_diff < 1000)
                {
                    if( g_ll_conn_ctx.scheduleInfo[i].remainder > tv_Masters )
                    {
                        g_ll_conn_ctx.scheduleInfo[i].remainder -= tv_diff;
                    }
                    else if( g_ll_conn_ctx.scheduleInfo[i].remainder < tv_Masters )
                    {
                        g_ll_conn_ctx.scheduleInfo[i].remainder += tv_diff;
                    }
                }
            }
        }
    }

    HAL_EXIT_CRITICAL_SECTION();
}

#define   CRY32_2_CYCLE_16MHZ_CYCLE_MAX    (976 + 98)     // tracking value range std +/- 20%
#define   CRY32_2_CYCLE_16MHZ_CYCLE_MIN    (976 - 98)
#define   CRY32_2_CYCLE_DELTA_LMT          (19)

uint32_t g_xtal16M_tmp=0;
static void check_16MXtal_by_rcTracking(void)
{
    /*
        for fiset wakeupini, not do rcCal, just skip the rcTacking

    */
    if(g_rc32kCalRes==0xFF)
    {
        WaitRTCCount(60);
        return;
    }

    uint32_t temp,temp1;
    uint32_t temp31,temp32,temp33;
    uint32_t temp_min,temp_max;
    // ======== enable tracking 32KHz RC timer with 16MHz crystal clock
    temp = *(volatile uint32_t*)0x4000f040;
    *(volatile uint32_t*)0x4000f040 = temp | BIT(18);
    temp = *(volatile uint32_t*)0x4000f05C;
    *(volatile uint32_t*)0x4000f05C = (temp & 0xfffefe00) | 0x0028;
    WaitRTCCount(3);
    temp31 = (*(volatile uint32_t*)0x4000f064 & 0x1ffff);
    WaitRTCCount(3);
    temp32 = (*(volatile uint32_t*)0x4000f064 & 0x1ffff);
    WaitRTCCount(3);
    temp33 = (*(volatile uint32_t*)0x4000f064 & 0x1ffff);

    while(1)
    {
        temp_min = (temp31 >=temp32) ? (temp32):(temp31);
        temp_min = (temp_min >=temp33) ? (temp33):(temp_min);
        temp_max = (temp31 >=temp32) ? (temp31):(temp32);
        temp_max = (temp_max >=temp33) ? (temp_max):(temp33);

        if( temp31>CRY32_2_CYCLE_16MHZ_CYCLE_MIN  &&
                temp31<CRY32_2_CYCLE_16MHZ_CYCLE_MAX  &&
                temp32 >CRY32_2_CYCLE_16MHZ_CYCLE_MIN  &&
                temp32 <CRY32_2_CYCLE_16MHZ_CYCLE_MAX  &&
                temp33 >CRY32_2_CYCLE_16MHZ_CYCLE_MIN  &&
                temp33 <CRY32_2_CYCLE_16MHZ_CYCLE_MAX  &&
                (temp_max-temp_min)<CRY32_2_CYCLE_DELTA_LMT
          )
        {
            break;
        }

        temp31= temp32;
        temp32 = temp33;
        WaitRTCCount(3);
        temp33 = (*(volatile uint32_t*)0x4000f064 & 0x1ffff);
    }

    WaitRTCCount(20);
    temp1 = (*(volatile uint32_t*)0x4000f064 & 0x1ffff);
    //disable tracking
    subWriteReg(0x4000f05C,3,3,0);
    g_xtal16M_tmp = temp1;
}

#define   TRACKING_96M_16M_MULTI6_DELTA_LIMIT       (10*6)            //96M:16M*6 +- 1%
#define   DLL_ENABLE_MAX                          (5)

uint32_t g_xtal96M_temp=0;
uint32_t DLL_enable_num=1;
static void check_96MXtal_by_rcTracking(void)
{
    uint32_t temp,temp1;

    //for first wakeupinit
    if(g_rc32kCalRes==0xFF)
    {
        //enable DLL
        temp = *(volatile uint32_t*)0x4000f044;
        *(volatile uint32_t*)0x4000f044 = temp | BIT(7);
        WaitRTCCount(3);
        return;
    }

    DLL_enable_num=0;
    // ======== enable tracking 32KHz RC timer with 16MHz crystal clock
    temp = *(volatile uint32_t*)0x4000f040;
    *(volatile uint32_t*)0x4000f040 = temp | BIT(18);

    while(1)
    {
        //enable DLL
        temp = *(volatile uint32_t*)0x4000f044;
        *(volatile uint32_t*)0x4000f044 = temp | BIT(7);
        WaitRTCCount(3);
        DLL_enable_num++;
        // gpio_write(P32,1);
        // gpio_write(P32,0);
        // //enable digclk 96M
        temp = *(volatile uint32_t*)0x4000f044;
        *(volatile uint32_t*)0x4000f044 = temp | BIT(16);

        for(uint8 index=0; index<5; index++)
        {
            temp = *(volatile uint32_t*)0x4000f05C;
            *(volatile uint32_t*)0x4000f05C = (temp & 0xfffefe00) | 0x0028 | BIT(16);
            WaitRTCCount(3);
            temp1 = (*(volatile uint32_t*)0x4000f064 & 0x1ffff);
            subWriteReg(0x4000f05C,3,3,0);

            if( (g_xtal16M_tmp*6 >=temp1 ? (g_xtal16M_tmp*6 -temp1):(temp1-g_xtal16M_tmp*6))<TRACKING_96M_16M_MULTI6_DELTA_LIMIT)
            {
                //disable 96M
                subWriteReg(0x4000f05C,16,16,0);
                subWriteReg(0x4000f044,16,16,0);
                g_xtal96M_temp = temp1;
                return;
            }
        }

        //disable 96M
        subWriteReg(0x4000f05C,16,16,0);
        subWriteReg(0x4000f044,16,16,0);

        //should not be here
        if(DLL_enable_num>= DLL_ENABLE_MAX)
        {
        	while(1);
            //NVIC_SystemReset();
        }

        //disable DLL
        subWriteReg(0x4000f044,7,7,0);
        WaitRTCCount(3);
        //update g_xtal16M_tmp
        temp = *(volatile uint32_t*)0x4000f05C;
        *(volatile uint32_t*)0x4000f05C = (temp & 0xfffefe00) | 0x0028 ;
        WaitRTCCount(3);
        g_xtal16M_tmp = (*(volatile uint32_t*)0x4000f064 & 0x1ffff);
        subWriteReg(0x4000f05C,3,3,0);
    }
}

#if 0
    uint32_t rtcCntTemp[10];
#endif
// now we split the initial fucntion to 3 kinds:
//    1. boot init function: which should be init when system boot. note: not include wakeup init function
//    2. wakeup init function: which should be init when wakeup from system sleep
//    3. parameter which should be init in APP, include: RF, board, ...
//  summary:
//      - normal boot, need: 1 + 2 + 3
//      - wakeup, need:       2 + 3

// init paramaters every time wakeup

uint32_t tracking_cnt=0;
void wakeup_init1()
{
    uint8_t pktFmt = 1;    // packet format 1: BLE 1M
    uint32  temp;
    //int int_state;
    // =========== clk gate for low power
    //*(volatile uint32_t *) 0x40000008 = 0x01e92190;
    // enable rng analog block. RNG analog need > 200us before stable, and it consume few current, so open it at wakeup
    //*(volatile uint32_t *) 0x4000f048 |= 1 << 23;
    // =========== config PCRM
//  *(volatile uint32_t *) 0x4000f040 = 0x501fb000; //enable xtal out
//  *(volatile uint32_t *) 0x4000f044 = 0x01ade8b0; //switch rf,adc to doubler,32M
//---by ZQ  2017-10-17
    //*(volatile uint32_t *) 0x4000f040 = 0x501fb820;  // enable xtal out
    // set the xtal cap to zero for faster settle
    // set [16] manually enable ac strigger f 20180613 by ZQ
    //*(volatile uint32_t *) 0x4000f044 = 0x01bdf8b0;//0x01bef830;  // switch rf,adc to doubler, dll_off, dll_ldo on
    // dll will be turn on in rf_ini after xtal settle
    //*(volatile uint32_t *) 0x4000f044 = 0x00be0830;  //[26:22] 0x02,[21:18]0x0f,[16:12]0x00,[7:4]0x03
    //< 22>:sel_rf_clk_16M;
    //< 23>:sel_rf_dbl_clk_32M;
    //< 24>:sel_rxadc_dbl_clk_32M;
    //< 25>:sel_rxadc_dbl_clk_32M_polarity;
    //< 26>:sel_rf_dbl_clk_32M_polarity
    // < 18>:en_rf_clk;
    // < 19>:en_rxadc_clk_32M;
    // < 20>:sel_cp_clk_32M;
    // < 21>:sel_dig_dble_clk_32M;
    // < 12>:en_cp_dll_clk;
    // < 13>:en_dig_clk_32M;
    // < 14>:en_dig_clk_48M;
    // < 15>:en_dig_clk_64M;
    // < 16>:en_dig_clk_96M;
    #if (DBG_BUILD_LL_TIMING)
    //====== for timing debug============
    gpio_write(DBG_PIN_SYS_CLK_SWITCH, 1);
    gpio_write(DBG_PIN_SYS_CLK_SWITCH, 0);
    //PHY_REG_WT(AP_IOMUX_BASE+8,1);//en debugMux[0]
    #endif
    //each rtc count is about 30.5us
    //after 15count , xtal will be feedout to dll and doubler
    //WaitRTCCount(pGlobal_config[WAKEUP_DELAY]);
    #if 0
    volatile uint32_t delay=0;

    for(uint8_t i=0; i<10; i++)
    {
        delay=500;
        rtcCntTemp[i]=rtc_get_counter();

        while(delay -- > 0) {};
    }

    #endif

    if(g_system_clk == SYS_CLK_XTAL_16M)
    {
        WaitRTCCount(pGlobal_config[WAKEUP_DELAY]);
    }
    else
    {
        uint32_t tracking_c1,tracking_c2;
        tracking_c1 = rtc_get_counter();
        check_16MXtal_by_rcTracking();
        check_96MXtal_by_rcTracking();
        tracking_c2 = rtc_get_counter();
        tracking_cnt = (tracking_c2>=tracking_c1) ? (tracking_c2-tracking_c1) : (0xffffffff-tracking_c1+tracking_c2);
        pGlobal_config[WAKEUP_ADVANCE] =1650+30*tracking_cnt;
    }

    // ============ config BB Top
    *(volatile uint32_t*) 0x40030000 = 0x3d068001;  // set tx pkt =2
    *(volatile uint32_t*) 0x400300bc = 0x834;       //[7:0] pll_tm [11:8] rxafe settle
    *(volatile uint32_t*) 0x400300a4 = 0x140;       //[6] for tpm_en
    clk_init(g_system_clk);
    // ================= clock selection
    // hclk_sel select hclk source. 0---rc 32m    1----dll 32m  2---xtal 16m   3---dll 48m  4----dll 64m   5----dll 96m
//    switch (pGlobal_config[CLOCK_SETTING])
//    {
//        case SYS_CLK_XTAL_16M:
////            *(int *) 0x4000f03C = 0x18001;                  // clock selection
//            *(int *) 0x4000f03C = 0x10002;                    // clock selection
//        break;
//        case SYS_CLK_DBL_32M:
//        case SYS_CLK_DLL_32M:
//            *(int *) 0x4000f03C = 0x10001;                    // clock selection
//        break;
//        case SYS_CLK_DLL_48M:
//            *(int *) 0x4000f03C = 0x10003;                    // clock selection
//        break;
//        case SYS_CLK_DLL_64M:
//            *(int *) 0x4000f03C = 0x10004;                    // clock selection
//        break;
//        case SYS_CLK_DLL_96M:
//            *(int *) 0x4000f03C = 0x10005;                    // clock selection
//        break;
//        default:
//            *(int *) 0x4000f03C = 0x10002;                    // clock selection
//        break;
//    }
    // ========== init timers
    set_timer(AP_TIM2, 625);      // OSAL 625us tick
    set_timer(AP_TIM3, BASE_TIME_UNITS);   // 1s timer
    // =========== open interrupt mask
    //int_state = 0x14;
    //set_int(int_state);
    //should use NVIC_EnableIRQn()
    NVIC_EnableIRQ(BB_IRQn);
    NVIC_EnableIRQ(TIM1_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM4_IRQn);
    // =========== ll HW setting
    set_max_length(0xff);
    ll_hw_set_empty_head(0x0001);
    //time related setting
    ll_hw_set_rx_timeout_1st(   500);
    ll_hw_set_rx_timeout(        88);       //ZQ 20180606, reduce rx timeout for power saving
    //preamble + syncword=40us, sync process = 8us
    //timeout should be larger then 48us,
    //ll_hw_set_rx_timeout(       268);     //for ble shoulde be larger than 80+128. if sync, the timeout timer stop.
    // (80 + 128) - BLE 5.0 preamble + access time, 60 for HW process delay
    // this time doesn't consider HW startup time, it is set in other regs
    ll_hw_set_loop_timeout(   30000);
//      ll_hw_set_tx_rx_release (10,     1);
//      ll_hw_set_rx_tx_interval(       57);        //T_IFS=150us for BLE 1M
//      ll_hw_set_tx_rx_interval(       65);        //T_IFS=150us for BLE 1M
//      ll_hw_set_trx_settle    (57, 8, 52);        //TxBB,RxAFE,PLL
    ll_hw_set_timing(pktFmt);
    ll_hw_ign_rfifo(LL_HW_IGN_SSN | LL_HW_IGN_CRC | LL_HW_IGN_EMP);
    // ======== enable tracking 32KHz RC timer with 16MHz crystal clock
    temp = *(volatile uint32_t*)0x4000f05C;
    *(volatile uint32_t*)0x4000f05C = (temp & 0xfffefe00) | 0x0108;  //[16] 16M [8:4] cnt [3] track_en_rc32k
    //get wakeup tracking counter
    // if (pGlobal_config[LL_SWITCH] & RC32_TRACKINK_ALLOW)
    // {
    //     WaitRTCCount(17);
    //     uint32_t  counter_tracking_wakeup = *(volatile uint32_t *)0x4000f064 & 0x1ffff;
    //     counter_tracking = (counter_tracking_wakeup + counter_tracking)>>1;
    // }
}

void config_RTC1(uint32 time)
{
//    *((volatile uint32_t *)(0xe000e100)) |= INT_BIT_RTC;   // remove, we don't use RTC interrupt
     //align to rtc clock edge
    WaitRTCCount(1);

    //update for cal ll next time after wakeup
    ll_remain_time = read_LL_remainder_time();

    // comparator configuration
    sleep_tick = *(volatile uint32_t *) 0x4000f028;         // read current RTC counter
    g_TIM2_IRQ_to_Sleep_DeltTick = (g_TIM2_IRQ_TIM3_CurrCount>(AP_TIM3->CurrentCount))
                                    ? (g_TIM2_IRQ_TIM3_CurrCount-(AP_TIM3->CurrentCount)): 0;

    AP_AON->RTCCC0 = sleep_tick + time;  //set RTC comparatr0 value

//	*(volatile uint32_t *) 0x4000f024 |= 1 << 20;           //enable comparator0 envent
//	*(volatile uint32_t *) 0x4000f024 |= 1 << 18;           //counter overflow interrupt
//	*(volatile uint32_t *) 0x4000f024 |= 1 << 15;           //enable comparator0 inerrupt

    //*(volatile uint32_t *) 0x4000f024 |= 0x148000;          // combine above 3 statement to save MCU time
    AP_AON->RTCCTL |= BIT(15)|BIT(18)|BIT(20);

    //compensate for cal wakeup next_time
    if (llState != LL_STATE_IDLE)
    {
        if(g_system_clk == SYS_CLK_XTAL_16M)
        {
            ll_remain_time -= 15;
        }
        else if(g_system_clk == SYS_CLK_DLL_48M)
        {
            ll_remain_time -= 5;
        }
        else
        {
            ll_remain_time -= 3;
        }
    }
}

#if 0
/*******************************************************************************
    @fn          wakeupProcess1

    @brief       wakeup from system sleep process function.


    input parameters

    @param       None

    output parameters

    @param       None.

    @return      None.
*/
void wakeupProcess1(void)
{
    uint32 current_RTC_tick;
    uint32 wakeup_time, wakeup_time0, next_time;
    uint32 sleep_total;
    uint32 dlt_tick;
    //restore initial_sp according to the app_initial_sp : 20180706 ZQ
    __set_MSP(pGlobal_config[INITIAL_STACK_PTR]);
    HAL_CRITICAL_SECTION_INIT();

    //====  20180416 commented by ZQ
    //      to enable flash access after wakeup
    //      current consumption has been checked. No big different
    //rom_set_flash_deep_sleep();

    //=======fix sram_rent issue 20180323
    //hal_pwrmgr_RAM_retention_clr();
    //subWriteReg(0x4000f01c,21,17,0);

    if (sleep_flag != SLEEP_MAGIC)
    {
        // enter this branch not in sleep/wakeup scenario
        set_sleep_flag(0);
        // software reset
        *(volatile uint32*)0x40000010 &= ~0x2;     // bit 1: M0 cpu reset pulse, bit 0: M0 system reset pulse.
    }

    // restore HW registers
    wakeup_init0();
    //TODO:Move to wakeup_init0
    NVIC_SetPriority((IRQn_Type)BB_IRQn,    IRQ_PRIO_REALTIME);
    NVIC_SetPriority((IRQn_Type)TIM1_IRQn,  IRQ_PRIO_HIGH);     //ll_EVT
    NVIC_SetPriority((IRQn_Type)TIM2_IRQn,  IRQ_PRIO_HIGH);     //OSAL_TICK
    NVIC_SetPriority((IRQn_Type)TIM4_IRQn,  IRQ_PRIO_HIGH);     //LL_EXA_ADV
    //wakeup flash
    spif_release_deep_sleep();
    //===20180417 added by ZQ
    //  could be move into wakeup_init
    //  add the patch entry for tx2rx/rx2tx interval config
    //2018-11-10 by ZQ
    //config the tx2rx timing according to the g_rfPhyPktFmt
    ll_hw_tx2rx_timing_config(g_rfPhyPktFmt);
    // 20200812 ZQ
    // DO NOT Turn OFF 32K Xtal
    // if (pGlobal_config[LL_SWITCH] & LL_RC32K_SEL)
    // {
    //     subWriteReg(0x4000f01c,16,7,0x3fb);   //software control 32k_clk
    //     subWriteReg(0x4000f01c,6,6 ,0x01);    //enable software control
    // }
    // else
    // {
    //     subWriteReg(0x4000f01c,9,8,0x03);   //software control 32k_clk
    //     subWriteReg(0x4000f01c,6,6,0x00);   //disable software control
    // }
    //20181201 by ZQ
    //restart the TIM2 to align the RTC
    //----------------------------------------------------------
    //stop the 625 timer
    AP_TIM2->ControlReg=0x0;
    AP_TIM2->ControlReg=0x2;
    AP_TIM2->LoadCount = 2500;
    //----------------------------------------------------------
    //wait rtc cnt change
    WaitRTCCount(1);
    //----------------------------------------------------------
    //restart the 625 timer
    AP_TIM2->ControlReg=0x3;
    current_RTC_tick = rtc_get_counter();
    //g_TIM2_wakeup_delay= (AP_TIM2->CurrentCount)+12; //12 is used to align the rtc_tick
    wakeup_time0 = read_current_fine_time();
    g_wakeup_rtc_tick = rtc_get_counter();
    // rf initial entry, will be set in app
    rf_phy_ini();

    if(current_RTC_tick>sleep_tick)
    {
        dlt_tick = current_RTC_tick - sleep_tick;
    }
    else
    {
        //dlt_tick = current_RTC_tick+0x00ffffff - sleep_tick;
        dlt_tick = (0xffffffff - sleep_tick)+current_RTC_tick;
    }

    //dlt_tick should not over 24bit
    //otherwise, sleep_total will overflow !!!
    if(dlt_tick>0x3fffff)
        dlt_tick &=0x3fffff;

    if (pGlobal_config[LL_SWITCH] & RC32_TRACKINK_ALLOW)
    {
        //sleep_total = ((current_RTC_tick - sleep_tick) * counter_tracking) >> 7; // shift 4 for 16MHz -> 1MHz, shift 3 for we count 8 RTC tick
        // sleep_total =   ((((dlt_tick &0xffff0000)>>16)*counter_tracking)<<9)
        //                 + (((dlt_tick &0xffff)*counter_tracking)>>7);
        //counter_tracking default 16 cycle
        sleep_total =   ((((dlt_tick &0xffff0000)>>16)*counter_tracking)<<8)
                        + (((dlt_tick &0xffff)*counter_tracking)>>8);
    }
    else
    {
        // time = tick * 1000 0000 / f (us). f = 32000Hz for RC, f = 32768Hz for crystal. We also calibrate 32KHz RC to 32768Hz
        //sleep_total =  ((current_RTC_tick - sleep_tick) * TIMER_TO_32K_CRYSTAL) >> 2;
        //fix sleep timing error
        sleep_total = ( ( (dlt_tick<<7)-(dlt_tick<<2)-(dlt_tick<<1) +2)       >>2 )   /* dlt_tick * (128-4-2)/4 */
                      +( ( (dlt_tick<<3)+ dlt_tick +128)                       >>9 ) ; /* dlt_tick *9/512 */
        //+2,+128 for zero-mean quanization noise
    }

    // restore systick
    g_osal_tick_trim = (pGlobal_config[OSAL_SYS_TICK_WAKEUP_TRIM]+g_TIM2_IRQ_to_Sleep_DeltTick+2500-g_TIM2_IRQ_PendingTick)>>2;        //16 is used to compensate the cal delay
    g_osalTickTrim_mod+=(pGlobal_config[OSAL_SYS_TICK_WAKEUP_TRIM]+g_TIM2_IRQ_to_Sleep_DeltTick+2500-g_TIM2_IRQ_PendingTick)&0x03;     //16 is used to compensate the cal delay

    if(g_osalTickTrim_mod>4)
    {
        g_osal_tick_trim+=1;
        g_osalTickTrim_mod = g_osalTickTrim_mod%4;
    }

    // restore systick
    osal_sys_tick += (sleep_total+g_osal_tick_trim) / 625;      // convert to 625us systick
    rtc_mod_value += ((sleep_total+g_osal_tick_trim)%625);

    if(rtc_mod_value > 625)
    {
        osal_sys_tick += 1;
        rtc_mod_value = rtc_mod_value%625;
    }

    osalTimeUpdate();

    // osal time update, not required. It will be updated when osal_run_system() is called after wakeup

    // TODO: should we consider widen the time drift window  ????

    //20190117 ZQ
    if(llState != LL_STATE_IDLE)
    {
        // SW delay
        wakeup_time = read_current_fine_time() - wakeup_time0;
        next_time = 0;

        if (ll_remain_time > sleep_total + wakeup_time)
        {
            next_time = ll_remain_time - sleep_total - wakeup_time;
            // restore LL timer
            set_timer(AP_TIM1, next_time);
        }
        else
        {
            // should not be here
            set_timer(AP_TIM1, 1000);
        }
    }

    if (g_llSleepContext.isTimer4RecoverRequired)
    {
        // SW delay
        wakeup_time = read_current_fine_time() - wakeup_time0;
        next_time = 0;

        if (g_llSleepContext.timer4Remainder > sleep_total + wakeup_time)
        {
            next_time = g_llSleepContext.timer4Remainder - sleep_total - wakeup_time;
            // restore LL timer
            set_timer(AP_TIM4, next_time);
        }
        else
        {
            // should not be here
            set_timer(AP_TIM4, 1500);
            //  next_time = 0xffff;
        }

        g_llSleepContext.isTimer4RecoverRequired = FALSE;
    }

    // app could add operation after wakeup
    app_wakeup_process();
//    uart_tx0(" 111 ");
    ll_debug_output(DEBUG_WAKEUP);
    set_sleep_flag(0);
    // ==== measure value, from RTC counter meet comparator 0 -> here : 260us ~ 270us
    // start task loop
    osal_start_system();
}
void enter_sleep_off_mode1(Sleep_Mode mode)
{
    if(mode==SYSTEM_SLEEP_MODE)
        spif_set_deep_sleep();

    enter_sleep_off_mode0(mode);
}
#endif
void LL_ENC_AES128_Encrypt1( uint8* key,
                             uint8* plaintext,
                             uint8* ciphertext )
{
    //only turn on while working
    AP_PCR->SW_CLK    |= BIT(MOD_AES);
    LL_ENC_AES128_Encrypt0(key,plaintext,ciphertext);
    AP_PCR->SW_CLK    &= ~BIT(MOD_AES);
}

#define LL_ENC_BASE         0x40040000               // LL HW AES engine Base address

#define LL_ENC_ENCRYPT_DONE_MASK        0x0001
#define LL_ENC_DECRYPT_FAIL_MASK        0x0002
#define LL_ENC_DECRYPT_SUCC_MASK        0x0004
#define LL_ENC_SINGLE_MODE_DONE_MASK    0x0008

extern void LL_ENC_LoadKey( uint8* key );
void  LL_ENC_Encrypt1( llConnState_t* connPtr, uint8 pktHdr, uint8 pktLen, uint8* pBuf )
{
    AP_PCR->SW_CLK    |= BIT(MOD_AES);
//    LL_ENC_Encrypt0(connPtr,  pktHdr,  pktLen, pBuf );
    {
        uint8* pByte = NULL;
        uint16 index;
        int i, len;
        uint32_t temp;
        // disable AES
        *(int*) 0x40040000 = 0x0;
        // Load Key
        // Note: Normally this would only need to be done once when the SK is derived
        //       from the LTK and SKD. However, when in sleep, the AES block loses
        //       this key. Also, when multiple connections are supported, the key
        //       will be different.
        LL_ENC_LoadKey( connPtr->encInfo.SK );

//      if ( llState == LL_STATE_CONN_MASTER )
        if( connPtr->llTbd1 == LL_LINK_CONNECT_COMPLETE_MASTER )
        {
            // generate the nonce based on packet count, IV, and direction
            LL_ENC_GenerateNonce( connPtr->encInfo.txPktCount,
                                  LL_ENC_TX_DIRECTION_MASTER,
                                  connPtr->encInfo.nonce );
        }
        else // assumed llState == LL_STATE_CONN_SLAVE
        {
            // generate the nonce based on packet count, IV, and direction
            LL_ENC_GenerateNonce( connPtr->encInfo.txPktCount,
                                  LL_ENC_TX_DIRECTION_SLAVE,
                                  connPtr->encInfo.nonce );
        }

        // confiig nounce
        pByte = connPtr->encInfo.nonce;
        *(volatile uint32_t*)(LL_ENC_BASE + 0x3c) = pByte[0] ;
        pByte ++;
        *(volatile uint32_t*)(LL_ENC_BASE + 0x38) = pByte[0] << 24 | pByte[1] << 16 | pByte[2] << 8 | pByte[3];
        pByte += 4;
        *(volatile uint32_t*)(LL_ENC_BASE + 0x34) = pByte[0] << 24 | pByte[1] << 16 | pByte[2] << 8 | pByte[3];
        pByte += 4;
        *(volatile uint32_t*)(LL_ENC_BASE + 0x30) = pByte[0] << 24 | pByte[1] << 16 | pByte[2] << 8 | pByte[3];
        // config plen & aad
        *(volatile uint32_t*)(LL_ENC_BASE + 0x0c) = (pktLen << 8) | pktHdr;
        // write packet to FIFO
        len = pktLen;
        index = 0;

        while (len >= 4)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index + 3] << 24 | pBuf[index + 2] << 16 |  pBuf[index + 1] << 8 | pBuf[index];
            index += 4;
            len -= 4;
        }

        // to check the byte order
        if(len == 3)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index + 2] << 16 |  pBuf[index + 1] << 8 | pBuf[index];
            index += 4;
        }
        else if(len == 2)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index + 1] << 8 | pBuf[index] ;
            index += 4;
        }
        else if(len == 1)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index] ;
            index += 4;
        }

        // AES FIFO legth is 256 bytes, set other bytes 0
        for (i = index; i < 0x100; i += 4)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + i) = 0x0;
        }

        // set AES ctrl reg
        *(int*) 0x40040004 = 0xf00;
        // set interrupt enable
        *(int*) 0x40040010 = 0xf;
        // enable AES
        *(int*) 0x40040000 = 0x1;

        // insert delay
        //    delay = 200;
        //    while (delay --);

        // query AES interrupt status register
        while (*(volatile uint32_t*)(LL_ENC_BASE + 0x0014) == 0) ;

        // disable AES, if not disable AES, there is no output in FIFO
        *(int*) 0x40040000 = 0x0;
        // read back the encrypt result
        index = 0;
        len = pktLen + 4;     // include 4 bytes MIC

        while (len > 0)
        {
            temp = *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index);
            pBuf[index ++] = temp & 0xff;
            pBuf[index ++] = (temp >> 8) & 0xff;
            pBuf[index ++] = (temp >> 16) & 0xff;
            pBuf[index ++] = (temp >> 24) & 0xff;
            len -= 4;
        }

        // up the count for the next TX'ed data packet
        // Note: This is supposed to be 39 bit counter, but for now, we don't
        //       envision receiving 550 billion packets during a connection!
        connPtr->encInfo.txPktCount++;
//      return;
    }
    AP_PCR->SW_CLK    &= ~BIT(MOD_AES);
}
uint8 LL_ENC_Decrypt1( llConnState_t* connPtr, uint8 pktHdr, uint8 pktLen, uint8* pBuf )
{
    AP_PCR->SW_CLK    |= BIT(MOD_AES);
//    uint8 ret = LL_ENC_Decrypt0( connPtr,  pktHdr,  pktLen, pBuf );
    {
        uint8* pByte = NULL;
        uint16 index;
        int i, len;
        uint32_t temp;
        // disable AES
        *(int*) 0x40040000 = 0x0;
        // Load Key
        // Note: Normally this would only need to be done once when the SK is derived
        //       from the LTK and SKD. However, when in sleep, the AES block loses
        //       this key. Also, when multiple connections are supported, the key
        //       will be different.
        LL_ENC_LoadKey( connPtr->encInfo.SK );

//    if ( llState == LL_STATE_CONN_MASTER )
        if( connPtr->llTbd1 == LL_LINK_CONNECT_COMPLETE_MASTER )
        {
            // generate the nonce based on packet count, IV, and direction
            LL_ENC_GenerateNonce( connPtr->encInfo.rxPktCount,
                                  LL_ENC_RX_DIRECTION_MASTER,
                                  connPtr->encInfo.nonce );
        }
        else // assumed llState == LL_STATE_CONN_SLAVE
        {
            // generate the nonce based on packet count, IV, and direction
            LL_ENC_GenerateNonce( connPtr->encInfo.rxPktCount,
                                  LL_ENC_RX_DIRECTION_SLAVE,
                                  connPtr->encInfo.nonce );
        }

        // confiig nounce
        pByte = connPtr->encInfo.nonce;
        *(volatile uint32_t*)(LL_ENC_BASE + 0x3c) = pByte[0]; // << 24 ;
        pByte ++;
        *(volatile uint32_t*)(LL_ENC_BASE + 0x38) = pByte[0] << 24 | pByte[1] << 16 | pByte[2] << 8 | pByte[3];
        pByte += 4;
        *(volatile uint32_t*)(LL_ENC_BASE + 0x34) = pByte[0] << 24 | pByte[1] << 16 | pByte[2] << 8 | pByte[3];
        pByte += 4;
        *(volatile uint32_t*)(LL_ENC_BASE + 0x30) = pByte[0] << 24 | pByte[1] << 16 | pByte[2] << 8 | pByte[3];
        // config plen & aad
        *(volatile uint32_t*)(LL_ENC_BASE + 0x0c) = (pktLen << 8) | pktHdr;
        // write packet to FIFO
        len = pktLen + 4;       // decrypt, add 4 for MIC field length
        index = 0;

        while (len >= 4)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index + 3] << 24 | pBuf[index + 2] << 16 |  pBuf[index + 1] << 8 | pBuf[index];
            index += 4;
            len -= 4;
        }

        // fill others bytes < 1 word
        if(len == 3)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index + 2] << 16 |  pBuf[index + 1] << 8 | pBuf[index];
            index += 4;
        }
        else if(len == 2)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index + 1] << 8 | pBuf[index] ;
            index += 4;
        }
        else if(len == 1)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index] ;
            index += 4;
        }

        // AES FIFO legth is 256 bytes, set other bytes 0
        for (i = index; i < 0x100; i += 4)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + i) = 0x0;
        }

        // set AES ctrl reg
        *(int*) 0x40040004 = 0xf08;
        // set interrupt enable
        *(int*) 0x40040010 = 0xf;
        // enable AES
        *(int*) 0x40040000 = 0x1;

        // insert delay
//    delay = 200;
//    while (delay --);

        // query AES interrupt status register and wait decrypt finish
        while (*(volatile uint32_t*)(LL_ENC_BASE + 0x0014) == 0) ;

        // read interrupt status reg
        temp = *(volatile uint32_t*)(LL_ENC_BASE + 0x0014);

        if ((temp & LL_ENC_DECRYPT_FAIL_MASK)
                || ((temp & LL_ENC_DECRYPT_SUCC_MASK) == 0))
        {
            AP_PCR->SW_CLK    &= ~BIT(MOD_AES);
            return FALSE;
        }

        // disable AES
        *(int*) 0x40040000 = 0x0;
        // read the decrypt result
        index = 0;
        len = pktLen;

        while (len > 0)
        {
            temp = *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index);
            pBuf[index ++] = temp & 0xff;
            pBuf[index ++] = (temp >> 8) & 0xff;
            pBuf[index ++] = (temp >> 16) & 0xff;
            pBuf[index ++] = (temp >> 24) & 0xff;
            len -= 4;
        }

        // up the count for the next RX'ed data packet
        // Note: This is supposed to be 39 bit counter, but for now, we don't
        //       envision receiving 550 billion packets during a connection!
        connPtr->encInfo.rxPktCount++;
        AP_PCR->SW_CLK    &= ~BIT(MOD_AES);
        return( TRUE );
    }
//    AP_PCR->SW_CLK    &= ~BIT(MOD_AES);
//    return ret;
}

//20200928 ZQ
//fix ADV_DIR_IND rxAdd setbit
llStatus_t LL_SetAdvParam1( uint16 advIntervalMin,
                            uint16 advIntervalMax,
                            uint8  advEvtType,
                            uint8  ownAddrType,
                            uint8  peerAddrType,
                            uint8*  peerAddr,
                            uint8  advChanMap,
                            uint8  advWlPolicy )
{
    uint8_t llState_reserve = llState;
    llStatus_t ret;
    ret=LL_SetAdvParam0(  advIntervalMin,
                          advIntervalMax,
                          advEvtType,
                          ownAddrType,
                          peerAddrType,
                          peerAddr,
                          advChanMap,
                          advWlPolicy );
    llState=llState_reserve;

    if(advEvtType==LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT
            || advEvtType==LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT)
    {
        SET_BITS(g_tx_adv_buf.txheader, peerInfo.peerAddrType, RX_ADD_SHIFT, RX_ADD_MASK);   // RxAdd need't set
    }

    return ret;
}

llStatus_t LL_SetAdvControl1( uint8 advMode )
{
    _HAL_CS_ALLOC_();
    //if random address isn't defined,can't set ownaddresstype to random
    if ((advMode)&&(((adv_param.ownAddrType == LL_DEV_ADDR_TYPE_RANDOM)    ||
                     (adv_param.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM))  &&
                    (  (ownRandomAddr[0] == 0xFF) &&
                       (ownRandomAddr[1] == 0xFF) &&
                       (ownRandomAddr[2] == 0xFF) &&
                       (ownRandomAddr[3] == 0xFF) &&
                       (ownRandomAddr[4] == 0xFF) &&
                       (ownRandomAddr[5] == 0xFF)  )))
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    if (g_llAdvMode == LL_MODE_EXTENDED )
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    g_llAdvMode = LL_MODE_LEGACY;

    // check if a direct test mode or modem test is in progress
    if ( (llState == LL_STATE_DIRECT_TEST_MODE_TX) ||
            (llState == LL_STATE_DIRECT_TEST_MODE_RX) ||
            (llState == LL_STATE_MODEM_TEST_TX)       ||
            (llState == LL_STATE_MODEM_TEST_RX)       ||
            (llState == LL_STATE_MODEM_TEST_TX_FREQ_HOPPING) )
    {
        return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );
    }

    // 2021-4-19, check init/scan state should not enable/disable adv
    if ( (llState == LL_STATE_SCAN) ||
            (llState == LL_STATE_INIT) )
    {
        return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );
    }

    // sanity checks again to be sure we don't start with bad parameters
    if ( ( (adv_param.advEvtType != LL_ADV_CONNECTABLE_UNDIRECTED_EVT)     &&
            (adv_param.advEvtType != LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT)   &&
            (adv_param.advEvtType != LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT)  &&
            (adv_param.advEvtType != LL_ADV_SCANNABLE_UNDIRECTED_EVT)       &&
            (adv_param.advEvtType != LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT) )        ||
            ( (adv_param.ownAddrType != LL_DEV_ADDR_TYPE_PUBLIC)              &&
              (adv_param.ownAddrType != LL_DEV_ADDR_TYPE_RANDOM)              &&
              (adv_param.ownAddrType != LL_DEV_ADDR_TYPE_RPA_PUBLIC)          &&
              (adv_param.ownAddrType != LL_DEV_ADDR_TYPE_RPA_RANDOM))                ||
            ( ((adv_param.advEvtType == LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT)        ||
               (adv_param.advEvtType == LL_ADV_SCANNABLE_UNDIRECTED_EVT))     &&
              (adv_param.advInterval < LL_ADV_CONN_INTERVAL_MIN) ) )     // should use LL_ADV_NONCONN_INTERVAL_MIN after update it to 20ms
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    #ifdef DEBUG_LL
    LOG("llState = %d\n", llState);
    #endif

    // check if we should begin advertising
    switch( advMode )
    {
    // Advertisment Mode is On
    case LL_ADV_MODE_ON:

        // check if command makes sense
        if ( adv_param.advMode == LL_ADV_MODE_ON )
        {
            // this is unexpected; something is wrong
            return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );
        }

        //add llState setting
        if((llState == LL_STATE_IDLE))
        {
            switch(adv_param .advEvtType)
            {
            case LL_ADV_CONNECTABLE_UNDIRECTED_EVT:
                llState=LL_STATE_ADV_UNDIRECTED;
                ll_debug_output(DEBUG_LL_STATE_ADV_UNDIRECTED);
                break;

            case LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT:
            case LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT:
                llState=LL_STATE_ADV_DIRECTED;
                ll_debug_output(DEBUG_LL_STATE_ADV_DIRECTED);
                break;

            case LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT:
                llState=LL_STATE_ADV_NONCONN;
                ll_debug_output(DEBUG_LL_STATE_ADV_NONCONN);
                break;

            case LL_ADV_SCANNABLE_UNDIRECTED_EVT:
                llState=LL_STATE_ADV_SCAN;
                ll_debug_output(DEBUG_LL_STATE_ADV_SCAN);
                break;

            default:
                llState=LL_STATE_IDLE;
                ll_debug_output(DEBUG_LL_STATE_IDLE);
                break;
            }
        }

        // llState changed when configure adv parameters
        if (llState == LL_STATE_ADV_UNDIRECTED
                || llState == LL_STATE_ADV_DIRECTED
                || llState == LL_STATE_ADV_NONCONN
                || llState == LL_STATE_ADV_SCAN )     // TODO: check this setting
        {
            g_llHdcDirAdvTime = 0;    // for HDC direct adv
            adv_param.advNextChan = LL_ADV_CHAN_LAST + 1;       // set adv channel invalid

            if ( llSetupAdv() != LL_STATUS_SUCCESS )
            {
                // indicate advertising is no longer active
                adv_param.advMode = LL_ADV_MODE_OFF;
                return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );
            }
        }
        // add in A2, simultaneous conn event & scan/adv event
        else if((llState == LL_STATE_CONN_SLAVE
                 || llState == LL_STATE_CONN_MASTER)
                && (pGlobal_config[LL_SWITCH] & SIMUL_CONN_ADV_ALLOW))
        {
            #ifdef DEBUG_LL
            LOG("LL_SetAdvControl: start sec adv\r\n");
            #endif

            if (llSecondaryState != LL_SEC_STATE_IDLE)
                return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );

            // adv event check
            if (adv_param.advEvtType  != LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT
                    && adv_param.advEvtType != LL_ADV_SCANNABLE_UNDIRECTED_EVT
                    && adv_param.advEvtType != LL_ADV_CONNECTABLE_UNDIRECTED_EVT)
                return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );

            // Note: we may need maximum slave number check here. If number of slave reach ceil,
            //       only no-connectable adv is allowed. The checking could be don't in host
            llSecondaryState = LL_SEC_STATE_ADV;
            adv_param.advNextChan = LL_ADV_CHAN_LAST + 1;        // set adv channel invalid
            osal_stop_timerEx( LL_TaskID, LL_EVT_SECONDARY_ADV );
            osal_set_event(LL_TaskID, LL_EVT_SECONDARY_ADV);     // set adv event
        }
        else           // other state
            return (LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE);

        // indicate advertising is no longer active
        adv_param.advMode = LL_ADV_MODE_ON;

        if (g_llRlDeviceNum > 0)
            osal_start_timerEx( LL_TaskID, LL_EVT_RPA_TIMEOUT, g_llRlTimeout * 1000 );

        break;

    case LL_ADV_MODE_OFF:
        // check if command makes sense
//            if ( adv_param.advMode == LL_ADV_MODE_OFF )
//            {
//                // this is unexpected; something is wrong
//                return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );
//            }
        HAL_ENTER_CRITICAL_SECTION();
        // free the associated task block
        //llFreeTask( &advInfo.llTask );
        // indicate we are no longer actively advertising
        adv_param.advMode = LL_ADV_MODE_OFF;

        if (llState != LL_STATE_CONN_SLAVE &&
                llState != LL_STATE_CONN_MASTER )      // no conn + adv case
        {
            llState = LL_STATE_IDLE;     // if not in connect state, set idle to disable advertise
            //ZQ 20190912
            //stop ll timer when idle, considering the scan-adv interleve case
            clear_timer(AP_TIM1);
            ll_debug_output(DEBUG_LL_STATE_IDLE);
        }

        if(llSecondaryState!=LL_SEC_STATE_IDLE)                       // conn + adv case
        {
//            uint8 i;
//            i = 0;
//            while (!(adv_param.advChanMap & (1 << i)))   i ++;    // get the 1st adv channel in the adv channel map
//            if ((llSecondaryState == LL_SEC_STATE_ADV)
//                    && (adv_param.advNextChan != (LL_ADV_CHAN_FIRST + i)))      // last adv event is not finished
//                llSecondaryState = LL_SEC_STATE_IDLE_PENDING;
//            else
            {
                llSecondaryState = LL_SEC_STATE_IDLE;
                osal_stop_timerEx( LL_TaskID, LL_EVT_SECONDARY_ADV );    // stop timer
            }
        }

        HAL_EXIT_CRITICAL_SECTION();
        osal_stop_timerEx(LL_TaskID, LL_EVT_RPA_TIMEOUT);
        break;

    default:
        // we have an invalid value for advertisement mode
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    return( LL_STATUS_SUCCESS );
}


#if 0
//2020.10.22,Jie,fix phyupdate issue
llStatus_t LL_PhyUpdate1( uint16 connId )
{
    llStatus_t    status;
    llConnState_t* connPtr;
    uint8 phyMode;

    // make sure connection ID is valid
    if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
    {
        return( status );
    }

    // get connection info
    connPtr = &conn_param[connId ];

    // check if an update control procedure is already pending
    if ( ((connPtr->ctrlPktInfo.ctrlPktCount > 0) &&
            (connPtr->ctrlPktInfo.ctrlPkts[0] == LL_CTRL_PHY_UPDATE_IND)) ||
            (connPtr->pendingPhyModeUpdate == TRUE) )
    {
        return( LL_STATUS_ERROR_CTRL_PROC_ALREADY_ACTIVE );
    }

    // we only support symmetric connection
    // tx rx phy should be same
    phyMode = connPtr->llPhyModeCtrl.req.txPhy & connPtr->llPhyModeCtrl.rsp.txPhy;
    phyMode &= connPtr->llPhyModeCtrl.req.rxPhy & connPtr->llPhyModeCtrl.rsp.rxPhy;

    //20200727 Jie add for no change case
    if((phyMode==0) || (phyMode == connPtr->llPhyModeCtrl.local.txPhy))
    {
        //no change case
        connPtr->phyUpdateInfo.m2sPhy = 0;
        connPtr->phyUpdateInfo.s2mPhy = 0;
    }
    else if((phyMode&LE_2M_PHY)&&(connPtr->llPhyModeCtrl.local.txPhy != LE_2M_PHY))
    {
        connPtr->phyUpdateInfo.m2sPhy = LE_2M_PHY;
        connPtr->phyUpdateInfo.s2mPhy = LE_2M_PHY;
    }
    else if((phyMode&LE_CODED_PHY)&&(connPtr->llPhyModeCtrl.local.txPhy != LE_CODED_PHY))
    {
        connPtr->phyUpdateInfo.m2sPhy = LE_CODED_PHY;
        connPtr->phyUpdateInfo.s2mPhy = LE_CODED_PHY;
    }
    else
    {
        //no perferce can not support the tx/rx same time
        connPtr->phyUpdateInfo.m2sPhy = LE_1M_PHY;
        connPtr->phyUpdateInfo.s2mPhy = LE_1M_PHY;
    }

    if(connPtr->phyUpdateInfo.m2sPhy==0)
    {
        connPtr->phyModeUpdateEvent = 0;
        connPtr->phyUpdateInfo.instant =   connPtr->phyModeUpdateEvent;
    }
    else
    {
        connPtr->phyModeUpdateEvent = (connPtr->curParam.slaveLatency+1) +
                                      LL_INSTANT_NUMBER_MIN;
        connPtr->phyUpdateInfo.instant =   connPtr->phyModeUpdateEvent;
    }

    // queue control packet for processing
    llEnqueueCtrlPkt( connPtr, LL_CTRL_PHY_UPDATE_IND );
    return( LL_STATUS_SUCCESS );
}
#endif

//2020.10.22,Jie,fix scanparam ownaddr setting issue
llStatus_t LL_SetScanParam1( uint8  scanType,
                             uint16 scanInterval,
                             uint16 scanWindow,
                             uint8  ownAddrType,
                             uint8  scanWlPolicy )
{
    llStatus_t ret;
    ret = LL_SetScanParam0(scanType,scanInterval,scanWindow,ownAddrType,scanWlPolicy);
//    LOG("%s,ret %d\n",__func__,ret);

    if(ret == LL_STATUS_SUCCESS)
    {
        scanInfo.ownAddrType = ownAddrType;

        if ( ownAddrType == LL_DEV_ADDR_TYPE_PUBLIC || ownAddrType == LL_DEV_ADDR_TYPE_RPA_PUBLIC)
        {
            LL_COPY_DEV_ADDR_LE( scanInfo.ownAddr, ownPublicAddr );
        }
        else
        {
            LL_COPY_DEV_ADDR_LE( scanInfo.ownAddr, ownRandomAddr );
        }
    }

    return ret;
}

//2020.10.22,Jie, modify sanity check:
//add ownaddrtype;
//add LL_STATUS_ERROR_BAD_PARAMETER case
llStatus_t LL_SetScanControl1( uint8 scanMode,
                               uint8 filterReports )
{
   _HAL_CS_ALLOC_();
//  LOG("%s,scanMode %d\n",__func__,scanMode);
    if (g_llScanMode == LL_MODE_EXTENDED )
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    g_llScanMode = LL_MODE_LEGACY;

    // check if a direct test mode or modem test is in progress
    if ( (llState == LL_STATE_DIRECT_TEST_MODE_TX) ||
            (llState == LL_STATE_DIRECT_TEST_MODE_RX) ||
            (llState == LL_STATE_MODEM_TEST_TX)       ||
            (llState == LL_STATE_MODEM_TEST_RX)       ||
            (llState == LL_STATE_MODEM_TEST_TX_FREQ_HOPPING) )
    {
        return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );
    }

    // sanity checks again to be sure we don't start with bad parameters
    if ( ( (scanInfo.scanType != LL_SCAN_PASSIVE)   &&
            (scanInfo.scanType != LL_SCAN_ACTIVE))              ||
            ( (scanInfo.ownAddrType != LL_DEV_ADDR_TYPE_PUBLIC) &&
              (scanInfo.ownAddrType != LL_DEV_ADDR_TYPE_RANDOM) &&
              (scanInfo.ownAddrType != LL_DEV_ADDR_TYPE_RPA_PUBLIC) &&
              (scanInfo.ownAddrType != LL_DEV_ADDR_TYPE_RPA_RANDOM))  ||
            ( (scanInfo.scanInterval < LL_SCAN_WINDOW_MIN)        ||
              (scanInfo.scanInterval > LL_SCAN_WINDOW_MAX))       ||
            ( (scanInfo.scanWindow < LL_SCAN_WINDOW_MIN)          ||
              (scanInfo.scanWindow > LL_SCAN_WINDOW_MAX))         ||
            ( (scanInfo.scanWindow > scanInfo.scanInterval) )     ||
            ( (filterReports != LL_FILTER_REPORTS_DISABLE)  &&
              (filterReports != LL_FILTER_REPORTS_ENABLE)) )
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    // check if we should begin scanning
    switch( scanMode )
    {
    // Scanning Mode is On
    case LL_SCAN_START:

//        LOG("LL_SCAN_START\n");

        // check if command makes sense
        if ( scanInfo.scanMode == LL_SCAN_START )
        {
            // this is unexpected; something is wrong
            return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );
        }

        //20200804 Jie :if random address isn't defined,can't set ownaddresstype to random
        if (((scanInfo.ownAddrType == LL_DEV_ADDR_TYPE_RANDOM)    ||
                (scanInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM))  &&
                (  (ownRandomAddr[0] == 0xFF) &&
                   (ownRandomAddr[1] == 0xFF) &&
                   (ownRandomAddr[2] == 0xFF) &&
                   (ownRandomAddr[3] == 0xFF) &&
                   (ownRandomAddr[4] == 0xFF) &&
                   (ownRandomAddr[5] == 0xFF)  ))
        {
            return( LL_STATUS_ERROR_BAD_PARAMETER );
        }

        // get a task block for this BLE state/role
        // Note: There will always be a valid pointer, so no NULL check required.
//          scanInfo.llTask = llAllocTask( LL_TASK_ID_SCANNER );

        // check if no other tasks are currently active
        if ( llState == LL_STATE_IDLE )
        {
            // indicate Scan has not already been initalized
            scanInfo.initPending = TRUE;
            // save the scan filtering flag
            scanInfo.filterReports = filterReports;
            // add by HZF
            scanInfo.nextScanChan  = LL_SCAN_ADV_CHAN_37;
            // set LL state
            llState = LL_STATE_SCAN;
            // Note: llState has been changed.
            LL_evt_schedule();
        }
        else if ((llState == LL_STATE_CONN_SLAVE
                  || llState == LL_STATE_CONN_MASTER)     // HZF: if we should support adv + scan, add more state here
                 && (pGlobal_config[LL_SWITCH] & SIMUL_CONN_SCAN_ALLOW))
        {
            if (llSecondaryState != LL_SEC_STATE_IDLE)
                return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );

            scanInfo.nextScanChan  = LL_SCAN_ADV_CHAN_37;
            llSecondaryState = LL_SEC_STATE_SCAN;
            osal_set_event(LL_TaskID, LL_EVT_SECONDARY_SCAN);
        }
        else
            return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );

        // indicate we are actively scanning
        scanInfo.scanMode = LL_SCAN_START;
        break;

    case LL_SCAN_STOP:
//        LOG("LL_SCAN_STOP\n");
    HAL_ENTER_CRITICAL_SECTION();

        if (llState == LL_STATE_SCAN)      // no conn + scan case
        {
            llState = LL_STATE_IDLE;     // if not in connect state, set idle to disable scan
            //ZQ 20190912
            //stop ll timer when idle, considering the scan-adv interleve case
            clear_timer(AP_TIM1);
            ll_debug_output(DEBUG_LL_STATE_IDLE);
        }
        else if (llState == LL_STATE_CONN_SLAVE
                 || llState == LL_STATE_CONN_MASTER)                      // conn + scan case
        {
            llSecondaryState = LL_SEC_STATE_IDLE;
            // bugfix for multi-role
            osal_stop_timerEx(LL_TaskID, LL_EVT_SECONDARY_SCAN);
        }

        // indicate we are no longer actively scanning
        scanInfo.scanMode = LL_SCAN_STOP;
        // A2 multiconn, should we consider current LL state to avoid change master/slave configuration
        // now LL slave/master event use same parameter 88
        ll_hw_set_rx_timeout(88);
        // HZF: should we stop scan task immediately, or wait scan IRQ then stop? Now use option 2.
        HAL_EXIT_CRITICAL_SECTION();

        while(read_reg(&llWaitingIrq) == TRUE);

        break;

    default:
        // we have an invalid value for advertisement mode
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    return( LL_STATUS_SUCCESS );
}

//2020.10.23 Jie,fix g_llPduLen.suggested.MaxTxTime setting error
llStatus_t LL_SetDataLengh1( uint16 connId,uint16 TxOctets,uint16 TxTime )
{
    if(TxOctets >   LL_PDU_LENGTH_SUPPORTED_MAX_TX_OCTECTS
            || TxTime   >   LL_PDU_LENGTH_SUPPORTED_MAX_TX_TIME
            || TxOctets <   LL_PDU_LENGTH_INITIAL_MAX_TX_OCTECTS
            || TxTime   <   LL_PDU_LENGTH_INITIAL_MAX_TX_TIME)
    {
        return(LL_STATUS_ERROR_PARAM_OUT_OF_RANGE);
    }
    else
    {
        g_llPduLen.suggested.MaxTxOctets= TxOctets;
        g_llPduLen.suggested.MaxTxTime  = TxTime;
        return LL_SetDataLengh0( connId,TxOctets,TxTime );
    }
}


volatile int sstmp = 1;
void llProcessTxData2( llConnState_t *connPtr, uint8 context )
{
    uint8         *pBuf;
  
    //HAL_ENTER_CRITICAL_SECTION();
  
    // try to put as many packets into the TX FIFO as possible
    while( connPtr->txDataQ.head != NULL )
    {
        // point to packet header
        pBuf = (uint8 *)(connPtr->txDataQ.head + 1);
        
        // check if TX is enabled and if there is room in TX FIFO
        if(pBuf[7] == 0x21){
          sstmp++;
        }
        
        if ( llWriteTxData( connPtr, pBuf[1], pBuf[0], &pBuf[2] ) == LL_STATUS_SUCCESS )
        {
            // remove entry from TX queue and free the buffer
            osal_bm_free( llDequeueDataQ( &connPtr->txDataQ ) );
            
            numComplPkts ++;
            
            // update counter
            connPtr->pmCounter.ll_hci_to_ll_pkt_cnt += numComplPkts;   
//            LOG("TX:%d\n", connPtr->connId);            
            
            continue;
        }
        
        // unable to complete the write, so keep packet on queue
        break;
    }
  
    // check if we completed any packets
    // The Number of Completed Packets event is sent when the number of completed
    // packets is equal to or greater than the user specified limit, which can
    // range from 1 to the LL_MAX_NUM_DATA_BUFFERS (default is one). If the
    // number of completed packets is less than the limit, then this event is only
    // sent if the user indicated that this event to be sent at the end of the
    // connection event.
    // Note: Spec indicates that while the Controller has HCI data packets in its
    //       buffer, it must keep sending the Number Of Completed Packets event
    //       to the Host at least periodically, until it finally reports that all
    //       the pending ACL Data Packets have been transmitted or flushed.
    //       However, this can potentially waste a lot of time sending events
    //       with a number of completed packets set to zero, so for now, this
    //       will not be supported.
    if ( (numComplPkts > 0) &&
         (numComplPkts >= numComplPktsLimit) ) // ||
          //((context == LL_TX_DATA_CONTEXT_POST_PROCESSING) && numComplPktsFlush)) )
    {
        uint16 connId = connPtr->connId;
        uint16 numCompletedPackets = numComplPkts;
                
        // and send credits to the Host
        HCI_NumOfCompletedPacketsEvent( 1,
                                        &connId,
                                        &numCompletedPackets );
        
        // clear count
        numComplPkts = 0;
    }
  
    //HAL_EXIT_CRITICAL_SECTION();
  
    return;
}


void llProcessTxData1( llConnState_t* connPtr, uint8 context )
{
    if(context==LL_TX_DATA_CONTEXT_SEND_DATA)
        return;

    llProcessTxData2(connPtr,context);
}
/*******************************************************************************
    @fn          ll_generateTxBuffer1

    @brief       This function generate Tx data and find in Tx FIFO
                there are 4 kinds of data:
                   1. control data
                   2. last no-ack data
                   3. last no-transmit data
                   4. new data
                 in the new RTLP buffer, the data should be in the below sequence:
                     2 --> 3 --> 1 --> 4   (changed)

    input parameters

    @param       txFifo_vacancy - allow max tx packet number.

    output parameters

    @param       None.

    @return      the pointer of 1st not transmit packet/new packet.

*/
uint16 ll_generateTxBuffer1(int txFifo_vacancy, uint16* pSave_ptr)
{
    int i, new_pkts_num, tx_num = 0;
    llConnState_t* connPtr;
    connPtr = &conn_param[g_ll_conn_ctx.currentConn];

    // 0. write empty packet
    if(connPtr->llMode == LL_HW_RTLP_EMPT
            || connPtr->llMode == LL_HW_TRLP_EMPT)     //  TRLP case, to be confirmed/test
    {
        LL_HW_WRT_EMPTY_PKT;
        connPtr->ll_buf.tx_not_ack_pkt->valid = 0;                    // empty mode, tx_not_ack buffer null or empty packet
        tx_num ++;
    }
    // 1. write last not-ACK packet
    else if (connPtr->ll_buf.tx_not_ack_pkt->valid != 0)            // TODO: if the valid field could omit, move the not-ACK flag to buf.
    {
        ll_hw_write_tfifo((uint8*)&(connPtr->ll_buf.tx_not_ack_pkt->header), ((connPtr->ll_buf.tx_not_ack_pkt->header & 0xff00) >> 8) + 2);
        //txFifo_vacancy --;
        tx_num ++;
        connPtr->ll_buf.tx_not_ack_pkt->valid = 0;
        AT_LOG("write last not-ACK packet \n");
    }

    // 1st RTLP event, no porcess 0/1, it should be 0 because we have reset the TFIFO
    // other case, it is 1st not transmit packet/new packet
    *pSave_ptr = ll_hw_get_tfifo_wrptr();

    // 3. write last not transmit packets
    if (connPtr->ll_buf.ntrm_cnt > 0
            && txFifo_vacancy >= connPtr->ll_buf.ntrm_cnt)
    {
        for (i = 0; i < connPtr->ll_buf.ntrm_cnt ; i++)
        {
            ll_hw_write_tfifo((uint8*)&(connPtr->ll_buf.tx_ntrm_pkts[i]->header), ((connPtr->ll_buf.tx_ntrm_pkts[i]->header & 0xff00) >> 8) + 2);
        }

        txFifo_vacancy -= connPtr->ll_buf.ntrm_cnt;
        tx_num += connPtr->ll_buf.ntrm_cnt;
        AT_LOG("write last not transmit packets\n");
        connPtr->ll_buf.ntrm_cnt = 0;
    }

    rfCounters.numTxCtrl = 0;    // add on 2017-11-15, set tx control packet number 0

    // 2. write control packet
    if ((connPtr->ll_buf.tx_not_ack_pkt->valid == 0 ||                 // no tx not_ack packet, add on 2017-11-15
            (connPtr->ll_buf.tx_not_ack_pkt->header & 0x3) != LL_DATA_PDU_HDR_LLID_CONTROL_PKT)    // last nack packet is not a control packet
            && connPtr->ctrlDataIsPending                                               // we only support 1 control procedure per connection
            && !connPtr->ctrlDataIsProcess
            && txFifo_vacancy > connPtr->ll_buf.ntrm_cnt)    // tricky here:  if the Tx FIFO is full and nothing is sent in last event, then it can't fill new packet(include ctrl pkt) in new event
    {
        // not in a control procedure, and there is control packet pending
        // fill ctrl packet
        ll_hw_write_tfifo((uint8*)&(connPtr->ctrlData .header), ((connPtr->ctrlData .header & 0xff00) >> 8) + 2);
        txFifo_vacancy --;
        tx_num ++;
        // put Ctrl packet in TFIFO, change the control procedure status
        connPtr->ctrlDataIsPending = 0;
        connPtr->ctrlDataIsProcess = 1;
        rfCounters.numTxCtrl = 1;     // add 2017-11-15, if put new ctrl packet in FIFO, add the counter
    }

    if (connPtr->ll_buf.ntrm_cnt != 0)
    {
        // should not be here, new packets should not be sent if there is not-transmit packets
        return tx_num;
    }

    // 4. write  new data packets to FIFO
    new_pkts_num = getTxBufferSize(connPtr);

    if ((new_pkts_num > 0)
            && txFifo_vacancy > 0)
    {
        // fill the data packet to Tx FIFO
        for (i = 0; i < new_pkts_num && i < txFifo_vacancy; i++)
        {
            uint8_t idx = get_tx_read_ptr(connPtr);
            ll_hw_write_tfifo((uint8*)&(connPtr->ll_buf.tx_conn_desc[idx]->header), ((connPtr->ll_buf.tx_conn_desc[idx]->header & 0xff00) >> 8) + 2);
            update_tx_read_ptr(connPtr);
            tx_num++;
            AT_LOG("write  new data packets to FIFO\n");
            // update PM counter, add A1 ROM metal change
            connPtr->pmCounter.ll_send_data_pkt_cnt ++;
        }
    }

    // 2020-02-13 periodic cte req & rsp
    if( ( connPtr->llConnCTE.enable ) && ( connPtr->llCTE_ReqFlag ))
    {
        if( connPtr->llConnCTE.CTE_Request_Intv > 0 )
        {
            if( connPtr->llConnCTE.CTE_Count_Idx < connPtr->llConnCTE.CTE_Request_Intv )
                connPtr->llConnCTE.CTE_Count_Idx++;
            else
            {
                connPtr->llConnCTE.CTE_Count_Idx = 0;
                llEnqueueCtrlPkt(connPtr, LL_CTRL_CTE_REQ );
            }
        }
    }

    return tx_num;
}


#if 0
//2020.10.23 Jie,fix setphymode issue
llStatus_t LL_SetPhyMode1( uint16 connId,uint8 allPhy,uint8 txPhy, uint8 rxPhy,uint16 phyOptions)
{
    uint8         i;
    llStatus_t    status;
    llConnState_t* connPtr;

    // make sure connection ID is valid
    if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
    {
        return( status );
    }

    // get connection info
    connPtr = &conn_param[connId];

    // check if a feature response control procedure has taken place
    if ( connPtr->featureSetInfo.featureRspRcved == FALSE )
    {
        // it hasn't so re-load this device's local Feature Set to the
        // connection as it may have been changed by the Host with HCI
        // extenstion Set Local Feature Set command
        for (i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
        {
            connPtr->featureSetInfo.featureSet[i] = deviceFeatureSet.featureSet[i];
        }
    }

    // check if dle is a supported feature set item
    if(     ( (connPtr->featureSetInfo.featureSet[1] & LL_FEATURE_2M_PHY) != LL_FEATURE_2M_PHY )
            &&  ( (connPtr->featureSetInfo.featureSet[1] & LL_FEATURE_CODED_PHY) != LL_FEATURE_CODED_PHY ) )
    {
        return( LL_STATUS_ERROR_FEATURE_NOT_SUPPORTED );
    }

    // check if an updated parameters control procedure is already what's pending
    if ( ((connPtr->ctrlPktInfo.ctrlPktCount > 0) &&
            (connPtr->ctrlPktInfo.ctrlPkts[0] == LL_CTRL_PHY_REQ)) ||
            (connPtr->pendingPhyModeUpdate== TRUE) ||
            (connPtr->llPhyModeCtrl.isWatingRsp == TRUE) || (connPtr->llPhyModeCtrl.isProcessingReq == TRUE) )
    {
        return( LL_STATUS_ERROR_CTRL_PROC_ALREADY_ACTIVE );
    }

    //support Symmetric Only
    if(allPhy==0 &&(txPhy!=rxPhy))
    {
        return( LL_STATUS_ERROR_FEATURE_NOT_SUPPORTED );
    }

    //jie 2020.9.3 check unsupport phy
    if ((txPhy > 0x07) || (rxPhy >0x07))
    {
        return( LL_STATUS_ERROR_FEATURE_NOT_SUPPORTED );
    }

    uint8 tx_chance = (txPhy ^ connPtr->llPhyModeCtrl.local.txPhy) ^connPtr->llPhyModeCtrl.local.txPhy;

    if(tx_chance & LE_1M_PHY)
    {
        txPhy = LE_1M_PHY;
    }
    else if(tx_chance & LE_2M_PHY)
    {
        txPhy = LE_2M_PHY;
    }
    else if(tx_chance & LE_CODED_PHY)
    {
        txPhy = LE_CODED_PHY;
    }
    else
    {
        //nothing
    }

    uint8 rx_chance = (rxPhy ^ connPtr->llPhyModeCtrl.local.rxPhy)^connPtr->llPhyModeCtrl.local.rxPhy;

    if(rx_chance & LE_1M_PHY)
    {
        rxPhy = LE_1M_PHY;
    }
    else if(rx_chance & LE_2M_PHY)
    {
        rxPhy = LE_2M_PHY;
    }
    else if(rx_chance & LE_CODED_PHY)
    {
        rxPhy = LE_CODED_PHY;
    }
    else
    {
        //nothing
    }

    // how to check the required param?
    //LL_TS_5.0.3 Table 4.43: PDU payload contents for each case variation for LE 2M PHY
    connPtr->llPhyModeCtrl.req.allPhy = allPhy;

    if(connPtr->llPhyModeCtrl.req.allPhy==0)
    {
        connPtr->llPhyModeCtrl.req.txPhy  = txPhy;
        connPtr->llPhyModeCtrl.req.rxPhy  = rxPhy;
    }
    else if(connPtr->llPhyModeCtrl.req.allPhy==1)
    {
        connPtr->llPhyModeCtrl.req.txPhy  = rxPhy;//0;
        connPtr->llPhyModeCtrl.req.rxPhy  = rxPhy;
    }
    else if(connPtr->llPhyModeCtrl.req.allPhy==2)
    {
        connPtr->llPhyModeCtrl.req.txPhy  = txPhy;
        connPtr->llPhyModeCtrl.req.rxPhy  = txPhy;//0;
    }
    else
    {
        //no prefer on both phy
        connPtr->llPhyModeCtrl.req.txPhy  = LE_1M_PHY;//0;
        connPtr->llPhyModeCtrl.req.rxPhy  = LE_1M_PHY;//0;
    }

    connPtr->llPhyModeCtrl.phyOptions = phyOptions;
    //update def.phy jie 2020.9.2
    connPtr->llPhyModeCtrl.def.allPhy = allPhy;
    // connPtr->llPhyModeCtrl.def.txPhy = connPtr->llPhyModeCtrl.req.txPhy;
    // connPtr->llPhyModeCtrl.def.rxPhy = connPtr->llPhyModeCtrl.req.rxPhy;
    // setup an LL_CTRL_PHY_REQ
    llEnqueueCtrlPkt( connPtr, LL_CTRL_PHY_REQ );
    return(LL_STATUS_SUCCESS);
}
#endif


/*  2020.11.11,Jie,fix ownaddr random address source issue
*/
llStatus_t LL_CreateConn1( uint16 scanInterval,
                           uint16 scanWindow,
                           uint8  initWlPolicy,
                           uint8  peerAddrType,
                           uint8*  peerAddr,
                           uint8  ownAddrType,
                           uint16 connIntervalMin,
                           uint16 connIntervalMax,
                           uint16 connLatency,
                           uint16 connTimeout,
                           uint16 minLength,        //  minimum length of connection needed for this LE conn, no use now
                           uint16 maxLength )       //  maximum length of connection needed for this LE conn, no use now
{
    CreateConn_Flag = TRUE;
    return LL_CreateConn0(scanInterval,
                          scanWindow,
                          initWlPolicy,
                          peerAddrType,
                          peerAddr,
                          ownAddrType,
                          connIntervalMin,
                          connIntervalMax,
                          connLatency,
                          connTimeout,
                          minLength,
                          maxLength );
}

#if 0
//2020.11.12, add case LL_REJECT_IND_EXT
void llProcessMasterControlPacket1( llConnState_t* connPtr,
                                    uint8*         pBuf )
{
    uint8 i;
    uint8 opcode = *pBuf++;
    uint8 iqCnt = 0;

    // check the type of control packet
    switch( opcode )
    {
    // Encryption Response
    case LL_CTRL_ENC_RSP:
        // concatenate slave's SKDs with SKDm
        // Note: The SKDs MSO is the MSO of the SKD.
        //PHY_READ_BYTE( (uint8 *)&connPtr->encInfo.SKD[LL_ENC_SKD_S_OFFSET], LL_ENC_SKD_S_LEN );
        pBuf = llMemCopySrc( (uint8*)&connPtr->encInfo.SKD[LL_ENC_SKD_S_OFFSET], pBuf, LL_ENC_SKD_S_LEN );
        // bytes are received LSO..MSO, but need to be maintained as
        // MSO..LSO, per FIPS 197 (AES), so reverse the bytes
        LL_ENC_ReverseBytes( &connPtr->encInfo.SKD[LL_ENC_SKD_S_OFFSET], LL_ENC_SKD_S_LEN );
        // concatenate the slave's IVs with IVm
        // Note: The IVs MSO is the MSO of the IV.
        //PHY_READ_BYTE( (uint8 *)&connPtr->encInfo.IV[LL_ENC_IV_S_OFFSET], LL_ENC_IV_S_LEN );
        pBuf = llMemCopySrc( (uint8*)&connPtr->encInfo.IV[LL_ENC_IV_S_OFFSET], pBuf, LL_ENC_IV_S_LEN );
        // bytes are received LSO..MSO, but need to be maintained as
        // MSO..LSO, per FIPS 197 (AES), so reverse the bytes
        // ALT: POSSIBLE TO MAINTAIN THE IV IN LSO..MSO ORDER SINCE THE NONCE
        //      IS FORMED THAT WAY.
        LL_ENC_ReverseBytes( &connPtr->encInfo.IV[LL_ENC_IV_S_OFFSET], LL_ENC_IV_S_LEN );

        // place the IV into the Nonce to be used for this connection
        // Note: If a Pause Encryption control procedure is started, the
        //       old Nonce value will be used until encryption is disabled.
        // Note: The IV is sequenced LSO..MSO within the Nonce.
        // ALT: POSSIBLE TO MAINTAIN THE IV IN LSO..MSO ORDER SINCE THE NONCE
        //      IS FORMED THAT WAY.
        for (i=0; i<LL_ENC_IV_LEN; i++)
        {
            connPtr->encInfo.nonce[ LL_END_NONCE_IV_OFFSET+i ] =
                connPtr->encInfo.IV[ (LL_ENC_IV_LEN-i)-1 ];
        }

        // generate the Session Key (i.e. SK = AES128(LTK, SKD))
        LL_ENC_GenerateSK( connPtr->encInfo.LTK,
                           connPtr->encInfo.SKD,
                           connPtr->encInfo.SK );
        //      LOG("LTK: %x\r\n", connPtr->encInfo.LTK);
        //      LOG("SKD: %x\r\n", connPtr->encInfo.SKD);
        //      LOG("SK: %x\r\n", connPtr->encInfo.SK[0], connPtr->encInfo.SK[1], connPtr->encInfo.SK[],connPtr->encInfo.SK[0],
        //      connPtr->encInfo.SK[0],connPtr->encInfo.SK[0],connPtr->encInfo.SK[0]);
        // Note: Done for now; the slave will send LL_CTRL_START_ENC_REQ.
        //LOG("ENC_RSP ->");
        break;

    // Start Encryption Request
    case LL_CTRL_START_ENC_REQ:
        // set a flag to indicate we've received this packet
        connPtr->encInfo.startEncReqRcved = TRUE;
        break;

    // Start Encryption Response
    case LL_CTRL_START_ENC_RSP:
        // set flag to allow outgoing data transmissions
        connPtr->txDataEnabled = TRUE;
        // okay to receive data again
        connPtr->rxDataEnabled = TRUE;
        // indicate we've received the start encryption response
        connPtr->encInfo.startEncRspRcved = TRUE;

        // notify the Host
        if ( connPtr->encInfo.encRestart == TRUE )
        {
            // a key change was requested
            LL_EncKeyRefreshCback( connPtr->connId,
                                   LL_ENC_KEY_REQ_ACCEPTED );
        }
        else
        {
            // a new encryption was requested
            LL_EncChangeCback( connPtr->connId,
                               LL_ENC_KEY_REQ_ACCEPTED,
                               LL_ENCRYPTION_ON );
        }

        // clear the restart flag in case of another key change request
        // Note: But in reality, there isn't a disable encryption in BLE,
        //       so once encryption is enabled, any call to LL_StartEncrypt
        //       will result in an encryption key change callback.
        connPtr->encInfo.encRestart = FALSE;
        //LOG("START_ENC_RSP ->");
        break;

    // Pause Encryption Response
    case LL_CTRL_PAUSE_ENC_RSP:
        // set a flag to indicate we have received LL_START_ENC_RSP
        connPtr->encInfo.pauseEncRspRcved = TRUE;
        break;

    // Reject Encryption Indication
    /*
        case LL_CTRL_REJECT_IND:
        // either the slave's Host has failed to provide an LTK, or
        // the encryption feature is not supported by the slave, so read
        // the rejection indication error code
        //connPtr->encInfo.encRejectErrCode = PHY_READ_BYTE_VAL();
        connPtr->encInfo.encRejectErrCode = *pBuf;

        // and end the start encryption procedure
        connPtr->encInfo.rejectIndRcved = TRUE;

        break;
    */

    // Controller Feature Setup   --> should be LL_CTRL_SLAVE_FEATURE_REQ
    //    case LL_CTRL_FEATURE_REQ:                // new for BLE4.2, to test

    //      for (i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
    //      {
    //        connPtr->featureSetInfo.featureSet[i] = deviceFeatureSet.featureSet[i];
    //      }

    //      // logical-AND with master's feature set to indicate which of the
    //      // controller features in the master the slave requests to be used
    //      for (i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
    //      {
    //        connPtr->featureSetInfo.featureSet[i] =
    //          *pBuf++ & deviceFeatureSet.featureSet[i];
    //      }

    //      // schedule the output of the control packet
    //      // Note: Features to be used will be taken on the next connection
    //      //       event after the response is successfully transmitted.
    //      llEnqueueCtrlPkt( connPtr, LL_CTRL_FEATURE_RSP );

    //      break;

    case LL_CTRL_FEATURE_RSP:
    {
        uint8 peerFeatureSet[ LL_MAX_FEATURE_SET_SIZE ];
        // get the peer's device Feature Set
        //for (i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
        //{
        //  peerFeatureSet[i] = PHY_READ_BYTE_VAL();
        //}
        pBuf = llMemCopySrc( peerFeatureSet, pBuf, LL_MAX_FEATURE_SET_SIZE );

        // read this device's Feature Set
        // Note: Must re-read the device feature set as it may have been
        //       changed by the Host with Set Local Feature Set.
        // Note: It is not clear this should be done. If the host sets
        //       the Local Set, then reads the Remote Set, but before
        //       that is done, sets the Local Set a second time, then
        //       perhaps re-reading this device's feature set makes the
        //       Local Set inconsistent. On the other hand, it will be
        //       consistent with the Local Set before the Remote Set
        //       read. Note sure.
        for (i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
        {
            connPtr->featureSetInfo.featureSet[i] = deviceFeatureSet.featureSet[i];
        }

        // logical-AND with slave's feature set to indicate which of the
        // controller features in the master the slave requests to be
        // used
        // Note: For now, there is only one feature that is supported
        //       controller-to-controller.
        // Note: If the peer supports the feature, then our setting is
        //       the controller-to-controller setting, so no action
        //       is required.
        if ( !(peerFeatureSet[0] & LL_FEATURE_ENCRYPTION) )
        {
            // this feature is not supported by the peer, so it doesn't
            // matter if we support it or not, it should not be supported
            connPtr->featureSetInfo.featureSet[0] &= ~LL_FEATURE_ENCRYPTION;
        }
    }

        // set flag to indicate the response has been received
    connPtr->featureSetInfo.featureRspRcved = TRUE;
    break;

    // Version Information Indication
    case LL_CTRL_VERSION_IND:

        // check if the peer's version information has already been obtained
        if ( connPtr->verExchange.peerInfoValid == TRUE )
        {
            // it has, so something is wrong as the spec indicates that
            // only one version indication should be sent for a connection
            // unknown data PDU control packet received so save the type
            connPtr->unknownCtrlType = opcode;
            // schedule the output of the control packet
            llEnqueueCtrlPkt( connPtr, LL_CTRL_UNKNOWN_RSP );
        }
        else // the peer version info is invalid, so make it valid
        {
            // get the peer's version information and save it
            //PHY_READ_BYTE( (uint8 *)&peerInfo.verInfo.verNum, 1 );
            connPtr->verInfo.verNum = *pBuf++;
            //PHY_READ_BYTE( (uint8 *)&peerInfo.verInfo.comId, 2 );
            pBuf = llMemCopySrc( (uint8*)&connPtr->verInfo.comId, pBuf, 2 );
            //PHY_READ_BYTE( (uint8 *)&peerInfo.verInfo.subverNum, 2 );
            pBuf = llMemCopySrc( (uint8*)&connPtr->verInfo.subverNum, pBuf, 2 );
            // set a flag to indicate it is now valid
            connPtr->verExchange.peerInfoValid = TRUE;

            // check if a version indication has been sent
            if ( connPtr->verExchange.verInfoSent == FALSE )
            {
                // no, so this is a peer's request for our version information
                llEnqueueCtrlPkt( connPtr, LL_CTRL_VERSION_IND );
            }
        }

        break;

    // Terminate Indication
    case LL_CTRL_TERMINATE_IND:
        // read the reason code
        connPtr->termInfo.reason = *pBuf;
        // set flag to indicate a termination indication was received
        connPtr->termInfo.termIndRcvd = TRUE;
        // received a terminate from peer host, so terminate after
        // confirming we have sent an ACK
        // Note: For the master, we have to ensure that this control
        //       packet was ACK'ed. For that, the nR has a new flag that
        //       is set when the control packet is received, and cleared
        //       when the control packet received is ACK'ed.
        // Note: This is not an issue as a slave because the terminate
        //       packet will re-transmit until the slave ACK's.
        // ALT: COULD REPLACE THIS CONTROL PROCEDURE AT THE HEAD OF THE
        //      QUEUE SO TERMINATE CAN TAKE PLACE ASAP.
        //llReplaceCtrlPkt( connPtr, LL_CTRL_TERMINATE_RX_WAIT_FOR_TX_ACK );
        llEnqueueCtrlPkt( connPtr, LL_CTRL_TERMINATE_RX_WAIT_FOR_TX_ACK );
        break;

    // LL PDU Data Length Req
    case LL_CTRL_LENGTH_REQ:

        // check if the feature response procedure has already been performed
        // on this connection
        if ( connPtr->featureSetInfo.featureRspRcved == FALSE )
        {
            // it hasn't so re-load this device's local Feature Set to the
            // connection as it may have been changed by the Host with HCI
            // extenstion Set Local Feature Set command
            for (i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
            {
                connPtr->featureSetInfo.featureSet[i] = deviceFeatureSet.featureSet[i];
            }
        }

        // check if supported DLE
        if ( (connPtr->featureSetInfo.featureSet[0] & LL_FEATURE_DATA_LENGTH_EXTENSION)
                != LL_FEATURE_DATA_LENGTH_EXTENSION )
        {
            // unknown data PDU control packet received so save the type
            connPtr->unknownCtrlType = opcode;
            // schedule the output of the control packet
            llEnqueueCtrlPkt( connPtr, LL_CTRL_UNKNOWN_RSP );
        }
        else
        {
            if(connPtr->llPduLen.isProcessingReq==FALSE)
            {
                pBuf = llMemCopySrc( (uint8*)& (connPtr->llPduLen.remote.MaxRxOctets), pBuf, 2 );
                pBuf = llMemCopySrc( (uint8*)& (connPtr->llPduLen.remote.MaxRxTime), pBuf, 2 );
                pBuf = llMemCopySrc( (uint8*)& (connPtr->llPduLen.remote.MaxTxOctets), pBuf, 2 );
                pBuf = llMemCopySrc( (uint8*)& (connPtr->llPduLen.remote.MaxTxTime), pBuf, 2 );
                connPtr->llPduLen.isProcessingReq=TRUE;
                llEnqueueCtrlPkt( connPtr, LL_CTRL_LENGTH_RSP );
            }
        }

        break;

    // LL PDU Data Length RSP
    case LL_CTRL_LENGTH_RSP:

        // check if supported DLE
        if ( (connPtr->featureSetInfo.featureSet[0] & LL_FEATURE_DATA_LENGTH_EXTENSION)
                != LL_FEATURE_DATA_LENGTH_EXTENSION )
        {
            // unknown data PDU control packet received so save the type
            connPtr->unknownCtrlType = opcode;
            // schedule the output of the control packet
            llEnqueueCtrlPkt( connPtr, LL_CTRL_UNKNOWN_RSP );
        }
        else
        {
            if(connPtr->llPduLen.isWatingRsp==TRUE )
            {
                pBuf = llMemCopySrc( (uint8*)& (connPtr->llPduLen.remote.MaxRxOctets), pBuf, 2 );
                pBuf = llMemCopySrc( (uint8*)& (connPtr->llPduLen.remote.MaxRxTime), pBuf, 2 );
                pBuf = llMemCopySrc( (uint8*)& (connPtr->llPduLen.remote.MaxTxOctets), pBuf, 2 );
                pBuf = llMemCopySrc( (uint8*)& (connPtr->llPduLen.remote.MaxTxTime), pBuf, 2 );
                llPduLengthUpdate((uint16)connPtr->connId);
                connPtr->llPduLen.isWatingRsp=FALSE;
            }
        }

        break;

    // LL PHY UPDATE REQ
    case LL_CTRL_PHY_REQ:

        // check if the feature response procedure has already been performed
        // on this connection
        if ( connPtr->featureSetInfo.featureRspRcved == FALSE )
        {
            // it hasn't so re-load this device's local Feature Set to the
            // connection as it may have been changed by the Host with HCI
            // extenstion Set Local Feature Set command
            for (i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
            {
                connPtr->featureSetInfo.featureSet[i] = deviceFeatureSet.featureSet[i];
            }
        }

        // check if supported PHY MODE UPDATE
        if (   (connPtr->featureSetInfo.featureSet[1] & LL_FEATURE_2M_PHY) != LL_FEATURE_2M_PHY
                && (connPtr->featureSetInfo.featureSet[1] & LL_FEATURE_CODED_PHY) != LL_FEATURE_CODED_PHY)
        {
            // unknown data PDU control packet received so save the type
            connPtr->unknownCtrlType = opcode;
            // schedule the output of the control packet
            llEnqueueCtrlPkt( connPtr, LL_CTRL_UNKNOWN_RSP );
        }
        else
        {
            //process for the protocol collision
            //2018-11-10 by ZQ
            if(connPtr->llPhyModeCtrl.isWatingRsp==TRUE ||
                    connPtr->pendingChanUpdate==TRUE  ||
                    connPtr->pendingParamUpdate==TRUE   )
            {
                connPtr->isCollision=TRUE;
                connPtr->rejectOpCode = LL_CTRL_PHY_REQ;
                // schedule the output of the control packet
                llEnqueueCtrlPkt( connPtr, LL_CTRL_REJECT_EXT_IND );
            }
            else
            {
                if(connPtr->llPhyModeCtrl.isProcessingReq==FALSE)
                {
                    connPtr->llPhyModeCtrl.req.txPhy=*pBuf++;
                    connPtr->llPhyModeCtrl.req.rxPhy=*pBuf++;
                    connPtr->llPhyModeCtrl.req.allPhy=connPtr->llPhyModeCtrl.def.allPhy;
                    connPtr->llPhyModeCtrl.rsp.txPhy=connPtr->llPhyModeCtrl.def.txPhy;
                    connPtr->llPhyModeCtrl.rsp.rxPhy=connPtr->llPhyModeCtrl.def.rxPhy;
                    //rsp and req will be used to determine the next phy mode
                    LL_PhyUpdate((uint16) connPtr->connId);
                    connPtr->llPhyModeCtrl.isProcessingReq=TRUE;
                }
                else
                {
                    //should no be here
                }
            }
        }

        break;

    // LL_CTRL_PHY_RSP
    case LL_CTRL_PHY_RSP:

        // check if supported PHY MODE UPDATE
        if (   (connPtr->featureSetInfo.featureSet[1] & LL_FEATURE_2M_PHY) != LL_FEATURE_2M_PHY
                && (connPtr->featureSetInfo.featureSet[1] & LL_FEATURE_CODED_PHY) != LL_FEATURE_CODED_PHY)
        {
            // unknown data PDU control packet received so save the type
            connPtr->unknownCtrlType = opcode;
            // schedule the output of the control packet
            llEnqueueCtrlPkt( connPtr, LL_CTRL_UNKNOWN_RSP );
        }
        else
        {
            if(connPtr->llPhyModeCtrl.isWatingRsp==TRUE)
            {
                connPtr->llPhyModeCtrl.rsp.txPhy=*pBuf++;
                connPtr->llPhyModeCtrl.rsp.rxPhy=*pBuf++;
                LL_PhyUpdate((uint16) connPtr->connId);
                connPtr->llPhyModeCtrl.isWatingRsp=FALSE;
            }
            else
            {
                //should no be here
            }
        }

        break;

    case LL_CTRL_CTE_REQ:

        // check if the feature response procedure has already been performed
        // on this connection
        if ( connPtr->featureSetInfo.featureRspRcved == FALSE )
        {
            // it hasn't so re-load this device's local Feature Set to the
            // connection as it may have been changed by the Host with HCI
            // extenstion Set Local Feature Set command
            for (i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
            {
                connPtr->featureSetInfo.featureSet[i] = deviceFeatureSet.featureSet[i];
            }
        }

        // check if supported CTE Response Feature
        //        if( connPtr->featureSetInfo.featureSet[LL_CTE_FEATURE_IDX] & LL_CONN_CTE_RSP)
        if(( ( connPtr->featureSetInfo.featureSet[LL_CTE_FEATURE_IDX] & LL_CONN_CTE_RSP) != LL_CONN_CTE_RSP) || \
                ( connPtr->llCTE_RspFlag != TRUE ))
        {
            // unknown data PDU control packet received so save the type
            connPtr->unknownCtrlType = opcode;
            // schedule the output of the control packet
            llEnqueueCtrlPkt( connPtr, LL_CTRL_UNKNOWN_RSP );
        }
        else
        {
            // process for the protocol collision
            // if other ctrl command procedure in processing , then reject
            if(connPtr->llCTEModeCtrl.isWatingRsp==TRUE)
            {
                connPtr->isCollision=TRUE;
                connPtr->rejectOpCode = LL_CTRL_CTE_REQ;
                // schedule the output of the control packet
                llEnqueueCtrlPkt( connPtr, LL_CTRL_REJECT_EXT_IND );
            }
            else
            {
                if(connPtr->llCTEModeCtrl.isProcessingReq==FALSE)
                {
                    uint8 CTE_tmp;
                    CTE_tmp = *pBuf++;
                    connPtr->llConnCTE.CTE_Length = CTE_tmp & 0x1F;
                    connPtr->llConnCTE.CTE_Type = CTE_tmp & 0xC0;
                    connPtr->llCTEModeCtrl.isProcessingReq=TRUE;

                    if( ( connPtr->llConnCTE.enable ) && ( connPtr->llRfPhyPktFmt < LL_PHY_CODE ))
                    {
                        llEnqueueCtrlPkt( connPtr, LL_CTRL_CTE_RSP );
                    }
                    else
                    {
                        if( connPtr->llRfPhyPktFmt >= LL_PHY_CODE )
                        {
                            connPtr->llCTEModeCtrl.errorCode = LL_STATUS_ERROR_INVALID_LMP_LL_PARAMETER;
                        }
                        else
                        {
                            connPtr->llCTEModeCtrl.errorCode = LL_STATUS_ERROR_UNSUPPORT_LMP_LL_PARAMETER;
                        }

                        connPtr->rejectOpCode = LL_CTRL_CTE_REQ;
                        // schedule the output of the control packet
                        llEnqueueCtrlPkt( connPtr, LL_CTRL_REJECT_EXT_IND );
                    }
                }
            }
        }

        break;

    case LL_CTRL_CTE_RSP:
        if( connPtr->llCTEModeCtrl.isWatingRsp == TRUE )
        {
            if( ( g_pLLcteISample != NULL ) && ( g_pLLcteQSample != NULL) )
                iqCnt = ll_hw_get_iq_RawSample( g_pLLcteISample, g_pLLcteQSample );

            if( iqCnt > 0)
            {
                LL_ConnectionIQReportCback( connPtr->connId,
                                            connPtr->llRfPhyPktFmt,
                                            connPtr->currentChan,
                                            connPtr->lastRssi,
                                            // before CTE Transmit and sampling , no Antenna change , default 0
                                            0,
                                            connPtr->llConnCTE.CTE_Type,
                                            connPtr->llConnCTE.slot_Duration,
                                            // Packet_Status=0, CRC success,cause only CRC Correctly that can run here
                                            0,
                                            connPtr->currentEvent,
                                            iqCnt,
                                            g_pLLcteISample,
                                            g_pLLcteQSample);
            }
            else
            {
                // packet contain LL_CTE_RSP , but did not contain CTE field
                // status = 0x0 : LL_CTE_RSP received successful , but without a CTE field
                LL_CTE_Report_FailedCback(  0x0,connPtr->connId);
            }

            connPtr->llCTEModeCtrl.isWatingRsp = FALSE;
        }

        break;

    // Peer Device Received an Unknown Control Type
    case LL_CTRL_UNKNOWN_RSP:

        // Note: There doesn't appear to be any action for this message,
        //       other than to ACK it.
        if(connPtr->llPduLen.isWatingRsp)
        {
            llPduLengthUpdate((uint16)connPtr->connId);
            connPtr->llPduLen.isWatingRsp=FALSE;//not support DLE
        }

        if(connPtr->llPhyModeCtrl.isWatingRsp)
        {
            llPhyModeCtrlUpdateNotify(connPtr,LL_STATUS_ERROR_UNSUPPORTED_REMOTE_FEATURE);
            connPtr->llPhyModeCtrl.isWatingRsp=FALSE;//not support PHY_UPDATE
        }

        // 2020-01-23 add for CTE
        if( connPtr->llCTEModeCtrl.isWatingRsp )
        {
            connPtr->llCTEModeCtrl.isWatingRsp = FALSE;
        }

        break;

    case LL_REJECT_IND:
    case LL_REJECT_IND_EXT:
        connPtr->rejectOpCode = *pBuf++;
        uint8 errorcode = *pBuf++;

        if(connPtr->rejectOpCode == LL_CTRL_ENC_REQ)
        {
            // either the slave's Host has failed to provide an LTK, or
            // the encryption feature is not supported by the slave, so read
            // the rejection indication error code
            //connPtr->encInfo.encRejectErrCode = PHY_READ_BYTE_VAL();
            connPtr->encInfo.encRejectErrCode = connPtr->rejectOpCode;
            // and end the start encryption procedure
            connPtr->encInfo.rejectIndRcved = TRUE;
            LL_EncChangeCback( connPtr->connId,
                               errorcode,
                               LL_ENCRYPTION_OFF );
        }
        else
        {
            //TBD
        }

        //connPtr->isCollision=FALSE;
        break;

    // Our Device Received an Unknown Control Type
    default:
        // unknown data PDU control packet received so save the type
        connPtr->unknownCtrlType = opcode;
        // schedule the output of the control packet
        llEnqueueCtrlPkt( connPtr, LL_CTRL_UNKNOWN_RSP );
        break;
    }

    return;
}
#endif

static uint32  read_LL_remainder_time1(void)
{
    uint32 currentCount;
    uint32 g_tim1_pass = read_current_fine_time();
    currentCount = AP_TIM1->CurrentCount;

    //if((currentCount < 6) || NVIC_GetPendingIRQ(TIM1_IRQn))
    //    return 0;
    //else
        return (currentCount >> 2);
}

uint8 llSecAdvAllow1(void)
{
    uint32 advTime, margin;
    uint32 remainTime;
    uint8 ret = FALSE;
    // Hold off interrupts.
    _HAL_CS_ALLOC_();
    HAL_ENTER_CRITICAL_SECTION();
    // read global config to get advTime and margin
    advTime = pGlobal_config[LL_NOCONN_ADV_EST_TIME];
    margin = pGlobal_config[LL_NOCONN_ADV_MARGIN];
    // remain time before trigger LL HW
    remainTime = read_LL_remainder_time1();

    if ((remainTime > advTime + margin)
            && !llWaitingIrq)
        ret = TRUE;
    else
    {
        llSecondaryState = LL_SEC_STATE_ADV_PENDING;
        g_pmCounters.ll_conn_adv_pending_cnt ++;
    }

    HAL_EXIT_CRITICAL_SECTION();
    return ret;
}

uint32 llCalcMaxScanTime1(void)
{
    uint32 margin, scanTime;
    uint32 remainTime;
    margin = pGlobal_config[LL_SEC_SCAN_MARGIN];
    // Hold off interrupts.
    _HAL_CS_ALLOC_();
    HAL_ENTER_CRITICAL_SECTION();
    // remain time before trigger LL HW
    remainTime = read_LL_remainder_time1();
    scanTime = 0;

    if (remainTime > margin + pGlobal_config[LL_MIN_SCAN_TIME]
            && !llWaitingIrq)
        scanTime = remainTime - margin;

    HAL_EXIT_CRITICAL_SECTION();
    return (scanTime);
}



llStatus_t LL_StartEncrypt1( uint16 connId,
                             uint8*  rand,
                             uint8*  eDiv,
                             uint8*  ltk )
{
    uint8         i;
    llStatus_t    status;
    llConnState_t* connPtr;

    // make sure we're in Master role
//  if ( llState != LL_STATE_CONN_MASTER )
//  {
//    return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
//  }

    // check parameters
    if ( (rand == NULL) || (eDiv == NULL) || (ltk == NULL) )
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    // make sure connection ID is valid
    if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
    {
        return( status );
    }

    // get connection info
    connPtr = &conn_param[connId];

    // check if a feature response control procedure has taken place
    if ( connPtr->featureSetInfo.featureRspRcved == FALSE )
    {
        // it hasn't so re-load this device's local Feature Set to the
        // connection as it may have been changed by the Host with HCI
        // extenstion Set Local Feature Set command
        for (i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
        {
            connPtr->featureSetInfo.featureSet[i] = deviceFeatureSet.featureSet[i];
        }
    }

    // check if encryption is a supported feature set item
    if ( (connPtr->featureSetInfo.featureSet[0] & LL_FEATURE_ENCRYPTION) != LL_FEATURE_ENCRYPTION )
    {
        return( LL_STATUS_ERROR_FEATURE_NOT_SUPPORTED );
    }

    // cache the master's random vector
    // Note: The RAND will be left in LSO..MSO order as this is assumed to be the
    //       order of the bytes that will be returned to the Host.
    for (i=0; i<LL_ENC_RAND_LEN; i++)
    {
        connPtr->encInfo.RAND[i] = rand[i];
    }

    // cache the master's encryption diversifier
    // Note: The EDIV will be left in LSO..MSO order as this is assumed to be the
    //       order of the bytes that will be returned to the Host.
    connPtr->encInfo.EDIV[0] = eDiv[0];
    connPtr->encInfo.EDIV[1] = eDiv[1];

    // cache the master's long term key
    // Note: The order of the bytes will be maintained as MSO..LSO
    //       per FIPS 197 (AES).
    for (i=0; i<LL_ENC_LTK_LEN; i++)
    {
        connPtr->encInfo.LTK[(LL_ENC_LTK_LEN-i)-1] = ltk[i];
    }

    // generate SKDm
    // Note: The SKDm LSO is the LSO of the SKD.
    // Note: Placement of result forms concatenation of SKDm and SKDs.
    // Note: The order of the bytes will be maintained as MSO..LSO
    //       per FIPS 197 (AES).
    LL_ENC_GenDeviceSKD( &connPtr->encInfo.SKD[ LL_ENC_SKD_M_OFFSET ] );
    // generate IVm
    // Note: The IVm LSO is the LSO of the IV.
    // Note: Placement of result forms concatenation of IVm and IVs.
    // Note: The order of the bytes will be maintained as MSO..LSO
    //       per FIPS 197 (AES).
    LL_ENC_GenDeviceIV( &connPtr->encInfo.IV[ LL_ENC_IV_M_OFFSET ] );
    // schedule a cache update of FIPS TRNG values for next SKD/IV usage
    //  postRfOperations |= LL_POST_RADIO_CACHE_RANDOM_NUM;
    (void)LL_ENC_GenerateTrueRandNum( cachedTRNGdata, LL_ENC_TRUE_RAND_BUF_SIZE );
    // set flag to stop all outgoing transmissions
    connPtr->txDataEnabled = FALSE;
    // invalidate the existing session key, if any
    connPtr->encInfo.SKValid = FALSE;
    // indicate the LTK is not valid
    connPtr->encInfo.LTKValid = FALSE;

    // check if we are already in encryption mode
    if ( connPtr->encEnabled == TRUE )
    {
        // set a flag to indicate this is a restart (i.e. pause-then-start)
        connPtr->encInfo.encRestart = TRUE;
        // setup a pause encryption control procedure
        llEnqueueCtrlPkt( connPtr, LL_CTRL_PAUSE_ENC_REQ );
    }
    else // no, so...
    {
        // clear flag to indicate this is an encryption setup
        connPtr->encInfo.encRestart = FALSE;
        // setup an encryption control procedure
        llEnqueueCtrlPkt( connPtr, LL_CTRL_ENC_REQ );
    }

    return( LL_STATUS_SUCCESS );
}

void TIM1_IRQHandler1(void)
{;
//  if(gpio_read( P18)){
//      gpio_write(P34,1);
//    }
//  HAL_ENTER_CRITICAL_SECTION()
//   gpio_write(P25,1);
//   gpio_write(P25,0);


   if(AP_TIM1->status&0x1)   
   {  
       clear_timer_int(AP_TIM1);
       clear_timer(AP_TIM1);
   LL_evt_schedule();
   }
// HAL_EXIT_CRITICAL_SECTION();
}

void TIM3_IRQHandler1(void)
{
//  HAL_ENTER_CRITICAL_SECTION();
   if(AP_TIM3->status&0x1)   
   {  
       clear_timer_int(AP_TIM3);
       //clear_timer(AP_TIM1);
  }
//   gpio_write(P20,1);
//   gpio_write(P20,0);
  
// HAL_EXIT_CRITICAL_SECTION();
}

uint8 osal_set_event1( uint8 task_id, uint16 event_flag )
{
    uint16 *events;
    uint8 task_cnt = *(uint8 *)JUMP_FUNCTION(TASK_COUNT);
    _HAL_CS_ALLOC_();
    if ( task_id < task_cnt)//tasksCnt )
    {
      HAL_ENTER_CRITICAL_SECTION();    // Hold off interrupts
  		events = *(uint16 **)(JUMP_FUNCTION(TASK_EVENTS));
  		if (events != NULL)
  		    events[task_id] |= event_flag;
          //tasksEvents[task_id] |= event_flag;  // Stuff the event bit(s)
          HAL_EXIT_CRITICAL_SECTION();     // Release interrupts
          osal_run_system();
          return ( SUCCESS );
      }
      else
      {
        HAL_EXIT_CRITICAL_SECTION();     // Release interrupts
        return ( INVALID_TASK );
      }
}

void llSlaveEvt_TaskEndOk1( void )
{
    llConnState_t *connPtr;
      
    uint32_t next_time;
    uint32   sw_delay, T2;
    int      calibra_time, i;                          // this parameter will be calibrate provided by global_config    
  
    // check if the connection is still valid
    if (FALSE == conn_param[g_ll_conn_ctx.currentConn].active)
    {
        // connection may have already been ended by a reset
        // HZF: if other procedure terminate the link, it should schedule the next event
        return;
    }
  
    // get connection information
    connPtr = &conn_param[g_ll_conn_ctx.currentConn];        
  
    // advance the connection event count
    connPtr->currentEvent = connPtr->nextEvent;
  
    // check if any data has been received
    // Note: numRxOk includes numRxCtrl
    // Note: numRxNotOk removed as 4.5.2 of spec says the LSTO is reset upon
    //       receipt of a "valid packet", which is taken to mean no CRC error.
    if ( rfCounters.numRxOk  || rfCounters.numRxIgnored ||        // we have only "numRxOk"
         rfCounters.numRxEmpty || rfCounters.numRxFifoFull 
         || connPtr->rx_crcok != 0)     // ever Rx CRC OK packet
    {                               
        // yes, so update the supervision expiration count
        connPtr->expirationEvent = connPtr->currentEvent + connPtr->expirationValue;
        
        // clear flag that indicates we received first packet
        // Note: The first packet only really needs to be signalled when a new
        //       connection is formed or a connection's parameters are updated.
        //       However, there's no harm in resetting it every time in order to
        //       simplify the control logic.
        // Note: True-Low logic is used here to be consistent with nR's language.
        connPtr->firstPacket = 0;
        
        // slave latency may have been disabled because a packet from the master
        // was not received (see Core spec V4.0, Vol 6, Section 4.5.1), so restore
        // the slave latency value; but note that if this is the beginning of a
        // connection, you can not allow slave latency until the first NESN change
        // has occurred from the master (i.e. until master ACK's slave)
        if ( connPtr->slaveLatencyAllowed == TRUE )
        {
            // activate slave latency
            connPtr->slaveLatency = connPtr->slaveLatencyValue;
        }

        //receiver ack notifty the host
        if(connPtr->llPhyModeCtrl.isChanged==TRUE)
        {
            connPtr->llPhyModeCtrl.isChanged = FALSE;
            llPhyModeCtrlUpdateNotify(connPtr,LL_STATUS_SUCCESS);
        }
        
    }
    else // either no packets received, or packets received with CRC error
    {
        // check if we received any packets with a CRC error
        // Note: Spec change (section 4.5.5) indicates any packet received,
        //       regardless of CRC result, determines the anchor point.
        if (connPtr->rx_timeout)     // no packet was received from the master
        {
            // collect packet error information
            connPtr->perInfo.numMissedEvts++;
            
            // so listen to every event until a packet is received
            connPtr->slaveLatency = 0;        
        }
        else   //   ( rfCounters.numRxNotOk )  // different to TI sequence
        {
            // clear flag that indicates we received first packet
            // Note: The first packet only really needs to be signalled when a new
            //       connection is formed or a connection's parameters are updated.
            //       However, there's no harm in resetting it every time in order to
            //       simplify the control logic.
            // Note: True-Low logic is used here to be consistent with nR's language.
            connPtr->firstPacket = 0;
            
            // slave latency may have been disabled because a packet from the master
            // was not received (see Core spec V4.0, Vol 6, Section 4.5.1), so restore
            // the slave latency value; but note that if this is the beginning of a
            // connection, you can not allow slave latency until the first NESN change
            // has occurred from the master (i.e. until master ACK's slave)
            if ( connPtr->slaveLatencyAllowed == TRUE )
            {
                // activate slave latency
                connPtr->slaveLatency = connPtr->slaveLatencyValue;
            }
        }

        // check if we have a Supervision Timeout
        if ( connPtr->expirationEvent == connPtr->currentEvent )   
        {
            // check if either we already got the first packet in a connection or
            // if this isn't the quick LSTO associated with connection establishment
            // Note: The Slave reuses firstPacket when a Update Parameter control
            //       procedure is started, but in that case, the expiration event
            //       will be well past the event associated with connection
            //       establishement.
            if ( (connPtr->firstPacket == 0) ||
                 (connPtr->currentEvent != LL_LINK_SETUP_TIMEOUT) )
            {
                // yes, so terminate with LSTO
                llConnTerminate( connPtr, LL_SUPERVISION_TIMEOUT_TERM );
                g_pmCounters.ll_link_lost_cnt ++;
            }
            else // this is a failure to establish the connection
            {
                // so terminate immediately with failure to establish connection
                llConnTerminate( connPtr, LL_CONN_ESTABLISHMENT_FAILED_TERM );
                g_pmCounters.ll_link_estab_fail_cnt ++;
            }
//#ifdef MULTI_ROLE   
	        ll_scheduler(LL_INVALID_TIME);
//#endif				
            return;

        }		
    }

    // for a new connection, slave latency isn't enabled until the master's NESN
    // bit changes, which is equivalent to receiving a TX ACK; if this is an
    // update parameters, slave latency is also disabled until any first packet arrives,
    // however, to keep things simple for now, we will use the same ACK constraint
    if (rfCounters.numTxAck > 0)//connPtr->firstPacket == 0)   /// TODO: test the scenario
    {
        // set a flag to indicates it is now okay to use slave latency on this
        // connection, if specified
        // Note: This is now needed due to a change in spec (section 4.5.1) which
        //       requires that slave latency be disabled when a packet is not
        //       received from the master. Since this routine is common for END_OK
        //       and RX_TIMEOUT, there's no way to know if the first Master NESN
        //       bit change has taken place. This flag will indicate it has, so if
        //       a master packet is received, the slave latency value can be
        //       restored.
        if (pGlobal_config[LL_SWITCH] & SLAVE_LATENCY_ALLOW)  
            connPtr->slaveLatencyAllowed = TRUE;
          
        // only update slave latency if no control procedure is active
        // Note: When a control procedure is active, slave latency has to be
        //       disabled in case it exceeds the control procedure timeout.
        // Note: Even when a control procedure is active, but a control transaction
        //       timeout isn't used, we can still skip setting SL since that kind
        //       of control procedure wouldn't have disabled slave latency to begin
        //       with.
        // ALT: COULD RESET SL IN llProcessSlaveControlProcedures.
        if ( (connPtr->ctrlPktInfo.ctrlPktActive == FALSE) &&
             (connPtr->pendingParamUpdate == FALSE)        &&
             (connPtr->pendingChanUpdate  == FALSE)        &&
             (connPtr->pendingPhyModeUpdate  == FALSE) )
        {
            // at least one ACK, so Slave Latency is operational
            // Note: This really only happens once for the very first connection
            //       interval, and when an update parameters procedure is taking place.
            connPtr->slaveLatency = connPtr->slaveLatencyValue;
        }
    }

    // check if the nR performed a anchor point capture
    // Note: This bit is cleared at the start of a new task.
    // Note: The anchor capture will occur even if the RX FIFO was too full to
    //       accept the packet.
//  if (connPtr->connected == 0)//rx_timeout ==1 )      // update by HZF 05-03, if not connected, disable slave latency
    if (connPtr->firstPacket)                             // update by HZF 12-18, if not receive 1st packet(CRC OK or not), disable slave latency      
    {
        connPtr->slaveLatency = 0;		
    }


    /*
    ** Process RX Data Packets
    */
    uint8_t  buffer_size;
    buffer_size = getRxBufferSize(connPtr);  
    if (buffer_size > 0)                 // 2018-10-22, disable slave latency if receives some data/ctrl packet
        connPtr->slaveLatencyAllowed = FALSE;
    for ( i = 0; i < buffer_size; i ++)     // note: i < getRxBufferSize()  will fail the loop
    {
        // there is, so process it; check if data was processed
        if ( llProcessRxData() == FALSE )
        {
            // it wasn't, so we're done
            break;
        }
    }
	
    // check if this connection was terminated
    // HZF: when llProcessRxData(), the link may be terminated
    if ( !connPtr->active )
    {
//#ifdef MULTI_ROLE   
	    ll_scheduler(LL_INVALID_TIME);
//#endif    
        return;
    }
  	
    /*
    ** Check Control Procedure Processing
    */
    if ( llProcessSlaveControlProcedures( connPtr ) == LL_CTRL_PROC_STATUS_TERMINATE )
    {
//#ifdef MULTI_ROLE   
        ll_scheduler(LL_INVALID_TIME);
//#endif

        return;
    }

    /*
    ** Process TX Data Packets
    */
    
    // copy any pending data to the TX FIFO
    llProcessTxData( connPtr, LL_TX_DATA_CONTEXT_POST_PROCESSING );

    // if any fragment l2cap pkt, copy to TX FIFO
    l2capPocessFragmentTxData((uint16)connPtr->connId);
  
    if (connPtr->rx_timeout)
        connPtr->accuTimerDrift += connPtr->timerDrift;
    else
        connPtr->accuTimerDrift = 0;  

    /*
    ** Setup Next Slave Event Timing
    */
	  
    // update next event, calculate time to next event, calculate timer drift,
    // update anchor points, setup NR T2E1 and T2E2 events
    if ( llSetupNextSlaveEvent() == LL_SETUP_NEXT_LINK_STATUS_TERMINATE )
    {
//#ifdef MULTI_ROLE   
	    ll_scheduler(LL_INVALID_TIME);
//#endif
        return;
    }
  
    // schedule next connection event
    // calculate the timer drift due to slave latency
//        llCalcTimerDrift(conn_param[connId].curParam.connInterval, 
//                           conn_param [connId].slaveLatency,
//                           conn_param [connId].scaFactor,
//                           (uint32 *)&conn_param [connId].timerDrift);       // 
       
    // calibrate time:
    // 100 for SW process, 100 for timing advance, 60 for hw engine startup        
    calibra_time = 100 + 100 + 60 ;    
    // calculate the delay
    T2 = read_current_fine_time();
                        
    sw_delay = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);      
    
    // schedule next connection event time
    //  <start time> ------> <anchor point> -----> <loop time>(Interrupt trigger) ------> <next conn event time>
    //  note that slaveLatency should be 0,i.e. no latency if there are data to send in slave
    // should not use conn_param[connId].curParam.connInterval, lastTimeToNextEvt also cover conn parameter update case       
    next_time = connPtr->lastTimeToNextEvt * (connPtr->lastSlaveLatency + 1) * 625;
        
    // rx -> anchor: 110us       
    //next_time = next_time - slave_conn_event_recv_delay - sw_delay - 110 - calibra_time - conn_param[connId].timerDrift ;

    //used to adj next time for multi-link
    ll_adptive_adj_next_time(next_time);      

    int32_t adj_time = slave_conn_event_recv_delay + sw_delay + 110 + calibra_time + connPtr->timerDrift; 
	uint32 aaaa = next_time;
    next_time = (next_time > adj_time || adj_time < 0) ? (next_time - adj_time) : 200;

//#ifdef MULTI_ROLE       
//	DBG_GPIO_WRITE(DBGIO_LL_IRQ,0);
    ll_scheduler(next_time);
//	DBG_GPIO_WRITE(DBGIO_LL_IRQ,1);
//	logx("n-%d-%d-%d\n",next_time,aaaa,adj_time);
	logx("%d\n",next_time);

//#else
//    ll_schedule_next_event(next_time);
//#endif
                  
    return;
}


// global configuration in SRAM, it could be change by application
// TODO: when integrate, the global_config should be set by APP project
uint32_t g_MAC_ADDRESS_LOC[2] = {0x55aa1234, 0x5c5c5c5c};
__ATTR_SECTION_XIP__ void init_config(void)
{
    pGlobal_config = (uint32*)(CONFIG_BASE_ADDR);
    int i;

    for (i = 0; i < 256; i ++)
        pGlobal_config[i] = 0;

    //save the app initial_sp  which will be used in wakeupProcess 20180706 by ZQ
    pGlobal_config[INITIAL_STACK_PTR] = 0;//(uint32_t)&__initial_sp;
    // LL switch setting
    pGlobal_config[LL_SWITCH] =  LL_DEBUG_ALLOW | SLAVE_LATENCY_ALLOW | LL_WHITELIST_ALLOW
                                 | SIMUL_CONN_ADV_ALLOW | SIMUL_CONN_SCAN_ALLOW; //RC32_TRACKINK_ALLOW

    if(g_clk32K_config==CLK_32K_XTAL)
        pGlobal_config[LL_SWITCH] &= 0xffffffee;
    else
        pGlobal_config[LL_SWITCH] |= RC32_TRACKINK_ALLOW | LL_RC32K_SEL;

    // sleep delay
    pGlobal_config[MIN_TIME_TO_STABLE_32KHZ_XOSC] = 10;      // 10ms, temporary set
    // system clock setting
    pGlobal_config[CLOCK_SETTING] = g_system_clk;//CLOCK_32MHZ;
    //------------------------------------------------------------------------
    // wakeup time cose
    // t1. HW_Wakeup->MCU relase 62.5us
    // t2. wakeup_process in waitRTCCounter 30.5us*[WAKEUP_DELAY] about 500us
    // t3. dll_en -> hclk_sel in hal_system_ini 100us in run as RC32M
    // t4. sw prepare cal sleep tick initial rf_ini about 300us @16M this part depends on HCLK
    // WAKEUP_ADVANCE should be larger than t1+t2+t3+t4
    //------------------------------------------------------------------------
    // wakeup advance time, in us
    pGlobal_config[WAKEUP_ADVANCE] = 1350;//650;//600;//310;

    if(g_system_clk==SYS_CLK_XTAL_16M)
    {
        pGlobal_config[WAKEUP_DELAY] = 16;
    }
    else if(g_system_clk==SYS_CLK_DLL_48M)
    {
        pGlobal_config[WAKEUP_DELAY] = 16;
    }
    else if(g_system_clk==SYS_CLK_DLL_64M)
    {
        pGlobal_config[WAKEUP_DELAY] = 16;
    }

    // sleep time, in us
    pGlobal_config[MAX_SLEEP_TIME] = 30000000;
    pGlobal_config[MIN_SLEEP_TIME] = 1600;
    pGlobal_config[ALLOW_TO_SLEEP_TICK_RC32K] = 55;// 30.5 per tick
    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    // LL engine settle time
    pGlobal_config[LL_HW_BB_DELAY] = 54;//54-8;
    pGlobal_config[LL_HW_AFE_DELAY] = 8;
    pGlobal_config[LL_HW_PLL_DELAY] = 40;//45;//52;
    // Tx2Rx and Rx2Tx interval
    //Tx2Rx could be advanced a little
    //Rx2Tx should be ensure T_IFS within150us+-2us
    pGlobal_config[LL_HW_Rx_TO_TX_INTV] = 62-RF_PHY_EXT_PREAMBLE_US;
    pGlobal_config[LL_HW_Tx_TO_RX_INTV] = 50;//65
    //------------------------------------------------2MPHY
    // LL engine settle time
    pGlobal_config[LL_HW_BB_DELAY_2MPHY] = 59;
    pGlobal_config[LL_HW_AFE_DELAY_2MPHY] = 8;
    pGlobal_config[LL_HW_PLL_DELAY_2MPHY] = 40;//45;//52;
    // Tx2Rx and Rx2Tx interval
    //Tx2Rx could be advanced a little
    //Rx2Tx should be ensure T_IFS within150us+-2us
    pGlobal_config[LL_HW_Rx_TO_TX_INTV_2MPHY] = 77-RF_PHY_EXT_PREAMBLE_US;//20200822 ZQ
    pGlobal_config[LL_HW_Tx_TO_RX_INTV_2MPHY] = 57;//72
    //------------------------------------------------CODEPHY 500K
    // LL engine settle time CODEPHY 500K
    pGlobal_config[LL_HW_BB_DELAY_500KPHY] = 50;//54-8;
    pGlobal_config[LL_HW_AFE_DELAY_500KPHY] = 8;
    pGlobal_config[LL_HW_PLL_DELAY_500KPHY] = 40;//45;//52;
    // Tx2Rx and Rx2Tx interval
    //Tx2Rx could be advanced a little
    //Rx2Tx should be ensure T_IFS within150us+-2us
    pGlobal_config[LL_HW_Rx_TO_TX_INTV_500KPHY] =  2;
    pGlobal_config[LL_HW_Tx_TO_RX_INTV_500KPHY] = 66;//72
    //------------------------------------------------CODEPHY 125K
    // LL engine settle time CODEPHY 125K
    pGlobal_config[LL_HW_BB_DELAY_125KPHY] = 30;//54-8;
    pGlobal_config[LL_HW_AFE_DELAY_125KPHY] = 8;
    pGlobal_config[LL_HW_PLL_DELAY_125KPHY] = 40;//45;//52;
    // Tx2Rx and Rx2Tx interval
    //Tx2Rx could be advanced a little
    //Rx2Tx should be ensure T_IFS within150us+-2us
    pGlobal_config[LL_HW_Rx_TO_TX_INTV_125KPHY] = 5;
    pGlobal_config[LL_HW_Tx_TO_RX_INTV_125KPHY] = 66;//72
    // LL engine settle time, for advertisement
    pGlobal_config[LL_HW_BB_DELAY_ADV] = 90;
    pGlobal_config[LL_HW_AFE_DELAY_ADV] = 8;
    pGlobal_config[LL_HW_PLL_DELAY_ADV] = 60;
    // adv channel interval
    pGlobal_config[ADV_CHANNEL_INTERVAL] = 1400;//6250;
    pGlobal_config[NON_ADV_CHANNEL_INTERVAL] = 666;//6250;

    //20201207 Jie modify
    if(g_system_clk==SYS_CLK_XTAL_16M)
    {
        // scan req -> scan rsp timing
        pGlobal_config[SCAN_RSP_DELAY] = 13+RF_PHY_EXT_PREAMBLE_US;//23;
    }
    else if(g_system_clk==SYS_CLK_DLL_48M)
    {
        // scan req -> scan rsp timing
        pGlobal_config[SCAN_RSP_DELAY] = 6 + RF_PHY_EXT_PREAMBLE_US + 6;//20201207 set           //4;        // 12    //  2019/3/19 A2: 12 --> 9
    }
    else if(g_system_clk == SYS_CLK_DLL_64M)        //  2019/3/26 add
    {
        pGlobal_config[SCAN_RSP_DELAY] = 4+RF_PHY_EXT_PREAMBLE_US;//2020.12.07 set         //3;
    }

    // conn_req -> slave connection event calibration time, will advance the receive window
    pGlobal_config[CONN_REQ_TO_SLAVE_DELAY] = 300;//192;//500;//192;
    // calibration time for 2 connection event, will advance the next conn event receive window
    // SLAVE_CONN_DELAY for sync catch, SLAVE_CONN_DELAY_BEFORE_SYNC for sync not catch
    //pGlobal_config[SLAVE_CONN_DELAY] = 300;//0;//1500;//0;//3000;//0;          ---> update 11-20
    pGlobal_config[SLAVE_CONN_DELAY] = 1000;//0;//1500;//0;//3000;//0;          ---> update 11-20
    pGlobal_config[SLAVE_CONN_DELAY_BEFORE_SYNC] = 500;//160 NG//500 OK   //eagle
    // RTLP timeout
    pGlobal_config[LL_HW_RTLP_LOOP_TIMEOUT] = 50000;
    pGlobal_config[LL_HW_RTLP_TO_GAP]       = 1000;
    //pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT]  = 2000 + pGlobal_config[SLAVE_CONN_DELAY] * 2;//500;
//    pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT]  = 1000 + pGlobal_config[SLAVE_CONN_DELAY];//500;//eagle
	pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT]	= 4500;//500;//eagle

    // direct adv interval configuration
    pGlobal_config[HDC_DIRECT_ADV_INTERVAL] = 1000;
    pGlobal_config[LDC_DIRECT_ADV_INTERVAL] = 6250;
    // A1 ROM metal change for HDC direct adv,
    pGlobal_config[DIR_ADV_DELAY] = 115;   // in us, consider both direct adv broadcast time & SW delay, ... etc.
    // A1 ROM metal change
    pGlobal_config[LL_TX_PKTS_PER_CONN_EVT] = 6;//8;
    pGlobal_config[LL_RX_PKTS_PER_CONN_EVT] = 6;//8;
    pGlobal_config[LL_TRX_NUM_ADAPTIVE_CONFIG] = 8;     //0:        disable adaptive
    //other:    adaptive max limitation
//    pGlobal_config[LL_TX_PWR_TO_REG_BIAS]   = 0x15;   // assume when g_rfPhyTxPower = 0x1f, tx power = 10dBm
    //smart window configuration
    pGlobal_config[LL_SMART_WINDOW_COEF_ALPHA]      = 2;
    pGlobal_config[LL_SMART_WINDOW_TARGET]          = 600;
    pGlobal_config[LL_SMART_WINDOW_INCREMENT]       = 9;
    pGlobal_config[LL_SMART_WINDOW_LIMIT]           = 20000;
    pGlobal_config[LL_SMART_WINDOW_ACTIVE_THD]      = 8;
    pGlobal_config[LL_SMART_WINDOW_ACTIVE_RANGE]    = 0;//300
    pGlobal_config[LL_SMART_WINDOW_FIRST_WINDOW]    = 5000;
    g_smartWindowSize = pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT] ;

    //====== A2 metal change add, for scanner & initiator
    if(g_system_clk==SYS_CLK_XTAL_16M)
    {
        pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY]    = 18+RF_PHY_EXT_PREAMBLE_US;//20;      //  2019/3/19 A2: 20 --> 18
        pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY]    = 25+RF_PHY_EXT_PREAMBLE_US;//27;      //  2019/3/19 A2: 27 --> 25
    }
    else if(g_system_clk==SYS_CLK_DLL_48M)
    {
        pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY]    = 8+RF_PHY_EXT_PREAMBLE_US;//12;       //  2019/3/19 A2: 12 --> 10
        pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY]    = 11+RF_PHY_EXT_PREAMBLE_US;
    }
    else if(g_system_clk==SYS_CLK_DLL_64M)
    {
        pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY]    = 6+RF_PHY_EXT_PREAMBLE_US;                //  2019/3/26 add
        pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY]    = 8+RF_PHY_EXT_PREAMBLE_US;
    }

    // TRLP timeout
    pGlobal_config[LL_HW_TRLP_LOOP_TIMEOUT] = 50000;    // enough for 8Tx + 8Rx : (41 * 8 + 150) * 16 - 150 = 7498us
    pGlobal_config[LL_HW_TRLP_TO_GAP]       = 1000;
    pGlobal_config[LL_MOVE_TO_MASTER_DELAY] = 100;
    pGlobal_config[LL_CONN_REQ_WIN_SIZE] = 5;
    pGlobal_config[LL_CONN_REQ_WIN_OFFSET] = 2;
    pGlobal_config[LL_MASTER_PROCESS_TARGET] = 200;   // reserve time for preparing master conn event, delay should be insert if needn't so long time
    pGlobal_config[LL_MASTER_TIRQ_DELAY] = 0;         // timer IRQ -> timer ISR delay
    pGlobal_config[OSAL_SYS_TICK_WAKEUP_TRIM] = 56;  // 0.125us
    pGlobal_config[MAC_ADDRESS_LOC] = (uint32_t)(&g_MAC_ADDRESS_LOC[0]);//0x11004000;
    // for simultaneous conn & adv/scan
    pGlobal_config[LL_NOCONN_ADV_EST_TIME] = 1400*3;
    pGlobal_config[LL_NOCONN_ADV_MARGIN] = 600;
    pGlobal_config[LL_SEC_SCAN_MARGIN] = 2500;//1400;  to avoid mesh proxy llTrigErr 0x15
    pGlobal_config[LL_MIN_SCAN_TIME] = 2000;
    //  BBB new
    pGlobal_config[TIMER_ISR_ENTRY_TIME] = 30;//15;
    pGlobal_config[LL_MULTICONN_MASTER_PREEMP] = 0;
    pGlobal_config[LL_MULTICONN_SLAVE_PREEMP] = 0;
    pGlobal_config[LL_EXT_ADV_TASK_DURATION] = 20000;
    pGlobal_config[LL_PRD_ADV_TASK_DURATION] = 20000;
    pGlobal_config[LL_CONN_TASK_DURATION] = 5000;
    pGlobal_config[LL_EXT_ADV_INTER_PRI_CHN_INT] = 5000;
    pGlobal_config[LL_EXT_ADV_INTER_SEC_CHN_INT] = 5000;
    pGlobal_config[LL_EXT_ADV_PRI_2_SEC_CHN_INT] = 1500;
    pGlobal_config[LL_EXT_ADV_RSC_PERIOD] = 1000000;
    pGlobal_config[LL_EXT_ADV_RSC_SLOT_DURATION] = 10000;
    pGlobal_config[LL_PRD_ADV_RSC_PERIOD] = 1000000;
    pGlobal_config[LL_PRD_ADV_RSC_SLOT_DURATION] = 10000;
    pGlobal_config[LL_EXT_ADV_PROCESS_TARGET] = 500;
    pGlobal_config[LL_PRD_ADV_PROCESS_TARGET] = 500;
    //-------------------------------------------------------------------
    // patch function register
    //--------------------------------------------------------------------
    JUMP_FUNCTION(LL_HW_GO)                         =   (uint32_t)&ll_hw_go1;
    //JUMP_FUNCTION(V4_IRQ_HANDLER)                   =   (uint32_t)&LL_IRQHandler1;
    JUMP_FUNCTION(OSAL_SET_EVENT)                    =   (uint32_t)&osal_set_event1;
    //JUMP_FUNCTION(V20_IRQ_HANDLER)                   =   (uint32_t)&TIM1_IRQHandler1;
    //JUMP_FUNCTION(V11_IRQ_HANDLER)                  =   (uint32_t)&hal_UART0_IRQHandler;
    extern void rf_calibrate1(void);
    JUMP_FUNCTION(RF_CALIBRATTE)                    =   (uint32_t)&rf_calibrate1;
    JUMP_FUNCTION(RF_PHY_CHANGE)                    =   (uint32_t)&rf_phy_change_cfg0;
    //JUMP_FUNCTION(LL_GEN_TRUE_RANDOM)               =   (uint32_t)&LL_ENC_GenerateTrueRandNum1;
    JUMP_FUNCTION(LL_AES128_ENCRYPT)                =   (uint32_t)&LL_ENC_AES128_Encrypt1;
    JUMP_FUNCTION(LL_ENC_ENCRYPT)                   =   (uint32_t)&LL_ENC_Encrypt1;
    JUMP_FUNCTION(LL_ENC_DECRYPT)                   =   (uint32_t)&LL_ENC_Decrypt1;
    //JUMP_FUNCTION(LL_PROCESS_SLAVE_CTRL_PROC)     =   (uint32_t)&llProcessSlaveControlProcedures1;
    //JUMP_FUNCTION(LL_PROCESS_TX_DATA)             =   (uint32_t)&llProcessTxData1;
    //JUMP_FUNCTION(OSAL_POWER_CONSERVE)              =   (uint32_t)&osal_pwrmgr_powerconserve1;
    //JUMP_FUNCTION(ENTER_SLEEP_OFF_MODE)              =   (uint32_t)&enter_sleep_off_mode1;
    //JUMP_FUNCTION(ENTER_SLEEP_PROCESS)              =   (uint32_t)&enterSleepProcess1;
    JUMP_FUNCTION(CONFIG_RTC)                       =   (uint32_t)&config_RTC1;
    JUMP_FUNCTION(LL_SCHEDULER)                     =   (uint32_t)&ll_scheduler1;
//    JUMP_FUNCTION(HAL_DRV_IRQ_ENABLE)               =   (uint32_t)&drv_enable_irq2;
//    JUMP_FUNCTION(HAL_DRV_IRQ_DISABLE)              =   (uint32_t)&drv_disable_irq2;
    //JUMP_FUNCTION(WAKEUP_INIT)                      =   (uint32_t)&wakeup_init1;
    //JUMP_FUNCTION(WAKEUP_PROCESS)                   =   (uint32_t)&wakeupProcess1;
    extern void l2capPocessFragmentTxData(uint16 connHandle);
    JUMP_FUNCTION(L2CAP_PROCESS_FREGMENT_TX_DATA)   =   (uint32_t)&l2capPocessFragmentTxData;
    //BQB bug fix,2020.11.17
    //JUMP_FUNCTION(LL_PHY_MODE_UPDATE)               =   (uint32_t)&LL_PhyUpdate1;
    JUMP_FUNCTION(LL_SET_DATA_LENGTH)               =   (uint32_t)&LL_SetDataLengh1;
    //JUMP_FUNCTION(LL_SET_PHY_MODE)                  =   (uint32_t)&LL_SetPhyMode1;
    JUMP_FUNCTION(LL_PROCESS_TX_DATA)               =   (uint32_t)&llProcessTxData1;
    JUMP_FUNCTION(LL_GENERATE_TX_BUFFER)            =   (uint32_t)&ll_generateTxBuffer1;
    JUMP_FUNCTION(LL_ADP_ADJ_NEXT_TIME)            =   (uint32_t)&ll_adptive_adj_next_time1;
    JUMP_FUNCTION(LL_CONN_TERMINATE)                =   (uint32_t)&llConnTerminate1;
//	JUMP_FUNCTION(LL_SLAVE_EVT_ENDOK)				=	(uint32_t)&llSlaveEvt_TaskEndOk1;
// ====================
    //disableSleep();
    //setSleepMode(MCU_SLEEP_MODE);//SYSTEM_SLEEP_MODE
    enableSleep();
    setSleepMode(SYSTEM_SLEEP_MODE);
}

void ll_patch_slave(void)
{
    JUMP_FUNCTION(LL_SET_ADV_PARAM)                 =   (uint32_t)&LL_SetAdvParam1;
    JUMP_FUNCTION(LL_CALC_MAX_SCAN_TIME)                = (uint32_t)&llCalcMaxScanTime1;
    JUMP_FUNCTION(LL_SEC_ADV_ALLOW)                = (uint32_t)&llSecAdvAllow1;
    JUMP_FUNCTION(LL_SET_ADV_CONTROL)               =   (uint32_t)&LL_SetAdvControl1;
    JUMP_FUNCTION(LL_SETUP_SEC_ADV_ENTRY)           =   (uint32_t)&llSetupSecAdvEvt1;
}

void ll_patch_master(void)
{
    JUMP_FUNCTION(LL_SET_ADV_PARAM)                 =   (uint32_t)&LL_SetAdvParam1;
    JUMP_FUNCTION(LL_SET_ADV_CONTROL)               =   (uint32_t)&LL_SetAdvControl1;
    JUMP_FUNCTION(LL_MASTER_EVT_ENDOK)              = (uint32_t)&llMasterEvt_TaskEndOk1;
    JUMP_FUNCTION(LL_SET_SCAN_PARAM)                = (uint32_t)&LL_SetScanParam1;
    JUMP_FUNCTION(LL_SET_SCAN_CTRL)                 = (uint32_t)&LL_SetScanControl1;
    //JUMP_FUNCTION(LL_PROCESS_MASTER_CTRL_PKT)       = (uint32_t)&llProcessMasterControlPacket1;
    JUMP_FUNCTION(LL_CREATE_CONN)                  =   (uint32_t)&LL_CreateConn1;
    JUMP_FUNCTION(LL_START_ENCRYPT)                 =   (uint32_t)&LL_StartEncrypt1;
    JUMP_FUNCTION(LL_ENC_DECRYPT)                   =   (uint32_t)&LL_ENC_Decrypt1;
    JUMP_FUNCTION(LL_PROCESS_MASTER_CTRL_PROC)      =   (uint32_t)&llProcessMasterControlProcedures1;
    JUMP_FUNCTION(LL_PROCESS_SLAVE_CTRL_PROC)       =   (uint32_t)&llProcessSlaveControlProcedures1;
    JUMP_FUNCTION(LL_PROCESSBASICIRQ_SRX)           =   (uint32_t )&ll_processBasicIRQ_SRX0;
    JUMP_FUNCTION(LL_PROCESSBASICIRQ_SCANTRX)       =   (uint32_t )&ll_processBasicIRQ_ScanTRX0;
    JUMP_FUNCTION(LL_SETUP_SEC_SCAN)                =   (uint32_t )&llSetupSecScan1;
}

/*
void ll_patch_multi(void)
{
    ll_patch_slave();
    ll_patch_master();
    JUMP_FUNCTION(LL_SCHEDULER)                     =   (uint32_t)&ll_scheduler1;
    JUMP_FUNCTION(LL_PROCESSBASICIRQ_SECADVTRX)     =   (uint32_t )&ll_processBasicIRQ_secondaryAdvTRX0;
}
*/

void hal_rom_boot_init(void)
{
    extern void _rom_sec_boot_init();
    //_rom_sec_boot_init();
    ble_main();
}
//-----------------------------------------------------------------------
// Patch for V105/V103 LL_ChanMapUpdate
// Copy chanMap to connPtr->chanMapUpdate.chaMap
hciStatus_t HCI_LE_SetHostChanClassificationCmd(uint8* chanMap)
{
    hciStatus_t status;
    status = LL_ChanMapUpdate(chanMap);

    //patch for LL_ChanMapUpdate
    if (status == LL_STATUS_SUCCESS)
    {
        // need to issue an update on all active connections, if any
        for (uint8_t i = 0; i < g_maxConnNum; i++)
        {
            if (conn_param[i].active)
            {
                llConnState_t* connPtr = &conn_param[i];
                osal_memcpy((uint8_t*)&(connPtr->chanMapUpdate.chanMap[0]), chanMap, LL_NUM_BYTES_FOR_CHAN_MAP);
            }
        }
    }

    //AT_LOG("ChanMap Patch %d \n", status);
    HCI_CommandCompleteEvent(HCI_LE_SET_HOST_CHANNEL_CLASSIFICATION, sizeof(status), &status);
    return (HCI_SUCCESS);
}

/*******************************************************************************
    @fn          pplus_enter_programming_mode

    @brief       force deive enter to programing mode.

    input parameters

    @param       none.

    output parameters

    @param       none.

    @return      none.
void pplus_enter_programming_mode(void)
{
    typedef void (*uart_init_t)(int baud, GPIO_Pin_e tx_pin, GPIO_Pin_e rx_pin,uint32_t cb_addr);
    typedef void (*uart_tx_t)(unsigned char* str);
    typedef void (*uart_cmd_t)(void);
    uart_init_t p_uart_init = (uart_init_t)0x0000b379;
    uart_tx_t p_uart_tx = (uart_tx_t)0x0000b4f5;
    uart_cmd_t p_uart_cmd = (uart_cmd_t)0x00015c51;
    uint32_t _cb_addr = 0x00015c8d;
    *(volatile unsigned int*) 0xe000e180 = 0xffffffff;
    HAL_ENTER_CRITICAL_SECTION();
    osal_memset((void*)0x1fff0000, 0, 256*4);
    HAL_EXIT_CRITICAL_SECTION();
    AP_CACHE->CTRL0 = 0x02;
    AP_PCR->CACHE_RST = 0x02;
    AP_PCR->CACHE_BYPASS = 1;
    *(volatile unsigned int*) 0xe000e100 |= BIT(11);
    p_uart_init(115200,P9, P10,_cb_addr);
    *(volatile unsigned int*) 0x40004004 |= BIT(0);
    p_uart_tx("cmd:");
    __set_MSP(0x1fff1830);
    p_uart_cmd();
}
*/


int8  LL_PLUS_GetCurrentRSSI(void)
{
    uint8 rssi;
    uint16 foff;
    uint8 carrSens;
    rf_phy_get_pktFoot(&rssi,&foff,&carrSens);
    return -rssi;
}

void LL_PLUS_GetCurrentPduDle(uint8_t connId, ll_pdu_length_ctrl_t* ppdu)
{
    if(LL_INVALID_CONNECTION_ID!=connId && ppdu!=NULL)
    {
        ppdu->MaxRxOctets   = conn_param[connId].llPduLen.local.MaxRxOctets;
        ppdu->MaxTxOctets   = conn_param[connId].llPduLen.local.MaxTxOctets;
        ppdu->MaxRxTime     = conn_param[connId].llPduLen.local.MaxRxTime;
        ppdu->MaxTxTime     = conn_param[connId].llPduLen.local.MaxTxTime;
    }
}


void LOG_PATCH_DATA_TIME(void)
{
    LOG("\n");
    LOG("PATCH_LIB:");
//    for(int i=0;i<12;i++)
//    {
//            LOG("%s",libRevisionDate[i]);
//    }
    LOG("%s",libRevisionDate);
    LOG("  ");
    LOG("%s",libRevisionTime);
//    for(int i=0;i<12;i++)
//    {
//            LOG("%s",libRevisionTime[i]);
//    }
    LOG("\n");
}


/*
extern inline uint32_t __psr(void)
{
    uint32_t i;
    __asm
    {
        MRS i, psr
    }
    return i;
}
*/
void rflib_vesion(uint8_t* major, uint8_t* minor, uint8_t* revision, char* test_build)
{
    *major = SDK_VER_MAJOR;
    *minor = SDK_VER_MINOR;
    *revision = SDK_VER_REVISION;
    *test_build = '\0';
    #ifdef SDK_VER_TEST_BUILD
    *test_build = SDK_VER_TEST_BUILD;
    #endif
}


#define OSALMEM_BIGBLK_IDX 157
// ===========================================================
// ptr: the header of osal heap
//uint32  osal_memory_statics(void *ptr)
/*
extern uint8 g_largeHeap[];
uint32  osal_memory_statics(void)
{
    osalMemHdr_t* header, *current;
    void* ptr;
    uint32  sum_alloc = 0;
    uint32  sum_free = 0;
    uint32  max_block = 0;
//    halIntState_t intState;
    ptr = (void*)g_largeHeap;
    header = (osalMemHdr_t*)ptr;
    current = (osalMemHdr_t*)ptr;

//    HAL_ENTER_CRITICAL_SECTION1( intState );  // Hold off interrupts.

    do
    {
        if ((uint32)ptr > (uint32)header + 4096)
        {
            LOG("==========error: memory audit failed===============\r\n");
            break;
        }

        // seek to the last block, return
        if ( current->val == 0 )       /// val = 0, so len = 0
        {
            break;
        }

        if (current->hdr.inUse)
            sum_alloc += current->hdr.len;
        else
        {
            sum_free += current->hdr.len;

            if (current->hdr.len > max_block && (void*)(&current->hdr) > (void*)(header + OSALMEM_BIGBLK_IDX))
                max_block = current->hdr.len;
        }

        current = (osalMemHdr_t*)((uint8*)current + current->hdr.len);
    }
    while (1);

//    HAL_EXIT_CRITICAL_SECTION1( intState );  // Re-enable interrupts.
//    printf("sum_alloc = %d, sum_free = %d, max_free_block = %d\r\n", sum_alloc, sum_free, max_block);
    LOG("sum_alloc = %d, max_free_block = %d ", sum_alloc, max_block);
    return sum_alloc;
}
*/
llStatus_t LL_ConnUpdate1( uint16 connId,
                           uint16 connIntervalMin,
                           uint16 connIntervalMax,
                           uint16 connLatency,
                           uint16 connTimeout,
                           uint16 minLength,
                           uint16 maxLength )
{
    llStatus_t    status;
    llConnState_t* connPtr;
    // unused input parameter; PC-Lint error 715.
    (void)minLength;
    (void)maxLength;

    // make sure we're in Master role
//  if ( llState != LL_STATE_CONN_MASTER )
//  {
//    return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
//  }
    if (g_ll_conn_ctx.scheduleInfo[connId].linkRole != LL_ROLE_MASTER )
        return( LL_STATUS_ERROR_COMMAND_DISALLOWED );

    // sanity checks again to be sure we don't start with bad parameters
    if ( LL_INVALID_CONN_TIME_PARAM( connIntervalMin,
                                     connIntervalMax,
                                     connLatency,
                                     connTimeout ) )
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    // make sure connection ID is valid
    if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
    {
        return( status );
    }

    // get connection info
    connPtr = &conn_param[connId];

    // check if an updated parameters control procedure is already what's pending
    if ( ((connPtr->ctrlPktInfo.ctrlPktCount > 0) &&
            (connPtr->ctrlPktInfo.ctrlPkts[0] == LL_CTRL_CONNECTION_UPDATE_REQ)) ||
            (connPtr->pendingParamUpdate == TRUE) )
    {
        return( LL_STATUS_ERROR_CTRL_PROC_ALREADY_ACTIVE );
    }

    // check if CI/SL/LSTO is valid (i.e. meets the requirements)
    // Note: LSTO > (1 + Slave Latency) * (Connection Interval * 2)
    // Note: The CI * 2 requirement based on ESR05 V1.0, Erratum 3904.
    // Note: LSTO time is normalized to units of 1.25ms (i.e. 10ms = 8 * 1.25ms).
    if ( LL_INVALID_CONN_TIME_PARAM_COMBO(connIntervalMax, connLatency, connTimeout) )
    {
        return( LL_STATUS_ERROR_ILLEGAL_PARAM_COMBINATION );
    }

    // if there is at least one connection, make sure this connection interval
    // is a multiple/divisor of all other active connection intervals; also make
    // sure that this connection's interval is not less than the allowed maximum
    // connection interval as determined by the maximum number of allowed
    // connections times the number of slots per connection.
    if ( g_ll_conn_ctx.numLLMasterConns > 1 )        //   if ( g_ll_conn_ctx.numLLConns > 0 )
    {
        uint16 connInterval = (connIntervalMax << 1);      // convert to 625us ticks
        uint16 minCI        = g_ll_conn_ctx.connInterval;

        //    // first check if this connection interval is even legal
        //    // Note: The number of active connections is limited by the minCI.
        //    if ( (minCI / NUM_SLOTS_PER_MASTER) < llConns.numActiveConns )
        //    {
        //      return( LL_STATUS_ERROR_UNACCEPTABLE_CONN_INTERVAL );
        //    }

        //    // does the CI need to be checked as a multiple of the minCI?
        if ( connInterval >= minCI )
        {
            // check if this connection's CI is valid (i.e. a multiple of minCI)
            if ( connInterval % minCI )
            {
                return( LL_STATUS_ERROR_UNACCEPTABLE_CONN_INTERVAL );
            }
        }
        else
            return( LL_STATUS_ERROR_UNACCEPTABLE_CONN_INTERVAL );
    }
    else
    {
        // only 1 master connection
        g_ll_conn_ctx.connInterval = connIntervalMax;
        g_ll_conn_ctx.per_slot_time = connPtr->curParam.connInterval * 2 / g_maxConnNum;       // unit: 625us
    }

    // no control procedure currently active, so set this one up
    // set the window size (units of 1.25ms)
    connPtr->paramUpdate.winSize = LL_WINDOW_SIZE;
    // set the window offset (units of 1.25ms)
//  connPtr->paramUpdate.winOffset = LL_WINDOW_OFFSET;
    connPtr->paramUpdate.winOffset = 0;                     // multiconnection, this value could be 0 or x * old conn interval and should be less than new conn interval
    // set the relative offset of the number of events for the parameter update
    // Note: The absolute event number will be determined at the time the packet
    //       is placed in the TX FIFO.
    // Note: The master should allow a minimum of 6 connection events that the
    //       slave will be listening for before the instant occurs.
    connPtr->paramUpdateEvent = (connPtr->curParam.slaveLatency+1) +
                                LL_INSTANT_NUMBER_MIN;
    // determine the connection interval based on min and max values
    // Note: Range not used, so assume max value.
    // Note: minLength and maxLength are informational.
    connPtr->paramUpdate.connInterval = connIntervalMax;
    // save the new connection slave latency to be used by the peer
    connPtr->paramUpdate.slaveLatency = connLatency;
    // save the new connection supervisor timeout
    connPtr->paramUpdate.connTimeout  = connTimeout;
    // queue control packet for processing
    llEnqueueCtrlPkt( connPtr, LL_CTRL_CONNECTION_UPDATE_REQ );
    return( LL_STATUS_SUCCESS );
}

hciStatus_t HCI_LE_ConnUpdateCmd( uint16 connHandle,
                                  uint16 connIntervalMin,
                                  uint16 connIntervalMax,
                                  uint16 connLatency,
                                  uint16 connTimeout,
                                  uint16 minLen,
                                  uint16 maxLen )
{
    hciStatus_t status;
    status = LL_ConnUpdate1( connHandle,
                             connIntervalMin,
                             connIntervalMax,
                             connLatency,
                             connTimeout,
                             minLen,
                             maxLen );
    HCI_CommandStatusEvent( status, HCI_LE_CONNECTION_UPDATE );
    return( HCI_SUCCESS );
}

CHIP_ID_STATUS_e chip_id_one_bit_hot_convter(uint8_t* b,uint32_t w)
{
    uint16 dh = w>>16;
    uint16 dl = w&0xffff;
    uint16 h1,h0,l1,l0;
    h0=l0=0xff;
    h1=l1=0;

    for(int i=0; i<16; i++)
    {
        l1+=((dl&(1<<i))>>i);

        if(l0==0xff && l1==1)
        {
            l0=i;
        }

        h1+=((dh&(1<<i))>>i);

        if(h0==0xff && h1==1)
        {
            h0=i;
        }
    }

    if(l1==1 && h1==1)
    {
        *b=((h0<<4)+l0);
        return CHIP_ID_VALID;
    }
    else if(l1==16 && h1==16)
    {
        return CHIP_ID_EMPTY;
    }
    else
    {
        return CHIP_ID_INVALID;
    }
}

/*******************************************************************************
    @fn          LL_PLUS_LoadMACFromFlash

    @brief       Used to load MAC Address from Flash

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      None.
*/
void LL_PLUS_LoadMACFromFlash(uint32_t addr)
{
    volatile uint8_t* p_ownPublicAddr = (volatile uint8_t*)0x1fff0965;
    uint32_t macAddr[2];
    macAddr[0]=*(volatile uint32_t*) (0x11000000+addr);
    macAddr[1]=*(volatile uint32_t*) (0x11000000+addr+4);
    *(p_ownPublicAddr++) = BREAK_UINT32(macAddr[0],3);
    *(p_ownPublicAddr++) = BREAK_UINT32(macAddr[0],2);
    *(p_ownPublicAddr++) = BREAK_UINT32(macAddr[0],1);
    *(p_ownPublicAddr++) = BREAK_UINT32(macAddr[0],0);
    *(p_ownPublicAddr++) = BREAK_UINT32(macAddr[1],1);
    *(p_ownPublicAddr++) = BREAK_UINT32(macAddr[1],0);
}


/*******************************************************************************
    @fn          pplus_LoadMACFromChipMAddr

    @brief       Used to load MAC Address from chip Maddr

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      CHIP_ID_STATUS_e.
*/
CHIP_ID_STATUS_e LL_PLUS_LoadMACFromChipMAddr(void)
{
    //check_chip_mAddr();
    volatile uint8_t* p_ownPublicAddr = (volatile uint8_t*)0x1fff0965;

    if(g_chipMAddr.chipMAddrStatus==CHIP_ID_VALID)
    {
        for(uint8_t i =0; i<CHIP_MADDR_LEN; i++)
            *(p_ownPublicAddr++) = g_chipMAddr.mAddr[i];
    }

    return g_chipMAddr.chipMAddrStatus;
}

extern const char* s_company_id;

__attribute__((aligned(4))) static uint8_t s_trng_seed[16];
__attribute__((aligned(4))) static uint8_t s_trng_iv[16];

__ATTR_SECTION_XIP__ static void TRNG_Output(uint32_t* buf, uint8_t len)
{
    uint32_t temp,temp1,status;
    temp = *(volatile uint32_t*)0x4000f05c;
    *(volatile uint32_t*)0x4000f05c = (temp & 0xfffefe00) | 0x0108;

    for(uint8_t j=0; j<len; j++)
    {
        status = 0;

        for(uint8_t i = 0; i<16; i++)
        {
            WaitRTCCount(17);
            temp1 = *(volatile uint32_t*)0x4000f064 & 0x1ffff;
            status |= ((temp1&0x03)<<(i<<1));
        }

        *buf++ = status;
    }

    return;
}
__ATTR_SECTION_XIP__ static void TRNG_IV_Updata()
{
    *(uint32*)(&s_trng_iv[0]) +=read_current_fine_time();
    *(uint32*)(&s_trng_iv[4]) +=read_current_fine_time();
    *(uint32*)(&s_trng_iv[8]) +=read_current_fine_time();
    *(uint32*)(&s_trng_iv[12])+=read_current_fine_time();
}
__ATTR_SECTION_XIP__ void TRNG_INIT(void)
{
    static uint8_t init_flag = 0;

    if (!init_flag)
    {
        TRNG_Output((uint32_t*)(s_trng_seed), 4);
        TRNG_Output((uint32_t*)(s_trng_iv), 4);
        init_flag = 1;
    }

    return;
}
const char* s_company_id = "PHY+62XXPLUS0504";

__ATTR_SECTION_XIP__ uint8_t TRNG_Rand(uint8_t* buf,uint8_t len)
{
    uint32_t t0=0;
    uint8_t i;
    uint8_t cryOut[16];
    uint8_t cryIn[16];
    uint8_t rand_len = 0;

    if(len > 16)
    {
        return PPlus_ERR_FATAL;
    }

    TRNG_INIT();

    for(i=0; i<16; i++)
        t0+=s_trng_seed[i];

    if(t0==0)
        return PPlus_ERR_NULL;

    if(len>16)
        return PPlus_ERR_DATA_SIZE;

    for(i=0; i<16; i++)
        cryIn[i] =s_trng_iv[i]^s_company_id[i];

    LL_ENC_AES128_Encrypt(s_trng_seed,cryIn,cryOut);
    rand_len = len > 16 ? 16 : len;
    osal_memcpy(buf,cryOut,rand_len);
    TRNG_IV_Updata();
    return PPlus_SUCCESS;
}


// bugfix for multi-Role
/*******************************************************************************
    @fn          LL_EncLtkReply API

    @brief       This API is called by the HCI to provide the controller with
                the Long Term Key (LTK) for encryption. This command is
                actually a reply to the link layer's LL_EncLtkReqCback, which
                provided the random number and encryption diversifier received
                from the Master during an encryption setup.

                Note: The key parameter is byte ordered LSO to MSO.

    input parameters

    @param       connId - The LL connection ID on which to send this data.
    @param       *key   - A 128 bit key to be used to calculate the session key.

    output parameters

    @param       None.

    @return      LL_STATUS_SUCCESS
*/
llStatus_t LL_EncLtkReply( uint16 connId,
                           uint8*  key )
{
    uint8         i;
    llStatus_t    status;
    llConnState_t* connPtr;
    // get connection info
    connPtr = &conn_param[ connId ];

    // make sure we're in Master role
//  if ( llState != LL_STATE_CONN_SLAVE )
    /* asynchronous send msg can not make sure llState = LL_STATE_CONN_SLAVE in multi-role */
    if( connPtr->llTbd1 != LL_LINK_CONNECT_COMPLETE_SLAVE )
    {
        return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
    }

    // check parameters
    if ( key == NULL )
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    // make sure connection ID is valid
    if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
    {
        return( status );
    }

    // ALT: COULD MAKE THIS PER CONNECTION.

    // save LTK
    for (i=0; i<LL_ENC_LTK_LEN; i++)
    {
        // store LTK in MSO..LSO byte order, per FIPS 197 (AES)
        connPtr->encInfo.LTK[(LL_ENC_LTK_LEN-i)-1] = key[i];
    }

    // indicate the host has provided the key
    connPtr->encInfo.LTKValid = TRUE;
    // got the LTK, so schedule the start of encryption
    // ALT: COULD MAKE THIS A REPLACE IF A DUMMY IS SITTING AT THE HEAD OF
    //      THE QUEUE.
    llEnqueueCtrlPkt( connPtr, LL_CTRL_START_ENC_REQ );
    return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
    @fn          LL_EncLtkNegReply API

    @brief       This API is called by the HCI to indicate to the controller
                that the Long Term Key (LTK) for encryption can not be provided.
                This command is actually a reply to the link layer's
                LL_EncLtkReqCback, which provided the random number and
                encryption diversifier received from the Master during an
                encryption setup. How the LL responds to the negative reply
                depends on whether this is part of a start encryption or a
                re-start encryption after a pause. For the former, an
                encryption request rejection is sent to the peer device. For
                the latter, the connection is terminated.

    input parameters

    @param       connId - The LL connection ID on which to send this data.

    output parameters

    @param       None.

    @return      LL_STATUS_SUCCESS
*/
llStatus_t LL_EncLtkNegReply( uint16 connId )
{
    llStatus_t    status;
    llConnState_t* connPtr;
    // get connection info
    connPtr = &conn_param[ connId ];

//  // make sure we're in Master role
//  if ( llState != LL_STATE_CONN_SLAVE )
    /* asynchronous send msg can not make sure llState = LL_STATE_CONN_SLAVE in multi-role */
    if( connPtr->llTbd1 != LL_LINK_CONNECT_COMPLETE_SLAVE )
    {
        return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
    }

    // make sure connection ID is valid
    if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
    {
        return( status );
    }

    // check if this is during a start or a re-start encryption procedure
    if ( connPtr->encInfo.encRestart == TRUE )
    {
        // indicate the peer requested this termination
        connPtr->termInfo.reason = LL_ENC_KEY_REQ_REJECTED;
        // queue control packet for processing
        // ALT: COULD MAKE THIS A REPLACE IF A DUMMY IS SITTING AT THE HEAD OF
        //      THE QUEUE.
        //llReplaceCtrlPkt( connPtr, LL_CTRL_TERMINATE_IND );
        llEnqueueCtrlPkt( connPtr, LL_CTRL_TERMINATE_IND );
    }
    else // during a start encryption
    {
        // set the encryption rejection error code
        connPtr->encInfo.encRejectErrCode = LL_STATUS_ERROR_PIN_OR_KEY_MISSING; // same as LL_ENC_KEY_REQ_REJECTED
        // and reject the encryption request
        // ALT: COULD MAKE THIS A REPLACE IF A DUMMY IS SITTING AT THE HEAD OF
        //      THE QUEUE.
        //llReplaceCtrlPkt( connPtr, LL_CTRL_REJECT_IND );
        llEnqueueCtrlPkt( connPtr, LL_CTRL_REJECT_IND );
    }

    return( LL_STATUS_SUCCESS );
}

hciStatus_t HCI_LE_LtkReqReplyCmd( uint16 connHandle,
                                   uint8*  ltk )
{
    // 0: Status
    // 1: Connection Handle (LSB)
    // 2: Connection Handle (MSB)
    uint8 rtnParam[3];
    rtnParam[0] = LL_EncLtkReply( connHandle, ltk );
    rtnParam[1] = LO_UINT16( connHandle );
    rtnParam[2] = HI_UINT16( connHandle );
    HCI_CommandCompleteEvent( HCI_LE_LTK_REQ_REPLY, sizeof(rtnParam), rtnParam );
    return ( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used by the Host to send to the Controller a negative LTK
    reply.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_LtkReqNegReplyCmd( uint16 connHandle )
{
    // 0: Status
    // 1: Connection Handle (LSB)
    // 2: Connection Handle (MSB)
    uint8 rtnParam[3];
    rtnParam[0] = LL_EncLtkNegReply( connHandle );
    rtnParam[1] = LO_UINT16( connHandle );
    rtnParam[2] = HI_UINT16( connHandle );
    HCI_CommandCompleteEvent( HCI_LE_LTK_REQ_NEG_REPLY, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}




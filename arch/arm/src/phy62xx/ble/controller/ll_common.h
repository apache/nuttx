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

#ifndef _LL_H_
#define _LL_H_

#include "types.h"
#include "mcu.h"
#include "ll.h"
#include "ll_def.h"

#define LL_DATA_PDU( pktHdr )     ((pktHdr) != LL_DATA_PDU_HDR_LLID_CONTROL_PKT)
#define LL_CTRL_PDU( pktHdr )     ((pktHdr) == LL_DATA_PDU_HDR_LLID_CONTROL_PKT)
#define LL_INVALID_LLID( pktHdr ) ((pktHdr) == LL_DATA_PDU_HDR_LLID_RESERVED)


void LL_IRQHandler(void);

void move_to_slave_function(void);

void LL_slave_conn_event(void);

void LL_master_conn_event(void);

void LL_set_default_conn_params(llConnState_t* connPtr);

//void ll_setMem(uint8_t  *buf, uint8_t v, int n);

//void ll_cpyMem(uint8_t *dst, uint8_t *src, int n);

void LL_evt_schedule(void);

llStatus_t llSetupAdv( void );

void llConvertLstoToEvent( llConnState_t* connPtr,
                           connParam_t*   connParams );

void llSlaveEvt_TaskEndOk( void );

// for master process
void llMasterEvt_TaskEndOk( void );

// Connection Management
extern llConnState_t*      llAllocConnId( void );
extern void               llReleaseConnId( llConnState_t* connPtr );
extern void               llReleaseAllConnId( void );
extern uint16             llGetMinCI( uint16 connInterval );
extern uint8              llGetNextConn( void );
extern void               llConnCleanup( llConnState_t* connPtr );
extern void               llConnTerminate( llConnState_t* connPtr, uint8 reason );
extern uint8              llPendingUpdateParam( void );
extern void               llInitFeatureSet( void );
extern uint32             llGenerateValidAccessAddr( void );
extern uint32             llGenerateCRC( void );
extern  uint8             llEventInRange( uint16 curEvent, uint16 nextEvent, uint16 updateEvent );
extern  uint16            llEventDelta( uint16 eventA, uint16 eventB );
extern void               llConvertLstoToEvent( llConnState_t* connPtr, connParam_t* connParams );
extern void               llConvertCtrlProcTimeoutToEvent( llConnState_t* connPtr );

// Task Setup
extern llStatus_t         llSetupAdv( void );
extern void               llSetupDirectedAdvEvt( void );
extern void               llSetupUndirectedAdvEvt( void );
extern void               llSetupNonConnectableAdvEvt( void );
extern void               llSetupScannableAdvEvt( void );
extern void               llSetupScan( uint8 chan );
extern void               llSetupScanInit( void );
extern void               llSetupInit( uint8 connId );
extern void               llSetupConn( void );
// A2 added
extern uint8              llSetupSecNonConnectableAdvEvt( void );
// A2 multi-connection
extern uint8              llSetupSecConnectableAdvEvt( void );
extern uint8              llSetupSecScannableAdvEvt( void );


extern void               llSetupSecScan( uint8 chan );
extern uint32             llCalcMaxScanTime(void);
extern uint8              llSecAdvAllow(void);
// A2 multi-connection
extern uint8              llSetupSecAdvEvt( void );

extern void               llSetupSecInit( uint8 chan );
extern uint8_t            ll_get_next_active_conn(uint8_t current_conn_id);
extern uint32             ll_get_next_timer(uint8 current_conn_id);

extern void               ll_scheduler(uint32 time);

extern void               ll_addTask(uint8 connId, uint32 time);
extern void               ll_deleteTask(uint8 connId);

// extended adv scheduler functions
void ll_adv_scheduler(void);

void ll_add_adv_task(extAdvInfo_t* pExtAdv);

void ll_delete_adv_task(uint8 index);

uint8 llSetupExtAdvEvent(extAdvInfo_t*  pAdvInfo);

// periodic adv functions
void ll_add_adv_task_periodic(periodicAdvInfo_t* pPrdAdv, extAdvInfo_t* pExtAdv);

void ll_add_adv_task_periodic(periodicAdvInfo_t* pPrdAdv, extAdvInfo_t* pExtAdv);

void ll_delete_adv_task_periodic(uint8 index);

uint8 llSetupPrdAdvEvent(periodicAdvInfo_t* pPrdAdv, extAdvInfo_t* pExtAdv);

void ll_adv_scheduler_periodic(void);


// extended scan functions
extern void llSetupExtScan( uint8 chan );

extern void llSetupExtInit(void);

extern void llSetupPrdScan( void );

extern uint16 llAllocateSyncHandle(void);

extern uint8 llDeleteSyncHandle(uint16 sync_handle);



// Data Management
extern uint8              llEnqueueDataQ( llDataQ_t* pDataQ, txData_t* pTxData );
extern uint8              llEnqueueHeadDataQ( llDataQ_t* pDataQ, txData_t* pTxData );
extern txData_t*          llDequeueDataQ( llDataQ_t* pDataQ );
extern uint8              llDataQEmpty( llDataQ_t* pDataQ );
extern uint8              llWriteTxData ( llConnState_t* connPtr, uint8 pktHdr, uint8 pktLen, uint8* pBuf );
extern uint8*              llMemCopySrc( uint8* pDst, uint8* pSrc, uint8 len );
extern uint8*              llMemCopyDst( uint8* pDst, uint8* pSrc, uint8 len );
extern void               llProcessMasterControlPacket( llConnState_t* connPtr, uint8* pBuf );
extern void               llProcessSlaveControlPacket( llConnState_t* connPtr, uint8* pBuf );
extern  void              llProcessTxData( llConnState_t* connPtr, uint8 context );
extern  uint8             llProcessRxData( void );

// Control Procedure Setup
extern uint8              llSetupUpdateParamReq( llConnState_t* connPtr );  // M
extern uint8              llSetupUpdateChanReq( llConnState_t* connPtr );   // M
extern uint8              llSetupEncReq( llConnState_t* connPtr );          // M
extern uint8              llSetupEncRsp( llConnState_t* connPtr );          // S
extern uint8              llSetupStartEncReq( llConnState_t* connPtr );     // S
extern uint8              llSetupStartEncRsp( llConnState_t* connPtr );     // M, S
extern uint8              llSetupPauseEncReq( llConnState_t* connPtr );     // M
extern uint8              llSetupPauseEncRsp( llConnState_t* connPtr );     // S
extern uint8              llSetupRejectInd( llConnState_t* connPtr,uint8 errCode);        // S
extern uint8              llSetupFeatureSetReq( llConnState_t* connPtr );   // M, S
extern uint8              llSetupFeatureSetRsp( llConnState_t* connPtr );   // M, S
extern uint8              llSetupVersionIndReq( llConnState_t* connPtr );   // M
extern uint8              llSetupTermInd( llConnState_t* connPtr );         // M, S
extern uint8              llSetupUnknownRsp( llConnState_t* connPtr );      // M, S

extern uint8              llSetupDataLenghtReq( llConnState_t* connPtr );//M,S
extern uint8              llSetupDataLenghtRsp( llConnState_t* connPtr );//M,S
extern uint8              llSetupPhyReq( llConnState_t* connPtr );          //M,S
extern uint8              llSetupPhyRsp( llConnState_t* connPtr );   //M,S
extern uint8              llSetupPhyUpdateInd( llConnState_t* connPtr );//M
extern uint8              llSetupRejectExtInd( llConnState_t* connPtr,uint8 errCode);

// Control Procedure Management
extern void               llEnqueueCtrlPkt( llConnState_t* connPtr, uint8 ctrlType );
extern void               llDequeueCtrlPkt( llConnState_t* connPtr );
extern void               llReplaceCtrlPkt( llConnState_t* connPtr, uint8 ctrlType );


// Data Channel Management
extern void               llProcessChanMap( llConnState_t* connPtr, uint8* chanMap );
extern  uint8    llGetNextDataChan( llConnState_t* connPtr, uint16 numEvents );
extern  void     llSetNextDataChan( llConnState_t* connPtr );
extern uint8              llAtLeastTwoChans( uint8* chanMap );

//2020-01-20 add for LL CTE
extern uint8            llSetupCTEReq( llConnState_t* connPtr );
extern uint8            llSetupCTERsp( llConnState_t* connPtr );



uint8_t llTimeCompare(int base_time, int fine_time);
uint32_t calculateTimeDelta(int base_time, int  fine_time);

void llSetNextDataChan( llConnState_t* connPtr );

// White List Related
extern llStatus_t         llCheckWhiteListUsage( void );

// function add by HZF
void llResetConnId( uint8 connId );
void llResetRfCounters(void);
extern void               llInitFeatureSet( void );


extern  uint16 llCalcScaFactor( uint8 masterSCA );


extern void llCalcTimerDrift( uint32    connInterval,
                              uint16    slaveLatency,
                              uint8     sleepClkAccuracy,
                              uint32* timerDrift );


// add by HZF
uint8 llGetNextAdvChn(uint8 cur_chn);

// Tx loop buffer process
void update_tx_write_ptr(llConnState_t* connPtr);

void update_tx_read_ptr(llConnState_t* connPtr);

uint8_t getTxBufferSize(llConnState_t* connPtr);
uint8_t getTxBufferFree(llConnState_t* connPtr);

uint8_t get_tx_read_ptr(llConnState_t* connPtr);

uint8_t get_tx_write_ptr(llConnState_t* connPtr);

// Rx loop buffer process
void update_rx_write_ptr(llConnState_t* connPtr);

void update_rx_read_ptr(llConnState_t* connPtr);

uint8_t getRxBufferSize(llConnState_t* connPtr);
uint8_t getRxBufferFree(llConnState_t* connPtr);

uint8_t get_rx_read_ptr(llConnState_t* connPtr);

uint8_t get_rx_write_ptr(llConnState_t* connPtr);

// reset buffer
void reset_conn_buf(uint8 index);

void ll_schedule_next_event(int time);

uint16 ll_generateTxBuffer(int txFifo_vacancy, uint16* pSave_ptr);

void ll_read_rxfifo(void);
void ll_hw_read_tfifo_rtlp(void);
int ll_hw_read_tfifo_packet(uint8* pkt);

// function in ll_slaveEndCause.c
uint8 llSetupNextSlaveEvent( void );
uint8 llProcessSlaveControlProcedures( llConnState_t* connPtr );
uint8 llCheckForLstoDuringSL( llConnState_t* connPtr );

// function in ll_hwItf.c
void ll_hw_process_RTO(uint32 ack_num);
void ll_debug_output(uint32 state);

void llAdjSlaveLatencyValue( llConnState_t* connPtr );

//function for DLE add by ZQ
void llPduLengthManagmentReset(void);
void llTrxNumAdaptiveConfig(void);
void llPduLengthUpdate(uint16 connHandle);
//uint8 LL_PLUS_GetLocalPduDataLength(ll_pdu_length_ctrl_t * pduLen);

//function for PHY UPDATE add by ZQ
void llPhyModeCtrlReset(void);
void llPhyModeCtrlUpdateNotify(llConnState_t* connPtr, uint8 status);
//llStatus_t LL_PLUS_GetLocalPhyMode(ll_phy_ctrl_t * phyCtrl);
void llSetNextPhyMode( llConnState_t* connPtr );
extern void llInitFeatureSetDLE(uint8 enable);
extern void llInitFeatureSet2MPHY(uint8 enable);
extern void llInitFeatureSetCodedPHY(uint8 enable);

// function for whitelist
extern uint8 ll_isAddrInWhiteList(uint8 addrType, uint8* addr);

// function for resolving list
uint8 ll_readLocalIRK(uint8** localIrk, uint8* peerAddr, uint8 peerAddrType);
uint8 ll_readPeerIRK(uint8** peerIrk, uint8* peerAddr, uint8 peerAddrType);
uint8_t ll_getRPAListEntry(uint8* peerAddr);

uint8_t ll_isIrkAllZero(uint8* irk);

uint8_t ll_CalcRandomAddr( uint8* pIRK, uint8* pNewAddr );
uint8_t ll_ResolveRandomAddrs(uint8* pIRK, uint8* pAddr);

uint16  ll_generateExtAdvDid(uint16 old);

// extended advertiser process
uint8 LL_extAdvTimerExpProcess(void);

uint8 LL_prdAdvTimerExpProcess(void);

uint8 LL_prdScanTimerExpProcess(void);

uint8 ll_isFirstAdvChn(uint8 chnMap, uint8 chan);

uint8 ll_getFirstAdvChn(uint8 chnMap);

void ll_ext_adv_schedule_next_event(int time);

void ll_prd_adv_schedule_next_event(int time);

void ll_ext_scan_schedule_next_event(int time);

void ll_ext_init_schedule_next_event(int time);

void ll_prd_scan_schedule_next_event(int time);


uint8 ll_allocAuxAdvTimeSlot(uint8 index);

void ll_updateAuxAdvTimeSlot(uint8 index);

void ll_updateExtAdvRemainderTime(uint32 time);


uint8 ll_allocAuxAdvTimeSlot_prd(uint8 index);

void LL_extScanTimerExpProcess(void);

void LL_extInitTimerExpProcess(void);

void ll_parseExtHeader(uint8* payload, uint16 length);

uint8  llGetNextAuxAdvChn(uint8 current);


/******************************************************************************
    fn:  llGetNextDataChanCSA2

    brief:   2020-01-07 add for CSA 2

    input parameters:    counter :   event counter( PA Counter or connection counter)
                        chan_id :   current data or periodic advertising channel Identifier,
                                    calculate from Access Address
                        chan_map:   PA or connection channel map
                        cMap_tab:   current chan map table that is in use for connection or PA event
                        chanCnt :   used channel count

    output parameters:   None


    return       uint8           :   next channel index

 ******************************************************************************/
uint8 llGetNextDataChanCSA2(uint16_t counter,uint16_t chan_id,uint8* chan_map,uint8* cMap_tab,uint8 chanCnt);

#endif


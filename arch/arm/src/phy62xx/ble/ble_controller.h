/****************************************************************************
 * arch/arm/src/phy62xx/ble/ble_controller.h
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

#ifndef _BLE_CONTROLLER_H_
#define _BLE_CONTROLLER_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#define HCI_SUCCESS                             0x00
#define LL_MAX_NUM_CTRL_PROC_PKTS               4
#define LL_ENC_IV_M_LEN                         4
#define LL_ENC_IV_S_LEN                         4

#define LL_ENC_IV_LEN                           (LL_ENC_IV_M_LEN + LL_ENC_IV_S_LEN)
#define LL_ENC_SKD_M_LEN                        8
#define LL_ENC_SKD_S_LEN                        8

#define LL_ENC_SKD_LEN                          (LL_ENC_SKD_M_LEN + LL_ENC_SKD_S_LEN)

#define LL_ENC_RAND_LEN                         8
#define LL_ENC_EDIV_LEN                         2
#define LL_ENC_NONCE_LEN                        13
#define LL_ENC_SK_LEN                           16
#define LL_ENC_LTK_LEN                          16

#define MAX_LL_BUF_LEN                          8    /* maximum LL buffer for Rx/Tx packet */
#define LL_MAX_NUM_DATA_CHAN                    37   /* 0 - 36 */

#define LL_CTE_MAX_PATTERN_LEN                  16

#define B_ADDR_LEN                              6
#define TX_CTRL_BUF_LEN                         34   /* (27+4+3) */

#define LL_DEVICE_ADDR_LEN                      6

#define HCI_EVENT_MIN_LENGTH                    3
#define HCI_DATA_MIN_LENGTH                     5

/****************************************************************************
 * TYPEDEFS
 ****************************************************************************/

typedef enum
{
    MCU_SLEEP_MODE,
    SYSTEM_SLEEP_MODE,
    SYSTEM_OFF_MODE
}  Sleep_Mode;

typedef struct
{
    uint16 CID;      /* local channel id */
    uint8 *pPayload;

  /* pointer to information payload. This contains the payload
   * received from the upper layer protocol (outgoing packet),
   * or delivered to the upper layer protocol (incoming packet).
   */

    uint16 len;      /* length of information payload */
} l2capPacket_t;

typedef struct
{
    uint8  len;           /* pkt len */
    uint8 *ptr ;          /* pkt point */
} segmentBuff_t;

typedef struct
{
    segmentBuff_t pkt[10];  /* 251/27->9.2 */
    uint8 depth;
    uint8 idx;
    uint8 *pBufScr;         /* source buffer ptr */
    uint8 fragment;
} l2capSegmentBuff_t;

typedef struct
{
    uint8  event;
    uint8  status;
} osal_event_hdr_t;

typedef struct
{
    osal_event_hdr_t  hdr;
    uint8  status;
    uint16 connHandle;      /* connection handle */
    uint8  reason;
} hciEvt_DisconnComplete_t;

typedef struct
{
    uint8_t  winSize;                              /* window size */
    uint16_t winOffset;                            /* window offset */
    uint16_t connInterval;                         /* connection interval */
    uint16_t slaveLatency;                         /* number of connection events the slave can ignore */
    uint16_t connTimeout;                          /* supervision connection timeout */
} connParam_t;

/* Channel Map */

typedef struct
{
    uint8_t chanMap[5];                            /* bit map corresponding to the data channels 0..39 */
} chanMap_t;

typedef struct
{
    uint8_t  chanMap[5];
    uint16_t chanMapUpdateEvent;                  /* event count to indicate when to apply pending chan map update */
    uint8_t  chanMapUpdated;
} preChanMapUpdate_t;

/* TX Data */

typedef struct txData_t
{
    struct txData_t *pNext;                       /* pointer to next Tx data entry on queue */
} txData_t;

/* Data Packet Queue */

typedef struct
{
    txData_t *head;                               /* pointer to head of queue */
    txData_t *tail;                               /* pointer to tail of queue */
} llDataQ_t;

/* Control Procedure Information */

typedef struct
{
    uint8_t  ctrlPktActive;                              /* flag that indicates a control packet is being processed */
    uint8_t  ctrlPkts[LL_MAX_NUM_CTRL_PROC_PKTS];        /* queue of control packets to be processed */
    uint8_t  ctrlPktCount;                               /* number of queued control packets */
    uint16_t ctrlTimeoutVal;                             /* timeout in CI events for control procedure for this connection */
    uint16_t ctrlTimeout;                                /* timeout counter in CI events for control procedure */
} ctrlPktInfo_t;

/* Encryption */

typedef struct
{
  /* Note: IV and SKD provide enough room for the full IV and SKD. When the
   *       Master and Slave values are provided, the result is one combined
   *       (concatenated) value.
   */

    uint8  IV[LL_ENC_IV_LEN];                        /* combined master and slave IV values concatenated */
    uint8  SKD [LL_ENC_SKD_LEN];                     /* combined master and slave SKD values concatenated */
    uint8  RAND[LL_ENC_RAND_LEN];                    /* random vector from Master */
    uint8  EDIV[LL_ENC_EDIV_LEN];                    /* encrypted diversifier from Master */
    uint8  nonce[LL_ENC_NONCE_LEN];                  /* current nonce with current IV value */
    uint8  SK[LL_ENC_SK_LEN];                        /* session key derived from LTK and SKD */
    uint8  LTK[LL_ENC_LTK_LEN];                      /* Long Term Key from Host */
    uint8  SKValid;                                  /* flag that indicates the Session Key is valid */
    uint8  LTKValid;                                 /* Long Term Key is valid */
    uint32 txPktCount;                               /* used for nonce formation during encryption (Note: 39 bits!)?? */
    uint32 rxPktCount;                               /* used for nonce formation during encryption (Note: 39 bits!)?? */
    uint8  encRestart;                               /* flag to indicate if an encryption key change took place */
    uint8  encRejectErrCode;                         /* error code for rejecting encryption request */

      /* ALT: COULD USE ONE VARIABLE AND STATES FOR THESE FLAGS; IF SO, THE
       *      CONTROL PROCEDURE WOULD NEED TO BE REWORKED.
       */

    uint8  startEncRspRcved;                           /* flag to indicate the Start Request has been responded to */
    uint8  pauseEncRspRcved;                           /* flag to indicate the Pause Request has been responded to */
    uint8  encReqRcved;                                /* flag to indicate an Enc Req was received in a Enc Pause procedure */

    uint8  startEncReqRcved;                           /* flag to indicate the Start Request has been responded to */
    uint8  rejectIndRcved;                             /* flag to indicate the Start Encryption needs to be aborted */
} encInfo_t;

/* Feature Set Data */

typedef struct
{
    uint8_t featureRspRcved;                           /* flag to indicate the Feature Request has been responded to */
    uint8_t featureSet[8];
} featureSet_t;

/* Version Information Exchange */

typedef struct
{
    uint8_t peerInfoValid;                            /* flag to indicate the peer's version information is valid */
    uint8_t hostRequest;                              /* flag to indicate the host has requested the peer's version information */
    uint8_t verInfoSent;                              /* flag to indicate this device's version information has been sent */
} verExchange_t;

typedef struct
{
    uint8_t  verNum;                                  /* controller spec version */
    uint16_t comId;                                   /* company identifier */
    uint16_t subverNum;                               /* implementation version */
} verInfo_t;

typedef struct
{
    uint8_t connId;                                   /* connection ID */
    uint8_t termIndRcvd;                              /* indicates a TERMINATE_IND was received */
    uint8_t reason;                                   /* reason code to return to Host when connection finally ends */
} termInfo_t;

/* Packet Error Rate Information - General */

typedef struct
{
    uint16 numPkts;                                    /* total number of packets */
    uint16 numCrcErr;                                  /* total number of packets with CRC error */
    uint16 numEvents;                                  /* total number of connection events */
    uint16 numMissedEvts;                              /* total number of missed connection events */
} perInfo_t;

typedef struct
{
    uint8_t m2sPhy;
    uint8_t s2mPhy;
    uint16_t instant;
} ll_phy_update_ind_t;

typedef struct
{
    uint16_t MaxTxOctets;
    uint16_t MaxTxTime;
    uint16_t MaxRxOctets;
    uint16_t MaxRxTime;
} ll_pdu_length_ctrl_t;

typedef struct
{
    ll_pdu_length_ctrl_t local;
    ll_pdu_length_ctrl_t remote;
    ll_pdu_length_ctrl_t suggested;      /* global setting */
    uint8_t isProcessingReq;
    uint8_t isWatingRsp;
    uint8_t isChanged;
    uint8_t dummy[1];
} llPduLenManagment_t;

typedef struct
{
    uint8_t allPhy;
    uint8_t txPhy;
    uint8_t rxPhy;
    uint8_t dummy[1];
} ll_phy_ctrl_t;

typedef struct
{
    ll_phy_ctrl_t def;
    ll_phy_ctrl_t local;
    ll_phy_ctrl_t req;
    ll_phy_ctrl_t rsp;
    uint16_t phyOptions;
    uint8_t isChanged;
    uint8_t isProcessingReq;
    uint8_t isWatingRsp;
    uint8_t status;
    uint8_t dummy[2];
} llPhyModeManagment_t;

struct ll_pkt_desc
{
    uint32_t    valid;  /* mean a valid data received from ble */
    uint16_t    header;
    uint8_t     data[2];
};

typedef struct
{
    #if 0
    struct buf_tx_desc tx_conn_desc[MAX_LL_BUF_LEN];     /* new Tx data buffer */
    struct buf_rx_desc rx_conn_desc[MAX_LL_BUF_LEN];
    struct buf_tx_desc tx_not_ack_pkt;
    struct buf_tx_desc tx_ntrm_pkts[MAX_LL_BUF_LEN];
    #endif
    struct ll_pkt_desc *tx_conn_desc[MAX_LL_BUF_LEN];     /* new Tx data buffer */
    struct ll_pkt_desc *rx_conn_desc[MAX_LL_BUF_LEN];
    struct ll_pkt_desc *tx_not_ack_pkt;
    struct ll_pkt_desc *tx_ntrm_pkts[MAX_LL_BUF_LEN];
    uint8_t ntrm_cnt;       /* number of packets not transmit */
    uint8_t tx_write;
    uint8_t tx_read;
    uint8_t tx_loop;        /* flag for write ptr & read ptr work in the same virtual buffer bank */
    uint8_t rx_write;
    uint8_t rx_read;
    uint8_t rx_loop;        /* flag for write ptr & read ptr work in the same virtual buffer bank */
} llLinkBuf_t;

typedef struct
{
    uint16_t header;

    /* uint8_t  data[TX_BUF_LEN]; */

    uint8_t  data[TX_CTRL_BUF_LEN];
}
__attribute__((aligned(4)))ctrl_packet_buf;

/* ======= multi-connection */

typedef struct
{
    /* connection packet statistics */

    uint32_t ll_recv_ctrl_pkt_cnt;
    uint32_t ll_recv_data_pkt_cnt;
    uint32_t ll_recv_invalid_pkt_cnt;
    uint32_t ll_recv_abnormal_cnt;
    uint32_t ll_send_data_pkt_cnt;
    uint32_t ll_conn_event_cnt;
    uint32_t ll_recv_crcerr_event_cnt;              /* CRC error detected in the connection event */
    uint32_t ll_conn_event_timeout_cnt;             /* timeout connection event countt */

    /* LL <-> HCI packets statistics */

    uint32_t ll_to_hci_pkt_cnt;
    uint32_t ll_hci_to_ll_pkt_cnt;
    uint32_t ll_hci_buffer_alloc_err_cnt;
    uint32_t ll_miss_master_evt_cnt;
    uint32_t ll_miss_slave_evt_cnt;

    /* reserve counter */

    uint32_t ll_tbd_cnt1;
    uint32_t ll_tbd_cnt2;
    uint32_t ll_tbd_cnt3;
    uint32_t ll_tbd_cnt4;
} llLinkStatistics_t;

/* 2020-02-21 add for CTE req & rsp logic */

typedef struct
{
    uint8_t isChanged;
    uint8_t isProcessingReq;
    uint8_t isWatingRsp;            /* wait other Ctrl command procedure */
    uint8_t errorCode;
} llCTEModeManagement_t;

/* 2020-01-15 add for connection & connectionless parameter */

typedef struct
{
    /* common */

    /* uint16  handle;  */             /* syncConnHandle for connectionless , connHandle for connection */

    uint8   enable;
    uint8   CTE_Length;             /* connectionless transmit CTE length or connection request and response CTE Length */
    uint8   CTE_Type;               /* AOA, ADO 1us , AOD 2us */
    uint8   CTE_Count;              /* number of CTE to transmit in each PA interval */

    /* IQ Sample:max number of CTE to sample and report in each PA interval */

    uint8   CTE_Count_Idx;          /* record the number of times that the CTE send , max equal to CTE_Count */
    uint8   pattern_LEN;
    uint8   AntID[LL_CTE_MAX_PATTERN_LEN];
    uint8   slot_Duration;          /* switching and sampling slot 1us or 2us */

    /* connectionless transmit param */

    /* uint8   advSet;  */               /* identify connectionless advertising set */

    /* CTEInfo_t merge to periodicAdvInfo_t , advSet not used */

    /* connection CTE request & response enable command */

    uint16  CTE_Request_Intv;
} CTEInfo_t;

/* Connection Data */

typedef struct
{
    uint8_t          rx_timeout;                     /* ----- */
    uint8_t          rx_crcok;                       /* ----- */
    uint8_t          allocConn;                      /* flag to indicate if this connection is allocated */
    uint8_t          active;                         /* flag to indicate if this connection is active */
    uint8_t          connId;                         /* connection ID */
    uint8_t          firstPacket;                    /* flag to indicate when the first packet has been received. 0 means TURE, 1 means FALSE */

    uint16_t         currentEvent;                   /* current event number */
    uint16_t         nextEvent;                      /* next active event number */
    uint16_t         lastCurrentEvent;
    uint16_t         expirationEvent;                /* event at which the LSTO has expired */
    uint16_t         expirationValue;                /* number of events to a LSTO expiration */

    uint16_t         scaFactor;                      /* SCA factor for timer drift calculation */
    uint32_t         timerDrift;                     /* saved timer drift adjustment to avoid recalc */
    uint32_t         accuTimerDrift;                 /* accumulate timer drift */

    /* Connection Parameters */

    uint32_t         lastTimeToNextEvt;              /* the time to next event from the previous connection event */
    uint8_t          slaveLatencyAllowed;            /* flag to indicate slave latency is permitted */
    uint16_t         slaveLatency;                   /* current slave latency; 0 means inactive */
    uint8_t          lastSlaveLatency;               /* last slave latency value used */
    uint16_t         slaveLatencyValue;              /* current slave latency value (when enabled) */

    uint32_t accessAddr;                             /* saved synchronization word to be used by Slave */
    uint32_t initCRC;                                /* connection CRC initialization value (24 bits) */

    uint8_t          sleepClkAccuracy;               /* peer's sleep clock accuracy; used by own device to determine timer drift */
    connParam_t     curParam;

    /* current connection parameters */

    /* Channel Map */

    uint8_t          nextChan;                       /* the channel for the next active connection event */
    uint8_t          currentChan;                    /* the channel for the currently completed connection event */
    uint8_t          lastCurrentChan;                /* the channel for the last currentChan for disable slavelatency usage */

    uint8_t          numUsedChans;                   /* count of the number of usable data channels */

    /* uint8_t          hopLength; */                /* used for finding next data channel at next connection event */

    uint8_t          chanMapTable[LL_MAX_NUM_DATA_CHAN]; /* current chan map table that is in use for this connection */

    uint8_t          chanMap[5];

    chanMap_t           chanMapUpdate;              /* slave chanMapUpdate for different connId */
    preChanMapUpdate_t preChanMapUpdate;            /* used for disable latency */
    uint8_t          hop;

    /* TX Related */

    uint8_t          txDataEnabled;                  /* flag that indicates whether data output is allowed */
    llDataQ_t        txDataQ;                        /* queue of Tx Data packets */

    /* RX Related */

    uint8_t          rxDataEnabled;                  /* flag that indicates whether data input is allowed */
    uint8_t          lastRssi;                       /* last data packet RSSI received on this connection */
    uint16_t         foff;                           /* A2 add, sync qualitiy indicator, estimated by rx BB */
    uint8_t          carrSens;                       /* A2 add, estimated freq offset by rx BB ,foff-512-->[-512 511]KHz */

    /* Control Packet Information */

    ctrlPktInfo_t   ctrlPktInfo;                     /* information for control procedure processing */

    /* Parameter Update Control Procedure */

    uint8_t          pendingParamUpdate;             /* flag to indicate connection parameter update is pending */
    uint16_t         paramUpdateEvent;               /* event count to indicate when to apply pending param update */
    connParam_t    paramUpdate;                      /* update parameters */

    /* Channel Map Update Control Procedure */

    uint8_t          pendingChanUpdate;              /* flag to indicate connection channel map update is pending */
    uint16         chanMapUpdateEvent;               /* event count to indicate when to apply pending chan map update */

    /* Encryption Data Control Procedure */

    uint8          encEnabled;                       /* flag to indicate that encryption is enabled for this connection */
    encInfo_t      encInfo;                          /* structure that holds encryption related data */

    /* Feature Set */

    featureSet_t   featureSetInfo;                   /* feature set for this connection */

    /* Version Information */

    verExchange_t  verExchange;                      /* version information exchange */
    verInfo_t      verInfo;                          /* peer version information */

    /* Termination Control Procedure */

    termInfo_t     termInfo;                         /* structure that holds connection termination data */

    /* Unknnown Control Packet */

    uint8          unknownCtrlType;                  /* value of unknown control type */

    /* Packet Error Rate */

    perInfo_t      perInfo;                          /* PER */

    uint8_t        isCollision;
    uint8_t        rejectOpCode;

    ll_phy_update_ind_t phyUpdateInfo;               /* ll_phy update */

    /* Parameter Update Control Procedure */

    uint8_t          pendingPhyModeUpdate;           /* flag to indicate connection ll phy update is pending */
    uint16_t         phyModeUpdateEvent;

    uint8_t  sn_nesn;                                /* use to save last sn/nesn in new IC */

    /* for new IC */

    uint8_t llMode;                                  /* for RTLP & TRLP loop, may need change the HW engine mode. */

    /* ===============  A2 multi connection */

    uint8_t ctrlDataIsProcess ;     /* seding a control packet or not */

    uint8_t ctrlDataIsPending ;   /* control packet is pending to be sent */

    /* uint8_t dummy[2]; */          /* for 4-bytes align */

    int  anchor_point_base_time;    /* do we need it? */
    int  anchor_point_fine_time;    /* do we need it? */
    int  next_event_base_time;      /* do we need it? */
    int  next_event_fine_time;      /* do we need it? */
    ctrl_packet_buf   ctrlData;
    llLinkBuf_t       ll_buf;

    /* DLE */

    llPduLenManagment_t  llPduLen;
    llPhyModeManagment_t llPhyModeCtrl;

    /* add after BBB ROM release, PHY format */

    uint8_t              llRfPhyPktFmt;

    /* add after BBB ROM release, channel selection algorithm */

    uint8_t              channel_selection;

    llLinkStatistics_t pmCounter;

    /* 2020-01-19 add for CTE */

      /* llCTE_ReqFlag,llCTE_RspFlag only indicate CTE Request and Response
       * enable or disable status
       */

    uint8 llCTE_ReqFlag;
    uint8 llCTE_RspFlag;

    /* CTE REQ & RSP Control */

    llCTEModeManagement_t llCTEModeCtrl;
    CTEInfo_t   llConnCTE;

    /* reserved variables */

    uint32      llTbd1;
    uint32      llTbd2;
    uint32      llTbd3;
    uint32      llTbd4;
} llConnState_t;

typedef uint8 llStatus_t;

typedef struct
{
    osal_event_hdr_t  hdr;
    uint8  BLEEventCode;
    uint8  status;
    uint16 connectionHandle;
    uint8  role;
    uint8  peerAddrType;
    uint8  peerAddr[B_ADDR_LEN];
    uint16 connInterval;
    uint16 connLatency;
    uint16 connTimeout;
    uint8  clockAccuracy;
} hciEvt_BLEConnComplete_t;

/* Extended Advertising parameters */

typedef struct
{
    /* uint8_t     advHandle;  */                          /* range: 0x00 - 0xEF */

    uint8_t     advertisingSID;                            /* range: 0x00 - 0x0F */
    uint16_t    advEventProperties;                        /* adv event type */
    uint32_t    priAdvIntMin;                              /* 3 octets, minimum primary adv interval */
    uint32_t    priAdvgIntMax;                             /* 3 octets, maximum primary adv interval */
    uint8_t     priAdvChnMap;
    uint8_t     ownAddrType;                               /* own device address type of public or random */
    uint8_t     isOwnRandomAddressSet;                     /* own random address type set flag. The address is set by HCI_LE_SET_ADVERTISING_SET_RANDOM_ADDRESS */
    uint8_t     ownRandomAddress[LL_DEVICE_ADDR_LEN];
    uint8_t     peerAddrType;
    uint8_t     peerAddress[LL_DEVICE_ADDR_LEN];
    uint8_t     wlPolicy;                                  /* white list policy for Adv */
    int8        advTxPower;
    uint8_t     primaryAdvPHY;
    uint8_t     secondaryAdvPHY;
    uint8_t     secondaryAdvMaxSkip;                       /* the maximum number of advertising events that can be skipped before the AUX_ADV_IND can be sent */
    uint8_t     scanReqNotificationEnable;
} extAdvParameter_t;

/* data of Advertising set or scan response data */

typedef struct
{
    /* uint8_t     advHandle; */

    uint8_t     dataComplete;                        /* all data of advert set received */
    uint8       fragmentPreference;
    uint16      advertisingDataLength;
    uint8       *advertisingData;

    /* LL generated */

    uint16      DIDInfo;                            /* 12bits */
} advSetData_t;

typedef struct
{
    uint8_t     advHandle;
    extAdvParameter_t  parameter;
    advSetData_t       data;                            /* only for extended adv */
    uint16             scanRspMaxLength;                /* length of scan rsp data */
    uint8              *scanRspData;

    /* ===================== advertisement enable info */

    uint32_t    duration;                               /* unit us, note spec parameter is 10ms unit */
    uint8_t     maxExtAdvEvents;

    /* ================= advertisement context parameters */

    uint8_t     isPeriodic;                              /* is the adv parameters for periodic adv */
    uint8_t     active;                                  /* extended adv enable or not */
    uint32_t    primary_advertising_interval;
    uint16_t    adv_event_counter;                       /* counter for extend adv event */
    uint32_t    adv_event_duration;                      /* duration of advertise */
    int8        tx_power;                                /* range -127 ~ 127 dBm, will be filled to field TxPower */
    uint8_t     sendingAuxAdvInd;

      /* below parameters only applicable to extended adv,
       * not for periodic adv
       */

    uint8_t     currentChn;                                /* current adv channel */
    uint8_t     auxChn;                                    /* 1st aux PDU channel No. */
    uint16_t    currentAdvOffset;                          /* current read ptr of adv data set, for fill AUX_XXX_IND PDUs */
} extAdvInfo_t;

/* OSAL HCI_DATA_EVENT message format.
 * This message is used to forward incoming
 */

/* data messages up to an application */

typedef struct
{
    osal_event_hdr_t hdr;                   /* OSAL event header */
    uint16 connHandle;                      /* connection handle */
    uint8  pbFlag;                          /* data packet boundary flag */
    uint16 len;                             /* length of data packet */
    uint8  *pData;                          /* data packet given by len */
} hciDataEvent_t;

/* data of periodic Advertising set */

typedef struct
{
    uint8       dataComplete;                /* all data of advert set received */
    uint16      advertisingDataLength;
    uint8       *advertisingData;
} periodicAdvSetData_t;

/* periodic adv: data + parameters + enable flag */

/* note that periodic adv also need extended adv parameters + enable */

typedef struct
{
    uint8_t     advHandle;
    periodicAdvSetData_t       data;
    uint16    adv_interval_min;
    uint16    adv_interval_max;
    uint16_t  adv_event_properties;                        /* adv event type */

    /* ================= advertisement context parameters */

    uint8_t     active;                                    /* extended adv enable or not */
    uint32_t    adv_interval;
    uint8_t     secondaryAdvPHY;                           /* reserved, should we copy this setting from ext adv info? ext adv may be disabled while keep periodic adv alive */
    uint8       chn_map[5];                                /* 37 bits */
    uint8_t     chanMapTable[LL_MAX_NUM_DATA_CHAN];
    uint8_t     numUsedChans;                              /* count of the number of usable data channels */
    uint8       sca;                                       /* 3 bit */
    uint32      AA;
    uint32      crcInit;
    uint8_t     tx_power;                                  /* not setting now, reserve for TxPwr field */
    uint16_t    periodic_adv_event_counter;                /* counter for periodic adv event */
    uint8       pa_current_chn;                            /* current periodic adv channel */
    uint8_t     currentChn;                                /* current adv channel */
    uint16_t    currentAdvOffset;                          /* current read ptr of adv data set, for fill AUX_XXX_IND PDUs */

    /* 2020-01-15 CTE global variable */

    CTEInfo_t           PrdCTEInfo;
} periodicAdvInfo_t;

typedef struct
{
    osal_event_hdr_t hdr;
    uint8            *pData;
} hciPacket_t;

/****************************************************************************
 * Functions
 ****************************************************************************/

void phy62xx_ble_init(void);

#endif

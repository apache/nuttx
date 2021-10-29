
typedef hciStatus_t (*hciFunc_t)( uint8* pBuf );
typedef struct
{
    uint16    opCode;
    hciFunc_t hciFunc;
} hciCmdFunc_t;
typedef const hciCmdFunc_t cmdPktTable_t;





void HCI_CommandCompleteEvent( uint16 opcode, uint8  numParam, uint8*  param );
void HCI_CommandStatusEvent( hciStatus_t status, uint16 opcode);


hciStatus_t hciDisconnect( uint8* pBuf )
{
  llStatus_t ll_st = LL_Disconnect(BUILD_UINT16(pBuf[0],pBuf[1]), pBuf[2]);
  HCI_CommandStatusEvent( ll_st, HCI_DISCONNECT );
  return( HCI_SUCCESS );

}



hciStatus_t hciReadRemoteVersionInfo( uint8* pBuf )
{
  hciStatus_t status;
  HCI_CommandStatusEvent( HCI_SUCCESS, HCI_READ_REMOTE_VERSION_INFO );
  status = LL_ReadRemoteVersionInfo(BUILD_UINT16(pBuf[0], pBuf[1]));
  
  // check if something went wrong
  // Note: If success is returned, then Command Complete is handled by Callback.
  if ( status != HCI_SUCCESS )
  {
      HCI_CommandCompleteEvent( HCI_READ_REMOTE_VERSION_INFO, sizeof(status), &status );
  }
  
  return( HCI_SUCCESS );
}



hciStatus_t hciSetEventMask( uint8* pBuf )
{
  uint8_t* pMask = pBuf;

  hciStatus_t status;
  
  // check parameters
  if( pMask != NULL )
  {
      (void)osal_memcpy( pHciEvtMask, pMask, B_EVENT_MASK_LEN );
      status = HCI_SUCCESS;
  }
  else // bad parameters
  {
      status = HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS;
  }
  
  HCI_CommandCompleteEvent( HCI_SET_EVENT_MASK, sizeof(status), &status );
  return( HCI_SUCCESS );

}


hciStatus_t hciReset( uint8* pBuf )
{
    hciStatus_t status;
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;

    // reset the Link Layer
    status = LL_Reset();
    // reset the Bluetooth and the BLE event mask bits
    hciInitEventMasks();
    // initialize Controller to Host flow control flag and counter
    ctrlToHostEnable = FALSE;
    numHostBufs      = 0;
    // complete the command
    HCI_CommandCompleteEvent( HCI_RESET, sizeof(status), &status);
    return( HCI_SUCCESS );
}



hciStatus_t hciReadTransmitPowerLevel( uint8* pBuf )
{
      // 0: Status
      // 1: Connection Handle LSB
      // 2: Connection Handle MSB
      // 3: Transmit Power Level
      uint8 rtnParam[4];
      rtnParam[0] = LL_ReadTxPowerLevel( BUILD_UINT16(pBuf[0],pBuf[1]),
                                          pBuf[2] ,
                                         (int8*)&(rtnParam[3]) );
      rtnParam[1] = pBuf[0];
      rtnParam[2] = pBuf[1];
      HCI_CommandCompleteEvent( HCI_READ_TRANSMIT_POWER, sizeof(rtnParam), rtnParam );
      return( HCI_SUCCESS );

}

hciStatus_t hciSetControllerToHostFlowCtrl( uint8* pBuf )
{
    hciStatus_t status = HCI_SUCCESS;
    uint8 flowControlEnable = pBuf[0];

    // check parameters
    if ( (flowControlEnable == HCI_CTRL_TO_HOST_FLOW_CTRL_OFF)              ||
            (flowControlEnable == HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_OFF) ||
            (flowControlEnable == HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_OFF_SYNCH_ON) ||
            (flowControlEnable == HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_ON) )
    {
        // check the parameter
        if( flowControlEnable == HCI_CTRL_TO_HOST_FLOW_CTRL_OFF )
        {
            // disable flow control
            ctrlToHostEnable = FALSE;
        }
        else if ( flowControlEnable == HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_OFF )
        {
            // enable flow control
            ctrlToHostEnable = TRUE;
        }
        else // other two combinations not supported
        {
            // so indidicate
            status = HCI_ERROR_CODE_UNSUPPORTED_FEATURE_PARAM_VALUE;
        }
    }
    else // bad parameters
    {
        status = HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS;
    }

    HCI_CommandCompleteEvent( HCI_SET_CONTROLLER_TO_HOST_FLOW_CONTROL, sizeof(status), &status);
    return( HCI_SUCCESS );}


hciStatus_t hciHostBufferSize( uint8* pBuf )
{
    uint16 hostTotalNumAclPkts = BUILD_UINT16(pBuf[3], pBuf[4]);


    hciStatus_t status;

    // check parameters
    // Note: Only Number of ACL Packets is supported. The rest of the parameters
    //       are ignored for now.
    if ( hostTotalNumAclPkts == 0 )
    {
        status = HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS;
    }
    else // parameter okay
    {
        status = HCI_SUCCESS;
        // so save in a counter
        numHostBufs = hostTotalNumAclPkts;
    }

    HCI_CommandCompleteEvent( HCI_HOST_BUFFER_SIZE, sizeof(status), &status );
    return( HCI_SUCCESS );

}


hciStatus_t hciHostNumCompletedPkt( uint8* pBuf )
{
  uint16 connHandles, numCompletedPkts;
  uint8   numHandles = pBuf[0];
  connHandles = BUILD_UINT16(pBuf[1], pBuf[2]);
  numCompletedPkts = BUILD_UINT16(pBuf[3], pBuf[4]);


  if ( (numHandles != 0) && (connHandles != NULL) &&
          ((numCompletedPkts != NULL) && (*numCompletedPkts != 0)) )
  {
    // check if flow control is enabled
    if ( ctrlToHostEnable == TRUE )
    {
      // check if the number of Host buffers was previously exhausted
      if ( numHostBufs == 0 )
      {
        // yes, so disable LL Rx flow control
        (void)LL_CtrlToHostFlowControl( LL_DISABLE_RX_FLOW_CONTROL );
      }

      for (uint8 i=0; i<numHandles; i++)
      {
        // host is indicating it has freed one or more buffers
        // Note: It is assumed that the Host will only free one buffer at a time,
        //       and in any case, number of Host buffers are not tracked as a
        //       function of connection handles.
        // Note: No checks are made to ensure the specified connection handles
        //       are valid or active.
        numHostBufs += numCompletedPkts[i*2];
      }
    }

    // Note: The specification indicates that no event is normally returned.
  }
  else // bad parameters
  {
    hciStatus_t status = HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS;
    // Note: The specification indicates that no event is normally returned,
    //       except if there are invalid parameters.
    HCI_CommandCompleteEvent( HCI_HOST_NUM_COMPLETED_PACKETS, sizeof(status), &status);
  }
  
  return( HCI_SUCCESS );

                                       
}
hciStatus_t hciSetEventMaskPage2        ( uint8* pBuf )
{
	hciStatus_t status;
	HCI_CommandCompleteEvent( HCI_SET_EVENT_MASK_PAGE2, sizeof(status), &status );
	return TRUE;
}

hciStatus_t hciReadLocalVersionInfo( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    // 0: Status
    // 1: HCI Version Number
    // 2: HCI Revision Number LSB
    // 3: HCI Revision Number MSB
    // 4: Version Number
    // 5: Connection Handle LSB
    // 6: Connection Handle MSB
    // 7: LL Subversion Number LSB
    // 8: LL Subversion Number MSB
    uint8  rtnParam[9];
    uint8  version;
    uint16 comID;
    uint16 subverNum;
    // status
    rtnParam[0] = LL_ReadLocalVersionInfo( &version,
                                           &comID,
                                           &subverNum );
    // HCI version and revision
    rtnParam[1] = HCI_VERSION;
    rtnParam[2] = LO_UINT16( HCI_REVISION );
    rtnParam[3] = HI_UINT16( HCI_REVISION );
    // LL version, manufacturer name, LL subversion
    rtnParam[4] = version;
    rtnParam[5] = LO_UINT16( comID );
    rtnParam[6] = HI_UINT16( comID );
    rtnParam[7] = LO_UINT16( subverNum );
    rtnParam[8] = HI_UINT16( subverNum );
    HCI_CommandCompleteEvent( HCI_READ_LOCAL_VERSION_INFO, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}

hciStatus_t hciReadLocalSupportedCommands( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    // 0:     Status (HCI_SUCCESS)
    // 1..64: Supported Commands
    HCI_CommandCompleteEvent( HCI_READ_LOCAL_SUPPORTED_COMMANDS,
                              SUPPORTED_COMMAND_LEN+1,
                              (uint8*)supportedCmdsTable );
    return( HCI_SUCCESS );
}


hciStatus_t hciReadLocalSupportedFeatures( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    // 0:    Status
    // 1..8: Supported Features
    uint8 rtnParam[9] = {HCI_SUCCESS, 0, 0, 0, 0, 0, 0, 0, 0};
    // set byte 4 of the feature list, which is the only byte that matters
    rtnParam[5] = LOCAL_SUPPORTED_FEATURE_SET_BYTE_4;
    HCI_CommandCompleteEvent( HCI_READ_LOCAL_SUPPORTED_FEATURES, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


hciStatus_t hciReadBDADDR( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    uint8 rtnParam[7];
    // status
    rtnParam[0] = LL_ReadBDADDR( &(rtnParam[1]) );
    HCI_CommandCompleteEvent( HCI_READ_BDADDR, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}



hciStatus_t hciReadRssi( uint8* pBuf )
{
  // 0: Status
  // 1: Connection Handle LSB
  // 2: Connection Handle MSB
  // 3: RSSI
  uint8 rtnParam[4];
  uint16_t connHandle = BUILD_UINT16(pBuf[0], pBuf[1]);
  // status
  rtnParam[0] = LL_ReadRssi( connHandle,
                             (int8*) &(rtnParam[3]) );
  // connection handle
  rtnParam[1] = LO_UINT16( connHandle);
  rtnParam[2] = HI_UINT16( connHandle );
  HCI_CommandCompleteEvent( HCI_READ_RSSI, sizeof(rtnParam), rtnParam );
  return( HCI_SUCCESS );
}

hciStatus_t hciLESetEventMask( uint8* pBuf )
{
  hciStatus_t status;

  // check parameters
  if ( pEventMask != NULL )
  {
      // set the BLE event mask
      // Note: So far, only the first byte is used.
//        bleEvtMask = pEventMask[0];
  bleEvtMask = BUILD_UINT32(pEventMask[0], pEventMask[1], pEventMask[2], pEventMask[3]);
  
      // extend le_meta Event masker to 32bit by ZQ 20181031
      // according to Core 5.0 LE Meta EVENT number is 20
      bleEvtMask32 = BUILD_UINT32(pBuf[0], pBuf[1], pBuf[2], pBuf[3]);
      status = HCI_SUCCESS;
  }
  else // bad parameters
  {
      status = HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS;
  }

  HCI_CommandCompleteEvent( HCI_LE_SET_EVENT_MASK, sizeof(status), &status );
  return( HCI_SUCCESS );

}
hciStatus_t hciLEReadBufSize( uint8* pBuf )
{
//  // unused input parameter; PC-Lint error 715.
//  (void)pBuf;
//
//  return HCI_LE_ReadBufSizeCmd();
    // 0: Status
    // 1: Data Packet Length LSB
    // 2: Data Packet Length MSB
    // 3: Buffer Size
    uint8 rtnParam[4];
    // status
    rtnParam[0] = HCI_SUCCESS;
    // data packet length
    rtnParam[1] = LO_UINT16( 251 );
    rtnParam[2] = HI_UINT16( 251 );
    // number of data packets allowed by Controller
    rtnParam[3] = HCI_MAX_NUM_DATA_BUFFERS;
    HCI_CommandCompleteEvent( HCI_LE_READ_BUFFER_SIZE, sizeof(rtnParam), rtnParam);
    return( HCI_SUCCESS );
}
hciStatus_t hciLEReadLocalSupportedFeatures( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
  uint8 rtnParam[9];
  rtnParam[0] = LL_ReadLocalSupportedFeatures( &(rtnParam[1]) );
  HCI_CommandCompleteEvent( HCI_LE_READ_LOCAL_SUPPORTED_FEATURES, sizeof(rtnParam), rtnParam );
  return( HCI_SUCCESS );
}

hciStatus_t hciLESetRandomAddr( uint8* pBuf )
{
  hciStatus_t status;
  
  // check parameters
  if ( pBuf != NULL )
  {
      status = LL_SetRandomAddress( pBuf );
  }
  else // bad parameters
  {
      status = HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS;
  }
  
  HCI_CommandCompleteEvent( HCI_LE_SET_RANDOM_ADDR, sizeof(status), &status );
  return( HCI_SUCCESS );
}

hciStatus_t hciLESetAdvParam( uint8* pBuf )
{
    //return HCI_LE_SetAdvParamCmd( BUILD_UINT16(pBuf[0], pBuf[1]),
    //                              BUILD_UINT16(pBuf[2], pBuf[3]),
    //                              pBuf[4],
    //                              pBuf[5],
    //                              pBuf[6],
    //                              &pBuf[7],
    //                              pBuf[13],
    //                              pBuf[14] );


    hciStatus_t status;
    status = LL_SetAdvParam( BUILD_UINT16(pBuf[0], pBuf[1]),//advIntervalMin,
                             BUILD_UINT16(pBuf[2], pBuf[3]),//advIntervalMax,
                             pBuf[4],//advType,
                             pBuf[5],//ownAddrType,
                             pBuf[6],//directAddrType,
                             &pBuf[7],//directAddr,
                             pBuf[13],//advChannelMap,
                             pBuf[14]);//advFilterPolicy );
    HCI_CommandCompleteEvent( HCI_LE_SET_ADV_PARAM, sizeof(status), &status );
    return( HCI_SUCCESS );                                  
}

hciStatus_t hciLESetAdvData( uint8* pBuf )
{
  hciStatus_t status;
  status = LL_SetAdvData( pBuf[0],&pBuf[1] );
  HCI_CommandCompleteEvent( HCI_LE_SET_ADV_DATA, sizeof(status), &status );
  return( HCI_SUCCESS );
}


hciStatus_t hciLESetScanRspData( uint8* pBuf )
{
  hciStatus_t status;
  status = LL_SetScanRspData(  pBuf[0], &pBuf[1] );
  HCI_CommandCompleteEvent( HCI_LE_SET_SCAN_RSP_DATA, sizeof(status), &status );
  return( HCI_SUCCESS );
}


hciStatus_t hciLESetAdvEnab( uint8* pBuf )
{
  hciStatus_t status;
  status = LL_SetAdvControl( pBuf[0] );
  HCI_CommandCompleteEvent( HCI_LE_SET_ADV_ENABLE, sizeof(status), &status );
  return( HCI_SUCCESS );
}

hciStatus_t hciLEReadAdvChanTxPower( uint8* pBuf )
{
  (void)pBuf;
  // 0: Status
  // 1: Advertising Transmit Power
  uint8 rtnParam[2];
  // status
  rtnParam[0] = LL_ReadAdvChanTxPower( (int8*)&(rtnParam[1]) );
  HCI_CommandCompleteEvent( HCI_LE_READ_ADV_CHANNEL_TX_POWER, sizeof(rtnParam), rtnParam );
  return( HCI_SUCCESS );
}

/*******************************************************************************
    @fn          hciLESetScanParam

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLESetScanParam( uint8* pBuf )
{
  hciStatus_t status;
  status = LL_SetScanParam( pBuf[0],                        //scanType,
                            BUILD_UINT16(pBuf[1], pBuf[2]), //scanInterval,
                            BUILD_UINT16(pBuf[3], pBuf[4]), //scanWindow,
                            pBuf[5],                        //ownAddrType,
                            pBuf[6]);                       //filterPolicy );
  HCI_CommandCompleteEvent( HCI_LE_SET_SCAN_PARAM, sizeof(status), &status );
  return( HCI_SUCCESS );
}

hciStatus_t hciLESetScanEnable( uint8* pBuf )
{
  hciStatus_t status;
  status = LL_SetScanControl( pBuf[0], pBuf[1]);
  HCI_CommandCompleteEvent( HCI_LE_SET_SCAN_ENABLE, sizeof(status), &status );
  return( HCI_SUCCESS );
}
hciStatus_t hciLECreateConn( uint8* pBuf )
{
    /*return HCI_LE_CreateConnCmd( BUILD_UINT16(pBuf[0], pBuf[1]),
                                 BUILD_UINT16(pBuf[2], pBuf[3]),
                                 pBuf[4],
                                 pBuf[5],
                                 &pBuf[6],
                                 pBuf[12],
                                 BUILD_UINT16(pBuf[13], pBuf[14]),
                                 BUILD_UINT16(pBuf[15], pBuf[16]),
                                 BUILD_UINT16(pBuf[17], pBuf[18]),
                                 BUILD_UINT16(pBuf[19], pBuf[20]),
                                 BUILD_UINT16(pBuf[21], pBuf[22]),
                                 BUILD_UINT16(pBuf[23], pBuf[24]) );*/
  hciStatus_t status;
  status = LL_CreateConn(  BUILD_UINT16(pBuf[0], pBuf[1]),
                           BUILD_UINT16(pBuf[2], pBuf[3]),
                           pBuf[4],
                           pBuf[5],
                           &pBuf[6],
                           pBuf[12],
                           BUILD_UINT16(pBuf[13], pBuf[14]),
                           BUILD_UINT16(pBuf[15], pBuf[16]),
                           BUILD_UINT16(pBuf[17], pBuf[18]),
                           BUILD_UINT16(pBuf[19], pBuf[20]),
                           BUILD_UINT16(pBuf[21], pBuf[22]),
                           BUILD_UINT16(pBuf[23], pBuf[24]));
  HCI_CommandStatusEvent( status, HCI_LE_CREATE_CONNECTION );
  return( HCI_SUCCESS );
}


hciStatus_t hciLECreateConnCancel( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    hciStatus_t status;
    status = LL_CreateConnCancel();
    HCI_CommandCompleteEvent( HCI_LE_CREATE_CONNECTION_CANCEL, sizeof(status), &status );
    return( HCI_SUCCESS );
}


hciStatus_t hciLEReadWhiteListSize( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    // 0: Status
    // 1: White List Size
    uint8 rtnParam[2];
    rtnParam[0] = LL_ReadWlSize( &(rtnParam[1]) );
    HCI_CommandCompleteEvent( HCI_LE_READ_WHITE_LIST_SIZE, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


hciStatus_t hciLEClearWhiteList( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    hciStatus_t status;
    status = LL_ClearWhiteList();
    HCI_CommandCompleteEvent( HCI_LE_CLEAR_WHITE_LIST, sizeof(status), &status );
    return( HCI_SUCCESS );
}


hciStatus_t hciLEAddWhiteList( uint8* pBuf )
{
  //return HCI_LE_AddWhiteListCmd( pBuf[0],
  //                               &pBuf[1] );
  hciStatus_t status;
  status = LL_AddWhiteListDevice( &pBuf[1], pBuf[0]);
  HCI_CommandCompleteEvent( HCI_LE_ADD_WHITE_LIST, sizeof(status), &status );
  return( HCI_SUCCESS );
}


hciStatus_t hciLERemoveWhiteList( uint8* pBuf )
{
  //return HCI_LE_RemoveWhiteListCmd( pBuf[0],
  //                                  &pBuf[1] );
  hciStatus_t status;
  status = LL_RemoveWhiteListDevice( &pBuf[1], pBuf[0] );
  HCI_CommandCompleteEvent( HCI_LE_REMOVE_WHITE_LIST, sizeof(status), &status );
  return( HCI_SUCCESS );
}

hciStatus_t hciLEConnUpdate( uint8* pBuf )
{
  /*return HCI_LE_ConnUpdateCmd( BUILD_UINT16(pBuf[0], pBuf[1]),
                                 BUILD_UINT16(pBuf[2], pBuf[3]),
                                 BUILD_UINT16(pBuf[4], pBuf[5]),
                                 BUILD_UINT16(pBuf[6], pBuf[7]),
                                 BUILD_UINT16(pBuf[8], pBuf[9]),
                                 BUILD_UINT16(pBuf[10], pBuf[11]),
                                 BUILD_UINT16(pBuf[12], pBuf[13]) );*/
  hciStatus_t status;
  status = LL_ConnUpdate(  BUILD_UINT16(pBuf[0], pBuf[1]),
                           BUILD_UINT16(pBuf[2], pBuf[3]),
                           BUILD_UINT16(pBuf[4], pBuf[5]),
                           BUILD_UINT16(pBuf[6], pBuf[7]),
                           BUILD_UINT16(pBuf[8], pBuf[9]),
                           BUILD_UINT16(pBuf[10], pBuf[11]),
                           BUILD_UINT16(pBuf[12], pBuf[13]));
  HCI_CommandStatusEvent( status, HCI_LE_CONNECTION_UPDATE );
  return( HCI_SUCCESS );
}


hciStatus_t hciLESetHostChanClass( uint8* pBuf )
{
  hciStatus_t status;
  status = LL_ChanMapUpdate( pBuf );
  HCI_CommandCompleteEvent( HCI_LE_SET_HOST_CHANNEL_CLASSIFICATION, sizeof(status), &status );
  return( HCI_SUCCESS );
}

hciStatus_t hciLEReadChanMap( uint8* pBuf )
{
  // return HCI_LE_ReadChannelMapCmd( BUILD_UINT16(pBuf[0], pBuf[1]) );
  // 0:    Status
  // 1:    Connection Handle LSB
  // 2:    Connection Handle MSB
  // 3..7: Channel Map (LSB to MSB)
  uint8 rtnParam[8];
  uint16_t connHandle = BUILD_UINT16(pBuf[0], pBuf[1]);
  rtnParam[0] = LL_ReadChanMap( connHandle,
                                &(rtnParam[3]) );
  // connection handle
  rtnParam[1] = LO_UINT16( connHandle );
  rtnParam[2] = HI_UINT16( connHandle );
  HCI_CommandCompleteEvent( HCI_LE_READ_CHANNEL_MAP, sizeof(rtnParam), rtnParam );
  return( HCI_SUCCESS );
}

hciStatus_t hciLEReadRemoteUsedFeatures( uint8* pBuf )
{
  //return HCI_LE_ReadRemoteUsedFeaturesCmd( BUILD_UINT16(pBuf[0], pBuf[1]) );
  hciStatus_t status;
  status = LL_ReadRemoteUsedFeatures( BUILD_UINT16(pBuf[0], pBuf[1]) );
  HCI_CommandStatusEvent( status, HCI_LE_READ_REMOTE_USED_FEATURES );
  return( HCI_SUCCESS );
}

hciStatus_t hciLEEncrypt( uint8* pBuf )
{
    // reverse byte order of key (MSB..LSB required)
    HCI_ReverseBytes( &pBuf[0], KEYLEN );
    // reverse byte order of plaintext (MSB..LSB required)
    HCI_ReverseBytes( &pBuf[KEYLEN], KEYLEN );
    //return HCI_LE_EncryptCmd( &pBuf[0], &pBuf[KEYLEN] );
    // 0:     Status
    // 1..16: Encrypted Data
    uint8 rtnParam[17];
    rtnParam[0] = LL_Encrypt( &pBuf[0],
                              &pBuf[KEYLEN],
                              &rtnParam[1] );

    // check for success
    if ( rtnParam[0] == LL_STATUS_SUCCESS )
    {
        // encryption process reverses the bytes, so restore to MSB..LSB
        HCI_ReverseBytes( &rtnParam[1], KEYLEN );
        HCI_CommandCompleteEvent( HCI_LE_ENCRYPT, sizeof(rtnParam), rtnParam );
    }
    else // error
    {
        HCI_CommandCompleteEvent( HCI_LE_ENCRYPT, sizeof(uint8), rtnParam );
    }

    return( HCI_SUCCESS );}


hciStatus_t hciLERand( uint8* pBuf )
{
    (void)pBuf;
    // 0:    Status
    // 1..8: Random Bytes
    uint8 rtnParam[B_RANDOM_NUM_SIZE+1];
    rtnParam[0] = LL_Rand( &rtnParam[1], B_RANDOM_NUM_SIZE );

    // check if the operation has been completed; if not, then it has been delayed
    // until a current radio operation completes as the radio is needed to
    // generate a true random number, or there was some kind of error
    if ( rtnParam[0] != LL_STATUS_ERROR_DUE_TO_DELAYED_RESOURCES )
    {
        // check if the operation was okay
        if ( rtnParam[0] == LL_STATUS_SUCCESS )
        {
            HCI_CommandCompleteEvent( HCI_LE_RAND, B_RANDOM_NUM_SIZE+1, rtnParam );
        }
        else // an error occurred
        {
            HCI_CommandCompleteEvent( HCI_LE_RAND, sizeof(uint8), rtnParam );
        }
    }

    return( HCI_SUCCESS );
}


hciStatus_t hciLEStartEncrypt( uint8* pBuf )
{
    //return HCI_LE_StartEncyptCmd( BUILD_UINT16(pBuf[0], pBuf[1]),
    //                              &pBuf[2],
    //                              &pBuf[10],
    //                              &pBuf[12] );
  hciStatus_t status;
  status = LL_StartEncrypt( BUILD_UINT16(pBuf[0], pBuf[1]),//connHandle,
                            &pBuf[2], //random,
                            &pBuf[10], //encDiv,
                            &pBuf[12]);//ltk );
  HCI_CommandStatusEvent( status, HCI_LE_START_ENCRYPTION );
  return( HCI_SUCCESS );
}

hciStatus_t hciLELtkReqReply( uint8* pBuf )
{
    return HCI_LE_LtkReqReplyCmd( BUILD_UINT16(pBuf[0], pBuf[1]),
                                  &pBuf[2] );
}


hciStatus_t hciLELtkReqNegReply( uint8* pBuf )
{
    return HCI_LE_LtkReqNegReplyCmd( BUILD_UINT16(pBuf[0], pBuf[1]) );
}

hciStatus_t hciLEReadSupportedStates( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_LE_ReadSupportedStatesCmd();
}


hciStatus_t hciLEReceiverTest( uint8* pBuf )
{
    return HCI_LE_ReceiverTestCmd( pBuf[0] );
}



hciStatus_t hciLETransmitterTest( uint8* pBuf )
{
    return HCI_LE_TransmitterTestCmd( pBuf[0],
                                      pBuf[1],
                                      pBuf[2] );
}


hciStatus_t hciLETestEnd( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_LE_TestEndCmd();
}


hciStatus_t hciLESetDataLength                     ( uint8* pBuf )
{
    return HCI_LE_SetDataLengthCmd( BUILD_UINT16(pBuf[0], pBuf[1]),
                                    BUILD_UINT16(pBuf[2], pBuf[3]),
                                    BUILD_UINT16(pBuf[4], pBuf[5]));
}

hciStatus_t hciLEReadSuggestedDefaultDataLength( uint8* pBuf )
{
    (void)pBuf;
    return HCI_LE_ReadSuggestedDefaultDataLengthCmd();
}

hciStatus_t hciLEWriteSuggestedDefaultDataLength( uint8* pBuf )
{
    return HCI_LE_WriteSuggestedDefaultDataLengthCmd( BUILD_UINT16(pBuf[0], pBuf[1]),
                                                      BUILD_UINT16(pBuf[2], pBuf[3]));
}

hciStatus_t hciLEReadPhyMode( uint8* pBuf )
{
    return HCI_LE_ReadPhyMode( BUILD_UINT16(pBuf[0], pBuf[1]));
}

hciStatus_t hciLESetDefaultPhyMode( uint8* pBuf )
{
    return HCI_LE_SetDefaultPhyMode( 0, pBuf[0], pBuf[1], pBuf[2]);
}

hciStatus_t hciLESetPhyMode( uint8* pBuf )
{
    return HCI_LE_SetPhyMode( BUILD_UINT16(pBuf[0], pBuf[1]),
                              pBuf[2], pBuf[3], pBuf[4],
                              BUILD_UINT16(pBuf[5], pBuf[6]));
}

// 2020-07  add
hciStatus_t hciLEAddDeviceToRL                     ( uint8* pBuf )
{
    return HCI_LE_AddDevToResolvingListCmd( pBuf[0],
                                            &pBuf[1],
                                            &pBuf[7],
                                            &pBuf[23]);
}

hciStatus_t hciLERemoveDeviceFromRL                ( uint8* pBuf )
{
    return HCI_LE_RemoveResolvingListCmd( pBuf[0],
                                          &pBuf[1]);
}

hciStatus_t hciLEClearRL                           ( uint8* pBuf )
{
    (void)pBuf;
    return HCI_LE_ClearResolvingListCmd();
}

hciStatus_t hciLEReadRLSize                        ( uint8* pBuf )
{
    (void)pBuf;
    return HCI_LE_ReadResolvingListSizeCmd();
}

hciStatus_t hciLEReadPeerRA                        ( uint8* pBuf )
{
    return HCI_LE_ReadPeerResolvableAddressCmd(pBuf[0],
                                               &pBuf[1]);
}

hciStatus_t hciLEReadLocalRA                       ( uint8* pBuf )
{
    return HCI_LE_ReadLocalResolvableAddressCmd(pBuf[0],
                                                &pBuf[1]);
}

hciStatus_t hciLESetAddrResolutionEnable           ( uint8* pBuf )
{
    return HCI_LE_SetAddressResolutionEnableCmd(pBuf[0]);
}

hciStatus_t hciLESetRpaTo                          ( uint8* pBuf )
{
    return HCI_LE_SetResolvablePrivateAddressTimeoutCmd( BUILD_UINT16(pBuf[0], pBuf[1]));
}


hciStatus_t hciLESetAdvSetRandomAddress            ( uint8* pBuf )
{
    return HCI_LE_SetExtAdvSetRandomAddressCmd(pBuf[0],
                                               &pBuf[1]);
}

hciStatus_t hciLESetExtAdvParam                    ( uint8* pBuf )
{
    return HCI_LE_SetExtAdvParamCmd(pBuf[0],
                                    BUILD_UINT16(pBuf[1], pBuf[2]),
                                    BUILD_UINT32(pBuf[3], pBuf[4], pBuf[5], 0),
                                    BUILD_UINT32(pBuf[6], pBuf[7], pBuf[8], 0),
                                    pBuf[9],
                                    pBuf[10],
                                    pBuf[11],
                                    &pBuf[12],
                                    pBuf[18],
                                    pBuf[19],
                                    pBuf[20],
                                    pBuf[21],
                                    pBuf[22],
                                    pBuf[23],
                                    pBuf[24]
                                   );
}

hciStatus_t hciLESetExtAdvData            ( uint8* pBuf )
{
    return HCI_LE_SetExtAdvDataCmd            (pBuf[0],
                                               pBuf[1],
                                               pBuf[2],
                                               pBuf[3],
                                               &pBuf[4]);
}

hciStatus_t hciLESetExtScanRspData        ( uint8* pBuf )
{
    return HCI_LE_SetExtScanRspDataCmd        (pBuf[0],
                                               pBuf[1],
                                               pBuf[2],
                                               pBuf[3],
                                               &pBuf[4]
                                              );
}

// note: number_of_sets should be 1
#define   LL_MAX_ADV_SET                     6
hciStatus_t hciLESetExtAdvEnable          ( uint8* pBuf )
{
    uint8 number_of_sets = pBuf[1];
    uint8  adv_handler[LL_MAX_ADV_SET];
    uint16 duration[LL_MAX_ADV_SET];
    uint8  max_ext_adv_evt[LL_MAX_ADV_SET];

    if (number_of_sets > LL_MAX_ADV_SET || number_of_sets == 0)
        return 0x12;

    for (int i = 0; i < number_of_sets; i++)
    {
        adv_handler[i] = pBuf[2 + i * 4];
        duration[i]    = BUILD_UINT16(pBuf[3 + i * 4], pBuf[4 + i * 4]);
        max_ext_adv_evt[i] = pBuf[5 + i * 4];
    }

    return HCI_LE_SetExtAdvEnableCmd          (pBuf[0],
                                               number_of_sets,                     // uint8  number_of_sets,
                                               adv_handler,                     // uint8  *advertising_handle,
                                               duration,                      // uint16 *duration,
                                               max_ext_adv_evt                // uint8  *max_extended_advertising_events
                                              );
}

hciStatus_t hciLEReadMaximumAdvDataLength            ( uint8* pBuf )
{
    return HCI_LE_ReadMaximumAdvDataLengthCmd();
}

hciStatus_t hciLEReadNumberOfSupportAdvSet            ( uint8* pBuf )
{
    return HCI_LE_ReadNumberOfSupportAdvSetCmd();
}

hciStatus_t hciLERemoveAdvSet            ( uint8* pBuf )
{
    return HCI_LE_RemoveAdvSetCmd(pBuf[0]);
}

hciStatus_t hciLEClearAdvSets            ( uint8* pBuf )
{
    return HCI_LE_ClearAdvSetsCmd();
}


hciStatus_t hciLESetPeriodicAdvParameter              ( uint8* pBuf )
{
    return HCI_LE_SetPeriodicAdvParameterCmd(pBuf[0],
                                             BUILD_UINT16(pBuf[1], pBuf[2]),
                                             BUILD_UINT16(pBuf[3], pBuf[4]),
                                             BUILD_UINT16(pBuf[5], pBuf[6])
                                            );
}

hciStatus_t hciLESetPeriodicAdvData            ( uint8* pBuf )
{
    return HCI_LE_SetPeriodicAdvDataCmd       (pBuf[0],
                                               pBuf[1],
                                               pBuf[2],
                                               &pBuf[3]);
}

hciStatus_t hciLESetPeriodicAdvEnable         ( uint8* pBuf )
{
    return HCI_LE_SetPeriodicAdvEnableCmd     (pBuf[0],
                                               pBuf[1]
                                              );
}

hciStatus_t hciLESetExtendedScanParameters             ( uint8* pBuf )
{
    uint8 scanPhy, numPhy, offset = 3;
    uint8 scan_types[2];
    uint16  scan_interval[2], scan_window[2];
    scanPhy = pBuf[2];
    numPhy = (scanPhy & LL_SCAN_PHY_1M_BITMASK) + ((scanPhy & LL_SCAN_PHY_CODED_BITMASK) >> 2);

    for (int i = 0; i < numPhy; i++)
    {
        scan_types[i] = pBuf[offset];
        offset ++;
        scan_interval[i] = BUILD_UINT16(pBuf[offset], pBuf[offset + 1]);
        offset += 2;
        scan_window[i] = BUILD_UINT16(pBuf[offset], pBuf[offset + 1]);
        offset += 2;
    }

    return HCI_LE_SetExtendedScanParametersCmd(pBuf[0],
                                               pBuf[1],
                                               pBuf[2],
                                               scan_types,                 // uint8 *scan_sype,
                                               scan_interval,             // uint16 *scan_interval
                                               scan_window);             // uint16 *scan_window
}

hciStatus_t hciLESetExtendedScanEnableCmd           ( uint8* pBuf )
{
    return HCI_LE_SetExtendedScanEnableCmd    (pBuf[0],
                                               pBuf[1],
                                               BUILD_UINT16(pBuf[2], pBuf[3]),
                                               BUILD_UINT16(pBuf[4], pBuf[5]));
}

hciStatus_t hciLEExtendedCreateConnection         ( uint8* pBuf )
{
    uint8 initPhy = pBuf[9], numPhy;
    uint8 offset;
    uint16 scan_interval[3];
    uint16 scan_window[3];
    uint16 conn_interval_min[3];
    uint16 conn_interval_max[3];
    uint16 conn_latency[3];
    uint16 supervision_timeout[3];
    uint16 minimum_CE_length[3];
    uint16 maximum_CE_length[3];
    numPhy = (initPhy & 0x01)
             + ((initPhy & 0x02) >> 1)
             + ((initPhy & 0x04) >> 2);
    offset = 10;

    for (int i = 0; i < numPhy; i++)
    {
        scan_interval[i] = BUILD_UINT16(pBuf[offset], pBuf[offset + 1]);
        offset += 2;
        scan_window[i]   = BUILD_UINT16(pBuf[offset], pBuf[offset + 1]);
        offset += 2;
        conn_interval_min[i] = BUILD_UINT16(pBuf[offset], pBuf[offset + 1]);
        offset += 2;
        conn_interval_max[i] = BUILD_UINT16(pBuf[offset], pBuf[offset + 1]);
        offset += 2;
        conn_latency[i] = BUILD_UINT16(pBuf[offset], pBuf[offset + 1]);
        offset += 2;
        supervision_timeout[i] = BUILD_UINT16(pBuf[offset], pBuf[offset + 1]);
        offset += 2;
        minimum_CE_length[i] = BUILD_UINT16(pBuf[offset], pBuf[offset + 1]);
        offset += 2;
        maximum_CE_length[i] = BUILD_UINT16(pBuf[offset], pBuf[offset + 1]);
        offset += 2;
    }

    return HCI_LE_ExtendedCreateConnectionCmd(pBuf[0],     // uint8 initiator_filter_policy,
                                              pBuf[1],     // uint8 own_address_type,
                                              pBuf[2],     // uint8 peer_address_type,
                                              &pBuf[3],     // uint8 *peer_address,
                                              pBuf[9],     // uint8  initiating_PHYs,
                                              scan_interval,        // uint16 *scan_interval,
                                              scan_window,          // uint16 *scan_window,
                                              conn_interval_min,    // uint16 *conn_interval_min,
                                              conn_interval_max,    // uint16 *conn_interval_max,
                                              conn_latency,         // uint16 *conn_latency,
                                              supervision_timeout,  // uint16 *supervision_timeout,
                                              minimum_CE_length,    // uint16 *minimum_CE_length,
                                              maximum_CE_length);   // uint16 *maximum_CE_length)
}


hciStatus_t hciLEPeriodicAdvertisingCreateSync           ( uint8* pBuf )
{
    return HCI_LE_PeriodicAdvertisingCreateSyncCmd    (pBuf[0],
                                                       pBuf[1],
                                                       pBuf[2],
                                                       &pBuf[3],
                                                       BUILD_UINT16(pBuf[9], pBuf[10]),
                                                       BUILD_UINT16(pBuf[11], pBuf[12]),
                                                       pBuf[13]);
}

hciStatus_t hciLEPeriodicAdvertisingCreateSyncCancel           ( uint8* pBuf )
{
    return HCI_LE_PeriodicAdvertisingCreateSyncCancelCmd    ();
}

hciStatus_t hciLEPeriodicAdvertisingTerminateSync           ( uint8* pBuf )
{
    return HCI_LE_PeriodicAdvertisingTerminateSyncCmd    (BUILD_UINT16(pBuf[0], pBuf[1]));
}


hciStatus_t hciLEAddDevToPeriodicAdvList           ( uint8* pBuf )
{
    return HCI_LE_AddDevToPeriodicAdvListCmd    (pBuf[0],
                                                 &pBuf[1],
                                                 pBuf[7]);
}

hciStatus_t hciLERemovePeriodicAdvList           ( uint8* pBuf )
{
    return HCI_LE_RemovePeriodicAdvListCmd    (pBuf[0],
                                               &pBuf[1],
                                               pBuf[7]);
}

hciStatus_t hciLEClearPeriodicAdvList           ( uint8* pBuf )
{
    return HCI_LE_ClearPeriodicAdvListCmd    ();
}

hciStatus_t hciLEReadPeriodicAdvListSize           ( uint8* pBuf )
{
    return HCI_LE_ReadPeriodicAdvListSizeCmd    ();
}


hciStatus_t hciLEReadTransmitPower           ( uint8* pBuf )
{
    return HCI_LE_Read_Transmit_PowerCmd    ();
}

hciStatus_t hciLEReadRfPathCompensation           ( uint8* pBuf )
{
    return HCI_LE_Read_Rf_Path_CompensationCmd    ();
}

hciStatus_t hciLEWriteRfPathCompensation           ( uint8* pBuf )
{
    return HCI_LE_Write_Rf_Path_CompensationCmd    (BUILD_UINT16(pBuf[0], pBuf[1]),
                                                    BUILD_UINT16(pBuf[2], pBuf[3]));
}

hciStatus_t hciLESetPrivacyMode           ( uint8* pBuf )
{
    return HCI_LE_Set_Privacy_ModeCmd    (pBuf[0],
                                          &pBuf[1],
                                          pBuf[7]);
}





hciStatus_t hciLE_ConnectionlessCTE_TransmitParamCmd(uint8* pBuf)
{
    return (HCI_LE_ConnectionlessCTE_TransmitParamCmd(  pBuf[0],				// uint8 advertising_handle,
													    pBuf[1],				// uint8 len,
                                                        pBuf[2],                // uint8 type,
                                                        pBuf[3],                // uint8 count,            
                                                        pBuf[4],                // uint8 Pattern_LEN,
                                                        &pBuf[5]                // uint8 *AnaIDs)
                                                     ));
}
hciStatus_t hciLE_ConnectionlessCTE_TransmitEnableCmd(uint8* pBuf)
{
    return (HCI_LE_ConnectionlessCTE_TransmitEnableCmd(     pBuf[0],	//         	uint8 advertising_handle,
                                                            pBuf[1]		//        	uint8 enable)
                                                      ));
}
hciStatus_t hciLE_ConnectionlessIQ_SampleEnableCmd(uint8* pBuf)
{
    return (HCI_LE_ConnectionlessIQ_SampleEnableCmd(    BUILD_UINT16(pBuf[0],pBuf[1]),//         uint16 sync_handle,
                                                        pBuf[2],                //   uint8 enable,
                                                        pBuf[3],                //   uint8 slot_Duration,
                                                        pBuf[4],                //   uint8 MaxSampledCTEs,
                                                        pBuf[5],                //   uint8 pattern_len,
                                                        &pBuf[6]                //   uint8 *AnaIDs)
                                                   ));
}

hciStatus_t hciLESet_ConnectionCTE_ReceiveParam(uint8* pBuf )
{
    // record for BQB
    llConnState_t* connPtr;
    // MAS BV 65
    static uint8 cnt=0;
    connPtr = connPtr = &conn_param[0];

    if( (pBuf[2] == 1) && (connPtr->llConnCTE.enable) )
        connPtr->llConnCTE.enable = FALSE;

//  if( pBuf[2] == 1 )
//  {
//      ll_hw_set_cte_rxSupp( CTE_SUPP_AUTO );
//      if( cnt == 0 )
//      {
//          connPtr->llConnCTE.CTE_Length = 2;
//          cnt = 1;
//      }
//      else if( cnt == 1 )
//      {
//          connPtr->llConnCTE.CTE_Length = 0x0A;
//          cnt = 2;
//      }
//      else if( cnt == 2)
//          connPtr->llConnCTE.CTE_Length = 0x14;
//      connPtr->llConnCTE.CTE_Type = 0;
//  }
    return (HCI_LE_Set_ConnectionCTE_ReceiveParamCmd(BUILD_UINT16(pBuf[0],pBuf[1]),         //connHandle,
                                                     pBuf[2],                                     // enable,
                                                     pBuf[3],                                     // slot_Duration,
                                                     pBuf[4],                                     // pattern_len,
                                                     &pBuf[5]));                                  // *AnaIDs);
}

hciStatus_t hciLESet_ConnectionCTE_TransmitParam(uint8* pBuf)
{
    return (HCI_LE_Set_ConnectionCTE_TransmitParamCmd(  BUILD_UINT16(pBuf[0],pBuf[1]),  // connHandle,
                                                        pBuf[2],                       // type,
                                                        pBuf[3],                       //pattern_len,
                                                        &pBuf[4]));                    // *AnaIDs
}


// 2020-02-13 periodic cte req & rsp
extern unsigned int CTE_Count_Idx;
hciStatus_t hciLE_ConnectionCTERequestEnable(uint8* pBuf)
{
    // record for BQB
    llConnState_t* connPtr;
    connPtr = connPtr = &conn_param[0];

    if( (pBuf[2] == 1) && (connPtr->llCTE_ReqFlag) )
        connPtr->llCTE_ReqFlag= FALSE;

    CTE_Count_Idx = 0;
    return (HCI_LE_Connection_CTE_Request_EnableCmd(    BUILD_UINT16(pBuf[0],pBuf[1]),  // connHandle,
                                                        pBuf[2],                        // enable,
                                                        BUILD_UINT16(pBuf[3],pBuf[4]),  // Interval,
                                                        pBuf[5],                        // len,
                                                        pBuf[6]));                      // type
}
hciStatus_t hciLE_ConnectionCTEResponseEnable(uint8* pBuf)
{
    // record for BQB
    llConnState_t* connPtr;
    connPtr = connPtr = &conn_param[0];

    if( (pBuf[2] == 0) && (connPtr->llConnCTE.enable) )
        connPtr->llConnCTE.enable = FALSE;

    return (HCI_LE_Connection_CTE_Response_EnableCmd(BUILD_UINT16(pBuf[0],pBuf[1]),pBuf[2]));
}

hciStatus_t hciLE_READ_Anatenna_Info(uint8* pBuf)
{
    return (HCI_LE_READ_Anatenna_InfoCmd());
}



hciStatus_t hciLESetPrdAdvRecvEnableCmd                 ( uint8* pBuf )
{
    return HCI_LE_SetPrdAdvRecvEnableCmd(BUILD_UINT16(pBuf[0], pBuf[1]),
                                         pBuf[2]);
}


hciStatus_t hciExtSetRxGain( uint8* pBuf )
{
    return HCI_EXT_SetRxGainCmd( pBuf[0] );
}

hciStatus_t hciExtSetTxPower( uint8* pBuf )
{
    return HCI_EXT_SetTxPowerCmd( pBuf[0] );
}

hciStatus_t hciExtExtendRfRange( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_EXT_ExtendRfRangeCmd();
}


hciStatus_t hciExtHaltDuringRf( uint8* pBuf )
{
    return HCI_EXT_HaltDuringRfCmd( pBuf[0] );
}

hciStatus_t hciExtOnePktPerEvt( uint8* pBuf )
{
    return HCI_EXT_OnePktPerEvtCmd( pBuf[0] );
}

hciStatus_t hciExtClkDivOnHalt( uint8* pBuf )
{
    return HCI_EXT_ClkDivOnHaltCmd( pBuf[0] );
}


hciStatus_t hciExtDeclareNvUsage( uint8* pBuf )
{
    return HCI_EXT_DeclareNvUsageCmd( pBuf[0] );
}

hciStatus_t hciExtDecrypt( uint8* pBuf )
{
    // reverse byte order of key (MSB..LSB required)
    HCI_ReverseBytes( &pBuf[0], KEYLEN );
    // reverse byte order of encText (MSB..LSB required)
    HCI_ReverseBytes( &pBuf[KEYLEN], KEYLEN );
    return HCI_EXT_DecryptCmd( &pBuf[0],
                               &pBuf[KEYLEN] );
}


hciStatus_t hciExtSetLocalSupportedFeatures( uint8* pBuf )
{
    return HCI_EXT_SetLocalSupportedFeaturesCmd( pBuf );
}


hciStatus_t hciExtSetFastTxResponseTime( uint8* pBuf )
{
    return HCI_EXT_SetFastTxResponseTimeCmd( pBuf[0] );
}

hciStatus_t hciExtSetSlaveLatencyOverride( uint8* pBuf )
{
    return HCI_EXT_SetSlaveLatencyOverrideCmd( pBuf[0] );
}


hciStatus_t hciExtModemTestTx( uint8* pBuf )
{
    return HCI_EXT_ModemTestTxCmd( pBuf[0], pBuf[1] );
}


hciStatus_t hciExtModemHopTestTx( uint8* pBuf )
{
    return HCI_EXT_ModemHopTestTxCmd();
}


hciStatus_t hciExtModemtestRx( uint8* pBuf )
{
    return HCI_EXT_ModemTestRxCmd( pBuf[0] );
}


hciStatus_t hciExtEndModemTest( uint8* pBuf )
{
    return HCI_EXT_EndModemTestCmd();
}


hciStatus_t hciExtSetBDADDR( uint8* pBuf )
{
    return HCI_EXT_SetBDADDRCmd( pBuf );
}


hciStatus_t hciExtSetSCA( uint8* pBuf )
{
    return HCI_EXT_SetSCACmd( BUILD_UINT16(pBuf[0], pBuf[1]) );
}


hciStatus_t hciExtSetMaxDtmTxPower( uint8* pBuf )
{
    return HCI_EXT_SetMaxDtmTxPowerCmd( pBuf[0] );
}



hciStatus_t hciExtMapPmIoPort( uint8* pBuf )
{
//  return HCI_EXT_MapPmIoPortCmd( pBuf[0],
//                                 pBuf[1] );
    return (HCI_SUCCESS);
}


hciStatus_t hciExtSetFreqTune( uint8* pBuf )
{
    return HCI_EXT_SetFreqTuneCmd( pBuf[0] );
}


hciStatus_t hciExtSaveFreqTune( uint8* pBuf )
{
    return HCI_EXT_SaveFreqTuneCmd();
}


hciStatus_t hciExtDisconnectImmed( uint8* pBuf )
{
    return HCI_EXT_DisconnectImmedCmd ( BUILD_UINT16(pBuf[0],
                                                     pBuf[1]) );
}


hciStatus_t hciExtPER( uint8* pBuf )
{
    return HCI_EXT_PacketErrorRateCmd ( BUILD_UINT16(pBuf[0],
                                                     pBuf[1]),
                                        pBuf[2] );
}


hciStatus_t hciExtOverlappedProcessing( uint8* pBuf )
{
    return HCI_EXT_OverlappedProcessingCmd ( pBuf[0] );
}


hciStatus_t hciExtNumComplPktsLimit( uint8* pBuf )
{
    return HCI_EXT_NumComplPktsLimitCmd( pBuf[0],
                                         pBuf[1] );
}



hciStatus_t hciExtBuildRevision( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_EXT_BuildRevisionCmd( pBuf[0], BUILD_UINT16( pBuf[1],
                                                            pBuf[2]) );
}


hciStatus_t hciExtDelaySleep( uint8* pBuf )
{
    return HCI_EXT_DelaySleepCmd( BUILD_UINT16(pBuf[0], pBuf[1]) );
}


hciStatus_t hciExtResetSystem( uint8* pBuf )
{
    return HCI_EXT_ResetSystemCmd( pBuf[0] );
}


// ================
hciStatus_t hciLEReadMaxDataLength                 ( uint8* pBuf )
{
    return HCI_LE_ReadMaxDataLengthCmd    ();
}


// HCI Packet Opcode Jump Table
cmdPktTable_t hciCmdTable[] =
{
    // Linker Control Commands
    {HCI_DISCONNECT, hciDisconnect                    },
    {HCI_READ_REMOTE_VERSION_INFO, hciReadRemoteVersionInfo         },

    // Controller and Baseband Commands
    {HCI_SET_EVENT_MASK, hciSetEventMask                  },
    {HCI_RESET, hciReset                         },

    {HCI_READ_TRANSMIT_POWER, hciReadTransmitPowerLevel        },
    {HCI_SET_CONTROLLER_TO_HOST_FLOW_CONTROL, hciSetControllerToHostFlowCtrl   },
    {HCI_HOST_BUFFER_SIZE, hciHostBufferSize                },
    {HCI_HOST_NUM_COMPLETED_PACKETS, hciHostNumCompletedPkt           },
    {HCI_SET_EVENT_MASK_PAGE2,hciSetEventMaskPage2},


// Informational Parameters
    {HCI_READ_LOCAL_VERSION_INFO, hciReadLocalVersionInfo          },
    {HCI_READ_LOCAL_SUPPORTED_COMMANDS, hciReadLocalSupportedCommands    },
    {HCI_READ_LOCAL_SUPPORTED_FEATURES, hciReadLocalSupportedFeatures    },
    {HCI_READ_BDADDR, hciReadBDADDR                    },
    {HCI_READ_RSSI, hciReadRssi                      },

    // LE Commands
    {HCI_LE_SET_EVENT_MASK, hciLESetEventMask                },
    {HCI_LE_READ_BUFFER_SIZE, hciLEReadBufSize                 },
    {HCI_LE_READ_LOCAL_SUPPORTED_FEATURES, hciLEReadLocalSupportedFeatures  },
    {HCI_LE_SET_RANDOM_ADDR, hciLESetRandomAddr               },

    {HCI_LE_SET_ADV_PARAM, hciLESetAdvParam                 },
    {HCI_LE_SET_ADV_DATA, hciLESetAdvData                  },
    {HCI_LE_SET_SCAN_RSP_DATA, hciLESetScanRspData              },
    {HCI_LE_SET_ADV_ENABLE, hciLESetAdvEnab                  },
    {HCI_LE_READ_ADV_CHANNEL_TX_POWER, hciLEReadAdvChanTxPower          },

    {HCI_LE_SET_SCAN_PARAM, hciLESetScanParam                },
    {HCI_LE_SET_SCAN_ENABLE, hciLESetScanEnable               },

    {HCI_LE_CREATE_CONNECTION, hciLECreateConn                  },
    {HCI_LE_CREATE_CONNECTION_CANCEL, hciLECreateConnCancel            },

    {HCI_LE_READ_WHITE_LIST_SIZE, hciLEReadWhiteListSize           },
    {HCI_LE_CLEAR_WHITE_LIST, hciLEClearWhiteList              },
    {HCI_LE_ADD_WHITE_LIST, hciLEAddWhiteList                },
    {HCI_LE_REMOVE_WHITE_LIST, hciLERemoveWhiteList             },

    {HCI_LE_CONNECTION_UPDATE, hciLEConnUpdate                  },
    {HCI_LE_SET_HOST_CHANNEL_CLASSIFICATION, hciLESetHostChanClass            },
    {HCI_LE_READ_CHANNEL_MAP, hciLEReadChanMap                 },
    {HCI_LE_READ_REMOTE_USED_FEATURES, hciLEReadRemoteUsedFeatures      },

    {HCI_LE_ENCRYPT, hciLEEncrypt                     },
    {HCI_LE_RAND, hciLERand                        },

    {HCI_LE_START_ENCRYPTION, hciLEStartEncrypt                },

    {HCI_LE_LTK_REQ_REPLY, hciLELtkReqReply                 },
    {HCI_LE_LTK_REQ_NEG_REPLY, hciLELtkReqNegReply              },

    {HCI_LE_READ_SUPPORTED_STATES, hciLEReadSupportedStates         },
    {HCI_LE_RECEIVER_TEST, hciLEReceiverTest                },
    {HCI_LE_TRANSMITTER_TEST, hciLETransmitterTest             },
    {HCI_LE_TEST_END, hciLETestEnd                     },
    {HCI_LE_SET_DATA_LENGTH, hciLESetDataLength               },

    {HCI_LE_READ_SUGGESTED_DEFAULT_DATA_LENGTH, hciLEReadSuggestedDefaultDataLength    },
    {HCI_LE_WRITE_SUGGESTED_DEFAULT_DATA_LENGTH, hciLEWriteSuggestedDefaultDataLength    },
    {HCI_LE_READ_PHY, hciLEReadPhyMode                 },
    {HCI_LE_SET_DEFAULT_PHY, hciLESetDefaultPhyMode           },
    {HCI_LE_SET_PHY, hciLESetPhyMode                  },

// ===== 2020-07 add, RPA
    {HCI_LE_ADD_DEVICE_TO_RESOLVING_LIST, hciLEAddDeviceToRL               },
    {HCI_LE_REMOVE_DEVICE_FROM_RESOLVING_LIST, hciLERemoveDeviceFromRL          },
    {HCI_LE_CLEAR_RESOLVING_LIST, hciLEClearRL                     },
    {HCI_LE_READ_RESOLVING_LIST_SIZE, hciLEReadRLSize                  },
    {HCI_LE_READ_PEER_RESOLVABLE_ADDRESS, hciLEReadPeerRA                  },
    {HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS, hciLEReadLocalRA                 },
    {HCI_LE_SET_ADDRESS_RESOLUTION_ENABLE, hciLESetAddrResolutionEnable     },
    {HCI_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TO, hciLESetRpaTo                    },
    {HCI_LE_READ_MAXIMUM_DATA_LENGTH,          hciLEReadMaxDataLength            },


    // extended adv
    {HCI_LE_SET_ADVERTISING_SET_RANDOM_ADDRESS, hciLESetAdvSetRandomAddress},
    {HCI_LE_SET_EXTENDER_ADVERTISING_PARAMETERS, hciLESetExtAdvParam        },
    {HCI_LE_SET_EXTENDED_ADVERTISING_DATA, hciLESetExtAdvData         },
    {HCI_LE_Set_EXTENDED_SCAN_RESPONSE_DATA, hciLESetExtScanRspData     },
    {HCI_LE_Set_EXTENDED_ADVERTISING_ENABLE, hciLESetExtAdvEnable       },
    {HCI_LE_READ_MAXIMUM_ADVERTISING_DATA_LENGTH, hciLEReadMaximumAdvDataLength},
    {HCI_LE_READ_NUMBER_OF_SUPPORTED_ADVERTISING_SETS, hciLEReadNumberOfSupportAdvSet},
    {HCI_LE_REMOVE_ADVERTISING_SET, hciLERemoveAdvSet          },
    {HCI_LE_CLEAR_ADVERTISING_SETS, hciLEClearAdvSets          },

    // periodic adv
    {HCI_LE_SET_PERIODIC_ADVERTISING_PARAMETERS, hciLESetPeriodicAdvParameter},
    {HCI_LE_SET_PERIODIC_ADVERTISING_DATA, hciLESetPeriodicAdvData   },
    {HCI_LE_Set_PERIODIC_ADVERTISING_ENABLE, hciLESetPeriodicAdvEnable },

    // extended scan
    {HCI_LE_SET_EXTENDED_SCAN_PARAMETERS, hciLESetExtendedScanParameters  },
    {HCI_LE_SET_EXTENDED_SCAN_ENABLE, hciLESetExtendedScanEnableCmd   },
    {HCI_LE_EXTENDED_CREATE_CONNECTION, hciLEExtendedCreateConnection   },

    // periodic scan
    {HCI_LE_PERIODIC_ADVERTISING_CREATE_SYNC, hciLEPeriodicAdvertisingCreateSync       },
    {HCI_LE_PERIODIC_ADVERTISING_CREATE_SYNC_CANCEL, hciLEPeriodicAdvertisingCreateSyncCancel },
    {HCI_LE_PERIODIC_ADVERTISING_TERMINATE_SYNC, hciLEPeriodicAdvertisingTerminateSync    },

    {HCI_LE_ADD_DEVICE_TO_PERIODIC_ADVERTISER_LIST, hciLEAddDevToPeriodicAdvList      },
    {HCI_LE_REMOVE_DEVICE_FROM_PERIODIC_ADVERTISER_LIST, hciLERemovePeriodicAdvList        },
    {HCI_LE_CLEAR_PERIODIC_ADVERTISER_LIST, hciLEClearPeriodicAdvList         },
    {HCI_LE_READ_PERIODIC_ADVERTISER_LIST_SIZE, hciLEReadPeriodicAdvListSize      },

    {HCI_LE_READ_TRANSMIT_POWER, hciLEReadTransmitPower        },
    {HCI_LE_READ_RF_PATH_COMPENSATION, hciLEReadRfPathCompensation     },
    {HCI_LE_WRITE_RF_PATH_COMPENSATION, hciLEWriteRfPathCompensation  },

    {HCI_LE_SET_PRIVACY_MODE, hciLESetPrivacyMode},

    // 2020-10-27 CTE
    {HCI_LE_SET_CONNLESS_CTE_TRANS_PARAMETER, hciLE_ConnectionlessCTE_TransmitParamCmd },
    {HCI_LE_SET_CONNLESS_CTE_TRANS_ENABLE, hciLE_ConnectionlessCTE_TransmitEnableCmd},
    {HCI_LE_SET_CONNLESS_IQ_SAMPLE_ENABLE, hciLE_ConnectionlessIQ_SampleEnableCmd},
    {HCI_LE_SET_CONNCTE_RECV_PARAMETER,   hciLESet_ConnectionCTE_ReceiveParam},
    {HCI_LE_SET_CONN_CTE_TRANSMIT_PARAMETER,   hciLESet_ConnectionCTE_TransmitParam},
    {HCI_LE_CONN_CTE_REQUEST_ENABLE,   hciLE_ConnectionCTERequestEnable},
    {HCI_LE_CONN_CTE_RESPONSE_ENABLE,   hciLE_ConnectionCTEResponseEnable},
    {HCI_LE_READ_ANTENNA_INFO,   hciLE_READ_Anatenna_Info},
    {HCI_LE_SET_PERIODIC_ADV_RECV_ENABLE, hciLESetPrdAdvRecvEnableCmd},

    // Vendor Specific Commands
    {HCI_EXT_SET_RX_GAIN, hciExtSetRxGain                  },
    {HCI_EXT_SET_TX_POWER, hciExtSetTxPower                 },
    {HCI_EXT_EXTEND_RF_RANGE, hciExtExtendRfRange              },
    {HCI_EXT_HALT_DURING_RF, hciExtHaltDuringRf               },

    {HCI_EXT_ONE_PKT_PER_EVT, hciExtOnePktPerEvt               },

    {HCI_EXT_CLK_DIVIDE_ON_HALT, hciExtClkDivOnHalt               },
    {HCI_EXT_DECLARE_NV_USAGE, hciExtDeclareNvUsage             },

    {HCI_EXT_DECRYPT, hciExtDecrypt                    },
    {HCI_EXT_SET_LOCAL_SUPPORTED_FEATURES, hciExtSetLocalSupportedFeatures  },

    {HCI_EXT_SET_FAST_TX_RESP_TIME, hciExtSetFastTxResponseTime      },
    {HCI_EXT_OVERRIDE_SL, hciExtSetSlaveLatencyOverride    },

    {HCI_EXT_MODEM_TEST_TX, hciExtModemTestTx                },
    {HCI_EXT_MODEM_HOP_TEST_TX, hciExtModemHopTestTx             },
    {HCI_EXT_MODEM_TEST_RX, hciExtModemtestRx                },
    {HCI_EXT_END_MODEM_TEST, hciExtEndModemTest               },
    {HCI_EXT_SET_BDADDR, hciExtSetBDADDR                  },

    {HCI_EXT_SET_SCA, hciExtSetSCA                     },

    {HCI_EXT_SET_MAX_DTM_TX_POWER, hciExtSetMaxDtmTxPower           },
    {HCI_EXT_MAP_PM_IO_PORT                   , hciExtMapPmIoPort                },
    {HCI_EXT_SET_FREQ_TUNE, hciExtSetFreqTune                },
    {HCI_EXT_SAVE_FREQ_TUNE, hciExtSaveFreqTune               },

    {HCI_EXT_DISCONNECT_IMMED, hciExtDisconnectImmed            },
    {HCI_EXT_PER, hciExtPER                        },
    {HCI_EXT_OVERLAPPED_PROCESSING, hciExtOverlappedProcessing       },
    {HCI_EXT_NUM_COMPLETED_PKTS_LIMIT, hciExtNumComplPktsLimit          },

    {HCI_EXT_BUILD_REVISION, hciExtBuildRevision              },
    {HCI_EXT_DELAY_SLEEP, hciExtDelaySleep                 },
    // TEMP: OVERLAPPED PROCESSING HOLDER
    {HCI_EXT_RESET_SYSTEM, hciExtResetSystem                },

    // Last Table Entry Delimiter
    {0xFFFF, NULL                             }
};


void hciProcessHostToCtrlCmd( uint8_t* pData )
{
    uint16 cmdOpCode;
    uint8  status;
    uint8  i = 0;
    // retrieve opcode
    cmdOpCode = BUILD_UINT16 (pData[1], pData[2]);

    // lookup corresponding function
    while ((hciCmdTable[i].opCode != 0xFFFF) && (hciCmdTable[i].hciFunc != NULL))
    {
        // there's a valid entry at this index, but check if it's the one we want
        if (hciCmdTable[i].opCode == cmdOpCode)
        {
            // it is, so jump to this function
            (void)(hciCmdTable[i].hciFunc)(&pData[4]);
            // done
            break;
        }

        // next...
        i++;
    }

    // check if a matching opcode was found
    if ((hciCmdTable[i].opCode == 0xFFFF) && (hciCmdTable[i].hciFunc == NULL))
    {
        // none found, so return error
        status = BT_HCI_ERR_UNKNOWN_CMD;
        HCI_CommandCompleteEvent ( cmdOpCode, 1, &status);
    }

    return;
}




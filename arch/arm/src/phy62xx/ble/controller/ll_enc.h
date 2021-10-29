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
    Filename:       ll_enc.h
    Revised:
    Revision:

    Description:    This file contains the Link Layer (LL) types, contants,
                  API's etc. for the Bluetooth Low Energy (BLE) Controller
                  CCM encryption and decryption.

                  This API is based on ULP BT LE D09R23.


*******************************************************************************/

#ifndef LL_ENC_H
#define LL_ENC_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
    INCLUDES
*/
#include "bcomdef.h"
#include "ll_def.h"

/*******************************************************************************
    MACROS
*/

/*******************************************************************************
    CONSTANTS
*/

#define LL_ENC_TX_DIRECTION_MASTER   1
#define LL_ENC_TX_DIRECTION_SLAVE    0
#define LL_ENC_RX_DIRECTION_MASTER   0
#define LL_ENC_RX_DIRECTION_SLAVE    1

#define LL_ENC_DATA_BANK_MASK 0xFF7F

#define LL_ENC_TRUE_RAND_BUF_SIZE     ((LL_ENC_IV_LEN/2) + (LL_ENC_SKD_LEN/2))

// Generate Session Key using LTK for key and SKD for plaintext.
#define LL_ENC_GenerateSK LL_ENC_AES128_Encrypt

/*******************************************************************************
    TYPEDEFS
*/

/*******************************************************************************
    LOCAL VARIABLES
*/

/*******************************************************************************
    GLOBAL VARIABLES
*/
extern uint8 dataPkt[2*LL_ENC_BLOCK_LEN];
extern uint8 cachedTRNGdata[ LL_ENC_TRUE_RAND_BUF_SIZE ];

/*******************************************************************************
    Functions
*/

// Random Number Generation
extern uint8 LL_ENC_GeneratePseudoRandNum( void );
extern uint8 LL_ENC_GenerateTrueRandNum( uint8* buf, uint8 len );

// CCM Encryption
extern void  LL_ENC_AES128_Encrypt( uint8* key, uint8* plaintext,  uint8* ciphertext );
extern void  LL_ENC_AES128_Decrypt( uint8* key, uint8* ciphertext, uint8* plaintext );
extern void  LL_ENC_LoadEmptyIV( void );
extern void  LL_ENC_ReverseBytes( uint8* buf, uint8 len );
extern void  LL_ENC_GenDeviceSKD( uint8* SKD );
extern void  LL_ENC_GenDeviceIV( uint8* IV );
extern void  LL_ENC_GenerateNonce( uint32 pktCnt, uint8 direction, uint8* nonce );
extern void  LL_ENC_EncryptMsg( uint8* nonce, uint8 pktLen, uint8* pbuf, uint8* mic );
extern void  LL_ENC_DecryptMsg( uint8* nonce, uint8 pktLen, uint8* pBuf, uint8* mic );
extern void  LL_ENC_Encrypt( llConnState_t* connPtr, uint8 pktHdr, uint8 pktLen, uint8* pBuf );
extern uint8 LL_ENC_Decrypt( llConnState_t* connPtr, uint8 pktHdr, uint8 pktLen, uint8* pBuf );
extern void LL_ENC_sm_ah( uint8* pK, uint8* pR, uint8* pAh );
//

extern void  LL_ENC_MoveData( uint8* pDst, uint8* pSrc, uint16 len );

#ifdef __cplusplus
}
#endif

#endif /* LL_ENC_H */

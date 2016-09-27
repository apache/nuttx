/****************************************************************************
 *  cc3000_common.h  - CC3000 Host Driver Implementation.
 *  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_CC3000_CC3300_COMMON_H
#define __INCLUDE_NUTTX_WIRELESS_CC3000_CC3300_COMMON_H

/****************************************************************************
 * Included files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include <sys/time.h>
#include <stdlib.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Error codes */

#define ESUCCESS          0
#define EFAIL             -1
#define EERROR            EFAIL

/* Common definitions */

#define ERROR_SOCKET_INACTIVE -57

#define WLAN_ENABLE       (1)
#define WLAN_DISABLE      (0)

#define MAC_ADDR_LEN      (6)

#define SP_PORTION_SIZE   (32)

/* Defines for minimal and maximal RX buffer size. This size includes the spi
 * header and hci header.
 * The maximal buffer size derives from:
 *   MTU + HCI header + SPI header + sendto() agrs size
 * The minimum buffer size derives from:
 *   HCI header + SPI header + max args size
 *
 * This buffer is used for receiving events and data.
 * The packet can not be longer than MTU size and CC3000 does not support
 * fragmentation. Note that the same buffer is used for reception of the data
 * and events from CC3000. That is why the minimum is defined.
 * The calculation for the actual size of buffer for reception is:
 * Given the maximal data size MAX_DATA that is expected to be received by
 * application, the required buffer is:
 * Using recv() or recvfrom():
 *
 *   max(CC3000_MINIMAL_RX_SIZE, MAX_DATA + HEADERS_SIZE_DATA + fromlen
 *   + ucArgsize + 1)
 *
 * Using gethostbyname() with minimal buffer size will limit the host name
 * returned to 99 bytes only.
 * The 1 is used for the overrun detection
 *
 * Buffer size increased to 130 following the add_profile() with WEP security
 * which requires TX buffer size of 130 bytes:
 * HEADERS_SIZE_EVNT + WLAN_ADD_PROFILE_WEP_PARAM_LEN + MAX SSID LEN + 4 * MAX KEY LEN = 130
 * MAX SSID LEN = 32
 * MAX SSID LEN = 13 (with add_profile only ascii key setting is supported,
 * therfore maximum key size is 13)
 */

#define CC3000_MINIMAL_RX_SIZE      (130 + 1)
#define CC3000_MAXIMAL_RX_SIZE      (1519 + 1)

/* Defines for minimal and maximal TX buffer size.
 * This buffer is used for sending events and data.
 * The packet can not be longer than MTU size and CC3000 does not support
 * fragmentation. Note that the same buffer is used for transmission of the
 * data and commands. That is why the minimum is defined.
 * The calculation for the actual size of buffer for transmission is:
 * Given the maximal data size MAX_DATA, the required buffer is:
 * Using Sendto():
 *
 *  max(CC3000_MINIMAL_TX_SIZE, MAX_DATA + SPI_HEADER_SIZE
 *  + SOCKET_SENDTO_PARAMS_LEN + SIMPLE_LINK_HCI_DATA_HEADER_SIZE + 1)
 *
 * Using Send():
 *
 *  max(CC3000_MINIMAL_TX_SIZE, MAX_DATA + SPI_HEADER_SIZE
 *  + HCI_CMND_SEND_ARG_LENGTH + SIMPLE_LINK_HCI_DATA_HEADER_SIZE + 1)
 *
 * The 1 is used for the overrun detection
 */

#define  CC3000_MINIMAL_TX_SIZE      (130 + 1)
#define  CC3000_MAXIMAL_TX_SIZE      (1519 + 1)

/* TX and RX buffer sizes, allow to receive and transmit maximum data at
 * length 8.
 */

#ifdef CC3000_TINY_DRIVER
#  define TINY_CC3000_MAXIMAL_RX_SIZE 44
#  define TINY_CC3000_MAXIMAL_TX_SIZE 59
#endif

/* In order to determine your preferred buffer size,
 * change CC3000_MAXIMAL_RX_SIZE and CC3000_MAXIMAL_TX_SIZE to a value between
 * the minimal and maximal specified above.
 * Note that the buffers are allocated by SPI.
 * In case you change the size of those buffers, you might need also to change
 * the linker file, since for example on MSP430 FRAM devices the buffers are
 * allocated in the FRAM section that is allocated manually and not by IDE.
 */

#ifndef CC3000_TINY_DRIVER
#  define CC3000_RX_BUFFER_SIZE   (CC3000_MAXIMAL_RX_SIZE)
#  define CC3000_TX_BUFFER_SIZE   (CC3000_MAXIMAL_TX_SIZE)

/* if defined TINY DRIVER we use smaller RX and TX buffer in order to minimize
 * RAM consumption
 */

#else
#  define CC3000_RX_BUFFER_SIZE   (TINY_CC3000_MAXIMAL_RX_SIZE)
#  define CC3000_TX_BUFFER_SIZE   (TINY_CC3000_MAXIMAL_TX_SIZE)
#endif

/* This macro is used for copying 8 bit to stream while converting to little
 * endian format.
 */

#define UINT8_TO_STREAM(_p, _val)  {*(_p)++ = (_val);}

/* This macro is used for copying 16 bit to stream while converting to little
 * endian format.
 */

#define UINT16_TO_STREAM(_p, _u16)  (UINT16_TO_STREAM_f(_p, _u16))

/* This macro is used for copying 32 bit to stream while converting to little
 * endian format.
 */

#define UINT32_TO_STREAM(_p, _u32)  (UINT32_TO_STREAM_f(_p, _u32))

/* This macro is used for copying a specified value length bits (l) to stream
 * while converting to little endian format.
 */

#define ARRAY_TO_STREAM(p, a, l)   {register int16_t _i; for (_i = 0; _i < l; _i++) *(p)++ = ((uint8_t *) a)[_i];}

/* This macro is used for copying received stream to 8 bit in little endian
 * format.
 */

#define STREAM_TO_UINT8(_p, _offset, _u8)  {_u8 = (uint8_t)(*(_p + _offset));}

/* This macro is used for copying received stream to 16 bit in little endian
 * format.
 */

#define STREAM_TO_UINT16(_p, _offset, _u16)  {_u16 = STREAM_TO_UINT16_f(_p, _offset);}

/* This macro is used for copying received stream to 32 bit in little endian
 * format.
 */

#define STREAM_TO_UINT32(_p, _offset, _u32)  {_u32 = STREAM_TO_UINT32_f(_p, _offset);}
#define STREAM_TO_STREAM(p, a, l)   {register int16_t _i; for (_i = 0; _i < l; _i++) *(a)++= ((uint8_t *) p)[_i];}


/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct wlan_buffer_desc_s
{
  uint8_t *pbuffer;
  ssize_t len;
} wlan_buffer_desc;

typedef char *(*tFWPatches)(unsigned long *usLength);

typedef char *(*tDriverPatches)(unsigned long *usLength);

typedef char *(*tBootLoaderPatches)(unsigned long *usLength);

typedef void (*tWlanCB)(long event_type, char * data, uint8_t length );

typedef long (*tWlanReadInteruptPin)(void);

typedef void (*tWlanInterruptEnable)(void);

typedef void (*tWlanInterruptDisable)(void);

typedef void (*tWriteWlanPin)(uint8_t val);

typedef struct
{
  uint16_t              usRxEventOpcode;
  uint16_t              usEventOrDataReceived;
  uint8_t              *pucReceivedData;
  uint8_t              *pucTxCommandBuffer;
  wlan_buffer_desc      usrBuffer;

  tFWPatches            sFWPatches;
  tDriverPatches        sDriverPatches;
  tBootLoaderPatches    sBootLoaderPatches;
  tWlanCB               sWlanCB;

  signed long           slTransmitDataError;
  uint16_t              usNumberOfFreeBuffers;
  uint16_t              usSlBufferLength;
  uint16_t              usBufferSize;
  uint16_t              usRxDataPending;

  unsigned long         NumberOfSentPackets;
  unsigned long         NumberOfReleasedPackets;

  uint8_t   InformHostOnTxComplete;
} sSimplLinkInformation;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef  __cplusplus
extern "C"
{
#endif

extern volatile sSimplLinkInformation tSLInformation;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: wlan_get_buffer
 *
 * Input Parameters:
 *   pdes - Location to return the buffer pointer.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void wlan_get_buffer(wlan_buffer_desc *pdes);

/****************************************************************************
 * Name: SimpleLinkWaitEvent
 *
 * Description:
 *   Wait for event, pass it to the hci_event_handler and update the event
 *   opcode in a global variable.
 *
 * Input Parameters:
 *   usOpcode      command operation code
 *   pRetParams    command return parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void SimpleLinkWaitEvent(uint16_t usOpcode, void *pRetParams);

/****************************************************************************
 * Name: SimpleLinkWaitData
 *
 * Description:
 *    Wait for data, pass it to the hci_event_handler and update in a global
 *    variable that there is data to read.
 *
 * Input Parameters:
 *   pBuf       data buffer
 *   from       from information
 *   fromlen  from information length
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void SimpleLinkWaitData(uint8_t *pBuf, uint8_t *from, uint8_t *fromlen);

/****************************************************************************
 * Name: UINT32_TO_STREAM_f
 *
 * Description:
 *   This function is used for copying 32 bit to stream while converting to
 *   little endian format.
 *
 * Input Parameters:
 *   p       pointer to the new stream
 *   u32     pointer to the 32 bit
 *
 * Returned Value:
 *   Pointer to the new stream
 *
 ****************************************************************************/

uint8_t *UINT32_TO_STREAM_f(uint8_t *p, unsigned long u32);

/****************************************************************************
 * Name: UINT16_TO_STREAM_f
 *
 * Description:
 *   This function is used for copying 16 bit to stream while converting to
 *   little endian format.
 *
 * Input Parameters:
 *   p       pointer to the new stream
 *   u16     pointer to the 16 bit
 *
 * Returned Value:
 *   Pointer to the new stream
 *
 ****************************************************************************/

uint8_t *UINT16_TO_STREAM_f(uint8_t *p, uint16_t u16);

/****************************************************************************
 * Name: STREAM_TO_UINT16_f
 *
 * Description:
 *   This function is used for copying received stream to 16 bit in little
 *   endian format.
 *
 * Input Parameters:
 *   p          pointer to the stream
 *   offset     offset in the stream
 *
 * Returned Value:
 *   Pointer to the new 16 bit
 *
 ****************************************************************************/

uint16_t STREAM_TO_UINT16_f(char* p, uint16_t offset);

/****************************************************************************
 * Name: STREAM_TO_UINT32_f
 *
 * Description:
 *   This function is used for copying received stream to 32 bit in little
 *   endian format.
 *
 * Input Parameters:
 *   p          pointer to the stream
 *   offset     offset in the stream
 *
 * Returned Value:
 *   Pointer to the new 32 bit
 *
 ****************************************************************************/

unsigned long STREAM_TO_UINT32_f(char* p, uint16_t offset);

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // __INCLUDE_NUTTX_WIRELESS_CC3000_CC3300_COMMON_H

/****************************************************************************
 * include/nuttx/wireless/cc3000.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *
 * References:
 *   CC30000 from Texas Instruments http://processors.wiki.ti.com/index.php/CC3000
 *
 * See also:
 * 		http://processors.wiki.ti.com/index.php/CC3000_Host_Driver_Porting_Guide
 * 		http://processors.wiki.ti.com/index.php/CC3000_Host_Programming_Guide
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_CC3000_H
#define __INCLUDE_NUTTX_WIRELESS_CC3000_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/wireless/wireless.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define DEV_FORMAT   "/dev/wireless%d" /* The device Name*/
#define DEV_NAMELEN  17		       /* The buffer size to hold formatted string*/

#define QUEUE_FORMAT   "wlq%d"         /* The Queue name */
#define QUEUE_NAMELEN  8	       /* The buffer size to hold formatted string*/

#define SEM_FORMAT   "wls%d"	       /* The Spi Resume Senaphore name*/
#define SEM_NAMELEN   8		       /* The buffer size to hold formatted string*/


/* IOCTL commands */

#define CC3000IOC_GETQUESEMID         _WLIOC_USER(0x0001)   /* arg: Address of int for number*/

/****************************************************************************
 * Public Types
 ****************************************************************************/
typedef char *(*tFWPatches)(unsigned long *usLength);
typedef char *(*tDriverPatches)(unsigned long *usLength);
typedef char *(*tBootLoaderPatches)(unsigned long *usLength);
typedef void (*tWlanCB)(long event_type, char * data, unsigned char length );
typedef long (*tWlanReadInteruptPin)(void);
typedef void (*tWlanInterruptEnable)(void);
typedef void (*tWlanInterruptDisable)(void);
typedef void (*tWriteWlanPin)(unsigned char val);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

  /*****************************************************************************
   *
   *   CC3000_wlan_init
   *
   *   @param  sWlanCB   Asynchronous events callback.
   *                     0 no event call back.
   *                   -call back parameters:
   *                    1) event_type: HCI_EVNT_WLAN_UNSOL_CONNECT connect event,
   *                      HCI_EVNT_WLAN_UNSOL_DISCONNECT disconnect event,
   *                      HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE config done,
   *                      HCI_EVNT_WLAN_UNSOL_DHCP dhcp report,
   *                      HCI_EVNT_WLAN_ASYNC_PING_REPORT ping report OR
   *                      HCI_EVNT_WLAN_KEEPALIVE keepalive.
   *                    2) data: pointer to extra data that received by the event
   *                      (NULL no data).
   *                    3) length: data length.
   *                   -Events with extra data:
   *                      HCI_EVNT_WLAN_UNSOL_DHCP: 4 bytes IP, 4 bytes Mask,
   *                      4 bytes default gateway, 4 bytes DHCP server and 4 bytes
   *                      for DNS server.
   *                      HCI_EVNT_WLAN_ASYNC_PING_REPORT: 4 bytes Packets sent,
   *                      4 bytes Packets received, 4 bytes Min round time,
   *                      4 bytes Max round time and 4 bytes for Avg round time.
   *
   *   @param    sFWPatches  0 no patch or pointer to FW patches
   *   @param    sDriverPatches  0 no patch or pointer to driver patches
   *   @param    sBootLoaderPatches  0 no patch or pointer to bootloader patches
   *
   *   @return   none
   *
   *   @sa       wlan_set_event_mask , wlan_start , wlan_stop
   *
   *   @brief    Initialize wlan driver
   *
   *   @warning This function must be called before ANY other wlan driver function
   *
   ****************************************************************************/

void wlan_init( tWlanCB sWlanCB,
			tFWPatches sFWPatches,
			tDriverPatches sDriverPatches,
			tBootLoaderPatches sBootLoaderPatches);


void cc3000_wlan_init(tWlanCB sWlanCB,
			      tFWPatches sFWPatches,
			      tDriverPatches sDriverPatches,
			      tBootLoaderPatches sBootLoaderPatches);


/************************************************************************************
 * Name: wireless_archinitialize
 *
 * Description:
 *   Called to configure wireless module (wireless_archinitialize).
 *
 ************************************************************************************/

int wireless_archinitialize();


#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_CC3000_H */

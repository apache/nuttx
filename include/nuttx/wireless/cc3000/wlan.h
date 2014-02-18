/*****************************************************************************
 * include/nuttx/wireless/cc3000/wlan.h
 *
 *  wlan.h  - CC3000 Host Driver Implementation.
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
 *****************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_CC3000_WLAN_H
#define __INCLUDE_NUTTX_WIRELESS_CC3000_WLAN_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include "cc3000_common.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#define WLAN_SEC_UNSEC (0)
#define WLAN_SEC_WEP   (1)
#define WLAN_SEC_WPA   (2)
#define WLAN_SEC_WPA2  (3)

/*****************************************************************************
 * Public Data
 *****************************************************************************/

#ifdef  __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

/*****************************************************************************
 * Name: wlan_init
 *
 * Input Parameters:
 *   sWlanCB   Asynchronous events callback.
 *              0 no event call back.
 *            - call back parameters:
 *              1) event_type: HCI_EVNT_WLAN_UNSOL_CONNECT connect event,
 *                 HCI_EVNT_WLAN_UNSOL_DISCONNECT disconnect event,
 *                 HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE config done,
 *                 HCI_EVNT_WLAN_UNSOL_DHCP dhcp report,
 *                 HCI_EVNT_WLAN_ASYNC_PING_REPORT ping report OR
 *                 HCI_EVNT_WLAN_KEEPALIVE keepalive.
 *              2) data: pointer to extra data that received by the event
 *                 (NULL no data).
 *              3) length: data length.
 *            - Events with extra data:
 *                HCI_EVNT_WLAN_UNSOL_DHCP: 4 bytes IP, 4 bytes Mask,
 *                4 bytes default gateway, 4 bytes DHCP server and 4 bytes
 *                for DNS server.
 *                HCI_EVNT_WLAN_ASYNC_PING_REPORT: 4 bytes Packets sent,
 *                4 bytes Packets received, 4 bytes Min round time,
 *                4 bytes Max round time and 4 bytes for Avg round time.
 *
 *   sFWPatches            0 no patch or pointer to FW patches
 *   sDriverPatches        0 no patch or pointer to driver patches
 *   sBootLoaderPatches    0 no patch or pointer to bootloader patches
 *   sReadWlanInterruptPin init callback. the callback read wlan
 *                         interrupt status.
 *   sWlanInterruptEnable  init callback. the callback enable wlan
 *                         interrupt.
 *   sWlanInterruptDisable init callback. the callback disable wlan
 *                         interrupt.
 *   sWriteWlanPin         init callback. the callback write value
 *                         to device pin.
 *
 * Returned Value:
 *   None
 *
 * Description:
 *   Initialize wlan driver
 *
 * WARNING: This function must be called before ANY other wlan driver function
 *
 *****************************************************************************/

void wlan_init(size_t max_tx_len,
               tWlanCB sWlanCB, tFWPatches sFWPatches,
               tDriverPatches sDriverPatches,
               tBootLoaderPatches sBootLoaderPatches);

/*****************************************************************************
 * Name: wlan_start
 *
 * Input Parameters:
 *   usPatchesAvailableAtHost -  flag to indicate if patches available
 *                               from host or from EEPROM. Due to the
 *                               fact the patches are burn to the EEPROM
 *                               using the patch programmer utility, the
 *                               patches will be available from the EEPROM
 *                               and not from the host.
 *
 * Returned Value:
 *   None
 *
 * Description:
 *   Start WLAN device. This function asserts the enable pin of
 *                the device (WLAN_EN), starting the HW initialization process.
 *                The function blocked until device Initialization is completed.
 *                Function also configure patches (FW, driver or bootloader)
 *                and calls appropriate device callbacks.
 *
 *  NOTE: Prior calling the function wlan_init shall be called.
 *  WARNING: This function must be called after wlan_init and before any
 *    other wlan API
 *
 *****************************************************************************/

void wlan_start(uint16_t usPatchesAvailableAtHost);

/*****************************************************************************
 * Name: wlan_stop
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Description:
 *   Stop WLAN device by putting it into reset state.
 *
 *****************************************************************************/

void wlan_stop(void);

/*****************************************************************************
 * Name: wlan_connect
 *
 * Input Parameters:
 *   sec_type   security options:
 *                WLAN_SEC_UNSEC,
 *                WLAN_SEC_WEP (ASCII support only),
 *                WLAN_SEC_WPA or WLAN_SEC_WPA2
 *   ssid       up to 32 bytes and is ASCII SSID of the AP
 *   ssid_len   length of the SSID
 *   bssid      6 bytes specified the AP bssid
 *   key        up to 16 bytes specified the AP security key
 *   key_len    key length
 *
 * Returned Value:
 *   On success, zero is returned. On error, negative is returned.
 *   Note that even though a zero is returned on success to trigger
 *   connection operation, it does not mean that CCC3000 is already
 *   connected. An asynchronous "Connected" event is generated when
 *   actual association process finishes and CC3000 is connected to
 *   the AP. If DHCP is set, An asynchronous "DHCP" event is
 *   generated when DHCP process is finish.
 *
 * Description:
 *   Connect to AP
 *
 * WARNING: Please Note that when connection to AP configured with security
 *          type WEP, please confirm that the key is set as ASCII and not
 *          as HEX.
 *
 *****************************************************************************/

#ifndef CC3000_TINY_DRIVER
long wlan_connect(unsigned long ulSecType, char *ssid, long ssid_len,
                  uint8_t *bssid, uint8_t *key, long key_len);
#else
long wlan_connect(char *ssid, long ssid_len);
#endif

/*****************************************************************************
 * Name: wlan_disconnect
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 disconnected done, other CC3000 already disconnected
 *
 * Description:
 *   Disconnect connection from AP.
 *
 *****************************************************************************/

long wlan_disconnect(void);

/*****************************************************************************
 * Name: wlan_add_profile
 *
 * Input Parameters:
 *   ulSecType  WLAN_SEC_UNSEC,WLAN_SEC_WEP,WLAN_SEC_WPA,WLAN_SEC_WPA2
 *   ucSsid    ssid  SSID up to 32 bytes
 *   ulSsidLen ssid length
 *   ucBssid   bssid  6 bytes
 *   ulPriority ulPriority profile priority. Lowest priority:0.
 *   ulPairwiseCipher_Or_TxKeyLen  key length for WEP security
 *   ulGroupCipher_TxKeyIndex  key index
 *   ulKeyMgmt        KEY management
 *   ucPf_OrKey       security key
 *   ulPassPhraseLen  security key length for WPA\WPA2
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned
 *
 * Description:
 *   When auto start is enabled, the device connects to
 *             station from the profiles table. Up to 7 profiles are supported.
 *             If several profiles configured the device choose the highest
 *             priority profile, within each priority group, device will choose
 *             profile based on security policy, signal strength, etc
 *             parameters. All the profiles are stored in CC3000 NVMEM.
 *
 *****************************************************************************/

long wlan_add_profile(unsigned long ulSecType, uint8_t* ucSsid,
                      unsigned long ulSsidLen, uint8_t *ucBssid,
                      unsigned long ulPriority,
                      unsigned long ulPairwiseCipher_Or_TxKeyLen,
                      unsigned long ulGroupCipher_TxKeyIndex,
                      unsigned long ulKeyMgmt, uint8_t* ucPf_OrKey,
                      unsigned long ulPassPhraseLen);

/*****************************************************************************
 * Name: wlan_ioctl_del_profile
 *
 * Input Parameters:
 *   index   number of profile to delete
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned
 *
 * Description:
 *   Delete WLAN profile
 *
 *  @Note      In order to delete all stored profile, set index to 255.
 *
 *****************************************************************************/

long wlan_ioctl_del_profile(unsigned long ulIndex);

/*****************************************************************************
 * Name: wlan_set_event_mask
 *
 * Input Parameters:
 *   mask   mask option:
 *       HCI_EVNT_WLAN_UNSOL_CONNECT connect event
 *       HCI_EVNT_WLAN_UNSOL_DISCONNECT disconnect event
 *       HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE  smart config done
 *       HCI_EVNT_WLAN_UNSOL_INIT init done
 *       HCI_EVNT_WLAN_UNSOL_DHCP dhcp event report
 *       HCI_EVNT_WLAN_ASYNC_PING_REPORT ping report
 *       HCI_EVNT_WLAN_KEEPALIVE keepalive
 *       HCI_EVNT_WLAN_TX_COMPLETE - disable information on end of transmission
 *       Saved: no.
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned
 *
 * Description:
 *   Mask event according to bit mask. In case that event is
 *            masked (1), the device will not send the masked event to host.
 *
 *****************************************************************************/

long wlan_set_event_mask(unsigned long ulMask);

/*****************************************************************************
 * Name: wlan_ioctl_statusget
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   WLAN_STATUS_DISCONNECTED, WLAN_STATUS_SCANING,
 *             STATUS_CONNECTING or WLAN_STATUS_CONNECTED
 *
 * Description:
 *   get wlan status: disconnected, scanning, connecting or connected
 *
 *****************************************************************************/

long wlan_ioctl_statusget(void);

/*****************************************************************************
 * Name: wlan_ioctl_set_connection_policy
 *
 * Input Parameters:
 *   should_connect_to_open_ap  enable(1), disable(0) connect to any
 *            available AP. This parameter corresponds to the configuration of
 *            item # 3 in the brief description.
 *   should_use_fast_connect enable(1), disable(0). if enabled, tries
 *            to connect to the last connected AP. This parameter corresponds
 *            to the configuration of item # 1 in the brief description.
 *   auto_start enable(1), disable(0) auto connect
 *            after reset and periodically reconnect if needed. This
 *            configuration configures option 2 in the above description.
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned
 *
 * Description:
 *   When auto is enabled, the device tries to connect according
 *              the following policy:
 *   1) If fast connect is enabled and last connection is valid,
 *      the device will try to connect to it without the scanning
 *      procedure (fast). The last connection will be marked as
 *      invalid, due to adding/removing profile.
 *   2) If profile exists, the device will try to connect it
 *      (Up to seven profiles).
 *   3) If fast and profiles are not found, and open mode is
 *      enabled, the device will try to connect to any AP.
 *   * Note that the policy settings are stored in the CC3000 NVMEM.
 *
 *****************************************************************************/

long wlan_ioctl_set_connection_policy(unsigned long should_connect_to_open_ap,
                                      unsigned long ulShouldUseFastConnect,
                                      unsigned long ulUseProfiles);

/*****************************************************************************
 * Name: wlan_ioctl_get_scan_results
 *
 * Input Parameters:
 *   scan_timeout   parameter not supported
 *   ucResults  scan results (_wlan_full_scan_results_args_t)
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned
 *
 * Description:
 *   Gets entry from scan result table.
 *   The scan results are returned one by one, and each entry
 *   represents a single AP found in the area. The following is a
 *   format of the scan result:
 *     - 4 Bytes: number of networks found
 *     - 4 Bytes: The status of the scan: 0 - aged results,
 *       1 - results valid, 2 - no results
 *     - 42 bytes: Result entry, where the bytes are arranged as  follows:
 *       - 1 bit isValid - is result valid or not
 *       - 7 bits rssi - RSSI value;
 *       - 2 bits: securityMode - security mode of the AP:
 *         0 - Open, 1 - WEP, 2 WPA, 3 WPA2
 *       - 6 bits: SSID name length
 *       - 2 bytes: the time at which the entry has entered into
 *           scans result table
 *       - 32 bytes: SSID name
 *       - 6 bytes:  BSSID
 *
 *  NOTE: scan_timeout, is not supported on this version.
 *
 *****************************************************************************/

long wlan_ioctl_get_scan_results(unsigned long ulScanTimeout, uint8_t *ucResults);

/*****************************************************************************
 * Name: wlan_ioctl_set_scan_params
 *
 * Input Parameters:
 *   uiEnable - start/stop application scan:
 *            1 = start scan with default interval value of 10 min.
 *            in order to set a different scan interval value apply the value
 *            in milliseconds. minimum 1 second. 0=stop). Wlan reset
 *           (wlan_stop() wlan_start()) is needed when changing scan interval
 *            value. Saved: No
 *   uiMinDwellTime   minimum dwell time value to be used for each
 *           channel, in milliseconds. Saved: yes
 *           Recommended Value: 100 (Default: 20)
 *   uiMaxDwellTime    maximum dwell time value to be used for each
 *           channel, in milliseconds. Saved: yes
 *           Recommended Value: 100 (Default: 30)
 *   uiNumOfProbeRequests  max probe request between dwell time.
 *           Saved: yes. Recommended Value: 5 (Default:2)
 *   uiChannelMask  bitwise, up to 13 channels (0x1fff).
 *           Saved: yes. Default: 0x7ff
 *   uiRSSIThreshold   RSSI threshold. Saved: yes (Default: -80)
 *   uiSNRThreshold    NSR threshold. Saved: yes (Default: 0)
 *   uiDefaultTxPower  probe Tx power. Saved: yes (Default: 205)
 *   aiIntervalList    pointer to array with 16 entries (16 channels)
 *           each entry (unsigned long) holds timeout between periodic scan
 *           (connection scan) - in millisecond. Saved: yes. Default 2000ms.
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned
 *
 * Description:
 *   start and stop scan procedure. Set scan parameters.
 *
 *  @Note     uiDefaultTxPower, is not supported on this version.
 *
 *****************************************************************************/

long wlan_ioctl_set_scan_params(unsigned long uiEnable,
                                unsigned long uiMinDwellTime,
                                unsigned long uiMaxDwellTime,
                                unsigned long uiNumOfProbeRequests,
                                unsigned long uiChannelMask,long iRSSIThreshold,
                                unsigned long uiSNRThreshold,
                                unsigned long uiDefaultTxPower,
                                unsigned long *aiIntervalList);

/*****************************************************************************
 * Name: wlan_smart_config_start
 *
 * Input Parameters:
 *   algoEncryptedFlag indicates whether the information is encrypted
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned
 *
 * Description:
 *   Start to acquire device profile. The device acquire its own
 *           profile, if profile message is found. The acquired AP information
 *           is stored in CC3000 EEPROM only in case AES128 encryption is used.
 *           In case AES128 encryption is not used, a profile is created by
 *           CC3000 internally.
 *
 *  @Note    An asynchronous event - Smart Config Done will be generated as soon
 *           as the process finishes successfully.
 *
 *****************************************************************************/

long wlan_smart_config_start(unsigned long algoEncryptedFlag);

/*****************************************************************************
 * Name: wlan_smart_config_stop
 *
 * Input Parameters:
 *   algoEncryptedFlag indicates whether the information is encrypted
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned
 *
 * Description:
 *   Stop the acquire profile procedure
 *
 *****************************************************************************/

long wlan_smart_config_stop(void);

/*****************************************************************************
 * Name: wlan_smart_config_set_prefix
 *
 * Input Parameters:
 *   newPrefix  3 bytes identify the SSID prefix for the Smart Config.
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned
 *
 * Description:
 *   Configure station ssid prefix. The prefix is used internally
 *           in CC3000. It should always be TTT.
 *
 *  @Note    The prefix is stored in CC3000 NVMEM
 *
 *****************************************************************************/

long wlan_smart_config_set_prefix(char* cNewPrefix);

/*****************************************************************************
 * Name: wlan_smart_config_process
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned
 *
 * Description:
 *   process the acquired data and store it as a profile. The acquired
 *           AP information is stored in CC3000 EEPROM encrypted.
 *           The encrypted data is decrypted and stored as a profile.
 *           behavior is as defined by connection policy.
 *
 *****************************************************************************/

long wlan_smart_config_process(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif  // __INCLUDE_NUTTX_WIRELESS_CC3000_WLAN_H

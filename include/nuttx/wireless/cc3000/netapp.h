/*****************************************************************************
 *  netapp.h  - CC3000 Host Driver Implementation.
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

#ifndef _INCLUDE_NUTTX_WIRELESS_CC3000_NETAPP_H
#define  _INCLUDE_NUTTX_WIRELESS_CC3000_NETAPP_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

/*****************************************************************************
 * Public Types
 *****************************************************************************/

typedef struct _netapp_dhcp_ret_args_t
{
  uint8_t aucIP[4];
  uint8_t aucSubnetMask[4];
  uint8_t aucDefaultGateway[4];
  uint8_t aucDHCPServer[4];
  uint8_t aucDNSServer[4];
} tNetappDhcpParams;

typedef struct _netapp_ipconfig_ret_args_t
{
    uint8_t aucIP[4];
  uint8_t aucSubnetMask[4];
  uint8_t aucDefaultGateway[4];
  uint8_t aucDHCPServer[4];
  uint8_t aucDNSServer[4];
  uint8_t uaMacAddr[6];
  uint8_t uaSSID[32];
} tNetappIpconfigRetArgs;

/*Ping send report parameters*/

typedef struct _netapp_pingreport_args
{
  unsigned long packets_sent;
  unsigned long packets_received;
  unsigned long min_round_time;
  unsigned long max_round_time;
  unsigned long avg_round_time;
} netapp_pingreport_args_t;

/*****************************************************************************
 * Public Data
 *****************************************************************************/

#ifdef  __cplusplus
extern "C"
{
#endif

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

/******************************************************************************
 * Name: netapp_config_mac_adrress
 *
 * Description:
 *   Configure device MAC address and store it in NVMEM. The value of the MAC
 *   address configured through the API will be stored in CC3000 non volatile
 *   memory, thus preserved  over resets.
 *
 * Input Parameters:
 *   mac   device mac address, 6 bytes. Saved: yes
 *
 * Returned Value:
 *   Return on success 0, otherwise error.
 *
 *****************************************************************************/

long netapp_config_mac_adrress(uint8_t *mac);

/******************************************************************************
 * Name: netapp_dhcp
 *
 * Description:
 *   netapp_dhcp is used to configure the network interface,  static or
 *   dynamic (DHCP).\n In order to activate DHCP mode,  aucIP, aucSubnetMask,
 *   aucDefaultGateway must be 0. The default mode of CC3000 is DHCP mode. Note
 *   that the configuration is saved in non volatile memory and thus preserved
 *   over resets.
 *
 *   NOTE: If the mode is altered a reset of CC3000 device is required in order
 *   to apply changes.\nAlso note that asynchronous event of DHCP_EVENT, which
 *   is generated when an IP address is allocated either by the DHCP server or
 *   due to static allocation is generated only upon a connection to the AP was
 *   established.
 *
 * Input Parameters:
 *   aucIP               device mac address, 6 bytes. Saved: yes
 *   aucSubnetMask       device mac address, 6 bytes. Saved: yes
 *   aucDefaultGateway   device mac address, 6 bytes. Saved: yes
 *   aucDNSServer        device mac address, 6 bytes. Saved: yes
 *
 * Returned Value:
 *   Return on success 0, otherwise error.
 *
 *****************************************************************************/

long netapp_dhcp(unsigned long *aucIP, unsigned long *aucSubnetMask,
                 unsigned long *aucDefaultGateway, unsigned long *aucDNSServer);

/******************************************************************************
 * Name: netapp_timeout_values
 *
 * Description:
 *   Set new timeout values. Function set new timeout values for: DHCP lease
 *   timeout, ARP  refresh timeout, keepalive event timeout and socket
 *   inactivity timeout
 *
 *   NOTE: If a parameter set to non zero value which is less than 20s, it will
 *   be set automatically to 20s.
 *
 * Input Parameters:
 *   aucDHCP      DHCP lease time request, also impact the DHCP renew timeout.
 *                Range: [0-0xffffffff] seconds, 0 or 0xffffffff == infinity
 *                  lease timeout.
 *                Resolution: 10 seconds.
 *                Influence: only after reconnecting to the AP
 *                Minimal bound value: MIN_TIMER_VAL_SECONDS - 20 seconds.
 *                The parameter is saved into the CC3000 NVMEM. The default
 *                  value on CC3000 is 14400 seconds.
 *   aucARP       ARP refresh timeout, if ARP entry is not updated by incoming
 *                  packet, the ARP entry will be  deleted by the end of the
 *                  timeout.
 *                Range: [0-0xffffffff] seconds, 0 == infinity ARP timeout
 *                Resolution: 10 seconds.
 *                Influence: on runtime.
 *                Minimal bound value: MIN_TIMER_VAL_SECONDS - 20 seconds.
 *                The parameter is saved into the CC3000 NVMEM. The default
 *                  value on CC3000 is 3600 seconds.
 *  aucKeepalive  Keepalive event sent by the end of keepalive timeout
 *                Range: [0-0xffffffff] seconds, 0 == infinity timeout
 *                Resolution: 10 seconds.
 *                Influence: on runtime.
 *                Minimal bound value: MIN_TIMER_VAL_SECONDS - 20 sec
 *                The parameter is saved into the CC3000 NVMEM. The default
 *                  value on CC3000 is 10 seconds.
 *  aucInactivity Socket inactivity timeout, socket timeout is refreshed by
 *                  incoming or outgoing packet, by the end of the socket
 *                  timeout the socket will be closed
 *                Range: [0-0xffffffff] sec, 0 == infinity timeout.
 *                Resolution: 10 seconds. Influence: on runtime.
 *                Minimal bound value: MIN_TIMER_VAL_SECONDS - 20 sec
 *                The parameter is saved into the CC3000 NVMEM. The default
 *                  value on CC3000 is 60 seconds.
 *
 * Returned Value:
 *   Return on success 0, otherwise error.
 *
 *****************************************************************************/

#ifndef CC3000_TINY_DRIVER
long netapp_timeout_values(unsigned long *aucDHCP, unsigned long *aucARP,
                           unsigned long *aucKeepalive,
                           unsigned long *aucInactivity);
#endif

/******************************************************************************
 * Name: netapp_ping_send
 *
 * Description:
 *   Send ICMP ECHO_REQUEST to network hosts
 *
 *   NOTE: If an operation finished successfully asynchronous ping report event
 *   will be generated. The report structure is as defined by structure
 *   netapp_pingreport_args_t.
 *
 *   WARNING: Calling this function while a previous Ping Requests are in
 *   progress will stop the previous ping request.
 *
 * Input Parameters:
 *   ip              destination IP address
 *   pingAttempts    number of echo requests to send
 *   pingSize        send buffer size which may be up to 1400 bytes
 *   pingTimeout     Time to wait for a response,in milliseconds.
 *
 * Returned Value:
 *   Return on success 0, otherwise error.
 *
 *****************************************************************************/

#ifndef CC3000_TINY_DRIVER
long netapp_ping_send(unsigned long *ip, unsigned long ulPingAttempts,
                      unsigned long ulPingSize, unsigned long ulPingTimeout);
#endif

/******************************************************************************
 * Name: netapp_ping_stop
 *
 * Description:
 *   Stop any ping request.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned.
 *
 *****************************************************************************/

#ifndef CC3000_TINY_DRIVER
long netapp_ping_stop(void);
#endif

/******************************************************************************
 * Name: netapp_ping_report
 *
 * Description:
 *   Request for ping status. This API triggers the CC3000 to send asynchronous
 *   events: HCI_EVNT_WLAN_ASYNC_PING_REPORT. This event will carry the report
 *   structure: netapp_pingreport_args_t. This structure is filled in with ping
 *   results up till point of triggering API.
 *
 *   netapp_pingreport_args_t:
 *     packets_sent - echo sent,
 *     packets_received - echo reply
 *     min_round_time - minimum round time,
 *     max_round_time - max round time,
 *     avg_round_time - average round time
 *
 *   NOTE: When a ping operation is not active, the returned structure fields
 *   are 0.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

#ifndef CC3000_TINY_DRIVER
void netapp_ping_report(void);
#endif


/******************************************************************************
 * Name: netapp_ipconfig
 *
 * Description:
 *   Obtain the CC3000 Network interface information. Note that the information
 *   is available only after the WLAN connection was established. Calling this
 *   function before associated, will cause non-defined values to be returned.
 *
 *   NOTE: The function is useful for figuring out the IP Configuration of the
 *   device when DHCP is used and for figuring out the SSID of the Wireless
 *   network the device is associated with.
 *
 * Input Parameters:
 *   ipconfig  This argument is a pointer to a tNetappIpconfigRetArgs structure.
 *   This structure is filled in with the network interface configuration.
 *
 *   tNetappIpconfigRetArgs:
 *     aucIP - ip address,
 *     aucSubnetMask - mask,
 *     aucDefaultGateway - default gateway address
 *     aucDHCPServer - dhcp server address
 *     aucDNSServer - dns server address
 *     uaMacAddr - mac
 *     address, uaSSID - connected AP ssid
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

void netapp_ipconfig(tNetappIpconfigRetArgs * ipconfig);

/******************************************************************************
 * Name: netapp_arp_flush
 *
 * Description:
 *   Flushes ARP table
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

#ifndef CC3000_TINY_DRIVER
long netapp_arp_flush(void);
#endif

/******************************************************************************
 * Name: netapp_set_debug_level
 *
 * Description:
 *   Debug messages sent via the UART debug channel, this function enable/disable
 *   the debug level
 *
 * Input Parameters:
 *   level    debug level. Bitwise [0-8], 0(disable)or 1(enable).
 *            Bitwise map:
 *              0 - Critical message
 *              1 - information message
 *              2 - core messages
 *              3 - HCI messages
 *              4 - Network stack messages
 *              5 - wlan messages
 *              6 - wlan driver messages
 *              7 - epprom messages,
 *              8 - general messages.
 *            Default: 0x13f. Saved: no
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned
 *
 *****************************************************************************/

#ifndef CC3000_TINY_DRIVER
long netapp_set_debug_level(unsigned long ulLevel);
#endif

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif  // _INCLUDE_NUTTX_WIRELESS_CC3000_NETAPP_H

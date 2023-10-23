========
Commands
========

This page shows ``wapi`` commands, their arguments and outputs. For a
complete list of ``wapi`` commands available to the system, just run
``wapi``::

    nsh> wapi
    Usage:
            wapi show         <ifname>
            wapi scan         <ifname>
            wapi scan_results <ifname>
            wapi ip           <ifname> <IP address>
            wapi mask         <ifname> <mask>
            wapi freq         <ifname> <frequency>  <index/flag>
            wapi essid        <ifname> <essid>      <index/flag>
            wapi psk          <ifname> <passphrase> <index/flag> <wpa>
            wapi disconnect   <ifname>
            wapi mode         <ifname>              <index/mode>
            wapi ap           <ifname>              <MAC address>
            wapi bitrate      <ifname> <bitrate>    <index/flag>
            wapi txpower      <ifname> <txpower>    <index/flag>
            wapi country      <ifname> <country code>
            wapi sense        <ifname>
            wapi pta_prio     <ifname>  <index/flag>
            wapi help

    Frequency Flags:
            [0] WAPI_FREQ_AUTO
            [1] WAPI_FREQ_FIXED

    ESSID Flags:
            [0] WAPI_ESSID_OFF
            [1] WAPI_ESSID_ON

    Passphrase algorithm Flags:
            [0] WPA_ALG_NONE
            [1] WPA_ALG_WEP
            [2] WPA_ALG_TKIP
            [3] WPA_ALG_CCMP

    Passphrase WPA version:
            [0] WPA_VER_NONE
            [1] WPA_VER_1
            [2] WPA_VER_2
            [3] WPA_VER_3

    Operating Modes:
            [0] WAPI_MODE_AUTO
            [1] WAPI_MODE_ADHOC
            [2] WAPI_MODE_MANAGED
            [3] WAPI_MODE_MASTER
            [4] WAPI_MODE_REPEAT
            [5] WAPI_MODE_SECOND
            [6] WAPI_MODE_MONITOR
            [7] WAPI_MODE_MESH

    Bitrate Flags:
            [0] WAPI_BITRATE_AUTO
            [1] WAPI_BITRATE_FIXED

    TX power Flags:
            [0] WAPI_TXPOWER_DBM
            [1] WAPI_TXPOWER_MWATT
            [2] WAPI_TXPOWER_RELATIVE

    pta prio Flags:
            [0] WAPI_PTA_PRIORITY_COEX_MAXIMIZED
            [1] WAPI_PTA_PRIORITY_COEX_HIGH
            [2] WAPI_PTA_PRIORITY_BALANCED
            [3] WAPI_PTA_PRIORITY_WLAN_HIGHD
            [4] WAPI_PTA_PRIORITY_WLAN_MAXIMIZED

Arguments
=========

Command's arguments are available on ``wapi``'s usage helper.

.. note:: ``<>`` means a required argument and ``[]`` an optional one.

A short explanation of them follows:

``<ifname>``
------------

The interface name is arch-dependent and it's usually set for a specific
operating mode. For instance, ``wlan0`` would be an interface used for STA
mode and ``wlan1`` for SoftAP.

Please refer to the :doc:`Supported Platforms </platforms/index>` for
platform-specific definitions. As an example, please check 
:ref:`ESP32 Wi-Fi Station Mode <esp32_wi-fi_sta>` and
:ref:`ESP32 Wi-Fi SoftAP Mode <esp32_wi-fi_softap>` Wi-Fi sections.

``<index/flag>``
----------------

The ``<index/flag>`` can be used as a numerical or textual value. For
instance, considering the ``wapi psk`` command, one could use indistinctly::
    
    nsh> wapi psk wlan0 mypasswd 3
    nsh> wapi psk wlan0 mypasswd WPA_ALG_CCMP

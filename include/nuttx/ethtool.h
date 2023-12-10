/****************************************************************************
 * include/nuttx/ethtool.h
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

#ifndef __INCLUDE_NUTTX_ETHTOOL_H
#define __INCLUDE_NUTTX_ETHTOOL_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CMDs currently supported */
#define ETHTOOL_GSET          0x00000001 /* DEPRECATED, Get settings.        \
                                          * Please use ETHTOOL_GLINKSETTINGS \
                                          **/
#define ETHTOOL_SSET          0x00000002 /* DEPRECATED, Set settings.        \
                                          * Please use ETHTOOL_SLINKSETTINGS \
                                          **/
#define ETHTOOL_GDRVINFO      0x00000003 /* Get driver info. */
#define ETHTOOL_GREGS         0x00000004 /* Get NIC registers. */
#define ETHTOOL_GWOL          0x00000005 /* Get wake-on-lan options. */
#define ETHTOOL_SWOL          0x00000006 /* Set wake-on-lan options. */
#define ETHTOOL_GMSGLVL       0x00000007 /* Get driver message level */
#define ETHTOOL_SMSGLVL       0x00000008 /* Set driver msg level. */
#define ETHTOOL_NWAY_RST      0x00000009 /* Restart autonegotiation. */

/* Get link status for host, i.e. whether the interface *and* the
 * physical port (if there is one) are up (ethtool_value).
 **/

#define ETHTOOL_GLINK         0x0000000a
#define ETHTOOL_GEEPROM       0x0000000b /* Get EEPROM data */
#define ETHTOOL_SEEPROM       0x0000000c /* Set EEPROM data. */
#define ETHTOOL_GCOALESCE     0x0000000e /* Get coalesce config */
#define ETHTOOL_SCOALESCE     0x0000000f /* Set coalesce config. */
#define ETHTOOL_GRINGPARAM    0x00000010 /* Get ring parameters */
#define ETHTOOL_SRINGPARAM    0x00000011 /* Set ring parameters. */
#define ETHTOOL_GPAUSEPARAM   0x00000012 /* Get pause parameters */
#define ETHTOOL_SPAUSEPARAM   0x00000013 /* Set pause parameters. */
#define ETHTOOL_GRXCSUM       0x00000014 /* Get RX hw csum enable (ethtool_value) */
#define ETHTOOL_SRXCSUM       0x00000015 /* Set RX hw csum enable (ethtool_value) */
#define ETHTOOL_GTXCSUM       0x00000016 /* Get TX hw csum enable (ethtool_value) */
#define ETHTOOL_STXCSUM       0x00000017 /* Set TX hw csum enable (ethtool_value) */
#define ETHTOOL_GSG           0x00000018 /* Get scatter-gather enable \
                                          * (ethtool_value) \
                                          **/
#define ETHTOOL_SSG           0x00000019 /* Set scatter-gather enable \
                                          * (ethtool_value). \
                                          **/
#define ETHTOOL_TEST          0x0000001a /* execute NIC self-test. */
#define ETHTOOL_GSTRINGS      0x0000001b /* get specified string set */
#define ETHTOOL_PHYS_ID       0x0000001c /* identify the NIC */
#define ETHTOOL_GSTATS        0x0000001d /* get NIC-specific statistics */
#define ETHTOOL_GTSO          0x0000001e /* Get TSO enable (ethtool_value) */
#define ETHTOOL_STSO          0x0000001f /* Set TSO enable (ethtool_value) */
#define ETHTOOL_GPERMADDR     0x00000020 /* Get permanent hardware address */
#define ETHTOOL_GUFO          0x00000021 /* Get UFO enable (ethtool_value) */
#define ETHTOOL_SUFO          0x00000022 /* Set UFO enable (ethtool_value) */
#define ETHTOOL_GGSO          0x00000023 /* Get GSO enable (ethtool_value) */
#define ETHTOOL_SGSO          0x00000024 /* Set GSO enable (ethtool_value) */
#define ETHTOOL_GFLAGS        0x00000025 /* Get flags bitmap(ethtool_value) */
#define ETHTOOL_SFLAGS        0x00000026 /* Set flags bitmap(ethtool_value) */
#define ETHTOOL_GPFLAGS       0x00000027 /* Get driver-private flags bitmap */
#define ETHTOOL_SPFLAGS       0x00000028 /* Set driver-private flags bitmap */
#define ETHTOOL_GRXFH         0x00000029 /* Get RX flow hash configuration */
#define ETHTOOL_SRXFH         0x0000002a /* Set RX flow hash configuration */
#define ETHTOOL_GGRO          0x0000002b /* Get GRO enable (ethtool_value) */
#define ETHTOOL_SGRO          0x0000002c /* Set GRO enable (ethtool_value) */
#define ETHTOOL_GRXRINGS      0x0000002d /* Get RX rings available for LB */
#define ETHTOOL_GRXCLSRLCNT   0x0000002e /* Get RX class rule count */
#define ETHTOOL_GRXCLSRULE    0x0000002f /* Get RX classification rule */
#define ETHTOOL_GRXCLSRLALL   0x00000030 /* Get all RX classification rule */
#define ETHTOOL_SRXCLSRLDEL   0x00000031 /* Delete RX classification rule */
#define ETHTOOL_SRXCLSRLINS   0x00000032 /* Insert RX classification rule */
#define ETHTOOL_FLASHDEV      0x00000033 /* Flash firmware to device */
#define ETHTOOL_RESET         0x00000034 /* Reset hardware */
#define ETHTOOL_SRXNTUPLE     0x00000035 /* Add an n-tuple filter to device */
#define ETHTOOL_GRXNTUPLE     0x00000036 /* deprecated */
#define ETHTOOL_GSSET_INFO    0x00000037 /* Get string set info */
#define ETHTOOL_GRXFHINDIR    0x00000038 /* Get RX flow hash indir'n table */
#define ETHTOOL_SRXFHINDIR    0x00000039 /* Set RX flow hash indir'n table */
#define ETHTOOL_GFEATURES     0x0000003a /* Get device offload settings */
#define ETHTOOL_SFEATURES     0x0000003b /* Change device offload settings */
#define ETHTOOL_GCHANNELS     0x0000003c /* Get no of channels */
#define ETHTOOL_SCHANNELS     0x0000003d /* Set no of channels */
#define ETHTOOL_SET_DUMP      0x0000003e /* Set dump settings */
#define ETHTOOL_GET_DUMP_FLAG 0x0000003f /* Get dump settings */
#define ETHTOOL_GET_DUMP_DATA 0x00000040 /* Get dump data */
#define ETHTOOL_GET_TS_INFO   0x00000041 /* Get time stamping and PHC info */
#define ETHTOOL_GMODULEINFO   0x00000042 /* Get plug-in module information */
#define ETHTOOL_GMODULEEEPROM 0x00000043 /* Get plug-in module eeprom */
#define ETHTOOL_GEEE          0x00000044 /* Get EEE settings */
#define ETHTOOL_SEEE          0x00000045 /* Set EEE settings */
#define ETHTOOL_GRSSH         0x00000046 /* Get RX flow hash configuration */
#define ETHTOOL_SRSSH         0x00000047 /* Set RX flow hash configuration */
#define ETHTOOL_GTUNABLE      0x00000048 /* Get tunable configuration */
#define ETHTOOL_STUNABLE      0x00000049 /* Set tunable configuration */
#define ETHTOOL_GPHYSTATS     0x0000004a /* get PHY-specific statistics */
#define ETHTOOL_PERQUEUE      0x0000004b /* Set per queue options */
#define ETHTOOL_GLINKSETTINGS 0x0000004c /* Get ethtool_link_settings */
#define ETHTOOL_SLINKSETTINGS 0x0000004d /* Set ethtool_link_settings */
#define ETHTOOL_PHY_GTUNABLE  0x0000004e /* Get PHY tunable configuration */
#define ETHTOOL_PHY_STUNABLE  0x0000004f /* Set PHY tunable configuration */
#define ETHTOOL_GFECPARAM     0x00000050 /* Get FEC settings */
#define ETHTOOL_SFECPARAM     0x00000051 /* Set FEC settings */

/* Duplex, half or full. */

#define DUPLEX_HALF           0x00
#define DUPLEX_FULL           0x01
#define DUPLEX_UNKNOWN        0xff

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* struct ethtool_cmd - DEPRECATED, link control and status
 * This structure is DEPRECATED, please use struct ethtool_link_settings.
 * cmd: Command number = ETHTOOL_GSET or ETHTOOL_SSET
 * supported: Bitmask of SUPPORTED_* flags for the link modes,
 *  physical connectors and other link features for which the
 *  interface supports autonegotiation or auto-detection.
 *  Read-only.
 * advertising: Bitmask of ADVERTISED_* flags for the link modes,
 *  physical connectors and other link features that are
 *  advertised through autonegotiation or enabled for
 *  auto-detection.
 * speed: Low bits of the speed, 1Mb units, 0 to INT_MAX or SPEED_UNKNOWN
 * duplex: Duplex mode; one of DUPLEX_*
 * port: Physical connector type; one of PORT_*
 * phy_address: MDIO address of PHY (transceiver); 0 or 255 if not
 *  applicable.  For clause 45 PHYs this is the PRTAD.
 * transceiver: Historically used to distinguish different possible
 *  PHY types, but not in a consistent way.  Deprecated.
 * autoneg: Enable/disable autonegotiation and auto-detection;
 *  either AUTONEG_DISABLE or AUTONEG_ENABLE
 * mdio_support: Bitmask of ETH_MDIO_SUPPORTS_* flags for the MDIO
 *  protocols supported by the interface; 0 if unknown.
 *  Read-only.
 * maxtxpkt: Historically used to report TX IRQ coalescing; now
 *  obsoleted by struct ethtool_coalesce.  Read-only; deprecated.
 * maxrxpkt: Historically used to report RX IRQ coalescing; now
 *  obsoleted by struct ethtool_coalesce.  Read-only; deprecated.
 * speed_hi: High bits of the speed, 1Mb units, 0 to INT_MAX or
 *  SPEED_UNKNOWN
 * eth_tp_mdix: Ethernet twisted-pair MDI(-X) status; one of
 *  ETH_TP_MDI_*.  If the status is unknown or not applicable, the
 *  value will be ETH_TP_MDI_INVALID.  Read-only.
 * eth_tp_mdix_ctrl: Ethernet twisted pair MDI(-X) control; one of
 *  ETH_TP_MDI_*.  If MDI(-X) control is not implemented, reads
 *  yield ETH_TP_MDI_INVALID and writes may be ignored or rejected.
 *  When written successfully, the link should be renegotiated if
 *  necessary.
 * lp_advertising: Bitmask of ADVERTISED_* flags for the link modes
 *  and other link features that the link partner advertised
 *  through autonegotiation; 0 if unknown or not applicable.
 *  Read-only.
 *
 * The link speed in Mbps is split between speed and speed_hi.  Use
 * the ethtool_cmd_speed() and ethtool_cmd_speed_set() functions to
 * access it.
 *
 * If autonegotiation is disabled, the speed and duplex represent the
 * fixed link mode and are writable if the driver supports multiple
 * link modes.  If it is enabled then they are read-only; if the link
 * is up they represent the negotiated link mode; if the link is down,
 * the speed is 0, SPEED_UNKNOWN or the highest enabled speed and
 * duplex is DUPLEX_UNKNOWN or the best enabled duplex mode.
 *
 * Some hardware interfaces may have multiple PHYs and/or physical
 * connectors fitted or do not allow the driver to detect which are
 * fitted.  For these interfaces port and/or phy_address may be
 * writable, possibly dependent on autoneg being AUTONEG_DISABLE.
 * Otherwise, attempts to write different values may be ignored or
 * rejected.
 *
 * Users should assume that all fields not marked read-only are
 * writable and subject to validation by the driver.  They should use
 * ETHTOOL_GSET to get the current values before making specific
 * changes and then applying them with ETHTOOL_SSET.
 *
 * Deprecated fields should be ignored by both users and drivers.
 **/

struct ethtool_cmd
{
  uint32_t cmd;
  uint32_t supported;
  uint32_t advertising;
  uint16_t speed;
  uint8_t duplex;
  uint8_t port;
  uint8_t phy_address;
  uint8_t transceiver;
  uint8_t autoneg;
  uint8_t mdio_support;
  uint32_t maxtxpkt;
  uint32_t maxrxpkt;
  uint16_t speed_hi;
  uint8_t eth_tp_mdix;
  uint8_t eth_tp_mdix_ctrl;
  uint32_t lp_advertising;
  uint32_t reserved[2];
};

#endif

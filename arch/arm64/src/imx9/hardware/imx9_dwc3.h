/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx9_dwc3.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_DWC3_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_DWC3_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* i.MX95 DWC3 Base Addresses */

#define IMX9_DWC3_BASE              (0x4c100000ul)
#define IMX9_DWC3_GLUE_BASE         (0x4c1f0000ul)
#define IMX9_DWC3_PHY_BASE          (0x4c1f0040ul)

/* USB3 PHY Registers (offset from PHY_BASE) */

#define IMX9_USB3_PHY_CTRL0                 0x00
#define IMX9_USB3_PHY_CTRL1                 0x04
#define IMX9_USB3_PHY_CTRL2                 0x08
#define IMX9_USB3_PHY_CTRL3                 0x0c
#define IMX9_USB3_PHY_CTRL4                 0x10
#define IMX9_USB3_PHY_CTRL5                 0x14
#define IMX9_USB3_PHY_CTRL6                 0x18

/* PHY_CTRL0 bits */

#define IMX9_USB3_PHY_CTRL0_REF_SSP_EN      (1 << 2)
#define IMX9_USB3_PHY_CTRL0_FSEL_SHIFT       5
#define IMX9_USB3_PHY_CTRL0_FSEL_MASK        (0x3f << 5)
#define IMX9_USB3_PHY_CTRL0_FSEL_24M         (0x2a << 5)
#define IMX9_USB3_PHY_CTRL0_FSEL_100M        (0x27 << 5)

/* PHY_CTRL1 bits */

#define IMX9_USB3_PHY_CTRL1_RESET            (1 << 0)
#define IMX9_USB3_PHY_CTRL1_ATERESET         (1 << 3)
#define IMX9_USB3_PHY_CTRL1_VDATSRCENB0      (1 << 19)
#define IMX9_USB3_PHY_CTRL1_VDATDETENB0      (1 << 20)

/* PHY_CTRL2 bits */

#define IMX9_USB3_PHY_CTRL2_TXENABLEN0       (1 << 8)
#define IMX9_USB3_PHY_CTRL2_OTG_DISABLE      (1 << 9)

/* PHY_CTRL6 bits */

#define IMX9_USB3_PHY_CTRL6_ALT_CLK_EN       (1 << 1)
#define IMX9_USB3_PHY_CTRL6_ALT_CLK_SEL      (1 << 0)
#define IMX9_USB3_PHY_CTRL6_RXTERM_OVERRIDE_SEL (1 << 29)

/* i.MX Glue Layer Registers (offset from GLUE_BASE) */

#define IMX9_DWC3_GLUE_CTRL0       0x00
#define IMX9_DWC3_GLUE_CTRL1       0x04

#define IMX9_DWC3_GLUE_CTRL0_PORTPWR_EN   (1 << 12)
#define IMX9_DWC3_GLUE_CTRL0_USB3_FIXED   (1 << 22)
#define IMX9_DWC3_GLUE_CTRL0_USB2_FIXED   (1 << 23)

#define IMX9_DWC3_GLUE_CTRL1_OC_POL       (1 << 16)
#define IMX9_DWC3_GLUE_CTRL1_PWR_POL      (1 << 17)

/* DWC3 Register Space Boundaries (offset from DWC3_BASE) */

#define DWC3_GLOBALS_REGS_START     0xc100
#define DWC3_DEVICE_REGS_START      0xc700
#define DWC3_OTG_REGS_START         0xcc00

/* Global Registers */

#define DWC3_GSBUSCFG0              0xc100
#define DWC3_GSBUSCFG1              0xc104
#define DWC3_GTXTHRCFG              0xc108
#define DWC3_GRXTHRCFG              0xc10c
#define DWC3_GCTL                   0xc110
#define DWC3_GEVTEN                 0xc114
#define DWC3_GSTS                   0xc118
#define DWC3_GUCTL1                 0xc11c
#define DWC3_GSNPSID                0xc120
#define DWC3_GUID                   0xc128
#define DWC3_GUCTL                  0xc12c
#define DWC3_GHWPARAMS0             0xc140
#define DWC3_GHWPARAMS1             0xc144
#define DWC3_GHWPARAMS2             0xc148
#define DWC3_GHWPARAMS3             0xc14c
#define DWC3_GHWPARAMS4             0xc150
#define DWC3_GHWPARAMS5             0xc154
#define DWC3_GHWPARAMS6             0xc158
#define DWC3_GHWPARAMS7             0xc15c
#define DWC3_GHWPARAMS8             0xc600
#define DWC3_GFLADJ                 0xc630

#define DWC3_GUSB2PHYCFG(n)         (0xc200 + ((n) * 0x04))
#define DWC3_GUSB3PIPECTL(n)        (0xc2c0 + ((n) * 0x04))
#define DWC3_GTXFIFOSIZ(n)          (0xc300 + ((n) * 0x04))
#define DWC3_GRXFIFOSIZ(n)          (0xc380 + ((n) * 0x04))

#define DWC3_GEVNTADRLO(n)          (0xc400 + ((n) * 0x10))
#define DWC3_GEVNTADRHI(n)          (0xc404 + ((n) * 0x10))
#define DWC3_GEVNTSIZ(n)            (0xc408 + ((n) * 0x10))
#define DWC3_GEVNTCOUNT(n)          (0xc40c + ((n) * 0x10))

/* Device Registers */

#define DWC3_DCFG                   0xc700
#define DWC3_DCTL                   0xc704
#define DWC3_DEVTEN                 0xc708
#define DWC3_DSTS                   0xc70c
#define DWC3_DGCMDPAR               0xc710
#define DWC3_DGCMD                  0xc714
#define DWC3_DALEPENA               0xc720

/* Device Endpoint Registers (per-endpoint, n = 0..31) */

#define DWC3_DEP_BASE(n)            (0xc800 + ((n) * 0x10))
#define DWC3_DEPCMDPAR2(n)          (DWC3_DEP_BASE(n) + 0x00)
#define DWC3_DEPCMDPAR1(n)          (DWC3_DEP_BASE(n) + 0x04)
#define DWC3_DEPCMDPAR0(n)          (DWC3_DEP_BASE(n) + 0x08)
#define DWC3_DEPCMD(n)              (DWC3_DEP_BASE(n) + 0x0c)

/* GCTL - Global Control Register */

#define DWC3_GCTL_PWRDNSCALE_SHIFT  19
#define DWC3_GCTL_PWRDNSCALE_MASK   (0x1fffu << 19)
#define DWC3_GCTL_PWRDNSCALE(n)     ((n) << 19)
#define DWC3_GCTL_U2RSTECN          (1 << 16)
#define DWC3_GCTL_PRTCAPDIR_SHIFT   12
#define DWC3_GCTL_PRTCAPDIR_MASK    (3 << 12)
#define DWC3_GCTL_PRTCAPDIR(n)      ((n) << 12)
#define DWC3_GCTL_PRTCAP_HOST       1
#define DWC3_GCTL_PRTCAP_DEVICE     2
#define DWC3_GCTL_PRTCAP_OTG        3
#define DWC3_GCTL_CORESOFTRESET     (1 << 11)
#define DWC3_GCTL_SOFITPSYNC        (1 << 10)
#define DWC3_GCTL_SCALEDOWN_SHIFT   4
#define DWC3_GCTL_SCALEDOWN_MASK    (3 << 4)
#define DWC3_GCTL_DISSCRAMBLE       (1 << 3)
#define DWC3_GCTL_U2EXIT_LFPS       (1 << 2)
#define DWC3_GCTL_DSBLCLKGTNG       (1 << 0)

/* GSTS - Global Status Register */

#define DWC3_GSTS_CURMOD_MASK       0x3
#define DWC3_GSTS_CURMOD_DEVICE     0
#define DWC3_GSTS_CURMOD_HOST       1

/* GUSB2PHYCFG - Global USB2 PHY Configuration */

#define DWC3_GUSB2PHYCFG_PHYSOFTRST     (1u << 31)
#define DWC3_GUSB2PHYCFG_U2_FREECLK     (1 << 30)
#define DWC3_GUSB2PHYCFG_SUSPHY         (1 << 6)
#define DWC3_GUSB2PHYCFG_PHYIF          (1 << 3)
#define DWC3_GUSB2PHYCFG_ENBLSLPM       (1 << 8)
#define DWC3_GUSB2PHYCFG_USBTRDTIM_SHIFT 10
#define DWC3_GUSB2PHYCFG_USBTRDTIM_MASK (0xf << 10)
#define DWC3_GUSB2PHYCFG_USBTRDTIM(n)   ((n) << 10)

/* GUCTL1 - Global User Control Register 1 */

#define DWC3_GUCTL1_PARKMODE_DISABLE_SS  (1 << 17)
#define DWC3_GUCTL1_PARKMODE_DISABLE_HS  (1 << 16)
#define DWC3_GUCTL1_DEV_DECOUPLE_L1L2_EVT (1u << 31)
#define DWC3_GUCTL1_DEV_FORCE_20_CLK_FOR_30_CLK (1 << 26)

/* GFLADJ - Global Frame Length Adjustment Register */

#define DWC3_GFLADJ_REFCLK_LPM_SEL      (1 << 23)
#define DWC3_GFLADJ_30MHZ_MASK           (0x3f << 0)
#define DWC3_GFLADJ_30MHZ_SDBND_SEL     (1 << 7)

/* GUSB3PIPECTL - Global USB3 PIPE Control */

#define DWC3_GUSB3PIPECTL_PHYSOFTRST    (1u << 31)
#define DWC3_GUSB3PIPECTL_DISRXDETINP3  (1 << 28)
#define DWC3_GUSB3PIPECTL_DELAYP1TRANS  (1 << 18)
#define DWC3_GUSB3PIPECTL_SUSPHY        (1 << 17)
#define DWC3_GUSB3PIPECTL_LFPSFILT      (1 << 9)
#define DWC3_GUSB3PIPECTL_RX_DETOPOLL   (1 << 8)

/* GEVNTSIZ - Global Event Buffer Size */

#define DWC3_GEVNTSIZ_INTMASK           (1u << 31)
#define DWC3_GEVNTSIZ_SIZE_MASK         0xffff

/* GEVNTCOUNT - Global Event Count */

#define DWC3_GEVNTCOUNT_MASK            0xfffc
#define DWC3_GEVNTCOUNT_EHB             (1u << 31)

/* DCFG - Device Configuration Register */

#define DWC3_DCFG_DEVADDR_SHIFT     3
#define DWC3_DCFG_DEVADDR_MASK      (0x7f << 3)
#define DWC3_DCFG_DEVADDR(a)        ((a) << 3)
#define DWC3_DCFG_SPEED_MASK        0x7
#define DWC3_DCFG_SUPERSPEED        (4 << 0)
#define DWC3_DCFG_HIGHSPEED         (0 << 0)
#define DWC3_DCFG_FULLSPEED         (1 << 0)
#define DWC3_DCFG_NUMP_SHIFT        17
#define DWC3_DCFG_NUMP_MASK         (0x1f << 17)
#define DWC3_DCFG_NUMP(n)           ((n) << 17)
#define DWC3_DCFG_LPM_CAP           (1 << 22)

/* DCTL - Device Control Register */

#define DWC3_DCTL_RUN_STOP          (1u << 31)
#define DWC3_DCTL_CSFTRST           (1 << 30)
#define DWC3_DCTL_LSFTRST           (1 << 29)
#define DWC3_DCTL_HIRD_THRES_SHIFT  24
#define DWC3_DCTL_HIRD_THRES_MASK   (0x1f << 24)
#define DWC3_DCTL_KEEP_CONNECT      (1 << 19)
#define DWC3_DCTL_INITU2ENA         (1 << 12)
#define DWC3_DCTL_ACCEPTU2ENA       (1 << 11)
#define DWC3_DCTL_INITU1ENA         (1 << 10)
#define DWC3_DCTL_ACCEPTU1ENA       (1 << 9)
#define DWC3_DCTL_ULSTCHNGREQ_SHIFT 5
#define DWC3_DCTL_ULSTCHNGREQ_MASK  (0x0f << 5)
#define DWC3_DCTL_TSTCTRL_MASK      (0xf << 1)

/* DEVTEN - Device Event Enable Register */

#define DWC3_DEVTEN_VNDRDEVTSTRCVEDEN   (1 << 12)
#define DWC3_DEVTEN_EVNTOVERFLOWEN      (1 << 11)
#define DWC3_DEVTEN_CMDCMPLTEN          (1 << 10)
#define DWC3_DEVTEN_ERRTICERREN         (1 << 9)
#define DWC3_DEVTEN_SOFEN               (1 << 7)
#define DWC3_DEVTEN_U3L2L1SUSPEN        (1 << 6)
#define DWC3_DEVTEN_WKUPEVTEN           (1 << 4)
#define DWC3_DEVTEN_ULSTCNGEN           (1 << 3)
#define DWC3_DEVTEN_CONNECTDONEEN       (1 << 2)
#define DWC3_DEVTEN_USBRSTEN            (1 << 1)
#define DWC3_DEVTEN_DISCONNEVTEN        (1 << 0)

/* DSTS - Device Status Register */

#define DWC3_DSTS_DCNRD             (1 << 29)
#define DWC3_DSTS_COREIDLE          (1 << 23)
#define DWC3_DSTS_DEVCTRLHLT        (1 << 22)
#define DWC3_DSTS_USBLNKST_SHIFT    18
#define DWC3_DSTS_USBLNKST_MASK     (0x0f << 18)
#define DWC3_DSTS_RXFIFOEMPTY       (1 << 17)
#define DWC3_DSTS_SOFFN_SHIFT       3
#define DWC3_DSTS_SOFFN_MASK        (0x3fff << 3)
#define DWC3_DSTS_CONNECTSPD_MASK   0x7
#define DWC3_DSTS_SUPERSPEED        (4 << 0)
#define DWC3_DSTS_HIGHSPEED         (0 << 0)
#define DWC3_DSTS_FULLSPEED         (1 << 0)

/* DGCMD - Device Generic Command Register */

#define DWC3_DGCMD_SET_LMP              0x01
#define DWC3_DGCMD_XMIT_FUNCTION        0x03
#define DWC3_DGCMD_SELECTED_FIFO_FLUSH  0x09
#define DWC3_DGCMD_ALL_FIFO_FLUSH       0x0a
#define DWC3_DGCMD_SET_EP_NRDY          0x0c
#define DWC3_DGCMD_DEV_NOTIFICATION     0x07
#define DWC3_DGCMD_STATUS_SHIFT         12
#define DWC3_DGCMD_STATUS_MASK          (0x0f << 12)
#define DWC3_DGCMD_CMDACT               (1 << 10)
#define DWC3_DGCMD_CMDIOC               (1 << 8)

/* DEPCMD - Device Endpoint Command Register */

#define DWC3_DEPCMD_PARAM_SHIFT         16
#define DWC3_DEPCMD_PARAM(x)            ((x) << 16)
#define DWC3_DEPCMD_GET_RSC_IDX(x)      (((x) >> 16) & 0x7f)
#define DWC3_DEPCMD_STATUS_SHIFT        12
#define DWC3_DEPCMD_STATUS_MASK         (0x0f << 12)
#define DWC3_DEPCMD_HIPRI_FORCERM       (1 << 11)
#define DWC3_DEPCMD_CLEARPENDIN         (1 << 11)
#define DWC3_DEPCMD_CMDACT              (1 << 10)
#define DWC3_DEPCMD_CMDIOC              (1 << 8)

/* Endpoint Command Types */

#define DWC3_DEPCMD_SETEPCONFIG         0x01
#define DWC3_DEPCMD_SETTRANSFRESOURCE   0x02
#define DWC3_DEPCMD_GETEPSTATE          0x03
#define DWC3_DEPCMD_SETSTALL            0x04
#define DWC3_DEPCMD_CLEARSTALL          0x05
#define DWC3_DEPCMD_STARTTRANSFER       0x06
#define DWC3_DEPCMD_UPDATETRANSFER      0x07
#define DWC3_DEPCMD_ENDTRANSFER         0x08
#define DWC3_DEPCMD_DEPSTARTCFG         0x09

/* DALEPENA - Device Active Logical Endpoint Enable */

#define DWC3_DALEPENA_EP(n)             (1 << (n))

/* Endpoint Types for DEPCFG */

#define DWC3_DEPCMD_TYPE_CONTROL        0
#define DWC3_DEPCMD_TYPE_ISOC           1
#define DWC3_DEPCMD_TYPE_BULK           2
#define DWC3_DEPCMD_TYPE_INTR           3

/* Event Types - bit 0 determines event category:
 *   bit 0 = 0: Endpoint event (depevt)
 *   bit 0 = 1: Device-specific event (check bits[7:1] for type)
 *              If bits[7:1] == 0: Device event (devt)
 *              If bits[7:1] != 0: Other event (gevt)
 */

#define DWC3_EVENT_TYPE_MASK            0xfe
#define DWC3_EVENT_TYPE_DEV             0
#define DWC3_EVENT_IS_DEV_SPECIFIC(e)   (((e) & 1) != 0)

/* Device Events (when bit 0 == 0) */

#define DWC3_DEVICE_EVENT_DISCONNECT    0
#define DWC3_DEVICE_EVENT_RESET         1
#define DWC3_DEVICE_EVENT_CONNECT_DONE  2
#define DWC3_DEVICE_EVENT_LINK_STATUS   3
#define DWC3_DEVICE_EVENT_WAKEUP        4
#define DWC3_DEVICE_EVENT_SUSPEND       6
#define DWC3_DEVICE_EVENT_SOF           7
#define DWC3_DEVICE_EVENT_ERRATIC       9
#define DWC3_DEVICE_EVENT_CMD_CMPL      10
#define DWC3_DEVICE_EVENT_OVERFLOW      11

/* Device Event Macros */

#define DWC3_DEVT_TYPE(e)               (((e) >> 8) & 0xf)
#define DWC3_DEVT_INFO(e)               (((e) >> 16) & 0xf)

/* Endpoint Events (when bit 0 == 1) */

#define DWC3_DEPEVT_EPNUM(e)            (((e) >> 1) & 0x1f)
#define DWC3_DEPEVT_TYPE(e)             (((e) >> 6) & 0xf)
#define DWC3_DEPEVT_STATUS(e)           (((e) >> 12) & 0xf)
#define DWC3_DEPEVT_PARAM(e)            (((e) >> 16) & 0xffff)

/* Endpoint Event Types */

#define DWC3_DEPEVT_XFERCOMPLETE        1
#define DWC3_DEPEVT_XFERINPROGRESS      2
#define DWC3_DEPEVT_XFERNOTREADY        3
#define DWC3_DEPEVT_STREAMEVT           6
#define DWC3_DEPEVT_EPCMDCMPLT          7

/* XferNotReady Status */

#define DWC3_DEPEVT_STATUS_CONTROL_DATA     1
#define DWC3_DEPEVT_STATUS_CONTROL_STATUS   2

/* TRB (Transfer Request Block) Fields */

/* TRB Size Field */

#define DWC3_TRB_SIZE_MASK              0x00ffffff
#define DWC3_TRB_SIZE_LENGTH(n)         ((n) & DWC3_TRB_SIZE_MASK)
#define DWC3_TRB_SIZE_PCM1(n)           (((n) & 0x03) << 24)
#define DWC3_TRB_SIZE_TRBSTS(n)         (((n) & (0x0f << 28)) >> 28)

/* TRB Status Values */

#define DWC3_TRBSTS_OK                  0
#define DWC3_TRBSTS_MISSED_ISOC         1
#define DWC3_TRBSTS_SETUP_PENDING       2

/* TRB Control Field */

#define DWC3_TRB_CTRL_HWO              (1 << 0)
#define DWC3_TRB_CTRL_LST              (1 << 1)
#define DWC3_TRB_CTRL_CHN              (1 << 2)
#define DWC3_TRB_CTRL_CSP              (1 << 3)
#define DWC3_TRB_CTRL_TRBCTL_SHIFT     4
#define DWC3_TRB_CTRL_TRBCTL_MASK      (0x3f << 4)
#define DWC3_TRB_CTRL_TRBCTL(n)        (((n) & 0x3f) << 4)
#define DWC3_TRB_CTRL_ISP_IMI          (1 << 10)
#define DWC3_TRB_CTRL_IOC              (1 << 11)
#define DWC3_TRB_CTRL_SID_SOFN(n)      (((n) & 0xffff) << 14)

/* TRB Control Types */

#define DWC3_TRBCTL_NORMAL             DWC3_TRB_CTRL_TRBCTL(1)
#define DWC3_TRBCTL_CONTROL_SETUP      DWC3_TRB_CTRL_TRBCTL(2)
#define DWC3_TRBCTL_CONTROL_STATUS2    DWC3_TRB_CTRL_TRBCTL(3)
#define DWC3_TRBCTL_CONTROL_STATUS3    DWC3_TRB_CTRL_TRBCTL(4)
#define DWC3_TRBCTL_CONTROL_DATA       DWC3_TRB_CTRL_TRBCTL(5)
#define DWC3_TRBCTL_LINK_TRB           DWC3_TRB_CTRL_TRBCTL(8)

/* GSNPSID - Core ID */

#define DWC3_GSNPSID_MASK              0xffff0000
#define DWC3_GSNPSREV_MASK             0xffff

/* GHWPARAMS0 */

#define DWC3_GHWPARAMS0_MODE_MASK      0x3
#define DWC3_GHWPARAMS0_MODE_GADGET    0
#define DWC3_GHWPARAMS0_MODE_HOST      1
#define DWC3_GHWPARAMS0_MODE_DRD       2

/* GHWPARAMS3 */

#define DWC3_GHWPARAMS3_SSPHY_IFC_MASK 0x3
#define DWC3_GHWPARAMS3_HSPHY_IFC_SHIFT 2
#define DWC3_GHWPARAMS3_HSPHY_IFC_MASK (0x3 << 2)

/* Constants */

#define DWC3_ENDPOINTS_NUM             32
#define DWC3_EVENT_BUFFERS_SIZE        4096
#define DWC3_EP0_SETUP_SIZE            512
#define DWC3_EP0_BOUNCE_SIZE           512
#define DWC3_TRB_NUM                   32

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_DWC3_H */

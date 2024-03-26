/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx9_tpm.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_TPM_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_TPM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define IMX9_TPM_VERID_OFFSET    0x0000              /* Version ID */
#define IMX9_TPM_PARAM_OFFSET    0x0004              /* Parameter */
#define IMX9_TPM_GLOBAL_OFFSET   0x0008              /* TPM Global */
#define IMX9_TPM_SC_OFFSET       0x0010              /* Status and Control */
#define IMX9_TPM_CNT_OFFSET      0x0014              /* Counter */
#define IMX9_TPM_MOD_OFFSET      0x0018              /* Modulo */
#define IMX9_TPM_STATUS_OFFSET   0x001c              /* Capture and Compare Status */
#define IMX9_TPM_CXSC_OFFSET(ch) (0x0020 + (ch) * 8) /* Channel Status and Control */
#define IMX9_TPM_CXV_OFFSET(ch)  (0x0024 + (ch) * 8) /* Channel Value */
#define IMX9_TPM_C1SC_OFFSET     0x0028              /* Channel n Status and Control */
#define IMX9_TPM_C1V_OFFSET      0x002c              /* Channel n Value */
#define IMX9_TPM_C2SC_OFFSET     0x0030              /* Channel n Status and Control */
#define IMX9_TPM_C2V_OFFSET      0x0034              /* Channel n Value */
#define IMX9_TPM_C3SC_OFFSET     0x0038              /* Channel n Status and Control */
#define IMX9_TPM_C3V_OFFSET      0x003c              /* Channel n Value */
#define IMX9_TPM_COMBINE_OFFSET  0x0064              /* Combine Channel */
#define IMX9_TPM_TRIG_OFFSET     0x006c              /* Channel Trigger */
#define IMX9_TPM_POL_OFFSET      0x0070              /* Channel Polarity */
#define IMX9_TPM_FILTER_OFFSET   0x0078              /* Filter Control */
#define IMX9_TPM_QDCTRL_OFFSET   0x0080              /* Quadrature Decoder Control and Status */
#define IMX9_TPM_CONF_OFFSET     0x0084              /* Configuration */

/* Register Bitfield Definitions ********************************************/

/* PARAM */

#define TPM_PARAM_WIDTH_SHIFT         (16) /* Bit[23:16]: Width of the counter and timer channels */
#define TPM_PARAM_WIDTH_MASK          (0xff << TPM_PARAM_WIDTH_SHIFT)

#define TPM_PARAM_TRIG_SHIFT          (8)  /* Bit[15:8]: Number of triggers that TPM implements */
#define TPM_PARAM_TRIG_MASK           (0xff << LPIT_PARAM_TRIG_SHIFT)

#define TPM_PARAM_CHAN_SHIFT          (0)  /* Bit[7:0]: Number of timer channels */
#define TPM_PARAM_CHAN_MASK           (0xff << TPM_PARAM_CHAN_SHIFT)

/* GLOBAL */

#define TPM_GLOBAL_RST_SHIFT          (1) /* Bit[1]: Software Reset */
#define TPM_GLOBAL_RST_MASK           (0x1 << TPM_GLOBAL_RST_SHIFT)

#define TPM_GLOBAL_NOUPDATE_SHIFT     (0) /* Bit[0]: Block updates to internal registers */
#define TPM_GLOBAL_NOUPDATE_MASK      (0x1 << TPM_GLOBAL_NOUPDATE_SHIFT)

/* SC */

#define TPM_SC_DMA_SHIFT              (8)  /* Bit[8]: DMA Enable */
#define TPM_SC_DMA_MASK               (0x1 << TPM_SC_DMA_SHIFT)

#define TPM_SC_TOF_SHIFT              (7)  /* Bit[7]: Timer Overflow Flag */
#define TPM_SC_TOF_MASK               (0x1 << TPM_SC_TOF_SHIFT)

#define TPM_SC_TOIE_SHIFT             (6)  /* Bit[6]: Timer Overflow Interrupt Enable */
#define TPM_SC_TOIE_MASK              (0x1 << TPM_SC_TOIE_SHIFT)

#define TPM_SC_CPWMS_SHIFT            (5)  /* Bit[5]: Center-Aligned PWM Select */
#define TPM_SC_CPWMS_MASK             (0x1 << TPM_SC_CPWMS_SHIFT)

#define TPM_SC_CMOD_SHIFT             (3)  /* Bit[4:3]: Clock Mode Selection */
#define TPM_SC_CMOD_MASK              (0x3 << TPM_SC_CMOD_SHIFT)

#define TPM_SC_PS_SHIFT               (0)  /* Bit[2:0]: Prescale Factor Selection */
#define TPM_SC_PS_MASK                (0x7 << TPM_SC_PS_SHIFT)

/* STATUS */

#define TPM_STATUS_TOF_SHIFT          (8) /* Bit[8]: Timer Overflow Flag */
#define TPM_STATUS_TOF_MASK           (0x1 << TPM_STATUS_TOF_SHIFT)

#define TPM_STATUS_CH3F_SHIFT         (3) /* Bit[3]: Channel 3 Flag */
#define TPM_STATUS_CH3F_MASK          (0x1 << TPM_STATUS_CH3F_SHIFT)

#define TPM_STATUS_CH2F_SHIFT         (2) /* Bit[2]: Channel 2 Flag */
#define TPM_STATUS_CH2F_MASK          (0x1 << TPM_STATUS_CH2F_SHIFT)

#define TPM_STATUS_CH1F_SHIFT         (1) /* Bit[1]: Channel 1 Flag */
#define TPM_STATUS_CH1F_MASK          (0x1 << TPM_STATUS_CH1F_SHIFT)

#define TPM_STATUS_CH0F_SHIFT         (0) /* Bit[0]: Channel 0 Flag */
#define TPM_STATUS_CH0F_MASK          (0x1 << TPM_STATUS_CH0F_SHIFT)

/* C0SC - C3SC */

#define TPM_CXSC_CHF_SHIFT            (7) /* Bit[7]: Channel Flag */
#define TPM_CXSC_CHF_MASK             (0x1 << TPM_CXSC_CHF_SHIFT)

#define TPM_CXSC_CHIE_SHIFT           (6) /* Bit[6]: Channel Interrupt Enable */
#define TPM_CXSC_CHIE_MASK            (0x1 << TPM_CXSC_CHIE_SHIFT)

#define TPM_CXSC_MSB_SHIFT            (5) /* Bit[5]: Channel Mode Select B */
#define TPM_CXSC_MSB_MASK             (0x1 << TPM_CXSC_MSB_SHIFT)

#define TPM_CXSC_MSA_SHIFT            (4) /* Bit[4]: Channel Mode Select A */
#define TPM_CXSC_MSA_MASK             (0x1 << TPM_CXSC_MSA_SHIFT)

#define TPM_CXSC_ELSB_SHIFT           (3) /* Bit[3]: Edge or Level Select B */
#define TPM_CXSC_ELSB_MASK            (0x1 << TPM_CXSC_ELSB_SHIFT)

#define TPM_CXSC_ELSA_SHIFT           (2) /* Bit[2]: Edge or Level Select A */
#define TPM_CXSC_ELSA_MASK            (0x1 << TPM_CXSC_ELSA_SHIFT)

#define TPM_CXSC_DMA_SHIFT            (0) /* Bit[0]: DMA Enable */
#define TPM_CXSC_DMA_MASK             (0x1 << TPM_CXSC_DMA_SHIFT)

/* COMBINE */

#define TPM_COMBINE_COMSWAP1_SHIFT    (9) /* Bit[9]: Combine Channels 2 and 3 Swap */
#define TPM_COMBINE_COMSWAP1_MASK     (0x1 << TPM_COMBINE_COMSWAP1_SHIFT)

#define TPM_COMBINE_COMBINE1_SHIFT    (8) /* Bit[8]: Combine Channels 2 and 3 */
#define TPM_COMBINE_COMBINE1_MASK     (0x1 << TPM_COMBINE_COMBINE1_SHIFT)

#define TPM_COMBINE_COMSWAP0_SHIFT    (1) /* Bit[1]: Combine Channel 0 and 1 Swap */
#define TPM_COMBINE_COMSWAP0_MASK     (0x1 << TPM_COMBINE_COMSWAP0_SHIFT)

#define TPM_COMBINE_COMBINE0_SHIFT    (0) /* Bit[0]: Combine Channels 0 and 1 */
#define TPM_COMBINE_COMBINE0_MASK     (0x1 << TPM_COMBINE_COMBINE0_SHIFT)

/* TRIG */

#define TPM_TRIG_TRIGX_MASK(ch)       (0x1 << (ch)) /* Channel trigger configure */

/* POL */

#define TPM_POL_POLX_MASK(ch)         (0x1 < (ch)) /* Channel polarity active low */

/* FILTER */

#define TPM_FILTER_CHXFVAL_MASK(ch)   (0xf << ((ch) * 4))) /* Channel filter value */

/* QDCTRL */

#define TPM_QDCTRL_QUADMODE_SHIFT     (3) /* Bit[3]: Quadrature Decoder Mode */
#define TPM_QDCTRL_QUADMODE_MASK      (0x1 << TPM_QDCTRL_QUADMODE_SHIFT)

#define TPM_QDCTRL_QUADIR_SHIFT       (2) /* Bit[2]: Counter Direction */
#define TPM_QDCTRL_QUADIR_MASK        (0x1 << TPM_QDCTRL_QUADIR_SHIFT)

#define TPM_QDCTRL_TOFDIR_SHIFT       (1) /* Bit[1]: Timer Overflow Direction */
#define TPM_QDCTRL_TOFDIR_MASK        (0x1 << TPM_QDCTRL_TOFDIR_SHIFT)

#define TPM_QDCTRL_QUADEN_SHIFT       (0) /* Bit[0]: Quadrature Decoder Enable */
#define TPM_QDCTRL_QUADEN_MASK        (0x1 << TPM_QDCTRL_QUADEN_SHIFT)

/* CONF */

#define TPM_CONF_TRGSEL_SHIFT         (24) /* Bit[25:24]: Trigger Select */
#define TPM_CONF_TRGSEL_MASK          (0x3 << TPM_CONF_TRGSEL_SHIFT)

#define TPM_CONF_TRGSRC_SHIFT         (23) /* Bit[23]: Trigger Source select */
#define TPM_CONF_TRGSRC_MASK          (0x1 << TPM_CONF_TRGSRC_SHIFT)

#define TPM_CONF_TRGPOL_SHIFT         (22) /* Bit[22]: Trigger Polarity */
#define TPM_CONF_TRGPOL_MASK          (0x1 << TPM_CONF_TRGPOL_SHIFT)

#define TPM_CONF_CPOT_SHIFT           (19) /* Bit[19]: Counter Pause on Trigger */
#define TPM_CONF_CPOT_MASK            (0x1 << TPM_CONF_CPOT_SHIFT)

#define TPM_CONF_CROT_SHIFT           (18) /* Bit[18]: Counter Reload on Trigger */
#define TPM_CONF_CROT_MASK            (0x1 << TPM_CONF_CROT_SHIFT)

#define TPM_CONF_CSOO_SHIFT           (17) /* Bit[17]: Counter Stop on Overflow */
#define TPM_CONF_CSOO_MASK            (0x1 << TPM_CONF_CSOO_SHIFT)

#define TPM_CONF_CSOT_SHIFT           (16) /* Bit[16]: Counter Start on Trigger */
#define TPM_CONF_CSOT_MASK            (0x1 << TPM_CONF_CSOT_SHIFT)

#define TPM_CONF_DBGMODE_SHIFT        (6) /* Bit[7:6]: Debug Mode */
#define TPM_CONF_DBGMODE_MASK         (0x3 << TPM_CONF_DBGMODE_SHIFT)

#define TPM_CONF_DOZEEN_SHIFT         (5) /* Bit[5]: Doze Enable */
#define TPM_CONF_DOZEEN_MASK          (0x1 << TPM_CONF_DOZEEN_SHIFT)

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_TPM_H */

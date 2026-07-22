/****************************************************************************
 * arch/mips/src/jz4780/jz4780_lcd.c
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *-
 * Copyright (c) 2015 Oleksandr Tymoshenko <gonzo@freebsd.org>
 * Copyright (c) 2016 Jared McNeill <jmcneill@invisible.ca>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/video/fb.h>

#include "mips_internal.h"
#include "chip.h"

#include <arch/board/board.h>

#ifdef CONFIG_ALLOW_BSD_COMPONENTS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define JZ4780_LCD_VRAMBASE                               0xAF800000
#define JZ4780_LCD_HWIDTH                                 1360
#define JZ4780_LCD_VHEIGHT                                768

#define PCFG_MAGIC                                        0xc7ff2100

#define FOREGROUND_OFFSET                                 0x400000

#define VID_PHSYNC                                        0x0001
#define VID_PVSYNC                                        0x0004
#define VID_INTERLACE                                     0x0010

#define JZ_REG_SHIFT                                      2

#define JZ4780_COLOR_FMT                                  FB_FMT_RGB16_565
#define JZ4780_BPP                                        16

#define JZ4780_STRIDE ((JZ4780_LCD_HWIDTH * JZ4780_BPP + 7) / 8)
#define JZ4780_FBSIZE (JZ4780_STRIDE * JZ4780_LCD_VHEIGHT)

/* HDMI controller registers */

#define HDMI_DESIGN_ID                                    0x0000
#define HDMI_REVISION_ID                                  0x0001
#define HDMI_PRODUCT_ID0                                  0x0002
#define HDMI_PRODUCT_ID1                                  0x0003

/* Interrupt Registers */
#define HDMI_IH_PHY_STAT0                                 0x0104
#  define HDMI_IH_PHY_STAT0_HPD                           (1 << 0)
#define HDMI_IH_I2CMPHY_STAT0                             0x0108
#  define HDMI_IH_I2CMPHY_STAT0_DONE                      (1 << 1)
#  define HDMI_IH_I2CMPHY_STAT0_ERROR                     (1 << 0)

#define HDMI_IH_MUTE_FC_STAT2                             0x0182
#  define HDMI_IH_MUTE_FC_STAT2_OVERFLOW_MASK             (0x3)

/* Video Sample Registers */
#define HDMI_TX_INVID0                                    0x0200
#define HDMI_TX_INVID0_INTERNAL_DE_GENERATOR_DISABLE      0x00
#define HDMI_TX_INVID0_VIDEO_MAPPING_MASK                 0x1F
#define HDMI_TX_INVID0_VIDEO_MAPPING_OFFSET               0
#define HDMI_TX_INSTUFFING                                0x0201
#define HDMI_TX_INSTUFFING_BDBDATA_STUFFING_ENABLE        0x4
#define HDMI_TX_INSTUFFING_RCRDATA_STUFFING_ENABLE        0x2
#define HDMI_TX_INSTUFFING_GYDATA_STUFFING_ENABLE         0x1
#define HDMI_TX_GYDATA0                                   0x0202
#define HDMI_TX_GYDATA1                                   0x0203
#define HDMI_TX_RCRDATA0                                  0x0204
#define HDMI_TX_RCRDATA1                                  0x0205
#define HDMI_TX_BCBDATA0                                  0x0206
#define HDMI_TX_BCBDATA1                                  0x0207

/* Video Packetizer Registers */
#define HDMI_VP_PR_CD                                     0x0801
#  define HDMI_VP_PR_CD_COLOR_DEPTH_MASK                  0xF0
#  define HDMI_VP_PR_CD_COLOR_DEPTH_OFFSET                4

#define HDMI_VP_STUFF                                     0x0802
#  define HDMI_VP_STUFF_IDEFAULT_PHASE_MASK               0x20
#  define HDMI_VP_STUFF_IDEFAULT_PHASE_OFFSET             5
#  define HDMI_VP_STUFF_YCC422_STUFFING_MASK              0x4
#  define HDMI_VP_STUFF_YCC422_STUFFING_STUFFING_MODE     0x4
#  define HDMI_VP_STUFF_PP_STUFFING_MASK                  0x2
#  define HDMI_VP_STUFF_PP_STUFFING_STUFFING_MODE         0x2
#  define HDMI_VP_STUFF_PR_STUFFING_MASK                  0x1
#  define HDMI_VP_STUFF_PR_STUFFING_STUFFING_MODE         0x1
#define HDMI_VP_REMAP                                     0x0803
#  define HDMI_VP_REMAP_YCC422_16BIT                      0x0
#define HDMI_VP_CONF                                      0x0804
#  define HDMI_VP_CONF_BYPASS_EN_MASK                     0x40
#  define HDMI_VP_CONF_BYPASS_EN_ENABLE                   0x40
#  define HDMI_VP_CONF_BYPASS_EN_DISABLE                  0x00
#  define HDMI_VP_CONF_PP_EN_ENMASK                       0x20
#  define HDMI_VP_CONF_PP_EN_ENABLE                       0x20
#  define HDMI_VP_CONF_PP_EN_DISABLE                      0x00
#  define HDMI_VP_CONF_PR_EN_MASK                         0x10
#  define HDMI_VP_CONF_PR_EN_DISABLE                      0x00
#  define HDMI_VP_CONF_YCC422_EN_MASK                     0x8
#  define HDMI_VP_CONF_YCC422_EN_ENABLE                   0x8
#  define HDMI_VP_CONF_YCC422_EN_DISABLE                  0x0
#  define HDMI_VP_CONF_BYPASS_SELECT_MASK                 0x4
#  define HDMI_VP_CONF_BYPASS_SELECT_VID_PACKETIZER       0x4
#  define HDMI_VP_CONF_OUTPUT_SELECTOR_MASK               0x3
#  define HDMI_VP_CONF_OUTPUT_SELECTOR_BYPASS             0x3
#  define HDMI_VP_CONF_OUTPUT_SELECTOR_YCC422             0x1
#  define HDMI_VP_CONF_OUTPUT_SELECTOR_PP                 0x0

/* Frame Composer Registers */
#define HDMI_FC_INVIDCONF                                 0x1000
#  define HDMI_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_HIGH 0x40
#  define HDMI_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_LOW  0x00
#  define HDMI_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_HIGH 0x20
#  define HDMI_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_LOW  0x00
#  define HDMI_FC_INVIDCONF_DE_IN_POLARITY_ACTIVE_HIGH    0x10
#  define HDMI_FC_INVIDCONF_DVI_MODEZ_DVI_MODE            0x0
#  define HDMI_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_HIGH  0x2
#  define HDMI_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_LOW   0x0
#  define HDMI_FC_INVIDCONF_IN_I_P_INTERLACED             0x1
#  define HDMI_FC_INVIDCONF_IN_I_P_PROGRESSIVE            0x0
#define HDMI_FC_INHACTV0                                  0x1001
#define HDMI_FC_INHACTV1                                  0x1002
#define HDMI_FC_INHBLANK0                                 0x1003
#define HDMI_FC_INHBLANK1                                 0x1004
#define HDMI_FC_INVACTV0                                  0x1005
#define HDMI_FC_INVACTV1                                  0x1006
#define HDMI_FC_INVBLANK                                  0x1007
#define HDMI_FC_HSYNCINDELAY0                             0x1008
#define HDMI_FC_HSYNCINDELAY1                             0x1009
#define HDMI_FC_HSYNCINWIDTH0                             0x100A
#define HDMI_FC_HSYNCINWIDTH1                             0x100B
#define HDMI_FC_VSYNCINDELAY                              0x100C
#define HDMI_FC_VSYNCINWIDTH                              0x100D
#define HDMI_FC_CTRLDUR                                   0x1011
#define HDMI_FC_EXCTRLDUR                                 0x1012
#define HDMI_FC_EXCTRLSPAC                                0x1013
#define HDMI_FC_CH0PREAM                                  0x1014
#define HDMI_FC_CH1PREAM                                  0x1015
#define HDMI_FC_CH2PREAM                                  0x1016
#define HDMI_FC_MASK2                                     0x10DA
#  define HDMI_FC_MASK2_LOW_PRI                           (1 << 1)
#  define HDMI_FC_MASK2_HIGH_PRI                          (1 << 0)

#define HDMI_PHY_CONF0                                    0x3000
#  define HDMI_PHY_CONF0_PDZ_MASK                         0x80
#  define HDMI_PHY_CONF0_PDZ_OFFSET                       7
#  define HDMI_PHY_CONF0_ENTMDS_MASK                      0x40
#  define HDMI_PHY_CONF0_ENTMDS_OFFSET                    6
#  define HDMI_PHY_CONF0_GEN2_PDDQ_MASK                   0x10
#  define HDMI_PHY_CONF0_GEN2_PDDQ_OFFSET                 4
#  define HDMI_PHY_CONF0_GEN2_TXPWRON_MASK                0x8
#  define HDMI_PHY_CONF0_GEN2_TXPWRON_OFFSET              3
#  define HDMI_PHY_CONF0_SELDATAENPOL_MASK                0x2
#  define HDMI_PHY_CONF0_SELDATAENPOL_OFFSET              1
#  define HDMI_PHY_CONF0_SELDIPIF_MASK                    0x1
#  define HDMI_PHY_CONF0_SELDIPIF_OFFSET                  0
#define HDMI_PHY_TST0                                     0x3001
#  define HDMI_PHY_TST0_TSTCLR_MASK                       0x20
#  define HDMI_PHY_TST0_TSTCLR_OFFSET                     5
#define HDMI_PHY_STAT0                                    0x3004
#  define HDMI_PHY_TX_PHY_LOCK                            0x01
#define HDMI_PHY_POL0                                     0x3007
#  define HDMI_PHY_POL0_HPD                               0x02

/* HDMI Master PHY Registers */
#define HDMI_PHY_I2CM_SLAVE_ADDR                          0x3020
#  define HDMI_PHY_I2CM_SLAVE_ADDR_PHY_GEN2               0x69
#define HDMI_PHY_I2CM_ADDRESS_ADDR                        0x3021
#define HDMI_PHY_I2CM_DATAO_1_ADDR                        0x3022
#define HDMI_PHY_I2CM_DATAO_0_ADDR                        0x3023
#define HDMI_PHY_I2CM_OPERATION_ADDR                      0x3026

/* Main Controller Registers */
#define HDMI_MC_CLKDIS                                    0x4001
#  define HDMI_MC_CLKDIS_HDCPCLK_DISABLE                  (1 << 6)
#  define HDMI_MC_CLKDIS_CECCLK_DISABLE                   (1 << 5)
#  define HDMI_MC_CLKDIS_AUDCLK_DISABLE                   (1 << 3)
#  define HDMI_MC_CLKDIS_TMDSCLK_DISABLE                  (1 << 1)
#  define HDMI_MC_CLKDIS_PIXELCLK_DISABLE                 (1 << 0)

#define HDMI_MC_SWRSTZ                                    0x4002
#  define HDMI_MC_SWRSTZ_TMDSSWRST_REQ                    0x02
#define HDMI_MC_FLOWCTRL                                  0x4004
#  define HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_BYPASS    0x0
#define HDMI_MC_PHYRSTZ                                   0x4005
#  define HDMI_MC_PHYRSTZ_ASSERT                          0x0
#  define HDMI_MC_PHYRSTZ_DEASSERT                        0x1
#define HDMI_MC_HEACPHY_RST                               0x4007
#  define HDMI_MC_HEACPHY_RST_ASSERT                      0x1

/* HDCP Encryption Engine Registers */
#define HDMI_A_HDCPCFG0                                   0x5000
#  define HDMI_A_HDCPCFG0_RXDETECT_MASK                   0x4
#  define HDMI_A_HDCPCFG0_RXDETECT_DISABLE                0x0
#define HDMI_A_HDCPCFG1                                   0x5001
#  define HDMI_A_HDCPCFG1_ENCRYPTIONDISABLE_MASK          0x2
#  define HDMI_A_HDCPCFG1_ENCRYPTIONDISABLE_DISABLE       0x2
#define HDMI_A_VIDPOLCFG                                  0x5009
#  define HDMI_A_VIDPOLCFG_DATAENPOL_MASK                 0x10
#  define HDMI_A_VIDPOLCFG_DATAENPOL_ACTIVE_HIGH          0x10

/* I2C Master Registers (E-DDC) */
#define HDMI_I2CM_SLAVE                                   0x7E00
#define HDMI_I2CMESS                                      0x7E01
#define HDMI_I2CM_DATAO                                   0x7E02
#define HDMI_I2CM_DATAI                                   0x7E03
#define HDMI_I2CM_OPERATION                               0x7E04
#  define HDMI_PHY_I2CM_OPERATION_ADDR_WRITE              0x10
#  define HDMI_PHY_I2CM_OPERATION_ADDR_READ               0x1
#define HDMI_I2CM_INT                                     0x7E05
#define HDMI_I2CM_CTLINT                                  0x7E06
#define HDMI_I2CM_DIV                                     0x7E07
#define HDMI_I2CM_SEGADDR                                 0x7E08
#define HDMI_I2CM_SOFTRSTZ                                0x7E09
#define HDMI_I2CM_SEGPTR                                  0x7E0A
#define HDMI_I2CM_SS_SCL_HCNT_1_ADDR                      0x7E0B
#define HDMI_I2CM_SS_SCL_HCNT_0_ADDR                      0x7E0C
#define HDMI_I2CM_SS_SCL_LCNT_1_ADDR                      0x7E0D
#define HDMI_I2CM_SS_SCL_LCNT_0_ADDR                      0x7E0E
#define HDMI_I2CM_FS_SCL_HCNT_1_ADDR                      0x7E0F
#define HDMI_I2CM_FS_SCL_HCNT_0_ADDR                      0x7E10
#define HDMI_I2CM_FS_SCL_LCNT_1_ADDR                      0x7E11
#define HDMI_I2CM_FS_SCL_LCNT_0_ADDR                      0x7E12

/* HDMI PHY register with access through I2C */
#define HDMI_PHY_I2C_CKCALCTRL                            0x5
#  define CKCALCTRL_OVERRIDE                              (1 << 15)
#define HDMI_PHY_I2C_CPCE_CTRL                            0x6
#  define CPCE_CTRL_45_25                                 ((3 << 7) | (3 << 5))
#  define CPCE_CTRL_92_50                                 ((2 << 7) | (2 << 5))
#  define CPCE_CTRL_185                                   ((1 << 7) | (1 << 5))
#  define CPCE_CTRL_370                                   ((0 << 7) | (0 << 5))
#define HDMI_PHY_I2C_CKSYMTXCTRL                          0x9
#  define CKSYMTXCTRL_OVERRIDE                            (1 << 15)
#  define CKSYMTXCTRL_TX_SYMON                            (1 << 3)
#  define CKSYMTXCTRL_TX_TRAON                            (1 << 2)
#  define CKSYMTXCTRL_TX_TRBON                            (1 << 1)
#  define CKSYMTXCTRL_TX_CK_SYMON                         (1 << 0)
#define HDMI_PHY_I2C_VLEVCTRL                             0x0E
#  define VLEVCTRL_TX_LVL(x)                              ((x) << 5)
#  define VLEVCTRL_CK_LVL(x)                              (x)
#define HDMI_PHY_I2C_CURRCTRL                             0x10
#define HDMI_PHY_I2C_PLLPHBYCTRL                          0x13
#define HDMI_PHY_I2C_GMPCTRL                              0x15
#define GMPCTRL_45_25                                     0x00
#define GMPCTRL_92_50                                     0x05
#define GMPCTRL_185                                       0x0a
#define GMPCTRL_370                                       0x0f
#define HDMI_PHY_I2C_MSM_CTRL                             0x17
#  define MSM_CTRL_FB_CLK                                 (0x3 << 1)
#define HDMI_PHY_I2C_TXTERM                               0x19
#define TXTERM_133                                        0x5

/* LCD controller registers */

#define LCDCFG                                            0x0000
#  define LCDCFG_NEWDES                                   (1 << 28)
#  define LCDCFG_RECOVER                                  (1 << 25)
#  define LCDCFG_PSM                                      (1 << 23)
#  define LCDCFG_CLSM                                     (1 << 22)
#  define LCDCFG_SPLM                                     (1 << 21)
#  define LCDCFG_REVM                                     (1 << 20)
#  define LCDCFG_PCP                                      (1 << 10)
#  define LCDCFG_24                                       (1 << 6)
#define LCDCTRL                                           0x0030
#  define LCDCTRL_BST                                     (0x7 << 28)
#  define LCDCTRL_BST_64                                  (4 << 28)
#  define LCDCTRL_OFUM                                    (1 << 11)
#  define LCDCTRL_DIS                                     (1 << 4)
#  define LCDCTRL_ENA                                     (1 << 3)

#define LCDSTATE                                          0x0034
#  define LCDSTATE_LDD                                    (1 << 0)
#define LCDOSDS                                           0x0108
#define LCDRGBC                                           0x0090
#  define LCDRGBC_RGBFMT                                  (1 << 7)
#define LCDVAT                                            0x000c
#  define LCDVAT_HT_SHIFT                                 16
#  define LCDVAT_VT_SHIFT                                 0
#define LCDDAH                                            0x0010
#  define LCDDAH_HDS_SHIFT                                16
#  define LCDDAH_HDE_SHIFT                                0
#define LCDDAV                                            0x0014
#  define LCDDAV_VDS_SHIFT                                16
#  define LCDDAV_VDE_SHIFT                                0
#define LCDVSYNC                                          0x0004
#define LCDHSYNC                                          0x0008
#define LCDDA0                                            0x0040
#define LCDDA1                                            0x0050
#define LCDPCFG                                           0x02c0

/* Descriptor flags */

#define LCDCMD_FRM_EN                                     (1 << 26)

#define LCDPOS_BPP01_15_16                                (4 << 27)
#define LCDPOS_BPP01_24_COMP                              (6 << 27)
#define LCDPOS_PREMULTI01                                 (1 << 26)
#define LCDPOS_COEF_SLE01                                 (0x3 << 24)
#define LCDPOS_COEF_BLE01_1                               (1 << 24)

#define LCDDESSIZE_ALPHA                                  (0xff << 24)
#define LCDDESSIZE_HEIGHT_SHIFT                           12
#define LCDDESSIZE_WIDTH_SHIFT                            0

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Get information about the video controller configuration and the
 * configuration of each color plane.
 */

static int jz_getvideoinfo(struct fb_vtable_s *vtable,
             struct fb_videoinfo_s *vinfo);
static int jz_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
             struct fb_planeinfo_s *pinfo);

/* The following is provided only if the video hardware supports RGB color
 * mapping
 */

#ifdef CONFIG_FB_CMAP
static int jz_getcmap(struct fb_vtable_s *vtable,
             struct fb_cmap_s *cmap);
static int jz_putcmap(struct fb_vtable_s *vtable,
             const struct fb_cmap_s *cmap);
#endif

/* The following is provided only if the video hardware supports a hardware
 * cursor
 */

#ifdef CONFIG_FB_HWCURSOR
static int jz_getcursor(struct fb_vtable_s *vtable,
             struct fb_cursorattrib_s *attrib);
static int jz_setcursor(struct fb_vtable_s *vtable,
             struct fb_setcursor_s *settings);
#endif

static uint8_t hdmi_rd1(uint32_t off);
static void hdmi_wr1(uint32_t off, uint8_t val);
static void dwc_hdmi_init(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure describes the video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt      = JZ4780_COLOR_FMT,
  .xres     = JZ4780_LCD_HWIDTH,
  .yres     = JZ4780_LCD_VHEIGHT,
  .nplanes  = 1,
};

/* This structure describes the single color plane */

static const struct fb_planeinfo_s g_planeinfo =
{
  .fbmem    = (void *)JZ4780_LCD_VRAMBASE,
  .fblen    = JZ4780_FBSIZE,
  .stride   = JZ4780_STRIDE,
  .display  = 0,
  .bpp      = JZ4780_BPP,
};

/* Current cursor position */

#ifdef CONFIG_FB_HWCURSOR
static struct fb_cursorpos_s g_cpos;

/* Current cursor size */

#ifdef CONFIG_FB_HWCURSORSIZE
static struct fb_cursorsize_s g_csize;
#endif
#endif

/* The framebuffer object -- There is no private state information in this
 * framebuffer driver.
 */

struct fb_vtable_s g_fbobject =
{
  .getvideoinfo  = jz_getvideoinfo,
  .getplaneinfo  = jz_getplaneinfo,
#ifdef CONFIG_FB_CMAP
  .getcmap       = jz_getcmap,
  .putcmap       = jz_putcmap,
#endif
#ifdef CONFIG_FB_HWCURSOR
  .getcursor     = jz_getcursor,
  .setcursor     = jz_setcursor,
#endif
};

struct lcd_frame_descriptor
{
  uint32_t next;
  uint32_t physaddr;
  uint32_t id;
  uint32_t cmd;
  uint32_t offs;
  uint32_t pw;
  uint32_t cnum_pos;
  uint32_t dessize;
} __packed;

struct videomode_s
{
  int dot_clock;    /* Dot clock frequency in kHz. */
  int hdisplay;
  int hsync_start;
  int hsync_end;
  int htotal;
  int vdisplay;
  int vsync_start;
  int vsync_end;
  int vtotal;
  int flags;         /* Video mode flags; see below. */
};

struct display_s
{
  uint32_t fbsize;
  uint32_t paddr;
  uint32_t vaddr;

  uint32_t fdesc_paddr;
  struct lcd_frame_descriptor *fdesc;

  struct videomode_s sc_mode;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t hdmi_rd1(uint32_t off)
{
  return getreg8(0xb0180000 + (off << JZ_REG_SHIFT));
}

static void hdmi_wr1(uint32_t off, uint8_t val)
{
  putreg8(val, 0xb0180000 + (off << JZ_REG_SHIFT));
}

static uint32_t lcd_read(struct display_s *sc, uint32_t reg)
{
  return getreg32(0xb3050000 + reg);
}

static void lcd_write(struct display_s *sc, uint32_t reg, uint32_t val)
{
  putreg32(val, 0xb3050000 + reg);
}

static void
dwc_hdmi_phy_wait_i2c_done(int msec)
{
  uint8_t val;

  val = hdmi_rd1(HDMI_IH_I2CMPHY_STAT0) &
      (HDMI_IH_I2CMPHY_STAT0_DONE | HDMI_IH_I2CMPHY_STAT0_ERROR);
  while (val == 0)
    {
      up_mdelay(10);
      msec -= 10;
      if (msec <= 0)
        return;
      val = hdmi_rd1(HDMI_IH_I2CMPHY_STAT0) &
          (HDMI_IH_I2CMPHY_STAT0_DONE | HDMI_IH_I2CMPHY_STAT0_ERROR);
    }
}

static void
dwc_hdmi_phy_i2c_write(unsigned short data, unsigned char addr)
{
  /* clear DONE and ERROR flags */

  hdmi_wr1(HDMI_IH_I2CMPHY_STAT0,
      HDMI_IH_I2CMPHY_STAT0_DONE | HDMI_IH_I2CMPHY_STAT0_ERROR);
  hdmi_wr1(HDMI_PHY_I2CM_ADDRESS_ADDR, addr);
  hdmi_wr1(HDMI_PHY_I2CM_DATAO_1_ADDR, ((data >> 8) & 0xff));
  hdmi_wr1(HDMI_PHY_I2CM_DATAO_0_ADDR, ((data >> 0) & 0xff));
  hdmi_wr1(HDMI_PHY_I2CM_OPERATION_ADDR, HDMI_PHY_I2CM_OPERATION_ADDR_WRITE);
  dwc_hdmi_phy_wait_i2c_done(1000);
}

static void
dwc_hdmi_disable_overflow_interrupts(void)
{
  hdmi_wr1(HDMI_IH_MUTE_FC_STAT2, HDMI_IH_MUTE_FC_STAT2_OVERFLOW_MASK);
  hdmi_wr1(HDMI_FC_MASK2, HDMI_FC_MASK2_LOW_PRI | HDMI_FC_MASK2_HIGH_PRI);
}

static void
dwc_hdmi_av_composer(struct display_s *sc)
{
  uint8_t inv_val;
  int hblank;
  int vblank;
  int hsync_len;
  int hfp;
  int vfp;

  /* Set up HDMI_FC_INVIDCONF */

  inv_val = ((sc->sc_mode.flags & VID_PVSYNC) ?
    HDMI_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_HIGH :
    HDMI_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_LOW);

  inv_val |= ((sc->sc_mode.flags & VID_PHSYNC) ?
    HDMI_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_HIGH :
    HDMI_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_LOW);

  inv_val |= HDMI_FC_INVIDCONF_DE_IN_POLARITY_ACTIVE_HIGH;

  inv_val |= ((sc->sc_mode.flags & VID_INTERLACE) ?
      HDMI_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_HIGH :
      HDMI_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_LOW);

  inv_val |= ((sc->sc_mode.flags & VID_INTERLACE) ?
    HDMI_FC_INVIDCONF_IN_I_P_INTERLACED :
    HDMI_FC_INVIDCONF_IN_I_P_PROGRESSIVE);

  /* TODO: implement HDMI part */

  inv_val |= HDMI_FC_INVIDCONF_DVI_MODEZ_DVI_MODE;

  hdmi_wr1(HDMI_FC_INVIDCONF, inv_val);

  /* Set up horizontal active pixel region width */

  hdmi_wr1(HDMI_FC_INHACTV1, sc->sc_mode.hdisplay >> 8);
  hdmi_wr1(HDMI_FC_INHACTV0, sc->sc_mode.hdisplay);

  /* Set up vertical blanking pixel region width */

  hdmi_wr1(HDMI_FC_INVACTV1, sc->sc_mode.vdisplay >> 8);
  hdmi_wr1(HDMI_FC_INVACTV0, sc->sc_mode.vdisplay);

  /* Set up horizontal blanking pixel region width */

  hblank = sc->sc_mode.htotal - sc->sc_mode.hdisplay;
  hdmi_wr1(HDMI_FC_INHBLANK1, hblank >> 8);
  hdmi_wr1(HDMI_FC_INHBLANK0, hblank);

  /* Set up vertical blanking pixel region width */

  vblank = sc->sc_mode.vtotal - sc->sc_mode.vdisplay;
  hdmi_wr1(HDMI_FC_INVBLANK, vblank);

  /* Set up HSYNC active edge delay width (in pixel clks) */

  hfp = sc->sc_mode.hsync_start - sc->sc_mode.hdisplay;
  hdmi_wr1(HDMI_FC_HSYNCINDELAY1, hfp >> 8);
  hdmi_wr1(HDMI_FC_HSYNCINDELAY0, hfp);

  /* Set up VSYNC active edge delay (in pixel clks) */

  vfp = sc->sc_mode.vsync_start - sc->sc_mode.vdisplay;
  hdmi_wr1(HDMI_FC_VSYNCINDELAY, vfp);

  hsync_len = (sc->sc_mode.hsync_end - sc->sc_mode.hsync_start);

  /* Set up HSYNC active pulse width (in pixel clks) */

  hdmi_wr1(HDMI_FC_HSYNCINWIDTH1, hsync_len >> 8);
  hdmi_wr1(HDMI_FC_HSYNCINWIDTH0, hsync_len);

  /* Set up VSYNC active edge delay (in pixel clks) */

  hdmi_wr1(HDMI_FC_VSYNCINWIDTH,
           sc->sc_mode.vsync_end - sc->sc_mode.vsync_start);
}

static void
dwc_hdmi_phy_enable_power(uint8_t enable)
{
  uint8_t reg;

  reg = hdmi_rd1(HDMI_PHY_CONF0);
  reg &= ~HDMI_PHY_CONF0_PDZ_MASK;
  reg |= (enable << HDMI_PHY_CONF0_PDZ_OFFSET);
  hdmi_wr1(HDMI_PHY_CONF0, reg);
}

static void
dwc_hdmi_phy_enable_tmds(uint8_t enable)
{
  uint8_t reg;

  reg = hdmi_rd1(HDMI_PHY_CONF0);
  reg &= ~HDMI_PHY_CONF0_ENTMDS_MASK;
  reg |= (enable << HDMI_PHY_CONF0_ENTMDS_OFFSET);
  hdmi_wr1(HDMI_PHY_CONF0, reg);
}

static void
dwc_hdmi_phy_gen2_pddq(uint8_t enable)
{
  uint8_t reg;

  reg = hdmi_rd1(HDMI_PHY_CONF0);
  reg &= ~HDMI_PHY_CONF0_GEN2_PDDQ_MASK;
  reg |= (enable << HDMI_PHY_CONF0_GEN2_PDDQ_OFFSET);
  hdmi_wr1(HDMI_PHY_CONF0, reg);
}

static void
dwc_hdmi_phy_gen2_txpwron(uint8_t enable)
{
  uint8_t reg;

  reg = hdmi_rd1(HDMI_PHY_CONF0);
  reg &= ~HDMI_PHY_CONF0_GEN2_TXPWRON_MASK;
  reg |= (enable << HDMI_PHY_CONF0_GEN2_TXPWRON_OFFSET);
  hdmi_wr1(HDMI_PHY_CONF0, reg);
}

static void
dwc_hdmi_phy_sel_data_en_pol(uint8_t enable)
{
  uint8_t reg;

  reg = hdmi_rd1(HDMI_PHY_CONF0);
  reg &= ~HDMI_PHY_CONF0_SELDATAENPOL_MASK;
  reg |= (enable << HDMI_PHY_CONF0_SELDATAENPOL_OFFSET);
  hdmi_wr1(HDMI_PHY_CONF0, reg);
}

static void
dwc_hdmi_phy_sel_interface_control(uint8_t enable)
{
  uint8_t reg;

  reg = hdmi_rd1(HDMI_PHY_CONF0);
  reg &= ~HDMI_PHY_CONF0_SELDIPIF_MASK;
  reg |= (enable << HDMI_PHY_CONF0_SELDIPIF_OFFSET);
  hdmi_wr1(HDMI_PHY_CONF0, reg);
}

static inline void
dwc_hdmi_phy_test_clear(unsigned char bit)
{
  uint8_t val;

  val = hdmi_rd1(HDMI_PHY_TST0);
  val &= ~HDMI_PHY_TST0_TSTCLR_MASK;
  val |= (bit << HDMI_PHY_TST0_TSTCLR_OFFSET) &
    HDMI_PHY_TST0_TSTCLR_MASK;
  hdmi_wr1(HDMI_PHY_TST0, val);
}

static void dwc_hdmi_clear_overflow(void)
{
  int count;
  uint8_t val;

  /* TMDS software reset */

  hdmi_wr1(HDMI_MC_SWRSTZ, (uint8_t)~HDMI_MC_SWRSTZ_TMDSSWRST_REQ);

  val = hdmi_rd1(HDMI_FC_INVIDCONF);

  for (count = 0 ; count < 4 ; count++)
    hdmi_wr1(HDMI_FC_INVIDCONF, val);
}

static int
dwc_hdmi_phy_configure(int dot_clock)
{
  uint8_t val;
  uint8_t msec;

  hdmi_wr1(HDMI_MC_FLOWCTRL, HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_BYPASS);

  /* gen2 tx power off */

  dwc_hdmi_phy_gen2_txpwron(0);

  /* gen2 pddq */

  dwc_hdmi_phy_gen2_pddq(1);

  /* PHY reset */

  hdmi_wr1(HDMI_MC_PHYRSTZ, HDMI_MC_PHYRSTZ_DEASSERT);
  hdmi_wr1(HDMI_MC_PHYRSTZ, HDMI_MC_PHYRSTZ_ASSERT);

  hdmi_wr1(HDMI_MC_HEACPHY_RST, HDMI_MC_HEACPHY_RST_ASSERT);

  dwc_hdmi_phy_test_clear(1);
  hdmi_wr1(HDMI_PHY_I2CM_SLAVE_ADDR, HDMI_PHY_I2CM_SLAVE_ADDR_PHY_GEN2);
  dwc_hdmi_phy_test_clear(0);

  /* Following initialization are for 8bit per color case */

  /* PLL/MPLL config, see section 24.7.22 in TRM
   * config, see section 24.7.22
   */

  if (dot_clock * 1000 <= 45250000)
    {
      dwc_hdmi_phy_i2c_write(CPCE_CTRL_45_25, HDMI_PHY_I2C_CPCE_CTRL);
      dwc_hdmi_phy_i2c_write(GMPCTRL_45_25, HDMI_PHY_I2C_GMPCTRL);
    }
  else if (dot_clock * 1000 <= 92500000)
    {
      dwc_hdmi_phy_i2c_write(CPCE_CTRL_92_50, HDMI_PHY_I2C_CPCE_CTRL);
      dwc_hdmi_phy_i2c_write(GMPCTRL_92_50, HDMI_PHY_I2C_GMPCTRL);
    }
  else if (dot_clock * 1000 <= 185000000)
    {
      dwc_hdmi_phy_i2c_write(CPCE_CTRL_185, HDMI_PHY_I2C_CPCE_CTRL);
      dwc_hdmi_phy_i2c_write(GMPCTRL_185, HDMI_PHY_I2C_GMPCTRL);
    }
  else
    {
      dwc_hdmi_phy_i2c_write(CPCE_CTRL_370, HDMI_PHY_I2C_CPCE_CTRL);
      dwc_hdmi_phy_i2c_write(GMPCTRL_370, HDMI_PHY_I2C_GMPCTRL);
    }

  /* Values described in TRM section 34.9.2 PLL/MPLL Generic
   * Configuration Settings. Table 34-23.
   */

  if (dot_clock * 1000 <= 54000000)
    dwc_hdmi_phy_i2c_write(0x091c, HDMI_PHY_I2C_CURRCTRL);
  else if (dot_clock * 1000 <= 58400000)
    dwc_hdmi_phy_i2c_write(0x091c, HDMI_PHY_I2C_CURRCTRL);
  else if (dot_clock * 1000 <= 72000000)
    dwc_hdmi_phy_i2c_write(0x06dc, HDMI_PHY_I2C_CURRCTRL);
  else if (dot_clock * 1000 <= 74250000)
    dwc_hdmi_phy_i2c_write(0x06dc, HDMI_PHY_I2C_CURRCTRL);
  else if (dot_clock * 1000 <= 118800000)
    dwc_hdmi_phy_i2c_write(0x091c, HDMI_PHY_I2C_CURRCTRL);
  else if (dot_clock * 1000 <= 216000000)
    dwc_hdmi_phy_i2c_write(0x06dc, HDMI_PHY_I2C_CURRCTRL);
  else
    {
      /* Unsupported mode */

      return -EINVAL;
    }

  dwc_hdmi_phy_i2c_write(0x0000, HDMI_PHY_I2C_PLLPHBYCTRL);
  dwc_hdmi_phy_i2c_write(MSM_CTRL_FB_CLK, HDMI_PHY_I2C_MSM_CTRL);

  /* RESISTANCE TERM 133 Ohm */

  dwc_hdmi_phy_i2c_write(TXTERM_133, HDMI_PHY_I2C_TXTERM);

  /* REMOVE CLK TERM */

  dwc_hdmi_phy_i2c_write(CKCALCTRL_OVERRIDE, HDMI_PHY_I2C_CKCALCTRL);

  if (dot_clock * 1000 > 148500000)
    {
      dwc_hdmi_phy_i2c_write(CKSYMTXCTRL_OVERRIDE | CKSYMTXCTRL_TX_SYMON |
          CKSYMTXCTRL_TX_TRBON | CKSYMTXCTRL_TX_CK_SYMON,
          HDMI_PHY_I2C_CKSYMTXCTRL);
      dwc_hdmi_phy_i2c_write(VLEVCTRL_TX_LVL(9) | VLEVCTRL_CK_LVL(9),
          HDMI_PHY_I2C_VLEVCTRL);
    }
  else
    {
      dwc_hdmi_phy_i2c_write(CKSYMTXCTRL_OVERRIDE | CKSYMTXCTRL_TX_SYMON |
          CKSYMTXCTRL_TX_TRAON | CKSYMTXCTRL_TX_CK_SYMON,
          HDMI_PHY_I2C_CKSYMTXCTRL);
      dwc_hdmi_phy_i2c_write(VLEVCTRL_TX_LVL(13) | VLEVCTRL_CK_LVL(13),
          HDMI_PHY_I2C_VLEVCTRL);
    }

  dwc_hdmi_phy_enable_power(1);

  /* toggle TMDS enable */

  dwc_hdmi_phy_enable_tmds(0);
  dwc_hdmi_phy_enable_tmds(1);

  /* gen2 tx power on */

  dwc_hdmi_phy_gen2_txpwron(1);
  dwc_hdmi_phy_gen2_pddq(0);

  /* Wait for PHY PLL lock */

  msec = 4;
  val = hdmi_rd1(HDMI_PHY_STAT0) & HDMI_PHY_TX_PHY_LOCK;
  while (val == 0)
    {
      up_mdelay(1);
      if (msec-- == 0)
        {
          lcderr("PHY PLL not locked\n");
          return -1;
        }

      val = hdmi_rd1(HDMI_PHY_STAT0) & HDMI_PHY_TX_PHY_LOCK;
    }

  return OK;
}

static int
dwc_hdmi_phy_init(int dot_clock)
{
  int i;
  int ret;

  /* HDMI Phy spec says to do the phy initialization sequence twice */

  for (i = 0 ; i < 2 ; i++)
    {
      dwc_hdmi_phy_sel_data_en_pol(1);
      dwc_hdmi_phy_sel_interface_control(0);
      dwc_hdmi_phy_enable_tmds(0);
      dwc_hdmi_phy_enable_power(0);

      /* Enable CSC */

      ret = dwc_hdmi_phy_configure(dot_clock);
    }

  return ret;
}

static void dwc_hdmi_enable_video_path(void)
{
  uint8_t clkdis;

  /* Control period timing
   * Values are minimal according to HDMI spec 1.4a
   */

  hdmi_wr1(HDMI_FC_CTRLDUR, 12);
  hdmi_wr1(HDMI_FC_EXCTRLDUR, 32);
  hdmi_wr1(HDMI_FC_EXCTRLSPAC, 1);

  /* Bits to fill data lines not used to transmit preamble
   * for channels 0, 1, and 2 respectively
   */

  hdmi_wr1(HDMI_FC_CH0PREAM, 0x0b);
  hdmi_wr1(HDMI_FC_CH1PREAM, 0x16);
  hdmi_wr1(HDMI_FC_CH2PREAM, 0x21);

  /* Save CEC clock */

  clkdis = hdmi_rd1(HDMI_MC_CLKDIS) & HDMI_MC_CLKDIS_CECCLK_DISABLE;
  clkdis |= ~HDMI_MC_CLKDIS_CECCLK_DISABLE;

  /* Enable pixel clock and tmds data path */

  clkdis &= ~HDMI_MC_CLKDIS_PIXELCLK_DISABLE;
  hdmi_wr1(HDMI_MC_CLKDIS, clkdis);

  clkdis &= ~HDMI_MC_CLKDIS_TMDSCLK_DISABLE;
  hdmi_wr1(HDMI_MC_CLKDIS, clkdis);
}

static void
dwc_hdmi_video_packetize(void)
{
  unsigned int color_depth = 0;
  unsigned int remap_size = HDMI_VP_REMAP_YCC422_16BIT;
  unsigned int output_select = HDMI_VP_CONF_OUTPUT_SELECTOR_PP;
  uint8_t val;

  output_select = HDMI_VP_CONF_OUTPUT_SELECTOR_BYPASS;
  color_depth = 4;

  /* set the packetizer registers */

  val = ((color_depth << HDMI_VP_PR_CD_COLOR_DEPTH_OFFSET) &
    HDMI_VP_PR_CD_COLOR_DEPTH_MASK);
  hdmi_wr1(HDMI_VP_PR_CD, val);

  val = hdmi_rd1(HDMI_VP_STUFF);
  val &= ~HDMI_VP_STUFF_PR_STUFFING_MASK;
  val |= HDMI_VP_STUFF_PR_STUFFING_STUFFING_MODE;
  hdmi_wr1(HDMI_VP_STUFF, val);

  val = hdmi_rd1(HDMI_VP_CONF);
  val &= ~(HDMI_VP_CONF_PR_EN_MASK |
    HDMI_VP_CONF_BYPASS_SELECT_MASK);
  val |= HDMI_VP_CONF_PR_EN_DISABLE |
    HDMI_VP_CONF_BYPASS_SELECT_VID_PACKETIZER;
  hdmi_wr1(HDMI_VP_CONF, val);

  val = hdmi_rd1(HDMI_VP_STUFF);
  val &= ~HDMI_VP_STUFF_IDEFAULT_PHASE_MASK;
  val |= 1 << HDMI_VP_STUFF_IDEFAULT_PHASE_OFFSET;
  hdmi_wr1(HDMI_VP_STUFF, val);

  hdmi_wr1(HDMI_VP_REMAP, remap_size);

  if (output_select == HDMI_VP_CONF_OUTPUT_SELECTOR_PP)
    {
      val = hdmi_rd1(HDMI_VP_CONF);
      val &= ~(HDMI_VP_CONF_BYPASS_EN_MASK |
        HDMI_VP_CONF_PP_EN_ENMASK |
        HDMI_VP_CONF_YCC422_EN_MASK);
      val |= HDMI_VP_CONF_BYPASS_EN_DISABLE |
        HDMI_VP_CONF_PP_EN_ENABLE |
        HDMI_VP_CONF_YCC422_EN_DISABLE;
      hdmi_wr1(HDMI_VP_CONF, val);
    }
  else if (output_select == HDMI_VP_CONF_OUTPUT_SELECTOR_YCC422)
    {
      val = hdmi_rd1(HDMI_VP_CONF);
      val &= ~(HDMI_VP_CONF_BYPASS_EN_MASK |
        HDMI_VP_CONF_PP_EN_ENMASK |
        HDMI_VP_CONF_YCC422_EN_MASK);
      val |= HDMI_VP_CONF_BYPASS_EN_DISABLE |
        HDMI_VP_CONF_PP_EN_DISABLE |
        HDMI_VP_CONF_YCC422_EN_ENABLE;
      hdmi_wr1(HDMI_VP_CONF, val);
    }
  else if (output_select == HDMI_VP_CONF_OUTPUT_SELECTOR_BYPASS)
    {
      val = hdmi_rd1(HDMI_VP_CONF);
      val &= ~(HDMI_VP_CONF_BYPASS_EN_MASK |
        HDMI_VP_CONF_PP_EN_ENMASK |
        HDMI_VP_CONF_YCC422_EN_MASK);
      val |= HDMI_VP_CONF_BYPASS_EN_ENABLE |
        HDMI_VP_CONF_PP_EN_DISABLE |
        HDMI_VP_CONF_YCC422_EN_DISABLE;
      hdmi_wr1(HDMI_VP_CONF, val);
    }
  else
    {
      return;
    }

  val = hdmi_rd1(HDMI_VP_STUFF);
  val &= ~(HDMI_VP_STUFF_PP_STUFFING_MASK |
    HDMI_VP_STUFF_YCC422_STUFFING_MASK);
  val |= HDMI_VP_STUFF_PP_STUFFING_STUFFING_MODE |
    HDMI_VP_STUFF_YCC422_STUFFING_STUFFING_MODE;
  hdmi_wr1(HDMI_VP_STUFF, val);

  val = hdmi_rd1(HDMI_VP_CONF);
  val &= ~HDMI_VP_CONF_OUTPUT_SELECTOR_MASK;
  val |= output_select;
  hdmi_wr1(HDMI_VP_CONF, val);
}

static void dwc_hdmi_video_sample(void)
{
  int color_format;
  uint8_t val;

  color_format = 0x01;
  val = HDMI_TX_INVID0_INTERNAL_DE_GENERATOR_DISABLE |
    ((color_format << HDMI_TX_INVID0_VIDEO_MAPPING_OFFSET) &
    HDMI_TX_INVID0_VIDEO_MAPPING_MASK);
  hdmi_wr1(HDMI_TX_INVID0, val);

  /* Enable TX stuffing: When DE is inactive, fix the output data to 0 */

  val = HDMI_TX_INSTUFFING_BDBDATA_STUFFING_ENABLE |
    HDMI_TX_INSTUFFING_RCRDATA_STUFFING_ENABLE |
    HDMI_TX_INSTUFFING_GYDATA_STUFFING_ENABLE;
  hdmi_wr1(HDMI_TX_INSTUFFING, val);
  hdmi_wr1(HDMI_TX_GYDATA0, 0x0);
  hdmi_wr1(HDMI_TX_GYDATA1, 0x0);
  hdmi_wr1(HDMI_TX_RCRDATA0, 0x0);
  hdmi_wr1(HDMI_TX_RCRDATA1, 0x0);
  hdmi_wr1(HDMI_TX_BCBDATA0, 0x0);
  hdmi_wr1(HDMI_TX_BCBDATA1, 0x0);
}

static void dwc_hdmi_tx_hdcp_config(void)
{
  uint8_t de;
  uint8_t val;

  de = HDMI_A_VIDPOLCFG_DATAENPOL_ACTIVE_HIGH;

  /* Disable RX detect */

  val = hdmi_rd1(HDMI_A_HDCPCFG0);
  val &= ~HDMI_A_HDCPCFG0_RXDETECT_MASK;
  val |= HDMI_A_HDCPCFG0_RXDETECT_DISABLE;
  hdmi_wr1(HDMI_A_HDCPCFG0, val);

  /* Set polarity */

  val = hdmi_rd1(HDMI_A_VIDPOLCFG);
  val &= ~HDMI_A_VIDPOLCFG_DATAENPOL_MASK;
  val |= de;
  hdmi_wr1(HDMI_A_VIDPOLCFG, val);

  /* Disable encryption */

  val = hdmi_rd1(HDMI_A_HDCPCFG1);
  val &= ~HDMI_A_HDCPCFG1_ENCRYPTIONDISABLE_MASK;
  val |= HDMI_A_HDCPCFG1_ENCRYPTIONDISABLE_DISABLE;
  hdmi_wr1(HDMI_A_HDCPCFG1, val);
}

static int
dwc_hdmi_set_mode(struct display_s *sc)
{
  int ret;

  dwc_hdmi_disable_overflow_interrupts();
  dwc_hdmi_av_composer(sc);
  ret = dwc_hdmi_phy_init(sc->sc_mode.dot_clock);
  if (ret != OK)
    {
      return ret;
    }

  dwc_hdmi_enable_video_path();

  dwc_hdmi_video_packetize();
  dwc_hdmi_video_sample();
  dwc_hdmi_tx_hdcp_config();
  dwc_hdmi_clear_overflow();

  return 0;
}

void
dwc_hdmi_init(void)
{
  lcdinfo("HDMI controller %02x:%02x:%02x:%02x\n",
      hdmi_rd1(HDMI_DESIGN_ID), hdmi_rd1(HDMI_REVISION_ID),
      hdmi_rd1(HDMI_PRODUCT_ID0), hdmi_rd1(HDMI_PRODUCT_ID1));

  hdmi_wr1(HDMI_PHY_POL0, HDMI_PHY_POL0_HPD);
  hdmi_wr1(HDMI_IH_PHY_STAT0, HDMI_IH_PHY_STAT0_HPD);
}

static void
jzlcd_start(struct display_s *sc)
{
  uint32_t ctrl;

  /* Clear status registers */

  lcd_write(sc, LCDSTATE, 0);
  lcd_write(sc, LCDOSDS, 0);

  /* Enable the controller */

  ctrl = lcd_read(sc, LCDCTRL);
  ctrl |= LCDCTRL_ENA;
  ctrl &= ~LCDCTRL_DIS;
  lcd_write(sc, LCDCTRL, ctrl);
}

static void
jzlcd_stop(struct display_s *sc)
{
  uint32_t ctrl;

  ctrl = lcd_read(sc, LCDCTRL);
  if ((ctrl & LCDCTRL_ENA) != 0)
    {
      /* Disable the controller and wait for it to stop */

      ctrl |= LCDCTRL_DIS;
      lcd_write(sc, LCDCTRL, ctrl);
      while ((lcd_read(sc, LCDSTATE) & LCDSTATE_LDD) == 0)
        {
          up_mdelay(100);
        }
    }

  /* Clear all status except for disable */

  lcd_write(sc, LCDSTATE, lcd_read(sc, LCDSTATE) & ~LCDSTATE_LDD);
}

static void
jzlcd_setup_descriptor(struct display_s *sc, const struct videomode_s *mode,
    u_int desno)
{
  uint32_t f0xpos = 0;
  uint32_t f0ypos = 0;
  uint32_t f1xpos = 0;
  uint32_t f1ypos = 0;
  uint32_t f0width  = mode->hdisplay;
  uint32_t f0height = mode->vdisplay;
  uint32_t f1width  = mode->hdisplay;
  uint32_t f1height = mode->vdisplay;

  uint32_t bpp;
  bpp = JZ4780_BPP == 16 ? LCDPOS_BPP01_15_16 : LCDPOS_BPP01_24_COMP;

  /* Frame size is specified in # words */

  int line_sz = ((desno ? f1width : f0width) * JZ4780_BPP) >> 3;
  line_sz = ((line_sz + 3) & ~3) / 4;

  struct lcd_frame_descriptor *fdesc = sc->fdesc + desno;
  fdesc->id = desno;
  fdesc->offs = 0;
  fdesc->pw = 0;

  if (desno == 0)
    {
      fdesc->next = sc->fdesc_paddr + sizeof(struct lcd_frame_descriptor);

      fdesc->physaddr = sc->paddr + 0;
      fdesc->cmd = LCDCMD_FRM_EN | (line_sz * f0height);
      fdesc->cnum_pos = bpp | LCDPOS_PREMULTI01 | (f0ypos << 12) | f0xpos |
          LCDPOS_COEF_BLE01_1;
      fdesc->dessize = LCDDESSIZE_ALPHA |
          ((f0height - 1) << LCDDESSIZE_HEIGHT_SHIFT) |
          ((f0width - 1) << LCDDESSIZE_WIDTH_SHIFT);
    }
  else
    {
      fdesc->next = sc->fdesc_paddr;

      fdesc->physaddr = sc->paddr + FOREGROUND_OFFSET;
      fdesc->cmd = LCDCMD_FRM_EN | (line_sz *f1height);
      fdesc->cnum_pos = bpp | LCDPOS_PREMULTI01 | (f1ypos << 12) | f1xpos |
           LCDPOS_COEF_SLE01;
      fdesc->dessize = LCDDESSIZE_ALPHA |
          ((f1height - 1) << LCDDESSIZE_HEIGHT_SHIFT) |
          ((f1width - 1) << LCDDESSIZE_WIDTH_SHIFT);
    }
}

static int
jzlcd_set_videomode(struct display_s *sc, const struct videomode_s *mode)
{
  u_int hbp, hfp, hsw, vbp, vfp, vsw;
  u_int hds, hde, ht, vds, vde, vt;
  uint32_t ctrl;

  hbp = mode->htotal - mode->hsync_end;
  hfp = mode->hsync_start - mode->hdisplay;
  hsw = mode->hsync_end - mode->hsync_start;
  vbp = mode->vtotal - mode->vsync_end;
  vfp = mode->vsync_start - mode->vdisplay;
  vsw = mode->vsync_end - mode->vsync_start;

  hds = hsw + hbp;
  hde = hds + mode->hdisplay;
  ht = hde + hfp;

  vds = vsw + vbp;
  vde = vds + mode->vdisplay;
  vt = vde + vfp;

  /* Setup timings */

  lcd_write(sc, LCDVAT,
      (ht << LCDVAT_HT_SHIFT) | (vt << LCDVAT_VT_SHIFT));
  lcd_write(sc, LCDDAH,
      (hds << LCDDAH_HDS_SHIFT) | (hde << LCDDAH_HDE_SHIFT));
  lcd_write(sc, LCDDAV,
      (vds << LCDDAV_VDS_SHIFT) | (vde << LCDDAV_VDE_SHIFT));
  lcd_write(sc, LCDHSYNC, hsw);
  lcd_write(sc, LCDVSYNC, vsw);

  /* Set configuration */

  lcd_write(sc, LCDCFG, LCDCFG_NEWDES | LCDCFG_RECOVER | LCDCFG_24 |
      LCDCFG_PSM | LCDCFG_CLSM | LCDCFG_SPLM | LCDCFG_REVM | LCDCFG_PCP);
  ctrl = lcd_read(sc, LCDCTRL);
  ctrl &= ~LCDCTRL_BST;
  ctrl |= LCDCTRL_BST_64 | LCDCTRL_OFUM;
  lcd_write(sc, LCDCTRL, ctrl);
  lcd_write(sc, LCDPCFG, PCFG_MAGIC);
  lcd_write(sc, LCDRGBC, LCDRGBC_RGBFMT);

  /* Update registers */

  lcd_write(sc, LCDSTATE, 0);

  /* Setup frame descriptors */

  jzlcd_setup_descriptor(sc, mode, 0);
  jzlcd_setup_descriptor(sc, mode, 1);

  /* Setup DMA channels */

  lcd_write(sc, LCDDA0, sc->fdesc_paddr
      + sizeof(struct lcd_frame_descriptor));
  lcd_write(sc, LCDDA1, sc->fdesc_paddr);

  /* Set display clock */

  putreg32(LPCS_VPLL | CE_LCD | LPCDR_VAL, LP1CDR_REG);
  while (getreg32(LP1CDR_REG) & LCD_BUSY)
    {
    }

  return 0;
}

static int
jzlcd_configure(struct display_s *sc, const struct videomode_s *mode)
{
  uint32_t vaddr0 = (uint32_t)JZ4780_LCD_VRAMBASE;

  sc->fbsize = 0x00300000;
  sc->vaddr  = vaddr0;
  sc->paddr  = jz_physramaddr(vaddr0);
  sc->fdesc  = (struct lcd_frame_descriptor *)(sc->vaddr + sc->fbsize);
  sc->fdesc_paddr = sc->paddr + sc->fbsize;

  /* Setup video mode */

  int error = jzlcd_set_videomode(sc, mode);
  if (error != 0)
    {
      return error;
    }

  return 0;
}

static void
jzlcd_hdmi_event(struct display_s *sc)
{
  /* Stop the controller */

  jzlcd_stop(sc);

  /* Configure LCD controller */

  int error = jzlcd_configure(sc, &sc->sc_mode);
  if (error != 0)
    {
      lcderr("failed to configure FB: %d\n", error);
      return;
    }

  error = dwc_hdmi_set_mode(sc);
  if (error != 0)
    {
      lcderr("hdmi error: %d\n", error);
      return;
    }

  /* Start the controller! */

  jzlcd_start(sc);
}

void jzlcd_clocks(void)
{
  uint32_t val;

  putreg32(0xa0000020, HDMICDR_REG);

  val = getreg32(CLKGR1_REG);
  val &= ~CLKGR1_HDMI;
  putreg32(val, CLKGR1_REG);
}

int jzlcd_attach(void)
{
  uint32_t val;
  static struct display_s sc_obj = {
    .fbsize = 0,
    .paddr = 0,
    .vaddr = 0,
    .fdesc_paddr = 0,
    .fdesc = NULL,

    .sc_mode = {
      .dot_clock = 85500,
      .hdisplay = 1360,
      .hsync_start = 1424,
      .hsync_end = 1536,
      .htotal = 1792,
      .vdisplay = 768,
      .vsync_start = 771,
      .vsync_end = 777,
      .vtotal = 795,
      .flags = 5,
    },
  };

  val = getreg32(CLKGR0_REG);
  val &= ~CLKGR0_LCD;
  val &= ~CLKGR0_TVE;
  putreg32(val, CLKGR0_REG);

  jzlcd_hdmi_event(&sc_obj);
  return 1;
}

/****************************************************************************
 * Name: jz_getvideoinfo
 ****************************************************************************/

static int jz_getvideoinfo(struct fb_vtable_s *vtable,
                              struct fb_videoinfo_s *vinfo)
{
  lcdinfo("vtable=%p vinfo=%p\n", vtable, vinfo);
  if (vtable && vinfo)
    {
      memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: jz_getplaneinfo
 ****************************************************************************/

static int jz_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                              struct fb_planeinfo_s *pinfo)
{
  lcdinfo("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);
  if (vtable && planeno == 0 && pinfo)
    {
      memcpy(pinfo, &g_planeinfo, sizeof(struct fb_planeinfo_s));
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: jz_getcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int jz_getcmap(struct fb_vtable_s *vtable,
                         struct fb_cmap_s *cmap)
{
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: jz_putcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int jz_putcmap(struct fb_vtable_s *vtable,
                         const struct fb_cmap_s *cmap)
{
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: jz_getcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int jz_getcursor(struct fb_vtable_s *vtable,
                           struct fb_cursorattrib_s *attrib)
{
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: jz_setcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int jz_setcursor(struct fb_vtable_s *vtable,
                           struct fb_setcursor_s *settings)
{
  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the framebuffer video hardware associated with the display.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  jzlcd_clocks();

  dwc_hdmi_init();

  jzlcd_attach();

  return OK;
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane of the specified plane.  Many OSDs support multiple planes of
 *   video.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *   vplane - Identifies the plane being queried.
 *
 * Returned Value:
 *   A non-NULL pointer to the frame buffer access structure is returned on
 *   success; NULL is returned on any failure.
 *
 ****************************************************************************/

struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  lcdinfo("vplane: %d\n", vplane);
  if (vplane == 0)
    {
      return &g_fbobject;
    }
  else
    {
      return NULL;
    }
}

/****************************************************************************
 * Name: up_fbuninitialize
 *
 * Description:
 *   Uninitialize the framebuffer support for the specified display.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_fbuninitialize(int display)
{
}

#endif /* CONFIG_ALLOW_BSD_COMPONENTS */

/********************************************************************************************
 * include/nuttx/lcd/ft80x.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *  - Document No.: FT_000792, "FT800 Embedded Video Engine", Datasheet
 *    Version 1.1, Clearance No.: FTDI# 334, Future Technology Devices
 *    International Ltd.
 *  - Document No.: FT_000986, "FT801 Embedded Video Engine Datasheet",
 *    Version 1.0, Clearance No.: FTDI#376, Future Technology Devices
 *    International Ltd.
 *  - Application Note AN_240AN_240, "FT800 From the Ground Up", Version
 *    1.1, Issue Date: 2014-06-09, Future Technology Devices International
 *    Ltd.
 *  - Some definitions derive from FTDI sample code.
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_LCD_FT80X_H
#define __INCLUDE_NUTTX_LCD_FT80X_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/lcd/lcd_ioctl.h>

#ifdef CONFIG_LCD_FT80X

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* Configuration ****************************************************************************/

#ifdef CONFIG_DISABLE_SIGNALS
#  error Signal support is required by this driver
#endif

#if defined(CONFIG_LCD_FT80X_WQVGA)
#  define FT80X_DISPLAY_WIDTH   480
#  define FT80X_DISPLAY_HEIGHT  272
#elif defined(CONFIG_LCD_FT80X_QVGA)
#  define FT80X_DISPLAY_WIDTH   320
#  define FT80X_DISPLAY_HEIGHT  240
#else
#  error Unknown display size
#endif

/* FT80x IOCTL commands *********************************************************************
 *
 * FT80X_IOC_CREATEDL:
 *   Description:  Write a display list to the FT80x display list memory
 *                 starting at offset zero.  This may or may not be the
 *                 entire display list.  Display lists may be created
 *                 incrementally, starting with FT80X_IOC_CREATEDL and
 *                 finishing the display list using FT80XIO_APPENDDL
 *   Argument:     A reference to a display list structure instance.  See
 *                 struct ft80x_displaylist_s below.
 *   Returns:      None
 *
 * FT80X_IOC_APPENDDL:
 *   Description:  Write additional display list entries to the FT80x
 *                 display list memory at the current display list offset.
 *                 This IOCTL command permits display lists to be completed
 *                 incrementally, starting with FT80X_IOC_CREATEDL and
 *                 finishing display list using FT80XIO_APPENDDL.
 *   Argument:     A reference to a display list structure instance.  See
 *                 struct ft80x_displaylist_s below.
 *   Returns:      None
 *
 * These two IOCTL command simply copy the display list as provided into the
 * FT80x display list memory.  Display lists should generally be formed as
 * follows:
 *
 *                                        # Various display commands ...
 *   FT80X_DISPLAY();                     # Terminate the display list
 *
 * Then write to the REG_DLSWAP register to set REG_swap to the new display list.
 *
 * NOTE: The functionality of FT80X_IOC_CREATEDL is the equivalent to that of
 * the driver write() method.  Either the write() method or the FT80X_IOC_CREATEDL
 * IOCTL command can be used to create the display list.
 *
 * The difference between appending and create a display list using write()
 * is that it is necessary to lseek() to the beginning of the display list
 * to create a new display list.  Subsequent writes will behave then append
 * to the end of the display list.
 *
 * Need to know the current display list offset?  You can set that using lseek()
 * too.  Just seek to the current position; the returned value will be the current
 * display list offset.
 *
 * Output values from display commands are not automatically written back in
 * either case but must be subsequently obtained using FT80X_IOC_GETRAMDL.
 *
 * FT80X_IOC_GETRAMDL:
 *   Description:  Read a 32-bit aligned data from the display list.
 *   Argument:     A reference to an instance of struct ft80x_relmem_s below.
 *   Returns:      The 32-bit value read from the display list.
 *
 * FT80X_IOC_PUTRAMG
 *   Description:  Write byte data to FT80x graphics memory (RAM_G)
 *   Argument:     A reference to an instance of struct ft80x_relmem_s below.
 *   Returns:      None.
 *
 * FT80X_IOC_PUTRAMCMD
 *   Description:  Write 32-bit aligned data to FT80x FIFO (RAM_CMD)
 *   Argument:     A reference to an instance of struct ft80x_relmem_s below.
 *   Returns:      None.
 *
 * FT80X_IOC_GETREG8:
 *   Description:  Read an 8-bit register value from the FT80x.
 *   Argument:     A reference to an instance of struct ft80x_register_s below.
 *   Returns:      The 8-bit value read from the register.
 *
 * FT80X_IOC_GETREG16:
 *   Description:  Read a 16-bit register value from the FT80x.
 *   Argument:     A reference to an instance of struct ft80x_register_s below.
 *   Returns:      The 16-bit value read from the register.
 *
 * FT80X_IOC_GETREG32:
 *   Description:  Read a 32-bit register value from the FT80x.
 *   Argument:     A reference to an instance of struct ft80x_register_s below.
 *   Returns:      The 32-bit value read from the register.
 *
 * FT80X_IOC_GETREGS:
 *   Description:  Read multiple 32-bit register values from the FT80x.
 *   Argument:     A reference to an instance of struct ft80x_registers_s below.
 *   Returns:      The 32-bit values read from the consecutive registers .
 *
 * FT80X_IOC_PUTREG8:
 *   Description:  Write an 8-bit register value to the FT80x.
 *   Argument:     A reference to an instance of struct ft80x_register_s below.
 *   Returns:      None.
 *
 * FT80X_IOC_PUTREG16:
 *   Description:  Write a 16-bit register value to the FT80x.
 *   Argument:     A reference to an instance of struct ft80x_register_s below.
 *   Returns:      None.
 *
 * FT80X_IOC_PUTREG32:
 *   Description:  Write a 32-bit register value to the FT80x.
 *   Argument:     A reference to an instance of struct ft80x_register_s below.
 *   Returns:      None.
 *
 * FT80X_IOC_PUTREGS:
 *   Description:  Write multiple 32-bit register values to the FT80x.
 *   Argument:     A reference to an instance of struct ft80x_registers_s below.
 *   Returns:      None.
 *
 * FT80X_IOC_EVENTNOTIFY:
 *   Description:  Setup to receive a signal when there is a change in any
 *                 touch tag value.  Additional information may be provided in
 *                 the signinfo.si_val file of the notification:
 *
 *                 For touch tag events, siginfo.si_value will indicate the
 *                 touch tag.  For the FT801 in extended mode, it will
 *                 indicate only the tag value for TOUCH0.
 *   Argument:     A reference to an instance of struct ft80x_notify_s.
 *   Returns:      None
 *
 * FT80X_IOC_FADE:
 *   Description:  Change the backlight intensity with a controllable fade.
 *   Argument:     A reference to an instance of struct ft80x_fade_s below.
 *   Returns:      None.
 *
 * FT80X_IOC_AUDIO:
 *   Description:  Enable/disable an external audio amplifer.
 *   Argument:     0=disable; 1=enable.
 *   Returns:      None.
 */

#define FT80X_IOC_CREATEDL          _LCDIOC(FT80X_NIOCTL_BASE + 0)
#define FT80X_IOC_APPENDDL          _LCDIOC(FT80X_NIOCTL_BASE + 1)
#define FT80X_IOC_GETRAMDL          _LCDIOC(FT80X_NIOCTL_BASE + 2)
#define FT80X_IOC_PUTRAMG           _LCDIOC(FT80X_NIOCTL_BASE + 3)
#define FT80X_IOC_PUTRAMCMD         _LCDIOC(FT80X_NIOCTL_BASE + 4)
#define FT80X_IOC_GETREG8           _LCDIOC(FT80X_NIOCTL_BASE + 5)
#define FT80X_IOC_GETREG16          _LCDIOC(FT80X_NIOCTL_BASE + 6)
#define FT80X_IOC_GETREG32          _LCDIOC(FT80X_NIOCTL_BASE + 7)
#define FT80X_IOC_GETREGS           _LCDIOC(FT80X_NIOCTL_BASE + 8)
#define FT80X_IOC_PUTREG8           _LCDIOC(FT80X_NIOCTL_BASE + 9)
#define FT80X_IOC_PUTREG16          _LCDIOC(FT80X_NIOCTL_BASE + 10)
#define FT80X_IOC_PUTREG32          _LCDIOC(FT80X_NIOCTL_BASE + 11)
#define FT80X_IOC_PUTREGS           _LCDIOC(FT80X_NIOCTL_BASE + 12)
#define FT80X_IOC_EVENTNOTIFY       _LCDIOC(FT80X_NIOCTL_BASE + 13)
#define FT80X_IOC_FADE              _LCDIOC(FT80X_NIOCTL_BASE + 14)
#define FT80X_IOC_AUDIO             _LCDIOC(FT80X_NIOCTL_BASE + 15)

/* FT80x Memory Map *************************************************************************/

/* Address region */

#define FT80X_RAM_G                    0x000000  /* Main graphics RAM (256Kb) */
#define FT80X_ROM_CHIPID               0x0c0000  /* FT80x chip identification and revision
                                                  * information (4b):
                                                  * Byte [0:1] Chip ID: 0800 or 0801
                                                  * Byte [2:3] Version ID: 0100 */
#define FT80X_ROM_FONT                 0x0bb23c  /* Font table and bitmap (275Kb) */
#define FT80X_ROM_FONT_ADDR            0x0ffffc  /* Font table pointer address (4b) */
#define FT80X_RAM_DL                   0x100000  /* Display List RAM (8Kb) */
#define FT80X_RAM_PAL                  0x102000  /* Palette RAM (1Kb) */
#define FT80X_REG                      0x102400  /* Registers (380b) */
#define FT80X_RAM_CMD                  0x108000  /* Command Buffer (4Kb) */

#ifdef CONFIG_LCD_FT801
#  define FT80X_RAM_SCREENSHOT         0x1c2000  /* Screenshot readout buffer  (2Kb) */
#endif

/* Memory buffer sizes */

#define FT80X_RAM_G_SIZE               (256 * 1024)
#define FT80X_CMDFIFO_SIZE             (4 * 1024)
#define FT80X_RAM_DL_SIZE              (8 * 1024)
#define FT80X_RAM_PAL_SIZE             (1 * 1024)

/* FT80x Register Addresses *****************************************************************/

#define FT80X_REG_ID                   0x102400  /* Identification register, always reads as 7c */
#define FT80X_REG_FRAMES               0x102404  /* Frame counter, since reset */
#define FT80X_REG_CLOCK                0x102408  /* Clock cycles, since reset */
#define FT80X_REG_FREQUENCY            0x10240c  /* Main clock frequency */

#if defined(CONFIG_LCD_FT800)
#  define FT80X_REG_RENDERMODE         0x102410  /* Rendering mode: 0 = normal, 1 = single-line */
#  define FT80X_REG_SNAPY              0x102414  /* Scan line select for RENDERMODE 1 */
#  define FT80X_REG_SNAPSHOT           0x102418  /* Trigger for RENDERMODE 1 */
#elif defined(CONFIG_LCD_FT801)
#  define FT80X_REG_SCREENSHOT_EN      0x102410  /* Set to enable screenshot mode */
#  define FT80X_REG_SCREENSHOT_Y       0x102414  /* Y line number for screenshot */
#  define FT80X_REG_SCREENSHOT_START   0x102418  /* Screenshot start trigger */
#endif

#define FT80X_REG_CPURESET             0x10241c  /* Graphics, audio and touch engines reset
                                                  * control */
#define FT80X_REG_TAP_CRC              0x102420  /* Live video tap crc. Frame CRC is computed
                                                  * every DL SWAP. */
#define FT80X_REG_TAP_MASK             0x102424  /* Live video tap mask */
#define FT80X_REG_HCYCLE               0x102428  /* Horizontal total cycle count */
#define FT80X_REG_HOFFSET              0x10242c  /* Horizontal display start offset */
#define FT80X_REG_HSIZE                0x102430  /* Horizontal display pixel count */
#define FT80X_REG_HSYNC0               0x102434  /* Horizontal sync fall offset */
#define FT80X_REG_HSYNC1               0x102438  /* Horizontal sync rise offset */
#define FT80X_REG_VCYCLE               0x10243c  /* Vertical total cycle count */
#define FT80X_REG_VOFFSET              0x102440  /* Vertical display start offset */
#define FT80X_REG_VSIZE                0x102444  /* Vertical display line count */
#define FT80X_REG_VSYNC0               0x102448  /* Vertical sync fall offset */
#define FT80X_REG_VSYNC1               0x10244c  /* Vertical sync rise offset */
#define FT80X_REG_DLSWAP               0x102450  /* Display list swap control */
#define FT80X_REG_ROTATE               0x102454  /* Screen 180 degree rotate */
#define FT80X_REG_OUTBITS              0x102458  /* Output bit resolution, 3x3x3 bits */
#define FT80X_REG_DITHER               0x10245c  /* Output dither enable */
#define FT80X_REG_SWIZZLE              0x102460  /* Output RGB signal swizzle */
#define FT80X_REG_CSPREAD              0x102464  /* Output clock spreading enable */
#define FT80X_REG_PCLK_POL             0x102468  /* PCLK polarity: 0=rising edge, 1= falling edge */
#define FT80X_REG_PCLK                 0x10246c  /* PCLK frequency divider, 0 = disable */
#define FT80X_REG_TAG_X                0x102470  /* Tag query X coordinate */
#define FT80X_REG_TAG_Y                0x102474  /* Tag query Y coordinate */
#define FT80X_REG_TAG                  0x102478  /* Tag query result */
#define FT80X_REG_VOL_PB               0x10247c  /* Volume for playback */
#define FT80X_REG_VOL_SOUND            0x102480  /* Volume for synthesizer sound */
#define FT80X_REG_SOUND                0x102484  /* Sound effect select */
#define FT80X_REG_PLAY                 0x102488  /* Start effect playback */
#define FT80X_REG_GPIO_DIR             0x10248c  /* GPIO pin direction, 0=input, 1=output */
#define FT80X_REG_GPIO                 0x102490  /* Pin value (bits 0,1,7); Drive strength
                                                  * (bits 2-6) */
                                                 /* 0x102494 Reserved */
#define FT80X_REG_INT_FLAGS            0x102498  /* Interrupt flags, clear by read */
#define FT80X_REG_INT_EN               0x10249c  /* Global interrupt enable */
#define FT80X_REG_INT_MASK             0x1024a0  /* Interrupt enable mask */
#define FT80X_REG_PLAYBACK_START       0x1024a4  /* Audio playback RAM start address */
#define FT80X_REG_PLAYBACK_LENGTH      0x1024a8  /* Audio playback sample length (bytes) */
#define FT80X_REG_PLAYBACK_READPTR     0x1024ac  /* Audio playback current read pointer */
#define FT80X_REG_PLAYBACK_FREQ        0x1024b0  /* Audio playback sampling frequency (Hz) */
#define FT80X_REG_PLAYBACK_FORMAT      0x1024b4  /* Audio playback format */
#define FT80X_REG_PLAYBACK_LOOP        0x1024b8  /* Audio playback loop enable */
#define FT80X_REG_PLAYBACK_PLAY        0x1024bc  /* Start audio playback */
#define FT80X_REG_PWM_HZ               0x1024c0  /* BACKLIGHT PWM output frequency (Hz) */
#define FT80X_REG_PWM_DUTY             0x1024c4  /* BACKLIGHT PWM output duty cycle 0=0%,
                                                  * 128=100% */
#define FT80X_REG_MACRO_0              0x1024c8  /* Display list macro command 0 */
#define FT80X_REG_MACRO_1              0x1024cc  /* Display list macro command 1 */

#if defined(CONFIG_LCD_FT800)
                                             /* 0x1024d0 – 0x1024e0 Reserved */
#elif defined(CONFIG_LCD_FT801)
                                             /* 0x1024d0 – 0x1024d4 Reserved */
#  define FT80X_REG_SCREENSHOT_BUSY    0x1024d8  /* Screenshot ready flags */
                                             /* 0x1024e0 Reserved */
#endif

#define FT80X_REG_CMD_READ             0x1024e4  /* Command buffer read pointer */
#define FT80X_REG_CMD_WRITE            0x1024e8  /* Command buffer write pointer */
#define FT80X_REG_CMD_DL               0x1024ec  /* Command display list offset */
#define FT80X_REG_TOUCH_MODE           0x1024f0  /* Touch-screen sampling mode */

#if defined(CONFIG_LCD_FT800)
#  define FT80X_REG_TOUCH_ADC_MODE     0x1024f4  /* Select single ended (low power) or
                                                  * differential (accurate) sampling */
#  define FT80X_REG_TOUCH_CHARGE       0x1024f8  /* Touch-screen charge time, units of 6 clocks */
#  define FT80X_REG_TOUCH_SETTLE       0x1024fc  /* Touch-screen settle time, units of 6 clocks */
#  define FT80X_REG_TOUCH_OVERSAMPLE   0x102500  /* Touch-screen oversample factor */
#  define FT80X_REG_TOUCH_RZTHRESH     0x102504  /* Touch-screen resistance threshold */
#  define FT80X_REG_TOUCH_RAW_XY       0x102508  /* Touch-screen raw (x-MSB16; y-LSB16) */
#  define FT80X_REG_TOUCH_RZ           0x10250c  /* Touch-screen resistance */
#  define FT80X_REG_TOUCH_SCREEN_XY    0x102510  /* Touch-screen screen (x-MSB16; y-LSB16) */
#  define FT80X_REG_TOUCH_TAG_XY       0x102514  /* Touch-screen screen (x-MSB16; y-LSB16)
                                                  * used for tag lookup */
#  define FT80X_REG_TOUCH_TAG          0x102518  /* Touch-screen tag result */
#  define FT80X_REG_TOUCH_TRANSFORM_A  0x10251c  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_REG_TOUCH_TRANSFORM_B  0x102520  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_REG_TOUCH_TRANSFORM_C  0x102524  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_REG_TOUCH_TRANSFORM_D  0x102528  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_REG_TOUCH_TRANSFORM_E  0x10252c  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_REG_TOUCH_TRANSFORM_F  0x102530  /* Touch-screen transform coefficient (s15.16) */
                                                 /* 0x102534 – 0x102470 Reserved */
#  define FT80X_REG_TOUCH_DIRECT_XY    0x102574  /* Touch screen direct (x-MSB16; y-LSB16)
                                                  * conversions */
#  define FT80X_REG_TOUCH_DIRECT_Z1Z2  0x102578  /* Touch screen direct (z1-MSB16; z2-LSB16)
                                                  * conversions */
#elif defined(CONFIG_LCD_FT801)
                                                 /* 0x1024d0 – 0x1024d4 Reserved */
#  define FT80X_REG_CTOUCH_EXTENDED    0x1024f4  /* Set capacitive touch operation mode:
                                                  * 0: extended mode (multi-touch)
                                                  * 1: FT800 compatibility mode (single touch) */
#  define FT80X_REG_CTOUCH_REG         0x1024f8  /* CTPM configure register write
                                                  * Bits [7:0]: configure register address
                                                  * Bits [15:8]: configure register value */
                                                 /* 0x1024fc - 0x102504 Reserved */
#  define FT80X_REG_CTOUCH_RAW_XY      0x102508  /* Compatibility mode: touch-screen raw
                                                  * (x-MSB16; y-LSB16) */
#  define FT80X_REG_CTOUCH_TOUCH1_XY   0x102508  /* Extended mode: touch-screen screen data for touch 1
                                                  * (x-MSB16; y-LSB16) */
#  define FT80X_REG_CTOUCH_TOUCH4_Y    0x10250c  /* Extended mode: touch-screen screen Y data for touch 4 */
#  define FT80X_REG_CTOUCH_SCREEN_XY   0x102510  /* Compatibility mode: touch-screen screen
                                                  * (x-MSB16; y-LSB16) */
#  define FT80X_REG_CTOUCH_TOUCH0_XY   0x102510  /* Extended mode: touch-screen screen data for touch 0
                                                  * (x-MSB16; y-LSB16) */
#  define FT80X_REG_CTOUCH_TAG_XY      0x102514  /* Touch-screen screen (x-MSB16; y-LSB16)
                                                  * used for tag lookup */
#  define FT80X_REG_CTOUCH_TAG         0x102518  /* Touch-screen tag result */
#  define FT80X_REG_CTOUCH_TRANSFORM_A 0x10251c  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_REG_CTOUCH_TRANSFORM_B 0x102520  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_REG_CTOUCH_TRANSFORM_C 0x102524  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_REG_CTOUCH_TRANSFORM_D 0x102528  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_REG_CTOUCH_TRANSFORM_E 0x10252c  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_REG_CTOUCH_TRANSFORM_F 0x102530  /* Touch-screen transform coefficient (s15.16) */
                                                 /* 0x102534 Reserved */
#  define FT80X_REG_CTOUCH_TOUCH4_X    0x102538  /* Extended mode: touch-screen screen X data for
                                                  * touch 4 */
                                                 /* 0x10253c – 0x102450 Reserved */
#  define FT80X_REG_SCREENSHOT_READ    0x102554  /* Set to enable readout of the screenshot of the
                                                  * selected Y line */
                                                 /* 0x10253c – 0x102468 Reserved */
#  define FT80X_REG_TRIM               0x10256c  /* Internal relaxation clock trimming */
                                                 /* 0x102570 Reserved */
#  define FT80X_REG_CTOUCH_DIRECT_XY   0x102574  /* Compatibility mode: Touch screen direct
                                                  * (x-MSB16; y-LSB16) conversions */
#  define FT80X_REG_CTOUCH_TOUCH2_XY   0x102574  /* Extended mode: touch-screen screen data for
                                                  * touch 2 (x-MSB16; y-LSB16) */
#  define FT80X_REG_CTOUCH_DIRECT_Z1Z2 0x102578  /* Compatibility mode: Touch screen direct
                                                  * (z1-MSB16; z2-LSB16) conversions */
#  define FT80X_REG_CTOUCH_TOUCH3_XY   0x102578  /* Extended mode: touch-screen screen data for
                                                  * touch 3 (x-MSB16; y-LSB16) */
#endif

#define FT80X_REG_TRACKER              0x109000  /* Track register (Track value – MSB16;
                                                  * Tag value - LSB8) */

/* FT80x Register Bit Definitions ***********************************************************/

/* FT80X_REG_ID */

#define ID_MASK                        0xff      /* Bits 0-7: Register ID */

/* FT80X_REG_DLSWAP */

#define DLSWAP_DONE                    0         /* Bits 0-1: 0=Swap is complete */
#define DLSWAP_LINE                    1         /* Bits 0-1: 1=Graphics engine will render
                                                  * the screen immediately after current line.
                                                  * May cause a tearing effect.
                                                  */
#define DLSWAP_FRAME                   2         /* Bits 0-1: 2=Graphics engine will render
                                                  * the screen immediately after the current
                                                  * frame is scanned out (recommended).
                                                  */
/* FT80X_REG_SOUND */
/* Sound effect (Bits 0-7) */

#define FT08X_EFFECT_SILENCE           0x00      /* Silence */
#define FT08X_EFFECT_SQUARE            0x01      /* Square wave */
#define FT08X_EFFECT_SINE              0x02      /* Sine wave */
#define FT08X_EFFECT_SAWTOOTH          0x03      /* Sawtooth wave */
#define FT08X_EFFECT_TRIANGLE          0x04      /* Triangle wave */
#define FT08X_EFFECT_BEEPING           0x05      /* Beeping */
#define FT08X_EFFECT_ALARM             0x06      /* Alarm */
#define FT08X_EFFECT_WARBLE            0x07      /* Warble */
#define FT08X_EFFECT_CAROUSEL          0x08      /* Carousel */
#define FT08X_EFFECT_PIP1              0x10      /* 1 short pip */
#define FT08X_EFFECT_PIP2              0x11      /* 2 short pips */
#define FT08X_EFFECT_PIP3              0x12      /* 3 short pips */
#define FT08X_EFFECT_PIP4              0x13      /* 4 short pips */
#define FT08X_EFFECT_PIP5              0x14      /* 5 short pips */
#define FT08X_EFFECT_PIP6              0x15      /* 6 short pips */
#define FT08X_EFFECT_PIP7              0x16      /* 7 short pips */
#define FT08X_EFFECT_PIP8              0x17      /* 8 short pips */
#define FT08X_EFFECT_PIP9              0x18      /* 9 short pips */
#define FT08X_EFFECT_PIP10             0x19      /* 10 short pips */
#define FT08X_EFFECT_PIP11             0x1a      /* 11 short pips */
#define FT08X_EFFECT_PIP12             0x1b      /* 12 short pips */
#define FT08X_EFFECT_PIP13             0x1c      /* 13 short pips */
#define FT08X_EFFECT_PIP14             0x1d      /* 14 short pips */
#define FT08X_EFFECT_PIP15             0x1e      /* 15 short pips */
#define FT08X_EFFECT_PIP16             0x1f      /* 16 short pips */
#define FT08X_EFFECT_DTMFHASH          0x23      /* DTMF # */
#define FT08X_EFFECT_DTMFASTERISK      0x2c      /* DTMF * */
#define FT08X_EFFECT_DTMF0             0x30      /* DTMF 0 */
#define FT08X_EFFECT_DTMF1             0x31      /* DTMF 1 */
#define FT08X_EFFECT_DTMF2             0x32      /* DTMF 2 */
#define FT08X_EFFECT_DTMF3             0x33      /* DTMF 3 */
#define FT08X_EFFECT_DTMF4             0x34      /* DTMF 4 */
#define FT08X_EFFECT_DTMF5             0x35      /* DTMF 5 */
#define FT08X_EFFECT_DTMF6             0x36      /* DTMF 6 */
#define FT08X_EFFECT_DTMF7             0x37      /* DTMF 7 */
#define FT08X_EFFECT_DTMF8             0x38      /* DTMF 8 */
#define FT08X_EFFECT_DTMF9             0x39      /* DTMF 9 */
#define FT08X_EFFECT_HARP              0x40      /* Harp */
#define FT08X_EFFECT_XYLOPHONE         0x41      /* Xylophone */
#define FT08X_EFFECT_TUBA              0x42      /* Tuba */
#define FT08X_EFFECT_GLOCKENSPIEL      0x43      /* Glockenspiel */
#define FT08X_EFFECT_ORGAN             0x44      /* Organ */
#define FT08X_EFFECT_TRUMPET           0x45      /* Trumpet */
#define FT08X_EFFECT_PIANO             0x46      /* Piano */
#define FT08X_EFFECT_CHIMES            0x47      /* Chimes */
#define FT08X_EFFECT_MUSICBOX          0x48      /* Music box */
#define FT08X_EFFECT_BELL              0x49      /* Bell */
#define FT08X_EFFECT_CLICK             0x50      /* Click */
#define FT08X_EFFECT_SWITCH            0x51      /* Switch */
#define FT08X_EFFECT_COWBELL           0x52      /* Cowbell */
#define FT08X_EFFECT_NOTCH             0x53      /* Notch */
#define FT08X_EFFECT_HIHAT             0x54      /* Hihat */
#define FT08X_EFFECT_KICKDRUM          0x55      /* Kickdrum */
#define FT08X_EFFECT_POP               0x56      /* Pop */
#define FT08X_EFFECT_CLACK             0x57      /* Clack */
#define FT08X_EFFECT_CHACK             0x58      /* Chack */
#define FT08X_EFFECT_MUTE              0x60      /* Mute */
#define FT08X_EFFECT_UNMUTE            0x61      /* Unmute */

/* MIDI Note Effect (Bots 8-15) */

#define FT08X_NOTE_A0                  ((uint16_t)21 << 8)  /* A0, 27.5 Hz */
#define FT08X_NOTE_ASHARP0             ((uint16_t)22 << 8)  /* A#0, 29.14 Hz */
#define FT08X_NOTE_B0                  ((uint16_t)23 << 8)  /* B0, 30.9 Hz */
#define FT08X_NOTE_C1                  ((uint16_t)24 << 8)  /* C1, 32.7 Hz */
#define FT08X_NOTE_CSHARP1             ((uint16_t)25 << 8)  /* C#1, 34.6 Hz */
#define FT08X_NOTE_D1                  ((uint16_t)26 << 8)  /* D1, 36.7 Hz */
#define FT08X_NOTE_DSHARP1             ((uint16_t)27 << 8)  /* D#1, 38.9 Hz */
#define FT08X_NOTE_E1                  ((uint16_t)28 << 8)  /* E1, 41.2 Hz */
#define FT08X_NOTE_F1                  ((uint16_t)29 << 8)  /* F1, 43.7 Hz */
#define FT08X_NOTE_FSHARP1             ((uint16_t)30 << 8)  /* F#1, 46.2 Hz */
#define FT08X_NOTE_G1                  ((uint16_t)31 << 8)  /* G1, 49.0 Hz */
#define FT08X_NOTE_GSHARP1             ((uint16_t)32 << 8)  /* G#1, 51.9 Hz */
#define FT08X_NOTE_A1                  ((uint16_t)33 << 8)  /* A1, 55.0 Hz */
#define FT08X_NOTE_ASHARP1             ((uint16_t)34 << 8)  /* A#1, 58.3 Hz */
#define FT08X_NOTE_B1                  ((uint16_t)35 << 8)  /* B1, 61.7 Hz */
#define FT08X_NOTE_C2                  ((uint16_t)36 << 8)  /* C2, 65.4 Hz */
#define FT08X_NOTE_CSHARP1             ((uint16_t)37 << 8)  /* C#2, 69.3 Hz */
#define FT08X_NOTE_DSHARP2             ((uint16_t)39 << 8)  /* D#2, 77.8 Hz */
#define FT08X_NOTE_E2                  ((uint16_t)40 << 8)  /* E2, 82.4 Hz */
#define FT08X_NOTE_F2                  ((uint16_t)41 << 8)  /* F2, 87.3 Hz */
#define FT08X_NOTE_FSHARP2             ((uint16_t)42 << 8)  /* F#2, 92.5 Hz */
#define FT08X_NOTE_G2                  ((uint16_t)43 << 8)  /* G2, 98.0 Hz */
#define FT08X_NOTE_GSHARP2             ((uint16_t)44 << 8)  /* G#2, 103.8 Hz */
#define FT08X_NOTE_A2                  ((uint16_t)45 << 8)  /* A2, 110.0 Hz */
#define FT08X_NOTE_ASHARP2             ((uint16_t)46 << 8)  /* A#2, 116.5 Hz */
#define FT08X_NOTE_B2                  ((uint16_t)47 << 8)  /* B2, 123.5 Hz */
#define FT08X_NOTE_C3                  ((uint16_t)48 << 8)  /* C3, 130.8 Hz */
#define FT08X_NOTE_CSHARP3             ((uint16_t)49 << 8)  /* C#3, 138.6 Hz */
#define FT08X_NOTE_D3                  ((uint16_t)50 << 8)  /* D3, 146.8 Hz */
#define FT08X_NOTE_DSHARP3             ((uint16_t)51 << 8)  /* D#3, 155.6 Hz */
#define FT08X_NOTE_E3                  ((uint16_t)52 << 8)  /* E3, 164.8 Hz */
#define FT08X_NOTE_F3                  ((uint16_t)53 << 8)  /* F3, 174.6 Hz */
#define FT08X_NOTE_FSHARP3             ((uint16_t)54 << 8)  /* F#3, 185.0 Hz */
#define FT08X_NOTE_G3                  ((uint16_t)55 << 8)  /* G3, 196.0 Hz */
#define FT08X_NOTE_GSHARP7             ((uint16_t)56 << 8)  /* G#3, 207.7 Hz */
#define FT08X_NOTE_A3                  ((uint16_t)57 << 8)  /* A3, 220.0 Hz */
#define FT08X_NOTE_ASHARP3             ((uint16_t)58 << 8)  /* A#3, 233.1 Hz */
#define FT08X_NOTE_B3                  ((uint16_t)59 << 8)  /* B3, 246.9 Hz */
#define FT08X_NOTE_C4                  ((uint16_t)60 << 8)  /* C4, 261.6 Hz */
#define FT08X_NOTE_CSHARP4             ((uint16_t)61 << 8)  /* C#4, 277.2 Hz */
#define FT08X_NOTE_D4                  ((uint16_t)62 << 8)  /* D4, 293.7 Hz */
#define FT08X_NOTE_DSHARP4             ((uint16_t)63 << 8)  /* D#4, 311.1 Hz */
#define FT08X_NOTE_E4                  ((uint16_t)64 << 8)  /* E4, 329.6 Hz */
#define FT08X_NOTE_F4                  ((uint16_t)65 << 8)  /* F4, 349.2 Hz */
#define FT08X_NOTE_FSHARP4             ((uint16_t)66 << 8)  /* F#4, 370.0 Hz */
#define FT08X_NOTE_G4                  ((uint16_t)67 << 8)  /* G4, 392.0 Hz */
#define FT08X_NOTE_GSHARP4             ((uint16_t)68 << 8)  /* G#4, 415.3 Hz */
#define FT08X_NOTE_A4                  ((uint16_t)69 << 8)  /* A4, 440.0 Hz */
#define FT08X_NOTE_ASHARP4             ((uint16_t)70 << 8)  /* A#4, 466.2 Hz */
#define FT08X_NOTE_B4                  ((uint16_t)71 << 8)  /* B4, 493.9 Hz */
#define FT08X_NOTE_C5                  ((uint16_t)72 << 8)  /* C5, 523.3 Hz */
#define FT08X_NOTE_CSHARP5             ((uint16_t)73 << 8)  /* C#5, 554.4 Hz */
#define FT08X_NOTE_D5                  ((uint16_t)74 << 8)  /* D5, 587.3 Hz */
#define FT08X_NOTE_DSHARP5             ((uint16_t)75 << 8)  /* D#5, 622.3 Hz */
#define FT08X_NOTE_E5                  ((uint16_t)76 << 8)  /* E5, 659.3 Hz */
#define FT08X_NOTE_F5                  ((uint16_t)77 << 8)  /* F5, 698.5 Hz */
#define FT08X_NOTE_FSHARP5             ((uint16_t)78 << 8)  /* F#5, 740.0 Hz */
#define FT08X_NOTE_G5                  ((uint16_t)79 << 8)  /* G5, 784.0 Hz */
#define FT08X_NOTE_G5                  ((uint16_t)80 << 8)  /* G#5, 830.6 Hz */
#define FT08X_NOTE_A5                  ((uint16_t)81 << 8)  /* A5, 880.0 Hz */
#define FT08X_NOTE_D2                  ((uint16_t)38 << 8)  /* D2, 73.4 Hz */
#define FT08X_NOTE_ASHARP5             ((uint16_t)82 << 8)  /* A#5, 932.3 Hz */
#define FT08X_NOTE_B5                  ((uint16_t)83 << 8)  /* B5, 987.8 Hz */
#define FT08X_NOTE_C6                  ((uint16_t)84 << 8)  /* C6, 1046.5 Hz */
#define FT08X_NOTE_CSHARP6             ((uint16_t)85 << 8)  /* C#6, 1108.7 Hz */
#define FT08X_NOTE_D6                  ((uint16_t)86 << 8)  /* D6, 1174.7 Hz */
#define FT08X_NOTE_DSHARP6             ((uint16_t)87 << 8)  /* D#6, 1244.5 Hz */
#define FT08X_NOTE_E6                  ((uint16_t)88 << 8)  /* E6, 1318.5 Hz */
#define FT08X_NOTE_F6                  ((uint16_t)89 << 8)  /* F6, 1396.9 Hz */
#define FT08X_NOTE_FSHARP6             ((uint16_t)90 << 8)  /* F#6, 1480.0 Hz */
#define FT08X_NOTE_G6                  ((uint16_t)91 << 8)  /* G6, 1568.0 Hz */
#define FT08X_NOTE_GSHARP6             ((uint16_t)92 << 8)  /* G#6, 1661.2 Hz */
#define FT08X_NOTE_A6                  ((uint16_t)93 << 8)  /* A6, 1760.0 Hz */
#define FT08X_NOTE_ASHARP6             ((uint16_t)94 << 8)  /* A#6, 1864.7 Hz */
#define FT08X_NOTE_B6                  ((uint16_t)95 << 8)  /* B6, 1975.5 Hz */
#define FT08X_NOTE_C7                  ((uint16_t)96 << 8)  /* C7, 2093.0 Hz */
#define FT08X_NOTE_CSHARP7             ((uint16_t)97 << 8)  /* C#7, 2217.5 Hz */
#define FT08X_NOTE_D7                  ((uint16_t)98 << 8)  /* D7, 2349.3 Hz */
#define FT08X_NOTE_DSHARP7             ((uint16_t)99 << 8)  /* D#7, 2489.03 Hz */
#define FT08X_NOTE_E7                  ((uint16_t)100 << 8) /* E7, 2637.0 Hz */
#define FT08X_NOTE_F7                  ((uint16_t)101 << 8) /* F7, 2793.8 Hz */
#define FT08X_NOTE_FSHARP7             ((uint16_t)102 << 8) /* F#7, 2960.0 Hz */
#define FT08X_NOTE_G7                  ((uint16_t)103 << 8) /* G7, 3136.0 Hz */
#define FT08X_NOTE_GSHARP7             ((uint16_t)104 << 8) /* G#7, 3322.4 Hz */
#define FT08X_NOTE_A7                  ((uint16_t)105 << 8) /* A7, 3520.0 Hz */
#define FT08X_NOTE_ASHARP7             ((uint16_t)106 << 8) /* A#7, 3729.3 Hz */
#define FT08X_NOTE_B7                  ((uint16_t)107 << 8) /* B7, 3951.1 Hz */
#define FT08X_NOTE_C8                  ((uint16_t)108 << 8) /* C8, 4186.0 Hz */

/* FT80X_REG_GPIO */

                                                 /* Bits 0-1:  GPIO 0-1 value */
                                                 /* Bits 2-3:  MISO and nINT drive strength */
                                                 /* Bits 4:    Display signal drive strength */
#define FT80X_GPIO_DRIVE_SHIFT         5         /* Bits 5-6:  GPIO output drive strength */
#define FT80X_GPIO_DRIVE_MASK          (3 << FT80X_GPIO_DRIVE_SHIFT)
#  define FT80X_GPIO_DRIVE_4MA         (0 << FT80X_GPIO_DRIVE_SHIFT)
#  define FT80X_GPIO_DRIVE_8MA         (1 << FT80X_GPIO_DRIVE_SHIFT)
#  define FT80X_GPIO_DRIVE_12MA        (2 << FT80X_GPIO_DRIVE_SHIFT)
#  define FT80X_GPIO_DRIVE_16MA        (3 << FT80X_GPIO_DRIVE_SHIFT)
                                                 /* Bit 7:     GPIO 7 value */

/* FT80X_REG_PLAYBACK_FORMAT */

#define AUDIO_FORMAT_LINEAR            0         /* Linear Sample format */
#define AUDIO_FORMAT_ULAW              1         /* uLaw Sample format */
#define AUDIO_FORMAT_ADPCM             2         /* 4-bit IMA ADPCM Sample format */

/* FT80X_REG_TOUCH_TAG */

#define TOUCH_TAG_MASK                 0xff      /* Bits 0-7: Tag of touched graphic object */

/* FT80X_REG_TOUCH_MODE */

#define TOUCH_MODE_OFF                 0         /* Acquisition stopped, touch detection
                                                  * interrupt is still valid. */
#define TOUCH_MODE_ONESHOT             1         /* Perform acquisition once every write of 1
                                                  * to REG_TOUCH_MODE. */
#define TOUCH_MODE_FRAMESYNC           2         /* Perform acquisition for every frame sync
                                                  * (~60 data acquisition/second). */
#define TOUCH_MODE_CONTINUOUS          3         /* Perform acquisition continuously at
                                                  * approximately 1000 data acquisition /
                                                  * second. */

/* Interrupts *******************************************************************************/
/* The interrupt output pin is enabled by REG_INT_EN.  When REG_INT_EN is 0, INT_N is
 * tri-state (pulled to high by external pull-up resistor).  When REG_INT_EN is 1, INT_N is
 * driven low when any of the interrupt flags in REG_INT_FLAGS are high, after masking with
 * REG_INT_MASK. Writing a '1' in any bit of REG_INT_MASK will enable the correspond
 * interrupt.  Each bit in REG_INT_FLAGS is set by a corresponding interrupt source.
 * REG_INT_FLAGS is readable by the host at any time, and clears when read.
 */

/* FT80X_REG_INT_EN */

#define FT80X_INT_ENABLE               (0)       /* Bit 0: 0=Interrupts disabled */
#define FT80X_INT_DISABLE              (1 << 0)  /*        1=Interrupts enabled */

/* FT80X_REG_INT_FLAGS and FT80X_REG_INT_MASK */

#define FT80X_INT_SWAP                 (1 << 0)  /* Bit 0: Display swap occurred */
#define FT80X_INT_TOUCH                (1 << 1)  /* Bit 1: Touch-screen touch detected */
#define FT80X_INT_TAG                  (1 << 2)  /* Bit 2: Touch-screen tag value change */
#define FT80X_INT_SOUND                (1 << 3)  /* Bit 3: Sound effect ended */
#define FT80X_INT_PLAYBACK             (1 << 4)  /* Bit 4: Audio playback ended */
#define FT80X_INT_CMDEMPTY             (1 << 5)  /* Bit 5: Command FIFO empty */
#define FT80X_INT_CMDFLAG              (1 << 6)  /* Bit 6: Command FIFO flag */
#define FT80X_INT_CONVCOMPLETE         (1 << 7)  /* Bit 7: Touch-screen conversions completed */

#define FT80X_INT_NEVENTS              8
#define FT80X_INT(n)                   (1 << (n))

/* FT80x Display List Commands **************************************************************/
/* Host commands.  3 byte commands.  The first byte begins with [7:6]==01.  Bits [5:0] of
 * the first byte are actual command.  The following two bytes must be zero.
 */

#define FT80X_CMD_ACTIVE           0x00        /* Switch from Standby/Sleep modes to active mode */
#define FT80X_CMD_STANDBY          0x41        /* Put FT80x core to standby mode */
#define FT80X_CMD_SLEEP            0x42        /* Put FT80x core to sleep mode */
#define FT80X_CMD_CLKEXT           0x44        /* Enable PLL input from oscillator or external clock */
#define FT80X_CMD_PWRDOWN          0x50        /* Switch off 1.2V internal regulator */
#define FT80X_CMD_CLK36M           0x61        /* Switch PLL output clock to 36MHz */
#define FT80X_CMD_CLK48M           0x62        /* Switch PLL output clock to 48MHz (default). */
#define FT80X_CMD_CORERST          0x68        /* Send reset pulse to FT800 core */

/* Display list command encoding
 *
 * Each display list command has a 32-bit encoding. The most significant bits
 * of the code determine the command.  Command parameters (if any) are present
 * in the least significant bits. Any bits marked reserved must be zero.
 */

/* FT800 graphics engine specific macros useful for static display list generation */
/* Setting Graphics state */
/* ALPHA_FUNC (0x09) - Set the alpha test function */

#define FT80X_ALPHA_FUNC(func,ref) \
  ((9 << 24) | (((func) & 7) << 8) | (((ref) & 255) << 0))

/* BITMAP_HANDLE (0x05) - Set the bitmap handle */

#define FT80X_BITMAP_HANDLE(handle) \
  ((5 << 24) | (((handle) & 31) << 0))

/* BITMAP_LAYOUT (0x07) - Set the source bitmap memory format and layout for
 * the current handle
 */

#define FT80X_BITMAP_LAYOUT(format,linestride,height) \
  ((7 << 24) | (((format) & 31) << 19) | (((linestride) & 1023) << 9) | \
   (((height) & 511) << 0))

/* format */

#define FT80X_FORMAT_ARGB1555      0
#define FT80X_FORMAT_L1            1
#define FT80X_FORMAT_L4            2
#define FT80X_FORMAT_L8            3
#define FT80X_FORMAT_RGB332        4
#define FT80X_FORMAT_ARGB2         5
#define FT80X_FORMAT_ARGB4         6
#define FT80X_FORMAT_RGB565        7
#define FT80X_FORMAT_PALETTED      8
#define FT80X_FORMAT_TEXT8X8       9
#define FT80X_FORMAT_TEXTVGA       10
#define FT80X_FORMAT_BARGRAPH      11

/* BITMAP_SIZE (0x08) - Set the screen drawing of bitmaps for the current
 * handle
 */

#define FT80X_BITMAP_SIZE(filter,wrapx,wrapy,width,height) \
  ((8 << 24) | (((filter) & 1) << 20) | (((wrapx) & 1) << 19) | \
   (((wrapy) & 1) << 18) | (((width) & 511) << 9) | (((height) & 511) << 0))

/* filter */

#define FT80X_FILTER_NEAREST       0
#define FT80X_FILTER_BILINEAR      1

/* wrapx/wrapy */

#define FT80X_WRAP_BORDER          0
#define FT80X_WRAP_REPEAT          1

/* BITMAP_SOURCE (0x01) - Set the source address for bitmap graphics */

#define FT80X_BITMAP_SOURCE(addr) \
  ((1 << 24) | (((addr) & 1048575) << 0))

/* BITMAP_TRANSFORM_A (0x15) - F (0x1a) - Set the components of the bitmap
 * transform matrix
 */

#define FT80X_BITMAP_TRANSFORM_A(a) \
  ((21 << 24) | (((a) & 131071) << 0))
#define FT80X_BITMAP_TRANSFORM_B(b) \
  ((22 << 24) | (((b) & 131071) << 0))
#define FT80X_BITMAP_TRANSFORM_C(c) \
  ((23 << 24) | (((c) & 16777215) << 0))
#define FT80X_BITMAP_TRANSFORM_D(d) \
  ((24 << 24) | (((d) & 131071) << 0))
#define FT80X_BITMAP_TRANSFORM_E(e) \
  ((25 << 24) | (((e) & 131071) << 0))
#define FT80X_BITMAP_TRANSFORM_F(f) \
  ((26 << 24) | (((f) & 16777215) << 0))

/* BLEND_FUNC(0x0b) - Set pixel arithmetic */

#define FT80X_BLEND_FUNC(src,dest) \
  ((11 << 24) | (((src) & 7) << 3) | (((dest) & 7) << 0))

/* src/dest */

#define FT80X_BLEND_ZERO                0
#define FT80X_BLEND_ONE                 1
#define FT80X_BLEND_SRC_ALPHA           2
#define FT80X_BLEND_DST_ALPHA           3
#define FT80X_BLEND_ONE_MINUS_SRC_ALPHA 4
#define FT80X_BLEND_ONE_MINUS_DST_ALPHA 5

/* CELL (0x06) - Set the bitmap cell number for the VERTEX2F command */

#define FT80X_CELL(cell) \
  ((6 << 24) | (((cell) & 127) << 0))

/* CLEAR (0x26) - Clear buffers to preset values */

#define FT80X_CLEAR(c,s,t) \
  ((38 << 24) | (((c) & 1) << 2) | (((s) & 1) << 1) | (((t) & 1) << 0))

/* CLEAR_COLOR_A (0x0f) - Set clear value for the alpha channel */

#define FT80X_CLEAR_COLOR_A(alpha) \
  ((15 << 24) | (((alpha) & 255) << 0))

/* CLEAR_COLOR_RGB (0x02) - Set clear values for red, green and blue
 * channels
 */

#define FT80X_CLEAR_COLOR_RGB(red,green,blue) \
  ((2 << 24) | (((red) & 255) << 16) | (((green) & 255) << 8) | \
   (((blue) & 255) << 0))

/* CLEAR_STENCIL (0x11) - Set clear value for the stencil buffer */

#define FT80X_CLEAR_STENCIL(s) \
  ((17 << 24) | (((s) & 255) << 0))

/* CLEAR_TAG (0x12) - Set clear value for the tag buffer */

#define FT80X_CLEAR_TAG(s) \
  ((18 << 24) | (((s) & 255) << 0))

/* COLOR_A (0x10) - Set the current color alpha */

#define FT80X_COLOR_A(alpha) \
  ((16 << 24) | (((alpha) & 255) << 0))

/* COLOR_MASK (0x20) - Enable or disable writing of color components */

#define FT80X_COLOR_MASK(r,g,b,a) \
  ((32 << 24) | (((r) & 1) << 3) | (((g) & 1) << 2) | (((b) & 1) << 1) | \
   (((a) & 1) << 0))

/* COLOR_RGB (0x04) - Set the current color red, green and blue */

#define FT80X_COLOR_RGB(red,green,blue) \
  ((4 << 24) | (((red) & 255) << 16) | (((green) & 255) << 8) | \
   (((blue) & 255) << 0))

/* LINE_WIDTH (0x0e) - Set the line width */

#define FT80X_LINE_WIDTH(width) \
  ((14 << 24) | (((width) & 4095) << 0))

/* POINT_SIZE (0x0d) - Set point size */

#define FT80X_POINT_SIZE(size) \
  ((13 << 24) | (((size) & 8191) << 0))

/* RESTORE_CONTEXT (0x23) - Restore the current graphics context from the
 * context stack
 */

#define FT80X_RESTORE_CONTEXT() \
  (35 << 24)

/* SAVE_CONTEXT (0x22) - Push the current graphics context on the context
 * stack
 */

#define FT80X_SAVE_CONTEXT() \
  (34 << 24)

/* SCISSOR_SIZE (0x1c) - Set the size of the scissor clip rectangle */

#define FT80X_SCISSOR_SIZE(width,height) \
  ((28 << 24) | (((width) & 1023) << 10) | (((height) & 1023) << 0))

/* SCISSOR_XY (0x1b) - Set the top left corner of the scissor clip rectangle */

#define FT80X_SCISSOR_XY(x,y) \
  ((27 << 24) | (((x) & 511) << 9) | (((y) & 511) << 0))

/* STENCIL_FUNC (0x0a) - Set function and reference value for stencil testing */

#define FT80X_STENCIL_FUNC(func,ref,mask) \
  ((10 << 24) | (((func) & 7) << 16) | (((ref) & 255) << 8) | (((mask) & 255) << 0))

/* func */

#define STENCIL_FUNC_NEVER       0
#define STENCIL_FUNC_LESS        1
#define STENCIL_FUNC_LEQUAL      2
#define STENCIL_FUNC_GREATER     3
#define STENCIL_FUNC_GEQUAL      4
#define STENCIL_FUNC_EQUAL       5
#define STENCIL_FUNC_NOTEQUAL    6
#define STENCIL_FUNC_ALWAYS      7

/* STENCIL_MASK (0x13) - Control the writing of individual bits in the
 * stencil planes
 */

#define FT80X_STENCIL_MASK(mask) \
  ((19 << 24) | (((mask) & 255) << 0))

/* STENCIL_OP (0x0c) - Set stencil test actions */

#define FT80X_STENCIL_OP(sfail,spass) \
  ((12 << 24) | (((sfail) & 7) << 3) | (((spass) & 7) << 0))

/* sfail and spass */

#define STENCIL_OP_ZERO          0
#define STENCIL_OP_KEEP          1
#define STENCIL_OP_REPLACE       2
#define STENCIL_OP_INCR          3
#define STENCIL_OP_DECR          4
#define STENCIL_OP_INVERT        5

/* TAG (0x03) - Set the current tag value */

#define FT80X_TAG(s) \
  ((3 << 24) | (((s) & 255) << 0))

/* TAG_MASK (0x14) - Control the writing of the tag buffer */

#define FT80X_TAG_MASK(mask) \
  ((20 << 24) | (((mask) & 1) << 0))

/* Drawing actions */
/* BEGIN (0x1f) - Start drawing a graphics primitive */

#define FT80X_BEGIN(prim) \
  ((31 << 24) | (((prim) & 15) << 0))

/* Graphics primitive operations: */

#define FT80X_PRIM_BITMAPS         1  /* Bitmap drawing primitive */
#define FT80X_PRIM_POINTS          2  /* Point drawing primitive */
#define FT80X_PRIM_LINES           3  /* Line drawing primitive */
#define FT80X_PRIM_LINE_STRIP      4  /* Line strip drawing primitive */
#define FT80X_PRIM_EDGE_STRIP_R    5  /* Edge strip right side drawing primitive */
#define FT80X_PRIM_EDGE_STRIP_L    6  /* Edge strip left side drawing primitive */
#define FT80X_PRIM_EDGE_STRIP_A    7  /* Edge strip above drawing primitive */
#define FT80X_PRIM_EDGE_STRIP_B    8  /* Edge strip below side drawing primitive */
#define FT80X_PRIM_RECTS           9  /* Rectangle drawing primitive */

/* END (0x21) -Finish drawing a graphics primitive */

#define FT80X_END() \
  (33 << 24)

/* VERTEX2F (0b01) -Supply a vertex with fractional coordinates */

#define FT80X_VERTEX2F(x,y) \
  ((1 << 30) | (((x) & 32767) << 15) | (((y) & 32767) << 0))

/* VERTEX2II (0b10) - Supply a vertex with positive integer coordinates */

#define FT80X_VERTEX2II(x,y,handle,cell) \
  ((2 << 30) | (((x) & 511) << 21) | (((y) & 511) << 12) | \
   (((handle) & 31) << 7) | (((cell) & 127) << 0))

/* Execution control */
/* JUMP (0x1e) - Execute commands at another location in the display list */

#define FT80X_JUMP(dest) \
  ((30 << 24) | (((dest) & 65535) << 0))

/* MACRO(0x25) - Execute a single command from a macro register */

#define FT80X_MACRO(m) \
  (37 << 24) | (((m) & 1) << 0)

/* CALL (0x1d) - Execute a sequence of commands at another location in the
 * display list
 */

#define FT80X_CALL(dest) \
  ((29 << 24) | (((dest) & 65535) << 0))

/* RETURN (0x24) -Return from a previous CALL command */

#define FT80X_RETURN() \
  (36 << 24)

/* DISPLAY (0x00) - End the display list */

#define FT80X_DISPLAY() \
  (0 << 24)

/* FT80x Graphic Engine Co-processor commands.
 *
 * Like the 32-bit commands above, these commands are elements of the display list.
 * Unlike the 32-bit commands, these all begin with 0xffffffxx and consist of multiple
 * 32-bit words.  In all cases, byte data such as strings must be padded to the even
 * 32-bit boundaries.
 */

#define FT80X_CMD_APPEND           0xffffff1e  /* Append memory to a display list */
#define FT80X_CMD_BGCOLOR          0xffffff09  /* Set the background color */
#define FT80X_CMD_BITMAP_TRANSFORM 0xffffff21
#define FT80X_CMD_BUTTON           0xffffff0d  /* Draw a button */
#define FT80X_CMD_CALIBRATE        0xffffff15  /* Execute touchscreen calibration routine */
#define FT80X_CMD_CLOCK            0xffffff14  /* Draw an analog clock */
#define FT80X_CMD_COLDSTART        0xffffff32  /* Set co-processor engine state to default values */
#define FT80X_CMD_CRC              0xffffff03  /* ? */
#define FT80X_CMD_DIAL             0xffffff2d  /* Draw a rotary dial control */
#define FT80X_CMD_DLSTART          0xffffff00  /* Start a new display list */
#define FT80X_CMD_EXECUTE          0xffffff07  /* ? */
#define FT80X_CMD_FGCOLOR          0xffffff0a  /* Set the foreground color */
#define FT80X_CMD_GAUGE            0xffffff13  /* Draw a gauge */
#define FT80X_CMD_GETMATRIX        0xffffff33  /* Retrieves the current matrix coefficients */
#define FT80X_CMD_GETPOINT         0xffffff08  /* ? */
#define FT80X_CMD_GETPROPS         0xffffff25  /* Get info for image in RAM_G from last CMD_LOADIMAGE command */
#define FT80X_CMD_GETPTR           0xffffff23  /* Get end address from CMD_INFLATE operation */
#define FT80X_CMD_GRADCOLOR        0xffffff34  /* Set 3D effects for BUTTON and KEYS highlight colors */
#define FT80X_CMD_GRADIENT         0xffffff0b  /* Draw a smooth color gradient */
#define FT80X_CMD_HAMMERAUX        0xffffff04  /* ? */
#define FT80X_CMD_IDCT             0xffffff06  /* ? */
#define FT80X_CMD_INFLATE          0xffffff22  /* Decompress data into memory */
#define FT80X_CMD_INTERRUPT        0xffffff02  /* Trigger interrupt INT_CMDFLAG */
#define FT80X_CMD_KEYS             0xffffff0e  /* Draw a row of keys */
#define FT80X_CMD_LOADIDENTITY     0xffffff26  /* Set the current matrix to identity */
#define FT80X_CMD_LOADIMAGE        0xffffff24  /* Load a JPEG image */
#define FT80X_CMD_LOGO             0xffffff31  /* Play a device logo animation */
#define FT80X_CMD_MARCH            0xffffff05  /* ? */
#define FT80X_CMD_MEMCPY           0xffffff1d  /* Copy a block of memory */
#define FT80X_CMD_MEMCRC           0xffffff18  /* Compute a CRC for memory */
#define FT80X_CMD_MEMSET           0xffffff1b  /* Fill memory with a byte value */
#define FT80X_CMD_MEMWRITE         0xffffff1a  /* Write bytes into memory */
#define FT80X_CMD_MEMZERO          0xffffff1c  /* Write zero to a block of memory */
#define FT80X_CMD_NUMBER           0xffffff2e  /* Draw a decimal number */
#define FT80X_CMD_PROGRESS         0xffffff0f  /* Draw a progress bar */
#define FT80X_CMD_REGREAD          0xffffff19  /* Read a register value */
#define FT80X_CMD_ROTATE           0xffffff29  /* Apply a rotation to the current matrix */
#define FT80X_CMD_SCALE            0xffffff28  /* Apply a scale to the current matrix */
#define FT80X_CMD_SCREENSAVER      0xffffff2f  /* Start screensaver animation */
#define FT80X_CMD_SCROLLBAR        0xffffff11  /* Draw a scroll bar */
#define FT80X_CMD_SETFONT          0xffffff2b  /* Register custom font into FT80x co-processor */
#define FT80X_CMD_SETMATRIX        0xffffff2a  /* Write current matrix as a bitmap transform */
#define FT80X_CMD_SKETCH           0xffffff30  /* Start a continuous sketch update */
#define FT80X_CMD_SLIDER           0xffffff10  /* Draw a slider */
#define FT80X_CMD_SNAPSHOT         0xffffff1f  /* Take a snapshot of the current screen */
#define FT80X_CMD_SPINNER          0xffffff16  /* Start an animated spinner */
#define FT80X_CMD_STOP             0xffffff17  /* Stop any spinner, screensave, or sketch */
#define FT80X_CMD_SWAP             0xffffff01  /* Swap the current display list */
#define FT80X_CMD_TEXT             0xffffff0c  /* Draw text */
#define FT80X_CMD_TOGGLE           0xffffff12  /* Draw a toggle switch */
#define FT80X_CMD_TOUCH_TRANSFORM  0xffffff20  /* ? */
#define FT80X_CMD_TRACK            0xffffff2c  /* Enable co-processor to track touch on graphics object */
#define FT80X_CMD_TRANSLATE        0xffffff27  /* Apply a translation to the current matrix */

/* Option parameter definitions */

#define FT80X_OPT_3D               0x00000000  /* Co-processor widget is drawn in 3D effect */
#define FT80X_OPT_RGB565           0x00000000  /* Co-processor decode the JPEG image to RGB565 format */
#define FT80X_OPT_MONO             0x00000001  /* Co-processor decode the JPEG image to monochrome format */
#define FT80X_OPT_NODL             0x00000002  /* No display list commands for decoded bitmap */
#define FT80X_OPT_FLAT             0x00000100  /* Co-processor widget is drawn without 3D effect */
#define FT80X_OPT_SIGNED           0x00000100  /* Number is treated as 32 bit signed integer */
#define FT80X_OPT_CENTERX          0x00000200  /* Co-processor widget centers horizontally */
#define FT80X_OPT_CENTERY          0x00000400  /* Co-processor widget centers vertically */
#define FT80X_OPT_CENTER           0x00000600  /* Co-processor widget centers horizontally and vertically */
#define FT80X_OPT_RIGHTX           0x00000800  /* Label on the Co-processor widget right justified */
#define FT80X_OPT_NOBACK           0x00001000  /* Co-processor widget has no background drawn */
#define FT80X_OPT_NOTICKS          0x00002000  /* Co-processor clock widget is drawn without hour ticks.
                                                * Gauge widget is drawn without major and minor ticks */
#define FT80X_OPT_NOHM             0x00004000  /* Co-processor clock widget is drawn without hour and minutes
                                                * hands, only seconds hand is drawn */
#define FT80X_OPT_NOPOINTER        0x00004000  /* Co-processor gauge has no pointer */
#define FT80X_OPT_NOSECS           0x00008000  /* Co-processor clock widget is drawn without seconds hand */
#define FT80X_OPT_NOHANDS          0x0000c000  /* Co-processor clock widget is drawn without hour, minutes or
                                                * seconds hands */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/* FT80x Lower Half Interface Definitions ***************************************************/
/* Pins relevant to software control.  The FT80X is a 48-pin part.  Most of the pins are
 * associated with the TFT panel and other board-related support.  A few a relevant to
 * software control of the part.  Those are listed here:
 *
 * FT80X PIN  DIR DESCRIPTION
 *  3          I  SPI: SCLK, I2C: SCL
 *  4          I  SPI: MISO, I2C: SDA
 *  5         I/O SPI: MOSI
 *  6          I  SPI: nCS
 *  11        OD  nINT Host interrupt
 *  12         *  nPD  Power down input
 *
 * In addition, if there is a audio amplifier on board (such as TPA6205A or LM4864), then
 * there may also be an active low audio shutdown output:
 *
 *  N/A        O  nSHDN Audio shutdown (active low)
 *
 * REVISIT:  In all of the architectures that I am aware of, the audio amplifier is
 * controlled by GPIOs driven by the FT80x and, hence, not controllable by board logic.
 *
 * SCL/SDA, SCLK/MISO/MOSI/nCS are handled by generic I2C or SPI logic. nInt and nPD are
 * directly managed by this interface.
 */

/* A reference to a structure of this type must be passed to the FT80X driver.  This
 * structure provides information about the configuration of the FT80X and provides some
 * board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by the driver and
 * is presumed to persist while the driver is active.  The memory may be read-only.
 */

struct ft80x_config_s
{
  /* Device characterization */

  uint32_t init_frequency;  /* I2C/SPI initialization frequency */
  uint32_t op_frequency;    /* I2C/SPI operational frequency */
#ifdef CONFIG_LCD_FT80X_I2C
  uint8_t address;          /* 7-bit I2C address */
#endif

  /* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks to isolate the
   * FT80X driver from differences in GPIO interrupt handling by varying boards and MCUs.
   * Interrupts should be configured on the falling edge of nINT.
   *
   *   attach  - Attach the ADS7843E interrupt handler to the GPIO interrupt
   *   enable  - Enable or disable the GPIO interrupt
   *   clear   - Acknowledge/clear any pending GPIO interrupt as necessary.
   *   pwrdown - Power the FT80X up or down.
   *   audio   - Enable audio (i.e., set the external audio amplifier shutdown pin to the
   *             appropriate level to enable or disable the external audio amplifier)
   *   destroy - The driver has been unlinked. Cleanup as necessary.
   */

  CODE int  (*attach)(FAR const struct ft80x_config_s *lower, xcpt_t isr,
                      FAR void *arg);
  CODE void (*enable)(FAR const struct ft80x_config_s *lower, bool enable);
  CODE void (*clear)(FAR const struct ft80x_config_s *lower);
  CODE void (*pwrdown)(FAR const struct ft80x_config_s *lower, bool pwrdown);
#ifdef CONFIG_LCD_FT80X_AUDIO_MCUSHUTDOWN
  CODE void (*audio)(FAR const struct ft80x_config_s *lower, bool enable);
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  CODE void (*destroy)(FAR const struct ft80x_config_s *lower);
#endif
};

/* FT80x Display List Command Structures ****************************************************/
/* This structure describes one generic display list command */

struct ft80x_dlcmd_s
{
  uint32_t cmd;     /* 0:  Display list command.  See FT80X_CMD_* definitions */
};

/* Specific display list command structures */
/* 32-bit commands */

struct ft80x_cmd32_s
{
  uint32_t cmd;     /* 0:  See macro encoding definitions above */
};

/* FT80X_CMD_APPEND:  Append memory to a display list */

struct ft80x_cmd_append_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_APPEND */
  uint32_t ptr;     /* 4:  Start of source commands in memory (input) */
  uint32_t num;     /* 8:  Number of bytes to copy (multiple of 4) */
};

/* FT80X_CMD_BGCOLOR:  Set the background color */

struct ft80x_cmd_bgcolor_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_BGCOLOR */
  uint32_t c;       /* 4:  New background color, as a 24-bit RGB number (input) */
};

/* FT80X_CMD_BITMAP_TRANSFORM */

struct ft80x_cmd_bitmaptransfer_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_BITMAP_TRANSFORM */
  int32_t x0;       /* 4:  (input) */
  int32_t y0;       /* 8:  (input) */
  int32_t x1;       /* 12: (input) */
  int32_t y1;       /* 16: (input) */
  int32_t x2;       /* 20: (input) */
  int32_t y2;       /* 24: (input) */
  int32_t tx0;      /* 28: (input) */
  int32_t ty0;      /* 32: (input) */
  int32_t tx1;      /* 36: (input) */
  int32_t ty1;      /* 40: (input) */
  int32_t tx2;      /* 44: (input) */
  int32_t ty2;      /* 48: (input) */
  uint16_t result;  /* 52: (output) */
};

/* FT80X_CMD_BUTTON:  Draw a button */

struct ft80x_cmd_button_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_BUTTON */
  int16_t x;        /* 4:  X-coordinate of button top-left, in pixels (input) */
  int16_t y;        /* 6:  Y-coordinate of button top-left, in pixels (input) */
  int16_t w;        /* 8:  Width of the button (input) */
  int16_t h;        /* 10: Height of the button (input) */
  int16_t font;     /* 12: bitmap handle to specify the font used in button label (input) */
  uint16_t options; /* 14: Button effects options (input) */
                    /* 16: Start of button label string (input)
                     *     NUL terminated and padded to 32-bit alignment */
};

/* FT80X_CMD_CALIBRATE:  Execute touchscreen calibration routine */

struct ft80x_cmd_calibrate_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_CALIBRATE */
  uint32_t result;  /* 4:  Result of calibration (0 on failure) (output) */
};

/* FT80X_CMD_CLOCK:  Draw an analog clock */

struct ft80x_cmd_clock_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_CLOCK */
  int16_t x;        /* 4:  X-coordinate of clock center, in pixels (input) */
  int16_t y;        /* 6:  Y-coordinate of clock center, in pixels (input) */
  int16_t r;        /* 8:  Radius of the clock in pixels (input) */
  uint16_t options; /* 10: Clock 3D effects and display options (input) */
  uint16_t h;       /* 12: Hours (input) */
  uint16_t m;       /* 14: Minutes (input) */
  uint16_t s;       /* 16: Seconds (input) */
  uint16_t ms;      /* 18: Milliseconds (input) */
};

/* FT80X_CMD_COLDSTART:  Set co-processor engine state to default values */

struct ft80x_cmd_coldstart_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_COLDSTART */
};

/* FT80X_CMD_DIAL:  Draw a rotary dial control */

struct ft80x_cmd_dial_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_DIAL */
  int16_t x;        /* 4:  X-coordinate of dial center, in pixels (input) */
  int16_t y;        /* 6:  Y-coordinate of dial center, in pixels (input) */
  int16_t r;        /* 8:  radius of dial, in pixels (input) */
  uint16_t options; /* 10: 3D and other display options (input) */
  uint16_t val;     /* 12: Position of dial points (0..65535) (input) */
};

/* FT80X_CMD_DLSTART: Start a new display list */

struct ft80x_cmd_dlstart_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_DLSTART */
};

/* FT80X_CMD_FGCOLOR:  Set the foreground color */

struct ft80x_cmd_fgcolor_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_FGCOLOR */
  uint32_t c;       /* 4:..New foreground color, as a 24-bit RGB number (input) */
};

/* FT80X_CMD_GAUGE: Draw a gauge */

struct ft80x_cmd_gauge_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_GAUGE */
  int16_t x;        /* 4:  X-coordinate of gauge center, in pixels (input) */
  int16_t y;        /* 6:  Y-coordinate of gauge center, in pixels (input) */
  int16_t r;        /* 8:  Radius of the gauge, in pixels (input) */
  uint16_t options; /* 10: 3D and other options (input) */
  uint16_t major;   /* 12: Number of major subdivisions on the dial, 1-10 (input) */
  uint16_t minor;   /* 14: Number of minor subdivisions on the dial, 1-10 (input) */
  uint16_t val;     /* 16: Gauge indicated value, between 0 and range, inclusive (input) */
  uint16_t range;   /* 18: Maximum value (input) */
};

/* FT80X_CMD_GETMATRIX: Retrieves the current matrix coefficients */

struct ft80x_cmd_getmatrix_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_GETMATRIX */
  int32_t a;        /* 4:  Matrix coefficient A (output) */
  int32_t b;        /* 8:  Matrix coefficient B (output) */
  int32_t c;        /* 12: Matrix coefficient C (output) */
  int32_t d;        /* 16: Matrix coefficient D (output) */
  int32_t e;        /* 20: Matrix coefficient E (output) */
  int32_t f;        /* 24: Matrix coefficient F (output) */
};

/* FT80X_CMD_GETPROPS - Get info for image in RAM_G from last CMD_LOADIMAGE command */

struct ft80x_cmd_getprops_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_GETPROPS */
  uint32_t ptr;     /* 4:  Address of image in RAM_G from last CMD_LOADIMAGE command (output) */
  uint32_t w;       /* 8:  Width of image from last CMD_LOADIMAGE command (output) */
  uint32_t h;       /* 12: Height of image from last CMD_LOADIMAGE command (output) */
};

/* FT80X_CMD_GETPTR - Get end address from CMD_INFLATE operation */

struct ft80x_cmd_getptr_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_GETPTR */
  uint32_t result;  /* 4:  End address of decompressed data from CMD_INFLATE (output) */
};

/* FT80X_CMD_GRADCOLOR: Set 3D effects for BUTTON and KEYS highlight colors */

struct ft80x_cmd_gradcolor_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_GRADCOLOR */
  uint32_t c;       /* 4:  New highlight gradient color, as a 24-bit RGB number (input) */
};

/* FT80X_CMD_GRADIENT:  Draw a smooth color gradient */

struct ft80x_cmd_gradient_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_GRADIENT */
  int16_t x0;       /* 4:  X-coordinate of point 0, in pixels (input) */
  int16_t y0;       /* 6:  Y-coordinate of point 0, in pixels (input) */
  uint32_t rgb0;    /* 8:  Color of point 0, as a 24-bit RGB number (input) */
  int16_t x1;       /* 12: X-coordinate of point 1, in pixels (input) */
  int16_t y1;       /* 14: Y-coordinate of point 1, in pixels (input) */
  uint32_t rgb1;    /* 16: Color of point 1, as a 24-bit RGB number (input) */
};

/* FT80X_CMD_INFLATE:  Decompress data into memory */

struct ft80x_cmd_inflate_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_INFLATE */
  uint32_t ptr;     /* 4:  Destination address (input) */
                    /* 8:  Start of compressed data (input)
                     *     Padded to 32-bit alignment  */
};

/* FT80X_CMD_INTERRUPT:  Trigger interrupt INT_CMDFLAG */

struct ft80x_cmd_interrupt_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_INTERRUPT */
  uint32_t ms;      /* 4:  Delay before interrupt triggers in ms (input) */
};

/* FT80X_CMD_KEYS:  Draw a row of keys */

struct ft80x_cmd_keys_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_KEYS */
  int16_t x;        /* 4:  X-coordinate of keys top-left, in pixels (input) */
  int16_t y;        /* 6:  Y-coordinate of keys top-left, in pixels (input) */
  int16_t w;        /* 8:  Width of the keys (input) */
  int16_t h;        /* 10: Height of the keys (input) */
  int16_t font;     /* 12: Bitmap handle to specify the font used in key label (input) */
  uint16_t options; /* 14: 3D and other display options (input) */
                    /* 16: Key labels, one character per key (input)
                     *     Padded to 32-bit alignment */
};

/* FT80X_CMD_LOADIDENTITY:  Set the current matrix to identity */

struct ft80x_cmd_loadidentity_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_LOADIDENTITY */
};

/* FT80X_CMD_LOADIMAGE:  Load a JPEG image */

struct ft80x_cmd_loadimage_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_LOADIMAGE */
  uint32_t ptr;     /* 4:  Destination address (input) */
  uint32_t options; /* 8:  Options (input) */
};

/* FT80X_CMD_LOGO:  Play a device logo animation */

struct ft80x_cmd_logo_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_LOGO */
};

/* FT80X_CMD_MEMCPY:  Copy a block of memory */

struct ft80x_cmd_memcpy_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_MEMCPY */
  uint32_t dest;    /* 4:  Address of the destination memory block (input) */
  uint32_t src;     /* 8:  Address of the source memory block (input) */
  uint32_t num;     /* 12: Number of bytes to copy (input) */
};

/* FT80X_CMD_MEMCRC:  Compute a CRC for memory */

struct ft80x_cmd_memcrc_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_MEMCRC */
  uint32_t ptr;     /* 4:  Starting address of the memory block (input) */
  uint32_t num;     /* 8:  Number of bytes in the source memory block (input) */
  uint32_t result;  /* 12: CRC32 output value (output) */
};

/* FT80X_CMD_MEMSET:  Fill memory with a byte value */

struct ft80x_cmd_memset_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_MEMSET */
  uint32_t ptr;     /* 4:  Starting address of the memory block (input) */
  uint32_t value;   /* 8:  Value to be written to memory (input) */
  uint32_t num;     /* 12: Number of bytes in the memory block (input) */
};

/* FT80X_CMD_MEMWRITE:  Write bytes into memory */

struct ft80x_cmd_memwrite_s
{
  uint32_t cmd;     /* FT80X_CMD_MEMWRITE */
  uint32_t ptr;     /* 4:  Memory address to be written (input) */
  uint32_t num;     /* 8:  Number of bytes to be written (input) */
                    /* 12: Start of data to be written (input)
                     *     Padded to 32-bit alignment. */
};

/* FT80X_CMD_MEMZERO:  Write zero to a block of memory */

struct ft80x_cmd_memzero_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_MEMZERO */
  uint32_t ptr;     /* 4:  Starting address of memory block (input) */
  uint32_t num;     /* 8:  Number of bytes in the memory block (input) */
};

/* FT80X_CMD_NUMBER:  Draw a decimal number */

struct ft80x_cmd_number_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_NUMBER */
  int16_t x;        /* 4:  x-coordinate of text base, in pixels (input) */
  int16_t y;        /* 6:  y-coordinate of text base, in pixels (input) */
  int16_t font;     /* 8:  font to use for text (input) */
  uint16_t options; /* 10: Justification options (input) */
  int32_t n;        /* 12: 32-bit number to display (signed or unsigned) (input) */
};

/* FT80X_CMD_PROGRESS:  Draw a progress bar */

struct ft80x_cmd_progress_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_PROGRESS */
  int16_t x;        /* 4:  X-coordinate of progress bar top-left, in pixels (input) */
  int16_t y;        /* 6:  Y-coordinate of progress bar top-left, in pixels (input) */
  int16_t w;        /* 8:  width of progress bar, in pixels (input) */
  int16_t h;        /* 10: height of progress bar, in pixels (input) */
  uint16_t options; /* 12: 3D and other display options (input) */
  uint16_t val;     /* 14: Displayed value of progress bar, between 0 and range inclusive (input) */
  uint16_t range;   /* 16: Maximum value (input) */
};

/* FT80X_CMD_REGREAD:  Read a register value */

struct ft80x_cmd_regread_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_REGREAD */
  uint32_t ptr;     /* 4:  Address of register to read (input) */
  uint32_t result;  /* 8:  Register value at ptr address (output) */
};

/* FT80X_CMD_ROTATE:  Apply a rotation to the current matrix */

struct ft80x_cmd_rotate_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_ROTATE */
  int32_t a;        /* 4:  Clockwise rotation angle (units 1/65536 of circle) (input) */
};

/* FT80X_CMD_SCALE:  Apply a scale to the current matrix */

struct ft80x_cmd_scale_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_SCALE */
  int32_t sx;       /* 4:  X scale factor (b16) (input) */
  int32_t sy;       /* 8:  Y scale factor (b16) (input) */
};

/* FT80X_CMD_SCREENSAVER -  Start screensaver animation */

struct ft80x_cmd_screensaver_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_SCREENSAVER */
};

/* FT80X_CMD_SCROLLBAR:  Draw a scroll bar */

struct ft80x_cmd_scrollbar_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_SCROLLBAR */
  int16_t x;        /* 4:  X-coordinate of scroll bar top-left, in pixels (input) */
  int16_t y;        /* 6:  Y-coordinate of scroll bar top-left, in pixels (input) */
  int16_t w;        /* 8:  Width of scroll bar, in pixels (input) */
  int16_t h;        /* 10: Height of scroll bar, in pixels (input) */
  uint16_t options; /* 12: 3D and other display options (input) */
  uint16_t val;     /* 14: Displayed value of scroll bar, between 0 and range inclusive (input) */
  uint16_t size;    /* 16: Size of the scrollbar (input) */
  uint16_t range;   /* 18: Maximum value (input) */
};

/* FT80X_CMD_SETFONT - Register custom font into FT80x co-processor */

struct ft80x_cmd_setfont_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_SETFONT */
  uint32_t font;    /* 4:  Bitmap handle (input) */
  uint32_t ptr;     /* 8:  Metric block address in RAM (32-bit aligned) (input) */
};

/* FT80X_CMD_SETMATRIX: Write current matrix as a bitmap transform */

struct ft80x_cmd_setmatrix_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_SETMATRIX */
};

/* FT80X_CMD_SKETCH:  Start a continuous sketch update */

struct ft80x_cmd_sketch_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_SKETCH */
  int16_t x;        /* 4:  X-coordinate of sketch area top-left, in pixels (input) */
  int16_t y;        /* 6:  Y-coordinate of sketch area top-left, in pixels (input) */
  uint16_t w;       /* 8:  Width of sketch area, in pixels (input) */
  uint16_t h;       /* 10: Height of sketch area, in pixels (input) */
  uint32_t ptr;     /* 12: Base address of sketch bitmap (input) */
  uint16_t format;  /* 16: Format of sketch bitmap, either L1 or L8 (input) */
};

/* FT80X_CMD_SLIDER:  Draw a slider */

struct ft80x_cmd_slider_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_SLIDER */
  int16_t x;        /* 4:  X-coordinate of slider top-left, in pixels (input) */
  int16_t y;        /* 6:  Y-coordinate of slider top-left, in pixels (input) */
  int16_t w;        /* 8:  width of slider, in pixels (input) */
  int16_t h;        /* 10: height of slider, in pixels (input) */
  uint16_t options; /* 12: 3D and other display option (input) */
  uint16_t val;     /* 14: Displayed value of slider, between 0 and range inclusive (input) */
  uint16_t range;   /* 16: Maximum value (input) */
};

/* FT80X_CMD_SNAPSHOT:  Take a snapshot of the current screen */

struct ft80x_cmd_snapshot_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_SNAPSHOT */
  uint32_t ptr;     /* 4:  Snapshot destination address, in RAM_G (input) */
};

/* FT80X_CMD_SPINNER: Start an animated spinner */

struct ft80x_cmd_spinner_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_SPINNER */
  int16_t x;        /* 4:  X coordinate of top left of spinner (input) */
  int16_t y;        /* 6:  Y coordinate of top left of spinner (input) */
  uint16_t style;   /* 8:  Style of the spinner (input) */
  uint16_t scale;   /* 10: Scaling coefficient of the spinner (input) */
};

/* FT80X_CMD_STOP:  Stop any spinner, screensaver, or sketch */

struct ft80x_cmd_stop_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_STOP */
};

/* FT80X_CMD_SWAP: Swap the current display list */

struct ft80x_cmd_swap_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_SWAP */
};

/* FT80X_CMD_TEXT: Draw text */

struct ft80x_cmd_text_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_TEXT */
  int16_t x;        /* 4:  X-coordinate of text base, in pixels (input) */
  int16_t y;        /* 6:  Y-coordinate of text base, in pixels (input) */
  int16_t font;     /* 8:  Font to use for text (input) */
  uint16_t options; /* 10: Justification options (input) */
                    /* 12: Start of text string (input)
                     *     Must be NUL terminated and padded to 32-bit alignment */
};

/* FT80X_CMD_TOGGLE:  Draw a toggle switch */

struct ft80x_cmd_toggle_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_TOGGLE */
  int16_t x;        /* 4:  x-coordinate of top-left of toggle, in pixels (input) */
  int16_t y;        /* 6:  y-coordinate of top-left of toggle, in pixels (input) */
  int16_t w;        /* 8:  width of toggle, in pixels (input) */
  int16_t font;     /* 10: font to use for text (input) */
  uint16_t options; /* 12: 3D options (input) */
  uint16_t state;   /* 14: state of the toggle: 0 is off, 65535 is on (input) */
                    /* 16: String label for toggle (0xff separated) (input)
                     *     Padded to 32-bit boundary */
};

/* FT80X_CMD_TRACK - Enable co-processor to track touch on graphics object */

struct ft80x_cmd_track_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_TRACK */
  int16_t x;        /* 4:  Linear tracker: X-coordinate of track area top-left, in pixels (input)
                     *     Rotary tracker: X-coordinate of track area center, in pixels (input) */
  int16_t y;        /* 6:  Linear tracker: Y-coordinate of track area top-left, in pixels (input)
                     *     Rotary tracker: Y-coordinate of track area center, in pixels (input) */
  int16_t w;        /* 8:  Width of track area, in pixels (input) */
  int16_t h;        /* 10: Height of track area, in pixels (input) */
  int16_t tag;      /* 12: tag of the graphics object to be tracked (input) */
};

/* FT80X_CMD_TRANSLATE:  Apply a translation to the current matrix */

struct ft80x_cmd_translate_s
{
  uint32_t cmd;     /* 0:  FT80X_CMD_TRANSLATE */
  int32_t tx;       /* 4:  X translate factor (b16) (input) */
  int32_t ty;       /* 8:  Y translate factor (b16) (input) */
};

/* FT80x IOCTL Argument Structures **********************************************************/
/* This container structure is used by FT80X_IOC_CREATEDL and FT80X_IOC_APPENDDL.  It
 * defines the list of display commands to be written into display list memory.
 */

struct ft80x_displaylist_s
{
  uint32_t dlsize;          /* Size of the display list in bytes */
  struct ft80x_dlcmd_s cmd; /* First command in the display list  */
};

/* This structure is used with the FT80X_IOC_GETRAMDL, FT80X_IOC_PUTRAMG, and
 * FT80X_IOC_PUTRAMCMD IOCTL commands to access particular memory regions via an offset.
 *
 * NOTES:
 *   - For FT80X_IOC_GET* commands, the value is an output; for FT80X_IOC_PUT* command, the
 *     value is an input.
 */

struct ft80x_relmem_s
{
  uint32_t offset;         /* 32-bit aligned offset into the display list */
  uint32_t nbytes;         /* Number of bytes to access */
  FAR void *value;         /* Value(s) read from memory base + offset */
};

/* This structure is used with the FT80X_IOC_EVENTNOTIFY IOCTL command to describe
 * the requested event notification.
 */

enum ft80x_notify_e
{
  FT80X_NOTIFY_SWAP = 0,     /* Bit 0: Display swap occurred */
  FT80X_NOTIFY_TOUCH,        /* Touch-screen touch detected */
  FT80X_NOTIFY_TAG,          /* Touch-screen tag value change */
  FT80X_NOTIFY_SOUND,        /* Sound effect ended */
  FT80X_NOTIFY_PLAYBACK,     /* Audio playback ended */
  FT80X_NOTIFY_CMDEMPTY,     /* Bit 5: Command FIFO empty */
  FT80X_NOTIFY_CMDFLAG,      /* Command FIFO flag */
  FT80X_NOTIFY_CONVCOMPLETE  /* Touch-screen conversions completed */
};

struct ft80x_notify_s
{
  int signo;                 /* Notify using this signal number */
  pid_t pid;                 /* Send the notification to this task */
  enum ft80x_notify_e event; /* Notify on this event */
  bool enable;               /* True: enable notification; false: disable */
};

/* This structure is used with the FT80X_IOC_GETREGnn and FT80X_IOC_PUTREGnn
 * IOCTL commands to describe the requested register access.
 *
 * NOTES:
 *   - For FT80X_IOC_GETREGnn, the value is an output; for FT80X_IOC_PUTREGnn,
 *     the value is an input.
 *   - The union field used to access the register value depends on the width
 *     of the requested access.
 */

struct ft80x_register_s
{
  uint32_t addr;             /* 32-bit aligned register address */
  union
  {
    uint8_t u8;              /* 8-bit register value */
    uint16_t u16;            /* 16-bit register value */
    uint32_t u32;            /* 32-bit register value */
  } value;
};

struct ft80x_registers_s
{
  uint32_t addr;             /* 32-bit aligned start register address */
  uint8_t nregs;             /* Number of 32-bit registers to be accessed */
  FAR uint32_t *value;       /* A pointer to an array of 32-bit register values */
};

/* Used with FT80X_IOC_FADE: */

struct ft80x_fade_s
{
  uint8_t duty ;             /* Terminal backlight duty as a percentage (0-100) */
  uint16_t delay;            /* Total number of milliseconds for the fade (10-16700)*/
};

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************************
 * Name: ft80x_register
 *
 * Description:
 *   Configure the ADS7843E to use the provided SPI device instance.  This will register
 *   the driver as /dev/ft800 or /dev/ft801, depending upon the configuration.
 *
 * Input Parameters:
 *   spi   - An SPI driver instance
 *   i2c   - An I2C master driver instance
 *   lower - Persistent board configuration data / lower half interface
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is returned to indicate
 *   the nature of the failure.
 *
 ********************************************************************************************/

#if defined(CONFIG_LCD_FT80X_SPI)
int ft80x_register(FAR struct spi_dev_s *spi,
                   FAR const struct ft80x_config_s *lower);
#elif defined(CONFIG_LCD_FT80X_I2C)
int ft80x_register(FAR struct i2c_master_s *i2c,
                   FAR const struct ft80x_config_s *lower);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LCD_FT80X */
#endif /* __INCLUDE_NUTTX_LCD_FT80X_H */

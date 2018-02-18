/****************************************************************************
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_LCD_FT80X_H
#define __INCLUDE_NUTTX_LCD_FT80X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/lcd/lcd_ioctl.h>

#ifdef CONFIG_LCD_FT80X

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

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

/* FT80x IOCTL commands *****************************************************
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
 *   struct ft80x_cmd_dlstart_s dlstart;  # Mark the start of the display list
 *                                        # Various display commands follow...
 *   FT80X_DISPLAY();                     # Finish the last display
 *   struct ft80x_cmd_swap_s swap;        # Swap to the new display list
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
 * either case but must be subsequently obtained using FT80X_IOC_GETDL32.
 *
 * FT80X_IOC_GETDL32:
 *   Description:  Read a 32-bit value from the display list.
 *   Argument:     A reference to an instance of struct ft80x_dlmem_s below.
 *   Returns:      The 32-bit value read from the display list.
 *
 * FT80X_IOC_GETREG8:
 *   Description:  Read an 8-bit register value from the FT80x.
 *   Argument:     A reference to an instance of struct ft80x_register_s below.
 *   Returns:      The 8-bit value read from the display list.
 *
 * FT80X_IOC_GETREG16:
 *   Description:  Read a 16-bit register value from the FT80x.
 *   Argument:     A reference to an instance of struct ft80x_register_s below.
 *   Returns:      The 16-bit value read from the display list.
 *
 * FT80X_IOC_GETREG32:
 *   Description:  Read a 32-bit register value from the FT80x.
 *   Argument:     A reference to an instance of struct ft80x_register_s below.
 *   Returns:      The 32-bit value read from the display list.
 *
 * FT80X_IOC_PUTREG8:
 *   Description:  Write an 8-bit register value to the FT80x.
 *   Argument:     A reference to an instance of struct ft80x_register_s below.
 *   Returns:      None.
 *
 * FT80X_IOC_PUTREG16:
 *   Description:  Write a 16-bit  register value to the FT80x.
 *   Argument:     A reference to an instance of struct ft80x_register_s below.
 *   Returns:      None.
 *
 * FT80X_IOC_PUTREG32:
 *   Description:  Write a 32-bit  register value to the FT80x.
 *   Argument:     A reference to an instance of struct ft80x_register_s below.
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
 */

#define FT80X_IOC_CREATEDL          _LCDIOC(FT80X_NIOCTL_BASE + 0)
#define FT80X_IOC_APPENDDL          _LCDIOC(FT80X_NIOCTL_BASE + 1)
#define FT80X_IOC_GETDL32           _LCDIOC(FT80X_NIOCTL_BASE + 2)
#define FT80X_IOC_GETREG8           _LCDIOC(FT80X_NIOCTL_BASE + 3)
#define FT80X_IOC_GETREG16          _LCDIOC(FT80X_NIOCTL_BASE + 4)
#define FT80X_IOC_GETREG32          _LCDIOC(FT80X_NIOCTL_BASE + 5)
#define FT80X_IOC_PUTREG8           _LCDIOC(FT80X_NIOCTL_BASE + 6)
#define FT80X_IOC_PUTREG16          _LCDIOC(FT80X_NIOCTL_BASE + 7)
#define FT80X_IOC_PUTREG32          _LCDIOC(FT80X_NIOCTL_BASE + 8)
#define FT80X_IOC_EVENTNOTIFY       _LCDIOC(FT80X_NIOCTL_BASE + 9)

/* FT80x Display List Commands **********************************************/
/* Host commands.  3 word commands.  The first word begins with 0b01, the next two are zero */

#define FT80X_CMD_ACTIVE           0x00        /* Switch from Standby/Sleep modes to active mode */
#define FT80X_CMD_STANDBY          0x41        /* Put FT80x core to standby mode */
#define FT80X_CMD_SLEEP            0x42        /* Put FT80x core to sleep mode */
#define FT80X_CMD_PWRDOWN          0x50        /* Switch off 1.2V internal regulator */
#define FT80X_CMD_CLKEXT           0x44        /* Enable PLL input from oscillator or external clock */
#define FT80X_CMD_CLK48M           0x62        /* Switch PLL output clock to 48MHz (default). */
#define FT80X_CMD_CLK36M           0x61        /* Switch PLL output clock to 36MHz */
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

/* BITMAP_SIZE (0x08) - Set the screen drawing of bitmaps for the current
 * handle
 */

#define FT80X_BITMAP_SIZE(filter,wrapx,wrapy,width,height) \
  ((8 << 24) | (((filter) & 1) << 20) | (((wrapx) & 1) << 19) | \
   (((wrapy) & 1) << 18) | (((width) & 511) << 9) | (((height) & 511) << 0))

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

#define FT80X_BLEND_FUNC(src,dst) \
  ((11 << 24) | (((src) & 7) << 3) | (((dst) & 7) << 0))

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

/* STENCIL_MASK (0x13) - Control the writing of individual bits in the
 * stencil planes
 */

#define FT80X_STENCIL_MASK(mask) \
  ((19 << 24) | (((mask) & 255) << 0))

/* STENCIL_OP (0x0c) - Set stencil test actions */

#define FT80X_STENCIL_OP(sfail,spass) \
  ((12 << 24) | (((sfail) & 7) << 3) | (((spass) & 7) << 0))

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

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* FT80x Lower Half Interface Definitions ***********************************/
/* Pins relevant to software control.  The FT80X is a 48-pin part.  Most of
 * the pins are associated with the TFT panel and other board-related
 * support.  A few a relevant to software control of the part.  Those are
 * listed here:
 *
 * FT80X PIN  DIR DESCRIPTION
 *  3          I  SPI: SCLK, I2C: SCL
 *  4          I  SPI: MISO, I2C: SDA
 *  5         I/O SPI: MOSI
 *  6          I  SPI: nCS
 *  11        OD  nINT Host interrupt
 *  12         *  nPD  Power down input
 *
 * SCL/SDA, SCLK/MISO/MOSI/nCS are handled by generic I2C or SPI logic. nInt
 * and nPD are directly managed by this interface.
 */

/* A reference to a structure of this type must be passed to the FT80X
 * driver.  This structure provides information about the configuration
 * of the FT80X and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.  The
 * memory may be read-only.
 */

struct ft80x_config_s
{
  /* Device characterization */

  uint32_t init_frequency;  /* I2C/SPI initialization frequency */
  uint32_t op_frequency;    /* I2C/SPI operational frequency */
#ifdef CONFIG_LCD_FT80X_I2C
  uint8_t address;          /* 7-bit I2C address */
#endif

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the FT80X driver from differences in GPIO
   * interrupt handling by varying boards and MCUs. Interrupts should be
   * configured on the falling edge of nINT.
   *
   *   attach  - Attach the ADS7843E interrupt handler to the GPIO interrupt
   *   enable  - Enable or disable the GPIO interrupt
   *   clear   - Acknowledge/clear any pending GPIO interrupt
   *   pwrdown - Power down the FT80X
   *   destroy - The driver has been unlinked. Cleanup as necessary.
   */

  CODE int  (*attach)(FAR const struct ft80x_config_s *state, xcpt_t isr,
                      FAR void *arg);
  CODE void (*enable)(FAR const struct ft80x_config_s *state, bool enable);
  CODE void (*clear)(FAR const struct ft80x_config_s *state);
  CODE bool (*pwrdown)(FAR const struct ft80x_config_s *state, bool pwrdown);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  CODE void (*destroy)(FAR const struct ft80x_config_s *state);
#endif
};

/* FT80x Display List Command Structures ************************************/
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
  char s[1];        /* 16: Start of button label string (input)
                     *     NUL terminated and padded to 32-bit alignment */
};

#define SIZEOF_FT80X_CMD_BUTTON_S(n) (sizeof(struct ft80x_cmd_button_s) + (n) - 1)

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
  uint8_t data[1];  /* 8:  Start of compressed data (input)
                     *     Padded to 32-bit alignment  */
};

#define SIZEOF_FT80X_CMD_INFLATE_S(n) (sizeof(struct ft80x_cmd_inflate_s) + (n) - 1)

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
  char s[1];        /* 16: Key labels, one character per key (input)
                     *     Padded to 32-bit alignment */
};

#define SIZEOF_FT80X_CMD_KEYS_S(n) (sizeof(struct ft80x_cmd_keys_s) + (n) - 1)

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
  uint8_t data[1];  /* 12: Start of data to be written (input)
                     *     Padded to 32-bit alignment. */
};

#define SIZEOF_FT80X_CMD_MEMWRITE_S(n) (sizeof(struct ft80x_cmd_memwrite_s) + (n) - 1)

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
  char s[1];        /* 12: Start of text string (input)
                     *     Must be NUL terminated and padded to 32-bit alignment */
};

#define SIZEOF_FT80X_CMD_TEXT_S(n) (sizeof(struct ft80x_cmd_text_s) + (n) - 1)

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
  char s[1]         /* 16: String label for toggle (0xff separated) (input)
                     *     Padded to 32-bit boundary */
};

#define SIZEOF_FT80X_CMD_TOGGLE_S(n) (sizeof(struct ft80x_cmd_toggle_s) + (n) - 1)

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

/* FT80x IOCTL Argument Structures ******************************************/
/* This container structure is used by FT80X_IOC_CREATEDL and FT80X_IOC_APPENDDL.  It
 * and defines the list of display commands to be written into display list memory.
 */

struct ft80x_displaylist_s
{
  uint32_t dlsize;          /* Size of the display list in bytes */
  struct ft80x_dlcmd_s cmd; /* First command in the display list  */
};

/* This structure is used with the FT80X_IOC_GETDL32 IOCTL command to
 * retrieve the result of the display list operation from display list memory.
 */

struct ft80x_dlmem_s
{
  uint32_t offset;         /* 32-bit aligned offset into the display list */
  uint32_t value;          /* 32-bit value read from display list + offset */
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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ft80x_register
 *
 * Description:
 *   Configure the ADS7843E to use the provided SPI device instance.  This
 *   will register the driver as /dev/ft80x.
 *
 * Input Parameters:
 *   spi     - An SPI driver instance
 *   i2c     - An I2C master driver instance
 *   lower   - Persistent board configuration data / lower half interface
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

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

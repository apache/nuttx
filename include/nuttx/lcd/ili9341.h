/**************************************************************************************
 * include/nuttx/lcd/ili9341.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
 **************************************************************************************/

#ifndef __INCLUDE_NUTTX_LCD_ILI9341_H
#define __INCLUDE_NUTTX_LCD_ILI9341_H

/**************************************************************************************
 * Included Files
 **************************************************************************************/

#include <nuttx/config.h>

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/
/* ILI9341 ID code */

#define ILI9341_DEVICE_CODE                    0x9341

/* ILI9341 LCD Register Addresses *****************************************************/

/* Level 1 commands */

#define ILI9341_NOP                            0x00 /* Nop operation */
#define ILI9341_SOFTWARE_RESET                 0x01 /* Software reset */
#define ILI9341_READ_DISP_ID                   0x04 /* Read Display Identification information */
#define ILI9341_READ_DISP_STATUS               0x09 /* Read display status*/
#define ILI9341_READ_DISP_POWER_MODE           0x0a /* Read display power mode */
#define ILI9341_READ_DISP_MADCTRL              0x0b /* Read display MADCTL */
#define ILI9341_READ_DISP_PIXEL_FORMAT         0x0c /* Read display pixel forma */
#define ILI9341_READ_DISP_IMAGE_FORMAT         0x0d /* Read display image format */
#define ILI9341_READ_DISP_SIGNAL_MODE          0x0e /* Read display signal mode */
#define ILI9341_READ_DISP_SELF_DIAGNOSTIC      0x0f /* Read display self-diagnostic result */
#define ILI9341_ENTER_SLEEP_MODE               0x10 /* Enter sleep mode */
#define ILI9341_SLEEP_OUT                      0x11 /* Sleep out */
#define ILI9341_PARTIAL_MODE_ON                0x12 /* Partial mode on */
#define ILI9341_NORMAL_DISP_MODE_ON            0x13 /* Normal display mode on */
#define ILI9341_DISP_INVERSION_OFF             0x20 /* Display inversion off */
#define ILI9341_DISP_INVERSION_ON              0x21 /* Display inversion on */
#define ILI9341_GAMMA_SET                      0x26 /* Gamma set */
#define ILI9341_DISPLAY_OFF                    0x28 /* Display off */
#define ILI9341_DISPLAY_ON                     0x29 /* Display on */
#define ILI9341_COLUMN_ADDRESS_SET             0x2a /* Column address set */
#define ILI9341_PAGE_ADDRESS_SET               0x2b /* Page address set */
#define ILI9341_MEMORY_WRITE                   0x2c /* Memory write */
#define ILI9341_COLOR_SET                      0x2d /* Color set */
#define ILI9341_MEMORY_READ                    0x2e /* Memory read */
#define ILI9341_PARTIAL_AREA                   0x30 /* Partial area */
#define ILI9341_VERT_SCROLL_DEFINITION         0x33 /* Vertical scrolling definition */
#define ILI9341_TEARING_EFFECT_LINE_OFF        0x34 /* Tearing effect line off */
#define ILI9341_TEARING_EFFECT_LINE_ON         0x35 /* Tearing effect line on */
#define ILI9341_MEMORY_ACCESS_CONTROL          0x36 /* Memory Access control */
#define ILI9341_VERT_SCROLL_START_ADDRESS      0x37 /* Vertical scrolling start address */
#define ILI9341_IDLE_MODE_OFF                  0x38 /* Idle mode off */
#define ILI9341_IDLE_MODE_ON                   0x39 /* Idle mode on */
#define ILI9341_PIXEL_FORMAT_SET               0x3a /* Pixel Format set */
#define ILI9341_WRITE_MEMORY_CONTINUE          0x3c /* Write memory continue */
#define ILI9341_READ_MEMORY_CONTINUE           0x3e /* Read memory continue */
#define ILI9341_SET_TEAR_SCANLINE              0x44 /* Set tear scanline */
#define ILI9341_GET_SCANLINE                   0x45 /* Get scanline */
#define ILI9341_WRITE_DISPLAY_BRIGHTNESS       0x51 /* Write display brightness */
#define ILI9341_READ_DISPLAY_BRIGHTNESS        0x52 /* Read display brightness */
#define ILI9341_WRITE_CTRL_DISPLAY             0x53 /* Write control display */
#define ILI9341_READ_CTRL_DISPLAY              0x54 /* Read control display */
#define ILI9341_WRITE_CONTENT_ADAPT_BRIGHTNESS 0x55 /* write content adaptive brightness control */
#define ILI9341_READ_CONTENT_ADAPT_BRIGHTNESS  0x56 /* Read content adaptive brightness control */
#define ILI9341_WRITE_MIN_CAB_LEVEL            0x5e /* Write CABC minimum brightness */
#define ILI9341_READ_MIN_CAB_LEVEL             0x5f /* Read CABC minimum brightness */
#define ILI9341_READ_ID1                       0xda /* Read ID1 */
#define ILI9341_READ_ID2                       0xdb /* Read ID2 */
#define ILI9341_READ_ID3                       0xdc /* Read ID3 */

/* Level 2 Commands */

#define ILI9341_RGB_SIGNAL_CONTROL             0xb0 /* RGB interface signal control */
#define ILI9341_FRAME_RATE_CONTROL_NORMAL      0xb1 /* Frame control */
#define ILI9341_FRAME_RATE_CONTROL_IDLE_8COLOR 0xb2 /* Frame control in idle mode */
#define ILI9341_FRAME_RATE_CONTROL_PARTIAL     0xb3 /* Frame control in partial mode */
#define ILI9341_DISPLAY_INVERSION_CONTROL      0xb4 /* Display inversion control */
#define ILI9341_BLANKING_PORCH_CONTROL         0xb5 /* Blanking porch control */
#define ILI9341_DISPLAY_FUNCTION_CTL           0xb6 /* Display function control */
#define ILI9341_ENTRY_MODE_SET                 0xb7 /* Entry mode set */
#define ILI9341_BACKLIGHT_CONTROL_1            0xb8 /* Backlight control1 */
#define ILI9341_BACKLIGHT_CONTROL_2            0xb9 /* Backlight control2 */
#define ILI9341_BACKLIGHT_CONTROL_3            0xba /* Backlight control3 */
#define ILI9341_BACKLIGHT_CONTROL_4            0xbb /* Backlight control 4 */
#define ILI9341_BACKLIGHT_CONTROL_5            0xbc /* Backlight control 5 */
#define ILI9341_BACKLIGHT_CONTROL_7            0xbe /* Backlight control 7 */
#define ILI9341_BACKLIGHT_CONTROL_8            0xbf /* Backlight control 8 */
#define ILI9341_POWER_CONTROL_1                0xc0 /* Power control 1 */
#define ILI9341_POWER_CONTROL_2                0xc1 /* Power control 2 */
#define ILI9341_VCOM_CONTROL_1                 0xc5 /* VCOM control 1 */
#define ILI9341_VCOM_CONTROL_2                 0xc7 /* VCOM control 2 */
#define ILI9341_POWER_CONTROL_A                0xcb /* Power control A */
#define ILI9341_POWER_CONTROL_B                0xcf /* Power control B */
#define ILI9341_NVMEM_WRITE                    0xd0 /* NV memory write */
#define ILI9341_NVMEM_PROTECTION_KEY           0xd1 /* NV memory protection key */
#define ILI9341_NVMEM_STATUS_READ              0xd2 /* NV memory status read */
#define ILI9341_READ_ID4                       0xd3 /* Read ID4 */
#define ILI9341_POSITIVE_GAMMA_CORRECTION      0xe0 /* Positive gamma correction */
#define ILI9341_NEGATIVE_GAMMA_CORRECTION      0xe1 /* Negative gamma correction */
#define ILI9341_DIGITAL_GAMMA_CONTROL_1        0xe2 /* Digital gamma control 1 */
#define ILI9341_DIGITAL_GAMMA_CONTROL_2        0xe3 /* Digital gamma control 2 */
#define ILI9341_DRIVER_TIMING_CTL_A            0xe8 /* Driver timing control A */
#define ILI9341_DRIVER_TIMING_CTL_B            0xea /* Driver timing control B */
#define ILI9341_POWER_ON_SEQUENCE_CONTROL      0xed /* Power-on sequence control */
#define ILI9341_ENABLE_3_GAMMA_CONTROL         0xf2 /* Enable 3g gamma control */
#define ILI9341_INTERFACE_CONTROL              0xf6 /* Interface control */
#define ILI9341_PUMP_RATIO_CONTROL             0xf7 /* Pump ration control */

/* ILI9341 LCD Register Bit Definitions ***********************************************/
/* Memory Access control */

#define ILI9341_MEMORY_ACCESS_CONTROL_MH       (1 << 2) /* Horizontal refresh order */
#define ILI9341_MEMORY_ACCESS_CONTROL_BGR      (1 << 3) /* RGB/BGR order */
#define ILI9341_MEMORY_ACCESS_CONTROL_ML       (1 << 4) /* Vertical refresh order */
#define ILI9341_MEMORY_ACCESS_CONTROL_MV       (1 << 5) /* Row/column exchange */
#define ILI9341_MEMORY_ACCESS_CONTROL_MX       (1 << 6) /* Column address order */
#define ILI9341_MEMORY_ACCESS_CONTROL_MY       (1 << 7) /* Row address order */

/* Display function control */

#define ILI9341_DISP_FUNC_CTL_ISC_SHIFT        (0)
#define ILI9341_DISP_FUNC_CTL_ISC_MASK         (15 << ILI9341_DISP_FUNC_CTL_ISC_SHIFT)
#  define ILI9341_DISP_FUNC_CTL_ISC(n)         ((n) << ILI9341_DISP_FUNC_CTL_ISC_SHIFT))
#define ILI9341_DISP_FUNC_CTL_SM               (1 << 4)
#define ILI9341_DISP_FUNC_CTL_SS               (1 << 5)
#define ILI9341_DISP_FUNC_CTL_GS               (1 << 6)
#define ILI9341_DISP_FUNC_CTL_REV              (1 << 7)

/**************************************************************************************
 * Public Types
 **************************************************************************************/

/**************************************************************************************
 * Public Data
 **************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/**************************************************************************************
 * Public Function Prototypes
 **************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_ILI9341_H */

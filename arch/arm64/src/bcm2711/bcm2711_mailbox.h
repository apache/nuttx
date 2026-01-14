/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_mailbox.h
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

#ifndef __ARCH_ARM64_SRC_BCM2711_BCM2711_MAILBOX_H
#define __ARCH_ARM64_SRC_BCM2711_BCM2711_MAILBOX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/compiler.h>

#include <arch/chip/chip.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "hardware/bcm2711_mailbox.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* List of mailbox channels */

#define MBOX_CHAN_POWER (0x0)    /* Power management interface */
#define MBOX_CHAN_FRAMEBUF (0x1) /* Frame buffer */
#define MBOX_CHAN_VUART (0x2)    /* Virtual UART */
#define MBOX_CHAN_VCHIQ (0x3)    /* VCHIQ interface */
#define MBOX_CHAN_LED (0x4)      /* LED interface */
#define MBOX_CHAN_BUTTON (0x5)   /* Button interface */
#define MBOX_CHAN_TOUCH (0x6)    /* Touchscreen interface */
#define MBOX_CHAN_COUNT (0x7)    /* Counter */
#define MBOX_CHAN_TAGS (0x8)     /* Tags (ARM to VC) */
#define MBOX_CHAN_TAGSVC (0x9)   /* Tags (VC to ARM) */

/* List of tags for mailbox communication */

#define MBOX_TAG_FWREV (0x1)           /* Get firmware revision */
#define MBOX_TAG_BMODEL (0x10001)      /* Get board model */
#define MBOX_TAG_BREV (0x10002)        /* Get board revision */
#define MBOX_TAG_MAC (0x10003)         /* Get board MAC address */
#define MBOX_TAG_SER (0x10004)         /* Get board serial number */
#define MBOX_TAG_ARMMEM (0x10005)      /* Get ARM memory */
#define MBOX_TAG_VCMEM (0x10006)       /* Get VideoCore memory */
#define MBOX_TAG_CLOCKS (0x10007)      /* Get clocks */
#define MBOX_TAG_CMDLINE (0x50001)     /* Get command line string */
#define MBOX_TAG_DMACHAN (0x60001)     /* Get DMA channels */
#define MBOX_TAG_DMACHAN (0x60001)     /* Get DMA channels */
#define MBOX_TAG_GETPWR (0x20001)      /* Get power state */
#define MBOX_TAG_GETTIME (0x20002)     /* Get power-on wait time */
#define MBOX_TAG_SETPWR (0x28001)      /* Set power state */
#define MBOX_TAG_GETCLKS (0x30001)     /* Get clock state */
#define MBOX_TAG_SETCLKS (0x38001)     /* Set clock state */
#define MBOX_TAG_GETCLKR (0x30002)     /* Get clock rate */
#define MBOX_TAG_GETLED (0x30041)      /* Get on-board LED status */
#define MBOX_TAG_TESTLED (0x34041)     /* Test on-board LED status */
#define MBOX_TAG_SETLED (0x38041)      /* Set on-board LED status */
#define MBOX_TAG_GETCLKRM (0x30047)    /* Get measured clock rate */
#define MBOX_TAG_SETCLKR (0x38002)     /* Set clock rate */
#define MBOX_TAG_GETMAXCLKR (0x30004)  /* Get max clock rate */
#define MBOX_TAG_GETMINCLKR (0x30007)  /* Get min clock rate */
#define MBOX_TAG_GETTURBO (0x30009)    /* Get turbo state */
#define MBOX_TAG_SETTURBO (0x38009)    /* Set turbo state */
#define MBOX_TAG_GETVOLT (0x30003)     /* Get voltage */
#define MBOX_TAG_SETVOLT (0x38003)     /* Set voltage */
#define MBOX_TAG_GETMAXVOLT (0x30005)  /* Get max voltage */
#define MBOX_TAG_GETMINVOLT (0x30008)  /* Get min voltage */
#define MBOX_TAG_GETTEMP (0x30006)     /* Get temperature */
#define MBOX_TAG_GETMAXTEMP (0x3000a)  /* Get max temperature */
#define MBOX_TAG_ALLOCMEM (0x3000c)    /* Allocate memory */
#define MBOX_TAG_LOCKMEM (0x3000d)     /* Lock memory */
#define MBOX_TAG_UNLOCKMEM (0x3000e)   /* Lock memory */
#define MBOX_TAG_RELMEM (0x3000f)      /* Release memory */
#define MBOX_TAG_EXEC (0x30010)        /* Execute code */
#define MBOX_TAG_DISPMANX (0x30014)    /* Get Dispmanx mem handle */
#define MBOX_TAG_EDID (0x30020)        /* Get EDID block */
#define MBOX_TAG_FALLOC (0x40001)      /* Allocate frame buffer */
#define MBOX_TAG_FREL (0x48001)        /* Release frame buffer */
#define MBOX_TAG_BLANK (0x40002)       /* Blank screen */
#define MBOX_TAG_GETDISP (0x40003)     /* Get physical display w/h */
#define MBOX_TAG_TESTDISP (0x44003)    /* Test physical display w/h */
#define MBOX_TAG_SETDISP (0x48003)     /* Set physical display w/h */
#define MBOX_TAG_GETVBUF (0x40004)     /* Get virtual buffer w/h */
#define MBOX_TAG_TESTVBUF (0x44004)    /* Test virtual buffer w/h */
#define MBOX_TAG_SETVBUF (0x48004)     /* Set virtual buffer w/h */
#define MBOX_TAG_GETDEPTH (0x40005)    /* Get depth */
#define MBOX_TAG_TESTDEPTH (0x44005)   /* Test depth */
#define MBOX_TAG_SETDEPTH (0x48005)    /* Set depth */
#define MBOX_TAG_GETPIXORD (0x40006)   /* Get pixel order */
#define MBOX_TAG_TESTPIXORD (0x44006)  /* Test pixel order */
#define MBOX_TAG_SETPIXORD (0x48006)   /* Set pixel order */
#define MBOX_TAG_GETALPHA (0x40007)    /* Get alpha */
#define MBOX_TAG_TESTALPHA (0x44007)   /* Test alpha */
#define MBOX_TAG_SETALPHA (0x48007)    /* Set alpha */
#define MBOX_TAG_GETPITCH (0x40008)    /* Get pitch */
#define MBOX_TAG_GETVIRTOFF (0x40009)  /* Get virtual offset */
#define MBOX_TAG_TESTVIRTOFF (0x44009) /* Test virtual offset */
#define MBOX_TAG_SETVIRTOFF (0x48009)  /* Set virtual offset */
#define MBOX_TAG_GETOVSCAN (0x4000a)   /* Get overscan */
#define MBOX_TAG_TESTOVSCAN (0x4400a)  /* Test overscan */
#define MBOX_TAG_SETOVSCAN (0x4800a)   /* Set overscan */
#define MBOX_TAG_GETPALETTE (0x4000b)  /* Get palette */
#define MBOX_TAG_TESTPALETTE (0x4400b) /* Test palette */
#define MBOX_TAG_SETPALETTE (0x4800b)  /* Set palette */
#define MBOX_TAG_CURSINFO (0x08010)    /* Set cursor info */
#define MBOX_TAG_CURSSTATE (0x08011)   /* Set cursor state */

/* Power domain IDs */

#define MBOX_PDOM_SDCARD (0x0)
#define MBOX_PDOM_UART0 (0x1)
#define MBOX_PDOM_UART1 (0x2)
#define MBOX_PDOM_USBHCD (0x3)
#define MBOX_PDOM_I2C0 (0x4)
#define MBOX_PDOM_I2C1 (0x5)
#define MBOX_PDOM_I2C2 (0x6)
#define MBOX_PDOM_SPI (0x7)
#define MBOX_PDOM_CCP2TX (0x8)
#define MBOX_PDOM_UNK1 (0x9) /* Unknown, RPi4B */
#define MBOX_PDOM_UNK2 (0xa) /* Unknown, RPi4B */

/* Clock IDs */

#define MBOX_CLK_EMMC (0x1)
#define MBOX_CLK_UART (0x2)
#define MBOX_CLK_ARM (0x3)
#define MBOX_CLK_CORE (0x4)
#define MBOX_CLK_V3D (0x5)
#define MBOX_CLK_H264 (0x6)
#define MBOX_CLK_ISP (0x7)
#define MBOX_CLK_SDRAM (0x8)
#define MBOX_CLK_PIXEL (0x9)
#define MBOX_CLK_PWM (0xa)
#define MBOX_CLK_HEVC (0xb)
#define MBOX_CLK_EMMC2 (0xc)
#define MBOX_CLK_M2MC (0xd)
#define MBOX_CLK_PIXELBVB (0xe)

/* Voltage IDs */

#define MBOX_VOLT_CORE (0x1)
#define MBOX_VOLT_SDRAMC (0x2)
#define MBOX_VOLT_SDRAMP (0x3)
#define MBOX_VOLT_SDRAMI (0x4)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: bcm2711_mbox_getcputemp
 *
 * Description:
 *   Gets the temperature of the SoC.
 *
 * Input parameters:
 *   temp - A pointer to where to store the temperature
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getcputemp(uint32_t *temp);

/****************************************************************************
 * Name: bcm2711_mbox_getrev
 *
 * Description:
 *   Gets the board revision.
 *
 * Input parameters:
 *   rev - A pointer to where to store the revision number
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getrev(uint32_t *rev);

/****************************************************************************
 * Name: bcm2711_mbox_getmodel
 *
 * Description:
 *   Gets the board model number.
 *
 * Input parameters:
 *   model - A pointer to where to store the model number
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getmodel(uint32_t *model);

/****************************************************************************
 * Name: bcm2711_mbox_getmac
 *
 * Description:
 *   Gets the board MAC address in network byte order
 *
 * Input parameters:
 *   mac - A pointer to where to store the MAC address
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getmac(uint64_t *mac);

/****************************************************************************
 * Name: bcm2711_mbox_getpwr
 *
 * Description:
 *   Gets the power state of `id`.
 *
 * Input parameters:
 *   id - The device ID to know the power state of
 *   state - A pointer to where to store the state
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getpwr(uint8_t id, bool *state);

/****************************************************************************
 * Name: bcm2711_mbox_setpwr
 *
 * Description:
 *   Sets the power state of `id`.
 *
 * Input parameters:
 *   id - The device ID to know the power state of
 *   on - True for power on, false for power off
 *   wait - True to block until power stable, false otherwise
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_setpwr(uint8_t id, bool on, bool wait);

/****************************************************************************
 * Name: bcm2711_mbox_ledset
 *
 * Description:
 *   Sets the state of the LED specified by `pin`
 *
 * Input parameters:
 *   pin - The pin number of the LED to modify the state of
 *   on - True to turn on the LED, false to turn it off
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_ledset(uint8_t pin, bool on);

/****************************************************************************
 * Name: bcm2711_mbox_getclken
 *
 * Description:
 *   Get the state of the clock (enabled/disabled) corresponding to `id`
 *
 * Input parameters:
 *   id - The ID of the clock to check
 *   state - Where to store the state of the clock
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getclken(uint8_t id, bool *state);

/****************************************************************************
 * Name: bcm2711_mbox_setclken
 *
 * Description:
 *   Set the state of the clock (enabled/disabled) corresponding to `id`
 *
 * Input parameters:
 *   id - The ID of the clock
 *   en - True to enable the clock, false otherwise
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_setclken(uint8_t id, bool en);

/****************************************************************************
 * Name: bcm2711_mbox_getclkrate
 *
 * Description:
 *   Get the rate of the clock corresponding to `id` in Hz.
 *
 * Input parameters:
 *   id - The ID of the clock
 *   rate - Where to store the rate of the clock
 *   measured - True to return clock rate as a measured value (true rate),
 *              false for configured rate.
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getclkrate(uint8_t id, uint32_t *rate, bool measured);

/****************************************************************************
 * Name: bcm2711_mbox_setclkrate
 *
 * Description:
 *   Set the rate of the clock corresponding to `id` in Hz.
 *
 * Input parameters:
 *   id - The ID of the clock
 *   rate - The desired clock rate in Hz. The set rate will be returned in
 *          this variable, even if the clock is not enabled
 *   turbo - True to allow turbo settings (voltage, sdram and gpu), false
 *           otherwise
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_setclkrate(uint8_t id, uint32_t *rate, bool turbo);

/****************************************************************************
 * Name: bcm2711_mbox_getfb
 *
 * Description:
 *   Allocates a frame buffer and returns it.
 *
 * Input parameters:
 *   fb - A place to store the frame buffer address
 *   size - The frame buffer size in bytes
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getfb(void **fb, uint32_t *size);

/****************************************************************************
 * Name: bcm2711_mbox_releasefb
 *
 * Description:
 *   Releases the previously allocated frame buffer.
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_releasefb(void);

/****************************************************************************
 * Name: bcm2711_mbox_getdisp
 *
 * Description:
 *   Get physical display width and height.
 *
 * Input parameters:
 *   x - Width in pixels
 *   y - Height in pixels
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getdisp(uint32_t *x, uint32_t *y);

/****************************************************************************
 * Name: bcm2711_mbox_getdepth
 *
 * Description:
 *   Get the bits per pixel used for the display.
 *
 * Input parameters:
 *   bpp - Bits per pixel
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getdepth(uint32_t *bpp);

/****************************************************************************
 * Name: bcm2711_mbox_getalpha
 *
 * Description:
 *   Get the alpha mode.
 *
 * Input parameters:
 *   alpha - Returned alpha state: 0 enabled, 1 reversed, 2 ignored
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getalpha(uint32_t *alpha);

/****************************************************************************
 * Name: bcm2711_mbox_getpitch
 *
 * Description:
 *   Get the number of bytes per line.
 *
 * Input parameters:
 *   bpl - Bytes per line
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getpitch(uint32_t *bpl);

/****************************************************************************
 * Name: bcm2711_mbox_getvirtres
 *
 * Description:
 *   Get the virtual resolution of the display.
 *
 * Input parameters:
 *   x - Width in pixels
 *   y - Height in pixels
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getvirtres(uint32_t *x, uint32_t *y);

/****************************************************************************
 * Name: bcm2711_mbox_setvirtres
 *
 * Description:
 *   Set the virtual resolution of the display. The new virtual resolution is
 *   returned in the input parameters.
 *
 * Input parameters:
 *   x - Width in pixels
 *   y - Height in pixels
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_setvirtres(uint32_t *x, uint32_t *y);

/****************************************************************************
 * Name: bcm2711_mbox_getvirtoff
 *
 * Description:
 *   Get the virtual buffer offset in pixels
 *
 * Input parameters:
 *   x - X in pixels
 *   y - Y in pixels
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getvirtoff(uint32_t *x, uint32_t *y);

/****************************************************************************
 * Name: bcm2711_mbox_getpixord
 *
 * Description:
 *   Get the pixel order of the frame buffer.
 *
 * Input parameters:
 *   rgb - True if RGB order, false for BGR order
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getpixord(bool *rgb);

/****************************************************************************
 * Name: bcm2711_mbox_setpixord
 *
 * Description:
 *   Set the pixel order of the frame buffer.
 *
 * Input parameters:
 *   rgb - True if RGB order, false for BGR order
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_setpixord(bool rgb);

/****************************************************************************
 * Name: bcm2711_mbox_getoscan
 *
 * Description:
 *   Get the overscan values of the frame buffer in pixels.
 *
 * Input parameters:
 *   top - The amount of top overscan in pixels
 *   bot - The amount of bottom overscan in pixels
 *   left - The amount of left overscan in pixels
 *   right - The amount of right overscan in pixels
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getoscan(uint32_t *top, uint32_t *bot, uint32_t *left,
                          uint32_t *right);

/****************************************************************************
 * Name: bcm2711_mbox_fbinit
 *
 * Description:
 *   Initializes the frame buffer video interface with the desired settings.
 *
 * Input parameters:
 *   x - The resolution (width) in pixels (physical display width returned
 *   here)
 *   y - The resolution (height) in pixels (physical display height returned
 *   here)
 *   bpp - The bits per pixel (depth) (real value returned her)
 *   fb - The returned frame buffer address
 *   fblen - The length of the frame buffer in bytes
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_fbinit(uint32_t *x, uint32_t *y, uint32_t *bpp, void **fb,
                        uint32_t *fblen);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_BCM2711_BCM2711_MAILBOX_H */

/****************************************************************************
 * drivers/video/max7456.c
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

/****************************************************************************
 * Theory of Operation
 *
 * The MAX7456 is a single-channel, monochrome, on-screen-display generator
 * that accepts an NTSC or PAL video input signal, overlays user-defined
 * character data, and renders the combined stream to CVBS (analog) output.
 * The typical use case then forwards that CVBS output to a video
 * transmitter, analog display, recording device, and/or other external
 * components.
 *
 * The chip is fundamentally an SPI slave device with a register bank to
 * configure the chip's analog components, update values in the display frame
 * buffer, and modify the chip's onboard non-volatile character set.
 *
 * The MAX7456 must by necessity recover the video stream's hsync and vsync
 * signals, as part of its normal operations. These signals are also made
 * available at pins on the chip body, and may be used to synchronize updates
 * of frame buffer data with the vertical-blanking period.  Such
 * synchronization prevents "glitches" during OSD updates.
 *
 * Up to 480 user-definable characters can be displayed at one time. Each
 * 16-bit "character" is expressed an 8-bit index into the chip's onboard
 * character set, followed by an 8-bit character attribute that controls the
 * character's local background, blinking, and inversion.
 *
 * The overlaid characters may be distributed across 13 (NTSC) or 16 (PAL)
 * rows of the visible display area. The attributes of each of those lines
 * are also controllable on a line-by-line basis.
 *
 * OSD insertion is ultimately an analog process, and a few of the chip's
 * control registers are provided to adjust the OSD multiplexer's rise and
 * fall times. This is necessary to strike the user's preferred balance
 * between overlay sharpness and certain, undesirable display artifacts. The
 * defaults are probably good enough to start with, though.
 *
 * Note: Although we use the term "frame buffer", we cannot use the NuttX
 * standard /dev/fbN interface because our buffer memory is accessible only
 * across SPI. This is an inexpensive, slow, simple chip, and you wouldn't
 * use it for intensive work, but you WOULD use it on a memory-constrained
 * device. We keep our RAM footprint small by not keeping a local copy of the
 * framebuffer data.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <limits.h>
#include <nuttx/mutex.h>

#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/fs/fs.h>
#include <nuttx/video/max7456.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Enables debug-related interfaces. Leave undefined otherwise. */

#define DEBUG 1

/* Sets bit @n */

#define BIT(n) (1 << (n))

/* Creates a mask of @m bits, i.e. MASK(2) -> 00000011 */

#define MASK(m) (BIT((m) + 1) - 1)

/* Masks and shifts @v into bit field @m */

#define TO_BITFIELD(m,v) ((v) & MASK(m ##__WIDTH) << (m ##__SHIFT))

/* Un-masks and un-shifts bit field @m from @v */

#define FROM_BITFIELD(m,v) (((v) >> (m ##__SHIFT)) & MASK(m ##__WIDTH))

/* SPI read/write codes and speed */

#define SPI_REG_READ 0x80
#define SPI_REG_WRITE 0
#define SPI_FREQ 10000000UL
#define SPI_MODE SPIDEV_MODE0

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Register file description. */

enum mx7_regaddr_e
{
  VM0                 = 0,                  /* video mode (config) 0 */
  VM0__PAL            = BIT(6),
  VM0__SYNCSEL__SHIFT = 4,
  VM0__SYNCSEL__WIDTH = 2,
  VM0__ENABLE         = BIT(3),
  VM0__VSYNC_EN       = BIT(2),
  VM0__RESET          = BIT(1),
  VM0__VBUF_EN        = BIT(0),

  VM1                 = 1,                  /* video mode (config) 1 */
  VM1__GRAY           = BIT(7),
  VM1__OSD_PCT__SHIFT = 4,
  VM1__OSD_PCT__WIDTH = 3,
  VM1__BT__SHIFT      = 2,
  VM1__BT__WIDTH      = 2,
  VM1__BD__SHIFT      = 0,
  VM1__BD__WIDTH      = 2,

  HOS                 = 2,                  /* horizontal position */
  HOS__HPOS__SHIFT    = 0,
  HOS__HPOS__WIDTH    = 6,

  VOS                 = 3,                  /* vertical position */
  VOS__VPOS__SHIFT    = 0,
  VOS__VPOS__WIDTH    = 5,

  DMM                 = 4,                  /* display memory mode */
  DMM__8BIT           = BIT(6),
  DMM__LBC            = BIT(5),
  DMM__BLK            = BIT(4),
  DMM__INV            = BIT(3),
  DMM__CA__SHIFT      = 3,                  /* character attr */
  DMM__CA__WIDTH      = 3,
  DMM__CLEAR          = BIT(2),
  DMM__VCLEAR         = BIT(1),
  DMM__AUTOINC        = BIT(0),

  DMAH                  = 5,                /* display mem addr, high */
  DMAH__ATTR            = BIT(1),
  DMAH__ADDRBIT8__SHIFT = 0,
  DMAH__ADDRBIT8__WIDTH = 1,

  DMAL                  = 6,                /* display mem addr, low */
  DMAL__ADDR__SHIFT     = 0,
  DMAL__ADDR__WIDTH     = 8,

  DMDI                  = 7,                /* display memory data in */
  DMDI__SHIFT           = 0,
  DMDI__WIDTH           = 8,

  CMM                   = 8,                /* character memory mode */
  CMM__READ_NVM         = BIT(6) | BIT(4),
  CMM__WRITE_NVM        = BIT(7) | BIT(5),

  CMAH                  = 9,                /* char memory addr, high */
  CMAH__SHIFT           = 0,
  CMAH__WIDTH           = 6,

  CMAL                  = 0xa,              /* char memory addr, low */
  CMAL__ADDR__SHIFT     = 0,
  CMAL__ADDR__WIDTH     = 6,

  CMDI                  = 0xb,              /* character memory data in */

  OSDM                  = 0xc,              /* osd insertion mux */
  OSDM__RISET__SHIFT    = 3,                /* rise time */
  OSDM__RISET__WIDTH    = 3,
  OSDM__SWITCHT__SHIFT  = 0,                /* switching time */
  OSDM__SWITCHT__WIDTH  = 3,

  RB0                   = 0x10,             /* row N brightness */
  RB1                   = (RB0 + 1),
  RB2                   = (RB0 + 2),
  RB3                   = (RB0 + 4),
  RB4                   = (RB0 + 5),
  RB6                   = (RB0 + 6),
  RB7                   = (RB0 + 7),
  RB8                   = (RB0 + 8),
  RB9                   = (RB0 + 9),
  RB10                  = (RB0 + 10),
  RB11                  = (RB0 + 11),
  RB12                  = (RB0 + 12),
  RB13                  = (RB0 + 13),
  RB14                  = (RB0 + 14),
  RB15                  = (RB0 + 15),

  OSDBL                 = 0x6c,             /* osd black level */
  OSDBL__DISABLE        = BIT(4),
  OSDBL__PRESET__SHIFT  = 0,
  OSDBL__PRESET__WIDTH  = 4,

  STAT                  = 0xa0,             /* status (ro) */
  STAT__INRESET         = BIT(6),           /* 1 = in power-on reset */
  STAT__CHARUNAVAIL     = BIT(5),           /* 1 = unavailable for writes */
  STAT__NVSYNC          = BIT(4),           /* 1 = in vertical sync time */
  STAT__NHSYNC          = BIT(3),           /* 1 = in horizontal sync time */
  STAT__LOS             = BIT(2),           /* 1 = lost sync */
  STAT__NTSC            = BIT(1),           /* 1 = ntsc video detected */
  STAT__PAL             = BIT(0),           /* 1 = pal video detected */

  DMDO                  = 0xb0,             /* data memory data out (ro) */
  CMDO                  = 0xc0,             /* char memory data out (ro) */
};

struct path_name_map_s
{
  uint8_t addr;
  FAR const char *path;
};

#define PATH_MAP_ENTRY(node) { .addr = (node), .path = "" #node "" }

enum mx7_interface_e
{
  FB,                         /* 8-bit read/write interface */
  RAW,                        /* 16-bit interface in chip's native format */
  VSYNC,                      /* blocks until vertical blanking interval */
  CM                          /* Character Memory, i.e. the character map */
};

static struct path_name_map_s node_map[] =
{
  PATH_MAP_ENTRY(FB),
  PATH_MAP_ENTRY(RAW),
  PATH_MAP_ENTRY(VSYNC),
  PATH_MAP_ENTRY(CM),
};

#define NODE_MAP_LEN (sizeof(node_map) / sizeof(*node_map))

#if defined(DEBUG)

/* Maps between register names and addresses */

static struct path_name_map_s reg_name_map[] =
{
  PATH_MAP_ENTRY(VM0),
  PATH_MAP_ENTRY(VM1),
  PATH_MAP_ENTRY(HOS),
  PATH_MAP_ENTRY(VOS),
  PATH_MAP_ENTRY(DMM),
  PATH_MAP_ENTRY(DMAH),
  PATH_MAP_ENTRY(DMAL),
  PATH_MAP_ENTRY(DMDI),
  PATH_MAP_ENTRY(CMM),
  PATH_MAP_ENTRY(CMAH),
  PATH_MAP_ENTRY(CMAL),
  PATH_MAP_ENTRY(CMDI),
  PATH_MAP_ENTRY(OSDM),
  PATH_MAP_ENTRY(RB0),
  PATH_MAP_ENTRY(RB1),
  PATH_MAP_ENTRY(RB2),
  PATH_MAP_ENTRY(RB3),
  PATH_MAP_ENTRY(RB4),
  PATH_MAP_ENTRY(RB6),
  PATH_MAP_ENTRY(RB7),
  PATH_MAP_ENTRY(RB8),
  PATH_MAP_ENTRY(RB9),
  PATH_MAP_ENTRY(RB10),
  PATH_MAP_ENTRY(RB11),
  PATH_MAP_ENTRY(RB12),
  PATH_MAP_ENTRY(RB13),
  PATH_MAP_ENTRY(RB14),
  PATH_MAP_ENTRY(RB15),
  PATH_MAP_ENTRY(OSDBL),
  PATH_MAP_ENTRY(STAT),
  PATH_MAP_ENTRY(DMDO),
  PATH_MAP_ENTRY(CMDO)
};
#endif

#define REG_NAME_MAP_LEN (sizeof(reg_name_map) / sizeof(*reg_name_map))

/* Used to manage the device. No user-serviceable parts inside. */

struct mx7_dev_s
{
  mutex_t lock;               /* mutex for this structure */
  struct mx7_config_s config; /* board-specific information */

  uint8_t ca;                 /* character attribute (lbc, blink, etc.) */

#if defined(DEBUG)
  char debug[2];              /* stash for debugging-related output */
#endif
};

/****************************************************************************
 * Private Function Function Prototypes
 ****************************************************************************/

static int mx7_open(FAR struct file *filep);
static ssize_t mx7_read(FAR struct file *filep,
                        FAR char *buf, size_t len);
static ssize_t mx7_write(FAR struct file *filep,
                         FAR const char *buf, size_t len);

#if defined(DEBUG)
static ssize_t mx7_debug_read(FAR struct file *filep,
                              FAR char *buf, size_t len);
static ssize_t mx7_debug_write(FAR struct file *filep,
                               FAR const char *buf, size_t len);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* General user interface operations. */

static const struct file_operations g_mx7_fops =
{
  mx7_open,      /* open */
  NULL,          /* close */
  mx7_read,      /* read */
  mx7_write,     /* write */
  NULL,          /* seek */
  NULL,          /* ioctl */
  NULL           /* poll */
};

#if defined(DEBUG)

/* Debug-only interface, mostly for direct register access. */

static const struct file_operations g_mx7_debug_fops =
{
  NULL,                /* open */
  NULL,                /* close */
  mx7_debug_read,      /* read */
  mx7_debug_write,     /* write */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Translates an interface name name to its associated mx7_interface_e
 * enumerator.
 */

static int node_from_name(FAR const char *name)
{
  int n;

  for (n = 0; n < NODE_MAP_LEN; n++)
    {
      if (!strcmp(name, node_map[n].path))
        {
          return node_map[n].addr;
        }
    }

  return -1;
}

/* Translates a register name to its associated address. */

static int regaddr_from_name(FAR const char *name)
{
  int n;

  for (n = 0; n < REG_NAME_MAP_LEN; n++)
    {
      if (!strcmp(name, reg_name_map[n].path))
        {
          return reg_name_map[n].addr;
        }
    }

  return -1;
}

/* NOTE :
 *
 * In all of the following code, functions named with a double leading
 * underscore '__' must be invoked ONLY if the mx7_dev_s lock is
 * already held. Failure to do this might cause the transaction to get
 * interrupted, which will likely confuse the data you're trying to send.
 *
 * The mx7_dev_s lock is NOT the same thing as, i.e. the SPI master
 * interface lock: the latter protects the bus interface hardware
 * (which may have other SPI devices attached), the former protects
 * our chip and its associated data.
 */

/****************************************************************************
 * Name: __mx7_read_reg
 *
 * Description:
 *   Reads @len bytes into @buf from @dev, starting at register address
 *   @addr. This is a low-level function used for reading a sequence of one
 *   or more register values, and isn't usually called directly unless you
 *   REALLY know what you are doing. Consider one of the register-specific
 *   helper functions defined below whenever possible.
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Input parameters:
 *   dev       - the target device's handle
 *   addr      - starting register address
 *   buf       - where to store the register values
 *   len       - number of registers to read
 *
 * Returned value:
 *   Returns number of bytes read, or a negative errno.
 *
 ****************************************************************************/

static int __mx7_read_reg(FAR struct mx7_dev_s *dev,
                          enum mx7_regaddr_e addr,
                          FAR uint8_t * buf, uint8_t len)
{
  int ret;
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;

  /* We'll probably return the number of bytes asked for. */

  ret = len;

  /* Grab the SPI master controller, and set the mode. */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPI_MODE);
  SPI_SETFREQUENCY(spi, SPI_FREQ);

  /* Select the chip. */

  SPI_SELECT(spi, id, true);

  /* Send the read request. */

  SPI_SEND(spi, addr | SPI_REG_READ);

  /* Clock in the data. */

  while (0 != len--)
    {
      *buf++ = (uint8_t) (SPI_SEND(spi, 0xff));
    }

  /* Deselect the chip, release the SPI master. */

  SPI_SELECT(spi, id, false);
  SPI_LOCK(spi, false);

  return ret;
}

/****************************************************************************
 * Name:  __mx7_write_reg
 *
 * Description:
 *   Writes @len bytes from @buf to @dev, starting at @addr. This is a
 *   low-level function used for updating a sequence of one or more register
 *   values, and it DOES NOT check that the register being requested is
 *   write-capable. This function isn't called directly unless you REALLY
 *   know what you are doing.
 *
 *   Consider one of the register-specific helper functions defined below
 *   whenever possible. If a helper function for the register you desire to
 *   write is not defined, it's probably because that register is read-only.
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Input parameters:
 *   dev       - the target device's handle
 *   addr      - starting register address
 *   buf       - byte sequence to write
 *   len       - length of @buf, number of bytes to write
 *
 * Returned value:
 *   Returns number of bytes written, or a negative errno.
 *
 ****************************************************************************/

static int __mx7_write_reg(FAR struct mx7_dev_s *dev,
                           enum mx7_regaddr_e addr,
                           FAR const uint8_t * buf, uint8_t len)
{
  int ret = len;
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;

  /* Grab and configure the SPI master device. */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPI_MODE);
  SPI_SETFREQUENCY(spi, SPI_FREQ);

  /* Select the chip. */

  SPI_SELECT(spi, id, true);

  /* Send the write request. */

  SPI_SEND(spi, addr | SPI_REG_WRITE);

  /* Send the data. */

  while (0 != len--)
    {
      SPI_SEND(spi, *buf++);
    }

  /* Release the chip and SPI master. */

  SPI_SELECT(spi, id, false);
  SPI_LOCK(spi, false);

  return ret;
}

/****************************************************************************
 * Name: __mx7_read_reg__stat
 *
 * Description:
 *   Reads the contents of the STAT register.
 *
 * Returned value:
 *   Returns the value in STAT, or negative errno.
 *
 ****************************************************************************/

static inline int __mx7_read_reg__stat(FAR struct mx7_dev_s *dev)
{
  uint8_t val = 0xff;
  int ret;

  ret = __mx7_read_reg(dev, STAT, &val, sizeof(val));

  /* Return the error code, if an error occurred. */

  if (ret < 0)
    {
      return ret;
    }

  /* Return the register value. */

  return val;
}

/****************************************************************************
 * Name:  __mx7_read_reg__dmm
 *
 * Description:
 *   Reads the contents of the DMM register.
 *
 * Returned value:
 *   Returns the value held in in DMM, or negative errno.
 *
 ****************************************************************************/

static inline int __mx7_read_reg__dmm(FAR struct mx7_dev_s *dev)
{
  uint8_t val = 0xff;
  int ret;

  ret = __mx7_read_reg(dev, DMM, &val, sizeof(val));
  if (ret < 0)
    {
      return ret;
    }

  return val;
}

/****************************************************************************
 * Name: __mx7_write_reg__vm0
 *
 * Description:
 *   Writes @val to VM0. A simple helper around __mx7_write_reg().
 *
 * Returned value:
 *   Returns the number of bytes written (always 1), or a negative errno.
 *
 ****************************************************************************/

static inline int __mx7_write_reg__vm0(FAR struct mx7_dev_s *dev,
                                       uint8_t val)
{
  return __mx7_write_reg(dev, VM0, &val, sizeof(val));
}

/****************************************************************************
 * Name: __mx7_read_reg__vm0
 *
 * Description:
 *   Returns the contents of VM0.
 *
 * Returned value:
 *   Returns the register value, or a negative errno.
 *
 ****************************************************************************/

static inline int __mx7_read_reg__vm0(FAR struct mx7_dev_s *dev)
{
  uint8_t val = 0xff;
  int ret;

  ret = __mx7_read_reg(dev, VM0, &val, sizeof(val));

  if (ret < 0)
    {
      return ret;
    }

  return val;
}

/****************************************************************************
 * Name: __mx7_write_reg__cmah
 *
 * Description:
 *   Writes @val to CMAH.
 *
 * Returned value:
 *   Returns the number of bytes written (always 1), or a negative errno.
 *
 ****************************************************************************/

static inline int __mx7_write_reg__cmah(FAR struct mx7_dev_s *dev,
                                        uint8_t val)
{
  return __mx7_write_reg(dev, CMAH, &val, sizeof(val));
}

/****************************************************************************
 * Name: __mx7_write_reg__cmm
 *
 * Description:
 *   Writes @val to CMM.
 *
 * Returned value:
 *   Returns the number of bytes written (always 1), or a negative errno.
 *
 ****************************************************************************/

static inline int __mx7_write_reg__cmm(FAR struct mx7_dev_s *dev,
                                       uint8_t val)
{
  return __mx7_write_reg(dev, CMM, &val, sizeof(val));
}

/****************************************************************************
 * Name: __mx7_write_reg__cmal
 *
 * Description:
 *   Writes @val to CMAL.
 *
 * Returned value:
 *   Returns the number of bytes written (always 1), or a negative errno.
 *
 ****************************************************************************/

static inline int __mx7_write_reg__cmal(FAR struct mx7_dev_s *dev,
                                        uint8_t val)
{
  return __mx7_write_reg(dev, CMAL, &val, sizeof(val));
}

/****************************************************************************
 * Name: __mx7_write_reg__osdbl
 *
 * Description:
 *   Writes @val to OSDBL.
 *
 * Returned value:
 *   Returns the number of bytes written (always 1), or a negative errno.
 *
 ****************************************************************************/

static inline int __mx7_write_reg__osdbl(FAR struct mx7_dev_s *dev,
                                         uint8_t val)
{
  return __mx7_write_reg(dev, OSDBL, &val, sizeof(val));
}

/****************************************************************************
 * Name: __mx7_read_reg__osdbl
 *
 * Description:
 *   Returns the contents of OSDBL.
 *
 * Returned value:
 *   Returns the register value, or a negative errno.
 *
 ****************************************************************************/

static inline int __mx7_read_reg__osdbl(FAR struct mx7_dev_s *dev)
{
  uint8_t val = 0xff;
  int ret;

  ret = __mx7_read_reg(dev, OSDBL, &val, sizeof(val));

  if (ret < 0)
    {
      return ret;
    }

  return val;
}

/****************************************************************************
 * Name: __mx7_read_reg__cmdo
 *
 * Description:
 *   Returns the contents of CMDO.
 *
 * Returned value:
 *   Returns the register value, or a negative errno.
 *
 ****************************************************************************/

static inline int __mx7_read_reg__cmdo(FAR struct mx7_dev_s *dev)
{
  uint8_t val = 0xff;
  int ret;

  ret = __mx7_read_reg(dev, CMDO, &val, sizeof(val));

  if (ret < 0)
    {
      return ret;
    }

  return val;
}

/****************************************************************************
 * Name: __mx7_wait_reset
 *
 * Description:
 *   Waits until the chip finishes its reset activities.
 *
 ****************************************************************************/

static inline void __mx7_wait_reset(FAR struct mx7_dev_s *dev)
{
  int stat = 0;                 /* contents of STAT register */
  int dmm = 0;                  /* contents of DMM register */
  int vm0 = 0;                  /* contents of VM0 register */

  do
    {
      /* If we're here, a reset command has probably just been issued; wait
       * 100usec before checking, per the datasheet.
       */

      up_udelay(100);

      vm0 = __mx7_read_reg__vm0(dev);
      if (vm0 & VM0__RESET)
        {
          continue;
        }

      stat = __mx7_read_reg__stat(dev);
      dmm = __mx7_read_reg__dmm(dev);
    }
  while ((stat & STAT__INRESET) || (dmm & DMM__CLEAR));
}

/****************************************************************************
 * Name: __mx7_read_nvm
 *
 * Description:
 *   Commands the NVM to move the current CMAH:CMAL into shadow RAM.
 ****************************************************************************/

static inline void __mx7_read_nvm(FAR struct mx7_dev_s *dev)
{
  int stat = 0;

  /* Initiate the command. */

  __mx7_write_reg__cmm(dev, CMM__READ_NVM);

  do
    {
      /* Wait for it to finish. */

      up_udelay(10);
      stat = __mx7_read_reg__stat(dev);
    }
  while (stat & STAT__CHARUNAVAIL);
}

/****************************************************************************
 * Name: mx7_reset
 *
 * Description:
 *   Asserts a RESET command in the chip, and waits for it to finish. Except
 *   for NVM and the OSD brightness trim, this action restores all register
 *   values in the chip to their factory defaults.
 *
 ****************************************************************************/

static void mx7_reset(FAR struct mx7_dev_s *dev)
{
  nxmutex_lock(&dev->lock);

  /* Issue the reset command. */

  __mx7_write_reg__vm0(dev, VM0__RESET);

  /* Wait for all reset-related activities to finish. */

  __mx7_wait_reset(dev);

  /* All done. */

  nxmutex_unlock(&dev->lock);
}

/****************************************************************************
 * Name: __write_fb
 *
 * Description:
 *   Writes a stream of bytes to character address memory. We use the chip's
 *   "16-bit auto-increment mode", in order to make this operation as fast
 *   as possible. All of the bytes written are given the same attribute @ca.
 *
 *   This operation is best performed with the CS held, so we do all of the
 *   SPI heavy-lifting ourselves here. This function is comparable to a
 *   giant __write_reg_N().
 *
 * Input parameters:
 *   buf   -  character addresses (data) to write
 *   len   -  length of @buf
 *   ca    -  character attribute, see the DMM register for details
 *   pos   -  starting address, 0 = upper-left corner of the display
 *
 * Returned value:
 *   Returns the number of bytes written, or a negative errno.
 *
 ****************************************************************************/

static ssize_t __write_fb(FAR struct mx7_dev_s *dev,
                          FAR const uint8_t * buf, size_t len,
                          uint8_t ca, size_t pos)
{
  ssize_t ret = len;
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;

  /* Configure the bus and grab the chip as usual. */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPI_MODE);
  SPI_SETFREQUENCY(spi, SPI_FREQ);
  SPI_SELECT(spi, id, true);

  while (len != 0)
    {
      /* Thus sayeth the datasheet (pp. 39-40):
       *
       * "When in 16-Bit [Auto-Increment] Operating Mode:
       *
       * 1) Write DMAH[0] = X to select the MSB and DMAL[7:0] = XX to select
       *    the lower order address bits of the starting address for
       *    auto-increment operation.  This address determines the location
       *    of the first character on the display (see Figures 10 and 21)."
       */

      SPI_SEND(spi, DMAH | SPI_REG_WRITE);
      SPI_SEND(spi, TO_BITFIELD(DMAH__ADDRBIT8, (pos >> 8)));
      SPI_SEND(spi, DMAL | SPI_REG_WRITE);
      SPI_SEND(spi, TO_BITFIELD(DMAL__ADDR, pos));

      /* "2) Write DMM[0] = 1 to set the auto-increment mode.
       *
       *  3) Write DMM[6] = 0 to set the 16-bit operating mode.
       *
       *  4) Write DMM[5:3] = XXX to set the Local Background Control
       *     (LBC), Blink (BLK) and Invert (INV) attribute bits that
       *     will be applied to all characters."
       */

      SPI_SEND(spi, DMM | SPI_REG_WRITE);
      SPI_SEND(spi, DMM__AUTOINC | TO_BITFIELD(DMM__CA, ca));

      /* "5) Write CA [Character Address: the index into the chip's onboard
       *     NVM character map] data in the intended character order to
       *     display text on the screen. It will be stored along with a
       *     Character Attribute byte derived from DMM[5:3].  See Figure
       *     19. This is the single byte operation. The DMDI[7:0] address is
       *     automatically set by autoincrement mode. The display memory
       *     address is automatically incremented following the write
       *     operation until the final display memory address is reached."
       */

      while (len != 0)
        {
          /* Send the byte to the DMDI register. The "auto-increment" mode
           * will update DMAH and DMAL for us.
           */

          SPI_SEND(spi, DMDI | SPI_REG_WRITE);
          SPI_SEND(spi, *buf);

          /* Check if we just exited auto-increment mode. */

          if (*buf == 0xff)
            {
              /* An embedded 0xff terminates auto-increment mode, and since
               * we've already sent it, pause here to deal with
               * it. Betaflight, et. al just skip the byte and continue, and
               * then retrace their steps later. I think it's a better
               * workflow to just deal with it now. Plus, there's only a
               * 1/256 chance of there being such a byte anyway, and if
               * performance ends up being a problem then the user can move
               * the CA to a different index in their NVM map.
               */

              break;
            }
          else
            {
              /* It was an ordinary byte, so we're still in auto-increment
               * mode; count it and keep going.
               */

              buf++;
              pos++;
              len--;
            }
        }

      /* (Use of while() here instead of if() catches repeated 0xff's while
       * we're already out of auto-increment mode. Since you mustached, this
       * shaves a transaction or two when they occur.)
       */

      while (len != 0 && *buf == 0xff)
        {
          /* We're out of the auto-increment loop but still have data
           * remaining, which means there's an 0xff in the data stream. We
           * must send it the hard way, but we can still use the attribute
           * byte already stored in DMM.
           */

          SPI_SEND(spi, DMAH | SPI_REG_WRITE);
          SPI_SEND(spi, TO_BITFIELD(DMAH__ADDRBIT8, (pos >> 8)));
          SPI_SEND(spi, DMAL | SPI_REG_WRITE);
          SPI_SEND(spi, TO_BITFIELD(DMAL__ADDR, pos));
          SPI_SEND(spi, DMDI | SPI_REG_WRITE);
          SPI_SEND(spi, *buf);

          /* Now we can retire the byte. */

          buf++;
          pos++;
          len--;
        }
    }

  /* "6) Write CA = FFh to terminate the auto-increment mode." */

  SPI_SEND(spi, DMDI | SPI_REG_WRITE);
  SPI_SEND(spi, 0xff);

  /* The datasheet suggests that the chip will drop DMM[1] when it leaves
   * auto-increment mode, but I don't see that happening. Let's force it to
   * drop here just in case, so as to not not confuse future DMDI writes.
   */

  SPI_SEND(spi, DMM | SPI_REG_WRITE);
  SPI_SEND(spi, TO_BITFIELD(DMM__CA, ca));

  /* And, finally, we're all done. */

  SPI_SELECT(spi, id, false);
  SPI_LOCK(spi, false);

  return ret;
}

/****************************************************************************
 * Name: __read_cm
 *
 * Description:
 *    Reads the chip's Character Memory area.
 *
 *    Each entry in the Character Memory area is 3x18=54 bytes, so one would
 *    expect that the @len parameter would always be an integer multiple of
 *    that quantity. But we don't require that here, because the chip doesn't
 *    either.
 *
 *    Each row in the CA EEPROM is 64 bytes wide, but only the first 54 bytes
 *    are used. The rest are marked as "unused memory" in the datasheet. All
 *    64 bytes of each row are included in the data we return, if the user's
 *    request spans that area. We assume that the user understands the
 *    format.
 *
 *    In total, the chip has 64 bytes per row x 256 rows of EEPROM.
 *
 *    Finally, each pixel of a character requires two bits to define. Thus,
 *    there are four pixels per byte.
 *
 * Input parameters:
 *   dev  - device handle
 *   pos - starting address to read from, i.e. offset in bytes from the start
 *          of character memory
 *   buf  - buffer to return the character map data
 *   len  - number of bytes to return
 *
 * Returned value:
 *  Returns the number of bytes read on success, or negative errno.
 *
 ****************************************************************************/

static ssize_t __read_cm(FAR struct mx7_dev_s *dev,
                         size_t pos, FAR uint8_t * buf, size_t len)
{
  const size_t eeprom_rows = 256;
  const size_t eeprom_cols = 64;
  const size_t eeprom_bytes = eeprom_rows * eeprom_cols;
  ssize_t ret = len;
  int vm0 = 0;
  int cmah = 0;
  int cmal = 0;

  /* Does the request stay in-bounds? */

  if (pos + len >= eeprom_bytes)
    {
      if (pos >= eeprom_bytes)
        {
          /* They want to start out-of-bounds. No. */

          len = 0;
        }

      /* The starting position is in-bounds, but somewhere after that they
       * run out of bounds. Truncate the length of their request to what
       * will fit, per the usual read() semantics. Next time, they'll
       * probably call us with a position that's out of bounds. We'll catch
       * them above, and return 0.
       */

      len = eeprom_bytes - pos;
    }

  /* If we have nothing to do, do nothing. */

  if (len == 0)
    {
      return 0;
    }

  /* Thus sayeth the datasheet (p. 38):
   *
   * "Steps for Reading Character Bytes from Character Memory:
   *
   * 1) Write VM0[3] = 0 to disable the OSD image."
   */

  vm0 = __mx7_read_reg__vm0(dev);
  __mx7_write_reg__vm0(dev, vm0 & ~VM0__ENABLE);

  while (len != 0)
    {
      /* "2) Write CMAH[7:0] = xxH to select the character (0–255) to be
       *     read (Figures 10 and 13)."
       *
       * Put another way: CMAH is the row number in the EEPROM.
       */

      cmah = pos / eeprom_cols;
      __mx7_write_reg__cmah(dev, cmah);

      /* "3) Write CMM[7:0] = 0101xxxx to read the character data from the
       *     NVM to the shadow RAM (Figure 13)."
       *
       * They forgot to mention STAT[5], but we remembered it.
       */

      __mx7_read_nvm(dev);

      /* "4) Write CMAL[7:0] = xxH to select the 4-pixel byte (0–63) in
       *     the character to be read (Figures 10 and 13)."
       *
       * That means CMAL is the column number.
       */

      cmal = pos % eeprom_cols;

      /* The shadow RAM is large enough to hold an entire row, so we don't
       * need to go back for another until we've read all of this one.
       */

      do
        {
          __mx7_write_reg__cmal(dev, cmal);

          /* "5) Read CMDO[7:0] = xxH to read the selected 4-pixel byte of
           * data (Figures 11 and 13)."
           */

          *buf = __mx7_read_reg__cmdo(dev);

          /* "6) Repeat steps 4 and 5 to read other bytes of 4-pixel data." */

          buf++;
          pos++;
          len--;
          cmal++;
        }
      while (cmal < eeprom_cols);
    }

  /* "7) Write VM0[3] = 1 to enable the OSD image display." */

  __mx7_write_reg__vm0(dev, vm0);

  return ret;
}

/****************************************************************************
 * Name: mx7_open
 *
 * Description:
 *    The usual open() interface for user accesses.
 *
 * Note: we don't deal with multiple users trying to access this interface at
 * the same time. Until further notice, you probably should just not do that.
 *
 * It's not as simple as just prohibiting concurrent opens or reads with a
 * mutex: there are legit reasons for concurrent access, but they must be
 * treated carefully in this interface lest a partial reader end up with a
 * mixture of old and new side-effects. This will make some users unhappy.
 *
 ****************************************************************************/

static int mx7_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mx7_dev_s *dev = inode->i_private;

  /* Reset any leftover CA from a previous operation. */

  dev->ca = 0;

  return 0;
}

/****************************************************************************
 * Name: mx7_read_cm
 *
 * Description:
 *   Reads from Character Memory, the chip's NVM character map.
 *
 ****************************************************************************/

static ssize_t mx7_read_cm(FAR struct file *filep, FAR char *buf, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mx7_dev_s *dev = inode->i_private;
  ssize_t ret;

  nxmutex_lock(&dev->lock);
  ret = __read_cm(dev, filep->f_pos, (FAR uint8_t *)buf, len);
  nxmutex_unlock(&dev->lock);

  return ret;
}

/****************************************************************************
 * Name: mx7_read
 *
 * Description:
 *   The usual file-operations read() method. I don't know what such an
 *   operation would mean in general, so we do nothing here.
 *
 *   TODO: One idea is to have interfaces allowing the user to discover
 *   details of our capabilities: display size, PAL vs. NTSC, etc., but I
 *   would want to have more experience with other chips before deciding how
 *   to best generalize those things.
 *
 ****************************************************************************/

static ssize_t mx7_read(FAR struct file *filep, FAR char *buf, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  ssize_t ret = 0;

  /* Which interface are they using? */

  switch (node_from_name(inode->i_name))
    {
    case CM:

      /* Reading from Character Memory (character map). */

      ret = mx7_read_cm(filep, buf, len);
      break;

    default:

      /* Someday we'll have others, I'm sure... */

      break;
    }

  if (ret > 0)
    {
      /* Successful read, so update the file position. */

      filep->f_pos += ret;
    }

  return ret;
}

/****************************************************************************
 * Name: mx7_write_fb
 *
 * Description:
 *   The usual file-operations write() method for the ".../fb" interface.
 *   The user is redirected here by the frontend write() helper defined
 *   below.
 *
 *   We send @len bytes from @buf to the chip's character address array,
 *   starting at the current file position as stored in @filep->f_pos. Users
 *   may adjust this value beforehand by calling seek() in the usual
 *   way. (Position 0 is the upper-left corner of the display window.)
 *
 *   Note: The contents of @buf aren't ASCII data, they're indices into the
 *   chip's onboard NVM character data. (It is possible to make those look
 *   like ASCII data, but that's not generally how the chip is used because
 *   it's a big waste of NVM.)
 *
 * Returned Value:
 *   Returns the number of bytes written, or negative errno.
 *
 ****************************************************************************/

static ssize_t mx7_write_fb(FAR struct file *filep, FAR const char *buf,
                            size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mx7_dev_s *dev = inode->i_private;
  ssize_t ret;

  nxmutex_lock(&dev->lock);
  ret = __write_fb(dev, (FAR uint8_t *)buf, len, dev->ca, filep->f_pos);
  nxmutex_unlock(&dev->lock);

  return ret;
}

/****************************************************************************
 * Name: mx7_write
 *
 * Description:
 *   A "frontend write() helper" that redirects the user's write() request
 *   to the correct handler. We are otherwise an ordinary file-operations
 *   write() function.
 *
 *   We use the approach you see here so that we don't have to have one
 *   distinct function (and a separate file_operations structure) for each of
 *   the many interfaces we're likely to create for interacting with this
 *   chip in its various useful ways. This schema also lets us re-use the
 *   interface code internally (see the test-pattern generator at startup.)
 *
 *   In general, any function we call from here uses the combination of
 *   seek() and write() to implement a zero-copy frame buffer. The seek()
 *   parameter sets the current cursor position, and successive write()s
 *   provide the character data starting at that position.
 *
 *   TODO: At the moment, we have no mechanism for setting the character
 *   attribute (the LBC, BLK, and INV fields in DMM) for the data arriving
 *   here. Fortunately, the default value of '0', asserted in open(), works
 *   for the basic stuff.
 *
 *   The above isn't a hard problem to solve, I just don't need to solve it
 *   right now. And, I don't know what the most convenient solution would
 *   look like: the obvious choice is ioctl(), but I don't like ioctl()
 *   because I can't test it from the command line.
 *
 *   One idea is to have "fb", "blink", "inv", and other entry points for
 *   writing data with specific attributes. That has a nice feel to it,
 *   actually...
 *
 ****************************************************************************/

static ssize_t mx7_write(FAR struct file *filep,
                         FAR const char *buf, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  ssize_t ret = -EINVAL;

  /* Which interface are they using? */

  switch (node_from_name(inode->i_name))
    {
    case FB:

      /* The "here is some stuff to display" interface */

      ret = mx7_write_fb(filep, buf, len);
      break;

    default:

      /* Someday we'll have others, I'm sure... */

      break;
    }

  if (ret > 0)
    {
      /* Successful read, so update the file position. */

      filep->f_pos += ret;
    }

  return ret;
}

#if defined(DEBUG)

/****************************************************************************
 * Name: uint8_to_hex
 *
 * Description:
 *   Converts an 8-bit integer value to its ascii-hex representation. Values
 *   less than 16 are right-justified and padded with zero.
 *
 * Input parameters:
 *   @n    -  integer value to convert
 *   @buf  -  two-byte buffer to store the converted representation
 *
 * Returned value:
 *   Always returns 2.
 *
 ****************************************************************************/

static int uint8_to_hex(uint8_t n, FAR char *buf)
{
  static FAR const char *hexchar = "0123456789abcdef";

  buf[0] = hexchar[(n >> 4) & 0xf];
  buf[1] = hexchar[n & 0xf];
  return 2;
}

/****************************************************************************
 * Name: hex_to_uint8
 *
 * Description:
 *   Converts a two-byte, ascii-hex string to its integer value.
 *
 * Input parameters:
 *   @buf   -  nul-terminated sequence of ascii-hex string characters
 *
 * Returned value:
 *   Returns the converted value.
 *
 ****************************************************************************/

static int hex_to_uint8(FAR const char *buf)
{
  /* Interpret as hex even without the leading "0x". */

  return strtol(buf, NULL, 16);
}

/****************************************************************************
 * Name: mx7_debug_read
 *
 * Description:
 *   Semi-ordinary file-operations read() method. Returns the value in the
 *   eponymous register, formatted as ascii hex. This allows users to observe
 *   raw hardware register values, like this:
 *
 *      $ cat /dev/osd0/DMM
 *      e5
 *
 *   This same function is used for all registers, which are distinguished
 *   by @filep->f_inode->i_name, i.e. there is a "/dev/osd0/DMM",
 *   "/dev/osd0/VM0", etc., and reads from all of those interfaces arrive
 *   here.
 *
 *   Utilities like cat(1) will exit automatically at EOF, which can be
 *   tricky to deliver at the right time. We achieve this by reading the
 *   associated register value only once, when filep->f_pos is at the
 *   beginning of the "file" we're emulating. The value obtained is stored
 *   in dev->debug[], and we work our way through that and increment the
 *   "file position" accordingly to keep track (because the user may ask for
 *   only one byte at a time, and our register values require two bytes to
 *   express as ascii-hex text).
 *
 *   When we reach the end of dev->debug[], we return EOF. If the user wants
 *   a fresh copy, they can either close and reopen the interface, or move
 *   the file pointer back to 0 via a seek operation.
 *
 ****************************************************************************/

static ssize_t mx7_debug_read(FAR struct file *filep,
                              FAR char *buf, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mx7_dev_s *dev = inode->i_private;
  FAR const char *name = inode->i_name;
  FAR const char *orig_buf = buf;
  int ret = 0;
  int addr = 0;
  uint8_t val = 0;

  /* If we've already sent them a copy of the register value, don't re-send
   * it until they ask for a fresh one by either reopening the interface, or
   * doing a seek() to reset the cursor. This causes cat(1), etc. to exit
   * nicely.
   */

  if (filep->f_pos >= sizeof(dev->debug))
    {
      return 0;                 /* 0 == "eof" */
    }

  /* Populate the register value "cache" if needed. */

  if (filep->f_pos == 0)
    {
      /* Map the interface name to its associated register address. */

      addr = regaddr_from_name(name);

      /* Read the register. */

      nxmutex_lock(&dev->lock);
      ret = __mx7_read_reg(dev, addr, &val, 1);
      nxmutex_unlock(&dev->lock);

      if (ret != 1)
        {
          return ret;
        }

      /* Save the value to our local cache. */

      uint8_to_hex(val, dev->debug);
    }

  /* Return as many bytes as we have that will fit. */

  while (len-- != 0 && filep->f_pos < sizeof(dev->debug))
    {
      *buf++ = dev->debug[filep->f_pos++];
    }

  return buf - orig_buf;
}

/****************************************************************************
 * Name: mx7_debug_write
 *
 * Description:
 *   Semi-ordinary file-operations write() method, for all debugging
 *   interfaces.
 *
 *   Specifically, we allow users to assert new register values, by sending
 *   us ascii-hex strings:
 *
 *       $  echo 3e > /dev/osd0/VM0
 *
 ****************************************************************************/

static ssize_t mx7_debug_write(FAR struct file *filep, FAR const char *buf,
                               size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mx7_dev_s *dev = inode->i_private;
  FAR const char *name = inode->i_name;

  /* Map the incoming interface name to the associated register address. */

  int addr = regaddr_from_name(name);

  /* Convert from ascii-hex to binary. */

  uint8_t val = hex_to_uint8(buf);

  /* Write the register value. */

  nxmutex_lock(&dev->lock);
  __mx7_write_reg(dev, addr, &val, 1);
  nxmutex_unlock(&dev->lock);

  return len;
}
#endif

/****************************************************************************
 * Name: add_interface
 *
 * Description:
 *   Creates an interface named "@path/@name", and registers it. If @name is
 *   NULL, the interface is named just "@path" instead.
 *
 * Input parameters:
 *   path    - The full path to the interface to register. E.g., "/dev/osd0"
 *   name    - Entry underneath @path (making the latter a directory)
 *   fops    - File operations for the interface
 *   mode    - Access permisisons
 *   private - Opaque pointer to forward to the file operation handlers
 *
 * Returned value:
 *   Zero on success, negative errno otherwise.
 *
 ****************************************************************************/

static int add_interface(FAR const char *path,
                         FAR const char *name,
                         FAR const struct file_operations *fops,
                         mode_t mode, FAR void *private)
{
  char buf[128];

  /* Start with calling @path the interface name. */

  strlcpy(buf, path, sizeof(buf));

  /* Is the interface actually in a directory named @path? */

  if (name != NULL)
    {
      /* Convert @path to a directory name. */

      strcat(buf, "/");

      /* Append the real interface name. */

      strcat(buf, name);
    }

  /* Register the interface in the usual way. NuttX will build the
   * (pseudo-)directory for us.
   */

  return register_driver(buf, fops, mode, private);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max7456_register
 *
 * Description:
 *   Creates awareness of a max7456 chip, and builds a user interface to it.
 *
 * Input parameters:
 *   path    - The full path to the interface to register. E.g., "/dev/osd0"
 *   config  - Configuration information
 *
 * Returned value:
 *   Zero on success, negative errno otherwise.
 *
 ****************************************************************************/

int max7456_register(FAR const char *path, FAR struct mx7_config_s *config)
{
  FAR struct mx7_dev_s *dev = NULL;
  int ret = 0;
  int osdbl = 0;
  int n;

  /* Without config info, we can't do anything. */

  if (config == NULL)
    {
      return -EINVAL;
    }

  /* Initialize the device structure. */

  dev = (FAR struct mx7_dev_s *)kmm_malloc(sizeof(struct mx7_dev_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  memset(dev, 0, sizeof(*dev));
  nxmutex_init(&dev->lock);

  /* Keep a copy of the config structure, in case the caller discards
   * theirs.
   */

  dev->config = *config;

  /* Reset the display, to give it a clean initial state. */

  mx7_reset(dev);

  /* Turn the display on. */

  /* Note: we don't _really_ need to lock this, because nobody can see our
   * device yet. But since we're using the lock-requiring functions below,
   * I'm doing it anyway for consistency.
   */

  nxmutex_lock(&dev->lock);

  /* Thus sayeth the datasheet (pp. 38):
   *
   * "The following two steps enable viewing of the OSD image. These steps
   *  are not required to read from or write to the display memory:
   *
   * 1) Write VM0[3] = 1 to enable the display of the OSD image."
   */

  __mx7_write_reg__vm0(dev, VM0__ENABLE);

  /* "2) Write OSDBL[4] = 0 to enable automatic OSD black level control
   *     [Note: there is no "manual" control]. This ensures the correct
   *     OSD image brightness. This register contains 4 factory-preset
   *     bits [3:0] that must not be changed. Therefore, when changing
   *     bit 4, first read OSDBL[7:0], modify bit 4, and then write back
   *     the updated byte."
   */

  osdbl = __mx7_read_reg__osdbl(dev);
  osdbl &= ~OSDBL__DISABLE;
  __mx7_write_reg__osdbl(dev, osdbl);

  /* Create device nodes for the ordinary user interfaces:
   *    /dev/osd0/fb
   *    /dev/osd0/raw
   *    /dev/osd0/vsync
   *    /dev/osd0/cm
   */

  for (n = 0; ret >= 0 && n < NODE_MAP_LEN; n++)
    {
      ret = add_interface(path, node_map[n].path, &g_mx7_fops, 0666, dev);
    }

#if defined(DEBUG)
  /* Add the register-debugging entries. These are device nodes with names
   * that match the associated register, which developers can read or write
   * through to see what the hardware is doing. Not useful in everyday
   * activities.
   */

  for (n = 0; ret >= 0 && n < REG_NAME_MAP_LEN; n++)
    {
      ret = add_interface(path, reg_name_map[n].path, &g_mx7_debug_fops,
                          0666, dev);
    }
#endif

  if (ret < 0)
    {
      snerr("ERROR: Failed to register max7456 interface: %d\n", ret);
      nxmutex_destroy(&dev->lock);
      kmm_free(dev);
      return ret;
    }

#if defined(DEBUG)
  /* For testing, display a test pattern of sorts. When this sequence is
   * longer than 254 bytes, we get a 0xff in the stream; this confirms that
   * __write_fb() can handle that situation properly.
   */

  uint8_t buf[300];
  for (n = 0; n < sizeof(buf); n++)
    {
      buf[n] = n;
    }

  __write_fb(dev, buf, sizeof(buf), 0, 0);
#endif

  /* Release the device to the world. */

  nxmutex_unlock(&dev->lock);

  return 0;
}

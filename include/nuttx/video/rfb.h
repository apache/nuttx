/****************************************************************************
 * include/nuttx/video/rfb.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference:
 *   The RFB Protocol, Tristan Richardson RealVNC Ltd (formerly of Olivetti
 *   Research Ltd / AT&T Labs Cambridge), Version 3.8,  Last updated
 *   26 November 2010
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

#ifndef __INCLUDE_NUTTX_VIDEO_RFB_H
#define __INCLUDE_NUTTX_VIDEO_RFB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * RFB Protocol
 ****************************************************************************/

/* 6.1 Hand shaking Messages ************************************************/

/* 6.1.1 ProtocolVersion
 *
 * "Handshaking begins by the server sending the client a ProtocolVersion
 *  message. This lets the client know which is the highest RFB protocol
 *  version number supported by the server. The client then replies with a
 *  similar message giving the version number of the protocol which should
 *  actually be used (which may be different to that quoted by the server).
 *  A client should never request a protocol version higher than that
 *  offered by the server. It is intended that both clients and servers may
 *  provide some level of backwards compatibility by this mechanism.
 *
 * "The only published protocol versions at this time are 3.3, 3.7, 3.8
 *  (version 3.5 was wrongly reported by some clients, but this should be
 *  interpreted by all servers as 3.3). Addition of a new encoding or
 *  pseudo-encoding type does not require a change in protocol version,
 *  since a server can simply ignore encodings it does not understand.
 *
 * "The ProtocolVersion message consists of 12 bytes interpreted as a string
 *  of ASCII characters in the format "RFB xxx.yyy\n" where xxx and yyy are
 *  the major and minor version numbers, padded with zeros."
 */

#define RFB_PROTOCOL_VERSION_FMT "RFB %03d.%03d\n"
#define RFB_PROTOCOL_VERSION_3p3 "RFB 003.003\n"
#define RFB_PROTOCOL_VERSION_3p7 "RFB 003.007\n"
#define RFB_PROTOCOL_VERSION_3p8 "RFB 003.008\n"

struct rfb_protocolversion_s
{
  uint8_t version[12];           /* U8  Version string */
};

/* 6.1.2 Security
 *
 * "Once the protocol version has been decided, the server and client must
 *  agree on the type of security to be used on the connection.
 */

#define RFB_SECTYPE_INVALID    0 /* Invalid */
#define RFB_SECTYPE_NONE       1 /* None */
#define RFB_SECTYPE_VNC        2 /* VNC Authentication */
#define RFB_SECTYPE_RA2        5 /* RA2 */
#define RFB_SECTYPE_RA2NE      6 /* RA2ne */
#define RFB_SECTYPE_TIGHT     16 /* Tight */
#define RFB_SECTYPE_ULTRA     17 /* Ultra */
#define RFB_SECTYPE_TLS       18 /* TLS */
#define RFB_SECTYPE_VENCRYPT  19 /* VeNCrypt */
#define RFB_SECTYPE_GTK       20 /* GTK-VNC SASL */
#define RFB_SECTYPE_MD5       21 /* MD5 hash authentication */
#define RFB_SECTYPE_COLINDEAN 22 /* Colin Dean xvp */

/* "Version 3.7 onwards The server lists the security types which it
 *  supports:"
 */

struct rfb_supported_sectypes_s
{
  uint8_t ntypes;                /* U8  Number of security types supported */
  uint8_t type[1];               /* U8  Security types supported.  Actual
                                  *     size if ntypes. */
};

#define SIZEOF_RFB_SUPPORTED_SECTYPES_S(n) \
  (sizeof(struct rfb_supported_sectypes_s) + (n) - 1)

/* "If the server listed at least one valid security type supported by the
 *  client, the client sends back a single byte indicating which security
 *  type is to be used on the connection:"
 */

struct rfb_selected_sectype_s
{
  uint8_t type;                  /* U8  The selected security type */
};

/* "If number-of-security-types is zero, then for some reason the connection
 *  failed (e.g. the server cannot support the desired protocol version).  This
 *  is followed by a string describing the reason (where a string is specified
 *  as a length followed by that many ASCII characters):
 */

struct rfb_sectype_connfail_s
{
  uint8_t len[4];                /* U32 Reason length */
  uint8_t str[1];                /* U8  Reason string.  Actual length is len. */
};

#define SIZEOF_RFB_SECTYPE_CONNFAIL_S(n) \
  (sizeof(struct rfb_sectype_connfail_s) + (n) - 1)

/* "Version 3.3 The server decides the security type and sends a single
 *  word:"
 */

struct rfb_sectype_s
{
  uint8_t type[4];               /* U32 Security type */
};

/* "Once the security-type has been decided, data specific to that security-
 *  type follows (see section 6.2 for details). At the end of the security
 *  handshaking phase, the protocol normally continues with the SecurityResult
 *  message.  Note that after the security handshaking phase, it is possible
 *  that further protocol data is over an encrypted or otherwise altered
 *  channel."
 */

/* 6.1.3 SecurityResult
 *
 * The server sends a word to inform the client whether the security
 * handshaking was successful.
 */

#define RFB_SECTYPE_SUCCESS 0
#define RFB_SECTYPE_FAIL    1

struct rfb_sectype_result_s
{
  uint8_t result[4];             /* U32 Security type result */
};

/* "If successful, the protocol passes to the initialization phase (section
 *  6.3)."
 *
 * "Version 3.8 onwards If unsuccessful, the server sends a string
 *  describing the reason for the failure, and then closes the connection:
 */

struct rfb_sectype_fail_s
{
  uint8_t len[4];                /* U32 Reason length */
  uint8_t str[1];                /* U8  Reason string.  Actual length is len. */
};

#define SIZEOF_RFB_SECTYPE_FAIL_S(n) \
  (sizeof(struct rfb_sectype_fail_s) + (n) - 1)

/* "Version 3.3 and 3.7 If unsuccessful, the server closes the connection." */

/* 6.2 Security Types *******************************************************/

/* 6.2.1 None
 *
 * "No authentication is needed and protocol data is to be sent unencrypted.
 *
 * "Version 3.8 onwards The protocol continues with the SecurityResult
 *  message."
 *
 * "Version 3.3 and 3.7 The protocol passes to the initialization phase
 *  (section 6.3)."
 *
 * 6.2.2 VNC Authentication
 *
 * "VNC authentication is to be used and protocol data is to be sent
 *  unencrypted.  The server sends a random 16-byte challenge:
 */

struct rfb_challenge_s
{
  uint8_t challenge[16];         /* U8  Random challenge */
};

/* "The client encrypts the challenge with DES, using a password supplied by
 *  the user as the key, and sends the resulting 16-byte response:
 */

struct rfb_response_s
{
  uint8_t response[16];          /* U8  Encrypted response */
};

/* The protocol continues with the SecurityResult message. */

/* 6.3 Initialisation Messages **********************************************/

/* 6.3.1 ClientInit
 *
 * "Once the client and server are sure that they’re happy to talk to one
 *  another using the agreed security type, the protocol passes to the
 *  initialization phase. The client sends a ClientInit message followed by
 *  the server sending a ServerInit message.
 *
 * "Shared-flag is non-zero (true) if the server should try to share the
 *  desktop by leaving other clients connected, zero (false) if it should
 *  give exclusive access to this client by disconnecting all other clients."
 */

#define RFB_FLAG_SHARED    0
#define RFB_FLAG_EXCLUSIVE 1

struct rfb_clientinit_s
{
  uint8_t shared;                /* U8  Shared flag */
};

/* 6.3.2 ServerInit
 *
 * "After receiving the ClientInit message, the server sends a ServerInit
 *  message. This tells the client the width and height of the server’s
 *  framebuffer, its pixel format and the name associated with the desktop:
 */

struct rfb_pixelfmt_s
{
  uint8_t bpp;                   /* U8  bits-per-pixel */
  uint8_t depth;                 /* U8  depth */
  uint8_t bigendian;             /* U8  big-endian-flag */
  uint8_t truecolor;             /* U8  true-colour-flag */
  uint8_t rmax[2];               /* U16 red-max */
  uint8_t gmax[2];               /* U16 green-max */
  uint8_t bmax[2];               /* U16 blue-max */
  uint8_t rshift;                /* U8  red-shift */
  uint8_t gshift;                /* U8  green-shift */
  uint8_t bshift;                /* U8  blue-shift */
  uint8_t padding[3];
};

struct rfb_serverinit_s
{
  uint8_t width[2];              /* U16 Framebuffer width */
  uint8_t height[2];             /* U16 Framebuffer height */
  struct rfb_pixelfmt_s format;  /*     Pixel format */
  uint8_t namelen[4];            /* U32 Name length */
  uint8_t name[1];               /* U8  Name (actual size if namelen) */
};

#define SIZEOF_RFB_SERVERINIT_S(n) \
  (sizeof(struct rfb_serverinit_s) + (n) - 1)

/* "Server-pixel-format specifies the server’s natural pixel format. This
 *  pixel format will be used unless the client requests a different format
 *  using the SetPixelFormat message (section 6.4.1).
 *
 * "Bits-per-pixel is the number of bits used for each pixel value on the
 *  wire. This must be greater than or equal to the depth which is the number
 *  of useful bits in the pixel value. Currently bits-per-pixel must be 8,
 *  16 or 32—less than 8-bit pixels are not yet supported. Big-endian-flag is
 *  non-zero (true) if multi-byte pixels are interpreted as big endian. Of
 *  course this is meaningless for 8 bits-per-pixel.
 *
 * "If true-colour-flag is non-zero (true) then the last six items specify
 *  how to extract the red, green and blue intensities from the pixel value.
 *  Red-max is the maximum red value (= 2n − 1 where n is the number of bits
 *  used for red). Note this value is always in big endian order. Red-shift
 *  is the number of shifts needed to get the red value in a pixel to the
 *  least significant bit. Green-max, green-shift and blue-max, blue-shift
 *  are similar for green and blue. For example, to find the red value
 *  (between 0 and red-max) from a given pixel, do the following:
 *
 * - "Swap the pixel value according to big-endian-flag (e.g. if
 *    big-endian-flag is zero (false) and host byte order is big endian,
 *    then swap).
 * - "Shift right by red-shift.
 * - "AND with red-max (in host byte order).
 *
 * "If true-colour-flag is zero (false) then the server uses pixel values
 *  which are not directly composed from the red, green and blue intensities,
 *  but which serve as indices into a colour map. Entries in the colour map
 *  are set by the server using the SetColourMapEntries message (section
 *  6.5.2)."
 */

/* 6.4 Client to Server Messages ********************************************/

/* "The client to server message types defined in this document are: */

#define RFB_SETPIXELFMT_MSG   0  /* SetPixelFormat */
#define RFB_SETENCODINGS_MSG  2  /* SetEncodings */
#define RFB_FBUPDATEREQ_MSG   3  /* FramebufferUpdateRequest */
#define RFB_KEYEVENT_MSG      4  /* KeyEvent */
#define RFB_POINTEREVENT_MSG  5  /* PointerEvent */
#define RFB_CLIENTCUTTEXT_MSG 6  /* ClientCutText */

/* 6.4.1 SetPixelFormat
 *
 * "Sets the format in which pixel values should be sent in FramebufferUpdate
 *  messages.  If the client does not send a SetPixelFormat message then the
 *  server sends pixel values in its natural format as specified in the
 *  ServerInit message (section 6.3.2).
 *
 * "If true-colour-flag is zero (false) then this indicates that a "colour
 *  map" is to be used.  The server can set any of the entries in the colour
 *  map using the SetColourMapEntries message (section 6.5.2). Immediately
 *  after the client has sent this message the colour map is empty, even if
 *  entries had previously been set by the server.
 */

struct rfb_setpixelformat_s
{
  uint8_t msgtype;               /* U8  Message type */
  uint8_t padding[3];
  struct rfb_pixelfmt_s format;  /* Pixel format */
};

/* 6.4.2 SetEncodings
 *
 * "Sets the encoding types in which pixel data can be sent by the server.
 *  The order of the encoding types given in this message is a hint by the
 *  client as to its preference (the first encoding specified being most
 *  preferred). The server may or may not choose to make use of this hint.
 *  Pixel data may always be sent in raw encoding even if not specified
 *  explicitly here.
 *
 * "In addition to genuine encodings, a client can request 'pseudo-
 *  encodings' to declare to the server that it supports certain extensions
 *  to the protocol. A server which does not support the extension will
 *  simply ignore the pseudo-encoding. Note that this means the client must
 *  assume that the server does not support the extension until it gets
 *  some extension-specific confirmation from the server.
 *
 * "See section 6.6 for a description of each encoding and section 6.7 for
 *  the meaning of pseudo-encodings."
 */

struct rfb_setencodings_s
{
  uint8_t msgtype;               /* U8  Message type */
  uint8_t padding;
  uint8_t nencodings[2];         /* U16 Number of encodings */
  uint8_t encodings[4];          /* S32 Encoding type, size = 4*nencodings */
};

#define SIZEOF_RFB_SETENCODINGS_S(n) \
  (sizeof(struct rfb_setencodings_s) + (((n) - 1) << 2))

/* 6.4.3 FramebufferUpdateRequest
 *
 * "Notifies the server that the client is interested in the area of the
 *  framebuffer specified by x-position, y-position, width and height. The
 *  server usually responds to a FramebufferUpdateRequest by sending a
 *  FramebufferUpdate. Note however that a single FramebufferUpdate may be
 *  sent in reply to several FramebufferUpdateRequests.
 *
 * "The server assumes that the client keeps a copy of all parts of the
 *  framebuffer in which it is interested. This means that normally the
 *  server only needs to send incremental updates to the client.
 *
 * "However, if for some reason the client has lost the contents of a
 *  particular area which it needs, then the client sends a
 *  FramebufferUpdateRequest with incremental set to zero (false). This
 *  requests that the server send the entire contents of the specified area
 *  as soon as possible. The area will not be updated using the CopyRect
 *  encoding.
 *
 * "If the client has not lost any contents of the area in which it is
 *  interested, then it sends a FramebufferUpdateRequest with incremental
 *  set to non-zero (true). If and when there are changes to the specified
 *  area of the framebuffer, the server will send a FramebufferUpdate. Note
 *  that there may be an indefinite period between the
 *  FramebufferUpdateRequest and the FramebufferUpdate.
 *
 * "In the case of a fast client, the client may want to regulate the rate
 *  at which it sends incremental FramebufferUpdateRequests to avoid hogging
 *  the network."
 */

struct rfb_fbupdatereq_s
{
  uint8_t msgtype;               /* U8  Message type */
  uint8_t incremental;           /* U8  Incremental */
  uint8_t xpos[2];               /* U16 X-position */
  uint8_t ypos[2];               /* U16 Y-position */
  uint8_t width[2];              /* U16 Width */
  uint8_t height[2];             /* U16 Height */
};

/* 6.4.4 KeyEvent
 *
 * "A key press or release. Down-flag is non-zero (true) if the key is now
 *  pressed, zero (false) if it is now released. The key itself is specified
 *  using the 'keysym' values defined by the X Window System.
 */

struct rfb_keyevent_s
{
  uint8_t msgtype;               /* U8  Message type */
  uint8_t down;                  /* U8  Down flag */
  uint8_t padding[2];
  uint8_t key[2];                /* U16 Key */
};

/* "The interpretation of keysyms is a complex area. In order to be as
 * widely interoperable as possible the following guidelines should be
 * used:
 *
 *   - "The 'shift state' (i.e. whether either of the Shift keysyms are down)
 *      should only be used as a hint when interpreting a keysym. For
 *      example, on a US keyboard the ’#’ character is shifted, but on a UK
 *      keyboard it is not. A server with a US keyboard receiving a ’#’
 *      character from a client with a UK keyboard will not have been sent
 *      any shift presses. In this case, it is likely that the server will
 *      internally need to 'fake' a shift press on its local system, in order
 *      to get a ’#’ character and not, for example, a ’3’.
 *   - "The difference between upper and lower case keysyms is significant.
 *      This is unlike some of the keyboard processing in the X Window System
 *      which treats them as the same. For example, a server receiving an
 *      uppercase ’A’ keysym without any shift presses should interpret it
 *      as an uppercase ’A’. Again this may involve an internal 'fake' shift
 *      press.
 *   - "Servers should ignore 'lock' keysyms such as CapsLock and NumLock
 *      where possible. Instead they should interpret each character-based
 *      keysym according to its case.
 *   - "Unlike Shift, the state of modifier keys such as Control and Alt
 *      should be taken as modifying the interpretation of other keysyms.
 *      Note that there are no keysyms for ASCII control characters such
 *      as ctrl-a - these should be generated by viewers sending a Control
 *      press followed by an ’a’ press.
 *   - "On a viewer where modifiers like Control and Alt can also be used to
 *      generate character-based keysyms, the viewer may need to send extra
 *      'release' events in order that the keysym is interpreted correctly.
 *      For example, on a German PC keyboard, ctrl-alt-q generates the ’@’
 *      character. In this case, the viewer needs to send 'fake' release
 *      events for Control and Alt in order that the ’@’ character is
 *      interpreted correctly (ctrl-alt-@is likely to mean something
 *      completely different to the server).
 *   - "There is no universal standard for 'backward tab' in the X Window
 *      System. On some systems shift+tab gives the keysym 'ISO Left Tab',
 *      on others it gives a private 'BackTab' keysym and on others it gives
 *      'Tab' and applications tell from the shift state that it means
 *      backward-tab rather than forward-tab. In the RFB protocol the latter
 *      approach is preferred. Viewers should generate a shifted Tab rather
 *      than ISO Left Tab. However, to be backwards-compatible with existing
 *      viewers, servers should also recognise ISO Left Tab as meaning a
 *      shifted Tab."
 */

/* 6.4.5 PointerEvent
 *
 * "Indicates either pointer movement or a pointer button press or release.
 *  The pointer is now at (x-position, y-position), and the current state of
 *  buttons 1 to 8 are represented by bits 0 to 7 of button-mask
 *  respectively, 0 meaning up, 1 meaning down (pressed).
 *
 * "On a conventional mouse, buttons 1, 2 and 3 correspond to the left,
 *  middle and right buttons on the mouse. On a wheel mouse, each step of
 *  the wheel upwards is represented by a press and release of button 4, and
 *  each step downwards is represented by a press and release of button 5."
 */

struct rfb_pointerevent_s
{
  uint8_t msgtype;               /* U8  Message type */
  uint8_t buttons;               /* U8  Button mask */
  uint8_t xpos[2];               /* U16 X-position */
  uint8_t ypos[2];               /* U16 Y-position */
};

/* 6.4.6 ClientCutText
 *
 * "The client has new ISO 8859-1 (Latin-1) text in its cut buffer. Ends of
 *  lines are represented by the linefeed / newline character (value 10)
 *  alone. No carriage-return (value 13) is needed. There is currently no
 *  way to transfer text outside the Latin-1 character set.
 */

struct rfb_clientcuttext_s
{
  uint8_t msgtype;               /* U8  Message type */
  uint8_t padding[3];
  uint8_t length[4];             /* U32 Length */
  uint8_t text[1];               /* U8  Text, actual length is Length */
};

#define SIZEOF_RFB_CLIENTCUTTEXT_S(n) \
  (sizeof(struct rfb_clientcuttext_s) + (n) - 1)

/* 6.5 Server to Client Messages ********************************************/

/* "The server to client message types defined in this document are:" */

#define RFB_FBUPDATE_MSG       0 /* FramebufferUpdate */
#define RFB_SETCOLORMAP_MSG    1 /* SetColourMapEntries */
#define RFB_BELL_MSG           2 /* Bell */
#define RFB_SERVERCUTTEXT_MSG  3 /* ServerCutText */

/* 6.5.1 FramebufferUpdate
 *
 * "A framebuffer update consists of a sequence of rectangles of pixel data
 *  which the client should put into its framebuffer. It is sent in response
 *  to a FramebufferUpdateRequest from the client. Note that there may be an
 *  indefinite period between the FramebufferUpdateRequest and the
 *  FramebufferUpdate."
 */

struct rfb_rectangle_s
{
  uint8_t xpos[2];               /* U16 X position */
  uint8_t ypos[2];               /* U16 Y position */
  uint8_t width[2];              /* U16 Width */
  uint8_t height[2];             /* U16 Height */
  uint8_t encoding[4];           /* S32 Encoding type */
  uint8_t data[1];               /* Pixel data, actual size varies */
};

#define SIZEOF_RFB_RECTANGE_S(d) \
  (sizeof(struct rfb_rectangle_s) + (d) - 1)

struct rfb_framebufferupdate_s
{
  uint8_t msgtype;                /* U8  Message type */
  uint8_t padding;
  uint8_t nrect[2];               /* U16 Number of rectangles */
  struct rfb_rectangle_s rect[1]; /* Actual number is nrect */
};

#define SIZEOF_RFB_FRAMEBUFFERUPDATE_S(r) \
  (sizeof(struct rfb_framebufferupdate_s) + (r) - sizeof(struct rfb_rectangle_s))

/* 6.5.2 SetColourMapEntries
 *
 * "When the pixel format uses a 'colour map', this message tells the client
 *  that the specified pixel values should be mapped to the given RGB
 *  intensities."
 */

struct rfb_rgb_s
{
  uint8_t r[2];                  /* U16 red */
  uint8_t g[2];                  /* U16 green */
  uint8_t b[2];                  /* U16 blue */
};

#define RFB_RGB_SIZE(n) \
  ((n) * sizeof(struct rfb_rgb_s))

struct rfb_setcolourmapentries_s
{
  uint8_t msgtype;                /* U8  Message type */
  uint8_t padding;
  uint8_t first[2];               /* U16 First colour */
  uint8_t ncolors[2];             /* U16 Number of colours */
  struct rfb_rgb_s color[1];      /*     Colors, actual number is ncolors */
};

#define SIZEOF_RFB_SETCOLOURMAPENTRIES_S(n,r) \
  (sizeof(struct rfb_setcolourmapentries_s) + RFB_RGB_SIZE((n) - 1))

/* 6.5.3 Bell
 *
 * "Ring a bell on the client if it has one.
 */

struct rfb_bell_s
{
  uint8_t msgtype;                /* U8  Message type */
};

/* 6.5.4 ServerCutText
 *
 * "The server has new ISO 8859-1 (Latin-1) text in its cut buffer. Ends of
 *  lines are represented by the linefeed / newline character (value 10)
 *  alone. No carriage-return (value 13) is needed. There is currently no
 *  way to transfer text outside the Latin-1 character set.
 */

struct rfb_servercuttext_s
{
  uint8_t msgtype;               /* U8  Message type */
  uint8_t padding[3];
  uint8_t length[4];             /* U32 Length */
  uint8_t text[1];               /* U8  Text, actual length is Length */
};

#define SIZEOF_RFB_SERVERCUTTEXT_S(n) \
  (sizeof(struct rfb_servercuttext_s) + (n) - 1)

/* 6.6 Encodings ************************************************************/

/* The encodings defined in this document are: */

#define RFB_ENCODING_RAW       0  /* Raw */
#define RFB_ENCODING_COPYRECT  1  /* CopyRect */
#define RFB_ENCODING_RRE       2  /* RRE */
#define RFB_ENCODING_HEXTILE   5  /* Hextile */
#define RFB_ENCODING_ZRLE     16  /* ZRLE */
#define RFB_ENCODING_CURSOR  -239 /* Cursor pseudo-encoding */
#define RFB_ENCODING_DESKTOP -223 /* DesktopSize pseudo-encoding */

/* 6.6.1 Raw encoding
 *
 * "The simplest encoding type is raw pixel data. In this case the data
 *  consists of width x height pixel values (where width and height are the
 *  width and height of the rectangle).  The values simply represent each
 *  pixel in left-to-right scanline order. All RFB clients must be able to
 *  cope with pixel data in this raw encoding, and RFB servers should only
 *  produce raw encoding unless the client specifically asks for some other
 *  encoding type.
 */

/* 6.6.2 CopyRect encoding
 *
 * "The CopyRect (copy rectangle) encoding is a very simple and efficient
 *  encoding which can be used when the client already has the same pixel
 *  data elsewhere in its framebuffer.  The encoding on the wire simply
 *  consists of an X,Y coordinate. This gives a position in the framebuffer
 *  from which the client can copy the rectangle of pixel data. This can be
 *  used in a variety of situations, the most obvious of which are when the
 *  user moves a window across the screen, and when the contents of a window
 *  are scrolled. A less obvious use is for optimising drawing of text or
 *  other repeating patterns. An intelligent server may be able to send a
 *  pattern explicitly only once, and knowing the previous position of the
 *  pattern in the framebuffer, send subsequent occurrences of the same
 *  pattern using the CopyRect encoding."
 */

struct rfb_copyrect_encoding_s
{
  uint8_t xpos[2];               /* U16 Source x position */
  uint8_t ypos[2];               /* U16 Source y position */
};

/* 6.6.3 RRE encoding
 *
 * "RRE stands for rise-and-run-length encoding and as its name implies, it
 *  is essentially a two-dimensional analogue of run-length encoding.
 *  RRE-encoded rectangles arrive at the client in a form which can be
 *  rendered immediately and efficiently by the simplest of graphics
 *  engines. RRE is not appropriate for complex desktops, but can be
 *  useful in some situations.
 *
 * "The basic idea behind RRE is the partitioning of a rectangle of pixel
 *  data into rectangular subregions (subrectangles) each of which consists
 *  of pixels of a single value and the union of which comprises the
 *  original rectangular region. The near-optimal partition of a given
 *  rectangle into such subrectangles is relatively easy to compute.
 *
 * "The encoding consists of a background pixel value, Vb (typically the
 *  most prevalent pixel value in the rectangle) and a count N, followed
 *  by a list of N subrectangles, each of which consists of a tuple
 *  < v, x, y, w, h > where v (6= Vb) is the pixel value, (x, y) are the
 *  coordinates of the subrectangle relative to the top-left corner of the
 *  rectangle, and (w, h) are the width and height of the subrectangle. The
 *  client can render the original rectangle by drawing a filled rectangle
 *  of the background pixel value and then drawing a filled rectangle
 *  corresponding to each subrectangle. On the wire, the data begins with
 *  the header:"
 */

struct rfb_rrehdr8_s
{
  uint8_t nsubrects[4];          /* U32 Number of sub-rectangle */
  uint8_t pixel;                 /* U8  Background pixel */
};

struct rfb_rrehdr16_s
{
  uint8_t nsubrects[4];          /* U32 Number of sub-rectangle */
  uint8_t pixel[2];              /* U16 Background pixel */
};

struct rfb_rrehdr32_s
{
  uint8_t nsubrects[4];          /* U32 Number of sub-rectangle */
  uint8_t pixel[4];              /* U32 Background pixel */
};

/* "This is followed by number-of-subrectangles instances of the following
 *  structure:"
 */

struct rfb_rrerect8_s
{
  uint8_t pixel;                 /* U8  sub-rect pixel value */
  uint8_t xpos[2];               /* U16 X position */
  uint8_t ypos[2];               /* U16 Y position */
  uint8_t width[2];              /* U16 Width */
  uint8_t height[2];             /* U16 Height */
};

struct rfb_rrerect16_s
{
  uint8_t pixel[2];              /* U16 sub-rect pixel value */
  uint8_t xpos[2];               /* U16 X position */
  uint8_t ypos[2];               /* U16 Y position */
  uint8_t width[2];              /* U16 Width */
  uint8_t height[2];             /* U16 Height */
};

struct rfb_rrerect32_s
{
  uint8_t pixel[4];              /* U32 sub-rect pixel value */
  uint8_t xpos[2];               /* U16 X position */
  uint8_t ypos[2];               /* U16 Y position */
  uint8_t width[2];              /* U16 Width */
  uint8_t height[2];             /* U16 Height */
};

/* 6.6.4 Hextile encoding
 *
 * "Hextile is a variation on the RRE idea. Rectangles are split up into
 *  16x16 tiles, allowing the dimensions of the subrectangles to be
 *  specified in 4 bits each, 16 bits in total. The rectangle is split
 *  into tiles starting at the top left going in left-to-right, top-to-
 *  bottom order. The encoded contents of the tiles simply follow one another
 *  in the predetermined order. If the width of the whole rectangle is not
 *  an exact multiple of 16 then the width of the last tile in each row will
 *  be correspondingly smaller. Similarly if the height of the whole
 *  rectangle is not an exact multiple of 16 then the height of each tile in
 *  the final row will also be smaller.
 *
 * "Each tile is either encoded as raw pixel data, or as a variation on RRE.
 *  Each tile has a background pixel value, as before. The background pixel
 *  value does not need to be explicitly specified for a given tile if it is
 *  the same as the background of the previous tile. However the background
 *  pixel value may not be carried over if the previous tile was Raw. If all
 *  of the subrectangles of a tile have the same pixel value, this can be
 *  specified once as a foreground pixel value for the whole tile. As with
 *  the background, the foreground pixel value can be left unspecified,
 *  meaning it is carried over from the previous tile. The foreground pixel
 *  value may not be carried over if the previous tile had the Raw or
 *  SubrectsColoured bits set. It may, however, be carried over from a
 *  previous tile with the AnySubrects bit clear, as long as that tile
 *  itself carried over a valid foreground from its previous tile.
 *
 * "So the data consists of each tile encoded in order. Each tile begins
 *  with a subencoding type byte, which is a mask made up of a number of
 *  bits:"
 */

#define RFB_SUBENCODING_RAW      1  /* Raw */
#define RFB_SUBENCODING_BACK     2  /* BackgroundSpecified*/
#define RFB_SUBENCODING_FORE     4  /* ForegroundSpecified*/
#define RFB_SUBENCODING_ANY      8  /* AnySubrects*/
#define RFB_SUBENCODING_COLORED  16 /* SubrectsColoured*/

/* "If the Raw bit is set then the other bits are irrelevant; width x height
 *  pixel values follow (where width and height are the width and height of
 *  the tile). Otherwise the other bits in the mask are as follows:
 *
 * "BackgroundSpecified - if set, a pixel value follows which specifies the
*   background colour for this tile:"
*/

struct rfb_backpixel8_s
{
  uint8_t pixel;                 /* U8  Background pixel value */
};

struct rfb_backpixel16_s
{
  uint8_t pixel[2];              /* U16 Background pixel value */
};

struct rfb_backpixel32_s
{
  uint8_t pixel[4];              /* U32 Background pixel value */
};

/* "The first non-raw tile in a rectangle must have this bit set. If this
 *  bit isn’t set then the background is the same as the last tile.
 *
 * "ForegroundSpecified - if set, a pixel value follows which specifies the
 *  foreground colour to be used for all subrectangles in this tile:"
 */

struct rfb_forepixel8_s
{
  uint8_t pixel;                 /* U8  Foreground pixel value */
};

struct rfb_forepixel16_s
{
  uint8_t pixel[2];              /* U16 Foreground pixel value */
};

struct rfb_forepixel32_s
{
  uint8_t pixel[4];              /* U32 Foreground pixel value */
};

/* "If this bit is set then the SubrectsColoured bit must be zero.
 *
 * "AnySubrects - if set, a single byte follows giving the number of
 *  subrectangles following:"
 */

struct rfb_nrects_s
{
  uint8_t nsubrects;             /* U8  Number of sub-rectangles */
};

/* "If not set, there are no subrectangles (i.e. the whole tile is just
 *  solid background colour).
 *
 * "SubrectsColoured - if set then each subrectangle is preceded by a pixel
 *  value giving the colour of that subrectangle, so a subrectangle is:"
 */

struct rfb_subrectscolored8_s
{
  uint8_t pixel;                 /* U8  Sub-rect pixel value */
  uint8_t xy;                    /* U8  X and y position */
  uint8_t wh;                    /* U8  Width and height */
};

struct rfb_subrectscolored16_s
{
  uint8_t pixel[2];              /* U16 Sub-rect pixel value */
  uint8_t xy;                    /* U8  X and y position */
  uint8_t wh;                    /* U8  Width and height */
};

struct rfb_subrectscolored32_s
{
  uint8_t pixel[4];              /* U32 Sub-rect pixel value */
  uint8_t xy;                    /* U8  X and y position */
  uint8_t wh;                    /* U8  Width and height */
};

/* "If not set, all subrectangles are the same colour, the foreground
 *  colour; if the ForegroundSpecified bit wasn’t set then the foreground
 *  is the same as the last tile. A subrectangle is:"
 */

struct rfb_subrect_s
{
  uint8_t xy;                    /* U8  X and y position */
  uint8_t wh;                    /* U8  Width and height */
};

/* "The position and size of each subrectangle is specified in two bytes,
 *  x-and-y-position and width-and-height. The most-significant four bits of
 *  x-and-y-position specify the X position, the least-significant specify
 *  the Y position. The most-significant four bits of width-and-height
 *  specify the width minus one, the least-significant specify the height
 *  minus one."
 */

/* 6.6.5 ZRLE encoding
 *
 * "ZRLE stands for Zlib1 Run-Length Encoding, and combines zlib
 *  compression, tiling, palettisation and run-length encoding. On the wire,
 *  the rectangle begins with a 4-byte length field, and is followed by that
 *  many bytes of zlib-compressed data. A single zlib 'stream' object is
 *  used for a given RFB protocol connection, so that ZRLE rectangles must
 *  be encoded and decoded strictly in order."
 */

struct rfb_srle_s
{
  uint8_t length[4];             /* U32 Length */
  uint8_t data[1];               /* U8  zlibData, actual size is length */
};

#define SIZEOF_RFB_SRLE_S(n,r) \
  (sizeof(struct rfb_srle_s) + (n) - 1)

/* "The zlibData when uncompressed represents tiles of 64x64 pixels in
 *  left-to-right, top-to-bottom order, similar to hextile. If the width of
 *  the rectangle is not an exact multiple of 64 then the width of the last
 *  tile in each row is smaller, and if the height of the rectangle is not
 *  an exact multiple of 64 then the height of each tile in the final row
 *  is smaller.
 *
 * "ZRLE makes use of a new type CPIXEL (compressed pixel). This is the same
 *  as a PIXEL for the agreed pixel format, except where true-colour-flag is
 *  non-zero, bitsper-pixel is 32, depth is 24 or less and all of the bits
 *  making up the red, green and blue intensities fit in either the least
 *  significant 3 bytes or the most significant 3 bytes. In this case a
 *  CPIXEL is only 3 bytes long, and contains the least significant or the
 *  most significant 3 bytes as appropriate. bytesPerCPixel is the number of
 *  bytes in a CPIXEL.
 *
 * "Each tile begins with a subencoding type byte. The top bit of this byte
 *  is set if the tile has been run-length encoded, clear otherwise. The
 *  bottom seven bits indicate the size of the palette used - zero means no
 *  palette, one means that the tile is of a single colour, 2 to 127
 *  indicate a palette of that size. The possible values of subencoding are:"
 */

#define RFB_SUBENCODING_RAW      0   /* Raw pixel data */
#define RFB_SUBENCODING_SOLID    1   /* A solid tile of a single color */
#define RFB_SUBENCODING_PACKED1  2   /* Packed palette types */
#define RFB_SUBENCODING_PACKED2  3
#define RFB_SUBENCODING_PACKED3  4
#define RFB_SUBENCODING_PACKED4  5
#define RFB_SUBENCODING_PACKED5  6
#define RFB_SUBENCODING_PACKED6  7
#define RFB_SUBENCODING_PACKED7  8
#define RFB_SUBENCODING_PACKED8  9
#define RFB_SUBENCODING_PACKED9  10
#define RFB_SUBENCODING_PACKED10 11
#define RFB_SUBENCODING_PACKED11 12
#define RFB_SUBENCODING_PACKED12 13
#define RFB_SUBENCODING_PACKED13 14
#define RFB_SUBENCODING_PACKED14 15
#define RFB_SUBENCODING_PACKED15 16
#define RFB_SUBENCODING_RLE      128 /* Plain RLE */
#define RFB_SUBENCODING_PALRLE   129 /* Palette RLE */


/* "Raw pixel data. width x height pixel values follow (where width and
 *  height are the width and height of the tile):"
 */

struct rfb_rawpixel8_s
{
  uint8_t pixels[1];             /* Actual size is w*h */
};

#define SIZEOF_RFB_RAWPIXEL8_S(n,r) \
  (sizeof(struct rfb_rawpixel8_s) + (n) - 1)

struct rfb_rawpixel16_s
{
  uint8_t pixels[2];             /* Actual size is 2*w*h */
};

#define SIZEOF_RFB_RAWPIXEL16_S(n,r) \
  (sizeof(struct rfb_rawpixel16_s) + (((n) - 1) << 1))

struct rfb_rawpixel32_s
{
  uint8_t pixels[4];             /* Actual size is 4*w*h */
};

#define SIZEOF_RFB_RAWPIXEL32_S(n,r) \
  (sizeof(struct rfb_rawpixel32_s) + (((n) - 1) << 2))

/* "A solid tile consisting of a single colour. The pixel value follows:" */

struct rfb_solid8_s
{
  uint8_t pixels;                /* Pixel value */
};

struct rfb_solid16_s
{
  uint8_t pixels[2];             /* Pixel value */
};

struct rfb_solid32_s
{
  uint8_t pixels[4];             /* Pixel value */
};

/* "Packed palette types. Followed by the palette, consisting of
 *  paletteSize(= subencoding) pixel values. Then the packed pixels follow,
 *  each pixel represented as a bit field yielding an index into the palette
 *  (0 meaning the first palette entry). For paletteSize 2, a 1-bit field is
 *  used, for paletteSize 3 or 4 a 2-bit field is used and for paletteSize
 *  from 5 to 16 a 4-bit field is used. The bit fields are packed into bytes,
 *  the most significant bits representing the leftmost pixel (i.e. big
 *  endian). For tiles not a multiple of 8, 4 or 2 pixels wide (as
 *  appropriate), padding bits are used to align each row to an exact number
 *  of bytes."
 *
 * REVISIT:  Difficult to represent since it is a variable length array of
 * RGB pixels follow by the variable length array of packed pixels.
 */

/* "Plain RLE. Consists of a number of runs, repeated until the tile is
 *  done. Runs may continue from the end of one row to the beginning of the
 *  next. Each run is a represented by a single pixel value followed by the
 *  length of the run. The length is represented as one or more bytes. The
 *  length is calculated as one more than the sum of all the bytes
 *  representing the length. Any byte value other than 255 indicates the
 *  final byte. So for example length 1 is represented as [0], 255 as [254],
 *  256 as [255,0], 257 as [255,1], 510 as [255,254], 511 as [255,255,0] and
 *  so on."
 *
 * REVISIT:  Difficult to represent.  Consists of (1) a color value, (2)
 * a variable length array of data, followed by (3) the run length.
 */

/* "Palette RLE. Followed by the palette, consisting of paletteSize =
 *  (subencoding − 128) pixel values:"
 */

struct rfb_palette8_s
{
  uint8_t palette[1];            /* Actual size is palleteSize */
};

struct rfb_palette16_s
{
  uint8_t palette[2];            /* Actual size is 2*palleteSize */
};

struct rfb_palette32_s
{
  uint8_t palette[4];            /* Actual size is 4*palleteSize */
};

/* Then as with plain RLE, consists of a number of runs, repeated until the
 * tile is done. A run of length one is represented simply by a palette index:
 */

struct rfb_palettendx_s
{
  uint8_t index;                /* U8  Palette Index */
};

/* A run of length more than one is represented by a palette index with the
 * top bit set, followed by the length of the run as for plain RLE."
 *
 * REVISIT:  See comment with similar plain RLE representation.
 */

/* 6.7 Pseudo-Encodings *****************************************************/

/* 6.7.1 Cursor pseudo-encoding
 *
 * "A client which requests the Cursor pseudo-encoding is declaring that it
 *  is capable of drawing a mouse cursor locally. This can significantly
 *  improve perceived performance over slow links. The server sets the
 *  cursor shape by sending a pseudo-rectangle with the Cursor pseudo-
 *  encoding as part of an update. The pseudo-rectangle’s x-position and
 *  y-position indicate the hotspot of the cursor, and width and height
 *  indicate the width and height of the cursor in pixels. The data consists
 *  of width x height pixel values followed by a bitmask. The bitmask
 *  consists of left-to-right, top-to-bottom scanlines, where each scanline
 *  is padded to a whole number of bytes floor((width + 7)/8). Within each
 *  byte the most significant bit represents the leftmost pixel, with a
 *  1-bit meaning the corresponding pixel in the cursor is valid."
 *
 * REVISIT:  Also difficult to represent:  A variable length pixel array
 * followed by a variable length bit mask.
 */

/* 6.7.2 DesktopSize pseudo-encoding
 *
 * "A client which requests the DesktopSize pseudo-encoding is declaring
 *  that it is capable of coping with a change in the framebuffer width
 *  and/or height. The server changes the desktop size by sending a
 *  pseudo-rectangle with the DesktopSize pseudo-encoding as the last
 *  rectangle in an update. The pseudo-rectangle’s x-position and y-position
 *  are ignored, and width and height indicate the new width and height of
 *  the framebuffer. There is no further data associated with the
 *  pseudo-rectangle.
 */

/* Data Access Helpers ******************************************************/

/* All multiple byte integers (other than pixel values themselves) are in
 * big endian order (most significant byte first).  The following do not
 * depend on the endianness of the target nor that they depend on any data
 * alignment.
 */

/* void rfb_putbe16(FAR uint8_t *dest, uint16_t value) */

#define rfb_putbe16(d,v) \
  do \
    { \
      register FAR uint8_t *__d = (FAR uint8_t *)(d); \
      *__d++ = ((uint16_t)(v) >> 8); \
      *__d   = ((uint16_t)(v) & 0xff); \
    } \
  while (0)

/* uin16_t rfb_getbe16(FAR const uint8_t *src) */

#define rfb_getbe16(s) \
  (((uint16_t)((s)[0]) << 8) | \
    (uint16_t)((s)[1]))

/* void rfb_putbe32(FAR uint8_t *dest, uint32_t value) */

#define rfb_putbe32(d,v) \
  do \
    { \
      register FAR uint8_t *__d = (FAR uint8_t *)(d); \
      *__d++ = ((uint32_t)(v) >> 24); \
      *__d++ = ((uint32_t)(v) >> 16) & 0xff; \
      *__d++ = ((uint32_t)(v) >> 8)  & 0xff; \
      *__d   = ((uint32_t)(v)        & 0xff); \
    } \
  while (0)

/* uint32_t rfb_getbe32(FAR const uint8_t *src) */

#define rfb_getbe32(s) \
  (((uint32_t)((s)[0]) << 24) | \
   ((uint32_t)((s)[1]) << 16) | \
   ((uint32_t)((s)[2]) <<  8) | \
    (uint32_t)((s)[3]))

/* There are also cases where the client may request pixel data in its
 * little-endian format.
 */

/* void rfb_putle16(FAR uint8_t *dest, uint16_t value) */

#define rfb_putle16(d,v) \
  do \
    { \
      register FAR uint8_t *__d = (FAR uint8_t *)(d); \
      *__d++ = ((uint16_t)(v) & 0xff); \
      *__d   = ((uint16_t)(v) >> 8); \
    } \
  while (0)

/* uin16_t rfb_getle16(FAR const uint8_t *src) */

#define rfb_getle16(s) \
  (((uint16_t)((s)[1]) << 8) | \
    (uint16_t)((s)[0]))

/* void rfb_putle32(FAR uint8_t *dest, uint32_t value) */

#define rfb_putle32(d,v) \
  do \
    { \
      register FAR uint8_t *__d = (FAR uint8_t *)(d); \
      *__d++ = ((uint32_t)(v)        & 0xff); \
      *__d++ = ((uint32_t)(v) >> 8)  & 0xff; \
      *__d++ = ((uint32_t)(v) >> 16) & 0xff; \
      *__d   = ((uint32_t)(v) >> 24); \
    } \
  while (0)

/* uint32_t rfb_getle32(FAR const uint8_t *src) */

#define rfb_getle32(s) \
  (((uint32_t)((s)[3]) << 24) | \
   ((uint32_t)((s)[2]) << 16) | \
   ((uint32_t)((s)[1]) <<  8) | \
    (uint32_t)((s)[0]))

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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif  /* __INCLUDE_NUTTX_VIDEO_RFB_H */

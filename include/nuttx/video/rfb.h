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
  (sizeof(struct rfb_supported_sectypes_s) + (n) - 1);

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
  (sizeof(struct rfb_sectype_connfail_s) + (n) - 1);

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
#define RFB_SECTYPE_FAILE   1

struct rfb_sectype_result_s
{
  uint8_t result[4];             /* U32 Security type result */
};

/* "If successful, the protocol passes to the initialisation phase (section
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
  (sizeof(struct rfb_sectype_fail_s) + (n) - 1);

/* "Version 3.3 and 3.7 If unsuccessful, the server closes the connection." */

/* 6.2 Security Types *******************************************************/

/* 6.2.1 None
 *
 * "No authentication is needed and protocol data is to be sent unencrypted.
 *
 * "Version 3.8 onwards The protocol continues with the SecurityResult
 *  message."
 *
 * "Version 3.3 and 3.7 The protocol passes to the initialisation phase
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
 *  initialisation phase. The client sends a ClientInit message followed by
 *  the server sending a ServerInit message.
 *
 * "Shared-flag is non-zero (true) if the server should try to share the
 *  desktop by leaving other clients connected, zero (false) if it should
 *  give exclusive access to this client by disconnecting all other clients."
 */

#defin RFB_FLAG_SHARED    0
#defin RFB_FLAG_EXCLUSIVE 1

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
  (sizeof(struct rfb_serverinit_s) + (n) - 1);

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

/* "The client to server message types defined in this document are:
 *
 * "Number   Name
 *   0       SetPixelFormat
 *   2       SetEncodings
 *   3       FramebufferUpdateRequest
 *   4       KeyEvent
 *   5       PointerEvent
 *   6       ClientCutText
 *
 * "Other registered message types are:
 *
 * "Number   Name
 *  255      Anthony Liguori
 *  254, 127 VMWare
 *  253      gii
 *  252      tight
 *  251      Pierre Ossman SetDesktopSize
 *  250      Colin Dean xvp
 *  249      OLIVE Call Control
 *
 * "Note that before sending a message not defined in this document a client
 *  must have determined that the server supports the relevant extension by 
 *  receiving some extension-specific confirmation from the server.
 */

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
  uint8_t padding[3];
  uint8_t nencodings[4];         /* U32 Number of encodings */
  uint8_t encodings[4];          /* S32 Encoding type, size = 4*nencodings */
};

#define SIZEOF_RFB_SERVERINIT_S(n) \
  (sizeof(struct rfb_serverinit_s) + (((n) - 1) << 2));

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
  uint8_t length[2];             /* U8  Length */
  uint8_t text[1];               /* U8  Text, actual length is Length */
};

#define SIZEOF_RFB_CLIENTCUTTEXT_S(n) \
  (sizeof(struct rfb_clientcuttext_s) + (n) - 1);

/* 6.5 Server to Client Messages ********************************************/

/* 6.6 Encodings ************************************************************/

/* 6.7 Pseudo-Encodings *****************************************************/

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

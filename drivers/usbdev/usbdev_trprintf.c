/****************************************************************************
 * drivers/usbdev/usbdev_trprintf.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <debug.h>

#include <nuttx/usb/usbdev_trace.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_trstring
 *
 * Description:
 *   Search the driver string data to find the string matching the
 *   provided ID.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_TRACE_STRINGS
static FAR const char *get_trstring(FAR const struct trace_msg_t *array,
                                    uint8_t id)
{
  FAR const struct trace_msg_t *p = array;
  while (p->str != NULL)
    {
      if (p->id == id)
        {
          return p->str;
        }

      p++;
    }

  return "???";
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbtrace_trprintf
 *
 * Description:
 *   Print the trace record using the supplied printing function
 *
 ****************************************************************************/

void usbtrace_trprintf(trprintf_t trprintf, uint16_t event, uint16_t value)
{
  switch (event)
    {
    case TRACE_DEVINIT:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "DCD initialize",
               "Initialized", value);
#else
      trprintf("%-18s   : %04x\n", "DCD initialize",
               value);
#endif
      break;

    case TRACE_DEVUNINIT:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "DCD un-initialize",
               "Un-initialized", value);
#else
      trprintf("%-18s   : %04x\n", "DCD un-initialize",
               value);
#endif
      break;

    case TRACE_DEVREGISTER:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "DCD register",
               "Registered", value);
#else
      trprintf("%-18s   : %04x\n", "DCD register",
               value);
#endif
      break;

    case TRACE_DEVUNREGISTER:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "DCD un-register",
               "Un-registered", value);
#else
      trprintf("%-18s   : %04x\n", "DCD un-register",
               value);
#endif
      break;

    case TRACE_EPCONFIGURE:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "EP configure",
               "Endpoint configured", value);
#else
      trprintf("%-18s   : %04x\n", "EP configure",
               value);
#endif
      break;

    case TRACE_EPDISABLE:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "EP disable",
               "Endpoint disabled", value);
#else
      trprintf("%-18s   : %04x\n", "EP disable",
               value);
#endif
      break;

    case TRACE_EPALLOCREQ:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "EP allocreq",
               "Allocate endpoint request", value);
#else
      trprintf("%-18s   : %04x\n", "EP allocreq",
               value);
#endif
      break;

    case TRACE_EPFREEREQ:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "EP freereq",
               "Free endpoint request", value);
#else
      trprintf("%-18s   : %04x\n", "EP freereq",
               value);
#endif
      break;

    case TRACE_EPALLOCBUFFER:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "EP allocbuffer",
               "Allocate endpoint buffer", value);
#else
      trprintf("%-18s   : %04x\n", "EP allocbuffer",
               value);
#endif
      break;

    case TRACE_EPFREEBUFFER:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "EP freebuffer",
               "Free endpoint buffer", value);
#else
      trprintf("%-18s   : %04x\n", "EP freebuffer",
               value);
#endif
      break;

    case TRACE_EPSUBMIT:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "EP submit",
               "Submit endpoint request", value);
#else
      trprintf("%-18s   : %04x\n", "EP submit",
               value);
#endif
      break;

    case TRACE_EPCANCEL:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "EP cancel",
               "Cancel endpoint request", value);
#else
      trprintf("%-18s   : %04x\n", "EP cancel",
               value);
#endif
      break;

    case TRACE_EPSTALL:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "EP stall",
               "Stall endpoint", value);
#else
      trprintf("%-18s   : %04x\n", "EP stall",
               value);
#endif
      break;

    case TRACE_EPRESUME:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "EP resume",
               "Resume endpoint", value);
#else
      trprintf("%-18s   : %04x\n", "EP resume",
               value);
#endif
      break;

    case TRACE_DEVALLOCEP:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "DCD allocep",
               "Allocate endpoint", value);
#else
      trprintf("%-18s   : %04x\n", "DCD allocep",
               value);
#endif
      break;

    case TRACE_DEVFREEEP:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "DCD freeep",
               "Free endpoint", value);
#else
      trprintf("%-18s   : %04x\n", "DCD freeep",
               value);
#endif
      break;

    case TRACE_DEVGETFRAME:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "DCD getframe",
               "Get frame number", value);
#else
      trprintf("%-18s   : %04x\n", "DCD getframe",
               value);
#endif
      break;

    case TRACE_DEVWAKEUP:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "DCD wakeup",
               "Wakeup event", value);
#else
      trprintf("%-18s   : %04x\n", "DCD wakeup",
               value);
#endif
      break;

    case TRACE_DEVSELFPOWERED:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "DCD selfpowered",
               "Self-powered", value);
#else
      trprintf("%-18s   : %04x\n", "DCD selfpowered",
               value);
#endif
      break;

    case TRACE_DEVPULLUP:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "DCD pullup",
               "Soft connect", value);
#else
      trprintf("%-18s   : %04x\n", "DCD pullup",
               value);
#endif
      break;

    case TRACE_CLASSBIND:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "Class bind",
               "Bind class", value);
#else
      trprintf("%-18s   : %04x\n", "Class bind",
               value);
#endif
      break;

    case TRACE_CLASSUNBIND:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "Class unbind",
               "Un-bind class", value);
#else
      trprintf("%-18s   : %04x\n", "Class unbind",
               value);
#endif
      break;

    case TRACE_CLASSDISCONNECT:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "Class disconnect",
               "Disconnect class", value);
#else
      trprintf("%-18s   : %04x\n", "Class disconnect",
               value);
#endif
      break;

    case TRACE_CLASSSETUP:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "Class setup",
               "Class SETUP request", value);
#else
      trprintf("%-18s   : %04x\n", "Class setup",
               value);
#endif
      break;

    case TRACE_CLASSSUSPEND:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "Class suspend",
               "Suspend class", value);
#else
      trprintf("%-18s   : %04x\n", "Class suspend",
               value);
#endif
      break;

    case TRACE_CLASSRESUME:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "Class resume",
               "Resume class", value);
#else
      trprintf("%-18s   : %04x\n", "Class resume",
               value);
#endif
      break;

    case TRACE_CLASSRDCOMPLETE:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "Class RD complete",
               "Read request complete", value);
#else
      trprintf("%-18s   : %04x\n", "Class RD complete",
               value);
#endif
      break;

    case TRACE_CLASSWRCOMPLETE:
#ifdef CONFIG_USBDEV_TRACE_STRINGS
      trprintf("%-18s   : %-40s %04x\n", "Class RW complete",
               "Write request complete", value);
#else
      trprintf("%-18s   : %04x\n", "Class RW complete",
               value);
#endif
      break;

    default:
      switch (TRACE_ID(event))
        {
        case TRACE_CLASSAPI_ID:        /* Other class driver system API calls */
#ifdef CONFIG_USBDEV_TRACE_STRINGS
          trprintf("%-18s %02x: %-40s %04x\n", "Class API call",
                   TRACE_DATA(event),
                   get_trstring(g_usb_trace_strings_clsapi,
                   TRACE_DATA(event)),
                   value);
#else
          trprintf("%-18s %02x: %04x\n", "Class API call",
                   TRACE_DATA(event), value);
#endif
          break;

        case TRACE_CLASSSTATE_ID:      /* Track class driver state changes */
#ifdef CONFIG_USBDEV_TRACE_STRINGS
          trprintf("%-18s %02x: %-40s %04x\n", "Class state",
                   TRACE_DATA(event),
                   get_trstring(g_usb_trace_strings_clsstate,
                   TRACE_DATA(event)),
                   value);
#else
          trprintf("%-18s %02x: %04x\n", "Class state",
                   TRACE_DATA(event), value);
#endif
          break;

        case TRACE_INTENTRY_ID:        /* Interrupt handler entry */
#ifdef CONFIG_USBDEV_TRACE_STRINGS
          trprintf("%-18s%3d: %-40s %04x\n", "Interrupt entry",
                   TRACE_DATA(event), "ENTRY", value);
#else
          trprintf("%-18s%3d: %04x\n", "Interrupt entry",
                   TRACE_DATA(event), value);
#endif
          break;

        case TRACE_INTDECODE_ID:       /* Decoded interrupt event */
#ifdef CONFIG_USBDEV_TRACE_STRINGS
          trprintf("%-18s%3d: %-40s %04x\n", "Interrupt decode",
                   TRACE_DATA(event),
                   get_trstring(g_usb_trace_strings_intdecode,
                   TRACE_DATA(event)),
                   value);
#else
          trprintf("%-18s%3d: %04x\n", "Interrupt decode",
                   TRACE_DATA(event), value);
#endif
          break;

        case TRACE_INTEXIT_ID:         /* Interrupt handler exit */
#ifdef CONFIG_USBDEV_TRACE_STRINGS
          trprintf("%-18s%3d: %-40s %04x\n", "Interrupt exit",
                   TRACE_DATA(event), "EXIT", value);
#else
          trprintf("%-18s%3d: %04x\n", "Interrupt exit",
                   TRACE_DATA(event), value);
#endif
          break;

        case TRACE_OUTREQQUEUED_ID:    /* Request queued for OUT endpoint */
#ifdef CONFIG_USBDEV_TRACE_STRINGS
          trprintf("EP%2d %-16s: %-40s %04x\n",
                   TRACE_DATA(event), "OUT queued",
                   "Read request queued", value);
#else
          trprintf("EP%2d %-16s: %04x\n",
                   TRACE_DATA(event), "OUT queued", value);
#endif
          break;

        case TRACE_INREQQUEUED_ID:     /* Request queued for IN endpoint */
#ifdef CONFIG_USBDEV_TRACE_STRINGS
          trprintf("EP%2d %-16s: %-40s %04x\n",
                   TRACE_DATA(event), "IN queued",
                   "Write request queued", value);
#else
          trprintf("EP%2d %-16s: %04x\n",
                   TRACE_DATA(event), "IN queued", value);
#endif
          break;

        case TRACE_READ_ID:            /* Read (OUT) action */
#ifdef CONFIG_USBDEV_TRACE_STRINGS
          trprintf("EP%2d %-16s: %-40s %04x\n",
                   TRACE_DATA(event), "OUT read",
                   "Incoming data read", value);
#else
          trprintf("EP%2d %-16s: %04x\n",
                   TRACE_DATA(event), "OUT read", value);
#endif
          break;

        case TRACE_WRITE_ID:           /* Write (IN) action */
#ifdef CONFIG_USBDEV_TRACE_STRINGS
          trprintf("EP%2d %-16s: %-40s %04x\n",
                   TRACE_DATA(event), "IN write",
                   "Outgoing data written", value);
#else
          trprintf("EP%2d %-16s: %04x\n",
                   TRACE_DATA(event), "IN write", value);
#endif
          break;

        case TRACE_COMPLETE_ID:        /* Request completed */
#ifdef CONFIG_USBDEV_TRACE_STRINGS
          trprintf("EP%2d %-16s: %-40s %04x\n",
                   TRACE_DATA(event), "Request complete",
                   "Request completed", value);
#else
          trprintf("EP%2d %-16s: %04x\n",
                   TRACE_DATA(event), "Request complete", value);
#endif
          break;

        case TRACE_DEVERROR_ID:        /* USB controller driver error event */
#ifdef CONFIG_USBDEV_TRACE_STRINGS
          trprintf("%-18s %02x: %-40s %04x\n", "Controller error",
                   TRACE_DATA(event),
                   get_trstring(g_usb_trace_strings_deverror,
                   TRACE_DATA(event)),
                   value);
#else
          trprintf("%-18s %02x: %04x\n", "Controller error",
                   TRACE_DATA(event),
                   value);
#endif
          break;

        case TRACE_CLSERROR_ID:        /* USB class driver error event */
#ifdef CONFIG_USBDEV_TRACE_STRINGS
          trprintf("%-18s %02x: %-40s %04x\n", "Class error",
                   TRACE_DATA(event),
                   get_trstring(g_usb_trace_strings_clserror,
                   TRACE_DATA(event)),
                   value);
#else
          trprintf("%-18s %02x: %04x\n", "Class error",
                   TRACE_DATA(event), value);
#endif
          break;

        default:
          trprintf("Unrecognized event   : %02x:%02x:%04x\n",
                TRACE_ID(event) >> 8, TRACE_DATA(event), value);
          break;
        }
    }
}

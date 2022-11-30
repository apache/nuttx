/********************************************************************************
 * tools/nxstyle.c
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <limits.h>
#include <unistd.h>
#include <libgen.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

#define NXSTYLE_VERSION "0.01"

#define LINE_SIZE      512
#define RANGE_NUMBER   4096
#define DEFAULT_WIDTH  78

#define FIRST_SECTION  INCLUDED_FILES
#define LAST_SECTION   PUBLIC_FUNCTION_PROTOTYPES

#define FATAL(m, l, o) message(FATAL, (m), (l), (o))
#define FATALFL(m, s)  message(FATAL, (m), -1, -1)
#define WARN(m, l, o)  message(WARN, (m), (l), (o))
#define ERROR(m, l, o) message(ERROR, (m), (l), (o))
#define ERRORFL(m, s)  message(ERROR, (m), -1, -1)
#define INFO(m, l, o)  message(INFO, (m), (l), (o))
#define INFOFL(m, s)   message(INFO, (m), -1, -1)

/********************************************************************************
 * Private types
 ********************************************************************************/

enum class_e
{
  INFO,
  WARN,
  ERROR,
  FATAL
};

const char *class_text[] =
{
  "info",
  "warning",
  "error",
  "fatal"
};

enum file_e
{
  UNKNOWN  = 0x00,
  C_HEADER = 0x01,
  C_SOURCE = 0x02
};

enum section_s
{
  NO_SECTION = 0,
  INCLUDED_FILES,
  PRE_PROCESSOR_DEFINITIONS,
  PUBLIC_TYPES,
  PRIVATE_TYPES,
  PRIVATE_DATA,
  PUBLIC_DATA,
  PRIVATE_FUNCTIONS,
  PRIVATE_FUNCTION_PROTOTYPES,
  INLINE_FUNCTIONS,
  PUBLIC_FUNCTIONS,
  PUBLIC_FUNCTION_PROTOTYPES
};

enum pptype_e
{
  PPLINE_NONE = 0,
  PPLINE_DEFINE,
  PPLINE_IF,
  PPLINE_ELIF,
  PPLINE_ELSE,
  PPLINE_ENDIF,
  PPLINE_OTHER
};

struct file_section_s
{
  const char *name;   /* File section name */
  uint8_t     ftype;  /* File type where section found */
};

/********************************************************************************
 * Private data
 ********************************************************************************/

static enum file_e g_file_type  = UNKNOWN;
static enum section_s g_section = NO_SECTION;
static int g_maxline            = DEFAULT_WIDTH;
static int g_status             = 0;
static int g_verbose            = 2;
static int g_rangenumber        = 0;
static int g_rangestart[RANGE_NUMBER];
static int g_rangecount[RANGE_NUMBER];
static char g_file_name[PATH_MAX];

static const struct file_section_s g_section_info[] =
{
  {
    " *\n",                             /* Index: NO_SECTION */
    C_SOURCE | C_HEADER
  },
  {
    " * Included Files\n",              /* Index: INCLUDED_FILES */
    C_SOURCE | C_HEADER
  },
  {
    " * Pre-processor Definitions\n",   /* Index: PRE_PROCESSOR_DEFINITIONS */
    C_SOURCE | C_HEADER
  },
  {
    " * Public Types\n",                /* Index: PUBLIC_TYPES */
    C_HEADER
  },
  {
    " * Private Types\n",               /* Index: PRIVATE_TYPES */
    C_SOURCE
  },
  {
    " * Private Data\n",                /* Index: PRIVATE_DATA */
    C_SOURCE
  },
  {
    " * Public Data\n",                 /* Index: PUBLIC_DATA */
    C_SOURCE | C_HEADER
  },
  {
    " * Private Functions\n",           /* Index: PRIVATE_FUNCTIONS */
    C_SOURCE
  },
  {
    " * Private Function Prototypes\n", /* Index: PRIVATE_FUNCTION_PROTOTYPES */
    C_SOURCE
  },
  {
    " * Inline Functions\n",            /* Index: INLINE_FUNCTIONS */
    C_SOURCE | C_HEADER
  },
  {
    " * Public Functions\n",            /* Index: PUBLIC_FUNCTIONS */
    C_SOURCE
  },
  {
    " * Public Function Prototypes\n",  /* Index: PUBLIC_FUNCTION_PROTOTYPES */
    C_SOURCE | C_HEADER
  }
};

static const char *g_white_prefix[] =
{
  "ASCII_",  /* Ref:  include/nuttx/ascii.h */
  "Elf",     /* Ref:  include/elf.h, include/elf32.h, include/elf64.h */
  "PRId",    /* Ref:  inttypes.h */
  "PRIi",    /* Ref:  inttypes.h */
  "PRIo",    /* Ref:  inttypes.h */
  "PRIu",    /* Ref:  inttypes.h */
  "PRIx",    /* Ref:  inttypes.h */
  "SCNd",    /* Ref:  inttypes.h */
  "SCNi",    /* Ref:  inttypes.h */
  "SCNo",    /* Ref:  inttypes.h */
  "SCNu",    /* Ref:  inttypes.h */
  "SCNx",    /* Ref:  inttypes.h */
  "SYS_",    /* Ref:  include/sys/syscall.h */
  "STUB_",   /* Ref:  syscall/syscall_lookup.h, syscall/sycall_stublookup.c */
  "XK_",     /* Ref:  include/input/X11_keysymdef.h */
  "b8",      /* Ref:  include/fixedmath.h */
  "b16",     /* Ref:  include/fixedmath.h */
  "b32",     /* Ref:  include/fixedmath.h */
  "ub8",     /* Ref:  include/fixedmath.h */
  "ub16",    /* Ref:  include/fixedmath.h */
  "ub32",    /* Ref:  include/fixedmath.h */
  "lua_",    /* Ref:  apps/interpreters/lua/lua-5.x.x/src/lua.h */
  "luaL_",   /* Ref:  apps/interpreters/lua/lua-5.x.x/src/lauxlib.h */

  NULL
};

static const char *g_white_suffix[] =
{
  /* Ref:  include/nuttx/wireless/nrf24l01.h */

  "Mbps",
  "kHz",
  "kbps",
  "us",
  NULL
};

static const char *g_white_list[] =
{
  /* Ref:  gnu_unwind_find_exidx.c */

  "__EIT_entry",

  /* Ref:  gnu_unwind_find_exidx.c */

  "__gnu_Unwind_Find_exidx",

  /* Ref:  lib_impure.c */

  "__sFILE_fake",

  /* Ref:  stdlib.h */

  "_Exit",

  /* Ref:  stdatomic.h */

  "_Atomic",

  /* Ref:  unwind-arm-common.h */

  "_Unwind",

  /* Ref:
   * https://pubs.opengroup.org/onlinepubs/9699919799/functions/tempnam.html
   */

  "P_tmpdir",

  /* Ref:
   * https://pubs.opengroup.org/onlinepubs/9699919799/functions/tempnam.html
   */

  "L_tmpnam",

  /* Ref:
   * nuttx/compiler.h
   */

  "_Far",
  "_Erom",

  /* Ref:
   * arch/sim/src/sim/up_wpcap.c
   */

  "Address",
  "Description",
  "FirstUnicastAddress",
  "GetAdaptersAddresses",
  "GetProcAddress",
  "LoadLibrary",
  "lpSockaddr",
  "Next",
  "PhysicalAddressLength",
  "PhysicalAddress",
  "WideCharToMultiByte",

  /* Ref:
   * drivers/segger/note_sysview.c
   */

  "SEGGER_SYSVIEW",
  "TaskID",
  "sName",
  "Prio",
  "StackBase",
  "StackSize",

  /* Ref:
   * drivers/segger/syslog_rtt.c
   */

  "SEGGER_RTT",

  /* Ref:
   * fs/nfs/rpc.h
   * fs/nfs/nfs_proto.h
   * fs/nfs/nfs_mount.h
   * fs/nfs/nfs_vfsops.c
   */

  "CREATE3args",
  "CREATE3resok",
  "LOOKUP3args",
  "LOOKUP3filename",
  "LOOKUP3resok",
  "WRITE3args",
  "WRITE3resok",
  "READ3args",
  "READ3resok",
  "REMOVE3args",
  "REMOVE3resok",
  "RENAME3args",
  "RENAME3resok",
  "MKDIR3args",
  "MKDIR3resok",
  "RMDIR3args",
  "RMDIR3resok",
  "READDIR3args",
  "READDIR3resok",
  "SETATTR3args",
  "SETATTR3resok",
  "FS3args",
  "SIZEOF_rpc_reply_read",
  "SIZEOF_rpc_call_write",
  "SIZEOF_rpc_reply_readdir",
  "SIZEOF_nfsmount",

  /* Ref:
   * mm/kasan/kasan.c
   */

  "__asan_loadN",
  "__asan_storeN",
  "__asan_loadN_noabort",
  "__asan_storeN_noabort",

  /* Ref:
   * tools/jlink-nuttx.c
   */

  "RTOS_Init",
  "RTOS_GetVersion",
  "RTOS_GetSymbols",
  "RTOS_GetNumThreads",
  "RTOS_GetCurrentThreadId",
  "RTOS_GetThreadId",
  "RTOS_GetThreadDisplay",
  "RTOS_GetThreadReg",
  "RTOS_GetThreadRegList",
  "RTOS_GetThreadRegList",
  "RTOS_SetThreadReg",
  "RTOS_SetThreadRegList",
  "RTOS_UpdateThreads",

  /* Ref:
   * sim/posix/sim_x11eventloop.c
   */

  "Display",
  "Button1Mask",
  "Button2Mask",
  "Button3Mask",
  "Button1",
  "Button2",
  "Button3",
  "XEvent",
  "XPending",
  "XNextEvent",
  "KeyPress",
  "KeyRelease",
  "MotionNotify",
  "ButtonPress",
  "ButtonRelease",
  "XLookupKeysym",

  /* Ref:
   * sim/posix/sim_x11framebuffer.c
   */

  "Window",
  "XShmSegmentInfo",
  "XImage",
  "XGCValues",
  "XTextProperty",
  "XSizeHints",
  "XOpenDisplay",
  "XCreateSimpleWindow",
  "DefaultRootWindow",
  "XStringListToTextProperty",
  "XSetWMProperties",
  "XMapWindow",
  "XSelectInput",
  "XAllowEvents",
  "XGrabButton",
  "XCreateGC",
  "XSetErrorHandler",
  "XSync",
  "XShmDetach",
  "XDestroyImage",
  "XUngrabButton",
  "XCloseDisplay",
  "XShmQueryExtension",
  "XShmCreateImage",
  "XShmAttach",
  "DefaultVisual",
  "XCreateImage",
  "XGetWindowAttributes",
  "DefaultColormap",
  "XAllocColor",
  "XShmPutImage",
  "XPutImage",
  "Colormap",
  "DefaultScreen",
  "BlackPixel",
  "PSize",
  "PMinSize",
  "PMaxSize",
  "ButtonPressMask",
  "ButtonReleaseMask",
  "PointerMotionMask",
  "KeyPressMask",
  "KeyReleaseMask",
  "ButtonMotionMask",
  "GrabModeAsync",
  "GCGraphicsExposures",
  "XErrorEvent",
  "AnyModifier",
  "None",
  "Status",
  "DoGreen",
  "DoRed",
  "DoBlue",
  "ZPixmap",
  "readOnly",
  "XWindowAttributes",
  "XColor",
  "AsyncBoth",
  "CurrentTime",

  /* Ref:
   * sim/posix/sim_deviceimage.c
   */

  "inflateInit",
  "inflateEnd",
  "Bytef",

  /* Ref:
   * sim/posix/sim_hostmemory.c
   */

  "CreateFileMapping",
  "MapViewOfFile",
  "CloseHandle",
  "UnmapViewOfFile",

  /* Ref:
   * sim/posix/sim_hostmisc.c
   */

  "CaptureStackBackTrace",

  /* Ref:
   * sim/posix/sim_hosttime.c
   */

  "GetSystemTimeAsFileTime",
  "QueryPerformanceFrequency",
  "QueryPerformanceCounter",
  "CreateWaitableTimer",
  "SetWaitableTimer",
  "WaitForSingleObject",
  "dwHighDateTime",
  "dwLowDateTime",
  "QuadPart",

  /* Ref:
   * sim/posix/sim_hostuart.c
   */

  "GetStdHandle",
  "GetConsoleMode",
  "SetConsoleMode",
  "WriteConsole",
  "ReadConsole",
  "FlushConsoleInputBuffer",
  "GetNumberOfConsoleInputEvents",

  /* Ref:
   * apps/testing/drivertest/drivertest_xxx.c
   */

  "CMUnitTest",

  /* Ref:
   * apps/examples/hello_nim/hello_nim_main.c
   */

  "NimMain",

  /* Ref:
   * sim/posix/sim_rawgadget.c
   */

  "bRequestType",
  "bRequest",
  "wValue",
  "wIndex",
  "wLength",
  "bLength",
  "bDescriptorType",
  "bEndpointAddress",
  "bmAttributes",
  "wMaxPacketSize",
  "bInterval",

  /* Ref:
   * sim/posix/sim_libusb.c
   */

  "bNumConfigurations",
  "bDeviceClass",
  "idVendor",
  "idProduct",

  NULL
};

/********************************************************************************
 * Private Functions
 ********************************************************************************/

/********************************************************************************
 * Name: show_usage
 *
 * Description:
 *
 ********************************************************************************/

static void show_usage(char *progname, int exitcode, char *what)
{
  fprintf(stderr, "%s version %s\n\n", basename(progname), NXSTYLE_VERSION);
  if (what)
    {
      fprintf(stderr, "%s\n", what);
    }

  fprintf(stderr, "Usage:  %s [-m <excess>] [-v <level>] "
                  "[-r <start,count>] <filename>\n",
          basename(progname));
  fprintf(stderr, "        %s -h this help\n", basename(progname));
  fprintf(stderr, "        %s -v <level> where level is\n",
          basename(progname));
  fprintf(stderr, "                   0 - no output\n");
  fprintf(stderr, "                   1 - PASS/FAIL\n");
  fprintf(stderr, "                   2 - output each line (default)\n");
  exit(exitcode);
}

/********************************************************************************
 * Name: skip
 *
 * Description:
 *
 ********************************************************************************/

static int skip(int lineno)
{
  int i;

  for (i = 0; i < g_rangenumber; i++)
    {
      if (lineno >= g_rangestart[i] && lineno < g_rangestart[i] +
          g_rangecount[i])
        {
          return 0;
        }
    }

  return g_rangenumber != 0;
}

/********************************************************************************
 * Name: message
 *
 * Description:
 *
 ********************************************************************************/

static int message(enum class_e class, const char *text, int lineno, int ndx)
{
  FILE *out = stdout;

  if (skip(lineno))
    {
      return g_status;
    }

  if (class > INFO)
    {
      out = stderr;
      g_status |= 1;
    }

  if (g_verbose == 2)
    {
      if (lineno == -1 && ndx == -1)
        {
          fprintf(out, "%s: %s: %s\n", g_file_name, class_text[class], text);
        }
      else
        {
          fprintf(out, "%s:%d:%d: %s: %s\n", g_file_name, lineno, ndx,
                  class_text[class], text);
        }
    }

  return g_status;
}

/********************************************************************************
 * Name: check_spaces_left
 *
 * Description:
 *
 ********************************************************************************/

static void check_spaces_left(char *line, int lineno, int ndx)
{
  /* Unary operator should generally be preceded by a space but make also
   * follow a left parenthesis at the beginning of a parenthetical list or
   * expression or follow a right parentheses in the case of a cast.
   */

  if (ndx-- > 0 && line[ndx] != ' ' && line[ndx] != '(' && line[ndx] != ')')
    {
       ERROR("Operator/assignment must be preceded with whitespace",
             lineno, ndx);
    }
}

/********************************************************************************
 * Name: check_spaces_leftright
 *
 * Description:
 *
 ********************************************************************************/

static void check_spaces_leftright(char *line, int lineno, int ndx1, int ndx2)
{
  if (ndx1 > 0 && line[ndx1 - 1] != ' ')
    {
       ERROR("Operator/assignment must be preceded with whitespace",
             lineno, ndx1);
    }

  if (line[ndx2 + 1] != '\0' && line[ndx2 + 1] != '\n' && line[ndx2 + 1] != ' ')
    {
       ERROR("Operator/assignment must be followed with whitespace",
             lineno, ndx2);
    }
}

/********************************************************************************
 * Name: check_nospaces_leftright
 *
 * Description:
 *   Check if there are whitespaces on the left of right. If there is, report
 *   an error.
 *
 ********************************************************************************/

static void check_nospaces_leftright(char *line, int lineno, int ndx1, int ndx2)
{
  if (ndx1 > 0 && line[ndx1 - 1] == ' ')
    {
      ERROR("There should be no spaces before the operator/assignment",
            lineno, ndx1);
    }

  if (line[ndx2 + 1] == ' ')
    {
      ERROR("There should be no spaces after the operator/assignment",
            lineno, ndx2);
    }
}

/********************************************************************************
 * Name: check_operand_leftright
 *
 * Description:
 *   Check if the operator is next to an operand. If not, report the error.
 *
 ********************************************************************************/

static void check_operand_leftright(char *line, int lineno, int ndx1, int ndx2)
{
  /* The cases below includes("xx" represents the operator):
   *   " xx " | " xx(end)" | " xx;" | " xx\n" | " xx)" | " xx]"  - (ndx1 > 0)
   *   "(xx " | "(xx(end)" | "(xx;" | "(xx\n" | "(xx)" | "(xx]"  - (ndx1 > 0)
   *   "[xx " | "[xx(end)" | "[xx;" | "[xx\n" | "[xx)" | "[xx]"  - (ndx1 > 0)
   *   "xx "  | "xx(end)"  | "xx;"  | "xx\n"  | "xx)"  | "xx]"   - (ndx1 = 0)
   * In these cases, the operators must be not next any operands, thus errors
   * are reported.
   */

  if (ndx1 > 0 && (line[ndx1 - 1] == ' ' || line[ndx1 - 1] == '(' ||
                   line[ndx1 - 1] == '[') &&
                  (line[ndx2 + 1] == ' ' || line[ndx2 + 1] == '\0' ||
                   line[ndx2 + 1] == ';' || line[ndx2 + 1] == '\n' ||
                   line[ndx2 + 1] == ')' || line[ndx2 + 1] == ']'))
    {
      ERROR("Operator must be next to an operand", lineno, ndx2);
    }
}

/********************************************************************************
 * Name: block_comment_width
 *
 * Description:
 *   Get the width of a block comment
 *
 ********************************************************************************/

static int block_comment_width(char *line)
{
  int b;
  int e;
  int n;

  /* Skip over any leading whitespace on the line */

  for (b = 0; isspace(line[b]); b++)
    {
    }

  /* Skip over any trailing whitespace at the end of the line */

  for (e = strlen(line) - 1; e >= 0 && isspace(line[e]); e--)
    {
    }

  /* Number of characters on the line */

  n = e - b + 1;
  if (n < 4)
    {
      return 0;
    }

  /* The first line of a block comment starts with "[slash]***" and ends with
   * "***"
   */

  if (strncmp(&line[b], "/***", 4) == 0 &&
      strncmp(&line[e - 2], "***", 3) == 0)
    {
      /* Return the the length of the line up to the final '*' */

      return e + 1;
    }

  /* The last line of a block begins with whitespace then "***" and ends
   * with "***[slash]"
   */

  if (strncmp(&line[b], "***", 3) == 0 &&
      strncmp(&line[e - 3], "***/", 4) == 0)
    {
      /* Return the the length of the line up to the final '*' */

      return e;
    }

  /* But there is also a special single line comment that begins with "[slash]* "
   * and ends with "***[slash]"
   */

  if (strncmp(&line[b], "/*", 2) == 0 &&
      strncmp(&line[e - 3], "***/", 4) == 0)
    {
      /* Return the the length of the line up to the final '*' */

      return e;
    }

  /* Return zero if the line is not the first or last line of a block
   * comment.
   */

  return 0;
}

/********************************************************************************
 * Name: get_line_width
 *
 * Description:
 *   Get the maximum line width by examining the width of the block comments.
 *
 ********************************************************************************/

static int get_line_width(FILE *instream)
{
  char line[LINE_SIZE]; /* The current line being examined */
  int max        = 0;
  int min        = INT_MAX;
  int lineno     = 0;
  int lineno_max = 0;
  int lineno_min = 0;
  int len;

  while (fgets(line, LINE_SIZE, instream))
    {
      lineno++;
      len = block_comment_width(line);
      if (len > 0)
        {
          if (len > max)
            {
              max = len;
              lineno_max = lineno;
            }

          if (len < min)
            {
              min = len;
              lineno_min = lineno;
            }
        }
    }

  if (max < min)
    {
      ERRORFL("No block comments found", g_file_name);
      return DEFAULT_WIDTH;
    }
  else if (max != min)
    {
      ERROR("Block comments have different lengths", lineno_max, max);
      ERROR("Block comments have different lengths", lineno_min, min);
      return DEFAULT_WIDTH;
    }

  return min;
}

/********************************************************************************
 * Name:  check_section_header
 *
 * Description:
 *   Check if the current line holds a section header
 *
 ********************************************************************************/

static bool check_section_header(const char *line, int lineno)
{
  int i;

  /* Search g_section_info[] to find a matching section header line */

  for (i = FIRST_SECTION; i <= LAST_SECTION; i++)
    {
      if (strcmp(line, g_section_info[i].name) == 0)
        {
          g_section = (enum section_s)i;

          /* Verify that this section is appropriate for this file type */

          if ((g_file_type & g_section_info[i].ftype) == 0)
            {
              ERROR("Invalid section for this file type", lineno, 3);
            }

          return true;
        }
    }

  return false;
}

/********************************************************************************
 * Name:  white_prefix
 *
 * Description:
 *   Return true if the identifier string begins with a white-listed prefix
 *
 ********************************************************************************/

static bool white_list(const char *ident, int lineno)
{
  const char **pptr;
  const char *str;
  size_t len2;
  size_t len;

  for (pptr = g_white_prefix;
       (str = *pptr) != NULL;
       pptr++)
    {
      len = strlen(str);
      if (strncmp(ident, str, len) == 0)
        {
          return true;
        }
    }

  len2 = strlen(ident);
  while (!isalnum(ident[len2 - 1]))
    {
      len2--;
    }

  for (pptr = g_white_suffix;
       (str = *pptr) != NULL;
       pptr++)
    {
      len = strlen(str);
      if (len2 >= len && strncmp(ident + len2 - len, str, len) == 0)
        {
          return true;
        }
    }

  for (pptr = g_white_list;
       (str = *pptr) != NULL;
       pptr++)
    {
      len = strlen(str);
      if (strncmp(ident, str, len) == 0 &&
          isalnum(ident[len]) == 0)
        {
          return true;
        }
    }

  return false;
}

/********************************************************************************
 * Public Functions
 ********************************************************************************/

int main(int argc, char **argv, char **envp)
{
  FILE *instream;       /* File input stream */
  char line[LINE_SIZE]; /* The current line being examined */
  char buffer[100];     /* Localy format error strings */
  char *lptr;           /* Temporary pointer into line[] */
  char *ext;            /* Temporary file extension */
  bool btabs;           /* True: TAB characters found on the line */
  bool bcrs;            /* True: Carriage return found on the line */
  bool bfunctions;      /* True: In private or public functions */
  bool bstatm;          /* True: This line is beginning of a statement */
  bool bfor;            /* True: This line is beginning of a 'for' statement */
  bool bswitch;         /* True: Within a switch statement */
  bool bstring;         /* True: Within a string */
  bool bquote;          /* True: Backslash quoted character next */
  bool bblank;          /* Used to verify block comment terminator */
  bool bexternc;        /* True: Within 'extern "C"' */
  enum pptype_e ppline; /* > 0: The next line the continuation of a
                         * pre-processor command */
  int rhcomment;        /* Indentation of Comment to the right of code
                         * (-1 -> don't check position) */
  int prevrhcmt;        /* Indentation of previous Comment to the right
                         * of code (-1 -> don't check position) */
  int lineno;           /* Current line number */
  int indent;           /* Indentation level */
  int ncomment;         /* Comment nesting level on this line */
  int prevncomment;     /* Comment nesting level on the previous line */
  int bnest;            /* Brace nesting level on this line */
  int prevbnest;        /* Brace nesting level on the previous line */
  int dnest;            /* Data declaration nesting level on this line */
  int prevdnest;        /* Data declaration nesting level on the previous line */
  int pnest;            /* Parenthesis nesting level on this line */
  int ppifnest;         /* #if nesting level on this line */
  int inasm;            /* > 0: Within #ifdef __ASSEMBLY__ */
  int comment_lineno;   /* Line on which the last comment was closed */
  int blank_lineno;     /* Line number of the last blank line */
  int noblank_lineno;   /* A blank line is not needed after this line */
  int lbrace_lineno;    /* Line number of last left brace */
  int rbrace_lineno;    /* Last line containing a right brace */
  int externc_lineno;   /* Last line where 'extern "C"' declared */
  int linelen;          /* Length of the line */
  int excess;
  int n;
  int i;
  int c;

  excess = 0;
  while ((c = getopt(argc, argv, ":hv:gm:r:")) != -1)
    {
      switch (c)
      {
      case 'm':
        excess = atoi(optarg);
        if (excess < 1)
          {
            show_usage(argv[0], 1, "Bad value for <excess>.");
            excess = 0;
          }

        break;

      case 'v':
        g_verbose = atoi(optarg);
        if (g_verbose < 0 || g_verbose > 2)
          {
            show_usage(argv[0], 1, "Bad value for <level>.");
          }

        break;

      case 'r':
        g_rangestart[g_rangenumber] = atoi(strtok(optarg, ","));
        g_rangecount[g_rangenumber++] = atoi(strtok(NULL, ","));
        break;

      case 'h':
        show_usage(argv[0], 0, NULL);
        break;

      case ':':
        show_usage(argv[0], 1, "Missing argument.");
        break;

      case '?':
        show_usage(argv[0], 1, "Unrecognized option.");
        break;

      default:
        show_usage(argv[0], 0, NULL);
        break;
      }
  }

  if (optind < argc - 1 || argv[optind] == NULL)
    {
      show_usage(argv[0], 1, "No file name given.");
    }

  /* Resolve the absolute path for the input file */

  if (realpath(argv[optind], g_file_name) == NULL)
    {
      FATALFL("Failed to resolve absolute path.", g_file_name);
      return 1;
    }

  /* Are we parsing a header file? */

  ext = strrchr(g_file_name, '.');

  if (ext == 0)
    {
    }
  else if (strcmp(ext, ".h") == 0)
    {
      g_file_type = C_HEADER;
    }
  else if (strcmp(ext, ".c") == 0)
    {
      g_file_type = C_SOURCE;
    }

  if (g_file_type == UNKNOWN)
    {
      return 0;
    }

  instream = fopen(g_file_name, "r");

  if (!instream)
    {
      FATALFL("Failed to open", g_file_name);
      return 1;
    }

  /* Determine the line width */

  g_maxline = get_line_width(instream) + excess;
  rewind(instream);

  btabs          = false;       /* True: TAB characters found on the line */
  bcrs           = false;       /* True: Carriage return found on the line */
  bfunctions     = false;       /* True: In private or public functions */
  bswitch        = false;       /* True: Within a switch statement */
  bstring        = false;       /* True: Within a string */
  bexternc       = false;       /* True: Within 'extern "C"' */
  ppline         = PPLINE_NONE; /* > 0: The next line the continuation of a
                                 * pre-processor command */
  rhcomment      = 0;           /* Indentation of Comment to the right of code
                                 * (-1 -> don't check position) */
  prevrhcmt      = 0;           /* Indentation of previous Comment to the right
                                 * of code (-1 -> don't check position) */
  lineno         = 0;           /* Current line number */
  ncomment       = 0;           /* Comment nesting level on this line */
  bnest          = 0;           /* Brace nesting level on this line */
  dnest          = 0;           /* Data declaration nesting level on this line */
  pnest          = 0;           /* Parenthesis nesting level on this line */
  ppifnest       = 0;           /* #if nesting level on this line */
  inasm          = 0;           /* > 0: Within #ifdef __ASSEMBLY__ */
  comment_lineno = -1;          /* Line on which the last comment was closed */
  blank_lineno   = -1;          /* Line number of the last blank line */
  noblank_lineno = -1;          /* A blank line is not needed after this line */
  lbrace_lineno  = -1;          /* Line number of last left brace */
  rbrace_lineno  = -1;          /* Last line containing a right brace */
  externc_lineno = -1;          /* Last line where 'extern "C"' declared */

  /* Process each line in the input stream */

  while (fgets(line, LINE_SIZE, instream))
    {
      lineno++;
      indent       = 0;
      prevbnest    = bnest;    /* Brace nesting level on the previous line */
      prevdnest    = dnest;    /* Data declaration nesting level on the
                                * previous line */
      prevncomment = ncomment; /* Comment nesting level on the previous line */
      bstatm       = false;    /* True: This line is beginning of a
                                * statement */
      bfor         = false;    /* REVISIT: Implies for() is all on one line */

      /* If we are not in a comment, then this certainly is not a right-hand
       * comment.
       */

      prevrhcmt = rhcomment;
      if (ncomment <= 0)
        {
          rhcomment = 0;
        }

      /* Check for a blank line */

      for (n = 0; line[n] != '\n' && isspace((int)line[n]); n++)
        {
        }

      if (line[n] == '\n')
        {
          if (n > 0)
            {
              ERROR("Blank line contains whitespace", lineno, 1);
            }

          if (lineno == 1)
            {
              ERROR("File begins with a blank line", 1, 1);
            }
          else if (lineno == blank_lineno + 1)
            {
              ERROR("Too many blank lines", lineno, 1);
            }
          else if (lineno == lbrace_lineno + 1)
            {
              ERROR("Blank line follows left brace", lineno, 1);
            }

          blank_lineno = lineno;
          continue;
        }
      else /* This line is non-blank */
        {
          /* Check for a missing blank line after a comment */

          if (lineno == comment_lineno + 1)
            {
              /* No blank line should be present if the current line contains
               * a right brace, a pre-processor line, the start of another
               * comment.
               *
               * REVISIT: Generates a false alarm if the current line is also
               * a comment.  Generally it is acceptable for one comment to
               * follow another with no space separation.
               *
               * REVISIT: prevrhcmt is tested to case the preceding line
               * contained comments to the right of the code.  In such cases,
               * the comments are normally aligned and do not follow normal
               * indentation rules.  However, this code will generate a false
               * alarm if the comments are aligned to the right BUT the
               * preceding line has no comment.
               */

              if (line[n] != '}' && line[n] != '#' && prevrhcmt == 0)
                {
                   ERROR("Missing blank line after comment", comment_lineno,
                         1);
                }
            }

          /* Files must begin with a comment (the file header).
           * REVISIT:  Logically, this belongs in the STEP 2 operations
           * below.
           */

          if (lineno == 1 && (line[n] != '/' || line[n + 1] != '*'))
            {
               ERROR("Missing file header comment block", lineno, 1);
            }

          if (lineno == 2)
            {
              if (line[n] == '*' && line[n + 1] == '\n')
                {
                  ERROR("Missing relative file path in file header", lineno,
                        n);
                }
              else if (isspace(line[n + 2]))
                {
                  ERROR("Too many whitespaces before relative file path",
                        lineno, n);
                }
              else
                {
                  const char *apps_dir = "apps/";
                  const size_t apps_len = strlen(apps_dir);
                  size_t offset;

#ifdef TOPDIR
                  /* TOPDIR macro contains the absolute path to the "nuttx"
                   * root directory. It should have been defined via Makefile
                   * and it is required to accurately evaluate the relative
                   * path contained in the file header. Otherwise, skip this
                   * verification.
                   */

                  char *basedir = strstr(g_file_name, TOPDIR);
                  if (basedir != NULL)
                    {
                      /* Add 1 to the offset for the slash character */

                      offset = strlen(TOPDIR) + 1;

                      /* Duplicate the line from the beginning of the
                       * relative file path, removing the '\n' at the end of
                       * the string.
                       */

                      char *line_dup = strndup(&line[n + 2],
                                               strlen(&line[n + 2]) - 1);

                      if (strcmp(line_dup, basedir + offset) != 0)
                        {
                          ERROR("Relative file path does not match actual file",
                                lineno, n);
                        }

                      free(line_dup);
                    }
                  else if (strncmp(&line[n + 2], apps_dir, apps_len) != 0)
                    {
                      /* g_file_name neither belongs to "nuttx" repository
                       * nor begins with the root dir of the other
                       * repository (e.g. "apps/")
                       */

                      ERROR("Path relative to repository other than \"nuttx\" "
                            "must begin with the root directory", lineno, n);
                    }
                  else
                    {
#endif

                      offset = 0;

                      if (strncmp(&line[n + 2], apps_dir, apps_len) == 0)
                        {
                          /* Input file belongs to the "apps" repository */

                          /* Calculate the offset to the first directory
                           * after the "apps/" folder.
                           */

                          offset += apps_len;
                        }

                      /* Duplicate the line from the beginning of the
                       * relative file path, removing the '\n' at the end of
                       * the string.
                       */

                      char *line_dup = strndup(&line[n + 2],
                                              strlen(&line[n + 2]) - 1);

                      ssize_t base =
                        strlen(g_file_name) - strlen(&line_dup[offset]);

                      if (base < 0 ||
                          (base != 0 && g_file_name[base - 1] != '/') ||
                          strcmp(&g_file_name[base], &line_dup[offset]) != 0)
                        {
                          ERROR("Relative file path does not match actual file",
                                lineno, n);
                        }

                      free(line_dup);
#ifdef TOPDIR
                    }
#endif
                }
            }

          /* Check for a blank line following a right brace */

          if (bfunctions && lineno == rbrace_lineno + 1)
            {
              /* Check if this line contains a right brace.  A right brace
               * must be followed by 'else', 'while', 'break', a blank line,
               * another right brace, or a pre-processor directive like #endif
               */

              if (dnest == 0 &&
                  strchr(line, '}') == NULL && line[n] != '#' &&
                  strncmp(&line[n], "else", 4) != 0 &&
                  strncmp(&line[n], "while", 5) != 0 &&
                  strncmp(&line[n], "break", 5) != 0)
                {
                   ERROR("Right brace must be followed by a blank line",
                         rbrace_lineno, n + 1);
                }

              /* If the right brace is followed by a pre-processor command
               * like #endif (but not #else or #elif), then set the right
               * brace line number to the line number of the pre-processor
               * command (it then must be followed by a blank line)
               */

              if (line[n] == '#')
                {
                  int ii;

                  for (ii = n + 1; line[ii] != '\0' && isspace(line[ii]); ii++)
                    {
                    }

                  if (strncmp(&line[ii], "else", 4) != 0 &&
                      strncmp(&line[ii], "elif", 4) != 0)
                    {
                      rbrace_lineno = lineno;
                    }
                }
            }
        }

      /* STEP 1: Find the indentation level and the start of real stuff on
       * the line.
       */

      for (n = 0; line[n] != '\n' && isspace((int)line[n]); n++)
        {
          switch (line[n])
            {
            case ' ':
              {
                indent++;
              }
              break;

            case '\t':
              {
                if (!btabs)
                  {
                    ERROR("TABs found.  First detected", lineno, n);
                    btabs = true;
                  }

                indent = (indent + 4) & ~3;
              }
              break;

            case '\r':
              {
                if (!bcrs)
                  {
                    ERROR("Carriage returns found.  "
                            "First detected", lineno, n);
                    bcrs = true;
                  }
              }
              break;

            default:
              {
                 snprintf(buffer, sizeof(buffer),
                          "Unexpected white space character %02x found",
                          line[n]);
                 ERROR(buffer, lineno, n);
              }
              break;
            }
        }

      /* STEP 2: Detect some certain start of line conditions */

      /* Skip over pre-processor lines (or continuations of pre-processor
       * lines as indicated by ppline)
       */

      if (line[indent] == '#' || ppline != PPLINE_NONE)
        {
          int len;
          int ii;

          /* Suppress error for comment following conditional compilation */

          noblank_lineno = lineno;

          /* Check pre-processor commands if this is not a continuation
           * line.
           */

          ii = indent + 1;

          if (ppline == PPLINE_NONE)
            {
              /* Skip to the pre-processor command following the '#' */

              while (line[ii] != '\0' && isspace(line[ii]))
                {
                  ii++;
                }

              if (line[ii] != '\0')
                {
                  /* Make sure that pre-processor definitions are all in
                  * the pre-processor definitions section.
                  */

                  ppline = PPLINE_OTHER;

                  if (strncmp(&line[ii], "define", 6) == 0)
                    {
                      ppline = PPLINE_DEFINE;

                      if (g_section != PRE_PROCESSOR_DEFINITIONS)
                        {
                          /* A complication is the header files always have
                           * the idempotence guard definitions before the
                           * "Pre-processor Definitions section".
                           */

                          if (g_section == NO_SECTION &&
                              g_file_type != C_HEADER)
                            {
                              /* Only a warning because there is some usage
                               * of define outside the Pre-processor
                               * Definitions section which is justifiable.
                               * Should be manually checked.
                               */

                              WARN("#define outside of 'Pre-processor "
                                   "Definitions' section",
                                   lineno, ii);
                            }
                        }
                    }

                  /* Make sure that files are included only in the Included
                   * Files section.
                   */

                  else if (strncmp(&line[ii], "include", 7) == 0)
                    {
                      if (g_section != INCLUDED_FILES)
                        {
                          /* Only a warning because there is some usage of
                           * include outside the Included Files section
                           * which may be is justifiable.  Should be
                           * manually checked.
                           */

                          WARN("#include outside of 'Included Files' "
                               "section",
                               lineno, ii);
                        }
                    }
                  else if (strncmp(&line[ii], "if", 2) == 0)
                    {
                      ppifnest++;

                      ppline = PPLINE_IF;
                      ii += 2;
                    }
                  else if (strncmp(&line[ii], "elif", 4) == 0)
                    {
                      if (ppifnest == inasm)
                        {
                          inasm = 0;
                        }

                      ppline = PPLINE_ELIF;
                      ii += 4;
                    }
                  else if (strncmp(&line[ii], "else", 4) == 0)
                    {
                      if (ppifnest == inasm)
                        {
                          inasm = 0;
                        }

                      ppline = PPLINE_ELSE;
                    }
                  else if (strncmp(&line[ii], "endif", 4) == 0)
                    {
                      if (ppifnest == inasm)
                        {
                          inasm = 0;
                        }

                      ppifnest--;

                      ppline = PPLINE_ENDIF;
                    }
               }
            }

          if (ppline == PPLINE_IF || ppline == PPLINE_ELIF)
            {
              int bdef = 0;

              if (strncmp(&line[ii], "def", 3) == 0)
                {
                  bdef = 1;
                  ii += 3;
                }
              else
                {
                  while (line[ii] != '\0' && isspace(line[ii]))
                    {
                      ii++;
                    }

                  if (strncmp(&line[ii], "defined", 7) == 0)
                    {
                      bdef = 1;
                      ii += 7;
                    }
                }

              if (bdef)
                {
                  while (line[ii] != '\0' &&
                      (isspace(line[ii]) || line[ii] == '('))
                    {
                      ii++;
                    }

                  if (strncmp(&line[ii], "__ASSEMBLY__", 12) == 0)
                    {
                      inasm = ppifnest;
                    }
                }
            }

          /* Check if the next line will be a continuation of the pre-
           * processor command.
           */

          len = strlen(&line[indent]) + indent - 1;
          if (line[len] == '\n')
            {
              len--;
            }

          /* Propagate rhcomment over preprocessor lines Issue #120 */

          if (prevrhcmt != 0)
            {
              /* Don't check position */

              rhcomment = -1;
            }

          lptr = strstr(line, "/*");
          if (lptr != NULL)
            {
              n = lptr - &line[0];
              if (line[n + 2] == '\n')
                {
                  ERROR("C comment opening on separate line", lineno, n);
                }
              else if (!isspace((int)line[n + 2]) && line[n + 2] != '*')
                {
                   ERROR("Missing space after opening C comment", lineno, n);
                }

              if (strstr(lptr, "*/") == NULL)
                {
                  /* Increment the count of nested comments */

                  ncomment++;
                }

              if (ppline == PPLINE_DEFINE)
                {
                  rhcomment = n;
                  if (prevrhcmt > 0 && n != prevrhcmt)
                    {
                      rhcomment = prevrhcmt;
                      WARN("Wrong column position of comment right of code",
                          lineno, n);
                    }
                }
              else
                {
                  /* Signal rhcomment, but ignore position */

                  rhcomment = -1;

                  if (ncomment > 0 &&
                      (ppline == PPLINE_IF ||
                       ppline == PPLINE_ELSE ||
                       ppline == PPLINE_ELIF))
                    {
                      /* in #if...  and #el... */

                      ERROR("No multiline comment right of code allowed here",
                          lineno, n);
                    }
                }
            }

          if (line[len] != '\\' || ncomment > 0)
            {
              ppline = PPLINE_NONE;
            }

          continue;
        }

      /* Check for a single line comment */

      linelen = strlen(line);
      if (linelen >= 5)      /* Minimum is slash, star, star, slash, newline */
        {
          lptr = strstr(line, "*/");
          if (line[indent] == '/' && line[indent + 1] == '*' &&
              lptr - line == linelen - 3)
            {
              /* If preceding comments were to the right of code, then we can
               * assume that there is a columnar alignment of columns that do
               * no follow the usual alignment.  So the rhcomment flag
               * should propagate.
               */

              rhcomment = prevrhcmt;

              /* Check if there should be a blank line before the comment */

              if (lineno > 1 &&
                  comment_lineno != lineno - 1 &&
                  blank_lineno   != lineno - 1 &&
                  noblank_lineno != lineno - 1 &&
                  rhcomment == 0)
                {
                  /* TODO:  This generates a false alarm if preceded
                   * by a label.
                   */

                   ERROR("Missing blank line before comment found", lineno, 1);
                }

              /* 'comment_lineno 'holds the line number of the last closing
               * comment.  It is used only to verify that the comment is
               * followed by a blank line.
               */

              comment_lineno = lineno;
            }
        }

      /* Check for the comment block indicating the beginning of a new file
       * section.
       */

      if (check_section_header(line, lineno))
        {
          if (g_section == PRIVATE_FUNCTIONS || g_section == PUBLIC_FUNCTIONS)
            {
              bfunctions = true;  /* Latched */
            }
        }

      /* Check for some kind of declaration.
       * REVISIT: The following logic fails for any non-standard types.
       * REVISIT: Terminator after keyword might not be a space.  Might be
       * a newline, for example.  struct and unions are often unnamed, for
       * example.
       */

      else if (inasm == 0)
        {
          if (strncmp(&line[indent], "auto ", 5) == 0 ||
                   strncmp(&line[indent], "bool ", 5) == 0 ||
                   strncmp(&line[indent], "char ", 5) == 0 ||
                   strncmp(&line[indent], "CODE ", 5) == 0 ||
                   strncmp(&line[indent], "const ", 6) == 0 ||
                   strncmp(&line[indent], "double ", 7) == 0 ||
                   strncmp(&line[indent], "struct ", 7) == 0 ||
                   strncmp(&line[indent], "struct\n", 7) == 0 || /* May be unnamed */
                   strncmp(&line[indent], "enum ", 5) == 0 ||
                   strncmp(&line[indent], "extern ", 7) == 0 ||
                   strncmp(&line[indent], "EXTERN ", 7) == 0 ||
                   strncmp(&line[indent], "FAR ", 4) == 0 ||
                   strncmp(&line[indent], "float ", 6) == 0 ||
                   strncmp(&line[indent], "int ", 4) == 0 ||
                   strncmp(&line[indent], "int16_t ", 8) == 0 ||
                   strncmp(&line[indent], "int32_t ", 8) == 0 ||
                   strncmp(&line[indent], "long ", 5) == 0 ||
                   strncmp(&line[indent], "off_t ", 6) == 0 ||
                   strncmp(&line[indent], "register ", 9) == 0 ||
                   strncmp(&line[indent], "short ", 6) == 0 ||
                   strncmp(&line[indent], "signed ", 7) == 0 ||
                   strncmp(&line[indent], "size_t ", 7) == 0 ||
                   strncmp(&line[indent], "ssize_t ", 8) == 0 ||
                   strncmp(&line[indent], "static ", 7) == 0 ||
                   strncmp(&line[indent], "time_t ", 7) == 0 ||
                   strncmp(&line[indent], "typedef ", 8) == 0 ||
                   strncmp(&line[indent], "uint8_t ", 8) == 0 ||
                   strncmp(&line[indent], "uint16_t ", 9) == 0 ||
                   strncmp(&line[indent], "uint32_t ", 9) == 0 ||
                   strncmp(&line[indent], "union ", 6) == 0 ||
                   strncmp(&line[indent], "union\n", 6) == 0 ||  /* May be unnamed */
                   strncmp(&line[indent], "unsigned ", 9) == 0 ||
                   strncmp(&line[indent], "void ", 5) == 0 ||
                   strncmp(&line[indent], "volatile ", 9) == 0)
            {
              /* Check if this is extern "C";  We don't typically indent
               * following this.
               */

              if (strncmp(&line[indent], "extern \"C\"", 10) == 0)
                {
                  externc_lineno = lineno;
                }

              /* bfunctions:  True:  Processing private or public functions.
               * bnest:       Brace nesting level on this line
               * dnest:       Data declaration nesting level on this line
               */

              /* REVISIT: Also picks up function return types */

              /* REVISIT: Logic problem for nested data/function declarations */

              if ((!bfunctions || bnest > 0) && dnest == 0)
                {
                  dnest = 1;
                }

              /* Check for multiple definitions of variables on the line.
               * Ignores declarations within parentheses which are probably
               * formal parameters.
               */

              if (pnest == 0)
                {
                  int tmppnest;

                  /* Note, we have not yet parsed each character on the line so
                   * a comma have have been be preceded by '(' on the same line.
                   * We will have parse up to any comma to see if that is the
                   * case.
                   */

                  for (i = indent, tmppnest = 0;
                       line[i] != '\n' && line[i] != '\0';
                       i++)
                    {
                      if (tmppnest == 0 && line[i] == ',')
                        {
                           ERROR("Multiple data definitions", lineno, i + 1);
                          break;
                        }
                      else if (line[i] == '(')
                        {
                          tmppnest++;
                        }
                      else if (line[i] == ')')
                        {
                          if (tmppnest < 1)
                            {
                              /* We should catch this later */

                              break;
                            }

                          tmppnest--;
                        }
                      else if (line[i] == ';')
                        {
                          /* Break out if the semicolon terminates the
                           * declaration is found.  Avoids processing any
                           * righthand comments in most cases.
                           */

                          break;
                        }
                    }
                }
            }

          /* Check for a keyword indicating the beginning of a statement.
           * REVISIT:  This, obviously, will not detect statements that do not
           * begin with a C keyword (such as assignment statements).
           */

          else if (strncmp(&line[indent], "break ", 6) == 0 ||
                   strncmp(&line[indent], "case ", 5) == 0 ||
    #if 0 /* Part of switch */
                   strncmp(&line[indent], "case ", 5) == 0 ||
    #endif
                   strncmp(&line[indent], "continue ", 9) == 0 ||

    #if 0 /* Part of switch */
                   strncmp(&line[indent], "default ", 8) == 0 ||
    #endif
                   strncmp(&line[indent], "do ", 3) == 0 ||
                   strncmp(&line[indent], "else ", 5) == 0 ||
                   strncmp(&line[indent], "goto ", 5) == 0 ||
                   strncmp(&line[indent], "if ", 3) == 0 ||
                   strncmp(&line[indent], "return ", 7) == 0 ||
    #if 0 /*  Doesn't follow pattern */
                   strncmp(&line[indent], "switch ", 7) == 0 ||
    #endif
                   strncmp(&line[indent], "while ", 6) == 0)
            {
              bstatm = true;
            }

          /* Spacing works a little differently for and switch statements */

          else if (strncmp(&line[indent], "for ", 4) == 0)
            {
              bfor   = true;
              bstatm = true;
            }
          else if (strncmp(&line[indent], "switch ", 7) == 0)
            {
              bswitch = true;
            }

          /* Also check for C keywords with missing white space */

          else if (strncmp(&line[indent], "do(", 3) == 0 ||
                   strncmp(&line[indent], "if(", 3) == 0 ||
                   strncmp(&line[indent], "while(", 6) == 0)
            {
              ERROR("Missing whitespace after keyword", lineno, n);
              bstatm = true;
            }
          else if (strncmp(&line[indent], "for(", 4) == 0)
            {
              ERROR("Missing whitespace after keyword", lineno, n);
              bfor   = true;
              bstatm = true;
            }
          else if (strncmp(&line[indent], "switch(", 7) == 0)
            {
              ERROR("Missing whitespace after keyword", lineno, n);
              bswitch = true;
            }
        }

      /* STEP 3: Parse each character on the line */

      bquote = false;   /* True: Backslash quoted character next */
      bblank = true;    /* Used to verify block comment terminator */

      for (; line[n] != '\n' && line[n] != '\0'; n++)
        {
          /* Report any use of non-standard white space characters */

          if (isspace(line[n]))
            {
              if (line[n] == '\t')
                {
                  if (!btabs)
                    {
                      ERROR("TABs found.  First detected", lineno, n);
                      btabs = true;
                    }
                }
              else if (line[n] == '\r')
                {
                  if (!bcrs)
                    {
                      ERROR("Carriage returns found.  "
                              "First detected", lineno, n);
                      bcrs = true;
                    }
                }
              else if (line[n] != ' ')
                {
                  snprintf(buffer, sizeof(buffer),
                           "Unexpected white space character %02x found",
                           line[n]);
                  ERROR(buffer, lineno, n);
                }
            }

          /* Skip over identifiers */

          if (ncomment == 0 && !bstring && (line[n] == '_' || isalpha(line[n])))
            {
              bool have_upper = false;
              bool have_lower = false;
              int ident_index = n;

              /* Parse over the identifier.  Check if it contains mixed upper-
               * and lower-case characters.
               */

              do
                {
                  have_upper |= isupper(line[n]);

                  /* The coding standard provides for some exceptions of lower
                   * case characters in pre-processor strings:
                   *
                   *   IPv[4|6]    as an IP version number
                   *   ICMPv6      as an ICMP version number
                   *   IGMPv2      as an IGMP version number
                   *   [0-9]p[0-9] as a decimal point
                   *   d[0-9]      as a divisor
                   *   Hz          for frequencies (including KHz, MHz, etc.)
                   */

                   if (!have_lower && islower(line[n]))
                     {
                       switch (line[n])
                       {
                         /* A sequence containing 'v' may occur at the
                          * beginning of the identifier.
                          */

                         case 'v':
                           if (n > 1 &&
                               line[n - 2] == 'I' &&
                               line[n - 1] == 'P' &&
                               (line[n + 1] == '4' ||
                                line[n + 1] == '6'))
                             {
                             }
                           else if (n > 3 &&
                                    line[n - 4] == 'I' &&
                                    line[n - 3] == 'C' &&
                                    line[n - 2] == 'M' &&
                                    line[n - 1] == 'P' &&
                                    line[n + 1] == '6')
                             {
                             }
                           else if (n > 3 &&
                                    line[n - 4] == 'I' &&
                                    line[n - 3] == 'G' &&
                                    line[n - 2] == 'M' &&
                                    line[n - 1] == 'P' &&
                                    line[n + 1] == '2')
                             {
                             }
                           else
                             {
                               have_lower = true;
                             }
                           break;

                         /* Sequences containing 'p', 'd', or 'z' must have
                          * been preceded by upper case characters.
                          */

                         case 'p':
                           if (!have_upper || n < 1 ||
                               !isdigit(line[n - 1]) ||
                               !isdigit(line[n + 1]))
                             {
                               have_lower = true;
                             }
                             break;

                         case 'd':
                           if (!have_upper || !isdigit(line[n + 1]))
                             {
                               have_lower = true;
                             }
                             break;

                         case 'z':
                           if (!have_upper || n < 1 ||
                               line[n - 1] != 'H')
                             {
                               have_lower = true;
                             }
                             break;
                           break;

                         default:
                           have_lower = true;
                           break;
                       }
                     }

                  n++;
                }
              while (line[n] == '_' || isalnum(line[n]));

              /* Check for mixed upper and lower case */

              if (have_upper && have_lower)
                {
                  /* Ignore symbols that begin with white-listed prefixes */

                  if (white_list(&line[ident_index], lineno))
                    {
                      /* No error */
                    }

                  /* Special case hex constants.  These will look like
                   * identifiers starting with 'x' or 'X' but preceded
                   * with '0'
                   */

                  else if (ident_index < 1 ||
                           (line[ident_index] != 'x' &&
                            line[ident_index] != 'X') ||
                           line[ident_index - 1] != '0')
                    {
                       ERROR("Mixed case identifier found",
                             lineno, ident_index);
                    }
                  else if (have_upper)
                    {
                       ERROR("Upper case hex constant found",
                             lineno, ident_index);
                    }
                }

              /* Check if the identifier is the last thing on the line */

              if (line[n] == '\n' || line[n] == '\0')
                {
                  break;
                }
            }

          /* Handle comments */

          if (line[n] == '/' && !bstring)
            {
              /* Check for start of a C comment */

              if (line[n + 1] == '*')
                {
                  if (line[n + 2] == '\n')
                    {
                      ERROR("C comment opening on separate line", lineno, n);
                    }
                  else if (!isspace((int)line[n + 2]) && line[n + 2] != '*')
                    {
                       ERROR("Missing space after opening C comment", lineno, n);
                    }

                  /* Increment the count of nested comments */

                  ncomment++;

                  /* If there is anything to the left of the left brace, then
                   * this must be a comment to the right of code.
                   * Also if preceding comments were to the right of code, then
                   * we can assume that there is a columnar alignment of columns
                   * that do no follow the usual alignment.  So the rhcomment
                   * flag should propagate.
                   */

                  if (prevrhcmt == 0)
                    {
                      if (n != indent)
                        {
                          rhcomment = n;
                        }
                    }
                  else
                    {
                      rhcomment = n;
                      if (prevrhcmt > 0 && n != prevrhcmt)
                        {
                          rhcomment = prevrhcmt;
                          if (n != indent)
                            {
                              WARN("Wrong column position of "
                                  "comment right of code", lineno, n);
                            }
                          else
                            {
                              ERROR("Wrong column position or missing "
                                  "blank line before comment", lineno, n);
                            }
                        }
                    }

                  n++;
                  continue;
                }

              /* Check for end of a C comment */

              else if (n > 0 && line[n - 1] == '*')
                {
                  if (n < 2)
                    {
                      ERROR("Closing C comment not indented", lineno, n);
                    }
                  else if (!isspace((int)line[n - 2]) && line[n - 2] != '*')
                    {
                       ERROR("Missing space before closing C comment", lineno,
                             n);
                    }

                  /* Check for block comments that are not on a separate line.
                   * This would be the case if we are we are within a comment
                   * that did not start on this line and the current line is
                   * not blank up to the point where the comment was closed.
                   */

                  if (prevncomment > 0 && !bblank && rhcomment == 0)
                    {
                       ERROR("Block comment terminator must be on a "
                              "separate line", lineno, n);
                    }

#if 0
                  /* REVISIT: Generates false alarms when portions of an
                   * expression are commented out within the expression.
                   */

                  if (line[n + 1] != '\n')
                    {
                       ERROR("Garbage on line after C comment", lineno, n);
                    }
#endif

                  /* Handle nested comments */

                  if (ncomment > 0)
                    {
                      /* Remember the line number of the line containing the
                       * closing of the outermost comment.
                       */

                      if (--ncomment == 0)
                        {
                          /* 'comment_lineno 'holds the line number of the
                           * last closing comment.  It is used only to
                           * verify that the comment is followed by a blank
                           * line.
                           */

                          comment_lineno = lineno;

                          /* Note that rhcomment must persist to support a
                           * later test for comment alignment.  We will fix
                           * that at the top of the loop when ncomment == 0.
                           */
                        }
                    }
                  else
                    {
                      /* Note that rhcomment must persist to support a later
                       * test for comment alignment.  We will will fix that
                       * at the top of the loop when ncomment == 0.
                       */

                      ncomment = 0;
                      ERROR("Closing without opening comment", lineno, n);
                    }

                  n++;
                  continue;
                }

              /* Check for C++ style comments */

              else if (line[n + 1] == '/')
                {
                  /* Check for URI schemes, e.g. "http://" or "https://" */

                  if ((ncomment == 0) &&
                      (n == 0 || strncmp(&line[n - 1], "://", 3) != 0))
                    {
                      ERROR("C++ style comment", lineno, n);
                      n++;
                      continue;
                    }
                }
            }

          /* Check if the line is blank so far.  This is only used to
           * to verify the the closing of a block comment is on a separate
           * line.  So we also need to treat '*' as a 'blank'.
           */

          if (!isblank(line[n]) && line[n] != '*')
            {
              bblank = false;
            }

          /* Check for a string... ignore if we are in the middle of a
           * comment.
           */

          if (ncomment == 0)
            {
              /* Backslash quoted character */

              if (line[n] == '\\')
                {
                  bquote = true;
                  n++;
                }

              /* Check for quoted characters: \" in string */

              if (line[n] == '"' && !bquote)
                {
                  bstring = !bstring;
                }

              bquote = false;
            }

          /* The rest of the line is only examined of we are not in a comment,
           * in a string or in assembly.
           *
           * REVISIT: Should still check for whitespace at the end of the
           * line.
           */

          if (ncomment == 0 && !bstring && inasm == 0)
            {
              switch (line[n])
                {
                /* Handle logic nested with curly braces */

                case '{':
                  {
                    if (n > indent)
                      {
                        /* REVISIT: dnest is always > 0 here if bfunctions ==
                         * false.
                         */

                        if (dnest == 0 || !bfunctions || lineno == rbrace_lineno)
                          {
                             ERROR("Left bracket not on separate line", lineno,
                                   n);
                          }
                      }
                    else if (line[n + 1] != '\n')
                      {
                        if (dnest == 0)
                          {
                             ERROR("Garbage follows left bracket", lineno, n);
                          }
                      }

                    bnest++;
                    if (dnest > 0)
                      {
                        dnest++;
                      }

                    /* Check if we are within 'extern "C"', we don't
                     * normally indent in that case because the 'extern "C"'
                     * is conditioned on __cplusplus.
                     */

                    if (lineno == externc_lineno ||
                        lineno - 1 == externc_lineno)
                      {
                        bexternc = true;
                      }

                    /* Suppress error for comment following a left brace */

                    noblank_lineno = lineno;
                    lbrace_lineno  = lineno;
                  }
                  break;

                case '}':
                  {
                   /* Decrement the brace nesting level */

                   if (bnest < 1)
                     {
                       ERROR("Unmatched right brace", lineno, n);
                     }
                   else
                     {
                       bnest--;
                       if (bnest < 1)
                         {
                           bnest = 0;
                           bswitch = false;
                         }
                     }

                    /* Decrement the declaration nesting level */

                    if (dnest < 3)
                      {
                        dnest = 0;
                        bexternc = false;
                      }
                    else
                      {
                        dnest--;
                      }

                    /* The right brace should be on a separate line */

                    if (n > indent)
                      {
                        if (dnest == 0)
                          {
                             ERROR("Right bracket not on separate line",
                                   lineno, n);
                          }
                      }

                    /* Check for garbage following the left brace */

                    if (line[n + 1] != '\n' &&
                        line[n + 1] != ',' &&
                        line[n + 1] != ';')
                      {
                        int sndx = n + 1;
                        bool whitespace = false;

                        /* Skip over spaces */

                        while (line[sndx] == ' ')
                          {
                            sndx++;
                          }

                        /* One possibility is that the right bracket is
                         * followed by an identifier then a semi-colon.
                         * Comma is possible to but would be a case of
                         * multiple declaration of multiple instances.
                         */

                        if (line[sndx] == '_' || isalpha(line[sndx]))
                          {
                            int endx = sndx;

                            /* Skip to the end of the identifier.  Checking
                             * for mixed case identifiers will be done
                             * elsewhere.
                             */

                            while (line[endx] == '_' ||
                                   isalnum(line[endx]))
                              {
                                endx++;
                              }

                            /* Skip over spaces */

                            while (line[endx] == ' ')
                              {
                                whitespace = true;
                                endx++;
                              }

                            /* Handle according to what comes after the
                             * identifier.
                             */

                            if (strncmp(&line[sndx], "while", 5) == 0)
                              {
                                 ERROR("'while' must be on a separate line",
                                        lineno, sndx);
                              }
                            else if (line[endx] == ',')
                              {
                                 ERROR("Multiple data definitions on line",
                                        lineno, endx);
                              }
                            else if (line[endx] == ';')
                              {
                                if (whitespace)
                                  {
                                     ERROR("Space precedes semi-colon",
                                           lineno, endx);
                                  }
                              }
                            else if (line[endx] == '=')
                              {
                                /* There's a struct initialization following */

                                check_spaces_leftright(line, lineno, endx, endx);
                                dnest = 1;
                              }
                            else
                              {
                                 ERROR("Garbage follows right bracket",
                                       lineno, n);
                              }
                          }
                        else
                          {
                             ERROR("Garbage follows right bracket", lineno, n);
                          }
                      }

                    /* The right brace should not be preceded with a a blank
                     * line.
                     */

                    if (lineno == blank_lineno + 1)
                      {
                         ERROR("Blank line precedes right brace at line",
                                lineno, 1);
                      }

                    rbrace_lineno  = lineno;
                  }
                  break;

                /* Handle logic with parentheses */

                case '(':
                  {
                    /* Increase the parenthetical nesting level */

                    pnest++;

                   /* Check for inappropriate space around parentheses */

                    if (line[n + 1] == ' ')  /* && !bfor */
                      {
                         ERROR("Space follows left parenthesis", lineno, n);
                      }
                  }
                  break;

                case ')':
                  {
                    /* Decrease the parenthetical nesting level */

                    if (pnest < 1)
                     {
                       ERROR("Unmatched right parentheses", lineno, n);
                       pnest = 0;
                     }
                   else
                     {
                       pnest--;
                     }

                    /* Allow ')' as first thing on the line (n == indent)
                     * Allow "for (xx; xx; )" (bfor == true)
                     */

                    if (n > 0 && n != indent && line[n - 1] == ' ' && !bfor)
                      {
                         ERROR("Space precedes right parenthesis", lineno, n);
                      }
                  }
                  break;

                /* Check for inappropriate space around square brackets */

                case '[':
                  {
                    if (line[n + 1] == ' ')
                      {
                         ERROR("Space follows left bracket", lineno, n);
                      }
                  }
                  break;

                case ']':
                  {
                    if (n > 0 && line[n - 1] == ' ')
                      {
                         ERROR("Space precedes right bracket", lineno, n);
                      }
                  }
                  break;

                /* Semi-colon may terminate a declaration */

                case ';':
                  {
                    if (!isspace((int)line[n + 1]))
                      {
                        ERROR("Missing whitespace after semicolon", lineno, n);
                      }

                    /* Semicolon terminates a declaration/definition if there
                     * was no left curly brace (i.e., dnest is only 1).
                     */

                    if (dnest == 1)
                      {
                        dnest = 0;
                      }
                  }
                  break;

                /* Semi-colon may terminate a declaration */

                case ',':
                  {
                    if (!isspace((int)line[n + 1]))
                      {
                        ERROR("Missing whitespace after comma", lineno, n);
                      }
                  }
                  break;

                /* Skip over character constants */

                case '\'':
                  {
                    int endndx = n + 2;

                    if (line[n + 1] != '\n' && line[n + 1] != '\0')
                      {
                        if (line[n + 1] == '\\')
                          {
                            for (;
                                 line[endndx] != '\n' &&
                                 line[endndx] != '\0' &&
                                 line[endndx] != '\'';
                                 endndx++);
                          }

                        n = endndx;
                      }
                  }
                  break;

                /* Check for space around various operators */

                case '-':

                  /* -> */

                  if (line[n + 1] == '>')
                    {
                      /* -> must have no whitespaces on its left or right */

                      check_nospaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }

                  /* -- */

                  else if (line[n + 1] == '-')
                    {
                      /* "--" should be next to its operand. If there are
                       * whitespaces or non-operand characters on both left
                       * and right (e.g. "a -- "， “a[i --]”, "(-- i)"),
                       * there's an error.
                       */

                      check_operand_leftright(line, lineno, n, n + 1);
                      n++;
                    }

                  /* -= */

                  else if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }

                  /* Scientific notation with a negative exponent (eg. 10e-10)
                   * REVISIT: This fails for cases where the variable name
                   *          ends with 'e' preceded by a digit:
                   *          a = abc1e-10;
                   *          a = ABC1E-10;
                   */

                  else if ((line[n - 1] == 'e' || line[n - 1] == 'E') &&
                           isdigit(line[n + 1]) && isdigit(line[n - 2]))
                    {
                      n++;
                    }
                  else
                    {
                      /* '-' may function as a unary operator and snuggle
                       * on the left.
                       */

                      check_spaces_left(line, lineno, n);
                    }

                  break;

                case '+':

                  /* ++ */

                  if (line[n + 1] == '+')
                    {
                      /* "++" should be next to its operand. If there are
                       * whitespaces or non-operand characters on both left
                       * and right (e.g. "a ++ "， “a[i ++]”, "(++ i)"),
                       * there's an error.
                       */

                      check_operand_leftright(line, lineno, n, n + 1);
                      n++;
                    }

                  /* += */

                  else if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else
                    {
                      /* '+' may function as a unary operator and snuggle
                       * on the left.
                       */

                      check_spaces_left(line, lineno, n);
                    }

                  break;

                case '&':

                  /* &<variable> OR &(<expression>) */

                  if (isalpha((int)line[n + 1]) || line[n + 1] == '_' ||
                      line[n + 1] == '(')
                    {
                    }

                  /* &&, &= */

                  else if (line[n + 1] == '=' || line[n + 1] == '&')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '/':

                  /* C comment terminator */

                  if (line[n - 1] == '*')
                    {
                      n++;
                    }

                    /* C++-style comment */

                  else if (line[n + 1] == '/')
                    {
                      /* Check for "http://" or "https://" */

                      if ((n < 5 || strncmp(&line[n - 5], "http://", 7) != 0) &&
                          (n < 6 || strncmp(&line[n - 6], "https://", 8) != 0))
                        {
                          ERROR("C++ style comment on at %d:%d\n",
                                lineno, n);
                        }

                      n++;
                    }

                  /* /= */

                  else if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }

                  /* Division operator */

                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '*':

                  /* *\/, ** */

                  if (line[n] == '*' &&
                      (line[n + 1] == '/' ||
                       line[n + 1] == '*'))
                    {
                     n++;
                     break;
                    }

                  /* *<variable>, *(<expression>) */

                  else if (isalpha((int)line[n + 1]) ||
                           line[n + 1] == '_' ||
                           line[n + 1] == '(')
                    {
                      break;
                    }

                  /* (<type> *) */

                  else if (line[n + 1] == ')')
                    {
                      /* REVISIT: This gives false alarms on syntax like *--ptr */

                      if (line[n - 1] != ' ' && line[n - 1] != '(')
                        {
                           ERROR("Operator/assignment must be preceded "
                                  "with whitespace", lineno, n);
                        }

                      break;
                    }

                  /* *= */

                  else if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else
                    {
                      /* A single '*' may be an binary operator, but
                       * it could also be a unary operator when used to deference
                       * a pointer.
                       */

                      check_spaces_left(line, lineno, n);
                    }

                  break;

                case '%':

                  /* %= */

                  if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '<':

                  /* <=, <<, <<= */

                  if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else if (line[n + 1] == '<')
                    {
                      if (line[n + 2] == '=')
                        {
                          check_spaces_leftright(line, lineno, n, n + 2);
                          n += 2;
                        }
                      else
                        {
                          check_spaces_leftright(line, lineno, n, n + 1);
                          n++;
                        }
                    }
                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '>':

                  /* >=, >>, >>= */

                  if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else if (line[n + 1] == '>')
                    {
                      if (line[n + 2] == '=')
                        {
                          check_spaces_leftright(line, lineno, n, n + 2);
                          n += 2;
                        }
                      else
                        {
                          check_spaces_leftright(line, lineno, n, n + 1);
                          n++;
                        }
                    }
                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '|':

                  /* |=, || */

                  if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else if (line[n + 1] == '|')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '^':

                  /* ^= */

                  if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '=':

                  /* == */

                  if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '~':
                  check_spaces_left(line, lineno, n);
                  break;

                case '!':

                  /* != */

                  if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }

                  /* !! */

                  else if (line[n + 1] == '!')
                    {
                      check_spaces_left(line, lineno, n);
                      n++;
                    }
                  else
                    {
                      check_spaces_left(line, lineno, n);
                    }

                  break;

                default:
                  break;
                }
            }
        }

      /* Loop terminates when NUL or newline character found */

      if (line[n] == '\n' || line[n] == '\0')
        {
          /* If the parse terminated on the NULL, then back up to the last
           * character (which should be the newline).
           */

          int m = n;
          if (line[m] == '\0' && m > 0)
            {
              m--;
            }

          /* Check for space at the end of the line.  Except for carriage
           * returns which we have already reported (one time) above.
           */

          if (m > 1 && isspace((int)line[m - 1]) &&
              line[m - 1] != '\n' && line[m - 1] != '\r')
            {
               ERROR("Dangling whitespace at the end of line", lineno, m);
            }

          /* The line width is determined by the location of the final
           * asterisk in block comments.  The closing line of the block
           * comment will exceed that by one one character, the '/'
           * following the final asterisk.
           */

          else if (m > g_maxline)
            {
              bool bslash;
              int a;

              for (bslash = false, a = m;
                   a > 2 && strchr("\n\r/", line[a]) != NULL;
                   a--)
                {
                  if (line[a] == '/')
                    {
                      bslash = true;
                    }
                }

              if (bslash && line[a] == '*')
                {
                  m = a + 1;
                }
            }

          /* Check for long lines
           *
           * REVISIT:  Long line checks suppressed on right hand comments
           * for now.  This just prevents a large number of difficult-to-
           * fix complaints that we would have otherwise.
           */

          if (m > g_maxline && !rhcomment)
            {
              ERROR("Long line found", lineno, m);
            }
        }

      /* STEP 4: Check alignment */

      /* Within a comment block, we need only check on the alignment of the
       * comment.
       */

      if ((ncomment > 0 || prevncomment > 0) && !bstring)
        {
          /* Nothing should begin in comment zero */

          if (indent == 0 && line[0] != '/' && !bexternc)
            {
              /* NOTE:  if this line contains a comment to the right of the
               * code, then ncomment will be misleading because it was
               * already incremented above.
               */

              if (ncomment > 1 || rhcomment == 0)
                {
                  ERROR("No indentation line", lineno, indent);
                }
            }
          else if (indent == 1 && line[0] == ' ' && line[1] == '*')
            {
              /* Good indentation */
            }
          else if (indent > 0 && line[indent] == '\n')
            {
              ERROR("Whitespace on blank line", lineno, indent);
            }
          else if (indent > 0 && indent < 2)
            {
              if (bnest > 0)
                {
                  ERROR("Insufficient indentation", lineno, indent);
                }
              else
                {
                  ERROR("Expected indentation line", lineno, indent);
                }
            }
          else if (indent > 0 && !bswitch)
            {
              if (line[indent] == '/')
                {
                  /* Comments should like at offsets 2, 6, 10, ...
                   * This rule is not followed, however, if the comments are
                   * aligned to the right of the code.
                   */

                  if ((indent & 3) != 2 && rhcomment == 0)
                    {
                       ERROR("Bad comment alignment", lineno, indent);
                    }

                  /* REVISIT:  This screws up in cases where there is C code,
                   * followed by a comment that continues on the next line.
                   */

                  else if (line[indent + 1] != '*')
                    {
                       ERROR("Missing asterisk in comment", lineno, indent);
                    }
                }
              else if (line[indent] == '*')
                {
                  /* REVISIT: Generates false alarms on comments at the end of
                   * the line if there is nothing preceding (such as the aligned
                   * comments with a structure field definition).  So disabled
                   * for comments before beginning of function definitions.
                   *
                   * Suppress this error if this is a comment to the right of
                   * code.
                   * Those may be unaligned.
                   */

                  if ((indent & 3) != 3 && bfunctions && dnest == 0 &&
                      rhcomment == 0)
                    {
                       ERROR("Bad comment block alignment", lineno, indent);
                    }

                  if (line[indent + 1] != ' ' &&
                      line[indent + 1] != '*' &&
                      line[indent + 1] != '\n' &&
                      line[indent + 1] != '/')
                    {
                       ERROR("Invalid character after asterisk "
                             "in comment block", lineno, indent);
                    }
                }

              /* If this is not the line containing the comment start, then this
               * line should begin with '*'
               */

              else if (prevncomment > 0)
                {
                  ERROR("Missing asterisk in comment block", lineno, indent);
                }
            }
        }

      /* Check for various alignment outside of the comment block */

      else if ((ncomment == 0 && prevncomment == 0) && !bstring)
        {
          if (indent == 0 && strchr("\n#{}", line[0]) == NULL)
            {
               /* Ignore if we are at global scope */

               if (prevbnest > 0)
                {
                  bool blabel = false;

                  if (isalpha((int)line[indent]))
                    {
                      for (i = indent + 1; isalnum((int)line[i]) ||
                           line[i] == '_'; i++);
                      blabel = (line[i] == ':');
                    }

                  if (!blabel && !bexternc)
                    {
                      ERROR("No indentation line", lineno, indent);
                    }
                }
            }
          else if (indent == 1 && line[0] == ' ' && line[1] == '*')
            {
              /* Good indentation */
            }
          else if (indent > 0 && line[indent] == '\n')
            {
              ERROR("Whitespace on blank line", lineno, indent);
            }
          else if (indent > 0 && indent < 2)
            {
              ERROR("Insufficient indentation line", lineno, indent);
            }
          else if (line[indent] == '{')
            {
              /* Check for left brace in first column, but preceded by a
               * blank line.  Should never happen (but could happen with
               * internal compound statements).
               */

              if (indent == 0 && lineno == blank_lineno + 1)
                {
                  ERROR("Blank line before opening left brace", lineno, indent);
                }

              /* REVISIT:  Possible false alarms in compound statements
               * without a preceding conditional.  That usage often violates
               * the coding standard.
               */

              else if (!bfunctions && (indent & 1) != 0)
                {
                  ERROR("Bad left brace alignment", lineno, indent);
                }
              else if ((indent & 3) != 0 && !bswitch && dnest == 0)
                {
                  ERROR("Bad left brace alignment", lineno, indent);
                }
            }
          else if (line[indent] == '}')
            {
              /* REVISIT:  Possible false alarms in compound statements
               * without a preceding conditional.  That usage often violates
               * the coding standard.
               */

              if (!bfunctions && (indent & 1) != 0)
                {
                  ERROR("Bad left brace alignment", lineno, indent);
                }
              else if ((indent & 3) != 0 && !bswitch && prevdnest == 0)
                {
                  ERROR("Bad right brace alignment", lineno, indent);
                }
            }
          else if (indent > 0)
            {
              /* REVISIT: Generates false alarms when a statement continues on
               * the next line.  The bstatm check limits to lines beginning
               * with C keywords.
               * REVISIT:  The bstatm check will not detect statements that
               * do not begin with a C keyword (such as assignment statements).
               * REVISIT: Generates false alarms on comments at the end of
               * the line if there is nothing preceding (such as the aligned
               * comments with a structure field definition).  So disabled for
               * comments before beginning of function definitions.
               */

              if ((bstatm ||                              /* Begins with C keyword */
                  (line[indent] == '/' &&
                  bfunctions &&
                  line[indent + 1] == '*')) &&            /* Comment in functions */
                  !bswitch &&                             /* Not in a switch */
                  dnest == 0)                             /* Not a data definition */
                {
                  if ((indent & 3) != 2)
                    {
                      ERROR("Bad alignment", lineno, indent);
                    }
                }

              /* Crazy cases.  There should be no small odd alignments
               * outside of comment/string.  Odd alignments are possible
               * on continued lines, but not if they are small.
               */

              else if (indent == 1 || indent == 3)
                {
                  ERROR("Small odd alignment", lineno, indent);
                }
            }
        }
    }

  if (!bfunctions && g_file_type == C_SOURCE)
    {
      ERROR("\"Private/Public Functions\" not found!"
            " File will not be checked", lineno, 1);
    }

  if (ncomment > 0 || bstring)
    {
      ERROR("Comment or string found at end of file", lineno, 1);
    }

  fclose(instream);
  if (g_verbose == 1)
    {
      fprintf(stderr, "%s: %s nxstyle check\n", g_file_name,
              g_status == 0 ? "PASSED" : "FAILED");
    }

  return g_status;
}

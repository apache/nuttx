/****************************************************************************
 * libs/libc/string/lib_strerror.c
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STRERROR_UNKNOWN "Unknown error"
#define STRERROR_BUFSIZE sizeof(STRERROR_UNKNOWN " 2000")

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct errno_strmap_s
{
  uint8_t   errnum;
  FAR char *str;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LIBC_STRERROR

/* This table maps all error numbers to descriptive strings.
 * The only assumption that the code makes with regard to this
 * this table is that it is ordered by error number.
 *
 * The size of this table is quite large.  Its size can be
 * reduced by eliminating some of the more obscure error
 * strings.
 */

#ifndef CONFIG_LIBC_STRERROR_SHORT

static const struct errno_strmap_s g_errnomap[] =
{
  { 0,                   "Success"           },
  { EPERM,               EPERM_STR           },
  { ENOENT,              ENOENT_STR          },
  { ESRCH,               ESRCH_STR           },
  { EINTR,               EINTR_STR           },
  { EIO,                 EIO_STR             },
  { ENXIO,               ENXIO_STR           },
  { E2BIG,               E2BIG_STR           },
  { ENOEXEC,             ENOEXEC_STR         },
  { EBADF,               EBADF_STR           },
  { ECHILD,              ECHILD_STR          },
  { EAGAIN,              EAGAIN_STR          },
  { ENOMEM,              ENOMEM_STR          },
  { EACCES,              EACCES_STR          },
  { EFAULT,              EFAULT_STR          },
  { ENOTBLK,             ENOTBLK_STR         },
  { EBUSY,               EBUSY_STR           },
  { EEXIST,              EEXIST_STR          },
  { EXDEV,               EXDEV_STR           },
  { ENODEV,              ENODEV_STR          },
  { ENOTDIR,             ENOTDIR_STR         },
  { EISDIR,              EISDIR_STR          },
  { EINVAL,              EINVAL_STR          },
  { ENFILE,              ENFILE_STR          },
  { EMFILE,              EMFILE_STR          },
  { ENOTTY,              ENOTTY_STR          },
  { ETXTBSY,             ETXTBSY_STR         },
  { EFBIG,               EFBIG_STR           },
  { ENOSPC,              ENOSPC_STR          },
  { ESPIPE,              ESPIPE_STR          },
  { EROFS,               EROFS_STR           },
  { EMLINK,              EMLINK_STR          },
  { EPIPE,               EPIPE_STR           },
  { EDOM,                EDOM_STR            },
  { ERANGE,              ERANGE_STR          },
  { EDEADLK,             EDEADLK_STR         },
  { ENAMETOOLONG,        ENAMETOOLONG_STR    },
  { ENOLCK,              ENOLCK_STR          },
  { ENOSYS,              ENOSYS_STR          },
  { ENOTEMPTY,           ENOTEMPTY_STR       },
  { ELOOP,               ELOOP_STR           },
  { ENOMSG,              ENOMSG_STR          },
  { EIDRM,               EIDRM_STR           },
  { ECHRNG,              ECHRNG_STR          },
  { EL2NSYNC,            EL2NSYNC_STR        },
  { EL3HLT,              EL3HLT_STR          },
  { EL3RST,              EL3RST_STR          },
  { ELNRNG,              ELNRNG_STR          },
  { EUNATCH,             EUNATCH_STR         },
  { ENOCSI,              ENOCSI_STR          },
  { EL2HLT,              EL2HLT_STR          },
  { EBADE,               EBADE_STR           },
  { EBADR,               EBADR_STR           },
  { EXFULL,              EXFULL_STR          },
  { ENOANO,              ENOANO_STR          },
  { EBADRQC,             EBADRQC_STR         },
  { EBADSLT,             EBADSLT_STR         },
  { EBFONT,              EBFONT_STR          },
  { ENOSTR,              ENOSTR_STR          },
  { ENODATA,             ENODATA_STR         },
  { ETIME,               ETIME_STR           },
  { ENOSR,               ENOSR_STR           },
  { ENONET,              ENONET_STR          },
  { ENOPKG,              ENOPKG_STR          },
  { EREMOTE,             EREMOTE_STR         },
  { ENOLINK,             ENOLINK_STR         },
  { EADV,                EADV_STR            },
  { ESRMNT,              ESRMNT_STR          },
  { ECOMM,               ECOMM_STR           },
  { EPROTO,              EPROTO_STR          },
  { EMULTIHOP,           EMULTIHOP_STR       },
  { EDOTDOT,             EDOTDOT_STR         },
  { EBADMSG,             EBADMSG_STR         },
  { EOVERFLOW,           EOVERFLOW_STR       },
  { ENOTUNIQ,            ENOTUNIQ_STR        },
  { EBADFD,              EBADFD_STR          },
  { EREMCHG,             EREMCHG_STR         },
  { ELIBACC,             ELIBACC_STR         },
  { ELIBBAD,             ELIBBAD_STR         },
  { ELIBSCN,             ELIBSCN_STR         },
  { ELIBMAX,             ELIBMAX_STR         },
  { ELIBEXEC,            ELIBEXEC_STR        },
  { EILSEQ,              EILSEQ_STR          },
  { ERESTART,            ERESTART_STR        },
  { ESTRPIPE,            ESTRPIPE_STR        },
  { EUSERS,              EUSERS_STR          },
  { ENOTSOCK,            ENOTSOCK_STR        },
  { EDESTADDRREQ,        EDESTADDRREQ_STR    },
  { EMSGSIZE,            EMSGSIZE_STR        },
  { EPROTOTYPE,          EPROTOTYPE_STR      },
  { ENOPROTOOPT,         ENOPROTOOPT_STR     },
  { EPROTONOSUPPORT,     EPROTONOSUPPORT_STR },
  { ESOCKTNOSUPPORT,     ESOCKTNOSUPPORT_STR },
  { EOPNOTSUPP,          EOPNOTSUPP_STR      },
  { EPFNOSUPPORT,        EPFNOSUPPORT_STR    },
  { EAFNOSUPPORT,        EAFNOSUPPORT_STR    },
  { EADDRINUSE,          EADDRINUSE_STR      },
  { EADDRNOTAVAIL,       EADDRNOTAVAIL_STR   },
  { ENETDOWN,            ENETDOWN_STR        },
  { ENETUNREACH,         ENETUNREACH_STR     },
  { ENETRESET,           ENETRESET_STR       },
  { ECONNABORTED,        ECONNABORTED_STR    },
  { ECONNRESET,          ECONNRESET_STR      },
  { ENOBUFS,             ENOBUFS_STR         },
  { EISCONN,             EISCONN_STR         },
  { ENOTCONN,            ENOTCONN_STR        },
  { ESHUTDOWN,           ESHUTDOWN_STR       },
  { ETOOMANYREFS,        ETOOMANYREFS_STR    },
  { ETIMEDOUT,           ETIMEDOUT_STR       },
  { ECONNREFUSED,        ECONNREFUSED_STR    },
  { EHOSTDOWN,           EHOSTDOWN_STR       },
  { EHOSTUNREACH,        EHOSTUNREACH_STR    },
  { EALREADY,            EALREADY_STR        },
  { EINPROGRESS,         EINPROGRESS_STR     },
  { ESTALE,              ESTALE_STR          },
  { EUCLEAN,             EUCLEAN_STR         },
  { ENOTNAM,             ENOTNAM_STR         },
  { ENAVAIL,             ENAVAIL_STR         },
  { EISNAM,              EISNAM_STR          },
  { EREMOTEIO,           EREMOTEIO_STR       },
  { EDQUOT,              EDQUOT_STR          },
  { ENOMEDIUM,           ENOMEDIUM_STR       },
  { EMEDIUMTYPE,         EMEDIUMTYPE_STR     },
  { ECANCELED,           ECANCELED_STR       },
  { ENOKEY,              ENOKEY_STR          },
  { EKEYEXPIRED,         EKEYEXPIRED_STR     },
  { EKEYREVOKED,         EKEYREVOKED_STR     },
  { EKEYREJECTED,        EKEYREJECTED_STR    },
  { EOWNERDEAD,          EOWNERDEAD_STR      },
  { ENOTRECOVERABLE,     ENOTRECOVERABLE_STR },
  { ERFKILL,             ERFKILL_STR         },
  { EHWPOISON,           EHWPOISON_STR       },
  { ELBIN,               ELBIN_STR           },
  { EFTYPE,              EFTYPE_STR          },
  { ENMFILE,             ENMFILE_STR         },
  { EPROCLIM,            EPROCLIM_STR        },
  { ENOTSUP,             ENOTSUP_STR         },
  { ENOSHARE,            ENOSHARE_STR        },
  { ECASECLASH,          ECASECLASH_STR      },
};

#else /* CONFIG_LIBC_STRERROR_SHORT */

static const struct errno_strmap_s g_errnomap[] =
{
  { 0,                   "OK"                },
  { EPERM,               "EPERM"             },
  { ENOENT,              "ENOENT"            },
  { ESRCH,               "ESRCH"             },
  { EINTR,               "EINTR"             },
  { EIO,                 "EIO"               },
  { ENXIO,               "ENXIO"             },
  { E2BIG,               "E2BIG"             },
  { ENOEXEC,             "ENOEXEC"           },
  { EBADF,               "EBADF"             },
  { ECHILD,              "ECHILD"            },
  { EAGAIN,              "EAGAIN"            },
  { ENOMEM,              "ENOMEM"            },
  { EACCES,              "EACCES"            },
  { EFAULT,              "EFAULT"            },
  { ENOTBLK,             "ENOTBLK"           },
  { EBUSY,               "EBUSY"             },
  { EEXIST,              "EEXIST"            },
  { EXDEV,               "EXDEV"             },
  { ENODEV,              "ENODEV"            },
  { ENOTDIR,             "ENOTDIR"           },
  { EISDIR,              "EISDIR"            },
  { EINVAL,              "EINVAL"            },
  { ENFILE,              "ENFILE"            },
  { EMFILE,              "EMFILE"            },
  { ENOTTY,              "ENOTTY"            },
  { ETXTBSY,             "ETXTBSY"           },
  { EFBIG,               "EFBIG"             },
  { ENOSPC,              "ENOSPC"            },
  { ESPIPE,              "ESPIPE"            },
  { EROFS,               "EROFS"             },
  { EMLINK,              "EMLINK"            },
  { EPIPE,               "EPIPE"             },
  { EDOM,                "EDOM"              },
  { ERANGE,              "ERANGE"            },
  { EDEADLK,             "EDEADLK"           },
  { ENAMETOOLONG,        "ENAMETOOLONG"      },
  { ENOLCK,              "ENOLCK"            },
  { ENOSYS,              "ENOSYS"            },
  { ENOTEMPTY,           "ENOTEMPTY"         },
  { ELOOP,               "ELOOP"             },
  { ENOMSG,              "ENOMSG"            },
  { EIDRM,               "EIDRM"             },
  { ECHRNG,              "ECHRNG"            },
  { EL2NSYNC,            "EL2NSYNC"          },
  { EL3HLT,              "EL3HLT"            },
  { EL3RST,              "EL3RST"            },
  { ELNRNG,              "ELNRNG"            },
  { EUNATCH,             "EUNATCH"           },
  { ENOCSI,              "ENOCSI"            },
  { EL2HLT,              "EL2HLT"            },
  { EBADE,               "EBADE"             },
  { EBADR,               "EBADR"             },
  { EXFULL,              "EXFULL"            },
  { ENOANO,              "ENOANO"            },
  { EBADRQC,             "EBADRQC"           },
  { EBADSLT,             "EBADSLT"           },
  { EBFONT,              "EBFONT"            },
  { ENOSTR,              "ENOSTR"            },
  { ENODATA,             "ENODATA"           },
  { ETIME,               "ETIME"             },
  { ENOSR,               "ENOSR"             },
  { ENONET,              "ENONET"            },
  { ENOPKG,              "ENOPKG"            },
  { EREMOTE,             "EREMOTE"           },
  { ENOLINK,             "ENOLINK"           },
  { EADV,                "EADV"              },
  { ESRMNT,              "ESRMNT"            },
  { ECOMM,               "ECOMM"             },
  { EPROTO,              "EPROTO"            },
  { EMULTIHOP,           "EMULTIHOP"         },
  { EDOTDOT,             "EDOTDOT"           },
  { EBADMSG,             "EBADMSG"           },
  { EOVERFLOW,           "EOVERFLOW"         },
  { ENOTUNIQ,            "ENOTUNIQ"          },
  { EBADFD,              "EBADFD"            },
  { EREMCHG,             "EREMCHG"           },
  { ELIBACC,             "ELIBACC"           },
  { ELIBBAD,             "ELIBBAD"           },
  { ELIBSCN,             "ELIBSCN"           },
  { ELIBMAX,             "ELIBMAX"           },
  { ELIBEXEC,            "ELIBEXEC"          },
  { EILSEQ,              "EILSEQ"            },
  { ERESTART,            "ERESTART"          },
  { ESTRPIPE,            "ESTRPIPE"          },
  { EUSERS,              "EUSERS"            },
  { ENOTSOCK,            "ENOTSOCK"          },
  { EDESTADDRREQ,        "EDESTADDRREQ"      },
  { EMSGSIZE,            "EMSGSIZE"          },
  { EPROTOTYPE,          "EPROTOTYPE"        },
  { ENOPROTOOPT,         "ENOPROTOOPT"       },
  { EPROTONOSUPPORT,     "EPROTONOSUPPORT"   },
  { ESOCKTNOSUPPORT,     "ESOCKTNOSUPPORT"   },
  { EOPNOTSUPP,          "EOPNOTSUPP"        },
  { EPFNOSUPPORT,        "EPFNOSUPPORT"      },
  { EAFNOSUPPORT,        "EAFNOSUPPORT"      },
  { EADDRINUSE,          "EADDRINUSE"        },
  { EADDRNOTAVAIL,       "EADDRNOTAVAIL"     },
  { ENETDOWN,            "ENETDOWN"          },
  { ENETUNREACH,         "ENETUNREACH"       },
  { ENETRESET,           "ENETRESET"         },
  { ECONNABORTED,        "ECONNABORTED"      },
  { ECONNRESET,          "ECONNRESET"        },
  { ENOBUFS,             "ENOBUFS"           },
  { EISCONN,             "EISCONN"           },
  { ENOTCONN,            "ENOTCONN"          },
  { ESHUTDOWN,           "ESHUTDOWN"         },
  { ETOOMANYREFS,        "ETOOMANYREFS"      },
  { ETIMEDOUT,           "ETIMEDOUT"         },
  { ECONNREFUSED,        "ECONNREFUSED"      },
  { EHOSTDOWN,           "EHOSTDOWN"         },
  { EHOSTUNREACH,        "EHOSTUNREACH"      },
  { EALREADY,            "EALREADY"          },
  { EINPROGRESS,         "EINPROGRESS"       },
  { ESTALE,              "ESTALE"            },
  { EUCLEAN,             "EUCLEAN"           },
  { ENOTNAM,             "ENOTNAM"           },
  { ENAVAIL,             "ENAVAIL"           },
  { EISNAM,              "EISNAM"            },
  { EREMOTEIO,           "EREMOTEIO"         },
  { EDQUOT,              "EDQUOT"            },
  { ENOMEDIUM,           "ENOMEDIUM"         },
  { EMEDIUMTYPE,         "EMEDIUMTYPE"       },
  { ECANCELED,           "ECANCELED"         },
  { ENOKEY,              "ENOKEY"            },
  { EKEYEXPIRED,         "EKEYEXPIRED"       },
  { EKEYREVOKED,         "EKEYREVOKED"       },
  { EKEYREJECTED,        "EKEYREJECTED"      },
  { EOWNERDEAD,          "EOWNERDEAD"        },
  { ENOTRECOVERABLE,     "ENOTRECOVERABLE"   },
  { ERFKILL,             "ERFKILL"           },
  { EHWPOISON,           "EHWPOISON"         },
  { ELBIN,               "ELBIN"             },
  { EFTYPE,              "EFTYPE"            },
  { ENMFILE,             "ENMFILE"           },
  { EPROCLIM,            "EPROCLIM"          },
  { ENOTSUP,             "ENOTSUP"           },
  { ENOSHARE,            "ENOSHARE"          },
  { ECASECLASH,          "ECASECLASH"        }
};

#endif /* CONFIG_LIBC_STRERROR_SHORT */

#define NERRNO_STRS (sizeof(g_errnomap) / sizeof(struct errno_strmap_s))

#endif /* CONFIG_LIBC_STRERROR */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strerror
 ****************************************************************************/

FAR char *strerror(int errnum)
{
#ifdef CONFIG_LIBC_STRERROR_ERRNUM
  static char s_err[STRERROR_BUFSIZE];
#endif
#ifdef CONFIG_LIBC_STRERROR
  int ndxlow = 0;
  int ndxhi  = NERRNO_STRS - 1;
  int ndxmid;

  do
    {
      ndxmid = (ndxlow + ndxhi) >> 1;
      if (errnum > g_errnomap[ndxmid].errnum)
        {
          ndxlow = ndxmid + 1;
        }
      else if (errnum < g_errnomap[ndxmid].errnum)
        {
          ndxhi = ndxmid - 1;
        }
      else
        {
          return g_errnomap[ndxmid].str;
        }
    }
  while (ndxlow <= ndxhi);
#endif
#ifdef CONFIG_LIBC_STRERROR_ERRNUM
  if (snprintf(s_err, sizeof(s_err), STRERROR_UNKNOWN " %d", errnum)
      < sizeof(s_err))
    {
      return s_err;
    }
#elif !defined(CONFIG_LIBC_STRERROR)
  UNUSED(errnum);
#endif

  return STRERROR_UNKNOWN;
}

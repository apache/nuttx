/****************************************************************************
 * include/nuttx/wireless/lte/lte_ioctl.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_LTE_LTE_IOCTL_H
#define __INCLUDE_NUTTX_WIRELESS_LTE_LTE_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/wireless/wireless.h>
#include <nuttx/net/sms.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LTE network device IOCTL commands. */

/* SIOCLTECMD
 *   Description:   Perform LTE command
 */

#define SIOCLTECMD            _LTEIOC(0)

/* SIOCSMSENSTREP
 *   Description:   Enable or disable the function to confirm whether or not
 *                  the SMS has been delivered to the destination device.
 */

#define SIOCSMSENSTREP         _LTEIOC(1)

/* SIOCSMSGREFID
 *   Description:   Obtain the ID associated with the submitted SMS.
 *                  If the submitted SMS is a concatenated SMS, multiple IDs
 *                  will be obtained;
 *                  otherwise, a single ID will be obtained.
 */

#define SIOCSMSGREFID          _LTEIOC(2)

/* SIOCSMSSSCA
 *   Description:   Set the service center address of the destination.
 */

#define SIOCSMSSSCA            _LTEIOC(3)

/* for cmdid */

#define _CMDOPT_LSB                  (28)
#define _CMDOPT_MASK                 (0xf << (_CMDOPT_LSB))
#define _CMDGRP_LSB                  (24)
#define _CMDGRP_MASK                 (0xf << (_CMDGRP_LSB))
#define _CMDGRP_SHIFT(nr)            ((nr) << (_CMDGRP_LSB))
#define _CMDGRP_NORMAL(nr)           (_CMDGRP_SHIFT(0) | (nr))
#define _CMDGRP_EVENT(nr)            (_CMDGRP_SHIFT(1) | (nr))
#define _CMDGRP_NOMDM(nr)            (_CMDGRP_SHIFT(2) | (nr))
#define _CMDGRP_POWER(nr)            (_CMDGRP_SHIFT(3) | (nr))
#define _CMDGRP_FWUPDATE(nr)         (_CMDGRP_SHIFT(4) | (nr))
#define _CMDGRP_EXTEND(nr)           (_CMDGRP_SHIFT(5) | (nr))
#define _CMDGRP_LWM2M(nr)            (_CMDGRP_SHIFT(6) | (nr))
#define LTE_CMDOPT_ASYNC_BIT         (0x1 << (_CMDOPT_LSB))
#define LTE_IS_ASYNC_CMD(cid)        ((cid) & LTE_CMDOPT_ASYNC_BIT)
#define LTE_PURE_CMDID(cid)          ((cid) & ~LTE_CMDOPT_ASYNC_BIT)
#define LTE_ISCMDGRP_NORMAL(cmd)     ((cmd & _CMDGRP_MASK) == _CMDGRP_NORMAL(0))
#define LTE_ISCMDGRP_EVENT(cmd)      ((cmd & _CMDGRP_MASK) == _CMDGRP_EVENT(0))
#define LTE_ISCMDGRP_NOMDM(cmd)      ((cmd & _CMDGRP_MASK) == _CMDGRP_NOMDM(0))
#define LTE_ISCMDGRP_POWER(cmd)      ((cmd & _CMDGRP_MASK) == _CMDGRP_POWER(0))
#define LTE_ISCMDGRP_FWUPDATE(cmd)   ((cmd & _CMDGRP_MASK) == _CMDGRP_FWUPDATE(0))
#define LTE_ISCMDGRP_EXTEND(cmd)     ((cmd & _CMDGRP_MASK) == _CMDGRP_EXTEND(0))
#define LTE_ISCMDGRP_LWM2M(cmd)      ((cmd & _CMDGRP_MASK) == _CMDGRP_LWM2M(0))

#define LTE_CMDID_POWERON                        _CMDGRP_POWER(0x00)
#define LTE_CMDID_POWEROFF                       _CMDGRP_POWER(0x01)
#define LTE_CMDID_FIN                            _CMDGRP_POWER(0x02)
#define LTE_CMDID_SETEVTCTX                      _CMDGRP_NOMDM(0x03)
#define LTE_CMDID_SETRESTART                     _CMDGRP_NOMDM(0x04)
#define LTE_CMDID_GETERRINFO                     _CMDGRP_NOMDM(0x05)
#define LTE_CMDID_GETVER                         _CMDGRP_NORMAL(0x06)
#define LTE_CMDID_RADIOON                        _CMDGRP_NORMAL(0x07)
#define LTE_CMDID_RADIOOFF                       _CMDGRP_NORMAL(0x08)
#define LTE_CMDID_ACTPDN                         _CMDGRP_NORMAL(0x09)
#define LTE_CMDID_ACTPDNCAN                      _CMDGRP_NORMAL(0x0a)
#define LTE_CMDID_DEACTPDN                       _CMDGRP_NORMAL(0x0b)
#define LTE_CMDID_GETNETINFO                     _CMDGRP_NORMAL(0x0c)
#define LTE_CMDID_IMSCAP                         _CMDGRP_NORMAL(0x0d)
#define LTE_CMDID_GETPHONE                       _CMDGRP_NORMAL(0x0e)
#define LTE_CMDID_GETIMSI                        _CMDGRP_NORMAL(0x0f)
#define LTE_CMDID_GETIMEI                        _CMDGRP_NORMAL(0x10)
#define LTE_CMDID_GETPINSET                      _CMDGRP_NORMAL(0x11)
#define LTE_CMDID_PINENABLE                      _CMDGRP_NORMAL(0x12)
#define LTE_CMDID_CHANGEPIN                      _CMDGRP_NORMAL(0x13)
#define LTE_CMDID_ENTERPIN                       _CMDGRP_NORMAL(0x14)
#define LTE_CMDID_GETLTIME                       _CMDGRP_NORMAL(0x15)
#define LTE_CMDID_GETOPER                        _CMDGRP_NORMAL(0x16)
#define LTE_CMDID_GETEDRX                        _CMDGRP_NORMAL(0x17)
#define LTE_CMDID_SETEDRX                        _CMDGRP_NORMAL(0x18)
#define LTE_CMDID_GETPSM                         _CMDGRP_NORMAL(0x19)
#define LTE_CMDID_SETPSM                         _CMDGRP_NORMAL(0x1a)
#define LTE_CMDID_GETCE                          _CMDGRP_NORMAL(0x1b)
#define LTE_CMDID_SETCE                          _CMDGRP_NORMAL(0x1c)
#define LTE_CMDID_GETSIMINFO                     _CMDGRP_NORMAL(0x1d)
#define LTE_CMDID_GETCEDRX                       _CMDGRP_NORMAL(0x1e)
#define LTE_CMDID_GETCPSM                        _CMDGRP_NORMAL(0x1f)
#define LTE_CMDID_GETQUAL                        _CMDGRP_NORMAL(0x20)
#define LTE_CMDID_GETCELL                        _CMDGRP_NORMAL(0x21)
#define LTE_CMDID_GETRAT                         _CMDGRP_NORMAL(0x22)
#define LTE_CMDID_SETRAT                         _CMDGRP_NORMAL(0x23)
#define LTE_CMDID_GETRATINFO                     _CMDGRP_NORMAL(0x24)
#define LTE_CMDID_REPNETINFO                     _CMDGRP_EVENT(0x25)
#define LTE_CMDID_REPSIMSTAT                     _CMDGRP_EVENT(0x26)
#define LTE_CMDID_REPLTIME                       _CMDGRP_EVENT(0x27)
#define LTE_CMDID_REPQUAL                        _CMDGRP_EVENT(0x28)
#define LTE_CMDID_REPCELL                        _CMDGRP_EVENT(0x29)
#define LTE_CMDID_SAVEAPN                        _CMDGRP_NOMDM(0x2a)
#define LTE_CMDID_GETAPN                         _CMDGRP_NOMDM(0x2b)
#define LTE_CMDID_TAKEWLOCK                      _CMDGRP_POWER(0x2c)
#define LTE_CMDID_GIVEWLOCK                      _CMDGRP_POWER(0x2d)
#define LTE_CMDID_SENDATCMD                      _CMDGRP_NORMAL(0x2e)
#define LTE_CMDID_INJECTIMAGE                    _CMDGRP_FWUPDATE(0x2f)
#define LTE_CMDID_GETIMAGELEN                    _CMDGRP_FWUPDATE(0x30)
#define LTE_CMDID_EXEUPDATE                      _CMDGRP_FWUPDATE(0x31)
#define LTE_CMDID_GETUPDATERES                   _CMDGRP_NORMAL(0x32)
#define LTE_CMDID_FACTORY_RESET                  _CMDGRP_EXTEND(0x33)
#define LTE_CMDID_SAVE_LOG                       _CMDGRP_NORMAL(0x34)
#define LTE_CMDID_GET_LOGLIST                    _CMDGRP_NORMAL(0x35)
#define LTE_CMDID_LOGOPEN                        _CMDGRP_NORMAL(0x36)
#define LTE_CMDID_LOGCLOSE                       _CMDGRP_NORMAL(0x37)
#define LTE_CMDID_LOGREAD                        _CMDGRP_NORMAL(0x38)
#define LTE_CMDID_LOGLSEEK                       _CMDGRP_NORMAL(0x39)
#define LTE_CMDID_LOGREMOVE                      _CMDGRP_NORMAL(0x3a)

#define LTE_CMDID_ACCEPT                         _CMDGRP_NORMAL(0x50)
#define LTE_CMDID_BIND                           _CMDGRP_NORMAL(0x51)
#define LTE_CMDID_CLOSE                          _CMDGRP_NORMAL(0x52)
#define LTE_CMDID_CONNECT                        _CMDGRP_NORMAL(0x53)
#define LTE_CMDID_FCNTL                          _CMDGRP_NORMAL(0x54)
#define LTE_CMDID_GETADDRINFO                    _CMDGRP_NORMAL(0x55)
#define LTE_CMDID_GETHOSTBYNAME                  _CMDGRP_NORMAL(0x56)
#define LTE_CMDID_GETHOSTBYNAMER                 _CMDGRP_NORMAL(0x57)
#define LTE_CMDID_GETSOCKNAME                    _CMDGRP_NORMAL(0x58)
#define LTE_CMDID_GETSOCKOPT                     _CMDGRP_NORMAL(0x59)
#define LTE_CMDID_LISTEN                         _CMDGRP_NORMAL(0x5a)
#define LTE_CMDID_RECV                           _CMDGRP_NORMAL(0x5b)
#define LTE_CMDID_RECVFROM                       _CMDGRP_NORMAL(0x5c)
#define LTE_CMDID_SELECT                         _CMDGRP_NORMAL(0x5d)
#define LTE_CMDID_SEND                           _CMDGRP_NORMAL(0x5e)
#define LTE_CMDID_SENDTO                         _CMDGRP_NORMAL(0x5f)
#define LTE_CMDID_SHUTDOWN                       _CMDGRP_NORMAL(0x60)
#define LTE_CMDID_SOCKET                         _CMDGRP_NORMAL(0x61)
#define LTE_CMDID_SETSOCKOPT                     _CMDGRP_NORMAL(0x62)

#define LTE_CMDID_SMS_INIT                       _CMDGRP_NORMAL(0x65)
#define LTE_CMDID_SMS_FIN                        _CMDGRP_NORMAL(0x66)
#define LTE_CMDID_SMS_SEND                       _CMDGRP_NORMAL(0x67)
#define LTE_CMDID_SMS_DELETE                     _CMDGRP_NORMAL(0x68)
#define LTE_CMDID_SMS_REPORT_RECV                _CMDGRP_EVENT(0x69)

#define LTE_CMDID_TLS_SSL_INIT                   _CMDGRP_NORMAL(0x80)
#define LTE_CMDID_TLS_SSL_FREE                   _CMDGRP_NORMAL(0x81)
#define LTE_CMDID_TLS_SSL_SETUP                  _CMDGRP_NORMAL(0x82)
#define LTE_CMDID_TLS_SSL_HOSTNAME               _CMDGRP_NORMAL(0x83)
#define LTE_CMDID_TLS_SSL_BIO                    _CMDGRP_NORMAL(0x84)
#define LTE_CMDID_TLS_SSL_HANDSHAKE              _CMDGRP_NORMAL(0x85)
#define LTE_CMDID_TLS_SSL_WRITE                  _CMDGRP_NORMAL(0x86)
#define LTE_CMDID_TLS_SSL_READ                   _CMDGRP_NORMAL(0x87)
#define LTE_CMDID_TLS_SSL_CLOSE_NOTIFY           _CMDGRP_NORMAL(0x88)
#define LTE_CMDID_TLS_SSL_VERSION                _CMDGRP_NORMAL(0x89)
#define LTE_CMDID_TLS_SSL_CIPHERSUITE            _CMDGRP_NORMAL(0x8a)
#define LTE_CMDID_TLS_SSL_CIPHERSUITE_ID         _CMDGRP_NORMAL(0x8b)
#define LTE_CMDID_TLS_SSL_RECORD_EXPANSION       _CMDGRP_NORMAL(0x8c)
#define LTE_CMDID_TLS_SSL_VERIFY_RESULT          _CMDGRP_NORMAL(0x8d)
#define LTE_CMDID_TLS_SSL_TIMER_CB               _CMDGRP_NORMAL(0x8e)
#define LTE_CMDID_TLS_SSL_PEER_CERT              _CMDGRP_NORMAL(0x8f)
#define LTE_CMDID_TLS_SSL_BYTES_AVAIL            _CMDGRP_NORMAL(0x90)
#define LTE_CMDID_TLS_CONFIG_INIT                _CMDGRP_NORMAL(0x91)
#define LTE_CMDID_TLS_CONFIG_FREE                _CMDGRP_NORMAL(0x92)
#define LTE_CMDID_TLS_CONFIG_DEFAULTS            _CMDGRP_NORMAL(0x93)
#define LTE_CMDID_TLS_CONFIG_AUTHMODE            _CMDGRP_NORMAL(0x94)
#define LTE_CMDID_TLS_CONFIG_RNG                 _CMDGRP_NORMAL(0x95)
#define LTE_CMDID_TLS_CONFIG_CA_CHAIN            _CMDGRP_NORMAL(0x96)
#define LTE_CMDID_TLS_CONFIG_OWN_CERT            _CMDGRP_NORMAL(0x97)
#define LTE_CMDID_TLS_CONFIG_READ_TIMEOUT        _CMDGRP_NORMAL(0x98)
#define LTE_CMDID_TLS_CONFIG_VERIFY              _CMDGRP_EVENT(0x99)
#define LTE_CMDID_TLS_CONFIG_VERIFY_CALLBACK     _CMDGRP_NORMAL(0x9a)
#define LTE_CMDID_TLS_CONFIG_ALPN_PROTOCOLS      _CMDGRP_NORMAL(0x9b)
#define LTE_CMDID_TLS_CONFIG_CIPHERSUITES        _CMDGRP_NORMAL(0x9c)
#define LTE_CMDID_TLS_SESSION_INIT               _CMDGRP_NORMAL(0x9d)
#define LTE_CMDID_TLS_SESSION_FREE               _CMDGRP_NORMAL(0x9e)
#define LTE_CMDID_TLS_SESSION_GET                _CMDGRP_NORMAL(0x9f)
#define LTE_CMDID_TLS_SESSION_SET                _CMDGRP_NORMAL(0xa0)
#define LTE_CMDID_TLS_SESSION_RESET              _CMDGRP_NORMAL(0xa1)
#define LTE_CMDID_TLS_X509_CRT_INIT              _CMDGRP_NORMAL(0xa2)
#define LTE_CMDID_TLS_X509_CRT_FREE              _CMDGRP_NORMAL(0xa3)
#define LTE_CMDID_TLS_X509_CRT_PARSE_FILE        _CMDGRP_NORMAL(0xa4)
#define LTE_CMDID_TLS_X509_CRT_PARSE_DER         _CMDGRP_NORMAL(0xa5)
#define LTE_CMDID_TLS_X509_CRT_PARSE             _CMDGRP_NORMAL(0xa6)
#define LTE_CMDID_TLS_X509_CRT_INFO              _CMDGRP_NORMAL(0xa7)
#define LTE_CMDID_TLS_X509_CRT_VERIFY_INFO       _CMDGRP_NORMAL(0xa8)
#define LTE_CMDID_TLS_PK_INIT                    _CMDGRP_NORMAL(0xa9)
#define LTE_CMDID_TLS_PK_FREE                    _CMDGRP_NORMAL(0xaa)
#define LTE_CMDID_TLS_PK_PARSE_KEYFILE           _CMDGRP_NORMAL(0xab)
#define LTE_CMDID_TLS_PK_PARSE_KEY               _CMDGRP_NORMAL(0xac)
#define LTE_CMDID_TLS_PK_CHECK_PAIR              _CMDGRP_NORMAL(0xad)
#define LTE_CMDID_TLS_PK_SETUP                   _CMDGRP_NORMAL(0xae)
#define LTE_CMDID_TLS_PK_INFO_FROM_TYPE          _CMDGRP_NORMAL(0xaf)
#define LTE_CMDID_TLS_PK_WRITE_KEY_PEM           _CMDGRP_NORMAL(0xb0)
#define LTE_CMDID_TLS_PK_WRITE_KEY_DER           _CMDGRP_NORMAL(0xb1)
#define LTE_CMDID_TLS_PK_RSA                     _CMDGRP_NORMAL(0xb2)
#define LTE_CMDID_TLS_CTR_DRBG_INIT              _CMDGRP_NORMAL(0xb3)
#define LTE_CMDID_TLS_CTR_DRBG_FREE              _CMDGRP_NORMAL(0xb4)
#define LTE_CMDID_TLS_CTR_DRBG_SEED              _CMDGRP_NORMAL(0xb5)
#define LTE_CMDID_TLS_ENTROPY_INIT               _CMDGRP_NORMAL(0xb6)
#define LTE_CMDID_TLS_ENTROPY_FREE               _CMDGRP_NORMAL(0xb7)
#define LTE_CMDID_TLS_CIPHER_INIT                _CMDGRP_NORMAL(0xb8)
#define LTE_CMDID_TLS_CIPHER_FREE                _CMDGRP_NORMAL(0xb9)
#define LTE_CMDID_TLS_CIPHER_INFO_FROM_STRING    _CMDGRP_NORMAL(0xba)
#define LTE_CMDID_TLS_CIPHER_SETUP               _CMDGRP_NORMAL(0xbb)
#define LTE_CMDID_TLS_CIPHER_SETKEY              _CMDGRP_NORMAL(0xbc)
#define LTE_CMDID_TLS_CIPHER_SET_IV              _CMDGRP_NORMAL(0xbd)
#define LTE_CMDID_TLS_CIPHER_UPDATE              _CMDGRP_NORMAL(0xbe)
#define LTE_CMDID_TLS_MD_INFO_FROM_TYPE          _CMDGRP_NORMAL(0xbf)
#define LTE_CMDID_TLS_MD_GET_SIZE                _CMDGRP_NORMAL(0xc0)
#define LTE_CMDID_TLS_MD                         _CMDGRP_NORMAL(0xc1)
#define LTE_CMDID_TLS_MD_DIGEST                  _CMDGRP_NORMAL(0xc2)
#define LTE_CMDID_TLS_BASE64_ENCODE              _CMDGRP_NORMAL(0xc3)
#define LTE_CMDID_TLS_SHA1                       _CMDGRP_NORMAL(0xc4)
#define LTE_CMDID_TLS_SSL_EXPORT_SRTP_KEYS       _CMDGRP_NORMAL(0xc5)
#define LTE_CMDID_TLS_SSL_USE_SRTP               _CMDGRP_NORMAL(0xc6)
#define LTE_CMDID_TLS_SSL_SRTP_PROFILE           _CMDGRP_NORMAL(0xc7)
#define LTE_CMDID_TLS_SSL_TURN                   _CMDGRP_NORMAL(0xc8)
#define LTE_CMDID_TLS_MPI_INIT                   _CMDGRP_NORMAL(0xc9)
#define LTE_CMDID_TLS_MPI_FREE                   _CMDGRP_NORMAL(0xca)
#define LTE_CMDID_TLS_MPI_READ_STRING            _CMDGRP_NORMAL(0xcb)
#define LTE_CMDID_TLS_MPI_WRITE_STRING           _CMDGRP_NORMAL(0xcc)
#define LTE_CMDID_TLS_X509_CSR_INIT              _CMDGRP_NORMAL(0xcd)
#define LTE_CMDID_TLS_X509_CSR_FREE              _CMDGRP_NORMAL(0xce)
#define LTE_CMDID_TLS_X509_CSR_PARSE_FILE        _CMDGRP_NORMAL(0xcf)
#define LTE_CMDID_TLS_X509_CSR_PARSE_DER         _CMDGRP_NORMAL(0xd0)
#define LTE_CMDID_TLS_X509_CSR_PARSE             _CMDGRP_NORMAL(0xd1)
#define LTE_CMDID_TLS_X509_DN_GETS_CRT           _CMDGRP_NORMAL(0xd2)
#define LTE_CMDID_TLS_X509_DN_GETS_CSR           _CMDGRP_NORMAL(0xd3)
#define LTE_CMDID_TLS_X509WRITE_CRT_INIT         _CMDGRP_NORMAL(0xd4)
#define LTE_CMDID_TLS_X509WRITE_CRT_FREE         _CMDGRP_NORMAL(0xd5)
#define LTE_CMDID_TLS_X509WRITE_CRT_DER          _CMDGRP_NORMAL(0xd6)
#define LTE_CMDID_TLS_X509WRITE_CRT_PEM          _CMDGRP_NORMAL(0xd7)
#define LTE_CMDID_TLS_X509WRITE_CRT_SUBJECT_KEY  _CMDGRP_NORMAL(0xd8)
#define LTE_CMDID_TLS_X509WRITE_CRT_ISSUER_KEY   _CMDGRP_NORMAL(0xd9)
#define LTE_CMDID_TLS_X509WRITE_CRT_SUBJECT_NAME _CMDGRP_NORMAL(0xda)
#define LTE_CMDID_TLS_X509WRITE_CRT_ISSUER_NAME  _CMDGRP_NORMAL(0xdb)
#define LTE_CMDID_TLS_X509WRITE_CRT_VERSION      _CMDGRP_NORMAL(0xdc)
#define LTE_CMDID_TLS_X509WRITE_CRT_MD_ALG       _CMDGRP_NORMAL(0xdd)
#define LTE_CMDID_TLS_X509WRITE_CRT_SERIAL       _CMDGRP_NORMAL(0xde)
#define LTE_CMDID_TLS_X509WRITE_CRT_VALIDITY     _CMDGRP_NORMAL(0xdf)
#define LTE_CMDID_TLS_X509WRITE_CRT_CONSTRAINTS  _CMDGRP_NORMAL(0xe0)
#define LTE_CMDID_TLS_X509WRITE_CRT_SUBJECT_ID   _CMDGRP_NORMAL(0xe1)
#define LTE_CMDID_TLS_X509WRITE_CRT_AUTHORITY_ID _CMDGRP_NORMAL(0xe2)
#define LTE_CMDID_TLS_X509WRITE_CRT_KEY_USAGE    _CMDGRP_NORMAL(0xe3)
#define LTE_CMDID_TLS_X509WRITE_CRT_NS_CERT_TYPE _CMDGRP_NORMAL(0xe4)
#define LTE_CMDID_TLS_RSA_INIT                   _CMDGRP_NORMAL(0xe5)
#define LTE_CMDID_TLS_RSA_FREE                   _CMDGRP_NORMAL(0xe6)
#define LTE_CMDID_TLS_RSA_GEN_KEY                _CMDGRP_NORMAL(0xe7)

/* for LwM2M stub */

#define LTE_CMDID_LWM2M_URC_DUMMY                _CMDGRP_EVENT(0x0101)
#define LTE_CMDID_LWM2M_READ_EVT                 _CMDGRP_EVENT(0x0102)
#define LTE_CMDID_LWM2M_WRITE_EVT                _CMDGRP_EVENT(0x0103)
#define LTE_CMDID_LWM2M_EXEC_EVT                 _CMDGRP_EVENT(0x0104)
#define LTE_CMDID_LWM2M_OVSTART_EVT              _CMDGRP_EVENT(0x0105)
#define LTE_CMDID_LWM2M_OVSTOP_EVT               _CMDGRP_EVENT(0x0106)
#define LTE_CMDID_LWM2M_SERVEROP_EVT             _CMDGRP_EVENT(0x0107)
#define LTE_CMDID_LWM2M_FWUP_EVT                 _CMDGRP_EVENT(0x0108)

#define LTE_CMDID_LWM2M_GETEP                    _CMDGRP_LWM2M(0x0109)
#define LTE_CMDID_LWM2M_SETEP                    _CMDGRP_LWM2M(0x010A)
#define LTE_CMDID_LWM2M_GETSRVNUM                _CMDGRP_LWM2M(0x010B)
#define LTE_CMDID_LWM2M_GETSRVINFO               _CMDGRP_LWM2M(0x010C)
#define LTE_CMDID_LWM2M_SETSRVINFO               _CMDGRP_LWM2M(0x010D)
#define LTE_CMDID_LWM2M_GETACTIVEOBJNUM          _CMDGRP_LWM2M(0x010E)
#define LTE_CMDID_LWM2M_GETACTIVEOBJ             _CMDGRP_LWM2M(0x010F)
#define LTE_CMDID_LWM2M_SETACTIVEOBJ             _CMDGRP_LWM2M(0x0110)
#define LTE_CMDID_LWM2M_APPLY_SETTING            _CMDGRP_LWM2M(0x0111)
#define LTE_CMDID_LWM2M_GETOBJRESNUM             _CMDGRP_LWM2M(0x0112)
#define LTE_CMDID_LWM2M_GETOBJRESOURCE           _CMDGRP_LWM2M(0x0113)
#define LTE_CMDID_LWM2M_SETOBJRESOURCE           _CMDGRP_LWM2M(0x0114)

#define LTE_CMDID_LWM2M_CONNECT                  _CMDGRP_LWM2M(0x0115)
#define LTE_CMDID_LWM2M_READRESP                 _CMDGRP_LWM2M(0x0116)
#define LTE_CMDID_LWM2M_WRITERESP                _CMDGRP_LWM2M(0x0117)
#define LTE_CMDID_LWM2M_OBSERVERESP              _CMDGRP_LWM2M(0x0118)
#define LTE_CMDID_LWM2M_EXECRESP                 _CMDGRP_LWM2M(0x0119)
#define LTE_CMDID_LWM2M_OBSERVEUPDATE            _CMDGRP_LWM2M(0x011A)

#define IS_LWM2M_EVENT(cid) (\
  ((cid) == LTE_CMDID_LWM2M_READ_EVT) ||  \
  ((cid) == LTE_CMDID_LWM2M_WRITE_EVT) || \
  ((cid) == LTE_CMDID_LWM2M_EXEC_EVT) ||  \
  ((cid) == LTE_CMDID_LWM2M_OVSTART_EVT) || \
  ((cid) == LTE_CMDID_LWM2M_OVSTOP_EVT) ||  \
  ((cid) == LTE_CMDID_LWM2M_SERVEROP_EVT) ||  \
  ((cid) == LTE_CMDID_LWM2M_FWUP_EVT) )

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*lte_evthndl_t)(uint64_t evtbitmap);

/* for SIOCLTECMD IOCTL command */

struct lte_ioctl_data_s
{
  uint32_t cmdid;                   /* Command ID */
  FAR void **inparam;               /* Pointer to input parameter */
  size_t inparamlen;                /* Length of input parameter */
  FAR void **outparam;              /* Pointer to output parameter */
  size_t outparamlen;               /* Length of output parameter */
  FAR void *cb;                     /* Pointer to callback function */
};

struct lte_evtctx_in_s
{
  FAR const char *mqname;
};

struct lte_evtctx_out_s
{
  lte_evthndl_t handle;
};

/* for IOCTL related SMS */

struct lte_smsreq_s
{
  union
    {
      /* Using for SIOCSMSENSTREP command */

      bool enable;

      /* Using for SIOCSMSGREFID command */

      struct sms_refids_s refid;

      /* Using for SIOCSMSSSCA command */

      struct sms_sc_addr_s scaddr;
    } smsru;
};

#endif /* __INCLUDE_NUTTX_WIRELESS_LTE_LTE_IOCTL_H */

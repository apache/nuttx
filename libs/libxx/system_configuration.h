/****************************************************************************
 * libs/libxx/system_configuration.h
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

#ifndef SYSTEM_CONFIGURATION_H
#define SYSTEM_CONFIGURATION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Version Number */

#define __UCLIBCXX_MAJOR__ 0
#define __UCLIBCXX_MINOR__ 2
#define __UCLIBCXX_SUBLEVEL__ 6-git

/* Target Features and Options */

#ifdef CONFIG_HAVE_FLOAT
#  define __UCLIBCXX_HAS_FLOATS__ 1
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
#  define __UCLIBCXX_HAS_LONG_DOUBLE__ 1
#endif
#undef __UCLIBCXX_HAS_TLS__
#define __UCLIBCXX_WARNINGS__ "-Wall"
#define __BUILD_EXTRA_LIBRARIES__ ""
#define __HAVE_DOT_CONFIG__ 1

/* String and I/O Stream Support */

#ifdef CONFIG_UCLIBCXX_WCHAR
#  define __UCLIBCXX_HAS_WCHAR__ 1
#endif
#define __UCLIBCXX_IOSTREAM_BUFSIZE__ CONFIG_UCLIBCXX_BUFSIZE
#undef __UCLIBCXX_HAS_LFS__
#define __UCLIBCXX_SUPPORT_CDIR__ 1
#define __UCLIBCXX_SUPPORT_CIN__ 1
#define __UCLIBCXX_SUPPORT_COUT__ 1
#define __UCLIBCXX_SUPPORT_CERR__ 1
#define __UCLIBCXX_SUPPORT_CLOG__ 1
#ifdef CONFIG_UCLIBCXX_WCHAR
#  define __UCLIBCXX_SUPPORT_WCIN__ 1
#  define __UCLIBCXX_SUPPORT_WCOUT__ 1
#  define __UCLIBCXX_SUPPORT_WCERR__ 1
#  define __UCLIBCXX_SUPPORT_WCLOG__ 1
#endif

/* STL and Code Expansion */

#define __UCLIBCXX_STL_BUFFER_SIZE__ CONFIG_UCLIBCXX_BUFSIZE
#define __UCLIBCXX_CODE_EXPANSION__ 1
#define __UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__ 1
#define __UCLIBCXX_EXPAND_STRING_CHAR__ 1
#define __UCLIBCXX_EXPAND_VECTOR_BASIC__ 1
#define __UCLIBCXX_EXPAND_IOS_CHAR__ 1
#define __UCLIBCXX_EXPAND_STREAMBUF_CHAR__ 1
#define __UCLIBCXX_EXPAND_ISTREAM_CHAR__ 1
#define __UCLIBCXX_EXPAND_OSTREAM_CHAR__ 1
#define __UCLIBCXX_EXPAND_FSTREAM_CHAR__ 1
#define __UCLIBCXX_EXPAND_SSTREAM_CHAR__ 1

/* Library Installation Options */

#define __UCLIBCXX_RUNTIME_PREFIX__ "/usr/uClibc++"
#define __UCLIBCXX_RUNTIME_INCLUDE_SUBDIR__ "/include"
#define __UCLIBCXX_RUNTIME_LIB_SUBDIR__ "/lib"
#define __UCLIBCXX_RUNTIME_BIN_SUBDIR__ "/bin"
#ifdef CONFIG_CXX_EXCEPTION
#  define __UCLIBCXX_EXCEPTION_SUPPORT__ 1
#endif
#undef __IMPORT_LIBSUP__
#undef __IMPORT_LIBGCC_EH__
#define __BUILD_STATIC_LIB__ 1
#define __BUILD_ONLY_STATIC_LIB__
#ifdef CONFIG_DEBUG_ASSERTIONS
#  define __DODEBUG__ 1
#endif

#endif /* SYSTEM_CONFIGURATION_H */

/****************************************************************************
 * include/nuttx/trace.h
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

#ifndef __INCLUDE_NUTTX_TRACE_H
#define __INCLUDE_NUTTX_TRACE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sched_note.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_TRACE
#  define trace_begin(tag) SCHED_NOTE_BEGIN(tag)
#  define trace_end(tag) SCHED_NOTE_END(tag)
#else
#  define trace_begin(tag)
#  define trace_end(tag)
#endif

#ifdef CONFIG_TRACE_APP
#  define app_trace_begin() trace_begin(NOTE_TAG_APP)
#  define app_trace_end() trace_end(NOTE_TAG_APP)
#else
#  define app_trace_begin()
#  define app_trace_end()
#endif

#ifdef CONFIG_TRACE_ARCH
#  define arch_trace_begin() trace_begin(NOTE_TAG_ARCH)
#  define arch_trace_end() trace_end(NOTE_TAG_ARCH)
#else
#  define arch_trace_begin()
#  define arch_trace_end()
#endif

#ifdef CONFIG_TRACE_AUDIO
#  define audio_trace_begin() trace_begin(NOTE_TAG_AUDIO)
#  define audio_trace_end() trace_end(NOTE_TAG_AUDIO)
#else
#  define audio_trace_begin()
#  define audio_trace_end()
#endif

#ifdef CONFIG_TRACE_BOARDS
#  define boards_trace_begin() trace_begin(NOTE_TAG_BOARDS)
#  define boards_trace_end() trace_end(NOTE_TAG_BOARDS)
#else
#  define boards_trace_begin()
#  define boards_trace_end()
#endif

#ifdef CONFIG_TRACE_CRYPTO
#  define crypto_trace_begin() trace_begin(NOTE_TAG_CRYPTO)
#  define crypto_trace_end() trace_end(NOTE_TAG_CRYPTO)
#else
#  define crypto_trace_begin()
#  define crypto_trace_end()
#endif

#ifdef CONFIG_TRACE_DRIVERS
#  define drivers_trace_begin() trace_begin(NOTE_TAG_DRIVERS)
#  define drivers_trace_end() trace_end(NOTE_TAG_DRIVERS)
#else
#  define drivers_trace_begin()
#  define drivers_trace_end()
#endif

#ifdef CONFIG_TRACE_FS
#  define fs_trace_begin() trace_begin(NOTE_TAG_FS)
#  define fs_trace_end() trace_end(NOTE_TAG_FS)
#else
#  define fs_trace_begin()
#  define fs_trace_end()
#endif

#ifdef CONFIG_TRACE_GRAPHICS
#  define graphics_trace_begin() trace_begin(NOTE_TAG_GRAPHICS)
#  define graphics_trace_end() trace_end(NOTE_TAG_GRAPHICS)
#else
#  define graphics_trace_begin()
#  define graphics_trace_end()
#endif

#ifdef CONFIG_TRACE_LIBS
#  define libs_trace_begin() trace_begin(NOTE_TAG_LIBS)
#  define libs_trace_end() trace_end(NOTE_TAG_LIBS)
#else
#  define libs_trace_begin()
#  define libs_trace_end()
#endif

#ifdef CONFIG_TRACE_MM
#  define mm_trace_begin() trace_begin(NOTE_TAG_MM)
#  define mm_trace_end() trace_end(NOTE_TAG_MM)
#else
#  define mm_trace_begin()
#  define mm_trace_end()
#endif

#ifdef CONFIG_TRACE_NET
#  define net_trace_begin() trace_begin(NOTE_TAG_NET)
#  define net_trace_end() trace_end(NOTE_TAG_NET)
#else
#  define net_trace_begin()
#  define net_trace_end()
#endif

#ifdef CONFIG_TRACE_SCHED
#  define sched_trace_begin() trace_begin(NOTE_TAG_SCHED)
#  define sched_trace_end() trace_end(NOTE_TAG_SCHED)
#else
#  define sched_trace_begin()
#  define sched_trace_end()
#endif

#ifdef CONFIG_TRACE_VIDEO
#  define video_trace_begin() trace_begin(NOTE_TAG_VIDEO)
#  define video_trace_end() trace_end(NOTE_TAG_VIDEO)
#else
#  define video_trace_begin()
#  define video_trace_end()
#endif

#ifdef CONFIG_TRACE_WIRELESS
#  define wireless_trace_begin() trace_begin(NOTE_TAG_WIRLESS)
#  define wireless_trace_end() trace_end(NOTE_TAG_WIRLESS)
#else
#  define wireless_trace_begin()
#  define wireless_trace_end()
#endif

#endif /* __INCLUDE_NUTTX_TRACE_H */

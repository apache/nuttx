/****************************************************************************
 * include/nuttx/trace.h
 *
 * SPDX-License-Identifier: Apache-2.0
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
#  define trace_begin(tag) sched_note_begin(tag)
#  define trace_end(tag) sched_note_end(tag)
#  define trace_beginex(tag, name) sched_note_beginex(tag, name)
#  define trace_endex(tag, name) sched_note_endex(tag, name)
#  define trace_mark(tag, s) sched_note_mark(tag, s)
#  define trace_printf(tag, fmt, ...) sched_note_printf(tag, fmt, ##__VA_ARGS__)
#else
#  define trace_begin(tag)
#  define trace_end(tag)
#  define trace_beginex(tag, name)
#  define trace_endex(tag, name)
#  define trace_mark(tag, s)
#  define trace_printf(...)
#endif

#ifdef CONFIG_TRACE_APP
#  define app_trace_begin() trace_begin(NOTE_TAG_APP)
#  define app_trace_end() trace_end(NOTE_TAG_APP)
#  define app_trace_beginex(name) trace_beginex(NOTE_TAG_APP, name)
#  define app_trace_endex(name) trace_endex(NOTE_TAG_APP, name)
#  define app_trace_mark(s) trace_mark(NOTE_TAG_APP, s)
#  define app_trace_printf(fmt, ...) sched_note_printf(NOTE_TAG_APP, fmt, ##__VA_ARGS__)
#else
#  define app_trace_begin()
#  define app_trace_end()
#  define app_trace_beginex(name)
#  define app_trace_endex(name)
#  define app_trace_mark(s)
#  define app_trace_printf(...)
#endif

#ifdef CONFIG_TRACE_ARCH
#  define arch_trace_begin() trace_begin(NOTE_TAG_ARCH)
#  define arch_trace_end() trace_end(NOTE_TAG_ARCH)
#  define arch_trace_beginex(name) trace_beginex(NOTE_TAG_ARCH, name)
#  define arch_trace_endex(name) trace_endex(NOTE_TAG_ARCH, name)
#  define arch_trace_mark(s) trace_mark(NOTE_TAG_ARCH, s)
#  define arch_trace_printf(fmt, ...) \
    trace_printf(NOTE_TAG_ARCH, fmt, ##__VA_ARGS__)
#else
#  define arch_trace_begin()
#  define arch_trace_end()
#  define arch_trace_beginex(name)
#  define arch_trace_endex(name)
#  define arch_trace_mark(s)
#  define arch_trace_printf(...)
#endif

#ifdef CONFIG_TRACE_AUDIO
#  define audio_trace_begin() trace_begin(NOTE_TAG_AUDIO)
#  define audio_trace_end() trace_end(NOTE_TAG_AUDIO)
#  define audio_trace_beginex(name) trace_beginex(NOTE_TAG_AUDIO, name)
#  define audio_trace_endex(name) trace_endex(NOTE_TAG_AUDIO, name)
#  define audio_trace_mark(s) trace_mark(NOTE_TAG_AUDIO, s)
#  define audio_trace_printf(fmt, ...) \
    trace_printf(NOTE_TAG_AUDIO, fmt, ##__VA_ARGS__)
#else
#  define audio_trace_begin()
#  define audio_trace_end()
#  define audio_trace_beginex(name)
#  define audio_trace_endex(name)
#  define audio_trace_mark(s)
#  define audio_trace_printf(...)
#endif

#ifdef CONFIG_TRACE_BOARDS
#  define boards_trace_begin() trace_begin(NOTE_TAG_BOARDS)
#  define boards_trace_end() trace_end(NOTE_TAG_BOARDS)
#  define boards_trace_beginex(name) trace_beginex(NOTE_TAG_BOARDS, name)
#  define boards_trace_endex(name) trace_endex(NOTE_TAG_BOARDS, name)
#  define boards_trace_mark(s) trace_mark(NOTE_TAG_BOARDS, s)
#  define boards_trace_printf(fmt, ...) \
    trace_printf(NOTE_TAG_BOARDS, fmt, ##__VA_ARGS__)
#else
#  define boards_trace_begin()
#  define boards_trace_end()
#  define boards_trace_beginex(name)
#  define boards_trace_endex(name)
#  define boards_trace_mark(s)
#  define boards_trace_printf(...)
#endif

#ifdef CONFIG_TRACE_CRYPTO
#  define crypto_trace_begin() trace_begin(NOTE_TAG_CRYPTO)
#  define crypto_trace_end() trace_end(NOTE_TAG_CRYPTO)
#  define crypto_trace_beginex(name) trace_beginex(NOTE_TAG_CRYPTO, name)
#  define crypto_trace_endex(name) trace_endex(NOTE_TAG_CRYPTO, name)
#  define crypto_trace_mark(s) trace_mark(NOTE_TAG_CRYPTO, s)
#  define crypto_trace_printf(fmt, ...) \
    trace_printf(NOTE_TAG_CRYPTO, fmt, ##__VA_ARGS__)
#else
#  define crypto_trace_begin()
#  define crypto_trace_end()
#  define crypto_trace_beginex(name)
#  define crypto_trace_endex(name)
#  define crypto_trace_mark(s)
#  define crypto_trace_printf(...)
#endif

#ifdef CONFIG_TRACE_DRIVERS
#  define drivers_trace_begin() trace_begin(NOTE_TAG_DRIVERS)
#  define drivers_trace_end() trace_end(NOTE_TAG_DRIVERS)
#  define drivers_trace_beginex(name) trace_beginex(NOTE_TAG_DRIVERS, name)
#  define drivers_trace_endex(name) trace_endex(NOTE_TAG_DRIVERS, name)
#  define drivers_trace_mark(s) trace_mark(NOTE_TAG_DRIVERS, s)
#  define drivers_trace_printf(fmt, ...) \
    trace_printf(NOTE_TAG_DRIVERS, fmt, )
#else
#  define drivers_trace_begin()
#  define drivers_trace_end()
#  define drivers_trace_beginex(name)
#  define drivers_trace_endex(name)
#  define drivers_trace_mark(s)
#  define drivers_trace_printf(...)
#endif

#ifdef CONFIG_TRACE_FS
#  define fs_trace_begin() trace_begin(NOTE_TAG_FS)
#  define fs_trace_end() trace_end(NOTE_TAG_FS)
#  define fs_trace_beginex(name) trace_beginex(NOTE_TAG_FS, name)
#  define fs_trace_endex(name) trace_endex(NOTE_TAG_FS, name)
#  define fs_trace_mark(s) trace_mark(NOTE_TAG_FS, s)
#  define fs_trace_printf(fmt, ...) \
    trace_printf(NOTE_TAG_FS, fmt, ##__VA_ARGS__)
#else
#  define fs_trace_begin()
#  define fs_trace_end()
#  define fs_trace_beginex(name)
#  define fs_trace_endex(name)
#  define fs_trace_mark(s)
#  define fs_trace_printf(...)
#endif

#ifdef CONFIG_TRACE_GRAPHICS
#  define graphics_trace_begin() trace_begin(NOTE_TAG_GRAPHICS)
#  define graphics_trace_end() trace_end(NOTE_TAG_GRAPHICS)
#  define graphics_trace_beginex(name) trace_beginex(NOTE_TAG_GRAPHICS, name)
#  define graphics_trace_endex(name) trace_endex(NOTE_TAG_GRAPHICS, name)
#  define graphics_trace_mark(s) trace_mark(NOTE_TAG_GRAPHICS, s)
#  define graphics_trace_printf(fmt, ...) \
    trace_printf(NOTE_TAG_GRAPHICS, fmt, ##__VA_ARGS__)
#else
#  define graphics_trace_begin()
#  define graphics_trace_end()
#  define graphics_trace_beginex(name)
#  define graphics_trace_endex(name)
#  define graphics_trace_mark(s)
#  define graphics_trace_printf(...)
#endif

#ifdef CONFIG_TRACE_INPUT
#  define input_trace_begin() trace_begin(NOTE_TAG_INPUT)
#  define input_trace_end() trace_end(NOTE_TAG_INPUT)
#  define input_trace_beginex(name) trace_beginex(NOTE_TAG_INPUT, name)
#  define input_trace_endex(name) trace_endex(NOTE_TAG_INPUT, name)
#  define input_trace_mark(s) trace_mark(NOTE_TAG_INPUT, s)
#  define input_trace_printf(fmt, ...) \
    trace_printf(NOTE_TAG_INPUT, fmt, ##__VA_ARGS__)
#else
#  define input_trace_begin()
#  define input_trace_end()
#  define input_trace_beginex(name)
#  define input_trace_endex(name)
#  define input_trace_mark(s)
#  define input_trace_printf(...)
#endif

#ifdef CONFIG_TRACE_LIBS
#  define libs_trace_begin() trace_begin(NOTE_TAG_LIBS)
#  define libs_trace_end() trace_end(NOTE_TAG_LIBS)
#  define libs_trace_beginex(name) trace_beginex(NOTE_TAG_LIBS, name)
#  define libs_trace_endex(name) trace_endex(NOTE_TAG_LIBS, name)
#  define libs_trace_mark(s) trace_mark(NOTE_TAG_LIBS, s)
#  define libs_trace_printf(fmt, ...) \
    trace_printf(NOTE_TAG_LIBS, fmt, ##__VA_ARGS__)
#else
#  define libs_trace_begin()
#  define libs_trace_end()
#  define libs_trace_beginex(name)
#  define libs_trace_endex(name)
#  define libs_trace_mark(s)
#  define libs_trace_printf(...)
#endif

#ifdef CONFIG_TRACE_MM
#  define mm_trace_begin() trace_begin(NOTE_TAG_MM)
#  define mm_trace_end() trace_end(NOTE_TAG_MM)
#  define mm_trace_beginex(name) trace_beginex(NOTE_TAG_MM, name)
#  define mm_trace_endex(name) trace_endex(NOTE_TAG_MM, name)
#  define mm_trace_mark(s) trace_mark(NOTE_TAG_MM, s)
#  define mm_trace_printf(fmt, ...) \
    trace_printf(NOTE_TAG_MM, fmt, ##__VA_ARGS__)
#else
#  define mm_trace_begin()
#  define mm_trace_end()
#  define mm_trace_beginex(name)
#  define mm_trace_endex(name)
#  define mm_trace_mark(s)
#  define mm_trace_printf(...)
#endif

#ifdef CONFIG_TRACE_NET
#  define net_trace_begin() trace_begin(NOTE_TAG_NET)
#  define net_trace_end() trace_end(NOTE_TAG_NET)
#  define net_trace_beginex(name) trace_beginex(NOTE_TAG_NET, name)
#  define net_trace_endex(name) trace_endex(NOTE_TAG_NET, name)
#  define net_trace_mark(s) trace_mark(NOTE_TAG_NET, s)
#  define net_trace_printf(fmt, ...) \
    trace_printf(NOTE_TAG_NET, fmt, ##__VA_ARGS__)
#else
#  define net_trace_begin()
#  define net_trace_end()
#  define net_trace_beginex(name)
#  define net_trace_endex(name)
#  define net_trace_mark(s)
#  define net_trace_printf(...)
#endif

#ifdef CONFIG_TRACE_SCHED
#  define sched_trace_begin() trace_begin(NOTE_TAG_SCHED)
#  define sched_trace_end() trace_end(NOTE_TAG_SCHED)
#  define sched_trace_beginex(name) trace_beginex(NOTE_TAG_SCHED, name)
#  define sched_trace_endex(name) trace_endex(NOTE_TAG_SCHED, name)
#  define sched_trace_mark(s) trace_mark(NOTE_TAG_SCHED, s)
#  define sched_trace_printf(fmt, ...) \
    trace_printf(NOTE_TAG_SCHED, fmt, ##__VA_ARGS__)
#else
#  define sched_trace_begin()
#  define sched_trace_end()
#  define sched_trace_beginex(name)
#  define sched_trace_endex(name)
#  define sched_trace_mark(s)
#  define sched_trace_printf(...)
#endif

#ifdef CONFIG_TRACE_VIDEO
#  define video_trace_begin() trace_begin(NOTE_TAG_VIDEO)
#  define video_trace_end() trace_end(NOTE_TAG_VIDEO)
#  define video_trace_beginex(name) trace_beginex(NOTE_TAG_VIDEO, name)
#  define video_trace_endex(name) trace_endex(NOTE_TAG_VIDEO, name)
#  define video_trace_mark(s) trace_mark(NOTE_TAG_VIDEO, s)
#  define video_trace_printf(fmt, ...) \
    trace_printf(NOTE_TAG_VIDEO, fmt, ##__VA_ARGS__)
#else
#  define video_trace_begin()
#  define video_trace_end()
#  define video_trace_beginex(name)
#  define video_trace_endex(name)
#  define video_trace_mark(s)
#  define video_trace_printf(...)
#endif

#ifdef CONFIG_TRACE_WIRELESS
#  define wireless_trace_begin() trace_begin(NOTE_TAG_WIRLESS)
#  define wireless_trace_end() trace_end(NOTE_TAG_WIRLESS)
#  define wireless_trace_beginex(name) trace_beginex(NOTE_TAG_WIRLESS, name)
#  define wireless_trace_endex(name) trace_endex(NOTE_TAG_WIRLESS, name)
#  define wireless_trace_mark(s) trace_mark(NOTE_TAG_WIRLESS, s)
#  define wireless_trace_printf(fmt, ...) \
    trace_printf(NOTE_TAG_WIRLESS, fmt, ##__VA_ARGS__)
#else
#  define wireless_trace_begin()
#  define wireless_trace_end()
#  define wireless_trace_beginex(name)
#  define wireless_trace_endex(name)
#  define wireless_trace_mark(s)
#  define wireless_trace_printf(...)
#endif

#endif /* __INCLUDE_NUTTX_TRACE_H */

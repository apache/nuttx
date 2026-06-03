/****************************************************************************
 * include/sys/tree_type.h
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * SPDX-FileCopyrightText: 2002 Niels Provos <provos@citi.umich.edu>
 *
 * Subset of tree.h containing only type definitions for use in headers
 * that need RB_ENTRY/SPLAY_ENTRY without full tree implementation.
 * This avoids conflicts with user-space tree.h.
 *
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_SYS_TREE_TYPE_H
#define __INCLUDE_SYS_TREE_TYPE_H

/****************************************************************************
 * Splay Tree Type Definitions
 ****************************************************************************/

#define SPLAY_HEAD(name, type) \
  struct name \
  { \
    struct type *sph_root; /* root of the tree */ \
  }

#define SPLAY_ENTRY(type) \
  struct \
  { \
    struct type *spe_left;  /* left element */ \
    struct type *spe_right; /* right element */ \
  }

/****************************************************************************
 * Red-Black Tree Type Definitions
 ****************************************************************************/

#define RB_HEAD(name, type) \
  struct name \
  { \
    struct type *rbh_root; /* root of the tree */ \
  }

#define RB_BLACK                       0
#define RB_RED                         1

#define RB_ENTRY(type) \
  struct \
  { \
    struct type *rbe_left;    /* left element */ \
    struct type *rbe_right;   /* right element */ \
    struct type *rbe_parent;  /* parent element */ \
    int rbe_color;            /* node color */ \
  }

#endif /* __INCLUDE_SYS_TREE_TYPE_H */

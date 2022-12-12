/****************************************************************************
 * include/obstack.h
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

/* This is based on the GlibC API but the implementation is not exactly same.
 * The major difference is how required memory is allocated. The GlibC
 * implementation starts with 4KB of allocated space. That would make it
 * impossible to use this on MCUs. This implementation rather tries to
 * allocated only required amount of space and it won't allocate chunk unless
 * grow functions are used and even then it uses realloc to release unused
 * space. It also in default won't use 4KB per chunk but rather just BUFSIZ.
 *
 * Not implemented interface:
 *   obstack_alignment_mask:
 *     The current implementation does not provide any alignment guaranties.
 *   obstack_chunk_alloc and obstack_chunk_free:
 *     Internal implementation uses not only alloc and free but also realloc
 *     and thus standard implementations are used unconditionally instead of
 *     requiring users to provide declaration for these functions.
 */

#ifndef __INCLUDE_OBSTACK_H
#define __INCLUDE_OBSTACK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <stdarg.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: obstack_chunk_size
 *
 * Description:
 *   The access to the obstack configuration specifying the size of the
 *   single chunk used when growing object.
 *   It is documented t hat this is macro and that it is possible to use
 *   assignment to change the chunk size (eq.: obstack_chunk_size(h) = 1024).
 *
 *   The default chunk size is set to BUFSIZ.
 *
 *   The chunks size has to be always power of two due to the limitations of
 *   the obstack_make_room implementation!
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *
 * Returned Value:
 *   Size of the single chunk.
 *
 ****************************************************************************/
#define obstack_chunk_size(h) ((h)->chunk_size)

/****************************************************************************
 * Name: obstack_base
 *
 * Description:
 *   Provides access to the tentative starting address of the
 *   currently growing object.
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *
 * Returned Value:
 *   Tentative starting address of the currently growing object.
 *
 ****************************************************************************/
#define obstack_base(h) ((h)->object_base)

/****************************************************************************
 * Name: obstack_next_free
 *
 * Description:
 *   Provides access to the tentative address just after the end of the
 *   currently growing object.
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *
 * Returned Value:
 *   Address just after the end of the currently growing object.
 *
 ****************************************************************************/
#define obstack_next_free(h) ((h)->next_free)

/****************************************************************************
 * Name: obstack_blank_fast
 *
 * Description:
 *   Moves the end of the currently growing object by given size and thus
 *   adding given number of uninitialized bytes to the growing object.
 *   There is no check if there is enough room and thus it is easy to cause
 *   buffer overrun. Use only when you are sure that there is enough room!
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *   size: number of bytes
 *
 * Returned Value:
 *   The new address just after the end of the currently growing object.
 *
 ****************************************************************************/
#define obstack_blank_fast(h, size) ((h)->next_free += (size))

/****************************************************************************
 * Name: obstack_1grow_fast
 *
 * Description:
 *   Adds one byte to the currently growing object.
 *   There is no check if there is enough room and thus it is easy to cause
 *   buffer overrun. Use only when you are sure that there is enough room!
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *   data: byte to be added
 *
 * Returned Value:
 *   Added byte.
 *
 ****************************************************************************/
#define obstack_1grow_fast(h, data) (*((h)->next_free++) = (data))

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct _obstack_chunk               /* Chunk head. */
{
  FAR char *limit;                  /* Address of char after this chunk */
  FAR struct _obstack_chunk *prev;  /* Address of prior chunk or NULL */
};

struct obstack
{
  size_t chunk_size;                /* Preferred size to allocate chunks in */
  FAR struct _obstack_chunk *chunk; /* Address of current struct _obstack_chunk */
  FAR char *object_base;            /* Address of object we are building */
  FAR char *next_free;              /* Where to add next char to current object */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: obstack_init
 *
 * Description:
 *   Initialize obstack for allocation of objects.
 *   Compared to the GlibC version this won't initialize a first chunk.
 *
 * Input Parameters:
 *   h: pointer to the handle to initialize
 *
 ****************************************************************************/

void obstack_init(FAR struct obstack *h);

/****************************************************************************
 * Name: obstack_alloc
 *
 * Description:
 *   Allocate an object of given size with uninitialized bytes.
 *   Compared to the GlibC version this uses malloc to allocate exactly
 *   required space (plus overhead) and nothing more.
 *
 * Input Parameters:
 *   h: pointer to the handle to allocate an object in
 *   size: number of bytes to allocate
 *
 ****************************************************************************/

FAR void *obstack_alloc(FAR struct obstack *h, size_t size);

/****************************************************************************
 * Name: obstack_copy
 *
 * Description:
 *   Allocate an object of given size with contents copied from address.
 *   The same remarks regarding the allocation apply here as for
 *   obstack_alloc.
 *
 * Input Parameters:
 *   h: pointer to the handle to allocate an object in
 *   address: pointer to the bytes to be used to initialize new object
 *   size: number of bytes to allocate
 *
 ****************************************************************************/

FAR void *obstack_copy(FAR struct obstack *h,
                       FAR const void *address, size_t size);

/****************************************************************************
 * Name: obstack_copy0
 *
 * Description:
 *   Allocate an object of given size+1 with contents copied from address and
 *   append null byte at the end.
 *   The same remarks regarding the allocation apply here as for
 *   obstack_alloc.
 *
 * Input Parameters:
 *   h: pointer to the handle to allocate an object in
 *   address: pointer to the bytes to be used to initialize new object
 *   size: number of bytes to allocate (excluding the null byte)
 *
 ****************************************************************************/

FAR void *obstack_copy0(FAR struct obstack *h,
                        FAR const void *address, size_t size);

/****************************************************************************
 * Name: obstack_free
 *
 * Description:
 *   Free objects (and everything allocated in the specified obstack more
 *   recently than object). You can pass NULL to free everything.
 *   The buffer the allocated object was preset is kept and thus can be
 *   immediately reused for growing. The only exception for this is when NULL
 *   is passed as in such case even the last buffer is freed.
 *
 * Input Parameters:
 *   h: pointer to the handle object belongs to
 *   object: the pointer to the object or NULL
 *
 ****************************************************************************/

void obstack_free(FAR struct obstack *h, FAR void *object);

/****************************************************************************
 * Name: obstack_make_room
 *
 * Description:
 *   This is non-standard function that is probably available only on NuttX!
 *   Make sure that there is room in the buffer to fit object with given
 *   size. The allocation performed is in multiples of chunk_size specified
 *   for the obstack.
 *
 * Input Parameters:
 *   h: pointer to the handle where room should be made
 *   size: number of bytes to be free for growth
 *
 * Assumptions/Limitations:
 *   The obstack's chunk_size is expected to be power of two. This helps to
 *   eliminate division that might not be implemented in the HW and thus
 *   inefficient.
 *
 ****************************************************************************/

void obstack_make_room(struct obstack *h, size_t size);

/****************************************************************************
 * Name: obstack_blank
 *
 * Description:
 *   Grow object by given size. The bytes are uninitialized.
 *
 * Input Parameters:
 *   h: pointer to the handle to allocated object to
 *   size: number of bytes to grow object
 *
 ****************************************************************************/

void obstack_blank(FAR struct obstack *h, size_t size);

/****************************************************************************
 * Name: obstack_grow
 *
 * Description:
 *   Grow object by given size and allocated it with bytes from address.
 *
 * Input Parameters:
 *   h: pointer to the handle to allocated object to
 *   address: pointer to the bytes to be used to initialize the new object
 *   size: number of bytes to grow object
 *
 ****************************************************************************/

void obstack_grow(FAR struct obstack *h,
                  FAR const void *address, size_t size);

/****************************************************************************
 * Name: obstack_grow0
 *
 * Description:
 *   Grow object by given size+1 and allocated it with bytes from address
 *   plus null byte at the end.
 *
 * Input Parameters:
 *   h: pointer to the handle to allocated object to
 *   address: pointer to the bytes to be used to initialize the new object
 *   size: number of bytes to grow object (excluding the null byte)
 *
 ****************************************************************************/

void obstack_grow0(FAR struct obstack *h,
                   FAR const void *address, size_t size);

/****************************************************************************
 * Name: obstack_1grow
 *
 * Description:
 *   Grow object by single data byte.
 *
 * Input Parameters:
 *   h: pointer to the handle to allocated object to
 *   data: byte to be added to the growing object
 *
 ****************************************************************************/

void obstack_1grow(FAR struct obstack *h, char data);

/****************************************************************************
 * Name: obstack_finish
 *
 * Description:
 *   Finish growing object and receive address to it.
 *   Compared to the GlibC version this uses realloc to reduce buffer size to
 *   only allocated amount. The non-standard obstack_finish_norealloc can be
 *   used if you want the standard behavior for ever reason.
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *
 * Returned Value:
 *   Permanent address to the object.
 *
 ****************************************************************************/

FAR void *obstack_finish(FAR struct obstack *h);

/****************************************************************************
 * Name: obstack_finish_norealloc
 *
 * Description:
 *   Finish growing object and receive address to it without reallocating
 *   buffer to fit the object (keeping space for more growth).
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *
 * Returned Value:
 *   Permanent address to the object.
 *
 ****************************************************************************/

FAR void *obstack_finish_norealloc(FAR struct obstack *h);

/****************************************************************************
 * Name: obstack_object_size
 *
 * Description:
 *   Calculate the size of the currently growing object.
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *
 * Returned Value:
 *   Size of the object.
 *
 ****************************************************************************/

size_t obstack_object_size(FAR struct obstack *h);

/****************************************************************************
 * Name: obstack_room
 *
 * Description:
 *   Calculate the number of bytes available for growth before reallocation
 *   is required.
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *
 * Returned Value:
 *   Number of free bytes.
 *
 ****************************************************************************/

size_t obstack_room(FAR struct obstack *h);

/****************************************************************************
 * Name: obstack_printf
 *
 * Description:
 *   This is similar to the asprintf except it uses obstack to allocate
 *   string on. The characters are written onto the end of the currently
 *   growing object and terminated by null byte.
 *
 *   This function is defined in stdio.h in GlibC. There is no definition
 *   that would be in stdio.h required for these here and thus it is easier
 *   to just keep these functions here as user has to include obstack anyway
 *   to get the full functionality.
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *   fmt: format string with its format inputs followed.
 *
 * Returned Value:
 *   Number of characters added to the obstack excluding the null byte.
 *
 ****************************************************************************/

int obstack_printf(FAR struct obstack *h, FAR const char *fmt, ...)
    printf_like(2, 3);

/****************************************************************************
 * Name: obstack_vprintf
 *
 * Description:
 *   This is similar to the vasprintf except it uses obstack to allocate
 *   string on. The characters are written onto the end of the currently
 *   growing object and terminated by null byte.
 *
 *   The same remarks are applied here as for obstack_printf regarding the
 *   definition location in GlibC.
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *   fmt: format string
 *   ap: format string input as a variable argument list
 *
 * Returned Value:
 *   Number of characters added to the obstack excluding the null byte.
 *
 ****************************************************************************/

int obstack_vprintf(FAR struct obstack *h, FAR const char *fmt, va_list ap)
    printf_like(2, 0);

/****************************************************************************
 * Name: obstack_alloc_failed_handler
 *
 * Description:
 *   Error handler called when 'obstack_chunk_alloc' failed to allocate more
 *   memory.  This can be set to a user defined function which should either
 *   abort gracefully or use longjump - but shouldn't return.  The default
 *   action is to print a message and abort.
 *
 ****************************************************************************/

extern void (*obstack_alloc_failed_handler) (void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_OBSTACK_H */

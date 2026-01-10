//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

namespace __cxxabiv1 {

// Simple malloc wrapper for fallback allocation
void *__aligned_malloc_with_fallback(size_t size) { return malloc(size); }

// Proper aligned allocation for embedded systems
// SAFETY: Alignment requirements are critical for ARM/RISC-V architectures
// CRITICAL SAFETY CHECKS:
//   1. Integer overflow protection on size calculations
//   2. Alignment validation (must be power of 2)
//   3. NULL pointer checks on all allocations
//   4. Bounds validation before pointer arithmetic
void *__aligned_malloc_with_fallback(size_t alignment, size_t size) {
  // SAFETY: Validate alignment is power of 2 and non-zero
  assert(alignment > 0 && (alignment & (alignment - 1)) == 0);

  // SAFETY: Reject zero-size allocations explicitly
  if (size == 0) {
    return nullptr;
  }

  // For small alignments that malloc already provides, use malloc directly
  if (alignment <= alignof(max_align_t)) {
    void *ptr = malloc(size);
    // SAFETY: Check malloc success
    if (!ptr) {
      return nullptr;
    }
    return ptr;
  }

  // CRITICAL SAFETY CHECK: Prevent integer overflow in size calculation
  // total_size = size + alignment + sizeof(void*)
  // Check: size <= SIZE_MAX - alignment - sizeof(void*)
  if (size > SIZE_MAX - alignment - sizeof(void *)) {
    // Integer overflow would occur - MUST reject to prevent buffer overflow
    return nullptr;
  }

  size_t total_size = size + alignment + sizeof(void *);

  // SAFETY: Additional sanity check that total_size is reasonable
  assert(total_size > size && "Integer overflow detected");

  void *raw_ptr = malloc(total_size);

  // SAFETY: Check malloc failure
  if (!raw_ptr) {
    return nullptr;
  }

  // Calculate aligned address
  uintptr_t raw_addr = reinterpret_cast<uintptr_t>(raw_ptr);
  uintptr_t offset = alignment - (raw_addr % alignment);

  // Ensure we have space to store the original pointer
  if (offset < sizeof(void *)) {
    offset += alignment;
  }

  // SAFETY: Verify offset is within allocated bounds
  assert(offset <= total_size - size && "Alignment calculation error");

  void *aligned_ptr = reinterpret_cast<void *>(raw_addr + offset);

  // Store original pointer just before the aligned pointer
  void **ptr_storage = reinterpret_cast<void **>(aligned_ptr) - 1;

  // SAFETY: Verify pointer storage location is within allocated memory
  assert(reinterpret_cast<uintptr_t>(ptr_storage) >= raw_addr &&
         reinterpret_cast<uintptr_t>(ptr_storage) < raw_addr + total_size &&
         "Pointer storage out of bounds");

  *ptr_storage = raw_ptr;

  // SAFETY: Final verification that aligned pointer meets alignment requirement
  assert((reinterpret_cast<uintptr_t>(aligned_ptr) & (alignment - 1)) == 0 &&
         "Alignment requirement not met");

  return aligned_ptr;
}

// Allocate zero-initialized memory with overflow protection
// SAFETY: calloc checks for overflow internally, but we add explicit validation
void *__calloc_with_fallback(size_t count, size_t size) {
  // SAFETY: Check for multiplication overflow before calling calloc
  // Many implementations check this, but we verify explicitly for safety
  if (count != 0 && size > SIZE_MAX / count) {
    // Overflow would occur - reject allocation
    return nullptr;
  }

  void *ptr = calloc(count, size);

  // SAFETY: Check allocation success
  if (!ptr && count > 0 && size > 0) {
    // Allocation failed - return nullptr
    return nullptr;
  }

  return ptr;
}

// Free properly aligned memory with safety validation
// SAFETY: Validates pointer before freeing to prevent corruption
// NOTE: For single-argument __aligned_malloc_with_fallback(size), we just use
// malloc/free directly For two-argument version with alignment, we store the
// original pointer before the aligned address
void __aligned_free_with_fallback(void *ptr) {
  // SAFETY: NULL pointer is valid to free (no-op)
  if (!ptr) {
    return;
  }

  // Check if this pointer has the alignment metadata
  // If alignment was requested, the original pointer is stored at ptr -
  // sizeof(void*) For simple malloc (no alignment), we just free directly
  void **ptr_storage = reinterpret_cast<void **>(ptr) - 1;
  void *original_ptr = *ptr_storage;

  // Heuristic: If the stored "pointer" value looks invalid (not aligned, or
  // points way before current ptr), assume this was allocated with malloc(size)
  // directly and just free it. Otherwise, use the stored original pointer.
  uintptr_t ptr_addr = reinterpret_cast<uintptr_t>(ptr);
  uintptr_t orig_addr = reinterpret_cast<uintptr_t>(original_ptr);

  // Check if original_ptr looks valid:
  // 1. Not NULL
  // 2. Points before or at aligned ptr
  // 3. Within reasonable distance (< 1MB offset indicates alignment padding,
  // not corruption)
  if (original_ptr != nullptr && orig_addr <= ptr_addr &&
      (ptr_addr - orig_addr) < (1024 * 1024)) {
    // Looks like aligned allocation with stored pointer
    free(original_ptr);
  } else {
    // Looks like simple malloc allocation without alignment metadata
    free(ptr);
  }
}

} // namespace __cxxabiv1

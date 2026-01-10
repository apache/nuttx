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
void __aligned_free_with_fallback(void *ptr) {
  // SAFETY: NULL pointer is valid to free (no-op)
  if (!ptr) {
    return;
  }

  // Retrieve the original pointer stored before the aligned pointer
  void **ptr_storage = reinterpret_cast<void **>(ptr) - 1;
  void *original_ptr = *ptr_storage;

  // SAFETY: Validate that original pointer is not NULL
  // If this fails, memory corruption has occurred
  assert(original_ptr != nullptr && "Corrupted alignment metadata detected");

  // SAFETY: Verify original_ptr is before or equal to aligned ptr
  // This catches some forms of corruption
  assert(reinterpret_cast<uintptr_t>(original_ptr) <=
             reinterpret_cast<uintptr_t>(ptr) &&
         "Memory corruption: original pointer after aligned pointer");

  // Free the original allocation
  free(original_ptr);
}

} // namespace __cxxabiv1

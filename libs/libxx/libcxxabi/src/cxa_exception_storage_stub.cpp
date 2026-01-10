//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

// ============================================================================
// CRITICAL SAFETY WARNING - READ BEFORE USE
// ============================================================================
//
// This implementation uses a SINGLE GLOBAL exception state without any
// synchronization mechanism. It is ONLY safe under these conditions:
//
// 1. Single-threaded execution (no RTOS task switching during exception
// handling)
// 2. No interrupt-based exception throwing
// 3. No concurrent exception handling across tasks
// 4. _LIBCPP_HAS_NO_THREADS must be defined at compile time
//
// UNSAFE SCENARIOS (will cause data corruption):
// - Multiple RTOS tasks throwing exceptions simultaneously
// - Exception thrown from ISR while task is handling exception
// - Nested exceptions across different execution contexts
//
// FOR MULTI-THREADED SYSTEMS:
// Implement per-task exception storage using NuttX TLS or task-local storage.
//
// RECOMMENDED FOR CRITICAL SYSTEMS:
// Disable C++ exceptions entirely (CONFIG_CXX_EXCEPTION=n) and use
// error codes instead.
// ============================================================================

#include "../libcxxabi/src/cxa_exception.h"
#include <cassert>
#include <cstdlib>
#include <cstring>

// Use NuttX Task Local Storage for per-task exception state
#include <nuttx/tls.h>

// Compile-time safety check: ensure threading is disabled for libcxx
#ifndef _LIBCPP_HAS_NO_THREADS
#error                                                                         \
    "cxa_exception_storage_stub.cpp requires _LIBCPP_HAS_NO_THREADS to be defined."
#endif

namespace __cxxabiv1 {

// Per-task exception storage using NuttX TLS
// This is THREAD-SAFE and works correctly with NuttX RTOS task switching
static int tls_key = -1;

// Initialize TLS key on first use (lazy initialization)
static void init_tls_key() {
  if (tls_key == -1) {
    // Allocate TLS slot with destructor to free memory on task exit
    tls_key = task_tls_alloc([](void *ptr) {
      if (ptr) {
        free(ptr);
      }
    });
    // Verify allocation succeeded
    assert(tls_key >= 0 && "Failed to allocate TLS key for exception storage");
  }
}

extern __cxa_eh_globals *__cxa_get_globals() {
  // Ensure TLS key is initialized
  init_tls_key();

  // Get per-task exception state from TLS (returns uintptr_t, needs
  // reinterpret_cast)
  __cxa_eh_globals *eh =
      reinterpret_cast<__cxa_eh_globals *>(task_tls_get_value(tls_key));

  if (!eh) {
    // First exception in this task - allocate storage
    eh = static_cast<__cxa_eh_globals *>(malloc(sizeof(__cxa_eh_globals)));
    if (!eh) {
      // Out of memory - this is fatal for exception handling
      assert(false && "Out of memory allocating exception state");
      return nullptr;
    }

    // Initialize to zero
    memset(eh, 0, sizeof(__cxa_eh_globals));

    // Store in TLS for this task (requires uintptr_t cast)
    task_tls_set_value(tls_key, reinterpret_cast<uintptr_t>(eh));
  }

  return eh;
}

extern __cxa_eh_globals *__cxa_get_globals_fast() {
  // Fast version assumes TLS is already initialized
  // SAFETY: Only call this after __cxa_get_globals() has been called once
  init_tls_key();
  return reinterpret_cast<__cxa_eh_globals *>(task_tls_get_value(tls_key));
}

} // namespace __cxxabiv1

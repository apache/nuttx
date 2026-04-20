//===-- from_chars_support.h - from_chars float support -------- -*- C++ -*-===//
//
// Minimal self-contained replacement for LLVM libc's shared/ headers:
//   shared/fp_bits.h, shared/str_to_float.h, shared/str_to_integer.h
//
// Delegates actual string-to-float conversion to system strtof/strtod,
// providing the API surface that libcxx's from_chars_floating_point.h expects.
//
// Supports float and double only. No long double, no wchar_t.
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LIBCXX_SRC_INCLUDE_FROM_CHARS_SUPPORT_H
#define LIBCXX_SRC_INCLUDE_FROM_CHARS_SUPPORT_H

#include <bit>
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <type_traits>

// POSIX locale isolation
#if __has_include(<xlocale.h>)
#include <xlocale.h>
#elif __has_include(<locale.h>)
#include <locale.h>
#endif

#ifndef LIBC_NAMESPACE
#define LIBC_NAMESPACE __llvm_libc
#endif

namespace LIBC_NAMESPACE {

// ===== Utility =============================================================
template <typename T, size_t count>
constexpr std::enable_if_t<std::is_unsigned_v<T>, T> mask_trailing_ones() {
  static_assert(count <= sizeof(T) * CHAR_BIT);
  if constexpr (count == 0) return 0;
  else if constexpr (count == sizeof(T) * CHAR_BIT) return T(-1);
  else return T(T(-1) >> (sizeof(T) * CHAR_BIT - count));
}

// ===== FPBits<T> ===========================================================
namespace fputil {

template <typename T> struct FPBitsBase;
template <> struct FPBitsBase<float> {
  using StorageType = uint32_t;
  static constexpr int STORAGE_LEN = 32, EXP_LEN = 8, SIG_LEN = 23;
  static constexpr int FRACTION_LEN = 23, EXP_BIAS = 127;
};
template <> struct FPBitsBase<double> {
  using StorageType = uint64_t;
  static constexpr int STORAGE_LEN = 64, EXP_LEN = 11, SIG_LEN = 52;
  static constexpr int FRACTION_LEN = 52, EXP_BIAS = 1023;
};

template <typename T> class FPBits : public FPBitsBase<T> {
  using Base = FPBitsBase<T>;
public:
  using StorageType = typename Base::StorageType;
  using Base::EXP_BIAS; using Base::EXP_LEN; using Base::FRACTION_LEN;
  using Base::SIG_LEN; using Base::STORAGE_LEN;

  static constexpr int MAX_BIASED_EXPONENT = (1 << EXP_LEN) - 1;
  static constexpr StorageType FRACTION_MASK = mask_trailing_ones<StorageType, FRACTION_LEN>();
  static constexpr StorageType EXP_MASK = mask_trailing_ones<StorageType, EXP_LEN>() << SIG_LEN;
  static constexpr StorageType EXP_SIG_MASK = mask_trailing_ones<StorageType, EXP_LEN + SIG_LEN>();

  StorageType bits = 0;

  constexpr FPBits() = default;
  constexpr explicit FPBits(T x) : bits(__builtin_bit_cast(StorageType, x)) {}
  constexpr explicit FPBits(StorageType x) : bits(x) {}

  constexpr T get_val() const { return __builtin_bit_cast(T, bits); }
  constexpr StorageType get_mantissa() const { return bits & FRACTION_MASK; }
  constexpr void set_mantissa(StorageType m) { bits = (bits & ~FRACTION_MASK) | (m & FRACTION_MASK); }
  constexpr uint16_t get_biased_exponent() const { return uint16_t((bits & EXP_MASK) >> SIG_LEN); }
  constexpr void set_biased_exponent(StorageType e) { bits = (bits & ~EXP_MASK) | ((StorageType(e) << SIG_LEN) & EXP_MASK); }
  constexpr StorageType get_explicit_mantissa() const {
    return get_biased_exponent() == 0 ? (bits & FRACTION_MASK)
                                      : (StorageType(1) << SIG_LEN) | (bits & FRACTION_MASK);
  }
  constexpr bool is_zero() const { return (bits & EXP_SIG_MASK) == 0; }
  constexpr bool is_subnormal() const { return get_biased_exponent() == 0 && !is_zero(); }
};

} // namespace fputil

// ===== StrToNumResult ======================================================
template <typename T> struct StrToNumResult {
  T value; int error; ptrdiff_t parsed_len;
  constexpr StrToNumResult(T v) : value(v), error(0), parsed_len(0) {}
  constexpr StrToNumResult(T v, ptrdiff_t pl) : value(v), error(0), parsed_len(pl) {}
  constexpr StrToNumResult(T v, ptrdiff_t pl, int e) : value(v), error(e), parsed_len(pl) {}
  constexpr bool has_error() const { return error != 0; }
  constexpr operator T() const { return value; }
};

// ===== strtointeger (char-only) ============================================
namespace internal {

template <typename T>
inline StrToNumResult<T>
strtointeger(const char *__restrict src, int base,
             size_t src_len = std::numeric_limits<size_t>::max()) {
  if (src_len == 0) return {0, 0, 0};

  // Copy to null-terminated buffer (strtol needs it)
  char buf[64];
  size_t n = src_len < sizeof(buf) - 1 ? src_len : sizeof(buf) - 1;
  std::memcpy(buf, src, n);
  buf[n] = '\0';

  char *end = nullptr;
  errno = 0;
  long val = std::strtol(buf, &end, base);
  int err = errno;
  ptrdiff_t parsed = end - buf;

  if (err == ERANGE || val > std::numeric_limits<T>::max() || val < std::numeric_limits<T>::min()) {
    return {val > 0 ? std::numeric_limits<T>::max() : std::numeric_limits<T>::min(), parsed, ERANGE};
  }
  return {T(val), parsed, 0};
}

// ===== Types shared with from_chars_floating_point.h =======================
enum class RoundDirection { Up, Down, Nearest };

template <class T> struct ExpandedFloat {
  typename fputil::FPBits<T>::StorageType mantissa;
  int32_t exponent;
};

template <class T> struct FloatConvertReturn {
  ExpandedFloat<T> num = {0, 0};
  int error = 0;
};

// ===== System libc conversion helpers ======================================

// Thread-safe C-locale strtof/strtod wrapper.
// Uses POSIX uselocale() to temporarily switch to "C" locale.
struct CLocaleGuard {
#if defined(_WIN32)
  _locale_t loc;
  CLocaleGuard() : loc(_create_locale(LC_ALL, "C")) {}
  ~CLocaleGuard() { _free_locale(loc); }
  float strtof_c(const char *s, char **end) { return _strtof_l(s, end, loc); }
  double strtod_c(const char *s, char **end) { return _strtod_l(s, end, loc); }
#elif defined(__GLIBC__) || defined(__APPLE__) || defined(__FreeBSD__) || \
    defined(__NetBSD__) || defined(__OpenBSD__) || defined(__ANDROID__)
  locale_t cloc, prev;
  CLocaleGuard() : cloc(newlocale(LC_ALL_MASK, "C", nullptr)),
                   prev(uselocale(cloc)) {}
  ~CLocaleGuard() { uselocale(prev); freelocale(cloc); }
  float strtof_c(const char *s, char **end) { return ::strtof(s, end); }
  double strtod_c(const char *s, char **end) { return ::strtod(s, end); }
#else
  // Fallback: no locale isolation (assumes C locale)
  float strtof_c(const char *s, char **end) { return ::strtof(s, end); }
  double strtod_c(const char *s, char **end) { return ::strtod(s, end); }
#endif
};

// Decompose a float/double into ExpandedFloat (mantissa + biased exponent)
template <typename T>
inline ExpandedFloat<T> decompose(T val) {
  fputil::FPBits<T> fb(val);
  return {fb.get_mantissa(), int32_t(fb.get_biased_exponent())};
}

// Format mantissa + base-10 exponent as a string for strtod/strtof.
// E.g. mantissa=314, exponent=-2 -> "314e-2"
template <typename ST>
inline size_t format_decimal(char *buf, size_t bufsz, ST mantissa, int32_t exp10) {
  // Print mantissa digits
  char digits[24]; // max uint64_t is 20 digits
  int dlen = 0;
  if (mantissa == 0) { digits[dlen++] = '0'; }
  else {
    ST m = mantissa;
    while (m > 0) { digits[dlen++] = char('0' + m % 10); m /= 10; }
    // Reverse
    for (int i = 0; i < dlen / 2; ++i) {
      char t = digits[i]; digits[i] = digits[dlen - 1 - i]; digits[dlen - 1 - i] = t;
    }
  }
  size_t pos = 0;
  for (int i = 0; i < dlen && pos < bufsz - 1; ++i) buf[pos++] = digits[i];
  if (exp10 != 0 && pos < bufsz - 12) {
    pos += size_t(std::snprintf(buf + pos, bufsz - pos, "e%d", int(exp10)));
  }
  buf[pos] = '\0';
  return pos;
}

// Format mantissa + base-2 exponent as hex float string for strtod.
// E.g. mantissa=0xFF, exponent=4 -> "0x1.FEp11" (normalized)
// Simpler: just use "0x<mantissa>p<exp2>"
template <typename ST>
inline size_t format_hex(char *buf, size_t bufsz, ST mantissa, int32_t exp2) {
  size_t pos = 0;
  pos += size_t(std::snprintf(buf, bufsz, "0x%llxp%d",
                              (unsigned long long)mantissa, int(exp2)));
  buf[pos] = '\0';
  return pos;
}

// ===== decimal_exp_to_float (via system strtod/strtof) =====================
template <typename T>
inline FloatConvertReturn<T>
decimal_exp_to_float(ExpandedFloat<T> init_num, bool /*truncated*/,
                     RoundDirection /*round*/,
                     const char *__restrict /*numStart*/,
                     size_t /*num_len*/ = std::numeric_limits<size_t>::max()) {
  FloatConvertReturn<T> output;

  char buf[64];
  format_decimal(buf, sizeof(buf), init_num.mantissa, init_num.exponent);

  CLocaleGuard guard;
  char *end = nullptr;
  errno = 0;

  T val;
  if constexpr (std::is_same_v<T, float>)
    val = guard.strtof_c(buf, &end);
  else
    val = guard.strtod_c(buf, &end);

  if (errno == ERANGE) output.error = ERANGE;

  output.num = decompose(val);
  return output;
}

// ===== binary_exp_to_float (via system strtod/strtof) ======================
template <class T>
inline FloatConvertReturn<T>
binary_exp_to_float(ExpandedFloat<T> init_num, bool /*truncated*/,
                    RoundDirection /*round*/) {
  FloatConvertReturn<T> output;

  char buf[80];
  format_hex(buf, sizeof(buf), init_num.mantissa, init_num.exponent);

  CLocaleGuard guard;
  char *end = nullptr;
  errno = 0;

  T val;
  if constexpr (std::is_same_v<T, float>)
    val = guard.strtof_c(buf, &end);
  else
    val = guard.strtod_c(buf, &end);

  if (errno == ERANGE) output.error = ERANGE;

  output.num = decompose(val);
  return output;
}

} // namespace internal

// ===== shared namespace (API consumed by from_chars_floating_point.h) ======
namespace shared {

using fputil::FPBits;
using LIBC_NAMESPACE::StrToNumResult;
using internal::strtointeger;
using internal::ExpandedFloat;
using internal::FloatConvertReturn;
using internal::RoundDirection;
using internal::binary_exp_to_float;
using internal::decimal_exp_to_float;

} // namespace shared
} // namespace LIBC_NAMESPACE

#endif // LIBCXX_SRC_INCLUDE_FROM_CHARS_SUPPORT_H

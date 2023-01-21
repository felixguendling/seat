#include "seat/random_booking.h"

#include "utl/zip.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

namespace seat {

// state for splitmix64
uint64_t splitmix64_x; /* The state can be seeded with any value. */

// call this one before calling splitmix64
static inline void splitmix64_seed(uint64_t seed) { splitmix64_x = seed; }

// floor( ( (1+sqrt(5))/2 ) * 2**64 MOD 2**64)
#define GOLDEN_GAMMA UINT64_C(0x9E3779B97F4A7C15)

// returns random number, modifies seed[0]
// compared with D. Lemire against
// http://grepcode.com/file/repository.grepcode.com/java/root/jdk/openjdk/8-b132/java/util/SplittableRandom.java#SplittableRandom.0gamma
static inline uint64_t splitmix64_r(uint64_t* seed) {
  uint64_t z = (*seed += GOLDEN_GAMMA);
  // David Stafford's Mix13 for MurmurHash3's 64-bit finalizer
  z = (z ^ (z >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
  z = (z ^ (z >> 27)) * UINT64_C(0x94D049BB133111EB);
  return z ^ (z >> 31);
}

// returns random number, modifies splitmix64_x
static inline uint64_t splitmix64(void) { return splitmix64_r(&splitmix64_x); }

// returns the 32 least significant bits of a call to splitmix64
// this is a simple (inlined) function call followed by a cast
static inline uint32_t splitmix64_cast32(void) {
  return (uint32_t)splitmix64();
}

// returns the value of splitmix64 "offset" steps from seed
static inline uint64_t splitmix64_stateless(uint64_t seed, uint64_t offset) {
  seed += offset * GOLDEN_GAMMA;
  return splitmix64_r(&seed);
}

std::uint64_t seed = 19;

using __uint128_t = std::uint64_t;

// Credits for fast random number generator:
// https://lemire.me/blog/2019/03/19/the-fastest-conventional-random-number-generator-that-can-pass-big-crush/
__uint128_t g_lehmer64_state =
    (((__uint128_t)splitmix64_stateless(seed, 0)) << 64) +
    splitmix64_stateless(seed, 1);

std::uint64_t lehmer64() {
  g_lehmer64_state *= 0xda942042e4dd58b5;
  return g_lehmer64_state >> 64;
}

double to_01(std::uint64_t const i) {
  constexpr uint64_t mask1 = 0x3FF0000000000000ULL;
  constexpr uint64_t mask2 = 0x3FFFFFFFFFFFFFFFULL;
  const uint64_t to_12 = (i | mask1) & mask2;
  double d;
  std::memcpy(&d, &to_12, 8);
  return d - 1;
}

wish generate_wish() {
  auto const x = to_01(lehmer64());
  if (x >= 0.75) {
    return wish::kYes;
  } else if (x >= 0.5) {
    return wish::kNo;
  } else {
    return wish::kAny;
  }
}

booking generate_random_booking(reservation const r,
                                unsigned const number_of_segments) {
  auto b = booking{};
  for (auto [w_br, w_r] : utl::zip(b.r_, r)) {
    if (w_r == wish::kAny) {
      w_br = generate_wish();
    } else {
      w_br = w_r;
    }
  }
  auto const from_p = to_01(lehmer64());
  b.interval_.from_ = static_cast<std::uint32_t>(
      std::floor(number_of_segments * (from_p == 1.0 ? 0.5 : from_p)));
  b.interval_.to_ = static_cast<std::uint32_t>(
      b.interval_.from_ + 1U +
      std::round((number_of_segments - b.interval_.from_ - 1) *
                 to_01(lehmer64())));
  return b;
}

}  // namespace seat

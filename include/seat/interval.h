#pragma once

#include <cinttypes>
#include <compare>
#include <iosfwd>

namespace seat {

struct interval {
  auto operator<=>(interval const&) const noexcept = default;
  bool contains(interval const&) const;
  bool overlaps(interval const&) const;
  uint8_t size() const;
  friend std::ostream& operator<<(std::ostream&, interval const&);
  std::uint8_t from_, to_;
};

}  // namespace seat

#pragma once

#include <cinttypes>
#include <compare>
#include <iosfwd>

#include "types.h"

namespace seat {

struct interval {
  auto operator<=>(interval const&) const noexcept = default;
  bool contains(interval const&) const;
  bool contains(uint8_t const&) const;
  bool equals(interval const&) const;
  bool overlaps(interval const&) const;
  uint8_t size() const;
  friend std::ostream& operator<<(std::ostream&, interval const&);
  small_station_id_t from_, to_;
};

}  // namespace seat

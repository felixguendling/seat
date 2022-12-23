#pragma once

#include <iosfwd>

namespace seat {

struct interval {
  auto operator<=>(interval const&) const noexcept = default;
  bool contains(interval const&) const;
  friend std::ostream& operator<<(std::ostream&, interval const&);
  unsigned from_, to_;
};

}  // namespace seat

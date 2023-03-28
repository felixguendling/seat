#pragma once

#include <cinttypes>
#include <iosfwd>
#include <vector>

#include "seat/interval.h"
#include "seat/reservation.h"

namespace seat {

struct booking {
  bool operator==(booking other);
  friend std::ostream& operator<<(std::ostream&, booking const&);
  reservation r_;
  interval interval_;
  uint8_t group_size_;
};

}  // namespace seat

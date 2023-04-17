#pragma once

#include <cinttypes>
#include <iosfwd>
#include <vector>

#include "seat/interval.h"
#include "seat/reservation.h"

namespace seat {

struct booking {
  bool operator==(booking const&) const;
  bool operator<(booking const&) const;
  friend std::ostream& operator<<(std::ostream&, booking const&);
  reservation r_;
  interval interval_;
  uint32_t group_id_ = 0;  // 0 -> no group
};

}  // namespace seat

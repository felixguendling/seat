#pragma once

#include <cinttypes>
#include <iosfwd>
#include <vector>

#include "seat/interval.h"
#include "seat/reservation.h"

namespace seat {

struct booking {
  auto operator<=>(booking const&) const noexcept = default;
  // bool operator==(booking const&) const;
  // bool operator<(booking const&) const;
  friend std::ostream& operator<<(std::ostream&, booking const&);
  reservation r_;
  interval interval_;
  group_id_t group_id_ = 0;  // 0 -> no group
  uint8_t group_size_ = 0;
  booking_type type_;
};

}  // namespace seat

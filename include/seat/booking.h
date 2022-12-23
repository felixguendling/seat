#pragma once

#include <cinttypes>
#include <iosfwd>

#include "seat/interval.h"
#include "seat/reservation.h"

namespace seat {

struct booking {
  friend std::ostream& operator<<(std::ostream&, booking const&);
  reservation r_;
  interval interval_;
};

}  // namespace seat

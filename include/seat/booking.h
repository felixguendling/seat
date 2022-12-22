#pragma once

#include <cinttypes>
#include <iosfwd>

#include "seat/reservation.h"

namespace seat {

struct booking {
  friend std::ostream& operator<<(std::ostream&, booking const&);
  reservation r_;
  std::uint32_t from_, to_;
};

}  // namespace seat

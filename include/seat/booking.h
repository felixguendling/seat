#pragma once

#include <cinttypes>

#include "seat/reservation.h"

namespace seat {

struct booking {
  reservation r_;
  std::uint32_t from_, to_;
};

}  // namespace seat

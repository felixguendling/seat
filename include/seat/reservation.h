#pragma once

#include <array>
#include <iosfwd>

#include "seat/properties.h"
#include "seat/wish.h"

namespace seat {

using reservation = std::array<wish, kNProperties>;

bool matches(reservation, reservation);

std::ostream& operator<<(std::ostream& out, reservation const& r);

}  // namespace seat

#pragma once

#include <array>
#include <iosfwd>

#include "seat/properties.h"
#include "seat/wish.h"

namespace seat {

using reservation = std::array<wish, kNProperties>;

bool matches(reservation, reservation);
int concreteness(reservation);
uint8_t to_int(reservation);
bool matches(reservation,uint8_t);

std::ostream& operator<<(std::ostream& out, reservation const& r);

}  // namespace seat

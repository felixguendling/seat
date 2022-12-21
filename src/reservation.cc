#include "seat/reservation.h"

#include <iostream>

#include "utl/zip.h"

namespace seat {

std::ostream& operator<<(std::ostream& out, reservation const& r) {
  for (auto const& w : r) {
    out << w;
  }
  return out;
}

bool matches(reservation const a, reservation const b) {
  for (auto const& [pa, pb] : utl::zip(a, b)) {
    if (pa != wish::kAny && pa != pb) {
      return false;
    }
  }
  return true;
}

}  // namespace seat

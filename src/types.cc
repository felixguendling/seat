#include "seat/types.h"

#include <ostream>

namespace seat {

std::ostream& operator<<(std::ostream& out, booking_type const& type) {
  switch (type) {
    case booking_type::NORMAL: return out << "normal";
    case booking_type::GSD: return out << "gsd";
    default: return out << "group";
  }
}
}  // namespace seat
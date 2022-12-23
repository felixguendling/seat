#include "seat/booking.h"

#include <ostream>

namespace seat {

std::ostream& operator<<(std::ostream& out, booking const& b) {
  return out << "(" << b.interval_ << ", " << b.r_ << ")";
}

}  // namespace seat

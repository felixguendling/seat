#include "seat/booking.h"

#include <ostream>

namespace seat {

std::ostream& operator<<(std::ostream& out, booking const& b) {
  return out << "(" << b.interval_ << ", " << b.r_ << ")";
}
bool booking::operator==(booking other) {
  return other.r_ == r_ && other.interval_ == interval_;
}

}  // namespace seat

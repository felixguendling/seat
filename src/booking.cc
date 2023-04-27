#include "seat/booking.h"

#include <ostream>

namespace seat {

std::ostream& operator<<(std::ostream& out, booking const& b) {
  return out << "(" << b.interval_ << ", " << b.r_ << ")";
}
/*
bool booking::operator==(booking const& other) const {
  return to_int_all(other.r_) == to_int_all(r_) && other.interval_ == interval_;
}

bool booking::operator<(booking const& other) const {
  if (interval_.from_ < other.interval_.from_) {
    return true;
  }
  if (other.interval_.from_ < interval_.from_) {
    return false;
  }
  if (interval_.to_ < other.interval_.to_) {
    return true;
  }
  if (other.interval_.to_ < interval_.to_) {
    return false;
  }
  if (to_int_all(r_) < to_int_all(other.r_)) {
    return true;
  }
  return false;
}*/
}  // namespace seat

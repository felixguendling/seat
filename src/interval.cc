#include "seat/interval.h"

#include <ostream>

namespace seat {

std::ostream& operator<<(std::ostream& out, interval const& i) {
  return out << "[" << static_cast<int>(i.from_) << ", "
             << static_cast<int>(i.to_) << ")";
}

bool interval::contains(interval const& o) const {
  return o.from_ >= from_ && o.to_ <= to_;
}

bool interval::overlaps(interval const& o) const {
  // 1-2 vs 2-3: 1 < 3 && 2 > 2 => false
  // 2-3 vs 1-2: 2 < 2 && 3 > 1 => false
  // 1-3 vs 2-4: 1 < 4 && 3 > 2 => true
  return from_ < o.to_ && to_ > o.from_;
}

uint8_t interval::size() const { return to_ - from_; }

bool interval::contains(const uint8_t& o) const {
  return o >= from_ && o < to_;
}
bool interval::equals(interval const& other) const {
  return (from_ == other.from_) && (to_ == other.to_);
}

bool interval::touches(interval const& other) const {
  return (from_ >= other.from_ && from_ < other.to_) ||
         (other.from_ >= from_ && other.from_ < to_);
}

}  // namespace seat

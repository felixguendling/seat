#include "seat/interval.h"

#include <ostream>

namespace seat {

std::ostream& operator<<(std::ostream& out, interval const& i) {
  return out << "[" << static_cast<int>(i.from_) << ", "
             << static_cast<int>(i.to_) << ")";
}

}  // namespace seat

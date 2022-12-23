#include "seat/interval.h"

#include <ostream>

namespace seat {

std::ostream& operator<<(std::ostream& out, interval const& i) {
  return out << "[" << i.from_ << ", " << i.to_ << ")";
}

}  // namespace seat

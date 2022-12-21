#include "seat/wish.h"

#include <ostream>

namespace seat {

std::ostream& operator<<(std::ostream& out, wish const w) {
  return out << (w == wish::kYes ? 'Y' : (w == wish::kNo ? 'N' : 'X'));
}

}  // namespace seat

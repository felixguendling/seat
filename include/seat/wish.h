#pragma once

#include <cinttypes>
#include <iosfwd>

namespace seat {

enum class wish : std::uint8_t { kNo, kYes, kAny };

std::ostream& operator<<(std::ostream& out, wish const w);

}  // namespace seat

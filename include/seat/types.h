#pragma once

#include <cstdint>
#include <iosfwd>

namespace seat {

enum class booking_type { NORMAL, GSD, GROUP };
std::ostream& operator<<(std::ostream& out, booking_type const& type);
using station_id_t = std::uint32_t;
using booking_id_t = std::uint32_t;
using encoded_reservation_t = std::uint8_t;
using small_station_id_t = std::uint8_t;
using wagon_id_t = std::uint8_t;
using seat_id_t = std::uint32_t;
using group_id_t = std::uint32_t;
using row_id_t = std::uint32_t;

}  // namespace seat

#pragma once

namespace seat {

enum FEASIBLE { FEASIBLE, INFEASIBLE, UNKNOWN };
enum BOOKING_TYPE { NORMAL, GSD, GROUP };
using station_id_t = std::uint32_t;
using node_id_t = std::uint32_t;
using path_id_t = std::uint32_t;
using capacity_t = std::uint32_t;
using color_t = std::uint16_t;
using booking_id_t = std::uint32_t;
using encoded_reservation_t = std::uint8_t;
using small_station_id_t = std::uint8_t;
using wagon_id_t = std::uint8_t;
using seat_id_t = uint32_t;

}  // namespace seat

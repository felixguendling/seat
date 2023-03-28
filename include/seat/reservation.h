#pragma once

#include <array>
#include <iosfwd>
#include <map>
#include <vector>

#include "seat/properties.h"
#include "seat/types.h"
#include "seat/wish.h"

namespace seat {

using reservation = std::array<wish, kNProperties>;

bool matches(reservation, reservation);
int concreteness(reservation);
encoded_reservation_t to_int(reservation);
encoded_reservation_t to_int_all(reservation);
bool matches(reservation, encoded_reservation_t);
template <typename F>
void for_each_concrete_reservation(reservation const&, F&& f);
std::map<reservation, std::uint32_t> generate_number_of_seats(
    std::vector<std::uint32_t> const&, std::uint8_t const);

std::vector<reservation> get_concrete_reservations(reservation const&);
std::vector<reservation> get_concrete_reservations(uint8_t const);
std::ostream& operator<<(std::ostream& out, reservation const& r);

}  // namespace seat

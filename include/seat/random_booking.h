#pragma once

#include "seat/booking.h"

namespace seat {

booking generate_random_booking(std::map<reservation, std::uint32_t> const&,
                                reservation, unsigned number_of_segments);

}  // namespace seat

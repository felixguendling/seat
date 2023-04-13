#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cista/containers/vector.h"
#include "ortools/linear_solver/linear_solver.h"

#include "seat/booking.h"
#include "seat/reservation.h"
#include "seat/types.h"

namespace seat {

struct verify {
  explicit verify(std::vector<booking> const&, std::vector<booking_id_t> const&,
                  std::vector<booking_id_t> const&,
                  std::vector<booking_id_t> const&,
                  std::vector<seat_id_t> const&, std::vector<seat_id_t> const&,
                  std::vector<seat_id_t> const&, seat_id_t const);

private:
  std::vector<booking> bookings_;
  std::vector<booking_id_t> mcf_ids_;
  std::vector<booking_id_t> gsd_ids_;
  std::vector<booking_id_t> pseudo_ids_;
  std::vector<seat_id_t> mcf_seats_;
  std::vector<seat_id_t> gsd_seats_;
  std::vector<seat_id_t> pseudo_seats_;
};

}  // namespace seat

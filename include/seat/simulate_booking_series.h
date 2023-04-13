#include <fstream>
#include <iostream>
#include <tuple>
#include <vector>

#include "utl/timing.h"
#include "utl/verify.h"

#include "seat/booking.h"
#include "seat/nogo_cache.h"
#include "seat/random_booking.h"
#include "seat/reservation.h"
#include "seat/seat_assignment.h"
#include "seat/solver.h"
#include "seat/train.h"

namespace seat {
struct simulation {
  explicit simulation(std::map<reservation, std::uint32_t> const&, solver*,
                      double const&, std::uint32_t const&, train const&);

  void simulate();

  std::map<reservation, std::uint32_t> seats_by_res_;
  solver* solver_;
  double gsd_prob_;
  std::uint32_t number_of_segments_;
  train train_;
};
}  // namespace seat
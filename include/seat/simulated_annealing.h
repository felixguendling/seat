#pragma once

#include <vector>

#include "seat/booking.h"
#include "seat/neighbourhood.h"
#include "seat/heuristic_state.h"

namespace seat{
struct simulated_annealing{
  bool add_booking(booking);
  static double temp_to_zero_seq(uint32_t);
  bool insert_gsd_booking(color_t, interval);

  heuristic_state hs_;
  uint32_t max_temp_;
};

}
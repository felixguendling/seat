#include "seat/simulated_annealing.h"
#include <iostream>

namespace seat {

bool seat::simulated_annealing::add_booking(booking b) {
  auto t = uint32_t{0};
  auto state = hs_;  // copy..??
  state.reset();
  state.insert_booking(b);
  if (state.feasible_ == FEASIBLE) {
    hs_ = state;
    return true;
  }
  if (state.feasible_ == INFEASIBLE) {
    return false;
  }
  while (t != max_temp_) {
    auto candidate_neighbour = state.get_random_neighbour();
    auto score = state.score(candidate_neighbour);
    if (score > 0) {
      std::cout << "score:" << score << "\n";
    }
    if (score > 0) {
      state.update(candidate_neighbour);
      if (state.feasible_ == FEASIBLE) {
        hs_ = state;  // copy current state into hs_ as a feasible solution has
                      // been found and this solution can be used as a starting
                      // point when the next booking is being processed.
        return true;
      }
    } else {
      auto const r = rand() % 1;
      auto const probability = exp(-score * temp_to_zero_seq(t));
      if (r - probability > 0) {
        state.update(candidate_neighbour);
        if (state.feasible_ == FEASIBLE) {
          hs_ = state;
          return true;
        }
      }
    }
    ++t;
  }
  return false;
}

double simulated_annealing::temp_to_zero_seq(uint32_t const t) {
  return 0;  // sequence does not converge to zero but infinity
                      // instead. -> use * instead of / in the formula
}
bool simulated_annealing::insert_gsd_booking(color_t const color,
                                             interval const i) {
  auto state = hs_;
  state.insert_gsd_booking(color, i);
  if (state.feasible_ == FEASIBLE) {
    hs_ = state;
    return true;
  }
  return false;
}
}  // namespace seat
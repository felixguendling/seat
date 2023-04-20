#include <tuple>

#include "seat/simulate_booking_series.h"
#include "seat/solver.h"

using namespace seat;

int main() {
  auto seat_amount = 27;
  auto const number_of_segments = 2U;
  for (auto j = seat_amount; j != seat_amount + 1; ++j) {
    for (auto i = 4; i != 5; ++i) {
      srand(i);
      auto t = train();
      auto total_seats = j;
      auto min_wagon_size = j / 2 + 1;
      auto max_wagon_size = 5;
      min_wagon_size = max_wagon_size;
      t.generate_random_train(total_seats, 0.5, 0.3, 0.2, max_wagon_size,
                              min_wagon_size);
      t.print();

      auto const& gsd_prob = 0;
      auto const& group_prob = 0.7;
      auto seats = t.get_number_of_seats();

      auto fpb_solver = solver{seats, number_of_segments};
      auto c = simulation(seats, &fpb_solver, gsd_prob, group_prob,
                          number_of_segments, t);
      c.simulate();
    }
  }
}

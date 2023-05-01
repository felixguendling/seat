#include <tuple>

#include "seat/simulate_booking_series.h"
#include "seat/solver.h"

using namespace seat;

int main() {
  auto specifics = simulation_specifics();
  specifics.number_of_segments_ = 30U;
  specifics.gsd_prob_ = 0.2;
  specifics.group_prob_ = 0.1;
  specifics.max_group_size_ = 5;
  specifics.timed_wagon_runs_ = 6;

  // specifics.output_options_.do_output_ = false;

  auto seat_amount = 400;
  for (auto j = seat_amount; j != seat_amount + 1; j++) {
    for (auto i = 2; i != 3; ++i) {
      srand(i);
      std::cout << "srand: " << i << "\n";
      auto t = train();
      auto total_seats = j;
      auto min_wagon_size = j / 2 + 1;
      auto max_wagon_size = 80;
      min_wagon_size = max_wagon_size;
      t.generate_random_train(total_seats, 0.5, 0.3, 0.2, max_wagon_size,
                              min_wagon_size);
      t.print_seat_ids(specifics.output_options_);
      auto seats = t.get_number_of_seats();
      auto fpb_solver = solver{seats, specifics.number_of_segments_};
      auto c = simulation(seats, &fpb_solver, t, specifics);
      c.simulate();
    }
  }
}

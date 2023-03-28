#include <fstream>
#include <tuple>
#include <vector>

#include "utl/timing.h"

#include "seat/flow_graph.h"
#include "seat/reservation.h"
#include "seat/simulate_booking_series.h"
#include "seat/solver_full_path.h"
#include "seat/solver_pathbased.h"

using namespace seat;

int main() {
  srand(0U);
  auto t = train();
  auto total_seats = 800;
  auto min_wagon_size = 50;
  auto max_wagon_size = 70;
  // min_wagon_size = max_wagon_size;
  t.generate_random_train(total_seats, 0.5, 0.3, 0.2, max_wagon_size,
                          min_wagon_size);
  t.print();
  std::map<reservation, uint32_t> a;

  auto const& gsd_prob = 0.0;
  auto const number_of_segments = 30U;
  auto seats = t.get_number_of_seats();

  std::vector<mcf_solver_i*> solvers;
  auto ilp_solver = flow_graph{seats, number_of_segments};
  auto fpb_solver = solver_fpb{seats, number_of_segments};
  auto pb_solver = solver_pb{seats, number_of_segments};
  solvers.emplace_back(&fpb_solver);
  // solvers.emplace_back(&pb_solver);
  //  solvers.emplace_back(&ilp_solver);
  auto c = simulation(seats, solvers, gsd_prob, number_of_segments, t);
  c.simulate();
}

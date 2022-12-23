#include <fstream>
#include <iostream>
#include <sstream>

#include "utl/timing.h"

#include "seat/flow_graph.h"
#include "seat/heuristic.h"
#include "seat/nogo_cache.h"
#include "seat/random_booking.h"

using namespace seat;
/*

  int i = 0U;
  while (i < 10) {
    auto const heuristic_worked =
        g.add_booking({.r_ = {wish::kNo, wish::kNo, wish::kNo, wish::kNo},
                       .from_ = 0U,
                       .to_ = 2U});
    if (!heuristic_worked) {
      break;
    }
    ++i;
  }
  while (i < 25) {
    auto const heuristic_worked =
        g.add_booking({.r_ = {wish::kNo, wish::kYes, wish::kNo, wish::kNo},
                       .from_ = 0U,
                       .to_ = 2U});
    if (!heuristic_worked) {
      break;
    }
    ++i;
  }
  while (i < 46) {
    auto const heuristic_worked =
        g.add_booking({.r_ = {wish::kAny, wish::kAny, wish::kNo, wish::kNo},
                       .from_ = 0U,
                       .to_ = 2U});
    if (!heuristic_worked) {
      break;
    }
    ++i;
  }
  std::cout << i << "\n";
  g.to_graphviz(std::cout, false);
 */

int main() {
  auto const number_of_segments = 30U;
  std::map<reservation, std::uint32_t> number_of_seats{
      {{wish::kNo, wish::kYes, wish::kNo, wish::kNo}, 50U},
      {{wish::kYes, wish::kYes, wish::kNo, wish::kNo}, 50U},
      {{wish::kNo, wish::kNo, wish::kNo, wish::kNo}, 50U},
      {{wish::kYes, wish::kNo, wish::kNo, wish::kNo}, 50U},

      {{wish::kNo, wish::kYes, wish::kYes, wish::kNo}, 50U},
      {{wish::kYes, wish::kYes, wish::kYes, wish::kNo}, 50U},
      {{wish::kNo, wish::kNo, wish::kYes, wish::kNo}, 50U},
      {{wish::kYes, wish::kNo, wish::kYes, wish::kNo}, 50U},

      {{wish::kNo, wish::kYes, wish::kNo, wish::kYes}, 50U},
      {{wish::kYes, wish::kYes, wish::kNo, wish::kYes}, 50U},
      {{wish::kNo, wish::kNo, wish::kNo, wish::kYes}, 50U},
      {{wish::kYes, wish::kNo, wish::kNo, wish::kYes}, 50U},

      {{wish::kNo, wish::kYes, wish::kYes, wish::kYes}, 50U},
      {{wish::kYes, wish::kYes, wish::kYes, wish::kYes}, 50U},
      {{wish::kNo, wish::kNo, wish::kYes, wish::kYes}, 50U},
      {{wish::kYes, wish::kNo, wish::kYes, wish::kYes}, 50U}};

  std::stringstream ss;

  // Plot with
  // gnuplot -p -e "plot 'timings.dat' with points pt 2"
  auto timings = std::ofstream{"timings.dat"};
  flow_graph g{number_of_seats, number_of_segments};
  nogo_cache nogo;

  auto failed = 500;
  auto success = 0U;
  auto nogo_count = 0U;
  auto i = 0U;
  auto solver_solved = 0U;
  auto heuristic_solved = 0U;
  std::vector<std::uint64_t> timings_vec;
  while (failed >= 0) {
    auto const b = generate_random_booking(
        {wish::kAny, wish::kAny, wish::kAny, wish::kAny}, number_of_segments);
    std::cout << "adding booking: #" << ++i << " " << b;
    try {
      g.add_booking(b);

      UTL_START_TIMING(add_booking);
      auto feasible = !nogo.is_nogo(b);
      if (!feasible) {
        ++nogo_count;
      } else {
        feasible = heuristic(g, b);
        if (!feasible) {
          feasible = g.solve();
          ++solver_solved;
        } else {
          ++heuristic_solved;
        }

        if (!feasible) {
          nogo.add_entry(b);
        }
      }
      UTL_STOP_TIMING(add_booking);

      auto const timing = UTL_GET_TIMING_US(add_booking);
      timings_vec.emplace_back(timing);

      std::cout << " -> " << (feasible ? "good" : "bad") << " [" << timing
                << "us, solver=" << solver_solved
                << ", heuristic=" << heuristic_solved << ", failed=" << failed
                << ", success=" << success << ", nogo=" << nogo_count << "]\n";
      timings << i << " " << timing << "\n";
      if (!feasible) {
        --failed;
      } else {
        ++success;
      }
      ss.str("");
      g.to_graphviz(ss, false);
    } catch (std::exception const& e) {
      std::cout << "ABORT: " << e.what() << "\n";
      break;
    }
  }

  std::uint64_t timing_sum;
  for (auto const& t : timings_vec) {
    timing_sum += t;
  }
  std::cout << "avg: " << static_cast<double>(timing_sum) / timings_vec.size()
            << "\n";

  std::ofstream{"seat_flow_graph.dot"} << ss.str();
  std::ofstream{"seat_flow.lp"} << g.lp_str();
}

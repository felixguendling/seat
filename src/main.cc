#include <fstream>
#include <iostream>
#include <sstream>

#include "utl/timing.h"

#include "seat/flow_graph.h"
#include "seat/random_booking.h"

using namespace seat;

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
  auto i = 0U;
  while (true) {
    auto const b = generate_random_booking(
        {wish::kAny, wish::kAny, wish::kAny, wish::kAny}, number_of_segments);
    std::cout << "adding booking: #" << ++i << " (" << b;
    UTL_START_TIMING(add_booking);
    g.add_booking(b);
    try {
      auto const feasible = g.solve();
      UTL_STOP_TIMING(add_booking);
      auto const timing = UTL_GET_TIMING_MS(add_booking);
      std::cout << ") -> " << (feasible ? "good" : "bad") << " [" << timing
                << "ms]\n";
      timings << i << " " << timing << "\n";
      if (!feasible) {
        break;
      }
      ss.str("");
      g.to_graphviz(ss, false);
    } catch (std::exception const& e) {
      std::cout << "ABORT: " << e.what() << "\n";
      break;
    }
  }
  std::ofstream{"seat_flow_graph.dot"} << ss.str();
  std::ofstream{"seat_flow.lp"} << g.lp_str();
}

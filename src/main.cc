#include <iostream>

#include "seat/flow_graph.h"

using namespace seat;

int main() {
  auto const number_of_segments = 2U;
  std::map<reservation, std::uint32_t> number_of_seats{
      {{wish::kNo, wish::kYes, wish::kNo, wish::kNo}, 2U},
      {{wish::kYes, wish::kYes, wish::kNo, wish::kNo}, 2U},
      {{wish::kNo, wish::kNo, wish::kNo, wish::kNo}, 2U},
      {{wish::kYes, wish::kNo, wish::kNo, wish::kNo}, 2U},
  };

  std::vector<booking> bookings{
      booking{.r_ = {wish::kAny, wish::kYes, wish::kAny, wish::kAny},
              .from_ = 0U,
              .to_ = 1U},
      booking{.r_ = {wish::kAny, wish::kYes, wish::kAny, wish::kAny},
              .from_ = 0U,
              .to_ = 2U},
      booking{.r_ = {wish::kYes, wish::kAny, wish::kAny, wish::kAny},
              .from_ = 0U,
              .to_ = 2U},
      booking{.r_ = {wish::kYes, wish::kAny, wish::kAny, wish::kAny},
              .from_ = 0U,
              .to_ = 2U},
      booking{.r_ = {wish::kYes, wish::kNo, wish::kAny, wish::kAny},
              .from_ = 1U,
              .to_ = 2U}};

  flow_graph g{number_of_seats, number_of_segments};
  for (auto const& b : bookings) {
    g.add_booking(b);
    auto const feasible = g.solve();
    if (!feasible) {
      break;
    }
  }
  g.to_graphviz(std::cout, false);
}

#include "gtest/gtest.h"

#include "seat/flow_graph.h"
#include "seat/heuristic.h"

using namespace seat;

TEST(heuristic, impossible) {
  auto const number_of_segments = 2U;
  std::map<reservation, std::uint32_t> number_of_seats{
      {{wish::kNo, wish::kYes, wish::kNo, wish::kNo}, 2U},
      {{wish::kYes, wish::kYes, wish::kNo, wish::kNo}, 2U},
      {{wish::kNo, wish::kNo, wish::kNo, wish::kNo}, 2U},
      {{wish::kYes, wish::kNo, wish::kNo, wish::kNo}, 2U}};
  flow_graph g{number_of_seats, number_of_segments};

  for (auto i = 0U; i != 4; ++i) {
    auto const b = booking{.r_ = {wish::kAny, wish::kAny, wish::kNo, wish::kNo},
                           .from_ = 0U,
                           .to_ = 1U};
    g.add_booking(b);
    EXPECT_TRUE(heuristic(g, b));
  }

  for (auto i = 0U; i != 2; ++i) {
    auto const b = booking{.r_ = {wish::kYes, wish::kYes, wish::kNo, wish::kNo},
                           .from_ = 0U,
                           .to_ = 1U};
    g.add_booking(b);
    if (i == 0U) {
      EXPECT_TRUE(heuristic(g, b));
    } else {
      EXPECT_FALSE(heuristic(g, b));
    }
  }

  EXPECT_TRUE(g.solve());
}

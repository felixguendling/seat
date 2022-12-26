#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "simp/SimpSolver.h"

#include "utl/enumerate.h"
#include "utl/raii.h"
#include "utl/timing.h"
#include "utl/verify.h"

#include "cista/containers/vecvec.h"

#include "seat/booking.h"
#include "seat/nogo_cache.h"
#include "seat/random_booking.h"
#include "seat/reservation.h"

using namespace seat;

template <typename It, typename ResultFn, typename CompFn>
void set_intersection(It first_a, It const last_a, It first_b, It const last_b,
                      ResultFn&& result, CompFn&& lt) {
  while (first_a != last_a && first_b != last_b) {
    if (lt(*first_a, *first_b)) {
      ++first_a;  // a < b  --> ++a
    } else if (lt(*first_b, *first_a)) {
      ++first_b;  // b < a  --> ++b
    } else {
      // match: b == a --> ++a, ++b
      result(*first_a, *first_b);
      ++first_a;
      ++first_b;
    }
  }
}

struct interval_graph {
  using node_id_t = std::uint16_t;
  using seat_number_t = std::uint16_t;
  using lit_t = int;
  static constexpr auto const kFinishClause = lit_t{0};

  struct assignment {
    seat_number_t seat_;
    lit_t lit_;
  };

  interval_graph(std::vector<reservation> seats) : seats_{std::move(seats)} {}

  void add_booking(booking const& b) {
    std::vector<node_id_t> overlap;
    for (auto const& [node_id, interval] : utl::enumerate(interval_)) {
      if (b.interval_.overlaps(interval)) {
        overlap.emplace_back(node_id);
      }
    }
    neighbors_.emplace_back(overlap);

    reservation_.emplace_back(b.r_);
    interval_.emplace_back(b.interval_);
  }

  bool solve() {
    auto solver = Glucose::SimpSolver{};
    utl::verify(solver != nullptr, "kissat init failed");
    UTL_FINALLY([&]() { kissat_release(solver); });

    auto get_lit = [next = lit_t{1}]() mutable { return next++; };

    // Every customer >=1 seat.
    std::vector<std::vector<assignment>> seat_assignment_lits;
    for (auto const& r : reservation_) {
      seat_assignment_lits.emplace_back();
      for (auto const& [seat_number, s_r] : utl::enumerate(seats_)) {
        if (matches(r, s_r)) {
          auto const lit = get_lit();
          seat_assignment_lits.back().emplace_back(
              assignment{static_cast<seat_number_t>(seat_number), lit});
          kissat_add(solver, lit);
        }
      }
      kissat_add(solver, kFinishClause);
    }

    // Two overlapping bookings are not on the same seat.
    std::set<std::pair<node_id_t, node_id_t>> done;
    for (auto const& [node_id, neighbors] : utl::enumerate(neighbors_)) {
      for (auto const& neighbor : neighbors) {
        auto const sorted =
            std::pair{std::min(static_cast<node_id_t>(node_id), neighbor),
                      std::max(static_cast<node_id_t>(node_id), neighbor)};
        if (auto const it = done.find(sorted); it != end(done)) {
          continue;
        }
        auto const& seat_options_a = seat_assignment_lits[node_id];
        auto const& seat_options_b = seat_assignment_lits[neighbor];
        set_intersection(
            begin(seat_options_a), end(seat_options_a), begin(seat_options_b),
            end(seat_options_b),
            [&](assignment const& a, assignment const& b) {
              kissat_add(solver, -a.lit_);
              kissat_add(solver, -b.lit_);
              kissat_add(solver, kFinishClause);
            },
            [](assignment const& a, assignment const& b) {
              return a.seat_ < b.seat_;
            });
      }
    }

    UTL_START_TIMING(solve);
    auto const sat = kissat_solve(solver) == 10;
    UTL_STOP_TIMING(solve);
    std::cout << "solve time: " << UTL_GET_TIMING_MS(solve) << "\n";

    return sat;
  }

  std::vector<reservation> seats_;
  std::vector<reservation> reservation_;
  std::vector<interval> interval_;
  std::vector<std::vector<node_id_t>> neighbors_;
};

bool heuristic(interval_graph const&, booking const&) { return false; }

int main() {
  auto const number_of_segments = 30U;
  std::map<reservation, std::uint32_t> number_of_seats{
      {{wish::kNo, wish::kYes, wish::kNo, wish::kNo}, 50U},
      {{wish::kYes, wish::kYes, wish::kNo, wish::kNo}, 50U},
      {{wish::kNo, wish::kNo, wish::kNo, wish::kNo}, 50U},
      {{wish::kYes, wish::kNo, wish::kNo, wish::kNo}, 50U},

      /*
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
      {{wish::kYes, wish::kNo, wish::kYes, wish::kYes}, 50U}
      */
  };

  std::vector<reservation> seats;
  for (auto const& [r, n] : number_of_seats) {
    for (auto i = 0U; i != n; ++i) {
      seats.emplace_back(r);
    }
  }

  std::stringstream ss;

  // Plot with
  // gnuplot -p -e "plot 'timings.dat' with points pt 2"
  auto timings = std::ofstream{"timings.dat"};
  interval_graph g{std::move(seats)};
  nogo_cache nogo;

  auto failed = 100;
  auto success = 0U;
  auto nogo_count = 0U;
  auto i = 0U;
  auto solver_solved = 0U;
  auto heuristic_solved = 0U;
  std::vector<std::uint64_t> timings_vec;
  while (failed >= 0) {
    auto const b = generate_random_booking(
        {wish::kAny, wish::kAny, wish::kNo, wish::kNo}, number_of_segments);
    std::cout << "adding booking: #" << ++i << " " << b << " - ";
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

      auto const timing = UTL_GET_TIMING_MS(add_booking);
      timings_vec.emplace_back(timing);

      std::cout << " -> " << (feasible ? "good" : "bad") << " [" << timing
                << "ms, solver=" << solver_solved
                << ", heuristic=" << heuristic_solved << ", failed=" << failed
                << ", success=" << success << ", nogo=" << nogo_count << "]\n";
      timings << i << " " << timing << "\n";
      if (!feasible) {
        --failed;
      } else {
        ++success;
      }
      ss.str("");
      // g.to_graphviz(ss, false);
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
  // std::ofstream{"seat_flow.lp"} << g.lp_str();
}

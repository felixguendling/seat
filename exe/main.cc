#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "kissat.h"

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

  interval_graph(std::vector<reservation> seats) : seats_{std::move(seats)} {
    solver_ = kissat_init();
    utl::verify(solver_ != nullptr, "kissat init failed");
  }

  void add_booking(booking const& b) {
    auto const node_id = static_cast<node_id_t>(neighbors_.size());

    std::vector<node_id_t> overlap;
    for (auto const& [node_id, interval] : utl::enumerate(interval_)) {
      if (b.interval_.overlaps(interval)) {
        overlap.emplace_back(node_id);
      }
    }
    auto const& neighbors = neighbors_.emplace_back(std::move(overlap));
    auto const r = reservation_.emplace_back(b.r_);
    interval_.emplace_back(b.interval_);

    // Make sure the new booking gets a seat.
    seat_assignment_lits_.emplace_back();
    for (auto const& [seat_number, s_r] : utl::enumerate(seats_)) {
      if (matches(r, s_r)) {
        auto const lit = create_lit();
        seat_assignment_lits_.back().emplace_back(
            assignment{static_cast<seat_number_t>(seat_number), lit});
        kissat_add(solver_, lit);
      }
    }
    kissat_add(solver_, kFinishClause);

    // Make sure the seat is not occupied in parallel by an overlapping booking.
    for (auto const& neighbor : neighbors) {
      auto const& seat_options_a = seat_assignment_lits_[node_id];
      auto const& seat_options_b = seat_assignment_lits_[neighbor];
      set_intersection(
          begin(seat_options_a), end(seat_options_a), begin(seat_options_b),
          end(seat_options_b),
          [&](assignment const& a, assignment const& b) {
            kissat_add(solver_, -a.lit_);
            kissat_add(solver_, -b.lit_);
            kissat_add(solver_, kFinishClause);
          },
          [](assignment const& a, assignment const& b) {
            return a.seat_ < b.seat_;
          });
    }
  }

  bool solve() { return kissat_solve(solver_) == 10; }

  lit_t create_lit() { return next_lit_++; }

  lit_t next_lit_;
  std::vector<std::vector<assignment>> seat_assignment_lits_;
  kissat* solver_;

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

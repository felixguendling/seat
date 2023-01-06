#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <tuple>
#include <vector>

#include "simp/SimpSolver.h"

#include "kissat.h"

#include "utl/enumerate.h"
#include "utl/raii.h"
#include "utl/timing.h"
#include "utl/verify.h"

#include "cista/containers/vecvec.h"

#include "seat/booking.h"
#include "seat/coloring_ilp.h"
#include "seat/flow_graph.h"
#include "seat/heuristic.h"
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

struct glucose_solver {
  using lit_t = Glucose::Lit;
  using var_t = Glucose::Var;
  using clause_t = Glucose::vec<Glucose::Lit>;
  glucose_solver() {
    solver_.setIncrementalMode();
    solver_.use_simplification = false;
  }
  void push(clause_t& cls, lit_t const lit) { cls.push(lit); }
  var_t create_var() { return solver_.newVar(); }
  lit_t make_lit(var_t const v) { return Glucose::mkLit(v); }
  lit_t negate(lit_t const l) { return ~l; }
  void add_clause(clause_t const& v) { solver_.addClause(v); }
  void add_clause(lit_t const x, lit_t const y) { solver_.addClause(x, y); }
  bool solve() { return solver_.solve(); }
  Glucose::SimpSolver solver_;
};

struct kissat_solver {
  using lit_t = int;
  using var_t = int;
  using clause_t = std::vector<lit_t>;
  static constexpr lit_t kFinish = 0;
  kissat_solver() {
    solver_ = kissat_init();
    utl::verify(solver_ != nullptr, "kissat solver init failed");
  }
  ~kissat_solver() { kissat_release(solver_); }
  void push(clause_t& cls, lit_t const lit) { cls.emplace_back(lit); }
  var_t create_var() { return next_var_++; }
  lit_t make_lit(var_t const v) { return v; }
  lit_t negate(lit_t const l) { return -l; }
  void add_clause(clause_t const& v) {
    for (auto const& lit : v) {
      kissat_add(solver_, lit);
    }
    kissat_add(solver_, kFinish);
  }
  void add_clause(lit_t const x, lit_t const y) {
    kissat_add(solver_, x);
    kissat_add(solver_, y);
    kissat_add(solver_, kFinish);
  }
  bool solve() { return kissat_solve(solver_) == 10; }
  kissat* solver_;
  int next_var_{1};
};

template <typename SATSolver>
struct interval_graph {
  using node_id_t = std::uint16_t;
  using seat_number_t = std::uint16_t;
  using clause_t = typename SATSolver::clause_t;
  using var_t = typename SATSolver::var_t;

  struct assignment {
    seat_number_t seat_;
    var_t var_;
  };

  interval_graph(std::vector<reservation> seats) : seats_{std::move(seats)} {
    reset();
  }

  void reset() {
    reservation_.clear();
    interval_.clear();
    neighbors_.clear();
    seat_assignment_lits.clear();

    std::cout << "RESET" << std::endl;
    solver = std::make_unique<SATSolver>();

    for (auto const& b : bookings_) {
      add_booking(b, true);
    }
  }

  void add_booking(booking const& b, bool replay = false) {
    if (!replay) {
      bookings_.emplace_back(b);
      return;
    }

    auto const node_id = neighbors_.size();

    std::vector<node_id_t> overlap;
    for (auto const& [node_id, interval] : utl::enumerate(interval_)) {
      if (b.interval_.overlaps(interval)) {
        overlap.emplace_back(node_id);
      }
    }
    auto const& neighbors = neighbors_.emplace_back(overlap);

    auto const& r = reservation_.emplace_back(b.r_);
    interval_.emplace_back(b.interval_);

    // Customer >=1 seat.
    seat_assignment_lits.emplace_back();
    clause_t clause;
    for (auto const& [seat_number, s_r] : utl::enumerate(seats_)) {
      if (matches(r, s_r)) {
        auto const var = solver->create_var();
        seat_assignment_lits.back().emplace_back(
            assignment{static_cast<seat_number_t>(seat_number), var});
        solver->push(clause, solver->make_lit(var));
      }
    }
    solver->add_clause(clause);

    // Overlapping bookings are not on the same seat.
    for (auto const& neighbor : neighbors) {
      auto const& seat_options_a = seat_assignment_lits[node_id];
      auto const& seat_options_b = seat_assignment_lits[neighbor];
      set_intersection(
          begin(seat_options_a), end(seat_options_a), begin(seat_options_b),
          end(seat_options_b),
          [&](assignment const& a, assignment const& b) {
            solver->add_clause(solver->negate(solver->make_lit(a.var_)),
                               solver->negate(solver->make_lit(b.var_)));
          },
          [](assignment const& a, assignment const& b) {
            return a.seat_ < b.seat_;
          });
    }
  }

  bool solve() {
    // solver->toDimacs("dimacs.txt");
    std::cout << "SOLVING" << std::endl;
    UTL_START_TIMING(solve);
    auto const sat = solver->solve();
    UTL_STOP_TIMING(solve);
    std::cout << "solve time: " << UTL_GET_TIMING_MS(solve) << "\n";
    return sat;
  }

  std::vector<booking> bookings_;

  std::unique_ptr<SATSolver> solver;
  std::vector<std::vector<assignment>> seat_assignment_lits;

  std::vector<reservation> seats_;
  std::vector<reservation> reservation_;
  std::vector<interval> interval_;
  std::vector<std::vector<node_id_t>> neighbors_;
};

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

  auto solver = interval_graph<kissat_solver>{seats};

  nogo_cache nogo;

  auto failed = 1000;
  auto success = 0U;
  auto nogo_count = 0U;
  auto i = 0U;
  auto solver_solved = 0U;
  std::vector<std::uint64_t> timings_vec;
  while (failed >= 0) {
    auto const b = generate_random_booking(
        {wish::kAny, wish::kAny, wish::kAny, wish::kAny}, number_of_segments);
    std::cout << "adding booking: #" << ++i << " " << b << " - ";
    try {
      solver.add_booking(b);

      UTL_START_TIMING(add_booking);
      auto feasible = !nogo.is_nogo(b);
      if (!feasible) {
        ++nogo_count;
      } else {
        solver.reset();
        feasible = solver.solve();
        ++solver_solved;

        if (!feasible) {
          nogo.add_entry(b);
        }
      }
      UTL_STOP_TIMING(add_booking);

      auto const timing = UTL_GET_TIMING_MS(add_booking);
      timings_vec.emplace_back(timing);

      std::cout << " -> " << (feasible ? "good" : "bad") << " [" << timing
                << "ms, solver=" << solver_solved << ", remaining=" << failed
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

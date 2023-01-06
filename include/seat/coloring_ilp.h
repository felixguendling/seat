#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include "utl/enumerate.h"

#include "ortools/linear_solver/linear_solver.h"

#include "seat/booking.h"

namespace seat {

namespace gor = operations_research;

constexpr auto const solver_name = "SCIP";

template <typename SATSolver>
struct interval_graph {
  using node_id_t = std::uint16_t;
  using seat_number_t = std::uint16_t;
  using clause_t = typename SATSolver::clause_t;
  using var_t = typename SATSolver::var_t;

  interval_graph(std::vector<reservation> s)
      : seat_properties_{std::move(s)},
        solver_{gor::MPSolver::CreateSolver(solver_name)},
  {}

  void reset() {}

  void add_booking(booking const& b, bool replay = false) {
    auto const node_id = neighbors_.size();

    std::vector<node_id_t> overlap;
    for (auto const& [node_id, interval] : utl::enumerate(interval_)) {
      if (b.interval_.overlaps(interval)) {
        overlap.emplace_back(node_id);
      }
    }
    neighbors_.emplace_back(overlap);
    booking_reservation_.emplace_back(b.r_);
    interval_.emplace_back(b.interval_);
  }

  bool solve() {
    for (auto const& [b_idx, b] : utl::enumerate(booking_reservation_)) {
      std::vector<interval> match_intervals;

      int from = -1, to = -1;
      for (auto const& [seat_idx, sr] : utl::enumerate(seat_properties_)) {
        auto const match = matches(b, sr);
        if (match && from == -1) {
          from = seat_idx;
          to = -1;
        }
        if (!match && from != -1) {
          to = seat_idx;
          match_intervals.emplace_back(from, to);
          from = -1;
        }
      }

      auto const color = node_colors_.emplace_back(solver_->MakeIntVar(
          0.0, match_intervals.back().to_, fmt::format("{}", b_idx)));

      auto j = 0U;
      for (auto const& [from, to] : match_intervals) {
        auto const v = solver_->MakeBoolVar(fmt::format("{}_h{}", b_idx, j++));
        auto const c = solver_->MakeRowConstraint();
        // c->
      }
    }
  }

  std::vector<reservation> seat_properties_;
  std::vector<reservation> booking_reservation_;
  std::vector<interval> interval_;
  std::vector<std::vector<node_id_t>> neighbors_;

  std::unique_ptr<operations_research::MPSolver> solver_;
  std::vector<operations_research::MPConstraint*> node_colors_;
};

}  // namespace seat

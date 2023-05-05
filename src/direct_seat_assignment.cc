#include "seat/direct_seat_assignment.h"

#include "utl/enumerate.h"
#include "utl/get_or_create.h"
#include "utl/verify.h"

#include "ortools/linear_solver/linear_solver.h"

namespace seat {

constexpr auto const solver_name = "SCIP";
solver_seat_direct::solver_seat_direct(
    uint32_t const& num_seg,
    std::map<seat_id_t, std::pair<wagon_id_t, reservation>> const&
        seat_attributes,
    std::vector<booking> const& bookings,
    std::vector<booking_id_t> const& mcf_ids,
    std::vector<booking_id_t> const& concrete_ids,
    std::vector<booking_id_t> const& pseudo_ids,
    std::vector<booking_id_t> const& gsd_ids,
    std::vector<seat_id_t> const& gsd_seats, uint32_t const& total_seats,
    train const& train)
    : solver_{gor::MPSolver::CreateSolver(solver_name)},
      number_of_segments_(num_seg),
      bookings_(bookings),
      gsd_seats_(gsd_seats),
      mcf_booking_ids_(mcf_ids),
      gsd_ids_(gsd_ids),
      seat_attributes_(seat_attributes),
      total_seats_(total_seats),
      train_(train) {
  for (auto const& id : concrete_ids) {
    mcf_booking_ids_.emplace_back(id);
  }
  for (auto const& id : pseudo_ids) {
    mcf_booking_ids_.emplace_back(id);
  }
}

void solver_seat_direct::solve(std::vector<seat_id_t> const& hint_seats) {
  seats_by_bookings_.first = std::vector<booking_id_t>();
  seats_by_bookings_.second = std::vector<seat_id_t>();
  reset();
  create_mcf_problem();
  set_hint(hint_seats);
  create_objective();
  // print();
  result_ = solver_->Solve();
  assign_seats();
  if (!feasible()) {
    abort();
  }
}

bool solver_seat_direct::feasible() const {
  return result_ == gor::MPSolver::OPTIMAL ||
         result_ == gor::MPSolver::FEASIBLE;
}

void solver_seat_direct::create_mcf_problem() {
  for (auto const& b_id : mcf_booking_ids_) {
    auto b = bookings_[b_id];
    auto from = b.interval_.from_;
    auto const to = b.interval_.to_;
    for (auto const& [s_id, res_wagon_pair] : seat_attributes_) {
      if (!matches(b.r_, res_wagon_pair.second)) {
        continue;
      }
      if (is_gsd_blocked(b.interval_, s_id)) {
        continue;
      }
      auto source_constraint = get_source_constraint(b_id);
      source_constraint->SetBounds(1, 1);
      from = b.interval_.from_;
      auto var = get_var(b_id, s_id);
      var->SetBounds(0, 1);
      source_constraint->SetCoefficient(var, 1);
      while (from != to) {
        auto capacity_constraint = get_capacity_constraint(s_id, from);
        capacity_constraint->SetCoefficient(var, 1);
        capacity_constraint->SetBounds(0, 1);
        ++from;
      }
    }
  }
}

bool solver_seat_direct::is_gsd_blocked(interval const& inter,
                                        seat_id_t const s_id) {
  auto check = [&](std::vector<seat_id_t> s_ids,
                   std::vector<booking_id_t> b_ids) {
    for (auto const& [idx, gsd_s_id] : utl::enumerate(s_ids)) {
      if (s_id != gsd_s_id) {
        continue;
      }
      if (inter.touches(bookings_[b_ids[idx]].interval_)) {
        return true;
      }
    }
    return false;
  };
  return check(gsd_seats_, gsd_ids_);
}

gor::MPVariable* solver_seat_direct::get_var(booking_id_t const& b_id,
                                             seat_id_t const& seat_id) {
  gor::MPVariable* v =
      solver_->MakeIntVar(0.0, 1.0, fmt::format("x_{}_{}", b_id, seat_id));
  vars_.emplace(std::make_pair(b_id, seat_id), v);
  return v;
}

gor::MPConstraint* solver_seat_direct::get_source_constraint(
    booking_id_t const& b_id) {
  return utl::get_or_create(source_constraints_, b_id, [&]() {
    return solver_->MakeRowConstraint(fmt::format("s_{}", b_id));
  });
}

gor::MPConstraint* solver_seat_direct::get_capacity_constraint(
    seat_id_t const& seat_id, small_station_id_t const station_id) {
  return utl::get_or_create(
      capacity_constraints_, std::make_pair(seat_id, station_id), [&]() {
        return solver_->MakeRowConstraint(
            fmt::format("cap_{}_{}", seat_id, station_id));
      });
}

void solver_seat_direct::print() const {
  std::string s;
  solver_->ExportModelAsLpFormat(false, &s);
  std::cout << s;
}

void solver_seat_direct::print_sizes() const {
  std::cout << "vars: " << solver_->NumVariables() << "\n"
            << "constraints: " << solver_->NumConstraints() << "\n";
}

void solver_seat_direct::print_name() const { std::cout << "seat assignment"; }

void solver_seat_direct::assign_seats() {
  for (auto seat_id = seat_id_t{0}; seat_id != total_seats_; ++seat_id) {
    for (auto const& [id, b_id] : utl::enumerate(mcf_booking_ids_)) {
      if (!matches(bookings_[b_id].r_, seat_attributes_[seat_id].second)) {
        continue;
      }
      if (is_gsd_blocked(bookings_[b_id].interval_, seat_id)) {
        continue;
      }
      auto key = std::make_pair(b_id, seat_id);
      if (vars_.find(key) == end(vars_)) {
        continue;
      }
      if (vars_[key]->solution_value() == 1) {
        seats_by_bookings_.first.emplace_back(b_id);
        seats_by_bookings_.second.emplace_back(seat_id);
      }
    }
  }
}

void solver_seat_direct::set_hint(std::vector<seat_id_t> const& seat_ids) {
  solver_->SetSolverSpecificParametersAsString(
      "heuristics/completesol/maxunknownrate = 1");
  std::vector<std::pair<const gor::MPVariable*, double>> hint;
  for (auto const& [pair, var] : vars_) {
    auto b_id = pair.first;
    auto s_id = pair.second;
    if (seat_ids[b_id] == s_id) {
      hint.emplace_back(std::make_pair(var, 1));
    } else {
      hint.emplace_back(std::make_pair(var, 0));
    }
  }
  auto sum = 0;
  for (auto const& h : hint) {
    if (h.second > 0) {
      ++sum;
    }
  }
  solver_->SetHint(hint);
  std::cout << "hint size: " << hint.size() << " sum: " << sum << "\n";
}

void solver_seat_direct::reset() {
  solver_->Clear();
  result_ = gor::MPSolver::INFEASIBLE;
  vars_.clear();
  source_constraints_.clear();
  capacity_constraints_.clear();
  row_constraints_.clear();
}

void solver_seat_direct::create_objective() {
  gor::MPObjective* const objective = solver_->MutableObjective();
  auto max_group_id = 0;
  for (auto const& b : bookings_) {
    if (b.group_id_ > max_group_id) {
      max_group_id = b.group_id_;
    }
  }
  std::map<group_id_t, gor::MPVariable*> objective_max_helper_vars;
  std::map<group_id_t, gor::MPVariable*> objective_min_helper_vars;
  for (auto group_id = 1; group_id != max_group_id + 1; ++group_id) {
    auto h = solver_->MakeBoolVar(fmt::format("h_{}", group_id));
    auto h_c = solver_->MakeRowConstraint(fmt::format("h_c_{}", group_id));
    h_c->SetBounds(-10000, 0);
    h_c->SetCoefficient(h, -(max_group_id + 1));
    auto abs_upper_bound_constraint = solver_->MakeRowConstraint(
        fmt::format("helper_abs_upper_{}", group_id));
    auto abs_lower_bound_constraint = solver_->MakeRowConstraint(
        fmt::format("helper_abs_lower_{}", group_id));
    auto abs_helper =
        solver_->MakeIntVar(-operations_research::MPSolver::infinity(),
                            operations_research::MPSolver::infinity(),
                            fmt::format("helper_{}", group_id));
    objective->SetCoefficient(h, 1);
    abs_upper_bound_constraint->SetBounds(-10000, 0);
    abs_lower_bound_constraint->SetBounds(-10000, 0);
    objective_max_helper_vars.emplace(
        group_id,
        solver_->MakeIntVar(-operations_research::MPSolver::infinity(),
                            operations_research::MPSolver::infinity(),
                            fmt::format("helper_max_{}", group_id)));
    objective_min_helper_vars.emplace(
        group_id,
        solver_->MakeIntVar(-operations_research::MPSolver::infinity(),
                            operations_research::MPSolver::infinity(),
                            fmt::format("helper_min_{}", group_id)));
    /*objective_abs_helper_vars_.emplace(
        group_id,
        solver_->MakeIntVar(-operations_research::MPSolver::infinity(),
                            operations_research::MPSolver::infinity(),
                            fmt::format("helper__abs{}", group_id)));*/
    abs_lower_bound_constraint->SetCoefficient(
        objective_max_helper_vars[group_id], -1);
    abs_lower_bound_constraint->SetCoefficient(
        objective_min_helper_vars[group_id], 1);
    abs_lower_bound_constraint->SetCoefficient(abs_helper, -1);

    abs_upper_bound_constraint->SetCoefficient(
        objective_max_helper_vars[group_id], 1);
    abs_upper_bound_constraint->SetCoefficient(
        objective_min_helper_vars[group_id], -1);
    abs_upper_bound_constraint->SetCoefficient(abs_helper, -1);

    objective->SetCoefficient(objective_max_helper_vars[group_id], 1);
    objective->SetCoefficient(objective_min_helper_vars[group_id], -1);
    h_c->SetCoefficient(objective_max_helper_vars[group_id], 1);
    h_c->SetCoefficient(objective_min_helper_vars[group_id], -1);
    for (auto const& [idx, booking] : utl::enumerate(bookings_)) {
      auto constraint_max =
          solver_->MakeRowConstraint(fmt::format("o_max_{}_{}", group_id, idx));
      auto constraint_min =
          solver_->MakeRowConstraint(fmt::format("o_min_{}_{}", group_id, idx));
      if (booking.group_id_ != group_id) {
        continue;
      }
      constraint_max->SetBounds(-10000, 0);
      constraint_min->SetBounds(-10000, 0);
      constraint_max->SetCoefficient(objective_max_helper_vars[group_id], -1);
      constraint_min->SetCoefficient(objective_min_helper_vars[group_id], 1);
      for (auto const& [pair, var] : vars_) {
        auto const b_id = pair.first;
        if (b_id != idx) {
          continue;
        }
        auto const s_id = pair.second;
        auto row =
            train_.seat_id_to_row_id(s_id, train_.seat_id_to_wagon_id(s_id));
        constraint_max->SetCoefficient(var, static_cast<int>(row));
        constraint_min->SetCoefficient(var, -static_cast<int>(row));
      }
    }
  }
}

}  // namespace seat

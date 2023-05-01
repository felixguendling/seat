#include "seat/wagon_solver.h"

#include "utl/enumerate.h"
#include "utl/get_or_create.h"
#include "utl/verify.h"

#include "ortools/linear_solver/linear_solver.h"

namespace seat {

constexpr auto const solver_name = "SCIP";
solver_wagon::solver_wagon(uint32_t const& num_seg,
                           std::map<std::pair<wagon_id_t, reservation>,
                                    uint32_t> const& wagon_res_capacities,
                           std::vector<booking> const& bookings,
                           std::vector<booking_id_t> const& mcf_ids,
                           std::vector<booking_id_t> const& concrete_ids,
                           std::vector<booking_id_t> const& pseudo_ids,
                           std::vector<booking_id_t> const& gsd_ids,
                           std::vector<seat_id_t> const& pseudo_seats,
                           std::vector<seat_id_t> const& gsd_seats)
    : solver_{gor::MPSolver::CreateSolver(solver_name)},
      number_of_segments_(num_seg),
      bookings_(bookings),
      pseudo_seats_(pseudo_seats),
      gsd_seats_(gsd_seats),
      mcf_booking_ids_(mcf_ids),
      gsd_ids_(gsd_ids),
      pseudo_ids_(pseudo_ids),
      wagon_res_capacities_(wagon_res_capacities) {
  for (auto const& id : concrete_ids) {
    mcf_booking_ids_.emplace_back(id);
  }
  for (auto const& id : pseudo_ids_) {
    mcf_booking_ids_.emplace_back(id);
  }
}

bool solver_wagon::solve() {
  result_ = solver_->Solve();
  return feasible();
};

bool solver_wagon::feasible() const {
  return result_ == gor::MPSolver::OPTIMAL ||
         result_ == gor::MPSolver::FEASIBLE;
}

void solver_wagon::create_mcf_problem(train& t) {
  for (auto const& b_id : mcf_booking_ids_) {
    auto b = bookings_[b_id];
    auto source_constraint = get_source_constraint(b_id);
    source_constraint->SetBounds(1, 1);
    auto from = b.interval_.from_;
    auto const to = b.interval_.to_;
    for (auto const& [wagon_res_pair, capacity] : wagon_res_capacities_) {
      auto r = wagon_res_pair.second;
      auto w_id = wagon_res_pair.first;
      if (!matches(b.r_, r)) {
        continue;
      }
      from = b.interval_.from_;
      auto var = get_var(b_id, w_id, r);
      var->SetBounds(0, 1);
      source_constraint->SetCoefficient(var, 1);
      while (from != to) {
        auto capacity_constraint = get_capacity_constraint(w_id, from, r);
        capacity_constraint->SetCoefficient(var, 1);
        capacity_constraint->SetBounds(0,
                                       wagon_res_capacities_[wagon_res_pair]);
        ++from;
      }
    }
  }
  for (auto const& [idx, gsd_seat] : utl::enumerate(gsd_seats_)) {
    auto interv = bookings_[gsd_ids_[idx]].interval_;
    for (auto i = interv.from_; i != interv.to_; ++i) {
      auto constraint = capacity_constraints_[std::make_pair(
          bookings_[gsd_ids_[idx]].r_,
          std::make_pair(t.seat_id_to_wagon_id(gsd_seat), i))];
      constraint->SetBounds(0, constraint->ub() - 1);
    }
  }
}

gor::MPVariable* solver_wagon::get_var(booking_id_t const& b_id,
                                       wagon_id_t const& wagon_id,
                                       reservation const& res) {
  gor::MPVariable* v = solver_->MakeIntVar(
      0.0, 1.0, fmt::format("x_{}_{}_{}", b_id, wagon_id, res));
  vars_.emplace(std::make_pair(std::make_pair(b_id, wagon_id), res), v);
  return v;
}

gor::MPConstraint* solver_wagon::get_source_constraint(
    booking_id_t const& b_id) {
  return utl::get_or_create(source_constraints_, b_id, [&]() {
    return solver_->MakeRowConstraint(fmt::format("s_{}", b_id));
  });
}

gor::MPConstraint* solver_wagon::get_capacity_constraint(
    wagon_id_t const& wagon_id, small_station_id_t const station_id,
    reservation const& res) {
  return utl::get_or_create(
      capacity_constraints_,
      std::make_pair(res, std::make_pair(wagon_id, station_id)), [&]() {
        return solver_->MakeRowConstraint(
            fmt::format("cap_{}_{}_{}", wagon_id, station_id, res));
      });
}

std::pair<std::vector<booking_id_t>, std::vector<wagon_id_t>>
solver_wagon::assign_seats() {
  auto b_ids = std::vector<booking_id_t>();
  auto w_ids = std::vector<wagon_id_t>();
  for (auto const& [id, b_id] : utl::enumerate(mcf_booking_ids_)) {
    for (auto const& [pair, capacity] : wagon_res_capacities_) {
      auto w_id = pair.first;
      auto r = pair.second;
      if (!matches(bookings_[b_id].r_, r)) {
        continue;
      }
      if (vars_[std::make_pair(std::make_pair(b_id, w_id), r)]
              ->solution_value() == 1) {
        b_ids.emplace_back(b_id);
        w_ids.emplace_back(w_id);
      }
    }
  }
  return std::make_pair(b_ids, w_ids);
}

void solver_wagon::print() const {
  std::string s;
  solver_->ExportModelAsLpFormat(false, &s);
  std::cout << s;
}

void solver_wagon::print_sizes() const {
  std::cout << "vars: " << solver_->NumVariables() << "\n"
            << "constraints: " << solver_->NumConstraints() << "\n";
}

void solver_wagon::print_name() const { std::cout << "seat assignment"; }

void solver_wagon::create_objective(wagon_id_t const& max_wagon_id) {
  gor::MPObjective* const objective = solver_->MutableObjective();
  auto max_group_id = 0;
  for (auto const& b : bookings_) {
    if (b.group_id_ > max_group_id) {
      max_group_id = b.group_id_;
    }
  }
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
    objective_max_helper_vars_.emplace(
        group_id,
        solver_->MakeIntVar(-operations_research::MPSolver::infinity(),
                            operations_research::MPSolver::infinity(),
                            fmt::format("helper_max_{}", group_id)));
    objective_min_helper_vars_.emplace(
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
        objective_max_helper_vars_[group_id], -1);
    abs_lower_bound_constraint->SetCoefficient(
        objective_min_helper_vars_[group_id], 1);
    abs_lower_bound_constraint->SetCoefficient(abs_helper, -1);

    abs_upper_bound_constraint->SetCoefficient(
        objective_max_helper_vars_[group_id], 1);
    abs_upper_bound_constraint->SetCoefficient(
        objective_min_helper_vars_[group_id], -1);
    abs_upper_bound_constraint->SetCoefficient(abs_helper, -1);

    objective->SetCoefficient(objective_max_helper_vars_[group_id], 1);
    objective->SetCoefficient(objective_min_helper_vars_[group_id], -1);
    h_c->SetCoefficient(objective_max_helper_vars_[group_id], 1);
    h_c->SetCoefficient(objective_min_helper_vars_[group_id], -1);
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
      constraint_max->SetCoefficient(objective_max_helper_vars_[group_id], -1);
      constraint_min->SetCoefficient(objective_min_helper_vars_[group_id], 1);
      for (auto const& [pair, var] : vars_) {
        auto const b_id = pair.first.first;
        if (b_id != idx) {
          continue;
        }
        auto const w_id = pair.first.second;
        constraint_max->SetCoefficient(var, w_id);
        constraint_min->SetCoefficient(var, -w_id);
      }
    }
  }
}

void solver_wagon::set_hint(
    std::vector<wagon_id_t> const& wagons_by_booking_ids) {
  solver_->SetSolverSpecificParametersAsString(
      "heuristics/completesol/maxunknownrate = 1");
  std::vector<std::pair<const gor::MPVariable*, double>> hint;
  for (auto const& [pair, var] : vars_) {
    wagon_id_t const w_id = pair.first.second;
    booking_id_t const b_id = pair.first.first;
    if (wagons_by_booking_ids[b_id] == w_id) {
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
  std::vector<int> found = std::vector<int>();
  solver_->SetHint(hint);
  std::cout << "hint size: " << hint.size() << " sum: " << sum << "\n";
}

int solver_wagon::print_helpers(bool const print) {
  auto error_counter = 0;
  for (auto const& [id, v] : objective_max_helper_vars_) {
    if (objective_max_helper_vars_[id]->solution_value() -
            objective_min_helper_vars_[id]->solution_value() >
        0) {
      error_counter++;
    }
    if (!print) {
      continue;
    }
    std::cout << "group id " << id
              << ": max: " << objective_max_helper_vars_[id]->solution_value()
              << ", min: " << objective_min_helper_vars_[id]->solution_value()
              << ", diff: "
              << objective_max_helper_vars_[id]->solution_value() -
                     objective_min_helper_vars_[id]->solution_value()
              << "\n";
  }
  return error_counter;
}

void solver_wagon::reset() {
  solver_->Reset();
  result_ = gor::MPSolver::INFEASIBLE;
  wagon_res_capacities_.clear();
  mcf_booking_ids_.clear();
  bookings_.clear();
  pseudo_seats_.clear();
  gsd_seats_.clear();
  gsd_ids_.clear();
  pseudo_ids_.clear();
  vars_.clear();
  source_constraints_.clear();
  capacity_constraints_.clear();
  objective_max_helper_vars_.clear();
  objective_min_helper_vars_.clear();
  objective_abs_helper_vars_.clear();
}
bool solver_wagon::solve(int i) {
  solver_->SetTimeLimit(absl::Minutes(i));
  return solve();
}

}  // namespace seat

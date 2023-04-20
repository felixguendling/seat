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
}

bool solver_wagon::solve() {
  result_ = solver_->Solve();
  return feasible();
};

bool solver_wagon::feasible() const {
  return result_ == gor::MPSolver::OPTIMAL ||
         result_ == gor::MPSolver::FEASIBLE;
}

void solver_wagon::create_mcf_problem() {
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

void solver_wagon::create_objective() {
  gor::MPObjective* const objective = solver_->MutableObjective();
  auto max_group_id =
      bookings_[std::max_element(
                    mcf_booking_ids_.begin(), mcf_booking_ids_.end(),
                    [&](booking_id_t const& b1, booking_id_t const& b2) {
                      return bookings_[b1].group_id_ < bookings_[b2].group_id_;
                    }) -
                mcf_booking_ids_.begin()]
          .group_id_;
  for (auto group_id = 1; group_id != max_group_id + 1; ++group_id) {
    for (auto const& [idx, booking] : utl::enumerate(bookings_)) {
      if (booking.group_id_ != group_id) {
        continue;
      }
      for (auto const& [idx2, booking2] : utl::enumerate(bookings_)) {
        if (booking2.group_id_ != group_id) {
          continue;
        }
        if (idx == idx2) {
          continue;
        }
        auto constraint1 =
            solver_->MakeRowConstraint(fmt::format("o_{}_{}_p", idx, idx2));
        auto constraint2 =
            solver_->MakeRowConstraint(fmt::format("o_{}_{}_n", idx, idx2));
        auto ccc = gor::MPSolver::infinity();
        constraint1->SetBounds(-5000, 0);
        constraint2->SetBounds(-5000, 0);
        objective_helper_vars_.emplace(
            std::make_pair(idx, idx2),
            solver_->MakeIntVar(
                -operations_research::MPSolver::infinity(),
                operations_research::MPSolver::infinity(),
                fmt::format("helper_{}_{}_{}", booking.group_id_, idx, idx2)));
        auto new_var = objective_helper_vars_.at(std::make_pair(idx, idx2));
        constraint1->SetCoefficient(new_var, -1);
        constraint2->SetCoefficient(new_var, -1);
        objective->SetCoefficient(new_var, 1);
        for (auto const& [pair, var] : vars_) {
          auto b_id = pair.first.first;
          if (b_id != idx && b_id != idx2) {
            continue;
          }
          auto sign = (b_id == idx) ? 1 : -1;
          constraint1->SetCoefficient(var, sign * pair.first.second);
          constraint2->SetCoefficient(var, -sign * pair.first.second);
        }
      }
    }
  }
  print();
}

void solver_wagon::set_hint(
    std::vector<wagon_id_t> const& wagons_by_booking_ids) {
  solver_->SetSolverSpecificParametersAsString(
      "heuristics/completesol/maxunknownrate = 1");
  std::vector<std::pair<const gor::MPVariable*, double>> hint;
  for (auto const& [pair, var] : vars_) {
    auto w_id = pair.first.second;
    if (wagons_by_booking_ids[w_id] == w_id) {
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

}  // namespace seat

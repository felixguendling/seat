#include "seat/pseudo_gsd_chooser.h"

#include "utl/verify.h"

#include "ortools/linear_solver/linear_solver.h"
#include "utl/enumerate.h"

namespace seat {

constexpr auto const solver_name = "SCIP";
pseudo_gsd_chooser::pseudo_gsd_chooser(
    std::vector<booking_id_t> const& b_ids,
    std::vector<booking> const& bookings, uint8_t const segments,
    interval const& gsd_interval,
    std::vector<small_station_id_t> const& tight_capacities)
    : solver_{gor::MPSolver::CreateSolver(solver_name)} {
  // b_ids should contain no ids of bookings that overlap the gsd_interval
  b_ids_ = b_ids;
  // create vars
  for (auto const& id : b_ids_) {
    auto v = solver_->MakeBoolVar(fmt::format("var_{}", id));
    vars_.emplace(id, v);
  }
  // create constraints
  for (auto i = small_station_id_t{0}; i != segments; ++i) {
    if (gsd_interval.from_ <= i && gsd_interval.to_ > i) {
      continue;
    }
    auto constraint = solver_->MakeRowConstraint(fmt::format("c_{}", i));
    // set coefficients of all vars in current constraint
    for (auto const& id : b_ids_) {
      if (bookings[id].interval_.from_ <= i && bookings[id].interval_.to_ > i) {
        constraint->SetCoefficient(vars_[id], 1.0);
      }
    }
    // decide bounds by checking whether associated capacity is tight
    if (std::find(tight_capacities.begin(), tight_capacities.end(), i) !=
        tight_capacities.end()) {
      constraint->SetBounds(1.0, 1.0);
      constraints_.emplace_back(constraint);
      continue;
    }
    constraint->SetBounds(0.0, 1.0);
    constraints_.emplace_back(constraint);
  }
  solver_->Solve();
  for (auto const& [id, var] : vars_) {
    if (var->solution_value() > 0) {
      chosen_bookings_.emplace_back(id);
    }
  }
}

void pseudo_gsd_chooser::print() const {
  std::string s;
  solver_->ExportModelAsLpFormat(false, &s);
  std::cout << s;
}
}  // namespace seat
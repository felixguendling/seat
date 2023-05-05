#include "seat/seat_from_wagon_assignment.h"

#include "utl/enumerate.h"
#include "utl/get_or_create.h"
#include "utl/verify.h"

#include "ortools/linear_solver/linear_solver.h"

namespace seat {

constexpr auto const solver_name = "SCIP";
solver_seat_from_wagon::solver_seat_from_wagon(
    uint32_t const& num_seg,
    std::pair<std::vector<booking_id_t>, std::vector<wagon_id_t>> const& w_ids,
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
      wagon_id_by_booking_ids_(w_ids),
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

void solver_seat_from_wagon::solve() {
  seats_by_bookings_.first = std::vector<booking_id_t>();
  seats_by_bookings_.second = std::vector<seat_id_t>();
  auto max_wagon_id = 0;
  for (auto const& w_id : wagon_id_by_booking_ids_.second) {
    if (w_id > max_wagon_id) {
      max_wagon_id = w_id;
    }
  }
  std::cout << "\n";
  for (auto const& [idx, gsd] : utl::enumerate(gsd_seats_)) {
    auto interv = bookings_[gsd_ids_[idx]].interval_;
    std::cout << "gsd seat: " << gsd << " from: " << +interv.from_
              << " to: " << +interv.to_ << "\n";
  }
  for (auto i = wagon_id_t{0}; i != max_wagon_id + 1; ++i) {
    reset();
    create_mcf_problem(i);
    // create_objective(i);
    // print();
    result_ = solver_->Solve();
    assign_seats(i);
    if (!feasible()) {
      abort();
    }
  }
};

bool solver_seat_from_wagon::feasible() const {
  return result_ == gor::MPSolver::OPTIMAL ||
         result_ == gor::MPSolver::FEASIBLE;
}

void solver_seat_from_wagon::create_mcf_problem(wagon_id_t const& w_id) {
  for (auto const& b_id : mcf_booking_ids_) {
    auto b = bookings_[b_id];
    auto pos = find(wagon_id_by_booking_ids_.first.begin(),
                    wagon_id_by_booking_ids_.first.end(), b_id) -
               wagon_id_by_booking_ids_.first.begin();
    if (w_id != wagon_id_by_booking_ids_.second[pos]) {
      continue;
    }
    auto from = b.interval_.from_;
    auto const to = b.interval_.to_;
    for (auto const& [s_id, res_wagon_pair] : seat_attributes_) {
      if (w_id != res_wagon_pair.first) {
        continue;
      }
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

bool solver_seat_from_wagon::is_gsd_blocked(interval const& inter,
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

gor::MPVariable* solver_seat_from_wagon::get_var(booking_id_t const& b_id,
                                                 seat_id_t const& seat_id) {
  gor::MPVariable* v =
      solver_->MakeIntVar(0.0, 1.0, fmt::format("x_{}_{}", b_id, seat_id));
  vars_.emplace(std::make_pair(b_id, seat_id), v);
  return v;
}

gor::MPConstraint* solver_seat_from_wagon::get_source_constraint(
    booking_id_t const& b_id) {
  return utl::get_or_create(source_constraints_, b_id, [&]() {
    return solver_->MakeRowConstraint(fmt::format("s_{}", b_id));
  });
}

gor::MPConstraint* solver_seat_from_wagon::get_capacity_constraint(
    seat_id_t const& seat_id, small_station_id_t const station_id) {
  return utl::get_or_create(
      capacity_constraints_, std::make_pair(seat_id, station_id), [&]() {
        return solver_->MakeRowConstraint(
            fmt::format("cap_{}_{}", seat_id, station_id));
      });
}

void solver_seat_from_wagon::print() const {
  std::string s;
  solver_->ExportModelAsLpFormat(false, &s);
  std::cout << s;
}

void solver_seat_from_wagon::print_sizes() const {
  std::cout << "vars: " << solver_->NumVariables() << "\n"
            << "constraints: " << solver_->NumConstraints() << "\n";
}

void solver_seat_from_wagon::print_name() const {
  std::cout << "seat assignment";
}

void solver_seat_from_wagon::assign_seats(wagon_id_t const& w_id) {
  for (auto const& [id, b_id] : utl::enumerate(mcf_booking_ids_)) {
    for (auto seat_id = seat_id_t{0}; seat_id != total_seats_; ++seat_id) {
      auto pos = find(wagon_id_by_booking_ids_.first.begin(),
                      wagon_id_by_booking_ids_.first.end(), b_id) -
                 wagon_id_by_booking_ids_.first.begin();
      if (w_id != train_.seat_id_to_wagon_id(seat_id)) {
        continue;
      }
      if (wagon_id_by_booking_ids_.second[pos] != w_id) {
        continue;
      }
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

void solver_seat_from_wagon::set_hint(
    std::pair<std::vector<booking_id_t>, std::vector<wagon_id_t>> const&
        seats_by_booking_ids) {
  solver_->SetSolverSpecificParametersAsString(
      "heuristics/completesol/maxunknownrate = 1");
  std::vector<std::pair<const gor::MPVariable*, double>> hint;
  for (auto const& [pair, var] : vars_) {
    auto b_id = pair.first;
    auto s_id = pair.second;
    auto pos = find(seats_by_booking_ids.first.begin(),
                    seats_by_booking_ids.first.end(), b_id) -
               seats_by_booking_ids.first.begin();
    if (seats_by_booking_ids.second[pos] == s_id) {
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

void solver_seat_from_wagon::reset() {
  solver_->Clear();
  result_ = gor::MPSolver::INFEASIBLE;
  vars_.clear();
  source_constraints_.clear();
  capacity_constraints_.clear();
  row_constraints_.clear();
}

void solver_seat_from_wagon::create_objective(wagon_id_t const w_id) {
  create_obj_constraints(row_constraints_, [&](seat_id_t const& s_id) {
    return train_.seat_id_to_row_id(s_id, w_id);
  });
  create_obj_constraints(lr_constraints_, [&](seat_id_t const& s_id) {
    return train_.seat_id_to_lr(s_id, w_id);
  });
  create_obj_constraints(corridor_constraints_, [&](seat_id_t const& s_id) {
    return seat_attributes_[s_id].second[0] == wish::kNo;
  });
  create_obj_constraints(big_constraints_, [&](seat_id_t const& s_id) {
    return seat_attributes_[s_id].second[3] == wish::kYes;
  });
}

}  // namespace seat

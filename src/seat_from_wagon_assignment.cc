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
    std::vector<seat_id_t> const& pseudo_seats,
    std::vector<seat_id_t> const& gsd_seats, uint32_t const& total_seats,
    train const& train)
    : solver_{gor::MPSolver::CreateSolver(solver_name)},
      wagon_id_by_booking_ids_(w_ids),
      number_of_segments_(num_seg),
      bookings_(bookings),
      pseudo_seats_(pseudo_seats),
      gsd_seats_(gsd_seats),
      mcf_booking_ids_(mcf_ids),
      gsd_ids_(gsd_ids),
      pseudo_ids_(pseudo_ids),
      seat_attributes_(seat_attributes),
      total_seats_(total_seats),
      train_(train) {
  for (auto const& id : concrete_ids) {
    mcf_booking_ids_.emplace_back(id);
  }
}

bool solver_seat_from_wagon::solve() {
  result_ = solver_->Solve();
  return feasible();
};

bool solver_seat_from_wagon::feasible() const {
  return result_ == gor::MPSolver::OPTIMAL ||
         result_ == gor::MPSolver::FEASIBLE;
}

void solver_seat_from_wagon::create_mcf_problem() {
  for (auto const& b_id : mcf_booking_ids_) {
    auto b = bookings_[b_id];
    auto source_constraint = get_source_constraint(b_id);
    source_constraint->SetBounds(1, 1);
    auto from = b.interval_.from_;
    auto const to = b.interval_.to_;
    for (auto const& [s_id, res_wagon_pair] : seat_attributes_) {
      if (!matches(b.r_, res_wagon_pair.second)) {
        continue;
      }
      auto pos = find(wagon_id_by_booking_ids_.first.begin(),
                      wagon_id_by_booking_ids_.first.end(), b_id) -
                 wagon_id_by_booking_ids_.first.begin();
      if (res_wagon_pair.first != wagon_id_by_booking_ids_.second[pos]) {
        continue;
      }
      if (is_gsd_blocked(b.interval_, s_id)) {
        continue;
      }
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

// void solver_seat_from_wagon::create_arbitrary_valid_solution() { for (auto
// const& r :) }

bool solver_seat_from_wagon::is_gsd_blocked(interval const& inter,
                                            seat_id_t const s_id) {
  auto check = [&](std::vector<seat_id_t> s_ids,
                   std::vector<booking_id_t> b_ids) {
    for (auto const& [idx, gsd_s_id] : utl::enumerate(s_ids)) {
      if (s_id != gsd_s_id) {
        continue;
      }
      if (inter.overlaps(bookings_[b_ids[idx]].interval_)) {
        return true;
      }
    }
    return false;
  };
  if (check(pseudo_seats_, pseudo_ids_)) {
    return true;
  }
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

std::pair<std::vector<booking_id_t>, std::vector<seat_id_t>>
solver_seat_from_wagon::assign_seats() {
  auto b_ids = std::vector<booking_id_t>();
  auto s_ids = std::vector<seat_id_t>();
  for (auto const& [id, b_id] : utl::enumerate(mcf_booking_ids_)) {
    for (auto seat_id = seat_id_t{0}; seat_id != total_seats_; ++seat_id) {
      auto pos = find(wagon_id_by_booking_ids_.first.begin(),
                      wagon_id_by_booking_ids_.first.end(), b_id) -
                 wagon_id_by_booking_ids_.first.begin();
      auto w_id = train_.seat_id_to_wagon_id(seat_id);
      if (wagon_id_by_booking_ids_.second[pos] != w_id) {
        continue;
      }
      if (!matches(bookings_[b_id].r_, seat_attributes_[seat_id].second)) {
        continue;
      }
      if (is_gsd_blocked(bookings_[b_id].interval_, seat_id)) {
        continue;
      }
      if (vars_[std::make_pair(b_id, seat_id)]->solution_value() == 1) {
        b_ids.emplace_back(b_id);
        s_ids.emplace_back(seat_id);
      }
    }
  }
  return std::make_pair(b_ids, s_ids);
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

void solver_seat_from_wagon::create_objective() {
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
            solver_->MakeRowConstraint(fmt::format("o_{}_{}", idx, idx2));
        auto constraint2 =
            solver_->MakeRowConstraint(fmt::format("o_{}_{}", idx, idx2));
        objective_helper_vars_.emplace(
            std::make_pair(idx, idx2),
            solver_->MakeIntVar(-solver_->infinity(), solver_->infinity(),
                                fmt::format("helper_{}_{}", idx, idx2)));
        auto new_var = objective_helper_vars_.at(std::make_pair(idx, idx2));
        constraint1->SetCoefficient(new_var, -1);
        constraint2->SetCoefficient(new_var, -1);
        objective->SetCoefficient(new_var, 1);
        for (auto const& [pair, var] : vars_) {
          auto b_id = pair.first;
          if (b_id != idx && b_id != idx2) {
            continue;
          }
          auto sign = (b_id == idx) ? 1 : -1;
          constraint1->SetCoefficient(
              var, sign * train_.seat_id_to_wagon_id(pair.second));
          constraint2->SetCoefficient(
              var, sign * train_.seat_id_to_wagon_id(pair.second));
        }
      }
    }
  }
  print();
}

}  // namespace seat

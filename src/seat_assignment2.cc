#include "seat/seat_assignment2.h"

#include "utl/enumerate.h"
#include "utl/get_or_create.h"
#include "utl/verify.h"

#include "ortools/linear_solver/linear_solver.h"

namespace seat {

constexpr auto const solver_name = "SCIP";
solver_seat2::solver_seat2(
    uint32_t const& num_seg,
    std::map<seat_id_t, std::pair<wagon_id_t, reservation>> const&
        seat_attributes,
    cista::raw::vector_map<booking_id_t, booking> const& bookings,
    std::map<seat_id_t, std::vector<booking>> const& gsd_bookings,
    uint32_t const& total_seats)
    : solver_{gor::MPSolver::CreateSolver(solver_name)},
      //      objective_{solver_->Objective()},
      number_of_segments_(num_seg),
      mcf_bookings_(bookings),
      pseudo_gsd_bookings_(gsd_bookings),
      seat_attributes_(seat_attributes),
      total_seats_(total_seats) {}

bool solver_seat2::solve() {
  result_ = solver_->Solve();
  return feasible();
};

bool solver_seat2::feasible() {
  return result_ == gor::MPSolver::OPTIMAL ||
         result_ == gor::MPSolver::FEASIBLE;
}

void solver_seat2::create_mcf_problem() {
  for (auto const& [b_id, b] : utl::enumerate(mcf_bookings_)) {
    auto source_constraint = get_source_constraint(b_id);
    source_constraint->SetBounds(1, 1);
    auto from = b.interval_.from_;
    auto const to = b.interval_.to_;
    for (auto const& [s_id, res_wagon_pair] : seat_attributes_) {
      if (!matches(b.r_, res_wagon_pair.second)) {
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

gor::MPVariable* solver_seat2::get_var(booking_id_t const& b_id,
                                       seat_id_t const& seat_id) {
  gor::MPVariable* v =
      solver_->MakeVar(0.0, 1.0, false, fmt::format("x_{}_{}", b_id, seat_id));
  vars_.emplace(std::make_pair(b_id, seat_id), v);
  return v;
}

gor::MPConstraint* solver_seat2::get_source_constraint(
    booking_id_t const& b_id) {
  return utl::get_or_create(source_constraints_, b_id, [&]() {
    return solver_->MakeRowConstraint(fmt::format("s_{}", b_id));
  });
}

gor::MPConstraint* solver_seat2::get_capacity_constraint(
    seat_id_t const& seat_id, small_station_id_t const station_id) {
  return utl::get_or_create(
      capacity_constraints_, std::make_pair(seat_id, station_id), [&]() {
        return solver_->MakeRowConstraint(
            fmt::format("cap_{}_{}", seat_id, station_id));
      });
}

void solver_seat2::print() const {
  std::string s;
  solver_->ExportModelAsLpFormat(false, &s);
  std::cout << s;
}

void solver_seat2::print_sizes() const {
  std::cout << "vars: " << solver_->NumVariables() << "\n"
            << "constraints: " << solver_->NumConstraints() << "\n";
}

void solver_seat2::print_name() const { std::cout << "seat assignment"; }

cista::raw::vector_map<station_id_t,
                       std::map<seat_id_t, std::pair<booking_id_t, booking>>>
solver_seat2::assign_seats() {
  auto ret = cista::raw::vector_map<
      station_id_t, std::map<seat_id_t, std::pair<booking_id_t, booking>>>();
  ret.resize(number_of_segments_);
  for (auto station = station_id_t{0}; station != number_of_segments_;
       ++station) {
    for (auto const& [id, b] : utl::enumerate(mcf_bookings_)) {
      for (auto seat_id = seat_id_t{0}; seat_id != total_seats_; ++seat_id) {
        if (!matches(b.r_, seat_attributes_[seat_id].second)) {
          continue;
        }
        if (!matches(b.r_, seat_attributes_[seat_id].second)) {
          continue;
        }
        if (b.interval_.from_ > station || b.interval_.to_ < station) {
          continue;
        }
        if (vars_[std::make_pair(id, seat_id)]->solution_value() == 1) {
          ret[station].emplace(seat_id, std::make_pair(id, b));
        }
      }
    }
  }
  return ret;
}

}  // namespace seat

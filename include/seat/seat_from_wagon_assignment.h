#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ortools/linear_solver/linear_solver.h"

#include "fmt/format.h"

#include "utl/get_or_create.h"

#include "seat/booking.h"
#include "seat/reservation.h"
#include "seat/train.h"
#include "seat/types.h"

namespace seat {

namespace gor = operations_research;

struct solver_seat_from_wagon {

  explicit solver_seat_from_wagon(
      uint32_t const&,
      std::pair<std::vector<booking_id_t>, std::vector<wagon_id_t>> const&,
      std::map<seat_id_t, std::pair<wagon_id_t, reservation>> const&,
      std::vector<booking> const&, std::vector<booking_id_t> const&,
      std::vector<booking_id_t> const&, std::vector<booking_id_t> const&,
      std::vector<booking_id_t> const&, std::vector<seat_id_t> const&,
      std::vector<seat_id_t> const&, uint32_t const&, train const&);
  void solve();
  void set_hint(
      std::pair<std::vector<booking_id_t>, std::vector<wagon_id_t>> const&);
  bool feasible() const;

  void assign_seats(wagon_id_t const&);
  void print() const;
  void print_sizes() const;
  void print_name() const;

  gor::MPVariable* get_var(booking_id_t const& b_id, seat_id_t const& seat_id);
  gor::MPConstraint* get_source_constraint(booking_id_t const& b_id);
  gor::MPConstraint* get_capacity_constraint(
      seat_id_t const& seat_id, small_station_id_t const station_id);
  void create_mcf_problem(wagon_id_t const&);
  void create_objective(wagon_id_t const);
  bool is_gsd_blocked(interval const&, seat_id_t const);
  void reset();

  template <class T, typename F>
  void create_obj_constraints(
      std::map<std::pair<group_id_t, T>, gor::MPConstraint*>& constraints,
      F&& f) {
    for (auto const& [key, var] : vars_) {
      auto const b_id = key.first;
      auto s_id = key.second;
      auto const group_id = bookings_[b_id].group_id_;
      if (group_id == 0) {
        continue;
      }
      T filtered = f(s_id);
      auto constraint = utl::get_or_create(
          constraints, std::make_pair(group_id, filtered), [&]() {
            return solver_->MakeRowConstraint(
                fmt::format("s_{}_{}", group_id, s_id));
          });
      constraint->SetCoefficient(var, 1);
    }
  }

  operations_research::MPSolver* solver_;
  operations_research::MPSolver::ResultStatus result_ =
      gor::MPSolver::INFEASIBLE;

  train train_;
  std::pair<std::vector<booking_id_t>, std::vector<wagon_id_t>>
      wagon_id_by_booking_ids_;
  std::map<seat_id_t, std::pair<wagon_id_t, reservation>> seat_attributes_;
  std::vector<booking_id_t> mcf_booking_ids_;
  std::vector<booking> bookings_;
  uint32_t total_seats_;
  uint32_t number_of_segments_;
  std::vector<seat_id_t> pseudo_seats_;
  std::vector<seat_id_t> gsd_seats_;
  std::vector<booking_id_t> gsd_ids_;
  std::vector<booking_id_t> pseudo_ids_;

  std::map<std::pair<booking_id_t, seat_id_t>, gor::MPVariable*> vars_;

  std::map<booking_id_t, gor::MPConstraint*> source_constraints_;
  std::map<std::pair<seat_id_t, small_station_id_t>, gor::MPConstraint*>
      capacity_constraints_;
  std::pair<std::vector<booking_id_t>, std::vector<seat_id_t>>
      seats_by_bookings_;
  std::map<std::pair<group_id_t, row_id_t>, gor::MPConstraint*>
      row_constraints_;
  std::map<std::pair<group_id_t, bool>, gor::MPConstraint*> lr_constraints_;
  std::map<std::pair<group_id_t, bool>, gor::MPConstraint*>
      corridor_constraints_;
  std::map<std::pair<group_id_t, bool>, gor::MPConstraint*> table_constraints_;
  std::map<std::pair<group_id_t, bool>, gor::MPConstraint*> big_constraints_;
};

}  // namespace seat

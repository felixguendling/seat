#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ortools/linear_solver/linear_solver.h"

#include "seat/booking.h"
#include "seat/reservation.h"
#include "seat/train.h"
#include "seat/types.h"

namespace seat {

namespace gor = operations_research;

struct solver_seat {

  explicit solver_seat(
      uint32_t const&,
      std::map<seat_id_t, std::pair<wagon_id_t, reservation>> const&,
      std::vector<booking> const&, std::vector<booking_id_t> const&,
      std::vector<booking_id_t> const&, std::vector<booking_id_t> const&,
      std::vector<booking_id_t> const&, std::vector<seat_id_t> const&,
      std::vector<seat_id_t> const&, uint32_t const&);
  bool solve();
  void set_hint(std::vector<seat_id_t> const&);
  bool feasible() const;

  std::pair<std::vector<booking_id_t>, std::vector<seat_id_t>> assign_seats();
  void print() const;
  void print_sizes() const;
  void print_name() const;

  gor::MPVariable* get_var(booking_id_t const& b_id, seat_id_t const& seat_id);
  gor::MPConstraint* get_source_constraint(booking_id_t const& b_id);
  gor::MPConstraint* get_capacity_constraint(
      seat_id_t const& seat_id, small_station_id_t const station_id);
  void create_mcf_problem();
  void create_objective();
  bool is_gsd_blocked(interval const&, seat_id_t const);

  operations_research::MPSolver* solver_;
  operations_research::MPSolver::ResultStatus result_ =
      gor::MPSolver::INFEASIBLE;

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
};

}  // namespace seat

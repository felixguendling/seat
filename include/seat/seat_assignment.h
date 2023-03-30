#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cista/containers/vector.h"
#include "ortools/linear_solver/linear_solver.h"

#include "mcf_solver_i.h"
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
      cista::raw::vector_map<booking_id_t, booking> const&,
      std::map<seat_id_t, std::vector<booking>> const&, uint32_t const&);
  bool solve();
  bool feasible();

  cista::raw::vector_map<station_id_t,
                         std::map<seat_id_t, std::pair<booking_id_t, booking>>>
  assign_seats();
  void print() const;
  void print_sizes() const;
  void print_name() const;

  gor::MPVariable* get_var(booking_id_t const& b_id, seat_id_t const& seat_id);
  gor::MPConstraint* get_source_constraint(booking_id_t const& b_id);
  gor::MPConstraint* get_capacity_constraint(
      seat_id_t const& seat_id, small_station_id_t const station_id);
  void create_mcf_problem();

private:
  operations_research::MPSolver* solver_;
  operations_research::MPSolver::ResultStatus result_;

  std::map<seat_id_t, std::pair<wagon_id_t, reservation>> seat_attributes_;
  cista::raw::vector_map<booking_id_t, booking> mcf_bookings_;
  uint32_t total_seats_;
  uint32_t number_of_segments_;
  std::map<seat_id_t, std::vector<booking>> pseudo_gsd_bookings_;

  std::map<std::pair<booking_id_t, seat_id_t>, gor::MPVariable*> vars_;

  std::map<booking_id_t, gor::MPConstraint*> source_constraints_;
  std::map<std::pair<seat_id_t, small_station_id_t>, gor::MPConstraint*>
      capacity_constraints_;
};

}  // namespace seat

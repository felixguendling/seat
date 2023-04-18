#pragma once
#include "ortools/linear_solver/linear_solver.h"

#include "seat/booking.h"
#include "seat/reservation.h"
#include "seat/types.h"

namespace seat {

namespace gor = operations_research;

struct solver_wagon {

  explicit solver_wagon(
      uint32_t const&,
      std::map<std::pair<wagon_id_t, reservation>, uint32_t> const&,
      std::vector<booking> const&, std::vector<booking_id_t> const&,
      std::vector<booking_id_t> const&, std::vector<booking_id_t> const&,
      std::vector<booking_id_t> const&, std::vector<seat_id_t> const&,
      std::vector<seat_id_t> const&);
  bool solve();
  bool feasible() const;

  void print() const;
  void print_sizes() const;
  void print_name() const;

  gor::MPVariable* get_var(booking_id_t const&, wagon_id_t const&,
                           reservation const&);
  gor::MPConstraint* get_source_constraint(booking_id_t const& b_id);
  gor::MPConstraint* get_capacity_constraint(wagon_id_t const&,
                                             small_station_id_t const,
                                             reservation const&);
  void set_hint(std::vector<wagon_id_t> const&);
  std::pair<std::vector<booking_id_t>, std::vector<wagon_id_t>> assign_seats();
  void create_mcf_problem();
  void create_objective();

  operations_research::MPSolver* solver_;
  operations_research::MPSolver::ResultStatus result_ =
      gor::MPSolver::INFEASIBLE;

  std::map<std::pair<wagon_id_t, reservation>, uint32_t> wagon_res_capacities_;
  std::vector<booking_id_t> mcf_booking_ids_;
  std::vector<booking> bookings_;
  uint32_t number_of_segments_;
  std::vector<seat_id_t> pseudo_seats_;
  std::vector<seat_id_t> gsd_seats_;
  std::vector<booking_id_t> gsd_ids_;
  std::vector<booking_id_t> pseudo_ids_;

  std::map<std::pair<std::pair<booking_id_t, wagon_id_t>, reservation>,
           gor::MPVariable*>
      vars_;

  std::map<booking_id_t, gor::MPConstraint*> source_constraints_;
  std::map<std::pair<reservation, std::pair<wagon_id_t, small_station_id_t>>,
           gor::MPConstraint*>
      capacity_constraints_;
};

}  // namespace seat

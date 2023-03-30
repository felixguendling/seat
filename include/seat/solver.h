#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ortools/linear_solver/linear_solver.h"

#include "mcf_solver_i.h"
#include "seat/booking.h"
#include "seat/reservation.h"
#include "seat/types.h"

namespace seat {

namespace gor = operations_research;

struct solver : mcf_solver_i {

  explicit solver(std::map<reservation, uint32_t> const& capacities, int const);
  bool solve();
  bool feasible();

  void add_booking(booking const&);
  void remove_booking(const booking& rem_b);
  void add_single_booking(const booking&);
  cista::raw::vector_map<booking_id_t, booking> get_mcf_bookings();
  std::map<seat_id_t, std::vector<booking>> get_gsd_bookings();

  void clear();
  void replace_ones();
  void check_initial_bounds(gor::MPConstraint* c);

  void print() const;
  void print_sizes() const;
  void print_name() const;

  gor::MPVariable* get_source_var(small_station_id_t const,
                                  small_station_id_t const,
                                  reservation const& res,
                                  reservation const& c_res);
  gor::MPConstraint* get_source_constraint(small_station_id_t const,
                                           small_station_id_t const,
                                           reservation const&);
  gor::MPConstraint* get_capacity_constraint(small_station_id_t const,
                                             reservation const&);
  void concrete_booking_adjust(booking const&, bool const);
  std::map<reservation, bool> gsd_request(interval const&);
  void add_gsd_booking(booking const&, uint32_t const&);

  std::map<reservation, std::vector<std::pair<booking, int>>>
  sort_bookings_into_res() const;
  void to_graphviz(std::ostream&, bool const) const;

private:
  operations_research::MPSolver* solver_;
  operations_research::MPSolver::ResultStatus result_;
  std::map<std::pair<reservation, small_station_id_t>, int> capacities_;
  std::vector<reservation> c_;

  std::vector<booking> mcf_bookings_;
  std::map<std::pair<reservation,
                     std::pair<reservation, std::pair<small_station_id_t,
                                                      small_station_id_t>>>,
           gor::MPVariable*>
      source_vars_;
  std::map<
      std::pair<reservation, std::pair<small_station_id_t, small_station_id_t>>,
      gor::MPConstraint*>
      source_constraints_;
  std::map<std::pair<reservation, small_station_id_t>, gor::MPConstraint*>
      capacity_constraints_;
  std::map<seat_id_t, booking> gsd_bookings_;
  std::map<seat_id_t, std::vector<booking>> pseudo_gsd_bookings_;
  int number_segments_;
  int one_ = int{1111};
  bool invalid = false;
  std::vector<booking> concrete_bookings_;
};

}  // namespace seat

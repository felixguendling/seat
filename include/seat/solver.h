#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ortools/linear_solver/linear_solver.h"

#include "seat/booking.h"
#include "seat/reservation.h"
#include "seat/types.h"

namespace seat {

namespace gor = operations_research;

struct solver {

  explicit solver(std::map<reservation, uint32_t> const& capacities, int const);
  bool solve();
  bool feasible();

  void add_booking(booking const&);
  void remove_booking(const booking& rem_b);
  void create_vars_and_constraints(const booking_id_t& new_b);
  std::vector<booking> get_mcf_bookings();
  std::map<seat_id_t, std::vector<booking>> get_gsd_bookings();
  void erase(std::vector<booking_id_t>&, booking const&);

  void clear();
  void replace_ones();
  void check_initial_bounds(gor::MPConstraint* c);

  void print() const;
  void print_sizes() const;
  void print_name() const;

  gor::MPVariable* get_path_var(booking const& b_id, reservation const& c_res);
  gor::MPConstraint* get_source_constraint(booking const&);
  gor::MPConstraint* get_capacity_constraint(small_station_id_t const,
                                             reservation const&);
  void concrete_booking_adjust(booking const&, bool const);
  std::vector<reservation> gsd_request(interval const&);
  void add_gsd_booking(booking const&, seat_id_t const&);

  std::vector<std::pair<booking, int>> sort_bookings_into_res(
      reservation const&) const;
  std::vector<small_station_id_t> find_tight_capacities(booking const&) const;
  void to_graphviz(std::ostream&, bool const) const;

  // bookings_ holds all bookings, the other vectors concerning bookings refer
  // to indices in bookings_.
  std::vector<booking> bookings_;
  // Once a gsd-booking has been accepted it gets a fixed seat, this lists all
  // ids of such bookings
  std::vector<booking_id_t> gsd_bookings_;

  // When a gsd-booking is asigned a fixed seat, the seat and the booking are no
  // longer considered in the mcf, use pseudo-gsd-bookings to fill this seat on
  // segments that aren't blocked by this gsd-booking, these bookings are called
  // pseudo-gsd-bookings
  std::vector<booking_id_t> pseudo_gsd_bookings_;
  // holds the seats for every (pseudo)gsd-booking
  std::vector<seat_id_t> gsd_seats_;
  std::vector<seat_id_t> pseudo_gsd_seats_;

  // these are all the bookings that require optimization variables in the mcf
  std::vector<booking_id_t> mcf_bookings_;
  // bookings without 'anys' are treated differently (adjusting the capacity)
  // and don't require optimization variables
  std::vector<booking_id_t> concrete_bookings_;

private:
  operations_research::MPSolver* solver_;
  operations_research::MPSolver::ResultStatus result_;
  std::map<std::pair<reservation, small_station_id_t>, int> capacities_;
  std::vector<reservation> c_;

  std::map<std::pair<reservation, booking>, gor::MPVariable*> path_vars_;
  std::map<booking, gor::MPConstraint*> source_constraints_;
  std::map<std::pair<reservation, small_station_id_t>, gor::MPConstraint*>
      capacity_constraints_;
  int number_segments_;
  int one_ = int{1111};
  bool invalid = false;

  bool output_ = false;
};

}  // namespace seat

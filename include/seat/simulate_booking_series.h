#include <fstream>
#include <iostream>
#include <tuple>
#include <vector>

#include "utl/timing.h"
#include "utl/verify.h"

#include "seat/booking.h"
#include "seat/nogo_cache.h"
#include "seat/random_booking.h"
#include "seat/reservation.h"
#include "seat/seat_assignment.h"
#include "seat/simulation_specifics.h"
#include "seat/solver.h"
#include "seat/train.h"

namespace seat {
struct simulation {
  explicit simulation(std::map<reservation, std::uint32_t> const&, solver*,
                      train const&, simulation_specifics const&);

  void simulate();
  void add_normal_booking();
  void add_gsd_booking();
  void add_group_booking();
  std::pair<std::vector<booking_id_t>, std::vector<wagon_id_t>>
  assign_to_wagons(std::vector<wagon_id_t> const&);
  void print_valid_times();
  void reset();
  void print_wagon_assignment_results(
      std::pair<std::vector<booking_id_t>, std::vector<wagon_id_t>> const&)
      const;
  void write_before_wagon_assignment();
  void read_bookings(std::string const& file_name);

  std::map<reservation, std::uint32_t> seats_by_res_;
  solver* solver_;

  train train_;

  nogo_cache nogo;
  nogo_cache gsd_nogo;
  nogo_cache group_nogo;

  std::ofstream timings_ = std::ofstream{"timings.dat"};

  int failed_ = 420;
  int i_ = 0U;
  int success_ = 0U;

  std::vector<std::uint64_t> timings_vec_;
  std::vector<booking_type> booking_type_;
  int gsd_counter_ = 0;
  int normal_counter_ = 0;
  int group_counter_ = 0;

  uint32_t last_group_id_ = 1;

  simulation_specifics specifics_;
};
}  // namespace seat
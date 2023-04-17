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
#include "seat/solver.h"
#include "seat/train.h"

namespace seat {
struct simulation {
  explicit simulation(std::map<reservation, std::uint32_t> const&, solver*,
                      double const&, double const&, std::uint32_t const&,
                      train const&);

  void simulate();
  void add_normal_booking();
  void add_gsd_booking();
  void add_group_booking();
  void reset();

  std::map<reservation, std::uint32_t> seats_by_res_;
  solver* solver_;
  double gsd_prob_;
  double group_prob_;
  std::uint32_t number_of_segments_;
  train train_;

  std::ofstream timings_ = std::ofstream{"timings.dat"};
  nogo_cache nogo;
  nogo_cache gsd_nogo;
  nogo_cache group_nogo;
  int failed_ = 420;
  int i_ = 0U;
  int success_ = 0U;
  std::vector<std::uint64_t> timings_vec_;
  std::vector<BOOKING_TYPE> booking_type_;
  int gsd_counter_ = 0;
  int normal_counter_ = 0;
  int group_counter_ = 0;
  uint32_t last_group_id_ = 0;
  uint8_t max_group_size_ = 40U;
};
}  // namespace seat
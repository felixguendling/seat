#include <iostream>
#include <vector>

#include "seat/booking.h"
#include "seat/reservation.h"

namespace seat {
struct interval_graph {
  explicit interval_graph(std::vector<booking> const&);
  void distribute_bookings();
  void create_graph(std::vector<booking_id_t> const&);
  void reset();

  std::vector<booking> all_bookings_;
  std::vector<booking_id_t> b_ids_;
  std::vector<seat_id_t> seats_by_booking_id_;
  std::vector<std::vector<int>> neighbours_;
};
}  // namespace seat

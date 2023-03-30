#pragma once

#include <map>
#include <memory>

#include "cista/containers/vector.h"

#include "seat/booking.h"
#include "seat/types.h"

namespace seat {

struct mcf_solver_i {

  virtual bool solve() = 0;
  virtual void add_booking(booking const&) = 0;
  virtual void remove_booking(booking const&) = 0;
  virtual std::map<reservation, bool> gsd_request(interval const&) = 0;
  virtual void add_gsd_booking(booking const&, uint32_t const&) = 0;

  virtual void print_sizes() const = 0;
  virtual void print_name() const = 0;
  virtual void print() const = 0;
  virtual void to_graphviz(std::ostream&, bool const) const = 0;
  virtual cista::raw::vector_map<booking_id_t, booking> get_mcf_bookings() = 0;
  virtual std::map<seat_id_t, std::vector<booking>> get_gsd_bookings() = 0;
};

}  // namespace seat
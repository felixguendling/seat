#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cista/containers/vector.h"
#include "ortools/linear_solver/linear_solver.h"

#include "seat/booking.h"
#include "seat/reservation.h"
#include "seat/types.h"

namespace seat {

namespace gor = operations_research;

struct pseudo_gsd_chooser {
  explicit pseudo_gsd_chooser(std::vector<booking_id_t> const&,
                              std::vector<booking> const&, uint8_t const,
                              interval const&,
                              std::vector<small_station_id_t> const&);
  void print() const;

  std::vector<booking_id_t> chosen_bookings_;

private:
  operations_research::MPSolver* solver_;
  operations_research::MPSolver::ResultStatus result_;

  std::map<booking_id_t, gor::MPVariable*> vars_;
  std::vector<gor::MPConstraint*> constraints_;
  bool invalid = false;

  std::vector<booking_id_t> b_ids_;
};

}  // namespace seat

#include "seat/pseudo_gsd_chooser.h"

#include "utl/get_or_create.h"
#include "utl/verify.h"

#include "ortools/linear_solver/linear_solver.h"
#include "utl/enumerate.h"

namespace seat {

constexpr auto const solver_name = "SCIP";
pseudo_gsd_chooser::pseudo_gsd_chooser(
    cista::raw::vector_map<booking_id_t, booking> const& bookings)
    : solver_{gor::MPSolver::CreateSolver(solver_name)} {
  bookings_ = bookings;
  for (auto const& [id, b] : utl::enumerate(bookings)) {
  }
}
}  // namespace seat
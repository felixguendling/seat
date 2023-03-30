#include "seat/solver.h"

#include "utl/get_or_create.h"
#include "utl/verify.h"

#include "ortools/linear_solver/linear_solver.h"
#include "utl/enumerate.h"

namespace seat {

constexpr auto const solver_name = "SCIP";
solver::solver(std::map<reservation, uint32_t> const& capacities,
               int const number_segments)
    : solver_{gor::MPSolver::CreateSolver(solver_name)},
      number_segments_{number_segments} {
  for (auto const& [r, c] : capacities) {
    if (c > 0) {
      c_.emplace_back(r);
    }
    for (auto i = small_station_id_t{0}; i != number_segments; ++i) {
      auto key = std::make_pair(r, i);
      capacities_.emplace(key, c);
    }
  }
}

bool solver::solve() {
  if (invalid) {
    invalid = false;
    return false;
  }
  result_ = solver_->Solve();
  return feasible();
};

bool solver::feasible() {
  return result_ == gor::MPSolver::OPTIMAL ||
         result_ == gor::MPSolver::FEASIBLE;
}

void solver::add_booking(booking const& new_b) {
  if (concreteness(new_b.r_) == 0) {
    concrete_booking_adjust(new_b, true);
  } else {
    mcf_bookings_.emplace_back(new_b);
  }
  clear();
  for (auto const& b : mcf_bookings_) {
    add_single_booking(b);
  }
  replace_ones();
}

void solver::concrete_booking_adjust(booking const& b, bool const added) {
  if (added) {
    concrete_bookings_.emplace_back(b);
  } else {
    auto r_id =
        std::find(begin(concrete_bookings_), end(concrete_bookings_), b);
    concrete_bookings_.erase(r_id);
  }
  for (auto i = b.interval_.from_; i != b.interval_.to_; ++i) {
    auto pair = std::make_pair(b.r_, i);
    if (added) {
      capacities_[pair]--;
    } else {
      capacities_[pair]++;
    }
    if (capacities_[std::make_pair(b.r_, i)] < 0) {
      invalid = true;
    }
  }
}

void solver::remove_booking(booking const& rem_b) {
  if (concreteness(rem_b.r_) == 0) {
    concrete_booking_adjust(rem_b, false);
    return;
  }
  auto r_id = std::find(begin(mcf_bookings_), end(mcf_bookings_), rem_b);
  mcf_bookings_.erase(r_id);
}

void solver::add_single_booking(booking const& new_b) {
  auto from = new_b.interval_.from_;
  auto const to = new_b.interval_.to_;
  auto source_constraint = get_source_constraint(from, to, new_b.r_);
  check_initial_bounds(source_constraint);
  source_constraint->SetBounds(source_constraint->lb() + 1,
                               source_constraint->ub() + 1);
  std::vector<reservation> concrete_reservations =
      get_concrete_reservations(new_b.r_);
  for (auto const& concrete_r : concrete_reservations) {
    if (std::find(c_.begin(), c_.end(), concrete_r) == c_.end()) {
      continue;
    }
    from = new_b.interval_.from_;
    auto source_var = get_source_var(from, to, new_b.r_, concrete_r);
    source_var->SetUB(source_var->ub() + one_);
    source_constraint->SetCoefficient(source_var, 1);
    while (from != to) {
      auto capacity_constraint = get_capacity_constraint(from, concrete_r);
      capacity_constraint->SetCoefficient(source_var, 1);
      auto key = std::make_pair(concrete_r, from);
      capacity_constraint->SetBounds(0.0, capacities_[key]);
      ++from;
    }
  }
}

gor::MPVariable* solver::get_source_var(small_station_id_t const from,
                                        small_station_id_t const to,
                                        reservation const& res,
                                        reservation const& c_res) {
  auto p = std::make_pair(from, to);
  auto p1 = std::make_pair(c_res, p);
  auto pair = std::make_pair(res, p1);
  return utl::get_or_create(source_vars_, pair, [&]() {
    return solver_->MakeIntVar(
        0.0, 0.0, fmt::format("source_{}_{}_{}_{}", res, from, to, c_res));
  });
}

gor::MPConstraint* solver::get_source_constraint(small_station_id_t const from,
                                                 small_station_id_t const to,
                                                 reservation const& res) {
  auto pair = std::make_pair(from, to);
  return utl::get_or_create(
      source_constraints_, std::make_pair(res, pair), [&]() {
        return solver_->MakeRowConstraint(
            fmt::format("source_c_{}_{}_{}", res, from, to));
      });
}

gor::MPConstraint* solver::get_capacity_constraint(
    small_station_id_t const from, reservation const& res) {
  return utl::get_or_create(capacity_constraints_, std::make_pair(res, from),
                            [&]() {
                              return solver_->MakeRowConstraint(
                                  fmt::format("capacity_c_{}_{}", res, from));
                            });
}

void solver::replace_ones() {
  for (const auto [key, var] : source_vars_) {
    if ((static_cast<int>(var->ub()) % one_) == 0) {
      var->SetUB(static_cast<int>(var->ub()) / one_);
    }
  }
}

void solver::clear() {
  solver_->Reset();
  solver_ = gor::MPSolver::CreateSolver(solver_name);
  capacity_constraints_.clear();
  source_constraints_.clear();
  source_vars_.clear();
}
void solver::check_initial_bounds(gor::MPConstraint* c) {
  if (c->ub() == gor::MPSolver::infinity()) {
    c->SetUB(0.0);
  }
  if (c->lb() == -gor::MPSolver::infinity()) {
    c->SetLB(0.0);
  }
}

void solver::print() const {
  std::string s;
  solver_->ExportModelAsLpFormat(false, &s);
  std::cout << s;
}

void solver::print_sizes() const {
  std::cout << "vars: " << solver_->NumVariables() << "\n"
            << "constraints: " << solver_->NumConstraints()
            << "\n"; /*
std::cout << "capacity_constraints_: " << capacity_constraints_.size() << "\n"
<< "source_constraints_: " << source_constraints_.size() << "\n"
<< "source_vars_: " << source_vars_.size() << "\n";*/
}

void solver::print_name() const { std::cout << "full path solver"; }

std::map<reservation, bool> solver::gsd_request(interval const& i) {
  std::map<reservation, bool> available_res;
  auto first_station_id = station_id_t{0};
  auto res_counter = 0U;
  for (auto const& [res, station] : capacities_) {
    if (station != first_station_id) {
      continue;
    }
    res_counter++;
  }
  std::vector<reservation> reservations =
      get_concrete_reservations(res_counter);

  for (auto const& reservation : reservations) {
    auto b = booking{};
    b.r_ = reservation;
    b.interval_ = i;
    add_booking(b);
    auto feasible = solve();
    available_res.insert(std::make_pair(reservation, feasible));
    remove_booking(b);
  }
  return available_res;
}

void solver::add_gsd_booking(booking const& gsd_booking,
                             uint32_t const& seat_number) {
  gsd_bookings_.insert(std::make_pair(seat_number, gsd_booking));
  add_booking(gsd_booking);
  solve();
  remove_booking(gsd_booking);
  std::map<reservation, std::vector<std::pair<booking, int>>> sorted_bookings =
      sort_bookings_into_res();
}

std::map<reservation, std::vector<std::pair<booking, int>>>
solver::sort_bookings_into_res() const {
  std::map<reservation, std::vector<std::pair<booking, int>>> sorted_bookings;
  for (auto const& [key, var] : source_vars_) {
    auto const amount = var->solution_value();
    if (amount == 0) {
      continue;
    }
    reservation const concrete_res = key.first;
    auto b = booking{};
    b.interval_.from_ = key.second.second.first;
    b.interval_.to_ = key.second.second.second;
    b.r_ = key.second.first;
    sorted_bookings[concrete_res].emplace_back(std::make_pair(b, amount));
  }
  return sorted_bookings;
}
cista::raw::vector_map<booking_id_t, booking> solver::get_mcf_bookings() {
  auto bookings = cista::raw::vector_map<booking_id_t, booking>{};
  for (auto const& b : mcf_bookings_) {
    bookings.emplace_back(b);
  }
  for (auto const& b : concrete_bookings_) {
    bookings.emplace_back(b);
  }
  return bookings;
}

std::map<seat_id_t, std::vector<booking>> solver::get_gsd_bookings() {
  std::map<seat_id_t, std::vector<booking>> ret;
  for (auto const& [id, vec] : pseudo_gsd_bookings_) {
    std::vector<booking> copy = vec;
    copy.emplace_back(gsd_bookings_[id]);
  }
  return ret;
}
void solver::to_graphviz(std::ostream&, bool const) const {}
}  // namespace seat
